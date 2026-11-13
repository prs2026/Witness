# HITL bench — audit evaluation

Evaluated against the code in Software/HITL, the Witness/Iris/Aperture firmware command handlers, and the saved runs in HITL/runs. Nothing was run against hardware. Analysis scripts are in the session scratchpad (audit_hitl/a.py, b.py, c.py, d.py).

## Summary table

| # | Title | Verdict | Severity |
|---|-------|---------|----------|
| 1 | Commands sent without checking the board is Witness | CONFIRMED | high (critical only via the Aperture spill, see details) |
| 2 | 0xE0/state timestamps use receipt-time sim time | CONFIRMED | medium (bias is conservative; headline 4.52 % is about 2x inflated) |
| 3 | GNSS error compared at receipt time | CONFIRMED mechanism, partly by design | low |
| 4 | 0.2 s socket timeout applies to sendall | PLAUSIBLE | low |
| 5 | PASS ignores run completeness / unhandled exceptions | NOT A BUG as described (unreachable-except part CONFIRMED, cosmetic) | low |
| 6 | Exception in end-of-run block leaves recorder/activity stuck | CONFIRMED (code path) | low |
| 7 | regenerate ignores vibration file overrides, overwrites imu960.npz | CONFIRMED | low (0 saved runs used overrides) |
| 8 | Monte Carlo tuning and statistics share realizations | CONFIRMED | low |
| 9 | RF GPS timeline from wall clock, no drift check | PLAUSIBLE | low |
| 10 | Power-cycle reconnect can connect after stop | PLAUSIBLE | low |
| 11 | Old reader tears down new connection | PLAUSIBLE, practically unreachable | low |
| 12 | Stale hitl_active skips ENTER | NOT A BUG | none |
| 13 | scp has no timeout, Stop cannot interrupt | CONFIRMED | low |
| 14 | HackRF stop() result ignored | CONFIRMED (remote variant worse) | medium for the remote path, low locally |
| 15 | Hub power cycles not rate limited | CONFIRMED | low |
| 16 | kalman_replay GGA regex drops GA/GL | NOT A BUG on this bench | none |
| 17 | Misleading lead-in warning, unreachable except | CONFIRMED | low (cosmetic) |
| N1 | bench_ports scans only 7100-7102; two Witnesses are indistinguishable | NEW, CONFIRMED | high |
| N2 | witnessOS HITL tab does not pass abort_on_no_hitl | NEW, CONFIRMED | medium (amplifies #1) |
| N3 | Default end-of-run hub cycle reshuffles ports, leaving the saved port stale | NEW, CONFIRMED mechanism | high (feeds #1) |
| N4 | Flash comparison offset has no independent anchor | NEW (part of the #2 fix) | medium |

## Details per finding

### 1. Commands sent without verifying the board is Witness — CONFIRMED, high
- **Evidence:** `HitlSession.connect` (hitl_session.py:452) only opens the TCP socket. `bench_ports.witness_port` is called in exactly one place, `_power_cycle_bench` (hitl_session.py:1099). The GUI connect path (wos_hitl_tab.py:969-998) and the manual ENTER button have no identity check. Identification is passive and is never run before a normal run. It cannot run on a live connection anyway, because `listen()` opens its own connection and ser2net serves one client per port.
- **Which commands go out:** `encode_command` allows only HITL ENTER (0x4849), HITL EXIT (0x4858), WITNESS DEBUG START (0x00E0), GPS COLD START (0x4752). Also sent: GPS assist frames (0x4741/44/45/51), the 0x0C and 0x0E streams, and the 3-byte `05 68 68` mass-storage export at the end of each run (hitl_outputs.py:216). Erase, clear and set-flight-state commands cannot be built. None of the deliberate frames is destructive.
- **Iris on the port:** Iris ignores unknown 16-bit opcodes (`execute_binary_command` default returns false). Its UART1 parser resyncs on 0x05 inside the stream, but a match needs an exact opcode, so it is harmless in practice. The run fails the HITL-active check.
- **Aperture on the port:** Aperture (main.cpp ~L100-150) echoes every byte and scans the incoming stream for any 0x05. It takes the next 9 bytes, and if the 10th is 0x0A it transmits the window over LoRa as a command. Witness accepts these radio commands: GPS cold start, assist, HITL EXIT, mass storage, log erase/clear, set flight state, and Iris load/regulator relays. HITL ENTER is refused over radio.
  - I replayed the real 0x0E byte stream of run 20261111-150452 (430 s, imu960.npz) through Aperture's parser: 35 spurious 10-byte frames would be transmitted over LoRa per run. Their opcodes were 0xF5xx-0xFAxx, so none was dangerous this time.
  - The 0x0C stream and the deliberate GPS COLD START and assist frames add to this. A cold start sent through Aperture reaches any Witness in LoRa range, including a flight unit.
  - The chance that a spurious frame hits a destructive opcode (0x686x, 0x73xx, 0x0031-0x0035) is small but non-zero. This is the only path to "command the wrong board"; I rate it high, and it is critical only if an Aperture sits on the saved port with a live flight Witness within radio range.
- **Spare Witness on the port (most likely wrong-board case):** HITL ENTER, the stream and the export all work, so the run PASSes. The run summary and FC logs then describe the spare board. This is the case where results could approve a bad config.
- **Failure path:** a hub cycle (N3) reorders ttyACM, the saved port 7100 now points at another board, the user presses Start, and nothing checks.
- **Fix:** after `connect`, passively classify the live stream for about 6 s on the already-open socket by feeding reader bytes to `bench_ports.classify`. Refuse every send until it returns "witness". For two Witnesses, also compare a board identifier (project name or MAC log line). Make the check a precondition of `start_run`, `send_command`, `upload_assist` and `fetch_fc_logs`.

### 2. Receipt-time stamping biases 0xE0 position/velocity/altitude and state times — CONFIRMED, medium
- **Mechanism:** `_on_packet` stamps `debug_records` with `_sim_s()` (hitl_session.py:649), which is the last frame sent. State times use `_pending_sim` and `_sim_s()`. Truth is therefore ahead of what the packet describes by the USB, ser2net and TCP latency. The attitude path corrects this with `att_latency_ms`; the position path does not.
- **Measured latency:** the offset `sim_s - uptime_ms` inside one run spans 34-59 ms (runs 20261110-130829, 133856 and 20261111-150452: 0.042, 0.059 and 0.034 s). That is per-packet jitter of about +/-17-30 ms. Shifting the truth by tau and minimising the error gives a best fit of 10-20 ms on all three runs:
  - 130829: worst ascent error 3.44 % at tau 0, 1.51 % at 10 ms.
  - 133856: 2.57 % at 0, 1.17 % at 10 ms.
  - 150452 live: 2.53 % at 0, 2.0 % at 20 ms.
  - Larger shifts get steadily worse, so the lag is real and about 10-25 ms.
  - This lag also includes Witness's own processing delay, which cannot be separated without an independent anchor (N4).
- **Bias = speed x latency:** about 2-5 m at 200 m/s, 13-33 m at the 1324 m/s peak. Jitter adds roughly +/-22-40 m at peak speed to any per-packet maximum.
- **The 4.52 % altitude figure (KALMAN_TUNING_REPORT.md):** I reproduced it exactly from runs/20261111-150452/comparison_flash.csv. The worst ascent point is -4.9 m at sim 21.21 s, with truth 108 m above the pad and climbing at 203 m/s. That error equals 24 ms of speed.
  - comparison_flash.csv places each flash row at `uptime + median(receipt_sim - uptime)`. That median carries the whole link latency.
  - Shifting truth by 10 ms gives 2.71 %. At 20 ms the worst point moves to 29.9 s (-65 m at 3045 m, 2.15 %), which is a real filter error.
  - The live comparison.csv gives 2.53 % for the same run.
  - The figure is materially affected: the timing-corrected value is about 2.1-2.7 %, versus 4.52 % reported.
- **Direction:** conservative. A late estimate always looks worse against a later truth, so this bias inflates error and does not hide it. It cannot approve a bad config, but it can send tuning after phantom error (the "-4.9 m at 1.2 s into boost" item).
- **State transitions:** observed deltas were 0.01-0.02 s against a 1.0 s tolerance, so they are not affected.
- **Fix:** anchor the offset on the HITL start instead of the median. Take the flash row where hitl_active first becomes 1 (or the first 0x0C-driven change) and align it to the known first-frame sim time (plus one tick). Alternatively, estimate the latency once per run and apply it as `att_latency_ms` already does, and report it. At minimum, note in KALMAN_TUNING_REPORT.md that the 4.52 % includes about 24 ms of link latency.

### 3. GNSS error compared at receipt time — CONFIRMED mechanism, low
- **Evidence:** `nmea_records` use `_sim_s()` at receipt, and hitl_report.py:339/352 interpolates truth at that time. In run 150452 the GGA altitude error against truth is 67 m RMS at tau 0 and 29 m RMS with a 0.22 s shift.
- **Interpretation:** the 0.22 s is mainly the real PX1120S output latency (the tuning report cites fixes 0.69 s behind under 43 g) plus the HackRF start-time uncertainty (#9). Only about 10-25 ms of it is link latency. Receipt-time stamping measures what Witness actually sees, which is what matters for flight. It is conservative, and the label "GPS error" mixes latency with accuracy.
- **Fix:** report the lag as a separate figure (best-fit shift) next to the RMS, or align on the GGA UTC epoch against the scenario UTC. Do not treat the receipt-stamped RMS as receiver accuracy.

### 4. sendall under a 0.2 s socket timeout — PLAUSIBLE, low
- **Evidence:** the semantics are real: since Python 3.5 the timeout covers the whole `sendall`, and a partial frame is left if it expires. The abort is the safe response, and the run then sends HITL EXIT. The aborting thread stops streaming, so there is no corruption beyond one partial frame. I found no "send failed" or "timed out" send errors in any saved run log. Stalls of 0.2 s with about 23 KB/s of traffic are unlikely, though assist uploads and slow ser2net are the exposure.
- **Fix:** `sock.settimeout(2.0)` for sends, with a short `select` timeout in the reader. Or use `SO_SNDTIMEO` and keep the recv timeout separate.

### 5. PASS does not check completeness — NOT A BUG as described (low residue)
- **Evidence:** `evaluate` reports "MISSED" for any expected state not observed, which sets passed = False (hitl_session.py:304-314). A run that dies before Descent or Landed therefore cannot PASS. An exception also cannot skip `evaluate`, because it runs in the `finally` block. The remaining gap is a run that was cut short after the last transition, which is irrelevant for a state-transition PASS.
- **Confirmed sub-point:** the `FileNotFoundError` clause (hitl_session.py:1218) is unreachable because OSError is caught first. The message "send failed: ..." is still informative.
- **Fix (optional):** add `frames_sent < len(frames)` to the problems list.

### 6. Exception in end-of-run block — CONFIRMED (code path), low
- **Evidence:** `finish_attitude`, `write_comparison` and the report are guarded. `evaluate` (which does not raise in practice) and `self._recorder.finish` are not (hitl_session.py:1240, 1252). A disk-full or JSON type error leaves `_recorder` set, `set_activity` stuck on "Writing run outputs", and no "run finished" event. `_run_finished` returns silently when `run.result` is absent.
- **Impact:** a stuck GUI label and lost outputs. It is cosmetic for results, because run.running and run.finished are already set.
- **Fix:** wrap the whole post-run block in try/finally that always clears `_recorder`, sets activity "", and emits the finished event, and log the exception.

### 7. regenerate ignores explicit vibration files — CONFIRMED, low
- **Evidence:** `regenerate` (hitl_outputs.py:341) calls `ims.find_profile` and ignores `vibration_thrust_file`, `vibration_lateral_file` and `vibration_full_scale_g`. `write_comparison` then rewrites imu960.npz. None of the saved runs used the file overrides (no params.json has a non-empty `vibration_thrust_file`), so there is no damage so far.
- **Fix:** use `ims.resolve_profile` with the same arguments as the live path, and do not overwrite an existing imu960.npz.

### 8. Monte Carlo tuning and reporting share the realizations — CONFIRMED, low
- **Evidence:** `tune` and `evaluate` use `realizations()` for both selection and the reported worst/p90. The effect is an optimistic worst case in the offline numbers. It is partly mitigated: the HITL acceptance run (150452) is an independent test, and even with the 24 ms bias it reported 4.52 %.
- **Fix:** hold out some seeds (for example tune on 8 and report on 8 new ones), and say which in KALMAN_TUNING_REPORT.md.

### 9. RF GPS timeline from wall clock — PLAUSIBLE, low
- **Evidence:** `started_at = monotonic() - 1.0` assumes the first rate line appears 1.000 s after the stream starts (gps_rf.py:568). Drift check: hackrf_transfer "Total time" in every recent run exceeds the IQ duration by a constant 0.148-0.150 s whatever the run length (240 s, 360 s, 175 s). That points to no accumulating underrun. The start-time uncertainty (+/-0.1 s, more over SSH) shows up as a constant GPS-versus-truth offset, indistinguishable from receiver latency (#3). `sim_t0_offset_s` records only the lead-in.
- **Fix:** log hackrf_transfer's per-second counts and warn if a "Total time" deviates by more than about 0.5 s from the IQ duration. Optionally calibrate the start offset with a known GGA UTC epoch.

### 10. Reconnect after stop during a power cycle — PLAUSIBLE, low
- **Evidence:** the identify loop (up to 18 s per scan) checks `_stop_run` only before each attempt. If stop arrives mid-scan, `witness_port` still returns and `connect()` runs. It needs a stop during a rare retry path.
- **Fix:** check `_stop_run` again after the loop and before `connect()`.

### 11. Old reader disconnects the new connection — PLAUSIBLE, practically unreachable, low
- **Evidence:** the reader uses its own captured `sock`. Both `disconnect` and `_power_cycle_bench` join the old reader (2 s, recv timeout 0.2 s) and wait seconds before `connect`. It would need a reader blocked for more than 2 s inside `_on_packet`.
- **Fix:** use a per-connection stop event instead of one shared `_closing`.

### 12. Stale hitl_active skips ENTER — NOT A BUG
- **Evidence:** `view.hitl_active` is refreshed from every status packet, which arrive continuously. Witness itself leaves HITL after 1 s without 0x0C frames, so the flag clears within about one status period of any EXIT or timeout. In RF mode the lead-in lasts 45-180 s, and the flag is read after it. A run still reports hitl_seen only when the bit is seen during the run window.

### 13. scp with no timeout, Stop cannot interrupt — CONFIRMED, low
- **Evidence:** `stage` runs a plain `subprocess.run` for scp (gps_rf.py:689). `stop_run` joins the sender for only 5 s, so the UI returns while the sender thread is still copying and "a run is already in progress" blocks a new start. Staging is skipped when the file size matches, which keeps it rare.
- **Fix:** use `Popen`, poll `_stop_run`, and terminate scp on stop; add an overall timeout.

### 14. HackRF stop() result ignored — CONFIRMED, medium for the remote path
- **Evidence:** the local `stop()` returns False and logs "the HackRF stays busy", but the caller drops the value. The remote `stop()` (gps_rf.py:718-734) swallows SSH errors and always returns True. If the SSH session is down, the Pi's hackrf_transfer keeps transmitting GPS L1 until the IQ file ends, and `proc.kill()` only kills the local ssh. `pkill -f` patterns also match any similar process on the Pi.
- **Fix:** after remote stop, verify with `pgrep` that no hackrf_transfer remains (retry with `-KILL`), return False otherwise, and raise or warn in `_send_loop`.

### 15. Hub power cycles not rate limited — CONFIRMED, low
- **Evidence:** a run can cycle at the lead-in (once) and at the FC fetch (once). `fetch_fc_logs` cycles hub 2 in its `finally` even if the export never started (hitl_outputs.py:256). The ganged Pi 4 cycle resets every board on the hub (and the HackRF, if it is on that hub). It is bounded and expected.
- **Fix:** skip the end-of-run cycle if the export command was never sent.

### 16. kalman_replay GGA regex — NOT A BUG on this bench
- **Evidence:** the PX1120S emits only `$GPGGA`, including in the Galileo run (38 477 sentences counted across three runs, all GP). The finding is also wrong on the other side: `parse_gga` accepts G[PNL], so it accepts GL, not GA, and both drop GA. No sentences are being lost today.
- **Fix (optional):** use `G[A-Z]GGA` in both places.

### 17. Misleading lead-in warning, unreachable except — CONFIRMED, low (cosmetic)
- **Evidence:** a stop during `_wait_for_fix` returns None, so "no GPS fix during the lead-in; the run continues" is logged right before `RuntimeError("stopped during the GPS lead-in")` aborts the run. The abort reason itself is correct.
- **Fix:** skip the warning when `_stop_run` is set.

### N1. bench_ports scans three ports; two Witnesses look identical — NEW, CONFIRMED, high
- **Evidence:** `RAW_PORTS = (7100, 7101, 7102)` (bench_ports.py:34), but the bench now has four ports (7103 carries the spare Witness). `classify` returns "witness" for either Witness (same log tags), and `witness_port` accepts exactly one.
- **Failure path:** after a lead-in power cycle, the flight Witness enumerates on 7103 and the spare on 7101. The scan sees only the spare, so `witness_port` returns it and `_power_cycle_bench` reconnects to the spare. The run continues on the wrong board and the FC log fetch exports the wrong board. If both land in 7100-7102 it raises (safe failure).
- **Fix:** scan all four ports (`RAW_PORTS = (7100, 7101, 7102, 7103)`) and discriminate the two Witnesses by an identifier (project name or MAC in the boot log, or a fixed-order USB serial mapping in ser2net via /dev/serial/by-id). Fail loudly on ambiguity.

### N2. witnessOS HITL tab starts runs without abort_on_no_hitl — NEW, CONFIRMED, medium
- **Evidence:** `wos_hitl_tab.py:1110` calls `session.start_run(run)` with the default False, while hitl_gui.py:297 passes True. In the witnessOS tab a run against a board that never sets the HITL-active bit logs a warning after 2 s and streams for the whole profile (up to about 7 minutes), which on Aperture means the spurious LoRa traffic in #1.
- **Fix:** pass `abort_on_no_hitl=True` from the witnessOS tab.

### N3. End-of-run hub cycle makes the saved port stale — NEW, CONFIRMED mechanism, high
- **Evidence:** `fetch_fc_logs` defaults to `restart=True` and `fetch_restart_hub` is True in settings (hitl_gui.py:162). It power-cycles the whole hub after every run with FC logs. The memory note and bench_ports docstring both say port order follows ttyACM enumeration, which changes after hub cycles. The next run connects to the stored port (7100 by default) without re-identifying.
- **Fix:** after the cycle, run the identity check (N1 fix) and update the stored port, or clear the connection and require reconnect. Pair with fix #1.

### N4. Flash comparison offset has no independent anchor — NEW, medium
- **Evidence:** `write_flash_comparison` uses `median(sim_s - uptime)` over the receipt-stamped debug records. That makes the flash-log comparison (the "exact" 50 Hz path) inherit the whole receipt latency. This is the direct cause of the inflated 4.52 %.
- **Fix:** see #2 (anchor on the HITL-start row).

## Significant issues to act on

- **#1 / N3 / N2 (high):** no board-identity check before commands or streaming, the default hub cycle changes the port mapping, and the witnessOS tab does not abort on missing HITL. Fix: verify identity passively on the open socket before any send, re-verify after any hub cycle, and pass `abort_on_no_hitl=True`.
- **N1 (high):** bench_ports ignores port 7103 and cannot tell the two Witnesses apart, so a power-cycle reconnect can attach to the spare. Fix: scan all four ports and add a board identifier.
- **#2 / N4 (medium, not high, but acts on headline numbers):** receipt-time stamping inflates position errors by about 24 ms of speed (4.52 % to about 2.1-2.7 %). Fix: anchor the flash offset on the HITL-start row and report the latency.
- **#14 remote (medium):** remote HackRF stop always reports success. Fix: verify with `pgrep` and fail loudly.
