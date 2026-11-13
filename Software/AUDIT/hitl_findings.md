# HITL bench — audit findings

## Overview
- Read-only audit of Software/HITL (hitl_session, hitl_protocol, hitl_outputs, hitl_report, gps_rf, bench_ports, attitude, kalman_mc, kalman_replay, sim_model, trajectory_csv, imu_synth) and the witnessOS HITL tab (lightly). Nothing was run against hardware.
- Unit tests: `../witnessOS/.venv/Scripts/python -m unittest discover -s tests` in Software/HITL: 87 tests, all OK.
- Wire format checked field by field against Software/Witness/softwarev1/main/comms.h: 0x0C (47 B, payload offsets 0..43), 0x0E (150 B, 8 samples x 18 B), decoders for 0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0xE0, 0xE1. Offsets, scales (0.488 mg, 10.417 mg, 70 mdps, 0.01 C, 0.0001 deg, u24 Pa) and big-endian order all match. No encoding bug found.
- Command allowlist holds: encode_command only builds HITL ENTER/EXIT, WITNESS DEBUG START and GPS COLD START; raw sends are limited to GPS assist frames (identity-checked) and the 3-byte mass-storage export.
- Process cleanup is mostly sound: HackRF players are stopped in a finally block, the local player closes stdin before killing, and the remote player SIGINTs then SIGKILLs the Pi process.
- Power cycling is bounded: at most one hub cycle per GPS lead-in failure and one per FC log fetch. There is no retry loop.
- Main risks: no check that the connected board is Witness before commands are sent; several timing and error metrics are stamped with the sender's sim time at packet receipt instead of the time the data describes; one socket timeout that can abort a run mid-frame; a PASS verdict that does not check run completeness.
- Not reviewed in depth: hitl_gui.py, galileo_rf.py, imu_synth.py math, trajectory_csv.py (sampled only), Horizon witness_download.py, kalman_replay C++ replay.

## Findings

### 1. Commands sent without checking the connected board is Witness — high
- **Where:** hitl_session.py:452-471 (`connect`), hitl_session.py:931-960 (`start_run`), hitl_session.py:1159 (HITL ENTER), hitl_session.py:1106 (reconnect by role only); wos_hitl_tab.py:246 (manual ENTER button); bench_ports.py:98 (`witness_port`, used only in `_power_cycle_bench` and gps_rf CLI)
- **What:** The GUI connects to whatever raw port is configured. Only the power-cycle path re-identifies the board by its traffic. Before HITL ENTER, GPS COLD START, GPS assist upload and 0x0C/0x0E streaming, nothing confirms the port is Witness. bench_ports' own docstring says ttyACM and ser2net port order changes after replugs and hub power cycles.
- **Scenario:** After a power cycle or replug, the saved port points at Iris or Aperture. The run then streams 0x0C/0x0E frames and 0x05 commands (cold start, ephemeris upload) to the wrong board. Aperture forwards some commands toward the radio, so the effect is not confined to the bench.
- **Confidence:** high that the check is absent. Impact depends on how Aperture and Iris handle these frames, which I could not verify.

### 2. 0xE0 and state-transition timestamps use sender sim time at receipt, biasing error statistics by speed times link latency — medium
- **Where:** hitl_session.py:649 (`run.debug_records.append((self._sim_s(), pkt))`), hitl_session.py:556-558 (`_sim_s`), hitl_session.py:748 and 757 (state times), hitl_session.py:613 (log-line states); hitl_report.py:150 (`rows_from_received`); hitl_outputs.py:288-301 (`write_flash_comparison` offset is a median of the same biased values)
- **What:** A Witness packet is paired with the truth sample the sender is transmitting when the packet arrives, not with the sample Witness used. The bytes travel through USB, ser2net and TCP first, so the truth is ahead by the link latency. The attitude path corrects this with `att_latency_ms` (hitl_session.py:660), but the position, velocity, altitude and state comparisons do not. The flash path's median offset is computed from the same biased values, so the bias carries into comparison_flash.csv. Witness's own `uptime_ms` is in the packet and could be used instead.
- **Scenario:** During Boost at about 300 m/s, a 30 ms latency gives roughly 9 m of apparent altitude and velocity error. That inflates the Boost/Coast RMS in summary.json and the PDF, and makes a state look LATE or EARLY relative to the expected transition time.
- **Confidence:** medium-high. The mechanism is clear from the code; the size of the latency on the bench is not measured.

### 3. GNSS error is compared at receipt time, not at the GGA epoch — medium
- **Where:** hitl_session.py:601-604 (NMEA stamped with `self._sim_s()` and `self.now()`); hitl_report.py:339 (`sim = [g[1] for g in gga]`), hitl_report.py:352-353 (interpolation against truth); gps_rf.py:764 (`parse_gga` keeps GGA `utc` but it is unused)
- **What:** Each GGA is aligned to the sender's sim time when the line arrives. The receiver's epoch was earlier by the receiver output latency plus link delay. The GGA UTC field would give the epoch directly and is parsed but never used for alignment.
- **Scenario:** The GPS horizontal and altitude RMS/max values, and the GNSS page plots, carry a bias of roughly speed times delay. This is largest in Boost and for slower GGA rates.
- **Confidence:** medium. The receipt-time stamping is certain; the delay magnitude depends on receiver configuration.

### 4. Sender socket timeout of 0.2 s applies to sendall and can abort a run mid-frame — medium
- **Where:** hitl_session.py:458 (`sock.settimeout(0.2)`), hitl_session.py:498-505 (`_send` uses `sendall`), hitl_session.py:1214-1217 (OSError abort)
- **What:** Since Python 3.5, the socket timeout passed to sendall is a total deadline for the call. If TCP back-pressure (ser2net or USB stalls) holds the send for more than 0.2 s, sendall raises socket.timeout after part of a frame is already written. The run is then aborted as "send failed", and the leftover partial frame misaligns the stream seen by Witness. The 0.2 s value was meant for recv in the reader thread.
- **Scenario:** Witness stalls on a flash write or GPS assist transfer and the socket buffer fills for a fraction of a second. The run aborts, and any later EXIT or frame may be misparsed.
- **Confidence:** medium. The sendall semantics are certain; how often the stall occurs on the bench is unknown.

### 5. PASS verdict does not check run completeness, and unexpected exceptions do not set aborted — medium
- **Where:** hitl_session.py:291-353 (`evaluate` has no check of `frames_sent` against `len(frames)`), hitl_session.py:1214-1221 (catches only OSError, RuntimeError, FileNotFoundError, ValueError), hitl_session.py:1240 (`evaluate` called unguarded)
- **What:** Any exception outside those types (for example KeyError, IndexError, TypeError, subprocess.TimeoutExpired) leaves `run.aborted` False. `evaluate` then judges only the transitions and status bits seen so far. The `FileNotFoundError` clause at line 1218 is unreachable because it is a subclass of OSError caught first at line 1214.
- **Scenario:** An unexpected error at 60 s of a 300 s flight leaves the state transitions that were already observed and HITL-active seen, so the run reports PASS with frames_sent well short of the profile.
- **Confidence:** medium. Requires a bug or an unforeseen exception, but the verdict has no safety net.

### 6. Exceptions in the end-of-run block skip the "run finished" event and leave the recorder open — medium
- **Where:** hitl_session.py:1240 (`evaluate`), hitl_session.py:1252-1256 (`self._recorder.finish`, `self._recorder = None`), hitl_session.py:1290-1291 (`set_activity("")`, final `_event("run", "finished")`); wos_hitl_tab.py:1621-1623 (`_run_finished` returns silently when `run.result` is missing)
- **What:** The finally block in `_send_loop` is not guarded from `evaluate` onward, except for the report and comparison steps. If `recorder.finish` raises (for example json.dump on an unexpected type, or write_attitude_csv), the function exits before `_recorder` is cleared and before the finished event. The GUI keeps "Writing run outputs" in its activity line and never shows the result or failure banner. The witnessOS handler also returns silently when there is no result.
- **Scenario:** A run ends, the outputs fail to write, and the GUI shows no result and no error, only a stuck activity label.
- **Confidence:** medium.

### 7. Regenerating a run ignores explicit vibration file overrides and overwrites imu960.npz — medium
- **Where:** hitl_outputs.py:341-352 (`regenerate` calls `ims.find_profile(csv_path, profile)` and ignores thrust/lateral overrides and full scale); hitl_outputs.py:173-179 (`write_comparison` saves imu960.npz from that stream); hitl_outputs.py:358 (called from `regenerate`); compare hitl_session.py:889-899 (live run uses `resolve_profile` with the overrides)
- **What:** A run that used `vibration_thrust_file` or `vibration_lateral_file` is regenerated with a different or missing vibration profile. The regenerated imu960.npz then overwrites the saved one, so the stored IMU input no longer matches what was sent in the run.
- **Scenario:** Running `python hitl_outputs.py runs/<timestamp>` on an older run with explicit vibration files changes the archived IMU record and the comparison numbers derived from it.
- **Confidence:** high on the code path; medium on how often explicit vibration files are used.

### 8. Monte Carlo tuning and reported statistics use the same fixed realizations — medium
- **Where:** kalman_mc.py:212-236 (`tune` coordinate search, scored with `cost` on `evaluate`); kalman_mc.py:171-172 (`evaluate` uses all `realizations()`); kalman_mc.py:186-197 (`cost`)
- **What:** Parameters are chosen by minimizing the worst ascent and descent errors over the same 8-per-run realizations that are then reported as worst_pct, p90 and nees. No held-out seeds or split are used, so the reported worst case is optimistic.
- **Scenario:** A tuned configuration looks better than it would on new vibration or dT draws, and KALMAN_TUNING_REPORT.md would overstate the margin.
- **Confidence:** high on the code, medium on the size of the effect.

### 9. RF GPS timeline uses wall clock, not IQ samples sent; no drift check — medium
- **Where:** gps_rf.py:568-569 (`started_at = time.monotonic() - 1.0` at first rate line), gps_rf.py:586-588 (`elapsed_s` is wall time since start), gps_rf.py:528-544 (local feed), gps_rf.py:704-716 (remote start, reader latency included); hitl_session.py:1007 and 1144 (sim t0 = started_perf + lead-in)
- **What:** The HackRF's sample count is never used. If TX underruns (the pipe feeder or SSH stream cannot keep up), the transmitted scenario falls behind wall time, and GPS position drifts relative to Witness's 0x0C timeline. hackrf_transfer's per-second report ("5.2 MB / 1.014 sec") carries the data needed to detect this, but the code only uses it to set the start time. On the Pi path the SSH hop adds latency to that start estimate, and gps_report["sim_t0_offset_s"] (hitl_session.py:1166-1168) records only the lead-in duration, not a measured alignment error.
- **Scenario:** A long Pi-played scenario with a brief underrun leaves the GPS fixes shifted by the accumulated lag. Nothing in the report shows it.
- **Confidence:** medium-low. Depends on how hackrf_transfer behaves under underrun, which I did not verify.

### 10. Power-cycle reconnect can open a connection after a stop was requested — low
- **Where:** hitl_session.py:1097-1106 (the identify loop checks `_stop_run` only before each attempt; `connect` at 1106 runs whenever a port was found)
- **What:** If disconnect or stop arrives during the up-to-12 s identify, the port is still found and the sender reconnects. `disconnect()` has already returned, so the GUI shows disconnected while a reader thread and socket stay open.
- **Scenario:** The user stops during a power cycle, then starts again. The orphan connection receives a second HITL session or stale events.
- **Confidence:** medium.

### 11. Reader thread can tear down a new connection after a quick reconnect — low
- **Where:** hitl_session.py:460 (`_closing.clear()` in `connect`), hitl_session.py:551-554 (reader starts a disconnect thread if it exits without `_closing` set), hitl_session.py:492 (join timeout 2 s)
- **What:** If the previous reader has not exited when connect() clears `_closing` (the join can time out), the old reader's exit path sees the flag clear and calls `disconnect()`, which closes the new socket.
- **Scenario:** A reconnect right after a disconnect on a slow link drops the new connection with no obvious error.
- **Confidence:** low-medium.

### 12. Stale hitl_active flag can skip HITL ENTER and mark HITL as seen — low
- **Where:** hitl_session.py:1158-1161
- **What:** If `view.hitl_active` is still True from the previous run, ENTER is not sent and `run.hitl_seen` is set immediately. The flag is only cleared by a status packet, and connect() resets the view, but a second run on the same connection does not reset it.
- **Scenario:** Witness has already left HITL (timeout) but the stale flag says active; the run streams without ENTER and then reports the HITL state as seen.
- **Confidence:** medium that the window exists; low impact in practice because EXIT normally clears the bit within a status period.

### 13. Scenario copy to the Pi has no timeout and Stop cannot interrupt it — low
- **Where:** gps_rf.py:675-691 (`RemoteHackRFPlayer.stage`), gps_rf.py:689-691 (`subprocess.run` for scp without timeout)
- **What:** A 2-3 GB scp runs inside the sender thread with no timeout, and nothing checks `_stop_run` during it. Stop can only take effect after the copy finishes.
- **Scenario:** A stalled network leaves the run in "Starting HackRF" with no way to cancel, and the GUI cannot start another run.
- **Confidence:** medium.

### 14. HackRF stop() result is ignored; the device may stay busy with no further action — low
- **Where:** gps_rf.py:598-616 (`HackRFPlayer.stop` returns False when hackrf_transfer does not exit); hitl_session.py:1127, 1134 and 1226 (return value discarded)
- **What:** The failure is logged through the player's log callback but not raised. The next run then fails later with an unclear "HackRF did not start streaming" error. The remote `pkill -INT -f '[h]ackrf_transfer -t '` at gps_rf.py:706 also matches any hackrf_transfer on the Pi.
- **Scenario:** A forced kill on Windows leaves the HackRF claimed; the next run fails at lead-in.
- **Confidence:** medium.

### 15. Hub power cycles are not rate limited and can run twice per run — low
- **Where:** hitl_session.py:982 (`HUB_POWER_CYCLE`), hitl_session.py:1127-1128 (cycle after a missed lead-in fix); hitl_outputs.py:256-270 (hub 2 cycle in `finally` whenever `restart` is set, even if the export never started)
- **What:** The design avoids loops, but a single run can cycle the ganged hub twice (lead-in retry, then FC fetch), and back-to-back runs each cycle again. The FC fetch cycle runs even when the Pi was unreachable.
- **Scenario:** Repeated failed runs power-cycle all ports on the Pi hub several times in a few minutes, resetting Witness and the GPS receiver each time.
- **Confidence:** medium that it happens; low concern for a single run.

### 16. kalman_replay GGA filter drops GA and GL talkers that parse_gga accepts — low
- **Where:** kalman_replay.py:100 (`\$G[NP]GGA`), compared with gps_rf.py:749 (`\$(G[PNL])GGA`)
- **What:** Galileo-only (GA) and GLONASS-only (GL) GGA sentences are ignored by the replay, so the replay can run with fewer GNSS fixes than the report uses.
- **Scenario:** A Galileo E1 RF run (galileo_rf) emits $GAGGA; the replay tuning proceeds without those fixes, and its GNSS-aided results do not match hitl_report.
- **Confidence:** low-medium; depends on the receiver's talker IDs.

### 17. Misleading lead-in warning on stop; unreachable except clause — low
- **Where:** hitl_session.py:1139-1143 (no-fix warning is logged even when stop was requested), hitl_session.py:1214-1218 (FileNotFoundError clause after OSError)
- **What:** A user stop during the lead-in is reported as "no GPS fix during the lead-in; the run continues". Missing hackrf_transfer is labelled "send failed" in the abort reason. The FileNotFoundError clause can never run.
- **Scenario:** Operators read a false "run continues" warning and a misleading abort reason in the log.
- **Confidence:** high.
