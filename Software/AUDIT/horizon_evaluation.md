# Horizon — audit evaluation

Method: read the cited code and ran synthetic byte streams through `PacketDecoder` (scripts in the scratchpad `audit_horizon/`: `d.py`, `e.py`, `f.py`). No source edited, no hardware touched.

## Summary table

| # | Title | Verdict | Severity |
|---|-------|---------|----------|
| 1 | Frame after a bad-EOF frame dropped (resync) | CONFIRMED (worse than reported) | medium |
| 2 | 23-byte sensors frame split read parsed as 21 bytes | CONFIRMED, harmless in effect | low |
| 3 | GPS upload can loop forever | CONFIRMED for "receiving, no progress" path only; restart path IS bounded | low-medium (call it medium) |
| 4 | BUSY/REFUSED answer to BEGIN is final | CONFIRMED | medium |
| 5 | Camera/Aperture uptime put on Witness time axis | CONFIRMED (viewer sync axis for camera rows; converter time_seconds for 0xF0/0xE1) | medium |
| 6 | P*.BIN scan stops at first bad record | CONFIRMED | medium |
| 7 | Blocking calls on GUI thread | PLAUSIBLE | low |
| 8 | Unsynchronised writes from several threads | PLAUSIBLE (consequence overstated) | low |
| 9 | Stale ground-station position shown as valid | NOT A BUG (by design, age shown) | low / none |
| 10 | Maintenance banner stuck after clear | CONFIRMED (conservative direction) | low |
| 11 | One accept() error ends local feed | PLAUSIBLE | low |
| 12 | Graph window only last 300 samples | CONFIRMED, by design; docstring stale | low |
| 13 | OTA `--no-wait` does not check byte count | PLAUSIBLE | low |
| 14 | "already complete" without length check | NOT A BUG (practically) | low |
| 15 | Decoder ignores 0x0E | PLAUSIBLE, unverified | low |
| N1 | 0xF0 (27/11) has the same split-read ambiguity, and it drops the Aperture GPS position | NEW, CONFIRMED | low-medium |
| N2 | Legacy 21-byte sensors frame misparsed when next packet's status byte 0 is 0x0A (not just split reads) | NEW, CONFIRMED (legacy firmware only) | low-medium |
| N3 | Exception in `on_packet`/parse deletes a byte of the NEXT frame | NEW, PLAUSIBLE | low |

## Details per finding

### 1. Resync after bad EOF — CONFIRMED, medium
Evidence: `witness_protocol.py:330-345`. `del self.buffer[:frame_length]` (332) removes the whole fixed-length frame *before* the resync search; `find_valid_frame(1)` (334) then skips offset 0 of what remains, and `del self.buffer[:1]` (344) deletes it when nothing is found.
Repro (`d.py`): a heartbeat with EOF 0x00 followed by a good heartbeat in the same feed gives 0 packets and 8 errors (both lost); bad + 2 good gives 1 packet. When the good frame arrives in a separate read it decodes, as the report says.
Worse than reported (`f.py`): the damage is not one frame. A single stray byte that happens to be a known packet ID (0xE0, 0x07, 0x01) in front of a clean 1 Hz stream (sensors 25 B + state 65 B + camera 19 B) makes the decoder treat the whole fixed length (up to 102 bytes for 0xE0, 74 for 0x07) as one frame, wait for it, then delete it all. Result: 12/15, 13/15, 14/15 packets decoded, i.e. 1 to 3 real packets (up to about 1 s of telemetry) lost per stray byte, for chunked and byte-by-byte arrival alike. Also adds up to ~1 s display latency while it waits for the bogus frame to complete.
Frequency: LoRa has hardware payload CRC (GROUND_STATION_PROTOCOL.md:21, 685) so radio-corrupted frames rarely reach the decoder. Realistic triggers are connecting mid-stream, boot/UART noise, a dropped byte on the Aperture UART, or a 1/65k CRC false pass. So rare per flight but it costs data exactly when it happens. Medium.
Fix: on a bad EOF, `del self.buffer[:1]` only, then continue the loop (the unknown-ID branch already does byte-wise resync via `find_valid_frame(1)` on the untrimmed buffer). Or search `find_valid_frame(1)` before deleting the frame. Consider also not waiting for a full 102-byte frame when a later offset already shows a valid frame.

### 2. 23/21-byte sensors ambiguity on split reads — CONFIRMED, low
Evidence: `witness_protocol.py:265` (`confirm_shorter` = {0x02}) and 294-301. Repro: a 25-byte sensors frame with payload[21]==0x0A split after 23 bytes yields one sensors packet and 2 unknown-ID errors.
Exploitable by normal traffic? Only on a read boundary that falls exactly at 23 or 24 bytes of a sensors frame *and* the trailing-u16 high byte equals 0x0A. Unsplit frames always take the 23-byte branch first (checked first, `buf[24]`). Impact is small: accel and gyro are at the same offsets so the spurious packet carries correct data; only `sensor_trailing_u16` is missing and two error lines appear. Low.
Fix: add 0x01 to `confirm_shorter`. Trade-off: a genuine 21-byte frame is then delayed until the next frame arrives (about 1 s at 1 Hz).

### 3. GPS upload retry loop — CONFIRMED (partly), medium
Evidence: `command_link.py:172-214`. The report is only half right.
- Bounded: the "restart" path (202-209) calls `begin_until_acked`, whose loop is `while ... time.monotonic() < deadline` (159), so after `resume_deadline_s` (600 s default, `monitor_core.py:487`) it returns False and the upload ends with "could not restart". The no-STATUS path is bounded (192). So "state failed/other -> restart forever" is NOT unbounded.
- Unbounded: the normal `receiving` path (210-214). If Witness answers STATUS with `receiving` and `chunks_received < expected` forever (a chunk Witness always rejects, bitmap window problem, bad offset), the loop resends and polls forever; the deadline is never consulted. Only Stop/Disconnect (`stop_event`) ends it. Progress lines show a flat N/M, which an operator can notice.
Fix: track the best `chunks_received`; if no progress for N rounds or `resume_deadline_s` since last progress, give up with a message. Low-to-medium (nothing is damaged, the uplink is occupied).

### 4. BUSY/REFUSED on BEGIN is final — CONFIRMED, medium
Evidence: `command_link.py:158-165`; `ack.get("result") == ACK_OK` returns False on any non-zero result (BUSY = 4 "try again later", ACK_RESULTS at `witness_protocol.py:148`). The upload then aborts at once with the misleading "did not acknowledge BEGIN (link down...)". On the resume/restart paths it yields "link did not come back".
Fix: in `begin_until_acked`, treat busy (4) as retry until deadline (sleep 1 s); report the real `result_name` for other refusals.

### 5. Time axis of camera / Aperture rows — CONFIRMED, medium
Evidence: `viewer_data.py:850-853` `sync_times_array` prefers `uptime` for every row; for camera rows (`witness_protocol.py:184-187`, `converter` same) `uptime` is the Iris MCU clock and `fc_uptime` is Witness's. `row_times_array` (844-847) and `converter.py:797` use `time_seconds`/`fc_uptime` for those rows, so the two axes disagree for camera rows. The scalar twin `sync_time_seconds` (774-784) has the same order, and its docstring ("camera packets carry the flight computer's fc_uptime") is wrong: camera rows carry both. 0xF0 `uptime` is Aperture's clock (`converter.py:126`) and 0xE1 `uptime` the Iris clock, yet both become `time_seconds` (`converter.py:797`: only `fc_uptime` is preferred) and the sync time.
Effect: on the default "auto (Witness uptime)" axis camera current/battery traces and Aperture RSSI/GPS traces sit at the wrong time relative to Witness traces, and any state-transition alignment (`find_sync_times`) uses them. Post-flight analysis only; no live path. Offset size depends on boot time differences (unverified, likely seconds to minutes).
Fix: prefer `fc_uptime` over `uptime` for `packet_type == camera`; give 0xF0 / 0xE1 rows no Witness-clock time (blank `time_seconds`/sync) or label them separately.

### 6. P*.BIN scan stops at first bad record — CONFIRMED, medium
Evidence: `converter.py:695-697` (`break`), 730 (`stats.discarded = len(data) - offset`). Any unknown `source` byte (including a new source added by future firmware, which is more likely than a bit flip: `PACKET_SOURCES` has only 1-5), bad length, or missing 0x0A ends the scan; the whole tail of the flight is dropped from the CSV and only counted as discarded bytes. Raw .BIN is untouched, so it is recoverable.
Fix: on a bad record, scan forward byte-wise for the next offset where (source valid, length>=2, frame[-1]==0x0A, timestamp plausible vs previous) and continue; count discarded bytes per gap; surface the warning in the GUI. Accept unknown sources by length when the record is otherwise sane.

### 7. Blocking calls on GUI thread — PLAUSIBLE, low
Evidence: `monitor_core.py:1042` (`tailscale_ipv4()` from `start_remote_feed`, itself called from `update_feed_status` every `REMOTE_RETRY_S`=10 s, `main.py:654-656` 100 ms timer), `witness_link.py:58-78`, `monitor_core.py:618`. psutil is installed in the venv (7.2.2), so the 3 s CLI fallback only runs when Tailscale has no interface and `tailscale` is on PATH; `tailscale ip -4` on a stopped daemon usually returns fast. `socket.create_connection(timeout=3.0)` on Connect to an unreachable host freezes the window up to 3 s (once per click). The stall log (`ui_stalls.log`) shows no stalls from these paths, only from test runs. Low.
Fix: run `tailscale_ipv4` and the TCP connect off the GUI thread (or cache the lookup).

### 8. Concurrent writes to one handle — PLAUSIBLE, low
Evidence: `monitor_core.py:405,419-420,477,505-521`, `command_link.py:109,129,182,188`. Two threads can write the same pyserial/socket object (resend thread, upload worker, GUI send). No lock exists. The report's "console can report ACK for a damaged command" is wrong: an ACK only arrives if Witness parsed a valid frame matching opcode and data. The realistic failure is a corrupted/discarded frame that the retry logic repairs, or (remote) a mis-framed fragment parsed as another command. Needs overlapping sends during an upload. Low.
Fix: a per-target `threading.Lock` around each `write` (wrap `write` once where it is created).

### 9. Stale Aperture position as valid — NOT A BUG, low
Evidence: `converter.py:139-150`; comms.h:648-651: fix age 0xFFFF means none or >=65.5 s, "Position fields are zero while no fix has been seen". Non-zero position with age 0xFFFF is therefore a last-known fix, which for a stationary ground station is exactly the pad-position fallback. The consumer shows it: `pad_reference.py:118-128` labels it "fix age >65 s" and q0. The telemetry table's fix-age row shows an invalid/holding state for None. Only cosmetic: the lat/lon rows read "Live" while the age row says invalid.

### 10. Maintenance banner — CONFIRMED, low
Evidence: `main.py:43,230-240`, `monitor_core.py:672-678`. After clear/disconnect `raw_values` is empty and the early return leaves the banner/title as is until the next state/sensors packet (1 Hz) arrives; conservative failure direction. A source sending only heartbeats never clears it, but heartbeat is not a flight-mode source on the current downlink. Fix: on `_clear_telemetry` also reset `_maintenance_on`; optionally add heartbeat/witness_debug to the keys.

### 11. accept() error ends local feed — PLAUSIBLE, low
Evidence: `witness_link.py:253,306-308`. The outer `except OSError` sets `error` and the thread exits; `publish()` then fills `_pending` and drops. For the local publisher `update_feed_status` (`monitor_core.py:1098`) only shows the message, no restart. On Windows a select-readable listener raising ECONNABORTED is uncommon. A non-OSError exception would kill the thread without even setting `error`. Fix: catch OSError per accept and continue; restart the local publisher like the remote one.

### 12. Graph history 300 samples — CONFIRMED (by design), low
Evidence: `monitor_core.py:160,1153`; docstring at 887-889 ("keeps every sample") is stale. At 1 Hz downlink it is 5 minutes, which is plenty for live view; over USB at higher rates it is seconds. Full data is in the raw logs and exports. Fix: fix the docstring; optionally size by time.

### 13. OTA `--no-wait` — PLAUSIBLE, low
Evidence: `witness_ota.py:146-149,241-246`. Byte count printed, not compared. Witness's HTTP 200 only follows its own image validation (not verifiable from Horizon), and the default path verifies version plus app_state valid. Cheap hardening: compare `result["bytes"]` with `len(image)`.

### 14. "already complete" — NOT A BUG (practically), low
Evidence: `command_link.py:202-204`, `monitor_core.py:491-492`. BEGIN is acked OK first and `request_status` clears any older matching 0x07 before polling (125), so a stale "complete" can only come from Witness itself after accepting a BEGIN with a (possibly different) CRC/length. Per the protocol doc a BEGIN with a different id/length/CRC starts over, so this would be a firmware bug. Optional: compare `st["length"]` with `len(blob)`.

### 15. 0x0E not known — PLAUSIBLE, low (unverified)
Evidence: `witness_protocol.py:260`; comms.h:838-860 says 0x0E is "Never forwarded", which argues against an echo on the normal telemetry path. If it were echoed, each 150-byte frame would cost many error lines and risk false frames. Cheap fix: add `0x0E: (148,)` to `host_payload_lengths` (4 + 8x18 payload bytes) once confirmed.

### N1. 0xF0 split-read ambiguity — NEW, CONFIRMED, low-medium
Evidence: `witness_protocol.py:254,265`: 0xF0 has `(27, 11)` and is not in `confirm_shorter`. If a read ends between frame byte 13 and 28 and frame byte 12 (payload[11], the first byte of longitude) is 0x0A, the decoder returns the 11-byte legacy form. Repro (`e.py`): lon 17.0 E, split at 20 -> one packet with `latitude=None` and 16 error lines; the whole frame at once decodes lat 48.0. Longitudes 16.78-18.45 E hit it (first byte 0x0A); the pad-position fallback and map position are lost for that update. Fix: add 0xF0 to `confirm_shorter` (also guards the legacy frame; Aperture legacy firmware is old).

### N2. Legacy 21-byte sensors frame misparsed, not only on split — NEW, CONFIRMED, low-medium
Evidence: `witness_protocol.py:294-301`, `converter.py:573-587` (the docstring's "followed by any valid packet is unambiguous" only holds for state, where the second candidate's EOF index is the next packet ID; for sensors it is the next packet's second byte, the status byte 0). Repro (`e.py`): a stream of legacy 21-byte frames with status 0x0A00 decodes 3 of 5 sensors packets with 2 errors; status 0x0000 decodes 5 of 5. Status byte 0 = sensor-ready flags (comms.h:138-152); 0x0A (low-g ready + IMU temp ready, heartbeat/gyro/baro not) is a startup-time state, so this affects legacy firmware only, briefly. Current 23-byte firmware is not affected. Fix: after a 21-byte frame check, require the byte at index 23 to be a known packet ID or end of buffer before accepting the 23-byte candidate.

### N3. Exception handling deletes the wrong byte — NEW, PLAUSIBLE, low
Evidence: `witness_protocol.py:332,353-357`. The frame is deleted at 332 before `parse_packet`/`on_packet`; if either raises (e.g. `command_link.feed` in `on_decoded_packet`, `monitor_core.py:758`, is outside any try), the generic handler runs `del self.buffer[0]`, removing a byte of the next, innocent frame and triggering the cascade in finding 1. Nothing is known to raise today. Fix: separate try around parse/on_packet that does not touch the buffer.

## Significant issues to act on
- 1 (CONFIRMED, medium; no high/critical items survived review): bad-EOF resync deletes the whole fixed-length frame and skips offset 0, so one stray known-ID byte costs 1-3 real packets; delete 1 byte and resync on the untrimmed buffer.
- 4 (CONFIRMED, medium): BEGIN answered BUSY/REFUSED aborts the GPS upload; retry BUSY until the deadline and show the real reason.
- 3 (CONFIRMED, medium): `receiving`-with-no-progress loop is unbounded; add a no-progress limit.
- 5 (CONFIRMED, medium) and 6 (CONFIRMED, medium): post-flight CSV/viewer integrity (camera/Aperture clocks on the Witness axis; P*.BIN tail dropped at the first bad or unknown record).
