# Horizon — audit findings

## Overview
- Read-only audit of Software/Horizon (main, monitor_core, command_link, gps_assist, witness_ota, witness_link, witness_protocol, converter, viewer_board, telemetry_model, viewer_data time-axis code). No source edited, no hardware or network touched, nothing committed.
- Unit tests run offscreen: 349 tests, all OK.
- Decoder findings 1 and 2 were reproduced with synthetic byte streams through PacketDecoder (not with hardware).
- Packet layouts checked against Horizon/comms.h: sensors (21/23), state (63/62), camera, heartbeat, command, command ack (0x06), transfer status (0x07), witness debug (0xE0), iris debug (0xE1) — offsets and scales match. Ack matching fields match the echo layout.
- Ephemeris subframe packing in gps_assist.subframes() reads back consistently with decode_skytraq_ephemeris(); CRC-16 and blob/chunk framing match command_link.
- Witness OTA image check offsets (esp_image_header, esp_app_desc) are internally consistent; the reboot wait only succeeds on the new version plus app_state valid.
- witness_link snapshot/pending sequencing has no duplicate or gap logic error on attach.
- Not read in depth: viewer.py (3510 lines; only its load/threading path was checked), csv_columns.py, any_csv.py, serial_discovery.py, witness_download.py, witness_pi_agent.py, attitude/flight modules.
- Highest-priority item: finding 1 (decoder drops the frame that follows a corrupted one).

## Findings

### 1. Frame right after a corrupted frame is dropped (resync bug) — high
- Where: `Software/Horizon/witness_protocol.py:330-345` (PacketDecoder.feed bad-EOF branch): `del self.buffer[:frame_length]` at line 332 runs before `find_valid_frame(1)` at line 334, and `del self.buffer[:1]` at line 344 then deletes byte 0 of the remaining stream.
- What / Scenario: the first byte after a corrupted frame is never a resync candidate and is deleted when no later frame is found. A valid frame that starts right after a bad one is lost. Reproduced: a heartbeat with EOF 0x00 followed by a valid heartbeat in the same read gives zero packets and a run of "unknown packet ID" errors. When the good frame comes in a later read it decodes, so the loss depends on read boundaries. Over a noisy radio path (Aperture/LoRa bridged as TCP) this loses telemetry exactly when the link is bad. Fix: search the untrimmed buffer, or search from offset 0 of the remaining bytes.
- Confidence: high (reproduced).

### 2. Sensor frame split after 23 bytes is mis-parsed as a 21-byte frame — medium
- Where: `Software/Horizon/witness_protocol.py:265` (`confirm_shorter = frozenset((0x02,))`, so 0x01 is not included) and `:294-301` (frame_payload_length returns 21 as soon as byte 22 is 0x0A).
- What / Scenario: with a 23-byte sensor payload, the byte at payload index 21 is the trailing u16 high byte. If a TCP read ends exactly after 23 bytes of the 25-byte frame and that byte is 0x0A, the decoder accepts a 21-byte frame, emits a spurious sensors packet, then reads the real trailing bytes as unknown IDs. Reproduced: one bogus sensors packet plus two unknown-ID errors. Probability is about 1/256 per split sensor frame at that boundary. Fix: add 0x01 to `confirm_shorter`, as already done for 0x02.
- Confidence: high (reproduced).

### 3. Acknowledged GPS upload can retry forever — medium
- Where: `Software/Horizon/command_link.py:172-214` (upload_assist main loop). The resume deadline is checked only on the no-STATUS path (line 192). The restart path at 205-209 has no bound.
- What / Scenario: if Witness keeps answering STATUS with state "receiving" while the same chunks stay missing, or keeps reporting other states, the loop resends and polls indefinitely. Only Stop/Disconnect (`stop_event`) ends it; the console shows progress lines but no timeout. Fix: a deadline or stall counter based on received-chunk progress.
- Confidence: high (code path); how often it occurs depends on Witness behaviour.

### 4. BUSY (and REFUSED) answer to BEGIN is final, not retried — medium
- Where: `Software/Horizon/command_link.py:158-165` (begin_until_acked returns on any ack, line 161-162 returns `ack.get("result") == ACK_OK`); callers at 167, 196 and 206.
- What / Scenario: comms.h defines IRIS_ACK_BUSY as "try again later". Any non-zero result ends the upload at once with "Witness did not acknowledge BEGIN" (or "link did not come back in time" on resume). A busy Witness at the start of a launch-day upload therefore fails instead of waiting. Fix: keep retrying on BUSY (and on refusals that clear) until resume_deadline_s.
- Confidence: high (code path).

### 5. Other devices' uptime is put on the Witness time axis — medium
- Where: `Software/Horizon/converter.py:797` and `:804` (iter_log: `time_seconds` taken from `fc_uptime` else `uptime`); `Software/Horizon/viewer_data.py:850-853` (sync_times_array prefers `uptime` for every row) against `:844-847` (row_times_array prefers `time_seconds`).
- What / Scenario: 0xF0 `uptime` is Aperture's clock (SECTION_SPECS "Ground-station uptime"), and 0xE1 `uptime` is the camera MCU's clock (comms.h IRIS uptime). Both differ from Witness's boot time. Camera rows (0x03) carry Witness `fc_uptime` and camera `uptime`; the converter uses `fc_uptime`, but sync_times_array uses camera `uptime` first, so the two time columns disagree for those rows. Effect: rows from these packets land minutes away from Witness rows on the sync axis and on time_seconds.
- Confidence: high on the mapping; impact depends on how the converted CSVs are used.

### 6. P*.BIN packet log stops at the first bad record — medium
- Where: `Software/Horizon/converter.py:695-697` (`break` on unknown source, bad length or missing 0x0A) and `:730` (`stats.discarded = len(data) - offset`).
- What / Scenario: one corrupt record, such as a partial write at power loss or a bit error in a length byte, ends the scan. Everything after it is counted only as "discarded bytes" with no resync, unlike the wire decoder (iter_log). The later part of a flight can be silently omitted from the CSV and viewer. The CLI prints the discarded count, but the GUI path does not warn. Fix: scan forward for the next valid record header and report the gap.
- Confidence: high on behaviour; frequency depends on flash reliability.

### 7. Blocking calls on the GUI thread (remote-feed retry, TCP connect) — medium
- Where: `Software/Horizon/monitor_core.py:1090-1097` (update_feed_status calls start_remote_feed every 10 s) and `:1042` (tailscale_ipv4()); `Software/Horizon/witness_link.py:58-78` (tailscale_ipv4 runs `tailscale ip -4` with a 3 s timeout); called from `Software/Horizon/main.py:654-656` (100 ms timer). Also `Software/Horizon/monitor_core.py:618` (`socket.create_connection(..., timeout=3.0)` in connect()).
- What / Scenario: with the remote feed enabled and Tailscale down, the psutil path finds nothing and the CLI is run on the GUI thread every 10 s, freezing the window for up to 3 s. A TCP connect to an unreachable host freezes the GUI for up to 3 s. Fix: move both onto a worker thread, or cache the Tailscale lookup.
- Confidence: medium (freeze length depends on the tailscale CLI).

### 8. Unsynchronised writes to one serial or TCP handle from several threads — medium
- Where: `Software/Horizon/monitor_core.py:405` (GUI transmit), `:419-420` (ack resend thread uses the same `write`), `:477` (send_gps_assist worker), `:505-521` (upload worker); `Software/Horizon/command_link.py:109`, `:129`, `:182`, `:188` (write calls from those threads).
- What / Scenario: when an ack resend or an upload is running and the operator sends another command, frames from both threads can interleave on the same port. A 10-byte command interleaved with a DATA frame corrupts both; Witness discards them and the retry logic hides it, but the console can still report ACK for a command that was damaged on the wire. Fix: one write lock per target, held for each frame.
- Confidence: medium (needs overlapping sends; pyserial writes are not atomic across threads).

### 9. Stale ground-station position shown as valid, without an age — medium
- Where: `Software/Horizon/converter.py:139-150` (parse_ground_station_status): `has_position = age != 0xFFFF or position != 0` (line 140) and `gps_fix_age_ms` set to None when 0xFFFF (line 149).
- What / Scenario: comms.h defines fix age 0xFFFF as "no fix, or 65.5 s or older", and the position fields keep the last fix. If the Aperture GPS has had no fix for over 65 s but the last position is non-zero, the parser reports latitude, longitude and altitude as valid with fix age None. The telemetry table then shows them with quality "Live" (`Software/Horizon/telemetry_model.py:192`). This is the pad-position fallback the packet is meant for, so a stale pad location can be presented as live. Fix: report the position as last-known and keep the age visible.
- Confidence: high on the code path; impact depends on what consumes the position.

### 10. Maintenance banner: narrow status sources, and stuck after disconnect — low
- Where: `Software/Horizon/main.py:43` (MAINTENANCE_STATUS_KEYS: state and sensors only); `:230-240` with the early return at `:233-234` when no raw value exists; `Software/Horizon/monitor_core.py:672-678` (_clear_telemetry clears `raw_values`).
- What / Scenario: heartbeat and witness_debug packets also carry the status word but do not drive the banner. On disconnect or test start, `raw_values` is cleared, the handler returns early, and the red banner plus the "MAINTENANCE" title stay until a new state or sensors packet arrives. The failure direction is conservative, but a source that sends only heartbeats would never clear it.
- Confidence: medium.

### 11. One accept() error ends the local feed for good — low
- Where: `Software/Horizon/witness_link.py:253` (accept inside the main try) and `:306-308` (outer `except OSError` ends the thread and sets `error`).
- What / Scenario: an OSError from accept (a client reset before accept, or a race on the non-blocking listener) stops the publisher thread permanently. The local feed status then shows "stopped" (monitor_core.py:1098-1099), but unlike the remote feed it is never restarted. publish() keeps filling `_pending` and then drops records silently (`dropped` counter). Fix: catch OSError per accept and continue.
- Confidence: medium.

### 12. Graph window is silently the last 300 samples — low
- Where: `Software/Horizon/monitor_core.py:160` (`max_history = 300`), `:1153` (RingHistory(self.max_history)); the docstring at `:887-889` says graph history keeps every sample.
- What / Scenario: at the sensors stream rate, the graph may show only a few seconds of a flight, with no indication. The graph looks complete when most of the flight is missing. Fix: size the history by time window, or label the visible window on the plot.
- Confidence: high on code; impact depends on packet rates.

### 13. OTA client --no-wait exits 0 without checking the byte count — low
- Where: `Software/Horizon/witness_ota.py:241-246` (cmd_upload prints `result.get("bytes")` and returns 0 with --no-wait) and `:146-149` (upload accepts any 200 response without comparing the count).
- What / Scenario: the byte count reported by Witness is printed but never compared with len(image), and --no-wait skips the reboot and version check, so a short write is reported as success. The wait path does verify the version. Fix: compare `bytes` with len(image) before returning success.
- Confidence: medium.

### 14. "already complete" accepted without a length check — low
- Where: `Software/Horizon/command_link.py:202-204`; transfer_id defaults to 1 for every upload (`Software/Horizon/monitor_core.py:491-492`).
- What / Scenario: a STATUS with state "complete" for the same transfer_id is treated as success. The status carries the blob length but the code does not compare it to len(blob). If Witness does not reset on a new BEGIN, an earlier completed upload with the same id and a different blob would be reported as delivered. The real behaviour depends on firmware not in Horizon.
- Confidence: low.

### 15. Decoder does not know 0x0E (HITL IMU batch) — low
- Where: `Software/Horizon/witness_protocol.py:254-260` (payload_lengths and host_payload_lengths; only 0x0C is listed as host). comms.h:838-860 defines 0x0E as host-to-Witness USB with a fixed frame length.
- What / Scenario: if Witness echoes 0x0E on USB as it does 0x0C, the monitor logs an error per discarded byte and resyncs byte by byte. Payload bytes that look like a frame ID followed by 0x0A at the right offset could then decode as false telemetry. The echo was not verified. Fix: add the 0x0E length to host_payload_lengths, once confirmed from comms.h.
- Confidence: low.
