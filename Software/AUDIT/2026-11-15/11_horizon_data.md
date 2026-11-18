# Horizon live data path audit (wire -> in-memory model)

Scope: Software/Horizon: monitor_core.py, witness_protocol.py, ground_receivers.py, witness_link.py,
command_link.py, auto_connect.py, serial_discovery.py, nmea_capture.py, main.py (pump only), qt_common.py,
stall_watchdog.py, witnessOS/wos_monitor_tab.py, plus logs/. Read-only; only parser snippets run (python -I).
Not fully read: flight_data.py and the viewer/attitude widgets (other agent's facet).

## 1. Threads and packet path

```
USB serial / TCP (ser2net 7100-7102)                 [reader thread per input, daemon]
  read_serial  monitor_core.py:829  (serial timeout=0.2, opened :612)
  read_tcp     monitor_core.py:844  (socket timeout 0.2, :618-619, NO keepalive)
    1 _log_input()          :823  raw .bin write+flush, SAME try as decode (:836/:853)
    2 events.put("bytes")   :837  unbounded queue.Queue (:182), one hex line per chunk
    3 PacketDecoder.feed    witness_protocol.py:366  fixed length + EOF 0x0A, NO CRC
        on_error -> events.put("decoder_error")    :818  (one per discarded byte)
        on_packet -> on_decoded_packet  monitor_core.py:785
            CommandLink.feed (:792, unguarded)
            WitnessPublisher.publish (:794) -> local JSON feed 127.0.0.1 (witness_link.py:218)
            remote Tailscale publisher (:797)
            events.put("packet")   :805
    4 NmeaExtractor.feed + events.put("nmea")   :839/:856
  stream_feeder (:863): mirror of the HITL tab's TCP, runs on the HITL reader thread
GUI thread
  QTimer 30 ms main.py:229 -> MonitorPanel.process_events main.py:676 -> core.process_events :884
    drains <=20000 events: record_history (RingHistory, 300 samples/field :160), a console
    line per event, latest-per-type -> update_values :1157 -> Cell.set -> labels
    _write_console_lines :1119 -> debug_*.log write+flush (unguarded)
  QTimer 100 ms main.py:231 -> update_age_indicators (age labels, feed status)
  QTimer 250 ms main.py:230 -> auto_poll :295 (USB auto-connect)
witness-feed thread (witness_link.py:238, select loop) -> clients
  WitnessReceiver thread (ground_receivers.py:181) -> accept :85 -> on_record (flight view history)
  + latest-coalescing -> drain :109 -> tab poll every 100 ms (auto_connect.py:136)
command threads: _await_ack (:419/:423), gps-assist workers (:481/:517); transmit_command itself runs on the GUI thread
```
Hand-off interface to the widgets agent: `core.values[key]` Cells (strings), `core.raw_values` (status ints),
`core.history[key]` RingHistory (arrival-time stamped), `core.last_packet_time` (monotonic, stamped at display
time, see F11), and the JSON feed -> WitnessReceiver.drain()/on_record for the flight/attitude tabs.

## 2. Findings

| # | Sev | Conf | Where | One line |
|---|-----|------|-------|----------|
| F1 | Critical | confirmed | monitor_core.py:829-861, 1012-1030 | Any reader exception or TCP close ends the input permanently; TCP never reconnects, manual serial never reconnects |
| F2 | High | confirmed | monitor_core.py:618-619, 844-861 | Half-open TCP (Pi/Wi-Fi blip): no keepalive, no inactivity timeout; link silently dead, status still "Connected" |
| F3 | High | confirmed | monitor_core.py:836-842, 853-861, 1019-1021 | Raw-log write and decode share one try; a log OSError or other reader exception kills the reader AND closes the raw log |
| F4 | High | confirmed (log evidence) | monitor_core.py:944-952, 1119-1125; witness_protocol.py:371-378 | Debug log is 485 MB for a 26 min session (7 MB raw); ~1 GB/h; unbounded; fills disk over a pad wait |
| F5 | High | reproduced | witness_protocol.py:338-349, 371-378 | One error event per garbage byte plus O(n^2) resync scan under the GIL: 4 KB garbage 0.5 s, 16 KB 8.4 s; matches a real UI stall |
| F6 | High | plausible (PyQt6 default) | monitor_core.py:914-915; main.py:229-231 | Unguarded exceptions in GUI timer slots (disk full, flush error) hit PyQt6 qFatal (no sys.excepthook anywhere) |
| F7 | High | confirmed | witness_link.py:238-312 | Local feed thread dies on any accept()/setblocking OSError (or any non-OSError, silently), never restarted; flight/attitude tabs read this feed |
| F8 | Medium | reproduced | witness_protocol.py:394, 412-419 | Exception in on_packet/parse_packet after the frame was removed deletes one extra byte: 3 good frames -> 0 delivered |
| F9 | Medium | confirmed | witness_protocol.py:394-407, 209-306 | No CRC/sequence; bad-EOF resync drops the whole frame length first; a newer/longer layout is dropped with only console noise |
| F10 | Medium | confirmed | monitor_core.py:391-421, 612, 619 | Commands written on the GUI thread: serial has no write_timeout; TCP sendall on a 0.2 s timeout can send half a frame |
| F11 | Medium | confirmed | monitor_core.py:1105-1110, 1157-1162; main.py:684-691 | Staleness is per-packet-type age only, stamped at GUI pump time; no global link-lost banner; backlog after a blip is stamped "now" |
| F12 | Medium | confirmed | main.py:236-260 vs witness_protocol.py:88,664 | LOG_SLOTS_FULL (status byte1 bit 7) only a tile in the Telemetry tab; no banner like maintenance mode |
| F13 | Medium | confirmed | monitor_core.py:160, 1142-1155 | Graph history is 300 samples per field (about 30 s of sensors) |
| F14 | Medium | confirmed | monitor_core.py:938-944, 1142 | Two sources (Witness direct + Aperture): interleaved in one history, newest-wins display, no source shown, duplicates kept |
| F15 | Medium | plausible | witness_link.py:58-78; monitor_core.py:1034-1045, 1094-1096 | tailscale_ipv4() falls back to a CLI call (3 s timeout) on the GUI thread every 10 s while Tailscale is not up |
| F16 | Medium | confirmed | monitor_core.py:322-329, 556; serial_discovery.py:16 | USB auto-reconnect needs auto_connect on; 5-10 s gap, no backfill; Disconnect click turns auto off |
| F17 | Low | confirmed | monitor_core.py:182 | events queue unbounded in live mode |
| F18 | Low | confirmed | monitor_core.py:608 | connect() overwrites debug_log_file without closing the previous handle |
| F19 | Low | confirmed | ground_receivers.py:245-263 | Featherweight serial input: any exception ends the thread, no reconnect unless auto-connect |

## 3. Details

### F1 No reconnect (Critical)
read_serial/read_tcp (monitor_core.py:840-842, 859-861) put an "error" event and break. _handle_input_error
(:1012-1030) pops the stream, closes it and the raw log, sets status "Serial error on X". There is no reconnect
path for TCP at all and none for a manually connected serial port. Only USB auto_poll (:295) re-opens, only
Witness-named or learned devices, only while auto_connect is True; toggle_connection (:556) sets it False on any
Disconnect. Scenario: Pi at the pad, Wi-Fi blip or ser2net restart (or the Pi USB over-current trip: port
re-enumerates, ser2net port order shifts) -> recv returns b"" or raises -> reader gone, raw log closed, telemetry
frozen; only the status label changes. The operator must click Connect; a new .bin starts. Direction: reconnect
loop with backoff inside the TCP reader, keep the same raw log open across reconnects.

### F2 Half-open TCP (High)
create_connection timeout 3 s then settimeout(0.2) (:618-619). No SO_KEEPALIVE (enable_keepalive exists in
witness_link.py:81 but is only used for the remote feed). A Wi-Fi drop without FIN: recv keeps timing out
(`except socket.timeout: continue`, :857) forever; nothing notices that no bytes arrive. Only the per-section
"Data age" labels change colour (main.py:684-691). status_text stays "Connected: ...". Needs a no-byte watchdog
(about 3 s) -> link-lost state + reconnect, plus keepalive on this socket.

### F3 Raw log shares fate with decode (High)
_log_input (:823-827) runs inside the reader's try (:836, :853). A disk-full / AV-locked OSError in write/flush goes
to `except Exception` -> error event -> stream closed and raw log closed (:1019-1021): displays and raw log die
together. Decode-side errors are mostly caught inside PacketDecoder.feed (try at :368-419), so display failure
does not normally stop logging, but see F8. No fsync anywhere; flush only (OS crash or power loss loses the
tail). Raw log should be written by its own guarded step (failure -> report and keep reading), ideally a writer
thread with periodic fsync.

### F4 Debug log volume (High)
Evidence: logs/debug_20261111_201412.log = 485,408,387 bytes vs raw TCP_prs2026hitl_7100_20261111_201412.bin
7,072,093 bytes, 26 min session. First 3 M lines: 2.97 M BAD_PACKET lines (273 MB of 289 MB), 20.7 K RX hex lines,
12.6 K PACKET lines. Every chunk -> "bytes" event -> console/debug line (:947-949), every PACKET -> a line (:944),
every discarded byte -> a BAD_PACKET line (:950-952); all written and flushed each pump tick (:1123-1125).
About 1.1 GB/hour: a 3 h pad wait is 3+ GB. Disk full then cascades into F6. Mitigations: cap/rotate the
debug log, aggregate decoder errors ("N text bytes skipped"), drop RX hex lines when the .bin exists.

### F5 Per-byte error events and quadratic resync (High)
witness_protocol.py:371-378: for an unknown ID byte, find_valid_frame(1) scans the whole buffer (O(n)), emits
one on_error, and deletes discard_count bytes; with no valid frame following it discards 1 byte and repeats:
O(n^2) per feed. Witness mirrors NMEA and ESP-IDF log text on the same stream (nmea_capture.py docstring), so
text is always present. Measured on the real capture: chunk 4096 -> 391,385 unknown-ID errors, 5.9 s CPU
(1.2 MB/s); chunk 64 (small TCP/ser2net segments) -> 6,317,510 errors, 36.9 s CPU (0.19 MB/s). Pure garbage in
one feed: 4096 B = 0.52 s, 16384 B = 8.36 s (GIL held). Average load (4.5 KB/s) keeps up, but a burst (post-blip
backlog, noisy link, a long text dump) stalls the reader and, through the GIL, the GUI.
Correlation with logs/ui_stalls.log: of 6 recorded stalls only one is the live monitor: 2026-11-11T20:15:41, 1.5 s,
GUI thread in process_events -> queue.get_nowait while thread read_tcp is inside PacketDecoder.feed /
find_valid_frame and events.put (queue lock contention), plus a CSV loader worker (np.unique) competing for the
GIL. The others: unittest runs of witnessOS tests (2026-11-11T21:08:28/29/48; 1.2 to 3.5 s) and HITL stop_run
joining the sender thread on the GUI thread (00:32:38, hitl_session.py:970, other facet). The watchdog
threshold is 1 s, so shorter stalls are invisible.

### F6 Unhandled exceptions in GUI slots (High, plausible)
process_events (monitor_core.py:884) wraps per-event handling (:901-905) but NOT `_write_console_lines` (:914,
debug log write+flush: disk full/locked), `_refresh_nmea_status` (:915, handle.flush) or the discovery.remember
loop. The slot is connected directly (main.py:229). No sys.excepthook is installed anywhere in the repo (grep).
PyQt5.5+/PyQt6 call qFatal -> abort() on an unhandled exception in a slot when the default excepthook is in
place, so a full disk would kill the entire witnessOS app mid-flight. Even without abort the exception fires every
30 ms and console/status updates stop. Not run (no GUI allowed); this is the documented PyQt default, hence
"plausible". update_age_indicators and update_feed_status are also unguarded. Throttle/GuiQueue/Debounce do
guard (qt_common.py:249, 283, 386).

### F7 Local feed thread fragility (High)
WitnessPublisher._run (witness_link.py:238-312): one try around the whole loop catching only OSError and storing
self.error. accept() (:253) raising ConnectionAbortedError/ConnectionResetError (a client that resets between
SYN-ACK and accept; easy with WitnessReceiver's 1 s reconnect loop), connection.setblocking (:254) or a select
error ends the thread; finally closes every client and the listener. update_feed_status (monitor_core.py:1098)
shows "Horizon feed stopped" but only the remote publisher is restarted (:1092-1096). A non-OSError (e.g.
ValueError from encode_record in the snapshot line :262) kills it with error None, so no message at all. The
flight view/attitude tabs consume this feed, so they freeze while the Telemetry tab (fed directly from events)
keeps updating. Needs per-client try/except and restart-on-error.

### F8 on_packet exception loses following frames (Medium)
feed() deletes the frame first (:394), then parse_packet/on_packet run inside the same try (:412-414). The except
(:415-419) does `del self.buffer[0]`, removing the first byte of the NEXT frame. Snippet: three valid 0xFF
heartbeat frames, on_packet raises on the first -> 0 of 3 delivered. Triggers: CommandLink.feed (monitor_core.py:792,
unguarded), any parse_packet error, converter.parse_ground_station_status (0xF0).

### F9 Integrity and format drift (Medium)
Frames are validated only by ID, fixed length and trailing 0x0A: a bit-flipped payload with intact EOF is shown as
real data (USB direct has no CRC; ser2net adds none; Aperture CRC on the radio hop not verified here). No packet
sequence number, so loss is invisible. On a bad EOF (:394-407) the whole fixed-length frame is deleted before
searching, so a real frame starting inside that span is destroyed. A firmware with a different length for an
existing ID (0x01/0x02/0x03/0x08/0x09/0xE0) never matches the length tuples -> bad-EOF/discard storm and no
decoded packets of that type; only console lines report it, no "unrecognised packets: N" indicator.
0x08 (17) and 0x09 (26) lengths match the table. 0x09 with invalid position gives None lat/lon: displayed "null"
and skipped by record_history (isinstance filter :1146). NaN/inf floats: RingHistory stores NaN, json_safe nulls
them in the feed; nothing raises.

### F10 Command writes (Medium)
transmit_command (monitor_core.py:391) runs on the GUI thread. serial.Serial is opened with timeout only (:612,
serial_discovery.py:142): a stalled CDC endpoint can block write() and freeze the GUI. TCP: timeout 0.2 applies
to sendall; a full send buffer raises socket.timeout after a partial send -> half a 10-byte frame on the wire,
reported as TX_ERROR (:412), and the next command is misframed. _await_ack (:423) resends at most
ack_attempts=5 (1.5 s apart on TCP), bounded. Its `write` runs on a worker thread concurrently with other writers
(no write lock), so interleaving with a paced GPS-assist upload is possible. upload_assist (command_link.py:137)
can keep BEGIN/STATUS traffic going for up to 600 s on a dead link: continuous uplink while the downlink is needed
(Aperture airtime).

### F11 Staleness (Medium)
data_ages (:1105) is per packet type; last_packet_time is set in update_values (:1162) with time.monotonic() at the
GUI pump, not at arrival, and only for the newest packet per type per tick. After a GUI stall the age resets to
"fresh" although the data is old. After a blip, buffered backlog is stamped with arrival time (:789): graphs show
old data compressed at "now" and values look live. Packet `uptime` is displayed but never compared (reset or
reorder undetected). No link-level indicator and no "NO DATA N s" banner; after an input error the retained values
stay on screen with only the status line changed.

### F12 LOG_SLOTS_FULL display (Medium)
Bit 7 of status byte 1 is decoded (witness_protocol.py:88, 94, 664) and shown as a red "LOG FULL: NOT LOGGING"
tile inside StatusIndicator (main.py:81-150) on the Telemetry tab. Unlike maintenance (main.py:236-260: banner,
title prefix, console alert) there is no banner or alert; an operator on another tab will not see it. The same
applies to RESUMED AFTER RESET.

### F13 History depth (Medium, hand-off)
max_history = 300 (monitor_core.py:160): about 30 s of sensors at the measured ~10 Hz (16,169 sensor packets in
26 min). Graphs cannot show a whole flight.

### F14 Multi-source (Medium)
_handle_event keeps latest[packet type] regardless of source (:943) and record_history (:942) appends all sources
into one key; Witness over USB plus the same packets via Aperture give duplicated, differently timed samples in
one series and the readout flips between sources. No source shown in update_values.

### F15 Tailscale probe on the GUI thread (Medium, plausible)
update_feed_status (100 ms timer) -> start_remote_feed (monitor_core.py:1096, every REMOTE_RETRY_S = 10 s) ->
tailscale_ipv4: psutil finds nothing when Tailscale is down, then `tailscale ip -4` with timeout 3 on the GUI
thread (witness_link.py:71). wos_app.py defaults remote_feed to "auto". Up to a 3 s freeze each 10 s if the CLI
hangs. Not seen in ui_stalls.log.

### F16 USB reconnect (Medium)
Discovery refresh every 5 s plus open job plus RETRY_SECONDS (:321, :329): 5-10 s lost after a replug or
brownout; new raw .bin per connection (:361). Auto-connect opens only Witness-named or learned devices, max 2,
never generic adapters. A port busy in another process yields "AUTO_CONNECT[...] ... will retry" every 5 s
(no fighting). A hung-but-open CDC port (no error, no data) is never detected (see F2/F11).

### F17-F19
F17: live-mode events queue has no maxsize; a long GUI stall grows memory and the backlog costs GUI time on
resume (cap 20,000 events/tick). F18: monitor_core.py:608 leaks the prior debug log handle. F19:
FeatherweightReceiver.run (ground_receivers.py:245) ends on any serial exception.

## 4. Checked and OK
- Parser buffer growth is bounded (frame consumed or 1 byte discarded; waits at most ~102 bytes). No length field,
  so a corrupted length cannot swallow megabytes. Progress is guaranteed on any input.
- parse_packet, decode_landed_position, decode_maintenance_status are length-checked; NaN/inf handled
  (RingHistory NaN, json_safe null, decode_record rejects non-finite).
- Raw log written before decode, per chunk with flush (monitor_core.py:836-838); raw log opened before the reader
  thread starts (:614-615, :380). Decode errors do not stop the raw log.
- process_events per-event try/except (:901-905); console bounded (2000 lines, 400/tick); history bounded;
  command_link acks/status bounded (64/16); publisher _latest and _pending bounded.
- Local feed does no socket I/O on reader/GUI threads and drops slow clients (witness_link.py:286-289); remote feed
  keepalive on; WitnessReceiver has recv timeout, 1 s retry, sequence-gap report, 64 KiB line cap.
- Test mode isolated (:934-937); test source uses backpressure (:736).
- Time: arrival time.time() only used for graph X; ages use monotonic; NMEA csv uses UTC.
- Auto-connect never probes or opens generic adapters; nothing is written to the device without a user action.
- Command resend is bounded (5 attempts); DATA/STATUS excluded from ack logic.

## 5. Suggested priority
1 F1+F2 (reconnect, no-byte watchdog, link-lost banner), 2 F3 (isolate raw log, own writer, fsync), 3 F4+F5+F6
(aggregate decoder errors, cap debug log, fix O(n^2) resync, install excepthook and guard the pump), 4 F7 (feed
thread supervisor), 5 F8/F9, 6 the rest.
