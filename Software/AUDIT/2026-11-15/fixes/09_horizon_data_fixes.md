# Horizon data path: fixes (2026-11-16)

Nothing committed or pushed. Tests: 377 run / 1 error before (display agent's `landed_position`), 431 run / 0 errors after (includes the display agent's new tests; mine are `tests/test_link_resilience.py`, 34 tests).

Files touched: monitor_core.py, witness_protocol.py (decoder only; QUICK_COMMANDS kept; BOM and CRLF kept), witness_link.py, ground_receivers.py, command_link.py, main.py, new link_health.py, new tests/test_link_resilience.py, tests/test_remote_feed.py (Tailscale probe is now asynchronous). auto_connect.py and featherweight.py needed no change (the Featherweight input is in ground_receivers.py).

## Parser timing on the real capture (logs/TCP_prs2026hitl_7100_20261111_201412.bin, 7.07 MB, CPU time)

| chunk | before | after | packets | error events before / after |
|---|---|---|---|---|
| 4096 B | 4.56 s | 0.17 s | 21,024 / 21,024 | 391,386 / 0 |
| 64 B | 15.66 s | 0.30 s | 21,024 / 21,024 | 6,317,513 / 2 |

The result is chunk-size independent (identical packet counts; the old parser produced false 0x08/0x06 hits at 64 B). 16 KB of garbage: 8.4 s before, under 0.1 s now.

## Per finding

- **H1 / F1, F16, F19 reconnect: fixed.** `monitor_core.Link`, `_run_link`, `_read_loop`, `_start_link`, `disconnect`. Every serial, TCP and USB-auto input is a `Link` whose reader thread ends only on operator Disconnect. Any other end (recv b"", exception, unplug) keeps the Link in "reconnecting" and re-opens with backoff 0.5, 1, 2, 4, 5, 5... s forever, same source name, same RawLog (append). Status text: "Reconnecting to X (attempt n)". USB auto ports are excluded from auto_poll while reconnecting; an auto port whose name vanished is released when a different candidate appears. Featherweight (`ground_receivers.FeatherweightReceiver.run`) and the Witness feed receiver reconnect too (feed receiver: also with auto-connect off, once it has connected). Serial ports open with DTR/RTS de-asserted before open (`link_health.open_serial_quiet`), first open and every reopen.
- **H5 / F2 half-open: fixed.** Keepalive on TCP inputs (3 s idle, 1 s interval; `witness_link.enable_keepalive` takes parameters; SIO_KEEPALIVE_VALS on Windows, guarded options elsewhere) plus a watchdog: a TCP connection that delivered data and then none for 5 s (`TCP_NO_DATA_RECONNECT_S`) is reconnected. Not applied to serial ports on purpose (a reopen could toggle DTR/RTS); serial silence only shows as STALE. Held off for 20 s after a 0x7300 reset. Per-source silence: `core.link_snapshots[i]["seconds_since_last_byte"]`, `core.source_silence(name)`.
- **H4 / F3 raw log: fixed.** `link_health.RawLog`: write never raises and runs before, and independent of, decode. Failures are counted and shown in `core.log_warning` ("RAW LOG NOT WRITING ... N byte(s) not saved"); the file is re-opened every 2 s. Flush every chunk; fsync every 5 s from a separate thread. Decode errors are caught separately and never touch the log.
- **H3 / F4, F18 debug log: fixed.** `link_health.DebugLog`: 50 MB x 3 files (`debug_*.log`, `.1`, `.2`), every op guarded, previous handle closed in `connect()`. Decoder messages are coalesced to one summary line per second per source (`Coalescer`).
- **H6 / F5, F8, F9 parser: fixed (integrity part not done).** `witness_protocol.PacketDecoder` rewritten: linear scan, text runs (NMEA, ESP-IDF) skipped with no event, a binary run gives one summary event, the frame is consumed before callbacks so an exception loses only that frame, a false ID candidate costs one byte (not a whole frame). Counters in `decoder.stats()`: packets_by_id, text_bytes_skipped, garbage_bytes_skipped, unknown_ids, bad_frames (known ID, wrong EOF/length), undecoded, callback_errors; per link `bad_bytes`/`text_bytes`. Still no CRC or sequence (protocol limit). Known limit: a false ID candidate in garbage waits for up to 103 bytes before it is rejected, so frames directly behind it are delayed until more data arrives (as before).
- **H2 / F6 exception hook: fixed.** `main.install_global_excepthook` (sys.excepthook, threading.excepthook, sys.unraisablehook): traceback to `logs/horizon_exceptions.log`, rate-limited line to the console and a red label, no abort. Installed by `MonitorPanel.__init__` (idempotent). Timer slots (event pump, auto-poll, age indicators) wrapped by `MonitorPanel._guarded`; `core.process_events` guards console write, NMEA status, discovery.remember, history, link health (rate-limited `GUI_ERROR[...]`); NMEA csv write errors are counted, not raised.
- **H11 / F7 feed thread: fixed.** `WitnessPublisher._run` supervises `_serve`: any exception closes the clients, rebuilds the listener on the same port and carries on; accept errors are ignored. Health: `.healthy`, `.restarts`, `.last_error`, `.error` (set only while down); the feed status line reports restarts.
- **F10 commands: fixed.** `Link.write`: per-link send lock, TCP `tcp_send_all` (1 s deadline), serial `write_timeout` 2 s; any failure drops the connection, so a half frame is never followed by another and reconnect heals it. Acked-command resends are 2.5 s apart, 3 tries (was 0.5/1.5 s, 5) and stop when the link drops. `upload_assist` aborts after 3 consecutive unanswered rounds (retries spaced 2 s) or when the link drops. GPS-assist frame sends raise on a dead link.
- **F11 / F12 status and banners: fixed (core and main.py).** Cells `link_snapshots`, `link_summary`, `link_banner`, `log_warning` (refreshed 4 Hz from `process_events`/`update_link_health`). "LINK LOST - ..." when a source that delivered data is silent more than 3 s, never delivered data 5 s after connecting, or is reconnecting. The panel shows the banner, a title prefix "LINK LOST | " and emits `MonitorPanel.linkBannerChanged(str)`. After 0x7300 an orange "WITNESS RESETTING" replaces it. LOG_SLOTS_FULL red banner (status byte 1 bit 7). Log-file warning banner.
- **F15 Tailscale probe: fixed.** Runs on a worker thread (`_probe_tailscale`); `update_feed_status` collects the result; status shows "checking for Tailscale...".
- **F17 queue: fixed.** `BoundedEvents(20000)` drops the oldest display event (never link/error events); `events.dropped` is shown in `log_warning`. Raw bytes never use the queue. **F13/F14: not done** (note only: history depth 300, multi-source merge).
- **Item 12 (0x7300): handled.** Sending it sets `expected_reset_until` (20 s): the watchdog is held off, the banner reads "WITNESS RESETTING", the console says the drop is expected, and auto reconnect restores the link. No manual reconnect, no error dialog.
- **Firmware agents' notes.** (1) Ack BUSY with detail 0xC0F1: `_await_ack` sends the command once more after 1 s ("CONFIRM[...]" console line; the dialog already confirmed). (2) Other BUSY acks print "BUSY ... RETRY the command". (3) Upload STATUS wait is 30 s on TCP/radio (`ASSIST_STATUS_TIMEOUT_S`, `upload_assist(status_timeout=)`), chunks not doubled in that case. (4) `format_display_value("rssi", -128)` returns "RECEIVER RADIO FAILED (-128)". (5) DTR/RTS low before open (see H1).

## Tests added (tests/test_link_resilience.py, 34)

Parser: text between frames, 4 KB and 16 KB garbage floods (time bound, no per-byte events), random flood resync, callback and parse_packet exceptions (3 good frames delivered 3, audit: 0), counters, byte-at-a-time, real capture (linear and chunk independent). Helpers: backoff, RawLog failure/reopen/append, DebugLog rotation and never-raise, Coalescer, BoundedEvents, LinkStatus, DTR/RTS order, RSSI label. Loopback TCP: reconnect after server close with the same raw log (both connections in one file), reconnect forever while the server is down then operator Disconnect, half-open watchdog, stale and reset banners, raw-log failure keeps reader and decoder, decode failure keeps raw log, debug-log handle closed and noise aggregated, 0x7300 whole-frame send and expected-reset flag, BUSY 0xC0F1 resend, BUSY retry text. Also: upload bounds, publisher restart after a non-OSError, excepthook (main and thread) and contained timer slot, off-thread Tailscale probe, Featherweight reconnect appending to one raw log.

## Trickle-down effects

- UI: new items in the Witness monitor panel (link summary line, LINK LOST, LOG FULL, log warning, exception label); title prefix "LINK LOST | ". Status text is now "Connected: ...; Reconnecting to X (attempt n)".
- Source names are unchanged (`TCP_host_port`, COM port), so Flight View keeps one series across a reconnect.
- Files: one raw .bin per Connect click (name unchanged) now continues across reconnects; no marker bytes are written. Debug log rotates to `.log.1`, `.log.2` (150 MB total at most). New `logs/horizon_exceptions.log`. RX hex lines still go to the debug log.
- Commands: during a drop `transmit_command` fails with "is reconnecting; nothing sent". `core.serials`/`tcp_sockets` hold only currently open streams; `core.links` holds every input including reconnecting ones.
- Display agent: `core.link_snapshots` and `WitnessPublisher.healthy` are available; the RSSI -128 text only applies where `format_display_value` is used (telemetry_model formats on its own).

## Changes needed in files I do not own

- witnessOS `wos_app.py`: call `main.install_global_excepthook(...)` at startup and show `MonitorPanel.linkBannerChanged` in its status bar so LINK LOST is visible from every tab.
- `serial_discovery.SerialOpenJob`: open with `dtr=False, rts=False` before `open()` (use `link_health.open_serial_quiet`); the core only sets `write_timeout` afterwards.
- Flight View / ground_tabs: show per-source silence and the RSSI -128 text.

## Not done

Serial hung-port forced reconnect (deliberate, DTR risk); a device returning under a different COM name opens as a new source (the old link is released when a new candidate appears); CRC/sequence (protocol); history depth and multi-source merge (F13/F14). Observed: `test_remote_feed ... stuck_remote_viewer` is slow and can fail on a loaded machine (publisher latency is about 20 ms per publish with a client attached, same with the original code); it passed in the final full run.
