# Aperture fixes (agent 08), 2026-11-15

Scope: `Software/Aperture/softwareattempt2` (not `tx_slot.h`, `comms.h`). Nothing committed, flashed or run on hardware. Host logic tests pass; firmware build result is in section "Build".

Files changed: `main/main.cpp` (rewritten loop), `main/ra01.cpp`, `main/ra01.h`, `sdkconfig.defaults`, `sdkconfig` (generated, edited by hand to match), `FIRMWARE_HANDOFF.md`. New: `main/frame_ring.h`, `main/command_parser.h`, `main/uplink_pacing.h`, `tests/aperture_logic_test.cpp`.

## Status per finding

| # | Finding | Status |
|---|---|---|
| 1 A1 | Init failure permanent | fixed (untested on hardware) |
| 2 A2 | No health check / TX leaves standby | fixed (untested on hardware) |
| 3 A3 | BUSY wait sleeps a tick | fixed |
| 4 A4/W10 | Uplink back-to-back | fixed (pure logic tested) |
| 5 A5/A6 | USB stalls RX / log text in stream | fixed; log text reduced, not removed |
| 6 | RX clear-after-read race | fixed |
| 7 | Stack, watchdog | fixed |
| 8 | Command parser resync, front end OFF | fixed (no checksum possible, see below) |
| 9 A7 | Boot TX power docs | fixed in Aperture docs; mismatch reported |
| 10 | DTR/RTS reset | report only, no config change |

## 1. Radio init retry (A1)
`main.cpp` app_main loop: `radio.initialize()` is retried with backoff 1, 2, 4, then every 10 s forever; `esp_restart()` only after 60 consecutive failures (~10 min). The 0xF0 status frame keeps going; while the radio is down (or the RX front end is off) its RSSI byte is **-128** (normal range -127..0; the frame has no spare field, lengths unchanged). `Ra01::initialize()` is now re-entrant (RESET pulse, SPI bus created once) and includes Witness's packet-RAM round-trip probe (`A5 3C 69 96`), so an absent radio returns `ESP_ERR_NOT_FOUND`.

## 2. Health and recovery (A2)
- `Ra01::transmit()` = `transmit_sequence()` + a tail that ALWAYS does ClearIrq + SetRx (retry via SetStandby + SetRx). If RX still cannot be entered `rx_fault()` is set and the main loop runs the health check at once.
- `Ra01::health_check()` (main loop: after 1 s without a packet, on `rx_fault()`, or after 3 consecutive receive SPI errors): GetStatus chip mode must be RX (5); GetPacketType must be LoRa (a chip reset by a supply dip comes back as GFSK/standby, which a mode check alone would miss); GetDeviceErrors counted and cleared. Wrong mode: SetRx re-issued and re-verified (`rearms`). Reset chip, or 2-3 consecutive failures: full `initialize()` through the RESET pin (`reinits`). If that fails the main loop falls back to the init retry loop.
- Stuck BUSY: timeout is 100 ms (was 500 ms), and once a timeout happened later commands give up after 5 ms (`busy_stuck_`) until one succeeds. A whole transmit with a dead chip therefore costs about 100 ms + 5 ms x commands, not 5 s, and leads to reset/re-init via the health check (3 failed checks = about 3 s).
- Counters (`rearms`, `reinits`, `busy_timeouts`, `device_errors`, `crc_errors`, `rx_overruns`) in `Ra01::Stats`; reported by log (see 5). No spare 0xF0 field.

## 3. BUSY wait (A3)
`wait_until_ready()` spins 300 us on `esp_timer_get_time()` (like Witness) and only then sleeps ticks. TX completion poll spins at 250 us instead of a 10 ms tick. Added a DIO1 interrupt (RxDone/TxDone/CrcErr, already routed to DIO1 by SetDioIrqParams) that wakes the main loop (`Ra01::wait_for_event`), with the 10 ms poll as fallback if the ISR cannot be installed or the line is not wired.
Estimates (CALCULATED from command counts and SPI at 8 MHz, NOT measured; needs a bench run):
- RX to USB: before about 5-7 commands x up to 10 ms + 10-20 ms poll = 50-90 ms; after about 1-2 ms (DIO1 wake under 1 ms plus six commands of 50-100 us each).
- Post-uplink deaf time: before about 11 commands x 10 ms + 10 ms airtime + 10-20 ms poll = 130-150 ms; after about 10 ms airtime (10-byte frame at SF7/500) + 2-3 ms setup + SetRx = 13-15 ms.

## 4. Uplink pacing (A4 + W10)
`main/uplink_pacing.h` (pure): `Pacer` allows at most `kMaxBurst`=3 sends, then requires `kQuietMs`=1500 ms since the last send (also when the queue is not empty, also without GPS: it uses free-running `esp_timer` time). `Queue<10,64>` drops exact duplicate frames already queued; queue is 64 (was 32, on the stack; now static). The rule is ANDed with the unchanged `tx_slot::aperture_may_send` (slot window and 800 ms burst rule stay). Combined effect: Witness's 1 s hold-off after each command is followed by at least 0.5 s free before the next burst. With GPS slots the next burst starts at the next 800-950 ms window after the quiet time.
Consequences: throughput is about 3 commands per 1.6-2.6 s. A 32-frame GPS-assist upload takes about 20-28 s instead of 5 s; PC retries (Horizon `send_reliable`) see the same latency as before for the first 3, more for later ones.

## 5. USB output (A5, A6)
- `main.cpp`: all USB output (0xF0, acks, received LoRa packets, byte echo) goes through `FrameRing<3072>` (`frame_ring.h`, whole frames, drop-OLDEST, drops counted) drained by task `usb_tx` (priority 2, 3 KiB stack, 20 ms write timeout). `usb_serial_jtag_write_bytes` writes a whole item or nothing, so a frame is never split. The radio loop never blocks on USB. A host that does not drain loses the oldest frames; counted as `usb_drop`.
- Echo is one ring entry per received chunk (was one entry per byte, 8+ bytes ring overhead each). USB RX buffer 128 -> 512 B.
- Reporting: no spare field in 0xF0, so a `counters:` WARN line (at most once per second, only when a counter changed) and a once-per-minute INFO line. Note the INFO line only appears because the app tag is raised to INFO.
- Log text: chosen option = lower the global log level to WARN in the flight config (`sdkconfig.defaults` and `sdkconfig`), keep the app tag at INFO for the minute line. Rationale: Horizon already tolerates text between frames (Witness mixes ESP_LOG text with binary; the decoder drops unknown IDs byte by byte, NmeaExtractor ignores non-`$` text), so the least disruptive change is to cut the volume rather than move the console. Removed: per-CRC-error log (now a counter), per-command "Sending" lines, GPS 30 s diagnostics (INFO). Remaining text: warnings/errors and the two lines above, all newline-terminated by ESP_LOG. Console is still USB Serial/JTAG; `ESP_CONSOLE_NONE` was not chosen because it would hide panic/boot output needed when debugging on the bench.

## 6. RX race
`Ra01::receive()`: reads IRQ; on CRC error clears only CrcErr|RxDone; otherwise reads buffer status and packet status, clears ONLY RxDone, reads the payload, then reads IRQ again. If RxDone/CrcErr is set again, a new packet finished during the read: the payload may be mixed, so it is dropped (counted `rx_overrun`) and the new RxDone is left set so the next call delivers the new packet intact. Previously the clear `0x00FF` after the read could erase the new packet's flag, and a mixed payload could be forwarded. Poll latency vs the single RX buffer: with the DIO1 wake and spin BUSY the read window is about 0.5 ms instead of 60 ms.

## 7. Stack and watchdog
- `CONFIG_ESP_MAIN_TASK_STACK_SIZE` 3584 -> 6144 (`sdkconfig.defaults` and `sdkconfig`, also the legacy `CONFIG_MAIN_TASK_STACK_SIZE` line). Radio object, packet buffer, queue, status buffer and parser are static.
- Once per minute INFO line with `uxTaskGetStackHighWaterMark` of main and usb_tx.
- `CONFIG_ESP_TASK_WDT_PANIC=y` (timeout stays 5 s, panic handler reboots). The main task subscribes (`esp_task_wdt_add`) and resets every loop iteration. Longest legitimate block in the loop: radio transmit with a dead chip, about 100 ms + 5 ms x 12; health re-init about 0.3 s; the init retry loop sleeps in 10 ms slices. All far below 5 s.

## 8. Command parser, front end OFF
- `command_parser.h` `CommandAssembler`: on a frame that does not end in 0x0A it realigns to the next 0x05 already collected, so one lost byte costs one frame, not two (tested). There is no checksum in the 10-byte command format, so a data byte 0x05 followed by 0x0A 9 bytes later can still pass; fixing that needs a format change (Horizon + Witness), not done.
- RX front end OFF via 0x5250 is still accepted (bench use) but reverts to ON automatically after 30 s (`kFrontEndOffMaxMs`), is never persisted (boot default ON, unchanged), and while off the 0xF0 RSSI byte reads -128 (and the ack carries the flag as before).

## 9. Boot TX power (A7)
Value unchanged (-17 dBm at boot; range -17..+3; 0x5250 raises it, not persistent). `hardware.h` already says -17. Fixed `FIRMWARE_HANDOFF.md` (said "-2 dBm" and BW 125 kHz; now -17 dBm and 500 kHz) and added a section on these changes. MISMATCH FOR THE DOCS AGENT: `Software/Aperture/softwareattempt2/main/GROUND_STATION_PROTOCOL.md` (and the Horizon/Witness copies) say "boot at -9 dBm, range -9..+3"; actual is -17 dBm, range -17..+3. I did not edit that file (not in scope of this change, other agents own the protocol docs).

## 10. DTR/RTS auto-reset (report only)
The ESP32-S3 USB Serial/JTAG peripheral implements the esptool reset (DTR/RTS sequence to EN/BOOT) in hardware, not in firmware, so the application cannot ignore it, and no sdkconfig option disables it (the options under `USJ_*` only enable/disable the peripheral or its console behaviour). It can only be removed by an eFuse (`DIS_USB_SERIAL_JTAG`, which also removes USB flashing; not acceptable here, and eFuses are irreversible). The hardware reacts to specific DTR/RTS sequences, not to a plain open with both lines asserted together (Linux default), so the host side is where to act: open the port with DTR and RTS held de-asserted (pyserial `dtr=False, rts=False` before open, or ser2net without RFC2217 control-line passthrough on the data ports 7100-7102). I made no config change. Needs a bench test (open/close ports from Windows, ser2net reconnect, RFC2217) to learn which clients actually reset the board.

## Tests
`tests/aperture_logic_test.cpp` (frame ring drop-oldest and pop_if race, wrap-around; command resync after a lost byte and garbage; pacing bursts of 3 with >= 1.5 s quiet, slow trickle also cut; queue duplicates/full). Build and run:
`g++ -std=c++17 -Wall -Wextra -Werror -I main tests/aperture_logic_test.cpp -o aperture_logic_test.exe` -> "ALL OK". The existing `tests/gps_status_test.cpp` is unaffected (0xF0 builder unchanged).
Not tested: anything involving the radio, DIO1, USB driver, watchdog (no hardware, per rules).

## Build
ESP-IDF v6.1, target esp32s3, scratch dir `%TEMP%\wf8` with its own copy of `sdkconfig` (CCACHE_DISABLE=1; the first attempt hit spurious ccache failures in IDF components, unrelated). Result: build complete, `softwareattempt2.bin` 0x3d470 bytes (76 % of the app partition free), no errors. Resolved config confirmed: main stack 6144, TWDT panic on, log default WARN. Not flashed, not run. (`main/tx_slot.h` shows modified in git: that is the Witness radio agent's edit, not mine.)

## Trickle-down effects
- Horizon: 0xF0 RSSI = -128 now means "receiver down" (radio init failed or front end off); there is no new field. Horizon may show -128 dBm literally; a "radio failed" label would need a Horizon change (not made).
- Horizon: uplink commands are paced (3 per burst, then >= 1.5 s quiet), duplicates already queued are dropped, queue 64. `upload_assist` (60 ms pace, repeat-2 of missing chunks) will take about 4-5x longer and its back-to-back duplicate chunk repeats are collapsed while the first copy is still queued; its status/resend rounds cover this but timeouts around BEGIN/STATUS should allow 30+ s.
- Horizon: log text is now rare (WARN level); the minute INFO line `stack_free_min ...` and `counters: ...` lines appear in the byte stream (newline-terminated). The 0x5250 ack is unchanged; its "front end on" flag is still the truth, and the front end now returns to ON by itself after 30 s.
- Witness: with pacing, its 1 s hold-off after commands leaves telemetry windows between bursts. No Witness change needed. If the Witness radio agent adds a hold-off cap, the 1.5 s quiet time still works; keep `kQuietMs` > hold-off + one telemetry slot.
- HITL: nothing in Aperture changes for HITL traffic, except command pacing if HITL sends commands through Aperture.
- Operators: after flashing, boot logs are no longer visible at INFO on the console (WARN default). Power-cycle behaviour unchanged (-17 dBm at boot). A watchdog reboot now happens if the main loop hangs.
- Docs: Aperture `GROUND_STATION_PROTOCOL.md` still has the -9 dBm statement and stale status text (see section 9). The new `-128` RSSI meaning should be added to the 0xF0 section of all four protocol docs.
- Existing `sdkconfig` was edited by hand (log level, stack, WDT panic) so the in-tree config matches `sdkconfig.defaults`; a fresh checkout with only defaults gets the same values.
