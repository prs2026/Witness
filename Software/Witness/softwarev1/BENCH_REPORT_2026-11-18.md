# Bench report 2026-11-18: firmware batch (acks, GPS power-on, erase, radio edge cases, restart flush, not-logging warning, bench commands) and test-closure plan phases 1 to 3

**Image under test:** working tree on `34a167b` plus the changes below, built from clean build directories
(`build_images.ps1 -Clean`, `-Os`, 32 KB icache, 16 KB dcache, 240 MHz from `sdkconfig.flight`). Flight image 427,408 bytes,
maintenance image 851,712 bytes. `WIMG` is at byte 288 in both (checked). Iris was rebuilt (new handler) and flashed; Aperture
was not rebuilt (only `comms.h` changed there, and the four `comms.h` copies are byte-identical, checked with `md5sum`).
Flash target verified with `flash_verified.ps1 -CheckOnly` first (ESP32-S3 `softwarev1` for Witness, ESP32-C3 `softwarev1` for
Iris); flashing was done with esptool at 115200 baud through the 700x ports, all hashes verified. Communication used 710x only.

**GPS caveat (owner message during the run).** The GPS receiver was unplugged for part of the run. From the logs it was silent
(`0 edges` on both pins, `GGA` count stuck) from my first check after flashing until after the warm-reset matrix; it was back
(PX1120S at 88398 baud, 25 Hz) at the next check, before the watchdog test. All GPS-dependent results below were measured after
that point (power-on timing, GGA counts, the receiver restart, the HITL presets and the soak). The warm-reset matrix ran partly in
the missing window, but its pass criteria (state resumed, telemetry back, log appended, status bits) do not use GPS, so it stands.

## Summary by plan item

| # | Item | Result | Evidence |
|---|---|---|---|
| 0.x | Owner decisions | applied | Iris restore kept; no Witness change for the false-flight soft reset (see below); 2 s ring, radio power rule, no authentication: untouched |
| 1.1 | Warm reset in every state | **PASS** (3 in a row now PASS, see addendum; header-write reset NOT RUN) | Pad Idle, Boost, Coast, Descent, each with software restart and panic: 9/9 resumed correctly (table below). Landed: boots Pad Idle, not resumed, by design (2/2). 3 in a row in Boost: NOT RUN. Reset during a session-header write: NOT RUN (see reasons) |
| 1.2 | Real task-watchdog trip | **PASS** | `0x4248` in Coast: ack 43 ms, `Task watchdog got triggered` 8.0 s later, `last reset TASK_WDT`, `RESUMING IN-FLIGHT STATE 2 after reset reason 6`, resumed bit set, log appended to the same session, telemetry back 8.44 s after the command (8 s plus 0.44 s) |
| 1.3 | HITL simulator fixes | other agent | not mine |
| 1.4 | All presets and Monte Carlo on the final build | **PASS** | 8/8 presets PASS, every transition within 0.04 s; Kalman figures equal to the `-Og` run (table below). Monte Carlo (host build of the firmware filter, the target cannot run it) worst ascent 4.72 %, worst descent 0.97 %, goal 5 % |
| 1.5 | Kalman sideways drift | **closed by owner: drift without GPS accepted** | No filter change was made, so there is nothing to revert. Side finding for the record only, see "Observations" |
| 1.6 | Real-sensor launch detection and shake | NOT RUN | needs the owner (drop/swing/shake); not a firmware or bench-script item |
| 1.7 | 4 h pad-wait soak | **NOT RUN (partial, stopped for session cap)** | 11 minutes of the 4 hours ran: core 1 24.0 %, core 0 8.1 %, heap 41,476 B free (minimum 37,348 B, flat), GGA 17,618, GPS ready at 25 Hz, 0 resets, 0 watchdog or fault lines, 0 USB drops. The final HITL flight after the soak was not run |
| 1.8 | Pre-reset log loss | **PASS** | the `bench restart` text line, 120 ms before the reset, is now in the flash text log (it was missing before); every stream is complete up to the last HITL frame |
| 1.9 | Not-logging warning on the vehicle | **PASS** (LED itself not seen) | with both slots full: `NOT LOGGING to flash (status 0x6780 ...): red LED shows 3 pulses a second` 1.1 s after boot, and the status bit; pattern logic host-tested |
| 1.10 | Log store full | **PASS** | three HITL flights without clearing: flights 1 and 2 logged (slots A and B), the boot after flight 2 reports `both log slots hold a flight: NOT LOGGING (nothing is erased)`; flight 3 flew (PASS, 0.04 s), logged nothing, nothing was erased, `LOG_SLOTS_FULL` set, no crash; `0x6869` then freed the store in 137 ms and logging resumed |
| 1.11 | Forced NAND error / power loss during a header write | power loss mid-flight **PASS**; header-write timing and NAND error NOT RUN | hub power cut in Coast: back in Pad Idle (`last reset POWERON`), new session, the cut session exports fine (88.8 KB IMU, 68 KB packets); never-erase untouched. A cut exactly during a header write cannot be timed over the Pi (about 40 ms window); NAND error injection needs a command or a fault build that I was not asked to add |
| 2.1-2.3 | Regulator, current/temperature, brownout | NOT RUN | need parts not on the bench (regulator part, meter, powered hub, adjustable supply) |
| 2.4 | Iris outputs, each on command only | **PASS** (no loads) | OUT1, OUT2, OUT3 and 5 V switched on one by one (status `0x7800` = all four bits) and off one by one (`0x7800` > `0x7000` > `0x6000` > `0x4000` > `0x0000`); after a Pi hub power cycle with OUT1 and 5 V on (`0x4800`) Iris came back with `0x0000`, all off. No loads, so no current readback |
| 2.5 | Iris outputs across a warm reset | **PASS** | all four on, new `0x4249` command: ack OK, Iris log `warm reset (reason 3), valid RTC record: restoring commanded load mask 0x0F`, `LOADS RESTORED ... at +5 ms since app start`; Witness re-sent the 4 load commands (`load state mismatch or reboot`) |
| 2.6 | Stack high-water marks | **PASS** Witness and Iris, NOT RUN Aperture | minimum free over the whole session, bytes: sensors 5336, uart1 1592, flash_logger 3432, flash_capture 3420, iris_can 2216, usb_serial_echo 4676, cpu_load 1532, data_forwarder 1172, heartbeat 1316, sensor_log 1068; Iris main 5524 of 6144. None below 1 KB. Aperture logs at WARN and prints no high-water line; it needs a debug build |
| 3.1 | Ack collisions on bursts | **PASS** | 5 rounds of a 5-frame burst over the coax LoRa path (Aperture to Witness): Witness received 24 of 25 frames (the one loss is at the first frame of round 1, before Witness could hear it) and sent 24 acks that all reached Aperture: **0 lost acks** (before: 3 acks for 4 received) |
| 3.2 | USB ack for `0x5746` on | **PASS** | 5/5 round trips, ack 160-173 ms after the command; before the fix 0/2 |
| 3.3 | Radio edge cases | code done, bench injection NOT RUN | stuck-low DIO1 and 3-byte frames fixed (host-tested); radio reset without an MCU reset and a held BUSY already have the W8/W9 recovery and cannot be injected without a fault build or a new command |
| 3.4 | Radio at full power | NOT RUN | per instruction (no powered hub) |
| 3.5 | Range test | you | |
| 3.6 | GPS recovery, loss and reacquire | receiver restart **PASS**; receiver power-cycle and RF loss NOT RUN | `0x4752` (cold start, Witness keeps running): back to `25 Hz GGA+RMC confirmed at 88398 baud (ready, configured)` 3.1 s after the command. The receiver is powered from the Witness 3.3 V rail, so a receiver power cycle without cycling Witness is not possible. The HackRF is not on the bench Pi (`hackrf_info`: no HackRF boards found, not in `lsusb`), so attenuation or stop of the RF could not be done |
| 3.7 | GPS flight dynamics through the HackRF | NOT RUN | same reason: no HackRF found on the Pi |
| 3.8 | GPS power-on to 25 Hz | **PASS** | 5 hub power-on boots: 25 Hz confirmed at 7.95, 9.95, 7.95, 9.03 and 7.95 s of uptime (before: 21.6-23.6 s); in every one the PX1120S is identified at 115200 at 2.5 s, the configuration is abandoned about 2 s after the receiver goes quiet, and it is found at 88398 at 6.9 s. 0 USB over-current events |
| 3.9 | OTA over the Pi | **PASS in the addendum** (first stopped: `wlan0` was in use) | see addendum |
| 3.10 | Full erase blocks the USB task | **PASS** | `0x6869` ack 137 ms (was 4.34 s); a ping sent right after was answered in 42 ms; no `packet log queue full` line; 1.0 s maximum gap in the 0x02 stream; the background wipe of the other slot took 2071 ms |
| 3.11 | GPS cold-start check at execution time | already in the tree | `UartDriver` drops the request when the flag the sensor task refreshes every pass says "not Pad Idle" (`uart_driver.cpp`, `GPS cold start dropped: no longer in Pad Idle`), documented in `COMMANDS_AND_PACKETS.md`; the policy is in the `command_policy` test. No code change was needed. Not forced on hardware (the window is one sensor pass) |

## Reset matrix (item 1.1), Quick hop preset at a bench altitude of 195 m

All with HITL active, `0x4252`, USB. First telemetry = first packet with a fresh uptime after the command.

| State | Mode | Ack (ms) | First telemetry (s) | Result |
|---|---|---|---|---|
| Pad Idle | software | 33 | 0.365 | Pad Idle, log session 147 created (pad session) |
| Boost | software | 32 | 0.427 | Boost resumed, resumed bit set, `RESUMED ... appending to flight session 148 in slot A` |
| Boost | panic | 31 | 0.428 | same |
| Coast | software | 33 | 0.432 | Coast resumed |
| Coast | panic | 32 | 0.429 | Coast resumed |
| Descent | software | 32 | 0.429 | Descent resumed |
| Descent | panic | 31 | 0.429 | Descent resumed |
| Landed | software | 33 | 0.365 | Pad Idle, resumed bit clear (a Landed record is never resumed) |
| Landed | panic | 31 | 0.364 | same |

The later states after a reset (Boost to Coast to Descent within 2 s) are the bench artifact of reading the real, flat,
stationary board after HITL ended; they only move upwards. Iris never reset in these runs (uptime continuous). The reset
flags: flash-fail bits clear in every packet afterwards.

## HITL presets (item 1.4), final `-Os`/240 MHz build, GPS present

| Preset | Result | Transition deltas (s) | Up RMS boost / coast / descent (m) | Horizontal RMS descent / landed (m) |
|---|---|---|---|---|
| Nominal 10 g | PASS | 0.02 / 0.01 / 0.02 / 0.02 | 1.7 / 1.0 / 0.2 | 7.8 / 13.7 |
| Long burn | PASS | 0.02 / 0.02 / 0.02 / 0.01 | 3.4 / 1.5 / 0.2 | 8.6 / 9.4 |
| High-g short burn | PASS | 0.02 / 0.02 / 0.01 / 0.02 | 2.0 / 0.8 / 0.2 | 7.8 / 8.9 |
| GPS dropout | PASS | 0.02 / 0.02 / 0.01 / 0.02 | 2.0 / 1.5 / 0.4 | 15.5 / 10.2 |
| No GPS fix | PASS | 0.02 / 0.01 / 0.01 / 0.01 | 1.6 / 0.8 / 0.4 | 3311 / 5771 (no GPS, not scored; see below) |
| Sea-level site | PASS | 0.02 / 0.02 / 0.01 / 0.02 | 1.9 / 1.1 / 0.2 | 10.9 / 11.1 |
| Pad abort - no launch | PASS | no transitions, as predicted | pad 0.0 | |
| Quick hop | PASS | 0.01 / 0.01 / 0.03 / 0.01 | 5.0 / 2.1 / 0.3 | 4.8 / 4.8 |

Against the `-Og` run (17 Nov): Nominal up 1.7 / 0.9 / 0.2 m then, 1.7 / 1.0 / 0.2 m now; horizontal 7.3 / 13.4 m then, 7.8 / 13.7 m
now. Within the `-Og` figures. The "No GPS fix" horizontal figure is a dead-reckoning drift of the filter with no GPS at all, which the
owner has accepted (item 1.5).

## Soak (item 1.7)

Partial, 11 minutes only, stopped for the session cap (see the table row). Not a pass for item 1.7.

## OTA conditions (item 3.9): stopped

Read-only checks on the bench Pi: `eth0` holds the primary default route, but `wlan0` is up and connected to another Wi-Fi network
as a second default route (metric 600), there is exactly one Wi-Fi adapter (no USB Wi-Fi in `lsusb`), and Tailscale is up. The
instruction was to proceed only if the uplink is ethernet and `wlan0` is free, or a second adapter exists. `wlan0` is not free (it is a
live backup uplink) and there is no second adapter, so I did not scan, join or touch any connection, and nothing on the Pi was
changed. To run item 3.9 the owner needs to say that `wlan0` may be given up for the test (it would drop the backup uplink for the
duration), or plug in a second Wi-Fi adapter.

## Code changes (all uncommitted)

| File | Change | Host test |
|---|---|---|
| `main/comms.h` (4 copies, `cp`) | new `IRIS_COMMAND_BENCH_HANG 0x4248`, `IRIS_COMMAND_BENCH_IRIS_RESET 0x4249`, arming bytes `0xA5 0x5A` | `command_policy_test` static_asserts against `command_policy.h` |
| `main/command_policy.h` | `is_bench_command`, `allow_bench_command` (USB and HITL only) | `command_policy_test`: all three opcodes, USB/radio x HITL on/off, nothing else is a bench command, no collisions |
| `main/command_bridge.{h,cpp}` | `check_policy` refuses bench opcodes (so a radio copy or a 3-byte/text USB copy is never forwarded to Iris or re-sent over LoRa; this also closes the old path that forwarded `0x4252`); `send_bench_iris_reset()` builds the armed frame itself | covered by the policy test |
| `main/usb_serial_echo.cpp` | bench handlers: `0x4248` acks then `sensors_.bench_hang()`; `0x4249` sends the armed CAN frame (OK or BUSY); `0x4252` unchanged semantics; all gated by `allow_bench_command`. Acks use `queue_packet_wait` (50 ms) and a failed radio reply is logged; the ack is flushed before the bench restart and before every deferred restart (`0x5746` on, `0x7300`) | policy test |
| `main/spi_data_forwarder.{h,cpp}` | `queue_packet_wait`, `flush()` (counter based, cannot be held up by a busy queue), `immediate_radio_pending()`, dropped-frame counter, immediate radio queue 8 to 16 | `radio_edge_logic_test` (`sequence_reached`, wrap) |
| `main/radio_edge_logic.h` (new) | `Dio1SlowPoll` (stuck-low interrupt line: IRQ status read every 200 ms), `arms_holdoff` (only a complete 10-byte frame arms it), `AckPacer` (an ack waits until the radio has heard nothing for 40 ms, at most 300 ms, in order), `sequence_reached` | `radio_edge_logic_test` |
| `main/sensors.{h,cpp}` | uses the three helpers above; `bench_hang()` stops feeding the task watchdog | `radio_edge_logic_test` |
| `main/gps_receiver.{h,cpp}` | first failure retried after 500 ms with 1.5 s of silence (later ones keep 10 s / 5 s); configuration fails at once after 2 s of silence once the receiver has been heard, instead of three 1 s readback timeouts | `gps_receiver_test`: power-on scenario (fail at 2.0 s, probe due 0.5 s later, second failure back to 10 s) and a 1 Hz receiver that ignores commands is not failed |
| `main/flash_logger.{h,cpp}`, `main/erase_wipe.h` (new) | full erase = header clear (about 40 ms, new session) plus a background wipe of the other slot in 8-block chunks, ground states only, cancelled by a launch; shutdown handler (`esp_register_shutdown_handler`) asks the logger to flush everything before any software restart (`flush_for_restart`, 150 ms bound) | `erase_wipe_test` (covers every block of the other slot, never the current one, chunk bound, launch cancels, slot change) |
| `main/heartbeat.{h,cpp}`, `main/led_pattern.h` (new) | red LED: 3 pulses a second instead of 1 when `FLASH_INIT_FAILED`, `FLASH_LOG_FAILED` or `LOG_SLOTS_FULL` is set; one log line on each change | `led_pattern_test` |
| Iris `main/usb_serial_echo.cpp` | `0x4249` from CAN with the arming bytes calls `esp_restart()`; ignored from USB/UART or without the bytes | none (Iris has no host test for command parsing); verified on hardware (2.5) |
| `COMMANDS_AND_PACKETS.md` | new rows for `0x4248`, `0x4249`, `0x4252` (never forwarded), `0x6869` (new behaviour, session numbers keep counting), ack pacing and flush rules | |

Behaviour changes to be aware of: after a full erase (`0x6869`) the session number now keeps counting up (it restarted at 1 before);
the old pages stay physically in the current slot until the writer erases a block before use, as after a clear, and the other slot is
wiped in the background. Host suites: 23 of 24 build and pass; the one that does not build is `imu_calibration_store_test` (plan item
5.7, FreeRTOS stubs, not mine). New suites: `radio_edge_logic`, `erase_wipe`, `led_pattern`.

## Answers to the open points

- **False flight on the pad, soft reset:** `0x7300` in Landed already acks OK and restarts Witness into Pad Idle (USB and radio, state
  checked again when the timer fires). It was confirmed here (the reset-to-pad step after every HITL flight, about 25 times). So no new Witness
  command is needed for Horizon's manual reset; nothing was added.
- **3 resets in a row in Boost cannot be done as specified.** After the first warm reset HITL is off, and `0x4252`, `0x4248` and
  `0x4249` need HITL active, which can only be entered in Pad Idle. Smallest change if the owner wants it: accept the bench commands
  also while the RESUMED status bit is set (USB only, never radio). I did not make it because the gating was fixed by the owner.
- **A reset exactly during a header write:** the header is written about 0.2 s after boot for about 40 ms. A repeatable test needs either
  a bench command that restarts N ms after the logger starts a header write, or a fault build. Not added.
- **NAND error injection and radio fault injection (stuck-low DIO1, BUSY held, radio reset without an MCU reset):** need a throwaway
  fault build; I did not make one because it would be a feature in the tree. A scratch copy of the tree could do it (the earlier runs
  used one) if the reviewer wants it.

## Observations

- **GPS** after a power cycle: the receiver is found at 115200 at 2.5 s, moves to 88398 baud by itself during the configuration, and is
  re-identified there at 6.9 s. This is now 8-10 s to 25 Hz; the open question of why it moves is unchanged.
- **Item 1.5, for the record only (no work, no filter change):** the horizontal "drift" in the HITL presets has a step-like form (jumps
  of 8-14 m at random times, the same in the three Nominal runs and with `descent_sway_dps=0`: 7.5 m / 13.7 m). The 0x0C packet carries
  latitude and longitude with 0.0001 degree resolution (about 11 m, `IRIS_PACKET_HITL_LATITUDE_OFFSET`, `hitl_protocol.LATLON_DEG_PER_COUNT`),
  and the simulated GPS random walk crosses those steps. It is a test-input quantisation, not the filter.
- The Pi `dmesg` showed 0 USB over-current events during every hub cycle and every radio test (radio at -17 dBm, front end on).
- Pi hub power cycles used: 9 (one at the start, 5 power-on boots, one to leave export mode, the power cut, and the one that starts the soak),
  each at least 4 minutes after the previous one. Port order was unchanged every time.
- The GPS fix is "none" on the bench (no sky signal, no HackRF); the GGA stream and the receiver state are what was checked.

## Not run, with reasons (summary)

1.1 three in a row and header-write reset (gating, timing); 1.6 (owner); 1.11 NAND error and exact header-write cut (no injection); 2.1-2.3
(parts); 2.6 Aperture (no stack output); 3.3 bench injection (no injection); 3.4 (instruction); 3.5 (owner); 3.6 RF loss and receiver
power cycle, 3.7 (no HackRF on the Pi, receiver on the Witness rail); 3.9 (wlan0 in use, no second adapter).

## State the bench was left in

- Witness: flight image (flight_a) and maintenance image (maint_a) from this build, Pad Idle, HITL off, GPS ready at 25 Hz, log session
  a pad session (store cleared, not full).
- Iris: new image, all outputs off (status `0x0000`, confirmed at the end).
- Aperture: running its previous image (unchanged).
- Pi: nothing changed (no Wi-Fi join was attempted); HackRF absent; no process of mine running; no ser2net port held.

## Not run because of the session cap

The 4 h soak (only 11 min ran) and the HITL flight after it. Everything else listed as NOT RUN above has its own reason.

---

# Addendum (same day, after the owner's approvals): resumed gating, 3 resets in a row, OTA over the Pi

## Bench commands also accepted after a warm reset (item 1.1, 3 in a row)

- **Change.** `command_policy::allow_bench_command(source, hitl_active, resumed_after_reset)` is now `source == Usb && (hitl_active || resumed)`.
  `Sensors::resumed_after_reset()` reads the RESUMED status bit. Still canonical USB frames only; radio is never allowed and the bridge
  still never forwards them. Documented in `comms.h` (all 4 copies identical, `cp`, md5 checked) and in `COMMANDS_AND_PACKETS.md`
  (the three rows now say "HITL active or RESUMED after a warm reset (USB only)").
- **Host test.** `command_policy_test`: USB with RESUMED and no HITL allowed; radio refused in every combination; USB with neither refused.
  Passes. Flight image rebuilt (427,472 bytes, `WIMG` at 288) and flashed to flight_a; Iris and Aperture unchanged.
- **Hardware, item 1.1 "3 resets in a row in Boost": PASS.** Pad Idle with no HITL and no resume: `0x4252` REFUSED. HITL flight to Boost, then
  three resets back to back (software, panic, software): acks 32/32/33 ms, first telemetry 0.435 / 0.432 / 0.431 s after each command, each
  boot `RESUMING IN-FLIGHT STATE 1` (reset reasons 3, 4, 3), all three append to the same flight session (209, slot B), resumed bit set,
  no log-failed bit. Cleared and returned to Pad Idle afterwards.

## OTA via the Pi (item 3.9): PASS

Pi state **before** (read-only): `eth0` is the only default route (metric 100) and carries Tailscale (the route to the desktop peer and to my
SSH client both leave through `eth0` / `tailscale0`); `wlan0` was disconnected and link DOWN (no second default route at that moment; the
earlier backup uplink was gone); one Wi-Fi adapter; profiles: `netplan-eth0`, a home Wi-Fi profile and `witness-ota` (SSID `witness-XXXX`,
`never-default yes`, autoconnect yes at priority -10, a profile that already existed); passwordless sudo only for the hub command.
Because `wlan0` carried neither the uplink nor Tailscale, I proceeded.

1. Witness `0x5746` on over USB (10 min, ack OK): maintenance image up, AP `witness-XXXX` visible from the Pi (signal 93).
2. I used the **existing** `witness-ota` profile (`nmcli con up witness-ota ifname wlan0`) instead of creating a new one, so no Wi-Fi
   password was typed by me. It is already never-default. After joining, the routing table still had only the `eth0` default; `wlan0`
   added just its own /24 link route.
3. `witness_ota.py info` (run on the Pi): maintenance image running, `flight_b` empty. `upload softwarev1.bin` (427,472 bytes, kind
   flight, from the build with the resumed gating): uploaded 100 % into `flight_b`, Witness rebooted into it.
4. Second Wi-Fi session: `info` shows `flight_a` and `flight_b` both `34a167b-dirty` and **"boot returns to flight_b"**, i.e. the new
   flight image booted, passed its 30 s self-test and confirmed itself; the upload itself had checked the image-info record
   (kind flight). Witness was in Pad Idle on USB after the first reboot.
5. Log download over Wi-Fi: `logs` listed 10 files (sessions 211 and 212); `get S00000D4.TXT` (13,470 bytes) and `get P00000D4.BIN`
   (287,685 bytes) downloaded with sizes equal to the listed sizes; the text starts with the normal flash-logger boot lines. (I did not
   convert the packet file.)
6. `wifi-off`: Witness rebooted into flight, Pad Idle, HITL off.

Pi state **after**: `nmcli con down witness-ota`; `wlan0` disconnected and link DOWN as before; routes: only the `eth0` default and its
/24; profiles and their autoconnect values unchanged (no profile created or modified); Tailscale still up with the desktop peer direct
over `eth0`; my scratch folder on the Pi removed. The Pi's network is as it was before. Note for the owner: the existing `witness-ota`
profile autoconnects at priority -10, so with Witness's AP on the Pi's `wlan0` would join it by itself.

No HackRF, TX gain or GPS RF test was touched. Port 7100 was used by me only; I told the reviewer when I started.
Final state: Witness Pad Idle, HITL off, flight image from flight_b; Iris outputs all off (`0x0000`); Aperture unchanged; the earlier
"not run" list still stands except items 1.1 (3 in a row) and 3.9, now PASS.
