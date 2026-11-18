# 05 GPS fixes (audit 06_gps, findings G1, G2, G4 receiver side, G5, G7, G9, G10)

Files changed (all mine): `main/gps_receiver.{h,cpp}`, `main/gps_assist.{h,cpp}`, `main/uart_driver.{h,cpp}`,
`tests/gps_receiver_test.cpp`, `tests/gps_assist_test.cpp`. Not touched: nmea_gps.*, px1120s_gps.*, sensors.cpp,
usb_serial_echo.cpp. Not committed, not flashed, no bench hardware used.

## Status per finding

| Finding | Status | What changed |
|---|---|---|
| W14 / G2 retry disturbs a streaming receiver | fixed | `GpsReceiver::service` turns Failed + valid GGA into `Streaming`; `retry_identification` refuses while a GGA arrived in the last 5 s; `fail()` no longer forces the host to 115200 while NMEA flows; `UartDriver::run` calls the pin probe only when `probe_due()` |
| W15 / G1 no supervision after Ready | fixed | `GpsReceiver::supervise`, `enter_recovery`, `service_recovery`, `observe_gga`; driver reports every GGA with `observe_gga(now, count)` |
| G1 "came back with default config" | fixed | GGA per second window in `supervise`; two consecutive windows below expected/4 (25 Hz expected: 6 Hz or less) trigger `start_background_config` (baud/pins untouched), with backoff |
| G7 Ready must mean configured | fixed | `unacked_required_`: a required PX1120S command that timed out and has no readback (the NMEA set, 0x08) gives `Streaming`, not `Ready`; mode and rate are covered by the Verify readback (`SkytraqCommand::readback`). Background retry per backoff |
| G4 receiver side: whitelist | fixed | `gps_assist::is_allowed_message`: only 0x41 "Set GPS Ephemeris", exactly 87 bytes (the only thing Horizon `gps_assist.py` sends). Enforced in `parse_records` (so COMMIT answers BadFormat), again in `begin_assist` and right before the wire in `service_assist`. Defence in depth in `send_skytraq` (refuses 0x05 baud, 0x04 factory defaults while `kFixedBaud`) and `send_ubx` (refuses CFG-PRT while `kFixedBaud`, CFG-CFG always) |
| G10 COMMIT BUSY later acked OK | partly (my half) | `Assembler::handover_failed()` reopens the transfer so the repeated COMMIT verifies and hands over again instead of `AlreadyComplete`. **The caller must use it: see "needed elsewhere"** |
| G5 silent pin / default orientation | fixed in my files | driver remembers the orientation that last gave a valid GGA (`have_good_pins_`) and uses it when neither pin shows edges in the probe, and returns to it after a Failed identification; receiver remembers the baud (`good_baud_`) and a later identification tries it first, then 115200, then measured rate, then the other pins. Board default (`HW_MCU_UART_SWAP_RX_TX` 0) is in hardware_pins.h, not mine |
| G9 timestamps | fixed (trivial) | NMEA stamped `now - backlog_bytes*10/baud` (bytes still in the ring are newer than the ones just read; capped at 1 s). Not host-testable (ESP only), compiled |
| 8 no baud change / factory reset path | confirmed + test | see below |

## State machine after the change

| Status | Meaning | Leaves when |
|---|---|---|
| Configuring (initial) | boot delay, identify, configure, verify | Verify done: Ready or Streaming; step failure: Failed |
| Configuring (background, `state_text` "streaming (reconfiguring)") | configuration re-sent at current baud and pins | done: Ready/Streaming; failure: Failed then Streaming at once if GGA flows; 3 s without GGA: Recovering |
| Ready | configured, supervised, assist allowed, sleep allowed | 3 s without GGA: Recovering; 2 windows with rate < expected/4: Configuring (background) |
| Streaming | valid GGA, configuration not confirmed (usable position) | backoff elapsed (5 s doubling to 60 s): Configuring (background); 3 s silence: Recovering |
| Recovering | listens 2 s at 115200 then 2 s at the fallback rate (good non-115200 baud, else the measured rate), known-good pins, no packets sent | GGA arrives: Configuring (background) at that baud; both stages silent: Failed (probe due at once) |
| Failed | no data | `probe_due()` (no GGA for 5 s and 10 s since last probe, or at once after Recovering): `retry_identification` + `select_gps_pins`; a GGA arriving turns it into Streaming |

The existing `Failed` retry (identification failure at startup, receiver still booting) is unchanged in behaviour when
no NMEA is heard: 10 s pacing, same probe.

## Worst-case UART-task blocking

Only the pin probe blocks: 2 x 400 ms = 0.8 s, and only when Failed with no valid GGA for at least 5 s (before: every
10 s on any Failed, streaming or not). Recovery and background configuration never block (set_baud and queue writes;
`uart_wait_tx_done` bounded 100 ms per command as before).

## Fixed-baud guarantee

No path sends a baud change, factory reset or flash write: built-in sequences never did (`kFixedBaud`); assist records
are whitelisted to 0x41; and the transport is guarded (see table). Test: `assert_no_forbidden(uart)` runs over the
streaming-NACK, missing-ACK, no-data, recovery + fallback + full-probe, default-config and assist-refusal sequences and
checks every frame sent: no SkyTraq 0x05/0x04, no UBX CFG-PRT/CFG-CFG. Only the host rate changes (set_baud).

## Tests

Host (g++ 15 -std=c++17 -O1 -Wall -Wextra, no warnings):
`tests/gps_receiver_test.cpp` + gps_receiver.cpp + gps_assist.cpp: PASS. `tests/gps_assist_test.cpp`: PASS.
New cases: streaming NACK -> Streaming with no baud change/no pin swap/no retry, backoff 5 s then background config to
Ready; missing ACKs -> Streaming, background retry -> Ready; no-data Failed probe gating; silence -> Recovering ->
fallback baud found -> reconfigured; total silence -> Failed -> probe -> remembered good baud first then 115200;
1 Hz default-config detection and a healthy 25 Hz stream not triggering; whitelist (0x05, 0x04, 0x01, 0x0B, 0x64, 0x08,
0x0E, 0x00, 0x42, wrong length, mixed blob); G10 reopen. Existing tests changed: the acks=false case now expects
`Streaming`; the assist-restart test now expects the 0x01 record to be refused; assist test blobs are now real
87-byte 0x41 records.

Firmware build (flight image, sdkconfig.defaults + sdkconfig.flight, dir C:\Users\E\AppData\Local\Temp\wf5): full image build NOT completed (per coordinator, a single full build comes later; two attempts failed in ESP-IDF component wpa_supplicant/eap_common.c with no compiler diagnostic, not my code). Syntax/object compile with the flight configuration of my own files: uart_driver.cpp, gps_receiver.cpp, gps_assist.cpp compile with no errors or warnings (fixed a std::min type mismatch at uart_driver.cpp:472).

## Trickle-down effects

* `GpsReceiver::Status` gained `Streaming` and `Recovering` (Ready now means configured). Only uart_driver.cpp uses it.
* `observe_nmea()` still exists; the driver now calls `observe_gga(now, count)`. `can_retry_identification()` is
  unchanged but no longer sufficient: use `probe_due(now)`.
* Log strings: periodic line now reads `receiver <model> <state_text>` with states `ready (configured)`,
  `streaming (unconfigured)`, `streaming (reconfiguring)`, `reconfiguring (no data)`, `configuring`,
  `recovering (listening)`, `FAILED (no data)`, plus `, N Hz, recoveries N, config retries N`. New warnings
  "streaming but NOT confirmed configured" and "GPS silent ...: recovering". The "airborne 4 g mode ... confirmed" line
  now ends "(ready, configured)" and appears only for Ready. Operators grepping for `FAILED` will see it only when no
  data flows.
* Position is used in Streaming and Ready alike; nothing outside uart_driver reads the status.
* Assist uploads of anything other than 87-byte 0x41 records now fail at COMMIT with BadFormat (`IRIS_ACK_BAD_FORMAT`).
  Horizon `gps_assist.py` is unaffected. The host-initiated hot start through an assist 0x01 record no longer works (docs
  mentioning it should be updated: FIRMWARE_STATE_MACHINE.md "assist restart", COMMANDS_AND_PACKETS.md assist section).
  Cold start (0x5752 path) is unchanged.
* FIRMWARE_STATE_MACHINE.md still says Failed is terminal / retried every 10 s: now Failed + data = Streaming; retry only
  without data.
* Two real behaviour changes to know on the bench: a receiver that moves from 115200 to 88.4 kbaud during configuration
  (the observed quirk) fails its readback at 115200, stops delivering data, then goes Failed and is re-probed after 5 s of
  silence (as before, but never while data flows). A receiver that never ACKs the 0x08 message set stays `Streaming` and
  is re-sent the configuration at 5/10/20/40/60 s intervals.

## Needed in files I do not own

1. `usb_serial_echo.cpp` (command agent), G10: in the `Assembler::Result::Complete` case, when `submit_gps_assist`
   returns anything but ESP_OK call `gps_assist_.handover_failed();` before sending IRIS_ACK_BUSY. Without it the repeated
   COMMIT is still answered OK by `AlreadyComplete`.
2. `usb_serial_echo.cpp` / `command_policy.h`: assist gating by flight state (G4 first half) is the command agent's.
3. `hardware_pins.h`: `HW_MCU_UART_SWAP_RX_TX` is 0 but this board needs the swap (G5); with the remembered orientation
   the first boot still starts on the probe result only.
4. Docs (FIRMWARE_STATE_MACHINE.md, COMMANDS_AND_PACKETS.md, GROUND_STATION_PROTOCOL.md) for the whitelist and the new
   status words.

Cold start gating (command agent request): the flag consumed in `UartDriver::run` (`cold_start_requested_`) has no flight state visible in the UART task, so a Pad Idle re-check cannot be done in my files. It needs either a setter fed by sensors.cpp (e.g. `UartDriver::set_gps_cold_start_allowed(bool)`, checked where the flag is consumed) or the check kept only in the USB/radio handlers.

## Not done

G3 (last-known position in telemetry), G6, G8, G11, G12 (not in my scope). G9 uses a backlog estimate, not true receive
time. The unacked-required rule can only prove the NMEA set by ACK: there is no readback for 0x08 in this code.
