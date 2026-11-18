# 06 Command fixes (2026-11-15)

Files changed: main/usb_serial_echo.cpp/.h, main/command_policy.h, main/command_bridge.cpp/.h, main/maintenance_mode.cpp/.h, tests/command_policy_test.cpp, and (unowned, small additions, flagged) main/boot_manager.cpp/.h. Nothing committed, flashed or bench-tested.

## Status per finding

| # | Finding | Status | What changed |
|---|---|---|---|
| 1 | W11/B2/B10/B9 restart without re-check | fixed | maintenance_mode: new `restart_deferred_after_ms/deferred_restart_due/deferred_restart_clear/restart`; the timer only raises a flag. usb_serial_echo `schedule_restart`, `service_pending_restart` (run loop): at fire time `command_policy::restart_still_allowed(kind,state)` (Maintenance: Pad Idle/Landed; ResetToPad: Landed). If not allowed: no restart, `boot_manager::revert_switch()` (NEW in boot_manager: cancel request + `esp_ota_set_boot_partition(running)` + mark valid). Timer-start failure also calls `revert_switch()`. `maintenance_mode::arm` now returns bool; `prepare_switch` refuses (SWITCH_FAILED) if it fails (B9). 0x7300 in Landed uses the same path (kind ResetToPad, still restarts) and reverts a pending maintenance switch. Residual: a launch in the last few ms between the re-check and `esp_restart` cannot be excluded. The maintenance image keeps the old `restart_after_ms`. |
| 2 | W1 wiring | partly | `radio_power_store::save(level)` is called after a successful 0x5250, only in Pad Idle/Landed (an NVS write stalls both cores), never the front-end flag. It sits behind `__has_include("radio_power_store.h")`; the header did not exist at build time, so a marked TODO is compiled instead. Rebuild once the header exists. Power changes made in flight are RAM only. |
| 3 | 0x5250 | fixed | `allow_radio_power_change`: Boost refused (as before); Coast/Descent refuse front end OFF and any drive below the level in use; raising allowed; Pad Idle/Landed everything. Order: range check (BAD_ARGUMENT), then policy (REFUSED). The ack is queued first (its detail shows the state it is about to have); over radio the OFF is applied 600 ms later (ack on air) and re-checked against the flight state then; over USB it is applied right after queuing the ack. |
| 4 | Iris loads / 5 V (07 #2, I4, I5) | fixed | OFF = 0x0030/32/34 and 0x0050 refused in Boost/Coast/Descent from USB and radio; ON always allowed. The CommandBridge applies `allow_iris_load_command` before anything goes to Iris or is re-sent over LoRa (so refused USB commands are no longer re-broadcast either, 07 #16). The bridge gets the state through `set_state_reader` (set in `UsbSerialEcho::start`). I5: the bridge keeps the CAN forward result per queued radio command (`receive_radio_command(..., &forward_result)`); load ack = OK only if the CAN queue accepted it, BUSY if not, REFUSED by policy. USB acks the same way. |
| 5 | GPS assist / cold start (G4/H2/G8) | partly | Assist BEGIN/DATA/COMMIT only in Pad Idle and Landed (STATUS everywhere). Cold start uses `allow_gps_cold_start`, checked immediately before the request. The real execution is in the GPS task (gps_receiver, not mine): it should re-check Pad Idle when it consumes the flag. |
| 6 | Radio erase/clear (07 #4, #6) | fixed | `command_policy::EraseConfirm`: same command twice within 10 s; the first send acks BUSY (0x04) with detail 0xC0F1 and does nothing. USB is single shot. Waits are bounded to 30 s using the existing timeout parameters of `erase_all_sessions/clear_all_sessions`. The ack now reflects the result (it was always OK): timeout/busy -> BUSY, detail = esp_err; other failure -> REFUSED, detail = esp_err. |
| 7 | echo_bytes (07 #5) | fixed | No host: echo skipped. Otherwise 20 ms budget; on failure echo is muted for 1 s. It is the only usb_serial_jtag write in this file. ESP_LOG console output goes through the IDF VFS driver (outside this file, unchanged; still needs the unplugged-cable test). |
| 8 | stray 0x0C/0x0E (07 #15) | fixed | While HITL is off the ID byte only starts a held candidate: a valid frame (EOF + decode) is dropped as before; anything else is replayed to the command parser (also on 20 ms idle). With HITL active the old behaviour is unchanged. |
| 9 | radio auth | not done | proposal below |
| 10 | stack (07 #19) | done | Raised 6144 -> 7168 (replay copy of up to 149 B per nesting level; the revert path is a subset of prepare_switch). Once a minute an INFO "stack high-water" line (WARN when under 1024 B free). Read it after a bench session and trim if the margin is large. |

## Command x state x source matrix (from code, * = changed)
PI Pad Idle, B Boost, C Coast, D Descent, L Landed. U = USB, R = radio.

| Command | Src | PI | B | C | D | L |
|---|---|---|---|---|---|---|
| Load/5V ON 0x0031/33/35, 0x0051 | U,R | OK | OK | OK | OK | OK |
| Load/5V OFF 0x0030/32/34, 0x0050 | U,R | OK | REFUSED* | REFUSED* | REFUSED* | OK |
| Load/5V command when CAN queue full | U,R | BUSY* (retry) | same | same | same | same |
| 0x5250 raise power, FE unchanged/on | U,R | OK | REFUSED | OK | OK | OK |
| 0x5250 lower power | U,R | OK | REFUSED | REFUSED* | REFUSED* | OK |
| 0x5250 front end OFF | U,R | OK (after ack) | REFUSED | REFUSED* | REFUSED* | OK |
| 0x4741/44/45 assist upload | U,R | OK | REFUSED | REFUSED* | REFUSED* | OK |
| 0x4751 assist status | U,R | OK | OK | OK | OK | OK |
| 0x4752 cold start | U,R | OK | REFUSED | REFUSED | REFUSED | REFUSED |
| 0x6869/0x686A erase, clear | U | OK single shot | REFUSED | REFUSED | REFUSED | OK single shot |
| 0x6869/0x686A | R | 1st send BUSY+0xC0F1, 2nd within 10 s OK* | REFUSED | REFUSED | REFUSED | same as PI* |
| 0x5746 on, 0x6868 (0x6868 USB only) | U,R | switch; restart re-checked* | REFUSED | REFUSED | REFUSED | switch; restart re-checked* |
| 0x7300 | U,R | no-op OK | REFUSED | REFUSED | REFUSED | ack, then restart (re-checked Landed)* |

All other rows (0x7301-0x7304, HITL, pad heading, calibration, debug, ping, text commands) are unchanged from 07_commands.md. A switch or reset restart that finds the state changed when its timer fires is abandoned (log "restart abandoned"), and the maintenance boot slot is reverted to flight.

## New ack codes
- IRIS_ACK_BUSY (0x04) with detail 0xC0F1 (`command_policy::kAckDetailConfirmNeeded`): radio erase/clear, send the same command again within 10 s. Should get a named constant in comms.h.
- Load commands: BUSY detail 0 = not forwarded to Iris, retry.
- Erase/clear failure: detail = esp_err_t value.

## Tests
tests/command_policy_test.cpp (host g++ -std=c++17 -Wall -Wextra): loads on/off x all states, radio power change exhaustive over dBm x state x front end, assist, cold start, persist rule, restart re-check, EraseConfirm (window edge, other command, tick wrap). Result: PASS.

## Firmware build (flight overlay, scratch dir wf6\b)
usb_serial_echo, command_bridge, maintenance_mode, boot_manager and sensors compile under -Werror with no warnings. A full link was blocked by other agents' files mid-edit (flash_logger.cpp: erase_requested_ undeclared; then uart_driver.cpp:472 std::min type mismatch). See the build line at the end of this file for the last retry.

## Not done / needed in files I do not own
- gps_receiver: re-check Pad Idle where the cold-start flag is consumed (G8).
- comms.h: name the 0xC0F1 detail. COMMANDS_AND_PACKETS.md: use the matrix above.
- Flush of NAND staging before a restart (07 finding 7) belongs to the logger (shutdown handler).
- radio_power_store.h: rebuild when present; main.cpp/radio code should call `radio_power_store::load` at boot.

## Trickle-down
- Horizon/Aperture: erase/clear over radio now needs two presses/sends within 10 s; treat BUSY+0xC0F1 as "confirm", not an error; BUSY on loads means retry. Load-OFF buttons will be REFUSED in flight. 0x5250 lowering/front-end-off is REFUSED in Coast/Descent. Assist upload tools only work on the pad or after landing.
- HITL: stray 0x0C no longer eats commands; tooling unaffected when HITL is active.
- Operators: a USB export or Wi-Fi switch can now abort if the rocket launches in the 0.4-1.1 s restart window (see "restart abandoned" in the log). Docs need the new matrix.

## Proposal: radio command authentication (not implemented)
Add a 16-bit rolling counter and a truncated (32-bit) HMAC-SHA256 over [counter, opcode, d0-d5] with a per-vehicle key stored in NVS. The 10-byte frame has no spare room, so define a new 0x05 variant in comms.h (ID, counter 2, opcode 2, data 6, mac 4, EOF). Witness accepts counters strictly above the last stored one (persist every N, small window for loss) and verifies before the bridge forwards to Iris. Aperture forwards frames unchanged; Horizon holds the key and counter and signs every uplink command; USB commands stay unsigned. Needs a protocol decision: key provisioning and counter handling after a reboot.

## Last build retry
Still blocked by uart_driver.cpp:472 (std::min uint32_t vs unsigned, another agent file, 3 retries done). flash_logger error from the first try had cleared. My files compile clean; link and image size not obtained.

## Follow-up 2026-11-16

1. False-flight restart OFF (owner decision): `flight_state_machine.h` `kFalseFlightRestartEnabled = false` (code and `set_false_flight_callback()` kept; nothing registers a callback). `tests/flight_state_machine_test.cpp: test_false_flight_callback` now asserts the switch is off and that the callback is never called, while Landed and `flight_evidence()` behave as before.
2. GPS cold start re-check at execution: `UartDriver::set_cold_start_allowed(bool)` (`std::atomic<bool>`, default false) in `uart_driver.h`; `Sensors::run` (`sensors.cpp`) refreshes it on every pass with `command_policy::allow_gps_cold_start(state)`; `uart_driver.cpp` GPS task drops the request with a warning when it is not allowed. Re-identification (audit G8) is already covered: `GpsReceiver::cold_start` sets Configuring and RestartWait, then re-identifies and reconfigures (existing `gps_receiver_test` cold-start cases check this); no change needed. The ESP-side gate itself is not host-testable; the predicate is covered by `command_policy_test`.
3. `IRIS_ACK_DETAIL_CONFIRM_NEEDED 0xC0F1U` added after the ack results in the four `comms.h` copies (cmp: identical); `usb_serial_echo.cpp` uses it; `command_policy.h` `kAckDetailConfirmNeeded` removed (comments updated).
4. Docs: COMMANDS_AND_PACKETS.md, FIRMWARE_STATE_MACHINE.md, FIRMWARE_HANDOFF.md.

Tests (host g++): flight_state_machine_test, command_policy_test, gps_receiver_test all pass; comms.h + command_policy.h syntax check OK. Full firmware build not run (coordinator).
