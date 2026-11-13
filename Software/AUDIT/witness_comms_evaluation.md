# Witness comms/storage/update — audit evaluation

Scope: code under `Software/Witness/softwarev1/main`, read against `FIRMWARE_HANDOFF.md` ("Wi-Fi OTA and log download", command table at lines 725-752). Read-only; nothing built or run. Line numbers refer to the current tree.

Key facts established while checking the two critical claims:
- `maintenance_mode::consume_boot_request()` runs first in `app_main` (`main.cpp:22`) and calls `cancel()` (`maintenance_mode.cpp:31`). So a panic, watchdog or brownout reset in flight boots into normal flight mode unless something re-armed the RTC request. The request is armed only by `maintenance_mode::request()` at `wifi_ota.cpp:148` (the `request_enable` path, gated on `state_allows()` = Pad Idle/Landed) and `wifi_ota.cpp:677` (end of a successful OTA, which already runs in maintenance). It is armed before the reboot is actually decided, which is the only way a request can survive (finding 4).
- The state machine is RAM only and starts in Pad Idle (`flight_state_machine.cpp:16`). After a reboot in flight:
  - `S*.BIN` (`FLIGHT_LOG_BIN_PAD_IDLE 0`, `flight_state_machine.h:10`; `flash_logger.cpp:882-890`) and `I*.BIN` (`flash_logger.cpp:600-608`) only feed RAM pre-launch rings. They are never written to NAND, because a mid-flight reboot never sees the 10 g / 300 ms launch trigger again (`flight_state_machine.h:31-32`).
  - The state machine can never reach Landed, because landing detection runs only from Boost/Coast/Descent (`flight_state_machine.cpp:211`).
  - Only `P*.BIN`, text and UART logs continue, in the new session.
  - The flight session itself survives: `scan_sessions` (`flash_logger.cpp:342-343`) puts the new session in the other slot, away from the slot holding the flight marker.
  - Every "Pad Idle only" gate is open in flight: Wi-Fi on, GPS cold start, pad heading, IMU calibration, commanded launch (see N1).

## Summary table

| # | Title | Verdict | Severity |
|---|-------|---------|----------|
| 1 | Radio mass-storage / erase / clear act in any flight state, no confirmation | CONFIRMED | critical |
| 2 | Radio flight-state set and Iris load commands unauthenticated | CONFIRMED (state set); load forwarding is by design | high (state set) / medium (loads) |
| 3 | Reboot in flight enters maintenance, Wi-Fi up, LoRa silent | PLAUSIBLE (as stated, "any reboot", it is wrong; needs finding 4 armed) | high (critical consequence, very low likelihood) |
| 4 | Armed maintenance request survives a cancelled reboot | CONFIRMED | high |
| 5 | Failed Wi-Fi start in maintenance boot leaves board unlogged and stuck | CONFIRMED | high |
| 6 | Maintenance-exit reboot fires after launch and resets flight state | CONFIRMED (path); flight impact depends on operator launching in maintenance | medium |
| 7 | Landed log download, then launch: flight unlogged | NOT A BUG (dead code path) | n/a (stale docs) |
| 8 | OTA self-test never exercises the NAND write path | CONFIRMED | high |
| 9 | No bad-block handling on write path; one error stops all logging | PLAUSIBLE | high |
| 10 | Export failure during mass-storage start blocks command task forever | CONFIRMED | medium |
| 11 | Mass-storage start failure leaves LoRa/USB/Wi-Fi off | CONFIRMED | medium |
| 12 | USB echo blocks forever if host stops reading | CONFIRMED (bench only) | low |
| 13 | Start watchdog restores LoRa; late AP-up does not re-disable it | CONFIRMED (needs a stall over 15 s) | low |
| 14 | Public default Wi-Fi password; image accepted by project name only | CONFIRMED | medium |
| 15 | Mass-storage ack queued but not sent | PLAUSIBLE (race) | low |
| 16 | `TX` non-0x05 payload silently discarded | CONFIRMED | low |
| 17 | 1 Hz partial-page flush of `S*.BIN` cuts capacity | CONFIRMED | low |
| 18 | UART overflow flag fails OTA self-test | NOT A BUG (unreachable) | n/a |
| N1 | No persistence of flight state across a reset in flight | NEW, CONFIRMED | critical (data), high (rest) |
| N2 | Maintenance entry accepted after panic/WDT/brownout resets | NEW, CONFIRMED | high (hardening) |
| N3 | `esp_restart` paths do not flush logger buffers | NEW, CONFIRMED | low |
| N4 | Dead/stale non-maintenance log-freeze code and handoff text | NEW | low |

## Details per finding

### 1. Radio mass-storage, erase and clear commands in flight — CONFIRMED, critical
- Evidence:
  - `usb_serial_echo.cpp:577-594`: the radio switch handles 0x6868, 0x6869 and 0x686A with an ack and then the action. There is no flight-state check, no source check and no confirmation.
  - Handlers: `start_mass_storage` (`:764-788`), `erase_log_sessions` (`:790`), `clear_log_sessions` (`:801`).
  - `command_bridge.cpp:12-30`: the 3-byte compact form `05 68 68` is normalised and queued, and `process_radio_command` (`:519-521`) accepts length 3. It needs no EOF and no canonical frame. By contrast `IRIS_COMMAND_WIFI`, pad heading, radio power and GPS assist handlers run only when `canonical` (`:550-562`).
  - `sensors.cpp:1361-1424`: any CRC-valid LoRa frame starting with 0x05 is queued. There is no sync, auth or sequence check beyond the radio's own CRC.
  - `FIRMWARE_HANDOFF.md:741-743` acknowledges "Radio can invoke destructive commands too", so the exposure is known but not mitigated.
- Failure path by command, in Boost/Coast/Descent:
  - **05 68 68**: `start_mass_storage` disables TX (`:769`), freezes the logger permanently (`flash_logger.cpp:1013-1014`), turns SPI output off, uninstalls the USB driver and starts TinyUSB. This is irreversible until reset. LoRa is silent apart from nothing (the beacon runs only under Wi-Fi, `sensors.cpp:1431`). Flight logging stops. Vehicle telemetry is gone for the rest of the flight.
  - **05 68 69**: `erase_all_sessions` blocks (`portMAX_DELAY`, `flash_logger.h:142`) while `run()` erases all 1,004 blocks (`flash_logger.cpp:957-968`, `426-448`). That destroys the current flight session (the one holding the flight marker), recreates session 1 and drops queued and pre-launch records. A recreated session has no flight marker (`flight_marker_pending_` is reset in `create_session`, `:406`), so later boots do not protect it. While it runs, the command task is blocked, so further radio commands are not processed.
  - **05 68 6A**: `clear_session_headers` erases block 0 of both slots (`:451-468`) and calls `create_session`, which erases the first block of every stream (`:381-384`). The flight is no longer listed or exportable.
  - Source scope: USB text commands are the same handlers but need physical USB, unreachable in flight. Radio is the real exposure. Note the USB-submitted commands are also re-broadcast on the radio (`command_bridge.cpp:52-63`).
- Edge case: after any reboot in flight the state is Pad Idle (N1), so a Pad-Idle-only gate would not protect this. A fix needs a persisted in-flight flag.
- Fix:
  1. Remove 0x6868/0x6869/0x686A from `process_radio_command`. Mass storage over radio is pointless now that Wi-Fi log download exists, and USB is the bench path.
  2. If a radio path is kept, require all of: canonical 10-byte frame with magic bytes in d0..d3 (for example 0xC0DE5AFE), state in Pad Idle or Landed, and the persisted "in flight" flag from N1 clear. Do not ack-and-act in one step for erase.

### 2. Radio flight-state set and Iris load commands — CONFIRMED (state set), high; load forwarding by design, medium
- Evidence: `usb_serial_echo.cpp:605-612` calls `set_flight_state` (`:1013-1029`), which calls `FlightStateMachine::set_state` (`flight_state_machine.cpp:25-38`) with no transition validation. Any of 0x7300-0x7304 is accepted from any state, over radio, 3-byte form allowed.
- Failure path: `05 73 00` in flight forces Pad Idle. Then `S*.BIN` and `I*.BIN` writing stops (Pad Idle feeds the RAM ring only), the attitude/position filters go to pad mode, and the Wi-Fi/OTA, GPS cold start and calibration gates open. `05 73 04` forces Landed and stops S logging. A stray or hostile packet on 926 MHz SF7/500 with the same sync does this.
- The CH/regulator load commands are forwarded to Iris at `sensors.cpp:1414-1423` and `command_bridge.cpp:79-80`. The ground reaching Iris loads over the radio looks intentional; whether Witness should gate or authenticate it is a system design question, not a Witness bug.
- Fix: over radio accept only state commands that make sense: reject 0x7300 and 0x7304 unless currently Pad Idle/Landed, and reject everything unless the persisted "in flight" flag is clear (N1). Add a short rolling-code or HMAC field to canonical command frames if authentication is wanted.

### 3. Reboot in flight enters maintenance — PLAUSIBLE, high (critical consequence, very low likelihood)
- Evidence: `main.cpp:22` consumes and clears the request at every boot. A reboot with no armed request boots normal flight mode. The claim "any `esp_restart`, panic or watchdog reset" is therefore wrong in general. It needs the RTC request to be armed at the time (finding 4) or a Pad-Idle Wi-Fi command arriving after the reboot (N1; a command, not the reset itself, then triggers the maintenance reboot).
- If it does happen: the maintenance boot queues Enable (`wifi_ota.cpp:112-119`), `state_allows()` is true because the state reset to Pad Idle, LoRa is cut except the 5 s beacon (`:295-296`), the logger is export-only (`main.cpp:65`) and the sensors run unlogged. Auto-off (15 min) then reboots again into normal flight mode.
- Fix: findings 4 and N2 close it. N1 (persisted in-flight flag) makes the Wi-Fi-after-reboot variant impossible.

### 4. Armed maintenance request survives a cancelled reboot — CONFIRMED, high
- Evidence: `wifi_ota.cpp:148` arms the RTC request, then queues Restart with a 400 ms delay (600+400 ms over radio). `monitor()` at `:208-218` refuses the reboot when `state_allows()` is false and maintenance is not active, and only clears `restart_scheduled_`. Nothing calls `maintenance_mode::cancel()` there. The same happens if `xQueueSend` fails (`:152` returns BUSY with the request already armed), and at `:677-679` (OTA) when the state changes inside the 1 s window.
- Failure path: Wi-Fi-on over radio in Pad Idle, a launch trigger (manual `05 73 01` or accel detect) inside the ~1 s window, reboot cancelled, RTC still armed. The flight proceeds normally. Any later panic or watchdog reset in flight boots maintenance (finding 3). RTC_NOINIT also survives a quick power cycle on the bench.
- Window is short (a few hundred ms to ~1 s), so likelihood is low, but cost is a fully unlogged, LoRa-silent remainder of the flight.
- Fix: do not arm in `request_enable`. Keep tx/minutes in WifiOta members and call `maintenance_mode::request(...)` in `monitor()` immediately before `esp_restart()` after the state check passes. Also call `maintenance_mode::cancel()` in the cancelled branch and on failed enqueue. Same for the OTA path.

### 5. Failed Wi-Fi start in maintenance leaves the board there — CONFIRMED, high
- Evidence: `start_wifi` failure unwind (`wifi_ota.cpp:409-425`) restores LoRa but schedules nothing. `monitor()` returns at `:219` because Wi-Fi is off. The auto-off timer is only evaluated when enabled. `handle_wifi` off (`usb_serial_echo.cpp:1008`) returns true without doing anything when `!enabled_`. So the board stays in maintenance, with logging off (`main.cpp:65`) and the status bit set, until power cycle. A launch then flies unlogged (no check in `trigger_launch`, `flight_state_machine.cpp:40-46`).
- Fix: in `handle_request` Enable failure, if `maintenance_mode::active()` call the leave-maintenance path (`maintenance_mode::cancel()`, schedule reboot). Add a maintenance watchdog: if maintenance and Wi-Fi not enabled within ~30 s, reboot to flight mode.

### 6. Maintenance-exit reboot occurs after launch — CONFIRMED (path), medium
- Evidence: `stop_wifi("flight state change")` (`wifi_ota.cpp:222-223`) then `:448-455` schedules a 2 s reboot, and `monitor()` `:211` permits it because maintenance is active and Wi-Fi is off, regardless of flight state.
- Failure path: Wi-Fi up in maintenance, state goes to Boost. Two seconds later the board reboots mid-boost, resets to Pad Idle and re-initialises estimators (N1). Without the reboot it would at least have run the state machine and telemetry, though still unlogged (maintenance has no writer). Launching in maintenance is already an operator error ("NOT FLIGHT READY"), so impact is limited to false launch triggers and operator mistakes.
- Fix: in `stop_wifi`, if maintenance and state is Boost/Coast/Descent, do not reboot: restore LoRa, clear the maintenance beacon, stay up. Reboot only when `state_allows()`.

### 7. Landed log download then launch — NOT A BUG (dead path)
- Evidence: Wi-Fi can only start inside a maintenance boot: `request_enable` (`wifi_ota.cpp:145-153`) reboots instead of enabling when not in maintenance, and `start()` queues Enable only when maintenance is active (`:112-119`). In maintenance the logger is already frozen (`flash_logger.cpp:264`), so `freeze_logs()` takes the `frozen()` shortcut (`:517-524`), never sets `logs_were_frozen_`, and the "Landed only" test is skipped. The "thaw reboot gated on `state_allows()`" branch (`:458`) therefore never runs. The reported failure (freeze in normal mode, thaw reboot skipped) cannot occur via Wi-Fi.
- The USB mass-storage freeze followed by launch is a different path (bench USB), see 1.
- What remains is the maintenance-launch case, which is findings 5 and 6.

### 8. OTA self-test does not exercise logging — CONFIRMED, high
- Evidence: `self_test_passed` (`wifi_ota.cpp:229-237`) checks IMU ready, `FLASH_INIT_FAILED/LOG_FAILED` clear and radio ready. After OTA the new image boots into maintenance (`:677`), where the logger runs `run_export_only` (`flash_logger.cpp:255-267`): only `initialize()` and `scan_sessions(true)`. Neither the startup write/read test (`run_startup_self_test`, `:271`) nor `create_session` nor any program/erase runs. The rollback protection promised in the handoff ("no flash init/log failure flag set") is vacuous for the write path. `esp_ota_mark_app_valid_cancel_rollback()` at `:243` then makes the broken image permanent.
- Fix: in `run_export_only` also run `run_startup_self_test()` (it touches only a reserved block, not session data) and set `FLASH_INIT_FAILED` on failure, so the existing self-test catches it. Do this before the Wi-Fi up, since the validity decision is in the first 30 s.

### 9. No bad-block handling on the write path — PLAUSIBLE, high
- Evidence: `is_bad_block` is used only in `run_startup_self_test` (`flash_logger.cpp:276`) and `erase_session_blocks`/`clear_session_headers` (`:433`, `458`). `create_session` (`:381-384`), the ring/stream block advance (`:523-525`) and `program` (`:534`) do not check it. Layout is a contiguous block map (`flash_logger.h:176-191`) over 1,004 blocks, which assumes no bad blocks in the range, although the part guarantees only about 1,004 good blocks out of 1,024. Any `erase_block` or `program` error propagates out of `append_bytes` and the main loop sets `r != ESP_OK` (`:1044-1051`), which stops all logging and deletes the logger task.
- Whether a factory-bad block fails erase/program or just returns bad ECC is not determinable from code. Plausible on any individual device. Existing flights suggest the current chip is fine.
- Fix: build a logical-to-physical block map at `create_session` and when advancing a ring/stream block, skipping blocks flagged by `is_bad_block`; on a runtime erase/program failure retire that block (skip to next) rather than propagate. As a floor, make `append_bytes` failures mark only that stream full (as the full-stream branch at `:492-502` already does) instead of killing every stream.

### 10. Export failure during mass-storage start blocks the command task — CONFIRMED, medium
- Evidence: `usb_serial_echo.cpp:771` calls `prepare_mass_storage()` with the default `portMAX_DELAY` (`flash_logger.h:139`). In `run()` an error in the export block (`flash_logger.cpp:997-1011`) leaves `r != ESP_OK`; the thread skips the success branch, falls to `:1044-1051`, sets the failure flag and deletes itself without giving `frozen_semaphore_`. The caller blocks forever, TX already off (`:769`), radio command processing stopped (same task as the command loop, `:1096-1123`).
- Fix: pass a finite timeout (`kLogFreezeTimeout`, 15 s) in `start_mass_storage`; on timeout or error restore TX. In the logger's failure path give `frozen_semaphore_` (or set an error flag the waiter checks).

### 11. Mass-storage start failure leaves everything off — CONFIRMED, medium
- Evidence: `usb_serial_echo.cpp:778-787`: output disabled, console off, USB driver uninstalled, then `mass_storage_started_ = mass_storage_.start() == ESP_OK`; on failure nothing is restored and `wifi_ota_.note_mass_storage_active()` (`:768`) blocks Wi-Fi permanently (`wifi_ota.cpp:143`). The task keeps running but TX stays disabled. Rare trigger.
- Fix: on failure, re-enable TX and output and reinstall the USB-serial-JTAG driver, or reboot when the state allows.

### 12. USB echo `portMAX_DELAY` — CONFIRMED (bench only), low
- Evidence: `usb_serial_echo.cpp:157-176`. Echo only happens for received bytes, so it needs a host that sent input and then stopped reading. The USB port is unreachable in flight. On the bench it stalls radio command processing (queue depth 8, `command_bridge.cpp:49`, extra commands dropped at `:89-91`) until the host reads.
- Fix: use a bounded timeout (for example 20 ms) and drop echo on timeout.

### 13. Start watchdog does not re-disable LoRa on a late AP-up — CONFIRMED, low
- Evidence: `wifi_ota.cpp:466-474` restores TX at 15 s; `:399-406` does not re-disable on success. Only reached if a step stalls over 15 s and then completes; a start taking that long is already abnormal. `watchdog_fired_` is used only for the status word.
- Fix: at AP up, if `watchdog_fired_` is set, call `set_radio_transmission_enabled(false)` again (or abort the start).

### 14. Public default password; project-name-only image check — CONFIRMED, medium
- Evidence: `Kconfig.projbuild:11-13` default `witness-ota-2026`; `sdkconfig.defaults` has no override. `check_image_head` (`wifi_ota.cpp:606`) checks magic, chip and project name only; `sdkconfig.defaults` has no secure boot or signed apps. Exposure is limited to maintenance windows (Wi-Fi is not up in flight) and Pad Idle/Landed, but anyone in range can replace the firmware during that window.
- Fix: set a per-vehicle password in an uncommitted `sdkconfig.defaults.local`; consider enabling signed app verification (RSA, `CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT`) so a rogue image fails `esp_ota_end`.

### 15. Mass-storage ack queued but not sent — PLAUSIBLE, low
- Evidence: the ack is queued (`usb_serial_echo.cpp:584`) then TX is disabled at `:769` within microseconds with no guard delay (Wi-Fi uses `kRadioAckGuardMs` for this reason). Whether the sensors task transmits it in between is a race. `queue_immediate_radio_packet` drops when radio is unavailable (`spi_data_forwarder.cpp:126`), so a stale ack after TX returns is possible only if queued before disable.
- Fix: mostly moot once finding 1 is fixed; otherwise add the same ack guard delay.

### 16. TX text payload discarded — CONFIRMED, low
- Evidence: `spi_data_forwarder.cpp:223-229` is a stub; the handoff already says "no hardware SPI transmission yet" (`FIRMWARE_HANDOFF.md:739`). Documented, no flight impact.

### 17. 1 Hz partial-page flush — CONFIRMED, low
- Evidence: `flash_logger.cpp:1024-1026`, `kFlushIntervalMs = 1000` (`flash_logger.h:57`), 40 blocks of 64 pages = 2,560 pages, one page/s gives ~43 min. Flights are shorter. A full stream sets `FLASH_LOG_FAILED` (`:496`), which is only a status flag, not a loss. Note the IMU stream is far tighter (96 blocks at ~960 Hz x 24 B is roughly 9 min of full-rate data, with Descent decimation), worth checking against longest expected pad-hold plus flight.

### 18. UART overflow flag fails OTA self-test — NOT A BUG (unreachable)
- Evidence: the post-OTA self-test runs in the maintenance boot, where `start_export_only` never creates `uart_capture_mutex_` or `uart_queue_`. `log_external_bytes` returns `ESP_ERR_INVALID_STATE` at `flash_logger.cpp:106`, before the overflow branch that sets the flag (`:116`). Pending-verify images never run in normal mode (any reset before validation rolls back).

## New issues missed by the first pass

### N1. No persistence of flight state across a reset in flight — CONFIRMED, critical (data) / high (gates)
- Evidence: `flight_state_machine.cpp:13-18` (state in RAM, init Pad Idle), plus the facts at the top of this file. After a panic, task-watchdog or brownout reset mid-flight:
  - S and I streams are dropped (RAM ring only), the state machine can never leave Pad Idle, and attitude/position estimators run in pad mode during flight.
  - All Pad-Idle-only commands become valid: Wi-Fi on (a radio command then drives the board into maintenance, finding 3), `05 73 01`, GPS cold start (`usb_serial_echo.cpp:567`, loses the fix mid-flight), pad heading, IMU calibration clear (`sensors.cpp:1519`).
  - If a chute-deploy shock exceeds 10 g for 300 ms, a second flight marker is written into the new session; the next boot then considers that slot the newest flight and erases the other slot, which holds the real flight (`flash_logger.cpp:342-343`, `create_session`).
- Fix: keep `{magic, check, state, launch_uptime}` in RTC_NOINIT updated on every state change. At boot, if `esp_reset_reason()` is panic/int_wdt/task_wdt/brownout and the stored state is Boost/Coast/Descent, restore it (and mark `flight_marker` as needed so logging continues in the already-selected flight session). Use the same flag to gate all radio commands in findings 1 and 2, and to refuse maintenance (N2).

### N2. Maintenance entry accepted after crash resets — CONFIRMED, high (hardening)
- Evidence: `maintenance_mode.cpp:24-32` trusts the RTC magic regardless of reset reason.
- Fix: in `consume_boot_request` require `esp_reset_reason() == ESP_RST_SW` (what `esp_restart` gives) and also check that the persisted flight state is not in flight. Panic/WDT/brownout/power-on always fall through to flight mode, which makes finding 3 impossible even if finding 4 is not fixed.

### N3. Reboots do not flush logger buffers — CONFIRMED, low
- Evidence: `wifi_ota.cpp:212-214` waits 100 ms then `esp_restart()`; text, packet and IMU buffers (flushed every 1 s, `flash_logger.cpp:1024-1029`) are lost. Entering maintenance from Landed to download logs therefore loses up to the last second of the flight's tail.
- Fix: call a `FlashLogger::flush_now()` (or `prepare_mass_storage`) before restart when the state is Landed.

### N4. Dead non-maintenance freeze path and stale docs — low
- `freeze_logs` non-maintenance branch, `logs_were_frozen_` and the "Landed only" text (`FIRMWARE_HANDOFF.md:1221-1228`) describe a mode that no longer exists; Wi-Fi only runs in maintenance. Either delete the dead code and update the docs, or the 409 guard could mislead future changes (it is what the first pass built finding 7 on).

## Significant issues to act on (CONFIRMED/PLAUSIBLE, high or critical)
- 1 (critical): remove radio 0x6868/0x6869/0x686A, or require canonical frame + magic + Pad Idle/Landed + persisted not-in-flight flag.
- N1 (critical for data): persist flight state in RTC_NOINIT and restore it on crash resets; use it to gate radio commands.
- 2 (high): reject radio 0x7300-0x7304 in flight; consider authenticated frames.
- 4 (high): arm the RTC maintenance request immediately before `esp_restart()`, cancel on a cancelled/failed enqueue.
- N2 / 3 (high): accept maintenance only when `esp_reset_reason() == ESP_RST_SW` and not in flight.
- 5 (high): on a failed Wi-Fi start in maintenance, reboot to flight mode; add a ~30 s maintenance watchdog.
- 8 (high): run the NAND write/read self-test in the maintenance export-only path before the OTA validity decision.
- 9 (high): skip bad blocks in a logical block map and retire blocks on runtime errors rather than stopping all logging.
