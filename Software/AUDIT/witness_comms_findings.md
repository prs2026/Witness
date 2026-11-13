# Witness comms/storage/update — audit findings

## Overview
- Read-only code audit of `Software/Witness/softwarev1/main` (comms, storage, OTA, maintenance). Nothing built, flashed, run or committed.
- Scope: USB parser, command bridge, radio driver, CAN, flash logger and NAND driver, USB mass storage, Wi-Fi OTA/log download, maintenance boot. FIRMWARE_HANDOFF.md "Wi-Fi OTA and log download" read for intent.
- Main theme: the radio and USB command paths have no flight-state or authentication gate, and the maintenance/Wi-Fi state machine can leave logging or LoRa off during flight.
- Two findings are critical: a LoRa frame can freeze and erase flight logs and silence telemetry in flight (1), and a reboot in flight into maintenance restarts Wi-Fi and silences LoRa (3).
- Maintenance has several paths to unlogged flight (4, 5, 6, 7). The OTA self-test does not exercise the logger write path (8).
- NAND write path has no bad-block handling (9). The mass-storage export failure path can hang the command task forever (10).
- Verified OK: radio drive is clamped to -17..+3 dBm (`hardware_pins.h:85-87`, `ra01.cpp:460-465`); log names are restricted to 8.3 characters, so `/logs/<name>` has no traversal (`wifi_ota_logic.cpp:155-171`); OTA requires Content-Length and checks the image head before any flash write; USB and radio frame lengths are bounded.
- Not verified on hardware. The W25N bad-block behaviour (9) and the timing claims (3, 6, 13) need a bench check.

## Findings

### 1. Radio-received mass-storage, erase and clear commands act in flight — critical
- Where: `usb_serial_echo.cpp:583-594` (LoRa dispatch of 0x6868/0x6869/0x686A, no state check); `usb_serial_echo.cpp:764-788` (`start_mass_storage`, TX disabled at 769, logger frozen at 771, output off at 778); `usb_serial_echo.cpp:790-810` (erase/clear); `flash_logger.cpp:819-845` (`prepare_mass_storage`, `erase_all_sessions`, `clear_all_sessions`, no flight-state check); `flash_logger.cpp:957-969` (erase stops binary/IMU capture while erasing).
- What / Scenario: an unauthenticated 3-byte LoRa frame `05 68 68` in Boost/Coast/Descent freezes flash logging for the rest of the flight (irreversible until reset). It also disables all LoRa transmission, and the beacon path only runs while Wi-Fi is on (`sensors.cpp:1427-1434`), so telemetry goes silent. `05 68 69` or `05 68 6A` erases the current flight's session.
- Confidence: high.

### 2. LoRa commands drive Iris actuators and set flight state without authentication — high
- Where: `usb_serial_echo.cpp:605-612` (radio 0x7300-0x7304 call `set_flight_state`, which accepts any transition, `usb_serial_echo.cpp:1013-1029`, `flight_state_machine.cpp:25-38`); `usb_serial_echo.cpp:613-624` (CH loads and 5 V regulator acked as "already retransmitted by CommandBridge"); `command_bridge.cpp:123-151`, forward at 138; `can_link.cpp:41-50`.
- What / Scenario: any 926 MHz transmitter can send `05 73 00` in flight. That sets PadIdle, which opens the Wi-Fi/OTA and log-download gates (`wifi_ota.cpp:142`, `568`, `780`). It can also switch a Iris load or the 5 V regulator off, since the frame is forwarded over CAN. The only protection is the LoRa CRC.
- Confidence: high on the code path. Whether the lack of authentication is intended is a design question.

### 3. Reboot in flight into maintenance auto-starts Wi-Fi and silences LoRa — critical
- Where: `wifi_ota.cpp:112-119` (maintenance boot queues an Enable at start); `wifi_ota.cpp:180` (checks `state_allows()`, which reads PadIdle after the reset); `wifi_ota.cpp:295-296` (`set_radio_transmission_enabled(false)`, beacon-only on); `flight_state_machine.cpp:13-18` (state is RAM-only and resets to PadIdle); `main.cpp:50-53` (maintenance boot only sets a status bit, logger export-only at `main.cpp:65`).
- What / Scenario: any `esp_restart`, panic or task-watchdog reset while the maintenance request is armed (see 4) boots into maintenance. The reset clears the flight state, so the boot reads PadIdle and Wi-Fi starts. LoRa stays off until the auto-off (default 15 min). Logging is off for the whole boot.
- Confidence: high.

### 4. Armed maintenance request survives a cancelled reboot — high
- Where: `wifi_ota.cpp:145-152` (`maintenance_mode::request` at 148, then Restart queued); `wifi_ota.cpp:208-218` (monitor cancels the Restart at 216-217 without disarming maintenance); `wifi_ota.cpp:677-679` (OTA path arms the same way); `maintenance_mode.cpp:24-31` (only `consume_boot_request` and `stop_wifi`, `wifi_ota.cpp:449`, clear the RTC request).
- What / Scenario: Wi-Fi is requested in Pad Idle or Landed. The state changes to Boost before the 400 ms (or 1000 ms) reboot, so the reboot is cancelled, but the RTC magic stays armed. Any later panic or watchdog reset in flight boots into maintenance (see 3).
- Confidence: high.

### 5. Maintenance boot can fly unlogged, and a failed Wi-Fi start leaves it there — high
- Where: `wifi_ota.cpp:409-425` (failure unwind: no maintenance exit, no reboot, LoRa restored at 424); `wifi_ota.cpp:219` (monitor returns when Wi-Fi is off); `flight_state_machine.cpp:25-46` (`set_state` and `trigger_launch` have no maintenance check); `main.cpp:50-53`.
- What / Scenario: Wi-Fi fails to start (for example `esp_wifi_init` with no memory). The board stays in maintenance with logging off. A launch (accel auto-detect or `05 73 01`) then flies unlogged. The status bit says "not ready", but the vehicle is not refused.
- Confidence: high.

### 6. Maintenance exit reboot happens during flight and resets the flight state — high
- Where: `wifi_ota.cpp:448-455` (leaving maintenance schedules a reboot without checking flight state); `wifi_ota.cpp:211` (reboot allowed in maintenance when Wi-Fi is off); `wifi_ota.cpp:222-223` (flight-state change stops Wi-Fi, which in maintenance then reboots).
- What / Scenario: Wi-Fi is on in maintenance and the state changes to Boost (automatic detect or radio command). The reboot about 2 s later loses that window and resets to PadIdle (see 3). Launch detection may miss the boost.
- Confidence: high on the code path, medium on the flight impact (timing not measured).

### 7. Landed log download, then launch: flight unlogged — high
- Where: `wifi_ota.cpp:510-535` (`freeze_logs`, Landed only, freeze at 526); `wifi_ota.cpp:458-463` (thaw reboot gated on `state_allows()`); `wifi_ota.cpp:222-223` (state change to Boost stops Wi-Fi and skips the reboot); `flash_logger.cpp:989-1013` (freeze stops capture and parks the logger task); `flight_state_machine.cpp:25-38` (Landed to Boost allowed).
- What / Scenario: the operator downloads logs in Landed, then sends `05 73 01`. The freeze is irreversible until reset, and the thaw reboot is skipped because the state is no longer allowed. The second flight is unlogged.
- Confidence: high.

### 8. OTA self-test does not exercise logging; a broken logger image is marked valid — high
- Where: `wifi_ota.cpp:229-237` (`self_test_passed` checks only status flags, IMU, radio); `wifi_ota.cpp:675-679` (new image boots into maintenance); `flash_logger.cpp:241-265` (maintenance mounts read-only and never creates the writer); `wifi_ota.cpp:242-246` (`esp_ota_mark_app_valid_cancel_rollback`).
- What / Scenario: an image whose NAND write path is broken boots into maintenance, passes self-test, and is marked valid. It then runs in flight with logging failing and no rollback.
- Confidence: high.

### 9. NAND write path has no bad-block handling; one bad block stops logging for the boot — high (medium confidence)
- Where: `flash_logger.cpp:379-390` (`create_session` erases stream blocks with no `is_bad_block` check); `flash_logger.cpp:524` and `534-535` (erase and program in `append_bytes`); `flash_logger.cpp:1044-1051` (first error stops the logger and deletes its task); `w25n01kv.cpp:370-398` (`is_bad_block` used only at startup and in erase-all, `flash_logger.cpp:435`).
- What / Scenario: a factory-bad block inside a stream range makes session creation fail, so logging is disabled for the boot. A block that fails mid-flight stops all logging for the rest of the flight. The usable region is 1004 of 1024 blocks, so bad blocks are likely to be present.
- Confidence: medium. How the W25N erases or programs marked blocks needs a bench check.

### 10. Export failure during mass-storage start hangs the command task and leaves LoRa off — high
- Where: `usb_serial_echo.cpp:769-771` (TX off, then `prepare_mass_storage()` with default `portMAX_DELAY`, per `flash_logger.h:139`); `flash_logger.cpp:1012-1013` (only success gives `frozen_semaphore_`); `flash_logger.cpp:1044-1051` (failure path deletes the logger task without giving it); `usb_serial_echo.cpp:1123` (radio command processing lives in the same loop).
- What / Scenario: a flush or scan error during export stops the logger without giving the semaphore. The USB/radio command task blocks forever, so radio commands stop being processed. TX stays off because the restore branch at 773 is never reached.
- Confidence: high.

### 11. Mass-storage start failure leaves LoRa, USB output and Wi-Fi off until reboot — medium
- Where: `usb_serial_echo.cpp:768-769`, `778`, `781-787` (USB driver uninstalled, `mass_storage_.start()` result only sets a flag, no re-enable); `wifi_ota.cpp:143` (`mass_storage_active_` refuses Wi-Fi forever).
- What / Scenario: `tinyusb_driver_install` fails. The board stays silent on LoRa and USB and cannot start Wi-Fi until a power cycle.
- Confidence: high on the code path; the trigger is rare.

### 12. USB echo blocks forever when the host stops reading — medium
- Where: `usb_serial_echo.cpp:157-176` (`echo_bytes`, `portMAX_DELAY` at 168); `usb_serial_echo.cpp:1104-1123` (same task runs radio command processing); `command_bridge.cpp:49` (radio queue depth 8).
- What / Scenario: a ser2net client or terminal sends a command and stops reading. The echo blocks, radio commands stop being processed, and the 8-deep queue fills and silently drops later radio commands. The CAN forward still happens.
- Confidence: high.

### 13. Wi-Fi start watchdog restores LoRa and nothing re-disables it if the start completes late — medium
- Where: `wifi_ota.cpp:466-474` (watchdog at 469-471 restores TX after 15 s); `wifi_ota.cpp:295-296` and `401-406` (no re-disable on AP up); `wifi_ota.cpp:30` (`kStartWatchdogUs`).
- What / Scenario: a start step stalls beyond 15 s, LoRa is restored, the AP then comes up, and LoRa keeps transmitting alongside Wi-Fi. This contradicts the handoff rule that LoRa stops while Wi-Fi is on.
- Confidence: medium.

### 14. Default Wi-Fi password is public and firmware is accepted by project name only — medium
- Where: `Kconfig.projbuild:13` (default `witness-ota-2026`); `wifi_ota.cpp:606-611` (only the image head is checked: magic, chip, descriptor, project name); `sdkconfig.defaults` (no secure boot or signing).
- What / Scenario: anyone within AP range who knows the default password can flash any image with the project name during Pad Idle or Landed. The default is committed to the repository.
- Confidence: high.

### 15. Radio acks for mass-storage start are queued but never sent — medium
- Where: `usb_serial_echo.cpp:583-586` (ack queued, then `start_mass_storage`); `usb_serial_echo.cpp:768-769`; `sensors.cpp:1427-1434` (immediate queue is not drained while TX is disabled).
- What / Scenario: the ground never gets the 0x06 ack for `05 68 68`. The stale ack sits in the 8-deep immediate queue and goes out later if TX resumes (for example after the path in 11).
- Confidence: high.

### 16. TX text command payload is silently discarded — low
- Where: `spi_data_forwarder.cpp:225-229` (`send_spi_packet` is a no-op); `spi_data_forwarder.cpp:190-211` (`queuecommand`); `usb_serial_echo.cpp:1079-1083` (reports success once queued).
- What / Scenario: `TX` commands are acknowledged by the parser but never leave the board.
- Confidence: high.

### 17. Binary stream flushes partial pages every second, cutting capacity by about 40% — low
- Where: `flash_logger.cpp:1024` (periodic `flush_binary`); `flash_logger.h:181` (40 blocks).
- What / Scenario: 50 Hz x 24 B = 1.2 KB/s is written as one partly filled 2036-byte page per second. The S*.BIN stream fills in about 43 min instead of about 72 min. Flight durations are well inside that, so this is a capacity issue, not a loss.
- Confidence: medium-high.

### 18. UART overflow sets a permanent log-failed flag that fails the OTA self-test — low
- Where: `flash_logger.cpp:112-117` (`log_external_bytes` sets FLASH_LOG_FAILED on overflow); `wifi_ota.cpp:234-236` (self-test rejects that flag).
- What / Scenario: a GPS burst that overflows the 32 KB buffer during a pending-verify boot makes the good image fail self-test and roll back.
- Confidence: medium.
