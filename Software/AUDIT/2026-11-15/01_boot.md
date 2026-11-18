# Audit 01: boot, images, bricking (read-only, working tree as found 2026-11-15)

Paths relative to Software/Witness/softwarev1. Line numbers approximate (+-3).
Not read in full (time): usb_mass_storage.*, maint_beacon.h, tests. sdkconfig read via grep of the relevant keys.

## Findings table

| # | Sev | Conf | Where | Summary |
|---|-----|------|-------|---------|
| B1 | Critical | confirmed by reading | main/main.cpp:88 + sensors.cpp:229-233,260-264 | ESP_ERROR_CHECK(sensors.start()): a single failed IMU SPI read/write or WHO_AM_I mismatch (no retry) or battery-ADC calibration failure aborts app_main -> panic -> reboot -> same init -> boot loop. No sensor task, no logging, no telemetry. |
| B2 | Critical | confirmed by reading | usb_serial_echo.cpp:~1000 (state check), :1022-1033 (switch), maintenance_mode.cpp restart_now | Maintenance switch is checked only when the command arrives. Restart fires 400 ms (USB) / 1100 ms (radio) later with no re-check. If the state becomes Boost in that window (launch, or USB 0x7301), the SW restart boots the maintenance image mid-flight: no sensors, no logging. |
| B3 | High | confirmed by reading | sdkconfig (ESP_TASK_WDT_PANIC not set), no esp_task_wdt use in main/ | No watchdog covers the flight tasks. TWDT watches only the two idle tasks and does not panic. A hung/deadlocked sensors, flash_logger, data_forwarder task (or hang in app_main) is never reset; only INT_WDT (300 ms, interrupts blocked) and panics recover. "Silently stops" is possible. |
| B4 | High | confirmed by reading | wifi_ota.cpp run() (~126-139) | Auto-off is gated on `enabled_` (AP up). If start_wifi() fails (heap, netif), enabled_ stays false and the maintenance image never leaves on its own; only a LoRa/USB "Wi-Fi off" or a power cycle does. Pad unit stuck in maintenance at launch. |
| B5 | High | confirmed by reading | main.cpp:59,64,65,68,89 | Other ESP_ERROR_CHECKs (forwarder, bridge, USB echo, flash_logger.start, heartbeat) abort on ENOMEM / USJ driver install / GPIO config failure. Flight heap is tight (sdkconfig.defaults notes Sensors::start ENOMEM once). Should degrade, not abort. |
| B6 | High | confirmed (logic) / plausible (field) | sensors.cpp:260-264, :711-726 | initialize_battery_adc requires curve-fitting calibration. adc_cali_create_scheme_curve_fitting returns an error on a chip without ADC eFuse calibration -> start() fails -> B1 boot loop. A different board/chip unit may be permanently unbootable for flight. maint_radio.cpp handles the same failure gracefully. |
| B7 | Medium | confirmed | flight_selftest.cpp:21-28,50-55 | Self-test of a pending-verify flight image requires radio_ready + IMU ready + no flash fail. Failing (dead radio, NAND fault) calls mark_invalid_rollback_and_reboot even if already in Boost/Coast (resumed boot within 30 s). Also a failed radio rejects a good new image. |
| B8 | Medium | confirmed | imu_calibration_store.cpp init_storage (~85-95) | Flight image erases the WHOLE NVS partition on NO_FREE_PAGES / NEW_VERSION_FOUND: IMU calibration lost (flies uncorrected), boot bookkeeping (fslot, mconf) lost. After a lost fslot, select_flight_slot picks newest build, which can be a just-rolled-back bad image. |
| B9 | Medium | confirmed | maintenance_mode.cpp arm(); boot_manager.cpp prepare_switch | arm() ignores NVS errors silently. With NVS broken, prepare_switch returns OK, ack OK, restart goes to maintenance, no request found -> immediately back to flight. Misleading, pad only. |
| B10 | Medium | confirmed / plausible for RTC clobber | usb_serial_echo.cpp:1030-1033 | If restart_after_ms fails, only cancel() runs; esp_ota_set_boot_partition(maint) is NOT reverted. Any later reset (including a panic in flight) boots maintenance first, which returns to flight (SW restart). Adds ~1-2 s, and the maintenance image may use RTC fast memory holding flight_state_store's RTC_NOINIT record (images do not share RTC layout; ALLOW_RTC_FAST_MEM_AS_HEAP=y) so resume could be lost. Needs test. |
| B11 | Medium | plausible | sdkconfig ESP_BROWNOUT_DET_LVL 7, INTR | Brownout = immediate reset, no holdoff. With the 3V3 LDO swap pending, repeated dips give repeated reboots; each reboot re-runs B1 init (IMU not answering during a sag -> B1 loop). Brownout classed Warm (flight_state_store.cpp); checksum protects the record. |
| B12 | Medium | confirmed | maint_main.cpp | Maintenance image has no sensors/launch detection; if entered and forgotten (HTTP activity keeps touching, auto-off max 240 min, recovery mode never leaves) the rocket cannot fly. Procedural mitigation (beacon flag) only. |
| B13 | Low | plausible | flight_selftest.cpp stack 3072 | Self-test task does NVS commit + otadata rewrite + ESP_LOG on a 3072 stack; usb_serial_echo overflowed 3072 doing similar work (usb_serial_echo.h:43). Overflow -> panic while pending verify -> rollback. Check high-water mark. |
| B14 | Low | confirmed | boot_manager.cpp return_to_flight comment | Verified against IDF 6.1 esp_ota_ops.c:1179: mark_valid acts on the active otadata entry whenever state != VALID (NEW included), so the second mark works; no bug. Handoff/comment wording about which entry rollback lands on is inconsistent (it lands on the old flight entry). |
| B15 | Low | confirmed | flash_logger.cpp ~1080-1100 | One transient NAND error permanently stops logging (task deleted, FLASH_LOG_FAILED). Outside my facet; also feeds B7. |

## Details

### B1 (Critical) sensors.start() failure aborts the boot
main.cpp:88 `ESP_ERROR_CHECK(sensors.start())`. Sensors::start returns early with an error when initialize_spi, the IMU init, or battery ADC init fails (sensors.cpp:200-233, 260-264). initialize_lsm6dsv320x (731-827) returns on the first non-OK SPI result or WHO_AM_I mismatch, after one fixed boot delay, no retry. Barometer and radio are correctly non-fatal; the IMU is the only mandatory part and is the piece most likely to be mid-state after a mid-flight brownout/WDT reset. Failure -> abort() -> PRINT_REBOOT -> reboot (delay 0) -> identical init -> loop. The RTC record stays valid so state is "resumed" every time, but no estimator/FSM runs, no packets are sent, nothing new is logged (heartbeat and the rest are started after this line, so they never start). Matches the worst outcome.
Fix direction: retry IMU init (with IMU soft reset) several times; on final failure continue with IMU flagged not ready and let the sensor task keep retrying (as for the baro). No ESP_ERROR_CHECK in app_main.

### B2 (Critical) TOCTOU on the maintenance switch
State gating exists in handle_wifi (wifi_ota_logic::state_allows_wifi: Pad Idle/Landed) and command_policy::allow_log_maintenance (0x6868). After the check, switch_to_maintenance -> prepare_switch (NVS + otadata writes, several ms) -> restart timer 400 ms (+700 ms radio guard). restart_now() is a bare esp_restart(); reset reason SW plus armed request is exactly what maintenance honours. If the FSM goes Pad Idle -> Boost inside that window (real launch, or USB set-state 0x7301 which is allowed from Pad Idle) the rocket reboots into the maintenance image, which ignores the RTC flight record and has no sensors/logging.
Fix: re-check state in the restart callback; if not Pad Idle/Landed, cancel the request AND restore the boot partition to the running flight slot; refuse Boost entry while a switch is pending. Revert the boot partition on every aborted switch (B10).
usb_serial_echo.cpp is being edited concurrently; line numbers may shift.

### B3 (High) no application watchdog
ESP_TASK_WDT_EN=y, 5 s, idle-task checks on both CPUs, `ESP_TASK_WDT_PANIC` not set, so TWDT only prints. No task subscribes (no esp_task_wdt use in main/). INT_WDT 300 ms panics. Bootloader WDT 9 s guards only the bootloader. A task wedged on a mutex/queue (flash_logger.cpp uses xQueueSend(..., portMAX_DELAY) ~line 952 and xSemaphoreTake(portMAX_DELAY) in capture paths; the sensors task) stops flight functions with no reset. Since warm resets already resume from RTC, add a subscribed TWDT with panic for sensors/flash_logger/data_forwarder, keeping idle checks non-panicking (NAND erase starvation).

### B4 (High) maintenance never auto-exits when the AP failed
run(): `if (enabled_.load() && window != 0 && now - last_activity >= window) leave(...)`. enabled_ is set only on success (step 12). On failure failed_ is set and the task idles forever. Operator can send Wi-Fi off by LoRa (if radio OK) or USB; otherwise a power cycle (boot slot already points at flight, so it recovers). Fix: apply auto-off independent of enabled_.

### B5/B6 other abort sites, ADC calibration
See table. flash_logger.start() fails only on ENOMEM; NAND failure is NOT fatal at boot (init runs inside the logger task, sets FLASH_INIT_FAILED, deletes the task). GPS UART, CAN, NVS failures degrade correctly. Radio, baro degrade. IMU, SPI bus, battery ADC do not.

## Questions asked
- Unable to boot flight / stuck in maintenance at launch? No flash-brick path found: every maintenance reset restores the flight boot slot first (maint_main.cpp 156-166, return_to_flight; consume() honours only ESP_RST_SW). Stuck-in-maintenance paths: B2, B4, B12. Both flight slots bad: maintenance enters recovery (Wi-Fi, no timeout) by design. Corrupt otadata: bootloader boots the first valid app in table order (flight_a even if older than flight_b). Power loss during OTA: only the inactive slot is touched; boot pointer changes after esp_ota_end validation. Power loss between arm() and restart: self-heals (non-SW reset ignores request).
- Panic/abort/WDT in flight: PRINT_REBOOT, delay 0; panic print over USJ bounded by s_usbserial_timeout (panic.c:113-120); bootloader verifies image then boots the valid flight slot; app_main resumes Boost/Coast/Descent from RTC_NOINIT (Warm classes include BROWNOUT). Reboot time not measured. Resume depends on B1 not firing.
- Rollback marking a good flight image invalid: only while pending-verify (first 30 s after an OTA boot) and self-test fails (B7). After return_to_flight the flight entry is VALID, so no selftest/rollback ping-pong.
- ESP_ERROR_CHECK/abort on runtime paths: only app_main (B1,B5). Assertions enabled (library asserts abort -> reboot). Stack canary overflow -> panic.
- NVS full/corrupt: flight erases (B8); maintenance never erases; all users tolerate failure; arm silent failure (B9).

## Earlier documented findings still present (FIRMWARE_STATE_MACHINE.md, image/boot facet)
- The doc note that rollback lands on the old flight entry (not maintenance) is correct per IDF source.
- Handoff claim "a switch is only allowed in Pad Idle/Landed" holds only at command time (B2); not documented as a finding.
- "Not bench verified: switch, return, rollback, OTA flows" still applies. BENCH_REPORT not re-read in this pass.

## Checked and OK
- partitions.csv: aligned, no overlaps, flight 2 MB vs 437 KB image, maint 1.5 MB vs 849 KB; otadata 0x2000; free space 0x720000+.
- maintenance_mode::consume: read-and-erase first, SW-reset only, check field; cancel() on every flight boot.
- return_to_flight: hint -> last_good -> newest; other-slot fallback; recovery if none.
- prepare_switch failure paths cancel the request; SHA-256 validated by esp_ota_set_boot_partition.
- OTA handler: header/chip/project/kind checks, slot size, never writes return/running slot, unconfirmed maintenance refuses OTA, buffers freed, stall limits, compat check with force.
- Maintenance verification boot: boot slot not pointed to flight until pass; failure rolls back to the previous entry.
- Flight image links no Wi-Fi/TinyUSB; maintenance flash logger is read-only (cannot destroy flight data).
- NAND/radio/GPS/CAN/baro init failures degrade without abort.
- USB console with no host: panic path bounded, VFS write drops when not connected.
- Bootloader: app rollback on, anti-rollback off, bootloader WDT 9 s.
