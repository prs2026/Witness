# Audit 08: cross-cutting concurrency and system resources (Witness flight firmware)

Scope: main/ plus sdkconfig (active flight config), sdkconfig.flight, partitions.csv, CMake. Read-only. Working tree as found (another agent is editing flight_state_machine.*, usb_serial_echo.cpp, sensors.cpp etc.; none of the findings below sit in obviously mid-edit code, but line numbers may drift).
Flight image links only: cpu_load, can_link, twai_driver, command_bridge, uart_driver, nmea_gps, hitl_input, gps_receiver, gps_assist, flight_state_machine, attitude/accel/position/heading, imu_calibration(+store), ms5607, sensors, sensors_landed, spi_data_forwarder, heartbeat, usb_serial_echo, flight_state_store, flight_selftest, flash_logger, w25n01kv, ra01, boot_manager, maintenance_mode. (gps_i2c_driver, px1120s, wifi, maint_* are NOT in the flight image.)

## 1. Task inventory (flight image)

| Task | Core | Prio | Stack B | Blocks on / cadence | Notes |
|---|---|---|---|---|---|
| sensors (sensors.cpp:274) | 1 | 4 | 8192 | xTaskDelayUntil 2 ticks (500 Hz) | Only task on core 1. Never blocks if a pass exceeds 2 ms (bench: ~13 overruns/s, max pass 3.4-7.5 ms, core 1 ~45 % busy). Owns SPI3 (LSM6, MS5607, RA-01). Logs from this task go straight to console + flash text hook. |
| sensor_log (sensors.cpp:268) | 0 | 1 | 3072 | queue (4 x 400 B) | prints deferred status lines |
| flash_capture (flash_logger.cpp:98) | 0 | 4 | 3072 | vTaskDelayUntil 20 ms; xQueueSend(portMAX_DELAY) | blocks forever if writer died |
| flash_logger (flash_logger.cpp:103) | 0 | 3 | 7168 | vTaskDelayUntil 20 ms; NAND polling with vTaskDelay(1) | sole user of SPI2/NAND. Deletes itself on first error (see F2) |
| usb_serial_echo (usb_serial_echo.cpp:130) | 0 | 3 | 6144 | usb read 20 ms | also runs ALL radio command handling, erase/clear, maintenance switch (NVS + OTA calls) |
| data_forwarder, spi_interface (spi_data_forwarder.cpp:31,43) | 0 | 2,2 | 3072 each | queues | forwarder: usb write 20 ms, then drops when stalled. spi_interface is a stub consuming a queue |
| uart1 GPS (uart_driver.cpp:72) | 0 | 3 | 4096 | uart_read 2 ticks | |
| iris_can (can_link.cpp:32) | 0 | 3 | 4096 | vTaskDelay(1) poll loop (1 kHz) | |
| heartbeat (heartbeat.cpp:62) | 0 | 2 | 2048 | vTaskDelayUntil | independent of sensors: its LED/packet stays alive if sensors is dead |
| cpu_load (cpu_load.cpp:42) | 0 | 1 | 4096 | 10 s | |
| selftest (flight_selftest.cpp:74) | 0 | 3 | 3072 | only when image pending verify; self-deletes | |
| imu_cal_save (imu_calibration_store.cpp:77) | 0 | 1 | 3072 | transient, NVS write (pad only) | |
| esp_timer task | 0 | 22 | 3584 | maintenance_mode restart timer only | |
| IDF: IDLE0/IDLE1 (1536), Tmr Svc (2048, prio 1, no affinity), ipc0/1 (1280), usb_serial_jtag driver ISR | | | | | |
| ISRs | | | | TWAI rx/tx/state callbacks (twai_driver.cpp:220-245, not IRAM ISR, so masked during flash ops); USJ driver ISR; UART driver ISR; SOF tick hook. No application GPIO ISR (DIO1 ISR only in unreachable run_landed). | |

Watchdog / fault config (sdkconfig): INT_WDT 300 ms both cores (l.2020-2024) -> panic -> CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT (l.1986, delay 0): reboot, not halt. TASK_WDT 5 s, checks only IDLE0/IDLE1, CONFIG_ESP_TASK_WDT_PANIC NOT set (l.2030): a timeout only prints. No esp_task_wdt_add() anywhere in main/. Bootloader WDT 9 s covers boot only. Stack overflow canary on (l.2518) -> panic -> reboot. Brownout detector on, level 7 (reset). C++ exceptions off; HEAP_ABORT_WHEN_ALLOCATION_FAILS off (a failed `new` aborts via the no-exceptions path; none is used at runtime anyway). Console = USB Serial/JTAG, log level INFO.
In-flight reset recovery exists (flight_state_store, RTC_NOINIT, resumes Boost/Coast/Descent and logs to the other NAND slot), so a panic/WDT reboot is survivable; a silent hang is not.

## 2. Shared state

| State | Writers / readers | Protection | Verdict |
|---|---|---|---|
| WitnessStatus bytes | all tasks | std::atomic bytes, fetch_or/and | OK (byte write() is 3 separate loads; harmless) |
| FlightStateMachine::state_ | sensors, USB/radio task (set_state), many readers | atomic + CAS one-way | OK |
| FSM maximum_altitude_/persist() | sensors writes, USB task persist() reads | none | benign (aligned float), see L1 |
| RTC_NOINIT flight record seal() | sensors + USB task | none; magic cleared first | torn only if two transitions race: negligible |
| Sensors latest_sample_ | sensors (publish, portMAX_DELAY take), flash_capture (take timeout 0) | FreeRTOS mutex | OK; capture silently writes zero pressure/temps if it loses the race (L2) |
| working_sample_, position/attitude filters | sensors only | n/a | OK |
| hitl_controller_, hitl IMU queue | USB task, sensors | portMUX hitl_lock_ ; copies <= 48 x 24 B, ~us | OK |
| pad_heading request | USB/radio task, sensors | portMUX | OK |
| UartDriver latest_fix_ (Fix ~64 B) | uart task, sensors | portMUX copy | OK. Freshness uses unsigned `now - received_ms`; now is read after the copy so no wrap race |
| assist_pending_ (4 KB) | USB task, uart task | portMUX with 4 KB memcpy (~10-20 us, irqs off on that core) | OK, Low |
| radio_cache_ (7 x ~104 B) | sensors (cache), sensors (read) | portMUX | OK |
| Packet queues (forward, command, immediate radio, flash packet/imu/binary/text) | FreeRTOS queues, all senders use timeout 0 except flash_capture | | OK; sender never blocks the sensor task |
| calibration_request_, radio_transmission_enabled_, tx power, front_end_enabled_ | atomics | | OK |
| CanLink pending_*/resend_* | CAN task only; commands via queue | | OK |
| TwaiDriver rx ring/tx slots | ISR + CAN task | portMUX (_ISR variants in ISR) | OK, no blocking calls in ISR |
| FlashLogger flags (erase_requested_, capture_*_, export_requested_) | atomics | | OK |
| FlashLogger erase_result_ | writer task then waiter after semaphore | semaphore ordering | OK |
| uart_capture_mutex_ | uart task, flash_logger | mutex (priority inheritance) | OK |
| imu_calibration_store pending/busy | sensors -> save task | atomic busy gate | OK |
| usb_serial_jtag console write_lock | every logger vs USB driver | vfs recursive lock | see F6 |

Lock ordering: no code path holds two locks. No portENTER_CRITICAL section calls blocking or long code. No non-ISR-safe call found in the TWAI ISR callbacks (uses *_ISR variants and twai_node_receive_from_isr).

## 3. Buses

| Bus | Devices | Lock | Notes |
|---|---|---|---|
| SPI3 (GPIO15/17/18) | LSM6 (CS35, 10 MHz, polling), MS5607 (CS21, polling), RA-01 (CS16, 8 MHz, polling) | none needed: all three are used only from the `sensors` task | FIFO burst <= 253 B ~0.2 ms; radio 255 B writes ~0.26 ms; but radio BUSY waits up to 500 ms (F3). The only other user of SPI3 would be nothing. |
| SPI2 (quad pins 9-14) | W25N01KV NAND only | none; only the flash_logger task (and export task in maintenance) | block erase 2-10 ms, page program ~0.7 ms, polled with vTaskDelay(1); 20 ms ready timeout (w25n01kv.h:46). Core 0, does not touch the IMU bus. |
| UART1 (GPIO4/5) | GPS | driver ring buffers 8 KB + 8 KB | |
| TWAI | Iris | ISR + portMUX | |
| Internal flash | NVS (calibration, boot_manager), esp_ota | flash op disables cache on both cores | only on pad/landed commands; INT_WDT 300 ms vs GD flash sector erase worst case: see M5 |

No device handle is used from two tasks. `spi_device_polling_transmit` is used everywhere (no queued mixing).

## 4. Findings

| ID | Sev | Conf | Where | Summary |
|---|---|---|---|---|
| F1 | High | confirmed (reading) | sdkconfig:2030, 2034-2036; no esp_task_wdt_add in main/ | No watchdog that can reset a hung/dead sensors, logger or radio path; no supervisor |
| F2 | High | confirmed | flash_logger.cpp:1101-1109; append_bytes 502-555; w25n01kv.cpp program/erase | First NAND error permanently ends all logging for the flight; no retry, no bad-block skip |
| F3 | High | confirmed (trigger plausible) | ra01.cpp:364-387 (kBusyTimeoutMs 500, ra01.h:54); sensors.cpp:2498-2501, 1368, 1447 | A misbehaving radio stalls the 500 Hz sensor/flight-state loop up to 500 ms per command |
| F4 | High | confirmed | sensors.cpp:247-258; no retry anywhere | Radio init is single-shot; one failed init at boot = no telemetry for the whole flight (and nothing recovers it) |
| F5 | Medium | confirmed | main.cpp:88; sensors.cpp:260-264, 266-272 | Non-critical init failures (battery ADC/cal, log task/queue) abort() -> reboot loop instead of degrading |
| F6 | Medium | confirmed (reading) / plausible (timing) | sensors.cpp:1397-1406, 1735, 1692 etc.; flash_logger.cpp:228-248 | ESP_LOG calls made directly in the 500 Hz task go through console write + extra vsnprintf; can stall if a USB host is attached but not draining (50 ms first-char block) |
| F7 | Medium | confirmed | flash_logger.cpp:934-936, 952; spi_data_forwarder.cpp:254 / usb_serial_echo.cpp:161-164 | Unbounded blocking sends/writes (portMAX_DELAY) that deadlock a task if the consumer dies |
| F8 | Medium | confirmed | sdkconfig:1848, 4068 (PM_ENABLE, TICKLESS_IDLE) | Power management compiled in for a flight that never sleeps: ISR/idle hooks and a light-sleep path that can still be reached by any code that calls esp_pm_configure |
| F9 | Medium | plausible, needs test | usb_serial_echo.cpp:~736-748 (erase_log_sessions), 830-870 | Radio-triggered erase/clear/maintenance switch run on the USB task, blocking radio command handling for seconds; NVS/OTA flash ops freeze both cores (INT_WDT 300 ms) |
| F10 | Medium | plausible, needs test | sensors stack 8192; usb 6144; uart1 4096; can 4096; heartbeat 2048 | Stack headroom unmeasured since Kalman/position filter was added to the sensors task |
| F11 | Medium | confirmed | cpu_load.cpp:91-120 | Core-1 loop has no starvation guard for IDLE1; 5 s TWDT only prints |
| L1 | Low | confirmed | flight_state_machine.cpp:44-49, 55 | persist() reads sensor-task fields from the USB task unsynchronised |
| L2 | Low | confirmed | flash_logger.cpp:569 (latest_sample(sample, 0)) | 50 Hz record gets zero pressure/temperature when the mutex is contended |
| L3 | Low | confirmed | heartbeat.cpp:285-314 | Heartbeat LED/USB packet reports liveness of the heartbeat task only; ground cannot infer sensor health from it |
| L4 | Low | plausible | ra01.cpp write_buffer/read_buffer 257/258 B stack buffers | SPI driver may heap-allocate a DMA bounce buffer per transaction if alignment demands it (fails with ESP_ERR_NO_MEM when the heap is ~35 KB and fragmented) |
| L5 | Low | confirmed | usb_serial_echo.cpp:156-172 | echo_bytes loops forever with portMAX_DELAY if USB host stops reading mid-echo (bench only) |

### F1 No effective watchdog / supervisor (High)
- sdkconfig:2030 `# CONFIG_ESP_TASK_WDT_PANIC is not set`; TWDT watches only IDLE0/IDLE1 (l.2034-2036), and nothing in main/ calls esp_task_wdt_add. The RTC/bootloader WDT is disabled after boot. Only INT_WDT (interrupts masked > 300 ms) and stack canary/panic lead to a reboot.
- Scenario: `sensors` task blocks forever (e.g. a radio SPI/GPIO wedge, mutex or driver deadlock, `publish_sample` xSemaphoreTake(portMAX_DELAY) at sensors.cpp:1354 if a reader never gives) or the flash_logger task has deleted itself (F2). No reset occurs, no log of it, flight state machine and radio telemetry (which only the sensors task generates/sends: sensors.cpp:2498-2501) stop. Heartbeat LED and heartbeat packet keep running, hiding it. If the sensors task merely never blocks (passes always > 2 ms) IDLE1 starves and TWDT prints, nothing more.
- Fix direction: subscribe `sensors`, `flash_logger` (and radio progress) to the TWDT, set CONFIG_ESP_TASK_WDT_PANIC=y (reboot resumes flight state via flight_state_store), or add a small supervisor task checking heartbeat counters and calling esp_restart(). Keep the resume path in mind (new session goes to other slot; if both slots hold flights nothing is logged).

### F2 One NAND error kills logging for good (High)
- flash_logger.cpp:1101-1109: any `r != ESP_OK` from drain/flush -> capture flags cleared, FLASH_LOG_FAILED set, `vTaskDelete(nullptr)`. Errors come from append_bytes: erase_block/program return ESP_FAIL on the chip's erase/program failure bits, or ESP_ERR_TIMEOUT after 20 ms (w25n01kv.h:46), or any SPI error. NAND program/erase failures are normal over life (and a bad block inside the contiguous 1004-block layout is hit at first use: is_bad_block is only consulted in erase_session_blocks/clear_session_headers, never in append_bytes; erase_block of a factory-bad block can fail).
- Scenario: pad wait for hours, then one failed page in IMU stream mid-boost: IMU/packet/GPS/text streams all stop; only radio telemetry remains. flash_capture then blocks forever in xQueueSend(portMAX_DELAY) (flash_logger.cpp:952), harmless but silent.
- Fix direction: on error mark that block bad, advance to the next block, retry; never delete the task; keep other streams running.

### F3 Radio faults stall the sensor loop (High)
- ra01.cpp wait_until_ready: 300 us spin then `vTaskDelay(1)` polling until 500 ms (ra01.h:54). Called before every command, from the `sensors` task (core 1, prio 4): start_transmit (>= 6 commands, sensors.cpp:1504/1447), poll_transmit, receive (when DIO1 reads high, sensors.cpp:1368).
- Scenario: RA-01 brownout/latch-up on TX (documented: USB over-current when it transmits; unstable 3V3 observed on the bench) or DIO1 stuck high: every pass blocks up to 500 ms, 1 ms-granular. IMU FIFO (76 ms) overruns every pass, attitude restarts timing (imu_clock_valid_=false), flight state machine/Kalman updates and launch/apogee detection are delayed or skipped; telemetry stops. With DIO1 stuck high, poll_radio_commands retries every pass => loop effectively ~2 Hz for the rest of the flight.
- Fix direction: much shorter BUSY timeout (a few ms), count consecutive failures and disable/reset the radio (radio_ready_=false, re-init attempt) instead of retrying every pass; or move the radio into its own task on a separate time budget.

### F4 Radio init single-shot (High)
- sensors.cpp:247-258: `radio_ready_ = (initialize()==ESP_OK)`; on failure `radio_.release()` and "radio disabled until reboot". Unlike MS5607 (retried every 1 s, sensors.cpp:2332-2346) there is no retry. Nothing else re-initialises it (also flight_selftest only gates rollback of a pending image).
- Scenario: at power-up the supply sags (bench: eFuse/MAC read flips, bootloader refused image) while the RA-01 reset/probe runs; init fails once (reset pulse, BUSY, RAM round-trip check at ra01.cpp ~lines 150-160); Witness then sits on the pad up to 12 h with a dead radio and no ground indication other than the `radio_ready` log and data_forwarder flag. Whole flight without telemetry although the radio hardware is fine.
- Fix direction: retry init from the sensor loop (as for MS5607) every few seconds while on the pad, and after any TX failure streak.

### F5 Non-critical init failure aborts boot (Medium)
- main.cpp:88 `ESP_ERROR_CHECK(sensors.start())`; sensors.cpp:260-272: battery ADC init/calibration failure (e.g. missing eFuse cal -> ESP_ERR_NOT_SUPPORTED at sensors.cpp:725 path) or sensor_log queue/task creation failure returns error -> abort -> reboot loop. The IMU failing (WHO_AM_I) is legitimately fatal, but a battery monitor or a log printer should not brick the flight. Also main.cpp:59,64,65,68,89 abort on heap/queue failure (flash_logger.start() allocates a 49 KB contiguous IMU queue + 32 KB stream buffer + 14 KB packet queue + 8 KB text queue, flash_logger.h kImuQueueDepth 2048). Documented in FIRMWARE_HANDOFF.md:85-87 (still present).

### F6 Logging from the sensor task (Medium)
- ESP_LOG calls remain in the 500 Hz task: radio RX at INFO plus ESP_LOG_BUFFER_HEX_LEVEL of up to 255 bytes (sensors.cpp:1397-1406, one call per received command frame, every uplink frame), heading alignment (1735), pad heading (1692), attitude init (2151), LSM/MS5607/battery errors (2327, 2356, 2387, 2509). Every log call: IDF formats via the registered hook FlashLogger::log_vprintf (flash_logger.cpp:228), which calls the console vprintf, then vsnprintf again into a 256 B LogLine on the caller's stack and a queue send (3 hook calls per ESP_LOG: prefix, body, newline, each a separate LogLine).
- With no USB host (flight) console writes fail fast (usb_serial_jtag_write returns -1 when not connected). With a host connected but not draining (pad with USB attached) the first character blocks 50 ms (usb_serial_jtag_vfs.c tx via driver) while holding the vfs write_lock; any other task logging waits on that lock (prio inheritance only).
- Risk in flight is low; on the pad it explains earlier 16-28 ms stalls. Move the remaining in-loop logs to defer_log().

### F7 Unbounded blocking calls (Medium)
- flash_logger.cpp:934-936, 952 xQueueSend(binary_record_queue_, ..., portMAX_DELAY) in capture task: blocks for good after F2 or an erase in progress (queue reset helps only partially).
- usb_serial_echo.cpp:161-164 (also L5) and the forwarder (spi_data_forwarder.cpp:254, bounded to 20 ms, OK).
- sensors.cpp:1354 publish_sample portMAX_DELAY on a mutex (only holder: flash_capture for a ~200 B copy; ok today).
- imu_calibration_store: fine.
No deadlock found today; these are single points that turn a dead consumer into a stuck producer.

### F8 Power management compiled in (Medium)
- sdkconfig:1848 CONFIG_PM_ENABLE=y, 4068 TICKLESS_IDLE=y, PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=y. With kLowPowerModeEnabled=false (landed_logic.h:19) esp_pm_configure is never called in flight (confirmed: only sensors_landed.cpp:251/422, unreachable), DFS is off (no PM_DFS_INIT_AUTO), CPU stays 160 MHz, vApplicationSleep skips light sleep (s_light_sleep_en false). So no sleep or slowdown today. Residual risk: PM hooks run on every ISR and idle entry, the USJ no-light-sleep lock config (`CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION`) exists only because sleeping killed USB, and re-enabling the flag later makes the chip sleep in flight on battery with the radio/GPS unattended. Recommend PM_ENABLE=n and TICKLESS_IDLE=n in sdkconfig.flight while low-power Landed stays off (also the CPU-power-down retention buffer allocation failure seen on the bench at ~8 KB).
- Brownout: enabled, level 7 (~2.51 V), reset-on-brownout; resume handles it. Panic: reboot (good, not halt). Console: no blocking when nothing attached.

### F9 Command-path blocking on the USB task (Medium, plausible)
- Radio-commanded erase/clear (usb_serial_echo.cpp erase_log_sessions/clear_log_sessions, allowed on Pad Idle/Landed) waits on erase_semaphore_ with portMAX_DELAY (flash_logger.cpp:854, timeout default) while the writer erases up to 1004 blocks (seconds; the fast clear is 2 blocks). During this the USB task cannot process further radio commands (including set-state/launch commands). Prelaunch logging is switched off meanwhile. A stray uplink erase on the pad right before launch costs the prelaunch record and several seconds of command handling.
- prepare_switch/NVS/OTA calls freeze both CPUs during flash erase/program (cache off); allowed only Pad Idle/Landed. A GD/XMC sector erase worst case can approach the 300 ms INT_WDT (sdkconfig:2022) -> panic reboot into flight (harmless on pad: Pad Idle not resumed).

### F10 Stack headroom (Medium)
- Last measured: sensors 6 KB free of 8 KB (2026-11-08, before Kalman/position/heading). The deepest paths on that task: poll_radio_commands (255 B packet + Ra01 read_buffer 2 x 258 B + ESP_LOG_BUFFER_HEX + log hook 256 B + printf of doubles ~2 KB), report_position/defer_log (400 B), finish_calibration_capture (double 6x6 matrices ~600 B, pad only), attitude window median. Likely fine but unmeasured; cpu_load prints stack free per task every 10 s (use it in a soak). usb_serial_echo already overflowed once (3072 -> 6144); heartbeat 2048 uses ESP_LOGE with %d only (ok); uart1 4096 runs gps_receiver with ESP_LOG of doubles/%lu (ok but unmeasured); iris_can 4096.

### F11 Core-1 starvation
- sensors.cpp:2529: xTaskDelayUntil returns pdFALSE when behind; the loop then never sleeps while behind. Passes reach 7.5 ms vs a 2 ms period (about 13 overruns/s on the bench). At 45 % average load there is idle time, but a sustained overload (F3, or HDOP/GPS bursts) leaves IDLE1 unfed; TWDT only prints "Task watchdog got triggered" every 5 s. See F1.

## 5. Heap and time

Heap (35 KB free per bench, startup allocation ~110 KB queues/buffers + stacks): no runtime malloc/new/std::vector/std::string in the flight image after startup (grep clean; only heap_caps_calloc of two 258 B DMA buffers at start, sensors.cpp:220). Runtime heap users are IDF internals: NVS open/commit (pad only), esp_timer_create for the restart (maintenance_mode.cpp:107), task creation for imu_cal_save, SPI bounce buffers if misaligned (L4), esp_ota (pad/landed). No fragmentation growth over a 12 h wait is expected. The bench message `sensor packet queue failed: ESP_ERR_NO_MEM` is NOT heap: xQueueSend on the 16-deep forward queue returns full, mapped to ESP_ERR_NO_MEM (spi_data_forwarder.cpp:75-77); radio telemetry uses a separate cache, so it is unaffected. A failing allocation at runtime returns NULL from IDF APIs (errors), `new` would abort; no `new` used.

Time: esp_timer int64 us truncated to u32 ms/us in several places, all handled with unsigned subtraction (nmea_gps.h gga_fresh, sensors.cpp:2332 tick diffs, note_gps_fix 32->64 extension at sensors_landed.cpp:~83, hitl timeouts). u32 microsecond timestamps in the NAND IMU/packet records wrap every 71.6 min (flash_logger.cpp:68 `static_cast<uint32_t>(timestamp_us)`, :141): a 12 h pad wait plus flight records wrap repeatedly; consumers must unwrap using sequence/order (the prelaunch ring only holds 2 s so launch itself is unambiguous). Tick counter diffs (TickType_t 32-bit at 1 kHz, 49.7 days) fine. UTC ms-of-day handled with fmod and day wrap (landed_logic.h:84-117), only for radio slots. No ms-of-day compare bug found. uint16 counters in Kalman log wrap by design (u16()). 12 h pad: no 32-bit overflow hazard in control paths (state-machine timers use u64 us).

## 6. Earlier documented findings still present
- IMU or battery ADC startup failure aborts boot via ESP_ERROR_CHECK (FIRMWARE_HANDOFF.md:85-87) -> F5, still true.
- "TWDT watches only the idle tasks, no panic" (FIRMWARE_HANDOFF.md:1775-1777) -> F1, still true.
- Radio init failure disables radio until reboot (FIRMWARE_HANDOFF.md:~84) -> F4, still true.
- Bench report bug 4: CONFIG_PM_SLP_DISABLE_GPIO in build/sdkconfig; active sdkconfig now has it unset (checked), no longer relevant while low-power is off.
- Bench report bug 3 (CPU power-down retention allocation failure at light-sleep entry) not reachable now.
- Transient `sensor packet queue failed: ESP_ERR_NO_MEM` during flash writes: explained above (queue full, not heap), harmless for radio.

## 7. Checked and OK
- No lock held across NAND erase/radio TX; no mutex taken in two orders; portMUX sections are short, no blocking inside.
- Only one task touches each SPI host; polling transactions only.
- TWAI ISR uses ISR-safe calls only; not IRAM ISR so it is masked (not crashing) during flash ops.
- All cross-task queues sent with timeout 0 from time-critical tasks (sensors never blocks on a queue).
- Flight state is a CAS-guarded atomic; one-way enforced; persisted in RTC_NOINIT with checksum; reset reasons PANIC/INT_WDT/TASK_WDT/WDT/SW/BROWNOUT resume.
- Panic = print + reboot, stack canary on, brownout reset on, bootloader rollback on; no HALT setting.
- No light sleep/DFS can occur in flight (esp_pm_configure never called; DFS off); CPU fixed at 160 MHz.
- NAND is not written from the sensor task; sensor loop only enqueues.
- No dynamic allocation in steady state; no exceptions; no big stack arrays in estimators.
- GPS fix freshness arithmetic is wrap-safe and race-safe.
- Flash text/GPS rings are launch-protected (no overwrite after Boost).
