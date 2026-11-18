# Audit 04: sensor task and sensor drivers (working tree as found, 2026-11-15)

Read-only audit. Other files were being edited concurrently; nothing cited here looked mid-edit, but doc line refs in FIRMWARE_STATE_MACHINE.md (e.g. sensors.cpp:2475) are already stale (now about 2515).

## Findings table

| # | Sev | Conf | Where | Summary |
|---|-----|------|-------|---------|
| 1 | High | confirmed | main.cpp:88, sensors.cpp:229-285 | Any init failure in Sensors::start (IMU SPI/WHO_AM_I, battery ADC calibration, task create) aborts through ESP_ERROR_CHECK: boot loop, and the radio is never brought up (it is initialised inside start) |
| 2 | High | confirmed (trigger plausible) | sensors.cpp:991-993, 1000-1034, 1076-1078, 2304-2321, 2408-2422 | No IMU liveness check or re-init in flight. A dead or reset IMU (FIFO diff 0, or 0x00/0xFF reads) is ESP_OK forever; has_*_sample_ flags are sticky, so the FSM keeps getting stale data marked valid (documented M8, still present) |
| 3 | High | confirmed | sdkconfig:2030-2036; no esp_task_wdt use in main/ | Nothing supervises the sensor loop. Task WDT is log-only (PANIC not set), sensors task not subscribed. A hung sensor task never resets the chip, though the firmware can resume flight state after a WDT reset |
| 4 | High | plausible, needs test | ra01.cpp:366-384, sensors.cpp:1498-1516, 2498-2501 | Radio runs inside the sensor task. wait_until_ready blocks up to 500 ms per command; radio_ready_ is never cleared. A wedged SX1262 stalls the loop 0.5-1.5 s per attempt each packet period: FIFO (about 76 ms) overruns, FSM/Kalman starve |
| 5 | High | confirmed by reading (cross-facet) | sensors.cpp:1604-1607, 2432-2439 | Launch detection needs position_filter_.initialized(), which needs a valid attitude (stationary window). If attitude never initialises there is no AGL, no Boost, no flight logging, 3 s telemetry |
| 6 | Medium | confirmed | sensors.cpp:484-516, flight_state_machine.cpp:217-224, flash_logger.cpp:644,946 | HITL exit/timeout keeps the simulated FSM state (documented M1, still present); a HITL flight writes a NAND flight marker and so consumes never-erase slots |
| 7 | Medium | confirmed (trigger unlikely) | imu_calibration_store.cpp:112-119, imu_calibration.cpp:294-308 | Stored calibration not validated on load; scale 0 or NaN would be applied (divide) in flight and poison filters permanently |
| 8 | Medium | confirmed | sensors.cpp:1026-1029, 1085-1088 | A read error after FIFO words were drained (second burst or temperature read) discards the whole batch silently |
| 9 | Low-Med | confirmed | ms5607.cpp:82-104, 271-284; pressure_altitude.h:18; sensors.cpp:2362-2374 | Baro has no plausibility check; valid=true on garbage; pressure<=0 leaves stale altitude; ready bit stays set on stale data. Kalman guards limit the impact |
| 10 | Low | confirmed | sensors.cpp:2121 | NAND IMU timestamp is uint32 microseconds: wraps at 71.6 min |
| 11 | Low | confirmed | sensors.cpp:1076-1078, 2309-2320 | Ready status bits (and flight_selftest) follow per-pass any_* flags and flicker on a pass with 0 new samples |
| 12 | Low | confirmed | sensors.cpp:1397-1406 | Radio RX logs a hex dump (up to 16 ESP_LOGI lines) from the sensor task: multi-ms stall per packet |
| 13 | Low | confirmed | sensors.cpp:1979-1983, 2042, 2103 | A calibration capture finishing just as motion starts (state still Pad Idle until AGL > 30 m) runs attitude_.reset(); NotStill normally stops it first |
| 14 | Low | confirmed | sensors.cpp:1354 | publish_sample takes sample_mutex_ with portMAX_DELAY (holders only copy a struct) |
| 15 | Low | confirmed | sensors.cpp:806-814 | INTERNAL_FREQ_FINE read not sanity-checked; effect bounded (period factor >= 0.83) |

## Details

### 1. Sensors::start failure is fatal and takes the radio with it (High)
main.cpp:88 `ESP_ERROR_CHECK(sensors.start())` aborts on any error. start() returns an error (after release_resources) when: IMU SPI or WHO_AM_I fails (sensors.cpp:229-233, 735-743); battery ADC unit/channel/curve-fitting calibration fails (260-264, 680-729; a board without ADC calibration data cannot boot); the log queue or task cannot be created (266-285). Radio init (247-258) happens inside start() after the IMU, so a dead IMU or a transient SPI glitch at power-up (bench notes an unstable 3V3 at power-up) gives a reboot loop with no beacon, no GPS capture, no telemetry. A mid-flight warm reset that hits this goes into the same loop. Fix: treat IMU, ADC and baro failures as degraded modes with retry in the task; fail only on resource exhaustion; never block radio/GPS on them.

### 2. No IMU supervision (High)
fetch_lsm6dsv320x fails only on SPI API errors, which a healthy-looking bus never returns. If the IMU loses configuration or stops (reset, bad write, EMI), FIFO_STATUS reads 0 (words 0) or 0xFFFF (words 511, overrun, all words unknown tag 0x1F, counted in fifo_bad_words_). Either way lsm_result is ESP_OK, imu_batch_count_ is 0, apply_imu_sample never runs, but has_gyro_sample_/has_low_g_accel_sample_/has_high_g_accel_sample_ (set at 991-993, cleared only in reset_sensor_sample) stay true, so run() builds observation.*_valid = true (2408-2422) from the last working_sample_. The FSM is updated every pass with frozen values and wall-clock timestamps, so dwell timers keep running on stale data. Only the status ready bits drop. initialize_lsm6dsv320x is never re-run (only landed_restore_sensors, disabled). Fix: if no FIFO sample for 20-50 ms, clear sticky flags, mark the observation invalid, re-initialise (check WHO_AM_I and CTRL1/FIFO_CTRL4 periodically). The barometer has an init retry (2332-2346); the IMU has none.

### 3. No supervision of the sensor loop (High)
sdkconfig: task WDT enabled, 5 s, idle-task checks both cores, but `# CONFIG_ESP_TASK_WDT_PANIC is not set`; no esp_task_wdt_add/reset anywhere in main/. Interrupt WDT (300 ms, CPU1 checked) covers only interrupt-disabled hangs. A deadlock or non-yielding loop in the sensors task (core 1, priority 4, alone on the core) would print at most and never reset. Since flight_state_store resumes in-flight state after WDT/panic resets, enabling panic-on-TWDT and subscribing the sensor task (feed once per pass) turns a silent hang into a recoverable reset. All blocking constructs I found in the task are bounded, so this is a missing safety net rather than a known hang.

### 4. Radio blocking inside the sensor loop (High, plausible)
ra01.cpp wait_until_ready: 300 us spin, then up to kBusyTimeoutMs = 500 ms of vTaskDelay(1). start_transmit issues about 8 commands, poll_transmit 3 (clear IRQ and SetRx run even after failure), receive 2-4; each can wait 500 ms; a failure returns at the first timeout. radio_ready_ is set at init only. With a faulted radio (PA/front-end brownout on TX; USB over-current trips are documented) service_radio_transmit retries every packet period (radio_last_sent_ is set before start_transmit, 1501), so the loop is blocked about half the time at 1 Hz packets. The FIFO holds about 76 ms and bench backlog already reached 99 of about 219 words. Kalman predict skips gaps over 100 ms (KALMAN_AUDIT #5), FSM updates stop, the IMU log has holes. Fix: disable the radio after N consecutive failures, cap the BUSY wait to a few ms in flight, or move the radio to a core-0 task.

### 5. Launch detection chain needs a valid attitude (High, cross-facet)
run() supplies AGL only when position_filter_.initialized() (2432-2439); predict_position returns without predicting while !attitude_.valid() (1604-1607) and initialized_ is set only inside predict()/predict_without_imu(). Attitude validity needs an accepted stationary window (attitude_estimator.cpp:268-347). The bench saw "attitude estimator never initialized" with gyro LPF1 and radio on (sensors.h:121-128). With no attitude there is no Boost, no NAND binary/IMU logging (RAM ring only) and 3 s telemetry for the whole flight. No GPS/baro/accel-magnitude fallback launch cue exists.

### 6. HITL (answers to the HITL questions)
- Entry: only a canonical 10-byte 0x05 frame with command 0x4849 over USB, and only when the FSM is Pad Idle (hitl_input.cpp:187-207, usb_serial_echo.cpp:407-411; radio entry rejected 591-593). Stray bytes or a stray 0x0C cannot enter it. A stray 0x0C while HITL is inactive does make the USB parser swallow the next 46 bytes (usb_serial_echo.cpp:178-200), which can eat a following real command (USB facet).
- Remain: leaves automatically 1000 ms after the last 0x0C/0x0E input (hitl_input.cpp:231-238); any source may exit. While active the loop reads no real sensor (sensors.cpp:2284-2298), so only a USB host streaming sim data keeps it on.
- After exit or timeout: sources are reset and the FSM rebaselined (484-516) but the FSM state is NOT reset (flight_state_machine.cpp:217-224); a HITL sim flight leaves Boost/Coast/Descent/Landed persisted in RTC with real sensors. Still present (M1). A bench HITL run just before launch can leave the unit in Landed (no launch detection; 0x7300 refused from Boost/Coast/Descent). HITL flights also write the NAND flight marker, so two HITL runs make both slots "flights" and the real flight is not logged (status byte 1 bit 7) until 0x6869/0x686A. Pre-launch checklist: state Pad Idle, LOG SLOTS FULL clear, HITL flag clear.
- Calibration and other Pad-Idle-only services: request_calibration requires Pad Idle and not HITL (1543-1551); a capture aborts when the state leaves Pad Idle (1976-1978); a capture does not change the data path until it finishes. Residual race in #13.

### 7. Calibration load
load() checks version and length only; NVS has entry CRC32 so random corruption is detected, but a logically bad blob is applied: apply() divides by accel_scale/high_g_scale (imu_calibration.cpp:302-305). Scale 0 gives inf/NaN; the Butterworth biquad (accel_filter.cpp:39-41) and attitude/Kalman recurrences never recover from NaN until a source reset. Fix: on load require finite values within Limits, else discard.

### 8. Partial drain loss
If burst 1 succeeds and burst 2 fails, or the 10 Hz temperature read fails after the FIFO read, fetch returns an error; run() processes imu_batch_ only on ESP_OK (2304). Samples already removed from the FIFO are dropped. Fix: process what was read, then report the error.

### 9. Barometer
Poll state machine cannot wedge: conversion==0 or SPI error restarts the conversion and returns an error (rate-limited log); a failed start_conversion keeps the old start time so the next pass retries. No range check on the compensated pressure; pressure<=0 leaves sample_.altitude_meters stale while valid stays true. The sensor is specified from 10 mbar (about 31 km); above that readings are unreliable, and baro is un-gated in Descent only (Kalman has a min-pressure guard and innovation gate).

## Earlier documented findings in this facet still present
- M8 (no IMU retry in the normal loop): present (#2).
- M1 (HITL exit leaves simulated flight state): present (#6).
- M6 (frozen barometer reported healthy): present; no stale-value detection in ms5607 poll.
- L9 (HITL exit acked OK when inactive): present (sensors.cpp:355-367).
- H3 (cold reset in flight leaves Pad Idle) interacts with #5.
- KALMAN_AUDIT #5 (predict gap over 100 ms skipped) is what #4 triggers.

## Checked and OK
- FIFO read arithmetic: burst <= 36 words = 252+1 bytes <= 258; fifo_tx_/fifo_rx_ are 258 B DMA-capable internal; rx index max 252; word cap 144 = 48 slots = imu_batch_ size; finish_fifo_slot guards the batch bound; HITL queue take bounded by 48.
- Unknown FIFO tags counted and skipped; slot grouping handles repeated channels and TAG_CNT change; overrun bit resets open slot and clock; DIFF_FIFO 9-bit decode; temperature register 0x20.
- Timestamp reconstruction: monotonic (increments are never negative because now >= clock), double precision, no wrap in 64-bit; snap above 5 ms.
- Register values vs comments: CTRL1/2 0x0C, CTRL8 0x43, CTRL9 0x08, CTRL6 0x3C, CTRL7 0x01, HG 0xA4 (320 g, 960 Hz), FIFO_CTRL3 0x99; sensitivities 488 ug, 10417 ug, 70 mdps consistent with comms.h.
- Low/high-g handoff: any axis of either sensor >= 12 g switches immediately; return after 96 samples below 10 g; HighGCalibration scale clamped 0.85-1.15 (no divide by zero).
- SPI3 is used only by the sensor task (LSM6, MS5607, RA-01); NAND is SPI2 in other tasks; core 1 hosts only this task.
- Loop pacing: xTaskDelayUntil deadline scheduling; an overrun runs bounded back-to-back catch-up passes; no IDLE1 starvation in normal operation.
- Calls from the sensor task into data_forwarder, command_bridge, can_link, flash_logger, uart fix read are non-blocking (timeout 0 or short spinlock copy).
- Battery ADC: errors return early, 64-bit math, packet encode clamps; not used by flight logic.
- MS5607: CRC4 and all-0/all-FF PROM check, 64-bit compensation with second-order terms, conversion timing 9.1 ms vs 9.04 max, 1 s init retry.
- Calibration capture buffers: gyro_stored_ < 1024 guard; 4 s at about 936 Hz / 4 = 936 samples; solver rejects non-finite or out-of-range results; NVS save/erase on a separate low-priority core-0 task; flight state persists to RTC, not NVS (no flash-cache stall from persist).
- Stack: 8 KB task; last measured 6 KB free before Kalman additions (not re-measured).
