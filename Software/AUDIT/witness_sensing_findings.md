# Witness sensing/estimation — audit findings

Scope: `Software/Witness/softwarev1/main` sensing, estimation and flight-logic modules. Read-only review; no source edited, nothing built, flashed or committed. Line numbers refer to the files as they are in the working tree on 2026-11-12.

## Overview

- `sensors.cpp/.h`: 500 Hz sensor task on core 1. Drains the LSM6DSV320X FIFO (960 Hz, 3 words per slot), timestamps samples from a steered sensor clock, applies calibration, high-g fusion and the 100 Hz low-pass, then feeds the estimator, position filter, FSM, NAND logging and packet queues. Also owns HITL intake, pad heading, radio TX scheduling and calibration captures.
- `attitude_estimator.*`: strapdown quaternion (body to ENU). Median-filtered gyro integration, 1 s stationary windows for bias and tilt, gyro-saturation latch in flight, landed hold.
- `accel_filter.*`: low-g/high-g selection with hysteresis, 2nd-order Butterworth, and an online high-g to low-g offset/scale fit.
- `position_filter.*`: 3-axis [p, v] Kalman filter with a barometer scale state. Barometer is Mach-gated at 0.8; GPS is gated and lag-compensated.
- `heading_align.*`: GPS-to-inertial heading alignment using horizontal velocity changes, plus pad-heading yaw.
- `imu_calibration*.*`: six-face accel fit, gyro bias capture, NVS store.
- `flight_state_machine.*`: PadIdle to Boost (10 g for 300 ms) to Coast (below 1 g for 500 ms) to Descent (95 % of max baro for 500 ms) to Landed (30 s still).
- `ms5607.*`, `pressure_altitude.h`: barometer SPI driver, second-order compensation, ISA altitude.
- `hitl_input.*`: 0x0C/0x0E frame decode, sequencing, timeout.
- `nmea_gps.*`, `gps_receiver.*`, `uart_driver.*`: NMEA decode, receiver identification/config, UART task, GPS fix publication.
- `witness_status.h`, `hardware_pins.h`, `comms.h`: status word, pin map, protocol constants (packed-field offsets and scales cross-checked against sensors.cpp).

Not reviewed in depth: `gps_assist.*`, `gps_i2c_driver.*`, `px1120s_gps.*`, `skytraq_i2c_mailbox.*`, `spi_data_forwarder.*`, `command_bridge.*`, `flash_logger.cpp` beyond the IMU record path, and `..\tests`.

## Findings

### 1. Landed state is effectively unreachable while the radio transmits — high
- Where: `flight_state_machine.cpp:106-116` (motion_stable test and reset); `flight_state_machine.h:37-41` (10 dps gyro limit, 30 s qualification); `sensors.cpp:2380-2387` (observation built from the newest sample); `sensors.cpp:1068-1070` (only the last sample of each pass is applied to the observation); `sensors.h:119-127` and `sensors.h:57-75` (interference spikes of several hundred dps while the PA transmits; ~1 Hz TX).
- What: `landing_detected()` requires every observation in a 30 s run to have |gyro| < 10 dps and |a| within 0.8 to 1.2 g. Any single qualifying-failing sample calls `reset_landing_condition()`. The FSM does not use the median filter that the estimator applies to the gyro, so one spike is enough to restart the clock.
- Scenario: Descent under canopy, radio on at 1 Hz. Each transmission produces one or more gyro spikes well above 10 dps. Any spike that lands on the newest sample of a pass resets the 30 s window. Consecutive clean seconds are needed, so the state stays in Descent indefinitely. Consequences: `Landed` logging policy never engages, the attitude is never held, and any landed-only behaviour never runs.
- Confidence: medium (the spike magnitude and TX correlation are from the code comments; no bench trace was read).
- Fix direction (not applied): feed the landing test from the estimator's median-filtered gyro, or count violations with a debounce instead of resetting on one sample.

### 2. Coast to Descent uses the un-gated barometer through transonic flight — high
- Where: `sensors.cpp:2388-2390` (`barometric_altitude_valid = barometer_healthy`, no Mach test); `flight_state_machine.cpp:249-268` (descent threshold and 500 ms hold); `position_filter.cpp:251-260` and `position_filter.h:17-24` (the filter zeroes baro trust at Mach >= 0.8 because shock waves corrupt static pressure).
- What: the FSM consumes raw barometric altitude at all speeds. The position filter explicitly refuses the same sensor above Mach 0.8, so the two disagree about whether the reading is trustworthy.
- Scenario: burnout at Mach 1 to 1.5 in coast. A shock-induced static-pressure error reads the altitude tens of metres low for longer than 500 ms. `below_descent_threshold` holds and the FSM enters Descent early, which is the signal that fires drogue or main deployment. A low error of about 5 % of apogee altitude (50 m at 1 km) is enough.
- Confidence: medium (depends on the size of the transonic error, which the code does not quantify).
- Fix direction (not applied): apply the same Mach gate to the FSM apogee test, or hold the Coast to Descent decision until Mach is below 0.8.

### 3. Position predict silently discards any IMU interval longer than 100 ms — medium
- Where: `position_filter.cpp:55-57` and `:119-121` (dt > 0.1 s returns before any state or covariance change); `sensors.cpp:1584-1588` and `:1571-1577` (`position_last_predict_us_` is advanced before the call, so the interval is not retried); compare `sensors.h:110-115` and `attitude_estimator.h` (attitude gap limit is 50 ms).
- What: for a gap longer than 0.1 s, neither position, velocity nor covariance changes. The acceleration over that gap is lost, and P is not inflated to represent the unknown motion. The attitude path rejects gaps above 50 ms, so the two paths disagree on what a gap is.
- Scenario: a FIFO stall of 120 ms during boost at 800 m/s (the FIFO overruns after about 76 ms, and the sample clock re-anchors, which produces exactly this gap). The position loses about 96 m of advance and velocity loses the integrated acceleration over those 120 ms. The covariance is unchanged, so the filter is overconfident and the barometer/GPS gates may reject the correction.
- Confidence: high on the code path; the magnitude depends on how often stalls over 100 ms occur on the bench.
- Fix direction (not applied): on a long gap, integrate with the last sample for the whole interval and inflate P by the process noise over dt, or mark the filter as degraded for the gap.

### 4. GPS UART byte stream is scanned for radio-bridge frames — medium
- Where: `uart_driver.cpp:24-44` (`canonical_frame_length` accepts IDs 0x01, 0x02, 0x03, 0x05, 0x0E-range and others); `uart_driver.cpp:441-490` (every received byte on UART1 goes through this scanner, and a complete frame is queued with `queue_immediate_radio_packet`); `comms.h:307-308` and `:330-381` (EOF is 0x0A, which is also the NMEA LF).
- What: the scanner is meant for host packets, but UART1 is the GPS port. Binary receiver replies (UBX ACK, SkyTraq replies, and the assist-upload responses) contain bytes equal to 0x01, 0x02, 0x03 and 0x05. Each such byte starts a "frame" that swallows the following `canonical_frame_length` bytes of genuine receiver output. The inter-byte timeout (20 ms) never fires on a continuous stream, so a false frame runs to its full length.
- Scenario: during receiver configuration or an assist upload the GPS emits a UBX ACK containing 0x05. The scanner collects the next N bytes of NMEA. If byte N is 0x0A (an NMEA line end, about 1 in 70 bytes), the collected text is queued as an immediate LoRa packet with packet ID 0x05 (COMMAND). Otherwise a "discarded UART packet with invalid EOF" warning is logged, and the real frame bytes that followed are lost to the scanner. Two effects: spurious bytes on the air and corrupted real frames if a host ever uses this port.
- Confidence: medium (depends on whether a host packet path over UART1 is intended and whether binary replies are enabled in flight configuration; the GPS driver does send UBX configuration).
- Fix direction (not applied): gate the bridge on a configuration with no GPS on UART1, or require the frame ID and an EOF plus a checksum or a longer, non-NMEA-compatible EOF.

### 5. Barometer "valid" is set without a range check; altitude failures are ignored — medium
- Where: `ms5607.cpp:281-284` (`pressure_altitude_meters` return value is discarded and `sample_.valid = true` is set regardless); `pressure_altitude.h:18-20` (returns false and leaves altitude unchanged for non-positive pressure); `sensors.cpp:2331-2332` and `:2354-2356` (`ms5607_ready` and `barometer_healthy` taken from `valid`); `flight_state_machine.cpp:198-209` and `:256-257` (FSM consumes the altitude with no further check). The position filter only checks `pressure >= 1000 Pa` (`position_filter.cpp:237`); there is no upper bound anywhere.
- What: a compensated pressure of zero or below leaves `altitude_meters` at its previous value and still reports a valid sample. A pressure above the sensor's 1200 mbar range produces a negative altitude that the FSM accepts. The only corruption guard is the ADC value being zero (`ms5607.cpp:82`).
- Scenario: a corrupted D1 read (no CRC on the ADC word) gives about 5500 mbar (approximately -4 km altitude). The FSM sees altitude far below 95 % of max during Coast and enters Descent after 500 ms of consecutive bad readings. A stuck-high or stuck-low D1 gives the same result indefinitely. A non-positive pressure keeps the last altitude, so a dead sensor reads as healthy.
- Confidence: medium-high on the code; frequency of bad D1 words on hardware is unknown.
- Fix direction (not applied): reject pressure outside 1000 to 120000 Pa (10 to 1200 mbar) before setting `valid`, and set `valid` from the altitude function's return value.

### 6. Blocking work inside the 500 Hz sensor loop — medium
- Where: `ms5607.cpp:48-49` (`vTaskDelay` inside `initialize()`); `sensors.cpp:2294-2308` (barometer re-initialisation attempted once per state period, i.e. every 1 s while absent, with SPI device add/remove, PROM reads and a reset delay); `sensors.cpp:2338-2353` and `:1320-1331` (battery ADC: 16 blocking `adc_oneshot_read` calls every 100 ms).
- What: each barometer retry stalls the loop for roughly 4 to 5 ms plus SPI transactions, and the battery sample stalls it for the 16 conversions. `xTaskDelayUntil` then counts an overrun and the next IMU pass runs late. The loop was designed to be non-blocking (the comment at `sensors.cpp:2227-2230` says so for radio work).
- Scenario: barometer unplugged or unresponsive on the pad. Every second the 500 Hz loop loses about 5 ms, eating into the 76 ms FIFO margin only slowly but adding timestamp steering error (the clock snaps for errors above 5 ms, `sensors.cpp:1040-1044`). Under this condition the FIFO timing is still safe, but a worst-case combined stall (barometer retry plus battery plus a late radio TX) can approach the FIFO budget.
- Confidence: medium (timing is inferred from the code; no measured loop times were read beyond the `max_pass_us` counter that the code already reports).
- Fix direction (not applied): move barometer re-initialisation and battery sampling to a helper task or spread the battery conversions across passes.

### 7. Barometer scale and altitude observations are not tied to fresh sensor data in the FSM — low (medium if a zero-sample pass occurs)
- Where: `sensors.cpp:1067-1070` (`apply_imu_sample` is only called when `imu_batch_count_ > 0`); `sensors.cpp:2362-2392` (FSM observation is built whenever `lsm_result == ESP_OK`, from `working_sample_` regardless of whether a new sample arrived); `sensors.cpp:2266-2268` (`process_imu_batch` is skipped on a zero-sample pass).
- What: when a pass drains no FIFO words, the observation repeats the previous sample while the timestamp advances. Launch, burnout and landing conditions accumulate time on a value that has not been re-measured. Only the newest sample of each pass is ever observed, which compounds finding 1.
- Scenario: a 0.5 ms FIFO empty window at 500 Hz (possible at startup or when the sample clock is re-anchored) yields a stale "10.2 g" or "still" observation counted as a new 2 ms sample.
- Confidence: medium on the code; impact on flight is small because zero-sample passes should be rare with 960 Hz data at 500 Hz passes.
- Fix direction (not applied): pass an explicit "new IMU sample" flag into the FSM observation and let the condition timers ignore passes without one.

### 8. FSM state can be overwritten by a concurrent command — low
- Where: `flight_state_machine.cpp:193-241` (state read once with `state()`, then `set_state(Boost/Coast/Descent/Landed)` written later in the same update); `flight_state_machine.cpp:25-45` (`set_state` accepts any state and `trigger_launch` checks then acts without a lock); `flight_state_machine.h:96-98` (`state_` atomic but no guard).
- What: the sensor task and the command task both write the FSM state. A command that moves the state between the sensor task's read and its write is silently overwritten, and `observed_state_` then trails the real state for one pass.
- Scenario: a manual `set_state(Landed)` issued from USB as the Coast to Descent threshold is crossed. The sensor task writes Descent after the command, and the logged phase is Descent instead of the commanded Landed.
- Confidence: medium on the code; practical impact limited to manual overrides at the moment of a transition.
- Fix direction (not applied): compare-and-swap the transition (`compare_exchange`) so an external change wins, or route all state changes through the sensor task.

### 9. IMU NAND record timestamp wraps at 71.6 minutes — low
- Where: `flash_logger.cpp:52-58` (`write_u32_be(d, timestamp_us)`, low 32 bits of the esp_timer time); `sensors.cpp:2087-2088` (`static_cast<std::uint32_t>(timestamp_us)`); `flash_logger.h:93-94` (layout comment).
- What: the 960 Hz IMU record stores only the low 32 bits of the microsecond clock, so it wraps every 4295 s. Any analysis that does not unwrap will reorder samples after that point.
- Scenario: the logger runs from boot through a long pad hold, or a bench session exceeds 71 minutes. The prelaunch ring and flight records then show a backward timestamp jump.
- Confidence: high on the layout; impact depends on whether the logging window can exceed 71 minutes.
- Fix direction (not applied): store a boot-relative millisecond base or a wrap counter in the record's spare byte, or document the wrap for the ground tool.

### 10. Heading alignment is applied to the aligner even when the attitude is not valid; GPS velocity uses pass time — low
- Where: `sensors.cpp:1699-1714` (`heading_aligner_.add_gps` always runs; `attitude_.rotate_heading` only if `attitude_.valid()`); `heading_align.cpp:136-146` (aligner rotates its own velocity and anchor unconditionally); `sensors.cpp:1703` (fix time is `pass_start_us`, not the fix's measurement time).
- What: when the attitude is lost (gyro saturation in flight), the aligner still rotates its inertial state, so the two diverge. When the attitude is later reset, it is re-initialised from gravity only on the pad, so the divergence does not persist in practice. Separately, GPS velocity over the 1 s window is computed from the time the pass saw the fix rather than the time it was measured; timing jitter of up to one pass (2 ms) is about 1 m/s error at 500 m/s.
- Scenario: saturation during descent followed by a GPS heading correction: the aligner accepts a correction that is not applied to the attitude, so `total_correction_rad` and the aligner's velocity drift from the real attitude.
- Confidence: medium on the mismatch; the magnitude is small.
- Fix direction (not applied): skip `add_gps` or set the aligner untrusted when the attitude is not valid, and time GPS samples from the fix's parse time.

### 11. Stored IMU calibration is not range-checked on load — low
- Where: `imu_calibration_store.cpp:112-119` (blob copied with only a version check); `imu_calibration.cpp:302-305` (`apply` divides by `accel_scale` and `high_g_scale` directly); `attitude_estimator.cpp:444-449` (`normalized()` returns identity silently when the norm is NaN).
- What: the blob is trusted without a finite or range check. The solver enforces 0.9 to 1.1 scale (low-g) and 0.85 to 1.15 (high-g) before storing, so only a corrupted-but-CRC-valid blob or a future layout change could reach this path. A zero or NaN scale becomes an infinite or NaN specific force; `normalized()` then quietly resets the attitude to identity while `valid_` stays true.
- Scenario: an NVS record is rewritten by a future firmware with a different scale convention. On boot the accelerometer scale is 0, the filtered acceleration is infinite or NaN, the attitude silently resets, and the position filter integrates NaN.
- Confidence: low (requires a corrupted or incompatible record).
- Fix direction (not applied): validate the loaded calibration with the same range checks as `solve()` and reject it otherwise.

## Notes on what was checked and found consistent

- Packed-field scales and offsets (`comms.h` against `sensors.cpp`), including the 10000 GPS scale, 40 µg filtered-accel count, 70 mdps gyro count and int24 ranges, all fit their fields.
- Units in the estimator, position filter and FSM (g, m/s², mdps, rad/s, Pa as centi-mbar) are consistent at every hand-off I traced.
- The attitude window reset and tilt correction, the running median centre index, the `exp_rotation` Taylor branch, and the 6-face solver were checked; no defects found.
- Wrap-safe tick comparisons (`TickType_t` subtraction) are used throughout the sensor loop.
- The sensor task's `log_imu_sample`, `log_state_packet` and `log_kalman_packet` are non-blocking (queue with zero timeout), so NAND stalls cannot block the 500 Hz loop.
