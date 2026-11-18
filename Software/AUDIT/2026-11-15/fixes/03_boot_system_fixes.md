# Fixes 03: boot, supervision, IMU liveness, config (audit W5, W7, W16, W19, B1/B5/B6/B7/B8/B13, F1/F8/F10)

Firmware: Software/Witness/softwarev1. Nothing committed, flashed or run on hardware. All new behaviour is host-tested only where isolated (below) and otherwise build-checked.

## Summary table

| Finding | Status |
|---|---|
| W5 / B1 / B5 / B6 init failure boot loop | fixed (IMU, ADC, log task non-fatal; radio init now always reached; only "cannot create sensor task" restarts) |
| W16 / sensors #2 IMU liveness | fixed (detect, mark invalid, re-init, count); thresholds below; not bench tested |
| W7 / F1 / B3 watchdog | partly: sensors task subscribed with panic; logger task needs 2 lines from the logging agent (hook provided) |
| F8 PM / tickless | fixed: both turned off in the flight overlay (compiles, see build) |
| W19 calibration validation | fixed + host test |
| B8 NVS erase | fixed (no erase in a resumed flight; erase on cold boot only) |
| B7 selftest rollback in flight | fixed |
| B13 / F10 stacks | selftest 3072 -> 4096, heartbeat 2048 -> 3072, once-a-minute high-water log added |
| Sensor #8 partial FIFO drain lost on read error | fixed as a side effect (in my region) |

## 1. W5 / B1 / B5 / B6: no init failure aborts or loops

main.cpp (`note_start`, `app_main`):
- `ESP_ERROR_CHECK` replaced by log-and-continue for data forwarder, command bridge, USB echo, flash logger, heartbeat (all later users are null-safe: `queue_packet` returns INVALID_STATE with no queue, command bridge checks its queue).
- `sensors.start()` is retried up to 3 times (500 ms apart; it releases its resources on failure). Only if all three fail (mutex/SPI bus/sensor task cannot be created, i.e. out of resources) the rest of boot still completes (heartbeat, self-test, CPU monitor), then `esp_restart()` after 10 s (2 s when a flight state was resumed) with an ERROR log. That is the only remaining fatal path, and it prints.
- No new status bit was invented. The existing ready bits (low-g/gyro ready) show IMU absence; radio availability is already reported through `set_radio_available`.

sensors.cpp `Sensors::start`:
- IMU init: up to 3 attempts (attempts 2-3 in "recovery" mode). On failure: log, `imu_ready_=false`, start the task anyway; the task retries every 500 ms. FIFO buffer allocation moved to `ensure_fifo_buffers()` (also retried by the task), so an ENOMEM there is not fatal either.
- Control flow now always reaches the MS5607 and radio init lines (untouched), then ADC, then tasks.
- Battery ADC (`initialize_battery_adc`): calibration failure (no eFuse data) keeps the ADC and falls back to uncalibrated conversion (`raw * 3100 mV / 4095`, approximate, logged WARN); `read_battery_voltage` no longer requires the calibration handle. Unit/channel failures free the unit and the monitor is simply unavailable (WARN). Same idea as maint_radio.cpp, which however returns 0.
- `sensor_log` queue/task creation failure is non-fatal (`defer_log` already drops with a null queue). A retried `start()` does not create a second log task (`deferred_log_task_started_`).

## 2. W16: IMU liveness (new header main/imu_supervisor.h, called from `Sensors::fetch_imu_supervised`)

Thresholds (all >= 25x normal; normal = about 2 samples per 2 ms pass):

| Check | Threshold |
|---|---|
| no FIFO sample delivered | 50 ms since the last sample (evaluated only on a pass that returns 0 samples, so a long loop stall followed by a full FIFO never trips it) |
| constant data | 100 consecutive slots where every present channel is 0x0000 (or every one 0xFFFF) and a low-g part is present |
| SPI read errors | 25 consecutive failed fetches (50 ms) |
| WHO_AM_I | read once a second; a mismatch/error must repeat on an immediate second read |

On a fault (`mark_imu_down`): `imu_ready_=false`, `has_gyro/low_g/high_g_sample_` cleared (so the observation validity, which also requires `lsm_result==OK`, is false), ready flags cleared (status bits drop), half slot dropped, sample clock invalidated, ERROR log with fault name and counters. While down, `fetch_imu_supervised` returns `ESP_ERR_INVALID_STATE` and calls `initialize_lsm6dsv320x(recovery=true)` every 500 ms (first retry immediate). Recovery mode skips the 35 ms boot delay and does the soft reset BEFORE the WHO_AM_I check, so a chip with a wrong ID is still reset. One attempt blocks the loop about 30-40 ms (the reset delay); the IMU delivers nothing then anyway. Counters: faults, reinit ok, reinit failed (logged; members `imu_fault_events_`, `imu_reinit_ok_`, `imu_reinit_failed_`). On a successful re-init the accel filter is re-configured only if the IMU had never come up.
Not done: no CTRL1/FIFO_CTRL4 read-back check (a wrong expected value would cause a re-init every second and I cannot bench-verify it); the no-data check covers a chip that lost its configuration.
Also in `fetch_lsm6dsv320x`: a read error after FIFO words were drained no longer discards them (`read_error` returned at the end; `run()` processes `imu_batch_` whenever it is non-empty).

Tests: tests/imu_supervisor_test.cpp (normal operation 20 s, empty pass, 500 ms stall, exactly 50 ms, constant 99/100, error run 24/25, WHO_AM_I cadence, retry schedule incl. millisecond wrap): all pass.

## 3. W7 / F1 / B3: task watchdog

- `sdkconfig.flight` (and the active `sdkconfig`): `CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_ESP_TASK_WDT_TIMEOUT_S=8` (was 5), `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/CPU1` off (was y). Idle checks are off so core starvation by itself (NAND erase, a busy pass) can never reboot; only subscribed tasks matter. `CONFIG_ESP_TASK_WDT_INIT=y` unchanged. Maintenance image: unchanged (these keys are in the flight overlay only; sdkconfig.defaults and sdkconfig.maint untouched).
- main/task_cores.h: new `task_supervision::subscribe()/feed()/unsubscribe()` inline wrappers.
- `Sensors::run`: subscribes at the top, `feed()` at the top of every pass. `unsubscribe()` / `subscribe()` around `run_landed()` (own loop, disabled by default).
- Reset classification: flight_state_store.cpp `classify()` maps ESP_RST_TASK_WDT (and PANIC, INT_WDT, WDT, SW, BROWNOUT) to Warm, so a TWDT panic resumes Boost/Coast/Descent (read only, no change).
- Why 8 s: normal pass 2-8 ms; the longest legitimate stretch is a radio command chain (each BUSY wait up to 500 ms, a failed `start_transmit` is about 8 commands = up to ~4 s with the present radio code; the radio agent is shortening it); IMU re-init 40 ms; calibration solve milliseconds.
- Long operations and how they are covered: NAND full erase (flash_logger task, NOT subscribed by me: the logging agent must feed inside the erase loop or unsubscribe around it); NVS writes and IMU calibration save (`imu_cal_save` task, not subscribed; the sensors task is never blocked by them except the short cache-off window); OTA / Wi-Fi (maintenance image only: no subscription, no panic config there); calibration capture (runs in the sensors task as short passes, finish_calibration_capture is a few ms); radio waits (above); sensors `run_landed` (unsubscribed).
- NOT DONE in files I do not own: subscribe the flash logger task. Exact lines for the logging agent: in the logger task function, first line `task_supervision::subscribe();` (include "task_cores.h"), and `task_supervision::feed();` once per loop iteration (every 20 ms pass), and `task_supervision::unsubscribe();` immediately before every `vTaskDelete(nullptr)` / task exit (a subscribed task that dies makes the watchdog reboot the MCU: that is wrong for an intentional end, right for a wedge). Erase of up to 1004 blocks must feed between blocks (or unsubscribe/subscribe around it). I did not subscribe `flash_capture`: it blocks on a full queue by design.

## 4. F8: PM_ENABLE / tickless idle

Turned off in `sdkconfig.flight` and the active `sdkconfig` (`CONFIG_PM_ENABLE`, `CONFIG_FREERTOS_USE_TICKLESS_IDLE`). IDF's `esp_pm_lock_create`/`esp_pm_configure` are compiled unconditionally and return ESP_ERR_NOT_SUPPORTED without PM_ENABLE, so `sensors_landed.cpp` still compiles and its code is untouched (see build result). The Landed low-power mode (`kLowPowerModeEnabled=false`) would therefore not sleep if someone enabled it: both keys must be set back to y together with that flag (comment added in the overlay). `CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION` left in place.

## 5. W19: calibration validation (imu_calibration_store.h/.cpp)

`imu_calibration::sanitize()` (inline, host-tested) rejects a section and replaces it by identity (valid flag cleared): gyro bias non-finite or beyond +-30000 mdps; accelerometer section (low-g offset +-500 mg, low-g and high-g scale 0.8..1.25, high-g offset +-6000 mg, all finite). Limits are looser than the capture solver's, so anything the solver accepts is kept. `load()` calls it for both layouts, logs an ERROR with the number of sections rejected and counts them (`rejected_sections()`). Test: tests/imu_calibration_store_test.cpp (good, scale 0, NaN/inf in each field, range edges, both sections bad, default record): all pass.

## 6. B8: NVS erase

`init_storage(bool allow_erase = true)`. main passes `!resumed`. On NO_FREE_PAGES / NEW_VERSION_FOUND: cold boot (pad) reformats exactly as before but the log now states that calibration and boot bookkeeping are lost; a resumed in-flight boot does NOT erase: NVS stays unavailable (every user already tolerates that) until the next cold boot. Not done: preserving calibration across a reformat (NVS cannot be opened when init fails).

## 7. B7: selftest

`flight_selftest::start` now takes the FlightStateMachine. On failure after the 30 s window the rollback reboot happens only in Pad Idle; in any other state it logs ERROR and ends the task, leaving the image pending-verify (a later reset rolls it back). Not changed: a dead radio still fails the self-test on the pad (policy decision, B7 second sentence).

## 8. B13 / F10: stacks and watermark log

- selftest 3072 -> 4096 (heap: +1 KB only while the task lives, <= 30 s on pending-verify boots; logs its own high-water mark when it ends).
- heartbeat 2048 -> 3072 (permanent +1 KB heap): it now logs, once a minute, `stack free: <task> <bytes>` for sensors, sensor_log, heartbeat, selftest (looked up by name, skipped if gone). Net permanent heap cost of this work: 1 KB (free heap was about 35 KB). Sensors stack unchanged (8192); read the logs in a soak before changing anything.

## Build and tests

- Host: imu_supervisor_test, imu_calibration_store_test pass (g++ -std=c++17 -Wall -Wextra).
- Syntax-only compile of main.cpp, sensors.cpp, heartbeat.cpp, flight_selftest.cpp, imu_calibration_store.cpp with the real IDF flags (PM on config): no errors, no warnings.
- Full flight image build (sdkconfig.defaults + sdkconfig.flight, PM off): see the final line below.
- Maintenance image: not rebuilt; its config files are untouched. Note task_cores.h now includes esp_task_wdt.h, which maint_radio.cpp, maint_usb.cpp, wifi_ota.cpp and flash_logger.cpp include transitively; esp_system is in the maintenance component requirements, so it should be fine, but it has not been compiled.

BUILD RESULT: the full flight image build (1152 steps, scratch dir C:\Users\E\AppData\Local\Temp\wf3) was NOT completed: with ten agents building at once it ran at about 8 objects per minute (about 160 done). It may still be running in the background and writes only to that scratch dir and C:\Users\E\AppData\Local\Temp\wf3s\b2.log (its last line says "Project build complete" if it finishes). What was verified instead, after the cmake configure of the final config (sdkconfig.defaults + sdkconfig.flight: PM off, tickless off, WDT panic 8 s; generated sdkconfig.h confirmed): syntax-only compile with the real IDF flags of main.cpp, sensors.cpp, sensors_landed.cpp, heartbeat.cpp, flight_selftest.cpp and imu_calibration_store.cpp: all rc 0, no errors or warnings. Not checked: link, image size, maintenance image. Run build_images.ps1 once on a free machine.

## Trickle-down effects

- New reboot cause: ESP_RST_TASK_WDT in flight if the sensors task misses a feed for 8 s. Warm class: resumes state (RESUMED status bit, "RESET IN FLIGHT" log). A bench pad test of the watchdog: block the sensors task (e.g. temporary infinite loop) and expect a reboot after about 8 s.
- IMU fault/recovery lines (ERROR "IMU FAULT #n", WARN "IMU (re)initialised") are new log text; during a fault the low-g/gyro ready status bits drop and sensor packets are not queued (they already require `lsm_result == OK`).
- Boot with no IMU now reaches radio, GPS, logging and telemetry; ground will see ready bits low. Operators should treat low ready bits on the pad as a no-go (the self-test of a pending image fails on them).
- Uncalibrated battery voltage is approximate (+-10 %) on chips without eFuse calibration.
- Heap: +1 KB permanent (heartbeat), +1 KB transient (selftest). Flash: imu_supervisor and logging add little.
- sdkconfig keys changed (flight overlay and active sdkconfig): CONFIG_ESP_TASK_WDT_PANIC=y, CONFIG_ESP_TASK_WDT_TIMEOUT_S=8, CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 off, CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 off, CONFIG_PM_ENABLE off, CONFIG_FREERTOS_USE_TICKLESS_IDLE off. The dependent PM_* lines in the old active sdkconfig are dropped by kconfig on the next build.
- Landed low-power mode cannot be turned on by the flag alone any more (needs PM keys back on).
- The other agents: radio agent should keep any single sensors-task pass well under 8 s; logging agent must add the three watchdog calls above (and the logger agent's own F2 change must unsubscribe before deleting its task); the USB agent's long commands run on the USB task, which is not subscribed.
- Shared scratchpad warning: other agents also write rep.py/e_*.py in the common scratchpad, which clobbered a file of mine once; my edits were reapplied from a private directory.
