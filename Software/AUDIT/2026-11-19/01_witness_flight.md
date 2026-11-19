# Witness flight firmware pre-flight audit: state machine, position filter, GPS trust ramp, attitude, supervision, BIT

Date: 2026-11-19. Scope: Software/Witness/softwarev1 (ESP32-S3, ESP-IDF 6.1), HEAD a06d14d.
Read-only audit, no source edited, no hardware touched. Findings are marked CONFIRMED (code path traced) or PLAUSIBLE.
Paths below are relative to Software/Witness/softwarev1.

## Summary

No Critical or High finding. I found nothing in the flight path that, by itself, loses the flight data, mis-detects
a normal flight or resets the board. The new GPS trust ramp (commit 0d3a403) is sound for the scenarios reviewed.
There are two Medium and four Low items, all about degraded-mode behaviour (IMU gone for a long time, missed
burnout) or documentation/telemetry accuracy.

## Critical

None.

## High

None.

## Medium

### M1. Position filter is not propagated while the IMU is down, so it freezes for the length of the outage (CONFIRMED code path; needs a long IMU outage to matter)
- main/sensors.cpp:2380-2400 `process_imu_batch()` -> `predict_position()` (2046); main/sensors.cpp:1206 `mark_imu_down()`.
- `predict_position()` is called once per IMU sample, inside `process_imu_batch()` (and the HITL path). When the
  supervisor marks the IMU down, `imu_batch_count_` is 0 and no predict runs until a re-init succeeds (retry every
  0.5 s, ~30-40 ms each). `service_position()` keeps feeding `update_baro()`, `update_gps()` and
  `update_constraints()` to a filter whose state is not advancing and whose covariance is not growing.
- Short outages are fine: the first predict afterwards has dt > 0.1 s and is bridged (position advanced, covariance
  grown, up to 10 s). It matters only if the LSM6 stays dead (re-init keeps failing) for many seconds in flight:
  Kalman AGL and vertical velocity stay at their last values (small P, so baro and GPS barely move them); the trust
  clock `clock_s_` (advanced only by predict) also stops, so `fix_dt` is 0 and every GPS correction is limited to
  the 1 m slew floor. The state machine then sees frozen AGL/vz: Descent can be late or missed, and Landed can never
  be declared (it also needs a low-g sample). Binary logging stays on in Boost/Coast/Descent, so data is not lost,
  but the telemetry state would be wrong for the rest of the flight.
- Fix: in `service_position()` (runs every pass), if no predict happened for more than ~100 ms outside HITL, call
  `position_filter_.predict_without_imu(dt, phase)` with the wall-clock dt and update `position_last_predict_us_`.
  That keeps the clock, covariance and gap bridging honest and lets baro/GPS carry the estimate.

### M2. Boost can only leave through burnout detection on the low-g axial sensor; no fallback (PLAUSIBLE)
- main/flight_state_machine.cpp:327-338 (Boost case); the Coast logic (340-380) is only reachable from Coast.
- Boost -> Coast needs `low_g_axial_acceleration_valid && low_g_axial < 1 g` continuously for 500 ms. If that is
  never seen (low-g sample invalid because the IMU was re-initialising at burnout, or a sensor/axis problem), the
  state stays Boost until the 30 s landing test fires, and Coast -> Descent never runs. Side effect:
  main/position_filter.cpp `update_baro()` keeps the barometer gated for the whole of Boost/Coast once it tripped the
  Mach cutoff, so with the state stuck in Boost the barometer stays out for the entire descent (IMU drift and GPS
  only).
- Trigger is narrow (burnout coincides with an IMU fault or a bad axial reading), so PLAUSIBLE, not likely.
  Cheap hardening: in Boost, after `climb_seen_`, also move to Coast when Kalman vertical velocity is below 0 for
  1-2 s (to Coast only, so the existing Coast logic takes over).

## Low

### L1. The trust ramp is not warm at launch, contrary to its comment (CONFIRMED)
- main/position_filter.cpp:632 comment "Trust weights are tracked in every phase, so they are warm at launch" vs the
  `if (hold) return false;` just above it, which exits before `on_fix()`.
- On the pad with the vehicle still, no `on_fix()` runs. The first fix after the filter leaves pad hold restarts the
  weight at 0.05 and needs ~3 s of velocity-consistent windows to reach 1. Meanwhile large GPS innovations
  (> 5 sigma, sigma capped at 100 m) are rejected and normal ones down-weighted 20x. Conservative, not dangerous:
  the IMU and the subsonic barometer carry the first seconds, and hard boost acceleration keeps trust low anyway.
- Fix (optional): call `gps_trust_[i].on_fix()` in the hold branch too (filter velocity ~0 and a steady GPS ramp it
  to 1 on the pad), or correct the comment.

### L2. Persisted maximum AGL is only written at state changes (CONFIRMED, benign)
- main/flight_state_machine.cpp `persist()` is called from `set_state()` only; max AGL tracking is in RAM.
- After a warm reset in Coast the restored maximum is the AGL at burnout, not the real one. The first update with AGL
  above the stale maximum raises it, and the velocity path has `climb_seen_` restored, so Descent still works. A reset
  after apogee with a slow drogue descent (> -30 m/s) may declare Descent late. Telemetry only.
- Fix (optional): persist the maximum when it grows by more than 5 % (a 2 Hz RTC write like `save_references`).

### L3. BIT "state record matches" is hardwired true (CONFIRMED)
- main/bit_monitor.cpp:194 `in.state_record_matches = true;` (consumed at main/bit_logic.h:444).
- The check can never report a mismatch between the RTC state record and the live state. Info class, no flight
  effect, but a green check proves nothing. Fix: compare the store record with `fsm.state()` or drop the item.

### L4. Restored filter converges slowly on horizontal position after a warm reset (CONFIRMED by simulation, acceptable)
- main/sensors.cpp:1962-2000 `apply_resume_references()`, position_filter.cpp `restore_after_reset()`, gps_trust.h.
- Host simulation (Coast at 250 m/s, 1 km horizontal offset, reset with 0.5 s stale references, baro/GPS only, no
  attitude): vertical error stays within about 20 m; horizontal error takes ~10 s to pull in (trust restarts at
  0.05, slew 100 m/s). Horizontal state is only used for log and telemetry; phase logic uses vertical only. No
  action needed; useful when reading the log of a resumed flight.

## Checked and found OK

- Flight state machine (flight_state_machine.cpp/.h): one-way CAS in `set_state()`, so USB and sensor task cannot
  move the state down; Pad Idle rules (Kalman AGL+vz 300 ms, 2.5 g leaky budget, baro-rate airborne-at-boot
  up/down) are independent and each keeps its own dwell; accel budget caps a stall at 50 ms; dwell unsigned
  subtraction is safe because timestamps only go forward outside HITL (rebaseline covers it). Boost -> Coast,
  Coast -> Descent (AGL path 5 %/10 m drop, velocity path with `climb_seen_` and 3 s minimum coast) and landing
  (30 s, 0.8-1.2 g, gyro < 10 dps, AGL span <= 10 m) traced; no path to Landed under a descending parachute (AGL
  span rule) or at apogee (low-g magnitude rule). Observation validity flags are cleared by `mark_imu_down()`, so
  stale IMU data cannot satisfy a condition.
- Resume after warm reset (flight_state_store.*, main.cpp): Warm class covers panic, int/task/RTC WDT, SW restart,
  brownout; records sealed with magic and checksum, a torn write is invalid not half-valid; Landed deliberately not
  resumed; a second warm reset still resumes; cold boot clears both records; NVS not erased on a resumed boot;
  `restore()` sets `climb_seen_`, `flight_inferred_` and re-arms timers; `restore_after_reset()` rebuilds baro
  reference, GPS origin, gravity and AGL/vz with a wide covariance; `apply_resume_references()` runs before the
  first predict. Attitude correctly not resumed (magnitude-only predict).
- GPS trust ramp (gps_trust.h, update_gps flight branch): no divide by zero (weight floor 0.02), NaN sigma handled,
  time reversal and gap restart handled, weight cap 0.1 below 6 satellites, slew limit formula gives a correction of
  `max_step` for large innovations, `update_axis` refuses non-finite or non-positive S, baro-first rule on the up
  axis preserved, Pad/Landed keep the old gated path, `reset()` rebuilds the trust state. Simulated restored-in-flight
  filter with 25 Hz GPS: converges, no divergence.
- Position filter: finite-input rejection, `check_health` -> `recover_from_divergence()`, covariance
  symmetrisation and clamping, gap bridging (dt > 0.1 s, cap 10 s), Mach-gated baro with resume rules, baro scale
  state.
- Attitude estimator: non-finite sample drop, gyro saturation -> lost (position filter switches to no-IMU predict),
  divergence -> invalid, no initialisation in flight, lost cleared only on the pad.
- IMU supervisor (no data 50 ms, constant 0x00/0xFF, SPI errors, WHO_AM_I): marks the IMU down, clears the sticky
  has_* flags, re-inits every 0.5 s; the observation uses those flags.
- BIT: verdict is informational (LED/buzzer/telemetry), never gates automatic transitions or logging; active tests
  run only in Pad Idle/Landed, never when resumed, and abort on state change, non-zero launch budget or > 1.5 g; no
  BIT radio summary in Boost except on change. (Iris BIT is out of scope by owner decision.)
- Build config: task WDT 8 s with panic (warm, resumes), idle-task WDT off, brownout detection on, panic reboot
  delay 0, RTC_NOINIT records.

## Host tests (g++ -std=c++20, own temp directory)

All passed: gps_trust_test, position_filter_test, attitude_estimator_test, flight_state_machine_test,
imu_supervisor_test, landed_logic_test, bit_logic_test, bit_policy_test. Plus an ad-hoc restore-in-flight simulation
(results under L4). kalman_replay.py and kalman_mc.py were not re-run (the owner reported them with commit 0d3a403).

## Not covered here

Flash logger/NAND, radio, command handling, Wi-Fi/OTA, boot manager and the 2 Hz telemetry change belong to other
audit areas.
