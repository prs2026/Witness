# Fixes 04: flight state machine, position filter, attitude, flight-state store

Date 2026-11-15. Scope: audit 05 (FSM), Kalman audit 2026-11-15, full-audit W3, W4, W12, W18. Nothing committed,
flashed or run on hardware. Line endings preserved per file (some files are CRLF, some LF).

## Summary table

| # | Finding | Status |
|---|---|---|
| 1a | Filter runs without an attitude | fixed |
| 1b | Second launch trigger from axial acceleration | fixed |
| 2a | Warm reset in flight: AGL continuous | fixed (attitude not resumed, by design) |
| 2b | Cold reset in flight: airborne-at-boot inference | fixed, ascent and fast descent |
| 3 | NaN / divergence recovery | fixed |
| 4 | Predict gap > 100 ms | fixed |
| 5 | `persist()` from two tasks | fixed (critical section) |
| 6 | False Boost on the pad | analysed; auto-reset implemented behind a switch, wiring needed outside my files |
| 7 | Pre-launch ring vs detection delay | analysed, numbers below |

## 1a. Filter without an attitude (W4)

- `position_filter.*`: new `Filter::predict_magnitude_only(|f|, dt, phase)`. Stillness (`still_`) and the pad
  gravity measurement only need |specific force|, so the pad hold (position/velocity pinned, baro reference and GPS
  origin tracking) works exactly as with an attitude. Not still, or in flight: zero acceleration with
  `no_imu_accel_noise` (10 m/s^2), baro and GPS carry the estimate (as `predict_without_imu` already did after a gyro
  saturation). It also sets `imu_ignored_`, so the existing Kalman-log flag 0x02 now also means "no attitude".
- `sensors.cpp: predict_position`: when `attitude_.lost() || !attitude_.valid()` the filter is predicted with
  `predict_without_imu` (lost) or `predict_magnitude_only` (never valid / diverged). `position_last_predict_us_` is
  no longer zeroed while the attitude is invalid, so when the attitude becomes valid the IMU `predict()` takes over
  with the state unchanged (host-tested: no jump, no reset). `service_position` and the FSM observation now see an
  initialised filter from the first IMU sample.
- Host results (no attitude, 960 Hz |f| predict, 25 Hz baro 0.6 m): pad hold pins AGL < 0.3 m, vz < 0.1 m/s, gravity
  measured; a 3 g net launch makes "AGL > 30 m and vz >= 30 m/s" true at t = 1.44 s after liftoff (truth: AGL 30 m at
  1.43 s), +0.3 s qualification. With a deliberately bad 2 m baro and |f| noise of 25 % (never still) for 120 s the
  filter reaches at most 16.5 m/s and 5.9 m: it cannot fake the 30 m AGL + 30 m/s rule.
- Limitation: without the IMU there is no vertical acceleration, so in supersonic Boost/Coast (baro gated) the
  estimate coasts on the last velocity plus GPS; it is a fallback, not a replacement.

## 1b. Acceleration launch trigger

`flight_state_machine.*: acceleration_launch_detected`. Pad Idle -> Boost when `observation.axial_acceleration_g`
(filtered, body +X) >= **2.5 g** for **300 ms**, as a leaky budget (fills at real time, drains 2x faster below the
threshold, credit per observation capped at 50 ms).

Why these values: 2.5 g for 0.3 s along the nose axis is >= 4.4 m/s of delta-v above 1 g, continuously. Handling
shocks are short and spiky (tests: 30 g for 10 ms every 100 ms for 60 s never fires; 3 g peaks 40 % of the time
never fires; a 250 ms 4 g shove never fires; a 50 ms+ gap in observations credits only 50 ms). A drop reads 0 g, a
toss or a car brakes below 2.5 g or far shorter than 0.3 s. A real motor (TWR >= 2.5, i.e. net >= 1.5 g) reads
TWR g at liftoff and fires 0.3 s after ignition; a chuffing motor (dips every 50 ms) still qualifies. I chose 2.5 not
3.0 because at TWR 3 (net 2 g) the axial reading is 3.0 g and sits on the threshold. I did not add a barometer
contradiction check: in 0.3 s a 3 g impulse moves the vehicle ~1.4 m, below the baro noise, so the barometer cannot
confirm or deny; it can only add false negatives. Weaker motors (TWR < 2.5) rely on the Kalman rule, which now works
without an attitude.

Boost = (Kalman AGL > 30 m and vz >= 30 m/s for 300 ms) OR (axial rule) OR (airborne at boot, see 2b).

Mirror: `HITL/sim_model.py ReferenceStateMachine` (constants `LAUNCH_ACCEL_*`, `update(..., axial_g, baro_rate_mps)`),
`HITL/tests/test_trajectory_csv.py launch_expected_s` (now the earlier of the two rules),
`test_sim_model.py` (existing Kalman-rule tests pass `axial_g=1.0` to isolate it; new tests for the acceleration
rule, airborne rule, false-flight evidence). `fake_witness.py` sets `ACCEL_MAX_STEP_US` large because the
time-scaled end-to-end test feeds sparse frames (the firmware observes every few ms, so the 50 ms cap is right there).
In HITL runs Boost is now declared about 0.3 s after the thrust starts, 1-2 s earlier than before.

## 2a. Warm reset in flight

- `flight_state_store.*`: a second RTC_NOINIT record (`Extension`, 12 x 32-bit words, own magic "WFXR" and checksum,
  no padding) with baro pad reference, GPS origin lat/lon/alt, pad gravity, last AGL and vz, each with a valid flag.
  The original `Record` (20 bytes, its checksum, its validation) is unchanged and asserted by a test, so records
  from older firmware and garbage still validate exactly as before; an older record simply has no extension and the
  resume proceeds as before (AGL restarts at 0). `save_references()` / `resume_references()`; the references are
  copied at boot and cleared on every non-resumed boot.
- `sensors.cpp: persist_position_references` (from `service_position`, 2 Hz, only once the filter is initialised,
  RTC RAM only) and `apply_resume_references` (first `predict_position`): `Filter::restore_after_reset` sets the
  baro reference, GPS origin, pad gravity (frozen) and the up position/velocity from the persisted AGL/vz, with
  covariance 500 m / 100 m/s on all axes so baro/GPS pull the state in. AGL is therefore relative to the pad, and
  Coast/Descent/Landed logic works after a reset. Host test: restored with 60 m / 5 m/s stale error, AGL error
  < 0.3 m after 2 s.
- The attitude is **not** resumed (it can only initialise on a still pad). The rest of the flight runs without it
  (1a). Heading alignment and the attitude telemetry stay invalid after such a reset.

## 2b. Cold reset in flight (RTC lost)

- New `position::BaroRate`: raw barometer altitude rate (mean of the newest 3 of ten 100 ms bins minus the oldest 3,
  0.7 s apart; noise ~0.4 m/s), independent of the Kalman filter. Needed because under a parachute |f| is about 1 g,
  so the pad hold would pin the filter to "still" and hide the descent. Fed from `service_position`, passed in
  `Observation.baro_rate_valid/_mps`.
- FSM, Pad Idle only: rate >= +20 m/s for 1 s -> Boost (the existing burnout rule then moves to Coast); rate <=
  -20 m/s for 1 s -> Descent. Total delay ~1.7 s. Both mark the flight as real (`flight_inferred_`).
- False-trigger analysis: 20 m/s for a second is 20 m, ~240 Pa; weather, door slams, wind on the port (a few metres,
  sub-second), a lift (< 10 m/s) and a car do not do that. An altitude chamber run faster than 20 m/s would. A main
  parachute descent slower than 20 m/s is not recognised (drogue is). Above Mach 0.8 the raw baro is wrong, so a
  cold reset in supersonic Coast may be classed by the wrong sign or late; either state starts logging and 1 Hz telemetry.
  A reset during the burn is caught by the acceleration rule instead. No GPS confirmation: the Observation carries
  no GPS vertical speed and the filter's GPS fusion is hidden by the pad hold.
- AGL after such a boot is relative to where the board restarted (unchanged limitation); Landed still works
  (relative span).

## 3. NaN / divergence recovery (W18, FSM #5)

- `Filter`: `predict` / `predict_without_imu` / `predict_magnitude_only` reject non-finite acceleration, |f| and dt
  before use; `update_baro` rejects non-finite altitude/pressure; `update_gps` rejects non-finite or out-of-range
  lat/lon/alt; `update_axis` refuses a non-finite innovation or variance (a NaN innovation used to pass the gate
  and poison the axis). All counted in `rejected_inputs()`.
- `check_health()` after every predict and update cycle: any non-finite state/covariance, or |p| > 1e7 m, |v| > 1e5
  m/s, P > 1e15, resets all axes to p = v = 0 with P = 500 m / 100 m/s variance, keeping baro reference, GPS origin,
  gravity and counters (`divergence_resets()`). `Sensors` logs the three counters (and the attitude counter) once per
  change at most every 0.5 s.
- `attitude_estimator.*`: non-finite gyro/accel samples are dropped like a gap (`nonfinite_events()`); a non-finite
  integrated quaternion (tested before `normalized()`, which would silently map NaN to the identity) invalidates
  the attitude, so 1a takes over; on the pad a new stationary window re-initialises it.
- Host tests with injected NaN (all four filter predicts, baro, GPS, 3e38 acceleration, NaN/overflowing gyro).

## 4. Predict gap (Kalman audit #5, FSM #10)

`predict` and `predict_without_imu` with dt > 0.1 s now call `propagate_without_acceleration` (position advances
with the current velocity, covariance grows with the no-IMU noise), capped at 10 s of bridging, counted in
`predict_gaps()`. Test: 0.5 s gap at 50 m/s moves 25 m and P00 grows. `attitude::Estimator` still treats > 50 ms as a gap
(unchanged).

## 5. `persist()` thread safety (FSM #13)

`flight_state_store.cpp`: both records are written inside a `portMUX` critical section (microseconds, RAM only).
Torn-record rejection by checksum is unchanged and also tested for the extension (every single-word corruption is
rejected).

## 6. False Boost on the pad (W12): analysis and what is implemented

Likelihood with the new rules:
- Kalman rule (AGL > 30 m and vz >= 30 m/s): now also runs without an attitude. It needs 30 m/s sustained; with 2 m
  baro noise and handling-like |f| the filter peaked at 16 m/s. A drifting filter needs a dead barometer plus a
  >1.05 g offset for a minute (see audit 05 #4); unchanged and low.
- Acceleration rule: 2.5 g continuously for 0.3 s along the nose. I find no handling, transport or drop scenario that
  does it. A deliberate fast upward shove or an accelerating vehicle (launch catapult, car at 2.5 g = a crash) would.
- Airborne rule: needs 20 m/s of baro rate for 1 s: an altitude-chamber or unpressurised-aircraft test.
- Commanded launch (USB `trigger_launch`) and HITL: deliberate.
Overall: low, but a bench or HITL Boost is common, and the consequence (Landed after ~31 s, logging off, slot used)
is large. The measured new risk is mostly from the new rules being armed in more situations (bench tests with the
board on a shaker or in a chamber).

Implemented (switch `FlightStateMachine::kFalseFlightRestartEnabled = true`): a flight is "real" if it was resumed,
inferred airborne at boot, reached AGL >= 50 m, or read axial acceleration >= 3 g in Boost/Coast/Descent. If the
automatic landing test declares Landed without that evidence, the FSM first declares and persists Landed (so the
reset is not resumed), then calls a callback registered with `set_false_flight_callback(void(*)())`. **The callback
is not registered anywhere**, so behaviour is unchanged until it is wired: in `usb_serial_echo.cpp` (or main.cpp),
register a function that schedules the same soft reset to Pad Idle that the 0x7300-in-Landed path performs (ack not
needed; restart after the same delay). Tested with: no evidence -> callback once; AGL 60 m, 3.5 g, resumed -> no
callback; no callback registered -> Landed only. Effects to know: a bench "commanded Boost then wait 31 s" test will
reboot the board once wired; the false flight still consumed a log slot and returns at -17 dBm (W1); the real launch
is only safe once the reboot (~1-2 s plus a 1 s attitude window, not needed any more for detection) has finished.
Not implemented (alternatives): refuse Landed without evidence (keeps logging, but a short real hop would never
land); require time since Boost.

## 7. Pre-launch ring (FSM #7)

Detection delay after liftoff for net acceleration a: Kalman rule max(sqrt(60/a), 30/a) + 0.3 s plus filter lag
(measured +0.01 s): 3 g net 1.74 s, **2 g net 2.05 s**, **1 g net 3.4 s**. The acceleration rule fires 0.3 s after
the axial reading reaches 2.5 g: 2 g net (TWR 3, reads 3 g) 0.3 s; 1 g net (TWR 2, reads 2 g) does not use it.
Conclusion: for TWR >= 2.5 the 2 s ring is ample (needs ~0.6 s). For TWR < 2.5 the Kalman rule is 3.4 s at 1 g
net, so the ring would need ~4 s to hold the liftoff; the logging owner should decide whether to grow it (or lower
`kLaunchAccelerationG` to ~1.8 g, at more false-trigger risk). I did not change the ring.

## Tests and builds

- Host (g++ 16.2, -Wall -Wextra, no warnings): position_filter_test, flight_state_machine_test,
  attitude_estimator_test, heading_align_test all pass; new tests: no-attitude pad/launch/noise, smooth switch, NaN
  recovery, gap bridge, restore after reset, BaroRate (filter); acceleration launch + handling rejection, airborne,
  false-flight callback, store extension record (FSM); NaN/overflow (attitude). `kalman_replay.cpp` compiles
  against the changed header (link needs its other sources, not checked).
- HITL python: `python -m unittest discover -s tests` = 94 tests (91 + 3 new), 93 pass; the 1 error is
  `test_gps_rf.Report` (no `matplotlib` module installed in this Python; unrelated to these changes). The end-to-end
  headless test passes again after the fake-witness cap change.
- Firmware: full flight build was started in C:\Users\E\AppData\Local\Temp\wf4 and failed in code I do not own
  (`uart_driver.cpp:472: no matching function for call to min(const uint32_t&, unsigned int)`). Per the change of
  plan I then did a `-fsyntax-only` compile with the exact ESP-IDF flags of `sensors.cpp`,
  `flight_state_store.cpp`, `flight_state_machine.cpp`, `position_filter.cpp`, `attitude_estimator.cpp`: all clean.
  A full link of the image has not been done.

## Trickle-down effects

- New Boost routes: acceleration rule (typically 0.3 s after ignition), airborne-at-boot (Boost), and a new way to
  reach Descent directly from Pad Idle. Log lines: "launch detected: axial acceleration", "airborne at boot".
- The filter is initialised and AGL/vz are valid from the first IMU sample, also with no attitude. The Kalman log
  flag IMU_IGNORED (0x02) now also appears on the pad until the attitude initialises; the 1 Hz position report says
  "no attitude (baro/GPS, |f| only)". Horizon's converter treats the flag as a bit, no change required, but a viewer
  that reads "IMU ignored" as "attitude lost in flight" should be told.
- After a warm reset in flight: state, AGL, vz, gravity, pad references resume; attitude, heading alignment and the
  quaternion in telemetry stay invalid (zero) for the rest of the flight. After a cold reset: Pad Idle until detected
  (about 1.7 s from a fast climb/descent, 0.3 s from a burn); AGL is then relative to the restart point.
- Operators: Boost can now be declared by thrust alone; a board on a shaker or in an altitude chamber can reach
  Boost/Descent. Once the false-flight callback is wired, a flight without 50 m AGL or 3 g reboots after Landed.
- `FIRMWARE_STATE_MACHINE.md`, `FIRMWARE_HANDOFF.md` and the HITL README still describe the single Kalman launch
  rule and the "attitude required" limitation; I did not edit the docs. They should list the three Boost routes, the
  persisted references and the new constants.
- `flash_logger`: the pre-launch ring question (item 7) and that Boost can now come from a Pad Idle board whose
  Kalman AGL is still near 0 (max AGL at Boost entry is the pad value). Landing evidence uses the FSM's own maximum.
- Not done / needed elsewhere: wiring `set_false_flight_callback` (usb_serial_echo.cpp/main.cpp); `uart_driver.cpp:472`
  build error belongs to its owner.
