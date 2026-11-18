# Audit 05: flight state machine, persistence, estimators

Read-only. Version reviewed: NEW rule (working tree as of 16:03 on 2026-11-15: one-way states, `usable_for_navigation`, pad-measured gravity, Boost = AGL > 30 m AND vz >= 30 m/s for 300 ms). Files read in full: flight_state_machine.*, flight_state_store.*, position_filter.*, attitude_estimator.*, heading_align.cpp, command_policy.h, witness_status.h, the observation/restore/predict code in sensors.cpp, the state-dependent parts of flash_logger.cpp and landed_logic.h. Findings marked "OLD" also exist in the committed b2f6610 version.

Line numbers refer to the working tree at 16:03.

## Findings table

| # | Sev | Conf | Finding | Where | Rule |
|---|-----|------|---------|-------|------|
| 1 | **Critical** | confirmed | Launch detection has a single path: stationary window -> attitude valid -> filter initialised -> AGL and vz. If any link is missing, Boost is never declared and there is no fallback. Pad Idle then records only a 2 s RAM ring, and the session is never flagged as a flight | flight_state_machine.cpp:263-275; sensors.cpp:1604-1607, 2432-2439; attitude_estimator.cpp:179-184 | OLD + NEW |
| 2 | **Critical** | confirmed | A cold reset (power-on, EN) in flight boots Pad Idle with no attitude. The filter cannot start in flight, so the rest of the flight is never detected. The 2026-11-15 "partly mitigated" note in FIRMWARE_STATE_MACHINE.md (H3) is wrong | same as 1; flight_state_store.cpp:15-28 | OLD + NEW |
| 3 | **High** | confirmed | A warm reset in Boost, Coast or Descent resumes the state, but the attitude (and so the filter) can never start in flight. AGL and vz are never valid, so Coast and Descent never end, Landed is never reached, and 0x7300, export and erase stay refused until a power cycle | sensors.cpp:1604-1607; attitude_estimator.cpp:179; flight_state_machine.cpp:58-71, 255-258, 296-331 | OLD + NEW |
| 4 | **High** | confirmed (logic), plausible (trigger) | A false Boost on the pad (or any commanded launch or HITL run) now cannot be undone. It runs Boost -> Coast -> Landed about 31 s later. In Landed the NAND IMU and binary logs are off. A real launch after that is not logged on board. Every false flight also marks a log slot, and two marked slots mean no logging at all | flight_state_machine.cpp:36-40, 129-188; flight_state_machine.h:15; flash_logger.cpp:39-43, 345 | NEW (one-way) |
| 5 | **High** | plausible, needs test | No NaN or divergence recovery anywhere in the filter. One NaN accelerometer sample makes AGL invalid for good (filter reset only on HITL source change). In Pad Idle that is a missed launch; in flight, Landed is unreachable | position_filter.cpp:55-96, 209-249; sensors.cpp:497, 1617-1629 | OLD + NEW |
| 6 | Medium | confirmed | The only gate that makes the attitude (and filter) valid is "every raw sample |a| in 0.95..1.05 g for 1 s". An uncalibrated or hot accelerometer, constant wind buffeting, or radio glitches can keep it from ever becoming valid on the pad. Nothing in telemetry flags "attitude not valid" other than a zero quaternion | attitude_estimator.h:77-78, .cpp:179-185 | OLD + NEW |
| 7 | Medium | confirmed | Boost is declared >= about 2.05 s after liftoff at 3 g (about 3.4 s at 2 g, later for slow motors). The pre-launch ring is 2 s, so the start of the burn is lost | flight_state_machine.h:44-46; flash_logger.h:57 | NEW |
| 8 | Medium | plausible | False Landed under a canopy is now permanent (frozen or stale barometer with an IMU reading 1 g gives an "AGL stable" window). Landed ends NAND logging for the rest of the flight. Landing and Coast->Descent depend only on the filter's AGL | flight_state_machine.cpp:129-188 | OLD (M6), worse under NEW |
| 9 | Medium | confirmed | HITL and bench runs leave a persisted flight state (Descent is resumable by a warm reset). Radio cannot clear Boost, Coast or Descent. A forgotten HITL leaves the vehicle in a non-Pad state on launch day, with Boost never declared | command_policy.h:81-95; usb_serial_echo.cpp:1073-1103 | OLD (M1), worse under NEW |
| 10 | Medium | confirmed | Predict gap > 100 ms skips the propagation and covariance growth entirely (Kalman audit #5, still present) | position_filter.cpp:60-62 | OLD + NEW |
| 11 | Low | confirmed | Persisted maximum is only saved at state change (Boost: the value at entry, Coast: at burnout, Descent: 95 % of the apogee). After a resume it is stale, and with no filter it never updates | flight_state_machine.cpp:50-56; flight_state_machine.h:106 | OLD + NEW |
| 12 | Low | confirmed | Boost has no timeout and no fallback if the low-g axial channel is invalid or stuck: Coast is only entered from a valid low-g axial reading < 1 g for 500 ms (M4, OLD) | flight_state_machine.cpp:277-288 | OLD + NEW |
| 13 | Low | plausible | `persist()` runs from the sensor task and the USB task with no lock; an interleaved `seal()` can leave a record with a bad checksum, which is rejected after a reset (the flight is then not resumed). Window is a few instructions | flight_state_store.h:35-44 | OLD + NEW |
| 14 | Low | confirmed | The persisted record is lost on a true power loss (RTC memory), so the cold path in #2 is the only path for a supply dip | flight_state_store.cpp:13-28 | OLD + NEW |

Earlier findings in this facet that are still present: FIRMWARE_STATE_MACHINE.md H3 (cold reset in flight, mitigation claim is wrong: see #2), M1 (HITL leaves a simulated state, #9), M4 (no Boost fallback, #12), M6 (false Landed, #8), L3 (stale persisted maximum, #11). Kalman audit 2026-11-15: #5 (predict gap, #10) and #6, #7, #8, #9 not re-examined. Kalman #1 (return to Pad Idle keeps the old position) is resolved by the one-way design plus restart (0x7300 restarts, the filter is fresh), but anything that re-enters Pad Idle without a restart does not exist now, so OK. Kalman #2 (GPS quality) and #3 (gravity) are addressed in the working tree. Kalman #4 (Boost on AGL only) is addressed by the vz condition, with the false-trigger analysis in #4 below.

## Details

### 1. Single-path launch detection (Critical)

In Pad Idle the state machine looks only at the filter's `up().p` and `up().v` (flight_state_machine.cpp:263-275). The Observation carries axial acceleration (`axial_acceleration_g`, `low_g_axial_acceleration_g`) but Pad Idle never looks at it. The observation is filled with `agl_valid` and `vertical_velocity_valid` only when `position_filter_.initialized()` (sensors.cpp:2432-2439), and `initialized_` is set only by the first `Filter::predict` (position_filter.cpp:58), which `predict_position` calls only when `attitude_.valid()` (sensors.cpp:1604-1620). The attitude becomes valid only in `finish_window`, which needs `phase == PadIdle` and a window of >= 1 s (>= 50 samples) in which EVERY sample has |a| within 0.95-1.05 g, accel SD <= 0.02 g, gyro MAD sigma <= 10 dps and bias < 5 dps (attitude_estimator.cpp:179-185, 256-343).

Ways Boost is never declared:
- The board sits on the pad but no clean window ever occurs (accelerometer reading outside +/-5 % of 1 g because of offset, temperature or missing calibration; sustained shaking; gyro interference). One out-of-band sample resets the window (`reset_window`). There is no fallback.
- The board reboots in Pad Idle (cold or warm, record = Pad Idle) shortly before or during the launch window: it needs another quiet second that never comes in flight (see #2).
- NaN poisons the filter (#5).
- The 2 s pad window is the only time the attitude can be initialised; the radio's 3 s TX cadence leaves 1 s gaps, which is enough on the bench but is the one margin.

What is recorded in Pad Idle (confirmed by reading flash_logger.cpp:612-650, 905-947 and flight_state_machine.h:11-15):
- 50 Hz binary log and 960 Hz IMU log: only the newest 2 s in RAM (`prelaunch_records_`, `imu_prelaunch_`), dumped only when the state becomes Boost/Coast/Descent. In a missed launch nothing from the flight ever reaches NAND.
- `flight_marker_pending_` is set only at that dump, so the session is never marked as a flight. `scan_sessions` then alternates between unmarked slots (flash_logger.cpp:345-362): two boots later this session, including its text/UART/packet rings that were wrapping (and overwriting their own oldest data) in Pad Idle, is erased.
- Telemetry: `landed::radio_period_ms` makes the locally generated sensors and state packets 3 s apart in Pad Idle (landed_logic.h:34-40). The state packet says Pad Idle all flight.
So a missed launch is: no on-board high-rate data, 3 s telemetry, and the leftovers are overwritten by the next boots. That is a lost flight, not a degraded one.

Compared with the old design, the 10 g accel trigger needed no attitude and no filter. The AGL trigger (OLD) and AGL + vz trigger (NEW) both depend on the chain above.

Suggested: add an attitude/filter-independent launch detector in Pad Idle (for example the filtered X accel > 2.5-3 g for 200-300 ms, or high-g channel > 4 g, as an OR with the Kalman rule), and make the Pad Idle logging path keep 50 Hz/960 Hz in the RAM ring only until either detector fires. Also show "attitude valid / filter initialised" in the status byte or state packet, so an operator can see it before arming (quaternion zero is the only cue today).

### 2. Cold reset in flight (Critical)

`classify()` treats power-on, EN pin, USB reset and unknown as Cold (flight_state_store.cpp:15-28), and `resumable` needs Warm (flight_state_store.h:60-64). On a Cold reset the record is sealed as Pad Idle and the board boots Pad Idle.

FIRMWARE_STATE_MACHINE.md H3 (status 2026-11-15) claims: after a cold reset "the filter's barometer reference is taken at the first reading, so AGL starts at 0 where the reset happened. A vehicle still climbing re-detects Boost about 30 m later". This is not what the code does. `Filter::update_baro` is reached only after `position_filter_.initialized()` (sensors.cpp:1753-1755), which needs a valid attitude, which needs a stationary 1 s window that does not exist in flight (accelerating, tumbling or falling, or at best |a| = 0 in coast). So after a cold reset in flight the filter never runs until the board lands and sits still for a second. Boost, Coast, Descent and Landed are never declared; logging stays in the 2 s ring; telemetry is the 3 s Pad Idle rate. The data from before the reset is kept (that session carried the flight marker). Everything after is lost. When the vehicle lands, the attitude initialises, the filter starts at AGL 0 on the ground, state stays Pad Idle.

A supply dip is plausible (RTC memory is also lost on a true power loss, #14, so even a persistent record would not help there).

Suggested: a Pad Idle-state "late join" path that does not need the attitude (a raw barometer-altitude slope or accel-magnitude/free-fall trigger from the low-g and high-g channels), or keep the full radio rate in Pad Idle until the vehicle has been still for N minutes since boot.

### 3. Warm resume in flight has no estimator (High)

After a Warm reset in Boost/Coast/Descent, `restore()` sets the state and `climb_seen_ = true` (flight_state_machine.cpp:58-71). `attitude_.update` with a flight phase never accepts a window (`accel_ok` needs `pad`, attitude_estimator.cpp:179-185), so `valid_` stays false, `predict_position` returns at sensors.cpp:1604-1607 and `position_filter_.initialized()` stays false. Hence `agl_valid` and `vertical_velocity_valid` stay false for the whole flight:
- Boost: burnout works (low-g only), so Boost -> Coast works.
- Coast: both Descent paths need AGL or vz (flight_state_machine.cpp:296-331); never taken.
- Descent and Coast: `landing_detected` needs `agl_valid` (line 146); never taken.
- The state sits in Coast or Descent until a power cycle. Landed-only log maintenance (`allow_log_maintenance`), `0x7300` (needs Landed) and USB export are refused in Boost/Coast/Descent (command_policy.h:35-40, 81-95); radio cannot set Landed. So the operator cannot export or erase over radio, and over USB only by the 0x7304 route.
- Logging in Coast stays at full rate (binary and IMU, not decimated, since the decimation is Descent only) until the NAND is full. The "ring full since launch; stopped to keep the flight" stop protects what was written.
- The only way a filter can start is `attitude_.lost()` on gyro saturation (chute shock, sensors.cpp:1592-1602), which starts `predict_without_imu` with a baro reference set at that moment, so AGL is relative to wherever the board is then (position_filter.cpp:282-286). That makes Landed reachable by accident only.

Suggested: for a resumed state, start the filter without an attitude (use `predict_without_imu` with the first baro reading as the reference), or let Landed detection use accel/gyro and a raw barometer span independent of the filter in the resumed states.

### 4. False Boost on the pad (High, consequence), likelihood with the new rule

Likelihood. New rule requires AGL > 30 m AND vz >= 30 m/s for 300 ms in Pad Idle (flight_state_machine.cpp:264-270). Handling, a hoist, a vehicle or a stale offset cannot produce 30 m/s. What can: (a) the filter diverging while still_ is false and the barometer unusable: if the board's |specific force| sits outside +/-5 % of kGravity (still band is referenced to kGravity, position_filter.h:105-107), the pad hold is never active, `a = |f| - g_const` is integrated and the only constraint is the baro. With the MS5607 not initialised or rejecting (init retry, sensors.cpp:2332-2346), the drift 0.5 m/s^2 would reach 30 m/s after 60 s and 900 m after that. (b) NaN is excluded because `isfinite(vz)` is tested. (c) A commanded launch (0x7301 style, USB only) or HITL. I rate the likelihood low with a healthy barometer, but the baro-down case is exactly the one where #1 also bites.

Consequence under one-way states. `set_state` refuses any lower state (flight_state_machine.cpp:36-40). Sequence on the pad: Boost; the burnout test (`low_g_axial_acceleration_g < 1.0` for 500 ms, line 278-283) reads about 1.0 g on a vertical pad (the X axis reads +1 g) so it may fire or not; the landing test runs in Boost/Coast/Descent (line 255-258): accel 0.8-1.2 g, gyro < 10 dps, AGL span <= 10 m for 30 s. As soon as the AGL (no longer pad-held after leaving Pad Idle, so it is baro-driven) has settled, the state goes to Landed about 31 s after the false Boost. Then:
- `FLIGHT_LOG_BIN_LANDED = 0` and `logged_state()` excludes Landed (flash_logger.cpp:39-43): no binary or IMU log on NAND for the real flight.
- Telemetry stays at the normal rate (landed low-power is off, only Pad Idle is throttled), with state = Landed.
- Re-arm only by 0x7300 over radio or USB (restart). That is available, but it takes an operator who notices state = Landed before launch.
- Each false flight writes a flight marker (flash_logger.cpp:425-436) and the session is never auto-erased. After two flight-marked sessions `slots_full_` is true and nothing is logged until an erase/clear (flash_logger.cpp:345, 960). Every commanded launch, HITL flight or false Boost therefore consumes one of two slots. The status byte has a LOG_SLOTS_FULL bit; check it in the pre-launch checklist.

Since there are only two slots and one-way states, bench tests that enter Boost are expensive; consider not marking the slot as a flight until the vehicle has actually flown (for example AGL > 100 m or elapsed Boost > 3 s).

### 5. No NaN or divergence recovery (High, plausible)

`Filter::predict` takes the world acceleration without an `isfinite` check (position_filter.cpp:55-96). `update_axis` accepts any innovation whose square is not greater than the gate; with a NaN innovation the comparison is false, so a NaN is accepted and the whole axis becomes NaN (line 227-240). `symmetrize` clamps with `fmax/fmin` that pass NaN or return the other operand and do not repair it. Sources: a NaN in `filtered_accel_mg` or the quaternion (calibration constants are checked for finite, imu_calibration.cpp:227, 250, but the filter output is not), a 0/0 anywhere. Barometer and GPS inputs are checked (position_filter.cpp:279, 365-368). Reset exists only on HITL source change (sensors.cpp:497).
Result: `observation.agl_valid = isfinite(up().p)` goes false permanently (sensors.cpp:2437); `vertical_velocity_valid` stays true with a NaN value, which is handled (`isfinite` in Pad Idle, comparisons false in Coast), so there is no crash, only a stuck state: Pad Idle never launches, Coast never leaves, Landed is unreachable.
Suggested: in `Filter::predict` and `update_axis`, if any state or covariance is not finite, call `reset()` (keeping `initialized_`) and restart from baro/GPS; plus the filter-independent fallbacks from #1 and #3.

### 6. Attitude validity gate (Medium)

Requirements for the first window: `stationary_min_g/max_g` 0.95-1.05 on every sample and 1 s (attitude_estimator.h:77-78, 89). The accelerometer comes from `low_g_accel_mg` after calibration (sensors.cpp:2134-2136). Without a stored calibration the raw LSM6 offsets apply ("IMU runs uncorrected" in main.cpp), and an offset or temperature drift near 5 % leaves no valid samples; a rocket sitting in a wind-shaken rail and radio transmissions both threaten the window. A non-empty prelaunch history (windows accepted/rejected counters are logged every few seconds, sensors.cpp:2199-2205) is the only diagnostic, so check "attitude initialized" in the pad log before arming.

### 7. Latency of the new rule vs the 2 s ring (Medium)

At net acceleration a (thrust a_total minus 1 g): AGL 30 m at t = sqrt(60/a); vz = 30 m/s at t = 30/a. Boost at max(those) + 0.3 s + up to one sensor pass.
- a = 2 g (19.6): AGL at 1.75 s, vz 34 m/s, Boost at about 2.05 s. The ring (2 s) just misses the first 50-100 ms.
- a = 1 g (9.8): vz at 3.06 s, Boost at about 3.4 s. The first 1.4 s of the burn are lost (ring covers 2 s before the declaration).
- a = 0.5 g: Boost at about 6.4 s.
The comment at flight_state_machine.h:38-43 ("30 m reached about 1.4 s after liftoff at 3 g, so the 2 s buffer still holds the liftoff") ignores the 300 ms qualification and the vz condition. Suggest 4-5 s of pre-launch ring, or declaring Boost on the first accel threshold crossing (see #1).

### 8. False Landed is permanent (Medium)

The landing test (flight_state_machine.cpp:129-188) uses stillness of accel/gyro and the filter's AGL span. A barometer that stops updating but stays "healthy" (sensors.cpp:2392-2394), an attitude-lost filter in `predict_without_imu` (zero acceleration assumption), or a long-standing baro gate can produce an AGL that does not move while the vehicle falls under a canopy at roughly 1 g and low rotation. 30 s of that gives Landed. Under the OLD rule it could be undone by USB; under the NEW rule it cannot. In Landed the NAND logs stop, the attitude holds (attitude_estimator.cpp:175-177), and the filter's Landed pseudo-velocity measurement pins v = 0 (position_filter.cpp:270-274). Descent/Landed also depend only on the filter AGL (see #3, #5). Suggested: require also a raw-barometer span (second source) and add a fallback "Landed -> Descent" only via reset, or keep logging in Landed for the first N minutes at a lower rate.

### 9. HITL and bench (Medium)

HITL entry/exit does not touch the flight state (sensors.cpp:480-510 only rebaselines the maximum and resets the filter/attitude). After a HITL run the state is Landed or Descent; a warm reset resumes Descent from RTC; radio refuses everything except Landed -> Pad Idle and the no-op (command_policy.h:87-95). USB can set Landed and then 0x7300 restarts. If the board arms in that state, the real launch is never declared (#4 logging consequences). Pre-launch check: state must read Pad Idle and the RESUMED flag must be clear.

### 10. Predict gap (Medium)

`predict` returns without propagating when dt > 0.1 s or <= 0 (position_filter.cpp:60-62); the attitude estimator drops steps > 50 ms. A flash-erase or SPI stall of 150 ms in Coast at 300 m/s loses 45 m and P does not grow, so the next barometer/GPS update looks like an outlier until the 25-reject resync. Unchanged from Kalman audit #5.

### 11-14 (Low)

- #11: `persist()` stores `maximum_altitude_meters_` as it is at the moment of the change (flight_state_machine.cpp:50-56): at Boost entry from Pad Idle it is the stale value from the last `reset_for_state` (0 or an earlier flight), at Coast the value at burnout. After a warm resume with a filter, the maximum is re-baselined by the first AGL > max; without a filter (#3) it never updates.
- #12: Boost -> Coast needs `low_g_axial_acceleration_valid` and < 1 g for 500 ms, with no timeout. An IMU read failure keeps the state in Boost (the landing test still works).
- #13: two tasks can call `set_state` -> `persist()` at once; `seal()` zeroes the magic first so an interleaved write at worst yields an invalid record (not resumed).
- #14: RTC_NOINIT memory survives panic, WDT, SW and brownout resets but not true power loss; accepted in the design, listed because it defines what "Cold" means.

## Answers to the specific questions

- Never still for 1 s on the pad: attitude and filter never start; no Boost (#1, #6). No fallback.
- Board boots or reboots on the pad or in flight: Pad Idle boot in flight never starts the filter (#2); warm resume has the state but no estimators (#3).
- Pad hold pinning the altitude during a real launch: `still_` needs |f| within 5 % of kGravity, so any real thrust above 1.05 g releases it at the next predict sample; pad-hold constraints are applied at 25 Hz only while `still_` is true (position_filter.cpp:256-269). A thrust dip back into 0.95-1.05 g exactly at a 25 Hz tick during the first 2 s would re-zero p and v (only for a very low-thrust or chuffing motor). Checked and not a problem for a normal motor.
- NaN: no recovery (#5).
- Still band vs slow motor: a motor with net accel < 0.5 m/s^2 never leaves the band, so the pad hold keeps AGL at 0 and Boost never fires (a vehicle that cannot lift anyway). A TWR of 1.5 works but is slow to declare (#7).
- Baro gating: gate is latched through Boost and Coast (position_filter.cpp:293-307), AGL then comes from the IMU only; no effect on Boost declaration (the Mach cutoff is far above the speeds in the first 300 ms).
- GPS origin: GPS is ignored in the pad hold (position_filter.cpp:369-387) and the filter does not start without the attitude, so GPS cannot help a missed launch.
- Missed launch cost: see #1 (2 s RAM ring only, 3 s telemetry, session erased after two boots).
- False Boost then Landed on the pad, real launch in Landed: #4.
- Reset mid-flight: #2 (cold) and #3 (warm); persistence uses RTC_NOINIT memory (no NVS wear, no blocking in the sensor task, writes only on state change).
- Descent/Landed false positives: Landed under a spinning canopy is delayed rather than false (gyro < 10 dps required); near-apogee Landed cannot fire (|a| ~ 0 g in a ballistic coast); the false case is a frozen AGL (#8).
- Timer wrap: all FSM timers are 64-bit microseconds from `esp_timer` and subtracted as unsigned; qualification timers are reset on every false evaluation and on every state change in `reset_for_state` / `update` (checked).

## Checked and OK

- `set_state` compare-and-swap: concurrent automatic and commanded transitions cannot move the state down (flight_state_machine.cpp:31-48).
- `command_policy::allow_set_flight_state`: radio only Landed -> Pad Idle (restart) and same state; USB forward only; Boost/Coast/Descent -> Landed only; matches the one-way rule. `is_reset_to_pad` requires Landed.
- 0x7300 in Landed: restarts after the ack (usb_serial_echo.cpp:1079-1094); Landed is never resumed (flight_state_store.h:60-64), so the boot is Pad Idle with a fresh filter (this also fixes Kalman audit #1).
- `FlightStateMachine::update`: sanitises AGL with `isfinite`; vz NaN is excluded in Pad Idle and harmless in Coast; Boost entry re-baselines the maximum at the first Boost update.
- Descent threshold: max(5 % of max AGL, 10 m) below the maximum with vz < 0 for 500 ms; a new maximum resets the dwell; the velocity path needs a climb seen, 3 s of Coast, vz < -30 m/s for 2 s.
- `flight_state_store`: record seal order (magic cleared first), checksum, `state <= 4` check, Warm/Cold mapping; no NVS use, so no wear or blocking.
- Gravity measurement: averaging is dt-based, clamps to kGravity +/- 5 %, falls back to kGravity until 1 s of still samples, freezes after 0.5 s of continuous motion or any non-Pad Idle phase; no NaN or divide-by-zero path (alpha = dt / min(still_s, tau)).
- GPS quality filter `usable_for_navigation` (1, 2, 4, 5): used only for the filter and heading; display flags unchanged; the sequence number is advanced only for usable fixes, so a skipped fix is not re-used.
- `restore()` leaves `observed_state_` equal to the restored state so the maximum survives; `coast_entered_us_` is set on the first update.
- Pad hold is ungated, and the zero-velocity pseudo-measurement is ungated: a stale pad offset is removed once still.
- Attitude: gap handling, saturation (lost) logic, Landed hold, median window and window statistics are consistent; nothing found that can hang (loops are bounded by `kMaxWindowSamples` = 1024).
- Heading aligner: bounded loops, ring-buffer indexing correct, `flying` resets inertial velocity on the pad.
