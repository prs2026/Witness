# Kalman filter audit, 2026-11-15 (sensors up)

Scope: the full chain that feeds `position::Filter` and what consumes it, at commit `b2f6610`:
LSM6DSV320X FIFO (960 Hz) -> `imu_calibration::apply` -> `HighGCalibration` -> `FilteredAccel`
(low/high-g selection + 100 Hz Butterworth) -> `attitude::Estimator` (gyro median, quaternion) ->
body-to-ENU rotation (`Sensors::predict_position`) -> `Filter::predict`; MS5607 -> `pressure_altitude_meters` ->
`update_baro`; NMEA GGA/GSA -> `update_gps`; pad/landed pseudo-measurements; the flight state machine
(AGL = `up().p`, vz = `up().v`).

Method: I read the code, checked the math against first principles and the datasheets, and ran the host tests
(position_filter, accel_filter, attitude_estimator and nmea_gps all pass). I also wrote a small host simulation to
confirm finding 1. Nothing was changed in the firmware.

## Findings

| # | Severity | Finding |
|---|---|---|
| 1 | **High** (reproduced) | After a flight, a return to Pad Idle never re-zeroes the position, so the next Boost can fire on the pad |
| 2 | **High** | GPS fixes of quality 6 (dead reckoning), 7 and 8, and 2D fixes, are used as real measurements |
| 3 | Medium | Gravity is fixed at 9.80665 m/s², with no accelerometer bias state and no g(h) |
| 4 | Medium | Boost depends on the Kalman AGL only: any real 30 m climb in Pad Idle while not still counts as a launch |
| 5 | Medium | A predict gap over 100 ms is skipped entirely: the position is not advanced and P does not grow |
| 6 | Medium (to verify) | The gyro median (5 samples) was sized for 100 Hz; at 960 Hz it removes only 2 ms spikes |
| 7 | Low | Barometer latency is not modelled, and half the pressure samples are thrown away |
| 8 | Low | The Schmidt "consider" step permanently zeroes the scale cross-covariances |
| 9 | Low | Unmodelled: Coriolis, Earth curvature, IMU lever arm, and ballistic coast without the IMU |
| 10 | Info | Stale comments |

### 1. A return to Pad Idle keeps the flight's position: false Boost (High)

`update_constraints` pins the pad with `update_position(axis, 0, 0.05², unused)`. That passes a reject counter,
so the pseudo-measurement goes through the 5-sigma innovation gate. `unused` is a local that is 0 on every call,
so it never reaches `max_consecutive_rejects`: a rejected pad hold stays rejected forever. The zero-velocity
pseudo-measurement is ungated (`nullptr`), which pins v = 0 and keeps P00 small, so the gate never opens.
The baro and GPS updates are skipped while still (they only track their references). Nothing resets the filter on
entry to Pad Idle: `position_filter_.reset()` runs only on a HITL source change.

Host reproduction (`scratchpad/kf/repad.cpp`): pad 5 s, a 75 m hop (Boost/Coast), 10 s Landed, then Pad Idle:

```
pad: up=0.00
after hop: true up=75.0  filter up=75.0
Landed: filter up=75.0
PadIdle +20 s at the new site: filter up=75.0
```

**Consequence:** a recovered rocket that landed 30 m or more above the pad altitude goes to Boost 0.3 s after
`0x7300` (which is allowed over radio). A rocket that landed lower starts its next flight with an AGL offset in the
FSM, the maximum and the telemetry. East/north also keep the landing offset. With never-erase logging, a false
flight uses up a log slot. The bench test of Landed -> Pad Idle passed only because the board had not moved, and
HITL resets the filter on every source change, so it cannot hit this path.

**Fix (small):**
- Reset the position filter, and the FSM's AGL history, on any transition into Pad Idle from another state.
- Make the pad position hold ungated (`update_axis(..., nullptr, ...)`), as the velocity hold already is.
- Either alone fixes the reproduction. Both are cheap.

### 2. Dead-reckoned, simulated and 2D GPS fixes are used (High)

`NmeaGps::parse_gga` accepts fix quality 1-8 as `valid`.
- **Quality 6:** the receiver's own dead reckoning (extrapolation after losing satellites). That is exactly the
  case after high g and during reacquisition, the receiver behaviour seen in HITL.
- **Qualities 7 and 8:** manual input and simulator mode.

They all reach `update_gps` with the normal DOP and lag sigma. The gate catches gross errors, but a smooth
extrapolation inside 5 sigma is accepted. After 25 rejects (1 s), the horizontal axes are forced in regardless.

Also, a 2D fix (GSA type 2) reports an altitude that is held, not measured. The 25 Hz configuration outputs only
GGA+RMC, so there is no GSA and the firmware cannot tell 2D from 3D. The vertical sigma then comes from
`1.6 x HDOP`.

**Fix:**
- Use only quality 1, 2, 4 and 5 for the filter. Keep 6 for display at most.
- If GSA is enabled (even at 1 Hz), skip the up axis when the fix type is 2. Otherwise, give the up axis a large
  sigma whenever fewer than about 5 satellites are in use.

### 3. Gravity and accelerometer bias are not estimated (Medium)

`predict` subtracts the constant `kGravity`. Three things show up as a constant vertical acceleration error that is
integrated freely whenever the baro is gated (supersonic Boost and Coast) and GPS is lagging or lost:
- the residual accelerometer offset after the six-face calibration (temperature drift; tens of mg if
  `accel_valid` is false);
- the local g at the site versus the site of the calibration (~0.01 m/s²);
- the drop of g with height (0.117 m/s² less at 38 km).

The error grows as ½·b·t²:
- 10 mg (0.098 m/s²) over a 60 s coast is about **180 m**;
- the g(h) term alone is several tens of metres on a 38 km flight.

The pad hold hides this, because v is pinned to 0. The data to remove it is already there: on the pad, the mean
\|specific force\| is exactly the local gravity plus the residual bias along the body axis that becomes "up".

**Fix (cheap):**
- During the pad hold, track the mean |f| of the still samples (a time constant of a few seconds).
- At departure, subtract that value instead of 9.80665.
- Then scale it with g(h) = g_pad · (R/(R+h))².

A fuller option is a per-axis accelerometer bias state on up, which the pad hold makes observable.

### 4. Boost needs only the Kalman AGL (Medium, design coupling)

Boost requires AGL > 30 m for 300 ms and nothing else.

**How a false Boost can happen:**
- In Pad Idle the filter integrates freely, and the baro is used, whenever |f| leaves the ±5 % band at an update
  tick.
- So a real 30 m climb without stillness is a launch: a vehicle or aircraft, a drone lift, a hoist, a tall
  building with vibration.
- Finding 1 is another route to the same result.

The old 10 g trigger was immune to all of these.

**Mitigation:** keep the AGL rule, but also require a thrust cue in the same window, for example low-g axial
> 2 g seen in the last 2 s, or vz > 15 m/s. Burnout already uses the axial channel, so it is known to be
available.

### 5. A predict gap freezes the position (Medium)

`Filter::predict` returns without any change when dt > 0.1 s (and `attitude::Estimator` treats > 50 ms as a gap).
It does not even apply p += v·dt, and P does not grow.

**Consequence:** a 150 ms stall (FIFO overrun, flash or erase contention) at 300 m/s loses 45 m. The barometer and
GPS that follow then look like outliers against an over-confident P. They are rejected until the 25-reject resync,
about 1 s.

**Fix:** on a gap, propagate with the current velocity and the `no_imu_accel_noise` process noise for the gap
length, which is what `predict_without_imu` already does. Count the gaps in the 1 Hz log.

### 6. The gyro median may not remove radio interference at 960 Hz (Medium, needs a log)

The attitude header describes 1-3 sample spikes "worst while the LoRa PA transmits" and sizes the median (5) for
that, quoting "20 ms latency at 100 Hz". The IMU now runs at 960 Hz. A 5-sample median there removes runs of 2
samples or fewer (2 ms). If the interference lasts as long as before in time rather than in samples (a TX burst is
11-44 ms), it passes through.

The host test (100 Hz model) already shows the effect of the interference:

| Interference model | Pure-gyro coast drift over 10 s |
|---|---|
| Radio off (median 5) | 1.4-2.9° |
| Radio on, 90 % bursts (median 5) | 10-12° |

During a burn, a tilt error θ costs a·(1 - cos θ) vertically. In the 43 g sustainer, 10° is 6.4 m/s².

**To check:** take a NAND IMU log on the bench with Witness transmitting, and look at the gyro around each TX.

**Possible fix:** with GPS-timed slots, Witness knows when its own PA is on. During its own TX (plus a margin),
hold the last good rate instead of trusting the gyro.

### 7. Barometer timing (Low)

- **Latency:** the MS5607 delivers a pressure about every 20 ms (OSR 4096, temperature and pressure alternating).
  The filter consumes only the newest one at the 25 Hz tick and treats it as "now". The reading is 10-30 ms old:
  3-8 m at 250 m/s subsonic, against a sigma of about 0.65 m at sea level.
- **Discarded samples:** every other sample is thrown away.

**Fix:**
- Average the samples since the last tick (about √2 less noise).
- Use the delayed-measurement form that already exists (`hv = -lag`), with the sample timestamp the driver
  records.

### 8. Barometer scale "consider" step (Low)

When GPS altitude is absent, `update_baro` zeroes P02/P12 every time, rather than only excluding k from the update.
When GPS returns, the scale and position correlation is rebuilt from zero. This is acceptable, and
the Monte Carlo was tuned with it. A textbook Schmidt update would keep the cross terms and set only the k gain to 0.

### 9. Unmodelled effects (Low)

- **Coriolis:** 2Ω×v is about 0.1 m/s² at 1000 m/s, roughly 180 m over 60 s without GPS.
- **Flat-earth ENU:** the integrated up and the GPS altitude differ by d²/2R (8 m at 10 km downrange, 125 m at
  40 km).
- **IMU lever arm:** the centripetal term under roll averages out in the world frame, but it inflates the
  `accel_noise_fraction` noise.
- **Attitude lost in a ballistic coast:** `predict_without_imu` assumes zero net acceleration. That is right under
  a parachute but 1 g wrong if the gyro saturates during a ballistic coast; the 10 m/s² noise covers it.

All of these are bounded by GPS when it is present.

### 10. Stale comments (Info)

- `position_filter.h` `Axis`: says the barometer "measures p + b". It is a scale: (1 + k)·p.
- `position_filter.h` header: says still means "near 1 g and slow", but there is no speed check.
- `attitude_estimator.h`: "20 ms latency at 100 Hz"; `sensors.h`: "100 Hz loop diagnostics". The IMU runs at
  960 Hz.

## Checked and correct

- **MS5607:** the first- and second-order compensation uses the MS5607 constants (C2·2^17, C1·2^16, OFF2 = 61·dT²/16),
  not the MS5611 ones. The PROM CRC4 is checked.
- **ISA:** the 5-layer inversion and the geopotential-to-geometric conversion are correct. The baro sigma model
  σ_p/(ρg) is also correct.
- **Rotation and attitude:** the body-to-ENU quaternion rotation (v + 2w(u×v) + 2u×(u×v)) is correct.
  Attitude propagation q ← q·exp(ω dt) is in the body frame and normalized.
- **Accelerometer filter:** the Butterworth biquad (pre-warped bilinear) is correct. Its steady-state priming
  satisfies the DC condition.
- **Low/high-g handoff:** the hysteresis is correct, and the filter state carries across the switch.
- **Predict covariance:** F P Fᵀ + Q, including the P02 += dt·P12 cross term, is correct. The Q terms
  (q dt³/3, q dt²/2, q dt) are correct.
- **Update:** the scalar EKF update (g = P Hᵀ, K = g/S, P -= g gᵀ/S) is correct for the H = [1+k, 0, p] row.
  The predicted value is (1+k)·p.
- **Mach gate:** the latch through Boost/Coast and the release in Descent below Mach 0.75 match the
  documentation.
- **GPS:** each fix is used once (sequence de-duplication). The origin is set from "fix minus estimate" for a fix
  that first arrives in flight. Local metres-per-degree are computed in double.
- **Float32 precision** is adequate to 40+ km (ulp ~4 mm at 38 km). P floors and clamps prevent negative
  variances.

## Test coverage gaps

- No host or HITL test of Landed -> Pad Idle -> a second flight. HITL resets the filter on every source change,
  so it would not find finding 1.
- HITL feeds ideal, calibrated IMU values (calibration is skipped). Accelerometer bias, the gravity mismatch and
  gyro interference appear only in `kalman_mc.py`, not in firmware-in-the-loop runs.
- No test of GGA quality 6 or of 2D fixes, and none of predict gaps over 100 ms.

## Suggested order

1. Finding 1 (filter reset on entry to Pad Idle and an ungated pad hold), plus a host test. This is a one-hour job.
2. Finding 2 (GGA quality filter).
3. Finding 5 (propagate through gaps).
4. Finding 3 (gravity from the pad) and finding 4 (thrust cue for Boost); re-run `kalman_mc.py` and the HITL nominal
   flight afterwards.
5. Finding 6: take a bench IMU log during TX first, then decide.
