# Witness position filter: altitude accuracy work (2026-11-11)

Goal: altitude error below 5 % throughout the flight, with ascent (launch to
apogee) the priority and descent kept physically sound. Acceptance trajectory:
eayf (FAR, two stages, 1300 m/s, 36 km apogee). Metric: |altitude error| /
max(true height above the pad, 100 m), from launch to landing; the 100 m floor
means "5 m" near the ground.

## Result

| | Ascent worst | Descent worst | Where the worst point was |
|---|---|---|---|
| Witness before this work, HITL eayf (`runs/20261111-132002`) | **19.0 %** | 0.9 % | -3.0 km at 46 s (sustainer to GPS return) |
| Old filter on the Monte Carlo set (16 realistic flights) | 21.0 % | 5.4 % | |
| New filter, Monte Carlo, no pre-flight high-g calibration | **4.73 %** | 0.94 % | early boost/coast (high-g scale still unknown) |
| New filter, Monte Carlo, with six-face calibration | **4.36 %** | 0.94 % | early boost, ~5 m at ~110 m |
| New filter, HITL, 70 s eayf, flight vibration, ISA+10 K, **no GPS fix** (`runs/20261111-145157`) | 5.01 % (firmware before the handoff fix); replay with the fix 1.1 % | – | -8 m at 21.5 s (handoff, fixed) |
| **New filter, HITL full eayf acceptance**, flight vibration, ISA+10 K (`runs/20261111-150452`) | **4.52 %** (coast 2.3 %, sustainer 1.35 %, gap 1.14 %, to apogee 1.07 %) | 0.80 % | -4.9 m at ~140 m, 1.2 s into boost; GPS out 72.5 s (35-108 s) and still within 1.1 % |

## Why the old filter missed by 3 km

1. **High-g zero offset.** The high-g accelerometer reads ~1.5 g off (datasheet
   worst case, modelled by HITL; the board has no stored calibration), and it
   carries every burn above 15 g: 45 m/s of error by burnout.
2. **Lagging GPS accepted under thrust.** In the 43 g sustainer the PX1120S
   fixes ran up to 0.69 s / 866 m behind with 4-5 satellites. The filter
   de-weighted them, but the GPS sigma was capped at 1 km, so they still
   pulled velocity down by ~230 m/s right before the GPS outage.
3. **Dead reckoning through the outage** (36-45 s, Mach 4, baro off above
   Mach 0.8) carried that velocity error into 3 km.

## Method

**Offline replay** (`Witness/softwarev1/tools/kalman_replay.cpp`,
`HITL/kalman_replay.py`): the firmware's own `accel_filter` and
`position_filter` compiled on the PC, fed from a HITL run's recorded IMU
(960 Hz), barometer, GPS fixes and flight states. It reproduces Witness's
in-flight result (19.4 % vs 19.0 %, same shape) in ~8 s instead of a 10-minute
HITL run. Every filter setting is a `key=value` argument.

**Monte Carlo set** (`HITL/kalman_mc.py`): each scenario reuses a recorded
run's truth trajectory, real RF GPS fixes (two different outages: 9 s and
51 s) and flight states, and redraws what real flight will differ in:
IMU synthesis seed (offsets, noise), the flight's predicted vibration at
scale 0.2-1.0, accelerometer sensitivity error (1 % low-g, 3 % high-g, 1σ),
0.5° mounting misalignment, 1° attitude error plus drift, and an atmosphere
ISA ± 15 K. 16 scenarios, evaluated in parallel in ~15 s.

**Tuning** follows the literature on noise-parameter tuning as black-box
optimization with consistency checks (Chen et al., "Weak in the NEES?", arXiv
1807.08855; "Kalman Filter Tuning with Bayesian Optimization", arXiv
1912.08601; and arXiv 2306.07225, which shows chi-square consistency alone
does not prove a good tuning). Here: coordinate search over a fixed set of
scenarios (common random numbers), cost = worst ascent error + 0.2 × mean
ascent error + a steep penalty above 5 % in descent, with NEES of the altitude
reported for every candidate. Lesson from the second search: once the
structure was right, further "improvements" of a few hundredths of a percent
pushed NEES from 5 to 70 (overconfident, overfitted to the 16 scenarios) and
were not adopted. A future cost should include a NEES term.

**Held-out stress case**: the morning's 10 Hz-simulator run (356 s GPS outage,
kilometre-off fixes after a bad reacquisition) was never used for tuning; it
found two descent weaknesses (below).

## Changes and their effect (Monte Carlo worst ascent unless noted)

| # | Change | Effect |
|---|---|---|
| 1 | Pad high-g offset: high-g fitted against low-g while both are in range (`accel_filter::HighGCalibration`) | replay of the HITL run 19.4 % -> 18.0 % alone |
| 2 | GPS de-weighting under acceleration 10x stronger (`gps_lag_accel_s2` 0.5 -> 5) | replay 18.0 % -> 2.6 % |
| 3 | No 1 km cap on the GPS sigma (`gps_max_sigma_m` 1 km -> 100 km) | replay 2.6 % -> 1.5 %; Monte Carlo 21 % -> 10.6 % with 1-2 |
| 4 | High-g scale fitted in flight from coasting (block means, 100 ms hold-off after clipping) | thrust-axis scale recovered to 0.01 %; Monte Carlo 10.6 % -> 5.97 % |
| 5 | Barometer error state, first as a bias random walk | 5.97 % -> 5.22 %, NEES 23 000 -> 6 |
| 6 | Tuning (stress decay 2 s, barometer 5 Pa) | 5.22 % -> 4.89 % |
| 7 | Six-face calibration also solves high-g scale (layout 2, layout 1 still loads) | with calibration 4.40 % |
| 8 | Barometer GPS resync guard: a GPS altitude stream failing the gate is forced in only when no barometer was accepted for 2 s | stress case descent 4830 % (filter jumped to 4.8 km after landing) -> 76 % |
| 9 | Barometer error modelled as a scale k on height (EKF), estimated only while GPS altitude is recent, a Schmidt "consider" parameter otherwise | 4.48 % uncal. / 4.30 % cal.; descent 2.4 % -> 0.94 %; stress descent 76 % -> 21 % (rest is 160 s with no GPS and no barometer) |
| 10 | Accelerometer handoff for vibration: to high-g at 12 g (was 15), back below 10 g for 100 ms (was 13 g / 10 ms) | HITL vibration run replay: boost 5.0 % -> 1.1 %; Monte Carlo 4.48 % -> 4.73 % (high-g scale error counts longer; gone with calibration) |
| – | GPS latency term (H = [1, -L]) | implemented, left at 0: after reacquisition the fixes have no delay; the pre-burn lag is the receiver's dynamics, handled by 2 |

Final defaults (`position_filter.h`): accel noise 0.5 m/s² + 5 % of |a|,
GPS lag 0.1 s × speed and 5 s² × acceleration (decay 2 s), GPS sigma cap
100 km, barometer 5 Pa, barometer scale prior 3 % growing 0.5 %/√km, gate 5σ,
resync after 25 rejects (vertical: only without a healthy barometer).

## What limits it now

- **First boost without calibration**: the high-g sensitivity is unknown until
  the first coast; a 6 % error is ~70 m by the first coast. A six-face
  calibration before flight (now also solving high-g scale) removes most of it.
- **No GPS and no barometer at the same time** (supersonic and above 31 km)
  is IMU-only; with the high-g calibration it held 0.6 % to Mach 4 on HITL.
- **Near the ground in boost** the 100 m floor makes ~5 m count as 5 %.

HITL acceptance details: transitions passed (Boost/Coast/Descent/Landed);
the GPS outage after burnout was 72.5 s in this run (8.9 s in
`runs/20261111-132002`; the receiver used only 5 of 11 satellites at launch,
see open issue 3), first fix after it 12 m from truth; altitude error stayed below
-1.1 % (-376 m at 100 s, before GPS returned) and was 3-13 m after it.

## Open issues

1. Intermittent no GPS fix on the bench (4 of ~15 RF runs; cold start and
   31/31 ephemerides normal; the same scenario can work on a rerun). Not a
   filter issue; unexplained. The HITL lead-in now power-cycles the bench
   USB ports and retries for 45 s before stopping the test (untested on
   hardware). The Pi 4's ports are ganged (one switch for all four), so
   this restarts all three boards; per-port cycling needs an external hub
   with per-port power switching supported by uhubctl (e.g. UUGear MEGA4)
   and a sudoers or udev entry for it.
2. The LSM6DSV320X high-g sensitivity tolerance and non-linearity are assumed
   (3 % / 1 % after calibration); the datasheet could not be fetched.
3. The GPS outage after burnout was 72.5 s in the acceptance run versus
   8.9 s before. Not vibration (HITL vibration only shapes the IMU stream;
   the RF signal follows the trajectory alone): the receiver was using only
   5 of 11 satellites in view at launch (10 of 10 in the 8.9 s run), so it
   fell below 4 a second earlier in the sustainer and had to reacquire from
   nothing, which it managed only near apogee (40 m/s). A longer lead-in,
   or holding launch until most satellites in view are used, would make
   runs comparable.

## Data that would make the simulation closer to flight

1. A six-face calibration of this board (now gives high-g offset and scale).
2. An atmosphere profile (radiosonde or forecast) for the launch: the filter
   could use it instead of ISA.
3. Logs from any real flight with IMU + barometer + GPS (vibration, receiver
   behaviour under thrust).
4. How the board is mounted relative to the airframe axis.

## Tools

```
python HITL/kalman_replay.py RUN_DIR [key=value ...] [--compare-flight]
python HITL/kalman_mc.py make RUN [RUN ...] --n 8
python HITL/kalman_mc.py eval [key=value ...]
python HITL/kalman_mc.py tune key=value ...
```
