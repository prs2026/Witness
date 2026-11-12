# GPS before launch: sky, lead-in and satellites at launch (HITL, 2026-11-11)

Question: why did GPS stay out for 72.5 s after the sustainer in one eayf run
and 8.9 s in another, and what should be checked on launch day so the
receiver goes into the flight in its best state.

Setup: eayf trajectory cut at 120 s of flight (past apogee at ~88 s), RF GPS
only, flight vibration profile, ISA +10 K, receiver cold-started and
ephemerides uploaded at the start of each run (worst case: real launch day
has the receiver powered much longer). "Sky" = the scenario start minute,
which sets which satellites are up and where (the simulator shifts the
broadcast orbits to the scenario hour; the minute moves the sky). Lead-in =
static time at the pad before the 20 s pad hold of the trajectory, so the
receiver has lead-in + 20 s before ignition.

## Results

| Run | Sky | Lead-in | In view | Fix after | 8 sats after | Sats at launch | GPS lost | Regained (speed) | Outage | Altitude error after boost |
|---|---|---|---|---|---|---|---|---|---|---|
| `runs/20261111-152632` | :04 | 60 s | 9 | 18 s | 25 s | 8 | 36.1 s | 69.0 s (395 m/s) | 32.9 s | 2.7 % |
| `runs/20261111-153238` | :04 | 180 s | 9 | 13 s | 15 s | 9 | 36.1 s | 42.9 s (827 m/s) | **6.8 s** | 2.3 % |
| `runs/20261111-154026` | :43 | 60 s | 12 | 7 s | 7 s | 12 | 36.1 s | 50.1 s (642 m/s) | 13.9 s | 2.8 % |
| `runs/20261111-154609` | :20 | 60 s | 12 | 7 s | 7 s | 9 | 36.1 s | 49.1 s (662 m/s) | 12.9 s | 3.2 % |
| `runs/20261111-155200` | :20 | 180 s | 12 | 7 s | 13 s | 11 | 36.1 s | 119.4 s (108 m/s) | **83.2 s** | 2.6 % |
| `runs/20261111-155947` | :55 | 60 s | 13 | 7 s | 13 s | 11 | **never** | – | 0 | **4.3 %** |
| `runs/20261111-150452` (acceptance, full flight) | (:04-like) | 60 s | 11 | 9 s | never | **5** | 35.1 s | 107.7 s (41 m/s) | **72.5 s** | 2.3 % |
| `runs/20261111-132002` (earlier, old filter) | :43-like | 45 s | 10 | 27 s | 27 s | 10 | 36.1 s | 45.1 s (757 m/s) | 8.9 s | (old filter) |

Planned 7th run (:30, 180 s) did not run: Witness stopped booting after the
previous run (see the end of this report).

Altitude error in early boost (sim ~21 s, ~100-150 m up) was 12-20 m in every
run (3.9-5.4 % against the 100 m floor), independent of GPS: it is the IMU
through the first second of thrust.

## What it shows

1. **The receiver's state at ignition matters more than the sky or the
   lead-in time.** All runs lose lock in the 43 g sustainer (or ride it out);
   how fast it comes back depends on how many satellites it was tracking. 5
   satellites at launch gave 72 s; 8 gave 33 s; 9-12 gave 7-14 s in four of
   five runs.
2. **Longer lead-in is not reliably better.** 180 s helped on one sky (8 -> 9
   satellites at launch, outage 33 -> 7 s) and not on another (same sky as a
   13 s run, 11 satellites at launch, outage 83 s). Reacquisition from zero at
   high speed has a random element: the same conditions gave 13 s and 83 s.
3. **The receiver can stall.** In the acceptance run it fixed in 9 s but sat
   at 4-6 satellites for the whole lead-in with 11 in view; on the other skies
   it reached 8+ in 7-25 s. Stalls (and complete no-fix runs, 4 of ~20 today)
   look like the receiver, not the sky.
4. **Keeping lock through the burn is not automatically good.** On the :55
   sky the receiver never lost the fix, but through the burn its altitude fell
   ~390 m behind with 4 satellites and stayed there for 25 s after burnout;
   the filter re-trusted it 2 s after the acceleration ended and was pulled
   to -466 m (4.3 %, the worst post-boost error of the set). A replay with a
   10 s de-weighting decay gives 2.9 %, 20 s gives 2.6 %. Better: treat a
   drop to 4-5 satellites during or after a burn as a degraded receiver (see
   recommendations).

## Launch-day checklist (GPS)

Before arming:
- Power the receiver (Witness) **early** with the antenna in final position
  and a clear sky: on the pad a real receiver has minutes to collect
  ephemerides and almanac, not the 60 s cold start of these tests.
- Watch the GGA satellite count and the GSA DOPs on the ground station. Go
  only with:
  - **satellites used >= 9** (8 is marginal: 33 s outage here; 5 gave 72 s),
  - **used >= ~75 % of those in view** (GSV, or the planning tool for the
    site and time) - a receiver stuck at 4-6 of 11 is stalled,
  - **HDOP <= 1.0, VDOP <= 1.5**,
  - all of this **stable for at least 60 s** (no satellites dropping out).
- If the count stalls well below the satellites in view for more than about
  2 minutes, cold-start or power-cycle the receiver and upload ephemerides
  again (the HITL lead-in now does this automatically after no fix).
- Check that GPS and barometer altitude agree on the pad (within ~10 m after
  the geoid correction) and that the Kalman pad hold is active.

Not GPS, but for the altitude filter on the same day:
- Run the six-face IMU calibration (now also solves the high-g scale) before
  flight day; keep the rocket still on the pad for the attitude to settle.
- Get a temperature profile (sounding or forecast) for the launch site and
  time: the further from ISA, the more the filter leans on GPS to calibrate
  the barometer.

Satellite geometry: more satellites in view is better (13 in view was the
only run that never lost lock), but the receiver's tracking state dominated.
If a launch window is flexible, a GNSS planning tool for FAR can show the
satellite count and PDOP over the window; prefer times with >= 10 GPS
satellites above 10 degrees.

## Recommendations for the filter (not yet changed)

- Treat a GPS satellite count of 4-5 during or within ~30 s after a burn as
  degraded: inflate the GPS sigma strongly, or skip vertical GPS until the
  count recovers. Today the count is ignored whenever DOP is present.
- A longer GPS de-weighting decay after a burn (5-10 s instead of 2 s)
  costs little in the cases tuned on and halves the :55-sky error.
Both should be re-checked on the Monte Carlo set before adoption.

## Bench issues found

1. **Witness stopped booting** after the :55 run's log fetch and USB power
   cycle (16:04): its bootloader loops with "Image requires efuse blk rev <=
   v1.99, but chip is v2.4 / Factory app partition is not bootable", ~30
   times a second. The same image booted after several power cycles earlier
   today. A read-only eFuse summary could not be taken (serial stream errors
   while it boot-loops). Not touched further.
2. gps-sdr-sim truncated file paths over 100 characters ("ephemeris file not
   found" for longer scenario names); fixed (MAX_CHAR 1024) on the local
   `witness-100hz` branch and rebuilt.
3. The run script started the next run while the boards were still
   rebooting from the log fetch; a wait for Witness is needed between
   back-to-back runs.
