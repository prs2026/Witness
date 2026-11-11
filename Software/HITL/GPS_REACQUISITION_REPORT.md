# GPS reacquisition after the sustainer: HITL report (2026-11-11)

Goal: on the eayf trajectory (FAR), bring the time from losing GPS lock to
regaining it below 30 s, find the receiver's dynamic limits, and look through
the SkyTraq PX1120S command set (AN0037) for anything that helps.

**Result: met.** On the updated (unlocked) receiver firmware and the 100 Hz
signal simulator, lock is lost for **8.9 s** at sustainer burnout and comes
back while the rocket is still doing 757 m/s. The first run of the day
(previous receiver firmware and 10 Hz simulator) lost it for 356 s, and every
fix after reacquisition was 1-100 km off.

Metric: outage = first no-fix GGA after lock loss to the first fix (sim time),
plus the time to the first fix within 150 m of truth.

## Headline numbers

| Run | Receiver FW | Simulator | Lock lost | Lock regained | Outage | Fixes after |
|---|---|---|---|---|---|---|
| `runs/20261111-023336` | previous | 10 Hz, GPS+Galileo | 33.97 s (591 m/s) | 389.85 s | 355.9 s | all 1-100 km off |
| `runs/20261111-124426` | previous | 100 Hz, GPS only | 34.06 s (just past 515 m/s) | 85.14 s | 51.1 s | all good (13 m at regain) |
| `runs/20261111-132002` | updated | 100 Hz, GPS only | 36.13 s (burnout, 43 g to 11 g, 1291 m/s) | 45.05 s (757 m/s) | **8.9 s** | all good (18 m at regain, 10 SVs by 52 s) |

The same scenario (`gps_sim/20261111-124320-flight-eayf-gps`) was used for
the last two rows. Flight-state transitions passed within 0.02 s in all three.

## What changed the result

1. **Simulator update rate (biggest effect on the old firmware).**
   gps-sdr-sim refreshed range, code rate and carrier Doppler every 100 ms and
   held them constant in between, so under acceleration the Doppler moved in
   steps: about 100 Hz every 100 ms at 20 g, about 230 Hz at the 40 g
   sustainer. Real flight has no such steps. gps-sdr-sim now takes
   `-DSIM_UPDATE_HZ` (HITL builds it at 100 Hz, motion files are written at
   100 Hz, Galileo gets a 10 Hz copy). On the old firmware this cut the eayf
   outage from 356 s to 51 s and removed the km-off fixes after reacquisition.
   In the burst sweep a 2 g burst went from a 2.8 s outage / 159 m error
   (10 Hz) to no loss / 8 m (100 Hz).
2. **Unlocked receiver firmware (biggest effect overall).** The old firmware
   dropped lock at exactly 516 m/s on a gentle 1 g ramp and only reacquired
   69 s after slowing back under 515 m/s (101 s outage). The updated firmware
   tracked through 600 m/s with no loss, held a fix at 1182 m/s and 43.5 g
   during the sustainer, and reacquired at 757 m/s. Static fix at 110 km
   altitude: PASS (35.3 s cold TTFF, 0.7 m H / -5 m V).

No receiver parameter change beyond the existing configuration (airborne
mode, 25 Hz, all constellations, 3 deg elevation mask, QZSS, SBAS) was
needed to reach the target.

## Receiver limits measured (updated firmware, 100 Hz simulator)

Jerk-limited level-flight bursts (raised-cosine 0.5 s edges, +a then -a, back
to rest) and a 1 g speed ramp:

| Profile | Satellites (before -> min) | Lock | Max / median error |
|---|---|---|---|
| 2, 4, 6, 8, 12 g bursts | 11 -> 11 | held | 7-35 m / 0.3-1.3 m |
| 16 g | 11 -> 10 | held | 34 m / 1.3 m |
| 20 g | 10 -> 7 | held | 34 m / 2.5 m |
| 25 g | 11 -> 0 | lost 1.8 s at the -25 g reversal | 88 m / 0.8 m |
| 30 g | 11 -> 7 | held | 45 m / 22.7 m |
| 40 g | 11 -> 9 | held | 36 m / 18.7 m |
| 50 g | 11 -> 10 | held | 36 m / 6.8 m |
| 1 g ramp to 600 m/s, down to 400 m/s | 9-10 | held | 166 m / ~24 m |

Steady acceleration up to 50 g is tracked; the loss at 25 g happened at the
reversal (jerk), and the eayf loss is at burnout, also a fast change in
acceleration on top of ~1300 m/s. The ~24 m median error at 400-600 m/s fits
the receiver's output latency (about 50 ms at that speed).

## Run start: cold vs hot start

The ask was to cut the lead-in. A cold start plus the ephemeris upload gives
a first fix in 11-35 s (5 runs), so lead-in is now 45 s instead of 90 s,
which saves 45 s of IQ generation and transmission per run.

A hot restart (0x01 mode 1 with the scenario UTC and pad position, sent
through Witness's assist path) gave a fix in 19 s once, but twice gave no fix
at all (60 s and 396 s) when the receiver already held a fix from another
scenario; it keeps its own last position rather than the one sent. It is
supported (`hot_start` in the HITL GPS options, `gps_rf.hot_restart_frames`)
but not used by default.

## SkyTraq AN0037: options reviewed

| Message | What it does | Status |
|---|---|---|
| 0x64/0x17 navigation mode | 0 auto ... 4 balloon, 5 airborne, 9 SLR (speed lag reduced) | airborne in use; SLR and balloon not needed once the target was met (candidates if lag matters: SLR targets the ~24 m latency error) |
| 0x0E update rate | 1-50 Hz | 25 Hz in use |
| 0x64/0x19, 0x2B, 0x62/0x03, 0x62/0x01 | constellations, elevation/CNR mask, QZSS, SBAS | in use |
| 0x39 / 0x3B position pinning | holds position at low speed | not changed (affects only near-static) |
| 0x63/0x01 SAEE | self-aided ephemeris estimation | not needed (ephemerides are uploaded) |
| 0x01 system restart, hot/warm with UTC + lat/lon/alt | re-seeds the search; altitude field limited to -1000..18300 m, 0.01 deg resolution | Witness now reconfigures after a host-sent restart; hot start unreliable as above |
| 0x04 factory defaults | | not used (per instruction) |
| Operating limits (datasheet) | 4 g dynamics, altitude < 80 km, velocity < 515 m/s | updated firmware: 515 m/s gone, 110 km static works, 50 g bursts tracked |

## Witness firmware changes (uncommitted)

- A 0x01 System Restart sent through the GPS assist path is detected on its
  ACK; Witness waits 2 s and reconfigures the receiver (host test added).
- `GpsReceiver::kFixedBaud = true`: the receiver is assumed at 115200 and the
  rate is never changed (no scan, no measured-rate hint, no 0x05 switch, no
  rescan after a restart), per bench decision after the receiver update.
  The scan, a rescan-after-restart path and a fallback that configures a
  receiver at its own rate (>= 57600) when it ignores the switch remain for
  `kFixedBaud = false`; host tests cover both.
- 25 Hz update rate unchanged (a 10 Hz build was only a test).

## HITL tool changes (uncommitted)

- gps_rf: 100 Hz motion and `check_motion_rate` (refuses a gps-sdr-sim build
  for another rate, which would play the motion 10x too slow); `hot_restart_body`
  / `hot_restart_frames`; `make_player` / `RemoteHackRFPlayer` to play IQ on a
  HackRF on the bench Pi (IQ copied first, 55 MB/s).
- hitl_session: `hot_start` GPS option; `remote` HackRF option.
- witnessOS HITL tab: "HackRF is on the bench Pi" option.
- gps-sdr-sim (sibling checkout `../gps-sdr-sim`): `SIM_UPDATE_HZ`, committed on
  the local branch `witness-100hz` (not pushed: the clone's remote is the
  upstream author's repository). HITL needs that branch checked out and
  `python gps_rf.py build`; a build without it is refused by
  `check_motion_rate`.

## Open issues

1. **Intermittent no first fix.** One run in about eight (L6) never got a fix
   in 145 s although the cold start, reconfiguration and 31/31 ephemeris
   ACKs were normal; the identical rerun was fine (13.4 s). Cause unknown.
2. **88.4 kbaud episode after the receiver update.** Right after the update,
   a cold start through Witness left the receiver talking at ~88.4 kbaud
   (115200 x 0.767) and configuration failed at boot. After the receiver was
   set to 115200 / 25 Hz in its flash over a direct USB-UART link and
   Witness was fixed to 115200, boot and cold start both configure cleanly.
   Worth keeping an eye on after any future receiver update.
3. **HackRF on the Pi** trips the Pi 4 USB over-current (all ports reset,
   Witness included) as soon as it transmits; it needs a powered hub. The
   HackRF is back on the PC. A wedged HackRF on Windows can be reset with
   `pnputil /restart-device "USB\VID_1D50&PID_6089\0000000000000000645061DE255E8413"`
   from an admin prompt.
4. The burnout outage (8.9 s) is the remaining loss; navigation mode 9 (SLR)
   is the next thing to try if it or the latency error needs to shrink.

## Run index

Limit sweeps are in `Software/HITL/runs_limits/` (nmea.txt with HackRF-time
stamps, meta.json). Scenarios: `gps_sim/20261111-112228-limits-AB` (10 Hz),
`gps_sim/20261111-122738-limits-AB` and `gps_sim/20261111-133700-limits-C`
(100 Hz), `gps_sim/20261111-124320-flight-eayf-gps` (eayf, 100 Hz, 45 s
lead-in).

## Full change log

| # | Run | Change | Result | Delta |
|---|---|---|---|---|
| R0 | runs/20261111-023336 | previous best (GPS+Galileo, 10 Hz sim, cold start, 90 s lead-in) | lost 33.97 at 591 m/s, regained 389.85; fixes after 1-100 km off | baseline: 355.9 s |
| L1 | runs_limits/20261111-121740-L1-hot | limit sweep, 10 Hz sim, hot start | hot TTFF 19 s; every burst 2-20 g drops 2-11 s; 1 g ramp lost at 516 m/s, regained 69 s after back under 515 m/s | limits mapped (10 Hz) |
| - | gps-sdr-sim | SIM_UPDATE_HZ=100 | static PASS 0.3 m | sim fidelity |
| L2 | runs_limits/...-L2-hot-100hz | 100 Hz, hot start after a run ending at 400 m/s | no fix in 396 s | hot start unreliable |
| L3 | runs_limits/...-L3-cold-100hz | 100 Hz, cold start (90 s) | fix 11.4 s; 2 g burst no loss, 8 m (10 Hz: 2.8 s, 159 m) | 10 Hz dropouts were a sim artifact |
| L4 | runs_limits/...-L4-hot-from-rest | hot start after a run ending with a fix | no fix in 60 s | cold start kept for runs |
| S110 | static 110 km | altitude check | PASS 35.3 s, 0.7 m / -5 m | altitude unlocked |
| E1 | runs/20261111-124426 | 100 Hz sim, GPS only, 45 s lead-in, old receiver FW | lost 34.06, regained 85.14, 13 m, 0 bad fixes | 355.9 -> 51.1 s |
| - | receiver FW update | user | 88.4 kbaud after cold start, config failed | two aborted runs |
| - | Witness FW | kFixedBaud, restart handling | boot and cold start configure at 115200, 25 Hz | unblocked |
| E2 | runs/20261111-132002 | updated receiver FW, same scenario as E1 | fix at 1182 m/s / 43.5 g; lost 36.13 at burnout, regained 45.05 at 757 m/s, 18 m | 51.1 -> **8.9 s** |
| L5 | runs_limits/20261111-132936 | updated FW, 100 Hz sweep | no loss 2-20 g; no loss to 600 m/s | speed limit gone |
| L6 | runs_limits/20261111-133720 | 25-50 g bursts | no first fix in 145 s (setup normal) | open issue 1 |
| L7 | runs_limits/20261111-134147 | rerun of L6 | 25 g: 1.8 s; 30/40/50 g held | high g tolerated |
