# Witness HITL simulator

A hardware-in-the-loop (HITL) tool for the Witness flight computer. It
simulates a flight on the host and streams the simulated sensor data to a real
Witness over its USB serial link. Witness runs its actual flight-state detector
on that data. The tool then compares the transitions Witness reports against a
reference model of the same firmware rules.

It uses only the Python standard library and Tkinter (tested with Python 3.14).

| File | Purpose |
|---|---|
| `hitl_protocol.py` | Encodes 0x0C frames and the ENTER/EXIT commands. Also holds the resynchronising decoder for the mixed USB stream. |
| `sim_model.py` | Deterministic 100 Hz vertical flight model, presets, and the reference copy of `flight_state_machine.cpp`. |
| `trajectory_csv.py` | CSV trajectory source: parses `name [unit]` exports, resamples to 100 Hz and converts to the same per-tick samples, plus the truth quaternion. |
| `attitude.py` | Quaternion helpers and the truth-vs-Witness attitude comparison and summary. |
| `trajectories/` | Reference trajectory CSVs. `a3241569-trajectory.csv` is the bundled fixture. |
| `hitl_session.py` | TCP connection, reader thread, 100 Hz sender thread, state tracking, pass/fail evaluation, run recording. |
| `hitl_gui.py` | Tkinter GUI and the headless CLI. |
| `tests/` | Unit tests, an end-to-end test, and `fake_witness.py`, a local fake Witness TCP server. |

## Connecting

On the bench, ser2net on host `prs2026hitl` exposes each board's USB serial port
as a raw bidirectional TCP socket. The ports are raw 7100, 7101 and 7102 and
follow `/dev/ttyACM` enumeration order. Defaults are `prs2026hitl:7100`, where
Witness is today. **The ports have moved between boards before.** If the
stream doesn't look like Witness, check `/dev/serial/by-id` on the bench host,
or run `esptool read-mac`. The Witness MAC is `84:c7:bb:17:62:24`. ser2net
serves one TCP client per board at a time, so close Horizon or any other
client first.

The GUI saves the last host, port, preset and tolerance to `hitl_settings.json`
next to the script. That file and `runs/` are gitignored.

## Running

GUI:

```
py -3.14 hitl_gui.py
```

1. Set the host and port, then click **Connect**. The flight state, the status
   bit indicators and the byte counters start updating.
2. Pick a preset. You can edit any parameter. **Predict transitions** shows the
   expected apogee and when each transition should happen.
3. Click **Start sim**. If HITL is not already active, this sends HITL ENTER
   first. It then streams 0x0C frames at 100 Hz from a dedicated thread that
   keeps an absolute schedule, so timing does not drift. When the simulation
   ends, the tool sends EXIT.
4. Click **Stop**, **Exit HITL**, Disconnect or close the window to stop. Each
   of these stops streaming and sends EXIT.

The plot shows simulated true altitude (MSL) against the barometric altitude
Witness reports, and the simulated low-g X against the accel X Witness reports.
The table compares the expected time of each transition with the time Witness
reported it. A red banner appears in these cases:

- the HITL-active bit is not seen within 2 s of starting;
- the bit drops during a run;
- a run fails.

Headless, for one run with a pass/fail exit code:

```
py -3.14 hitl_gui.py --headless --preset "Nominal 10 g" --host prs2026hitl --port 7100
py -3.14 hitl_gui.py --list-presets
py -3.14 hitl_gui.py --headless --preset "Long burn" --set burn_time_s=5 --tolerance 0.5 -v
py -3.14 hitl_gui.py --headless --csv trajectories/a3241569-trajectory.csv
py -3.14 hitl_gui.py --headless --csv trajectories/a3241569-trajectory.csv --dry-run
py -3.14 hitl_gui.py --headless --csv my.csv --set site_alt_msl_m=1200 --set after_apogee_attitude=pendulum --att-final-tilt 3
```

`--dry-run` builds the frames and prints the conversion report and expected
transitions without connecting. `--speed N` sends N ticks per 10 ms and is
refused unless the host is loopback, because Witness integrates with its own
clock, so hardware runs are always real time.

The headless run prints the transition table. Exit codes:

- `0`: pass.
- `1`: fail. One of these happened:
  - a transition was missed, unexpected or out of order;
  - a transition was off by more than `--tolerance` seconds (default 1.0);
  - HITL never became active (the run is aborted after 2 s) or dropped;
  - Witness was not in Pad idle at the start.
- `2`: connection or argument error.

Transition times are given in simulation seconds. A Witness transition is
timestamped with the sim time being sent when the host sees it. The host sees
it either in the `flight_state: a -> b` log line or in two consecutive status
packets, whichever comes first. The timestamp therefore includes USB, ser2net
and network latency, plus the 10 Hz sensor-packet period.

Each run is saved to `runs/<timestamp>/` with these files:

| File | Contents |
|---|---|
| `params.json` | Parameters, expected transitions and model summary. |
| `sent.csv` | One row per 0x0C frame. |
| `received.jsonl` | Every decoded Witness packet, with host time and sim time. Echoes are excluded. |
| `log.txt` | Witness log lines and tool events. |
| `nmea.txt` | Raw GPS NMEA lines. |
| `transitions.csv` | The transition table. |
| `summary.json` | The pass/fail result, including the attitude summary for CSV runs. |
| `attitude.csv` | CSV runs only. One row per 0x02/0xE0 packet, truth vs Witness. The layout is loaded directly by Horizon's Attitude tab (see below). |

`sent.csv` also has the Y/Z axes and, for CSV runs, the truth quaternion for
every tick (`q_truth_x..w`).

## Presets

| Preset | What it does |
|---|---|
| Nominal 10 g | 14 g average thrust for 2 s, launched from 1401 m MSL (about 1.7 km apogee). Drogue at 25 m/s, main at 200 m AGL at 6 m/s, landing detected 30 s after touchdown. |
| Long burn | 11.5 g for 6 s, about 5.7 km apogee. Long descent, about 330 s. |
| High-g short burn | 40 g for 0.6 s with a 120 dps roll. Low-g saturates at 16 g, so launch detection uses high-g. |
| GPS dropout | No fix and 0 sats from 11 s to 40 s. |
| No GPS fix | Fix quality 0 for the whole run. |
| Sea-level site | Launch site at 5 m MSL. |
| Pad abort - no launch | 60 s on the pad. No transitions are expected. |
| Quick hop (test) | Short flight with no main chute, used by the tests. |

Model conventions match the firmware's flight-state detector:

- The long axis is +X, and the accelerometers read specific force, so +X reads
  +1 g on the pad.
- During boost, X reads thrust minus drag. During coast it reads minus drag,
  which is negative.
- Under a canopy, X reads (dv/dt + g)/g, which includes a deployment spike.
  The gyros show a conical sway, 15 dps by default. That keeps the "Landed"
  gyro condition from qualifying until touchdown.
- Pressure follows the ISA atmosphere at true MSL altitude, so Witness's
  1013.25 mbar pressure altitude roughly equals true MSL altitude.

## CSV trajectories

A run can be driven from a trajectory export instead of a preset. In the GUI,
click **Load CSV…**. The parameter panel then switches to the CSV parameters.
Pick a preset to go back. Headless, use `--csv path`. `--set key=value`
then applies to the CSV parameters below.

### Input

- Header cells are `name [unit]` and matched by name, case-insensitive.
- `time` and `altitude` are required. Altitude is AGL.
- The other recognised columns are `east`, `north`, `east velocity`,
  `north velocity`, `vertical velocity` and `roll rate`.
- Supported units:
  - time: s, ms;
  - length: ft, m, km;
  - speed: ft/s, m/s;
  - roll rate: rev/s, rpm, deg/s, rad/s.

  An unknown unit is an error. A missing unit is assumed SI, and the tool
  notes it.
- Missing east or north is taken as 0.
- A missing velocity, or a `nan` in one, is derived from the position by
  non-uniform central differences.
- A missing roll rate is taken as 0.
- Rows whose time or altitude is `nan`, or whose time goes backwards, are
  dropped.
- Rows that share a timestamp are averaged. The bundled export prints time to
  about 6 significant digits, so distinct samples can share a printed time
  (4 cases, all around 15.85 s).
- The first time any other column turns `nan` is recorded as the "aero-nan"
  time. In the bundled file this is 91.62 s, which is probably recovery
  deployment.

### Conversion assumptions

Conventions match the firmware and the comms.h "Orientation quaternion"
comment:

- WORLD is ENU, with +Z up.
- BODY is the LSM6 frame, with +X along the nose.
- Quaternions are Hamilton, BODY to WORLD, written x, y, z, w on the wire.
- For a vertical rail the pad truth is `(0, -0.70711, 0, 0.70711)`. This is
  the shortest arc from body +X to world +Z, the same as Horizon's reference.

| Item | Assumption |
|---|---|
| Timeline | Sim time 0 starts a stationary pad hold (`pad_hold_s`, default 20 s). CSV time 0, minus the first CSV time, is sim `pad_hold_s`. After the last CSV row, velocity ramps linearly to 0 over `touchdown_s` (0.2 s), which models the impact. The vehicle then sits still for `post_landing_s` (40 s), so the Landed rule can complete (30 s). |
| Resampling | Exact 100 Hz grid. Positions and velocities are linearly interpolated. |
| Acceleration | Central difference of the interpolated velocity over one tick, then a centred moving average of `accel_smooth_s` (50 ms, 5 samples). Edges stay sharp. On the bundled CSV, the sent high-g X first exceeds 10 g at 20.03 s (CSV 20.031 s). Low-g X falls below 1 g at 22.50 s and 35.83 s (CSV 22.489 s and 35.838 s), and rises again at sustainer ignition at 32.48 s (CSV 32.49 s). |
| Wind / AoA | Zero wind, so air-relative velocity is the CSV velocity. The angle-of-attack column is ignored. It is about 0 after the rail, and it is meaningless at zero speed, where it reads 90°. |
| Rail | The rail direction is the velocity direction at the first row faster than `rail_dir_speed_mps` (0.5 m/s). The bundled file is vertical on the rail. |
| Truth attitude | The nose (body +X) follows the velocity direction. Between `follow_speed_mps` (10 m/s) and twice that, it blends linearly from the rail direction to the velocity direction, so it never jumps. Tilt is propagated by minimal rotation (parallel transport), so no spurious roll is added. Roll is the integral of the roll-rate column about body +X. |
| After apogee | From the earlier of apogee (max altitude) and the aero-nan time, the attitude is held (`after_apogee_attitude=hold`, the default). It does not follow the descending velocity vector, which would flip the body 180°. `pendulum` adds a swing about body Y of `pendulum_amp_deg` (10°) with period `pendulum_period_s` (4 s), faded in after apogee and out before touchdown. The final attitude is held on the ground. On the bundled flight the nose is about 90° from vertical (horizontal) at apogee and stays there. |
| Gyro | `omega_k = log(q_{k-1}^-1 q_k) / dt` in BODY. Integrating `q_k = q_{k-1} exp(omega_k dt)` reproduces the truth, so gyro and attitude are consistent by construction. Noise `gyro_noise_dps` (0.05, seeded) and biases `gyro_bias_[xyz]_dps` (0) are added. The values are then quantised to exact 70 mdps counts with error feedback, so rounding does not accumulate as a bias. They saturate at ±2000 dps. |
| Accelerometers | `f_body = R(q)^T (a + (0, 0, g0))` in g. On the pad this is (+1, 0, 0). Noise is 0.003 g on low-g and 0.01 g on high-g. Low-g saturates at ±16 g, which encodes as ±15.99 g. High-g saturates at ±320 g. |
| Baro / temperature | ISA 1976, multi-layer to 85 km, at geopotential altitude `site_alt_msl_m` (default 1400 m) + AGL, with optional `baro_noise_pa` (1 Pa). Baro and IMU temperature are the ISA temperature + `temp_offset_c`. |
| GPS | Lat/lon is the launch site (`lat_deg`, `lon_deg`) offset by north/east metres (111 320 m/deg). Altitude is MSL. `gps_sats` (10) and `gps_fix` are configurable. Battery is constant. |
| Expected transitions | The reference rule model (`sim_model.ReferenceStateMachine`) runs on exactly the encoded values. |

**MS5607 range:** the real sensor's range ends at 10 mbar (1000 Pa, about
31 km ISA). Lower pressures are still sent, but the real sensor could not
measure them. The run summary flags the window as a warning.

**Barometric altitude:** Witness uses the troposphere formula with
1013.25 mbar, so its barometric altitude compresses above 11 km. The
reference model uses the same formula, so the Descent prediction still
matches the firmware rule.

### Bundled CSV (`a3241569-trajectory.csv`, defaults)

**Source data**

- 3595 rows; dt from 0.0001 to 0.5 s; 4 duplicate timestamps merged.
- Apogee is 124 564 ft (37 967 m) AGL at CSV 91.54 s.
- Peak speed is 1351 m/s (Mach 4.2).
- Touchdown is at CSV 376.05 s, at -12.4 m/s.

**Sensor values sent**

- Peak specific force is 74.6 g, on the nose axis, near the end of the
  sustainer burn.
- Low-g saturates for 4.9 s: sim 20.4 to 22.4 s and 32.7 to 35.5 s. There is
  also a 20 ms spike at the 349 s deployment shock. High-g covers all of these.
  High-g peaks at 74.6 g, inside ±320 g.
- Max gyro is 53.4 dps, from the roll in the first burn. Nothing saturates.
- Pressure is below 10 mbar from sim 70.7 s to 153.8 s (83 s). The minimum is
  310 Pa near apogee.

**Run length:** 436.3 s; 43 626 frames.

**Expected transitions (sim s)**

| Transition | Time | Note |
|---|---|---|
| Boost | 20.33 | Pad hold + 0.33 s. |
| Coast | 23.00 | First burnout + 0.5 s. |
| Descent | 139.56 | 5% rule, 28 s after apogee at 111.54 s. |
| Landed | 426.28 | Touchdown at 396.05 s + 30 s. |

Witness has no Coast to Boost transition, so the sustainer at 32.5 s does not
re-enter Boost. This is expected.

### Attitude verification

For CSV runs the tool sends Witness debug start `0x00E0`
(`05 00 00 00 00 00 00 00 E0 0A`) once, as soon as HITL is active, to get
10 Hz 0xE0 packets.

Quaternions are decoded from two packet types. The offsets match
`Software/Horizon/GROUND_STATION_PROTOCOL.md`.

| Packet | Rate | x/y/z/w frame offsets (float32 BE) |
|---|---|---|
| 0xE0 | 10 Hz | 19, 23, 27, 31 |
| 0x02 | 1 Hz | 7, 11, 15, 19 |

Each packet is paired with the truth quaternion of the last frame sent.
`--att-latency-ms` shifts this. Witness reports all-zero quaternions until it
has initialised on the pad. It sets heading arbitrarily, by shortest-arc tilt
alignment. The tool computes these metrics:

- **tilt error**: |tilt from vertical (truth) − tilt from vertical (Witness)|.
  Tilt is the angle of body +X from world +Z. This is heading-invariant.
- **heading offset**: the twist about world Z of `q_w q_t^-1`. It is locked
  when Witness reports Boost, from the last valid packet sent while Witness was
  still in Pad idle. On the pad the vehicle is stationary, so packet latency
  cannot turn the boost roll into heading: while the nose is vertical, roll is
  a rotation about world Z. If no such packet exists, the first valid packet
  after Boost is used. If Boost is never seen, the first valid packet is used.
- **nose error**: the angle between the truth nose and the Witness nose, after
  the heading offset is removed.
- **att error**: the full rotation angle between truth and Witness, after the
  heading offset is removed. This includes roll.

The summary gives max/RMS per expected Witness state and the final error at
the last packet before the run ends.

Pass/fail thresholds apply only when Witness reports non-zero quaternions:

| Threshold | Option | Default |
|---|---|---|
| Final tilt error | `--att-final-tilt` (GUI: "Final tilt max deg") | 5° |
| Final attitude error | `--att-final-error` | off |
| In-flight tilt error, Boost to Descent | `--att-flight-tilt` | off |

If the quaternions stay all-zero, the run reports a warning, "attitude not
available", and does not fail. This happens when the estimator firmware is
not flashed.

The GUI adds a third plot: truth tilt, Witness tilt, tilt error and attitude
error against sim time.

`attitude.csv` columns (Horizon Attitude-tab layout):

| Column | Contents |
|---|---|
| `time_s` | Sim time of the paired truth sample. |
| `packet_id` | Decimal: 2 = 0x02, 224 = 0xE0. |
| `witness_qx..qw` | As reported; all zero while unavailable. |
| `truth_qx..qw` | Same convention as Witness. |
| `flight_state` | Witness state code 0-4 from that packet. |
| `hitl_active` | 0 or 1. |

After these come these columns: `wall_s`, `packet`, `flight_state_name`,
`expected_state`, `valid`, the two tilts, `tilt_err_deg`, `nose_err_deg` and
`att_err_deg`.

### Long runs

The sender keeps an absolute 100 Hz schedule. A 436 s CSV run therefore stays
drift-free, and late ticks are counted. Building a run takes about 2 s. The
GUI plots are decimated to about 1500 points per series.

### Limitations

- Zero wind is assumed, and angle of attack is ignored.
- The truth attitude is idealised. It follows the velocity in flight and is
  held, or swings, after apogee.
- Lateral loads at deployment come only from the trajectory's velocity change.
- Attitude pairing uses receive time. Network latency, and at `--speed` the
  batching, shows up as a small error during fast motion. In the fake run at
  40x the errors are about 0.3° or less.
- Heading is only compared relative to the offset locked at Boost.
- Pressure outside the MS5607 range is sent but is not physical.

## 960 Hz IMU synthesis

CSV runs synthesize the IMU data Witness would really see, on the desktop
(`imu_synth.py`), and send it as 960 Hz `0x0E` frames. This needs numpy, scipy
and soundfile **in the Python that runs the tool**. Without them the run
refuses to start, and the error gives the exact install command for that
interpreter, for example the ESP-IDF venv's Python. Set `imu_synthesis=false`
for the old 100 Hz IMU values instead. Witness runs its filtered acceleration, attitude,
position filter and IMU log on these, at the real rate. The 100 Hz `0x0C`
frames still carry the barometer, GPS, temperature and battery.

Per channel, at the sensor's internal 7.68 kHz:

1. The 100 Hz truth (body specific force and rate) is linearly interpolated.
2. **Vibration** is added to the accelerometers only, from launch, when the
   trajectory has a vibration profile (below). The thrust recording goes on
   body x. The lateral recording goes on y, and the same recording delayed
   0.137 s goes on z. The 48 kHz recordings are scaled by the profile's full
   scale and anti-alias resampled to 7.68 kHz.
3. **Datasheet worst-case errors** (LSM6DSV320X DS14623 Table 3) are applied
   with a random sign per axis, from the seed:
   - zero-rate level ±1 dps;
   - low-g zero-g ±12 mg;
   - high-g zero-g ±1.5 g;
   - gyro sensitivity ±0.3%.

   No accelerometer sensitivity tolerance is given, so none is applied.
4. **White noise** at the datasheet densities is added: gyro 3.8 mdps/√Hz,
   low-g 60 µg/√Hz, high-g 1000 µg/√Hz, which is 0.235 dps, 3.7 mg and 62 mg
   RMS at 7.68 kHz.
5. The value is **clipped at full scale** (±16 g, ±320 g, ±2000 dps), because
   the ADC clips before the digital filter.
6. A **2nd-order low-pass** at the bandwidths Witness configures is applied:
   gyro LPF1+LPF2 407 Hz, low-g LPF2 384 Hz, high-g ~400 Hz. These are
   Butterworth approximations; ST doesn't publish the filter structure.
7. **Every 8th sample is kept** (960 Hz, the FIFO batch rate) and quantized to
   int16 counts.

Parameters (`--set KEY=VALUE`):

| Parameter | Default | Effect |
|---|---|---|
| `imu_synthesis` | `true` | `false` falls back to 100 Hz IMU values in `0x0C` |
| `vibration_profile` | `""` | `""`: the profile made for this trajectory, if there is one; `none`: no vibration; or a profile folder name under `vibration/` |
| `vibration_scale` | 1.0 | 0 disables the vibration |
| `imu_noise_scale` | 1.0 | 1 is the datasheet noise density |
| `imu_offset_scale` | 1.0 | 1 is the datasheet worst-case zero levels |

Synthesis takes about 1.5 s for the 436 s bundled run (418,800 samples). Pad
noise at 960 Hz comes out as low-g 1.22 mg, gyro 0.083 dps and high-g 21 mg
sd. In boost the low-g x saturates and the high-g x spans about −53 to +70 g.

**Vibration profiles belong to one trajectory.** A profile is a folder
`vibration/<trajectory file name without .csv>/` holding `profile.json` and
the recordings. `profile.json` gives the `thrust` and `lateral` file names and
`full_scale_g` (the g that sample value 1.0 represents). A CSV run picks up the
folder named after its trajectory automatically. A trajectory with no profile
folder gets no vibration, unless `--set vibration_profile=<folder>` names one
on purpose.

| Profile | For | Content |
|---|---|---|
| `vibration/eayftrajectory/` | `trajectories/eayftrajectory.csv` | PANDORA EAYF_CONFIG_1 thrust and lateral flight vibration, launch to apogee (89 s; the trajectory's apogee is at 88.8 s), 150.67 g full scale, 25.2 g RMS in its loudest second (`vibration_audio.txt`) |

To add one, create `vibration/<trajectory stem>/`, copy in the two recordings
and write `profile.json` like the existing one. Each run's log line and
`summary.json`/`params.json` record which profile was used. The headless and
GUI runs print it, for example `vibration: profile 'eayftrajectory'` or
`vibration: none (no profile for this trajectory)`.

In the GUI (and the witnessOS HITL tab), `vibration_profile` is a dropdown:
- **Auto (profile for this trajectory)**, the default (`""`).
- **None** (`"none"`).
- Every folder under `vibration/`.

Use it together with the `vibration_scale` field. Both are saved in
`hitl_settings.json`. A blue line under the parameters shows what the choice
resolves to before a run (e.g. `Vibration: Auto -> profile 'eayftrajectory'
x1`) and which profile the last run actually used.

Loading a CSV, predicting and building a run (CSV parsing, 960 Hz synthesis,
`0x0E` encoding: seconds) run on a worker thread with a busy indicator, so the
window stays responsive. Start, Load CSV and the preset list are disabled
while a run is being built or is streaming. Log lines are written in one
batch per 100 ms poll.

**Parachute deployment event** (default off):
- `deployment_spin_dps` adds a pitch rate (above 2000 dps it saturates the
  gyro) from `deployment_delay_s` (1 s) after the attitude hold starts at
  apogee, for `deployment_duration_s` (0.3 s).
- `deployment_shock_g` adds an axial shock over the same window.
- The truth attitude does not follow the tumble. Witness is expected to report
  its attitude as lost (all-zero quaternion) and carry position on the
  barometer and GPS.

## Run outputs and FC logs

Each run directory (`runs/<timestamp>/`) holds:

| File | Contents |
|---|---|
| `params.json`, `sent.csv`, `received.jsonl`, `log.txt`, `nmea.txt`, `transitions.csv`, `attitude.csv`, `summary.json` | As before |
| `truth.csv` | Expected values per 100 Hz tick: phase, expected Witness state, truth position and velocity (ENU from the pad), altitude MSL, body specific force and rate, quaternion, tilt |
| `comparison.csv` | Each Witness `0xE0` packet (10 Hz, while HITL is active) with the truth at the same sim time: observed vs expected position, velocity, altitude, tilt, flight state, and the errors (ENU, horizontal, speed, attitude) |
| `imu960.npz` | The exact 960 Hz counts sent in `0x0E` frames, with time and scales |
| `fc_logs/` | The Witness flash files of the run's session (I, S, P, U and the text log), converted to CSV |
| `comparison_flash.csv` | The 50 Hz state packets from the flash `P*.BIN` matched to truth. Uptime is mapped to sim time through the `0xE0` packets. |

`summary.json` gains `comparison` and `comparison_flash`, which hold the RMS
and max of the up, horizontal, speed and attitude errors per expected phase.
The headless run prints them. To rebuild the comparison files of a saved run:
`python hitl_outputs.py runs/<timestamp>`.

**FC log fetch** (hardware runs; on by default):
1. After the run, the tool sends the export command `05 68 68` over its own
   connection, then closes it.
2. It reads the Witness USB disk on the bench Pi over SSH, using Horizon's
   `witness_download.py` (see the Horizon README for the one-time udev setup).
3. It downloads the newest session's files into `fc_logs/`.
4. It power-cycles the Pi's USB hub 2, so Witness reboots into normal logging.
   **This also resets Iris and Aperture.**

SSH settings are `pi_user`, `pi_key` and `pi_host` in `hitl_settings.json`
(untracked), or the `WITNESS_PI_USER` / `_KEY` / `_HOST` environment
variables. Set `fetch_fc_logs: false`, or pass `--no-fetch-logs`, to skip the
fetch, and `fetch_restart_hub: false` to skip the power cycle.

The fetch is skipped, with a log line, in two cases:
- the run was stopped or aborted (it would otherwise export logs and reset
  boards after you pressed Stop);
- no Pi user is set (SSH would otherwise try your Windows user name and fail,
  leaving Witness in export mode).

If Witness is ever left in export mode, power-cycle the hub
(`sudo uhubctl -l 2 -a 2` on the Pi) or replug it.

Bench results, 2026-11-09 (`a3241569-trajectory.csv`, worst-case noise,
offsets and vibration):

| Run | Transitions | Up RMS, flash 50 Hz (boost / coast / descent) | Horizontal RMS, descent | Final attitude error |
|---|---|---|---|---|
| Nominal, before the bias fix (`runs/20261109-120942`) | all within 0.02 s | 3.8 / 4.8 / 2.1 m | 2.2 m | 10.6 deg (FAIL) |
| Nominal, with the bias fix (`runs/20261109-123242`) | all within 0.02 s | 3.8 / 4.8 / 2.1 m | 1.9 m | 2.2 deg (PASS) |
| Deployment, 2500 dps + 20 g (`runs/20261109-121755`) | all within 0.02 s | 4.5 / 7.7 / 3.3 m | 3.3 m | attitude lost at deployment (sim 112.57 s), as designed |

Notes on these runs:
- All 418,800 samples (52,350 `0x0E` frames) were sent, and Witness's
  attitude ran at 960 Hz.
- In the nominal runs Witness never logged a gyro saturation.
- In the deployment run it logged `gyro saturated ... attitude tracking
  stopped`, and from then on `IMU ignored (attitude lost)`, with position
  carried by the barometer and GPS.
- With the bias fix, the tilt error RMS is 0.96 deg on the pad, 1.05 deg in
  coast, 1.67 deg in descent and 2.15 deg landed (before the fix: 0.94 / 1.25 /
  6.92 / 10.56). About 1 deg of that is the worst-case 12 mg accelerometer
  offset, visible already on the pad.
- The live `0xE0` comparison (`comparison.csv`) shows larger up errors in boost
  and coast than the flash comparison, because it is matched by receive time
  and so includes the link latency. `comparison_flash.csv` is matched by
  Witness uptime and is the better measure.

The bias fix (2026-11-09): the pad gyro bias used to be the per-axis median of
counts quantized at 0.07 dps. With about 1.2 counts of noise, that left up to
about 0.035 dps of residual bias, which drifted the nominal run 10.6 deg by
landing. It is now the median refined by the mean of the samples within 3
robust sigmas of it, clamped to one quantization step (see the firmware
handoff, "Attitude estimator").

## Safety rules

- **Run HITL on the pad only.** Witness accepts HITL ENTER only over USB and
  only in Pad Idle, and it ignores sensor input while HITL is active. Never use
  HITL with a vehicle that is armed, or anywhere other than the bench.
- **Witness exits HITL by itself if no 0x0C frame arrives for 1000 ms**, for
  example when the tool stalls, the network drops or the window closes. It then
  returns to real sensors.
- **Flash logging records simulated data during the run.** A simulated launch
  opens the logging gate. The sensor log for that boot session holds the
  simulated flight, so treat it as test data.
- After a simulated flight, Witness stays in Landed or whatever state it
  reached, and ENTER is rejected outside Pad Idle. Reboot Witness before the
  next run. The tool won't send flight-state commands.
- The tool can only encode these frames:
  - HITL ENTER `0x4849`;
  - HITL EXIT `0x4858`;
  - Witness debug start `0x00E0`, which the tool sends once per CSV run;
  - 0x0C sensor frames and 0x0E IMU batch frames.

  The one exception is the end-of-run FC log fetch. It sends the export
  command `05 68 68` directly, after the run, and then power-cycles Pi USB
  hub 2 (`--no-fetch-logs` disables it).

  `encode_command` refuses every other value, including mass storage
  `0x6868`, log erase `0x6869` and flight-state commands `0x7300` to
  `0x7304`.

## Protocol summary

The authoritative layout is in `Software/Witness/softwarev1/main/comms.h`. The
rest of the wire protocol is in `Software/Horizon/GROUND_STATION_PROTOCOL.md`.

- **Framing:** frames are `ID | fixed payload | 0x0A`, big-endian. 0x0A can
  appear inside a payload, so frames are found by fixed length per ID and the
  EOF is checked at the last byte only. On a mismatch, the decoder resyncs one
  byte at a time.
- **Frame lengths:** 0x01 = 23, 0x02 = 65, 0x03 = 19, 0x05 = 10, 0xE0 = 103,
  0xE1 = 14, 0xFF = 8. The 0x0C echo (47) and the 0x0E echo (150, fake only)
  are ignored.
- **0x0E HITL IMU batch:** 150 bytes. It holds a u32 first-sample index, then
  8 samples of int16 low-g xyz, high-g xyz and gyro xyz (counts as in 0x0C),
  then EOF. The samples are 960 Hz. Frames are sent by sim time, before each
  tick's 0x0C.
- **What else is on the stream:** ESP-IDF log lines (ANSI colour codes are
  stripped), raw NMEA, and an echo of every byte the host sends.
- **Status:**
  - byte 0 holds the flags: heartbeat, low-g, gyro, IMU temp, NAND fail,
    high-g, baro, flash-log fail;
  - byte 1 bits 0-2 hold the flight state; bit 3 is GPS lock and bit 4 is
    HITL active.
  - Frames with reserved bits 5-7 set are rejected as false matches.
- **Command frame:** `05 00 00 00 00 00 00 HH LL 0A`. ENTER is `48 49`, EXIT is
  `48 58`.
- **0x0C HITL input:** 47 bytes. Payload offsets:

  | Payload offset | Field |
  |---|---|
  | 0 | Status (0) |
  | 2 | Sim ms, u32 |
  | 6 / 8 / 10 | Low-g xyz, i16, 0.488 mg |
  | 12 / 14 / 16 | High-g xyz, i16, 10.417 mg |
  | 18 / 20 / 22 | Gyro xyz, i16, 70 mdps |
  | 24 | IMU temp, i16, 25 + raw/256 C |
  | 26 | Pressure, u24 Pa |
  | 29 | Baro temp, i16, centi-C |
  | 31 / 34 | Lat / lon, i24, 1e-4 deg |
  | 37 | GPS alt, f32 m MSL |
  | 41 | Sats, u8 |
  | 42 | GGA fix quality, u8 |
  | 43 | Battery, u16 mV |
  | 45 | EOF |

  Values are saturated to the int16 range.

## Tests

From `Software/HITL`:

```
py -3.14 -B -m unittest discover -s tests -v
```

The end-to-end tests start `tests/fake_witness.py` on 127.0.0.1. The fake
does the following:

- echoes input;
- honours ENTER, EXIT and debug start;
- auto-exits after 1 s;
- runs the reference state machine;
- emits 0x01, 0x02, 0xFF and, after debug start, 0xE0 frames, mixed with log
  text, NMEA and junk;
- after 2 s on the pad, aligns a quaternion to the measured specific force
  (shortest arc plus an arbitrary 37° heading), then integrates the received
  gyro with the same convention. It reports the result in 0x02 and 0xE0, so
  the comparison path is exercised. `attitude=False` keeps the quaternion all
  zero.

`tests/synthetic_csv.py` writes small test CSVs: ft or m, with duplicates,
`nan` values and an optional missing velocity column. The CSV end-to-end test
runs the bundled file at `--speed 40`, which takes about 11 s. To make these tests fast, the fake uses the frame sim time as its
clock and the tests run at `--time-scale 20`, a hidden option. **The real
Witness clocks transitions with its own timer, so always use the default scale
of 1 against hardware.** The tests never touch the network beyond localhost.
