# Antenna pointer: design

Status: investigation and design only. Nothing here is wired into the GUI.
Prototype maths: [antenna_pointer_math.py](antenna_pointer_math.py), tests in
[tests/test_antenna_pointer_math.py](tests/test_antenna_pointer_math.py).

Goal: a two-axis CNC-style mount (X = azimuth in degrees clockwise from true
north, Y = elevation in degrees above the horizon) driven by G-code from the
most trusted current rocket position, to keep a ~30 to 40 degree beamwidth
900 MHz Yagi (and the rubber duck, which needs no pointing) on the target.

## 1. What exists today (findings that shape the design)

| Source | Where it is decoded | Position fields | Quality fields | On the JSON feed? |
|---|---|---|---|---|
| Witness `0x02` state | `witness_protocol.py` | `latitude`, `longitude` (0.0001 deg counts, about 11 m), `gps_altitude`, `kalman_altitude`, `velocity` (3 floats, filter frame), `position` | `gps_satellites` (63-byte form only), `status` (GPS lock bits) | Yes, source = monitor name |
| Witness `0xE0` debug | same | same lat/lon/alt, `gps_time`, `velocity` | none | Yes |
| Aperture `0xF0` ground_station_status | `converter.parse_ground_station_status` | Aperture's own LC86G fix, 1e-7 deg, `gps_altitude` | `gps_fix_quality`, `gps_satellites`, `gps_hdop`, `gps_fix_age_ms`, **`rssi`** (last valid LoRa RSSI) | Yes |
| Featherweight tracker | `featherweight.py` text `GPS_STAT` | `latitude_deg`, `longitude_deg`, `altitude_ft`, velocity fps + heading, vertical fps | `fix_type`, `satellites_total`, sats over 24/32/40 dB, RX_NOMTK `rssi_dbm`/`snr_db` | **No** (serial `FeatherweightReceiver` -> GUI only) |
| AIM tracker | `aim_mqtt.py` | `gnss_payload/v1`: lat, lon, altitude, `satellites_used` | `rssi_payload/v1`, `received_packet_signal_quality_payload/v1` (rssi, snr) | **No** (MQTT to Horizon's embedded broker on 1883) |

Key facts:

- The Horizon feed (`witness_link.WitnessPublisher`, schema `horizon.witness.v1`,
  `127.0.0.1:8766`, Tailscale `:8767`) is newline-delimited JSON with
  `source`, `session`, `sequence`, `received_at` (host epoch), `replay` flag
  and the decoded `packet`. New clients first get a `replay: true` snapshot
  of the latest packet per (source, type); a pointer must ignore replays for
  freshness (or age them from `received_at`). Clients cannot send anything:
  inbound data disconnects them. That is a strength (a pointer bug cannot
  affect Witness) and a constraint (no command path needed or wanted).
- The feed carries only what `MonitorPanel.on_decoded_packet` sees: Witness
  and Aperture. Featherweight (exclusive serial port) and AIM (MQTT) are not
  republished. The AIM broker is multi-subscriber, so a pointer can subscribe
  to MQTT directly. Featherweight cannot be read by two processes, so it
  needs a relay (see section 2.4).
- Witness lat/lon are quantised to 0.0001 deg (about 11 m N-S, about 9 m E-W
  at 33 N). At 300 m range that is up to about 2 degrees of pointing error,
  fine for a 30 to 40 degree beam, but it makes the pointing "steppy" at
  close range. Witness `position`/`velocity` are Kalman ENU relative to the
  filter's GPS origin and are smoother; they are usable for interpolation
  once the origin (pad) is known (`pad_reference.py` already tracks it).
- `pad_reference.py` already implements a prioritised, de-hopping source
  selector (manual > Featherweight base > Aperture > Witness pad mean >
  tracker). The pointer's ground-station selector should reuse that idea
  (and, in-process, that class).
- `geodesy.py` already provides WGS-84 `geodetic_to_enu`, `enu_to_geodetic`,
  `enu_range_bearing`. Look angles need nothing new beyond elevation, added
  in the prototype (`look_angles`).

## 2. Architecture

### 2.1 Recommendation: a separate process

`antenna_pointer.py` (headless core, CLI) subscribing to the Horizon feed,
plus an optional small status GUI or a thin Horizon tab that only displays
what the pointer process publishes. Reasons:

1. Robustness. The pointer owns a serial port or TCP socket to a motor
   controller and runs a real-time loop; Horizon's GUI has a stall watchdog
   (`stall_watchdog.py` logs when the GUI thread is blocked over 1 s), and the
   Witness feed was deliberately designed so a slow client never stalls
   telemetry. Motion control must not depend on the Qt event loop, a
   spectrogram load, a map tile fetch, or a viewer analysis freezing.
2. Safety. If Horizon crashes or hangs, the mount should hold or park under
   its own timeout, not keep following stale data; a standalone process with
   a watchdog and `?`-polling does that cleanly. The reverse also holds: a
   pointer fault cannot take down telemetry recording.
3. Testability. Pure functions plus a fake feed and a fake controller (a
   socket or `serial_factory` like `FeatherweightReceiver` already uses)
   make the whole loop runnable in `unittest` with no Qt.
4. Placement. It can run on the laptop next to the antenna (feed on
   127.0.0.1), on the bench Pi beside the mount (feed over Tailscale `:8767`,
   which already has keepalive and large buffers for flaky links), or on the
   Pi with the serial/USB link to the controller short and the laptop only
   for display. The Pi already runs `witness_pi_agent.py`.
5. Same machine sharing. Because the feed has no command path, the pointer
   writes nothing back; its status goes out on its own tiny feed (below).

Trade-off: two processes to launch. Mitigate with an optional "Start pointer"
helper in Horizon that just spawns the same script, mirroring how it already
starts the MQTT broker and HITL helpers, and have the pointer fail safe when
the feed disappears.

### 2.2 Data path

```
Witness -> LoRa -> Aperture --serial--> witnessOS/Horizon MonitorPanel
                                         |-- WitnessPublisher 127.0.0.1:8766 / Tailscale :8767
Featherweight (serial) -> FeatherweightReceiver -------+
AIM -> MQTT (embedded broker :1883) ------------------+
                                                       v
                                          antenna_pointer.py
                                          (fusion -> look angles -> motion -> G-code)
                                                       v  serial / telnet
                                          GRBL / grblHAL / FluidNC
                                                       v
                                          its own status feed (NDJSON, 127.0.0.1:8768)
                                                       v
                                          Horizon "Pointer" tab (display + big STOP/hold)
```

### 2.3 Source plumbing

- Witness and Aperture: consume the existing feed.
- AIM: subscribe to `entacore/aim/1234/#` on the broker directly
  (read-only, as Horizon does).
- Featherweight: add one small, additive change in `MonitorPanel`/the unified
  app: publish Featherweight records on the same feed (new `source` string, the
  packet dict already carries `packet`, `latitude_deg` etc.). `decode_record`
  already accepts arbitrary `source` and packet names, so existing clients are
  unaffected. The same hook could publish AIM GNSS/RSSI as normalised packets
  (`aim_gnss`, `aim_rssi`) so the pointer needs only one input. This is the
  one required change to existing code; it is deliberately not made here
  (other agents are editing the protocol files).
- Per-source adapters inside the pointer turn any packet into one
  `Observation(source, t_rx, lat, lon, alt, vel_enu | None, quality)`. Keep
  adapters pure and unit-tested (fixtures from `tests/` captures).

### 2.4 Status output

The pointer serves its own receive-only NDJSON feed (same pattern and code
style as `WitnessPublisher`): selected source, target, commanded and
reported az/el, controller state, error, link RSSI. Horizon (or a tiny
standalone PyQt window) only displays; the STOP/hold/jog buttons talk to the
pointer through a small authenticated-by-loopback command socket (loopback or
Tailscale only), never to the mount directly.

## 3. Source fusion: "most trusted current location"

### 3.1 Per-source trust

Each observation gets a trust record; the selector picks the best usable
source and exposes why. Gates, in order (any failure removes the source from
consideration, with the reason shown in the UI):

1. Freshness: age = now - `received_at` (ignore `replay: true` snapshots
   except to seed an "old" fix). Staleness limits: Witness 0x02 about 3 s
   (it is 1 Hz over LoRa; allow 2 missed), Featherweight about 5 s, AIM about
   3 s. A source more than about 10 s old is only usable by the dropout logic.
2. Fix validity: finite, in range, not 0/0 (`geodesy.valid_lat_lon`).
   Witness: GPS lock status bit set and `gps_satellites >= 4` (when present;
   legacy form has none). Aperture: `gps_fix_quality > 0`. Featherweight:
   `fix_type >= 2` (3 for altitude). AIM: `satellites_used >= 4`.
3. Plausibility: implied speed from the previous accepted fix from the same
   source below a bound (for example 700 m/s), and agreement within a
   tolerance with the current estimate when two sources are fresh (outlier
   rejection; a single source jumping several km is dropped, a persistent
   jump is accepted after N consistent fixes).
4. Altitude sanity: GPS altitude within a window of the Kalman altitude or
   previous estimate; large disagreement flags the GPS as lagging or wrong.

Score for ranking among passing sources (higher better): freshness (exponential
in age), 1/HDOP or sat count, a prior by source (resolution and update rate:
Featherweight tracker and AIM have 10 Hz-ish/own-GPS positions, Witness 1 Hz
and 11 m quantised, but Witness is the thing we care about), and a penalty
for known lag.

### 3.2 Receiver lag

Witness' GPS receiver lags under high g (documented in the pad-freeze and
Kalman weighting work; see the recent "Kalman GPS weighting for receiver lag"
commit). During boost and shortly after:

- Prefer the Witness Kalman solution for altitude (`kalman_altitude`,
  `position`) over `gps_altitude`; use GPS lat/lon only after sanity-checking
  against Kalman-integrated horizontal position when the filter has an
  origin. In the first seconds after burnout, elevation matters most and
  horizontal drift is small, so Kalman-vertical + GPS-horizontal is a good
  hybrid.
- Treat GPS position as delayed by a source-specific lag (default 1 to 2 s
  under boost, 0.3 s coast; tune from HITL replay logs) and extrapolate
  forward with velocity.
- Feed `status` flight state (Boost/Coast/etc.) into the lag model.

### 3.3 Prediction to the command time

For the selected observation:

```
t_target = now + t_cmd_latency      # serial write + mount settling, ~0.2-0.5 s
dt       = t_target - t_fix         # includes the measurement lag and link latency
p_hat    = p_fix + v * dt           # v from the source (Featherweight vel/heading,
                                    #   Witness velocity); else d(p)/dt from the last 2-3 fixes
```

`antenna_pointer_math.predict_lla` does the constant-velocity step. Cap dt
(for example 5 s) and cap acceleration assumptions; vertical velocity decays
in coast (optionally apply drag/gravity ballistic during coast using the
existing `test_ballistic_replay.py` model).

### 3.4 Hysteresis and switching

- Switch to a higher-ranked source only after it has been better for about
  2 s or the current one fails a gate; fall back immediately on a hard gate.
- Never blend raw positions from different GPS receivers directly (offsets
  of metres to tens of metres appear as pointing jumps); instead blend the
  look-angle with a short smoothing filter (rate-limited, see 4.4) on switch.
- Optionally run one small filter (alpha-beta or fixed-lag Kalman in ENU) that
  ingests every accepted observation with per-source variance
  (sigma = base * HDOP-ish, bigger for Witness' 11 m quantisation); this is
  the clean generalisation of the above and removes most switching logic.
  Start with selection + hysteresis (simple, explainable) and graduate.

### 3.5 Dropout behaviour (state machine)

| Dwell | State | Behaviour |
|---|---|---|
| fresh fix | TRACK | Command predicted look angle |
| 0 to ~5 s since last fix | COAST | Keep extrapolating on last velocity; UI amber |
| ~5 s to ~30 s (configurable) | BALLISTIC | Propagate with a coast/descent model (gravity + drag, drogue/main events from last state); widen allowed error; slower slew |
| beyond | HOLD / SEARCH | Hold last good direction; if a signal metric is available start a search (section 8); else stay and alarm |

Coasting into a "parked" estimate must never drive the mount through limits;
every state uses the same limit checks.

## 4. Geometry

### 4.1 Ground station position

Priority (reuse `pad_reference.py`'s pattern, but for the antenna, which is not
necessarily the pad): manual surveyed entry (best; a one-time phone/RTK fix of
the mount pivot) > a dedicated GPS on the mount (Featherweight base station or
Aperture LC86G if physically at the mount) > Aperture `0xF0` GPS (if the
Aperture is at the mount) > pad reference. Show which one is used and the
horizontal distance between candidates; a mount/Aperture offset of 20 m is
meaningless at 5 km but not at 300 m. Mount altitude from the same source;
keep a manual height-of-phase-centre offset (a few metres matter only at very
short range).

### 4.2 Look-angle maths

ENU about the ground station (WGS-84 via `geodesy.geodetic_to_enu`):
`az = atan2(E, N) mod 360`, `el = atan2(U, hypot(E, N))`, `range = |ENU|`.
This is exactly `antenna_pointer_math.look_angles`. At rocket ranges
(under about 30 km) a flat ENU with curvature handled by the ECEF-based
transform is more than adequate; refraction is below 0.05 degrees at
these elevations and ignored. Use geometric height (MSL both ends, or both
AGL against the same datum; do not mix GPS ellipsoidal and barometric).

### 4.3 True north and declination

Azimuth is true north. If the mount's zero was set with a compass, use
`mount_command(..., declination=)` (magnetic = true - declination, declination
east positive). Better: calibrate az zero against a known bearing
(sun, a surveyed landmark, or another GPS fix of a person walking away) and
store a signed `az_offset`; also an `el_offset` (level the base with a
bubble/inclinometer; an IMU is cheap). Store calibration in a JSON beside
`ground_settings.py` with a "calibrated at" timestamp.

### 4.4 Singularities and wrap

- Zenith keyhole: when `el >= ~85 deg` azimuth is ill-conditioned; freeze
  azimuth (`in_keyhole`) and track elevation only. The rocket spends little
  time near zenith from a ground antenna 1 km or more downrange, but close-in
  pads and descent can pass near it. The antenna beam (30 to 40 degrees)
  tolerates frozen azimuth near zenith easily; also limit elevation to at
  most 90 degrees (or 95 if the mount can flip, which a 2-axis az/el mount
  usually cannot).
- Azimuth range: cable wrap limits the accumulated rotation (for example
  -180..+180 or -270..+270). `choose_azimuth` returns the equivalent angle in the
  limits nearest the controller's reported position, or the nearest limit with
  `in_range=False` (the UI should say "target outside travel"). Keep reported
  position as the truth, not an internal counter.
- Near the wrap seam (for example target moving through 180 degrees with
  limits -180..180) the mount must unwind through the long way; detect an
  imminent seam crossing (target within ~20 degrees) and pre-position if
  slack allows, or accept a brief loss since the beam is wide.
- Elevation below horizon: clamp at the mechanical minimum (0 or slightly
  negative) and flag "below horizon"; geometric elevation below 0 is possible
  at long range and over terrain.

## 5. G-code controller

### 5.1 Firmware comparison

| | GRBL 1.1 | grblHAL | FluidNC | Marlin |
|---|---|---|---|---|
| Hardware | Arduino Uno/Nano (8-bit) | ARM/ESP32/etc., lots of boards | ESP32 | 3D-printer boards |
| Live target updates | `$J=` jog + `0x85` cancel | same, plus more realtime | same as GRBL, plus WebSocket/telnet | no jog command; planner queue only |
| Real-time status | `?` -> `<Idle|MPos:..|FS:..>` | same | same, also auto-report | `M114`, polling is slow |
| Streaming | character counting, 127-byte RX buffer | same, larger buffers | same + network | `ok`-per-line |
| Network | no | optional (Ethernet/Wi-Fi plugins) | built in: Wi-Fi, telnet, WebUI, config by YAML | optional |
| Limits/homing | yes (`$20`, `$21`, `$H`) | yes | yes | yes |
| Fit | OK, simple | good | **best for this**: cheap, wireless option, easy axis config | poor for continuous targets |

Recommendation: FluidNC on an ESP32 CNC shield if the hardware is not
chosen yet (YAML-configured axes, network or USB, same protocol as GRBL);
grblHAL if a specific board is already owned; plain GRBL on an Uno works
but is limiting. Avoid Marlin for live tracking. The pointer should talk the
common GRBL subset so these are interchangeable behind a thin `Controller`
interface (serial, telnet).

### 5.2 Units

Treat X and Y as degrees: set `$100`/`$101` (steps/mm) to steps per degree
(gear ratio x microsteps x steps per rev / 360), `G21` (mm, harmless), and
feed in deg/min. Examples: a 1.8 degree motor, 16x micro-stepping, 20:1
reducer = 200 x 16 x 20 / 360 = 177.8 steps/deg. Set max rates
(`$110`/`$111`) and accelerations (`$120`/`$121`) in deg/min and deg/s^2 to
what the mount can follow; a Yagi on a pan/tilt can safely slew a few tens
of deg/s only with sufficient torque and a rigid mount; a conservative
10 to 30 deg/s is plenty for rocket tracking.

### 5.3 Commanding smoothly

Rocket motion in az/el is slow (a few deg/s typical, rising on close
overhead passes); the beam is 30 to 40 degrees. The simplest robust scheme
is: absolute jog (`$J=G21G90X..Y..F..`) at 5 Hz (or 10 Hz), feed set to
reach the target in about one cycle plus margin (`feed_for_move`). Because
each jog is one planner move, a newer target replaces the previous
one only if the old one finished or was cancelled:

- Slow case (target moves less than ~one cycle of travel): just queue the next
  jog after `Idle`/low-queue; GRBL jog executes immediately and does not queue
  behind G-code.
- Fast case: send `0x85` (jog cancel), which decelerates and flushes, then
  the new `$J=`. Rate limit cancels to about 5 Hz so the mount does not
  chatter.
- Alternative for GRBL-family when a stream of G1 moves is acceptable:
  character-counting streaming with a short look-ahead (never more than
  about 0.5 s of motion in the planner, or the mount lags the rocket); this
  is only needed for planned search patterns, not tracking.
- `G0` vs `G1`: `G0` uses max rate, so for coordinated azimuth/elevation
  keep `G1`/jog with explicit F, which also gives the smooth path
  (`feed_for_move` uses the vector length because GRBL feed applies to the
  Cartesian vector).

Position feedback: poll `?` at 5 to 10 Hz (or rely on FluidNC auto-report
`$Report/Interval`); parse `MPos`/`WPos` (`parse_grbl_status`). Use reported
angles, not commanded, to choose the azimuth branch and to detect following
error and stalls (lost steps are invisible on open-loop steppers; consider
an encoder, a limit-switch re-home at park, or a magnetometer/IMU
cross-check as a later phase).

### 5.4 Limits, homing, alarms

- Hard limit switches at the elevation endpoints and (if wrapping is
  restricted) at the azimuth ends; soft limits (`$20=1`) with machine
  coordinates set so that X range = the allowed cable travel and Y range =
  0..90 (or the mount's real range). Homing (`$H`) on power-up to define
  zero; then apply the calibration offsets (they live in the pointer, not in
  firmware, so recalibration needs no re-flash).
- Alarm handling: on `ALARM:n`/`error:n` stop sending, show the code, require
  the operator to `$X` (unlock) or re-home; never auto-clear alarms.
- Realtime commands: `!` feed hold for STOP, `~` resume, `0x18` soft reset,
  `0x85` jog cancel. STOP in the UI must send `!` (and cancel pending
  pointer output) first, `0x18` only if the operator asks.
- Command timeouts: if no `?` reply for about 2 s, mark the controller link
  dead and stop commanding; on FluidNC consider its safety door/feed-hold
  pin for a hardware E-stop.

### 5.5 Update rate and rate limiting

5 Hz command rate; 10 Hz status poll. Apply an angular rate limit
(`rate_limit`) to the commanded target (for example max 20 deg/s az,
10 deg/s el) so a source switch, outlier or seam flip cannot whip the mount.
Dead-band: do not command if the new target differs from the last by less
than about 0.3 degrees (the beam is wide; this saves motor wear and noise).
Smooth the target with a small low-pass over the last 2 to 3 s of velocity-
corrected observations.

### 5.6 Transport

Serial (115200 USB-CDC) is the default; the FluidNC telnet (port 23) or
WebSocket path allows the controller beside the mount with Wi-Fi to the
laptop or Pi, at the cost of added latency and a lost link needing the dead-
man (mount stops/holds when commands cease; FluidNC has no built-in
motion timeout, so the pointer should hold the mount with `!` when it loses
the feed, not coast on stale target).

## 6. Safety and UX

- Modes: OFF -> MANUAL -> TRACK; SIM/dry-run prints the G-code and simulated
  reported angles without opening a port (the same `Controller` interface
  with a fake backend) and is the default until explicitly armed.
- Controls: arm/disarm, large STOP (feed hold, then disarm), manual jog (arrow
  keys with a fixed 1 and 5 degree step and hold-to-repeat), go-to az/el,
  park (stow elevation/azimuth), set az/el offset from "currently pointing
  at known target", home, unlock.
- Always-on limits: pointer clamps to configured az/el ranges before
  formatting any G-code, independent of firmware soft limits.
- Wind/people safety: audible/visible warning before a move from
  parked or after unlock; max-speed cap; a "dead-man" so that if the pointer
  process dies the controller holds (heartbeat via periodic `?` plus
  firmware idle timeout where available; otherwise accept hold-on-silence).
- Logging (NDJSON next to `logs/`): every cycle: selected source and reason,
  per-source ages and scores, ground-station position, predicted LLA, look
  angles, mount command (after offsets, branch, limits), the actual G-code line,
  reported MPos, following error, controller state, link RSSI. This is
  what makes post-flight tuning (lag constants, deadband, rates) possible, and
  matches the project's existing record-everything habit.
- UI: a "Pointer" tab or standalone window: source badge and age, a small
  polar plot (az/el of target and mount, beam cone, limits wedge), numbers for
  az, el, range, rate, following error, controller state, RSSI sparkline, and
  a visible reason when not in TRACK (stale, outside travel, alarm, SIM).
  Built in the existing `qt_widgets` style and updated by a throttled timer
  (like other tabs) so it never blocks.

## 7. Future: signal-driven search on dropout

Data available for a link-quality metric:

- Witness downlink: Aperture `0xF0` `rssi` (last valid LoRa RSSI, signed byte).
  Currently it is only updated when a packet is received, about 1 Hz, so it
  is both slow and absent during the very dropout we want to fight. Needed:
  a faster, always-on measurement from Aperture (for example a periodic
  RSSI/noise-floor reading of the channel even without a decoded packet; the
  SX1276/RA-01 can give instantaneous RSSI via `RegRssiValue` in continuous
  receive and RSSI-per-packet via `RegPktRssiValue`) reported at 5 to 10 Hz
  in `0xF0` or a new packet, with a "valid packet" flag and the receive
  timestamp. Also antenna-selection info if two antennas are used.
- Featherweight `rssi_dbm`/`snr_db` (RX_NOMTK) and AIM RSSI/SNR (MQTT).
  These are on separate receivers and radios; use them as a cross-check of
  the same antenna geometry only if they share the mount.
- Expected size of the effect: a 13-element Yagi has about 14 dBi and 30
  to 40 degree beamwidth; the 3 dB point is about half that off axis
  (about 15 to 20 degrees); a search pattern with a radius of about 10 degrees
  loses up to about 3 dB, which a LoRa link margin can often absorb.

Strategy (ordered by effort):

1. Prediction only (this design).
2. Raster/spiral around the predicted position when the link is lost:
   conical scan of half-angle about 5 to 8 degrees at a few deg/s, grown
   slowly (Archimedean spiral to about 20 degrees), re-centred as soon as a
   decoded packet gives a new position. Lets the mount recover if the
   estimate drifts or the mount is miscalibrated.
3. Step-track / hill-climb: when RSSI is available at >= 2 Hz and the link is
   up, dither az/el by a fraction of the beamwidth and move toward higher
   RSSI (gradient on a smoothed RSSI), always bounded to within N degrees of
   the predicted pointing, with prediction as the prior (a simple fusion:
   commanded = predicted + bounded correction). Conical scanning
   (constant small offset rotating around the axis) gives gradient in both
   axes per revolution.
4. Calibration use: while the link is good and the rocket is on the pad,
   mount-to-GPS mismatch can be estimated (RSSI peak vs predicted bearing) and
   fed back to the az/el offsets.

Needed telemetry changes: faster Aperture RSSI (above), a link-timestamped
RSSI field, and optionally mount-pointing in 0xF0 for post-flight analysis.
Not in scope for the first versions; leave extension points: the pointer's
final stage is `target = predicted + search_offset(state)` with
`search_offset == 0` initially.

## 8. Phased plan

| Phase | Content | Effort |
|---|---|---|
| 0 (done here) | Look-angle, wrap, rate, jog formatting, status parse math + tests | done |
| 1 | `antenna_pointer.py` core: feed client (reuse `decode_record`), adapters for Witness 0x02/0xE0 and Aperture 0xF0, freshness/validity gates, selector with hysteresis, prediction, fake controller, SIM mode, NDJSON log; tests with recorded feed/captures | 3 to 4 days |
| 2 | Real GRBL/FluidNC backend (serial + telnet), `?` polling, jog/cancel, limits, alarms, STOP, manual jog, calibration file, bench with a stepper | 3 to 4 days plus bench time |
| 3 | Republish Featherweight and AIM through the feed (small change in Horizon), adapters, receiver-lag handling, plausibility cross-checks, source-fusion filter | 2 to 3 days |
| 4 | Status feed + Horizon "Pointer" tab (polar plot, source/age/RSSI, STOP) or standalone window | 2 to 3 days |
| 5 | Dropout state machine (coast, ballistic), search patterns (spiral/conical), faster Aperture RSSI firmware, step-track | 1 to 2 weeks incl. firmware and field testing |
| 6 | Closed-loop checks: encoder or IMU, auto sun-calibration | optional, 1 week |

Test strategy: replay logged feed streams (the `logs/` BIN/CSV, HITL runs)
through the pointer in SIM mode and compare look angles against a known
trajectory; add a mount simulator (first-order lag + rate limits) to evaluate
pointing error vs beamwidth for different command rates and lags.

## 9. Open questions

1. Controller firmware and board (GRBL Uno, grblHAL, FluidNC/ESP32, other)?
   Serial/USB or Wi-Fi?
2. Mechanics: azimuth travel (continuous slip ring vs cable wrap limit),
   elevation range (0 to 90? to 180 flip?), gear ratios, motor/driver, max safe
   slew, homing switches, backlash, payload (Yagi weight, wind loading).
3. Where is the mount relative to the Aperture/Featherweight base and the pad?
   Is there a survey-grade fix for the pivot?
4. Which antennas go on the mount: the Yagi only, or Yagi plus the Aperture
   LoRa antenna? Which receiver is the pointing target (Witness at 900 MHz,
   AIM, Featherweight at different frequencies)? Can several share the mount?
5. How is zero north set (compass, sun, landmark)? Is an IMU/encoder planned?
6. Where does the pointer run: laptop, Pi near the mount, or both?
7. What should happen on dropout: hold, ballistic, or search; how aggressive
   is acceptable near hardware limits and bystanders?
8. May we add Featherweight/AIM to the Horizon feed (a small additive change
   to the monitor), and extend the Aperture `0xF0` for faster RSSI?
9. Priority between sources when they disagree: always Witness, or the
   best-quality fix?
