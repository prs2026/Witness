# Horizon Unified Ground Station

Horizon is the desktop ground-station software for viewing Witness, Featherweight GPS, and AIM XTRA telemetry together. It includes a launch dashboard, a layout editor, an attitude view, a map/orientation visualizer, logging, and an embedded MQTT broker. The UI is PyQt6 with pyqtgraph for every plot. This folder contains Python applications, not the embedded firmware that runs on the flight computers.

Start `horizon.py` for the unified display. Witness hardware input comes from the **Witness monitor** tab of witnessOS (`Software/witnessOS/witnessos.py --tab monitor`); the standalone `main.py` window is retired (running `main.py` just prints where the monitor went). AIM BASE still connects to AIM hardware and publishes its telemetry; Horizon replaces the separate MQTT broker application, not AIM BASE.

## Data connections

| Input | Data path | Where to configure it |
|---|---|---|
| Witness | Serial or TCP → witnessOS Witness monitor → local JSON telemetry feed → Horizon | witnessOS Witness monitor tab for hardware; Horizon Witness tab for feed |
| Featherweight GPS | Serial COM port → Horizon | Featherweight GPS tab |
| AIM XTRA | AIM BASE → MQTT broker → Horizon subscriber | AIM BASE publisher settings and Horizon AIM XTRA MQTT tab |

Horizon’s telemetry clients are receive-only: they do not send flight commands to Witness, Featherweight, or AIM hardware. The Witness monitor (witnessOS tab, built on `main.MonitorPanel`) has separate command controls that can transmit to connected serial/TCP devices. The embedded broker routes MQTT messages between clients; it does not translate them into hardware commands.

## Installation and startup

The application has been tested with Python 3.14 on Windows. Run these commands in this folder using PowerShell:

```powershell
py -3.14 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe horizon.py
```

If `.venv` already exists, use it and run the dependency-install command after upgrading. Dependencies include `PyQt6`, `pyqtgraph`, `PyOpenGL` (3D attitude; a 2D projection is used without it), `numpy`, `pyserial`, `paho-mqtt` and `amqtt`.

For Witness hardware, start witnessOS and use its **Witness monitor** tab:

```powershell
.\.venv\Scripts\python.exe ..\witnessOS\witnessos.py --tab monitor
```

Keys in Horizon: **F9** Visualizer, **F11** fullscreen (**Escape** leaves), **Ctrl+P** pause Flight View, **Ctrl+1..6** switch tabs, **Ctrl+Q** quit. In the Attitude tab **Space** plays/pauses a replay and **Ctrl+O** loads a CSV.

Either app may start first. Horizon retries its enabled inputs when unavailable. A connected device with no fresh packets is not the same as a healthy telemetry stream: check packet ages and the Flight View banner.

### Default endpoints

| Setting | Default |
|---|---|
| Witness decoded telemetry feed | `127.0.0.1:8766` |
| Witness feed over Tailscale (from another machine) | `<witnessOS PC's Tailscale name or 100.x IP>:8767` |
| AIM subscriber host and port | `localhost:1883` |
| Embedded broker listener | `127.0.0.1:1883` |
| AIM subscriber UUID setting | `124` |
| AIM base topic | `entacore/aim/1234` |
| AIM subscription | `entacore/aim/1234/#` |
| Embedded broker at startup | Enabled |

### Watching from another machine (Tailscale)

witnessOS also publishes the same receive-only feed on its PC's Tailscale
address, port 8767 (Witness monitor > **Share over Tailscale**, on by default).
On the remote machine, in Horizon's Witness tab, set the host to the witnessOS
PC's Tailscale name (MagicDNS, e.g. `benchpc`) or its `100.x` address and the
port to 8767, then Connect. Horizon only accepts this PC (loopback) or a
Tailscale address, so the unauthenticated feed is never read over the LAN or
the internet; your tailnet's login and ACLs decide who can watch.

The remote feed is a separate server with its own thread and buffers: a slow
or dropped remote viewer is disconnected from it (Horizon reconnects
automatically) and never delays the local `127.0.0.1:8766` feed. TCP
keepalive detects a dead link within seconds.

These defaults are in [ground_settings.py](ground_settings.py). Witness raw TCP defaults are at the top of [monitor_core.py](monitor_core.py). Do not point the Horizon Witness feed at a raw telemetry TCP port: the feed carries decoded JSON records, not binary application packets.

The subscriber UUID is an identifier, not a password. Horizon adds a unique suffix to its MQTT client ID to avoid collisions between instances.

## Embedded MQTT broker

Horizon starts its broker on a background thread when the app opens. It supports MQTT 3.1.1, topic wildcards, QoS 0/1/2, and retained messages through aMQTT. No separate broker process or Windows service is required.

To receive AIM data:

1. Open Horizon and select **AIM XTRA MQTT**.
2. Leave **Run embedded MQTT broker (local only)** checked and confirm the broker status says it is running.
3. In AIM BASE, configure MQTT publication to `127.0.0.1`, port `1883`, with base path `entacore/aim/1234`.
4. Leave Horizon’s AIM Auto-connect enabled, or press Connect. Its subscriber must use the same broker endpoint and base path.
5. Confirm messages and receive timestamps advance. Broker running by itself does not mean AIM BASE is publishing.

The broker switch and subscriber connection are independent. Disconnecting the AIM subscriber does not stop the broker. Unchecking the broker box stops only Horizon’s broker and disconnects its clients; it does not stop an external broker. Checking it again starts a new broker session. After a startup failure, resolve the error and toggle the box off/on to retry.

The broker is loopback-only, unauthenticated, and has no TLS. Other processes on this computer can connect; other computers cannot connect directly. It allows up to 32 simultaneous connections. Do not expose this listener through port forwarding as though it were an authenticated network service.

Retained messages and sessions are held in memory. Stopping the broker or closing Horizon clears them; diagnostic and telemetry files remain on disk. Retained MQTT samples are marked as retained and must not be mistaken for fresh flight data.

### External broker or alternate startup settings

To use an existing broker without starting Horizon’s listener:

```powershell
.\.venv\Scripts\python.exe horizon.py --no-broker
```

Set the AIM subscriber host and port in its tab to the external broker. Editing those fields does not change the embedded listener. Changing the embedded port requires changing `DEFAULT_MQTT_BROKER_PORT` in `ground_settings.py`; also update the subscriber and AIM BASE publisher endpoints to match.

To start without automatic input connections or a broker:

```powershell
.\.venv\Scripts\python.exe horizon.py --no-auto-connect --no-broker
```

`--no-auto-connect` alone does **not** disable the broker. Checkbox changes apply to the current run; to change the broker startup default across runs, set `DEFAULT_MQTT_BROKER_ENABLED = False` in `ground_settings.py` or consistently use `--no-broker`.

If port 1883 is occupied, Horizon reports the bind failure without taking over or terminating the other program. Its subscriber can still use an existing broker at that address. Disable Horizon’s broker or stop the other broker yourself, then retry. Two Horizon instances cannot both own the same listener.

## Witness and Featherweight inputs

Witness hardware is opened by the Witness monitor (witnessOS tab; logic in `monitor_core.py`), not by Horizon. The monitor supports two serial inputs, two TCP inputs, decoded telemetry, diagnostic displays, graphs, and command transmission. Select `(no port)` for an unused serial slot. Serial/TCP streams use the same application decoder and the TCP inputs expect raw binary telemetry, not Telnet negotiation or terminal echo traffic.

Automatic USB discovery is enabled by default. Recognizable devices are discovered automatically. For a generic adapter, select its port manually once; after a valid packet, its USB identity can be learned in `auto_connect_witness.json` or `auto_connect_featherweight.json`. Changing a COM number does not change a learned USB identity.

Each Horizon input has its own Auto-connect checkbox. Turning it off stops future retries without dropping an already healthy connection. Manual Disconnect also disables that input’s auto-connect. Only one application should own a given COM port at a time.

Featherweight is read directly in its Horizon tab. Tracker battery voltage and local/ground-station battery voltage are separate fields. No Featherweight commands are sent.

## Flight View

Flight View is the launch display. It places key flight numbers and state on the left, up to six pyqtgraph graphs in the center, and GPS/power readouts on the right, in a resizable splitter (the side panels scroll only when the window is too small). Panels are dark text on white whatever the system theme. Redraws are timer-driven (the layout's `refresh_ms`, at most 30 Hz), never once per packet. Battery readouts use three decimal places; AIM CPU/ejection batteries share a row. Default AIM continuity placeholders are removed.

- **Witness GPS** (GPS panel, row titled `W`) shows the state packet's satellite count beside the status GPS lock bit, for example `GPS sats 7  lock 1`. `lock` is a literal `1` (fresh fix) or `0`. Satellites are those used in the solution and read 0 when the fix is stale. Each value keeps its own `!` age/invalid marker.
- **HITL** (state area, under the flight state) shows `HITL ACTIVE - SIMULATED SENSORS` in red whenever Witness reports status byte 1 bit 4, and the banner adds `HITL - SIMULATED SENSORS`. A held HITL value stays red even when old, so a simulated run is never mistaken for flight data. Otherwise the line reads `HITL off - real sensors (...)`.

The **Flight controls** menu provides layout loading/saving, Pause/Resume, Clear plots, Set pad, a graph time-window selector, fullscreen, and display details. F11 toggles fullscreen; Escape exits it; Ctrl+P pauses/resumes. Witness traces are solid, AIM traces dashed.

- Pause freezes the display, not telemetry reception or logging.
- Changing the graph window preserves history. History is bounded, so a longer window may contain empty space.
- Auto-scale fits each graph axis to its visible valid samples, including small changes around large offsets such as negative RSSI.
- Invalid samples retain the last valid readout, its original receive age, and an invalid indication. Graphs leave a gap instead of drawing the held value as a fresh sample.
- Clear plots resets histories, source bindings, and derived estimates, not logs.
- Set pad re-references the GPS horizontal distances and the Featherweight/AIM altitude-above-pad traces. The Witness Kalman altitude and position are zeroed by Witness itself on the pad, not by Horizon.
- Graph time is host receive time, displayed in UTC. Device uptime and AIM publisher timestamps serve different purposes.

### Units and derived values

Witness data uses the decoder’s protocol scaling. The Altitude AGL graph and readout show metres above the pad for every source:

- **W KF** is `kalman_altitude` from `0x02`/`0xE0`: the Witness position Kalman filter's up estimate, already relative to the pad (firmware before 2026-11-10 sent barometric altitude in this slot).
- **F GPS** is the Featherweight tracker's altitude (ft ASL × 0.3048) minus the pad's MSL altitude (`_derived.altitude_agl_m`).
- **A baro~** is AIM pressure altitude minus its first live sample (`_derived.barometric_altitude_agl_m`), because a standard-atmosphere pressure altitude is not MSL.
- Witness GPS altitude is available as `_derived.gps_altitude_agl_m` (only while the GPS-locked bit is set).

The pad's position and MSL altitude come from `pad_reference.py`, in priority order: a manual **Set pad here** on the Visualizer map (position only), the Featherweight ground station (`GPS_STAT` unit `BASE`), Aperture's LC86G GPS in the `0xF0` status packet, the mean Witness GPS fix while it reports Pad Idle with GPS lock, and finally the first Featherweight tracker fix. Until any pad altitude is known, an MSL source is referenced to its own first live value. Horizontal distance is `hypot(E, N)` of the Witness Kalman position (its origin is the pad) and the great-circle distance of GPS sources from the pad position.

Featherweight velocity is converted from feet per second.

AIM pressure is interpreted as pascals. Horizon estimates pressure altitude using a standard atmosphere with a 101325 Pa reference, then estimates vertical velocity from successive altitude differences divided by publisher timestamp differences. These are not AIM’s calibrated/fused outputs or height above ground.

AIM gyro traces are estimates from quaternion differences in degrees per second, not direct gyro samples. The calculation assumes Hamilton body-to-world quaternions; mounting and axis agreement with Witness are not verified. Derived gyro and velocity estimates require fresh, ordered samples and are not extrapolated across gaps.

AIM fields whose publisher units remain unverified, including GPS altitude and Z acceleration, retain their raw labeling and separate graph axes where configured. Do not compare raw-unit and physical-unit values as if their units were identical. See [flight_view.py](flight_view.py), [aim_altitude.py](aim_altitude.py), and [aim_orientation.py](aim_orientation.py) for the detailed assumptions.

## Attitude tab

**Attitude** (next to Flight View, which is unchanged) draws the Witness orientation quaternion as a 3D rocket with `pyqtgraph.opengl` (or, where OpenGL is unavailable such as headless or some remote desktops, an orthographic 2D projection of the same wireframe): body tube, nose cone and four fins. The fin on body +Y and a stripe along the body are orange so roll is visible. The view also shows a body-axis triad (`xb` = nose, `yb`, `zb`), world ENU axes (E, N, U) and a ground grid. **Iso**, **From East**, **From North** and **From Above** set the camera, and dragging the view orbits it. Readouts sit beside the view in a resizable splitter; below is a pyqtgraph strip chart of qx/qy/qz/qw and tilt, with a cursor at the replay time. **Conventions...** summarises the definitions below.

**Live mode** (default, or **Live feed**) uses the same Witness decoded feed as Flight View. A fresh `0xE0` debug quaternion is used when one has arrived in the last 2.5 s (10 Hz over USB after command `0x00E0`; the LoRa downlink relays it at most 1 Hz). Otherwise Horizon uses the newest 1 Hz `0x02` state quaternion. The readouts show the source (`0xE0`/`0x02` and feed source name), sample age (red and `STALE` after 3 s, `SNAPSHOT` for a relay replay), flight state, HITL flag (red when active) and GPS lock from the latest Witness status. The strip chart shows the last 30 to 600 s of host receive time; `0x02` samples are not added to it while `0xE0` is fresh.

**File mode** (**Load CSV...**) replays a CSV with **Play/Pause**, a speed selector and a scrub bar. Between samples it slerps if they are no more than 3 median sample intervals apart (minimum 0.25 s); across a longer gap it holds the previous sample. Time going backwards (a reboot or looped replay) is shifted forward so samples stay in file order. Accepted columns (case-insensitive):

| Purpose | Column names |
|---|---|
| Time, seconds | `time_s`, `time`, `t`, `t_s`, `time_seconds` (converter), `sim_time_s`, `host_time_s`, `timestamp_s` |
| Time, milliseconds | `time_ms`, `t_ms`, `uptime_ms`, `uptime`, `sim_time_ms` (used if no seconds column; otherwise row number x 0.1 s) |
| Witness estimate | First complete set of `Pqx,Pqy,Pqz,Pqw`, `Pq_x..`, `Pquat_x..` or `Porientation_x..`, with prefix `P` = `witness_`, `est_`, `estimate_`, `wit_` or none. The converter writes `orientation_x,orientation_y,orientation_z,orientation_w`. |
| Truth (optional) | The same patterns with prefix `truth_`, `true_`, `sim_` or `ref_`, for example `truth_qx,truth_qy,truth_qz,truth_qw` |
| Status (optional) | `flight_state`, `hitl_active`, `gps_locked`, or raw `status` (uint16); `packet_id` or `source` as a label |

The recommended HITL attitude file layout is `time_s,packet_id,witness_qx,witness_qy,witness_qz,witness_qw,truth_qx,truth_qy,truth_qz,truth_qw,flight_state,hitl_active`. In converter CSVs, rows without orientation (heartbeat, sensors, etc.) are skipped. If any `0xE0` rows carry orientation, only those rows are used.

When a truth quaternion is present, truth is drawn as a blue dashed ghost and as dashed strip-chart traces. Readouts add truth tilt, **tilt error** (estimate minus truth) and **attitude error**: the full rotation angle between truth and the estimate after a constant heading offset (rotation about U) is removed. By default the offset is estimated at the first sample where both are valid. **Align heading here** re-estimates it at the current replay time, and **Auto align** reverts. With **Heading-align estimate** checked, the estimate rocket is drawn rotated by the offset so it overlays the ghost.

### Conventions

- Quaternion order is x, y, z, w (scalar last, as on the wire). It is a Hamilton quaternion that rotates BODY vectors into WORLD (`v_world = q v_body q*`), per `comms.h`. WORLD is ENU: +X east, +Y north, +Z up. BODY +X is the nose; upright on the pad, body +X points to world +Z.
- All four components zero means attitude is not available. The tab shows `ATTITUDE NOT AVAILABLE` and draws no rocket. Non-finite or missing values are treated the same way. Other quaternions are normalised for display, and `|q|` is flagged `CHECK NORM` when it differs from 1 by more than 0.01.
- **Tilt from vertical** is the angle between body +X and world +Z (0 = upright, 180 = nose down). It does not depend on heading.
- **Nose azimuth** is the compass bearing of the nose's horizontal projection: 0 = north, 90 = east. It is meaningless when the rocket is nearly vertical (shown grey under 1 degree of tilt).
- **Spin about nose** is the twist about the nose, right-handed, relative to the upright reference `(0, -0.7071, 0, 0.7071)`: body +Y north, +Z west. That reference is what the firmware's shortest-arc tilt alignment gives for a vertical rocket. It is computed as the swing-twist split of `q * q_ref^-1` about world +Z and is defined everywhere except nose straight down.
- **ZYX yaw/pitch/roll** is the intrinsic Z-Y'-X'' sequence of body relative to ENU, `R = Rz(yaw) Ry(pitch) Rx(roll)`, each angle right-handed. Yaw is measured from east toward north, and pitch > 0 means nose down. An upright rocket is at pitch -90 degrees, this sequence's gimbal lock. Near vertical, roll is folded into yaw and marked `gimbal lock`, so use tilt/azimuth/spin for flight.

### Limitations

- Heading is not observable on Witness (no magnetometer). Its initial heading is arbitrary, so nose azimuth, spin, yaw and roll include an unknown constant offset. For a vertical rocket, heading and spin are the same rotation. Only tilt is absolute.
- The `0x02` quaternion is 1 Hz. The 10 Hz `0xE0` stream must be enabled with command `0x00E0` from the Witness monitor (Horizon sends no commands). Over LoRa, `0xE0` is relayed at 1 Hz at most. Fast motion between 1 Hz samples is not visible, and live mode does not interpolate.
- Live age is host receive time and includes link latency. File time is the CSV's own time base. For converter CSVs that is device uptime.
- The drawing is a simple wireframe/mesh model (perspective in OpenGL, orthographic in the 2D fallback), not to scale.

## Visualizer window

**Windows > Visualizer** (or **F9**) in `horizon.py` opens a separate window with the **orientation view on the left and the map on the right, side by side** in a resizable splitter, so both are visible at once. Both use the same Witness and Featherweight records as Flight View. Tracks are collected from startup, so opening the window later still shows the whole flight.

**Orientation** draws the Witness quaternion as the Attitude tab's 3D rocket (OpenGL, drag to orbit; 2D fallback without OpenGL) (fresh `0xE0`, else `0x02`; same conventions), with large **tilt from vertical**, **roll** (spin about the nose) and **heading** (nose azimuth) readouts. Heading and roll are relative to the arbitrary heading fixed at initialisation, because Witness has no magnetometer. An all-zero quaternion is explained rather than drawn:

- on the pad: `ATTITUDE NOT INITIALISED` (waiting for the 1 s stationary window; HITL entry/exit resets it);
- in Boost/Coast/Descent/Landed after a valid attitude: `ATTITUDE LOST`. Witness stops its attitude for the rest of the flight once a gyro axis saturates (≥ 1950 dps), and its Kalman filter then ignores the IMU. The last valid attitude is drawn as a blue ghost.

**Map** is a top-down, north-up pyqtgraph plot in local ENU metres centred on the pad, with locked aspect, showing:

- range rings, a north arrow and a scale bar;
- a track for each source, distinguished by colour and marker:
  - Witness Kalman ENU: black line and dot;
  - Witness GPS (int24, about 11 m steps, only with GPS lock): blue squares;
  - Featherweight tracker: green dashed line and circle;
- distance, compass bearing, altitude above the pad and age for each source;
- the pad source in use, its position and its MSL altitude.

The Kalman position is relative to the vehicle's own pad. It is placed using the mean Witness GPS fix on the pad when one exists, and is otherwise assumed to start at the selected pad. Drag to pan, use the mouse wheel or **+/-** to zoom (the tile zoom level follows the view), and tick **Follow rocket** to keep the newest position centred. **Set pad here...** then a click overrides the pad position. **Auto pad** returns to automatic selection.

The map always works offline as a metric grid. With **OSM tiles** ticked (the default), OpenStreetMap raster tiles are drawn underneath. They are fetched on one background thread at no more than 2 requests/s, with a User-Agent naming Horizon and no personal data, and cached in `tile_cache/osm/<z>/<x>/<y>.png` (ignored by git). Cached tiles are re-used offline and refreshed only after 30 days. **Pre-cache area...** downloads up to 600 tiles covering about 2 km around the pad at zoom 12–17. Do this once at home with internet, after the pad is known or set manually. Start with `horizon.py --offline-map` to never download. The map shows "© OpenStreetMap contributors" whenever tiles are drawn.

Redraws are timer-driven: at most 25 Hz, and only when data or the view changed. Tiles are downloaded by the `map_tiles.py` worker (same User-Agent and rate policy), decoded on a worker thread and shown as pyqtgraph `ImageItem`s.

## Layout Editor

Use **Layout Editor** to customize the dashboard without editing JSON by hand:

1. Choose **Use current view**, **Load JSON**, or **Defaults** to initialize the draft.
2. Select a graph, readout, or trace in the tree. Edit its title, units, input, packet/topic, field, source, scale, offset, color, or filter.
3. Use Add graph/readout/trace, Remove, and Move up/down to arrange content. Graph order runs left-to-right, then top-to-bottom. Readouts belong to the flight, state, GPS, or power section.
4. Choose **Apply to Flight View** to validate and apply the draft. Confirming resets graph history and source bindings; connections and logging continue.
5. Choose **Save JSON as...** to keep the setup. Saving and applying are separate operations.

The editor form scrolls when needed. The dashboard supports at most six graphs in two columns and three rows, with up to three flight readout rows, one state row, six GPS rows, and ten power rows. Flight rows hold up to three values; state, GPS and power rows hold up to two. Validation rejects layouts exceeding these limits.

Display formats are `number`, `state` (flight-state name), `continuity`, `flag` (a 0/1 bit shown literally, used for GPS lock) and `hitl` (the HITL warning line; use it as the state row's second value). Witness status fields available to traces are `status.flight_state`, `status.gps_locked` and `status.hitl_active` (and the `fc_status.*` equivalents on camera packets), plus `status.byte0.bit0` to `bit7`. Layouts saved before this change still load, but their `Witness GPS satellites` placeholder stays N/A and they have no HITL line until edited or replaced with the default.

Trace mapping uses `input`, `packet`, and `field`. AIM packet names are relative MQTT topics such as `pressure_payload/v1`. Array field names use indexes such as `gyro[0]`. A source of `*` binds an available live source; use an exact source name to avoid choosing a different device. A filter such as `{"unit_type": "TRACKER"}` selects Featherweight tracker records. Displayed values are `decoded_value * scale + offset`.

Start with a saved layout using:

```powershell
.\.venv\Scripts\python.exe horizon.py --flight-layout flight_view_custom.json
```

The default is [flight_view_default.json](flight_view_default.json). The full schema and validation rules are in [flight_data.py](flight_data.py). This layout format is different from the Witness monitor's graph-selection JSON; use the editor belonging to the app whose view you want to change.

## Testing without hardware

Start Horizon normally, then in witnessOS's **Witness monitor** tab use the test-source row: leave the file blank for the built-in 60 s simulated flight, or **Browse JSON** for a replay such as `test_data/ballistic_10g.json`, set **Speed** (0.1-10) and **Loop**, and press **Start test**. Embedders can pass the same options to `main.MonitorPanel(test_mode=True, test_source=..., test_speed=..., test_loop=...)`.

Without **Loop**, the scenario plays once; with it, the scenario repeats and its device uptimes repeat too. Test bytes pass through the normal Witness decoder and local telemetry feed; they are not transmitted to hardware. Witness test mode disables automatic USB opening and labels test traffic. Horizon’s other enabled inputs can still receive real data, so turn those inputs off if you want an isolated demonstration.

Replay JSON has `version: 1`, `duration_ms`, and a `frames` list containing ordered `at_ms` times and hexadecimal byte strings. See [test_data/example_packets.json](test_data/example_packets.json) and [witness_test_source.py](witness_test_source.py). The built-in scenario and both example files use 65-byte state frames. GPS has no fix at first (0 satellites, lock 0), then locks, and some samples have 10 satellites so a `0x0A` byte appears inside the payload. None of them set the HITL bit.

## Logs and postflight analysis

Logs are written under `logs/` next to the scripts:

| File family | Contents |
|---|---|
| `serial_*.bin`, `TCP_*.bin` | Witness monitor raw transport bytes |
| `debug_*.log` | Witness monitor console and parse diagnostics (every line, even ones the on-screen console skips) |
| `nmea_<source>_*.csv` | GPS NMEA sentences the Witness monitor found in the stream: `received_utc,checksum_ok,sentence` |
| `test_*.bin` | Simulated Witness bytes |
| `horizon_witness_*.jsonl` | Decoded Witness feed records and diagnostics |
| `horizon_featherweight_*.bin`, `horizon_featherweight_*.jsonl` | Featherweight bytes and decoded/diagnostic records |
| `aim_mqtt_*.jsonl` | AIM message payloads, decoded fields, and connection diagnostics |
| `mqtt_broker_*.log` | Embedded broker lifecycle and warnings/errors |

The Witness monitor picks GPS NMEA sentences (`$GNGGA`, `$GNRMC`, ...) out of the mixed Witness stream and checks each `*hh` checksum. Valid and damaged sentences both go to the NMEA CSV (`checksum_ok` 1 or 0); a damaged one is usually a sentence with a telemetry frame spliced into it. A status line under the feed status shows the counts per input and the latest GGA fix, satellites and altitude.

The monitor keeps up with fast streams: each GUI tick (30 ms) drains every queued event, keeps every graph sample at its arrival time in numpy ring buffers, updates the labels from the newest packet of each type, and redraws the pyqtgraph graphs through a throttle (at most 20 times a second, and only while the Graphs tab is shown). The on-screen console keeps its newest 2,000 lines and, in a burst, skips lines with a note; the debug log keeps everything.

Broker diagnostic logs rotate at 5 MB with two backups per run. This does not limit the other telemetry log files or remove logs from previous runs; monitor disk space. Broker logs are not a replacement for the AIM message log.

Convert a Witness binary log and open it for plotting:

```powershell
.\.venv\Scripts\python.exe converter.py S0000434.BIN --output S0000434.csv
.\.venv\Scripts\python.exe viewer.py S0000434.csv
```

Viewer data boards: each graph can have several y axes, left or right, each autoranging over its own traces for the visible time range (x stays shared). Drag a trace (from the list, or a legend entry) onto an existing axis to put it on that axis, onto the left or right edge of the plot to give it a new axis on that side, or into the middle of the plot for a new axis on the side with fewer axes (right when tied); a blue highlight shows where it will land. Drag an axis onto another axis to merge them, or to a plot edge to move it to that side; right-click an axis for "Move axis to left/right", merge and split. An axis with several traces is labelled `first +n` in grey; with one trace it takes that trace's colour. The header's **Own axes** button (or **Overlays on own y axes**) gives every trace its own axis. **Add spectrogram of selected trace to board** (also in the trace and legend right-click menus) docks a spectrogram between the graphs, with its time axis linked to theirs; the up/down arrows reorder it. **Sync logs on** (Time axis panel) sets every open log's time offset so the first rise into a flight state (Pad idle to Boost, Boost to Coast, Coast to Descent, Descent to Landed) is t = 0: from the log's own `flight_state`, else the session's `.TXT` log, else another open log with the same session suffix (`I00000F7` pairs with `S00000F7`). Logs without that transition keep their offset and are listed. **Save board…** writes the graphs (columns, axis groups and sides, order, heights), docked spectrograms (column, FFT size, overlap, window, detrend, Welch, colour range, time window), the x axis, packet filter, graphs per row, the sync transition and each file's time column, unit and offset to a versioned JSON file (`format: horizon-csv-board`, `version: 2`, see `viewer_board.py`; version 1 files still load). **Load board…** rebuilds it on the open log(s), matching columns by name and re-applying the sync; the first open file takes the board's first file, and so on. Columns the new log lacks are listed in a message and skipped. Open the logs before loading a board.

The converter detects raw Witness application streams and `WL` flash records from Witness mass storage. These are 48-byte v1 records, or 24-byte v2 records at 50 Hz with status bytes 0-1 (flight state), sequence, uptime, pressure and both temperatures; the record size comes from the version byte. Files named `I*.BIN` are decoded as the store-v3 IMU stream (24-byte records: low-g and high-g accelerations in g, gyro in dps, flight state, HITL and freshness flags, `high_g_active` (the filtered acceleration came from the high-g sensor), and `missing_before` from the 8-bit sequence; in Descent, records are deliberately decimated 1 in 10, so `missing_before` reads 9 there; time is unwrapped from the 32-bit microsecond counter). Files named `P*.BIN` are the store-v4 packet stream, records of `[u32 us][source][length][frame]`. They are decoded with the normal packet parser into one row per frame, with `log_source` (`witness_state` at 50 Hz, `can_rx`, `can_tx`, `radio_rx`) and `time_seconds` from the record clock. `--format {auto,imu,packets,flash,wire}` overrides detection. It does not decode Featherweight BIN captures or AIM JSONL logs. CSV `time_seconds` derives from device uptime, not host receipt time. The viewer opens with a summary and lets you select traces, including flight state and individual status bits.

### Downloading Witness logs through the bench Pi

`witness_download.py` fetches the flash logs from a Witness plugged into the bench Pi, from this computer. It uses the normal USB export: it sends `05 68 68` to the Witness raw TCP data port (default 7100, so close Horizon first), waits for the read-only `WITNESSLOGS` disk to appear on the Pi, and reads it over SSH with `witness_pi_agent.py`. That reader is piped to `python3 -`, so nothing is installed or mounted on the Pi. Files are saved under `logs/witness_<timestamp>/`.

```powershell
.\.venv\Scripts\python.exe witness_download.py setup        # once: udev rule so the Pi user can read the disk (asks for the Pi sudo password)
.\.venv\Scripts\python.exe witness_download.py list
.\.venv\Scripts\python.exe witness_download.py get --latest --skip-gps --convert --view --restart
.\.venv\Scripts\python.exe witness_download.py get I0000012.BIN S0000012.TXT
```

`setup` installs `/etc/udev/rules.d/99-witness-logs.rules`, giving group `plugdev` read access to the Witness disk (USB 303a:4012). `--convert` runs `converter.py` on each `.BIN` except the raw GPS `U*.BIN`, and `--view` opens the first CSV (IMU first) in `viewer.py`. Witness stays in export mode, with no logging and no USB serial, until it is power cycled. `--restart` power-cycles Pi USB hub 2 afterwards (the allowed passwordless `uhubctl -l 2 -a 2`), which resets every board on that hub. SSH options: `--host` (default `prs2026hitl`), `--user` and `--key`, or `WITNESS_PI_HOST` / `WITNESS_PI_USER` / `WITNESS_PI_KEY`, otherwise your `~/.ssh/config`. Large files go at the speed of the Witness USB full-speed disk; the GPS ring can be about 45 MB, so `--skip-gps` is useful.

Witness status rows get `status_byte0`, `status_byte1`, `flight_state`, `flight_state_name`, `gps_locked` (byte 1 bit 3) and `hitl_active` (byte 1 bit 4). State rows also get `gps_satellites`; it is blank for legacy 64-byte state frames. The viewer plots `gps_locked`, `hitl_active` and the `status_byte1_bit3_gps_locked` / `status_byte1_bit4_hitl_active` bit traces on a 0..1 axis. For CSVs converted before these columns existed, it derives `gps_locked` and `hitl_active` from `status`. The summary line reports how many records had HITL active. Echoed `0x0C` HITL input packets are skipped and are not counted as discarded bytes.

## Firmware protocol references

[GROUND_STATION_PROTOCOL.md](GROUND_STATION_PROTOCOL.md) describes the Witness wire protocol and ground-station status packets; [comms.h](comms.h) contains the local field definitions. Application frames use packet ID, a payload whose size depends on that ID, and final `0x0A`. Multi-byte values are big-endian. `0x0A` can occur inside payloads, so splitting the stream on newlines is not valid deframing.

The monitor and converter also accept captured sensor payloads of 23 bytes alongside the 21-byte documented form. Check the deployed firmware revision before changing lengths or scaling. Witness flash `WL` records are a different format from application frames.

The state packet (`0x02`) is a 65-byte frame: ID, a 63-byte payload, then EOF at frame offset 64. The payload is status, uptime, orientation, position, velocity, latitude, longitude, GPS altitude, barometric altitude, battery voltage (uint16 mV at frame offset 61) and GPS satellites (uint8 at frame offset 63). Witness status byte 1 (the low byte of `status`) holds the flight state in bits 2:0, GPS locked in bit 3, HITL active in bit 4 and MAINTENANCE boot (Wi-Fi, logging off, not flight ready) in bit 5; bits 7:6 are reserved. Horizon shows a red banner and a window-title prefix while the maintenance bit is set; in that mode only a 0x02 state packet arrives about every 5 s over LoRa. These bits appear in the sensor, state, command, debug and heartbeat packets, and in camera `fc_status`. The Witness monitor shows the satellites as `7  lock 1`. Its status decoders add `GPS lock` and `HITL (SIM)` indicators, with HITL shown in red.

**Legacy 64-byte state frames.** Older firmware and logs use a 64-byte state frame with no satellite byte. There is no version field, so both decoders try the 65-byte form first. It is chosen when frame offset 64 is `0x0A`, even if offset 63 is also `0x0A` (10 satellites). The 64-byte form is chosen only when offset 63 is `0x0A` and offset 64 is not. No packet ID is `0x0A`, so a legacy frame followed by any valid packet is unambiguous. To avoid cutting a split read short, the live monitor waits for the 65th byte before deciding. A legacy frame at the very end of a stream is therefore decoded only once another byte arrives. Two rare cases can still be misread: a stray `0x0A` noise byte straight after a legacy frame (read as a 65-byte frame with 10 satellites), and a corrupted 65-byte frame with 10 satellites and a bad EOF (read as legacy, then resynchronized). The converter also accepts a legacy-looking frame at the end of a file.

`0x0C` is the 47-byte HITL sensor-input packet sent from host to Witness over USB, and Witness may echo it. Horizon's monitor and converter recognize it by its fixed length and EOF and skip it. It is never decoded or forwarded as telemetry. The monitor console logs `HOST_ECHO` on the first echo and then on every 100th.

Commands are sent from the Witness monitor, not Horizon. All command shortcuts, including USB mass storage and log erase, use full 10-byte canonical frames so they can cross the Aperture LoRa bridge. Erasing logs is destructive; do not use it as a connectivity test. Other command shortcuts use the monitor’s framed command encoder. This guide does not describe how to build or flash device firmware.

`HITL Enter (USB, pad only)` sends `0x4849` and `HITL EXIT -> real sensors` sends `0x4858`. Each button sends one command when pressed; nothing is sent automatically. Witness accepts HITL entry only over USB in Pad Idle. It leaves HITL after an exit command, a reboot, or one second without `0x0C` input. The monitor does not generate `0x0C` sensor input itself.

## Troubleshooting

| Symptom | Check |
|---|---|
| Broker unavailable or dependency error | Install `requirements.txt` with the same Python interpreter used to run Horizon. |
| Broker cannot bind port 1883 | Another process may own it. Disable the embedded broker to use the existing service, or free the port and toggle the checkbox. |
| Broker running but no AIM messages | AIM BASE must be connected and publishing; verify its host, port, and base path against Horizon. Try `127.0.0.1` instead of `localhost` if name resolution differs. |
| Remote computer cannot connect to embedded broker | The listener is intentionally loopback-only. Use an appropriately secured external broker for network access. |
| AIM BASE USB connection error | Horizon does not open AIM USB hardware. Resolve the connection in AIM BASE; starting a broker cannot fix a USB failure. |
| Witness tab disconnected | Start the witnessOS Witness monitor, check that its local feed is available, and use feed port 8766 rather than a raw TCP telemetry port. |
| Featherweight missing | Check COM ownership and connect a generic adapter manually once so its identity can be learned. |
| Values visible but not changing | Inspect receive age, retained/invalid flags, pause state, and the source tab. A held value is not a fresh packet. |
| Custom graph empty | Verify packet/topic and flattened field names, source selection, filters, enabled state, axis units, and received messages. |
| Qt startup error / no window | Install `requirements.txt` (PyQt6, pyqtgraph) into the interpreter you run. |
| Attitude view says "2D projection (OpenGL unavailable)" | OpenGL could not be initialised (remote desktop, old driver, missing PyOpenGL). The 2D view shows the same attitude; set `HORIZON_NO_GL=1` or `--no-gl` to force it. |

## Development and verification

| Module | Responsibility |
|---|---|
| `horizon.py` | App startup, tabs, defaults, and shutdown (Qt main window) |
| `qt_common.py`, `qt_widgets.py`, `observable.py` | Shared Qt/pyqtgraph setup, widget bindings, observable `Cell` state |
| `mqtt_broker.py` | Background embedded broker and diagnostic log |
| `aim_tab.py`, `aim_mqtt.py` | AIM tab (fields + live strip chart of a selected field), MQTT subscriber, decoding, logging |
| `ground_tabs.py`, `auto_connect.py`, `ground_receivers.py`, `featherweight.py` | Witness feed / Featherweight tabs, connection + auto-connect logic (no Qt), reception |
| `telemetry_view.py`, `telemetry_model.py` | Field tree widget; field schemas and row model (no Qt) |
| `flight_view.py`, `flight_render.py`, `flight_data.py` | Dashboard widgets; what to display (no Qt); validated layouts and histories |
| `flight_editor.py`, `flight_layout_draft.py` | Layout editor widget; draft editing logic and JSON save/load (no Qt) |
| `attitude_view.py`, `attitude_scene.py`, `attitude_data.py`, `attitude_math.py` | Attitude tab and 3D/2D rocket widget; rocket geometry (no Qt); live/CSV samples; quaternion math |
| `visualizer.py`, `map_data.py`, `map_tiles.py`, `pad_reference.py`, `geodesy.py` | Visualizer window (orientation + map side by side); tracks, tile cache, pad selection, geodesy |
| `aim_altitude.py`, `aim_orientation.py` | Derived AIM altitude, velocity, and angular rates |
| `main.py`, `monitor_core.py`, `witness_protocol.py`, `witness_link.py`, `witness_test_source.py` | `MonitorPanel` Qt widget (embedded by witnessOS); monitor logic and protocol (no Qt); local relay; test playback |
| `converter.py`, `viewer.py` | Postflight conversion and analysis |
| `witness_download.py`, `witness_pi_agent.py` | Download Witness flash logs through the bench Pi |

Run the regression suite from this folder:

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -q
```

Broker tests use isolated loopback ports rather than the live AIM port. They cover MQTT delivery through the AIM receiver, QoS, wildcard subscriptions, retained messages, startup defaults, checkbox shutdown/restart, missing dependencies, and port conflicts. GUI tests run headless (`QT_QPA_PLATFORM=offscreen`, which also exercises the 2D attitude fallback); they include checks that live packets are batched into throttled redraws and that the Visualizer shows the map and attitude side by side. These tests do not replace verification with the actual receivers and deployed firmware before a flight.

## Shared firmware protocol

Iris channel currents use 20 mA/count; the monitor and converter decode this scale.
Earlier captures encoded at 100 mA/count require a matching older decoder. The
wire layout has no version field to distinguish those scales automatically.
Witness forwards Iris camera/debug telemetry received over CAN to USB and, when
enabled, LoRa. The CAN fragment envelope is removed before reaching Horizon.
See [shared CAN contract](../Intercomm/README.md).

## Antenna pointer (design only)

A G-code-driven antenna pointer (separate process on the Horizon feed, GRBL/FluidNC mount) is designed in [ANTENNA_POINTER_DESIGN.md](ANTENNA_POINTER_DESIGN.md); the look-angle maths is prototyped in `antenna_pointer_math.py`. Nothing is wired into the GUI yet.
