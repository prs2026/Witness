# Horizon Unified Ground Station

Horizon is the desktop ground-station software for viewing Witness, Featherweight GPS, and AIM XTRA telemetry together. It includes a fixed launch dashboard, a layout editor, logging, and an embedded MQTT broker. This folder contains Python applications, not the embedded firmware that runs on the flight computers.

Start `horizon.py` for the unified display. Run `main.py` alongside it for Witness hardware input. AIM BASE still connects to AIM hardware and publishes its telemetry; Horizon replaces the separate MQTT broker application, not AIM BASE.

## Data connections

| Input | Data path | Where to configure it |
|---|---|---|
| Witness | Serial or TCP → `main.py` → local JSON telemetry feed → Horizon | Witness monitor for hardware; Horizon Witness tab for feed |
| Featherweight GPS | Serial COM port → Horizon | Featherweight GPS tab |
| AIM XTRA | AIM BASE → MQTT broker → Horizon subscriber | AIM BASE publisher settings and Horizon AIM XTRA MQTT tab |

Horizon’s telemetry clients are receive-only: they do not send flight commands to Witness, Featherweight, or AIM hardware. The Witness monitor (`main.py`) has separate command controls that can transmit to connected serial/TCP devices. The embedded broker routes MQTT messages between clients; it does not translate them into hardware commands.

## Installation and startup

The application has been tested with Python 3.14 on Windows. Install Python with Tcl/Tk support, then run these commands in this folder using PowerShell:

```powershell
py -3.14 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe horizon.py
```

If `.venv` already exists, use it and run the dependency-install command after upgrading. Dependencies are `pyserial`, `paho-mqtt`, and `amqtt`; the graphs use Tk canvas drawing.

In a second terminal, start the Witness monitor:

```powershell
.\.venv\Scripts\python.exe main.py
```

Either app may start first. Horizon retries its enabled inputs when unavailable. A connected device with no fresh packets is not the same as a healthy telemetry stream: check packet ages and the Flight View banner.

### Default endpoints

| Setting | Default |
|---|---|
| Witness decoded telemetry feed | `127.0.0.1:8766` |
| AIM subscriber host and port | `localhost:1883` |
| Embedded broker listener | `127.0.0.1:1883` |
| AIM subscriber UUID setting | `124` |
| AIM base topic | `entacore/aim/1234` |
| AIM subscription | `entacore/aim/1234/#` |
| Embedded broker at startup | Enabled |

These defaults are in [ground_settings.py](ground_settings.py). Witness raw TCP defaults are at the top of [main.py](main.py). Do not point the Horizon Witness feed at a raw telemetry TCP port: the feed carries decoded JSON records, not binary application packets.

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

Witness hardware is opened by `main.py`, not by Horizon. The monitor supports two serial inputs, two TCP inputs, decoded telemetry, diagnostic displays, graphs, and command transmission. Select `(no port)` for an unused serial slot. Serial/TCP streams use the same application decoder and the TCP inputs expect raw binary telemetry, not Telnet negotiation or terminal echo traffic.

Automatic USB discovery is enabled by default. Recognizable devices are discovered automatically. For a generic adapter, select its port manually once; after a valid packet, its USB identity can be learned in `auto_connect_witness.json` or `auto_connect_featherweight.json`. Changing a COM number does not change a learned USB identity.

Each Horizon input has its own Auto-connect checkbox. Turning it off stops future retries without dropping an already healthy connection. Manual Disconnect also disables that input’s auto-connect. Only one application should own a given COM port at a time.

Featherweight is read directly in its Horizon tab. Tracker battery voltage and local/ground-station battery voltage are separate fields. No Featherweight commands are sent.

## Flight View

Flight View is a single, non-scrolling launch display. It places key flight numbers and state on the left, up to six graphs in the center, and GPS/power readouts on the right. Text is dark on white backgrounds. Battery readouts use three decimal places; AIM CPU/ejection batteries share a row. Default AIM continuity placeholders are removed.

- **Witness GPS** (GPS panel, row titled `Witness`) shows the state packet's satellite count beside the status GPS lock bit, for example `GPS sats 7  lock 1` (compact) or `GPS sats 7` | `lock 1` (large windows). `lock` is a literal `1` (fresh fix) or `0`. Satellites are those used in the solution and read 0 when the fix is stale. Each value keeps its own `!` age/invalid marker.
- **HITL** (state area, under the flight state) shows `HITL ACTIVE - SIMULATED SENSORS` in red whenever Witness reports status byte 1 bit 4, and the banner adds `HITL - SIMULATED SENSORS`. A held HITL value stays red even when old, so a simulated run is never mistaken for flight data. Otherwise the line reads `HITL off - real sensors (...)`.

The **Flight controls** menu provides layout loading/saving, Pause/Resume, Clear plots, Set pad, a graph time-window selector, fullscreen, and display details. F11 toggles fullscreen; Escape exits it.

- Pause freezes the display, not telemetry reception or logging.
- Changing the graph window preserves history. History is bounded, so a longer window may contain empty space.
- Auto-scale fits each graph axis to its visible valid samples, including small changes around large offsets such as negative RSSI.
- Invalid samples retain the last valid readout, its original receive age, and an invalid indication. Graphs leave a gap instead of drawing the held value as a fresh sample.
- Clear plots resets histories, source bindings, and derived estimates, not logs.
- Set pad resets horizontal-distance references. It does not zero altitude.
- Graph time is host receive time, displayed in UTC. Device uptime and AIM publisher timestamps serve different purposes.

### Units and derived values

Witness data uses the decoder’s protocol scaling. Featherweight altitude and velocity are converted from feet/feet per second for the default dashboard; its altitude is not automatically height above the launch pad.

AIM pressure is interpreted as pascals. Horizon estimates pressure altitude using a standard atmosphere with a 101325 Pa reference, then estimates vertical velocity from successive altitude differences divided by publisher timestamp differences. These are not AIM’s calibrated/fused outputs or height above ground.

AIM gyro traces are estimates from quaternion differences in degrees per second, not direct gyro samples. The calculation assumes Hamilton body-to-world quaternions; mounting and axis agreement with Witness are not verified. Derived gyro and velocity estimates require fresh, ordered samples and are not extrapolated across gaps.

AIM fields whose publisher units remain unverified, including GPS altitude and Z acceleration, retain their raw labeling and separate graph axes where configured. Do not compare raw-unit and physical-unit values as if their units were identical. See [flight_view.py](flight_view.py), [aim_altitude.py](aim_altitude.py), and [aim_orientation.py](aim_orientation.py) for the detailed assumptions.

## Attitude tab

**Attitude** (next to Flight View, which is unchanged) draws the Witness orientation quaternion as a 3D wireframe rocket on a Tk canvas: body tube, nose cone and four fins. The fin on body +Y and a stripe along the body are orange so roll is visible. The view also shows a body-axis triad (`xb` = nose, `yb`, `zb`), world ENU axes (E, N, U) and a ground grid. **Iso**, **From East**, **From North** and **From Above** set the camera, and dragging the view orbits it. Below the view is a strip chart of qx/qy/qz/qw and tilt. **Conventions...** summarises the definitions below.

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
- The drawing is an orthographic wireframe with simple depth ordering, not a rendered model, and is not to scale.

## Layout Editor

Use **Layout Editor** to customize the dashboard without editing JSON by hand:

1. Choose **Use current view**, **Load JSON**, or **Defaults** to initialize the draft.
2. Select a graph, readout, or trace in the tree. Edit its title, units, input, packet/topic, field, source, scale, offset, color, or filter.
3. Use Add graph/readout/trace, Remove, and Move up/down to arrange content. Graph order runs left-to-right, then top-to-bottom. Readouts belong to the flight, state, GPS, or power section.
4. Choose **Apply to Flight View** to validate and apply the draft. Confirming resets graph history and source bindings; connections and logging continue.
5. Choose **Save JSON as...** to keep the setup. Saving and applying are separate operations.

The editor may scroll; Flight View never does. A fixed dashboard supports at most six graphs in two columns and three rows, with up to three flight readout rows, one state row, six GPS rows, and ten power rows. Flight rows hold up to three values; state, GPS and power rows hold up to two. Validation rejects layouts exceeding these limits.

Display formats are `number`, `state` (flight-state name), `continuity`, `flag` (a 0/1 bit shown literally, used for GPS lock) and `hitl` (the HITL warning line; use it as the state row's second value). Witness status fields available to traces are `status.flight_state`, `status.gps_locked` and `status.hitl_active` (and the `fc_status.*` equivalents on camera packets), plus `status.byte0.bit0` to `bit7`. Layouts saved before this change still load, but their `Witness GPS satellites` placeholder stays N/A and they have no HITL line until edited or replaced with the default.

Trace mapping uses `input`, `packet`, and `field`. AIM packet names are relative MQTT topics such as `pressure_payload/v1`. Array field names use indexes such as `gyro[0]`. A source of `*` binds an available live source; use an exact source name to avoid choosing a different device. A filter such as `{"unit_type": "TRACKER"}` selects Featherweight tracker records. Displayed values are `decoded_value * scale + offset`.

Start with a saved layout using:

```powershell
.\.venv\Scripts\python.exe horizon.py --flight-layout flight_view_custom.json
```

The default is [flight_view_default.json](flight_view_default.json). The full schema and validation rules are in [flight_data.py](flight_data.py). This layout format is different from the Witness monitor's graph-selection JSON; use the editor belonging to the app whose view you want to change.

## Testing without hardware

Start Horizon normally, then run the built-in Witness test source in a separate terminal:

```powershell
.\.venv\Scripts\python.exe main.py --test
```

For the example ballistic flight:

```powershell
.\.venv\Scripts\python.exe main.py --test-file test_data/ballistic_10g.json --test-once
```

`--test-speed 2` runs replay at twice normal speed. Without `--test-once`, the scenario repeats and its device uptimes repeat too. Test bytes pass through the normal Witness decoder and local telemetry feed; they are not transmitted to hardware. Witness test mode disables automatic USB opening and labels test traffic. Horizon’s other enabled inputs can still receive real data, so turn those inputs off if you want an isolated demonstration.

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

The monitor keeps up with fast streams: each GUI tick drains every queued event, keeps every graph sample at its arrival time, and redraws the labels and graphs (at most 10 times a second) from the newest packet of each type. The on-screen console keeps its newest 2,000 lines and, in a burst, skips lines with a note; the debug log keeps everything.

Broker diagnostic logs rotate at 5 MB with two backups per run. This does not limit the other telemetry log files or remove logs from previous runs; monitor disk space. Broker logs are not a replacement for the AIM message log.

Convert a Witness binary log and open it for plotting:

```powershell
.\.venv\Scripts\python.exe converter.py S0000434.BIN --output S0000434.csv
.\.venv\Scripts\python.exe viewer.py S0000434.csv
```

The converter detects raw Witness application streams and 48-byte `WL` flash records from Witness mass storage. Files named `I*.BIN` are decoded as the store-v3 IMU stream (24-byte records: low-g and high-g accelerations in g, gyro in dps, flight state, HITL and freshness flags, and `missing_before` from the 8-bit sequence; time is unwrapped from the 32-bit microsecond counter). `--format {auto,imu,flash,wire}` overrides detection. It does not decode Featherweight BIN captures or AIM JSONL logs. CSV `time_seconds` derives from device uptime, not host receipt time. The viewer opens with a summary and lets you select traces, including flight state and individual status bits.

Witness status rows get `status_byte0`, `status_byte1`, `flight_state`, `flight_state_name`, `gps_locked` (byte 1 bit 3) and `hitl_active` (byte 1 bit 4). State rows also get `gps_satellites`; it is blank for legacy 64-byte state frames. The viewer plots `gps_locked`, `hitl_active` and the `status_byte1_bit3_gps_locked` / `status_byte1_bit4_hitl_active` bit traces on a 0..1 axis. For CSVs converted before these columns existed, it derives `gps_locked` and `hitl_active` from `status`. The summary line reports how many records had HITL active. Echoed `0x0C` HITL input packets are skipped and are not counted as discarded bytes.

## Firmware protocol references

[GROUND_STATION_PROTOCOL.md](GROUND_STATION_PROTOCOL.md) describes the Witness wire protocol and ground-station status packets; [comms.h](comms.h) contains the local field definitions. Application frames use packet ID, a payload whose size depends on that ID, and final `0x0A`. Multi-byte values are big-endian. `0x0A` can occur inside payloads, so splitting the stream on newlines is not valid deframing.

The monitor and converter also accept captured sensor payloads of 23 bytes alongside the 21-byte documented form. Check the deployed firmware revision before changing lengths or scaling. Witness flash `WL` records are a different format from application frames.

The state packet (`0x02`) is a 65-byte frame: ID, a 63-byte payload, then EOF at frame offset 64. The payload is status, uptime, orientation, position, velocity, latitude, longitude, GPS altitude, barometric altitude, battery voltage (uint16 mV at frame offset 61) and GPS satellites (uint8 at frame offset 63). Witness status byte 1 (the low byte of `status`) holds the flight state in bits 2:0, GPS locked in bit 3 and HITL active in bit 4; bits 7:5 are reserved. These bits appear in the sensor, state, command, debug and heartbeat packets, and in camera `fc_status`. The Witness monitor shows the satellites as `GPS sats: 7  lock 1`. Its status decoders add `GPS lock` and `HITL (SIM)` indicators, with HITL shown in red.

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
| Witness tab disconnected | Start `main.py`, check that its local feed is available, and use feed port 8766 rather than a raw TCP telemetry port. |
| Featherweight missing | Check COM ownership and connect a generic adapter manually once so its identity can be learned. |
| Values visible but not changing | Inspect receive age, retained/invalid flags, pause state, and the source tab. A held value is not a fresh packet. |
| Custom graph empty | Verify packet/topic and flattened field names, source selection, filters, enabled state, axis units, and received messages. |
| Tcl/Tk startup error | Use a Python installation with Tcl/Tk included and run from the intended virtual environment. |

## Development and verification

| Module | Responsibility |
|---|---|
| `horizon.py` | App startup, tabs, defaults, and shutdown |
| `mqtt_broker.py` | Background embedded broker and diagnostic log |
| `aim_tab.py`, `aim_mqtt.py` | AIM controls, MQTT subscriber, decoding, logging |
| `ground_tabs.py`, `ground_receivers.py`, `featherweight.py` | Witness feed and Featherweight reception |
| `flight_view.py`, `flight_render.py`, `flight_data.py` | Dashboard controls, drawing, validated layouts, histories |
| `flight_editor.py` | Draft layout editing and JSON save/load |
| `attitude_view.py`, `attitude_data.py`, `attitude_math.py` | Attitude tab drawing, live/CSV sample handling, pure quaternion math |
| `aim_altitude.py`, `aim_orientation.py` | Derived AIM altitude, velocity, and angular rates |
| `main.py`, `witness_link.py`, `witness_test_source.py` | Witness monitor, local relay, test playback |
| `converter.py`, `viewer.py` | Postflight conversion and analysis |

Run the regression suite from this folder:

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -q
```

Broker tests use isolated loopback ports rather than the live AIM port. They cover MQTT delivery through the AIM receiver, QoS, wildcard subscriptions, retained messages, startup defaults, checkbox shutdown/restart, missing dependencies, and port conflicts. GUI tests require a working Tcl/Tk environment. These tests do not replace verification with the actual receivers and deployed firmware before a flight.

## Shared firmware protocol

Iris channel currents use 20 mA/count; the monitor and converter decode this scale.
Earlier captures encoded at 100 mA/count require a matching older decoder. The
wire layout has no version field to distinguish those scales automatically.
Witness forwards Iris camera/debug telemetry received over CAN to USB and, when
enabled, LoRa. The CAN fragment envelope is removed before reaching Horizon.
See [shared CAN contract](../Intercomm/README.md).
