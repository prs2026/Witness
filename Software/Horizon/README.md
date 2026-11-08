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

## Layout Editor

Use **Layout Editor** to customize the dashboard without editing JSON by hand:

1. Choose **Use current view**, **Load JSON**, or **Defaults** to initialize the draft.
2. Select a graph, readout, or trace in the tree. Edit its title, units, input, packet/topic, field, source, scale, offset, color, or filter.
3. Use Add graph/readout/trace, Remove, and Move up/down to arrange content. Graph order runs left-to-right, then top-to-bottom. Readouts belong to the flight, state, GPS, or power section.
4. Choose **Apply to Flight View** to validate and apply the draft. Confirming resets graph history and source bindings; connections and logging continue.
5. Choose **Save JSON as...** to keep the setup. Saving and applying are separate operations.

The editor may scroll; Flight View never does. A fixed dashboard supports at most six graphs in two columns and three rows, with up to three flight readout rows, one state row, six GPS rows, and ten power rows. Validation rejects layouts exceeding these limits.

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

Replay JSON has `version: 1`, `duration_ms`, and a `frames` list containing ordered `at_ms` times and hexadecimal byte strings. See [test_data/example_packets.json](test_data/example_packets.json) and [witness_test_source.py](witness_test_source.py).

## Logs and postflight analysis

Logs are written under `logs/` next to the scripts:

| File family | Contents |
|---|---|
| `serial_*.bin`, `TCP_*.bin` | Witness monitor raw transport bytes |
| `debug_*.log` | Witness monitor console and parse diagnostics |
| `test_*.bin` | Simulated Witness bytes |
| `horizon_witness_*.jsonl` | Decoded Witness feed records and diagnostics |
| `horizon_featherweight_*.bin`, `horizon_featherweight_*.jsonl` | Featherweight bytes and decoded/diagnostic records |
| `aim_mqtt_*.jsonl` | AIM message payloads, decoded fields, and connection diagnostics |
| `mqtt_broker_*.log` | Embedded broker lifecycle and warnings/errors |

Broker diagnostic logs rotate at 5 MB with two backups per run. This does not limit the other telemetry log files or remove logs from previous runs; monitor disk space. Broker logs are not a replacement for the AIM message log.

Convert a Witness binary log and open it for plotting:

```powershell
.\.venv\Scripts\python.exe converter.py S0000434.BIN --output S0000434.csv
.\.venv\Scripts\python.exe viewer.py S0000434.csv
```

The converter detects raw Witness application streams and 48-byte `WL` flash records from Witness mass storage. It does not decode Featherweight BIN captures or AIM JSONL logs. CSV `time_seconds` derives from device uptime, not host receipt time. The viewer opens with a summary and lets you select traces, including flight state and individual status bits.

## Firmware protocol references

[GROUND_STATION_PROTOCOL.md](GROUND_STATION_PROTOCOL.md) describes the Witness wire protocol and ground-station status packets; [comms.h](comms.h) contains the local field definitions. Application frames use packet ID, a payload whose size depends on that ID, and final `0x0A`. Multi-byte values are big-endian. `0x0A` can occur inside payloads, so splitting the stream on newlines is not valid deframing.

The monitor and converter also accept captured sensor payloads of 23 bytes alongside the 21-byte documented form. Check the deployed firmware revision before changing lengths or scaling. Witness flash `WL` records are a different format from application frames.

Commands are sent from the Witness monitor, not Horizon. All command shortcuts, including USB mass storage and log erase, use full 10-byte canonical frames so they can cross the Aperture LoRa bridge. Erasing logs is destructive; do not use it as a connectivity test. Other command shortcuts use the monitor’s framed command encoder. This guide does not describe how to build or flash device firmware.

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
