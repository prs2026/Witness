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
```

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
| `summary.json` | The pass/fail result. |

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
- The tool can only encode HITL ENTER `0x4849`, HITL EXIT `0x4858` and 0x0C
  frames. `encode_command` refuses every other value, including mass storage
  `0x6868` and log erase `0x6869`.

## Protocol summary

The authoritative layout is in `Software/Witness/softwarev1/main/comms.h`. The
rest of the wire protocol is in `Software/Horizon/GROUND_STATION_PROTOCOL.md`.

- **Framing:** frames are `ID | fixed payload | 0x0A`, big-endian. 0x0A can
  appear inside a payload, so frames are found by fixed length per ID and the
  EOF is checked at the last byte only. On a mismatch, the decoder resyncs one
  byte at a time.
- **Frame lengths:** 0x01 = 23, 0x02 = 65, 0x03 = 19, 0x05 = 10, 0xE0 = 103,
  0xE1 = 14, 0xFF = 8. The 0x0C echo is 47 and is ignored.
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
echoes input, honours ENTER and EXIT, auto-exits after 1 s, runs the reference
state machine, and emits 0x01, 0x02 and 0xFF frames mixed with log text, NMEA
and junk. To make these tests fast, the fake uses the frame sim time as its
clock and the tests run at `--time-scale 20`, a hidden option. **The real
Witness clocks transitions with its own timer, so always use the default scale
of 1 against hardware.** The tests never touch the network beyond localhost.
