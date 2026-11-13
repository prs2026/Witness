# Witness Ground-Station Communications Standard

This document describes the protocol implemented by the Witness firmware. The
authoritative machine-readable definitions remain in `main/comms.h`; this file
explains how a ground station should encode, transmit, find, and decode them.

## 1. Canonical application frame

All defined application packets use the same framing:

```text
+-----------+----------------------+------------+
| packet ID | fixed-length payload | EOF = 0x0A |
| 1 byte    | determined by ID     | 1 byte     |
+-----------+----------------------+------------+
```

- Multi-byte integers are big-endian.
- Signed integers use two's-complement representation.
- Floating-point values are big-endian IEEE-754 binary32 values.
- There is currently no application-level CRC. LoRa uses the SX1262 hardware
  CRC.
- `0x0A` is not escaped and may occur inside a payload. Never search for the
  next `0x0A` to find a frame boundary. Read the packet ID, select its fixed
  frame length, and check `0x0A` only at that frame's final byte.

Packet lengths include the packet ID and EOF:

| ID | Name | Payload bytes | Frame bytes | Local/USB rate | Default radio rate |
|---:|---|---:|---:|---|---|
| `0x01` | Sensor | 21 | 23 | 10 Hz | latest frame at 1 Hz |
| `0x02` | Estimated state | 63 | 65 | 1 Hz | latest frame at 1 Hz |
| `0x03` | Camera telemetry | 17 | 19 | Iris CAN, 1 Hz | latest frame at 1 Hz |
| `0x05` | Command | 8 | 10 | received externally | immediate; latest local frame at 1 Hz |
| `0x0C` | HITL sensor input | 45 | 47 | host to Witness over USB only, ~100 Hz | never transmitted |
| `0xE0` | Witness debug | 101 | 103 | 10 Hz after debug enable | latest frame at 1 Hz |
| `0xE1` | IRIS debug | 12 | 14 | Iris CAN, 10 Hz after debug enable | latest frame at 1 Hz |
| `0xFF` | Heartbeat | 6 | 8 | 1 Hz | latest frame at 1 Hz |
| `0xF0` | Ground-station status | 27 | 29 | Aperture USB, 1 Hz | never transmitted |

All offsets in the packet tables below are absolute frame offsets: packet ID
is byte 0, the first payload byte is byte 1, and EOF is the last byte.

## 2. Common field encodings

| Field | Wire type | Conversion |
|---|---|---|
| Witness/IRIS uptime | `uint32` BE | milliseconds since boot; wraps after about 49.7 days |
| Witness/IRIS MCU temperature | `int8` | degrees C = raw |
| Witness/IRIS battery | `uint16` BE | millivolts = raw |
| Filtered acceleration | `int24` BE | g = raw x 0.000040 |
| Quaternion X/Y/Z/W | float32 BE | unitless normalized Hamilton quaternion, scalar last, body to ENU world; all zero = unavailable |
| Position X/Y/Z | float32 BE | meters, estimator local frame |
| Velocity X/Y/Z | float32 BE | meters/second, estimator local frame |
| LSM6 low-g acceleration | `int16` BE | g = raw x 0.000488 |
| LSM6 high-g acceleration | `int16` BE | g = raw x 0.010417 |
| LSM6 angular rate | `int16` BE | degrees/second = raw x 0.070 |
| LSM6 temperature | `int16` BE | degrees C = 25 + raw / 256 |
| MS5607 pressure | `uint24` BE | pascals = raw |
| MS5607 temperature | `int16` BE | degrees C = raw / 100 |
| MS5607 altitude | float32 BE | meters relative to 1013.25 mbar |
| Latitude/longitude | `int24` BE | degrees = raw / 10,000; north/east positive |
| GPS time | `uint32` BE | Unix seconds |
| GPS altitude | float32 BE | meters above mean sea level |
| GPS satellites | `uint8` | satellites used in the solution (NMEA GGA field 7) |
| GPS fix quality | `uint8` | NMEA GGA field 6: 0 no fix, 1 GPS, 2 DGPS, ... |
| Channel current | `uint8` | amperes = raw x 0.02 |
| Command | `uint16` BE | enumeration; no scaling |

To decode a signed 24-bit integer:

```python
def i24be(data: bytes) -> int:
    value = int.from_bytes(data, "big", signed=False)
    return value - (1 << 24) if value & 0x800000 else value
```

## 3. Witness status bytes

The two-byte Witness status field appears in packets `0x01`, `0x02`, `0x05`,
`0xE0`, and `0xFF`.

Status byte 0 contains active-high flags:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x01` | Heartbeat phase; toggles once per heartbeat |
| 1 | `0x02` | Valid LSM6 low-g acceleration available |
| 2 | `0x04` | Valid LSM6 gyro sample available |
| 3 | `0x08` | Valid LSM6 temperature available |
| 4 | `0x10` | External NAND initialization or startup test failed |
| 5 | `0x20` | Valid LSM6 high-g acceleration available |
| 6 | `0x40` | Valid MS5607 sample available |
| 7 | `0x80` | Flash initialized but logging later failed |

Status byte 1 bits 0 through 2 contain the flight state. Bits 3 and 4 are
flags; bits 5 through 7 are reserved and transmitted as zero.

| Bit | Mask | Meaning |
|---:|---:|---|
| 3 | `0x08` | GPS locked: a fresh GGA fix (quality >= 1, within 3 s) exists |
| 4 | `0x10` | HITL mode active: sensor, GPS and battery values are simulated |

Decode the flight state as `byte1 & 0x07`, not as the whole byte.

| Value | Flight state |
|---:|---|
| `0x00` | Pad idle |
| `0x01` | Boost |
| `0x02` | Coast |
| `0x03` | Descent |
| `0x04` | Landed |

## 4. Packet layouts

### `0x01` Sensor packet, 23 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x01` |
| 1 | 2 | Witness status |
| 3 | 4 | Witness uptime, ms |
| 7 | 3 | Filtered acceleration X, signed int24 |
| 10 | 3 | Filtered acceleration Y, signed int24 |
| 13 | 3 | Filtered acceleration Z, signed int24 |
| 16 | 2 | Gyro X, signed int16 |
| 18 | 2 | Gyro Y, signed int16 |
| 20 | 2 | Gyro Z, signed int16 |
| 22 | 1 | EOF `0x0A` |

No acceleration filter is implemented yet. The firmware currently generates the
"filtered acceleration" fields from the most recent low-g accelerometer
samples, converted to the defined 40 micro-g/count format.

### `0x02` Estimated-state packet, 65 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x02` |
| 1 | 2 | Witness status |
| 3 | 4 | Witness uptime, ms |
| 7 | 4 | Quaternion X, float32 |
| 11 | 4 | Quaternion Y, float32 |
| 15 | 4 | Quaternion Z, float32 |
| 19 | 4 | Quaternion W, float32 |
| 23 | 4 | Position X, float32 m |
| 27 | 4 | Position Y, float32 m |
| 31 | 4 | Position Z, float32 m |
| 35 | 4 | Velocity X, float32 m/s |
| 39 | 4 | Velocity Y, float32 m/s |
| 43 | 4 | Velocity Z, float32 m/s |
| 47 | 3 | Latitude, signed int24 |
| 50 | 3 | Longitude, signed int24 |
| 53 | 4 | GPS altitude, float32 m MSL |
| 57 | 4 | MS5607 barometric altitude, float32 m |
| 61 | 2 | Witness/FC battery voltage, uint16 mV |
| 63 | 1 | GPS satellites used, uint8 |
| 64 | 1 | EOF `0x0A` |

Status, uptime, the orientation quaternion, barometric altitude, battery
voltage, valid UART1 GPS fixes and the satellite count are populated; position
and velocity remain zero. Satellites come from the most recent checksum-valid
GGA within 3 s and are reported even when that GGA has no fix (position fields
are then zero); they are 0 when stale.

Quaternion convention (`0x02` and `0xE0`): Hamilton quaternion sent X, Y, Z,
W (scalar last), rotating BODY vectors into WORLD (`v_world = q v_body q^-1`).
WORLD is local ENU (+X east, +Y north, +Z up). BODY is the LSM6 sensor frame,
with +X along the rocket nose axis, so an upright rocket on the pad has body +X
pointing to world +Z. **All four components zero means attitude is not
available**: before the first accepted 1 s stationary window on the pad,
after HITL entry/exit until a new window is accepted, or when no window was
accepted before launch. Heading (rotation about world +Z) is not observable without a
magnetometer; it is fixed arbitrarily by the shortest-arc tilt alignment at
initialization (an exactly upright rocket gets body +Y = north, body +Z =
west), so compare yaw only relative to that start. The attitude is gyro
propagated on the pad with a gated accelerometer tilt correction, purely gyro
integrated in Boost/Coast/Descent, and held constant from Landed onwards.

### `0x03` Camera telemetry packet, 19 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x03` |
| 1 | 2 | Flight-controller/Witness status |
| 3 | 4 | Flight-controller/Witness uptime, ms |
| 7 | 2 | IRIS status |
| 9 | 4 | IRIS uptime, ms |
| 13 | 1 | Channel 1 current, 0.02 A/count |
| 14 | 1 | Channel 2 current, 0.02 A/count |
| 15 | 1 | Channel 3 current, 0.02 A/count |
| 16 | 2 | IRIS battery voltage, mV |
| 18 | 1 | EOF `0x0A` |

IRIS status flags are defined in `comms.h`: monitor error, battery error, command flag, OUT1/2/3 enabled, and 5 V enabled.

### `0x05` Command packet, 10 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x05` |
| 1 | 2 | Witness status supplied by sender |
| 3 | 4 | Sender uptime in milliseconds |
| 7 | 2 | Command value, uint16 BE |
| 9 | 1 | EOF `0x0A` |

Example command packet with zero status/uptime and command `0x0051`:

```text
05 00 00 00 00 00 00 00 51 0A
```

Command values currently defined:

| Value | Meaning |
|---:|---|
| `0x0030` | Channel 1 load off |
| `0x0031` | Channel 1 load on |
| `0x0032` | Channel 2 load off |
| `0x0033` | Channel 2 load on |
| `0x0034` | Channel 3 load off |
| `0x0035` | Channel 3 load on |
| `0x0050` | 5 V regulator off |
| `0x0051` | 5 V regulator on |
| `0x00E0` | Enable Witness debug packets at 10 Hz |
| `0x4849` | Enter HITL mode (USB only, Pad Idle only; see section 4a) |
| `0x4858` | Exit HITL mode (USB or LoRa) |
| `0x6868` | Enter read-only USB mass-storage mode |
| `0x6869` | Erase all stored log sessions |
| `0x7300` through `0x7304` | Set flight state using the values above |

The last three commands also have legacy compact USB forms described in
section 7. Canonical commands received from USB or LoRa are retransmitted
byte-for-byte through the Iris CAN link. The firmware does not rewrite their status or uptime.
Radio commands are additionally interpreted locally.

### `0xFF` Heartbeat packet, 8 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xFF` |
| 1 | 2 | Witness status |
| 3 | 4 | Witness uptime, ms |
| 7 | 1 | EOF `0x0A` |

### `0xF0` Ground-station status packet, 29 bytes

Aperture emits this frame only over USB once per second. It is never sent over
LoRa. Its GPS fields come from Aperture's own Quectel LC86 receiver in the pad
box and give the launch pad position when the Featherweight ground station's
location is not available.

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xF0` |
| 1 | 1 | Last valid LoRa RSSI, int8 dBm (`-127` before the first packet) |
| 2 | 4 | Station uptime, uint32 ms |
| 6 | 1 | GPS fix quality, uint8, NMEA GGA field 6 (0 = no fix) |
| 7 | 1 | GPS satellites used, uint8 (GGA field 7) |
| 8 | 4 | Latitude, int32, 1e-7 degree/count, north positive |
| 12 | 4 | Longitude, int32, 1e-7 degree/count, east positive |
| 16 | 4 | GPS altitude, float32 m above mean sea level |
| 20 | 4 | GPS time, uint32 Unix seconds (0 = unknown) |
| 24 | 2 | HDOP, uint16, 0.01/count (`0xFFFF` = unknown) |
| 26 | 2 | Fix age, uint16 ms since the reported position (`0xFFFF` = none, or 65.535 s or older) |
| 28 | 1 | EOF `0x0A` |

Payload offsets in `comms.h` (`IRIS_PACKET_GROUND_STATION_STATUS_*_OFFSET`)
are these frame offsets minus one. All multi-byte fields are big-endian.

- Fix quality, satellites and HDOP come from the most recent checksum-valid
  GGA, including a no-fix GGA. When that GGA is 3 s old or older (or none has
  arrived), quality and satellites are 0 and HDOP is `0xFFFF`.
- Latitude, longitude and altitude hold the last valid fix (GGA quality 1..8
  with a parseable position), however old; use the fix-age field to judge it.
  They are zero, with fix age `0xFFFF`, until a fix has been seen since boot.
- GPS time is the UTC of the last RMC with status `A`, advanced by the
  station's elapsed time since that RMC arrived, so it keeps counting through a
  dropout. It is 0 until such an RMC has been received.
- Hosts should still accept the pre-GPS 13-byte form (11-byte payload: RSSI,
  uptime, six zero bytes; `IRIS_PACKET_GROUND_STATION_STATUS_LEGACY_DATA_LENGTH`).

### `0xE0` Witness debug packet, 103 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xE0` |
| 1 | 2 | Witness status |
| 3 | 4 | Witness uptime, ms |
| 7 | 1 | Witness MCU temperature, int8 C |
| 8 | 2 | Witness battery voltage, uint16 mV |
| 10, 13, 16 | 3 each | Filtered acceleration X/Y/Z, int24 |
| 19, 23, 27, 31 | 4 each | Quaternion X/Y/Z/W, float32 |
| 35, 39, 43 | 4 each | Position X/Y/Z, float32 m |
| 47, 51, 55 | 4 each | Velocity X/Y/Z, float32 m/s |
| 59, 61, 63 | 2 each | LSM6 low-g X/Y/Z, int16 |
| 65 | 2 | LSM6 temperature, int16 |
| 67, 69, 71 | 2 each | LSM6 high-g X/Y/Z, int16 |
| 73, 75, 77 | 2 each | LSM6 gyro X/Y/Z, int16 |
| 79 | 3 | MS5607 pressure, uint24 Pa |
| 82 | 2 | MS5607 temperature, int16 centi-C |
| 84 | 4 | MS5607 altitude, float32 m |
| 88 | 3 | Latitude, int24 |
| 91 | 3 | Longitude, int24 |
| 94 | 4 | GPS Unix time, uint32 s |
| 98 | 4 | GPS altitude, float32 m MSL |
| 102 | 1 | EOF `0x0A` |

MCU temperature, position, velocity and GPS Unix time remain zero. The
quaternion is populated exactly as in `0x02` (same convention; all zero until
the estimator is initialized), so enabling debug output with command `0x00E0`
gives 10 Hz quaternions on USB (LoRa still sends the newest debug frame at its
1 Hz radio rate). GPS latitude, longitude and MSL altitude come from UART1 NMEA
GGA fixes.

### `0xE1` IRIS debug packet, 14 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xE1` |
| 1 | 2 | IRIS status |
| 3 | 4 | IRIS uptime, ms |
| 7 | 1 | IRIS MCU temperature, int8 C |
| 8 | 2 | IRIS battery voltage, uint16 mV |
| 10 | 1 | Channel 1 current, 0.02 A/count |
| 11 | 1 | Channel 2 current, 0.02 A/count |
| 12 | 1 | Channel 3 current, 0.02 A/count |
| 13 | 1 | EOF `0x0A` |

## 4a. HITL (hardware-in-the-loop) mode

HITL replaces Witness's IMU, barometer, GPS and battery readings with values
from host `0x0C` packets, so the flight state machine, telemetry and flash
logging run on simulated data. Real IMU, barometer and battery reads stop;
LoRa, CAN and logging continue.

- **Enter**: canonical 10-byte command `0x4849` (for example
  `05 00 00 00 00 00 00 48 49 0A`), accepted only from USB and only in Pad
  Idle. From LoRa, in another flight state, or in compact `05 48 49` form it is
  rejected and logged. Reboot always starts with HITL off.
- **Exit**: canonical command `0x4858` from USB or LoRa, or automatically when
  no valid `0x0C` packet arrives for 1000 ms (from entry or from the last
  packet). Exit does not change the flight state; send `0x7300` to return to
  Pad Idle if the simulation launched.
- **Detect**: status byte 1 bit 4 (`0x10`) is set while HITL is active.
- **Between enter and the first `0x0C` packet** all readiness flags (byte 0
  bits 1, 2, 3, 5, 6) are clear, battery and barometer fields are zero, the
  GPS reports no fix and 0 satellites, no `0x01` packet is produced and the
  flight state machine receives no input. No real reading is mixed in. The
  same reset happens on exit until real sensors produce new data.
- While active, each `0x0C` packet sets the low-g, high-g, gyro and IMU
  temperature ready flags; the barometer flag is set when pressure is non-zero.
  Barometric altitude is computed from the pressure with the MS5607 formula.
  The GPS fields feed state, debug and the GPS-locked bit exactly like a UART
  GGA (quality 1..8 is a fix; satellites are reported without one).
- The attitude estimator is reset on HITL entry and exit, then runs on the
  `0x0C` gyro and low-g accelerometer counts exactly as on real data: the
  quaternion stays all zero until a 1 s stationary window of simulated
  samples is accepted in Pad Idle (every |a| 0.95..1.05 g, accel scatter
  <= 0.02 g, gyro median <= 5 dps with at most 40% of samples more than 10 dps
  from it). Its time step is the Witness time of the sensor pass that consumed
  each frame (the `sim_time_ms` field is not used); a pass without a new frame
  is skipped and the next step spans 20 ms. The gyro passes through a 5-sample
  running median, so the reported attitude lags by 2 samples (20 ms). Send
  frames at a steady 100 Hz.
- HITL commands are bridged to Iris like any other `0x05` command (Iris ignores
  them). `0x0C` packets are never forwarded to CAN or LoRa.

### `0x0C` HITL sensor input packet, 47 bytes (host to Witness, USB only)

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x0C` |
| 1 | 2 | Sender status, ignored |
| 3 | 4 | Simulation time, uint32 ms (logged only) |
| 7, 9, 11 | 2 each | LSM6 low-g X/Y/Z, int16, 0.488 mg/count |
| 13, 15, 17 | 2 each | LSM6 high-g X/Y/Z, int16, 10.417 mg/count |
| 19, 21, 23 | 2 each | LSM6 gyro X/Y/Z, int16, 70 mdps/count |
| 25 | 2 | LSM6 temperature, int16, C = 25 + raw / 256 |
| 27 | 3 | Pressure, uint24 Pa (0 = no barometer data) |
| 30 | 2 | Barometer temperature, int16 centi-C |
| 32 | 3 | Latitude, int24, 0.0001 degree |
| 35 | 3 | Longitude, int24, 0.0001 degree |
| 38 | 4 | GPS altitude, float32 m MSL |
| 42 | 1 | GPS satellites used, uint8 |
| 43 | 1 | GGA fix quality, uint8 (0 = no fix) |
| 44 | 2 | Battery voltage, uint16 mV |
| 46 | 1 | EOF `0x0A` |

Send one complete frame per write, back to back at about 100 Hz (at least
every 1000 ms to stay in HITL). Witness frames them by fixed length and checks
EOF at byte 46; on a bad EOF it resynchronizes on the next `0x0C` byte. A `0x0C`
byte starts a frame unless an unfinished binary `0x05` command is in
progress; a `0x0C` or `0x05` byte discards any unterminated text line first.
Send commands as complete canonical frames between `0x0C` frames. Bytes consumed as `0x0C` frames, valid or not,
are **not echoed**; all other USB input is still echoed. Frames received while
HITL is inactive are discarded with a rate-limited log line.

## 5. LoRa/RA-01 settings

Witness radio is enabled in the checked-in configuration (`HW_RADIO_ENABLED=1`).
The RA-01/SX1262 link uses:

| Setting | Value |
|---|---|
| Packet type | LoRa |
| RF frequency | 926,000,000 Hz |
| TX power | -8 dBm (Witness), -2 dBm (Aperture) |
| Spreading factor | SF7 |
| Bandwidth | 500 kHz (SX1262 erratum 15.1 applied before every TX; see HITL/LORA_MODULATION_OPTIONS.md) |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Header | Explicit |
| Payload CRC | Enabled |
| IQ | Normal, not inverted |
| Payload length | Variable; set to the canonical frame length for TX |

The firmware does not currently program the LoRa sync-word registers. Both
ends must use the same SX1262 reset/default sync-word setting, or the firmware
should be updated to configure an explicit value before deployment.

Every locally generated canonical packet updates a latest-frame cache keyed by
packet ID. The radio scheduler sends the newest cached frame at the independent
per-type rates declared as `kRadio*PacketRateHz` in `main/sensors.h`. All seven
rates default to 1 Hz. Changing a radio rate does not change the packet's normal
producer or USB rate. A packet type that has not yet been produced is skipped.
Rates must be non-zero integer divisors of 1000 Hz.

Only one scheduled frame is transmitted per sensor-task pass. Due packet types
are serviced round-robin, so simultaneous deadlines do not permanently favor
one ID. The requested rates are upper targets: LoRa airtime and immediate
traffic can reduce the achieved aggregate rate. Transmission is non-blocking:
the sensor task loads the frame and issues SetTx, then checks DIO1 for TxDone on
later 10 ms passes (about 43 ms of airtime for a 103-byte frame at SF7/500
kHz), so radio traffic no longer slows the 100 Hz sensor loop. A new frame is
started only after the previous one completes; reception is impossible while
transmitting. After each transmission the radio is returned to continuous
receive mode. DIO1 indicates RX or TX completion, and all radio work is
performed by the sensor task so SPI3 access does not race the sensors.

`SpiDataForwarder::kForwardExternalPacketsImmediately` controls the immediate
forwarding path and defaults to `true`. With it enabled, complete command frames
received over USB are
queued ahead of scheduled telemetry. Radio-originated packets are never sent
back to the radio, preventing an RF echo loop. Set the flag to `false` to make
all radio output use only the per-packet latest-frame schedule.

Every valid received radio packet up to the canonical maximum of 103 bytes is
forwarded unchanged to USB. A received packet beginning with `0x05` is also
queued unchanged for Iris CAN transmission and local interpretation. Debug enable,
mass storage, log erase, flight-state and HITL-exit commands execute locally;
HITL enter is rejected over LoRa. Load and
5 V regulator commands have no Witness-local hardware action because they are
implemented by the external device. The GPS UART never receives these
AMA load/regulator commands. SX1262 packets with a CRC error are
discarded.

## 6. Iris CAN and GPS UART1 interfaces

### Iris CAN device link

Iris and Witness use TCAN3414-compatible transceivers with the ESP32 Classical
CAN controller at **500 kbit/s**, standard 11-bit identifiers. Witness TX is
GPIO37 and RX GPIO36. Iris TX is GPIO3 and RX GPIO1. Witness-to-Iris uses
`0x120`; Iris-to-Witness uses `0x121`. CAN FD is not used.

Each CAN data frame carries `[sequence, fragment_index, canonical_length]`
followed by up to five consecutive application-frame bytes. Indices start at
zero; every fragment repeats the sequence and total length. DLC is exactly
three plus the bytes carried. Reassembly requires ordered fragments, matching
sequence/length, and completion within 250 ms. The common implementation is
`Software/Intercomm/can_transport.h`; packets retain their canonical ID and EOF.

USB compact commands are normalized to canonical 10-byte command frames;
malformed commands are rejected before forwarding. Commands go to Iris through
CAN, never to the GPS UART. Iris replies and camera/debug telemetry are mirrored
to Witness USB and cached for the normal radio schedule; replies are not echoed
back to CAN. Witness sends a 1 Hz heartbeat over CAN, and Iris uses its status
and uptime in camera telemetry, clearing those FC fields after 3 s without a
fresh heartbeat. Iris current fields use **20 mA/count**, with a 5.10 A maximum.
Camera currents are one-second peaks; debug currents are latest samples.

Both bus ends require 120 ohm termination and a common ground. TCAN3414 STB must
be low and SHDN low or left at its specified internal pull-down for normal mode;
these boards ground STB and have no MCU standby-control signal.

### GPS UART1 receiver

GPS uses UART1; the I2C driver is not started or included in the build.

| Setting | Value |
|---|---|
| ESP32 RX | GPIO4 <- GPS TX (GPIO5 while `HW_MCU_UART_SWAP_RX_TX` is 1, as currently built) |
| ESP32 TX | GPIO5 -> GPS RX (GPIO4 while swapped) |
| Ground | Shared with receiver |
| Logic voltage | 3.3 V |
| Operational baud | 115200, 8N1, no flow control |

At startup `GpsReceiver` (`main/gps_receiver.h`) identifies the receiver as a
SkyTraq PX1120S or a u-blox NEO-M8N by which binary protocol answers a poll,
then sets airborne 4 g dynamics, the receiver's maximum update rate, and NMEA
output of GGA + RMC only. A receiver already streaming NMEA at 115200 is never
baud-probed; only a silent link is retried at 9600 (the u-blox factory rate),
and a u-blox found there is switched to 115200. A five-second `GPS RX`
diagnostic reports host baud, total bytes, valid GGA sentence count, fix
validity and time since the last UART byte. A failed startup configuration does
not stop raw collection, sensors, heartbeat or flash logging.

| Receiver | Dynamics | Update rate | NMEA output |
|---|---|---|---|
| PX1120S | navigation mode 5, airborne (4 g rated) | 25 Hz (part maximum) | GGA + RMC |
| NEO-M8N | `dynModel` 8, airborne <4 g | 10 Hz (concurrent-GNSS maximum) | GGA + RMC |

Checksum-validated NMEA GGA sentences (including GP/GN talkers) populate
latitude, longitude and GPS MSL altitude in state (0x02) and Witness debug (0xE0)
packets. Arbitrarily fragmented UART reads are supported. Coordinates convert
from NMEA degrees/minutes and hemisphere into signed decimal degrees, then are
rounded to the canonical signed int24 scale of 0.0001 degree/count.
Altitude is big-endian float32 metres above mean sea level, not ellipsoid height.
GPS Unix time remains zero (GGA contains no date).

A no-fix GGA invalidates the current fix immediately, but its satellite count
is still reported. A GGA (and its fix) expires after
`NmeaGps::kFixTimeoutMs` (3000 ms). Invalid/missing/stale fixes emit zero in all
three GPS position fields and clear status byte 1 bit 3 (GPS locked); stale
GGA data also reports 0 satellites. Zero is not a unique invalid marker: a
legitimate zero coordinate/altitude has the same representation; use the
GPS-locked bit. Bad checksums are discarded and cannot refresh the fix age.
Raw UART bytes are logged unchanged regardless of fix validity.

Settings are in `main/gps_receiver.h`: `kConfigureAtStartup`,
`kSkytraqAirborneMode`, `kSkytraqUpdateRateHz`, `kUbloxAirborne4gModel`,
`kUbloxMeasurementPeriodMs`, startup delay, reply timeout and retry count. All
settings go to receiver RAM and are reapplied on every boot without writing
receiver flash. The PX1120S datasheet specifies a 4 g dynamics capability; its
binary protocol selects `airborne` rather than a separate numeric `4 g` profile.

SkyTraq binary frames use `A0 A1`, a two-byte big-endian payload length, the
payload, its XOR checksum, and `0D 0A`. PX1120S commands (SRAM-only attributes):

| Action | Complete frame (hex) | Reply |
|---|---|---|
| Query navigation mode (also the identification poll) | `A0 A1 00 02 64 18 7C 0D 0A` | `A0 A1 00 03 64 8B 05 EA 0D 0A` when airborne |
| Set airborne navigation | `A0 A1 00 04 64 17 05 00 76 0D 0A` | ACK `83 64 17` |
| NMEA output GGA + RMC every fix | `A0 A1 00 09 08 01 00 00 00 01 00 00 00 08 0D 0A` | ACK `83 08` |
| Position update rate 25 Hz | `A0 A1 00 03 0E 19 00 17 0D 0A` | ACK `83 0E` |
| Query update rate | `A0 A1 00 01 10 10 0D 0A` | `A0 A1 00 02 86 19 9F 0D 0A` at 25 Hz |

u-blox UBX frames use `B5 62`, class, id, a little-endian payload length, the
payload and a two-byte Fletcher checksum. NEO-M8N commands (RAM only):

| Action | Complete frame (hex) | Reply |
|---|---|---|
| Identification poll `MON-VER` | `B5 62 0A 04 00 00 0E 34` | any valid UBX frame |
| `CFG-PRT` UART1 115200, UBX+NMEA (only when found at 9600) | `B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 03 00 03 00 00 00 00 00 BC 5E` | host follows after 150 ms |
| `CFG-MSG` GLL / GSA / GSV / VTG off | `B5 62 06 01 03 00 F0 01 00 FB 11`, `… F0 02 00 FC 13`, `… F0 03 00 FD 15`, `… F0 05 00 FF 19` | `ACK-ACK` each |
| `CFG-MSG` GGA / RMC on every fix | `B5 62 06 01 03 00 F0 00 01 FB 10`, `… F0 04 01 FF 18` | `ACK-ACK` each |
| `CFG-NAV5` dynamic model 8 (mask 0x0001) | `B5 62 06 24 24 00 01 00 08 00` + 32 zero bytes + `57 F8` | `ACK-ACK` |
| `CFG-RATE` 100 ms, 1 cycle, GPS time | `B5 62 06 08 06 00 64 00 01 00 01 00 7A 12` | `ACK-ACK` |
| Readback | `CFG-NAV5` and `CFG-RATE` polls (empty payload) | `dynModel` 8, `measRate` 100 |

References: [SkyTraq AN0037 binary message protocol](https://www.skytraq.com.tw/homesite/AN0037.pdf)
and the u-blox M8 receiver description (UBX-13003221).
The supplied [PX1120S manual](c4dba7.pdf) lists the receiver's dynamics rating
and factory baud rate on PDF page 3.

Every received UART byte is also queued unchanged for flash before USB forwarding or
GPS parsing. Each boot exports three files with a
matching hexadecimal session number:

| File | Contents |
|---|---|
| `U0000001.BIN` | Raw external GPS payload bytes, received over UART1 |
| `S0000001.BIN` | Existing 48-byte sensor records, using the flight-state logging gate |
| `S0000001.TXT` | ESP-IDF diagnostic output |

GPS capture runs in every flight state, including Pad Idle and Landed. A
32 KiB RAM queue absorbs flash write delays. Flash staging uses 4,080-byte batches and flushes partial batches every
second. Entering read-only mass storage (`05 68 68`) stops capture and writes
all accepted bytes before publishing the files. GPS data arriving after
capture stops is not part of the exported snapshot. Sudden power loss can lose
the bytes still buffered in RAM; use the export command before disconnecting.

New sessions use storage layout version 2: 10 NAND blocks for sensor data,
40 for external GPS data and 12 for diagnostics, retaining 16 rotating sessions.
The GPS allocation holds up to 5,222,400 raw bytes per session; recording duration
depends on the receiver's actual output rate.
Old version-1 sensor and diagnostic sessions remain readable. No format or
erase is required to upgrade. Buffer overflow or flash write failure sets the
existing flash-log-failed status bit. When a file's allocation fills, that
stream stops with a diagnostic and the other streams continue. These are
finite buffers, not an unlimited lossless recorder.

Radio initialization failure disables radio RX/TX for the rest of the boot,
without repeated initialization attempts in the sensor loop. An SPI packet-RAM
round trip rejects an absent radio even when SPI writes return success.
`HW_RADIO_ENABLED` in `hardware_pins.h` may be set to `0` to skip initialization
entirely on boards with no radio fitted. GPS logging, USB, sensors and flash
continue operating without a radio.

## 7. USB command input

The preferred ground-station command format is a complete canonical 10-byte
binary frame. For command `0x0051`:

```text
05 00 00 00 00 00 00 00 51 0A
```

USB also accepts a newline-terminated ASCII-hex representation:

```text
05 00 00 00 00 00 00 00 51 0A\r\n
```

The spaces are part of the ASCII representation. They are removed during hex
decoding, and the ten decoded bytes are queued for Iris CAN. A terminal's "send hex"
mode should instead send the ten binary bytes with no extra newline.

Legacy compact binary commands are accepted after a 20 ms idle interval:

```text
05 68 68       enter mass storage
05 68 69       erase stored sessions
05 73 00..04   set flight state
```

HITL enter/exit (`0x4849`/`0x4858`) are accepted only as canonical 10-byte
frames (binary or ASCII hex), not in compact form.

The same compact commands may be typed as ASCII hex followed by Enter. `TX 05
...` is another accepted ASCII form and queues the decoded bytes for Iris CAN.

USB input is echoed back byte-for-byte, except bytes consumed as `0x0C` HITL
frames (section 4a). A ground station using the same USB connection for
commands and telemetry must therefore recognize or suppress its own echoed
command bytes.

## 8. USB stream parsing

USB Serial/JTAG is not currently a telemetry-only channel. It may contain:

- canonical binary telemetry frames;
- received GPS UART1 or radio bytes;
- echoed ground-station input; and
- human-readable ESP-IDF log lines.

A practical stream decoder should scan for known binary packet IDs and accept
a candidate only when the fixed-length final byte is `0x0A`:

```python
FRAME_LENGTH = {
    0x01: 23,
    0x02: 65,
    0x03: 19,
    0x05: 10,
    0xE0: 103,
    0xE1: 14,
    0xFF: 8,
    0xF0: 29,  # pre-GPS Aperture firmware sent 13
}

def extract_frames(buffer: bytearray):
    frames = []
    while buffer:
        length = FRAME_LENGTH.get(buffer[0])
        if length is None:
            del buffer[0]
            continue
        if len(buffer) < length:
            break
        if buffer[length - 1] != 0x0A:
            del buffer[0]
            continue
        frames.append(bytes(buffer[:length]))
        del buffer[:length]
    return frames
```

This is a resynchronizing parser, not a guarantee against false positives in a
mixed byte stream. For flight use, a dedicated binary channel or an additional
sync word plus application CRC would be more robust. `0x0C` is input only and
is never emitted by Witness, so it is absent from this table.

## 9. Ground-station transmit checklist

1. Configure the LoRa modem exactly as described in section 5.
2. Build the complete canonical command frame in a byte array.
3. Encode status and uptime even if both are zero.
4. Encode the command as a big-endian uint16 at bytes 7 and 8.
5. Put `0x0A` at byte 9.
6. Send exactly ten LoRa payload bytes; do not send ASCII hexadecimal text over
   LoRa.
7. By default, expect the newest available frame of every produced packet type
   as its own LoRa payload at 1 Hz. Packet `0x02` remains a complete 65-byte
   payload.



## Aperture host input

Aperture accepts only full canonical 10-byte binary command frames. ASCII-hex
and compact forms in section 7 apply to a direct Witness USB connection. Horizon
shortcuts emit full frames for both routes. Aperture transparently forwards
received LoRa payloads unchanged to USB. CAN fragment headers are never carried
over LoRa or exposed to the host.

Shared CAN transport reference: [Intercomm](../../../Intercomm/README.md).
