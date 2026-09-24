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
| `0x02` | Estimated state | 62 | 64 | 1 Hz | latest frame at 1 Hz |
| `0x03` | Camera telemetry | 17 | 19 | not currently produced | 1 Hz when available |
| `0x05` | Command | 8 | 10 | received externally | immediate; latest local frame at 1 Hz |
| `0xE0` | Witness debug | 101 | 103 | 10 Hz after debug enable | latest frame at 1 Hz |
| `0xE1` | IRIS debug | 12 | 14 | not currently produced | 1 Hz when available |
| `0xFF` | Heartbeat | 6 | 8 | 1 Hz | latest frame at 1 Hz |

All offsets in the packet tables below are absolute frame offsets: packet ID
is byte 0, the first payload byte is byte 1, and EOF is the last byte.

## 2. Common field encodings

| Field | Wire type | Conversion |
|---|---|---|
| Witness/IRIS uptime | `uint32` BE | milliseconds since boot; wraps after about 49.7 days |
| Witness/IRIS MCU temperature | `int8` | degrees C = raw |
| Witness/IRIS battery | `uint16` BE | millivolts = raw |
| Filtered acceleration | `int24` BE | g = raw x 0.000040 |
| Quaternion X/Y/Z/W | float32 BE | unitless normalized quaternion |
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
| Channel current | `uint8` | amperes = raw x 0.1 |
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

Status byte 1 bits 0 through 2 contain the flight state. Bits 3 through 7 are
reserved and transmitted as zero.

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

The estimator is not implemented yet. The firmware currently generates the
"filtered acceleration" fields from the most recent low-g accelerometer
samples, converted to the defined 40 micro-g/count format.

### `0x02` Estimated-state packet, 64 bytes

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
| 63 | 1 | EOF `0x0A` |

Currently only status, uptime, barometric altitude, and battery voltage are
populated. Estimator and GPS fields are zero until those producers exist.

### `0x03` Camera telemetry packet, 19 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x03` |
| 1 | 2 | Flight-controller/Witness status |
| 3 | 4 | Flight-controller/Witness uptime, ms |
| 7 | 2 | IRIS status |
| 9 | 4 | IRIS uptime, ms |
| 13 | 1 | Channel 1 current, 0.1 A/count |
| 14 | 1 | Channel 2 current, 0.1 A/count |
| 15 | 1 | Channel 3 current, 0.1 A/count |
| 16 | 2 | IRIS battery voltage, mV |
| 18 | 1 | EOF `0x0A` |

IRIS status is two raw bytes. Its individual flag assignments have not yet
been defined by this firmware.

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
| `0x6868` | Enter read-only USB mass-storage mode |
| `0x6869` | Erase all stored log sessions |
| `0x7300` through `0x7304` | Set flight state using the values above |

The last three commands also have legacy compact USB forms described in
section 7. Canonical commands received from USB or LoRa are retransmitted
byte-for-byte on UART2. The firmware does not rewrite their status or uptime.
Radio commands are additionally interpreted locally.

### `0xFF` Heartbeat packet, 8 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xFF` |
| 1 | 2 | Witness status |
| 3 | 4 | Witness uptime, ms |
| 7 | 1 | EOF `0x0A` |

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

MCU temperature, estimator fields, and GPS fields currently remain zero.

### `0xE1` IRIS debug packet, 14 bytes

| Frame offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xE1` |
| 1 | 2 | IRIS status |
| 3 | 4 | IRIS uptime, ms |
| 7 | 1 | IRIS MCU temperature, int8 C |
| 8 | 2 | IRIS battery voltage, uint16 mV |
| 10 | 1 | Channel 1 current, 0.1 A/count |
| 11 | 1 | Channel 2 current, 0.1 A/count |
| 12 | 1 | Channel 3 current, 0.1 A/count |
| 13 | 1 | EOF `0x0A` |

## 5. LoRa/RA-01 settings

The RA-01/SX1262 link currently uses:

| Setting | Value |
|---|---|
| Packet type | LoRa |
| RF frequency | 915,000,000 Hz |
| TX power | 0 dBm |
| Spreading factor | SF7 |
| Bandwidth | 125 kHz |
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
traffic can reduce the achieved aggregate rate. Between transmissions the
radio is returned to continuous receive mode. DIO1 indicates RX completion,
and all radio work is performed by the sensor task so SPI3 access does not race
the sensors.

`SpiDataForwarder::kForwardExternalPacketsImmediately` controls the immediate
forwarding path and defaults to `true`. With it enabled, complete command frames
received over USB and complete canonical frames reconstructed from UART2 are
queued ahead of scheduled telemetry. Radio-originated packets are never sent
back to the radio, preventing an RF echo loop. Set the flag to `false` to make
all radio output use only the per-packet latest-frame schedule.

Every valid received radio packet up to the canonical maximum of 103 bytes is
forwarded unchanged to USB. A received packet beginning with `0x05` is also
forwarded unchanged to UART2 and queued for local interpretation. Debug enable,
mass storage, log erase, and flight-state commands execute locally. Load and
5 V regulator commands have no Witness-local hardware action because they are
implemented by the UART-connected device. SX1262 packets with a CRC error are
discarded.

## 6. UART2 bridge

UART2 uses:

| Setting | Value |
|---|---|
| Baud | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| ESP32 TX | GPIO4 |
| ESP32 RX | GPIO5 |

Command packets are written exactly as received. No length, newline, checksum,
or additional UART framing is added. UART2 receive data is copied unchanged to
USB serial. The UART byte stream is also parsed by fixed packet length; when a
complete canonical frame with a valid EOF is found, it enters the immediate
radio queue if that feature is enabled.

At boot, Witness transmits the two bytes `05 22` and waits one second for a
reply containing `05 22`. This is a link test, not a canonical 10-byte command.

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
decoding, and the ten decoded bytes are sent to UART2. A terminal's "send hex"
mode should instead send the ten binary bytes with no extra newline.

Legacy compact binary commands are accepted after a 20 ms idle interval:

```text
05 68 68       enter mass storage
05 68 69       erase stored sessions
05 73 00..04   set flight state
```

The same compact commands may be typed as ASCII hex followed by Enter. `TX 05
...` is another accepted ASCII form and forwards the decoded bytes to UART2.

USB input is echoed back byte-for-byte. A ground station using the same USB
connection for commands and telemetry must therefore recognize or suppress its
own echoed command bytes.

## 8. USB stream parsing

USB Serial/JTAG is not currently a telemetry-only channel. It may contain:

- canonical binary telemetry frames;
- received UART2 or radio bytes;
- echoed ground-station input; and
- human-readable ESP-IDF log lines.

A practical stream decoder should scan for known binary packet IDs and accept
a candidate only when the fixed-length final byte is `0x0A`:

```python
FRAME_LENGTH = {
    0x01: 23,
    0x02: 64,
    0x03: 19,
    0x05: 10,
    0xE0: 103,
    0xE1: 14,
    0xFF: 8,
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
sync word plus application CRC would be more robust.

## 9. Ground-station transmit checklist

1. Configure the LoRa modem exactly as described in section 5.
2. Build the complete canonical command frame in a byte array.
3. Encode status and uptime even if both are zero.
4. Encode the command as a big-endian uint16 at bytes 7 and 8.
5. Put `0x0A` at byte 9.
6. Send exactly ten LoRa payload bytes; do not send ASCII hexadecimal text over
   LoRa.
7. By default, expect the newest available frame of every produced packet type
   as its own LoRa payload at 1 Hz. Packet `0x02` remains a complete 64-byte
   payload.
