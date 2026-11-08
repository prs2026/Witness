# Aperture Ground-Station Firmware Handoff

## Role

This is the ground-station-side ESP32-S3 firmware. It is not the Witness
flight firmware.

Its responsibilities are:

1. Receive LoRa packets from Witness.
2. Forward received LoRa payloads unchanged to the host computer over USB
   Serial/JTAG.
3. Accept canonical command frames from the host computer and transmit them
   unchanged to Witness over LoRa.
4. Publish a ground-station status frame to the host once per second.

It does not generate Witness telemetry or periodic LoRa heartbeats.

## Source files

- `main/main.cpp`: USB bridge, command parser, received-packet forwarding, and
  ground-station status packet generation.
- `main/ra01.cpp` / `main/ra01.h`: SX1262 command-mode driver.
- `main/hardware.h`: authoritative pin map and radio configuration.
- `main/comms.h`: canonical packet IDs, field offsets, and frame lengths.
- `main/GROUND_STATION_PROTOCOL.md`: protocol description for host software.

## Hardware and radio configuration

The current values in `main/hardware.h` are authoritative:

| Signal | ESP32-S3 GPIO |
|---|---:|
| SPI SCK | 36 |
| SPI MOSI | 35 |
| SPI MISO | 37 |
| NSS/CS | 4 |
| RESET | 12 |
| BUSY | 34 |
| DIO1 | 10 |
| DIO2 | 11 |
| DIO3 | 9 |
| RF_EN | 3 |

Current LoRa settings:

- Frequency: 926 MHz (`IRIS_RADIO_FREQUENCY_HZ`)
- Spreading factor: SF7
- Bandwidth: 125 kHz
- Coding rate: 4/5
- Preamble: 8 symbols
- Explicit header
- CRC enabled
- Normal IQ
- SX1262 TX power setting: -2 dBm
- Maximum received packet buffer: 128 bytes

DIO2 RF-switch control is enabled through `SetDIO2AsRfSwitchCtrl`. The
`IRIS_RADIO_DIO2_RF_SWITCH_INVERT` option selects the opposite command state;
the SX1262 does not expose a separate polarity bit. DIO2 is also configured as
an MCU input when `IRIS_RADIO_DIO2_MCU_INPUT` is enabled.

## LoRa receive path

`Ra01::initialize()` resets the module, enables the FEM, configures the SX1262
for LoRa, and enters continuous receive mode.

`Ra01::receive()`:

- Checks `GetIrqStatus()`.
- Discards CRC-error packets.
- Reads packet length and FIFO offset with `GetRxBufferStatus()`.
- Reads RSSI with `GetPacketStatus()` and stores it as the last valid packet
  RSSI.
- Reads payload bytes from the FIFO.
- Clears the RX IRQ.

The ESP32-S3 SPI host is used without DMA. FIFO reads are split into 60-byte
payload chunks so command overhead remains below the host transfer limit.

Valid received payloads are written to USB without modification. No packet ID,
length, checksum, or newline is added.

## USB command uplink

The bridge accepts a complete canonical binary command frame:

```text
05 status0 status1 uptime[4] command[2] 0A
```

The frame is exactly 10 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0x05` |
| 1 | 2 | Sender status |
| 3 | 4 | Sender uptime, big-endian milliseconds |
| 7 | 2 | Command, big-endian uint16 |
| 9 | 1 | EOF `0x0A` |

USB bytes are echoed byte-for-byte. Once a complete 10-byte frame ending in
`0x0A` is received, it is transmitted unchanged over LoRa. The parser searches
for packet ID `0x05` to resynchronize.

The current `main.cpp` implements canonical binary command input. ASCII-hex,
newline-terminated, and legacy compact command forms described in the protocol
document are not currently parsed by this bridge.

## USB-only ground-station status

Once per second, the bridge sends a 13-byte status frame to the host. It is not
sent over LoRa:

```text
F0 RSSI uptime[4] reserved[6] 0A
```

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xF0` |
| 1 | 1 | Last valid LoRa RSSI, signed dBm |
| 2 | 4 | Ground-station uptime, big-endian milliseconds |
| 6 | 6 | Reserved, currently zero |
| 12 | 1 | EOF `0x0A` |

Before the first valid LoRa packet, RSSI is reported as `-127 dBm`.

## USB stream caveat

The USB stream is mixed. It can contain:

- Human-readable ESP-IDF logs.
- Echoed USB command bytes.
- Received LoRa payloads.
- The periodic `0xF0` ground-station status frame.

Host software must use packet IDs and fixed frame lengths from `comms.h` and
validate the final `0x0A`. It must not search for the next newline or EOF byte,
because `0x0A` may occur inside payloads.

## Build and flash

Target: ESP32-S3, 2 MB flash, USB Serial/JTAG console.

The local ESP-IDF installation used previously was:

- IDF: `C:\esp\v6.1\esp-idf`
- Python: `C:\Espressif\tools\python\v6.1\venv`
- CMake: `C:\Espressif\tools\cmake\4.0.3\bin`
- Ninja: `C:\Espressif\tools\ninja\1.12.1`

The generated application image is:

```text
build/softwareattempt2.bin
```

Use the ESP-IDF extension or the generated esptool command from `idf.py
build`. Confirm the serial port after flashing because USB Serial/JTAG can
enumerate under a different COM number.

## Important troubleshooting

- If initialization reports `BUSY timeout`, check module power, reset, BUSY
  wiring, and the GPIO values in `hardware.h`.
- If USB disconnects during RF operation, check the RA-01SH-P 3V3/VCCPA rail
  with an oscilloscope. The FEM can cause transient current demand and a
  brownout can disconnect USB without a software backtrace.
- If LoRa packets are received but host output appears as garbage, remember
  that received payloads are intentionally forwarded as raw binary.
- Keep the host ground station and Witness on identical LoRa settings,
  including frequency, bandwidth, SF, coding rate, preamble, CRC, header, IQ,
  and sync word.

## Shared protocol revision

The four comms.h copies now share Iris status flags, flight-state definitions,
all command values, and the 20 mA/count current scale. Aperture continues to
forward canonical packets unchanged. Witness–Iris CAN fragmentation is internal
to those boards and never appears on LoRa or USB. See
[shared CAN contract](../../Intercomm/README.md).

Remote bench: `prs2026hitl`, programming RFC2217 port 7001, raw USB data TCP
port 7101. Run `flashit.ps1` from this folder to flash the Aperture image.
