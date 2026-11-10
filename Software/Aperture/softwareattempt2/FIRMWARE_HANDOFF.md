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
4. Publish a ground-station status frame to the host once per second,
   including Aperture's own LC86 GPS fix. Aperture sits in the pad box, so
   that fix gives Horizon the launch pad position when the Featherweight
   ground station's location is not available.

It does not generate Witness telemetry or periodic LoRa heartbeats.

## Source files

- `main/main.cpp`: USB bridge, command parser, received-packet forwarding, and
  ground-station status packet generation.
- `main/ra01.cpp` / `main/ra01.h`: SX1262 command-mode driver.
- `main/lc86_gps.cpp` / `main/lc86_gps.h`: LC86 reset, UART driver,
  configuration and PPS counter on a dedicated FreeRTOS task.
- `main/lc86_nmea.cpp` / `main/lc86_nmea.h`: pure C++ NMEA GGA/RMC and
  PAIR/PMTK acknowledgement parser (host-tested).
- `main/ground_station_status.cpp` / `.h`: pure C++ `0xF0` frame builder
  (host-tested).
- `tests/gps_status_test.cpp`: host test for the two pure modules.
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
| GPS UART RX (MCU RX <- LC86 TX) | 38 |
| GPS UART TX (MCU TX -> LC86 RX) | 39 |
| GPS PPS (input) | 18 |
| GPS RESET_N (open-drain, active low) | 14 |

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

Once per second, the bridge sends a 29-byte status frame
(`IRIS_PACKET_GROUND_STATION_STATUS_FRAME_LENGTH`) to the host. It is not sent
over LoRa. All multi-byte fields are big-endian:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet ID `0xF0` |
| 1 | 1 | Last valid LoRa RSSI, signed dBm |
| 2 | 4 | Ground-station uptime, milliseconds |
| 6 | 1 | GPS fix quality (GGA field 6; 0 = no fix) |
| 7 | 1 | GPS satellites used |
| 8 | 4 | Latitude, int32, 1e-7 degree |
| 12 | 4 | Longitude, int32, 1e-7 degree |
| 16 | 4 | GPS altitude, float32 m MSL |
| 20 | 4 | GPS time, uint32 Unix seconds (0 = unknown) |
| 24 | 2 | HDOP, uint16 x0.01 (`0xFFFF` = unknown) |
| 26 | 2 | Fix age, uint16 ms (`0xFFFF` = none or >= 65.535 s) |
| 28 | 1 | EOF `0x0A` |

Before the first valid LoRa packet, RSSI is reported as `-127 dBm`. Quality,
satellites and HDOP come from the latest GGA and read 0 / 0 / `0xFFFF` once it
is 3 s old. Position holds the last valid fix with its age; it is zero until a
fix has been seen. GPS time is the last status-`A` RMC's UTC advanced by the
time since it arrived. `build_ground_station_status()` fills the frame using
the `comms.h` offset macros; `GROUND_STATION_PROTOCOL.md` has the full
description.

## GPS (Quectel LC86)

The LC86 (L80-R footprint) is on UART1 at 115200 8N1, the LC86G factory rate:
GPIO38 is MCU RX (GPS TX) and GPIO39 MCU TX (GPS RX). Pins and settings are in
`main/hardware.h` (`IRIS_PIN_GPS_*`, `IRIS_GPS_*`).

`Lc86Gps` runs on its own FreeRTOS task (priority 2, 4 KiB stack), so it never
blocks the LoRa RX / USB loop in `main.cpp`. At boot it:

1. Pulses RESET_N (GPIO14) low for 100 ms and releases it. The pin is driven
   open-drain: LC86G RESET_N is active low with an internal 10 kOhm pull-up to
   1.8 V, and the hardware design forbids an external pull-up. If a board
   drives RESET_N through an inverting transistor instead, set
   `IRIS_GPS_RESET_ACTIVE_LOW` to 0.
2. Waits up to 3 s for the first checksum-valid sentence.
3. Configures 10 Hz GGA + RMC with Airoha PAIR commands, each resent up to
   three times until `$PAIR001,<id>,0` arrives (500 ms per attempt). Settings
   go to receiver RAM only and are reapplied on every boot:

   | Command | Purpose |
   |---|---|
   | `$PAIR062,1,0*3F` .. `$PAIR062,9,0` | GLL, GSA, GSV, VTG, ZDA, GRS, GST, GNS off (ZDA/GRS/GST/GNS may answer "not supported") |
   | `$PAIR062,0,1*3F` | GGA every fix |
   | `$PAIR062,4,1*3B` | RMC every fix |
   | `$PAIR050,100*22` | 100 ms fix interval (10 Hz; range 100..1000) |

   Source: Quectel LC26G(AB)&LC76G&LC86G Series GNSS Protocol Specification
   V1.1, sections 2.3.1 (`PAIR001`), 2.3.9 (`PAIR050`) and 2.3.12
   (`PAIR062`). Checksums are computed by `Lc86Nmea::build_sentence()`; the
   host test checks it against the specification's printed examples.
4. If the receiver never answers a PAIR command, it is probably a MediaTek
   LC86L, so it falls back to PMTK: `$PMTK314,0,1,0,1,0,0,...` (GGA + RMC,
   22 fields) and `$PMTK220,100` (L76-LB&L26-LB&LC86L GNSS Protocol
   Specification V1.1, sections 3.2.13 and 3.2.25; ack `$PMTK001,<id>,3`).
   Note that the LC86L factory baud rate is 9600, not 115200; the firmware does
   not search other baud rates.

If configuration fails the task logs a warning and keeps parsing whatever
arrives. `Lc86Nmea` accepts GGA and RMC from any talker (`$GP`, `$GN`, `$GL`,
`$GA`, `$GB`, ...), requires a valid checksum, and tolerates arbitrarily
fragmented reads. The latest state is published under a spinlock with esp_timer
millisecond timestamps; `main.cpp` reads it once per second for `0xF0`.

PPS (GPIO18, rising edge) is counted in an ISR with the last edge's esp_timer
time. It is diagnostic only, appears in the 30 s `gps` log line, and is not in
any packet.

Log lines (tag `gps`, on the USB console): configuration result, fix acquired
or lost, and every 30 s a summary of UART bytes, valid/bad sentences, quality,
satellites, HDOP, fix age and PPS count.

Host test (from this folder, WinLibs g++):

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/gps_status_test.cpp main/lc86_nmea.cpp main/ground_station_status.cpp -o build/gps_status_test.exe
& .\build\gps_status_test.exe
```

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

Without `export.ps1`, `idf.py build` also works from a plain PowerShell with
`IDF_PATH=C:\esp\v6.1\esp-idf`, `IDF_TOOLS_PATH=C:\Espressif\tools`,
`IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v6.1\venv`,
`ESP_IDF_VERSION=6.1`, and the venv Scripts, CMake, Ninja and
`xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin` directories on `PATH`.

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
forward canonical packets unchanged. Witness�Iris CAN fragmentation is internal
to those boards and never appears on LoRa or USB. See
[shared CAN contract](../../Intercomm/README.md).

Remote bench: `prs2026hitl`, programming RFC2217 port 7000, raw USB data TCP
port 7100. Run `flashit.ps1` from this folder to flash the Aperture image.
