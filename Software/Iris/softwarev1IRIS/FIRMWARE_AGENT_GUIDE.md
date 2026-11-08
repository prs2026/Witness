# Iris Firmware Agent Guide

This document is a handoff guide for an engineer or coding agent working on
the Iris firmware. It describes the behavior implemented by the current
source tree. When this document and the code disagree, the source files named
below are authoritative.

## 1. Project summary

- Target: ESP32-C3FH4 (`esp32c3`, 4 MB embedded flash).
- Framework: ESP-IDF 6.1-era APIs.
- Execution model: one direct polling loop in `app_main`; the application does
  not create separate FreeRTOS tasks.
- Primary transport: built-in USB Serial/JTAG.
- Secondary transport: UART1 at 115200 baud.
- Active peripherals: GPIO, I2C0, ADC1, UART1 and TWAI/Classical CAN.
- Wi-Fi, Bluetooth and other Espressif RF stacks are intentionally excluded.

The main application controls four switched outputs, drives red and blue
heartbeat LEDs, samples three current-monitor channels, measures battery
voltage, accepts commands and emits binary telemetry.

## 2. Source map

| File | Responsibility |
|---|---|
| `main/main.cpp` | Hardware startup and the application's direct loop |
| `main/hardware.h` | Board pins, addresses, calibration and timing constants |
| `main/comms.h` | Authoritative command values and binary packet schema |
| `main/usb_serial_echo.*` | USB/UART command parsing, heartbeat and telemetry scheduling |
| `main/mcp23008.*` | MCP23008 GPIO-expander driver and shared I2C bus creation |
| `main/pac1931.*` | PAC193x current-monitor sampling and conversion |
| `main/battery_voltage.*` | Calibrated ADC battery-voltage measurement |
| `main/twai_driver.*` | Active interrupt-driven TWAI driver with bounded queues and bus-off recovery |
| `main/can_link.*` | Nonblocking packet queue, CAN fragmentation and reassembly |
| `../../../Intercomm/can_transport.h` | Shared Witness/Iris CAN transport codec |
| `main/CMakeLists.txt` | Active application sources and ESP-IDF dependencies |
| `sdkconfig.defaults` | Scheduler, flash, RF and USB-console defaults |

## 3. Startup and main loop

`app_main` performs these operations in order:

1. Initializes the MCP23008 and therefore creates I2C0.
2. Configures OUT1, OUT2, OUT3, 5V_EN and the blue LED as low outputs.
3. Adds the PAC193x device to the MCP23008's I2C bus.
4. Initializes the calibrated ADC battery monitor.
5. Initializes the red LED, UART1, USB Serial/JTAG and CAN (CAN failure is nonfatal).
6. Enters a permanent loop which reads battery voltage at its configured
   interval, calls `UsbSerialEcho::poll()`, then calls `vTaskDelay(1)`.

PAC193x initialization failure is nonfatal: it is logged and the application
continues. MCP23008, ADC, UART or USB initialization failure is fatal through
`ESP_ERROR_CHECK`.

The scheduler tick is 100 Hz, so `vTaskDelay(1)` nominally yields for 10 ms.
All application schedules use `esp_timer_get_time()` and catch up their next
deadline rather than relying on loop iteration counts.

## 4. Hardware map

The authoritative definitions are in `main/hardware.h`.

| Function | ESP32-C3 pin |
|---|---:|
| Battery VSENSE / ADC1 channel 0 | GPIO0 |
| CAN RX | GPIO1 |
| Button | GPIO2 |
| CAN TX | GPIO3 |
| I2C SDA | GPIO4 |
| I2C SCL | GPIO5 |
| UART1 TX | GPIO6 |
| UART1 RX | GPIO7 |
| Red LED | GPIO8 |
| MCP23008 reset / LS_EN | GPIO9 |
| Green LED | GPIO10 |
| USB D- | GPIO18 |
| USB D+ | GPIO19 |
| UART0 RX | GPIO20 |
| UART0 TX | GPIO21 |

The CAN definitions intentionally follow signal direction. The PCB net names
at the TCAN3414 are reversed: the transceiver TXD input is on the net labeled
CANRX (GPIO3), while RXD is on CAN_TX (GPIO1).

### MCP23008 outputs

- I2C address: `0x20` (`A2:A0` tied low).
- GP0: OUT1_EN.
- GP1: OUT2_EN.
- GP2: OUT3_EN.
- GP3: 5V_EN.
- GP7: blue LED.
- Bus rate: 400 kHz.
- Internal ESP32 I2C pull-ups are disabled; the board must provide pull-ups.

The driver writes OLAT before changing IODIR to output, preventing a startup
glitch. It maintains cached direction and output-latch bytes, so MCP23008
writes must go through this driver unless the cache is also synchronized.

### Battery input

VSENSE uses a 15 kohm high-side resistor and 5 kohm low-side resistor, giving
a 4:1 battery-to-ADC ratio. The ADC driver averages 16 samples, applies the
ESP-IDF curve-fitting calibration and then scales the calibrated pin voltage
by four. The normal read interval is 1000 ms.

### Current monitor

- Device address: `0x10`.
- Three channels are read from VSENSE registers `0x0B` through `0x0D`.
- Each nominal shunt is 20 milliohms.
- The driver assumes the default unipolar 0-100 mV VSENSE range.
- A `REFRESH_V` command is followed by a 1.1 ms stabilization delay.
- Results are calculated internally in microamps.
- `IRIS_PAC1931_CURRENT_CALIBRATION_DIVISOR` is currently `5`, an empirical
  correction added because uncorrected readings were approximately five times
  the applied current.

The firmware transmits monitor channels in register order without remapping
them to load-switch names. Confirm schematic routing before interpreting a
channel as OUT1, OUT2 or OUT3.

## 5. Timing and produced data

| Activity | Rate | Behavior |
|---|---:|---|
| Main-loop yield | 100 Hz nominal | `vTaskDelay(1)` |
| Current sample | 10 Hz | Reads all three channels |
| Camera packet `0x03` | 1 Hz | Reports peak current from the preceding interval |
| Battery sample | 1 Hz | Last valid value retained; validity tracked separately |
| Heartbeat LED pulse | 1 Hz | Red and blue outputs invert for 100 ms |
| IRIS debug packet `0xE1` | 10 Hz | Disabled until command `0x00E1` |

Heartbeat pulses temporarily invert each LED from its commanded steady state,
then restore that state. The blue LED is driven through the MCP23008.

Every generated telemetry packet is written to USB Serial/JTAG and UART1,
and queued for CAN by `write_packet()`. Ping responses return to their source.

## 6. Binary protocol

`main/comms.h` is the schema source of truth. Frames have this form:

```text
packet ID (1 byte) | fixed-length binary payload | EOF 0x0A (1 byte)
```

- Multi-byte values are big-endian.
- There is no application CRC.
- `0x0A` is not escaped and can occur inside a payload. Use the packet ID's
  fixed length; do not scan for the first newline byte.
- Uptime fields are milliseconds since boot, truncated to `uint32`.

The header defines packet IDs `0x01`, `0x02`, `0x03`, `0x05`, `0xE0`, `0xE1`
and `0xFF`, but this firmware currently produces only `0x03` and, when
enabled, `0xE1`. It sends `0x05` only as a command response.

### Camera telemetry: packet `0x03`, 19 bytes

| Frame offset | Length | Meaning |
|---:|---:|---|
| 0 | 1 | Packet ID `0x03` |
| 1 | 2 | FC/Witness status from latest CAN heartbeat; zero when stale |
| 3 | 4 | FC/Witness uptime from latest CAN heartbeat; zero when stale |
| 7 | 2 | IRIS status; flags in first byte, second byte zero |
| 9 | 4 | IRIS uptime in ms |
| 13 | 1 | Channel 1 peak current |
| 14 | 1 | Channel 2 peak current |
| 15 | 1 | Channel 3 peak current |
| 16 | 2 | Battery voltage, unsigned mV |
| 18 | 1 | EOF `0x0A` |

Currents are the maximum valid samples observed during the preceding one-second
report window. A failed sample sets the current-monitor error for that report,
even if other samples succeeded.

### IRIS debug: packet `0xE1`, 14 bytes

| Frame offset | Length | Meaning |
|---:|---:|---|
| 0 | 1 | Packet ID `0xE1` |
| 1 | 2 | IRIS status; flags in first byte, second byte zero |
| 3 | 4 | IRIS uptime in ms |
| 7 | 1 | MCU temperature; currently zero/unimplemented |
| 8 | 2 | Battery voltage, unsigned mV |
| 10 | 1 | Latest channel 1 current |
| 11 | 1 | Latest channel 2 current |
| 12 | 1 | Latest channel 3 current |
| 13 | 1 | EOF `0x0A` |

### Current wire encoding

Each current field remains an unsigned byte but now uses 20 mA per count:

```text
current_mA = encoded_byte * 20
encoded_byte = round(current_uA / 20000), saturated to 255
```

The representable range is 0.00-5.10 A. Examples: `0x01` is 20 mA, `0x32`
is 1.00 A, `0x96` is 3.00 A and `0xFF` is 5.10 A. Any receiver still using
the former 100 mA/count definition will decode current five times too high.

### IRIS status field

The status field is two bytes. Byte 0 contains active-high flags; byte 1 is
reserved and currently transmitted as zero.

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x01` | Current monitor failed or no valid current sample is available |
| 1 | `0x02` | Battery reading is invalid |
| 2 | `0x04` | Command-controlled flag, set by `0x0020` and cleared by `0x0021` |
| 3 | `0x08` | OUT1 enabled |
| 4 | `0x10` | OUT2 enabled |
| 5 | `0x20` | OUT3 enabled |
| 6 | `0x40` | 5 V regulator enabled |
| 7 | `0x80` | Reserved, currently zero |

The error flags are added while constructing telemetry; they are not stored in
`iris_status_flags_`. The four output flags are updated only after a successful
MCP23008 write.

## 7. Commands

### Canonical binary command frame

Binary commands use packet `0x05`, always 10 bytes:

| Frame offset | Length | Meaning |
|---:|---:|---|
| 0 | 1 | Packet ID `0x05` |
| 1 | 2 | Sender status |
| 3 | 4 | Sender uptime in ms |
| 7 | 2 | Big-endian command value |
| 9 | 1 | EOF `0x0A` |

Example: enable OUT1 with zero status and uptime:

```text
05 00 00 00 00 00 00 00 31 0A
```

The same fixed frame is accepted on USB, UART1 and reassembled CAN. UART1 ignores bytes until
it sees `0x05`, then collects exactly ten bytes. USB input is echoed back
before parsing. A malformed frame is silently ignored after collection.

| Command | Meaning |
|---:|---|
| `0x0020` | Set command-controlled status flag |
| `0x0021` | Clear command-controlled status flag |
| `0x0022` | Ping; reply to the same source with command `0x0023` |
| `0x0023` | Pong response value |
| `0x0030` / `0x0031` | OUT1 off / on |
| `0x0032` / `0x0033` | OUT2 off / on |
| `0x0034` / `0x0035` | OUT3 off / on |
| `0x0050` / `0x0051` | 5 V regulator off / on |
| `0x00E1` | Start 10 Hz IRIS debug packets; no stop command exists |

Unknown binary commands have no effect and produce no error response.

### ASCII commands

ASCII commands are newline-terminated and normalized to uppercase with runs of
spaces/tabs collapsed. The exact form is:

```text
S <output> <state>
```

`<state>` is `0` or `1`. Outputs are `1`, `2`, `3`, `5`, `R` and `B` for the
three load switches, 5 V enable, red LED and blue LED respectively. Examples:

```text
S 1 1
S 5 0
S R 1
S B 0
```

Because application log levels are disabled after initialization to protect
the binary USB stream, ASCII command success/error log messages are normally
not visible after startup. Input echo remains visible.

## 8. UART1 and CAN transport

UART1 is active at 115200, 8 data bits, no parity, one stop bit and no flow
control. Generated `0x03` and `0xE1` packets are duplicated to UART1. UART1 RX
accepts only canonical 10-byte `0x05` commands; arbitrary UART data is not
bridged to USB.

CAN is active at 500 kbit/s using the ESP-IDF `esp_driver_twai` component.
The TCAN3414 is a 3.3 V transceiver; STB pin 8 must remain low and SHDN pin 5
low (its internal pulldown permits an unconnected pin). TX is GPIO3 and RX
GPIO1. CANH/CANL and a common ground join Witness; terminate only the two bus ends.
The fitted 120-ohm resistor is R39 on the existing board schematic.

Standard identifier `0x120` carries Witness-to-Iris packets; `0x121` carries
Iris-to-Witness packets. Each Classical CAN frame contains sequence, zero-based
fragment index, total canonical packet length, then up to five packet bytes.
The final frame uses its exact DLC with no padding. Reassembly rejects wrong
identifiers, extended/remote frames, malformed DLC, inconsistent totals,
out-of-order fragments and transfers older than 250 ms. The canonical packet
ID, length and EOF are validated before commands execute. The codec lives in
`Intercomm/can_transport.h` at the Software root, shared with Witness.

The existing direct application loop polls CAN; it creates no application task.
Its eight-packet queue retains complete packets under hardware backpressure;
an exhausted queue drops the new packet. The driver retries a failed frame
at most three times, counts failed TX/RX drops and initiates bus-off recovery.
This is bounded best-effort transport, without an application delivery ACK.
Camera and debug telemetry travel to Witness, while commands arrive from it.
CAN ping produces a CAN-only pong. Witness heartbeat status/uptime fill camera
FC fields while the most recent heartbeat is at most three seconds old;
otherwise both FC fields return to zero. Incoming CAN traffic is not echoed
back onto CAN, preventing routing loops.

## 9. Build configuration

The root `CMakeLists.txt` forces the target to `esp32c3` and builds only the
`main` component and its declared dependencies. `CONFIG_APP_NO_BLOBS=y` plus
the minimal component list prevent Wi-Fi/Bluetooth/RF binary libraries from
being linked.

From an ESP-IDF 6.1 environment:

```sh
idf.py set-target esp32c3
idf.py build
```

The expected application image is:

```text
build/softwarev1.bin
```

The checked-in Linux flashing script uses:

```sh
./flashit.sh
```

Both `flashit.sh` and `flashit.ps1` default to `rfc2217://prs2026hitl:7001`
and flash the configured 4 MB ESP32-C3 image. The shell script accepts a port
as its first argument; PowerShell accepts `-Port` and `-Python`.
USB data is read independently at raw TCP `prs2026hitl:7101`.
`openserial.sh` starts minicom on
`/dev/ttyUSB0` at 115200 and may also need adjustment.

`flashit copy.ps1` delegates to `flashit.ps1`. The normal image offsets are bootloader `0x0`,
partition table `0x8000` and application `0x10000`.

## 10. Maintenance rules and known traps

1. Keep board-specific pins, addresses, divider values and calibrations in
   `hardware.h`; application code should not introduce raw pin numbers.
2. Treat `comms.h` as the protocol authority. If a field length changes,
   update offsets, total lengths, compile-time assertions and every receiver.
3. Current fields are 20 mA/count, not 100 mA/count. Coordinate this change
   with all ground-station and peer firmware decoders.
4. The EOF byte is not escaped. Parsers must use fixed packet lengths.
5. USB Serial/JTAG carries both echoed input and binary telemetry. Do not emit
   ordinary logs into a packet stream after initialization.
6. The MCP23008 driver caches OLAT and IODIR. Out-of-band I2C register writes
   can make later pin operations overwrite unexpected bits.
7. The current monitor uses a board-specific divide-by-five correction. Do not
   remove it merely because the schematic shunts say 20 milliohms; recalibrate
   against measured hardware first.
8. `0x03` current is a one-second peak, while `0xE1` current is the most recent
   10 Hz sample. They are intentionally different measurements.
9. The first battery read occurs roughly one second after startup. Until then,
   telemetry reports zero with the battery error flag set.
10. The active application has no custom RTOS tasks. Preserve the nonblocking
    polling model unless a deliberate architecture change is requested.
11. Keep the shared CAN codec and both peers compatible; its three-byte
    fragment header is distinct from the canonical application frame.

