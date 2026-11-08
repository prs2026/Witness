# Witness firmware agent handoff

## Purpose and current configuration

Witness is C++ firmware for the ESP32-S3 flight computer. ESP-IDF and FreeRTOS
coordinate inertial sensing, pressure sensing, battery measurement, flight-state
detection, NAND logging, USB commands and export, a UART GPS receiver, and an
optional LoRa radio. The user is back at the desk developing normal firmware;
the preceding reduced-board GPS-logger experiment is context, not a request to
continue stripping down the flight firmware.

The checked-in configuration still includes experimental settings. Review these
with the user before changing them:

- `HW_RADIO_ENABLED` is **1** in `main/hardware_pins.h`. Radio code is present,
  and initialization/service are enabled for the end-to-end setup.
- `app_main()` starts **UART1**, RX GPIO4 and TX GPIO5, for the PX1120S GPS.
  Its operational baud is **115200**, 8N1, without flow control.
- GPS I2C sources remain in `main/`, but are **not compiled or started**.
- Automatic flight transitions are enabled. Binary sensor logging is enabled
  only in Boost, Coast and Descent; idle samples go to a RAM prelaunch ring.
- The heartbeat buzzer is enabled at 2700 Hz. It can sound on the desk.
- The current `sdkconfig` selects a 2 MB boot-flash image configuration, despite
  the board header identifying an ESP32-S3FN8. Do not conflate boot flash with
  the separate 128 MiB logging NAND, or change image geometry casually.

This handoff describes the source at the time it was written. Follow the current
source when it differs from an older conversation, workbook, or prose document.

## Start here

| File | Responsibility |
|---|---|
| [main/main.cpp](main/main.cpp) | Static object graph and startup sequence |
| [main/hardware_pins.h](main/hardware_pins.h) | GPIO assignments, radio enable and RF settings |
| [main/comms.h](main/comms.h) | Canonical packet IDs, lengths, offsets, units, scales and status bits |
| [GROUND_STATION_PROTOCOL.md](GROUND_STATION_PROTOCOL.md) | Ground-station framing, field tables and command examples |
| [main/sensors.h](main/sensors.h), [main/sensors.cpp](main/sensors.cpp) | IMU driver, ADC, sensor task, radio scheduling and telemetry generation |
| [main/ms5607.h](main/ms5607.h), [main/ms5607.cpp](main/ms5607.cpp) | Pressure sensor driver |
| [main/flight_state_machine.h](main/flight_state_machine.h), [main/flight_state_machine.cpp](main/flight_state_machine.cpp) | Flight transitions and sensor-log state gate |
| [main/witness_status.h](main/witness_status.h) | Atomic ownership of the two status bytes |
| [main/heartbeat.cpp](main/heartbeat.cpp) | Heartbeat packets, red LED pulse and state-specific PWM beeps |
| [main/usb_serial_echo.cpp](main/usb_serial_echo.cpp) | USB echo, command parsing, local command execution and export switch |
| [main/command_bridge.cpp](main/command_bridge.cpp) | Forwarding USB/radio commands to Iris CAN and queueing radio commands for local execution |
| [main/spi_data_forwarder.cpp](main/spi_data_forwarder.cpp) | Queued USB output, latest-frame radio cache and immediate radio queue |
| [main/uart_driver.cpp](main/uart_driver.cpp) | UART receive task, raw logging, USB mirroring and GPS parser integration |
| [main/px1120s_gps.cpp](main/px1120s_gps.cpp), [main/nmea_gps.cpp](main/nmea_gps.cpp) | GPS startup configuration and independent streaming NMEA parser |
| [main/ra01.cpp](main/ra01.cpp) | SX1262-style radio commands, transmission and continuous receive |
| [main/w25n01kv.cpp](main/w25n01kv.cpp) | External SPI NAND driver |
| [main/flash_logger.cpp](main/flash_logger.cpp) | Sensor-record capture, prelaunch ring, UART/text storage and session exports |
| [main/usb_mass_storage.cpp](main/usb_mass_storage.cpp) | Read-only FAT16 projection over frozen NAND files |

The workbook is `AMA_comms_standard.xlsx`. For software changes, inspect
`comms.h` first; do not silently regenerate it from a potentially different
workbook revision. Spreadsheet edits require their own validation workflow.

## Startup and execution model

`app_main()` constructs all long-lived objects as function-local statics and
starts them in this order:

1. `SpiDataForwarder`: output queues and forwarding tasks.
2. `CommandBridge`: queue for radio commands awaiting local interpretation.
3. `UsbSerialEcho`: built-in USB Serial/JTAG driver and command task.
4. `FlashLogger`: diagnostic capture hook, capture task and NAND writer task.
5. UART1, after attaching the flash log sink.
6. `Sensors`, including the SPI3 devices and battery ADC.
7. `Heartbeat`.

Most startup calls use `ESP_ERROR_CHECK`, so a returned error can abort startup.
UART startup failure is explicitly logged without aborting. Flash initialization
happens inside its writer task: failure sets status bit 4 and stops that writer,
not the rest of the application. A failed radio initialization is nonfatal and
disables radio service until reboot. MS5607 initialization failure is nonfatal
and is retried from the sensor task. IMU or battery ADC startup failure currently
causes `Sensors::start()` to fail and therefore aborts through `ESP_ERROR_CHECK`.

All application tasks use `xTaskCreate`, not explicit core pinning. There is one
FreeRTOS scheduler frequency, **100 Hz**, configured by `CONFIG_FREERTOS_HZ` in
`sdkconfig.defaults` and the active `sdkconfig`. It is not independently set per
core or per task. A task's requested millisecond timeout is quantized to the
10 ms scheduler tick; UART's nominal 2 ms wait is clamped to at least one tick.

| Task | Priority | Stack bytes | Cadence or role |
|---|---:|---:|---|
| `sensors` | 4 | 4096 | Nominal 100 Hz; sole active SPI3 application service loop |
| `flash_capture` | 4 | 3072 | 50 Hz sensor snapshots and prelaunch ring |
| `flash_logger` | 3 | 7168 | Queue draining and NAND writes; default 1 s flush interval |
| `uart1` | 3 | 4096 | Blocking UART reads, parsing and raw byte dispatch |
| `usb_serial_echo` | 3 | 3072 | USB input and local/radio command handling |
| `heartbeat` | 2 | 2048 | 1 Hz heartbeat, LED and beep pattern |
| `data_forwarder` | 2 | 3072 | USB output queue consumer |
| `spi_interface` | 2 | 3072 | Consumes the future SPI-command queue; hardware send is a TODO |

The flight state machine and GPS startup state machine are objects serviced by
existing tasks, not separate FreeRTOS tasks. Sensor sample snapshots use a mutex;
GPS fix snapshots and radio caches use short critical sections; state/status and
several enables are atomic. Keep slow I/O outside critical sections.

## Hardware ownership

Use macros from `hardware_pins.h`, not new literal GPIO assignments in drivers.

| Interface | GPIO assignments |
|---|---|
| SPI3 shared sensors and radio | SCLK 15, COPI 17, CIPO 18 |
| LSM6DSV320X | CS 35, INT1 41, INT2 42 |
| MS5607 | CS 21 |
| Radio | CS 16, RESET 8, BUSY 39, RF enable 40, DIO1 7, DIO2 6, DIO3 2 |
| SPI2 external NAND | SCLK 12, IO0/MOSI 11, IO1/MISO 13, IO2 14, IO3 9, CS 10 |
| UART1 breakout | ESP32 RX 4, TX 5 |
| Inactive GPS I2C aliases | SDA 4, SCL 5; cannot run concurrently with UART1 |
| Built-in USB | D- 19, D+ 20 |
| Battery divider | VSENSE GPIO1, ADC1 channel 0 |
| LEDs | Red 33, green 34, blue 47 |
| Passive buzzer | GPIO48 |
| CAN-labelled breakout | TX 37, RX 36; active 500 kbit/s Classical CAN |

`Sensors` initializes SPI3 and serializes IMU, barometer and radio operations in
its own task. Do not add an independent radio task that competes with those
operations without explicitly redesigning bus ownership. Radio transmit waits
are synchronous and can stretch the nominal sensor cadence when radio is enabled.
`FlashLogger` owns its NAND instance on separate SPI2; export reads occur after
the writer has frozen.

## Sensing and telemetry production

Configuration constants live principally in `sensors.h` and the device headers:

- IMU SPI mode 3 at 10 MHz. Low-g accelerometer: +/-16 g, configured 120 Hz;
  high-g accelerometer: +/-320 g, configured 960 Hz; gyro: +/-2000 dps, 120 Hz.
  Polling at 100 Hz does not retrieve every high-g sample; there is no FIFO
  streaming implementation here.
- MS5607 SPI mode 3 at 10 MHz, pressure and temperature OSR 4096. Its polling
  state machine alternates conversions with a 9100 us conversion time. Altitude
  uses the configured sea-level reference of 1013.25 mbar, not a launch-site zero.
- Battery sampling: 10 Hz, derived as a divisor of the 100 Hz sensor loop.
  Sixteen ADC readings are averaged, with 12 dB attenuation and a 15k/5k divider
  (battery voltage is four times the ADC-pin voltage). Wire units are millivolts.
- Sensor telemetry: 10 Hz. State telemetry: 1 Hz. Witness debug: 10 Hz only after
  enabling it. Heartbeat: 1 Hz.

The `filtered_accel` wire field is currently encoded from low-g acceleration;
its name does not imply that a fused estimator or a high/low-g blending filter
has been implemented. State orientation, position and velocity are currently
zero. MCU temperature and GPS Unix time also remain zero in debug output.
GPS coordinates and altitude are supplied separately by the UART GPS snapshot,
not by `Sensors::Sample`.

## Canonical communication format

All transports share `packet ID | fixed payload | 0x0A`. Integers are big-endian;
signed values are two's complement; float fields are big-endian IEEE-754 float32.
There is no application CRC or byte escaping. Payload bytes can equal `0x0A`:
use the ID-specific length and check EOF only at the calculated final offset.
`comms.h` offsets are generally payload-relative; the ground-station document's
tables use absolute frame offsets. Account for the initial ID byte.

| ID | Packet | Payload bytes | Complete frame bytes |
|---|---|---:|---:|
| `01` | Sensors | 21 | 23 |
| `02` | State | 62 | 64 |
| `03` | Camera telemetry | 17 | 19 |
| `05` | Command | 8 | 10 |
| `E0` | Witness debug | 101 | 103 |
| `E1` | IRIS debug | 12 | 14 |
| `FF` | Heartbeat | 6 | 8 |

Camera and IRIS debug layouts exist but are not locally generated. Witness debug
is **0xE0**, not the obsolete 0x06 ID mentioned in some earlier conversation.
The two-byte canonical command value starts at frame offset 7; a full command is
`05 status[2] uptime_ms[4] command_be[2] 0A`.

Important scales: filtered acceleration is signed int24 at 40 micro-g/count;
GPS coordinates are signed int24 at 0.0001 degree/count; GPS altitude is float32
metres above mean sea level. See the field catalog in `comms.h` for every field.

USB is deliberately a **mixed byte stream**: canonical binary telemetry, echoed
host input, diagnostic text, raw GPS/NMEA and received radio payloads. A text
terminal displays binary telemetry as garbage, and a UART chunk boundary can
allow a telemetry packet to appear between parts of a displayed NMEA sentence.
GPS parsing and raw logging occur before USB mirroring, so USB interleaving does
not itself corrupt the internal parser or the raw UART file. Do not treat the
entire USB stream as clean NMEA or as uninterrupted canonical frames.

`SpiDataForwarder::queue_packet()` also caches the most recent canonical frame
for radio scheduling. Its USB queue has depth 16 and nonblocking enqueue;
congestion can drop output. The radio immediate queue has depth 8.
The output worker retries USB writes, and direct USB echo uses an indefinite
write timeout. A host that stops reading can stall those output paths. Keep
UART capture independent of host consumption; do not assume every apparent
terminal stall means sensor acquisition or GPS reception stopped.

## Status and flight state

`WitnessStatus::write()` emits two bytes. Byte 0 is the flag byte:

| Bit | Meaning when set |
|---:|---|
| 0 | Heartbeat phase; toggles every second |
| 1 | Low-g acceleration ready |
| 2 | Gyro ready |
| 3 | IMU temperature ready |
| 4 | NAND initialization or startup self-test failed |
| 5 | High-g acceleration ready |
| 6 | Barometer ready |
| 7 | Flash logging fault; also set on raw UART log buffer overflow |

Byte 1 bits 0..2 contain the state enum: Pad Idle 0, Boost 1, Coast 2, Descent 3,
Landed 4. Bits 3..7 are reserved zero. There is no GPS-fix or radio-ready status
bit. Readiness flags should not be mistaken for a universal freshness guarantee.

The automatic transitions in `flight_state_machine.cpp` are:

| Transition | Consecutive qualifying condition |
|---|---|
| Pad Idle to Boost | Positive X acceleration >10 g for 300 ms; high-g X preferred, low-g fallback |
| Boost to Coast | Low-g X acceleration <1 g for 500 ms |
| Coast to Descent | Barometric altitude below peak minus 5% of the absolute peak for 500 ms |
| Boost, Coast or Descent to Landed | Acceleration vector magnitude 0.8..1.2 g, gyro vector magnitude <10 dps, and barometric altitude span <=10 m for 30 s |

Landing is not restricted to Descent. Its altitude rule is a flat 10 m span, not
the earlier percentage rule or an absolute altitude below 10 m. The descent rule
uses pressure altitude rather than height above launch. Peak tracking resets on
entry to Boost; the sign-aware drop expression also handles negative pressure
altitudes. Condition timing uses microsecond timestamps rather than loop counts.

A commanded Pad Idle to Boost invokes `trigger_launch()`, just like automatic
launch, and opens the logging gate. Automatic transitions remain enabled after
a manual state command: a board on the desk may subsequently coast or land.

## Commands and forwarding

USB accepts normalized line-oriented text, raw compact binary commands, and full
10-byte canonical command packets. Binary compact parsing uses a 20 ms idle
boundary; send the bytes together. The following text forms are convenient on
the bench; append CR/LF. Hex shown here is text unless sent as actual binary.

| Input | Effect |
|---|---|
| `LED 1`, `LED ON`, `LED 0`, `LED OFF` | Sets the red LED baseline; heartbeat still pulses it |
| `CAM 1 ON/OFF`, `CAM 2 ON/OFF` | Updates in-memory camera flags and logs; no camera hardware driver |
| `E0` or raw byte `E0` | Enables Witness debug output; no corresponding stop command |
| `05 73 00` through `05 73 04` | Sets flight state; `01` from Pad Idle is a launch trigger |
| `05 68 68` | Freezes logs and switches USB to read-only mass storage until reset |
| `05 68 69` | Destructively erases all stored sessions and creates a fresh session |
| `TX 05 ...` | Bridges decoded command bytes to CAN/radio; do not assume it performs the same local actions as the non-TX form |
| `TX ID payload...` with another ID | Queues the future SPI command path; no hardware SPI transmission yet |

The protected erase is a magic opcode, **not authentication or a confirmation
dialog**. Radio can invoke destructive commands too. Do not transmit erase or
export commands merely to inspect or test the system.

`CommandBridge` forwards every accepted `05` command unchanged to the current
UART1 instance. This is currently the GPS port, not a separate external device
link. USB commands can additionally enter the immediate radio queue. Radio
commands are forwarded to UART first, then queued for execution by the USB
command task. Canonical packets with invalid EOF may be forwarded unchanged
while local execution rejects them. Forwarding and validation are not equivalent.

Radio local handlers support debug enable, export, erase and state setting.
Load opcodes `30..35` and regulator opcodes `50..51` are external-device commands,
not implemented local output controls. For radio debug enable, use command value
`00 E0` in the compact or canonical representation; do not infer arbitrary
one-byte USB shortcuts work over radio.

## Radio behavior

Despite the `Ra01` class name, this driver uses **SX1262 command semantics**, not
an SX127x register interface. Verify the actual fitted module before substituting
a driver. Current RF macros specify 926 MHz, -8 dBm, SF7, 125 kHz bandwidth,
coding rate 4/5 and preamble length 8. Packet configuration uses explicit headers,
hardware CRC and normal IQ. Read `ra01.cpp` for the complete setup.

With `HW_RADIO_ENABLED=1` and successful initialization, the sensor task polls
DIO1 for receive data, logs each received packet and a hex dump, mirrors it to
USB, and submits `05` packets to `CommandBridge`. The radio returns to continuous
receive after transmit. Init failure releases the device and does not schedule
retries or stop sensing/logging.

Per-type downlink rates are independently configured in `sensors.h`, currently
1 Hz for every supported type. Scheduling sends the latest cached local frame,
not every intermediate sample. Immediate external forwarding is enabled by
`SpiDataForwarder::kForwardExternalPacketsImmediately`; immediate packets take
precedence over scheduled traffic. Only one transmit is serviced per loop.
Raw GPS NMEA is not automatically downlinked as canonical GPS telemetry; state
and debug packets carry the parsed coordinates instead.

## GPS operation and unresolved hardware check

`UartDriver` logs received bytes before parsing and USB forwarding. A streaming
`NmeaGps` object accepts checksum-valid GGA sentences across arbitrary UART read
boundaries, including GP/GN talkers, converts degrees/minutes and hemisphere to
decimal degrees, and reads altitude in metres MSL. It does not derive Unix time
from RMC/ZDA. No-fix GGA immediately invalidates a fix; fixes expire after 3000 ms.
Invalid or stale state/debug GPS fields are zero, which is not a unique invalid
sentinel because zero coordinates and altitude can also be legitimate.

`Px1120sGps` performs boot delay, navigation-mode query, airborne configuration
and mode verification. It uses fixed 115200 baud and SRAM-only settings by
default; it no longer hunts baud rates or commands the GPS to change baud.
Checksum-valid no-fix NMEA is enough to prove the link and proceed past an
unanswered initial mode query, but only mode readback confirms configuration.

| Operation | Complete binary frame in hex |
|---|---|
| Query navigation mode | `A0 A1 00 02 64 18 7C 0D 0A` |
| Set airborne in SRAM | `A0 A1 00 04 64 17 05 00 76 0D 0A` |
| Expected airborne readback | `A0 A1 00 03 64 8B 05 EA 0D 0A` |

Last bench report: UART reception was stable at 115200, but airborne verification
timed out. That does not prove the GPS rejected the command or entered airborne
mode. The ESP32 TX GPIO5 to GPS RX path and possible evaluation-board USB-UART
contention were suggested checks, not established causes. The sample NMEA also
reported fix quality 0 and zero satellites. Do not mark airborne setup or a GPS
position fix as hardware-verified on the strength of the successful parser tests.
Five-second `GPS RX` diagnostics report baud, byte count, GGA count, fix freshness
and idle time. Configuration failure leaves collection enabled.

## Flash storage and export

The `W25n01kv` driver expects JEDEC **EF AE 21**, using SPI2 mode 0 at 10 MHz,
2048-byte data pages, 64 pages/block and 1024 blocks. Spare size is currently 64
bytes. Check actual ID and ECC handling when changing chip variants; older
comments and file/class names are not sufficient identification.

The logger uses a **custom append-only NAND store**, not a mounted writable FAT
filesystem. On successful initialization it tests a reserved good block by
erase/write/read comparison, scans sessions, rotates to a new slot and creates
a new session. There are 16 rotating slots of 62 blocks each, with 20 reserved
blocks outside the slots. Store version 2 allocates 10 blocks for sensor records,
40 for UART bytes and 12 for diagnostic text. Version 1 exports are also handled.
Rotation reuses old storage; this is not unlimited retention.

| Export filename example | Contents |
|---|---|
| `S0000001.BIN` | Fixed 48-byte sensor records; not canonical telemetry packets |
| `S0000001.TXT` | Captured ESP logging output after the application log hook is installed |
| `U0000001.BIN` | Exact raw UART bytes, including NMEA and binary responses; no added timestamps or framing |

Sensor capture is independent of the USB packet rates. It samples at 50 Hz,
holds the most recent 100 records in a two-second Pad Idle ring, and on entry
to an enabled logging state queues the oldest prelaunch record first, followed
by live records. A separate writer drains that queue while capture continues.
The 256-record queue accommodates the snapshot and a finite live backlog;
`portMAX_DELAY` sends can block capture if the writer cannot keep up indefinitely.
Do not describe the buffering as an unconditional no-loss guarantee.

Raw UART logging is continuous across flight states, with a 32 KiB input stream
buffer. Sensor and text staging buffers are 4096 bytes; UART staging is two
2040-byte NAND payloads. Flush defaults to 1 s and may be configured to 500 ms.
Data can flush earlier when staging fills. Power loss can lose unflushed data;
this store is not a transactional general-purpose filesystem. The text queue is
bounded and long log messages are truncated to fit its entries.
Exhausting one file allocation marks that stream full and sets the flash fault
flag without intentionally stopping the other allocations; a fatal writer I/O
error stops all capture. Check which condition occurred before assuming that
every stream has stopped or that a healthy initialization guarantees logging.

The exported sensor record layout is independent of `comms.h`. Multi-byte fields
are big-endian; the source of truth is `FlashLogger::serialize_record()`:

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 2 | Magic `57 4C` |
| 2 | 1 | Record version 1 |
| 3 | 1 | Witness flags only; no flight-state byte |
| 4 | 4 | uint32 capture sequence |
| 8 | 4 | uint32 uptime ms |
| 12 | 6 | Three signed int16 low-g raw samples |
| 18 | 6 | Three signed int16 high-g raw samples |
| 24 | 6 | Three signed int16 gyro raw samples |
| 30 | 2 | Signed int16 IMU raw temperature |
| 32 | 4 | Signed int32 pressure in centi-mbar |
| 36 | 4 | Signed int32 barometer temperature in centi-Celsius |
| 40 | 4 | Float32 barometric altitude in metres |
| 44 | 4 | uint32 battery millivolts |

There are **no parsed GPS coordinates or state enum in these 48-byte records**.
GPS survives in `Uxxxxxxx.BIN`; parsed GPS is exposed in live state/debug packets.
Changing the binary record requires a deliberate version/decoder migration.

Export command `05 68 68` disables radio TX, stops capture, drains accepted
records, flushes files and freezes the writer. It then disables normal USB output,
uninstalls Serial/JTAG and starts TinyUSB MSC. Reset is required to resume normal
logging. The disk is a synthesized read-only FAT16 view labelled `WITNESSLOGS`,
128 MiB, with 512-byte sectors and 4 KiB clusters. Host writes are rejected;
formatting it on the desktop is not how sessions are erased.

Export reads use a retained NAND-page cache to avoid rescanning a file from its
beginning for every host read. Preserve that cache when addressing large-copy
problems; never load an entire file into RAM as the export mechanism. Export and
erase waits can be indefinite by default, so keep failure handling in view when
changing synchronization.

## Heartbeat indicators

The heartbeat task toggles status bit 0, produces `FF`, pulses the red LED for
100 ms relative to its commanded baseline, then plays the current state's beep
pattern. PWM uses LEDC low-speed timer 0/channel 0, 10-bit resolution, duty 512.

- Pad Idle: 75 ms chirp every fifth cycle.
- Boost: 700 ms tone each cycle.
- Coast: two 100 ms tones with a 100 ms gap.
- Descent: three 100 ms tones with 100 ms gaps.
- Landed: two 250 ms tones with a 150 ms gap every second cycle.

`HEARTBEAT_BUZZER_ENABLED` and `HEARTBEAT_BUZZER_FREQUENCY_HZ` are at the top of
`heartbeat.cpp`. Disabled and resting buzzer output must remain low. Do not
repurpose LEDC timer/channel 0 without coordinating this module.

## Build and verification

The dependency lock targets ESP-IDF **6.1.0**, ESP32-S3,
`espressif/esp_tinyusb` 2.3.0 and `espressif/tinyusb` 0.21.0~2.
`main/CMakeLists.txt` is the authoritative active source list and enables the
TinyUSB MSC class with a 512-byte transfer buffer. The inactive I2C sources
`gps_i2c_driver.*` and `skytraq_i2c_mailbox.*` are intentionally outside that list.

From an activated ESP-IDF environment in this directory:

```text
idf.py build
idf.py -p <actual-port> flash monitor
```

Inspect the local environment before selecting ports or running scripts.
`flashit.sh`, `openserial.sh` and `sendserial.sh` contain Linux-specific paths;
`flashit copy.ps1` targets an RFC2217 remote endpoint. The VS Code settings list
COM4, but an old IDE setting is not proof that a board is connected there.
Do not flash a remote bench or erase NAND without an explicit request.

Host tests are standalone C++17 programs, not an integrated CTest suite. With
`g++` available, examples for PowerShell are:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/nmea_gps_test.cpp main/nmea_gps.cpp -o build/nmea_gps_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\nmea_gps_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/px1120s_gps_test.cpp main/px1120s_gps.cpp -o build/px1120s_gps_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\px1120s_gps_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/skytraq_i2c_mailbox_test.cpp main/skytraq_i2c_mailbox.cpp -o build/skytraq_i2c_mailbox_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\skytraq_i2c_mailbox_test.exe
```

The tests exercise parsing, fixed-baud configuration protocol handling and the
inactive mailbox implementation using fake transports. They do not validate
RF, SPI timing, storage power-loss behavior, flight detection or physical wiring.

In the previous sandbox the full firmware built using Ninja directly after an
ESP-IDF configure. ESP-IDF Git ownership warnings and offline registry checks
were environmental, not reasons to change global Git configuration. If ccache
cannot write its default location, point its cache/temp directories inside the
workspace. Do not copy stale absolute tool paths into project source files.

## Rules for subsequent changes

1. Confirm the intended full-board versus reduced-board configuration before
   re-enabling radio or removing GPS. This handoff does not authorize either.
2. Keep application packet schema, raw sensor record format and NAND store
   version distinct. Their lengths and versioning are separate contracts.
3. Preserve SPI3 serialization, the capture/writer split, chronological
   prelaunch-to-live queueing and the bounded-memory export reader.
4. Changing nominal packet rates does not change flash sampling rates. Changing
   the sensor task rate also affects battery divisors and scheduler granularity.
5. Keep command bridging separate from local validation and actuation. Commands
   currently share UART1 with the GPS; check that architecture before extending
   external-device control.
6. Treat missing radio as nonfatal, but do not assume missing IMU is nonfatal.
7. Update `GROUND_STATION_PROTOCOL.md` with `comms.h` changes, and deliberately
   synchronize the workbook when requested. Do not revive obsolete LoRa/CAN
   compression distinctions or packet IDs from conversational history.
8. Inspect the worktree and preserve unrelated changes. Generated build outputs
   and temporary PDF tooling have appeared in repository status; do not clean,
   reset or delete them as a side effect of firmware work.


