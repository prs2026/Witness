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
- `app_main()` starts **UART1** for the GPS (PX1120S or u-blox M8, auto-detected).
  `HW_MCU_UART_SWAP_RX_TX` in `main/hardware_pins.h` is currently **1**: ESP32 RX
  is GPIO5 and TX is GPIO4, which is what the fitted u-blox harness needed on
  2026-11-08. The normal wiring (0) is RX GPIO4, TX GPIO5; the earlier PX1120S
  streamed on that. Zero UART bytes from boot onward points at this setting.
  Its operational baud is **115200**, 8N1, without flow control.
- GPS I2C sources remain in `main/`, but are **not compiled or started**.
- Automatic flight transitions are enabled. Binary sensor logging is enabled
  only in Boost, Coast and Descent; idle samples go to a RAM prelaunch ring.
- The heartbeat buzzer is currently **disabled** for bench work
  (`HEARTBEAT_BUZZER_ENABLED 0` in `heartbeat.cpp`). Set it to 1 before flight.
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
| [main/attitude_estimator.h](main/attitude_estimator.h), [main/attitude_estimator.cpp](main/attitude_estimator.cpp) | Host-testable strapdown attitude estimator feeding the `0x02`/`0xE0` quaternion |
| [main/pressure_altitude.h](main/pressure_altitude.h) | Pressure-to-altitude formula shared by the MS5607 driver and HITL |
| [main/hitl_input.h](main/hitl_input.h), [main/hitl_input.cpp](main/hitl_input.cpp) | Host-testable HITL `0x0C` decoder, frame assembler and enter/exit/timeout state |
| [main/flight_state_machine.h](main/flight_state_machine.h), [main/flight_state_machine.cpp](main/flight_state_machine.cpp) | Flight transitions and sensor-log state gate |
| [main/witness_status.h](main/witness_status.h) | Atomic ownership of the two status bytes |
| [main/heartbeat.cpp](main/heartbeat.cpp) | Heartbeat packets, red LED pulse and state-specific PWM beeps |
| [main/usb_serial_echo.cpp](main/usb_serial_echo.cpp) | USB echo, command parsing, `0x0C` HITL framing, local command execution and export switch |
| [main/command_bridge.cpp](main/command_bridge.cpp) | Forwarding USB/radio commands to Iris CAN and queueing radio commands for local execution |
| [main/can_link.h](main/can_link.h), [main/can_link.cpp](main/can_link.cpp), [main/twai_driver.cpp](main/twai_driver.cpp) | Iris CAN link: command/heartbeat transmit, fragmentation and reassembly via [Intercomm](../../Intercomm/README.md) |
| [main/spi_data_forwarder.cpp](main/spi_data_forwarder.cpp) | Queued USB output, latest-frame radio cache and immediate radio queue |
| [main/uart_driver.cpp](main/uart_driver.cpp) | UART receive task, raw logging, USB mirroring and GPS parser integration |
| [main/gps_receiver.cpp](main/gps_receiver.cpp), [main/nmea_gps.cpp](main/nmea_gps.cpp) | GPS identification (PX1120S / u-blox M8) and startup configuration; independent streaming NMEA parser. `px1120s_gps.*` remains only for the inactive I2C path |
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

Tasks are pinned with `xTaskCreatePinnedToCore` using `main/task_cores.h`:
the `sensors` task runs alone on **core 1** (`task_cores::kEstimation`, reserved
for sensing, attitude and the planned Kalman filter); every other application
task is on **core 0** (`kSystem`). Core-1 SPI3 users (IMU, MS5607, RA-01) use
`spi_device_polling_transmit`, so they do not depend on an SPI interrupt
serviced from core 0. `main/cpu_load.cpp` logs every 10 s (tag `cpu`) the busy
share of each core (100 % minus its IDLE task) and the busiest tasks, from
FreeRTOS run-time stats (`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`, esp_timer
counter, enabled in `sdkconfig.defaults`). Bench, 2026-11-08, radio on, 960 Hz
IMU path: core 0 about 11 % busy on the pad (13-14 % while logging in flight),
core 1 about 28-30 %, heap free about 104 KB (minimum 101 KB), sensor stack
6 KB free of 8 KB. Before pinning, the scheduler put nearly everything on core 0
(32.5 % vs 3.3 %). There is one
FreeRTOS scheduler frequency, **1000 Hz (1 ms tick)**, configured by
`CONFIG_FREERTOS_HZ` in `sdkconfig.defaults` and the active (gitignored)
`sdkconfig`; ESP-IDF prefers an existing `sdkconfig` over the defaults, so
change both. It is not independently set per core or per task. Iris and
Aperture remain at 100 Hz. 1000 Hz is the ESP-IDF maximum (`FREERTOS_HZ`
Kconfig `range 1 1000`); a faster tick needs a patched ESP-IDF, and a faster
loop should instead be driven by the LSM6 data-ready interrupt (INT1, GPIO41)
or its FIFO. A task's requested millisecond timeout is quantized to the 1 ms
tick (`pdMS_TO_TICKS` truncates).

Tick-rate audit (100 to 500 Hz, then 1000 Hz, 2026-11-08; the notes below are
from the 500 Hz step and still hold at 1 kHz). Millisecond-specified periods keep
their length, except the sensor loop, now 2 ms (500 Hz, 2 ticks,
`static_assert`ed in `sensors.h`; was 10 ms),
flash capture 20 ms (50 Hz, the barometer's rate), sensor/state/debug packets, radio
schedule, heartbeat, USB 20 ms binary idle boundary, HITL 1000 ms timeout
(millisecond clock) and the 1 s flash flush. Changes: the UART read wait and the
CAN service loop's `vTaskDelay(1)` now wake every 2 ms instead of 10 ms (lower
latency; roughly 800 extra short wake-ups per second in total); the 75 ms Pad
Idle chirp is 74 ms instead of 70 ms; the RA-01 2 ms reset pulse is a real tick
instead of zero; the MS5607 PROM-reload wait is rounded up explicitly (at least
4 ms instead of one 10 ms tick); NAND busy polling (`pdMS_TO_TICKS(1)` = 0, a
yield) is unchanged. Estimated extra CPU: tick interrupts about 0.3% per core
plus the extra wake-ups, roughly 1-2% of one core in total. The idle-task
watchdog (5 s) is unaffected: no new busy loops longer than the 300 us radio
BUSY spin.

| Task | Priority | Stack bytes | Cadence or role |
|---|---:|---:|---|
| `sensors` | 4 | 8192 | Core 1. 500 Hz deadline-scheduled (2 ticks); drains the 960 Hz LSM6 FIFO, attitude, sole active SPI3 application service loop |
| `flash_capture` | 4 | 3072 | 50 Hz sensor snapshots (WL v2) and prelaunch ring |
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
| UART1 breakout | ESP32 RX 4, TX 5; swapped to RX 5, TX 4 while `HW_MCU_UART_SWAP_RX_TX` is 1 (currently) |
| Inactive GPS I2C aliases | SDA 4, SCL 5; cannot run concurrently with UART1 |
| Built-in USB | D- 19, D+ 20 |
| Battery divider | VSENSE GPIO1, ADC1 channel 0 |
| LEDs | Red 33, green 34, blue 47 |
| Passive buzzer | GPIO48 |
| CAN-labelled breakout | TX 37, RX 36; active 500 kbit/s Classical CAN |

`Sensors` initializes SPI3 and serializes IMU, barometer and radio operations in
its own task. Do not add an independent radio task that competes with those
operations without explicitly redesigning bus ownership. Radio transmission is
non-blocking (see Radio behavior), so it no longer stretches the sensor cadence.
`FlashLogger` owns its NAND instance on separate SPI2; export reads occur after
the writer has frozen.

## Sensing and telemetry production

Configuration constants live principally in `sensors.h` and the device headers:

- IMU SPI mode 3 at 10 MHz. Gyro (+/-2000 dps) and low-g accelerometer
  (+/-16 g) run at the 7.68 kHz ODR with on-chip low-pass filters (gyro LPF1 +
  LPF2 407 Hz, low-g LPF2 384 Hz); high-g accelerometer (+/-320 g) at 960 Hz.
  All three are batched into the LSM6 FIFO at 960 Hz and every sample is used
  (see "960 Hz IMU path").
- MS5607 SPI mode 3 at 10 MHz, pressure and temperature OSR 4096. Its polling
  state machine alternates conversions with a 9100 us conversion time. Altitude
  uses the configured sea-level reference of 1013.25 mbar, not a launch-site zero,
  converted with the full ISA (1976) layers to 47 km (`pressure_altitude.h`,
  shared with HITL input). The single-layer troposphere formula used before
  2026-11-08 read 0.2 km low at 15 km, 1.1 km low at 20 km and 4.6 km low at
  30 km; the new conversion is within 1 m to 32 km. It returns geometric
  altitude (h = r H / (r - H) from the ISA's geopotential H, r = 6,356,766 m),
  the same kind GPS reports. Without that the barometer reads 45 m low at 17 km,
  and the position filter rejected it against GPS. The MS5607 itself only
  measures down to 10 mbar (about 31 km).
- Battery sampling: 10 Hz, derived as a divisor of the 500 Hz sensor loop.
  Sixteen ADC readings are averaged, with 12 dB attenuation and a 15k/5k divider
  (battery voltage is four times the ADC-pin voltage). Wire units are millivolts.
- Sensor telemetry: 10 Hz. State telemetry: 1 Hz. Witness debug: 10 Hz only after
  enabling it. Heartbeat: 1 Hz.

Filtered acceleration (`main/accel_filter.{h,cpp}`, host test
`tests/accel_filter_test.cpp`) is computed for every 960 Hz FIFO sample from
the calibrated accelerometers:

- **Source selection:** low-g (+/-16 g, 0.49 mg/count) normally. On the
  first sample where any axis of either sensor reaches 15 g, all three axes
  switch to high-g (+/-320 g, 10.4 mg/count) on that same sample. The trigger
  uses both sensors because the chip filters the low-g after its ADC clips, so
  a clipped low-g reading can stay below full scale. It switches back once
  every axis of both sensors has stayed below 13 g for 10 samples (about
  10 ms), so it cannot chatter.
- **Low-pass:** 2nd-order Butterworth at 100 Hz
  (`Sensors::kFilteredAccelCutoffHz`; 0 disables it), bilinear transform with
  pre-warping, about 2.5 ms group delay. It runs on the selected signal and
  keeps its state across a switch, so switching adds no step (the test checks
  the output matches the same filter fed the unclipped truth to within 1 mg).
  It starts in steady state, so there is no start-up transient.
- **Uses:** `filtered_accel` in `0x01` and `0xE0`, and the flight state
  machine's launch detection (`axial_acceleration_g` = filtered x). Burnout
  detection still uses calibrated low-g x. NAND IMU status bit 7 (0x80) marks
  samples where high-g was the source.
- **HITL:** a second selector without the low-pass (frames arrive at about
  100 Hz). Both reset on every real/HITL source change.

Bench, board still, radio on: telemetry |a| noise 0.74 mg sd (1.29 mg before
the filter), still low-g, 0 switches. The switch to high-g has only been tested
on the host; it needs a real boost or a HITL run above 15 g. The `imu fifo`
log line now ends `filtered accel <low-g|high-g>, N switches to high-g, peak P
g; core-1 time: FIFO read R%, sample processing S%`. A one-off split measured
sample processing as about 1.6% calibration plus filter, 2% NAND queue and
7.6% attitude estimator (about 80 us per sample, the largest single cost),
with core 1 at about 34% busy in total.
The `0x02` and `0xE0` orientation quaternions come from the attitude estimator
(next section); position and velocity are still zero. MCU temperature and GPS
Unix time also remain zero in debug output.
GPS coordinates, altitude and satellite count are supplied separately by the
UART GPS snapshot (or the HITL sample), not by `Sensors::Sample`.

## IMU calibration

Bench measurement 2026-11-08 (board still, 40 s, radio on): gyro offsets of
about +0.6 to +0.9 dps on x and -0.4 to -0.9 dps on z, y near zero, and the
low-g accelerometer reading 0.992 g instead of 1 g. The x offset also drifts
while the board warms up (0.49-1.05 dps seen within minutes).

Two layers remove them (`main/imu_calibration.{h,cpp}`, pure and
host-tested; `main/imu_calibration_store.{h,cpp}`, NVS):

1. **Stored calibration** (NVS namespace `imu_cal`, survives reboot): gyro
   bias, low-g per-axis offset and scale, high-g per-axis offset. Applied to the
   engineering values the estimator, flight state machine and telemetry use
   (`Sensors::apply_imu_sample`). Captures use uncorrected values.
2. **Live pad gyro bias**: the attitude estimator keeps estimating the residual
   bias from stationary windows in Pad Idle (it freezes at launch). Telemetry
   gyro (`0x01`, `0xE0`) also subtracts it, so warm-up drift does not show; the
   estimator's own input is not, to avoid a feedback loop.

The NAND IMU stream (`I*.BIN`) and HITL data stay **raw/uncorrected**; the
calibration in force is printed at boot and after each change (captured in
`S*.TXT`). Telemetry `0xE0` low-g/high-g fields are now calibrated counts.

Commands (USB, Pad Idle, real sensors; `comms.h`
`IRIS_COMMAND_IMU_CALIBRATION`):

| Command | Action |
|---|---|
| `05 43 47` | Gyro bias: 4 s still capture, median per axis (robust to spikes), spread limit 3 dps; resets the estimator and saves |
| `05 43 41` | One face: 2 s still capture; classified by dominant axis (+X, -X, +Y, -Y, +Z, -Z, any order, up to ~14 deg off square). After all six, solves and saves |
| `05 43 58` | Clears the stored calibration and NVS entry |
| `05 43 53` | Logs the calibration and faces captured this boot |

Face solve: Gauss-Newton fit of per-axis offset and scale so every corrected
face has magnitude exactly 1 g (so tilted faces do not bias the scale), from
the opposite-face estimate; limits: offset <= 300 mg, scale 0.9-1.1. High-g
offset = mean (high-g - corrected low-g) over the faces. Results log as
`IMU face <F> captured ...`, `IMU accelerometer calibration done: ...` or a
`failed: <reason>` line. Guided host tool:
`python tools/imu_calibrate.py [--tcp HOST:PORT | --serial COMx]` (close Horizon
first; `--status`, `--clear`, `--gyro-only`).

Bench after the gyro calibration (same 40 s still test): telemetry gyro means
-0.04 / +0.01 / -0.36 dps (were +0.91 / -0.05 / -0.89), medians within 3
counts (0.21 dps). The six-face accelerometer calibration needs the board
turned by hand and has not been run yet; until then |a| stays 0.992 g.

## Position Kalman filter

`main/position_filter.{h,cpp}` (pure, host test `tests/position_filter_test.cpp`)
estimates position and velocity in ENU metres with the origin at the pad. It's
a linear Kalman filter with a constant-acceleration kinematic model:

- **Predict, every IMU sample (960 Hz; each 0x0C frame in HITL):** the
  filtered acceleration (see "Filtered acceleration") is rotated body to ENU by
  the attitude quaternion. Gravity is removed, and it drives p += v dt +
  a dt^2/2, v += a dt. Process noise is white acceleration noise of
  0.5 m/s^2 + 5% of |a| (attitude error turns part of the thrust sideways).
  The model and measurements are per axis, so the 6-state filter runs as three
  independent [p, v] filters (about 2.6% of core 1 at 960 Hz). Predict waits
  for a valid attitude.
- **Update, 25 Hz** (`Sensors::kPositionUpdateRateHz`): the newest
  barometer reading, plus any GPS fix that has arrived since the last tick.
  The PX1120S at 25 Hz gets every fix used; the u-blox M8 at 10 Hz appears
  every 2nd or 3rd tick.
  - **GPS:** E/N/U from latitude/longitude/altitude using WGS-84 metres per
    degree at the origin. Sigma is 2.5 m horizontal and 5 m vertical, doubled
    below 6 satellites. GGA has no HDOP or velocity, so it is position only.
    The origin is set on the pad. A first fix in flight is aligned to the
    current estimate.
  - **Barometer:** U, relative to a pad reference that is tracked while still
    on the pad.
- **Barometer trust:**
  - It is zero (no update) from Mach 0.8, and comes back below Mach 0.75.
    Mach is the filter's speed over the ISA speed of sound at the estimated
    altitude.
  - Otherwise its sigma follows the altitude accuracy:
    sigma_h = sqrt(0.5^2 + (10 Pa / (rho g))^2), with rho = p / (R T_ISA).
    That is 0.97 m at sea level, 2.5 m at 10 km and 11.6 m at 20 km.
  - Below the MS5607's 10 mbar floor the barometer is not used.
- **Gating:** innovations beyond 5 sigma are rejected. After 25 consecutive
  rejects, one is accepted so the filter can re-synchronize.
- **Pad and landing:** while still on the pad (|specific force| within 5% of
  1 g), zero position/velocity pseudo-measurements hold the state at the
  origin, and the baro and GPS references track the pad. Once the
  acceleration departs from 1 g it integrates freely, so the 0.3 s launch
  qualification before Boost is kept. In Landed, a zero-velocity
  pseudo-measurement stops drift.
- **HITL:** the filter resets on every real/HITL source change.
- **After gyro saturation** (attitude lost, see "Attitude estimator"): the
  accelerometer can no longer be rotated into ENU, so the filter predicts with
  zero acceleration and 10 m/s^2 of process noise (`predict_without_imu`).
  The barometer and GPS carry the estimate. The 10 s line then ends with `IMU
  ignored (attitude lost)`.
- **Outputs:** position and velocity in `0x02` and `0xE0` (float32, ENU m and
  m/s), and a 10 s log line: `position: ENU E N U m, velocity ..., Mach M,
  alt MSL A m; baro U used/G gated/R rejected, sigma S m; GPS U used/R
  rejected; pad hold|integrating`.

The estimate is not logged to NAND. Every input is, though (I*.BIN IMU,
S*.BIN pressure at 50 Hz, U*.BIN raw NMEA), so a flight can be re-filtered
offline.

Host test (simulated 11 g, 4 s boost to Mach 1.3, coast and descent, with a
150 m transonic baro error injected between Mach 0.8 and 1.2): baro-only
up error max 10.7 m and 0.2 m at apogee, with the transonic error never
entering (557 gated updates); 10 Hz GPS 18.9 m max / 0.5 m at apogee; 25 Hz GPS
13.0 m / 0.1 m. The motion before Boost is kept (1.35 m/s error out of about
32 m/s).

HITL on the bench (`a3241569-trajectory.csv`: 38 km apogee, Mach 4.2,
pressure below the MS5607 range for 83 s):

| Phase | Up RMS (max) | Horizontal RMS (max) |
|---|---|---|
| Pad idle | 0.1 m (0.2) | 0.0 m |
| Boost | 3.2 m (4.6) | 2.2 m (5.7) |
| Coast | 4.7 m (16.9) | 1.3 m (4.4) |
| Descent | 2.0 m (11.3) | 1.3 m (4.9) |

At apogee: 37,966.8 m against 37,967.1 m true. Final position within 1 m of
truth. Counters: baro 5,002 used, 2,206 gated (above Mach 0.8), 0 rejected;
GPS 9,285 used, 0 rejected (run `HITL/runs/20261108-223257`, scored against
the trajectory truth). HITL GPS is noiseless, so the horizontal numbers show
the filter tracking, not real GPS error. Two bugs this run exposed were fixed
first. The baro altitude conversion was troposphere-only and geopotential
(1,275 baro rejections at 9-16 km before). The HITL simulator converted GPS
with a 111,320 m/deg sphere (a 43 m north offset over 11 km); it now uses
WGS-84.

Outputs: the `0x02` state and `0xE0` debug packets carry the filter's ENU
position and velocity, and their former barometric-altitude slot now carries
the Kalman altitude above the pad (`position_filter_.up().p`, the same value
as Position Z). Raw pressure remains in `0x01`, `0xE0` and S*.BIN. Every update
cycle also writes the `0xE2` Kalman log to flash (see Flash storage) with the
covariances, innovations, gates and counters for post-flight review.
`Filter::begin_update_cycle()` clears the per-cycle innovation records
(`baro_innovation()`, `gps_innovation(axis)`), which `update_position` fills
for both accepted and gated measurements.

Limitations: no GPS velocity (GGA only) and no GPS latency compensation
(fixes are applied when parsed, typically tens of ms late). Accelerometer bias
is not estimated. Heading comes from the attitude estimator's initialization,
which has no magnetometer, so the horizontal frame is only as good as that
heading; GPS corrects position but not the rotation of the integrated
acceleration. The barometer pad reference uses the standard 1013.25 mbar
sea-level pressure, so "alt MSL" is pressure altitude.

## Attitude estimator

`attitude::Estimator` ([main/attitude_estimator.h](main/attitude_estimator.h))
is pure C++ with injected time; `Sensors::update_attitude()` feeds it once per
sensor pass that has a successful IMU read (or a new HITL frame) with body gyro
(rad/s, from `gyro_mdps`), body low-g specific force (g, from `low_g_accel_mg`),
a microsecond timestamp and the current flight state (read after the flight
state machine update of the same pass).

Convention (fixed in the `comms.h` "Orientation quaternion" comment): Hamilton
quaternion, wire order X, Y, Z, W, rotating BODY to WORLD. WORLD is ENU, +Z up;
BODY is the LSM6 frame with +X along the nose. All four components zero means
unavailable. Heading is unobservable; initialization uses the shortest-arc
rotation from the measured specific-force direction to world +Z, so an
exactly upright board gets body +Y = north and body +Z = west.

**Gyro interference is a hardware problem.** On the bench (2026-11-08, board
still on the desk) the LSM6 gyro shows impulsive spikes on all three axes at the
same instants: with the radio on, standard deviations of 6.6 / 10.4 / 43.6 dps
(x/y/z), spikes up to -358/+116 dps on z and 38% of samples above 10 dps; with
`HW_RADIO_ENABLED=0` still 1.25 / 1.49 / 4.7 dps with isolated 1-2-sample spikes
up to about 30 dps. The accelerometer is clean (|a| 0.9875..0.9923 g). The
pattern follows LoRa transmissions, so the gyro supply rail or layout couples
with the RA-01SH power amplifier (non-blocking TX made it visible, because the
IMU is now also read while the PA is on). The software below reduces the
effect but **cannot recover the true rate during a corrupted sample**; fix the
hardware (gyro/LDO decoupling, PA supply isolation, layout) before relying on
in-flight attitude with the radio transmitting.

| Phase | Behaviour |
|---|---|
| Pad Idle | Window membership comes from the accelerometer: every sample \|a\| in 0.95..1.05 g (any other sample, or a dt gap, restarts the window). A window completes after 1 s (at least 50 samples) and is **accepted** when: RMS of the per-axis accel standard deviations <= 0.02 g; per axis robust gyro sigma (1.4826 x MAD) <= 10 dps; at most 40% of samples have any axis more than 10 dps from that axis's median; and the per-axis median vector is <= 5 dps. The bias is the per-axis median refined by the mean of the samples within 3 robust sigmas of it, with the refinement clamped to one 70 mdps quantization step (`gyro_quantum_rad_s`). A median of quantized counts snaps to whole counts: with about 1.2 counts of noise it left up to 35 mdps of bias, which drifted a 300 s descent about 10 deg in HITL. The mean resolves below a count, and the clamp stops dense interference from pulling it further than one step. The first accepted window initializes the attitude (shortest arc from the mean accel). Later accepted windows set the bias, **restore the attitude to its value at the window start** (the board did not rotate, so gyro noise integrated over the window is discarded) and apply a tilt correction toward the window's mean accel (fraction window/2 s, about a horizontal axis, so heading is not changed). Every step is gyro propagated (median-filtered, bias-corrected), so non-stationary samples, including the >10 g, 300 ms launch-detection window, are kept and the attitude at the Boost transition already includes the motion since ignition. |
| Boost, Coast, Descent | Pure median-filtered, bias-corrected gyro integration; no accelerometer use. |
| Landed | Integration stops and the attitude is held ("attitude after flight"); logged once on entry. Re-entering Pad Idle by command resumes pad behaviour. |

Spike rejection: the integration path uses a per-axis running median of 5 gyro
samples (`Config::gyro_median_length`, 1 = off, 3 or 5), so the attitude lags by
2 samples (4 ms at 500 Hz; was 20 ms at 100 Hz) and each sample is integrated
with its own dt. At 500 Hz a median of 5 spans only 10 ms, so an interference
burst lasting more than 2 samples (4 ms) is no longer removed; LPF1 also spreads
a burst over several samples.
Runs of 1-2 outlier samples are removed; monotonic ramps (tested 0..500 dps)
pass exactly, and steps lasting 3 or more samples (tested 1000 dps) pass
unchanged, only delayed. Median of 3 would pass the observed 2-sample runs.
Cost: the first 2 samples after a reset or after leaving Landed are never
integrated, and the last 2 before entering Landed are dropped.

Integration is the exact quaternion exponential of the rotation vector
`(w_filtered - bias) * dt` (series form of sin(t/2)/t below 1e-3 rad),
right-multiplied in the body frame and renormalized each step, using the actual
timestamp difference. A dt <= 0 or > 50 ms is a gap: counted, that sample is not
integrated (it still feeds the median), and it restarts a pad window. dt > 15 ms
is counted as late but integrated. State is float. The estimator is reset (back
to all-zero) on every HITL entry, exit or timeout so real and simulated
attitudes never mix. If Boost is declared before any window was accepted (for
example a launch command right after boot), the quaternion stays zero for the
whole flight.

On-chip filtering, from the LSM6DSV320X datasheet (DS14623 Rev 3; register
bits also match ST's `lsm6dsv320x_reg.h`):

| Register | Value | Meaning | Source |
|---|---|---|---|
| CTRL1 (10h) | 0x0C | low-g ODR 7.68 kHz, high performance | Table 22 codes |
| CTRL8 (17h) | 0x43 | LPF2 ODR/20 = 384 Hz, +/-16 g | Tables 69, 71 |
| CTRL9 (18h) | 0x08 | LPF2_XL_EN, low-pass path | Table 72 |
| CTRL2 (11h) | 0x0C | gyro ODR 7.68 kHz, high performance | Section 6.1.3 |
| CTRL6 (15h) | 0x3C | LPF1_G_BW 011 (LPF1+LPF2 407 Hz), bit 3 = 1, +/-2000 dps | Tables 64, 66 |
| CTRL7 (16h) | 0x01 | gyro LPF1 on | Table 67 |
| CTRL1_XL_HG (4Eh) | 0xA4 | high-g output on, 960 Hz, +/-320 g | Tables 147-150 |
| FIFO_CTRL3 (09h) | 0x99 | BDR_GY = BDR_XL = 960 Hz | 9.7 |
| COUNTER_BDR_REG1 (0Bh) | 0x08 | XL_HG_BATCH_EN | Table 43 |
| FIFO_CTRL4 (0Ah) | 0x06 | continuous mode, no temperature or timestamp batching | Table 42 |

CTRL6 bit 3 "must be set to 1 for the correct operation of the device"; older
firmware wrote 0x04 with it clear.

Filter choice for the 960 Hz stream (Nyquist 480 Hz). At 7.68 kHz the gyro's
fixed LPF2 alone is 537 Hz (Table 23), above Nyquist, and no LPF1 code gives
about 532 Hz (7.68 kHz column of Table 66: 281, 213, 156, 407, 102, 58, 28.8,
14.4 Hz), so LPF1 code 011 (407 Hz) is used. For LPF2-only (537 Hz, aliasing
480-537 Hz) set `kGyroFilterConfiguration` to 0x00. The low-g LPF2 options at
7.68 kHz are ODR/10 = 768 Hz or ODR/20 = 384 Hz; 384 Hz is used. The high-g
channel has no selectable filter, so it runs at 960 Hz rather than being
decimated from 7.68 kHz. The 960 Hz BDR is exactly 1/8 of the ODR. 1 kHz would
need HAODR mode (`HAODR_SEL` 01: 8 kHz ODR, different timing) or software
polling at the 1 ms tick, which jitters and drifts against the sensor clock, so
960 Hz is the better of the two.

Earlier gyro LPF1 measurements (2026-11-08, 120 and 960 Hz ODR, same session,
board still, LoRa radio transmitting) showed LPF1 making the radio interference
worse; LPF1 is on now because the 960 Hz stream needs an anti-alias filter.
Recheck gyro noise with the radio on at the new settings:

| Gyro configuration | z sd (dps) | z mean (dps) | Attitude init |
|---|---|---|---|
| 120 Hz, LPF1 off, CTRL6 0x04 (old) | 76 | -13 | yes, ~27 s |
| 120 Hz, LPF1 off, CTRL6 0x0C (current) | 84 | -9 | yes, ~20 s; half as many samples > 3 dps |
| 120 Hz, LPF1 code 110 (24.2 Hz) | 114 | -38 | no |
| 960 Hz, LPF1 code 110 (28.7 Hz) | 177 | -60 | no |

The interference reaches the gyro's analog front end, so digital filtering
rectifies it into bias rather than removing it; the estimator's running median
handles isolated spikes instead. Revisit LPF1 after the board-level radio
coupling is fixed. The interference level itself varies over time (an earlier
baseline measured 44 dps z sd), so compare only same-session numbers.

Timestamps: real IMU steps come from a sample clock that advances one sensor
period per FIFO sample. The period is measured, not nominal: the LSM6 ODR
differs from typical by 0.13 % x `INTERNAL_FREQ_FINE` (4Fh); this board runs
about 2.5 % slow (about 936 Hz instead of 960). Each pass the newest sample is
taken as nominally "now" (`esp_timer` after the FIFO read), and the clock is
steered 1/16 of the way toward that, snapping to it at startup, after a FIFO
overrun or after an error above 5 ms. dt therefore stays smooth (bench max dt
1.1 ms, no late steps) while its sum tracks real time. A sample is about one
FIFO drain interval (2 ms) older than its timestamp; that constant lag does not
affect integration. In HITL the step time is the start of the sensor pass that
consumed the newest `0x0C` frame (`sim_time_ms` is not used). Frames arrive
asynchronously at about 100 Hz; a pass with no new frame is skipped and the next
step spans about 20 ms (counted as late), and if two frames arrive within one
pass only the newest is used. The integrated time therefore tracks Witness
wall-clock time, which equals simulation time only if the host streams in real
time. Add the 20 ms median lag when comparing against a truth trajectory.

Gyro saturation (2026-11-09): if any raw gyro axis reads at or above
1950 dps (the full scale is +/-2000) for 2 consecutive samples in Boost, Coast
or Descent, as a parachute deployment can cause, the true rate is unknown and
the attitude cannot be recovered.
- The estimator stops integrating and `valid()` becomes false, so `0x02` and
  `0xE0` report an all-zero quaternion.
- `lost()` stays set for the rest of the flight, including Landed.
- Only a new stationary window after returning to Pad Idle re-initializes it.
- One saturated sample (an interference spike) does not trigger it.
- The position filter then ignores the IMU.
- Log line (deferred, tag `sensors`): `gyro saturated (|rate| >= 1950 dps on an
  axis for 2 samples): attitude tracking stopped for this flight; the position
  filter now ignores the IMU`. The 10 s attitude line shows `lost (gyro
  saturated)`.

Limitations: no magnetometer (heading is relative to the initialization only);
gyro saturation ends attitude tracking for the flight (above); no
coning compensation or gyro scale/
misalignment calibration; the bias is assumed constant in flight; slow pad
rotation below 5 dps about the gravity vector (invisible to the accelerometer)
for a 1 s window is treated as bias and undone; the spike filter cannot recover
true rates during clustered interference; no accelerometer use after launch, so
attitude error grows with flight time.

Log lines (tag `sensors`): `attitude initialized: tilt T deg from vertical, gyro
bias X Y Z dps`; `attitude reset: sensor source is now HITL 0x0C frames|the
real IMU`; on entering Landed `attitude after flight: q xyzw [x y z w], tilt T
deg from vertical, integrated N steps over S s (gaps G, late L)` (or `attitude
after flight unavailable: ...`); and every 10 s `attitude: R Hz, max dt D ms,
gaps G, late L, spikes rejected S, windows A ok/B accel/C gyro rejected,
overruns O, max pass P us, radio TX N (failed F), <valid|held|not initialized>,
tilt T deg, q xyzw [...]`. R is attitude updates per second over the interval;
max dt and max pass reset each report, the counters are cumulative. A second
line follows: `imu fifo: R Hz, overruns O, partial slots P, unknown words U, max
backlog W words` (FIFO samples per second; latched FIFO overruns; time slots
missing a channel; words with unexpected tags; largest DIFF_FIFO seen in the
interval). A spike is
counted when a raw sample differs from the median output by more than 10 dps on
any axis. `overruns` counts passes whose 2 ms deadline had already passed when
the loop reached `xTaskDelayUntil`. If the line keeps saying `not initialized`,
the `gyro rejected` count shows the interference exceeded the window limits.

Host test: `tests/attitude_estimator_test.cpp` (tilt initialization from eight
orientations, bias estimation, rotating-board rejection, constant-rate
integration at 100 Hz and with 6..16 ms jittered dt up to 1720 dps, 500 dps ramp,
1000 dps 3/4-sample steps, 1/2-sample spike rejection, gaps, launch window, pad
tilt correction, pure integration in flight, Landed hold, zero before
initialization) plus a bench-interference model: the measured z sequence `0 -5
-573 1384 541 77 -19 0 -33 94 177 -702 -2 697 6 8 -17 1` (counts) injected as
bursts at random times (x/y scaled 0.15/0.24 with random sign) over 2-count
Gaussian noise and a 0.4/-0.3/0.6 dps bias, at 15% ("radio off") and 90%
("radio on") burst coverage, three seeds each. Results:

| Case | Init | Bias error | Pad 60 s drift, final / max | Coast 10 s drift (pure integration) |
|---|---|---|---|---|
| radio off, median 5 | 1.01 s | 0.012-0.029 dps (median alone: 0.02-0.04) | <= 0.04 / 0.51-0.79 deg | 1.4-2.9 deg |
| radio off, no filter | 1.01 s | same | <= 0.02 / 2.8-3.5 deg | 6.8-14.7 deg |
| radio on, median 5 | 1.01 s | 0.065-0.19 dps (median alone: 0.07-0.13) | <= 0.07 / 1.44 deg | 10.4-11.7 deg |
| radio on, no filter | 1.01 s | same | <= 0.01 / 7.1-7.2 deg | 56-58 deg |

The pad final drift is small with or without the filter because each accepted
window restores the attitude; the filter keeps the within-window excursion
(max) small. The coast column shows what interference does in flight.

## 960 Hz IMU path

The sensor loop still runs at 500 Hz (`Sensors::kTaskRateHz`, 2 ticks of the
1 kHz FreeRTOS tick, the ESP-IDF maximum), but IMU data no longer depends on
it. The LSM6 batches gyro, low-g and high-g words into its 1.5 KB FIFO at
960 Hz (`Sensors::kImuSampleRateHz`) on its own clock; each pass reads
FIFO_STATUS1/2 and drains the waiting words in one SPI burst from
FIFO_DATA_OUT_TAG (78h) (with IF_INC the address rolls back from 7Eh to 78h; up
to 36 words per burst into DMA-capable buffers). Words are grouped into time
slots by TAG_CNT; a slot is complete when it has all three channels, and a
missing channel keeps its previous value (counted as a partial slot). Each
sample, oldest first, is logged to the NAND IMU stream and steps the attitude
estimator, whose timing comes from `Sensors::attitude_config()` (nominal
1.04 ms, late above 1.56 ms, a stationary window needs 480 samples; the window
buffer holds 1024). The flight state machine and the telemetry packets use the
newest sample once per pass. Temperature is read from OUT_TEMP at 10 Hz. A late
pass loses nothing: the FIFO holds about 76 ms. HITL still steps once per 0x0C
frame (100 Hz from the tool).

Bench, real IMU, radio transmitting, 2026-11-08 (10 s reports): 936-940 Hz
(the part's ODR is about 2.5 % slow), FIFO overruns 0, partial slots 0,
unknown words 0, maximum backlog 12-99 words, attitude max dt 1.1 ms with no gaps or late
steps, about 13 loop deadline overruns per second with max pass 3.4-7.5 ms
(these no longer lose samples), core 1 28-30 % busy. A commanded flight (Boost,
Coast, Descent, Landed) logged the 1920-record prelaunch IMU dump and live
records with no IMU queue drops. Not yet run: a HITL flight at the new rate, a
long soak, a flight-rate gyro test, or a gyro noise comparison at the new
filters.

IMU logging capacity (store v3, 24-byte records, 84 per page after the
12-byte page header, 96 blocks = 516,096 records per slot): 960 Hz until
Descent, then every 10th record (96 Hz). A 20-minute flight with a 3-minute
ascent is about 270,000 records (about 50 blocks); 960 Hz for the whole slot
lasts about 9 minutes. NAND bandwidth is not a limit (about 11 page programs/s
at 960 Hz).

Known issues: (1) some passes exceed the 2 ms budget (battery ADC, radio TX
start, log formatting); the FIFO absorbs this, but moving the battery ADC out
of the SPI3 task would still help; (2) LPF1 made radio interference worse in
the earlier bench test (see the LPF1 table), so recheck gyro noise with the
radio on; (3) the 5-sample spike median now spans only about 5 ms.

The sensor task wakes every `kTaskPeriod` (2 ticks = 2 ms) with
`xTaskDelayUntil`, so periods are measured from the previous deadline, not from
the end of a pass. Each pass first services HITL, reads the IMU (timestamped),
the barometer and the battery divisor, updates the flight state machine and the
attitude, queues telemetry, and only then services the radio. Before this
change the loop measured about 70 Hz on the bench because `Ra01::transmit()`
waited for TxDone (about 30-170 ms of LoRa airtime per frame, several frames per
second) inside the loop. Now `Ra01::start_transmit()` loads the frame and issues
SetTx (well under 1 ms of SPI), and `poll_transmit()` on later passes reads the
IRQ only once DIO1 rises (or a 1.1 s host deadline passes), then clears it and
restores continuous RX. While a frame is on the air the loop does not poll RX
(the radio cannot receive while transmitting) and does not start another frame;
the packet schedule and round-robin are otherwise unchanged. `wait_until_ready`
spins up to 300 us on BUSY before falling back to a one-tick sleep, instead of
sleeping a whole tick whenever BUSY is briefly high. The blocking
`Ra01::transmit()` remains as a wrapper but is no longer used.

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
| `02` | State | 63 | 65 |
| `03` | Camera telemetry | 17 | 19 |
| `05` | Command | 8 | 10 |
| `0C` | HITL sensor input (host to Witness, USB only, never forwarded) | 45 | 47 |
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
host input (except `0C` HITL frames), diagnostic text, raw GPS/NMEA and received radio payloads. A text
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
Landed 4. Bit 3 is GPS locked (a fresh GGA fix with quality >= 1 from UART or
HITL), set each sensor-loop pass. Bit 4 is HITL active. Bits 5..7 are reserved
zero. `WitnessStatus::set_state_flag()` owns bits 3/4; `flight_state()` returns
only the state. There is no radio-ready bit. Readiness flags should not be
mistaken for a universal freshness guarantee.

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
| `05 43 47` / `05 43 41` / `05 43 58` / `05 43 53` | IMU calibration: gyro, face, clear, status (USB, Pad Idle; see "IMU calibration") |
| `05 00 00 00 00 00 00 48 49 0A` (canonical only) | Enter HITL mode; USB and Pad Idle only |
| `05 00 00 00 00 00 00 48 58 0A` (canonical only) | Exit HITL mode |
| `TX 05 ...` | Bridges decoded command bytes to CAN/radio; do not assume it performs the same local actions as the non-TX form |
| `TX ID payload...` with another ID | Queues the future SPI command path; no hardware SPI transmission yet |

The protected erase is a magic opcode, **not authentication or a confirmation
dialog**. Radio can invoke destructive commands too. Do not transmit erase or
export commands merely to inspect or test the system.

`CommandBridge` validates canonical commands and normalizes legacy three-byte commands to complete 10-byte frames before forwarding to Iris CAN. UART1 is exclusively the GPS receiver. USB commands can additionally enter the immediate radio queue; radio commands are queued for local interpretation independently of CAN availability. Invalid EOF/length commands are rejected before forwarding.

Radio local handlers support debug enable, export, erase, state setting and
HITL exit; HITL enter from radio is rejected and logged.
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
USB, and submits `05` packets to `CommandBridge`. Transmission is non-blocking
(started in one pass, completed on a later pass when DIO1 signals TxDone; see
"100 Hz sensor loop"). The radio returns to continuous receive after transmit. Init failure releases the device and does not schedule
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
from RMC/ZDA. It also keeps GGA fix quality (field 6) and satellites used
(field 7). No-fix GGA immediately invalidates the position but its satellite
count is still reported; the GGA expires after 3000 ms (`NmeaGps::gga_fresh`).
Invalid or stale state/debug GPS fields are zero, which is not a unique invalid
sentinel because zero coordinates and altitude can also be legitimate; use the
GPS-locked status bit. State packet byte 63 carries satellites (0 when stale).

`GpsReceiver` ([main/gps_receiver.h](main/gps_receiver.h)) identifies the fitted
receiver, then configures it in receiver RAM on every boot (no receiver flash
writes). Both are set to their airborne 4 g dynamics and their maximum update
rate, with NMEA reduced to GGA + RMC so the higher rate fits 115200 baud:

| Receiver | Identified by | Dynamics | Rate | Maximum for the part |
|---|---|---|---|---|
| SkyTraq PX1120S | SkyTraq binary reply to `64 18` | mode 5, airborne (4 g rated) | 25 Hz | 25 Hz; 50/100 Hz only on PX1120S-H |
| u-blox NEO-M8N | any UBX reply to the `MON-VER` poll | `dynModel` 8, airborne <4 g | 10 Hz | 10 Hz concurrent GNSS; 18 Hz single GNSS |

Identification, after a 1.5 s boot delay:

1. At 115200, send both polls (each family ignores the other's) up to three times.
2. A checksum-valid reply picks the model. If none arrives but valid NMEA does,
   the link is left untouched and the PX1120S is assumed (its mode query has
   gone unanswered before); readback still decides success.
3. Only a **silent** link is retried at 9600, the u-blox factory rate. A u-blox
   found there gets `CFG-PRT` (UART1, 115200, UBX+NMEA); the host follows after
   150 ms and confirms with another `MON-VER` poll.
4. Configure, then read back. PX1120S: navigation mode and update rate. u-blox:
   `CFG-NAV5` dynamic model and `CFG-RATE` period. Missing PX1120S ACKs are
   tolerated; u-blox ACKs are required.

Failure of any step leaves raw logging running; an unidentified receiver leaves
the host at 115200. Log lines: `GPS identified as <model> at <baud>`,
`GPS UART host rate now <baud>`, `<model> airborne 4 g mode and <N> Hz GGA+RMC
confirmed`, or `<model> startup failed: <reason>`. Frames are listed in
[GROUND_STATION_PROTOCOL.md](GROUND_STATION_PROTOCOL.md) section 6.

**NAND budget at the higher rates.** Raw UART bytes are logged in every flight
state into a **circular** 341-block section (about 44.7 MB) per session. GGA +
RMC is about 150 bytes per fix, so it holds about 3.3 h with the PX1120S at
25 Hz. When full it erases and overwrites its oldest block (logged once as `GPS
log ring wrapped`), so a long pad wait loses the oldest pad NMEA, never the
flight. The export reads the ring oldest page first, by page sequence number.

Bench status: on 2026-11-07 the PX1120S confirmed airborne mode with the previous
firmware. The rate and NMEA changes, and the whole u-blox path, are covered by
host tests only and are not yet verified on hardware. Five-second `GPS RX`
diagnostics report baud, byte count, GGA count, fix freshness and idle time.

## HITL mode

Hardware-in-the-loop mode lets a USB host drive the flight software with
simulated sensors. Pure logic (decode, framing, enter/exit/timeout with
injected time) is in `hitl_input.*`; `Sensors` owns a `hitl::Controller` under
a spinlock and the USB task feeds it.

- **Enter** `0x4849`, canonical 10-byte frame only, accepted only from USB and
  only in Pad Idle. Radio, compact form and other flight states are rejected
  and logged. Reboot always starts with HITL off.
- **Exit** `0x4858` from USB or radio, or automatically after
  `IRIS_HITL_INPUT_TIMEOUT_MS` (1000 ms) without a valid `0x0C` frame, counted
  from entry or the last frame. Exit does not change the flight state.
- **Simulated**: low-g, high-g, gyro, IMU temperature, pressure (altitude via
  `pressure_altitude.h`, the MS5607 formula), barometer temperature, GPS
  position/altitude/satellites/quality and battery. While active the sensor
  task does not read the LSM6, MS5607 or battery ADC; SPI3 radio RX/TX,
  CAN, heartbeat and flash logging continue (flash records simulated data).
- **Substitution**: `Sensors::service_hitl()` runs at the top of each sensor
  pass. A new sample is copied into `working_sample_` and counts as one IMU,
  barometer and battery update, so the flight state machine, `0x01`, `0x02`,
  `0xE0`, `latest_sample()` and flash capture all use it.
  `Sensors::current_gps_fix()` returns the simulated fix in place of
  `UartDriver::latest_gps_fix()` for state, debug and the GPS-locked bit. The
  UART parser and raw GPS logging keep running.
- **Attitude**: the estimator is reset on enter, exit, timeout or re-entry and
  then runs on the simulated gyro/low-g values exactly as on real data; the
  quaternion is zero until the first accepted 1 s stationary window of
  simulated samples, and lags by the 20 ms gyro median. The step time
  is the sensor-pass time that consumed each frame (see "Attitude estimator").
- **Readiness**: on enter, exit, timeout or re-entry the working sample and
  all five readiness flags are cleared. Until the first frame (or, after
  exit, the first real reading) readiness flags are clear, battery/barometer
  are zero, GPS has no fix and 0 satellites, no `0x01` is sent and the flight
  state machine gets no observation. In HITL each frame sets the four IMU
  flags; the barometer flag follows pressure != 0.
- **USB framing**: a `0x0C` byte starts a 47-byte frame unless a binary
  `0x05` command is in progress. Text commands are printable, so a `0x0C` or
  `0x05` byte discards any unterminated text line first; otherwise stale text
  (seen on the bench as echoed boot log output with no newline, from the tty
  being in default echo mode while ser2net reopens it) swallowed the next
  binary command. The frame is
  checked for EOF at byte 46; a bad EOF resynchronizes on the next `0x0C` in the
  rejected bytes, and a partial frame idle for 20 ms is dropped. There is no CRC,
  so a resync through payload bytes can rarely accept one misaligned frame.
  Bytes consumed as frames are **not echoed** (saving ~4.7 KB/s at 100 Hz) and
  never reach `CommandBridge`; `TX 0C ...` is rejected. HITL `0x05` commands are
  still bridged to Iris and the immediate radio queue like other USB commands.
- **960 Hz IMU (`0x0E`, 2026-11-09)**: 150-byte frames, each carrying a u32
  first-sample index and 8 consecutive 960 Hz samples (int16 low-g, high-g,
  gyro xyz, native counts). They're framed like `0x0C`; 0x0E is used because
  0x0D is CR, which typed text commands contain. `Sensors::submit_hitl_imu_batch`
  queues them under the HITL spinlock (256 samples). The sensor task drains
  them into the same per-sample path as real FIFO samples
  (`process_imu_batch(true)`: filtered accel, attitude, position predict, NAND
  IMU log with the HITL flag), with calibration bypassed.
  - Sample times come from the index (1/960 s apart), re-anchored if the index
    goes backwards or jumps by more than 1 s.
  - While `0x0E` frames arrive (any within 250 ms), the IMU fields of `0x0C`
    are ignored. `0x0C` still supplies barometer, GPS, temperature and battery
    at 100 Hz.
  - Hosts that send only `0x0C` work as before.
  - The noise, vibration and filtering are synthesized on the desktop
    (`Software/HITL/imu_synth.py`); Witness receives only the 960 Hz
    readings.
- **Log lines** (tag `hitl` unless noted): `HITL mode entered from USB: ...`
  (W), `HITL enter rejected: received from radio; ...` (W),
  `HITL enter rejected: flight state is <state>; HITL requires pad idle` (W),
  `HITL enter ignored: already active`, `HITL mode exited by <USB|radio>
  command; real sensors restored`, `HITL exit from <src> ignored: HITL not
  active`, `HITL input timeout: no valid 0x0C packet for 1000 ms; exiting HITL,
  real sensors restored` (W), `first HITL sample received (sim time N ms)`, a
  10 s `HITL active: N samples ...` summary, and in tag `serial_command`
  rate-limited `0x0C HITL frame rejected: ...` and `N 0x0C HITL frame(s)
  ignored: HITL mode is not active` warnings.

## Flash storage and export

The `W25n01kv` driver expects JEDEC **EF AE 21**, using SPI2 mode 0 at 10 MHz,
2048-byte data pages, 64 pages/block and 1024 blocks. Spare size is currently 64
bytes. Check actual ID and ECC handling when changing chip variants; older
comments and file/class names are not sufficient identification.

The logger uses a **custom append-only NAND store**, not a mounted writable FAT
filesystem. On successful initialization it tests a reserved good block by
erase/write/read comparison, scans sessions and creates a new session. Store
version 4 (2026-11-08) is built for **one flight at a time**: two slots, A and
B, of 502 blocks each, plus 20 reserved blocks at the top of the chip. Each
slot:

| Blocks | Export | Contents |
|---|---|---|
| 0 | - | Session header (page 0) and flight marker (page 1, magic `WLFT`) |
| 1-96 | `I<gen>.BIN` | IMU stream, 24-byte records (below) |
| 97-136 | `S<gen>.BIN` | 24-byte WL v2 sensor records at 50 Hz (about 72 min) |
| 137-160 | `S<gen>.TXT` | Captured ESP log text, circular (keeps the newest) |
| 161-280 | `P<gen>.BIN` | Packet stream, circular (below) |
| 281-501 | `U<gen>.BIN` | Raw GPS UART bytes, circular (see GPS section) |

Store version 3 had no packet stream and 341 GPS blocks (161-501). Its
sessions still export through that layout (`FlashLogger::file_layout` by
version), so a v3 flight survives the upgrade until a v4 flight replaces it.

Slot choice at boot: the slot holding the newest *flight* (marker written) is
kept and the other slot is reused; with no flight on the chip, the empty or
older slot is reused. The flight marker is written when the first logged state
(Boost) begins, so pad-only power cycles never overwrite the last flight, and
the next flight overwrites the flight before it. Every page starts with a
12-byte header (magic, type, store version, payload length, u32 page sequence).
Store versions before 3 are not exported.

**Circular streams and launch.** The text, packet and GPS streams wrap freely
on the pad, so a long pad wait never fills them. When the flight marker is
written, at the first logged state (Boost), each ring is launch-protected:
nothing written after that point is overwritten. A ring that comes back round
to its launch point stops, with `ring full since launch; stopped to keep the
flight`. That warning is not a failure and does not set the flash fault bit.
After launch the packet ring holds about 40 min (state at 50 Hz, the 25 Hz Kalman log and CAN, ~6.3 KB/s), the GPS ring (221 blocks,
about 28.8 MB) about 2.1 h at 25 Hz GGA+RMC, and the text about 5 h.

**Packet stream (`P*.BIN`).** It records every packet in four classes:
- Witness's own `0x02` state packet at 50 Hz, the barometer's rate
  (`Sensors::kStateLogRateHz`). It is built in the sensor task every 20 ms for
  the log only; USB and radio keep 1 Hz.
- Every packet reassembled from CAN (from Iris), whether or not it is
  forwarded.
- Every packet sent to Iris over CAN (commands, the 1 Hz heartbeat).
- Every command received over the radio.
- The `0xE2` Kalman filter log at 25 Hz, one per position-filter update cycle
  (`Sensors::log_kalman_packet`, source 5): ENU position/velocity, the P00,
  P11 and P01 covariance terms per axis, the barometer and GPS innovations of
  that cycle (NaN if not offered) with the barometer innovation variance and
  sigma, Mach, gate/hold flags and the update/reject counters. Layout in
  `comms.h` (`IRIS_PACKET_KALMAN_LOG_*`) and GROUND_STATION_PROTOCOL.md. It
  adds ~2.7 KB/s, so the packet ring now lasts about 40 min after launch.

`FlashLogger::log_packet()` never blocks. Its queue holds 128 packets, and
drops are counted and warned at flush (`packet log queue full: N packets
dropped so far`). Records are big-endian:

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 4 | uint32 esp_timer microseconds (wraps every 71.6 min) |
| 4 | 1 | Source: 1 Witness state, 2 CAN in, 3 CAN out, 4 radio in, 5 Kalman log |
| 5 | 1 | Frame length N |
| 6 | N | Complete frame, ID through EOF `0x0A` |

Horizon `converter.py` decodes it (`P*.BIN`, or `--format packets`) with the
normal packet parser. There is one row per frame, with `log_source` and
`time_seconds`.

Bench check on 2026-11-08, with a commanded flight and Iris on CAN:
- 9,171 state packets over 92 s, 100.01 Hz with a 10.00 ms median.
- Iris camera packets at about 1 Hz (CAN in).
- The heartbeat and forwarded commands (CAN out).
- No queue drops and no undecodable bytes.

Before the 10 s status lines were moved off the sensor task (they are now
formatted there and printed by a core-0 `sensor_log` task), printing them
stalled the loop 16-28 ms and skipped 1-2 state samples every 10 s.

Cost: about 14 KB of RAM for the queue (free heap was about 64 KB in that
check). Core 1 measured 45% busy, up from about 38%. Most of the increase is in
the FIFO-read and sample-processing timings, which this change did not touch;
the cause is not confirmed.

| Export filename example | Contents |
|---|---|
| `I0000001.BIN` | 24-byte IMU records (decode with Horizon `converter.py`) |
| `P0000001.BIN` | Packet stream: state at 50 Hz, CAN in/out, radio commands (decode with `converter.py`) |
| `S0000001.BIN` | Fixed 24-byte WL v2 sensor records; not canonical telemetry packets |
| `S0000001.TXT` | Captured ESP logging output after the application log hook is installed |
| `U0000001.BIN` | Exact raw UART bytes, including NMEA and binary responses; no added timestamps or framing |

IMU stream: the sensor task hands every FIFO sample to
`FlashLogger::log_imu_record()` (non-blocking; a 2048-record queue, drops
counted and warned at flush). In Pad Idle the writer keeps the last 2 s (1920
records) in a RAM ring and dumps it when Boost begins; 960 Hz records are then
kept through Coast, every 10th in Descent (96 Hz), and none in Landed.
Big-endian record layout (`FlashLogger::make_imu_record()`):

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 4 | uint32 sample time, low 32 bits of esp_timer microseconds (wraps every 71.6 min) |
| 4 | 6 | Three int16 low-g raw counts |
| 10 | 6 | Three int16 high-g raw counts |
| 16 | 6 | Three int16 gyro raw counts |
| 22 | 1 | Status: bits 0-2 flight state, 0x08 HITL, 0x10/0x20/0x40 low-g/high-g/gyro fresh, 0x80 filtered accel from high-g |
| 23 | 1 | uint8 record sequence (wraps at 256; gaps reveal drops) |

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
Exhausting one linear allocation (IMU or sensor records) marks that stream full
and sets the flash fault flag without stopping the other allocations. The text,
packet and GPS streams are circular and never fill on the pad (after launch see
"Circular streams and launch"). A fatal writer I/O error stops all
capture.

Flash-log-failed (status bit 7) history, 2026-11-08: the bit latched about an
hour after power-on on the bench and in HITL runs. The cause was USB
backpressure, not the NAND. With no host reading USB (always the case in
flight, and on the bench with no ser2net client), the telemetry forwarder
retried each write forever. Its 16-packet queue filled, and every sensor, state
and debug packet then logged `packet queue failed: ESP_ERR_NO_MEM`, 10-20
lines/s and 77% of the text log at 890 B/s. The 3.1 MB text stream filled in
about an hour, and a full stream sets the bit.

Fixes:
- The forwarder drops packets once USB stops draining (one 20 ms wait, then
  no wait until a write succeeds) and logs `USB output not drained ... N
  packets dropped so far` every 10 s.
- Queue failures and UART forward warnings are summarized every 10 s.
- Text is circular.

The text log now runs at about 160 B/s with no USB client.

Circular streams and session tags: a new session erases only the first
block of each stream, so a slot still holds valid pages from older sessions.
Each page's sequence number carries the session generation in its top 12 bits
(`FlashLogger::sequence_base`), and the scan and export skip pages from other
sessions. Before this, U*.BIN (since store v3) and the circular text exported
stale pages from older sessions mixed into the current one. Files written
before the tagging export untagged as before. Check which condition occurred before assuming that
every stream has stopped or that a healthy initialization guarantees logging.

The exported sensor record layout is independent of `comms.h`. Multi-byte fields
are big-endian; the source of truth is `FlashLogger::serialize_record()`:

Record version 2 (2026-11-09) is 24 bytes at 50 Hz, the rate at which the
MS5607 delivers new pressure (100 Hz only repeated it). It keeps only what no
other stream logs:
- the raw low-g, high-g and gyro samples are in I*.BIN at 960 Hz;
- battery, GPS, attitude, position, velocity and the Kalman altitude are in
  the 50 Hz state packets in P*.BIN; barometric altitude is recomputed from the
  logged pressure (converter.py adds `barometric_altitude` to WL v2 rows).

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 2 | Magic `57 4C` |
| 2 | 1 | Record version 2 |
| 3 | 1 | Witness status byte 0 (flags) |
| 4 | 1 | Witness status byte 1 (flight state bits 0-2, GPS lock 0x08, HITL 0x10) |
| 5 | 1 | Reserved, 0 |
| 6 | 4 | uint32 capture sequence |
| 10 | 4 | uint32 uptime ms |
| 14 | 2 | Signed int16 LSM6 raw temperature |
| 16 | 4 | Signed int32 pressure in centi-mbar (numerically Pa) |
| 20 | 4 | Signed int32 MS5607 temperature in centi-Celsius |

At 50 Hz about 6% of consecutive records repeat the previous pressure (it
was 52% at 100 Hz). Version 1 (48 bytes at 50 Hz: status
byte 0 only, sequence, uptime, raw low-g/high-g/gyro, IMU temperature,
pressure, baro temperature, float32 baro altitude, uint32 battery mV) is
still decoded by `converter.py`. The size comes from the version byte.

Bench, commanded flight on 2026-11-09 at 50 Hz: 1,600 records at 50.03 Hz with
no sequence gaps, and 50.02 Hz state packets in P*.BIN. The only gap is
70 ms at boot.

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
`flashit.ps1` targets an RFC2217 remote endpoint. The VS Code settings list
COM4, but an old IDE setting is not proof that a board is connected there.
Do not flash a remote bench or erase NAND without an explicit request.

Host tests are standalone C++17 programs, not an integrated CTest suite. With
`g++` available, examples for PowerShell are:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/nmea_gps_test.cpp main/nmea_gps.cpp -o build/nmea_gps_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\nmea_gps_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/gps_receiver_test.cpp main/gps_receiver.cpp -o build/gps_receiver_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\gps_receiver_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/px1120s_gps_test.cpp main/px1120s_gps.cpp -o build/px1120s_gps_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\px1120s_gps_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/skytraq_i2c_mailbox_test.cpp main/skytraq_i2c_mailbox.cpp -o build/skytraq_i2c_mailbox_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\skytraq_i2c_mailbox_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/hitl_input_test.cpp main/hitl_input.cpp -o build/hitl_input_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\hitl_input_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/attitude_estimator_test.cpp main/attitude_estimator.cpp -o build/attitude_estimator_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\attitude_estimator_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/imu_calibration_test.cpp main/imu_calibration.cpp -o build/imu_calibration_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\imu_calibration_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/accel_filter_test.cpp main/accel_filter.cpp -o build/accel_filter_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\accel_filter_test.exe

g++ -std=c++17 -Wall -Wextra -Werror -I main tests/position_filter_test.cpp main/position_filter.cpp -o build/position_filter_test.exe
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
& .\build\position_filter_test.exe
```

This PC has host `g++` 16.2 from WinLibs (installed with
`winget install BrechtSanders.WinLibs.POSIX.UCRT`; it lives under
`%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs*\mingw64\bin`,
add that to `PATH`). All nine tests above passed there on 2026-11-08.
esp-clang under `C:\Espressif\tools` targets the ESP chips only and cannot
build host tests.

The tests exercise parsing, fixed-baud configuration protocol handling, HITL
decoding/framing/state rules and the inactive mailbox implementation using
fake transports. They do not validate
RF, SPI timing, storage power-loss behavior, flight detection or physical wiring.
The attitude test checks the estimator's math and phase logic only; the
achieved 100 Hz rate and non-blocking radio TX have been built but not yet
measured on hardware (read the 10 s `attitude:` log line).

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
5. Keep command bridging separate from local validation and actuation. External-device
   commands go to Iris over CAN (`can_link.cpp`); UART1 belongs to the GPS only.
6. Treat missing radio as nonfatal, but do not assume missing IMU is nonfatal.
7. Update `GROUND_STATION_PROTOCOL.md` with `comms.h` changes, and deliberately
   synchronize the workbook when requested. Do not revive obsolete LoRa/CAN
   compression distinctions or packet IDs from conversational history.
8. Inspect the worktree and preserve unrelated changes. Generated build outputs
   and temporary PDF tooling have appeared in repository status; do not clean,
   reset or delete them as a side effect of firmware work.



