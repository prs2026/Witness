# Bench report 2026-11-17b (run 11:30-13:35 PST): GPS power-on, barometer, core-1 load, radio, Wi-Fi switch, on the working tree over b2f6610

**Result:** the GPS power-on problem is fixed and the barometer problem is fixed at its root cause. Two real bugs
were found and fixed on the way. The core-1 overload is explained: **it is instruction-cache thrash, not arithmetic.**
Code-only fixes lower it by only 2-6 points. The real fixes are build-configuration options that need approval
(`-Os` plus a 32 KB instruction cache brings core 1 to about 31 %). The radio tests and the Wi-Fi maintenance switch
all work, with a few observations below.

**Image on the board at the end:** flight (flight_a, 474,064 bytes, default configuration: `-Og`, 160 MHz, 16 KB icache)
plus the unchanged 08:52 maintenance image (maint_a). Witness is in Pad Idle with the GPS at 25 Hz.

## Summary

| Test | Result | Evidence |
|---|---|---|
| A. Power-on GPS (5 hub power-on boots, 2 RTS resets) | PASS, with a 21.6-23.6 s delay | GPS identified and at 25 Hz in 5/5 power-on boots (build under test in boots 2-5), no `NO_MEM`. NVS bug found and fixed (see "Bugs"). |
| B. Barometer | PASS after a root-cause fix | The retry never helped (3/3 retries failed, ready at 1.30-1.40 s). Cause: the RA-01 chip select floats during the first PROM read. Fix: park all three SPI3 chip selects high first. Barometer ready at 269 ms, no PROM errors. |
| C. Core-1 load | CAUSE FOUND; target needs approval | Cache thrash. Code fixes: 95.7 % to 91-94 % of wall time. Options: `-Os` 82 %, 32 KB icache 47 %, both about 31 %. Details below. |
| D. 0x4252 over radio | PASS | UNKNOWN ack, Witness did not restart (2/2). |
| D. Command gating over radio in HITL flight | PASS | 21 of 23 acked REFUSED/UNKNOWN as specified; the other 2 acks were lost on the air but the Witness log shows them refused. |
| D. 0x686A confirm over radio | PASS | First send BUSY `0xC0F1`, second within 10 s OK, cleared in 37 ms. |
| D. 0x5250 over radio | PASS | OK / OK / OK, +9 dBm BAD_ARGUMENT; Coast/Descent rules held. |
| D. Aperture pacing, RSSI, health | PASS with observations | 0xF0 at 1 Hz, RSSI -28..-32 dBm. A 5-frame burst: Witness acted on 4, 3 acks reached USB. |
| D. HITL telemetry over radio, Horizon decoder | PASS | 757 packets in 200 s, 0 callback errors. |
| D. Wi-Fi maintenance switch (`0x5746`) | PASS with one bug | On/off/status over USB and radio, beacons, auto-off 62.4 s for 1 min, refusals in flight. The USB "on" ack never reaches the host (bug 3). |
| E. HITL nominal flight (final image) | PASS | Deltas 0.06 / 0.02 / 0.01 / 0.01 s. |
| E. 0x4252 warm reset in Coast (final image) | PASS | Ack 44 ms, resumed Coast, telemetry back at 0.58 s, barometer back at 343 ms (was 1.4 s). |
| Host tests | PASS (20 of 21 suites built and passed) | Details below. |

## A. Power-on GPS

Method: flash the flight image, then `uhubctl` power-on boots at least 4 min apart (5 used), then 2 RTS resets
through the flashing port with the console captured from t=0. Times are Witness uptime (from the log stamps).

| Boot | Build | Stored pin value used? | GPS identified | Pin / rate | 25 Hz confirmed | GGA | `NO_MEM` |
|---|---|---|---|---|---|---|---|
| Post-flash (hard reset) | race build | no (see bug 1) | yes, before capture | GPIO5, 88398 | before capture | 1493 per 60 s | 0 |
| Power-on 1 | race build | no: "stored" printed at 2.0 s | PX1120S at 115200 at 2.5 s, config failed 8.6 s, probe at 18.6 s, found at 88398 | GPIO5 (swapped), 88398 | 23.6 s | 25 Hz after that | 0 |
| Power-on 2 | NVS fix | yes (no "stored" line) | same sequence | GPIO5, 88398 | 21.6 s | 1500 per 60 s at 25 Hz | 0 |
| Power-on 3 | NVS fix | yes | same | GPIO5, 88398 | 21.6 s | 1500 per 60 s | 0 |
| Power-on 4 | NVS fix | yes | same | GPIO5, 88398 | 23.6 s | 1500 per 60 s | 0 |
| Power-on 5 | final image | yes | same (retries 1) | GPIO5, 88398 | by 35 s (GGA counter 374 at 35 s) | 25 Hz | 0 |
| RTS reset A | final image | "GPS pins from the last good session: swapped" at 180 ms | PX1120S at 88398 at 3.57 s | GPIO5, 88398 | 5.67 s | 673 per 30 s | 0 |
| RTS reset B | final image | same, 180 ms | 3.57 s | GPIO5, 88398 | 4.68 s | 562 per 25 s | 0 |

- Time to first GGA after a power-on: GGA sentences flow at 115200 (1 Hz) before the identification at 2.5 s, so the
  first GGA is earlier than 2.5 s uptime; 25 Hz GGA+RMC is confirmed after 21.6-23.6 s. The early boot lines
  (before about 2 s) are not captured on a power-on boot: the USB port is not enumerated yet.
- The old failure (GPS never identified, GGA count stuck at 1) did not occur in any power-on boot.
- Why it takes about 22 s. The receiver starts at 115200. Witness identifies it (2.5 s) and configures it. The readback
  then times out (8.6 s) because the receiver has moved to 88398 baud. The supervision waits `kProbeRetryMs` (10 s),
  probes again at 18.6 s, decodes NMEA on GPIO5 at 88398, re-identifies and confirms 25 Hz at 21.6-23.6 s. After an
  RTS reset the receiver is already at 88398 and the same sequence takes 5 s. I did not change this: the GPS works
  and the time is an option (shorter first retry after a failed readback, or no readback timeout when NMEA keeps
  flowing). The open question about why the receiver ends up at 88398 still stands.
- The no-NVS first boot was not exercised on hardware after the fix (clearing NVS would also erase the IMU
  calibration and the boot bookkeeping). The only no-NVS boots were race-affected (bug 1).

## B. Barometer

| Boot / build | PROM words logged | Retry result | Barometer ready |
|---|---|---|---|
| 7300 restart, retry build (11:30) | `0808 8C2C 8808 0800 00A0 8028 2828 0880` | 3/3 failed, `ESP_ERR_INVALID_CRC` | about 1.3 s (sensor-task retry) |
| 7300 restart, retry build (12:52) | `0818 8C2C 8808 0800 00A0 8028 2828 0880` | 3/3 failed | about 1.3 s |
| 0x4252 resume, retry build (13:05) | `0018 9010 8808 5800 50D0 8018 4848 5880` | 3/3 failed | 1.40 s |
| RTS reset A and B, final image | none | none needed | **269 ms** |
| 0x4252 resume, final image (13:24) | none | none needed | **343 ms** |

Cause: the words are garbage, not a timing problem (retries 5 ms apart failed the same way). In `Sensors::start()`
the three devices on SPI3 get their chip select driven only when each is added to the bus, and the RA-01 is added
last, after the barometer. GPIO16 (RA-01 NSS) floated through the first PROM read, so the RA-01 could drive MISO. The
sensor task's own retry about 1 s later worked because the radio had been added by then. Fix: configure the IMU, MS5607
and RA-01 chip selects as outputs, high, before `initialize_spi()`. The retry code stays as a safety net.

## C. Core-1 overload

### Cause

The measurements point to **instruction-cache thrash**, not CPU cycles:

1. Switching to 240 MHz changes nothing: core 1 stays at 100 % and every section keeps its share of wall time
   (hg apply 5.4 to 4.2 %, all others identical). Code that is CPU-bound would speed up by 1.5x.
2. A 32 KB instruction cache (`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`) takes core 1 from 99.4 % to 46.8 % **and core 0 from
   39 % to 14 %**. The two cores share one 16 KB icache, and the code is fetched from flash (DIO, 80 MHz, 32-byte lines).
   One pass of the 500 Hz loop touches the IMU path, SPI driver, attitude, position filter, state machine, packet
   queue, radio and logger code. That is more than 16 KB, so every 2 ms pass refetches most of it from flash.
3. `-Os` (smaller code) helps, `-O2` barely helps, IRAM placement of the SPI master functions does nothing.
4. The earlier measurements (28-45 %, 2026-11-08/09) were taken before the Kalman filter, the high-g calibration, the
   NAND log queue and the radio scheduling grew the per-pass footprint; the code size crossed what the cache holds.

There is also real, smaller per-sample cost in software double-precision arithmetic (about 100 cycles an operation).
The largest piece was `HighGCalibration::apply`, which ran the whole double-precision fit three times per axis per sample.

### Profile, before and after (real sensors, Pad Idle, quiet bench, 60 s, default configuration)

Percent of wall time per section (the `core-1 profile` line; 936 Hz, 500 passes/s).

| Section | Before (power-on 2, 11:53) | After code fixes (12:42) | Final image (13:27) |
|---|---|---|---|
| prep (calibration, high-g fit, filtered accel) | 17.7 | 12.9 | 12.5 |
| IMU log | 4.6 | 4.0 | 3.8 |
| attitude | 20.1 | 17.0 | 16.6 |
| predict (position filter) | 10.4 | 10.3 | 10.3 |
| baro | 2.9 | 3.3 | 3.2 |
| state + publish | 7.7 | 9.7 | 8.7 |
| position | 2.3 | 2.4 | 2.2 |
| packets | 1.6 | 2.0 | 1.9 |
| radio | 4.8 | 5.4 | 5.4 |
| FIFO read (separate `imu fifo` line) | 16.6-17.8 | 17.8 | not captured |
| Whole passes | 95.7 % (4604 per 10 s, 2081 us avg) | 94.1 % (4999, 1882 us) | 91.0 % (4999, 1820 us) |
| `cpu` line, core 1 / core 0 | 100 % / 38.5 % | 99.4 % / 41 % | 84-85 % / 27 % (RTS capture) |
| Loop overruns | 460 per s | 491 per s | not measured (attitude not initialized; see below) |

Sample processing (`prep + log + attitude + predict`) fell from 52.8 % to 44.8 % (the `imu fifo` line: 48.1 % to
45.0 %). The loop still overruns about 480 times a second: the average pass is 1.88 ms against a 2 ms period.
The 13:27 column is lower mainly because attitude was not initialized (bench vibration, see "Bugs"), so the
estimator skips its window work. It is not a like-for-like gain.

**HITL, final image, 60+ s:** core 1 17-18 %, core 0 18 % (`cpu` line). Passes 15-16 % (about 310-325 us). The 100 Hz
0x0C path processes about 9x fewer samples (and each costs more, because the code is cold between samples), which
is why HITL looks light.

### Fine timers (temporary, since removed), per sample at 936 Hz, default configuration

| Section | % of wall time | us per sample |
|---|---|---|
| high-g observe (double math) | 4.7 | 50 |
| high-g apply, before the fix | 9.3 | 99 |
| high-g apply, after the fix | 5.4 | 58 |
| filtered accel | 1.7 | 18 |
| attitude: median filter, before insertion sort | 3.9 | 42 |
| attitude: median filter, after insertion sort | 2.5 | 27 |
| attitude: rotation update (sin, cos, sqrt) | 4.1 | 44 |
| attitude: stationary-window sums (doubles) | 1.8 | 19 |
| position filter predict | 6.2 | 66 |
| IMU record build + queue | 1.2 + 2.3 | 13 + 25 |
| FIFO read (SPI + decode + timestamps) | 17 | 180 |

### Experiments (each flashed and measured 40-45 s, quiet bench, temporary timers in)

| Experiment | Core 1 | Core 0 | Overruns/s | Free heap | Verdict |
|---|---|---|---|---|---|
| Baseline (default config, code fixes) | 100 % | 39 % | 455 | 34.2 KB | |
| 240 MHz | 100 % | 40 % | 495 | 34.2 KB | no gain: not CPU-bound |
| `-O2` (`CONFIG_COMPILER_OPTIMIZATION_PERF`) | 92 % | 39 % | 385 | 38.3 KB (+4) | small gain; needs the `small_median` change below |
| `-Os` (`CONFIG_COMPILER_OPTIMIZATION_SIZE`) | 82 % | 33 % | 270 | 43.1 KB (+9) | best single option without RAM cost |
| 32 KB icache | 47 % | 14 % | about 12 | 17.8 KB (-16.4), minimum 14.5 KB | big gain, heap gets tight |
| **`-Os` + 32 KB icache** | **31 %** (passes 30.9 %, 617 us) | about 10 % | about 12 | about 27 KB (estimate: 43.1 - 16.4) | target met |
| `CONFIG_SPI_MASTER_IN_IRAM` | 100 % | 41 % | not taken | 34.2 KB | no gain |
| QIO flash at 80 MHz | does not boot | n/a | n/a | n/a | ruled out (see below) |

Recommendation for the reviewer: `-Os` plus the 32 KB icache meets the target with about 27 KB of heap left (minimum
free heap was 30.2 KB in the default build, so the free-heap margin drops by about 3 KB). Free internal RAM is small
because of the flash logger (73 KB static) and the sensors object (41 KB static). Trimming either would give back
the icache's 16 KB. `-Os` alone (82 %, no overruns solved) is the zero-cost step. **I left none of these in the
build.**

QIO result: building with `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y` and flashing the bootloader, partition table and app
in QIO at 80 MHz gives a boot loop in the ROM loader (`rst:0x7 TG0WDT_SYS_RST`, "mode:QIO", at `ets_loader.c 79`).
The embedded 8 MB XMC flash is reported as quad in the eFuse, but it does not work as QIO at 80 MHz here. I recovered
with the DIO bootloader, partition table and app (verified, boots).

## D. Radio and Wi-Fi

Radio path: Aperture on its own USB port to Witness over coax and attenuators. USB over-current events in the Pi
`dmesg` before and after every run: **0** (radio commands, acks and telemetry, Wi-Fi up for about 6 min).

### 0x4252, 0x5250, pacing, confirm (Pad Idle, `d1.py`, run twice: retry build and final image)

| Command over radio | Result (both runs) |
|---|---|
| 0x4252 d0=0 | UNKNOWN, Witness stays up (uptime continuous) |
| 0x5250 -17 (unchanged) | OK, detail `0x01ef` |
| 0x5250 -14 | OK, detail `0x01f2` |
| 0x5250 back to -17 | OK |
| 0x5250 +9 | BAD_ARGUMENT, detail `0x01ef` (log: "allowed -17..3; still -17 dBm") |
| 0x686A first | BUSY, detail `0xC0F1` |
| 0x686A second (1.7 s later) | OK; `all stored sessions cleared (headers only, 37 ms)` |
| 0x4849 HITL enter | REFUSED (HITL stayed off) |

- Pacing (5 distinct 0x4252 frames sent to Aperture at once, both runs the same): Witness logged 4 receipts (d0=0,1
  in the first burst, then d0=3,4 about 2 s later); the third and the fifth frames never reached it. USB showed 3
  acks (d0=1, 3, 4). The first-burst ack for d0=0 was replaced in the 0x06 latest-frame cache. This matches "3 per burst
  then 1.5 s", but with a half-duplex link the burst of 3 loses at least one frame.
- Ack latency 0.06-0.8 s (0.4 s before the build with the chip-select change, 0.07 s after).
- Aperture 0xF0 status (Horizon decoder, `ground_station_status`): 1 Hz (68 in about 67 s), RSSI -28 to -32 dBm,
  Aperture uptime continuous (no spontaneous restart), GPS fix quality 2 with 0-5 satellites.

### Gating over radio in a HITL nominal flight (`d2.py`, run D2c; USB streamer 0 late ticks)

| State | Command | Ack |
|---|---|---|
| Pad Idle | 0x4849 | REFUSED |
| Boost | 0x0030 CH1 off, 0x5250 -17, 0x4252 | REFUSED, REFUSED (detail `0x01ef`), UNKNOWN |
| Coast | 0x0030, 0x5250 front-end off, 0x686A, 0x6869, 0x5746 on, 0x4752, 0x4741, 0x7302, 0x6868 | REFUSED each |
| Coast | 0x0050 5 V off | ack lost on the air; Witness log: `radio load command 0x0050 refused in flight` |
| Coast | 0x4252 | UNKNOWN |
| Descent | 0x0030, 0x5250 front-end off, 0x6869, 0x5746 on | REFUSED each |
| Descent | 0x7301 | ack lost; log: `flight state 0x03 -> 0x01 over radio refused` |
| Descent | 0x5250 to -14 (raise), then -17 (lower) | OK (`0x01f2`), then REFUSED |
| Landed | 0x0030, 0x4858 (HITL exit), 0x4252 | OK, OK, UNKNOWN |
| Landed | 0x7300 | OK; `Landed -> Pad Idle over radio: restarting`, back in Pad Idle |

The first run (D2, 12:5x) gave the same answers where it got an ack. Two later runs with threaded senders overlapped
Aperture's pacing and are not used.

### HITL flight telemetry over the radio link

Aperture capture of D2c (24,174 bytes) decoded with Horizon `PacketDecoder`: 757 packets (sensors 159, state 158,
camera 188, ground status 201, command 26, ack 25), 0 callback errors, 2 "discarded non-packet bytes" notices (partial
frames), 2 bad frames. In flight the radio carried about 0.8 sensor packets and 0.8 state packets per second.

### Wi-Fi maintenance switch, `0x5746` (`w1.py`; coax; AP never joined)

| Step | Result |
|---|---|
| Status in the flight image over USB (d0=2) | OK, detail `0x0000` |
| Status in the flight image over radio | no ack (1 try) |
| On over USB (d0=1, 2 dBm, 5 min) | restarted into maintenance (`boot_mgr: next boot: maintenance slot maint_a`; restart after 0.4 s). **The ack was not seen on USB.** Same on the second USB on. |
| Status in maintenance over radio and USB | OK, detail `0x010c` (bit 8 = AP up, start step `0x0c`) |
| 0x08 beacon | 25 beacons, every 5 s, identical on USB and on the radio link. First beacon flags `0x20` (radio ok, AP not up), later `0x25` (AP up, store mounted, 10 log files, 5.4 V). Maintenance and flight image IDs equal (25272), store version 4. |
| Off over radio (d0=0) | ack OK 0.07 s; `leaving maintenance (Wi-Fi off over radio): restarting in 700 ms`; flight back, Pad Idle |
| On over radio (d0=1, 2 dBm, 1 min) | ack OK (2.0 s) |
| Auto-off | `leaving maintenance (Wi-Fi auto-off, no HTTP activity): restarting in 200 ms`; flight telemetry was back **62.4 s** after the on-ack |
| Off over USB (d0=0) | ack OK; `leaving maintenance (Wi-Fi off over USB): restarting in 300 ms`; flight back |
| Refused outside Pad Idle / Landed (radio) | REFUSED in Coast and in Descent |
| Refused outside Pad Idle / Landed (USB) | not run: no USB flight session this time (see Not run) |
| AP visibility | not recorded: the desktop's read-only `netsh wlan show networks` listed only an unrelated network before, during and after the AP was up. No desktop setting changed; the desktop never connected. |
| OTA upload | not run (as instructed) |

## E. Regression on the final image

- **B, nominal 10 g:** PASS, 15,819 frames, 0 late ticks. Boost 10.36 (+0.06), Coast 12.51 (+0.02), Descent 32.39 (+0.01),
  Landed 148.20 (+0.01). Error against truth: up 2.0 / 0.8 / 0.2 m RMS (Boost / Coast / Descent), horizontal
  3.0 / 7.4 / 13.5 m (Coast / Descent / Landed), the same numbers as 2026-11-17.
- **E, 0x4252 SW warm reset in Coast:** ack OK after 44 ms; `RESUMING IN-FLIGHT STATE 2 after reset reason 3 (max 349.1 m)`
  at 147 ms; `RESUMED ... appending to flight session 73 in slot B` at 248 ms; first telemetry 0.58 s after the command;
  barometer ready at 343 ms (no PROM errors); `GPS pins from the last good session: swapped` at about 180 ms; Coast then
  ran into Descent and Landed with the bench barometer (the known bench artifact). HITL off gave REFUSED.

## Code changes

All uncommitted, in `Software/Witness/softwarev1`:

| File | Change | Reason |
|---|---|---|
| `main/main.cpp` | The NVS init block (`imu_calibration::init_storage`, `maintenance_mode::cancel`) now runs before `gps_uart.start()`. | The GPS task read its stored pin orientation before NVS was initialized and never saw it (bug 1). |
| `main/sensors.cpp` | `Sensors::start()` parks the IMU (35), MS5607 (21) and RA-01 (16) chip selects high before `initialize_spi()`. The `core-1 profile` line is kept (about ten `esp_timer_get_time` calls per pass). | Root cause of the MS5607 PROM CRC failures (B). |
| `main/accel_filter.cpp` | `HighGCalibration::apply` fits each axis once instead of calling `scale()` and `offset_mg()` (three fits per axis per sample). Same expressions, so bit-identical. | Per-sample software double arithmetic, 99 to 58 us. |
| `tests/accel_filter_test.cpp` | New `test_apply_matches_definition`: `apply` equals `(h - offset_mg) / scale` bit for bit, before data, on the pad (no spread), and after a flight fit. | Guards the change above. |
| `main/attitude_estimator.cpp` | `small_median` uses an insertion sort (at most 5 values) and clamps the count. Same median value. | 42 to 27 us per sample, and GCC `-O2` reported a false `-Werror=array-bounds` from `std::sort` that stopped an `-O2` build. |
| (earlier work, kept) `main/uart_driver.cpp`, `main/gps_receiver.cpp`, `main/ms5607.cpp` | Verified on hardware (A, B). The PROM retry in `ms5607.cpp` stays; it never fires now. | |

Nothing was changed in `comms.h` (the four copies are still byte-identical, checked with `cmp`).

Host tests: built and passed with `g++`: accel_filter (new test), attitude_estimator, command_policy, flight_state_machine,
gps_assist, gps_receiver, heading_align (with `attitude_estimator.cpp`), hitl_input, imu_calibration, imu_supervisor,
iris_load_tracker, landed_logic, log_slot_logic, nmea_gps, position_filter, px1120s_gps, skytraq_i2c_mailbox,
tx_slot, wifi_ota_logic, image_info (exit 0, no output). Not built: `imu_calibration_store_test` (needs the FreeRTOS headers;
not available on the host). No build error or warning from my changes.

## Bugs found

1. **GPS pin orientation in NVS was never loaded (fixed).** `UartDriver::load_good_pins()` ran in the GPS task
   before `imu_calibration::init_storage()` in `app_main`. Every boot printed `GPS pin orientation (swapped) stored`
   and none printed `GPS pins from the last good session`. Fixed by moving the NVS init before `gps_uart.start()`;
   afterwards `GPS pins from the last good session: swapped` appears at about 180 ms on every boot and "stored" no longer does.
   Impact before the fix: the pin memory added in the earlier GPS fix did nothing (the decode check alone still found
   GPIO5).
2. **MS5607 first PROM read fails at every boot (fixed).** Floating RA-01 chip select; see B.
3. **No USB ack for `0x5746` "on" (not fixed, medium).** Two of two USB "on" commands restarted into maintenance, but
   no ack frame reached the host (the radio "on" ack did arrive). The firmware reference says the ack comes first
   and the restart 0.4 s later. The host cannot tell "accepted" from "lost". Check the USB flush before
   `maintenance_mode::restart_after_ms` for this path.
4. **`-O2` does not build (fixed as a side effect).** `std::sort` in `small_median` produced `-Werror=array-bounds`.
5. **Over radio, acks of consecutive commands can overwrite each other** (observed, not fixed, low). The ack goes
   through the per-ID latest-frame cache: in a burst of 3-5 commands only the last ack of the burst is transmitted,
   and 2 of 23 in-flight acks were lost. The command is still executed. One command at a time is reliable.
6. **Bench vibration from a 3D printer (not firmware, not a board fault).** The owner confirmed that a 3D printer
   running on the same bench causes it. From about 12:55 the IMU noise rose from 0.6 mg to 7-9 mg, then to 25 mg
   (13:11) and stayed there after a full power cycle. With 25 mg the stationary-pad window never completes, so the
   attitude estimator reports `windows 0 ok` and `not initialized` (the Pad Idle profile at 13:11-13:18 is not
   comparable). It had cleared again by 13:27, when the printer was presumably idle. This is expected behaviour of the
   stationary-window check and is not a regression.
7. **Headless HITL runs power-cycle the hub by default** (tool behaviour): the first `hitl_gui.py --headless` run
   fetched the logs and restarted the Pi USB ports. `--no-fetch-logs` avoids that; I used it for the final regression.
8. **PX1120S power-on timing** (see A): about 22 s to 25 Hz, driven by the 10 s retry delay after the first config
   readback fails.

## Measurements

| Item | Value |
|---|---|
| Witness heap (default config) | free 34.2-34.4 KB, minimum 30.2-30.7 KB |
| Flight image size | 474,064 bytes (0x73bd0 with the chip-select fix, `-Og`); `-Os` 426,144; `-O2` 469,312 |
| GPS | 25 Hz GGA+RMC, 1500 GGA per 60 s; `GPS NMEA decoded on GPIO5 at 88398 baud` at about 1.0 s on a reset |
| Barometer ready | 269 ms (RTS), 343 ms (warm reset in flight), was 1.30-1.40 s |
| IMU | 936.3-936.7 Hz, FIFO overruns 0, max backlog 12-24 words, max pass 3-10 ms |
| HITL, final image | core 1 17-18 %, core 0 18 %; passes 15-16 % |
| Radio ack latency | 0.06-2.1 s |
| Aperture RSSI | -28 to -32 dBm with Witness at -17 dBm |
| Wi-Fi switch | on to maintenance about 0.4 s after the command (USB), auto-off 62.4 s for 1 min |
| USB over-current events (Pi `dmesg`) | 0 before and after every radio run |

## Not run and why

- **Wi-Fi OTA upload:** not asked for.
- **Wi-Fi AP name and visibility:** the desktop could not see the AP without changing its Wi-Fi settings; nothing was changed.
- **`0x5746` refusal over USB in flight:** the regression flight used no USB `0x5746`; the radio refusals in Coast and
  Descent ran. The USB gating of `0x5746` is the same `command_policy` rule (checked in the previous report for the
  other commands).
- **Boot with an empty GPS-pin NVS value:** would require erasing NVS.
- **Power-on boot lines before 2 s:** the USB port is not enumerated yet; only the flash text log would show them (the
  log store starts after the logger is installed, so the early GPS lines are missing from it too).
- **`-Os`/icache on HITL and on radio tests:** the final configuration was not changed, so the radio, Wi-Fi and HITL
  tests ran on the default configuration only.
- **`imu_calibration_store_test`:** needs FreeRTOS headers.
- **Instruction-cache placement of hot code in IRAM (instead of a bigger cache):** not tried. The hot per-sample
  functions are only about 10 KB, but the working set of a pass is wider than that, and an IRAM region costs the
  same internal RAM as the icache option.

## State the bench was left in

- Witness: flight image (flight_a) from the final tree, default configuration, plus the unchanged 08:52 maintenance
  image in maint_a. The bootloader and partition table were re-flashed from the default (DIO) build after the QIO
  experiment. Witness is in Pad Idle, GPS identified and at 25 Hz, HITL off, attitude initialized. Iris loads all off,
  Aperture running, `bench_ports`: Witness, Iris, Aperture on the usual ports.
- Log store: cleared (`0x686A`) at 13:27; the current session is a Pad Idle session. Everything cleared was test
  data from this session and from the stopped run before it (nothing was downloaded before clearing).
- Radio power: set over the radio to -14 and back to -17 in Pad Idle, so NVS now holds an explicit -17 dBm
  (the same as the compiled default; it was "nothing stored" before).
- No HITL session, streamer or other process of mine is running, and no ser2net port is held.
- **uhubctl cycles used: 6.** Five deliberate power-on boots (11:47, 11:53, 11:57, 12:01 and 13:17) and one by the
  HITL runner's log fetch at 13:04. Port order was unchanged after each. RTS resets: 2 for the GPS test (one failed to
  connect and was repeated), plus resets by the flasher.
- Scratch, in `%TEMP%\wbench3`: `cap.py`, `rst_cap.py` (captures), `d1.py`, `d2.py`, `w1.py` (radio and Wi-Fi tests),
  `hz.py` (Horizon decode), `bf.ps1`, `bx.ps1` (builds) and all `*_witness.log/.raw`, `*_aperture.log/.raw`. Experiment
  build directories `wfx_*` and a source snapshot `wtree` are in `%TEMP%`; they can be deleted.
