# Bench report 2026-11-17 (run 08:53-09:58 PST): working-tree fixes on b2f6610, with the bench-restart command

**Result: the fixes work on hardware in every test that ran, and one serious bug was found.** All three boards were
flashed with images built from the current working tree. Witness got both images: flight in `flight_a` and
maintenance in `maint_a`. The HITL flights, the six warm resets in flight (W2/W3), the one-way states, never-erase,
command gating and the full erase all behaved as designed.

**Bug found (new, high severity):** after a power-on boot, the Witness GPS receiver is never identified, so there is no
GPS at all (bug 1). A reset that does not cut power brings it back. The bench boards are powered from USB, so a
real flight that starts from power-on is the case that fails.

## Summary

| Test | Result | Evidence |
|---|---|---|
| Board identification | PASS | `bench_ports`: 7100 Witness, 7101 Iris, 7102 Aperture. The order was the same after every hub cycle. 7103/ttyACM3 has no board. |
| Flash (Aperture, Iris, Witness) | PASS | `flash_verified.ps1 -CheckOnly` was OK on all three. esptool at 115200 baud, every hash verified. Witness: bootloader, partition table, `ota_data_initial`, flight at 0x20000, maintenance at 0x420000. It boots `flight slot 0 (b2f6610-dirty)`. |
| A. Smoke (boot, Pad Idle, stack/heap, reset reason) | PASS (see bug 2) | Boots into Pad Idle. Reset reasons are reported correctly (USB, POWERON, SW, PANIC). Free heap 34.5 KB (minimum 30.8 KB). Stack margins are listed under Measurements. The MS5607 PROM CRC check fails on the first read at every boot and recovers about 1 s later. |
| B. HITL nominal flight | PASS | Nominal 10 g: each transition 0.02 s after its prediction. 15,819 frames sent, 0 late ticks. |
| C. One-way states, 0x7300, second flight | PASS | Downward state commands are REFUSED in Coast, Descent and Landed. 0x7304 is OK in Descent. In Landed, 0x7300 restarts the board (SW) into Pad Idle, not resumed. A second flight passes (deltas 0.01-0.03 s). |
| E. Warm reset in flight (0x4252, SW and PANIC, in Boost, Coast and Descent) | PASS (6/6) | Every run resumed the same state, then only moved up. The log appends to the same session with a source-6 record. Telemetry is back 0.58-0.60 s after the command. Iris did not reset. |
| E. Refusals of 0x4252 | PASS (USB) / NOT RUN (radio) | HITL off: REFUSED (6/6). d0=0x02: BAD_ARGUMENT. Radio: not run, because the RF path is not confirmed closed. |
| E. Cold reset (RTS through 7000) | PASS (as designed) | Reset reason USB, so the boot is Cold: Pad Idle, resumed bit clear. |
| D. Launch-detection robustness | PASS (firmware); HITL predictor gap (bug 3) | Nominal: 10.32 s. 2 g net motor: 10.38 s (acceleration rule). 1 g net motor: 13.48 s (Kalman rule). Slow climb: no transition. Extra 25 m/s smooth climb: Boost at 31.22 s by the barometer-rate rule, which the HITL predictor does not model. |
| F. Logging robustness (never-erase, 0x686A USB) | PASS (part) | Slots full: the second flight logged nothing, nothing was erased, and the LOG_SLOTS_FULL bit stayed set (3489/3489 packets). 0x686A over USB: one command, OK, about 37 ms, and logging resumed. Radio double-confirm, power loss and NAND fault injection: NOT RUN. |
| G. Command gating (USB, in flight) | PASS | 13/13 refusals as specified. In Landed, CH1 off is accepted. |
| H. Watchdog and timing | PASS (part) | A full erase (0x6869) takes 4.27 s with no watchdog reset, and telemetry kept its 1 Hz state packet. Maintenance switch and calibration capture: NOT RUN. |
| I. Iris | PASS (part) | No Iris reset during any Witness reset (uptime continuous). Main stack: 5444-5524 B free of 6144. Load restore and reset-pin hold: NOT RUN. |
| J. Aperture | PASS (passive only) | Receives and forwards Witness telemetry. RSSI -29 dBm. 0xF0 status at 1 Hz. No spontaneous reset. Uplink latency and pacing: NOT RUN (radio). |
| K3. Decode HITL captures with Horizon's decoder | PASS | Horizon `PacketDecoder` and the HITL `StreamDecoder` agree packet for packet on 3 captures (warm reset in Coast and in Boost, and test C). 0 callback errors. No 0x0C echo on the stream any more. |
| K1/K2. Host unit suites | NOT RUN | Not repeated today. They passed on 2026-11-16. |

## Per-test details

### Flash
- `flash_verified.ps1 -CheckOnly`: 7000 is an ESP32-S3 running `softwarev1` (flight image). 7002 is an ESP32-S3
  running `softwareattempt2`. 7001 is an ESP32-C3 running `softwarev1` (Iris).
- `flash_verified.ps1` flashes at 460800 baud, but the rule is 115200 only. So I ran esptool directly at 115200 with
  the same checks. Witness images came from the flight and maintenance build directories, built 08:52. Iris and
  Aperture images were built 11-16 11:44. Since then, only `comms.h` has changed in their trees: comments and the
  new `0x4252` defines, which they do not use. The `comms.h` copies are still byte-identical across Witness, Iris,
  Aperture and Horizon.

### A. Smoke
- Cold boot (RTS) and power-on boot both reach Pad Idle. The boot log shows the startup self-test,
  `session N in slot X (store v4)`, `radio TX power -17 dBm (compiled default, nothing stored)` (W1: nothing was
  stored, so the default applies) and `selftest: running flight slot 0`.
- With real sensors: core 1 is 96-100 % busy (the sensors task), and core 0 is 35-40 %. In HITL mode core 1 drops to
  about 19 %. Attitude runs at 936 Hz, IMU FIFO overruns 0, maximum sensor pass 7-11 ms.
- **MS5607:** `PROM CRC mismatch` followed by `MS5607 initialization failed: ESP_ERR_INVALID_CRC` at every boot
  (power-on, RTS, SW and PANIC: 10 of 10 boots seen), then `MS5607 initialization recovered` at about 1.3 s. See bug 2.
- IMU-temperature-ready status bit: with real sensors it is set in only about 1.5 % of 0x01/0x02/0xFF packets. In
  HITL it is always set. Not investigated further.

### B. HITL nominal flight (`runs/20261117-090727`)
| State | Expected s | Witness s | Delta |
|---|---|---|---|
| Boost | 10.30 | 10.32 | 0.02 |
| Coast | 12.49 | 12.51 | 0.02 |
| Descent | 32.38 | 32.40 | 0.02 |
| Landed | 148.19 | 148.21 | 0.02 |

- Boost came from the acceleration rule (`launch detected: axial acceleration ...`). The 2 s of pre-launch records
  were queued: 100 sensor records and 1920 IMU records.
- Error of the 0xE0 estimate against truth (RMS): up 1.7 / 0.9 / 0.2 m in Boost / Coast / Descent; horizontal
  7.3 m in Descent and 13.4 m in Landed. This preset is purely vertical; see the observations.
- The flash log was exported: 2.0 MB of packets, 8173 Kalman (0xE2) rows. The flash text log holds the full boot.

### C. One-way states, 0x7300, second flight
- During a HITL flight, over USB: 0x7300 and 0x7301 in Coast were REFUSED. 0x7302 and 0x7300 in Descent were REFUSED.
  0x7304 in Descent was OK (moved to Landed). 0x7303 in Landed was REFUSED. The log lines say
  `flight state 0x02 -> 0x00 over USB refused`, and so on.
- 0x7300 in Landed: ack OK, `Landed -> Pad Idle over USB: restarting (flight states are one-way)`. The board
  restarted 0.4 s later (`rst:0xc RTC_SW_CPU_RST`), came back in Pad Idle with the resumed bit clear, and reported
  `last reset SW`. The USB link did not drop.
- Second flight (`runs/20261117-091400`): PASS, deltas 0.01-0.03 s. Both slots held flights at that point. Witness
  logged `both log slots hold a flight: NOT LOGGING (nothing is erased)` and set LOG_SLOTS_FULL in all 3489
  telemetry packets. No pre-launch records were queued. This is the never-erase rule working (F).

### E. Reset in flight
Method: a custom streamer (`test_e.py`) ran the Nominal 10 g preset. It sent `05 d0 00 00 00 00 00 42 52 0A` over
USB. In Boost it sent it 0.4 s after the state was seen; in Coast, 3 s after; in Descent, 5 s after. It stopped
streaming at the ack and watched the board for 75-90 s. Each run first sent 0x4252 with HITL off. The log slots
were cleared (0x686A) between pairs of runs.

| Run | State, mode | Ack (ms after send) | Reset reason after boot | Resumed state | Next states (s after command) | Log append | Restored filter (truth) |
|---|---|---|---|---|---|---|---|
| E1 | Coast, SW | 49 | SW (3) | Coast | Descent 2.5, Landed 31.9 | session 871 slot B | AGL 897.5 m, vz 143.1 (897.4, 143.2) |
| E2 | Boost, PANIC | 55 | PANIC (4) | Boost | Coast 1.1, Descent 2.3, Landed 32.1 | session 872 slot A | 39.8 m, 99.6 (48.3, 109.2) |
| E3 | Descent, SW | 58 | SW | Descent | Landed 31.6 | session 874 slot A | 1435.9 m, -25.2 (1431.1, -25.0) |
| E4 | Descent, PANIC | 55 | PANIC | Descent | Landed 31.6 | session 875 slot B | 1435.2 m, -25.2 |
| E5 | Coast, PANIC | 57 | PANIC | Coast | Descent 2.5, Landed 31.9 | session 877 slot B | 886.6 m, 144.8 |
| E6 | Boost, SW | 45 | SW | Boost | Coast 1.1, Descent 2.3, Landed 32.1 | session 878 slot A | 41.8 m, 102.3 |

- The board never came back in a lower state, and the resumed bit was set in every packet after the reset. Every
  run showed `RESUMING IN-FLIGHT STATE n after reset reason r` at 146 ms uptime and
  `RESET IN FLIGHT: resumed state ...` at 147 ms. `RESUMED after an in-flight reset: appending to flight session N in
  slot X` came at 247-249 ms, and `position filter restored after a reset in flight ... (no attitude: baro/GPS
  only)` at about 369 ms. The panic runs print `abort() was called` and a backtrace first.
- The restored AGL and vertical speed are the 2 Hz RTC snapshot. They match the truth at reset time to within the
  snapshot age (up to 0.5 s; E2 is about 0.1 s old).
- Telemetry: the first packet arrived 0.577-0.596 s after the command (Witness uptime 325-327 ms). The USB link did
  not drop on either reset type (USB-Serial-JTAG stays enumerated). In 90 s after E1: 895 sensor packets (0x01),
  89 state (0x02), 89 heartbeat (0xFF) and 90 camera (0x03). Radio telemetry resumed at 3 packets/s (flight rate).
- After the reset, HITL is off and the real sensors are used. What the state machine then does (bench artifact,
  expected): the restored references belong to the simulated pad at 609 m MSL, and the bench barometer reads about
  195 m. So AGL is about -418 m:
  - Coast and Boost runs: `descent detected (Kalman AGL): AGL -418 m` after 2.3-2.5 s.
  - Boost runs first: `burnout detected: axial acceleration 0.00 g` at 0.88 s. The board lies flat (tilt 89.8 deg),
    so body +X reads about 0 g.
  - All runs: `landing detected` after 30 s stationary.
  - Attitude stays `not initialized` (by design: the attitude is not resumed).
- Flash logs, exported and converted with Horizon's converter for E1, E2, E5 and E6 (`logs_E12`, `logs_E56`): each
  session has exactly one `log_resumed` record (`log_source` log_marker = packet source 6). Its payload is
  `01 <state> <generation>`, for example `01 02 00 00 03 67` = Coast, session 871. State packets continue after it
  (E1: Pad 1495 / Boost 109 / Coast 146 before; Coast 100 / Descent 1469 / Landed 5309 after). The text log of the
  same session holds both boots. The converter marks the gap as `1.000 s (ESTIMATED, real reset duration unknown)`.
  E3/E4 were checked live only; their flash sessions were cleared before export.
- **Data lost before the reset:** the last 0.1-0.5 s before the reset is not in flash. E2 (PANIC, 0.58 s into
  Boost) has 5 Boost state packets where about 29 are expected. E1 (SW) has 146 Coast packets where about 155 are
  expected. In both, the `bench restart ...` log line is missing from the flash text log. This is expected with
  the 1 s flush interval and no shutdown flush. See the observations.
- Iris: its uptime ran on through every Witness reset (for example 366062 ms before E1, then 367062, 377062 ms
  after), and its status stayed 0x0000. All loads were off for the whole session, so "outputs unaffected" means only
  "no Iris reset and no status change". Restoring a load that is on was not tested (see Not run).
- Refusals: HITL off gave REFUSED in 6/6 runs (`bench restart refused: HITL not active`). d0=0x02 with HITL active
  gave BAD_ARGUMENT. Over radio: not run.
- **Cold case:** HITL flight, 3 s into Coast. I closed the raw port, toggled RTS on 7000 and reopened the raw port
  (1.25 s from close to reset). Witness came back with `last reset USB`, which is Cold: Pad Idle, resumed bit clear,
  as designed. The airborne-at-boot barometer rule cannot catch it on the bench, because the board is stationary.
  ser2net does not let 7000 and 7100 be open at the same time, so the boot lines before about 1.4 s were not seen
  live.

### D. Launch-detection robustness
| Scenario | Predicted (dry run 11-16) | Witness | Rule (Witness log) |
|---|---|---|---|
| Nominal 10 g | B 10.30, C 12.49, D 32.38, L 148.19 | 10.32 / 12.51 / 32.40 / 148.21 | acceleration |
| 2 g net (`thrust_accel_g=3, burn_time_s=6`) | 10.34 / 16.48 / 28.63 / 111.77 | 10.38 / 16.49 / 28.65 / 111.78 | `axial acceleration 3.00 g (>= 2.5 g for 300 ms)` |
| 1 g net (`thrust_accel_g=2, burn_time_s=8`) | 13.44 / 18.47 / 27.45 / 100.57 | 13.48 / 18.48 / 27.46 / 100.58 | `Kalman AGL 57.2 m, vertical velocity 33.1 m/s` |
| Slow climb, 10 m/s to 150 m | none | none (Pad Idle all run) | n/a |
| New: smooth climb at 25 m/s to 550 m (`rate_climb.csv`, 0.2 g ramps, `imu_synthesis=false`) | none (predictor) | Boost 31.22, Coast 42.46, Descent 69.66, Landed 132.00 | barometer rate (the only rule that can fire: speed under 30 m/s, acceleration under 1.3 g) |

- The 25 m/s scenario reached Boost at CSV 11.2 s. The speed passes 20 m/s at 9.6 s; add the 1 s dwell and the
  rate-filter lag. The firmware rule works. The HITL predictor does not model this rule (bug 3), so the headless run
  reports FAIL with 4 "UNEXPECTED" transitions.
- During the slow climb the Kalman AGL stays at 0 in Pad Idle (pad hold): up error up to 150 m. This is by design,
  but it means a slow ascent with little acceleration is only caught by the barometer-rate rule, and only at 20 m/s
  or more.
- D2-D4 ran with the saved 10 s tolerance; every delta was 0.04 s or less.

### F. Logging robustness
- Never-erase: see C. Both slots held flights, so Witness logged nothing, set the status bit and erased nothing.
- 0x686A over USB in Landed and in Pad Idle: one command is enough. The ack is OK after 0.14 s, followed by
  `all stored sessions cleared (headers only, 35-38 ms); new session N` and `log store freed: logging resumed`. The
  slots-full bit cleared within 6 s. Session numbers keep counting up after a clear.
- 0x6869 (full erase, H): `all stored sessions erased (4268 ms); new session 1 created`. Session numbers restart at 1
  after a full erase.

### G. Command gating (USB, HITL flight)
| State | Command | Result |
|---|---|---|
| Boost | 0x5250 Witness -17 dBm (unchanged) | REFUSED |
| Boost | 0x0030 CH1 off | REFUSED (`not forwarded`) |
| Coast | 0x0030 CH1 off, 0x0050 5 V off | REFUSED, REFUSED |
| Coast | 0x5250 front end off | REFUSED (`still -17 dBm`) |
| Coast | 0x6868 export, 0x686A clear | REFUSED, REFUSED |
| Coast | 0x4752 GPS cold start, 0x4741 assist BEGIN | REFUSED, REFUSED |
| Descent | 0x0030 CH1 off, 0x5250 front end off, 0x6869 erase | REFUSED x3 |
| Landed | 0x0030 CH1 off | OK |

Wi-Fi (0x5746) was not sent in flight: if gating failed, Witness would turn on a Wi-Fi access point and radiate
over the air.

### H. Watchdog and timing
- Full erase over USB in Landed: ack after 4.34 s (the USB task waits for the erase), erase 4268 ms, no reset. The
  0x02 state packets kept a 1.0 s maximum gap. `packet log queue full: 202 packets dropped` during the erase (the
  logger is busy).
- Reset reasons seen over the session: SW 84 lines, USB 39, POWERON 10, PANIC 19. No TASK_WDT, no INT_WDT, no Guru
  Meditation except the 2 deliberate aborts.
- HITL sender: 0 late ticks in every run.

### I. Iris
- No Iris reset in 6 Witness warm resets, 1 cold reset, the 0x7300 restarts or the erase. Status 0x0000 (all loads
  off) the whole time, and currents 0 A.
- `main: main stack high-water: 5444-5524 B free of 6144` (I3).

### J. Aperture (passive)
- Forwards Witness 0x01/0x02/0x03 frames. 0xF0 status at 1 Hz with RSSI -29 dBm and no GPS fix. Its uptime matched
  the last hub cycle: no spontaneous restart (A1/A2 watchdog and restart did not fire).

### K3. Horizon decoder on HITL captures
| Capture | Bytes | Horizon `PacketDecoder` | HITL `StreamDecoder` |
|---|---|---|---|
| E1 (warm SW reset in Coast) | 481,557 | sensors 1075, state 107, camera 108, heartbeat 107, command 4, ack 4 | identical |
| E2 (PANIC in Boost) | 460,860 | 1028 / 102 / 104 / 102 / 3 / 3 | identical |
| C (state commands, 0x7300) | 285,917 | 627 / 62 / 63 / 63 / 9 / 9, transfer status 1 | identical |

Callback errors: 0, undecoded: 0. The skipped garbage is the GPS UART noise of bug 1 and the ESP log text. Witness
no longer echoes 0x0C frames (0 host echoes), so the decoder's 0x0C deframing was not exercised. A capture with 0x0E
frames (960 Hz synthesis) was not decoded.

## Bugs found

### 1. GPS never identified after a power-on boot (Witness `uart_driver.cpp` / `gps_receiver.cpp`). High.
**Evidence.** After every hub power cycle (4 of 4), the GGA count stays at 1 for the whole capture: 200 s in the
dedicated capture from 09:53. The supervision loops every 20 s:
1. `GPS not identified: probing and identifying again (retry n)`
2. `GPS RX probe on GPIO4 (TX GPIO5): 1023 edges ... about 88398 baud` and `... on GPIO5 (TX GPIO4): 1023 edges ...
   about 88398 baud`
3. `GPS RX bit rate 88398 baud matches no standard rate; trying it first`
4. `receiver unknown GPS configuring, pins as wired` at 88403 baud, with `idle_ms` about 4-6 s (no bytes on that pin)
5. `GPS: listening on RX GPIO5` and `host rate now 115200 baud`
6. `unknown GPS startup failed: no GPS replied at any scanned baud rate on either RX/TX orientation`

After a reset that does not cut power (RTS, SW, PANIC), the boot probe sees `GPIO4: 0 edges` and `GPIO5: 88398 baud`
and chooses swapped pins. It then reports `GPS identified as PX1120S at 88398 baud` and `... 25 Hz GGA+RMC confirmed`
within 9.6 s; this happened at every such boot.

**Cause (from the log and the code).**
- `select_gps_pins()` sets `swap` only when GPIO4 has no edges and GPIO5 has edges. After a power-on, both pins show
  1023 edges at the same rate. GPIO4 probably picks up crosstalk while it is released (`listen_on` resets both
  pins). So Witness listens "as wired", on the wrong pin.
- `have_good_pins_` is false because no GGA has been parsed since boot, so the last-good fallback does not apply.
- When identification then swaps to GPIO5, it goes back to 115200 and the standard scan list. The measured 88398 is
  not tried on that orientation.

**Open question for the owner.** At 5.1 s after the power-on, the receiver was heard at 115200 (`receiver PX1120S
configuring, pins swapped`). It failed with `PX1120S configuration readback timed out` at 8.6 s, and from then on it
runs at 88.4 kbaud. On boots without a power cycle it is always found at 88398. This suggests the configuration sent
at power-on leaves the receiver at a non-standard rate. Check what the PX1120S setup sends, since a real power-on
sees the same sequence.

**Impact.** The flight power-on sequence (Witness and receiver power up together) gives no GPS for the whole flight.
The garbage at the wrong rate also floods the USB forwarder: `could not forward 64 UART bytes: ESP_ERR_NO_MEM`, and
once `sensors: sensor packet queue failed: ESP_ERR_NO_MEM`.

**Suggested fix.**
- When both pins show traffic, prefer the orientation that worked last: store it, with the working rate, in NVS or
  RTC memory. Or try both orientations at the measured rate before falling back to the scan.
- Keep the measured rate in the scan list across the orientation swap.
- Find out why the receiver ends up at 88.4 kbaud after the power-on configuration. Add a host test with "both pins
  active, measured 88398".

### 2. MS5607 PROM CRC fails on the first read at every boot (`ms5607.cpp`, `sensors.cpp`). Low to medium.
**Evidence.** `E (259) ms5607: PROM CRC mismatch: stored=1 calculated=12` (the values vary), then `MS5607
initialization failed: ESP_ERR_INVALID_CRC`, then `MS5607 initialization recovered` at 1.30-1.37 s. This happened at
every boot seen: power-on, RTS, SW and PANIC.

**Impact.** After a reset in flight, the barometer is missing for about 1 s. In W3 the barometer is one of the only
two sources the resumed filter has, and the GPS is also absent for about 9 s (bug 1 makes that permanent after a
power loss).

**Suggested fix.** In `Ms5607::initialize`, on a CRC mismatch, re-send the reset, wait at least 3 ms and re-read the
PROM up to 3 times before failing. Log the 8 PROM words on the first failure to tell an SPI problem (the shared bus at
boot) from a timing problem.

### 3. HITL reference model ignores the barometer-rate launch rule (`Software/HITL/sim_model.py`). Tool bug, medium.
**Evidence.** `_feed()` calls `update_from_sample()`, which never passes `baro_rate_mps`, so `ReferenceStateMachine`
rule 3 (`AIRBORNE_RATE_MPS`) can never fire in a prediction. The 25 m/s smooth climb is predicted as
"no transitions", but the firmware correctly enters Boost at 31.22 s (D5), and the run is reported as FAIL. The
README says the reference uses all three rules.

**Suggested fix.** Compute the raw barometer rate from the encoded 0x0C pressure in `_feed()`, the same way the
firmware does (pressure altitude, the same differencing and filter), and pass it through. Add a test with a 25 m/s
low-g climb.

### Observations (not bugs, for the owner)
- **Pre-reset flash data:** up to about 0.5 s before a warm reset is lost (E2: 24 of about 29 Boost state packets;
  the last log line). Possible fix: an `esp_register_shutdown_handler` flush for SW restarts. Accept the loss for
  panics.
- **Horizontal Kalman drift on the vertical preset:** 7.3-7.4 m RMS in Descent and 13.4-13.5 m in Landed (B and C2;
  9-10 m in D2/D3). The README's CSV runs show 1.9-3.3 m in Descent. Worth a look in the Kalman follow-up.
- Core 1 is 96-100 % busy with real sensors (19-20 % in HITL).
- During the 4.3 s full erase the USB command task is blocked, and 202 packet-log records are dropped.
- Session numbers restart at 1 after a full erase (0x6869) and keep counting after a clear (0x686A).

## Measurements
| Item | Value |
|---|---|
| Witness heap | free 34,448-34,504 B; minimum ever 30,812 B (158 reports) |
| Witness stack free (minimum over the session) | sensors 5036 B; usb_serial_echo 4700 of 7168 B; flash_capture 3396 B; flash_logger 3248 B; iris_can 2360 B; uart1 1564 B; cpu_load 1512 B; heartbeat 1328 B; data_forwarder 1140 B; sensor_log 1024 B |
| Iris main stack | 5444-5524 B free of 6144 |
| CPU (Witness) | real sensors: core0 35-40 %, core1 96-100 %; HITL: about 23 % / 19 % |
| Attitude / IMU | 936.3-936.7 Hz, FIFO overruns 0, maximum sensor pass 7.2-11.0 ms |
| Warm reset timeline | ack 45-58 ms after the command; reboot banner +0.13 s; RESUMING at 146 ms uptime; log append at 247-249 ms; filter restore at 368-370 ms; first telemetry 0.58-0.60 s after the command; barometer back at 1.3 s; GPS confirmed at 9.6 s (non-power resets) |
| Detection delay against prediction | B 0.02 / 0.02 / 0.02 / 0.02 s; C2 0.02 / 0.01 / 0.03 / 0.01; D2 0.04 / 0.01 / 0.02 / 0.01; D3 0.04 / 0.01 / 0.01 / 0.01 |
| Full erase | 4268 ms, no watchdog |
| Clear | 35-38 ms |
| HITL packets received, B (158 s) | witness_debug 1589, sensors 1586, state 159, heartbeat 159, camera 159, command acks 3 |
| Telemetry after a warm reset (90 s) | sensors 895, state 89, heartbeat 89, camera 90 |
| Aperture link | RSSI -29 dBm (Witness at -17 dBm) |
| USB over-current events (Pi `dmesg`) | 0 |

## Not run and why
- **Radio tests**: 0x4252 over radio (expect UNKNOWN), radio erase/clear double-confirm, Aperture uplink latency
  and pacing. Witness and Aperture do link (RSSI -29 dBm), but nothing on the bench or in the docs confirms that the
  LoRa path is a closed coax path. The rule is that any RF transmission goes only into a closed coax path, so I sent
  no uplink. Witness's own telemetry downlink was on, as it always is on this bench.
- **Iris load restore and reset-pin hold (I1/I2)**: needs loads switched on, with unknown hardware on OUT1-3/5 V,
  plus a way to warm-reset Iris (there is none). The loads stayed off.
- **Maintenance switch under the watchdog (0x5746)**: it turns on a Wi-Fi access point, which is an over-the-air
  emission.
- **Calibration capture under the watchdog**: it would overwrite the stored IMU calibration in NVS.
- **Power loss during a session-header write, forced NAND error, held radio BUSY or radio-only reset**: these need
  hardware fault injection.
- **E3/E4 flash check**: their sessions were cleared to free slots for E5/E6. They were verified live by the append
  log line.
- **K1/K2**: not repeated (passed 2026-11-16). **0x0E capture decode**: no 960 Hz synthesis run today.

## State the bench was left in
- All three boards run the working-tree images: Witness flight (flight_a) plus maintenance (maint_a), Iris, and
  Aperture.
- Witness is in Pad Idle after a final RTS reset (reason USB), with the GPS identified and configured (PX1120S at
  88398 baud, 25 Hz). I did this RTS reset on purpose so the bench is not left in the bug 1 state.
- Log store: fully erased at 09:47 (0x6869). The session counter restarted at 1, and the current session is a pad
  session with no flight in either slot. Everything erased or cleared was test data from this session. The original
  sessions (0x362 empty, 0x363 = flight B) were downloaded first.
- Radio power: nothing stored (compiled default, -17 dBm). IMU calibration in NVS is untouched. Iris loads are all
  off.
- No HITL session, streamer or other process from this session is running. No ser2net port is held.
- **uhubctl cycles used: 4.** 09:11:36, 09:23:06 and 09:33:41 were log exports (`witness_download --restart`).
  09:53:12 was deliberate, to capture a power-on boot for bug 1. Port order was unchanged after each.
- Scratch, in `%TEMP%\wbench2`:
  - raw captures and logs: `*_witness.raw/.log`
  - HITL runs: `runs/`
  - exported flash logs: `logs_B`, `logs_E12`, `logs_E56`
  - scripts: `wb.py`, `test_*.py`, `topad.py`, `k3.py`, `mk_rate.py`, `rate_climb.csv`
