# Iris — audit evaluation

Scope: `Software/Iris/softwarev1IRIS/main` (paths below are relative to that folder), `sdkconfig`, and the Iris schematics in `Hardware/witnessv0.2/iris` (`MCU.kicad_sch`, `loadswitches.kicad_sch`, `power.kicad_sch`). Read-only; nothing was built or run.

Hardware facts used:
- The current sensor is a PAC1934 (4-channel part). Three channels are used, each with a 20 mOhm shunt (R43, R44, R49, also 20m on the PCB).
- Nominal full scale is therefore 100 mV / 20 mOhm = 5.0 A. That matches the wire format: 255 counts x 20 mA = 5.10 A (`comms.h:301`).
- `LS_EN` (GPIO9) is wired to the MCP23008 RESET pin and to R55, a 10k resistor to GND (`MCU.kicad_sch`, around 157,127).
- EN1-EN3 each have a 10k resistor (R52-R54), so loads default OFF in hardware.

## Summary table

| # | Title | Verdict | Severity |
|---|---|---|---|
| 1 | MCP23008 latch cache can skip a write after failed I2C | PLAUSIBLE (mostly self-healing) | low |
| 2 | Current telemetry clips at 1.00 A | CONFIRMED (firmware scaling, not a hardware limit) | medium (high if any load exceeds 1 A) |
| 3 | Any Iris reset switches every load off; nothing restores state | CONFIRMED | **critical** |
| 4 | No fail-safe on command silence / no command gating | NOT A BUG (hold-last-state is correct; sender gating is low) | low |
| 5 | UART1/USB command frames have no integrity check | PLAUSIBLE | low |
| 6 | No resync in command parser; no per-command ack | CONFIRMED (USB/UART1 only) | low |
| 7 | Partial USB writes leave truncated frames | CONFIRMED | low |
| 8 | Battery ADC may saturate above about 10 V | PLAUSIBLE | low |
| 9 | GPIO9 strapping pin shared with MCP reset | PLAUSIBLE | low |
| 10 | Output pins float between expander reset and config | NOT A BUG | low |
| 11 | Main loop has no watchdog | PLAUSIBLE (first pass misread TWDT config) | low |
| 12 | CAN fault counters never reported | CONFIRMED | low |
| 13 | No I2C bus recovery | PLAUSIBLE | low |
| N1 | Boot uses `ESP_ERROR_CHECK` on I2C/ADC/USB init, so a transient failure causes a panic loop | NEW, CONFIRMED | medium |
| N2 | PAC range never written; default FSR assumed unipolar; instantaneous (not averaged) samples | NEW, PLAUSIBLE | medium |
| N3 | Witness does not detect an Iris reboot and re-assert load state | NEW, CONFIRMED (part of #3 fix) | critical, merged with #3 |
| N4 | USB writes block 10 ms each when no host is attached | NEW, CONFIRMED | low |

## Details per finding

### 1. Latch cache skip after failed I2C — PLAUSIBLE, low
- Code: `mcp23008.cpp:121-131` is as described. A write that fails after the chip latched it leaves the cache stale, and the next opposite command compares equal and skips the write.
- Mitigation the first pass missed: `poll_heartbeat` (`usb_serial_echo.cpp:427-431` and `445-450`) calls `write_pin` on the blue LED every 100 ms and 1 s. That sends the full cached OLAT byte, including the OUT bits. Any OLAT drift between the chip and the cache is overwritten from the cache within about 1 s. The "load stays on" example therefore lasts at most 1 s.
- Real remaining gap: only OLAT is refreshed. IODIR is never rewritten, and the cache cannot detect an MCP-only reset (glitch on `LS_EN`, whose only pull is a 10k resistor to GND, or a 3V3 dip). After such a reset the pins are inputs, so the loads go off through the EN pulldowns, while `direction_`/`output_latch_` and the status flags still say ON. Nothing ever corrects this.
- Fix: in the 1 Hz heartbeat path also write IODIR (and OLAT) from the cache. Better, read back IODIR/OLAT periodically and set a mismatch flag. Optionally make command writes unconditional by dropping the `new_latch == output_latch_` early return.

### 2. 1.00 A clip — CONFIRMED (scaling defect, not a hardware limit), medium
- Code: `pac1931.cpp:74-88` computes raw x 100 mV / 65536 / 20 mOhm / divisor (5). Full scale is 1,000,000 uA. `hardware.h:71-74` documents the 5 as an "empirical" correction. `usb_serial_echo.cpp:704-710` then saturates the 8-bit field at 255 counts.
- Hardware check: with 20 mOhm shunts and a 100 mV range the sensor spans 5.0 A, which equals the 5.10 A wire range. The 1.00 A ceiling comes only from the divisor 5. Anything above 1 A is silently reported as 1.00 A, with no flag. Board design (SQA403EJ load switches, "2Aloadswitch" sheet) allows loads up to 2 A.
- The origin of the 5x is not explained, and physically it is odd:
  - A sense voltage 5x too high would need roughly 100 mOhm shunts. The PCB and schematic both say 20m.
  - Alternatively the "applied current" used during calibration was wrong.
  - The default bipolar range (see N2) would make readings about 2x too low, not 5x too high.
  So the calibration is unverified, and a flat divisor can be wrong in either direction.
- Failure path: a camera or load drawing more than 1 A (inrush, a camera charging) is reported as 1.00 A. Any current-based sanity check on Witness is wrong.
- Fix:
  1. Measure each channel against a bench meter at about 0.3, 1 and 2 A. Fit a per-channel gain and offset.
  2. Remove the blanket divisor, or replace it with the fitted value.
  3. Use the full 5 A range.
  4. Set a status bit when a count is 255 or when raw is at full scale.

### 3. Any Iris reset switches every load off, nothing restores state — CONFIRMED, critical
- Evidence:
  - `mcp23008.cpp:29-34` pulses MCP RESET on every `initialize()`. That clears OLAT and sets IODIR to inputs.
  - `main.cpp:22-33` then drives OUT1-3, 5V_EN and the LED low.
  - The hardware has the same effect without firmware: `LS_EN` is also GPIO9 with a 10k pulldown (R55), and EN1-3 have 10k resistors. So loads are off from the moment the ESP resets until a command re-enables them.
  - Reset sources enabled in `sdkconfig`:
    - Brownout at level 7 (`sdkconfig:1141-1157`, `2185-2201`).
    - Panic is set to reboot (`1286`, `1290`, delay 0).
    - SPI-flash brownout reset (`2017-2019`).
  - Witness only forwards commands (`Software/Witness/softwarev1/main/usb_serial_echo.cpp:613-620`). Nothing keeps a desired state or re-sends it. Iris's own `iris_status_flags_` starts at 0 (RAM only).
- Failure path: any brownout, panic or ESP-only reset in flight drops camera power (recording is lost or the file is cut), and it stays off after boot. Total outage is at least the boot time of a few hundred ms, plus however long until an operator command. The 0x03 stream resumes with OUT flags cleared and a small uptime, but nothing reacts.
- Fix (do all three):
  1. Persist the commanded load mask in `RTC_NOINIT` RAM with a magic number and CRC (survives brownout/panic resets unless the RTC domain loses power). On boot, if the reset reason is not power-on or USB, re-apply that mask instead of forcing zero.
  2. Skip the MCP reset pulse on a warm boot where the stored mask is valid. Also set GPIO9 high before `gpio_config` so the pin is not briefly driven low. Check on the bench whether the R55 pulldown against the C3's internal pull-up already resets the MCP during the ROM boot window. If it does, hardware must change (see below).
  3. Witness side (see N3): compare Iris uptime and OUT flags from 0x03 against the desired state, and re-send 0x31/0x33/0x35/0x51 on any mismatch or uptime regression.
- Hardware for the next revision: take the MCP RESET off GPIO9, or change R55 to a pull-up, so an ESP reset does not reset the expander.

### 4. No fail-safe on silence; no gating — NOT A BUG (design is correct), low
- Code: `usb_serial_echo.cpp:519-538`, `582-647`. Loads hold their last state when Witness or the CAN link goes silent.
- Why this is right for a rocket: dropping cameras after N seconds without a heartbeat would shut them off exactly when the link is lost (for example during a Witness reset). Holding the last state is the safe behaviour. The suggested "drop after N seconds" fix would be harmful. Do not add it.
- Remaining minor point: any node on CAN ID 0x120 can send 0x05 commands. CAN has hardware CRC and Witness is the only peer, so this is low. Flight-state gating belongs in the Witness bridge.

### 5. No integrity check on UART1/USB commands — PLAUSIBLE, low
- Code: `usb_serial_echo.cpp:540-580`. Confirmed: only ID 0x05 and EOF 0x0A are checked, and a single bit flip in the opcode could change 0x30 to 0x31.
- USB has its own link CRC. CAN has CRC. Only UART1 is at risk, and only through noise on an unterminated line. A floating UART1 RX could also start a bogus frame that swallows nine bytes of real traffic.
- Fix: add a validity rule that is cheap, for example byte 1 (status) = 0x00 or a rolling nibble check, or require that the parsed frame's reserved byte equals its XOR with the opcode. Alternatively, disable UART1 command RX if the port is unused in flight.

### 6. No resync; no ack — CONFIRMED (USB/UART1 only), low
- Code: `usb_serial_echo.cpp:247-266` and `546-560`. Both swallow exactly 10 bytes once a 0x05 is seen, with no timeout. A single dropped byte makes the next frame be lost as well.
- The CAN path is unaffected (packet-level reassembly with a timeout). In flight Witness uses CAN, so this is a bench issue.
- "No ack" is by design: the 0x03/0xE1 status flags (and PING/PONG) serve as the confirmation, with up to 1 s latency.
- Fix: on a bad EOF, rescan the 10 bytes for the next 0x05. Add an inter-byte timeout (about 20 ms) that resets the partial frame.

### 7. Partial USB writes leave truncated frames — CONFIRMED, low
- Code: `usb_serial_echo.cpp:792-807`. Correct as described. A zero return mid-frame leaves a partial frame in the ring buffer. It affects only the USB mirror; CAN and UART1 receive whole packets.
- Fix: if the free space in the USB TX buffer is below the frame length, drop the whole frame before writing.

### 8. Battery ADC saturation — PLAUSIBLE, low
- Code: `battery_voltage.cpp:99-111` (4:1 scaling, correct) and `battery_voltage.h:23` (12 dB). The C3 12 dB range is about 2.5 V, so the pack-voltage ceiling is about 10 V (the first pass said 9.8 V; this should be confirmed against the IDF range table). A 2S pack (8.4 V) is fine, a 3S pack (12.6 V) would read flat at the ceiling. `power.kicad_sch` uses a TPS63070 buck-boost, which supports a wide input range, so the pack size is not constrained by the hardware. The pack is not stated anywhere I could find.
- Also, a 20k divider source impedance into the C3 SAR ADC gives some error without a bypass capacitor; this is cosmetic for a monitor.
- Fix: confirm the pack. Add a saturation flag when the ADC raw value is near full scale.

### 9. GPIO9 strapping pin — PLAUSIBLE, low
- Evidence: `hardware.h:21` and `65`. `MCU.kicad_sch` shows R55, a 10k resistor from `LS_EN` to GND. On the ESP32-C3, GPIO9 low at reset selects download boot when GPIO8 is high. The internal pull-up (about 45k) against 10k gives about 0.6 V at power-up, which reads low.
- The board is evidently booting in normal use, probably because GPIO8 (the LED_RED line) is low or otherwise outside the download-boot condition. That makes this a margin and temperature fragility rather than an observed bug.
- Fix: on the next revision change R55 to a pull-up, or isolate it, so GPIO9 is high at reset. This also fixes #3 at the hardware level.

### 10. Outputs float between expander reset and config — NOT A BUG, low
- All three EN nets have 10k resistors (R52-R54), so a floating output is held at the OFF level. The MCP inputs cannot pulse a load on. The OLAT-before-IODIR order (`mcp23008.cpp:88-98`) is correct. The real boot-time consequence is the one in #3.

### 11. No main-loop watchdog — PLAUSIBLE, low (and first pass misread config)
- `main.cpp:50-65` is correct: nothing feeds a watchdog.
- However, `sdkconfig:1331` has `# CONFIG_ESP_TASK_WDT_PANIC is not set`, so even the idle-task TWDT only prints a message and does NOT reset. A spin hang would therefore leave the loads in their last state (good for flight), not reboot.
- Do not enable a reset watchdog until #3 is fixed, because a watchdog reset would switch the cameras off.
- Fix: after #3, subscribe the main task to the TWDT with panic enabled.

### 12. CAN counters never reported — CONFIRMED, low
- Code: `twai_driver.cpp:178-190` (counters), `can_link.cpp:14-26` (full queue drops the newest), `can_link.cpp:46-48`. No caller reports the counters.
- Bus-off recovery exists (`twai_driver.cpp:147-165`). The 8-deep queue holds up to 8 stale packets, a minor effect.
- Fix: add a "CAN fault seen" bit to the status flags.

### 13. No I2C bus recovery — PLAUSIBLE, low
- Code: `mcp23008.cpp:39-51`, `135-145`, `pac1931.cpp`. If the bus hangs, load commands fail with an error that only reaches the log, which is disabled after init (`usb_serial_echo.cpp:182`). Telemetry still shows the monitor error flag for PAC reads.
- Loads hold their state, so it is not dangerous; it only prevents later commands.
- Fix: after N consecutive timeouts call `i2c_master_bus_reset()`; report a flag.

## NEW findings

### N1. `ESP_ERROR_CHECK` in `app_main` turns a transient init failure into a reboot loop — CONFIRMED, medium
- `main.cpp:22`, `32`, `44`, `48`: failures of the MCP init or config, the battery ADC init or USB init abort the firmware. Panic is set to reboot with zero delay (`sdkconfig:1286-1290`).
- Failure path: after a brownout, a single NACK from the MCP23008 or PAC while 3V3 is still settling aborts boot. Each retry pulses the expander RESET again, so loads flicker or stay off until the rail is stable. A permanent fault in the ADC or USB driver makes Iris unusable even though the load switches would work.
- Fix: retry the MCP init a few times with a short delay. Treat ADC and USB init failures as non-fatal and continue, as is already done for CAN and the PAC.

### N2. PAC range is never programmed; default assumed unipolar; samples are instantaneous — PLAUSIBLE, medium
- `pac1931.cpp:76` comment and `FIRMWARE_AGENT_GUIDE.md` section 4 say the default range is unipolar 0-100 mV. The firmware never writes the NEG_PWR_FSR register (0x1D) or CTRL. As I recall the PAC193x datasheet, the default is bipolar (verify against the datasheet). In bipolar mode the raw value is signed 16-bit, and the scale is 100 mV per 32768 counts. The firmware would then:
  - report 2x too low for positive currents, and
  - report any small negative offset (raw 0xFFxx) as almost full scale, which would then clip as the maximum.
- This conflicts with the "5x too high" calibration note, which suggests the real cause was never established.
- `read_currents_microamps` reads VSENSE after REFRESH_V, which is a single sample. Peak current is the peak of 10 instantaneous samples per second, so brief inrush spikes can be missed.
- Fix: write the FSR register explicitly (unipolar) at init, read it back to verify, and consider the VSENSE_AVG registers. This should be done together with the calibration in #2.

### N3. Witness does not re-assert load state after an Iris reboot — CONFIRMED, critical (part of #3)
- Witness forwards load commands only (`Software/Witness/softwarev1/main/usb_serial_echo.cpp:613-620`). It does not track desired state or compare it with the OUT flags in the 0x03 status byte. There is no recovery path from Witness.
- Fix: keep a desired-state mask in Witness. When 0x03 shows a mismatch, or the Iris uptime goes backwards, re-send the ON commands (with rate limiting).

### N4. USB writes block when no host is attached — CONFIRMED, low
- `usb_serial_echo.cpp:792-807` and `225-241` wait up to 10 ms per call once the TX ring is full, which is the normal state in flight with USB unplugged. At 1 Hz this is harmless, but the optional 10 Hz 0xE1 stream costs about 100 ms/s of loop time, delaying CAN polling.
- Fix: skip USB writes if the TX buffer is full or no host is enumerated.

## Significant issues to act on
- #3 / N3 (CONFIRMED, critical): an Iris reset (brownout, panic, ESP reset) turns every camera and load off and nothing restores them. Persist the commanded mask in RTC memory and re-apply it on warm boot, skip the expander reset on a warm boot, and have Witness re-send load commands when Iris uptime goes backwards or the OUT flags differ from the desired state.
- #2 (CONFIRMED, medium; high if any load exceeds 1 A): the 1.00 A clip is a firmware divisor, not a hardware limit (20 mOhm and 100 mV is 5 A, matching the wire format). Calibrate each channel with a bench meter, remove the blanket /5 divisor, and flag saturation.
- N2 (PLAUSIBLE, medium): the PAC range register is never written. Program and verify FSR explicitly, together with the #2 calibration.
- N1 (CONFIRMED, medium): `ESP_ERROR_CHECK` on init turns a transient I2C glitch into a reboot loop. Retry the MCP init and make ADC/USB init failures non-fatal.
