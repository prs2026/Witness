# Aperture — audit evaluation

Scope: second-pass review of `aperture_findings.md` against `Software/Aperture/softwareattempt2/main` (main.cpp, ra01.cpp/.h, hardware.h, comms.h, lc86_*.cpp, ground_station_status.cpp), plus ESP-IDF v6.1 `usb_serial_jtag.c` and `sdkconfig`. Read-only; nothing built or run.
Context honoured: low-power PA below -9 dBm (bench-tested), SF7/500 with erratum 15.1 handling, and the AP2112 3V3 limit are treated as intentional/known.

## Summary table

| # | Title | Verdict | Severity |
|---|---|---|---|
| 1 | Radio left in standby after a failed transmit | CONFIRMED | high (low likelihood, total downlink loss) |
| 2 | USB command parser cannot re-scan after a bad window | CONFIRMED (mechanics); fabricated-command case PLAUSIBLE | medium |
| 3 | Low-power PA / out-of-range power on SX1262 | NOT A BUG | n/a |
| 4 | Blocking USB writes truncate frames / stall loop | NOT A BUG (truncation claim false; stall is minor) | low |
| 5 | Echo writes use zero timeout and drop bytes | PLAUSIBLE | low |
| 6 | Radio serviced once per 10 ms; back-to-back packets overwrite | PLAUSIBLE | medium |
| 7 | Console logs share the binary USB stream | PLAUSIBLE | low |
| 8 | No supply/current guard on 600 mA rail | NOT A BUG (known hardware limit) | n/a |
| 9 | Radio init failure halts the whole bridge | CONFIRMED | medium |
| 10 | transmit() discards a pending received packet | CONFIRMED | low |
| 11 | Compile-time power assert looser than hardware limit | NOT A BUG (hardening nit) | low |
| 12 | transmit() accepts 255 B but non-DMA SPI caps at 64 B | CONFIRMED (latent) | low |
| 13 | GGA with bad fields refreshes quality, position stays old | NOT A BUG | n/a |
| 14 | GPS configured once; 1 Hz window "barely holds" | NOT A BUG | n/a |
| 15 | FIRMWARE_HANDOFF.md drift | CONFIRMED | low |
| N1 | No radio health check or re-init (NEW) | PLAUSIBLE | high |
| N2 | Max drive setting equals the FEM absolute limit (NEW) | PLAUSIBLE | low |

## Details per finding

### 1. Radio left in standby after a failed transmit — CONFIRMED, high
- Evidence: `ra01.cpp:212-213` sets STDBY_RC; early returns at 214, 218, 220, 226, 234, 237, 241, 246 skip the cleanup at `ra01.cpp:267-271` (ClearIrq + SetRx continuous). `receive()` (`ra01.cpp:274-317`) only reads IRQ status and never re-arms RX. The same applies if the SetRx at line 269 itself fails (BUSY/SPI).
- Failure path: any one SPI error or 500 ms BUSY timeout (`wait_until_ready`, line 323-328) in any of the 8 steps during a command uplink leaves the SX1262 in standby. Every later `receive()` returns ESP_ERR_NOT_FOUND. Telemetry is silently lost until reboot. Likelihood is low (SPI/BUSY failure), consequence is total downlink loss, and it is cheap to fix. The handoff already records FEM-transient glitches.
- Fix: in `transmit()`, run all steps after SetStandby through one exit path: `esp_err_t result = do_tx(); (void)ClearIrq; rx = SetRx(continuous); return result != OK ? result : rx;` (a lambda or a small helper `transmit_locked()` called from `transmit()`). Also see N1 for a catch-all.

### 2. USB frame parser — CONFIRMED (mechanics), medium
- Evidence: `main.cpp:97-100` resyncs on 0x05 only when `command_length == 0`; `main.cpp:102-105,133-143` always consume a full 10-byte window and reset to 0 after a bad EOF without re-scanning those bytes. No inter-byte timeout.
- Failure path A (real): a partial frame left by host reconnect or abort means the next real frame completes the window with a wrong last byte. Both are discarded, and the leftover bytes of the real frame are skipped until the next 0x05. One command is lost. The next one parses correctly, and the host retries unacked commands, so it self-heals after one retry.
- Failure path B (fabricated): needs a stray 0x05 followed by 9 bytes ending in 0x0A, i.e. non-Horizon junk on the port. Very unlikely, but the payload goes straight to LoRa with no CRC (and could be Wi-Fi on, flight state, HITL).
- Note: the first pass's claim that the doc's "resynchronize" statement is wrong is only half right; resync does work from idle.
- Fix: on a bad EOF, search `command[1..9]` for a 0x05, shift that tail to the front and set `command_length` to its length, then keep going. Record `last_byte_tick`; if `command_length > 0` and no byte for about 50 ms, set `command_length = 0`. Optionally validate the opcode against a whitelist before transmitting.

### 3. Low-power PA on SX1262 — NOT A BUG
- Evidence: `ra01.cpp:434-447`. Parameter order (duty, hpMax, deviceSel, lut) matches {0x01,0x00,0x01,0x01} for the low-power PA and {0x04,0x07,0x00,0x01} for the high-power PA. `configure_pa_for()` runs before SetTxParams, in standby. This is the datasheet 13.4.4 configuration, validated on the bench at -17 dBm, and the module's FEM input stays far below +3 dBm. The first pass's range claim (-9..+22 only) ignores the low-power PA option. Only comment drift remains: `ra01.cpp:145-147` still says "high-power PA / 2 dBm".
- Fix: none; optionally refresh the stale comment.

### 4. Blocking USB writes — NOT A BUG (mostly), low
- Evidence: `usb_serial_jtag_write_bytes` (IDF `usb_serial_jtag.c:259-283`) does a single `xRingbufferSend` on a byte ring buffer, which is all-or-nothing, and returns `size` or 0. A truncated/partial frame cannot occur, and `main.cpp:152-158` can only drop a whole packet when the ring is full. Dropped frames only happen when the host is not draining, in which case nobody receives them anyway.
- Residual: with a stalled host (Horizon has a `ui_stalls.log`), once the 512 B ring is full each write blocks 100 ms, which slows the radio poll (feeds #6). Low.
- Fix: optional; use timeout 0 for the 0xF0 status and skip forwarding when the ring is full, so the radio loop never blocks.

### 5. Echo writes with zero timeout — PLAUSIBLE, low
- Evidence: `main.cpp:93`. Single-byte all-or-nothing send; it fails only if the 512 B TX ring is full or `tx_mux` is held, i.e. the host is not draining. Horizon would then be in trouble regardless.
- Fix: none needed; if echo verification matters, check the return value and count drops in the next 0xF0.

### 6. One packet read per 10 ms loop — PLAUSIBLE, medium
- Evidence: `main.cpp:147-164`, `vTaskDelay(1)` is 10 ms (`CONFIG_FREERTOS_HZ=100`, sdkconfig:2536). `ra01.cpp:274-317` polls IRQ over SPI; DIO1 (GPIO10) is configured and routed (IRQ mask on DIO1 at `ra01.cpp:182-187`) but never used. Continuous RX uses a single buffer at base 0.
- Failure path: Witness downlink bursts of several packets (25/65/19 B per `HITL/LORA_MODULATION_OPTIONS.md`) arrive within tens of ms; a 19 B packet is about 13 ms on air. If the next packet starts demodulating before the loop reads the previous one (loop latency of up to 10 ms plus any USB-write block), the earlier packet is lost or read with mixed data. Not measured. It depends on Witness's gap between packets in a burst.
- Fix: attach a DIO1 GPIO ISR that notifies the main task, wait on `ulTaskNotifyTake` (with a short timeout) instead of `vTaskDelay(1)`, and call `receive()` in a loop until NOT_FOUND. Alternatively set `CONFIG_FREERTOS_HZ=1000`.

### 7. Console logs share the binary stream — PLAUSIBLE, low
- Evidence: `sdkconfig.defaults:4` routes the console to USB Serial/JTAG, and main.cpp never calls `usb_serial_jtag_vfs_use_driver()`. So ESP_LOG writes go straight to the FIFO, bypassing the ring buffer that carries the frames, and can land inside a frame.
- Failure path: log lines are infrequent (GPS fix transitions `lc86_gps.cpp:266-274`, a 30 s diagnostic, "Sending command"), so a splice is rare. The first pass overstated "logs on every loop": receive errors are rate-limited by 500 ms BUSY timeouts, and CRC errors are per event. The host already validates by fixed length and EOF; the cost is at most one corrupted frame.
- Fix: after `usb_serial_jtag_driver_install`, call `usb_serial_jtag_vfs_use_driver()` (frames are then atomic ring items) or set the log level to WARN for "gps" and "aperture".

### 8. No supply/current guard — NOT A BUG
- Evidence: the 600 mA AP2112 limit is known; software already caps drive at +3 dBm (`hardware.h:26`, `ra01.cpp:423`). The code has no way to measure rail headroom, so "guard" has nothing to act on. The real risk (SX1262 sagging or resetting without the MCU) is covered by N1.
- Fix: none.

### 9. Radio init failure halts the bridge — CONFIRMED, medium
- Evidence: `main.cpp:55-66`: after a failed `initialize()` the code loops forever logging; no echo, no 0xF0, no retry. `Ra01::initialize()` also leaks state on failure: `spi_bus_initialize`/`add_device` succeed at `ra01.cpp:116-124` and the later `return result` paths (132-203) do not free them, so even a caller-side retry would hit `ESP_ERR_INVALID_STATE` at line 116.
- Failure path: a transient BUSY/SPI fault at boot makes the device a silent brick until power cycle, with no pad GPS status for Horizon.
- Fix: make `initialize()` clean up on failure (`spi_bus_remove_device`/`spi_bus_free`, reset `spi_device_`), retry every few seconds in the main loop, and keep the USB/echo/0xF0 path running in degraded mode (an optional radio-down flag in the status).

### 10. transmit() drops a pending RX — CONFIRMED, low
- Evidence: `main.cpp:86-145` handles USB (and may transmit) before `receive()` at 147, and `ra01.cpp:239-240` clears all IRQs. A packet that finished within the last loop iteration is cleared unread. In-flight RX is inherently lost (half duplex).
- Fix: call `receive()`/forward once immediately before `radio.transmit()` in main.cpp.

### 11. Compile-time assert allows +22 — NOT A BUG (nit), low
- Evidence: `ra01.cpp:39-40` only checks the default; every runtime path goes through `set_tx_power_dbm`, which enforces the -17..+3 limits (`ra01.cpp:423`), and `hardware.h:27` asserts the default. No bypass exists today.
- Fix (optional): change the assert upper bound to `IRIS_RADIO_TX_POWER_MAX_DBM`.

### 12. 255-byte transmit vs 64-byte non-DMA SPI — CONFIRMED (latent), low
- Evidence: `ra01.cpp:208` allows up to 255 B; `write_buffer` (350-363) is one `spi_device_transmit` of length+2 with `SPI_DMA_DISABLED` (line 116); `read_buffer` already chunks at 60 B. Today only 10-byte commands are sent, so it cannot trigger.
- Fix: reject `length > 62` in `transmit()` (before SetStandby), or chunk WriteBuffer with advancing offsets.

### 13. GGA with bad fields — NOT A BUG
- Evidence: `lc86_nmea.cpp:133-166`, `ground_station_status.cpp:51-66`. A checksum-valid GGA with quality >=1 and unparseable position is not something the receiver emits; and the fix age field is computed from `fix_ms`, which only advances on a fully parsed fix, so the host can see the position is old (the code comment at line 51 says so).
- Fix: none.

### 14. GPS configured once at boot — NOT A BUG
- Evidence: `lc86_gps.cpp:244-257` configures once; after a module reset the default rate is 1 Hz GGA, which is well inside the 3 s `kGgaTimeoutMs` (`lc86_nmea.h:16`). The "flicker to 0" claim is wrong. The only effect is a lower update rate, and fix age still reports correctly.
- Fix: none.

### 15. Documentation drift — CONFIRMED, low
- Evidence: `FIRMWARE_HANDOFF.md:63` (125 kHz) vs `hardware.h:37` (500 kHz); line 69 (-2 dBm) vs `hardware.h:24` (-17 dBm). Line 118 is correct for the idle state.
- Fix: update the handoff.

### N1 (NEW). No radio health check or re-init — PLAUSIBLE, high
- Evidence: the SX1262 is configured only once at boot (`main.cpp:55`, `ra01.cpp:69-204`); nothing ever calls GetStatus (0xC0) or compares the chip mode with RX. If the SX1262 resets or latches into standby/brownout while the ESP32-S3 survives (shared AP2112 3V3, FEM transients, and `FIRMWARE_HANDOFF.md` already notes USB disconnects from FEM transients), `receive()` returns NOT_FOUND forever and the host sees silence. The MCU brownout detector would reboot the MCU and run init again, so the exposure is the SX1262-only case.
- Fix: once a second, in the main loop, read GetStatus and require chip mode RX (0x5) when idle. On mismatch or read failure, re-run a clean init (see #9) and log it. Count re-inits in the 0xF0 status or a log line.

### N2 (NEW). Max drive equals the FEM absolute limit — PLAUSIBLE, low
- Evidence: `hardware.h:26` and `ra01.cpp:423` allow exactly +3 dBm, which is the Ra-01SH-P FEM's maximum input. The SX1262's output power setting has some tolerance across PA settings, temperature and supply, so the actual FEM input at a +3 dBm setting could exceed the limit. Unquantified; I did not check against measurements.
- Fix: bench-measure at +3 dBm (high-power PA, with a 30 dB-plus attenuator), and consider capping at +1/0 dBm (Ai-Thinker recommends 0..+3).

## Significant issues to act on

- #1 CONFIRMED, high: restore RX on every `transmit()` exit path (single cleanup/exit block).
- N1 PLAUSIBLE, high: add a 1 Hz GetStatus watchdog that re-inits the radio if not in RX.
