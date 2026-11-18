# Bench report 2026-11-17c: build settings for core-1 performance (flight image only)

**Result:** `sdkconfig.flight` now holds `-Os`, a 32 KB instruction cache, a 16 KB data cache and 240 MHz. Core 1 went
from 96-100 % busy with about 480 overruns a second to **23-25 % with 1-6 overruns a second**, and free heap went
**up** from 34 KB to 42.6 KB (minimum 38.3 KB, was 30.2 KB). No risk is rated high, so this is the configuration
left in the repo and flashed on the board.

**One real bug was found on the way** and fixed in code (see "Bug found"): the flight image's image-info record is
removed by the linker at `-Os`/`-O2`, which made the maintenance image unable to return to flight (the Wi-Fi switch
got stuck in recovery). Without that fix the new optimisation level must not be used.

Printer note: the owner confirmed that the IMU noise on the bench (0.6 mg rising to 25 mg) comes from a 3D printer
running on the same bench, not from the firmware or the board. It was running during part of this work (accel noise
23-25 mg, attitude `not initialized` or windows being rejected) and stopped during the rest. I note which runs had it.
IMU calibration was not run. No IMU-side result was treated as a regression.

## Summary of the steps

All numbers are the `cpu` and `core-1 profile` log lines over 60-85 s of Pad Idle with real sensors, same bench, each
configuration flashed and measured on its own. "Printer" = the printer was running (attitude not initialized, so
the per-sample attitude and position work is skipped and the load reads lower). Overruns are `xTaskDelayUntil` late
wake-ups per second. Free heap is the `cpu` line (last value / minimum ever).

| # | Configuration (on top of the one above it) | Core 1 | Core 0 | Overruns/s | Heap free / min | Printer |
|---|---|---|---|---|---|---|
| 0 | Default: `-Og`, 16 KB icache, 32 KB dcache, 160 MHz (before) | 99-100 % | 38-41 % | 455-490 | 34.2 / 30.2 KB | off |
| 1 | 32 KB icache (`-Og`) | 33.0 % / 36.6 % | 11.6 / 12.2 % | 10.1 / 14.4 | 17.9 / 13.8 KB | on / off |
| 2a | 32 KB icache + `-O2` | 28.9 % / 31.5 % | 10.7 / 11.6 % | 3.6 / 9.6 | 22.1 / 18.0 KB | on / off |
| 2b | 32 KB icache + `-Os` (**kept**) | 25.6 % / 29.4 % | 9.1 / 9.9 % | 2.9 / 7.8 | 26.9 / 22.8 KB | on / off |
| 3a | + 64-byte dcache lines | 28.8 % | 9.9 % | 5.4 | 26.9 / 22.8 KB | off (attitude valid late) |
| 3b | + 16 KB dcache (**kept**) | 27.6 % | 9.1 % | 9.4 | **42.6 / 38.4 KB** | off |
| 3c | 64 KB dcache | not built | | | would cost another 32 KB | |
| 4 | `-Os` + 32 KB icache + **240 MHz** (**kept**) | 23.5 % | 7.8 % | 3.1 | 26.9 / 22.8 KB | off |
| 5 | `-Os` + 32 KB icache + `CONFIG_SPI_MASTER_IN_IRAM` | 29.6 % | 9.9 % | 12.0 | 26.9 / 22.8 KB | off |
| **F** | **Final: `-Os`, 32 KB icache, 16 KB dcache, 240 MHz** | **23.3 %** | **7.5 %** | **0.9** | **42.6 / 38.4 KB** | on |

Two kinds of column pairs: where two values are given ("33.0 % / 36.6 %") they are printer on, then printer off.
The step 3 and 5 rows were measured with the printer off; their comparison base is the step 2b "off" value (29.4 %).

### Step details

1. **32 KB icache.** `CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`. 64-byte instruction-cache lines do not exist on the ESP32-S3
   (the choice is 16 or 32 bytes), so there is nothing to try there. Heap fell by 16.4 KB (34.2 to 17.9 KB), which is the
   cost of this step. Core 1 falls from 99 % to 33-37 %, core 0 from 39 % to 12 %.
2. **`-Os` against `-O2`** (assertions stay enabled: `CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE=y`, level 2, in all builds).
   `-Os` wins on every number: lower core-1 load (29.4 against 31.5 % with the printer off), 4.8 KB more heap (the image
   is 52 KB smaller than `-Og`, 47 KB smaller than `-O2`), and all stack margins equal or better. Kept `-Os`.
3. **Data cache.** There is no PSRAM, so only flash rodata goes through the data cache. 64-byte lines: no gain (28.8 %
   against about 29.4 %), no cost, not kept. **16 KB data cache: no measurable change in load (27.6 %), and it returns
   16 KB to the heap** (26.9 to 42.6 KB free), which pays for the larger icache completely. 64 KB data cache: not built;
   it would take 32 KB more heap than the 32 KB default, which this board cannot spare (26.9 KB free with the 32 KB icache).
4. **240 MHz.** On top of the cache setup: core 1 29.4 to 23.5 % (the loop is no longer cache-bound, so the clock
   helps now; with the small icache it did not). Overruns 7.8 to 3.1 per second. Kept; see the risk section for the
   current and heat question.
5. **Hot code in IRAM.** `CONFIG_SPI_MASTER_IN_IRAM` gave no gain (29.6 %), not kept. `CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH`
   stays off (the default). I did not add `IRAM_ATTR` to per-sample functions: with the final cache setup the profile
   is flat (attitude 3 %, prep 4.5 %, predict 1.6 %), the best candidate saves under 1 point, and IRAM code that calls
   flash code (libm, soft-float helpers, the logger) crashes while the flash cache is off. IRAM text is 59,075 bytes at
   `-Os` (67,407 at `-Og`); IRAM left over is part of the heap pool, so there is no separate spare: the 42.6 KB of free
   heap is the budget.
6. **Anything else.** Checked and not worth it: `-O2` (above), 64-byte dcache lines (above), instruction-cache wrap mode
   (marked "depends on complex conditions" in the Kconfig, not tried), QIO flash (boot-loops, not retried), flash frequency
   (not raised, as instructed). The 16 KB data cache was the one extra find.

## Validation of the final configuration

The final image is `-Os`, 32 KB icache, 16 KB dcache, 240 MHz, plus the image-info fix below. Flashed with the verified
target (`flash_verified.ps1 -CheckOnly` first, 115200 baud, 700x for flashing only, 710x for communication).

| Check | Result |
|---|---|
| Soak 1, Pad Idle, 1730 s (28.8 min), printer **on** (accel noise 23 mg) | core 1 23.6 % (23.1-24.2), core 0 7.8 %, **1.0 overruns/s**, max pass 1.6 ms (last 10 s; 4 ms earlier), heap 42,596 free / 38,436 min constant for 143 `cpu` lines, no reset, no task watchdog, no abort, no `Guru`. Attitude `valid`. Image: before the image-info fix. |
| Soak 2, Pad Idle, 590 s, printer **off** (accel noise 0.7 mg, all attitude windows complete) | core 1 25.2 % (24.9-25.6), core 0 8.3 %, **5.5 overruns/s** (1.1 % of passes), max pass 3.9 ms, heap 42,464 / 38,304, no reset, no watchdog. This is the heavier case. Image: final. |
| HITL nominal 10 g (before and after the fix) | PASS both times: Boost +0.02, Coast +0.01, Descent +0.01/+0.03, Landed +0.02/+0.08 s; 15,819 frames. |
| 0x4252 in Coast, software restart (mode 0) | ack 50 ms, `RESUMING IN-FLIGHT STATE 2 after reset reason 3`, telemetry back at 0.53 s, resumed bit set, barometer ready at 295 ms |
| 0x4252 in Coast, panic (mode 1), run twice | ack 47 ms, `abort() was called` (assertions are live), `RESUMING ... reset reason 4`, telemetry 0.53 s, barometer ready 295 ms |
| Power-on boot (final image, uhubctl), run twice | PX1120S found at 115200 at 2.5 s, config readback timed out, probe at 18.6 s, found at 88398, **25 Hz confirmed at 22.5 s** (second boot: same sequence); 2001 GGA by 100 s; barometer ready in all 1029 sensor packets, no `PROM` or `MS5607` error lines; 0 `NO_MEM` |
| 0x5746 Wi-Fi round trip over USB (final image, after the fix) | status OK; on (2 dBm, 5 min) OK; restarts into maint_a; `boot partition restored to flight slot 0`; AP up (`witness-6025`, 2 dBm, auto-off 5 min); status in maintenance `0x010c`; off OK; `leaving maintenance (Wi-Fi off over USB)`; flight back and in Pad Idle, `selftest: running flight slot 0` |
| Heap and stacks after everything | heap free 42,464, **minimum 38,304 B** (after soaks, HITL, resets, Wi-Fi round trips and two power-on boots; the default build had 30.2 KB). Stack free minimum in bytes: sensors 6072, uart1 1592, flash_logger 3448, iris_can 2376, usb_serial_echo 4964, flash_capture 4172, cpu_load 1532, data_forwarder 1172, sensor_log 1068, heartbeat 1380, serial_command 4964. Equal to or above the default build in every task (sensors 6012-6168 before, uart1 1508-1588 before). |
| USB over-current events (Pi `dmesg`) | 0 before and after every run in this report, including the 240 MHz soaks |

Vibration dependence: the load numbers **do** depend a little on the printer, in a way that is not a fault. With
noise present the stationary-window check rejects windows, so the estimator skips its window work. Same
build, soak 1 (printer on) 23.6 % and 1.0 overruns/s against soak 2 (printer off) 25.2 % and 5.5 overruns/s. Judge
the configuration by the quiet numbers (25 %, 5.5 per second). The ranking of the options was the same with and without.

## Bug found: the image-info record is dropped at `-Os` and `-O2` (fixed)

- **Symptom.** The first 0x5746 round trip with the new configuration went into the maintenance image and then stayed
  there: `boot_mgr: no valid flight image installed: staying in maintenance (recovery)`, `flight slot -1`, Wi-Fi up
  with no auto-off, and a second restart landed in recovery again. The board had to be re-flashed (flight image plus
  `ota_data_initial.bin`) to get back to flight.
- **Cause.** The flight image carries a 48-byte record (`WIMG`, `g_image_info` in `boot_manager.cpp`) in
  `.rodata_custom_desc`, which must sit directly behind `esp_app_desc_t` (image offset 288). At `-Os` and `-O2` the
  compiler folds every read of the record, nothing references it, and `--gc-sections` discarded it (the map file lists it
  under "Discarded input sections"); the IDF linker script has no `KEEP` for this section. At `-Og` it happened to survive.
  The maintenance image then found no record in the flight slot. The same would have broken the OTA checks that read
  the record of the other image. Checked by searching each `.bin` for the `WIMG` magic: offset 288 in `-Og` builds
  and the maintenance image, 135,184 (not in the header) in the `-Os`/`-O2` flight builds.
- **Fix.** `main/boot_manager.cpp`: `own_info()` now reads the record through a volatile pointer
  (`g_image_info_ref`), so it stays referenced. (The GCC `retain` attribute was tried first; this toolchain ignores
  it and the build stops on `-Werror=attributes`.) After the fix the magic is at offset 288 in the `-Os` image, and the
  round trip works.
- **Guard to keep.** After any change of optimisation level, check `WIMG` at byte 288 of `softwarev1.bin`
  (`python -c "b=open('build/softwarev1.bin','rb').read(); print(b.find(bytes.fromhex('57494d47')))"` must print 288).
  The maintenance image still builds at `-Og` and has it at 288; it was not changed. If it is ever built at `-Os` it
  needs the same check (it uses the same code).

## Other observations

- The first soak ran on the image without the image-info fix; the fix changes one cold path (a pointer read in
  `own_info()`), so I repeated HITL, the Coast reset, the Wi-Fi round trip, a 10-minute soak and a power-on boot on the
  final image rather than the full 29-minute soak.
- Two esptool connection resets on the flash port (rfc2217) made two measurements run on the previous image; both were
  caught (identical heap figures) and repeated.
- Power-on GPS still takes about 22 s to reach 25 Hz (unchanged, see the 17b report).

## Risk flags

| Setting kept | What could go wrong | Rating | What I measured to rule it out |
|---|---|---|---|
| 32 KB icache | 16 KB less heap: allocation failure or hang after a long run or a burst of allocations | low | Heap went **up** overall because of the 16 KB dcache: 42.6 KB free, 38.3 KB minimum after 29 + 10 min soaks, HITL, two warm resets (software and panic), two Wi-Fi round trips and two power-on boots; it never moved between the first and last `cpu` line. |
| 16 KB dcache | Slower flash rodata reads, or a problem if PSRAM or large constant tables are added later | low | No change in load (27.6 % against 29.4 % with the 32 KB cache, within run-to-run spread). There is no PSRAM on this board. Revisit if PSRAM is ever added. |
| `-Os` | Stack overflow from different inlining and spills; timing or race bugs that `-Og` hid; assertion behaviour | low (stack, races) | Stack free minimum of every task is equal to or above the default build (sensors 6072 against 5980-6168, the smallest, sensor_log 1068, unchanged). HITL flight, both warm-reset modes, 39 min of soak, two power-on boots, 0 resets or watchdog trips. Assertions stay on (`ASSERTION_LEVEL=2`; the panic reset printed `abort() was called`). |
| `-Os` | Linker drops data that only the other image reads (the bug above) | **was high, now low** | Found on the bench, fixed, record at offset 288, Wi-Fi round trip passes. Guard in the section above. Any other section that is only read by another image or by a tool would behave the same; the other one is `.rodata_desc`, which the IDF app descriptor already keeps. |
| `-Os` | Floating-point rounding differences from different instruction selection (contraction) | low | Not measured directly: the HITL transition times matched the prediction to 0.01-0.08 s (as at `-Og`) and the attitude windows complete as before, but I did not compare the Kalman error figures against truth in this run. The host tests were built with the host compiler, not at `-Os` on the target. |
| 240 MHz | More current and heat on the USB power budget (the Pi trips when a radio transmits on an unpowered hub) | low to medium | Pi `dmesg` showed 0 over-current events through two soaks, 3 power-on boots and a radio-active HITL run at 240 MHz. I did not measure the supply current or the chip temperature: there is no meter on the bench, so the margin is unmeasured. The bench radio runs at -17 dBm; a +3 dBm radio plus 240 MHz was not run. Check the supply current once on the real battery. |
| 240 MHz | Timing of peripherals (UART, CAN, SPI clocks, GPS baud) | low | APB stays at 80 MHz, so the SPI clock, the UARTs and CAN do not change. GPS: same 88398 baud, 25 Hz, 2001 GGA by 100 s. CAN to Iris: `iris_can` task healthy, Iris status unchanged. IMU 936.2 Hz, FIFO overruns 0. |
| Code in IRAM | Not used, so no flash-cache-off crash risk | none | `CONFIG_SPI_MASTER_IN_IRAM` and `IRAM_ATTR` were not kept. IRAM text is the stock 59 KB. |
| Load measured with the printer running | Real worst case is a little higher | low | Soak 2 (printer off) is the heavier case and is the one to quote: 25.2 % and 5.5 overruns/s. |
| A stale build directory | An old `sdkconfig` in an existing build directory ignores the new `sdkconfig.flight` | low | I deleted the build directory's `sdkconfig` before the build. Do the same, or do a fresh build, on any other machine. |

No setting is rated high.

## Code and configuration changes (uncommitted)

| File | Change |
|---|---|
| `sdkconfig.flight` | Appended: `-Os` (`COMPILER_OPTIMIZATION_SIZE`, debug off), 32 KB icache, 16 KB dcache, 240 MHz. Assertions unchanged. The maintenance image (`sdkconfig.maint`) and `sdkconfig.defaults` were not changed: the maintenance image keeps `-Og`, the 16 KB icache, 32 KB dcache and 160 MHz, and its Wi-Fi heap (190 KB free in the log) is unaffected. |
| `main/boot_manager.cpp` | `own_info()` reads `g_image_info` through a volatile pointer so the image-info record survives `--gc-sections` (see the bug section). |

Earlier uncommitted changes from the 17b run are untouched.

## State the bench was left in

- Witness: flight image (flight_a, 424,816 bytes) built from `sdkconfig.flight` with the new settings, plus the unchanged
  maintenance image in maint_a; otadata re-written to boot flight. Pad Idle, GPS 25 Hz (`retries 1`, normal after a power-on), HITL off, attitude valid.
  Iris loads off. Aperture running. `bench_ports`: Witness, Iris, Aperture on the usual ports.
- The Wi-Fi AP was up for about 4 minutes (the stuck recovery state) before I re-flashed; it is off now.
- Log store: cleared after the last warm-reset test; the current session is a Pad Idle session.
- **uhubctl cycles used in this run: 2** (power-on boots with the final image, more than 4 min apart; 8 in the whole day).
  Port order unchanged each time.
- No HITL session, streamer or other process of mine is running and no ser2net port is held.
- Scratch in `%TEMP%\wbench3` (`meas.py`, `fm.ps1`, `w2.py`, `*_witness.log/.raw`); experiment build directories
  `wc_*`, `wd_*` and the source snapshot `wtree` in `%TEMP%` can be deleted.
