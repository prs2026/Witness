# 07 Iris fixes (cameras must stay on)

Scope: Software/Iris/softwarev1IRIS plus Witness iris_load_tracker.h and its test. Nothing committed, flashed or run on hardware. Everything marked "bench" below is untested on a board.

## Status

| Finding | Status | Summary |
|---|---|---|
| I1 loads off after cold reset | Fixed in code (bench test needed) | Commanded mask kept in NVS and RTC; restored at every reset type. |
| I2 warm reset power-cycles cameras | Partly: fix is a pad hold, unverified on hardware | GPIO9 held high through CPU/system resets so the expander is never reset. Power loss / brownout of the 3V3 rail cannot be fixed in firmware. |
| I3 main stack | Fixed | Object moved to .bss, stack 3584 to 6144 B, high-water log. |
| I6 TWAI slot leak | Fixed (defensive) | Stuck-slot watchdog rebuilds the node. Witness driver verified, not changed. |
| Medium: expander-only reset / I2C stuck | Fixed | 100 ms readback heal, bus recovery, immediate re-apply. |
| Medium: EXT/USB/UNKNOWN count as cold | Fixed (follows from I1) | No reset class selects "off" any more. |
| Witness tracker | Fixed | Backoff added; empty tracker sends nothing (verified by test). |
| Low: USB blocking writes | Fixed | Dropped like a disconnected host. |
| Audit #10 command lost when expander not ready | Fixed as a side effect | Command is adopted and stored; main-loop retry applies the newest commanded mask. |

## What changed

- `main/load_state_logic.h`: `decide_boot(rtc, nvs)` has no reset class: valid RTC record wins, else valid NVS record, else all off. Added `CommitState`/`commit_due` (deferred write rule).
- `main/load_state.cpp/.h`: `begin_boot()` uses RTC if valid (no flash access, fast path); otherwise opens NVS (erases it only if unusable) and reads namespace `iris`, key `loads` (12-byte magic+mask+CRC32 blob). `store()` updates RTC at once; `service()` (main loop) opens NVS lazily 2 s after a warm boot and commits a changed mask 200 ms after the last change (write only on change, retry in 5 s on failure, never blind-write before reading). `commanded_mask()` is the newest commanded state.
- `main/mcp23008.cpp/.h`: `initialize()` calls `gpio_hold_en(GPIO9)` once the pad is driven high, and `gpio_hold_dis` before reconfiguring (level latched first, so no glitch). New `verify_and_heal()` (reads IODIR/OLAT, re-pushes the cache on mismatch, never touches the cache on an I2C error) and `recover_bus()` (`i2c_master_bus_reset`: controller reset plus SCL clocking).
- `main/usb_serial_echo.cpp/.h`: 100 ms expander readback, recovery via `recover_i2c()` (bus clear then `resync()` of the cache, never a default), PAC1931 failures also trigger it (`recovery_logic.h`, spacing 500 ms, threshold 2 failures). Load commands are stored even if the expander is not configured. USB writes: skipped when no host (`usb_serial_jtag_is_connected`), and one timed-out write suspends USB output for 1 s. 512 B poll buffers now members.
- `main/main.cpp`: `Mcp23008`, `Pac1931`, `UsbSerialEcho` are static. Retry path applies `load_state::commanded_mask()` and clears the bus on repeated failure. Calls `load_state::service()`. Stack high-water: kept in RTC (min free), printed at next boot, and logged once a minute for the first 10 minutes only (logs are disabled after init to protect the binary USB stream; a permanent per-minute line would corrupt it).
- `main/twai_driver.cpp/.h`: transmit-progress clock; `poll()` rebuilds the node (shutdown, clear slots, initialize) if all 8 slots stay in use for 1 s with no tx-done. A failed rebuild is retried every second.
- `main/recovery_logic.h` (new), `tests/recovery_logic_test.cpp` (new), `tests/load_state_test.cpp` (rewritten).
- `main/CMakeLists.txt`: `nvs_flash` added. `sdkconfig.defaults` and `sdkconfig`: `CONFIG_ESP_MAIN_TASK_STACK_SIZE=6144` (and the legacy alias).
- Witness `main/iris_load_tracker.h`: mismatch re-send interval doubles 2, 4, 8, 16 s up to 30 s, reset by a match, a new command or an Iris reboot. It never gives up (a camera left off must keep being retried), it just stops running at full rate. `tests/iris_load_tracker_test.cpp`: updated and extended.

## Safety review: can restoring a load be dangerous?

Searched hardware.h, FIRMWARE_AGENT_GUIDE.md and the Witness docs: OUT1..OUT3 are generic switched loads with PAC1931 current monitors, named "camera" in all docs; the 5 V enable is a regulator. Nothing in the repo describes an e-match, pyro or deployment channel on Iris, so all four channels are restored. Please confirm against the schematic that none of OUT1..3/5V feeds anything that must not energise on power-up. If one does, exclude it by masking it out of `kMaskBits`-based restore in `decide_boot`.

## Reset type -> what happens to each load

| Reset | Before | After |
|---|---|---|
| Warm (SW, panic, INT/TASK/RWDT, lockup) | GPIO9 released, R55 resets expander: all loads off from reset until app_main restore, then restored from RTC. Hundreds of ms off (bootloader plus app start; not measured). | GPIO9 held high through the reset (in theory the expander is never reset): loads should stay powered, estimated gap 0. If the hold does not survive on silicon, behaviour is the same as before (RTC restore at app_main start). Bench test needed. |
| Brownout | Expander loses 3V3 too; loads off, RTC restore if record survived. | Same physical gap; restored from RTC or NVS whichever is valid. |
| Power-on / power cycle | All off, forever, unless Witness re-sends. | All off during ROM+bootloader+NVS read (est. 300-500 ms), then restored from NVS to last commanded state. |
| EXT (EN pin), USB, UNKNOWN | All off (counted cold). | RTC record valid (RTC RAM retained): restored; GPIO9 hold normally cleared by the chip reset, so the expander resets and the gap is as for power-on without the NVS read. |
| Corrupt RTC, valid NVS | All off. | Restored from NVS. |
| Both invalid / first ever boot | All off. | All off (expander reset pulsed). |
| Expander-only reset, I2C glitch | Off up to 1 s until blind resync. | Detected by 100 ms readback: off for at most about 100 ms. |
| Stuck I2C bus | Every transaction 20 ms timeout, resync never succeeds. | Bus cleared after 2 failures (500 ms spacing), cache re-pushed immediately. |

What cannot be fixed in firmware: the R55 pulldown on GPIO9/RESET (power on, EN reset, anything that clears the RTC domain) and the EN pulldowns of the load switches. A hardware fix (pull-up on RESET, or a supervisor that keeps the expander alive) would be needed to remove the power-on/EXT gap.

## Item 3: stack numbers

- Before: `UsbSerialEcho` was a local of app_main on a 3584 B stack. Its size in the new build is 0x0DC8 = 3528 B including the two 512 B buffers now added (about 2.5 KB before), plus a 1 KB of poll buffers in `poll()`/`poll_uart1()`, plus frames and ESP_LOG formatting: estimated 3.5-4 KB against 3584 B, i.e. at or over the limit.
- After: object, expander and PAC are in .bss (not heap). Deepest measured frames in the image are 128 B (poll_can), 112 B (initialize), 96 B; estimated stack use is 1-1.5 KB with ESP_LOG formatting. Stack is now 6144 B. The real high-water mark has to be read on a board (log lines in the first 10 minutes, minimum printed at next boot).
- .bss grows by about 3.5 KB; image 266,400 B.

## Item 4: TWAI

IDF source (esp_twai_onchip.c) shows frames are kept in the driver queue across bus-off and restarted after recovery (tx-done fires then), so a permanent leak is unlikely, but a stuck state (recovery never completing, a lost completion) would still starve all 8 slots. The new watchdog rebuilds the node in that case (about 1 s). Witness's driver has the same structure: recovery is the same, and `suspend()` clears its slots. I verified by reading and did not change it. It could take the same watchdog if desired.

## Tests

- Host: `iris_load_tracker_test` (Witness, g++ -std=c++17) passed; `load_state_test` passed (restore from RTC or NVS regardless of reset type, corruption cases, deferred commit rule); `recovery_logic_test` passed.
- Firmware: ESP-IDF 6.1 build of Iris into `C:\Users\E\AppData\Local\Temp\wf7`: success, `softwarev1.bin` 266,400 B, text 259,046 B, bss 252,992 B. No errors reported.
- Not run: anything on hardware. In particular untested: GPIO hold surviving each reset type, NVS write timing, power-on restore delay.

## Bench tests to run

1. Command OUT1 on, `esp_restart` (or panic) while measuring OUT1: does it stay on with no dip (hold works)?
2. Command ON, power-cycle Iris: loads come back ON about half a second after power.
3. Command OFF, power-cycle: stays off (clears the stored state).
4. Short SDA to ground briefly: recovery log, loads re-applied.
5. Watch `main stack high-water` lines for the first 10 minutes.

## Trickle-down effects

- Loads now power up ON after a power cycle if last commanded ON. At pack-down or bench work, send OFF (or pull the load) before disconnecting; the stored state otherwise brings cameras on at the next power-up, also on launch day. The Iris USB port can always turn a load off (Iris has no flight-state knowledge, by design); flight-state gating is the Witness command agent's job.
- NVS is used (default 24 KB `nvs` partition of the single-app table): one small blob, written only when the mask changes, negligible wear. A flash write briefly stalls the main loop (and interrupts not in IRAM) for a few ms to tens of ms; only happens after a command.
- Flashing/reset over USB no longer drops the loads if the hold works (they keep their state while the new image boots).
- During flashing or a crash loop the loads are intentionally left powered.
- Witness: after a Witness reset the tracker is empty and sends nothing, so it cannot override what Iris restored; it still enforces its last forwarded command after an Iris reboot, which equals what Iris stored. A load changed directly on Iris USB while Witness has a different commanded state will be reverted by Witness after about 2.5 s (existing behaviour).
- Docs to update (not mine): FIRMWARE_AGENT_GUIDE.md "Reset recovery (C4)" and "MCP init failure is fatal" text (stale), COMMANDS_AND_PACKETS.md note on Witness re-send.
- Stack log lines appear on the Iris USB stream for the first 10 minutes after boot (text between packets); Horizon should tolerate them (see audit H6).
- No change to comms.h, packet formats or Witness-side behaviour besides the re-send backoff.

## Needed outside my files

None required. Optional: the command agent's ack/refusal work stands independently; docs above.

## Follow-up 2026-11-16

Owner decision (2026-11-16): "None of the Iris outputs must be energized at power up, but retaining the power through soft resets is good." This supersedes the "restore at every reset type" design above (the earlier Status/Safety/"Trickle-down" text about loads coming back ON after a power cycle is no longer true). Nothing committed, flashed or run on hardware.

### What changed (all under Software/Iris/softwarev1IRIS)

- `main/load_state_logic.h`: `decide_boot(ResetClass, rtc)` restores only if the reset class is warm AND the RTC record is valid; otherwise all off. Removed the NVS record path, `RestoreSource`, `decide_boot_rtc_only`, `CommitState`/`commit_due`. New `record_for_commanded(mask)`: mask 0 gives a zeroed (invalid) record, i.e. an "all off" command clears the record.
- `main/load_state.cpp`: `classify()` maps the reset reason (warm: SW, PANIC, INT_WDT, TASK_WDT, WDT, BROWNOUT; everything else cold). `begin_boot()` no longer touches NVS or flash. `store()` writes the RTC record only (cleared when all off). `service()` now only erases the stale NVS key `iris/loads` once (3 s after boot, off the restore path, retried every 30 s on failure); nothing is read or written otherwise. `nvs_flash` stays in `main/CMakeLists.txt` for that erase.
- `main/load_state.h`, `main/main.cpp`, `main/usb_serial_echo.cpp`: comments updated. `main.cpp` already pulses the expander reset when `boot.restore` is false (cold), and skips the pulse (GPIO9 hold intact) on a warm restore. `mcp23008.cpp` GPIO9 hold unchanged.
- `tests/load_state_test.cpp` rewritten; `FIRMWARE_AGENT_GUIDE.md` (reset section, source map, header, trap 12: outputs always off after a power cycle, "send OFF before pack-down" removed).

### Brownout decision and RTC record strength

Brownout is treated as warm, restore only with a valid RTC record. In flight a battery sag at ignition leaves RTC RAM intact, so the cameras stay commanded on (the point of the retention); on a bench power-up that ends in a brownout reset, or after any real power-off, RTC RAM is lost and reads back random, so the record is invalid and the outputs stay off. A valid record needs 32-bit magic, 28 zero mask bits and a matching CRC32 by chance: about 2^-92 per boot. Cold reset classes (power-on, EXT, USB) additionally ignore the record outright, so even a retained record (EXT/USB keep RTC RAM) cannot energise anything. The record is cleared (zeroed) whenever all loads are commanded off, and after a cold boot it is rewritten with the all-off state.

### Reset type -> what happens to each output (OUT1, OUT2, OUT3, 5 V)

| Reset | Outputs |
|---|---|
| Power-on | All four OFF until commanded (RTC garbage, ignored; expander reset pulsed) |
| EN/EXT pin, USB/JTAG, deep sleep, unknown, eFuse, glitch | All four OFF (RTC record, if any, ignored; GPIO9 hold normally cleared by the chip reset) |
| Software restart, panic, INT/TASK/other WDT | Each output keeps its commanded state; GPIO9 held so no glitch expected (bench unverified; fallback: RTC restore at app_main start) |
| Brownout, valid RTC record | Restored to the commanded state (gap: the physical supply dip, not firmware) |
| Brownout or warm reset, record missing/corrupt, or last command was all off | All four OFF |
| Expander-only reset / I2C glitch | Re-pushed from the cache within about 100 ms (unchanged) |

### Tests and build

- Host: `load_state_test` (g++ -std=c++17 -Wall -Wextra) passed: cold + any record (all 16 masks, garbage, none) -> off; warm + valid -> restored; warm + corrupt (every single-bit flip of magic/mask/CRC, out-of-range bits, garbage) -> off; "all off" clears the record. `recovery_logic_test` passed.
- Firmware: ESP-IDF 6.1 build into `C:\Users\E\AppData\Local\Temp\wg2\b`: success, `softwarev1.bin` 265,760 B.
- Not run on hardware: warm-reset hold, brownout behaviour, stale-key erase on a board that ran the previous build.

### Consequence for audit finding I1

I1 is only partly fixed now: a cold reset of Iris alone in flight leaves the cameras (and 5 V) off until Witness re-sends the commanded states over CAN from its RAM tracker (`iris_load_tracker.h`, unchanged; its backoff caps at 30 s). If Witness reset too (empty tracker), the outputs stay off for the rest of the flight until a new command arrives. Warm resets (panic, watchdog, software, brownout with a valid record) are covered by the RTC retention. Operationally, outputs are always off after a power cycle, so no OFF is needed before pack-down.
