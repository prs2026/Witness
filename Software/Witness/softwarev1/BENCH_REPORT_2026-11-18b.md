# Bench report 2026-11-18b: BIT (merge `ea64c69` plus working tree) on the bench Witness

**Image:** flight image from the working tree on `ea64c69`, flashed with `flash_verified.ps1` after `-CheckOnly` (ESP32-S3
`softwarev1`, flight image). Bootloader, partition table, `otadata` (initial), flight at `flight_a` 0x20000 and maintenance at
`maint_a` 0x420000 were written and hash-verified, so the new image is the one that boots (`otadata` reset to the first OTA
slot; WIMG record at byte 288 checked: kind 1 = flight). Radio at -17 dBm, GPS without sky signal, HITL via the 710x port only.

**One source change was needed and is in the working tree (not committed):** `FlashLogger::run()` never set `session_ready_`
after a normal start (it was only set in `clear_session_headers()`). Result with the first image (flashed as merged): BIT check
6.1 "log session" FAILED (blocking) on every boot, `core done` never became true, the commanded NAND test (5.2) and the
end-to-end test (6.3) returned ERROR (value 255). One line was added after the init loop (`session_ready_ = !slots_full_;`,
`main/flash_logger.cpp`), the flight image was rebuilt (447,248 bytes, WIMG unchanged), flashed again, and everything below is
from that second image except where it says "first image".

## Results

| # | Item | Result | Evidence |
|---|---|---|---|
| 0 | Flash and slot | **PASS** | target verified as Witness first; hashes verified; image-info valid; boots from the flashed slot (otadata rewritten) |
| 1 | Boot, startup BIT | **PASS** (after the one-line fix) | cold boot (`last reset USB`): startup BIT verdict logged at 46 s, **NO-GO [bench condition]**, blocking group 0x0100 (radio only), first failing check 8.3 "radio power" value 65519 (-17 dBm as a signed 16-bit value), shown with the bench flag. Packet 0x0B decoded: verdict NO-GO, bench=1, kind 0 (startup), core done 1, settled 1, blocking mask 0x0100, degraded 0x0896, not-evaluated 0. Degraded (all expected on the bench): 1.2 task watchdog (value 4), 1.3 nvs (2), 2.8 body orientation (board lies flat), 4.6 calibration, 4.7 pad heading, 7.4 GPS fix, 7.5 UTC, 11.1 battery (5.4 V supply), 12.4 time of day. So the bench radio is classified as a **blocking NO-GO flagged "bench condition"**, never GO. First image: NO-GO with bench=0 (see the one-line fix above) |
| 1 | Heap and core-1 load | **PASS, with a cost** | heap free 34,904 B, minimum 30,744 B (before BIT: about 41.3 KB and 37 KB, so about 6.4 KB less; limit in the plan is D below 20 KB minimum). Core 1 25.1 to 26.3 % in Pad Idle (before: about 25 %); overruns about 2.9 per second (before about 2.6 to 3); max pass 3.9 to 5.2 ms (before 3.7 to 4.7 ms). New `bit` task: 0.9 % on core 0, stack 1992 B free of its allocation. All other stack margins as before (flash_logger 3.4 KB, uart1 1588 B) |
| 2 | Commanded run `0x5442` and `0x5452` in Pad Idle | **PARTLY PASS** | `0x5442` d0=0: ack OK, detail 0x0402 (NO-GO, 2 blocking groups), arrives about 1.4 s after the command. Summary kind 1 (commanded), active tests ran. `0x5452` d0=0xFF, 0, 1, 2: ack OK each, 0x0B and 0x10 pages 0, 1, 2 arrive and decode (check ID, result, class, bench flag, value per record). NAND write test 5.2 passes: block 1023 erase about 3.7 to 3.9 ms, program about 2.8 to 3.1 ms. **FAIL: 6.3 log end-to-end** reports FAIL (value 4 = marker not found in the pages read back after the flush) in 3 of 3 runs without HITL and in 1 of 4 runs with HITL active (the other 3 passed, latency not read). It is a blocking class check, so every commanded run without HITL ends NO-GO because of it even though the NAND write test passes. Cause not found (not a hardware fault: 5.2 passes and logging works). `0x5442` d0=1 (passive) works: ack OK, kind 2 |
| 3 | Fault injection `0x4246`, NO-GO and recovery | **PASS** | gate: with HITL off both ON and CLEAR are acked REFUSED and `BIT fault injection refused (HITL not active)` is logged. With HITL active (Pad abort preset, 60 s on the pad): fault 3 (IMU silent, 25 s) acked OK, `BIT fault 3 ON for 25000 ms`; the next commanded run gave NO-GO with blocking groups 0x0116 (supervision, IMU, attitude, radio), failing checks 2.1 imu alive (7), 2.2 imu rate (0), 4.3 launch readiness (5), plus the two bench checks; ack detail 0x0404 (4 blocking groups) and **the bench flag cleared (bench=0)**, so the ground would show a real NO-GO. After `0x4246` d0=0xFF d1=0: ack OK, next run back to blocking 0x0102 and bench=1 (**NO-GO [bench condition]**: HITL active 1.4 and radio power only). The pad abort run itself passed (no transitions, as expected). **Cannot return to GO on this bench**: the best reachable result is NO-GO flagged bench (HITL on) or NO-GO bench (radio -17 dBm). I did not raise the radio power |
| 3 | LED selects 4 pulses on NO-GO | **PARTLY OBSERVED** | by code (`heartbeat.cpp`, `bit_policy.h`): 4 pulses when the published verdict is NO-GO (bench-flagged or not) in Pad Idle, and a NOT LOGGING state (3 pulses) outranks it. The firmware has **no log line per pulse or for the BIT LED choice**, so the pulse count itself was **not observed**; what was observed is the verdict that feeds it (NO-GO in every pad state above). So on this bench the red LED would show 4 pulses all the time because of the radio at -17 dBm. The LED itself and the buzzer (compiled off) were not seen or heard |
| 4 | Warm reset | **PASS** | Quick hop preset, `0x4252` (software) at sim 8 s in Coast: ack OK; boot log: `RESUMING IN-FLIGHT STATE 2`, `resumed in flight: NAND start-up test skipped`, `RESUMED ... appending to flight session 234 in slot A` (same session, streams continue at their pages), `bit: reset class 3`. Startup BIT after it was passive (0x0B kind 2, no NAND test line, no `BIT NAND write test` line), verdict RUNNING at 3.3 s then GO WITH WARNINGS at 5.4 s (degraded 0x0802). Core 1 22.5 % afterwards |
| 5 | HITL Nominal 10 g, BIT must not disturb | **PASS** | Witness states from its log: launch at sim 10.30 s, burnout 12.50, descent 32.41, landed 148.21 (reference 10.30, 12.49, 32.38, 148.19, so deltas 0.01 to 0.03 s). `0x5442` on the pad (sim 4 s) OK; `0x5442` in Boost: **REFUSED** (`BIT run (USB) refused`), ack detail 0x0401 (the doc says 0x8000 + reason for gate refusals; the ack carries the verdict instead, a small doc mismatch); `0x5452` in Coast: OK. Flash log: flight marked in slot B, no logger errors, no NOT LOGGING, no watchdog or fault lines, overruns 102 at start, 134 at the end of the flight (low). BIT only reports: verdict NO-GO bench the whole flight, no effect on state detection. The HITL tool printed "board reset seen at sim 5.4 s" and marked the later states AFTER RESET (not scored by the tool). The board did **not** reset (continuous uptime, no boot lines); it is probably the tool misreading uptime after the new 0x0B/0x10/ack frames. Not investigated |
| 6 | End state | done, with one note | Pad Idle, HITL off, port 7100 free (my connections closed). **Both NAND slots now hold a flight** (Quick hop and Nominal): the startup BIT correctly reports 6.1 "log session" FAIL value 2 (slots full) and the board is NOT LOGGING until the store is erased (`0x6869`). I did not erase anything (never-erase policy; it is your call) |

## Not observed or not done

- The red LED pulse count and the buzzer (compiled off): no observation is possible remotely and there is no log line for it.
- GO and GO-with-warnings in Pad Idle: not reachable here (radio at -17 dBm is a blocking bench NO-GO). GO WITH WARNINGS was seen only after the warm reset in Coast, because the radio check is not judged outside the ground states.
- Radio-source refusal of `0x4246` and `0x5442` over LoRa: not tested (USB only).
- First-boot lines of the second image up to about 1 s were not captured by the USB reader; the startup verdict and all BIT lines were.
- No soak, no long runs.

## Bench notes

- esptool resets through the 700x port leave `last reset PANIC` on this board (BIT 0.1 "reset reason" then FAILs as degraded value 4, and BIT counts it as a fault reset); a flash hard reset shows `last reset USB`. The 700x port cannot be used while a 710x client is connected.
- One hub power cycle was not possible (no SSH access from this machine); none was used.

## Follow-up: erase, 6.3 fix, HITL reset detection

| # | Item | Result | Evidence |
|---|---|---|---|
| 1 | Erase the log store (`0x6869`, USB, once, owner-approved) | **PASS** | ack OK 0.1 s after the command; `all stored sessions erased (headers 33 ms ...); new session 237 created`; `logging to flash again: red LED back to one pulse a second`; status has no log-failed or slots-full bit. **Found by it: BIT 6.1 stayed FAIL (value 2, slots full) after the erase** because `choice_action_` kept the start-up value `SlotsFull` (6.1 reads it). Fixed (below); 6.1 passes after the reflash (blocking groups then 0x0100 = radio only, bench) |
| 2 | BIT 6.3 end-to-end | **PASS after fix** | before: FAIL value 4 in 5 of 5 commanded runs after the erase (the first run on a fresh session passed). After: **8 of 8 PASS with HITL off** (1 after flash, 6 in a row, 1 after the HITL flights) and **3 of 3 PASS with HITL on** (Pad abort preset, sim 6, 14 and 22 s). Run time about 1 s (flush interval). Only 8.3 radio (and 1.4 HITL, with HITL on) remain as bench-flagged blocking failures |
| 3 | HITL "board reset seen" | **PASS after fix** | Nominal 10 g with `0x5442` at sim 4 s, no log fetch: PASS, transitions 0.02 / 0.02 / 0.03 / 0.02 s, **no reset warning** (before: false reset at sim 5.4 s). Pad abort run with three `0x5442`: no warning; `0x4252` at sim 32.07 s: **reset reported at sim 32.5 s** (uptime path; the boot lines follow) |

**Root cause 6.3.** The marker was looked for in the last 3 pages of the packet stream. It is queued behind older traffic and flushed with all traffic after it
(about 2.6 pages per second at the 50 Hz packet rate), so with one flush interval of traffic after it, it is 4 or more pages from the end and the check
reported "not found" (value 4), independent of NAND health. HITL on varies the traffic, hence the occasional pass. Fix: the scan starts at the page the writer was on when
the marker was queued, walks forward (up to 40 pages, around the ring), matches across a page boundary, and skips unreadable pages instead of
failing on the first (a read error is only reported if the marker is not found). A failure now logs the page range scanned.

**Root cause HITL.** `hitl_protocol.StreamDecoder` did not know the BIT frames `0x0B` (14 bytes, sent about once a second) and `0x10` (101 bytes). Its data
bytes were scanned for other frame IDs: the `0x01` inside the summary started a bogus 23-byte "sensors" frame that ended on a `0x0A` of the next real frame, with a wild
uptime (167,995,136 ms in the capture) every other second. The next real packet then looked like an uptime drop of more than 2 s: a reset. Fix: both frames are now
known (length, name, short decode), so they are consumed as frames. The detector was also made robust (`ResetDetector`): a drop needs a second
packet within 5 s above the new value, a stray value in either direction is ignored and never becomes the baseline, the RESUMED bit needs two consecutive packets; the sim time of the first packet is kept.

### Code changes (uncommitted)

| File | Change |
|---|---|
| `main/bit_marker_scan.h` (new) | pure scan logic: pages to scan, needle match across page boundaries |
| `main/flash_logger_bit.cpp`, `main/flash_logger.h` | 6.3 phase B uses it; `bit_marker_start_page_` recorded in phase A; failure log line |
| `main/flash_logger.cpp` | `clear_session_headers()` resets `choice_action_` to NewSession (BIT 6.1 after an erase) |
| `tests/bit_marker_scan_test.cpp` (new) | marker anywhere in a 60 record stream (more than 3 pages from the end), split at every needle offset, not found, wrong nonce, gap, ring wrap and cap |
| `HITL/hitl_protocol.py` | frames `0x0B` and `0x10` known |
| `HITL/hitl_session.py` | `ResetDetector`, used by `_on_status` |
| `HITL/tests/test_reset_in_run.py` | 10 new tests: summary and detail frames next to sensor frames (fail with the old decoder: it returned one bogus "sensors" frame), detector cases, session case |

Tests: `bit_marker_scan_test` builds with `g++ -std=c++17 -Wall -Wextra -Werror` and passes; HITL suite (`python -m unittest discover -s tests`): 111 tests OK.
Images: flight 447,776 bytes and maintenance 851,728 bytes, `WIMG` at byte 288 in both (kinds 1 and 2); flashed with `flash_verified.ps1` after `-CheckOnly`, hashes verified.

### Open finding (not investigated, not caused by these changes as far as seen)

After the post-flight `0x7300` (Landed to Pad Idle) the board came up, then **panicked about 0.5 s after boot** (`Guru Meditation Error: Core 0 panic'ed (Cache error)`, backtrace
in `CanLink::task_entry`, `can_link.cpp:62`, via the interrupt entry) and rebooted (second boot: `last reset PANIC`). It followed an `E spi_common: spicommon_dma_setup_priv_buffer: sync failed`
line at 0.26 s of the first boot. One occurrence in 5 warm or software resets of this session (`0x4252` x1, `0x7300` x1, flash resets); the other resets were clean.
Worth a look (an ISR running while the cache is off, or the BIT start-up path). The board is running normally now; BIT 0.1 shows the PANIC reset as degraded.

### End state

Pad Idle, HITL off, logging (session in slot B; slot A holds the Nominal flight, so one more flight fills the store again), no log-failed or slots-full bit, port 7100 free
(my connections closed), last reset reason PANIC (see above).

## Follow-up: boot cache-error panic

**Result: root cause found and fixed (configuration), reproduced before, not reproduced after.**

**Reproduction.** Software restart `0x4252` (HITL active, from Pad Idle) over USB, 13 restarts on the image under test (flight image
447,776 bytes): **2 panics** (restarts 3 and 12 of the 12-restart run; both identical: `Guru Meditation Error: Core 0 panic'ed (Cache
error)`, `Cache disabled but cached memory region accessed`, PC in `_frxt_int_enter` (`portasm.S:160`, the store of the interrupted
task's stack pointer into its task control block), entered from `_xt_lowint1`, interrupted task `CanLink::task_entry`, about 0.5 s
after boot, right after the BIT reset-record line (an NVS write, so the flash cache is off). The `sync failed` line appeared on **every**
boot (not only the bad one), at 0.26 s, with the text `esp_cache_msync: start address 0x3c0036a4, size 0x4 is not aligned with cache line
size`, and each time the first IMU and baro reads of the boot failed with `ESP_ERR_INVALID_ARG`.

**Root cause.** The flight sdkconfig uses a 16 KB data cache. On ESP32-S3 that hands 16 KB of cache SRAM to the heap at **0x3C000000**
(boot log: `At 3C000000 len 4000 (16 KiB) DRAM`). That range is reached through the cache: it is cacheable address space. Two symptoms
followed. (1) The heap allocator placed task stacks and task control blocks there (the failing SPI receive buffer was a stack variable at
0x3c0036a4), and the SPI driver refuses a cache-range buffer that is not 32-byte aligned (the boot-time `sync failed`). (2) While a flash write
(NVS, at boot the BIT reset record) has the cache disabled, any interrupt that arrives on the core whose current task has its TCB in that
range faults in the interrupt entry itself (it stores the stack pointer into the TCB), whatever the ISR is, IRAM or not. It is a race
between a boot-time NVS write and any level-1 interrupt, hence about 15 % of restarts. The earlier suspicion (TWAI ISR, BIT probes, flush
handler) is ruled out: the TWAI ISR is allocated non-IRAM and is masked during flash writes, and the fault address is the kernel entry code.

**Fix.** `sdkconfig.flight`: `CONFIG_ESP32S3_DATA_CACHE_32KB=y` (was 16 KB), so the 0x3C000000 heap region no longer exists; the comment in that
file explains why. No source change, no new command. Flight image rebuilt (447,664 bytes), maintenance image unchanged in size (851,728; it already
used the 32 KB default), `WIMG` at byte 288 in both, flashed with `flash_verified.ps1` after `-CheckOnly`, hashes verified.

**After the fix.** 15 software restarts (1 + 14): **0 panics**, 0 `sync failed` lines, no failed first IMU or baro read; boot log shows only the
`3FCF0000` 32 KB DRAM region plus the normal DIRAM regions. No other test was run on the new image beyond restarts and boot logs (no HITL flight).

**Cost to decide on.** The 16 KB that the 16 KB data cache used to give back is gone: heap free now **19,300 B, minimum 15,156 B** (before 34,904 and 30,744).
BIT check 0.3 "heap" therefore reports **degraded (class 1, value 947 = minimum/16)** because its proposed degraded limit is 20 KB (blocking limit
6 KB is not near). Options if that is unwanted: lower `kHeapMinDegradedBytes` in `main/bit_limits.h` (marked PROPOSED), or recover heap
by trimming task stacks (stack margins are 1.2 to 6 KB). I changed neither. Core 1 load after the change is 24.5 %, in line with before.

**Not done.** The 3-restart-in-a-row and Landed `0x7300` paths were not repeated (same boot path). The maintenance image was not re-tested.

**State.** Pad Idle, HITL off, logging (session in slot B), port 7100 free. The log store was not erased.
