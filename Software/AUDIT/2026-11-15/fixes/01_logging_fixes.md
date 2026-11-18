# Fix report 01: NAND flight logger (audit 02)

Files changed: `main/flash_logger.cpp`, `main/flash_logger.h`, `main/w25n01kv.cpp`, `main/w25n01kv.h`; new `main/log_slot_logic.h` (pure logic) and `tests/log_slot_logic_test.cpp`. Nothing committed, flashed or run on hardware. The on-NAND format is unchanged (see Format).

## Status per finding

| Finding | Status |
|---|---|
| C1 / W2 reset in flight stops logging | fixed (host-tested logic; not exercised on hardware) |
| C2 torn header/marker bricks init | fixed |
| H1 no bad-block handling | fixed in the writer; reader skips by content (see below) |
| H2 one NAND error kills the task | fixed |
| H3 one unreadable page fails the export mount | fixed |
| M1 capture stack 3072 | fixed (5120) |
| M3 erase with no state re-check | fixed (re-check at execution and during the erase) |
| M4 erase caller waits forever | fixed (30 s timeout) |
| L2 erase_quick_ race | fixed as a side effect (single atomic request word) |

## C1: resume appends to the flight slot

- `log_slot_logic.h: choose()`: pure slot decision. Resumed boot: append to the NEWEST valid slot if it is store v4 (works for both slots flights, one flight, or marker not yet written). Otherwise the old rule: pad boot never uses a flight slot; both flights and a pad boot gives SlotsFull; else the empty/older slot. Slots whose header block would not erase are avoided when the other is free.
- `flash_logger.cpp: start()` records `resumed_in_flight_` from `flight_state_machine_->state()` in Boost/Coast/Descent. main.cpp already calls `restore()` before `flash_logger.start()` (main.cpp:30 vs :68), so no new accessor and no file I do not own was needed.
- `resume_session()`: for each of the five streams finds the end with `log_slot::stream_end()` (block-first-page scan, then binary search in the newest block; about 500 page reads), continues after it (a non-erased page inside an entered block is skipped, flight pages are never erased or overwritten), restores sequence numbers, `last_erased_block`, `wrapped`, launch-protects the rings if the slot already has the flight marker, and keeps `flight_marker_written_` consistent (never re-programs the marker). Linear streams with no room are marked full as at runtime (capture of that stream off, FLASH_LOG_FAILED set). If probing fails or no stream has room, it falls back to the old never-erase rule (`choose_slot(false)`).
- Resumed marker: a packet-stream record (new source 6) plus a WARN text line. Layout below.
- Pad Idle boots: unchanged (never erase a flight, both flights gives LOG_SLOTS_FULL).

## C2 / H2 / H1: errors never end logging

- `read_slot_info()`: header page uncorrectable: slot decided from the marker (flight, header rebuilt in RAM, version probed from the first IMU page) or treated empty/dirty (erased on reuse). Marker uncorrectable: slot kept as a flight (marker is only programmed at launch). Other read errors are retried 3 times (chip reset between); if still failing the init loop retries every 2 s forever (INIT_FAILED shown while failing, cleared on success). The startup self test failing 3 times is downgraded to a warning.
- `run()` init is a retry loop; the task is never deleted. Write errors in the main loop no longer delete the task.
- `append_bytes()`: erases via `prepare_block()` (skips factory-bad and session-bad blocks, 3 attempts, chip reset on the 2nd), programs via `program_page_robust()` (3 attempts for transient errors; after a timeout waits for ready and reads back so a late-finished program is not repeated; P_FAIL is final for the page). A failing block is abandoned for this boot and the chunk is rewritten in the next block; after 4 skipped blocks in one chunk the chunk is dropped and counted (`chunks_dropped_`).
- `log_slot::WriteHealth`: FLASH_LOG_FAILED is set while blocks fail and cleared on the next good page (kept if a stream is full). After 8 consecutive failed blocks writes pause (`ESP_ERR_NOT_FINISHED`, buffers kept) and retry every 3 s forever, with a chip `recover()` at each retry.
- `create_session()`: header block that will not erase/program marks the slot unusable and the init loop takes the other slot; a stream-first block that will not erase is skipped by the writer.
- `w25n01kv`: ready timeout 20 ms to 100 ms (datasheet: read 60 us, program 700 us, erase 10 ms; the status is checked before the deadline, so a finished op is never a timeout). New `wait_idle()` and `recover()` (reset, unprotect, ECC and buffer mode, verified). `is_bad_block()` now reads the spare marker even when the page reports uncorrectable ECC: a torn good block is no longer called bad (a factory-bad block still is, by its marker; unreadable spare is bad). Page reads spin-poll 30 times before sleeping a tick (reads were 1 tick each; scans are much faster).
- Bad blocks in the reader: the layout stays fixed (stream block = first_block + page/64) and bad blocks are simply never written, so offsets cannot diverge. The reader accepts this session's pages wherever they are (tag check), so it does not need its own bad-block list and gaps are handled.

## H3: unreadable pages

`scan_file()` and `load_export_page()` count (`unreadable_page_count()`, warning at the end of the scan) and skip pages that fail ECC; same rule in both so sizes match. Tagged linear files are no longer cut at the first non-matching page: they are scanned to the end of their allocation (gaps from skipped blocks or a torn page). Untagged (pre-tag firmware) linear files keep the old stop-at-first-invalid rule. Cost: tagged linear files read the whole allocation (+8.7k page reads per slot on mount, fast with the spin-poll).

## M3 / M4 / L2

`request_erase()`: one atomic request word (0/1 full/2 clear), waits at most 30 s (`kEraseWaitMs`) when given portMAX_DELAY, cancels an unstarted request on timeout, drops a stale completion. The task refuses the erase with ESP_ERR_INVALID_STATE if the state is not Pad Idle/Landed at execution, and `erase_session_blocks()` re-checks every 16 blocks and stops (new session in slot 0 with generation continued). Erase errors no longer leave logging off: a new session is created anyway and the first error is returned.

## M1

`kCaptureTaskStackSize` 3072 to 5120: +2048 B internal heap. Not measured on hardware (no high-water data); verify with `uxTaskGetStackHighWaterMark` on a commanded Boost.

## Tests

`tests/log_slot_logic_test.cpp` (g++ -std=c++17 -Wall -Wextra): slot choice (empty chip; pad boot never uses a flight slot; both full + pad = SlotsFull; both full + resume = append newest; one flight + resume = append; resume before marker; old store version falls back; generation wrap; unusable slot), stream-end finder (linear and ring, empty, block boundary, wrap, torn last page, gap from a skipped block, stale foreign data), first_free_page, write-health back-off (8 failures, retry cycle, recovery, ms wrap). Result: all passed. The NAND-touching code (`resume_session`, writer, reader) is not host-tested (needs the chip); only its pure decisions are.

## Build

Full flight build (sdkconfig.defaults + sdkconfig.flight, scratch dir C:\Users\E\AppData\Local\Temp\wf1) was stopped by the coordinator's change of plan: flash_logger.cpp and w25n01kv.cpp compile cleanly with the project flags (-Wall -Wextra -Werror, gnu++26; re-checked after the watchdog edit). The full build failed only in main/uart_driver.cpp:472 (std::min(const uint32_t&, unsigned), not my file) on two attempts, so no link was done.

## Task watchdog (task_cores.h task_supervision)

The flash_logger task subscribes just before its main loop (not during init retries or the slots-full wait, which can exceed the timeout), feeds every pass, inside erase_session_blocks (per block), append_bytes (per page) and scan_file (every 256 pages), and unsubscribes before the permanent frozen wait. The task no longer calls vTaskDelete. If the Landed low-power mode is ever enabled its wait period must stay below the watchdog timeout. The capture task and export-only task are not subscribed.

## Format and reader notes

- No change to page headers, session header, marker, layout or tag scheme. Existing sessions read identically (their pages are contiguous, so scanning the whole allocation gives the same file).
- New packet-stream record, source 6 (`kPacketSourceLogMarker`): `[u32 us BE][u8 source=6][u8 length=6][u8 kind=1 (resumed)][u8 flight state][u32 BE session generation]`. Readers that skip unknown sources by their length byte are unaffected; Horizon's/witnessOS packet parsers should be checked for an unknown-source branch.
- After a resume marker, in the same session: the u32 us timestamps (IMU, WL, packets) restart from about 0, the IMU u8 sequence and the WL u32 sequence restart at 0. A reader that assumes monotonic time within a file must treat the marker as a discontinuity. Up to 1 s of RAM-buffered data before the reset is lost (M2, unchanged).
- Gaps: a linear file may now contain erased or torn pages in the middle; the reader skips them. A page whose program failed may in rare cases still read as a valid duplicate of the chunk rewritten later (not seen; document only).
- Resumed rings are launch-protected from the resume point, not from the original launch (the launch page is not recorded), so a ring that laps after the resume can overwrite pre-reset flight pages of that ring.

## Trickle-down effects

- Status bits: FLASH_LOG_FAILED (byte 0 bit 7) is now cleared when writes recover (before: latched). FLASH_INIT_FAILED now clears when a retry succeeds. LOG_SLOTS_FULL unchanged. A resumed flight does not set LOG_SLOTS_FULL.
- The logger task is no longer deleted on any error: `erase_all_sessions`/`clear_all_sessions` no longer return INVALID_STATE after a logger failure; they time out at 30 s at worst (ESP_ERR_TIMEOUT) and can now return INVALID_STATE for a refused erase (flight state). Callers in `usb_serial_echo.cpp` (erase/clear) need no change (they log the error); an operator may see "log erase failed: ESP_ERR_INVALID_STATE" if launched meanwhile.
- Timing: logger start after a reset now includes about 500 page probes (tens of ms, up to about 0.5 s without the spin-poll). During a NAND fault the logger task can block up to about 2.4 s in retries per chunk; the IMU queue (2 s) may drop records then.
- Heap: +2 KB (capture stack). RAM: ~60 B of new state in the logger.
- Mount/export: tagged linear files scan the full allocation; with a dead/unreadable page the maintenance mount now succeeds and logs the count.
- A resumed boot with an older store version (v3) slot falls back to the old rule.
- HITL/bench (H4), M2, M5-M8 and L1/L3-L6 are not in scope and unchanged.
- Needs from files I do not own: none required. Optional: Horizon reader should recognise packet source 6 and the timestamp restart; a buzzer/LED for LOG_SLOTS_FULL / FLASH_*_FAILED is still missing (H4).
