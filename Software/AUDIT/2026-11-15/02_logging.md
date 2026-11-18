# Audit 02: flight data recording (NAND logger)

Scope: flash_logger.cpp/.h, w25n01kv.cpp/.h, usb_mass_storage.*, flight_state_store.*, log sinks in sensors.cpp, every logger caller. Read-only, working tree as found. flash_logger.*, w25n01kv.*, usb_mass_storage.*, flight_state_store.* are NOT in the modified list (stable). sensors.cpp is being edited by another agent; the sinks at sensors.cpp:1138-1147, 1774-1821, 2111-2124 looked complete, not mid-edit. Nothing was built or run. "confirmed" = traced by reading; "plausible" = needs a bench test.

## Findings table

| # | Sev | Conf | Where | One line |
|---|-----|------|-------|----------|
| C1 | Critical | confirmed | flash_logger.cpp:345,357-358,960-977 | Mid-flight reset (brownout/WDT/panic) while ANY other flight is on the chip leaves both slots "flight" -> LOG_SLOTS_FULL; the rest of the flight is not recorded |
| C2 | Critical | plausible | flash_logger.cpp:310-328,337-339,959-989; w25n01kv.cpp:254-257 | A torn/partly erased header or marker page makes read_slot_info fail on an ECC error -> init fails on EVERY later boot, logger task deleted, erase/clear refused |
| H1 | High | plausible | flash_logger.cpp:394-423,539,549; w25n01kv.cpp:370-398 | No bad-block handling in the logging region; an erase/program failure on a slot block kills logging, deterministic at session start |
| H2 | High | confirmed (design), plausible (trigger) | flash_logger.cpp:1101-1109; w25n01kv.h:46; w25n01kv.cpp:213-236 | Any single NAND error (incl. 20 ms ready timeout under pre-emption) permanently deletes the logger task; no retry/re-init |
| H3 | High | plausible | flash_logger.cpp:267-280,368-391,710-751,775-778 | One uncorrectable page (e.g. torn by brownout) fails the whole maintenance export mount: no file of either flight can be read |
| H4 | High | confirmed | flash_logger.cpp:352,921-946,1076; flight_selftest.cpp:25-33; heartbeat.cpp:199-250 | HITL/bench/false Boost writes the flight marker and fills a slot; the only warning is one status bit (no buzzer, no self-test, no arming gate) |
| M1 | Medium | plausible | flash_logger.h:60; flash_logger.cpp:938 | Capture task has 3072 B stack and runs ESP_LOGI (via log_vprintf 256 B LogLine + console printf) at the Boost transition |
| M2 | Medium | confirmed | flash_logger.h:58; flash_logger.cpp:1081-1087 | Up to 1 s of RAM-buffered data lost on power cut |
| M3 | Medium | confirmed | flash_logger.cpp:1008-1045,441-464; usb_serial_echo.cpp:774-796 | Full erase takes seconds with capture OFF, blocks the command task, no state re-check at execution |
| M4 | Medium | confirmed | flash_logger.cpp:847-858,1101-1109 | erase/clear caller waits forever if the logger dies after the request is posted |
| M5 | Medium | confirmed | flash_logger.cpp:519-541 | Launch-protected ring erases the launch block on lap: up to 63 earliest flight pages destroyed before the stop |
| M6 | Medium | confirmed | flash_logger.cpp:594-602,544-547 | IMU stream has no per-record magic/CRC and records straddle pages; a lost page misaligns the rest |
| M7 | Medium | confirmed | flash_logger.h:184; flash_logger.cpp:507-516 | IMU stream holds ~9 min at 960 Hz; then stops silently (status bit only) |
| M8 | Medium | confirmed | flash_logger.cpp:228-248 | log_vprintf puts a 256 B LogLine + vsnprintf on every logging task's stack |
| L1 | Low | confirmed | flash_logger.cpp:128 | UART queue overflow latches FLASH_LOG_FAILED |
| L2 | Low | confirmed | flash_logger.cpp:860-866 | erase_quick_ set before CAS: racing erase becomes header-only clear |
| L3 | Low | confirmed | flash_logger.cpp:973,1004 | register_current_task twice for the writer (landed_power has 4 slots) |
| L4 | Low | confirmed | flash_logger.cpp:141,575 | u32 us timestamps wrap every 71.6 min |
| L5 | Low | confirmed | usb_mass_storage.h:19-34 | FAT volume 134 MB vs worst case ~130 MB of content: ~3 % margin |
| L6 | Low | confirmed | tests/ | No host test for logger slot selection/scan/append |

## Details

### C1. Mid-flight reset with another flight on the chip stops logging (CRITICAL, confirmed)
- scan_sessions (flash_logger.cpp:345): slots_full_ = both slots valid and both flight. The new session goes to the slot NOT holding the newest flight (357-358). run() (960-977) then turns every capture flag off, sets LOG_SLOTS_FULL and waits for an erase command.
- A slot becomes "flight" at the first Boost record (write_flight_marker 425-439, called from 1076-1077). The slot in use during a flight is therefore already a flight slot.
- Scenario: F1 (earlier flight, or HITL/bench Boost, see H4) sits in slot A, the normal state at the pad. Launch: session F2 in slot B, marker written at Boost. A brownout/WDT/panic later: flight_state_store resumes Boost/Coast/Descent (flight_state_store.h:60-66). On the resumed boot scan sees A=flight, B=flight -> nothing is logged for the rest of the flight (IMU, 50 Hz sensors, packets, text, GPS). Telemetry continues; only status byte 1 bit 7 shows it.
- With an empty chip the SECOND reset in one flight does the same (documented, HANDOFF 1581). FIRMWARE_STATE_MACHINE.md:717-725 marks M7 "fixed", but the fix trades "erase the first part" for "lose the whole remainder"; the first-reset-with-prior-flight case is not called out. Brownouts around ignition/TX are a known bench problem (memory: USB over-current on radio TX).
- Fix idea: the logger has flight_state_machine_; in run(), if the state is Boost/Coast/Descent (resumed) and slots are full, erase the OLDER flight slot and use it (a flight in progress outranks a stored one); better, resume appending to the in-flight slot by scanning stream tails.

### C2. ECC error on header/marker read bricks logging on every boot (CRITICAL, plausible)
- read_slot_info (310-328) returns the error of flash_.read (316, 324). load_page_to_cache returns ESP_ERR_INVALID_CRC for ECC status 10b (w25n01kv.cpp:254-257). scan_sessions propagates (337-339); run() treats r != ESP_OK as init failure: FLASH_INIT_FAILED, capture off, vTaskDelete (982-989).
- Pages 0 and 1 of each slot's block 0 are erased/programmed on every boot (create_session 396, 404) and at Boost (431). Power loss or brownout during that erase/program (bench power cycle, or brownout at ignition exactly during the marker program) can leave a page that reads uncorrectable. Each later boot reads the same page and fails the same way. No software recovery: erase/clear return INVALID_STATE because task_handle_ is null (849, 862). The code knows the pattern (is_bad_block treats INVALID_CRC as bad, w25n01kv.cpp:377-383; self-test skips such blocks) but read_slot_info does not.
- Fix idea: treat INVALID_CRC in read_slot_info as "slot invalid" for the header and as "flight" for a torn marker (keep the data); always erase block 0 in create_session.

### H1. No bad-block management in the logging region (HIGH, plausible)
- Layout is fixed contiguous blocks 0..1003 (file_layout 202-220). append_bytes erases/programs without is_bad_block (539, 549); create_session erases each stream-first block unchecked (396-398). Only erase_session_blocks/clear_session_headers consult the marker.
- W25N01KV parts can ship with bad blocks (the code's own comment w25n01kv.cpp:379 and the reserved-block self-test assume it); I found no bench measurement of the count. A bad block hit gives ESP_FAIL (w25n01kv.cpp:337-339, 364-366) -> logger dies mid-flight when the stream reaches it.
- At boot it is deterministic: a bad stream-first block makes create_session fail before the header is written; the slot stays invalid and the next boot picks the same slot again (359-365): FLASH_INIT_FAILED every boot.
- Erasing a bad block also wipes the factory marker, so later is_bad_block calls it good.
- Fix idea: build a 1004-entry bad-block map at boot, skip bad blocks in Writer, never erase one.

### H2. No retry or recovery after any NAND error (HIGH)
- All failures end at 1101-1109: capture off, task_handle_ = nullptr, vTaskDelete. IMU, sensor, packet, GPS and text logging stop for the rest of the power cycle.
- Non-media triggers: wait_until_ready uses kReadyTimeoutMs = 20 (w25n01kv.h:46) measured in ticks (w25n01kv.cpp:217-233); block erase max is ~10 ms. If the prio-3 logger is pre-empted >10 ms during an erase wait (e.g. by the prio-4 flash_capture burst of 100 prelaunch records + ESP_LOGI at Boost) it returns ESP_ERR_TIMEOUT though the chip is fine. write_enable's latch check (208-210) can also return INVALID_STATE.
- Program success is judged only by the P_FAIL bit; no read-back.
- Fix idea: retry 2-3 times, longer timeouts (erase ~100 ms), mark the block bad and move on, re-create the logger task on fatal error.

### H3. One uncorrectable page makes every export fail (HIGH, plausible)
- scan_file returns on the first read error (730-731) -> scan_sessions(true) returns it (387) -> run_export_only sets FLASH_INIT_FAILED and never gives frozen_semaphore_ (267-280): Wi-Fi/USB log download reports "log store not mounted" for ALL files of BOTH slots.
- Linear streams are scanned to the first invalid page; the page after the last good one is normally erased (fine) but may be torn if power died while programming it (the brownout case). Ring streams are scanned across their full capacity, so any such page anywhere stops the mount.
- load_export_page (775-778) also returns on a read error mid-file; for non-ring files a non-matching page returns INVALID_RESPONSE (787) and the rest of the file is unreadable although later pages are fine.
- Fix idea: treat INVALID_CRC as an invalid page in scan_file and the reader.

### H4. HITL / bench / false Boost consumes a slot; weak pilot warning (HIGH, confirmed; operational)
- "Holds a flight" = valid header + marker page 1 magic WLFT with matching generation (322-326). The marker is written when the writer drains the first Boost/Coast/Descent record (630-645, 921-946, 1076-1077). The HITL flag in IMU records (flash_logger.h:83) is ignored, as are commanded bench states and false auto-Boosts.
- Two such events without erase/clear -> both slots flight -> a real launch is not logged at all (C1 reaches the same state with one).
- Warning: status byte 1 bit 7 only (comms.h:184-185). flight_selftest.cpp:25-33 passes with it set; heartbeat buzzer patterns (heartbeat.cpp:199-250) depend only on flight state; no LED/beeper/arming gate; FLASH_INIT_FAILED also only a status bit. Horizon behaviour not audited.
- Fix idea: no marker for HITL records or auto-recycle slots that only hold HITL/bench data; distinct buzzer pattern while LOG_SLOTS_FULL/FLASH_*_FAILED in Pad Idle.

### M1. Capture task stack at Boost (MEDIUM, plausible)
flash_capture stack is 3072 B. At 938-943 it runs ESP_LOGI with BinaryRecord and Sensors::Sample (~130 B) in frame, then log_vprintf adds LogLine 256 B + vsnprintf + console printf. No watermark data for this task in the handoff/bench report; usb_serial_echo overflowed at 3072 under similar logging. A canary overflow here is a panic reset at launch (then C1). Measure with uxTaskGetStackHighWaterMark on a commanded Boost.

### M2. 1 s flush interval (MEDIUM)
Data reaches NAND at most every 1 s (kFlushIntervalMs, 1081-1087) with partial pages; a power cut/impact loses up to 1 s plus queue. Flushing IMU/packets every 100 ms would cost ~10 half-empty pages/s, small against capacity.

### M3. Erase/clear is slow, blind and blocks the command task (MEDIUM)
erase_session_blocks (441-464) is 1004 x (marker read + erase 3-10 ms) = seconds, capture off (1009-1012), caller (usb_serial_echo task, also the radio command path, usb_serial_echo.cpp:577-596, 774-796) blocked in xSemaphoreTake(portMAX_DELAY) (854). command_policy (command_policy.h:35-41) is checked once at acceptance; a launch in the window loses Boost data until the erase ends. The writer polls erase_requested_ every 20 ms. Prefer quick clear; refuse full erase unless Pad Idle/Landed at execution time.

### M4. Caller hang if the logger dies after posting an erase (MEDIUM)
erase_all_sessions sets erase_requested_ then waits forever (847-858). If the writer takes the error exit (1101-1109) in that pass, the semaphore is never given and the USB/radio command task hangs. Narrow race, unbounded effect.

### M5. Launch-protected ring destroys its launch block on lap (MEDIUM, only on a full lap)
The stop is at pages_since_launch >= capacity (519-526) but entering block b erases it (537-541) after capacity-k pages (k = launch page offset in block). Flight pages k..63 of that block are erased up to 63 pages before the stop. Needs a full ring since launch (text ~3 MB, packet ~15 MB, GPS ~29 MB), so unlikely in a normal flight.

### M6. No resync in the IMU stream (MEDIUM)
Page header has no CRC (relies on chip ECC). IMU records (24 B, no magic, u8 sequence) and WL records straddle 2036-byte pages and are concatenated by the reader (read_file_data 815-837). A lost page shifts record boundaries for everything after it. Packet stream is framed and WL records carry "WL", so they can resync; IMU cannot. Pad partial pages to a multiple of 24 or start records on page boundaries.

### M7. IMU stream capacity (MEDIUM)
96 blocks x 64 x 2036 B = 12.5 MB = ~522k records = ~544 s at 960 Hz. If Descent is not reached the stream is marked full, capture_imu_records_ goes false and only FLASH_LOG_FAILED (byte 0 bit 7) is set (507-516).

### M8. log_vprintf stack cost (MEDIUM)
LogLine 256 B + vsnprintf + va_copy on the calling task's stack (228-248), on top of the console printf. This is the margin that cost usb_serial_echo (BENCH_REPORT 2026-11-14 item 1); the 3072 B tasks (capture, spi_interface, data_forwarder, selftest) are exposed. Use a static scratch buffer under a lock or a smaller line.

### Low items
- L1: log_external_bytes sets FLASH_LOG_FAILED on a transient UART queue overflow (128); never cleared; it also feeds flight_selftest passed() on pending-verify boots.
- L2: clear_all_sessions stores erase_quick_ before erase_all_sessions' CAS (860-866).
- L3: double register (973, 1004); landed_power::g_tasks has 4 slots. Only matters if kLowPowerModeEnabled (false, landed_logic.h:19) is turned on; then Landed buffers also flush only every 10 s and erase would wait up to 10 s (no notify).
- L4: u32 timestamps wrap at 71.6 min.
- L5: USB MSC volume 262144 sectors; worst-case content 2 x (12.5+5.2+3.1+15.6+28.8) = 130.4 MB vs 134.1 MB; build_index returns NO_MEM if exceeded.
- L6: no host tests for logger slot choice/scan/ring.

## Answers to the questions asked
- Slot scan: "flight" = header valid (WLS1, v3/v4) AND page-1 marker WLFT AND marker generation == header generation (310-328). An empty (FF) slot reads as not valid (correct). Marker only at first logged state, never on the pad, but HITL/bench/false Boost do mark a flight (H4). Torn header/marker = init failure, not "empty" (C2).
- Session never opened: slots-full (C1/H4); create_session failure (H1, C2); init/self-test failure (H2).
- Queue overflow: IMU 2048 (~2.1 s), binary 256, packet 128, UART 32 KB. IMU and packet drops are counted and warned at flush, no status bit. Writer drains every 20 ms with large margin.
- Starvation/SPI: NAND is alone on SPI2 (w25n01kv.h:43); sensors/radio/baro use SPI3 (sensors.h:263): no bus contention. Only core-0 pre-emption matters (H2). Polling uses vTaskDelay(1 ms), so no idle WDT risk.
- Pointer arithmetic: linear page_index never exceeds capacity (check at 507); ring wraps at 527-536; no underflow. 20-bit sequence = 2 GB per stream; tag wrap handled. No overflow found. Slot filling mid-flight: that stream is marked full, others continue, status bit set.
- Reset mid-flight: C1. Data before the reset stays readable (flight slot never erased).
- Prelaunch rings: IMU (960 Hz x 2 s) and 50 Hz sensor ring are dumped oldest-first before the first Boost record; index math checked (624-645, 926-937) and correct. Empty after resume/erase (by design).
- Erase/clear vs launch: M3. Power loss during erase: C2 for the header; lazily erased data blocks are tolerated by the tag logic.
- Read-out: maintenance image only, read-only; MSC write10 rejected (usb_mass_storage.cpp:162-164); cannot corrupt or delete NAND. Reader weaknesses: H3, M6.
- NAND return values: all program/erase/read results are checked and propagated, but the only response is to die (H2).

## Earlier documented findings in this facet, still present
- FIRMWARE_STATE_MACHINE.md M7 (double mid-flight reset erases earliest data): "fixed" by never erasing flights; the remaining behaviour is C1 (logging stops).
- FIRMWARE_STATE_MACHINE.md:287 "brownout mid NAND write: writer and scan tolerate this (not reviewed in detail)": reviewed, NOT tolerated (C2, H3).
- M6 (false Landed has no automatic recovery): Landed is not logged (flight_state_machine.h:15), so a false Landed silences the I/S streams (cross-facet).

## Checked and OK
- Slot choice never erases a slot holding a flight; generation compare is a signed difference (wrap safe); tag 0 reserved.
- Header is written last in create_session: a half-created session is simply invalid.
- Page header checks bound n <= 2036; no out-of-range memcpy in read_file_data.
- Buffer sizes (packet 4072 B vs 109 B record, imu flush-before-overflow, text, uart) safe; kMaxPacketFrame 103 matches the 0xE2 frame (1+101+1).
- Lazy per-block erase in append_bytes correct for linear and wrapped rings; ring start-page by lowest sequence (742-745) correct.
- Prelaunch ring counting/dump order; reset_session_buffers generation hand-off to the capture task.
- Producers (sensor/CAN/radio/GPS) never block on the logger; log_packet validates length.
- W25n01kv: write-enable latch verified, protection cleared and verified, ECC+buffer mode verified, JEDEC checked, addressing and ECC decode consistent; self-test tolerates ECC-bad reserved blocks.
- USB mass storage: read-only; index/cluster math bounds-checked (46).
- flight_state_store: magic+checksum, magic cleared first on write, Landed not resumed, RTC_NOINIT survives warm resets; brownout classed Warm as intended.
