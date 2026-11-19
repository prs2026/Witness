# Witness pre-flight audit 02: radio, commands, NAND logging, boot, maintenance image

Date: 2026-11-19. Scope: `Software/Witness/softwarev1` at HEAD (a06d14d) plus the one uncommitted
edit (`main/bit_limits.h`, heap degraded limit 8 KB to 10 KB). Read-only review; host tests built in a
scratch folder only (tx_slot, command_policy, log_slot_logic, landed_logic, radio_edge_logic,
erase_wipe, image_info all pass at HEAD). No hardware touched. Paths below are relative to
`Software/Witness/softwarev1`.

Result: no Critical or High defect found in the areas assigned. Two Medium items (one firmware
behaviour, one operational), several Low items. The 2 Hz telemetry change fits the GPS slot with a
large margin (airtime table below).

## Findings

### Medium

**M1. A failed Wi-Fi AP start strands the board in the maintenance image with no automatic return** (CONFIRMED by reading)
- `main/wifi_ota.cpp:120-137` (`WifiOta::run`), `main/maint_main.cpp:218-262` (main loop).
- The auto-off test is `if (enabled_.load() && window != 0 && ...)` (wifi_ota.cpp:133). `enabled_` is set
  only when the AP and HTTP server came up (wifi_ota.cpp:253). If `start_wifi()` fails (any of the 12
  steps, for example a heap shortfall, which is a failure this image has had before), `run()` only
  sets `failed_` and the auto-off never fires. maint_main.cpp has a time limit only for USB export
  (line 255), none for Wi-Fi mode, and the recovery case deliberately passes minutes = 0.
- Scenario: operator sends Wi-Fi ON on the pad (legal in Pad Idle), the AP fails, the rocket is left on
  the pad. The board sits "NOT FLIGHT READY" (no sensors, no logging, no downlink except the 0x08
  beacon) until someone sends Wi-Fi OFF over LoRa or power-cycles it. A power cycle returns to flight
  (the boot slot is already restored), so it is recoverable, but only if someone notices.
- Fix: in `run()`, when `failed_` is set (and not in recovery) call `maint::leave()` after a short
  delay (for example 30-60 s); also give Wi-Fi mode an absolute ceiling like the USB export one
  (for example 4 h, matching `kMaxAutoOffMinutes`) so a forgotten session with a chatty client cannot
  hold the board out of flight indefinitely.

**M2. Pad boot with both log slots holding a flight logs nothing; HITL/bench runs create such flights** (CONFIRMED; owner policy is "never erase a flight", so this is a checklist item, not a bug)
- `main/log_slot_logic.h` (`choose`: both slots valid and flight, not resumed, gives `SlotsFull`),
  `main/flash_logger.cpp:1335-1355` (all capture left off, wait for an erase command).
- Any HITL run or bench boot that reaches Boost writes the flight marker (flash_logger.cpp:664), so
  after two such runs both slots are "flights". On flight day the board then boots with
  `LOG_SLOTS_FULL` and no NAND logging at all (telemetry unaffected). Same outcome after a COLD reset
  (power glitch, ESP_RST_POWERON, not a Warm class) in flight if the other slot still holds an older
  flight: the pad-boot rule applies and the rest of the flight is not logged (what was already on NAND is safe).
- Signalled by: 3-pulse red LED, status flag, BIT check 6.1, Horizon. The memory note says bench logs
  are HITL runs that may be erased, so the fix is procedural: download if wanted, run erase/clear
  (0x6869 or 0x686A, two sends over radio), confirm "session ready" in BIT, then arm. Add it to the
  pad checklist. Optional firmware change (owner decision, bends the "never erase" rule): when the
  state goes to Boost with SlotsFull, erase the OLDER flight slot instead of dropping the live flight.

### Low

**L1. After a mid-flight reset the ring-stream launch protection restarts from zero** (CONFIRMED in code; impact PLAUSIBLE, needs a reset AND a long powered wait after landing)
- `main/flash_logger.cpp:557-623` (`resume_session`): `Writer w{}` has `pages_since_launch = 0` and
  `launch_protected = ring && flight` (line 608); the pre-reset post-launch page count is not restored.
  `append_bytes` (line ~765) stops a protected ring only after `capacity` MORE pages. The true limit is
  `capacity - (pages already written since launch)`.
- Packets (7680 pages, about 3-4 pages/s) and GPS (14144 pages) keep logging in Landed (only the binary
  and IMU streams are gated by state). After a resume at page X (launch at L) the writer wraps and starts
  overwriting the early flight (burn, apogee) after about `capacity - X + L` pages: roughly 35 min for the
  packet ring if the board stays powered after landing.
- Fix: in `resume_session`, for a flight-marked slot set `w.pages_since_launch` conservatively to what is
  already in the ring (`w.wrapped ? capacity : next`), so everything present is protected.

**L2. Maintenance Wi-Fi: committed default password, unauthenticated HTTP (/ota, /reboot, /wifi/off)** (CONFIRMED)
- `main/Kconfig.projbuild:34-36` ships a fixed default WPA2 password in a public repo; any station that
  joins can flash an image over `/ota` (image kind, project and chip are checked, the sender is not).
  Exposure is limited to the window when Wi-Fi was commanded on the pad (max 2 stations, auto-off).
  Set `CONFIG_WITNESS_WIFI_PASSWORD` to a private value for the flight build, or accept the risk.

**L3. Stale legacy flash helpers can damage the two-image layout; one has a personal path** (CONFIRMED)
- `flashit.ps1` and `flashit.sh` write an app at 0x10000 with `--flash_size 2MB` (the old single-app
  layout). In the current table 0x10000 is `otadata` and a 450 KB write runs on into `flight_a`. The shell
  script also hard-codes a home-directory path (personal name) and a serial device in a public repo. They are
  not used by `build_images.ps1` / `flash_verified.ps1`; delete them or make them exit with a message.

**L4. Build hygiene before imaging** (CONFIRMED observations)
- `images/flight.bin` was built before the working-tree `bit_limits.h` edit made today, so it does not
  contain the 10 KB heap limit; rebuild with `build_images.ps1` before imaging. `images/maint.bin` is older
  than the GPS-trust commit but the maintenance image does not link that code (compatibility is checked by
  `image_info`, not by build time).
- The ignored `sdkconfig` in the project root is a stale plain-defaults config (-Og, 160 MHz, 16 KB
  icache). Only `build/sdkconfig` carries the flight settings (verified: -Os, 240 MHz, 32 KB icache, 32 KB
  dcache, task WDT 8 s with panic, rollback on). A plain `idf.py build` or `idf.py flash` from the project
  root would give a core-1 overload image (the 11-17c bench measured 96-100 % busy with the 16 KB icache).
  Always flash from `images/`.

**L5. Heap and stack margins were measured before the latest in-flight code change** (PLAUSIBLE)
- Measured (11-18b, 32 KB dcache): heap 19.3 KB free, 15.2 KB minimum; stacks: sensor_log about 1.07 KB,
  data_forwarder about 1.17 KB, uart1 about 1.6 KB, cpu_load about 1.5 KB free. Commit 0d3a403 added the GPS
  trust ramp to the sensor task and doubled the state packets; no bench report exists after it. The sensor
  task (5.3 KB free) has room, but run one 10-minute HITL flight and read the heap and stack high-water
  lines before imaging. Steady-state flight code allocates nothing (no malloc or new; only the 3 KB cal-save
  task and the 4 KB self-test task, both on the ground); allocations of a 4-5 KB block are what fail first,
  so the 10 KB degraded limit is a sensible tripwire.

**L6. IMU stream has no resync marker if a chunk is ever dropped** (PLAUSIBLE, very rare)
- `main/flash_logger.cpp:795-830`: after 4 consecutive unusable or failed blocks a chunk is dropped
  (`chunks_dropped_`). Chunks are 2036 bytes, records 24 bytes, so the rest of I*.BIN is misaligned.
  Records carry an incrementing sequence byte, so a reader can resync; consider noting it in the export tool.

## What was checked and found OK

### LoRa airtime (SF7, 500 kHz, CR 4/5, preamble 8, explicit header, CRC on; Semtech AN1200.13)
Tsym = 2^7 / 500000 = 0.256 ms. Payload symbols = 8 + ceil((8*PL + 16) / 28) * 5 (28 = 4*SF).
Total = (8 + 4.25 + payload symbols) * 0.256 ms. Computed with `tx_slot::lora_airtime_ms` and the real frame
length macros from `comms.h`:

| Packet | Frame bytes | Airtime | Rate | ms per second |
|---|---|---|---|---|
| 0x01 sensors | 23 | 16 ms | 2 Hz | 32 |
| 0x02 state | 65 | 31 ms | 2 Hz | 62 |
| 0x03 camera (Iris relay) | 19 | 13 ms | 1 Hz | 13 |
| Telemetry total | | | | 107 ms (10.7 %) |
| 0x06 ack | 13 | 12 ms | on demand | |
| 0x0B BIT summary | 14 | about 12 ms | every 10 s while not GO, bypasses the slot | |
| largest frame (103 B debug, not sent) | 103 | 44 ms | | |

- Window: a telemetry packet starts only if `phase + airtime <= 700` (`tx_slot::witness_telemetry`) with a
  20 ms GGA latency added. The whole second's telemetry (107 ms) fits in the 700 ms window with 593 ms
  spare; the latest legal start for the longest packet (state) is phase 669 ms.
- I simulated `service_radio_transmit` (rotation, per-type periods, `Wait` returning without marking sent)
  at 1 ms steps for 118 s over six UTC phase offsets: 2.00 sensors/s, 2.00 state/s, 1.00 camera/s, worst gap
  670 ms (500 ms nominal) for the 2 Hz types, no packet ever ended past phase 692 ms. Pad Idle (3 s period):
  0.33/s each, camera stays 1 Hz (`kPadIdleThrottlesCanRelayed=false`); the period test reads the current
  state on every pass, so leaving Pad Idle restores 2 Hz on the first pass of Boost.
- Command RX room: Witness transmits only in 0-700 ms, Aperture commands in 800-950 ms, so Witness listens
  for the guard and uplink part of every second plus all the gaps. Without GPS time (`utc_known` false after
  3 s) it falls back to sending whenever free: 107 ms/s, about 11 % blind time, still plenty of RX time.
  `poll_radio_commands` is skipped only while a TX is on the air. The ack pacer holds telemetry while an ack
  waits (up to 300 ms per ack); only a command storm matters, and Boost never arms the hold-off.
- Hold-off governor: validated 10-byte frames only, never in Boost, capped at 3 s then one forced cycle of
  all three packet types (tx_slot_test passes).
- TX queue overflow: telemetry is not queued (latest-packet cache, one send per type per period). Only the
  immediate queue (acks, BIT summary, 16 deep) can overflow; drops are counted (`immediate_dropped_`) and it
  is reachable only by a command burst on the pad.
- Radio fault handling (BUSY timeout, 3 SPI failures, 3 TX failures latch, non-blocking re-init every 5 s,
  health check every 2 s) is bounded per pass and cannot stall the 500 Hz loop.
- Boot default TX power is -17 dBm, restored from NVS; BIT check blocks below -3 dBm, so a board with erased
  NVS shows NO-GO until power is set (set it on the pad and read the ack).

### Command handling
- 0x5746 Wi-Fi ON: `handle_wifi` (usb_serial_echo.cpp:1235) refuses unless Pad Idle or Landed
  (`wifi_ota_logic::state_allows_wifi`); `prepare_switch` runs only after that; the deferred restart
  re-checks the state 0.4-1.1 s later (`service_pending_restart`, `restart_still_allowed`) and puts the boot
  partition back to flight if a launch happened. Radio and USB share the function. Boost, Coast and Descent
  cannot reach it. OFF and status are harmless in the flight image. The maintenance request (NVS blob) is
  honoured only after an ESP_RST_SW, so a brownout or watchdog reset cannot enter maintenance; the flight
  image drops a stale request on every boot (`maintenance_mode::cancel` writes NVS only if the key exists).
- USB export (0x6868) is USB only with a 120 min ceiling. Erase/clear (0x6869/0x686A) need Pad Idle or
  Landed, a double send within 10 s over radio, and are re-checked in the logger task (a launch aborts
  them). The background wipe only touches the non-current slot in 8-block chunks and cancels itself when the
  state leaves the ground.
- Radio load-OFF, bench, GPS assist, cold start, pad heading, radio power: gated as documented; power rule in
  Coast/Descent per the owner decision. Radio flight-state setting: only Landed to Pad (restart) and Pad to Pad.
- Compact 3-byte commands get no ack and cannot arm the hold-off.

### NAND logging
- Slot logic (pure, tested): a pad boot never reuses a flight slot; a resumed boot appends to the newest
  slot; a slot whose header block will not erase is avoided; torn headers are rebuilt from the marker.
- Append on resume: per-stream end found by block scan plus binary search, torn page skipped, no erase of
  flight pages except entering the next block of the same stream; rings launch-protected (see L1).
- Write path: 3 erase and 3 program attempts per block and page, bad blocks skipped, back-off after 8 failed
  blocks, producers never block (drops counted), task never exits, watchdog fed in the long loops.
- Bandwidth at 960 Hz: IMU 23.0 KB/s (11.3 pages/s) + sensors 1.2 + packets about 6.3 (state 50 Hz, Kalman
  25 Hz, CAN) + GPS up to about 11 KB/s gives about 40 KB/s, about 20 pages/s, far inside NAND capability
  (page program <= 0.7 ms, block erase <= 10 ms per 64 pages). The IMU queue (2048 records, 2.1 s) and
  record queue (256) absorb an erase easily. Capacities: IMU 96 blocks = 540 s at 960 Hz (96 Hz after
  apogee); sensors 40 blocks; packets 120 blocks (about 35 min); GPS 221 blocks (about 45-80 min).
- The restart flush handler (150 ms) writes buffers before deliberate restarts (maintenance switch,
  Landed to Pad reset); panic or watchdog resets lose up to the last second of buffered data (documented).

### Boot, rollback, watchdog, maintenance image
- Two images: flight 2 x 2 MB, maintenance 2 x 1.5 MB, otadata and NVS at the old offsets; images are 450 KB
  and 852 KB. A switch needs a readable, compatible maintenance image (kind, struct, store, radio and
  command-set versions) and `esp_ota_set_boot_partition` validation; failure leaves the boot slot on flight.
- The maintenance image restores the flight boot slot first thing (before Wi-Fi), so power loss or any reset
  in maintenance boots flight; a cut before that point rolls back through the pending-verify rule. New flight
  image: pending-verify, 30 s self-test (IMU, flash, radio), rollback only if Pad Idle; a resumed in-flight
  boot never rolls back. New maintenance image: verification boot with its own 30 s self-test.
- OTA handler: Content-Length required, header, descriptor, chip, project and kind checks before the first
  flash write, target is always the inactive slot of that kind (the running slot is never written), size
  within the slot, compatibility check against the other image (override header), 3/6 stall limits,
  `esp_ota_abort` on failure, `esp_ota_end` validation, boot slot set only after a complete valid image. A
  partial upload leaves the installed images untouched. Logs are served read-only (the maintenance image has
  no write path, so logging is frozen by construction), file names are whitelisted 8.3.
- sdkconfig.flight (built config verified in `build/sdkconfig`): -Os, 240 MHz, 32 KB icache, 32 KB dcache
  (the 16 KB data cache boot panic is fixed), task WDT 8 s with panic (sensors and logger subscribed, idle
  checks off), interrupt WDT 300 ms, bootloader WDT 9 s, brownout reset, stack-overflow canary, assertions on,
  PM and tickless off. Maintenance: -Og, 160 MHz, 16 KB icache, Wi-Fi code in flash, idle WDT on both cores.
- Tasks: sensors prio 4 alone on core 1; logger, uart, can, usb, gps_i2c 3, capture 4, heartbeat and
  forwarder 2, BIT and cpu_load 1, all on core 0. Nothing at or above the sensor priority on core 1.
- BIT radio summaries and acks use the immediate path and ignore the 700 ms window (about 12 ms every 10 s
  while the verdict is not GO): negligible against the Aperture window.
