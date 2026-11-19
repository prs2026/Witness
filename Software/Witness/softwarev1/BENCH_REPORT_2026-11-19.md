# Bench report 2026-11-19: full HITL flight on the bench Witness (flight image of commit `0d3a403`: GPS trust ramp, 2 Hz telemetry)

**Image under test (not reflashed).** The flight image on the board is the 449,680-byte build of 2026-11-18 22:44 that contains the trust ramp
(`gps_trust.h`, `position_filter.cpp`) and the 2 Hz telemetry (`sensors.h`). Evidence: the running image prints ELF SHA-256 `ded17bc42...`, which equals the
SHA-256 stored in the app descriptor of `images/flight.bin` and `build/softwarev1.bin` (both 449,680 bytes). The descriptor says version `168373c-dirty` and
compile time 22:07:27 because that object was last compiled before the commit was made; the SHA is what identifies the image. The maintenance image
(`baba9abf9...`, 22:08) is the one that matches `images/maint.bin`. Nothing was flashed, no firmware or source file was changed, nothing was committed.
Radio at -17 dBm (low power, unchanged), GPS only through the HackRF into the closed coax path at TX VGA 0 dB, amp off (never raised), HITL only through the 710x ports.
The flight uses the same profile as `20261118-214918` (eayf trajectory, FAR site, apogee 36,536 m MSL, 45 g second burn at Mach 3+, GPS outage in the burn, 20 s pad hold, 0.2 vibration scale).

## Runs (all under `Software/HITL/runs/`)

| Run | What it was | Use |
|---|---|---|
| `20261119-125242` | first attempt, 90 s lead-in, scenario as in 11-18 | **aborted by the tool**: no GPS fix in 90 s, tool power-cycled the bench hub, no fix in the next 45 s, run stopped before HITL ENTER (nothing flew). The receiver did decode the scenario time at 83 s but gave no position |
| `20261119-130540` | 300 s lead-in scenario, flew on without a fix | full flight, **no GPS fix until sim 82 s**. Log kept (`fc_logs`, session 120). Evidence for finding 2 |
| `20261119-132253` | 480 s lead-in, fix at the pad | full flight with GPS. **Its `fc_logs` folder is not this flight** (store was full, see finding 3; it is a second copy of run `130540`) |
| **`20261119-134233`** | 480 s lead-in, fix at the pad (242 s after TX start), store erased first | **the reference run for goals 1 to 5**. Everything with a number below is from this run unless stated |

The two longer scenarios (`gps_sim/...-lead480`) were made with the repo's own `gps_rf.create_flight` (lead-in 300/480 s instead of 90 s), same trajectory and site.
Extra analysis files and the scripts I used are in `runs/20261119-134233/bench_analysis/`.

## Results

| # | Goal | Result | Evidence |
|---|---|---|---|
| 1 | Full nominal flight with GPS, launch to Landed | **PASS** | HITL result PASS (tolerance 10 s). Witness transitions against the reference: Boost 20.31 s (expected 20.29, +0.02), Coast 23.01 (22.99, +0.02), Descent 114.00 (113.96, +0.04), Landed 419.75 (419.73, +0.02). In `20261118-214918` Descent came 3.21 s early (110.75 s) because the filter velocity was corrupted by the GPS reacquisition; now it is on time. Same in run `132253` (+0.02 / +0.02 / +0.04 / +0.02) and, without a pad fix, in `130540` (+0.03 / +0.03 / +0.06 / +0.05). GPS outage in this run: fixes lost 35.5 to 43.5 s (5 satellites, then none), back at 43.5 s with 6, 7, 9, then 11 satellites. Attitude verdict PASS (final tilt error 0.96 deg, final nose error 2.4 deg). No reset, no panic during the flight |
| 2 | Position filter against truth (flash log, 50 Hz, launch to landing) | **PASS** (goal is below 5 % of height) | `kalman_replay.py --compare-flight`: **max 278.9 m, RMS 31.6 m, max 2.46 % of height** (at 21.2 s, floor 100 m). Before the fix, run `214918` with the same script: max 5795.3 m (21.98 %), RMS 1178.7 m, 5.9 % of the time above 5 %. The commit predicted max about 606 m and RMS about 116 m from the replay of that run; the replay of `214918` with the current code reproduces it (605.6 m, 116.0 m), and the real board did better than the prediction because this outage is shorter (8 s against 26 s). Replay of this run (host build of the same filter, truth attitude): max 89.7 m, RMS 15.5 m, 1.83 %. Per phase below |
| 3 | Telemetry rates | **PASS** | table below: radio 2.000 Hz for state and sensors in Boost, Coast, Descent and Landed, Pad Idle one packet every 3.002 s (median). USB state 2 Hz in every state, USB sensors 10 Hz (HITL with debug on) |
| 4 | BIT, heap, load, resets, errors | **PASS**, with notes | startup BIT NO-GO [bench condition] (expected), commanded BIT with and without HITL NO-GO [bench] (expected), 6.3 end-to-end and the NAND write test pass. Heap free 19,176 B, minimum 15,052 B (flat, no trend). Core 1 24 to 26 % on the pad, 13.6 to 27.6 % over the run, core 0 9 to 14 %. No panic, watchdog, brownout, stack or queue error, no reset in any flight. Details below |
| 5 | Flight log from NAND | **PASS** | export of session 122 (5 files). State/E2 stream complete, details below |

### Goal 2: error against truth per phase (flash state packets, 50 Hz, by Witness state)

Up error, horizontal error and speed error; "% of height" is the up error divided by max(true height above the pad, 100 m).

| Phase | Samples | Up max (m) | Up RMS (m) | Up max, % of height | Horizontal max / RMS (m) | Speed err max (m/s) | Before the fix (`214918`): up max / RMS (m), % |
|---|---|---|---|---|---|---|---|
| Pad idle | 1015 | 0.6 | 0.0 | 0.58 | 0.0 / 0.0 | 0.0 | 0.5 / 0.0, 0.51 |
| Boost | 135 | 11.5 | 5.8 | 3.20 (small heights, floor) | 4.9 / 2.4 | 1.5 | 14.0 / 5.4, 4.56 |
| Coast (includes the burn and the outage) | 4549 | 282.2 | 65.3 | 2.48 | 59.7 / 11.8 | 6.3 | 5796.1 / 2515.3, 21.99 |
| Descent | 15288 | 19.9 | 9.3 | 1.44 | 2.3 / 1.1 | 5.7 | 33.4 / 7.5, 0.99 |
| Landed | 498 | 0.1 | 0.0 | 0.06 | 0.6 / 0.6 | 0.0 | 0.0 / 0.0, 0.03 |

Boost shows 3.2 % only because the floor is 100 m and the first meters are small; the official score from launch gives 2.46 % at 21.2 s.

**GPS trust after the outage (flash 0xE2 log, `kalman_log` rows).** No trust weight is logged, only the accepted flag, innovations, counters and sigmas, so this is read from those.

| Sim time (s) | What the log and the NMEA show |
|---|---|
| to 34 | 10 to 11 satellites, every fix accepted, vertical innovation +25 to +19 m (the filter lags the receiver slightly at 300 to 1300 m/s, as in 11-18) |
| 35.2 | 5 satellites, innovation -474 m, update accepted but with the satellite-count cap (position sigma only rises 37 to 53 m) |
| 36 to 43.5 | no fix (GGA quality 0), filter dead-reckons: up error -92 m at 35 s growing to -280 m at 43 s, vertical position sigma 37 to 298 m, velocity error about -24 m/s |
| 43.5 | fix back, 6 satellites, innovation +217 m, **accepted at once** (no reject, no wait): up error -215 m at 43.5, -161 m at 44, -39 m at 45, **-0.7 m at 46 s**; below 20 m at 45.1 s, below 5 m at 45.3 s |
| 44.5 to 50 | 7 to 11 satellites, innovations +98, +1, then -5 to -8 m, then under 3 m after 52 s; sigma back to 17 m by 50 s |
| whole flight | **10,061 GPS updates, 0 rejects** (before the fix: 9,593 updates and 1,006 rejects, with the 25-reject forced resync). Up error stays between -1 m and +20 m from 46 s to the Landed transition (12.9 m at 114 s, no step) |

Peak error in this run is the end of the outage (282 m at 43 s). The old failure (receiver reacquires 11 s late with 4 to 5 satellites, 4.4 km innovation accepted because sigma had grown to 983 m)
did not happen because the outage was short; it was not reproduced exactly. The 6-satellite fix at 43.5 s was accepted at full weight at once (innovation 217 m, sigma 287 m), which is the intended
large-innovation path (trust at least 0.5 when the receiver velocity agrees with the filter); I cannot see the trust value itself. Run `132253` (same inputs, outage 38 to 50 s) behaves the same:
up error -506 m at 50 s, -177 m at 55 s, -2 m at 60 s, max 557.8 m (10 Hz 0xE0 comparison, its own flash log was not recorded).

### Goal 3: telemetry rate (reference run, host and packet uptimes)

State is packet `0x02`, sensors `0x01`. "Gaps" are intervals longer than 1.6 times the median. Radio rows are from a separate recorder on the Aperture port (the radio path
Witness to Aperture to USB, coax, -17 dBm), counted from the packets' Witness uptime; USB rows are the HITL session's own decode of Witness USB.

| State | Radio state Hz (packets, gaps) | Radio sensors Hz (packets, gaps) | USB state Hz | USB sensors Hz (max interval) |
|---|---|---|---|---|
| Pad Idle | one per 3.002 s median (181, none) | one per 3.002 s median (181, none) | 1.999 | 9.995 (109 ms) |
| Boost (2.0 s) | 2.000 (5 of 5) | 2.002 (6 of 6) | 2.000 | 10.008 (101 ms) |
| Coast (90.5 s) | 2.000 (182 of 182) | 2.001 (182 of 182; one 530 ms interval) | 2.000 | 9.939 (110 ms) |
| Descent (305.5 s) | 1.993 (610 of 612, **two missed, two 1.0 s gaps**) | 1.997 (610 of 611, **one missed, one 1.0 s gap**) | 2.000 | 9.983 (111 ms) |
| Landed (11.5 s) | 2.000 (24 of 24) | 1.967 (25; one 700 ms interval) | 2.000 | 10.000 (101 ms) |

Radio loss in the flight: 3 of about 1,400 state and sensors packets (0.2 %), all in Descent, none in Boost or Coast. USB: no gap at all in state, sensors never more than 111 ms apart. Camera packets `0x03` arrive
about 1 Hz on the radio path (957 frames in the run). Pad Idle on USB is not throttled (HITL connected, debug packets on); the Pad Idle throttle to 1 per 3 s applies to the radio only.
The first (no-fix) run showed USB sensor gaps up to 1.0 s in Descent (554 ms in Coast); runs `132253` and `134233` show none, so I do not count it as a defect but it was not explained.

### Goal 4: BIT, heap, load, resets

| Item | Result |
|---|---|
| Startup BIT (boot via the `0x5746` on then off round trip, HITL off, boot log captured from the first line) | **NO-GO [bench condition]**, as expected. Verdict RUNNING at 0.9 s of uptime, NO-GO [bench] at 1.1 s (core group done). Final at 46 s of uptime: blocking groups `0x0100` (radio power 8.3, value 65519 = -17 dBm, flagged bench), degraded `0x0896`: 1.2 task watchdog (4), 1.3 nvs (2), 2.8 body orientation (board flat), 4.6 calibration, 4.7 pad heading, 7.4 GPS fix (no sky), 7.5 UTC, 11.1 battery (5.7 V supply), 12.4 time of day. `bit: reset class 3; 0 fault resets in the last 2` (software reset). The heap check (0.3) passes |
| Commanded `0x5442` before launch, HITL off (lead-in, GPS fixed, store erased) | NO-GO [bench condition], blocking `0x0100`, ack OK detail `0x0401` (NO-GO, 1 blocking group), 1.0 s after the command arrives. NAND write test: block 1023 erase 3.68 ms, program 2.14 ms, ok. `0x5452` pages 0xFF, 0, 1, 2 all acked OK. 6.3 log end-to-end: no failure line |
| Commanded `0x5442` before launch, HITL on (sim 5 s, pad) | NO-GO [bench condition], blocking `0x0102` (HITL active 1.4 and radio), ack OK detail `0x0402`, NAND erase 4.49 ms, program 3.02 ms, ok, 6.3 passes. `0x5452` pages OK. HITL active makes it NO-GO [bench] as expected; the bench flag stays set |
| Passive BIT during the flight | verdict stays NO-GO [bench] while the group list is only HITL + radio. At sim 37 s one NO-GO **without** the bench flag (blocking `0x000A`) with check 3.2 "baro plausible" FAIL class 2 (value 19357) and 3.4 "baro rate" degraded; it clears by sim 291 s (back to bench only). Not investigated; the barometer is gated above Mach 0.8 and the simulated pressure is outside the MS5607 range for 67 s |
| Heap | free 19,176 B, minimum 15,052 B (startup 15,032 B), no change from the pad through the flight and the export boot; the startup minimum is 124 B below `BENCH_REPORT_2026-11-18b` (15,156 B), same build family. Free minimum is above both the shipped limit and the 10 KB owner limit now in the working tree (`bit_limits.h`, not flashed) |
| Core load | core 1 24 to 26 % in Pad Idle, 13.6 to 27.6 % across the run (mean 21.5 %, lower in flight, max pass 2.3 to 3.2 ms in flight, 7.2 ms peak overall); core 0 9 to 14 % (mean 11.4 %). Stack minimums: sensors 6,084 B, uart1 1,584 B, iris_can 2,376 B, flash_capture 4,372 B, data_forwarder 1,172 B, bit 1,988 B, sensor_log 1,064 B (all above 1 KB) |
| Resets, panics, watchdog, error lines | **none in any flight** (continuous uptime, no boot lines, no `Guru`, `panic`, `abort`, `Task watchdog`, brownout, stack overflow, queue full, or logger error). Last reset reason at the start of the reference run: POWERON. The warning lines seen: 19 `uart: discarded UART packet ... invalid EOF` during the GPS assist upload (known), one `uart: GPS silent for 3000 ms: recovering` at the receiver's self-reset (finding 1), `attitude after flight unavailable` after the Landed export command (no pad window, expected), and the BIT lines above. Sensor attitude loop: 936.6 Hz, no gaps, max dt 1.1 ms. The loop "overruns" counter runs at about 3 per second on the pad and 10 to 21 per second in flight (the 11-18 run: 3 and 11 to 17 per second), so it is not new |
| Bench hub | no `over-current` message in the Pi `dmesg` at any check (start, middle, end); the Pi saw USB disconnects only at the hub cycles and at the `0x6868` export switches. Hub cycles by this session: 1 by the tool in the aborted run, 1 after run `130540`, 1 after `132253`, 1 after the reference run, never closer than 4 minutes |

### Goal 5: flight log

Fetched by the tool as before (Landed set with `0x7304`, `0x6868` export into the maintenance image, copy over the Pi, hub cycle). Run `134233/fc_logs`: `S0000122.TXT`, `I`, `P`, `S` and `U` files for **session 122**.

| File | Check |
|---|---|
| `P0000122` (state, E2 Kalman, camera, heartbeat) | 51,971 state packets at 50 Hz (median 20 ms, max 37 ms between packets), 25,985 Kalman rows (25 Hz, max 80 ms), 1,040 camera, 1,039 heartbeat; log time 0 to 1,039 s with Boost at 627.4, Coast 630.1, Descent 721.1, Landed 1,026.8 s. 0 time reversals. No resync or discard message from the converter. The flash comparison (`comparison_flash.csv`) has 21,485 rows from sim 0.016 to 429.696 s, no gap over 24 ms |
| `I0000122` (960 Hz IMU) | 121,209 samples, no reversal, 0 unexpected gaps; the IMU and sensor streams are rings (96 and 40 blocks), so they hold the last part of the session only (`S` rows start 57 s after Boost): the same as in run `214918` |
| `S0000122.TXT` | 250 KB, starts with the erase lines and ends with the `0x6868` export restart lines; no `NOT LOGGING` or logger error |
| Complete? | **yes** for state, E2, camera and heartbeat from the pad to the end; IMU and sensor logs complete only within their ring size |

## Open findings

1. **RF GPS acquisition is much slower than on 11-18 and only started after a receiver self-reset.** With the same HackRF path (VGA 0 dB, amp off, closed coax) the first fix came 242.5 s (reference run) and 321.8 s (run `132253`) after TX start, and
   not within 90 + 45 s (aborted run) or 300 s (`130540`, fix at sim 82 s, about 382 s). On 11-18 the first fix came at 33 s with 14 to 17 satellites; now 7 to 11 are used (scenarios had 9 to 12 in view). A static test with `gps_rf.py test` (cold start) also needed 225 s for its first fix, with 4 satellites. In both runs with a pad fix the log shows the same sequence a few seconds before the fix: a burst of binary garbage on the GPS UART,
   `GPS silent for 3000 ms: recovering`, `GPS UART host rate now 115200 baud`, then `25 Hz confirmed`, and the first fix 4 s (reference) or 27 s (`132253`) later: the receiver restarts itself and only then acquires, which matches the 11-18 observation in the bench notes. The Pi hub power cycle done by the tool at 90 s did not help (it
   resets the receiver again). The coax path was not touched and I did not raise TX gain or enable the amp. Cause not found; worth a look at the cable, connectors and attenuator on the closed path, and the lead-in of the stock scenario (90 s) is too short for this behaviour (the tool aborts the run).
2. **A flight without a pad fix keeps a large position error for the whole flight (run `130540`, not the nominal case).** The first fix arrived at sim 82 s; the filter had dead-reckoned the 45 g burn with IMU only (baro gated, vertical velocity error about -21 m/s from 40 s), so its up error was -1.18 km and horizontal 1.07 km when the GPS origin was set (`kf_gps_origin_set` at about sim 88 s). From then on the GPS innovation was near zero (the origin was taken in the filter's own frame), the error stayed at -1,176 m through the whole descent, and only after the barometer returned (about sim 305 to 340 s) the filter moved to the true altitude while the GPS was then rejected (innovation -716 to -1,185 m, 2,431 rejects by landing). Result: up error max 1,274 m, RMS 947 m (44.6 % of height near the end); the transitions were still on time. The IMU-only velocity error is consistent with about 1.2 deg of attitude error over the 45 g burn (the host replay, which uses the truth attitude, stays within 34 m on the same inputs), not verified. Accepted drift without GPS (11-18 item 1.5) may cover the first part; the permanent anchoring of the frame to a mid-flight fix is the new observation.
3. **HITL tool does not notice a full log store (run `132253`).** After `130540` both NAND slots held a flight. The next run started with `LOG_SLOTS_FULL` set in every status packet (17,395 packets), logged nothing, and the end-of-run fetch exported the previous session (`S0000120`, byte-identical to run `130540`) under the new run folder without a warning. Nothing was erased automatically (as designed). I erased with `0x6869` once (allowed on the bench, logs of the earlier runs were kept in their run folders) and flew again. Suggest the tool checks the status bit before a run, and compares the exported session number with the previous run. The BIT also reported 6.1 log session FAIL, 6.5 logger heartbeat, 5.2 and 6.3 ERROR during that run, which is the correct report for a full store.
4. **BIT 6.1 "log session" FAIL for about 2 s after an erase.** Two BIT results after the `0x6869` (59.2 s and 60.7 s of uptime) still showed `61 log session: FAIL class 2 value 2`; it is gone by the next full BIT (minutes later, not present in the commanded runs). Looks like a stale value for one or two cycles, minor.
5. **One boot reported `last reset WDT`** (start of run `132253`, after the end-of-run export of run `130540` and its hub cycle; BIT 0.1 then FAIL class 1, degraded). All other boots reported POWERON (the later hub cycles) or SW. No watchdog line was captured because the boot happened during the hub cycle; no relation found. Maintenance image only (seen in the `0x5746` round trip, not a flight problem): `E task_wdt: esp_task_wdt_reset(707): task not found` about every 65 ms while the Wi-Fi AP is up, and `phy_init: RF calibration data MAC check failed ... falling back to full calibration` (stored for a different MAC; the calibration is redone and saved).
6. **Attitude error in Coast varied between identical inputs:** reference run Coast attitude 4.0 deg RMS (max 6.4, nose 4.3 deg max), run `132253` 0.74 deg RMS, `214918` about 1.2 deg. Final tilt is fine in all (0.96 deg). The 3D printer is on the same table; I did not see it in any other number, but the IMU vibration noise may be relevant. Not investigated.
7. The flash comparison of a run with a duplicated or missing log is silently wrong (see run `132253`: "Flash state packets vs truth" printed Descent 24,824 m RMS because it compared the old log), a consequence of finding 3.
8. Working tree notes: it has uncommitted changes newer than the flashed image (`sensors.cpp/.h`, `wifi_ota.*`, `bit_limits.h`, Aperture `main.cpp`, `ra01.cpp`); none of them was on the board for this run (the board runs the `ded17bc42...` build).

## Not done or limited

- The flight in the reference run is the nominal profile; the "no GPS" and "outage" variants of the preset list were not run (the HITL tool runs the RF path only through the GUI, so I drove `HitlSession` directly from a script with the same settings the GUI saved; `power_cycle_retry` was turned off in the no-fix retries so the tool would not cycle the hub again).
- The `0x5452` report pages were requested but their payloads (check list per page) were not decoded here; the verdicts and failing checks come from the firmware log lines, and the ack detail values from the `0x06` acks.
- The in-flight (post-launch) commanded BIT is refused by design and was not repeated; in run `130540` the script sent the BIT command at sim 20.24 s, 0.08 s before Boost (script timing slip, fixed for the later runs); no commanded result came back and the transitions were not affected.
- Heap and load figures are from log lines every 10 s; no soak.

## End state

Witness in Pad Idle, HITL off, flight image (ELF `ded17bc42...`), last reset POWERON after the final hub cycle, logging to flash (status words `0x6600`/`0x6700`: no flash-failed bit, no log-slots-full bit); the store holds the reference-run flight in one slot and the other slot is free, so **one more flight fits and the next one after it will fill the store again**. Ports 7100, 7101 (Iris) and 7102 (Aperture) free (all my connections closed), HackRF stopped normally (no force kill), self-test PASS at the last check. Radio power untouched at -17 dBm. No `over-current` event in the Pi `dmesg`. Disk: the three flight scenarios made today were reduced to one (`gps_sim/20261119-134041-flight-eayftrajectory-lead480`, about 4.7 GB, gitignored); the two earlier ones I made were deleted.
