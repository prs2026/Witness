# Bench report 2026-11-19c: GPS power-up retest on the corrected 3.3 V regulator, HITL flights, checks of the `32cc6a2` fixes

**Scope.** The owner fitted the corrected 3V3 regulator on Witness (rail measures 3.3 V, was about 4.3 V). This session retests the GPS (PX1120S) power-up behaviour found in 11-19b ("state A"), flies HITL flights with RF GPS, and checks on hardware the `32cc6a2` fixes not yet exercised. Run folders written: `Software/HITL/runs/20261119-c-powerup/` (power-up and fix-check logs, scripts in `bench_analysis/`), flights `20261119-203957`, `20261119-210705`, `20261119-213205`. No source, config or firmware file was changed, nothing was committed. HITL only through the 710x ports; no flashing was needed (see Part 0). Radio power untouched at -17 dBm (BIT NO-GO [bench] expected). RF only into the closed coax path, HackRF TX VGA 0 dB, amp off, every stream ended by closing its stdin.

## Short answer

**No. The corrected regulator did not fix the GPS slow power-up state.** After every power-up the receiver still comes up at 88.4 kbaud (Witness measures 88,398 to 88,403 baud), still gets no fix, and still leaves that state only by a random self-restart (garbage burst, 3 s silence, then NMEA at 115.2 kbaud). Of 6 power-ups watched, 6 were in state A (115,200 x 0.767), 4 restarted themselves (168, 232, 244 and 509 s of RF; and one at 942 s after power-up, after a first RF stream of 520 s with no restart), 1 had not restarted in 520 s of RF and 1 restarted at 295 s of a second stream. The 11-19b numbers (A: 148 to 509 s, 3 of 11 not within 520 to 1,000 s) are unchanged. After the restart (state B) the first fix came 9 to 33 s after RF start, as before. The over-voltage was not the cause (or not the only one).

Other results in one line each: the Witness-only reset (0x5746 round trip) leaves the receiver where it was (B stayed B, A stayed A). The no-IMU predict check **could not be done** (no safe documented way, see Part 3a). Aperture uplink acks all came back (3b PASS). Wi-Fi ON/OFF over LoRa through Aperture PASS (3c). Two full scored HITL flights PASS. One new observation: the reset reason after a hub power cycle is sometimes **WDT** instead of POWERON (3 of 7 boots after a cycle, see findings).

## Part 0: image check (no flash needed)

The owner allowed flashing, only to make sure the newest build is tested. `git log 32cc6a2..HEAD -- Software/Witness/softwarev1/main Software/Aperture` was empty before every HITL flight (checked three times), so no firmware change landed.

| Image | Boot log (Witness, 0x5746 round trips at 20:37 and 21:21) | `images/` file (app descriptor read from the file) | Result |
|---|---|---|---|
| flight | version `32cc6a2`, compile 14:10:52, ELF SHA-256 `353150532...`, `selftest: running flight slot 0 (32cc6a2)` | `flight.bin` 449,808 B, version `32cc6a2`, 14:10:52, ELF `353150532...` | equal |
| maintenance | `32cc6a2`, compile 14:11:55, ELF `5a6ec4365...`, "boot partition restored to flight slot 0" | `maint.bin` 851,856 B, `32cc6a2`, 14:11:55, ELF `5a6ec4365...` | equal |

Witness runs exactly the images in `images/`; nothing was flashed. Aperture untouched (no change after `32cc6a2`). Tested versions: Witness flight `32cc6a2`/`353150532...`, maintenance `32cc6a2`/`5a6ec4365...`.

## Part 1: GPS power-up retest

Method (same as 11-19b): Pi hub cycle (`uhubctl -l 2 -a 2`), Witness boot log recorded on 7100, 70 s after the cycle a cold start (`0x4752`, "GGA+RMC confirmed" waited for), then HackRF streaming the 21:40 GPS 480 s-lead-in scenario (the one used for the A/B experiments in 11-19b) with the 31 ephemeris records uploaded; time zero = HackRF streaming. "Lock" = receiver clock first set from satellites. Give-up rule 520 s of RF.

| # | Hub cycle at | Receiver at boot (Witness) | Config lines | Lock (clock set) | Receiver self-restart | First fix (satellites at fix / 5 s later) | Last reset reason |
|---|---|---|---|---|---|---|---|
| p1 | 20:10:41 | **88,398 baud** | identified at 115,200 at 2.5 s, "startup failed: receiver went silent during configuration", found at 88,398 at 5.9 s, "airborne 4 g mode and 25 Hz GGA+RMC confirmed at 88398 baud" at 8 s (after cold start again at 65 s) | 74.5 s | 168.0 s | 176.6 s (4 / 11) | POWERON |
| p2 | 20:16:00 | 88,398 | same | 74.3 s | none in 520 s | none in 520 s (stopped) | POWERON |
| p3 | 20:26:03 | 88,398 | same | 97.1 s | 243.7 s | 277.5 s (4 / 8) | WDT |
| p4 | 20:32:06 | 88,398 | same | 74.6 s | 232.2 s | 242.4 s (5 / 9) | POWERON |
| pf1 (hub cycle of the tool after flight 1, 20:50:34; RF 92 s after it) | 20:50:34 | 88,403 | same | 74.4 s | none in 520 s of RF | none in 520 s; second stream (`pf1b`, cold start again) 309 s after its start | WDT |
| pf1b (same power-up, second RF stream) | | 88,403 | | 74.5 s | **295.4 s** (Witness uptime 942 s) | **308.8 s** (6 / 9) | WDT |
| pf2 (hub cycle of the tool after flight 2, 21:17:44; Witness-only reset by the Wi-Fi test at 21:21; RF 4.8 min after the cycle) | 21:17:44 | 88,403 | same | 74.3 s | 508.9 s | 542.2 s (9) | POWERON at the cycle, SW after the Wi-Fi reset |

Counted: **A in 6 of 6 power-ups** (p1 to p4, pf1, pf2; pf1b is pf1). Restarts after 168, 232, 244, 509 s of RF (p1, p4, p3, pf2) and 295 s of a second stream (pf1), no restart in 520 s (p2, pf1). Compared with the 11-19b table: A with no fix before the restart in every case, restart 148 to 509 s, some power-ups with none in 520 to 1,000 s. The only difference is a tendency to lock earlier (74 s with cold start, same as 11-19b's 74 to 97 s) which is unchanged. In state B the fix came after 9.0 s (flight 1) and 33.0 s (flight 3), the same as 11-19b (8.5 to 35 s).

**The slow state looks the same with a 3.3 V rail.** The boot log shows one more detail that supports the old suspicion about the power-on configuration: in every power-up Witness identifies the receiver at **115,200** baud at 2.5 s (readable NMEA), starts the configuration, and within about 15 ms of that the receiver output turns to garbage and it is only found again at 88,398 baud 3 s later ("went silent during configuration (it probably changed rate)"). That is the open question from 11-17 (bug 1) and is still not answered by the regulator: the receiver is at 115.2 kbaud when Witness first sees it and at 88.4 kbaud after Witness's setup sequence. I cannot say whether the sequence causes it (Witness has `kFixedBaud` set and sends no baud command; the setup is airborne mode, NMEA set, constellations, mask, QZSS/SBAS, 25 Hz); testing it needs a build that skips the setup or listens only, which I did not make.

**Witness-only reset (0x5746 round trip, 20:37:46, receiver in state B).** Witness restarted into the maintenance image and back to the flight image in about 25 s; the receiver stayed in B (115,201 baud, `recoveries 0`, receiver clock running on, the time of the previous test); the configuration was confirmed "at 115200 baud" with no change of rate and no startup failure. A second Witness-only reset (the Wi-Fi test, 21:21, receiver in A) left it in A (88,398, "identified ... at 88398 baud"). So the receiver state follows the hub cycle only, as in 11-19b.

## Part 2: HITL flights

Profile `eayftrajectory.csv`, FAR site, apogee 36,536 m MSL, 20 s pad hold, vibration scale 0.2, GPS from the HackRF over the coax, scenario `...-lead150-geom480` (150 s lead-in), tool tolerance 10 s, run through `HitlSession` from a script as in 11-19b (cold start on, `power_cycle_retry` off), **with the tool's normal hub cycle after the log export** (`fetch_restart_hub` on). The store was erased (`0x6869`, status words `0x6600`/`0x6700`, no slots-full bit) before each flight; the exported session number equals the one the erase created each time (324 = 0x144, 326 = 0x146, 329 = 0x149). Before each flight the receiver was brought into state B with a conditioning RF stream (pf1b, pf2: that is why they are in the power-up table).

| | Flight 1 `203957` | Flight 3 `213205` | Flight 2 `210705` (warm-reset test, not scored) |
|---|---|---|---|
| Firmware | `32cc6a2` | `32cc6a2` | `32cc6a2` |
| Receiver state before | B (after the p-series) | B (after pf2) | B (after pf1b) |
| First pad fix after RF start (tool) | 9.0 s | 33.0 s | n/a (about 10 s) |
| Boost / Coast / Descent / Landed vs truth (s) | +0.02 / +0.03 (from status packet) / +0.05 / +0.03 | +0.02 / +0.02 / +0.05 / +0.02 | +0.02 / +0.02 / after the reset (not scored) |
| Tool verdict | PASS | PASS | FAIL (reset at sim 70 s) |
| Filter up error max / RMS (m), launch to landing | 218.8 / 24.1 | 278.6 / 31.4 | not meaningful after the reset |
| Filter max % of height (floor 100 m) | 3.13 (40.0 s, floor effect) | 3.09 (43.0 s, floor effect) | |
| Time over 5 % | 0 % | 0 % | |
| GPS outage in the burn (no fix, sim time) | 36.13 to 40.01 s (3.9 s) | 36.13 to 43.13 s (7.0 s) | 36.12 to 62.37 s (26.2 s, 6 satellites at return) |
| Satellites before / at return | 11 / 5 | 11 / 5 | 11 / 6 |
| Radio state Hz, Boost / Coast / Descent / Landed (via Aperture) | 2.000 / 2.000 / 2.000 / 1.962 (one 700 ms gap) | 2.000 / 2.000 / 2.000 / 1.978 | |
| USB sensors Hz | 9.99 to 10.0 in all states | 9.98 to 10.0 | |
| Heap free / minimum (B) | 19,036 / 14,908 | 19,036 / 14,908 | same |
| Core 1 / core 0 load (range, mean) | 14.2 to 29.1 (19.1) / 11.1 to 14.8 (12.7) | 14.1 to 29.0 (19.1) / 10.5 to 14.5 (12.5) | |
| Resets, panic, watchdog, brownout, stack, queue, logger errors | none in flight (last reset SW from my Wi-Fi test before it) | none (last reset SW) | the commanded warm reset only |
| Commanded BIT (HITL off / on) | NO-GO [bench] both | same | same |
| Attitude final tilt error | 1.00 deg PASS | PASS | PASS until the reset |
| NAND log session | `144` | `149` | `146` |

Same as 11-19b: one NO-GO without the bench flag in the burn (baro plausible, simulated pressure outside the MS5607 range 67 s), the startup/commanded BIT blocking groups are radio power and HITL, `BIT: 12 task watchdog FAIL class 1 value 4` appears in every run's BIT list (also in 11-19b logs, not new). Compared with 11-19b the transitions (0.00 to +0.07 s), filter errors (follow the outage length: 3.9 s gives 219 m, 7.0 s gives 279 m, the same as 7.0 s in the reference), radio rates, heap and loads are unchanged: **no firmware or regulator effect on flight behaviour**. Flight 1 and 3 radio: 0 missed in Descent.

**Next power-up after the normal hub cycle:** not fast. The three tool cycles (20:50:34, 21:17:44, 21:42:43) were each followed by state A at 88,403 baud, no fix for 520 s (pf1), restart at 508.9 s of RF (pf2), and again 88,403 baud after the last one (end state check). So the "HITL tool puts the receiver back into A" finding from 11-19b is unchanged on the new regulator.

## Part 3: checks of `32cc6a2` fixes on hardware

**3a. No-IMU prediction: NOT RUN as asked; the one documented route I tried does not test it.**
* `0x4246` fault 3 (IMU silent) is documented and in the firmware (`bit_monitor.cpp`) as changing "only what the BIT sees, never a flight function": it sets `imu_ready=false` in the BIT view only. The position filter keeps using the IMU, so it cannot exercise the no-IMU predict path. I did not send it.
* The only documented command that puts the filter in the "no attitude, predict without the IMU" state in flight is the bench warm reset `0x4252` (the resume code in `sensors.cpp` says the attitude is not resumed). I ran it in flight 2 (`210705`) in Coast at sim 70.02 s with a GPS fix (reset acked, resumed Coast, telemetry back). Result: **the HITL-active bit drops with the reset** ("HITL is off after a reset"), the board continues on its real bench sensors, and the tool stops scoring ("later states come from the bench sensors"). The log therefore has no simulated data after the reset, so "AGL and vertical speed keep changing, Descent and Landed still declared" cannot be judged: Witness did log Descent at sim 114.50 s and Landed at 367.02 s, but on bench sensors, which proves nothing about the fix. Not a pass or fail.
* To test it properly I need a firmware or tool change (a HITL option that withholds the IMU frames in Coast while HITL stays on, or `0x4252` keeping HITL active); recommendation below.

**3b. Aperture uplink: PASS (informational notes).** Commands sent to Aperture on 7102 (binary command frames), acks read on 7102 (`0x06`). Pad Idle, HITL off:

| Test | Result |
|---|---|
| BIT report `0x5452` d0=0, BIT passive run `0x5442` (bench option), ping `0x0022` | ack for each: OK / OK / UNKNOWN (ping, as documented) in 80 to 140 ms |
| Burst of 8 back-to-back `0x5452` | 8 of 8 acks (0.12 to 3.23 s, in groups of 3 per 1.5 s = Aperture pacing); 2 acks BUSY detail `0x0401` |
| Burst of 12 back-to-back | 12 of 12 acks (0.11 to 4.77 s); 4 acks BUSY detail `0x0401` |
| Burst of 8 in the pad hold of flight 3 (HITL on, telemetry 2 Hz, sim 8 s) | 8 of 8 acks, 1 BUSY `0x0401`; flight and scores unaffected |
No BUSY with detail `0xA901` or `0xA902` appeared. The BUSY acks seen are Witness's own (`0x5452` while its single BIT request slot is taken, detail = BIT verdict word `0x040x`), not Aperture's. Note: **the first command sent after opening a new connection to the Aperture port was lost both times** (no ack, Witness logged no radio receive for it; the second command went through). 11-18 saw the same ("the one loss is at the first frame of round 1"). Not seen when the connection had been open for minutes (flight 3). I did not find the cause (Aperture parser after connect, or the ser2net open); for a flight ground station: keep the connection open, send a throwaway first.

**3c. Wi-Fi ON then OFF over LoRa via Aperture: PASS.** `0x5746` on sent through Aperture at 21:21:19: ack OK 0.21 s later; Witness logged "Wi-Fi on over radio: switching to the maintenance image (detail 0x0000)", restarted into the maintenance image (`32cc6a2`, compile 14:11:55, "NOT FLIGHT READY"), Wi-Fi AP started and reported up on channel 6 (about 0.56 s into the maintenance boot). `0x5746` off through Aperture at 21:21:44: ack OK 0.06 s later (the maintenance image accepted the radio command), Witness booted the flight image (`32cc6a2`, ELF `353150532...`, `selftest: running flight slot 0`, last reset SW) within the 30 s. The 60 s failed-AP-start timeout cannot be forced without code changes: **untested**.

## Open findings

1. **GPS slow power-up state A is not caused by the old over-voltage** (Short answer). The receiver is at 115.2 kbaud when Witness first reads it (2.5 s) and at 88.4 kbaud right after Witness's setup sequence starts. Candidate for the owner: a test build that does not send the setup (or sends it in another order) and just listens, to see whether the receiver stays at 115.2 kbaud and fixes quickly.
2. **Reset reason after a hub power cycle is WDT in 3 of 7 boots** (p3, the boot after flight 1's cycle = pf1/pf1b, and the boot after flight 3's cycle = the final one; POWERON in p1, p2, p4, after flight 2's cycle, and in all of 11-19b). When the reason is WDT, the BIT also reports "01 reset reason FAIL class 1 value 6" a few seconds after boot (p3, pf1b and flight 2 logs) and stays NO-GO in that item. Not seen on 11-19b power cycles. The cause is not clear to me: the first line of the boot is not captured on the 710x port (the USB reconnects), so I cannot see whether a first boot hung. Worth a check on the new regulator (supply droop at power-on, or the cycle being shorter than the rail discharge): a flight power-up should give POWERON, and a WDT-flagged boot costs a BIT NO-GO.
3. **No-IMU predict path has no HITL test route** (3a). Recommendation: let HITL stay active across a `0x4252` warm reset, or add a bench fault that really withholds the IMU feed in flight (`0x4246` fault 3 only changes the BIT).
4. **First command after opening the Aperture port is lost** (3b), twice here and once in 11-18.
5. **A glitch at connect:** when my recorder connected to 7100, Witness logged `unknown command` with a few bytes of garbage (once; not repeated in any other connect). Cosmetic.
6. **HITL tool:** after a no-fix lead-in it still continues silently (11-19b finding 2), unchanged; I set `power_cycle_retry` off as before so the tool never added a retry cycle.
7. **Aperture BUSY detail `0xA901`/`0xA902` never triggered** (3b), so the new retry path was not exercised by a real radio failure.

## Not done or limited

* 3a (above). The IMU-silent injection was not sent because it cannot do what was asked.
* The Wi-Fi AP failure timeout (60 s) not exercised.
* Only 6 power-ups were watched in state A (4 clean ones: p1 to p4, plus the two after the tool's cycles), not 11; each is enough to show A, the restart time spread is as before.
* The first boot line after a power cycle is not in any log (USB re-enumeration), so a first-boot hang cannot be excluded (finding 2).
* No measurement of the 3V3 rail by me (the owner's 3.3 V reading is taken as given).

## End state

Witness in Pad Idle, HITL off, flight image `32cc6a2` (boot after the last hub cycle, last reset **WDT**, see finding 2), logging to flash; the store was erased once more after the last export (new session 331 = 0x14B, status words `0x6600`/`0x6700`, no slots-full or flash-failed bit); all three run logs are in their folders. The receiver is in state A again (88,403 baud, `recoveries 0`) after the last cycle, so the next RF run needs conditioning or the 10-minute lead-in described in 11-19b. Aperture unchanged (not flashed, forwarding Witness packets). Ports 7100, 7101, 7102 free (my recorders are closed; the two other python processes on the desktop are not mine and were not touched). HackRF: no `hackrf_transfer` running, every stream ended by closing its stdin (all exited cleanly with code 0). Radio power untouched at -17 dBm. **Pi hub cycles: 7** (20:10:41, 20:16:00, 20:26:03, 20:32:06 by my script; 20:50:34, 21:17:44, 21:42:43 by the tool after the log export), never closer than 5 min 19 s. No `over-current` message in the Pi `dmesg` (checked before the first test and at the end; count 0). Nothing flashed, no source/config/firmware file changed, nothing committed; written: `Software/HITL/runs/20261119-c-powerup/`, the three flight run folders and this report.
