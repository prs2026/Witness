# Bench report 2026-11-19d: GPS slow power-up fix (query the update rate first, skip the 25 Hz command) flashed and tested

**Scope.** Flash of the uncommitted GPS fix (version stamp `c908e12-dirty`), four hub power-ups with RF, two full HITL flights, the next power-up after each tool hub cycle. Run folders written: `Software/HITL/runs/20261119-d-powerup/` (power-up logs, copied scripts in `bench_analysis/`), flights `20261119-223558` and `20261119-224837`. No source, config or firmware file was changed, nothing was committed. Radio power untouched at -17 dBm (BIT NO-GO [bench] for radio power expected). RF only into the closed coax path, HackRF TX VGA 0 dB, amp off, every stream ended by closing its stdin. Flashing only through the 700x port, everything else through the 710x ports; Aperture and Iris not touched.

## Short answer

**Yes. The fix removed the slow power-up state in 6 of 6 power-ups.** After each hub power cycle (4 of mine, 2 by the HITL tool after the log export) Witness identified the receiver at **115,200 baud** (no 88.4 kbaud, the "slow state" warning never appeared, `recoveries 0`, no "went silent during configuration", no self-restart), and the line `PX1120S already at 25 Hz: update-rate command not sent` appeared in every boot. With RF (cold start, ephemeris upload, 21:40 GPS 480 s scenario) the first fix came **8.6 to 8.8 s after RF start** in all 6 (4 to 5 satellites at the fix, 11 about 5 s later), receiver clock set at 8.4 s. In 11-19c the same test gave 88.4 kbaud, no fix, and a self-restart after 168 to 509 s (or none in 520 s). The tool's hub cycle no longer puts the receiver back into the slow state either.

**One surprise: the new image's flights show a new fault in the host-to-Witness HITL stream** (287 and 325 rejected `0x0C HITL frame ... invalid EOF` frames per flight, none in 11-19c's flights), the USB sensor packets drop to 8.8 to 9.0 Hz (10.0 Hz before) and the filter errors are larger. Both flights still PASS. Whether it is the new firmware or the link is not determined (see finding 1).

## Part 0: flash check

`flash_verified.ps1 -CheckOnly` on 7000: ESP32-S3 running `softwarev1` (flight image at flight_a), OK to flash. `flash_verified.ps1` then wrote five regions, every write printed "Hash of data verified": bootloader 21,232 B at 0x0, partition table 3,072 B at 0x8000, otadata 8,192 B at 0x10000, **flight.bin 450,320 B at 0x20000**, **maint.bin 851,856 B at 0x420000**; hard reset after. Not used: flashit, plain `idf.py flash`, the root sdkconfig.

| Image | Boot log (0x5746 round trip, 22:18) | App descriptor in the `images/` file | Result |
|---|---|---|---|
| flight | `c908e12-dirty`, compile Nov 19 22:14:17, ELF SHA-256 `8b536b6fc...`, `selftest: running flight slot 0 (c908e12-dirty)` | `flight.bin` 450,320 B, `c908e12-dirty`, 22:14:17, ELF `8b536b6fc77b69b7...` | equal |
| maintenance | `c908e12-dirty`, 22:15:24, ELF `2a137ab36...`, "boot partition restored to flight slot 0" | `maint.bin` 851,856 B, `c908e12-dirty`, 22:15:24, ELF `2a137ab3613076c8...` | equal |

The version stamp ends in `-dirty` as expected. Side observation from that first round trip: the receiver was still in the old slow state (left over from the last 11-19c power-up, no hub cycle since): the probe measured 88,398 baud, the new warning "GPS receiver is in its ~88.4 kbaud slow state (115200 x 0.767): expect no fix until it restarts itself" appeared, and after identification at 88,398 baud "PX1120S already at 25 Hz: update-rate command not sent (1 time(s))" appeared. So both new log lines work.

## Part 1: GPS power-up test

Method as in 11-19c (`powerup.py`): hub cycle (`uhubctl -l 2 -a 2`), Witness boot recorded on 7100, 70 s after the cycle cold start `0x4752` ("GGA+RMC confirmed" waited for), then HackRF streaming the 21:40 GPS 480 s scenario with the 31 ephemeris records; time zero = HackRF streaming; give up at 520 s (not needed). Results next to the 11-19c table:

| # | Hub cycle at | Receiver at boot (Witness) | Config lines | "already at 25 Hz ... not sent" | 88.4k warning / silent / recovery | Lock (clock set) | Self-restart | First fix (satellites at fix / ~5 s later) | Last reset reason |
|---|---|---|---|---|---|---|---|---|---|
| p1 | 22:20:41 | **115,200** (identified at 2.5 s) | "airborne 4 g mode and 25 Hz GGA+RMC confirmed at 115200 baud (ready, configured)" at 2.6 s | yes, 2.6 s | none / none / none | 8.4 s | none needed | **8.8 s** (5 / 11) | POWERON |
| p2 | 22:25:03 | 115,200 | same | yes | none | 8.4 s | none | **8.6 s** (4 / 11) | POWERON |
| p3 | 22:29:24 | 115,200 | same | yes | none | 8.4 s | none | **8.7 s** (4 / 11) | POWERON |
| p4 | 22:33:45 | 115,200 | same | yes | none | 8.4 s | none | **8.8 s** (4 / 11) | POWERON |
| pf1 (boot after the tool's hub cycle after flight 1, cycle about 22:46:35; RF started 70 s uptime later) | about 22:46:35 | 115,200 (receiver steady at 115,201, `recoveries 0`; the boot lines themselves were before my recorder, the count "2 time(s)" in the log shows the boot one) | confirmed at 115200 | yes (count 2 after the cold-start reconfiguration) | none | 8.4 s | none | **8.6 s** (4 / 11) | POWERON |
| pf2 (after the tool's hub cycle after flight 2, cycle about 22:59:20; RF at 36 s uptime) | about 22:59:20 | 115,200 (115,201) | confirmed at 115200 | yes (count 2) | none | 8.4 s | none | **8.7 s** (5 / 8) | POWERON |

For comparison 11-19c: A (88,398 to 88,403 baud) in 6 of 6, fix only after a self-restart (168 to 509 s) or none in 520 s. Here: 6 of 6 at 115,200, fix after 8.6 to 8.8 s. A fix this fast also means the cold start (clock back to 1980) plus the 692-frame assist upload is acquired in the same time as the state-B case of 11-19b/c (8.5 to 9 s). The first boot line of p1 to p4 (the probe lines at about 1 s) is lost because the 710x USB port re-enumerates at power-up, so the "baud the probe measures" is read from the identification line ("GPS identified as PX1120S at 115200 baud", 2.53 to 2.57 s) and from the periodic `GPS RX: baud=115201` line; no 88,4xx value appeared in any of the 6 power-ups.

Hub cycles of mine were 4 min 21 s apart (22:20:41, 22:25:03, 22:29:24, 22:33:45); the first one was more than 40 min after the previous (21:42:43, 11-19c).

## Part 2: HITL flights

Profile `eayftrajectory.csv`, FAR site, 20 s pad hold, vibration scale 0.2, GPS from the HackRF over the coax, scenario `...-lead150-geom480`, tolerance 10 s, run through `HitlSession` with `flight.py` as flights 1 and 3 of 11-19c (cold start on, `power_cycle_retry` off), **with the tool's normal hub cycle after the log export**. Store erased (`0x6869`, status words `0x6600`/`0x6700`) before each flight; exported session numbers equal the erase's new session: 342 = `0x156` (flight A), 344 = `0x158` (flight B). The receiver was already in the fast state from the power-up series, so no conditioning stream was needed. I made a second flight because the first one showed the stream fault below; it is an extra, not asked for.

| | Flight A `20261119-223558` | Flight B `20261119-224837` | 11-19c flight 1 / flight 3 |
|---|---|---|---|
| Firmware | `c908e12-dirty` | `c908e12-dirty` | `32cc6a2` |
| Receiver state before | 115,200, fast | 115,201, fast | B after conditioning |
| First pad fix after RF start (tool) | 9.0 s | 8.9 s | 9.0 s / 33.0 s |
| Boost / Coast / Descent / Landed vs truth (s) | +0.05 / **+0.23** (log) / +0.04 / +0.03 | +0.05 / +0.02 / +0.05 / +0.03 | +0.02 / +0.03 / +0.05 / +0.03 and +0.02 / +0.02 / +0.05 / +0.02 |
| Tool verdict | PASS | PASS | PASS / PASS |
| Filter up error max / RMS (m), launch to landing | **359.3 / 33.3** (at 40.2 s) | **300.8 / 27.3** (at 39.8 s) | 218.8 / 24.1 and 278.6 / 31.4 |
| Filter max % of height (floor 100 m) | 18.24 (Boost, floor effect at about 340 m height); 4.19 after 25 s | 21.35 (Boost, floor effect); 4.74 after 25 s | 3.13 and 3.09 |
| Time over 5 % | 0.2 % (32 of 19,988 rows) | 0.1 % (23 rows) | 0 % |
| Error in Boost (max / RMS, m) | 62.9 / 17.3 | 27.9 / 9.1 | 11.6 / 5.9 and 10.8 / 5.5 |
| GPS outage in the burn (no fix, sim time) | 37.14 to 40.26 s (3.1 s) | 37.13 to 40.01 s (2.9 s) | 3.9 s and 7.0 s |
| Satellites before / at return | 11 / 5 | 11 / 5 | 11 / 5 |
| Radio state Hz via Aperture, Boost / Coast / Descent / Landed | 2.000 / 1.989 (one 999 ms gap) / 1.997 (one 1000 ms gap) / 2.000 | 2.000 / 2.000 / 2.000 / 2.000 | 2.000 / 2.000 / 2.000 / 1.962 and 1.978 |
| USB sensors Hz, Boost / Coast / Descent / Landed | **8.86 / 8.96 / 8.93 / 8.88** | **9.45 / 8.80 / 8.80 / 8.87** | 9.99 to 10.0 |
| USB state Hz | 2.000 in all | 2.000 in all | 2.000 |
| Heap free / minimum (B) | 19,036 / 14,908 | 19,036 / 14,908 | 19,036 / 14,908 |
| Core 1 / core 0 load (range, mean) | 12.2 to 27.7 (17.7) / 11.1 to 14.1 (11.9) | 12.9 to 28.1 (18.2) / 10.9 to 14.0 (11.9) | 14.2 to 29.1 (19.1) / 11.1 to 14.8 (12.7) |
| Resets, panic, watchdog, brownout, stack, queue, logger errors | none; last reset POWERON in the CPU line | none; POWERON | none |
| Commanded BIT (HITL off / on) | NO-GO [bench] both | same | same |
| Attitude final tilt error | 0.20 deg PASS | 0.80 deg PASS | 1.00 deg PASS |
| **Rejected HITL frames (`0x0C HITL frame rejected: invalid EOF`) / `invalid EOF` lines** | **287 / 303** | **325 / ~330** | **0 / 14 and 15** (only the GPS-assist-upload "discarded UART packet") |
| NAND log session | `156` (342) | `158` (344) | `144`, `149` |

Details of the stream fault: the rejected frames are in the host-to-Witness direction (the HITL IMU frames, 42,971 frames sent by the tool in each run). They start at sim 4 s (flight A) and continue through Coast and Descent to Landed (A: 47 before 20 s, 53 in Coast, 187 in Descent and later; B: 139 in Coast, 186 later), each time followed by "USB command rejected: invalid EOF" and "external command forwarding failed: ESP_ERR_INVALID_ARG" (11 and 8 times). The flights still PASS (state times within 0.02 to 0.23 s of truth, the +0.23 s Coast transition of flight A is the largest of any flight in 11-19b/c) but the filter errors in Boost and Coast are 2 to 5 times those of 11-19c's flights with an outage of 3 s (flights with 3.9 s outage then gave 219 m, now 3.1 s gives 359 m and 2.9 s gives 301 m), consistent with lost IMU frames. Everything else (radio rates, loads, heap, BIT) is as before. I did not find the cause. Candidates: (a) a new firmware effect on the USB receive path (the fix touches `uart_driver.cpp`; with the receiver now at 115.2 kbaud from power-up the UART traffic is higher than in the 88.4k boots, but the 11-19c flights also ran at 115.2 kbaud after the receiver restart and had none); (b) the link (Pi serial server or desktop load) - the two python processes on the desktop that are not mine were running, as before. An A/B test would be: flash the `32cc6a2` flight image back (flight.bin of that build) and fly the same flight; I did not do that (no flashing beyond the requested image).

## Reset reasons and BIT after the hub cycles

* **Last reset reason after the hub power cycles: POWERON in 6 of 6** (p1 to p4, after flight A's tool cycle, after flight B's tool cycle). 11-19c had WDT in 3 of 7. A WDT boot did not occur here. The boot before p1 was the 0x5746 round trip (SW, expected).
* BIT in the four fresh boots: `12 task watchdog: FAIL class 1 value 4` and `13 nvs: FAIL class 1 value 2` appear in every boot's BIT list (about 2.6 s after boot, again in later BIT runs); `28 body orientation`, `46 calibration`, `47 pad heading set`, `83 radio power (bench)` and `B1 battery voltage` also FAIL as on the bench in 11-19c. `12 task watchdog` also appears in every flight's BIT list (as in 11-19b/c, not new), `13 nvs` in the boot BIT and in some of flight B's BIT runs. No `01 reset reason FAIL` was seen in any of these boots (it came with the WDT boots in 11-19c).
* No `over-current` in the Pi `dmesg` (count 0 before the power-up series and at the end).

## Open findings

1. **New HITL stream fault in both flights with the new image** (Part 2): 287 and 325 rejected `0x0C` frames per flight, USB sensor packets 8.8 to 9.0 Hz instead of 10.0 Hz, larger filter errors, Coast transition +0.23 s once. Not in any 11-19b/c flight. Needs a decision: A/B flight with the previous image (`32cc6a2` flight.bin) to tell the firmware from the link. I did not flash the old image.
2. **GPS slow power-up state is gone** (6 of 6). The cause was the 0x0E 25 Hz command sent to a receiver that already runs at 25 Hz; the query-first logic removes it. Whether a receiver whose flash has another rate would still hit it is not tested (the receiver on this bench has 25 Hz in its flash).
3. The first boot line (probe baud) after a hub cycle is still not captured (USB re-enumeration); the baud is read from the identification line instead.
4. Findings carried over unchanged from 11-19c and not retested here: first command after opening the Aperture port is lost, no-IMU predict path has no HITL route, Wi-Fi AP 60 s timeout untested.

## Not done

* The 11-19c power-up table was reproduced with 6 power-ups (4 mine, 2 after tool cycles), each with RF time to fix; the "last 520 s give-up" was never needed.
* The pf1 and pf2 boot lines before about 70 s and 36 s of uptime are not in the recordings, so for those two the "already at 25 Hz" line is shown by its count (2 after the cold-start reconfiguration), not by the first boot.
* Not checked: other boards on the hub (the owner confirmed there is no second Witness board; that check was skipped as instructed).
* No A/B with the previous firmware (finding 1).

## End state

Witness in Pad Idle, HITL off (state packets show Pad idle, "HITL mode exited" in the last flight log), flight image `c908e12-dirty` (compile 22:14:17), last reset POWERON, logging to flash; store erased once more after the last flight (new session 346 = 0x15A, status words `0x6600`/`0x6700`, no slots-full bit). Receiver at 115,201 baud, `recoveries 0`, fast (receiver clock running from the last RF stream; it is not cold). Ports 7100, 7101, 7102 free (my recorders closed; the two python processes on the desktop are not mine and were not touched). HackRF: no `hackrf_transfer` running (checked on desktop and on the Pi, every stream ended by closing its stdin, all exited with code 0). Radio power untouched at -17 dBm. **Pi hub cycles: 6** (mine 22:20:41, 22:25:03, 22:29:24, 22:33:45; the tool's after flight A about 22:46:35 and after flight B about 22:59:20), never closer than 4 min 21 s. No `over-current` message in the Pi `dmesg` (count 0). Flashed once through 7000 (five writes, all verified); Aperture and Iris not flashed. Nothing in source, config or firmware was changed, nothing committed; written: `Software/HITL/runs/20261119-d-powerup/`, the two flight run folders and this report.
