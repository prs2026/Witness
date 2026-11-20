# Bench report 2026-11-19e: A/B test of the HITL stream fault (baseline image against GPS-fix image)

**Scope.** Two images flashed one after the other, one HITL flight on each (B1 on the baseline, F3 on the GPS-fix image), then read-only bench checks. Run folders written: `Software/HITL/runs/20261119-e-ab/` (copied scripts in `bench_analysis/`, flash logs), flights `20261119-231106` (B1) and `20261119-232957` (F3). No source, config or firmware file was changed, nothing was committed (the only files touched outside the run folder are the images in `images/`, as the task required). Radio power untouched at -17 dBm. RF only into the closed coax path, HackRF TX VGA 0 dB, amp off, every stream ended by closing its stdin. Flashing only through the 7000 port, everything else through 710x.

## Short answer

**The fault is the bench/link, not the new firmware.** The baseline image (same code as `32cc6a2`, no GPS fix) shows the same fault as the GPS-fix image, in the same size.

| | 19c flights (`32cc6a2`) | B1 baseline `c908e12` | F3 GPS-fix `c908e12-dirty` | 19d flight A / B (GPS-fix) |
|---|---|---|---|---|
| `0x0C HITL frame rejected` | **0** / 0 | **238** | **353** | 287 / 325 |
| "unknown command" lines | 0 / 0 | 28 | 50 | 57 / 56 |
| "invalid EOF" lines (19d method) | 14 / 15 (GPS assist upload only) | 281 | 377 | 303 / ~333 |
| USB sensors Hz, Boost / Coast / Descent / Landed | 9.99 to 10.0 | 8.39 / 8.79 / 8.80 / 8.91 | 9.19 / 8.96 / 8.87 / 9.18 | 8.86 / 8.96 / 8.93 / 8.88 and 9.45 / 8.80 / 8.80 / 8.87 |

Both of mine show the fault, so by the rule given the cause is outside the firmware. Counts were made with the same method as 19d (my counts for the two 19d runs reproduce its numbers, 287/303 and 325/333). What changed on the bench cannot be proven from the logs of the earlier sessions, but the link I can see today is poor: **a 40-ping series from the desktop to the Pi gave 8 to 142 ms (mean 65 ms), 0 % loss**, over a direct (not relayed) VPN path; the desktop is on Wi-Fi for this. A burst of delayed bytes written at once to the Witness USB port would produce exactly the "invalid EOF; resynchronizing" bursts (several rejects in the same millisecond) and the 8.8 to 9.0 Hz sensor rate. This is the candidate, not a measured proof (see "Bench checks").

## Part 0: flash checks

Order: baseline flash, B1, hub cycle (tool), restore images, flash, F3. Every write printed "Hash of data verified".

| Step | `flash_verified.ps1 -CheckOnly` | Flash writes (all "Hash of data verified") | Boot log (0x5746 round trip) | App descriptor read from `images/` file | Equal |
|---|---|---|---|---|---|
| Baseline | 7000: ESP32-S3 running `softwarev1`, flight image at 0x20000, OK to flash | bootloader 21,232 B at 0x0; partition table 3,072 B at 0x8000; otadata 8,192 B at 0x10000; **flight.bin 449,808 B at 0x20000**; **maint.bin 851,856 B at 0x420000** | flight `c908e12`, compile 23:03:30, ELF SHA-256 `6aa45a00c...`, `selftest: running flight slot 0 (c908e12)`; maintenance `c908e12`, 23:04:33, ELF `caf8734bf...`, "boot partition restored to flight slot 0" | flight `c908e12`, 23:03:30, ELF `6aa45a00cb44f423...`; maint `c908e12`, 23:04:33, ELF `caf8734bf...ae840` | yes, both |
| GPS-fix | same, OK to flash | first attempt failed at connect ("Failed to set baud rate 115200", rfc2217 glitch, nothing written); second attempt wrote the same five regions, **flight.bin 450,320 B**, maint.bin 851,856 B, all verified | flight `c908e12-dirty`, 22:14:17, ELF `8b536b6fc...`; maintenance `c908e12-dirty`, 22:15:24, ELF `2a137ab36...` | flight `c908e12-dirty`, 22:14:17, ELF `8b536b6fc77b69b7...`; maint ELF `2a137ab3613076c8...` | yes, both |

`images/` before step 1 equalled `images_base/` (flight.bin identical); after step 3 all six files in `images/` (including `flash_args`) were compared by hash and equal those of `images_gpsfix/`. The baseline flight.bin has the same size (449,808 B) as the 19c `32cc6a2` image. Not used: flashit, plain `idf.py flash`. Flash logs (host names and MAC removed): `Software/HITL/runs/20261119-e-ab/flash_*.txt`.

## Part 1: flights

Same settings as 19d: profile `eayftrajectory.csv`, FAR site, 20 s pad hold, vibration scale 0.2, GPS from the HackRF over the coax, scenario `...-lead150-geom480`, tolerance 10 s, cold start on, `power_cycle_retry` off (checked in the prepared run and set to false), `flight.py` from the 19d folder unchanged, tool's normal hub cycle after the log export. Store erased (`0x6869`, status words `0x6600`/`0x6700`) before each flight; exported sessions equal the erase's new session (353 = 0x161, 364 = 0x16C).

| | B1 `20261119-231106` | F3 `20261119-232957` | 19d flight A / B |
|---|---|---|---|
| Firmware | `c908e12` (baseline) | `c908e12-dirty` | `c908e12-dirty` |
| Receiver before | **88,398 baud slow state, no fix** (see below) | 115,201 fast, after one hub cycle of mine | 115,200 fast |
| First pad fix after RF start (tool) | **none** (150 s lead-in, 0 satellites; the run continued, as asked) | 8.9 s, 11 satellites | 9.0 / 8.9 s |
| Boost / Coast / Descent / Landed vs truth (s, tool table) | +0.18 / +0.04 / **-2.25** / +0.09 | **+1.01** / +0.11 / +0.01 / +0.03 | +0.05 / +0.23 / +0.04 / +0.03 and +0.05 / +0.02 / +0.05 / +0.03 |
| Tool verdict | PASS | PASS | PASS / PASS |
| Filter up error max / RMS (m), launch to landing | 3,350 / 1,232 (no GPS all flight, dead reckoning only, not comparable) | 303.3 / 27.4 (at 39.3 s) | 359.3 / 33.3 and 300.8 / 27.3 |
| GPS in the flight | no fix in any GGA | fix lost 37.14 to about 38.0 s in the burn (23 no-fix GGA lines), then back | outage 3.1 / 2.9 s |
| Radio state Hz via Aperture, Boost / Coast / Descent / Landed | 2.000 / 2.000 / 2.000 / 2.000 | 2.000 / 2.000 / 1.990 (3 gaps of 1 s) / 2.000 | 2.000 / 1.989 / 1.997 / 2.000 and 2.000 in all |
| USB sensors Hz, Pad idle / Boost / Coast / Descent / Landed | 9.87 / 8.39 / 8.79 / 8.80 / 8.91 | 9.77 / 9.19 / 8.96 / 8.87 / 9.18 | Boost to Landed 8.8 to 9.5 |
| USB state Hz | 2.000 in all | 2.000 in all | 2.000 |
| Heap free / minimum (B) | 19,036 / 14,908 | 19,036 / 14,908 | same |
| Core 1 / core 0 load (range, mean) | 12.8 to 27.1 (17.5) / 9.9 to 12.7 (10.8) | 12.2 to 27.8 (17.3) / 10.6 to 14.0 (11.7) | 12.2 to 28.1 (about 18) / 10.9 to 14.1 (11.9) |
| Resets, panic, watchdog, brownout, stack, queue, logger errors | none; last reset SW (my 0x5746 round trip) | none; last reset POWERON | none |
| Attitude final tilt error | 0.64 deg PASS | 2.99 deg PASS (the tool's last attitude sample is from Descent, sim 238.3 s, nose error 90.8 deg; flight A of 19d likewise ended at Descent, sim 246.6 s) | 0.20 / 0.80 deg |
| **`0x0C HITL frame rejected`** | **238** (231 "invalid EOF; resynchronizing", 7 "incomplete frame followed by idle input") | **353** | 287 / 325 |
| "unknown command" / "USB command rejected" / "external command forwarding failed" | 28 / 31 / 31 | 50 / 23 / 22 | 57 / ... |
| "invalid EOF" lines (all, incl. GPS assist upload "discarded UART packet": 19 / 14) | 281 | 377 | 303 / ~333 |
| NAND log session | `161` (353) | `16C` (364) | `156`, `158` |

Notes. (1) The baseline boot put the receiver into the slow state: before the flash the receiver was in the fast state (the 19d end state, 115,201 baud); after the baseline image's boot it was at 88,398 baud with no fix, so B1 flew without GPS, as the task expected. After the GPS-fix flash it still was at 88,398 baud (its one boot query-first logic does not pull it out of the slow state); I did one hub cycle (23:28:03) before F3 to get the same fast, fix-capable start as the 19d flights. So B1 (no GPS) and F3 (GPS) differ in GPS state; the fault appears in both, which also shows it does not depend on the GPS or on the receiver baud (88.4 or 115.2 kbaud). (2) First rejected frames of B1: sim 10.78 s "incomplete frame followed by idle input", then from sim 26.84 s bursts of 4 or more "invalid EOF; resynchronizing" in the same millisecond, each burst followed by "unknown command" and "USB command rejected". F3 starts at sim 1.15 s (first reject line `invalid EOF; resynchronizing`). (3) Largest transition errors of the day, both within the 10 s tolerance: F3 Boost +1.01 s (late; lost IMU frames in the burn detection) and B1 Descent -2.25 s (early, no GPS). The 19d "+0.23 s" Coast of flight A is no longer the worst. (4) The "attitude after flight unavailable: no stationary pad window" line appears in 19c, 19d and F3 flights (1 to 2 times), not in B1; it comes after landing and is not new.

## Part 2: bench checks (read-only)

* **Other clients on 7100:** during B1 (23:11) and F3 (23:30) the Pi showed exactly one established connection on 7100 (the flight tool) and one on 7102 (my Aperture recorder); after the flights none. On the desktop, no connection to ports 7000/7001/7002/7100/7101/7102 owned by any other process (only a TIME_WAIT of my own). Nothing of mine or anyone else's was attached to 7100 besides the HITL tool. The two python processes of 11/18 (not mine, untouched) hold no connection to the Pi.
* **ser2net on the Pi:** one process, running 2 days 14 h since the last Pi boot (unchanged during all sessions), 0.4 % CPU, service active; 7100 is a raw TCP endpoint (7000 range = rfc2217), all at 115200n81 in the config. Send-Q on 7100 during flights 217 to 327 B (normal).
* **Pi dmesg:** no USB errors, no over-current, no under-voltage (count 0 at the start, during F3 and at the end); the only events are the hub-cycle disconnects/enumerations and Witness's own USB re-enumerations at reset. `get_throttled` 0x0. Pi load average 0.1 to 0.6, 55 C. Ethernet and Wi-Fi both up on the Pi (eth0 0 errors, 0 drops; wlan0 36 TX drops over its uptime).
* **Desktop:** total CPU 1 to 4 % during the flights (sampled during B1 and F3); Wi-Fi link 802.11ax, 88 % signal, Ethernet also up, no NIC errors or discards.
* **Link quality (the one anomaly):** 40 pings to the Pi: 8 to 142 ms, mean 65 ms, 0 loss; the VPN reports a direct path (113 ms for that sample). A wired LAN path would give 1 to 3 ms. The earlier sessions did not record this, so I cannot say it got worse since 19c; it is the only abnormal reading found. The ser2net config says the serial ports are served on the Pi's VPN address, so the HITL stream (up to 960 Hz IMU frames in the host-to-Witness direction, no flow control) crosses this path.

**Reading of the evidence.** The same firmware logic fails the same way on two images, the Pi, Witness loads, heap and USB side show nothing abnormal, nobody else is on the port, and the network path shows large latency swings. The 19c flights (older image, 0 rejects, sensors 10.0 Hz) are the exception that needs an explanation by the path (for example Wi-Fi or relay conditions then against now), but the test cannot show what the path was then. I did not change any network setting.

## Open findings and recommendations (no code changed)

1. **The HITL stream fault is not caused by the GPS-fix firmware** (B1: 238 rejects, 8.4 to 8.9 Hz sensors on the baseline). Do not hold the GPS fix for it.
2. **Recommended next step on the bench (not done, I may not change network settings):** put the desktop on the wired connection to the Pi's network, or run the HITL tool on the Pi, and fly once; expect 0 rejects. A ping log running next to a flight would show whether the rejects follow the latency swings.
3. **Robustness idea for the firmware, not necessary for the bench:** the Witness USB receive path loses bytes when the Pi delivers a backlog at once (rejects come in bursts of 4 or more in one millisecond, then resynchronize). A bigger USB RX buffer or lower per-flush frame batches on the host side (tool pacing) would make the link tolerant.
4. **GPS slow state after a Witness-only reset on the baseline image:** the receiver was fast before the baseline's boot and slow (88,398 baud) after it, with no hub cycle in between. This supports the 19d diagnosis (the 25 Hz command on a receiver that already runs at 25 Hz). With the GPS-fix image after one hub cycle the receiver came up fast again (115,201 baud, fix at 8.9 s). A Witness-only reset on the GPS-fix image while the receiver is slow does not get it out of the slow state (it needs a hub cycle, as before).
5. Carried over unchanged from 19c/19d: first command after opening the Aperture port is lost, no-IMU predict path has no HITL route, Wi-Fi AP 60 s timeout untested.

## Not done

* The baseline image's GPS-slow-state behaviour was not conditioned away (as asked), so B1 has no GPS data to compare filter errors with 19d.
* No third flight, so the "wired path" test in finding 2 is not run; no recording of ping latency during a flight.
* No measurement of the link in the earlier sessions exists, so "what changed since 19c" stays a candidate (the link) not a proven cause.

## End state

Witness in Pad Idle (state packets show status words `0x6600`/`0x6700`, no HITL lines), HITL off, running the **GPS-fix image** (`c908e12-dirty`, ELF `8b536b6fc...`, last reset POWERON from the tool's hub cycle after F3), store erased once more after the F3 export (new session 366 = 0x16E, status words `0x6600`/`0x6700`). Receiver at 115,201 baud, 25 Hz, `recoveries 0` (no fix now, nothing transmitting). Ports 7100, 7101, 7102 free (my recorders closed; the two python processes of 11/18 not mine and untouched). No `hackrf_transfer` running (desktop and Pi checked; every stream ended by closing stdin). Radio power untouched at -17 dBm. **Pi hub cycles this session: 3** (tool after B1 at 23:21:45, mine at 23:28:03, tool after F3 at 23:40:36; gaps 6 min 18 s and 12 min 33 s; the previous cycle, the tool's after 19d flight B at about 22:59:20, was 22 min before the first); with 19d's 6 and 19c's 7 that is 16 since the start of 19c. No over-current in the Pi dmesg (count 0). Flashed twice through 7000 (the GPS-fix flash needed a second attempt after a connect error that wrote nothing; ten verified writes in total); Aperture and Iris not flashed. `images/` now holds the GPS-fix set (hash-equal to `images_gpsfix/`); `images_base/` untouched. Nothing in source, config or firmware was changed, nothing committed; written: `Software/HITL/runs/20261119-e-ab/`, the two flight run folders and this report.
