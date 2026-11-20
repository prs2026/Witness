# Bench report 2026-11-20: candidate flight image 3a66505 (committed GPS fix) flashed and verified

**Scope.** Flash of the clean build from commit `3a66505`, boot and maintenance-image checks, three hub power-ups (two with RF), one full HITL flight, then the BIT lists asked for checks 12 and 13. Run folders written: `Software/HITL/runs/20261120-f-verify/` (copied scripts in `bench_analysis/`, flash logs with host name, MAC and user path removed, power-up logs) and the flight `20261120-000822`. No source, config or firmware file was changed by me, nothing was committed. (At the end `git status` shows `main/bit_logic.h` and `tests/bit_logic_test.cpp` as modified; they were not modified at the start of the session and not by me.) Radio power untouched at -17 dBm. RF only into the closed coax path, HackRF TX VGA 0 dB, amp off, every stream ended by closing its stdin. Flashing only through the 7000 port, everything else through 710x; Aperture and Iris not flashed.

## Short answer

**Image 3a66505 is verified for flash, boot and GPS. The HITL flight passes the tool's criteria, but it is not as clean as the 19d/19e flights and has two points to look at.**

* **Flash:** `flash_verified.ps1 -CheckOnly` OK, then the flash worked at the first attempt (no connect error). All five writes printed "Hash of data verified" (flight.bin 450,320 B, maint.bin 851,856 B).
* **Boot:** version `3a66505`, `selftest: running flight slot 0 (3a66505)`, no "pending verification" in 70 s of log. Both ELF hashes in the boot logs equal the app descriptors of `images/flight.bin` and `images/maint.bin`; the 0x5746 round trip came back to the flight slot.
* **GPS:** 3 of 3 power-ups at **115,200 baud**, never the 88.4 kbaud slow state, no warning, `recoveries 0`. `PX1120S already at 25 Hz: update-rate command not sent` appeared in every boot where the boot line was captured (p1 directly; p2 and p3 by the count 2 / by the steady 115,201 baud, the boot lines are lost to the USB re-enumeration as in 19d). With RF the first fix was **8.7 s in both RF power-ups** (receiver clock set at 8.4 s).
* **Flight:** tool verdict PASS (Boost +0.20 / Coast +0.24 / Descent +0.05 / **Landed +3.27 s**, tolerance 10 s), attitude PASS 0.59 deg. 346 `0x0C HITL frame rejected` frames: the known bench-link issue (19e), same size as 19d/19e (238 to 353).
* **Points to look at:** (1) Landed transition 3.27 s late (all earlier flights of 19c to 19e: at most 0.09 s). (2) GPS outage in the burn of 7.9 s (sim 36.15 to 44.02 s) against 2.9 to 3.1 s in 19d, filter max up error 374.9 m / RMS 42.3 m (19d: 359 / 33 and 301 / 27; 19e F3: 303 / 27). Both look like effects of the lost-frame fault, not of the image, but I have not proved that. (3) The boot after the tool's post-flight hub cycle reported **last reset WDT** and `01 reset reason: FAIL class 1 value 6` (see BIT section); the three other hub boots reported POWERON.

## Part 0: flash and boot checks

| Step | Result |
|---|---|
| `flash_verified.ps1 -CheckOnly` | 7000: ESP32-S3 running `softwarev1` (flight image) at 0x20000, OK to flash |
| `flash_verified.ps1` | first attempt worked. bootloader 21,232 B at 0x0; partition table 3,072 B at 0x8000; otadata 8,192 B at 0x10000; **flight.bin 450,320 B at 0x20000**; **maint.bin 851,856 B at 0x420000**. Every write "Hash of data verified". Hard reset after. |
| Not used | flashit.ps1/.sh, plain `idf.py flash` |

| Image | Boot log (0x5746 round trip) | App descriptor read from the `images/` file | Equal |
|---|---|---|---|
| flight | `3a66505`, compile Nov 19 23:54:15, ELF `3fa3a7f5f...`, `selftest: running flight slot 0 (3a66505)`; no "pending", "rollback" or "verif" text in 70 s | `flight.bin` 450,320 B, `3a66505`, 23:54:15, ELF `3fa3a7f5f7ec3dfe1b6e1ef2fad2594117f65574f4a14486271a638a0f815a03` | yes |
| maintenance | `3a66505`, compile 23:55:18, ELF `24a4a024e...`, "boot partition restored to flight slot 0: any reset returns to flight", Wi-Fi AP up | `maint.bin` 851,856 B, `3a66505`, 23:55:18, ELF `24a4a024e4a9ce09b93b8389ba07c15b535070f5e17c8b2c3a44b7c70a854376` | yes |

The version stamp is `3a66505` without `-dirty`. Witness returned to the flight slot (the round trip ended in a flight boot, last reset SW).

## Part 1: GPS power-up table

Method as 19d (`powerup.py`): hub cycle `uhubctl -l 2 -a 2`, Witness boot recorded on 7100, 70 s after the cycle cold start `0x4752`, then HackRF streaming the 21:40 GPS 480 s scenario with the 31 ephemeris records, give up at 520 s (not needed). Hub cycles: p1 00:01:40, p2 00:06:15 (4 min 35 s after p1), p3 00:23:52 (the first of the session was 21 min after the last one of 19e, 23:40:36).

| # | Hub cycle | Identification baud | "already at 25 Hz ... not sent" | 88.4k warning / silent / recovery | Last reset | BIT list after boot (about 2.6 s) | Lock / first fix with RF (sats at fix / ~5 s later) |
|---|---|---|---|---|---|---|---|
| p1 | 00:01:40 | **115,200** ("GPS identified as PX1120S at 115200 baud", 2.57 s; steady `baud=115201`) | yes at 2.65 s (1 time), 2 times after the cold-start reconfiguration | none / none / none, `recoveries 0` | POWERON | 12, 13, 28, 46, 47, 83, B1 | RF: clock set 8.4 s, **first fix 8.7 s** (7 / 10 to 11) |
| p2 | 00:06:15 | 115,200 (boot lines lost to the USB re-enumeration; "confirmed at 115200 baud" at 67.9 s, steady `baud=115201`) | count "2 time(s)" at 67.9 s (so the boot one happened) | none / none / none, `recoveries 0` | POWERON | same | RF: clock 8.4 s, **first fix 8.7 s** (4 / 11) |
| p3 | 00:23:52, no RF (asked: the next power-up after the flight) | 115,200 only by the steady `GPS RX: baud=115201 ... PX1120S ready (configured) ... 25 Hz, recoveries 0` line from 5.1 s; the identification and "already at 25 Hz" lines (before about 7 s uptime) were **not captured** | not captured | none / none / none | POWERON | 12, 13, 28, 46, 47, 74, 75, 83, B1, C4 (at 46 s, no fix) | no RF |

So "next power-up's identification line": I did not get it. The tool's own cycle after the flight (about 00:19:05) and my p3 both lost the first seconds to the USB re-enumeration; what is visible is a steady 115,201 baud, 25 Hz, `recoveries 0`, no slow-state warning (0 occurrences in all three recordings, no `88xxx` baud value anywhere).

## Part 2: HITL flight

Profile `eayftrajectory.csv`, FAR site, 20 s pad hold, vibration scale 0.2, GPS from the HackRF over the coax, scenario `...-lead150-geom480`, tolerance 10 s, cold start on, `power_cycle_retry` off, `flight.py` of 19d/19e unchanged, tool's normal hub cycle after the log export. Store erased (`0x6869`, status words `0x6600`/`0x6700`) before the flight; the exported session 177 = 0x177 equals the erase's new session 375. Receiver was fast (115,201) from p2. Flight run `20261120-000822`.

| | Flight `20261120-000822` (3a66505) | 19e F3 (`c908e12-dirty`) |
|---|---|---|
| First pad fix after RF start (tool) | 9.0 s, 11 satellites | 8.9 s |
| Boost / Coast / Descent / Landed vs truth (s) | +0.20 / +0.24 / +0.05 / **+3.27** | +1.01 / +0.11 / +0.01 / +0.03 |
| Tool verdict | PASS | PASS |
| Filter up error max / RMS (m), launch to landing | 374.9 / 42.3 (max 18.35 % of height at 43.0 s) | 303.3 / 27.4 |
| Phase max up error (m): Boost / Coast / Descent / Landed | 72.3 / 374.9 / 146.0 / 0.0 | |
| GPS in the flight | no fix sim 36.15 to 44.02 s (7.9 s), 5 sats at return, 11 by sim 100 s | fix lost about 0.9 s |
| Radio state Hz via Aperture, Boost / Coast / Descent / Landed | 2.000 / 1.989 (one 1 s gap) / 2.000 / 1.999 | 2.000 / 2.000 / 1.990 / 2.000 |
| USB sensors Hz, Pad idle / Boost / Coast / Descent / Landed | 9.86 / 8.94 / 8.95 / 8.85 / 9.12 | 9.77 / 9.19 / 8.96 / 8.87 / 9.18 |
| USB state Hz | 2.000 / 2.000 / 2.000 / 1.997 (one 1 s gap) / 2.000 | 2.000 |
| Heap free / minimum (B) | 19,036 / 14,908 | same |
| Core 1 / core 0 load (range, mean) | 12.4 to 27.8 (18.1) / 10.7 to 13.8 (11.9) | 12.2 to 27.8 / 10.6 to 14.0 |
| Resets, panic, watchdog, brownout, stack, queue, logger errors | none; reset reason POWERON in all 49 CPU lines | none |
| Attitude final tilt error | 0.59 deg PASS (last sample from Descent, sim 392.9 s, nose error 8.07 deg) | 2.99 deg PASS |
| **`0x0C HITL frame rejected`** | **346** (known bench-link issue) | 353 |
| "unknown command" / "USB command rejected" / "external command forwarding failed" / "invalid EOF" lines / "discarded UART packet" (GPS assist upload) | 66 / 34 / 34 / 383 / 14 | 50 / 23 / 22 / 377 |
| NAND session | `177` (375) | `16C` |

Counts made with the 19e method (`cnt.py`). The first reject is at sim 0.05 s, the last at sim 413.95 s ("incomplete frame followed by idle input"). No over-current in the Pi dmesg (count 0 before, during and at the end), `get_throttled` 0x0.

Notes. (1) The Landed transition (expected sim 419.73 s, Witness 423.00 s) is 3.27 s late; every earlier flight of 19b to 19e was within 0.09 s. Descent is on time (+0.05 s). The simulated pressure is below the MS5607 range for 67 s (tool warning, as in earlier runs). I did not find the cause; a lost-frame effect on the landing detection is plausible. (2) The 7.9 s GPS outage in the burn is the longest of the 19c to 19e flights (3.9 and 7.0 s in 19c, 2.9 to 3.1 s in 19d, 0.9 s in F3). It correlates with the 374.9 m maximum filter error; the tool still passes. (3) Commanded BIT during the flight (lead-in HITL off, sim 1.4 s and 5.7 s HITL on): `12 task watchdog` FAIL appears in every BIT list of the flight (all the lists I looked at), `13 nvs` in the early ones (lead-in, sim 1.36, 5.74 s) and not later; the sim-time ones add `14 hitl inactive (bench)`, `25 high-g consistency`, `34 baro rate`, `32 baro plausible` as expected.

## BIT FAIL lists (verbatim, for checks 12 and 13)

**A. Commanded BIT `0x5442` after erase at the end (boot after the tool's post-flight hub cycle, last reset WDT, uptime about 50 s); the same list in all three runs at 50.2, 53.2 and 55.2 s:**

```
W (50215) bit: BIT verdict NO-GO [bench condition] (blocking groups 0x0100, degraded 0x08D7, first 83).
W (50217) bit: BIT: 01 reset reason: FAIL class 1 value 6.
W (50218) bit: BIT: 12 task watchdog: FAIL class 1 value 4.
W (50219) bit: BIT: 13 nvs: FAIL class 1 value 2.
W (50220) bit: BIT: 28 body orientation: FAIL class 1 value 2009.
W (50221) bit: BIT: 46 calibration: FAIL class 0 value 1.
W (50222) bit: BIT: 47 pad heading set: FAIL class 1 value 1.
W (50223) bit: BIT: 66 prelaunch ring: FAIL class 1 value 51.
W (50224) bit: BIT: 74 gps fix: FAIL class 1 value 0.
W (50224) bit: BIT: 75 utc valid: FAIL class 0 value 1.
W (50225) bit: BIT: 83 radio power: FAIL class 2 value 65519 (bench).
W (50227) bit: BIT: B1 battery voltage: FAIL class 1 value 4784.
W (50227) bit: BIT: C4 time of day: FAIL class 0 value 1.
```

The second and third runs (degraded 0x0897) are the same without `66 prelaunch ring`; `B1` values 4796 and 4780.

**B. Startup BIT of one boot (flight boot at the end of the 0x5746 round trip, last reset SW, "reset class 3; 0 fault resets in the last 7"). Three BIT runs: at 1.14 s, 1.65 s, 2.65 s uptime (NMEA bytes interleaved in the raw log; lines cleaned):**

```
W (1140) bit: BIT verdict NO-GO [bench condition] (blocking groups 0x0100, degraded 0x0000, first 83).
W (1144) bit: BIT: 83 radio power: FAIL class 2 value 65519 (bench).

W (1646) bit: BIT verdict NO-GO [bench condition] (blocking groups 0x0100, degraded 0x0812, first 83).
W (1648) bit: BIT: 12 task watchdog: FAIL class 1 value 4.
W (1649) bit: BIT: 13 nvs: FAIL class 1 value 2.
W (1650) bit: BIT: 46 calibration: FAIL class 0 value 1.
W (1651) bit: BIT: 47 pad heading set: FAIL class 1 value 1.
W (1651) bit: BIT: 83 radio power: FAIL class 2 value 65519 (bench).
W (1652) bit: BIT: B1 battery voltage: FAIL class 1 value 5300.

W (2655) bit: BIT verdict NO-GO [bench condition] (blocking groups 0x0100, degraded 0x0816, first 83).
W (2657) bit: BIT: 12 task watchdog: FAIL class 1 value 4.
W (2658) bit: BIT: 13 nvs: FAIL class 1 value 2.
W (2658) bit: BIT: 28 body orientation: FAIL class 1 value 2008.
W (2659) bit: BIT: 46 calibration: FAIL class 0 value 1.
W (2660) bit: BIT: 47 pad heading set: FAIL class 1 value 1.
W (2662) bit: BIT: 83 radio power: FAIL class 2 value 65519 (bench).
W (2663) bit: BIT: B1 battery voltage: FAIL class 1 value 5292.
```

The p1 boot (POWERON, 2.64 s) shows the same last list: 12 (value 4), 13 (value 2), 28 (2008), 46, 47, 83, B1 (5292). Every later BIT run of every recording (p1, p2 at 46 s and 81 s, the flight lists, the end lists) carries `12 task watchdog FAIL class 1 value 4`; `13 nvs FAIL class 1 value 2` is in the boot lists and in the end lists but not in the later flight lists (from sim 20.8 s on).

Observation: `01 reset reason: FAIL class 1 value 6` appears only when the last reset reason is WDT (value 6, which is `ESP_RST_TASK_WDT` in the IDF enum, my reading, not checked in the source), as in 19c. It was not present in the SW or POWERON boots. I have no panic or watchdog text in the console of that boot (the boot lines were not captured).

## End state

Witness in Pad Idle (state packets show status words `0x6600`/`0x6700`), HITL off, flight image `3a66505` (ELF `3fa3a7f5f...`, compile 23:54:15), last reset POWERON (p3 boot), store erased once more after the p3 boot (new session 379 = 0x17B, no slots-full bit). Receiver at 115,201 baud, 25 Hz, `recoveries 0`, no fix (nothing transmitting). Ports 7100, 7101, 7102 free (my recorders closed; no connection on those ports on the desktop or the Pi). No `hackrf_transfer` running (desktop and Pi checked; every stream ended by closing stdin, all exited with code 0). Radio power untouched at -17 dBm. **Pi hub cycles this session: 4** (mine 00:01:40, 00:06:15 and 00:23:52; the tool's after the flight at about 00:19:05; gaps 4 min 35 s, about 13 min, 4 min 47 s; the previous cycle, 23:40:36, was 21 min before the first); with 19e's total of 16 that is 20 since the start of 19c. No over-current or under-voltage in the Pi dmesg (count 0). Flashed once through 7000 (five writes, all verified, no retry needed); Aperture and Iris not flashed. Nothing in source, config or firmware was changed by me, nothing committed; written: `Software/HITL/runs/20261120-f-verify/`, the flight run folder `20261120-000822` and this report.

## Not done / limits

* The identification line of the post-flight power-ups was not captured (twice: the tool's cycle and p3), and in p2 neither; the baud is read from the steady `GPS RX: baud=115201` lines. The USB port re-enumeration at power-up swallows the first seconds, as in 19d.
* The boot after the post-flight hub cycle had last reset WDT; no console text from that boot was captured, so the cause of that WDT is not known. A second observation would need more hub cycles.
* The cause of the Landed +3.27 s and the 7.9 s GPS outage is not determined (see flight notes).
* The `report.pdf` was not written by the tool (matplotlib is not installed for the desktop's Python); the tables above come from the run folder's data.
