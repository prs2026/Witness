# Pre-flight audit, 2026-11-19

This audit was done before imaging the flight boards. It is based on commit `a06d14d` plus the 10 KB heap-limit edit.

Five reviewers each read one area of the code and wrote a report in this folder:

| Report | Area |
|---|---|
| `01_witness_flight.md` | State machine, launch and landing detection, resume after reset, position filter and GPS trust ramp, attitude, sensor supervision |
| `02_witness_comms_logging_boot.md` | Radio and 2 Hz airtime, commands, NAND logging, two-image boot, Wi-Fi OTA |
| `03_iris_aperture.md` | Iris and Aperture firmware |
| `04_horizon_witnessos.md` | Horizon and witnessOS |
| `05_release_readiness.md` | sdkconfig, bench values left in, flash scripts, imaging checklist, host tests |

The main findings were checked against the code before being listed here. **No Critical findings** in any area. Every host test run passed:

- 29 C++ tests
- 534 Horizon tests

The bench HITL run of the same firmware is in `Software/Witness/softwarev1/BENCH_REPORT_2026-11-19.md`.

## Must do before imaging (process, no code change)

1. **Set the flight radio power.**
   - Witness boots at the bench-safe -17 dBm (`hardware_pins.h:85`), unless NVS holds a value set by command `0x5250`.
   - BIT treats anything below -3 dBm as NO-GO.
   - On each flight board, set `0x5250` to between -3 and +3 dBm (never above +3: front-end limit). Check that the value survives a reboot.
2. **Clear the NAND slots.**
   - If both slots already hold a "flight", nothing is logged. HITL and bench runs that reach Boost count as flights.
   - Erase or clear the slots, then confirm that BIT 6.1 shows the session open.
3. **Rebuild the images from a clean, committed tree.**
   - `images/flight.bin` is older than the heap-limit edit.
   - Every build is stamped `-dirty` while a tracked log file (`Software/Horizon/logs/ui_stalls.log`) is modified.
   - Commit or restore that file, rebuild with `-Clean`, and flash only with `flash_verified.ps1` from `images/`.
4. **Do not use the old flash scripts or the root config.**
   - `flashit.ps1` and `flashit.sh` write the old single-image layout over `otadata` and the `flight_a` slot.
   - The untracked root `sdkconfig` is a stale debug config: a plain `idf.py build` would produce a slow image.
5. **Aperture uplink power.** It returns to -17 dBm after any Aperture reset (RAM only). Re-send the ground power command after every Aperture reset.
6. **Pre-cache the map tiles before flight day.** The one-time tile-cache migration runs on the GUI thread.

## Code findings worth fixing before flight

| # | Severity | Where | Finding |
|---|---|---|---|
| 1 | Medium | Witness `sensors.cpp:2394` | The position filter is only advanced by IMU samples. While the IMU is down it freezes: AGL and vertical speed stop changing, so Descent can come late and Landed may never be declared. Logging continues. Fix: advance it without the IMU after about 100 ms with no IMU sample. |
| 2 | Medium | Witness `wifi_ota.cpp:133` | If the Wi-Fi access point fails to start, the 15-minute auto-off never arms. Witness stays in maintenance (no sensors, no logging) until Wi-Fi off is sent over LoRa or the power is cycled. Fix: return to flight about 60 s after a failed start, plus a hard time limit on maintenance. |
| 3 | Medium | Aperture `ra01.cpp:113` | After any radio re-init, the DIO1 receive interrupt stays off. Aperture then polls every 10 ms. With two packets sent back to back at 2 Hz, the receive buffer can be overwritten mid-read and a corrupted packet passed on silently. Fix: re-arm the interrupt on every init. |
| 4 | Medium | Aperture `main.cpp:295` | A command is dropped silently when its transmission fails, so Wi-Fi ON can vanish with no ack. Fix: retry once or twice and send a BUSY ack to the PC. |
| 5 | High (display) | Horizon `ground_tabs.py:113` | The Witness tab's note says ft and ft/s, but its table shows raw SI values: metres, m/s, and metres MSL. Fix: convert the table values, or change the note to say SI. |
| 6 | Medium | Horizon `monitor_core.py:1467` | Debug-log `PACKET` lines now contain ft and ft/s with no unit. Older logs contain metres in the same format. Raw `.bin`, JSONL and CSV logs are still SI. |
| 7 | Medium (plausible) | Horizon `map_tiles.fetch_now` | A truncated tile response (`IncompleteRead`) kills the tile worker for the rest of the session. Telemetry is not affected. |
| 8 | Medium (plausible) | Witness `flight_state_machine.cpp:327` | Boost ends only when low-g axial acceleration stays under 1 g for 500 ms. If that is missed, the board stays in Boost: no Descent, and the barometer stays gated all flight. Suggested hardening: also move to Coast once vertical speed has been negative for 1-2 s after a climb. |
| 9 | Low | Witness `flash_logger.cpp:557` | After a reset in flight, the protection that stops the packet ring overwriting launch data restarts at zero. About 35 min later the ring could overwrite the burn data. |
| 10 | Low | Witness `position_filter.cpp:619` | Pad fixes return before reaching the GPS trust ramp, so trust starts again at 0.05 at launch and takes about 3 s to reach full trust. This is conservative; the comment says the opposite. |
| 11 | Low | Iris | No retry if CAN init fails. The task watchdog does not panic. Current monitoring is lost all flight if the I/O expander bus is down at boot. Iris is not flight-critical. |
| 12 | Low | Docs | `COMMANDS_AND_PACKETS.md` and `GROUND_STATION_PROTOCOL.md` still say 1 Hz telemetry. |
| 13 | Low | Public repo | The Wi-Fi OTA password and AP address are in `Kconfig.projbuild` and `wifi_ota_box.py`. Use a private password for the flight build. |

**Fixed on 2026-11-19 (built, host-tested, not yet on hardware):**

- Item 1: `service_position()` now predicts without the IMU every 10 ms while the IMU is down.
- Item 2: after a failed access-point start, Witness returns to the flight image after 60 s. Recovery mode, which has no flight image, is excluded.
- Item 3: the DIO1 interrupt is re-armed on every Aperture radio re-init.
- Item 4: Aperture retries a failed command send up to 3 times. It then sends the PC a BUSY ack with detail `0xA901`, or `0xA902` if its queue was full. The monitor shows these acks as `NOT_SENT`.
- Item 5: the Witness tab table converts position, velocity, `kalman_altitude` and `gps_altitude` to ft and ft/s.
- Item 12: `COMMANDS_AND_PACKETS.md` now says 2 Hz.

## Bench HITL run (2026-11-19)

Full report: `Software/Witness/softwarev1/BENCH_REPORT_2026-11-19.md`. The reference run is `Software/HITL/runs/20261119-134233`.

The board ran the commit `0d3a403` firmware (GPS trust ramp, 2 Hz telemetry), checked by image hash. The run used the Nov 18 trajectory with an 8 s GPS outage, and GPS came from the HackRF over coax.

| Goal | Result |
|---|---|
| Flight states | PASS. Boost, Coast, Descent and Landed each came within +0.02 to +0.04 s of truth. |
| Filter against truth | PASS. Max error 279 m, RMS 32 m, worst 2.46 % of height. With the old code: 5795 m, 1179 m, 22 %. After the outage, GPS pulled the error from -280 m to -0.7 m within 3 s. There were 0 GPS rejects (the Nov 18 run had 1006). |
| Telemetry | PASS. State and sensor packets each arrived at 2.000 Hz over the radio in every flight state, and once every 3 s in Pad Idle. About 3 of 1400 packets were lost, all in Descent. |
| BIT, heap, load | PASS with notes. BIT is NO-GO [bench] because of the -17 dBm bench radio power (expected). Minimum heap 15.0 KB, flat. Core 1 load 14-28 %. No resets, panics, watchdog trips or over-current. |
| NAND log | PASS. The state and Kalman log is continuous from the pad to Landed. |

Things the run turned up:

- **GPS was slow to get its first fix on the bench.** It took 242-322 s, against 33 s on Nov 18, and both times it followed a receiver self-reset. Check the coax path.
- **A flight with no fix on the pad stays about 1.2 km off in altitude** until the barometer returns (run `130540`). This is not the normal case, but it supports requiring a pad GPS fix before launch.
- **The HITL tool does not notice a full log store.** One run flew with nothing logged, and the log fetch exported the previous session without saying so.
- **Minor:**
  - BIT 6.1 reads FAIL for about 2 s after an erase.
  - One boot after a log fetch reported `last reset WDT`.
  - The maintenance image repeatedly logs `esp_task_wdt_reset: task not found`.
  - Coast attitude error varied between runs on identical inputs.

## Checked and fine

- **Radio at 2 Hz.**
  - 107 ms of airtime per second (10.7 %).
  - In a scheduler simulation, every packet finished by 692 ms into the second, inside Witness's 700 ms slot, and the longest gap between packets was 670 ms.
  - Pad Idle still sends one packet of each type every 3 s.
  - Aperture's buffers fit the longest packet (101 B in a 128 B buffer).
- **Wi-Fi ON.**
  - It is impossible in Boost, Coast and Descent.
  - Maintenance mode is only entered after a software restart.
  - Any reset in maintenance returns to flight.
- **OTA.**
  - The image is checked before the first flash write.
  - Only the inactive slot is written.
  - A failed self-test rolls back.
- **Flight logic.**
  - One-way state machine, launch triggers, the two descent paths, resume after a warm reset.
  - GPS trust ramp: no NaN or divide-by-zero.
  - Filter divergence recovery.
- **Flight sdkconfig.**
  - Panic reboots; task watchdog 8 s with panic; brownout detector on; rollback on.
  - 32 KB data cache, 240 MHz, -Os.
  - Images use 22 % (flight) and 55 % (maintenance) of their slots.
- **Horizon.**
  - Imperial units are converted exactly once, on the display path only.
  - All link and staleness thresholds still work at 2 Hz.
  - The OTA box never blocks the GUI thread.
  - All buffers are bounded.

## Stale points in the reports

These were already resolved on 2026-11-19, after the reports were written:

- `05_release_readiness.md` finding 8: `stash@{0}` and the `gps-detect-hitl` branch. Every change in the stash was already in main, so it was dropped. The branch was merged. Only `main` remains locally.
- The heap limit is 10 KB, not 8 KB.

## Owner decisions on 2026-11-19

The following are accepted for now:

- the boot-panic re-tests not yet repeated;
- the IMU self-test placeholder;
- the remaining bench checks that have not been run (stack high-water marks, power loss during a NAND write, radio reset, Iris cold reset, Aperture DTR/RTS reset).
