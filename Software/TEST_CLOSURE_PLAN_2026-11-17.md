# Test and issue closure plan (2026-11-17)

Closes every open issue and testing gap from the 2026-11-16 fixes report, the 2026-11-17 / 17b / 17c bench reports
and the Kalman audit. Ordered by importance: what could lose the vehicle, hurt someone or lose the data first, then
what weakens confidence, then tooling and comfort.

Legend: **Who**: Bench = agent on the HITL bench, Code = code change plus host tests, You = needs you or hardware
only you have. **Needs** = a dependency that must be in place first.

## Phase 0: decisions (you, before any code that depends on them)

| # | Decision | Why it comes first |
|---|---|---|
| 0.1 | Iris outputs restored at power-up: confirm against the schematic that this is safe | Pyro/load safety; gates Iris tests in phase 2 |
| 0.2 | False-flight-on-pad mitigation: connect it or not (a real hop under 50 m and 3 g would also reset) | Changes the state machine tested in phase 1 |
| 0.3 | Pre-launch ring 2 s, or 4 s for low thrust-to-weight motors | Changes launch detection tested in phase 1 |
| 0.4 | Radio power rule in Coast/Descent (currently stricter than asked) | Small; affects radio tests |
| 0.5 | Radio command authentication: build it (proposal in `AUDIT/2026-11-15/fixes/06_command_fixes.md`) or accept the risk | Affects command tests; can run in parallel |
| 0.6 | Approve two bench-only commands needed below (you asked to check before adding commands): a deliberate hang (to trip the real task watchdog) and an Iris warm reset | Without them two tests cannot run |

## Phase 1: flight-critical, on the current bench (no new hardware)

| # | Item | Who | Method | Pass |
|---|---|---|---|---|
| 1.1 | Warm reset in every flight state | Bench | 0x4252 (both modes) in Pad Idle, Boost, Coast, Descent, Landed via HITL; 3 resets in a row in Boost; a reset during a log-session header write | State resumes each time, logging continues, telemetry back < 1 s, no session corruption |
| 1.2 | Real task-watchdog trip | Bench | Bench-only hang command (needs 0.6) in Coast | WDT reset classified Warm, Coast resumes |
| 1.3 | Fix HITL sim barometer-rate bug and the -418 m post-reset GPS artifact | Code | Compute the baro rate in `_feed()` like the firmware; fix the GPS AGL after reset; test with a 25 m/s low-g climb | Boost by the baro rule scored PASS; no false Descent after reset |
| 1.4 | All HITL presets and Monte Carlo on the final `-Os`/240 MHz build | Bench | Every preset in `hitl_gui.py`, plus the Monte Carlo/replay tools; compare Kalman error against truth with the `-Og` numbers | Transition deltas and Kalman error within the `-Og` figures |
| 1.5 | Kalman sideways drift (7 m Descent, 13 m Landed in HITL vs 2-3 m in CSV runs) | Code | Find why the vertical preset drifts; fix or explain | Back to CSV-run figures or a documented reason |
| 1.6 | Real-sensor launch detection and load under motion | Bench + You | Drop/swing test and a hand shake with real sensors (printer vibration helps); log core-1 load and overruns during it | Each launch rule fires on a real stimulus; no false launch from shaking; core 1 < 60 %, no sustained overruns |
| 1.7 | Long pad-wait soak | Bench | 4 h idle on the final image (printer off), then a HITL flight at the end | Heap flat, no resets/WDT, GPS 25 Hz throughout, log growth as expected |
| 1.8 | Pre-reset log loss (up to 0.5 s) | Code + Bench | Shutdown-handler flush for software restarts; retest 1.1 | Loss < 50 ms on SW restart; panics documented |
| 1.9 | Not-logging warning on the vehicle | Code | Arming check or buzzer pattern when logging is off/failed | Warning fires on a forced log failure |
| 1.10 | Log store full | Bench | Fill to capacity (scripted), then fly HITL | Behaviour matches "never erase": defined stop, no crash, ground warned |
| 1.11 | Forced flash error and power loss during header write | Bench | Injected NAND error; hub power cut during a session-header write (uhubctl, 4 min spacing) | Logger recovers, earlier sessions intact |

## Phase 2: power, Iris and hardware-dependent (needs parts or your hardware)

| # | Item | Who | Needs | Pass |
|---|---|---|---|---|
| 2.1 | Witness 3.3 V regulator swap, then rerun the tests that wait on it | You, then Bench | TPL740F33 (ordered 2026-11-13) | Rail in spec under load |
| 2.2 | Current and temperature at 240 MHz, radio at full power | You + Bench | Battery, a meter, powered USB hub | Within regulator and battery budget with margin |
| 2.3 | Brownout and battery sag sweep | You + Bench | Adjustable supply | Clean brownout reset, Warm/Cold class correct, state resumes |
| 2.4 | Iris outputs with dummy loads (resistors/LEDs, never pyros) | Bench | 0.1 decision | Each output switches on command only; correct state after power cycle |
| 2.5 | Iris output hold across a warm reset | Bench | 0.6 command | Outputs held through the reset |
| 2.6 | Iris and Witness stack high-water marks under load | Bench | 2.4 | Margins recorded, none below 1 KB |

## Phase 3: radio, GPS and Wi-Fi

| # | Item | Who | Method | Pass |
|---|---|---|---|---|
| 3.1 | Fix ack collisions on bursts; Horizon retry on a lost ack | Code + Bench | Serialise acks or tag them; test a 5-frame burst over coax | 0 lost acks, or Horizon retries recover all |
| 3.2 | Fix missing USB ack for 0x5746 "on" | Code + Bench | Flush the ack before the restart | Ack arrives 5/5 |
| 3.3 | Radio edge cases: stuck-low interrupt pin, 3-byte frames arming hold-off, radio reset without MCU reset, BUSY held | Code + Bench | Fault injection on the bench | Radio recovers within the W9 retry times |
| 3.4 | Radio at full power into coax | Bench | Powered hub (2.2), attenuators sized for full power | Link clean, no over-current |
| 3.5 | Link budget and ground range test | You | Real antennas, outdoor, Aperture + Horizon at distance | Margin at max range meets the flight-altitude budget |
| 3.6 | GPS: receiver power-cycle recovery, signal loss and reacquire | Bench | Power-cycle the receiver; attenuate the coax feed | Fix back without a reboot, time recorded |
| 3.7 | GPS flight dynamics | Bench | HackRF GPS simulation (`gps_rf.py`) of a high-g, high-speed trajectory into the coax | Fix held or reacquired as expected through Boost and Coast |
| 3.8 | GPS power-on 22 s to 25 Hz | Code + Bench | Shorter first config retry | < 10 s on 5 power-on boots |
| 3.9 | Wi-Fi firmware upload and log download | You + Bench | A laptop or phone that can join the Witness AP (not the bench desktop) | New `-Os` flight image accepted via OTA, image-info valid, logs download intact |
| 3.10 | Full erase blocks USB commands for 4.3 s | Code | Move erase off the USB task or chunk it | No dropped packet-log records during erase |
| 3.11 | GPS cold-start check at execution time | Code | Give the GPS task the flight state | Refused outside Pad Idle in a host test |

## Phase 4: environment

| # | Item | Who | Pass |
|---|---|---|---|
| 4.1 | Cold soak (freezer) and warm soak (enclosure ~60 C) of the full stack, powered | You + Bench | Boots, runs, logs and links at both ends |
| 4.2 | Barometer in a vacuum chamber or sealed jar with a pump | You | Altitude tracks a reference; no PROM/CRC errors |

## Phase 5: ground station and tooling

| # | Item | Who | Pass |
|---|---|---|---|
| 5.1 | End-to-end Witness to Aperture to Horizon at full flight rate with injected packet loss | Bench | Horizon stays live, staleness correct, no UI stalls |
| 5.2 | Horizon portrait layout on a real 9:16 monitor with live hardware and all feeds | You | Readable, nothing clipped, correct DPI scaling |
| 5.3 | Horizon remote-feed ages use the sender's clock | Code | Ages correct with a skewed clock in a test |
| 5.4 | witnessOS: crash hook, link banner, ports opened with DTR/RTS low | Code | Tests pass; no board reset on connect |
| 5.5 | Log readers accept the "resumed" record and counter restart | Code | Reader tests with a resumed session |
| 5.6 | `viewer.py` destroyed-signal lambda (same crash pattern as Horizon) | Code | Replaced with a callable; tests pass |
| 5.7 | Build `imu_calibration_store_test` (needs FreeRTOS stubs) | Code | 21/21 host suites |
| 5.8 | Document `--no-fetch-logs` (headless HITL power-cycles the hub) | Code | README updated |

## Housekeeping (any time)

- Merge or delete `Software/Horizon/logs/ui_stalls.local.log` against `ui_stalls.log`.
- Commit `VIDEO_S3_MEMORY_AND_MUX_STUDY.md` and this plan.
- Clean build on any machine with an old Witness build folder.
- Video board (S3R8, second computer): measure the TVP5151 re-lock time on the first prototype.

## Suggested order of work

1. Phase 0 decisions (you).
2. In parallel: 1.3 and 1.5 (code), 1.1, 1.4, 1.7 (bench). Then 1.2 once 0.6 is approved; 1.6 with you.
3. 1.8 to 1.11.
4. Phase 3 code fixes (3.1, 3.2, 3.8, 3.10, 3.11) in one firmware batch, then one bench regression.
5. Phase 2 when the regulator, meter and powered hub are in.
6. 3.4 to 3.9, then phase 4, then phase 5.
