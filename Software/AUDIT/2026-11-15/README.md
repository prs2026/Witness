# Audit 2026-11-15: folder index

Audit reports (one per facet, read-only, written 2026-11-15) and the fix reports
(written 2026-11-15 and 2026-11-16) for the audit of commit `b2f6610` plus the
uncommitted Kalman fixes. Start with `../FULL_AUDIT_2026-11-15.md` for the verdict
and the finding IDs (W1-W19, I1-I6, A1-A7, H1-H11).

## Audit reports

| File | Facet |
|---|---|
| `01_boot.md` | Boot, images, bricking, watchdog, init failure paths (Witness) |
| `02_logging.md` | NAND flight logger: slots, resume, erase, export (Witness) |
| `03_radio.md` | LoRa downlink and uplink, hold-off, radio faults (Witness) |
| `04_sensors.md` | Sensor task, IMU, barometer, calibration (Witness) |
| `05_fsm.md` | Flight state machine, persistence, estimators (Witness) |
| `06_gps.md` | GPS receiver, assist upload, NMEA (Witness) |
| `07_commands.md` | Command surface, USB, CAN and Iris forwarding (Witness) |
| `08_system.md` | Concurrency, stacks, resources, watchdogs (Witness) |
| `09_iris.md` | Iris camera and load board firmware |
| `10_aperture.md` | Aperture ground receiver firmware |
| `11_horizon_data.md` | Horizon data path: wire to model |
| `12_horizon_display.md` | Horizon display: model to pixels |

## Fix reports

Each fix report lists what changed, the trickle-down effects, and the doc changes it needs.
The fix reports were applied by separate agents; none is committed or flashed.

| File | Scope |
|---|---|
| `fixes/01_logging_fixes.md` | NAND logger: resume append, error retry, bad blocks, erase timeout, resume marker (source 6) |
| `fixes/02_radio_fixes.md` | Radio: TX power stored in NVS, fault latch and re-init, health check, hold-off cap, held GPS position |
| `fixes/03_boot_system_fixes.md` | Boot without abort, IMU supervisor, calibration sanitising, task watchdog 8 s, PM off, stacks |
| `fixes/04_fsm_filter_fixes.md` | Flight state machine and filter: axial and airborne launch rules, filter without attitude, reset survival, NaN recovery, false-flight hook (not wired) |
| `fixes/05_gps_fixes.md` | GPS receiver states (Streaming, Recovering), supervision, assist whitelist, fixed-baud guarantee |
| `fixes/06_command_fixes.md` | Command policy: load-off refusal in flight, radio power rules, assist gating, erase confirm (0xC0F1), restart re-check |
| `fixes/07_iris_fixes.md` | Iris: load state in RTC and NVS, GPIO9 hold, I2C recovery, TWAI rebuild, stack |
| `fixes/08_aperture_fixes.md` | Aperture: radio init retry and health check, BUSY wait, uplink pacing, USB frame ring, RX race, watchdog |
| `fixes/09_horizon_data_fixes.md` | Horizon data path: auto-reconnect, link health, raw and debug logs, parser rewrite, exception hook |
| `fixes/10_horizon_display_fixes.md` | Horizon display: guarded timers, source following, status strip, landed position, staleness, history |

## Other files in this folder

None. The older files one level up (`../SUMMARY.md`, `../*_findings.md`, `../*_evaluation.md`,
file dates 2026-11-12) are from the earlier audit and are not replaced by this folder.
