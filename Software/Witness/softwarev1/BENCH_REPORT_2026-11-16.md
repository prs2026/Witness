# Bench report 2026-11-16 (run 2026-11-17 00:23-01:04 PST): working-tree fixes on b2f6610

**Result: no hardware test could be run.** The bench Pi (`prs2026hitl`) was offline for
the whole session. Tailscale reported it "offline, last seen 12h ago" at 00:24 and "last seen 13h ago" at 01:04.
TCP connects to 22, 7000 and 7100-7103 timed out, ping got no reply, and a 40-try poll (00:24-01:04, every 45 s) never
reached it. The `.local` names do not resolve and there is no neighbour on the desktop's Ethernet subnets. Nothing was
flashed, no board was contacted, and no uhubctl cycle was used. Only the host-side suites (test K) ran.

## Summary

| Test | Result | Evidence |
|---|---|---|
| A. Smoke (boot, Pad Idle, stack/heap, reset reason) | NOT RUN | bench Pi offline |
| B. HITL nominal flight | NOT RUN | bench Pi offline |
| C. One-way states, 0x7300, second flight | NOT RUN | bench Pi offline |
| D. Launch-detection robustness | NOT RUN | bench Pi offline (scenarios prepared, see below) |
| E. Reset in flight (warm/cold) | NOT RUN | bench Pi offline; warm case has no firmware hook anyway (see E) |
| F. Logging robustness (never-erase, 0x686A USB/radio) | NOT RUN | bench Pi offline |
| G. Command gating | NOT RUN | bench Pi offline |
| H. Watchdog and timing | NOT RUN | bench Pi offline |
| I. Iris | NOT RUN | bench Pi offline |
| J. Aperture | NOT RUN | bench Pi offline |
| K1. Horizon unit tests | PASS | 458 run, OK (64 s), witnessOS venv, offscreen |
| K2. HITL unit tests | PASS (as expected) | `python` (C:\Python314): 95 run, 1 error = `test_gps_rf.Report` (no matplotlib). `py -3.14` (has matplotlib): 95 run, all OK |
| K3. Decode captured HITL byte streams with Horizon's decoder | NOT RUN | no HITL capture exists from this firmware |

## Per-test details

### K. Host side
- Horizon: `QT_QPA_PLATFORM=offscreen ..\witnessOS\.venv\Scripts\python.exe -m unittest discover -s tests` from
  `Software\Horizon`: `Ran 458 tests in 64.020s, OK`. Log: `C:\Users\E\AppData\Local\Temp\wbench\horizon_tests.txt`.
- HITL: `python -B -m unittest discover -s tests` from `Software\HITL`: `Ran 95 tests`, `FAILED (errors=1)`. The error
  is `ModuleNotFoundError: No module named 'matplotlib'` in `test_gps_rf.Report.test_report_from_a_saved_run_layout`,
  as expected. The same suite under `py -3.14` (a different interpreter that has matplotlib) passed with 0 errors.
  Logs: `wbench\hitl_tests_python.txt`, `wbench\hitl_tests.txt`.

### D. Prepared scenarios (dry runs only, no hardware)
These come from the HITL reference state machine (`--dry-run`). They are predictions, not results:

| Scenario | Command | Expected (sim s) |
|---|---|---|
| Nominal 10 g | `--preset "Nominal 10 g"` | Boost 10.30, Coast 12.49, Descent 32.38, Landed 148.19 (ignition at 10.0) |
| Slow motor, ~2 g net | `--preset "Nominal 10 g" --set thrust_accel_g=3 --set burn_time_s=6` | Boost 10.34 (accel rule), Coast 16.48, Descent 28.63, Landed 111.77 |
| Slow motor, ~1 g net | `... --set thrust_accel_g=2 --set burn_time_s=8` | Boost 13.44 (Kalman rule, 3.44 s after ignition, longer than the 2 s pre-launch ring), Coast 18.47, Descent 27.45, Landed 100.57 |
| Slow climb 10 m/s to 150 m | `--csv wbench\slow_climb.csv --set imu_synthesis=false` (smooth 0.2 g ramps; made by `wbench\mk_climb.py`) | no transitions |

A first version of the slow-climb CSV, with velocity steps, produced a false Boost from the step's acceleration
spike (2-21 g). The trajectory has to be smooth for this test.

### E. Warm reset in flight: what the firmware offers
- `flight_state_store.cpp classify()`: Warm = PANIC, INT_WDT, TASK_WDT, WDT, SW, BROWNOUT. Anything else, including
  `ESP_RST_USB` and `ESP_RST_EXT`, is Cold.
- No command causes a software restart, panic or watchdog trip in Boost, Coast or Descent. `0x5746` and `0x6868`
  are allowed only in Pad Idle and Landed, `0x7300` restarts only from Landed, and the false-flight restart callback
  is not wired. **A warm in-flight reset cannot be caused on this firmware.** To test it, the firmware would need a
  USB-only bench command (for example in HITL mode only) that calls `esp_restart()` (SW) or `abort()` (PANIC) in any
  state, or that blocks the sensor task for longer than 8 s (TASK_WDT).
- Planned cold-case method, not run: an RTS chip reset through the flashing port (gives `ESP_RST_USB`, classed Cold).
  This resets only Witness: Iris and Aperture stay up and the USB device stays enumerated. A custom streamer has to
  re-send HITL ENTER after the reboot, because Witness boots in Pad Idle with HITL off. The alternative is
  uhubctl, which resets every board.

## Bugs found
None. No firmware ran.

## Measurements
None from hardware: no stack high-water marks, heap, timings, detection delays or packet counts.

## Not run and why
All hardware tests, A to J, and K3: the bench Pi was unreachable for the whole session (offline on Tailscale for
12-13 h). It needs someone on site to check its power and network. When it is back, run them in this order:
`bench_ports.py`, then flash Aperture, Iris and Witness after `flash_verified.ps1 -CheckOnly`, then A, B, C, E, D.

## State the bench was left in
Unknown and unchanged by this session. Nothing was flashed, sent or power-cycled. uhubctl cycles used: 0. No HITL
session or process started by this session is still running. The boards hold whatever was on them before, probably
`b2f6610`-era images per the last bench report. Their flight state, log slots and stored TX power were not read.
Scratch: `C:\Users\E\AppData\Local\Temp\wbench`.
