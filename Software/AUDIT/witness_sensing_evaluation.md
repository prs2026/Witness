# Witness sensing/estimation — audit evaluation

Evaluated: `Software/AUDIT/witness_sensing_findings.md` against `Software/Witness/softwarev1/main` (working tree, 2026-11-12) and the HITL runs in `Software/HITL/runs/`. Read-only; nothing edited, built or flashed. Analysis scripts are in the session scratchpad (`audit_sensing\`).

Context that matters for severity: this FC has no pyro or deployment output (grep for pyro/deploy/drogue finds nothing in `main`). The flight state drives (a) NAND logging policy (`flash_logger.cpp:600-632, 880-895`: PadIdle logs only to a 2 s ring, Descent keeps 1 of 10 IMU records, Landed logs nothing), (b) attitude phase (Landed freezes the attitude), (c) position-filter constraints (PadIdle pad hold, Landed zero velocity). So a wrong transition costs flight data and estimate quality, not recovery. The eayf trajectory (HITL truth) is a two-stage flight: stage 1 burns 20.0-22.5 s, stage 2 ignites at 32.5 s (45 g, Mach 4.2 at 7 km), apogee 34.8 km. That is above the MS5607's 10 mbar range (above about 31 km), and the vehicle is above Mach 0.8 from about 23 s to about 83 s.

## Summary table

| # | Title | Verdict | Severity |
|---|---|---|---|
| 1 | Landed unreachable (single gyro spike restarts 30 s timer) | PLAUSIBLE (code path confirmed, trigger not seen in data) | medium (auditor: high) |
| 2 | Coast to Descent uses un-gated barometer through transonic flight | PLAUSIBLE | high |
| 3 | Position predict discards IMU gaps over 100 ms | CONFIRMED (code); trigger rare | medium |
| 4 | GPS UART scanned for radio-bridge frames | CONFIRMED (seen in logs), impact smaller than claimed | low |
| 5 | Barometer `valid` set without range/plausibility check | CONFIRMED | medium (auditor's fix needs changing) |
| 6 | Blocking work in 500 Hz loop | PLAUSIBLE, measured margin is large | low |
| 7 | FSM observation repeats stale sample on zero-sample pass | PLAUSIBLE (stale-hold variant only) | low |
| 8 | FSM state overwritten by concurrent command | PLAUSIBLE, race negligible; real issue is NEW-3 | low |
| 9 | NAND IMU timestamp wraps at 71.6 min | CONFIRMED | low |
| 10 | Heading aligner/attitude mismatch, GPS time = pass time | NOT A BUG (mostly) | low |
| 11 | Stored IMU calibration not range-checked | NOT A BUG | low |
| NEW-1 | FSM peak-altitude has no outlier rejection: one bad baro sample causes permanent premature Descent | CONFIRMED (seen in HITL run 20261111-145157) | critical |
| NEW-2 | No in-flight state recovery after an MCU reset (stays PadIdle) | PLAUSIBLE | high |
| NEW-3 | Radio/USB command can set any FSM state in any state, including backwards in flight | CONFIRMED (code) | medium |
| NEW-4 | HITL exit leaves FSM state and tracked maximum in place (explains the 130 s Descent) | CONFIRMED | low (test artefact) |
| NEW-5 | Launch detection is one trigger only (10 g for 300 ms) | PLAUSIBLE | medium |
| NEW-6 | No host unit tests for `flight_state_machine` | CONFIRMED | low |

## The HITL "Descent UNEXPECTED" runs

Two different things are in the data, and neither is finding 2.

1. Runs `20261111-145703`, `-171728`, `-172157`, `-205500` (Descent at sim 130.08 s). Confirmed as the trajectory-end artefact. The `eayf_first70s.csv` truth ends at sim 90 s with the rocket frozen at apogee (alt 34791 m, speed 0, f_x 0.978 g for the last 40 s, truth.csv). The tool sends HITL EXIT at 130.08 (log.txt: "sent HITL EXIT", then "HITL mode exited by USB command; real sensors restored"). The FC logs "descent detected: altitude 208.10 m, maximum 34826.86 m" 0.55 s later (the 500 ms qualification). 208 m is the real bench MS5607; 34827 m is the HITL peak still held in the FSM. The sim barometer is noise-free of transonic effects, and `S*.csv` shows `barometric_altitude` steady at 34791 m, state Coast, until the jump at exit. See NEW-4.
2. Run `20261111-145157` ("K1-short-vib-hot10"). This is NOT the trajectory end. Witness went Coast to Descent at sim 66.78 s while climbing: log.txt line 342 "descent detected: altitude 27890.97 m, maximum 43037.93 m", with the position filter at the same moment reporting velocity +422 m/s up, Mach 1.41. The commanded pressure never exceeds the truth (min sent pressure before 66.8 s is 1640.8 Pa = 27898 m, sent.csv), so 43038 m cannot come from the sim. Log lines 279-292, sim 66.13-66.15 s, show a burst of "0x0C HITL frame rejected: invalid EOF; resynchronizing" plus "unknown command: 2 / X". One garbage frame accepted after resynchronisation (frames carry ID and EOF but no checksum) gave a pressure near 190 Pa, i.e. 43 km. The FSM kept that as its maximum. Cause chain: see NEW-1. The 50 Hz `S*.csv` log did not capture the single sample (its max is 34826 m).

The early-November runs where Descent was 7 s early (`20261108-222347`, `-223257`, `20261109-004511`, `-115242`: expected 139.56, Witness 132.19) are a harness expectation problem. The expected value was recomputed on 11-09 (139.56 to 132.18 with the same Landed time) and every later run matches within 0.01-0.03 s. The firmware did not change behaviour.

## Details per finding

### 1. Landed unreachable while the radio transmits — PLAUSIBLE, medium

- Code (all confirmed): `flight_state_machine.cpp:106-116` resets the 30 s window on any observation with |gyro| of 10 dps or more; the observation gyro is the raw calibrated newest sample (`sensors.cpp:2380-2387`, `working_sample_` is the last sample of the batch via `process_imu_batch`, `sensors.cpp:1858-1881`), with no median. At 500 Hz passes and 960 Hz samples about half the samples are never observed, so each spike has roughly a 50 % chance of being seen.
- Data against the stated trigger: the estimator's "spikes rejected" counter uses the same 10 dps threshold. In every logged real-IMU interval with the radio transmitting (about 4 TX/s: "radio TX" counter rises 40 per 10 s) the counter is flat. Examples: `20261111-171728` 271 at 27, 37 and 57 s while TX 320 to 439; `20261111-205500` 854 at 29, 39 and 49 s; `20261111-200145` 282307 flat from 189 s to 259 s (TX 37518 to 37718); `20261111-000555` 14 flat for 60 s. The counter only jumps in the first 10-20 s after boot (GPS cold start/RF injection start), e.g. 222 to 829 between 7.5 and 17.5 s in `-153238`. So on the current bench configuration the SF7/500 transmissions are not producing 10 dps excursions. The comment in `sensors.h:111-122` (76 dps sd with the radio) refers to the 2026-11-08 setup.
- No real landing is available: every HITL Landed (e.g. 419.75 s) uses synthetic IMU data without spikes. So the failure is unproven both ways. Real descent under canopy and vibration at touchdown are unknown.
- Consequence if it happens: Landed never entered; logging continues in Descent (IMU 96 Hz, binary full rate) until the flash fills; the attitude is not held; position keeps integrating. No recovery or data-loss risk beyond that.
- Fix: run the landing test on a short median/min of |gyro| (for example the estimator's median-filtered rate, or the minimum |gyro| of the last 5 samples), and replace the "reset on one bad sample" with a debounce (reset only if the violation persists 200 ms or occurs in more than 5 % of samples in 1 s).

### 2. Coast to Descent uses the un-gated barometer — PLAUSIBLE, high

- Code (confirmed): `sensors.cpp:2388-2390` (`barometric_altitude_valid = barometer_healthy`, no Mach test); `flight_state_machine.cpp:245-270` (Descent when altitude is below max minus 5 % of |max| for 500 ms); `position_filter.cpp:~256-266` gates the same reading above Mach 0.8.
- Exposure on this vehicle: Coast covers Mach 1.3 down to 0.8 (23 to 31 s), the stage-2 burn to Mach 4.2 (32.5 to about 47 s), and deceleration back to Mach 0.8 at about 83 s (computed from truth.csv with ISA speed of sound). The FSM is never in Boost during the second burn, so the whole of it is under this test. HITL cannot reveal the problem: the sim sends ideal ISA pressure (no shock error), and the baro tests in `position_filter_test.cpp` assume 150 m.
- Quantification (host simulation of the Coast logic on the truth trajectory, `audit_sensing\fsm_sim.py`): a step altitude error of -150 m or less during Mach 0.8-1.2 does not trigger; -300 m does (Descent at t=24.2 s, 1.87 km, Mach 1.15). A sustained pressure error of 5 % or more while above Mach 0.8 triggers when it ends or flips sign (Descent at t=30.4 s, Mach 0.78, -5 %). So the trigger needs a transonic reading error of roughly 5 % of the current altitude (about 100 m at 2 km, 200 m at 4 km), or a positive error that then disappears. Plausible for a body-mounted static port at Mach 1.2-4, not demonstrated, and the damage is irreversible (Descent has no exit except Landed).
- Consequence: IMU log drops to 96 Hz for the rest of the flight (including the Mach 4 burn if early); Landed test arms early. Not a recovery event on this FC.
- Fix: do not evaluate the Descent condition, and do not update the max, while the position filter says baro is gated (or Mach 0.8 or more); additionally require the filter vertical velocity to be negative (or the IMU specific force to be near zero with v_up < 0) for the 500 ms. Add a transonic pressure error to the HITL sim so this is exercised.

### 3. Position predict discards gaps over 100 ms — CONFIRMED (code), medium

- `position_filter.cpp:55-57` and `:118-121` return before touching state or P; `sensors.cpp:1566-1610` advances `position_last_predict_us_` first, so the interval is never recovered. The attitude path rejects gaps over 50 ms (`attitude_estimator.h`), so for 50-100 ms the attitude skips integration while position integrates; above 100 ms both skip.
- Data: real FIFO overruns appear in logs, but only at mode transitions (`imu fifo: ... overruns 1 ... max backlog 256 words` at sim 429.70 s in `-010058`, `-132002`, `-164501`, `-200145` which is the HITL-exit instant; `-185511`/`-214231`/`-225134` at HITL start). Nothing mid-flight. A full 256-word backlog is about 89 ms of data. Attitude `gaps` counters are 0-3 per run with max dt 5-10 ms (HITL start). So the trigger exists but is rare and not shown during flight phases.
- Consequence: position error of v dt (about 100 m at 800 m/s for 120 ms) and velocity loss of a dt (about 24 m/s for 20 g); covariance not inflated; baro gated above Mach 0.8 so only GPS corrects. FSM is not affected.
- Fix: on dt over 0.1 s, still apply the predict with the last acceleration for the whole interval (cap at, say, 1 s) and add process noise q*dt to P; or reset v to the GPS value and inflate P.

### 4. GPS UART scanned for radio-bridge frames — CONFIRMED, low

- `uart_driver.cpp:24-44, 441-490`: every UART byte that equals a canonical packet ID starts a frame of that length. Observed: log.txt in every run shows "uart: discarded UART packet 0x02/0x03 with invalid EOF" during GPS configuration (for example 14 warnings in 0.6 s at 12.2-12.9 s in `-145157`).
- Correction to the claim: GPS decoding is not harmed. `gps_.consume`, `nmea.consume` and the flash log (`uart_driver.cpp:~403-426`) receive the whole buffer before the scanner runs, and the scanner has its own state. NMEA ASCII never contains IDs 0x01-0x05 (only binary receiver replies do), so false frames start only on binary replies during configuration/restart. A false frame is forwarded to the radio only if its last byte is 0x0A, which is possible (NMEA lines end in LF).
- Consequence: occasional garbage LoRa packet and the log noise; no state or data effect. Binary replies occur at boot, cold start and GPS restart.
- Fix: validate before queuing (ID plus length plus a field check, or the bridge disabled for the GPS port); or only run the scanner on a port that carries host packets.

### 5. Barometer `valid` without plausibility check — CONFIRMED, medium

- `ms5607.cpp:~281-284` discards the return value of `pressure_altitude_meters` and always sets `valid = true`; the only input check is `conversion == 0` (`ms5607.cpp:~82`). A MISO line stuck high (0xFFFFFF) is accepted: with typical PROM values this compensates to about 5800 mbar (about -4 km). `sensors.cpp:2354-2356` takes `barometer_healthy` from that; the FSM trusts it.
- Correction: a dead or failing sensor is flagged. SPI or read errors and zero words increment `consecutive_ms5607_errors` and clear `barometer_healthy`. Only non-zero garbage words and non-positive compensated pressure are the hole.
- A single bad sample does not trigger Descent (500 ms continuous is needed), but a bad HIGH altitude does permanent damage (NEW-1), and a stuck-high word does trigger it.
- The auditor's fix must change: "reject pressure outside 1000-120000 Pa" matches the sensor's rated range, but this flight reaches 592 Pa (34.8 km). A hard 1000 Pa bound would drop the barometer for about 80 s around apogee, and the FSM would hold a stale maximum. Use a wide hard bound (for example 10 Pa to 130 kPa), set `valid` from the return of `pressure_altitude_meters`, and add a slew test in the FSM (NEW-1). Note also that above 31 km the MS5607 is out of specification, so altitude there has unknown error.
- Also: at 592 Pa the altitude noise for 1 Pa of pressure noise is about 12 m (S*.csv spread 34791-34815 m), which is above `kLandingAltitudeStabilityMeters` (10 m); this is why the 40 s apogee hold in the first70s runs never produced Landed. It does not matter at ground level (about 0.1 m).

### 6. Blocking work in the 500 Hz loop — PLAUSIBLE, low

- Code is as described (`ms5607.cpp:48-49`, `sensors.cpp:2294-2308`, battery 16 oneshot reads every 100 ms). Measured effect is small: "max pass" is 5-12 ms in all logs, against an 89 ms FIFO margin; FIFO overruns 0 except at the transitions listed under finding 3. The "overruns N" in the attitude line counts late 2 ms passes (about 150/s) and has not produced a data loss. The barometer re-init path runs only when the barometer is absent.
- Fix (optional): spread the battery reads over passes (one conversion per pass), and skip barometer re-init once the state is not PadIdle.

### 7. FSM observation repeats the stale sample — PLAUSIBLE, low

- Code confirmed (`sensors.cpp:2362-2392`: observation is built whenever `lsm_result == ESP_OK`, validity flags come from the sticky `has_*_sample_`). The auditor's concern that timers "accumulate on a value that has not been re-measured" is harmless: the FSM timers are wall-clock durations of a boolean, so holding a 2 ms-old sample for 2 ms changes nothing.
- The real variant: if the FIFO stops delivering (IMU reset or hung but SPI reads OK), the FSM holds the last observation forever (stays in Boost at "10 g", or "still"). No staleness limit exists.
- Fix: carry the sample timestamp into the observation and mark the IMU inputs invalid when the newest sample is older than about 20 ms.

### 8. FSM state overwritten by a concurrent command — PLAUSIBLE, low

- `flight_state_machine.cpp:193, 214, 228, 241, 267` read the state once and write it with an unconditional `exchange`; `set_state` (25-38) and `trigger_launch` (40-46) are written from the command task. The race window is microseconds and only matters for a manual command exactly at a transition. Real hazard is that commands are unconstrained (NEW-3).
- Fix: make automatic transitions `compare_exchange(expected, next)`.

### 9. NAND IMU timestamp wraps at 71.6 minutes — CONFIRMED, low

- `sensors.cpp:2087-2088` truncates to 32 bits; fc_logs `timestamp_us` is e.g. 102950770. A rocket that sits on the pad more than about 70 min after boot has a roughly 14 % chance (10 min flight window out of 71.6 min) of a wrap inside the logged flight. Samples are still in file order, so an unwrap-by-backward-jump in the decoder fixes it.
- Fix: unwrap in the ground tools now; optionally store a wrap counter in a spare bit.

### 10. Heading aligner vs. attitude — NOT A BUG (mostly), low

- After gyro saturation the attitude is `lost_`, `predict_position` calls `predict_without_imu`, and `heading_aligner_.add_inertial` is not called (`sensors.cpp:1566-1608`), so the aligner's inertial velocity stops changing. `add_gps` then returns at `ins_dv < min_dv_mps` (`heading_align.cpp:~117-118`) and issues no correction. The described drift does not occur. The GPS-time point is real but small (parse-time jitter of tens of ms on 1 s windows, i.e. a few percent of speed in the heading comparison) and heading is only a yaw refinement.
- Optional fix: use the NMEA/UART receive timestamp, and pass `attitude_.valid()` to `add_gps`.

### 11. Stored IMU calibration not range-checked — NOT A BUG, low

- NVS entries are CRC-protected, the loader checks blob length and `version`, and the v1 migration starts from `Calibration{}`, whose scales default to 1 (`imu_calibration.h:33-35`). A zero scale cannot come from a valid record. No realistic path. Cheap hardening only: reject non-finite or out-of-range scales at load.

## Missed issues

### NEW-1. Peak-altitude tracking has no outlier rejection — CONFIRMED, critical

- Failure path: `flight_state_machine.cpp:198-209` raises `maximum_altitude_meters_` to any single larger valid baro reading, and nothing lowers it. `flight_state_machine.cpp:245-270` then fires Descent when the reading is more than 5 % of the max below it for 500 ms. One high outlier therefore causes Descent 500 ms later, and since the real altitude cannot reach the false peak, nothing undoes it (Descent only exits to Landed).
- Evidence: HITL run `20261111-145157`, premature Descent at sim 66.78 s at 27.9 km while climbing at 422 m/s (log.txt:342, S*.csv state change at 180.7 s uptime), max 43037.93 m versus a true peak of 34791 m. Trigger: a burst of corrupted `0x0C` frames at sim 66.13 s (log.txt:279-292); `hitl_input.cpp:98-160` decodes a frame on ID and EOF only, and `FrameAssembler::push` (hitl_input.cpp:~180-215) resynchronises on any 0x0C/0x0E byte in the rejected data, so a misaligned frame can pass and carry an arbitrary 24-bit pressure.
- Real hardware has the same weakness: a single bad D1 word (finding 5, no checksum on the ADC word, SPI shared with an RF environment) produces the same outcome.
- Fix: (a) reject baro samples that imply an impossible vertical speed relative to the previous accepted sample (for example more than 2000 m/s, or more than 300 m between 20 ms samples); (b) update the max only from a 3- or 5-sample median; (c) add a checksum or sequence validation to HITL `0x0C`/`0x0E` frames. Also apply the finding 2 fix (Mach gate and v_up < 0).

### NEW-2. No in-flight recovery of the flight state after a reset — PLAUSIBLE, high

- The FSM starts in PadIdle on every boot and nothing restores it (no reset-reason handling outside `cpu_load.cpp` logging). After a brownout or watchdog reset in flight, launch detection (10 g for 300 ms) cannot fire during coast, so the FC stays in PadIdle: binary logging goes only to the 2 s ring (`flash_logger.cpp:880-890`), position filter tries pad-hold constraints whenever specific force is near 1 g, attitude waits for a stationary window.
- Fix: persist the state to RTC memory (survives software/WDT reset) on each transition and restore it when the reset reason is not power-on; or infer flight from baro climb (more than 100 m in 2 s) or GPS speed in PadIdle.

### NEW-3. State can be set from any state to any state by command — CONFIRMED (code), medium

- `usb_serial_echo.cpp:489, 606-611` (radio 0x7300-0x7304), `:669` (text command), `:1013-1026` call `set_state`, which accepts any value 0-4 and applies it unconditionally (`flight_state_machine.cpp:25-38`). A stray or malicious radio command in flight can set PadIdle (stops logging) or Landed (stops logging, freezes attitude). Authentication of radio commands was not traced.
- Fix: refuse commands that move backwards once past PadIdle, and refuse PadIdle/Landed unless the vehicle is stationary and low; keep manual launch only from PadIdle.

### NEW-4. HITL exit leaves the FSM state and maximum in place — CONFIRMED, low (test artefact)

- On HITL exit the attitude is reset ("attitude reset: sensor source is now the real IMU") but the FSM is not. Result: the observed "Descent UNEXPECTED" at 130.08 s in four recent runs. Fix in firmware: reset the FSM to PadIdle on HITL exit/entry; or in the scorer, ignore transitions after the exit command.

### NEW-5. Single launch trigger — PLAUSIBLE, medium

- `flight_state_machine.h:31-32` (10 g for 300 ms on the filtered X axis). A motor with sustained acceleration below 10 g (many L and M class flights) is never detected, so the whole flight is logged to the 2 s ring only. The eayf rocket exceeds it (more than 10 g for 2.4 s) so HITL cannot show this.
- Fix: add a second trigger such as the position-filter vertical velocity above 30 m/s or baro altitude gain above 30 m within 1 s, and lower the accel threshold to about 4 g (with 300 ms) if pad handling allows.

### NEW-6. FSM has no host tests — CONFIRMED, low

- `Software/Witness/softwarev1/tests` has no `flight_state_machine_test`. NEW-1, finding 1 and finding 2 are straightforward to put under test (outlier spike, gyro spike during landing, transonic baro step).

## Significant issues to act on

- NEW-1 (CONFIRMED, critical): one bad baro sample sets a permanent false peak and forces Descent 500 ms later; fix with a slew/median filter on the FSM altitude input and checksums on HITL frames.
- Finding 2 (PLAUSIBLE, high): un-gated barometer in Coast across Mach 0.8-4.2; gate the max/Descent test with the filter's baro-gated flag and require v_up < 0.
- NEW-2 (PLAUSIBLE, high): FSM resets to PadIdle after an in-flight MCU reset; persist state in RTC memory or infer flight from baro/GPS.
