# Fixes report, 2026-11-16

This covers the fixes for the findings in [FULL_AUDIT_2026-11-15.md](FULL_AUDIT_2026-11-15.md). Ten Sonnet agents
implemented them, one area each. Each agent's own report, with file and function for every change, is in
[2026-11-15/fixes/](2026-11-15/fixes/). Finding numbers (W1, I1, A1, H1, ...) are the audit's.

## 1. Status

**Nothing here has run on hardware, and nothing is committed.** All changes are in the working tree on top of
`b2f6610`.

| Check | Result |
|---|---|
| Witness flight image | builds and links (0x72e40 bytes, 70 % of the slot free) |
| Witness maintenance image | builds and links (first attempt hit a compiler crash inside ESP-IDF on the loaded machine; the retry was clean) |
| Iris image | builds and links |
| Aperture image | builds and links |
| Witness host tests | 21 of 21 pass |
| HITL tests | 94 run, 93 pass. The one error is a plotting test that needs matplotlib, which is not installed. One timing test failed once while the firmware builds were running and passed on the re-run. |
| Horizon tests | 431 run. One new reconnect test failed once while the builds were running; it passed three times in a row alone. Before the fixes: 377 run, 1 error. |
| `tx_slot.h` and `comms.h` copies | still byte-identical across the boards |

What I did myself: read the first set of diffs (the Kalman fixes), checked the shared headers, ran every build and
test above, and added one missing call (section 2, GPS assist). I did **not** line-by-line review the ten agents'
diffs; the statements below about how each fix works come from their reports.

## 2. Witness: the nine flight-loss findings

| # | Finding | Fix | Trickle-down effects |
|---|---|---|---|
| W1 | Every boot at -17 dBm | The level set with `0x5250` is saved in NVS (`radio_power_store`) and applied at radio init. The compiled default stays -17 dBm (bench USB limit). Front-end on/off is never saved. | Set the flight level once and it stays, including on the bench. A board left at flight power will transmit at that power on the bench Pi and may trip its USB port. |
| W2 | A reset in flight stops logging | A resumed boot appends to the newest flight slot after the last written page. Flight pages are never erased. Pad boots keep the never-erase rule. Slot choice is pure logic with a host test. | One new record (packet-stream source 6, "resumed"); timestamps and sequence counters restart after it. Horizon and witnessOS log readers must tolerate both. **Not checked.** |
| W3 | A reset in flight leaves the filter dead | The filter now runs without an attitude (barometer and GPS only). Pad references, pad gravity and the last altitude and speed are saved to RTC memory at 2 Hz and restored on a warm reset. A cold reset is covered by W4's barometer rule. | The attitude is not resumed, so after a reset the filter has no IMU for the rest of the flight. A main-parachute descent slower than 20 m/s is not recognised after a cold reset. |
| W4 | One launch-detection path | Boost now fires on any of three rules. (1) Kalman altitude > 30 m and speed >= 30 m/s for 300 ms. (2) Axial acceleration >= 2.5 g for 300 ms. (3) In Pad Idle, a raw barometer rate >= 20 m/s for 1 s (up = Boost, down = Descent). | 2.5 g, not 3 g: a 2 g-net motor reads exactly 3.0 g. A board on a shaker or in an altitude chamber can now reach Boost or Descent. The HITL reference state machine mirrors the rules. The "IMU ignored" Kalman flag now also shows on the pad until the attitude starts. |
| W5 | Init failure boot-loops | Startup checks log and continue. IMU init is retried from the sensor task every 0.5 s. The radio always starts. A battery-ADC calibration failure falls back to an approximate reading. | The board can now come up with a sensor missing; the status bits are the only sign. Battery voltage may be approximate on a chip without eFuse calibration. |
| W6 | First NAND error ends logging | The logger task is never deleted. Erase and program failures are retried, then the block is skipped. After 8 failed blocks, writes pause and retry every 3 s. A torn header page is decided from the slot's other page. Bad blocks are skipped; the layout is unchanged. Ready timeout 20 ms to 100 ms. | The "log failed" bit now clears when writes recover, so it can flicker. The NAND read/write paths have no host test (they need the chip). Mount reads about 8.7k more pages per slot. |
| W7 | No watchdog on the main tasks | Task watchdog: 8 s, panic and reboot. The sensor and logger tasks are subscribed and feed inside their long loops. A watchdog reset is a warm reset, so the state resumes. | A new reboot cause. Any future code that blocks the sensor task for 8 s reboots the board. Idle-task checks are off. |
| W8 | Radio fault stalls the sensor loop | After boot the driver waits at most 2 ms on BUSY. A BUSY timeout, 3 SPI errors or 3 failed transmits latch a fault, and every call then returns at once. | Telemetry stops while faulted, until the re-init (W9) succeeds. |
| W9 | Radio never re-initialised | Boot failures retry every 5 s; an in-flight fault retries after 2 s, one SPI command per sensor pass. A 2 s health check detects a radio that reset alone and re-applies its settings. | About 50 ms of re-init spread over passes. A stuck-low interrupt pin still means no command is received (not fixed). |

### Other Witness fixes

| # | Finding | Fix | Trickle-down effects |
|---|---|---|---|
| W10 | Hold-off silences telemetry | Armed only for a frame accepted as a command, never in Boost, capped at 3 s, then one telemetry cycle is forced. | 3-byte frames are still accepted as commands by the parser, so they still arm it. |
| W11 | Maintenance switch can fire after launch | The restart timer only raises a flag; the state is re-checked when it fires. If it is no longer Pad Idle or Landed, the restart is dropped and the boot slot reverted (`boot_manager::revert_switch`). | `0x7300` in Landed still restarts. |
| W13 | Position 0,0,0 when the fix is stale | The last good position is sent. The ground tells by the GPS-locked bit being clear with a non-zero position. | The age of the held fix is not sent (no spare field). Horizon shows it as a "last known" marker. |
| W14 | GPS retry disturbs a working receiver | New status "streaming": a receiver that delivers fixes but is not configured is used as is; only its setup is retried (5 s to 60 s backoff). The full probe runs only after 5 s without a fix. | Diagnostic strings changed. "Ready" now means configured. |
| W15 | Nothing watches the GPS | 3 s without a fix starts a recovery (115200, then the fallback rate, on the known pins). A drop to about 1 Hz re-sends the setup. | Worst UART-task block is the 0.8 s pin probe, only after 5 s of silence. |
| W16 | No IMU liveness check | `imu_supervisor`: 50 ms without samples, 100 all-zero or all-ones samples, 25 SPI errors, or two failed ID checks reset and re-initialise the IMU; stale data stops being marked valid. | Thresholds are untested against real IMU behaviour. |
| W18 | No NaN recovery | Non-finite inputs are rejected; a diverged filter resets and keeps the pad references. | |
| W19 | Calibration not validated | Rejected on load if non-finite or out of range (scale 0.8 to 1.25); falls back to no correction for that section. | A slightly odd but valid calibration could be rejected. |
| Assist | GPS assist content unchecked | Only the one message type Horizon sends is accepted. A host test asserts no baud change, factory reset or restart reaches the receiver. I added the call that reopens a transfer when the hand-over fails. | An assist "restart" record is now refused. |
| Config | Power management on in flight | `PM_ENABLE` and tickless idle are off in the flight build. | The Landed low-power mode needs both back on if re-enabled. |

### Command rule changes (decisions I made in the briefs)

| Command | New rule | Operator effect |
|---|---|---|
| Camera and 5 V **off** | Refused in Boost, Coast and Descent, USB and radio. On is always allowed. | A camera cannot be switched off in flight. |
| Camera commands over radio | Acked OK only if the CAN queue accepted it, else BUSY. | BUSY means retry. Horizon prints "RETRY". |
| `0x5250` radio power | Still refused in Boost. Front-end off and reductions are now also refused in Coast and Descent. | You asked for Boost only; say if you want this reverted. |
| GPS assist upload | Pad Idle and Landed only. | |
| Erase / clear over **radio** | The same command twice within 10 s. The first acks BUSY with detail `0xC0F1`. USB is single-shot. | Horizon re-sends once automatically after 1 s. |
| USB task stack | 6144 to 7168 bytes. | 1 KB less heap. |

## 3. Iris

| # | Fix | Trickle-down effects |
|---|---|---|
| I1 | Commanded load states are stored in RTC memory and NVS and restored at boot on every reset type. "Off" clears them. | **Cameras come back on after a power cycle if last commanded on.** Send off before pack-down. Confirm on the schematic that none of OUT1-3 or the 5 V enable feeds anything that must not energise at power-up. |
| I2 | The expander reset pin is held through CPU resets, so a warm reset should not power-cycle the cameras. | From the IDF documentation; **needs a bench test.** A real power loss or brownout still drops the loads (board pull resistors). |
| I3 | Main stack 3584 to 6144 bytes; the large object is static. | |
| I5 | (Witness side) see "camera commands over radio" above. | |
| I6 | The CAN node is rebuilt if all transmit slots stay busy for 1 s. | |
| I2C | A 100 ms readback re-applies the load states; a stuck bus is reset; recovery never writes "all off". | |
| Tracker | Witness's re-send backs off 2 s to 30 s and never gives up. | |

## 4. Aperture

| # | Fix | Trickle-down effects |
|---|---|---|
| A1 | Radio init retries forever and restarts the board after 60 failures. | RSSI -128 in the status frame means "receiver radio down". Horizon shows "RECEIVER RADIO FAILED". |
| A2 | 1 s health check; a transmit always ends in receive; re-init through the reset pin. | |
| A3 | A 300 us spin replaces the 10 ms sleep. | Estimated, not measured: receive-to-USB 50-90 ms to 1-2 ms; deaf time per uplink 140 ms to 15 ms. |
| A4 | At most 3 commands per burst, then 1.5 s quiet. Duplicates in the queue are dropped. | **A 32-frame GPS assist upload takes 20-28 s instead of about 5 s.** Horizon's upload wait is now 30 s. |
| A5 | USB output goes through a drop-oldest ring drained by its own task. Log level WARN. | A slow PC loses the oldest frames, counted in a log line. |
| Other | 5 s watchdog; main stack 6144 bytes; parser resyncs after a bad frame. | |
| Not fixable | The reset on port open is in the USB hardware. | Horizon now opens ports with DTR/RTS low. Needs a bench test. |

## 5. Horizon

| # | Fix | Trickle-down effects |
|---|---|---|
| H1 | Every input reconnects forever (0.5 s to 5 s backoff), keeps its source name and appends to the same raw log. | Only Disconnect stops it. |
| H2 | Global exception hook; guarded timers; each widget and trace guarded separately. | Tracebacks go to `logs/horizon_exceptions.log`. witnessOS does not install the hook yet. |
| H3 | Debug log rotates at 50 MB x 3; decoder noise is one summary line per second. | |
| H4 | The raw log never raises; it reopens every 2 s; fsync every 5 s on its own thread. | |
| H5 | TCP keepalive and a 5 s no-data watchdog. | Serial ports are not cycled on silence. |
| H6 | Parser rewritten to linear time. Real capture: 4.56 s to 0.17 s; 16 KB of garbage: 8.4 s to under 0.1 s. Same 21,024 packets decoded. | |
| H7 | Flight View follows the freshest Witness source after 3 s. | |
| H8 | Status strip built into Flight View: red for not logging, log or flash failed, maintenance; amber for resumed, HITL, no fix, sensor not ready. Plus "LINK LOST" and "LOG SLOTS FULL" banners. | |
| H9 | Landed packet in the feed table, Flight View and the map. | Fixes the test that was failing. |
| H10 | Green "LIVE" needs Witness state or position within 3 s; stale readouts go grey. | |
| Other | The pad stays frozen after a flight, including across the `0x7300` reboot. A bad layout falls back to the default. | |
| Partly | Plot performance: one inner step 3x faster; overall redraw gain is modest. | |

## 6. Open items

**Decisions for you**
1. **False flight on the pad.** The mitigation is built but not connected. If connected, reaching Landed with less
   than 50 m and less than 3 g soft-resets to Pad Idle. A real hop under 50 m and 3 g would also reset.
2. **Radio power rule in Coast and Descent** (section 2, command rules): stricter than you asked.
3. **Pre-launch ring (2 s).** Enough for thrust-to-weight of 2.5 or more; weaker motors need about 4 s.
4. **Radio command authentication.** Not built. A proposal is in `fixes/06_command_fixes.md`.
5. **Iris outputs.** Confirm against the schematic that restoring them at power-up is safe.

**Not done**
- No on-board warning (buzzer or arming check) when the vehicle is not logging; only the ground banners.
- A stuck-low radio interrupt pin on Witness; 3-byte command frames arming the hold-off.
- The GPS cold-start re-check at execution (the GPS task has no flight state).
- The compiled GPS pin-swap default is wrong for this board (the probe covers it).
- witnessOS: install the exception hook, show the link banner, open ports with DTR/RTS low.
- Log readers: tolerate the new "resumed" record and the counter restart.
- Horizon: remote-feed ages still use the sender's clock.

**Bench tests needed before flight**
- A reset in flight (warm and cold) in HITL: logging continues, states advance, telemetry power holds.
- Launch detection by each of the three rules, and with the board never still on the pad.
- Power loss during a session-header write; a forced NAND error.
- A radio reset without an MCU reset; a held BUSY pin.
- The watchdog: no trip during a full erase, a maintenance switch or a calibration capture.
- Stack high-water marks on all three boards (the logs are in place).
- Iris: the reset-pin hold across a warm reset; restore after a power cycle.
- Aperture: latency, pacing, and the DTR/RTS reset on reconnect.
- GPS: recovery after a receiver power-cycle.

## 7. Process notes

- The agents shared one scratch folder at first and two reported another agent's helper script overwriting theirs.
  Both say no wrong edit was written. The clean builds and passing tests support that, but it is a reason for the
  bench tests above.
- One agent edited a file outside its list (`qt_widgets.py`, a log-console throttle) and one a HITL test helper
  (`fake_witness.py`). Both are small and the suites pass.
