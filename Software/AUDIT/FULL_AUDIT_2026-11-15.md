# Full code audit, 2026-11-15

Twelve Sonnet agents audited one facet each, read-only: eight on Witness, one each on Iris and Aperture, two on
Horizon. The working tree is `b2f6610` plus the uncommitted Kalman fixes (section 7). The per-facet reports, with
file:line for every finding, are in [2026-11-15/](2026-11-15/).

**How to read the "Checked" column**

| Mark | Meaning |
|---|---|
| **V** | I read the code path myself and the claim holds |
| **R** | The agent reproduced it with a snippet or test |
| **A** | Agent-reported from reading; not re-checked by me |
| **T** | Plausible, but needs a bench test to confirm |

Nothing in this audit was run on hardware.

## 1. Verdict

No path was found that bricks the flash, and noise cannot trigger a command (about 2^-45 per frame). But there are
**nine ways to lose a flight's data or telemetry** that hold in the code today. Most share one of three root causes:

1. **A reset in flight is not survivable.** The state is resumed, but logging, the filter, the radio power and the
   cameras are not.
2. **Single failures are permanent.** One NAND error, one failed radio init or one failed IMU init ends that function
   until a power cycle, and no watchdog covers the main tasks.
3. **Launch detection has one path.** It depends on the filter, which depends on an attitude that can only start
   while still on the pad.

## 2. Flight-loss findings (Witness)

Ordered by how directly they lose data or telemetry.

| # | Finding | Checked | Detail |
|---|---|---|---|
| W1 | **Every boot starts at -17 dBm.** TX power is RAM-only. A flight needs a manual `0x5250` first, and any reset in flight (which the firmware is built to resume from) drops the downlink by about 20 dB. The new `0x7300` reboot does the same. | V | 03 F6, 07 #1 |
| W2 | **A reset in flight stops logging for the rest of the flight.** After Boost the slot in use is marked as a flight. On the next boot, if the other slot also holds a flight, both count as full and nothing is written. HITL and bench Boosts mark slots too, so an older flight on the chip is the normal pad state. This comes from the never-erase change. | V | 02 C1 |
| W3 | **A reset in flight leaves the filter dead.** The attitude can only initialise from a 1 s still window in Pad Idle. After a warm reset the state is resumed, but altitude and vertical speed are never valid: Coast and Descent never end and Landed is never reached. After a cold reset (power glitch) the board boots in Pad Idle and never detects the flight. | V | 05 #2, #3 |
| W4 | **Launch detection has no fallback.** Boost needs the filter's altitude and speed. If the attitude never becomes valid on the pad (never still for 1 s, accelerometer more than 5 % off, a reboot just before launch), Boost is never declared. Pad Idle keeps only a 2 s RAM ring, and telemetry stays at one packet per 3 s. | V | 05 #1, 04 #5 |
| W5 | **An IMU or battery-ADC init failure boot-loops the board.** `ESP_ERROR_CHECK(sensors.start())` aborts. The radio is started inside the same call, so there is no telemetry either. A chip without ADC eFuse calibration would loop permanently. | V | 01 B1, B6 |
| W6 | **The first NAND error deletes the logger task for good.** No retry, no bad-block skip in the log region. A torn header page (power loss while writing a session header or the Boost marker) makes logger start-up fail on every later boot, and erase is then refused. | V (task delete, header error path); T (torn page) | 02 C2, H1, H2 |
| W7 | **No watchdog covers the sensor, logger or radio tasks.** The task watchdog watches only the idle tasks and only prints. The heartbeat keeps running, so a dead sensor task looks alive. Panics do reboot. | V | 08 F1, 01 B3 |
| W8 | **A radio fault stalls the 500 Hz sensor loop.** Each radio command waits up to 500 ms on the BUSY pin, in the sensor task, against about 76 ms of IMU FIFO. Nothing marks the radio as failed. Needs a radio fault first. | V (code); T (trigger) | 03 F1 |
| W9 | **The radio is never re-initialised.** A failed init at boot disables it until reboot (the barometer does retry). If the radio alone resets in flight (3V3 dip), its settings are lost while TX keeps reporting success. | V (no retry); T (in-flight reset) | 03 F2, F3 |

### High, not flight-ending on their own

| # | Finding | Checked | Detail |
|---|---|---|---|
| W10 | **The 1 s uplink hold-off can silence telemetry.** Any received frame whose first byte is 0x05 arms it, before validation, in every state, with no cap. Aperture sends a queued burst back-to-back by design, so a ground station that keeps sending suppresses the downlink. A GPS-assist upload does this for its whole length. | V | 03 F4, 10 #4 |
| W11 | **The maintenance switch can fire after launch.** The state is checked when the command arrives; the restart fires 0.4 s (USB) or 1.1 s (radio) later with no re-check. | V | 01 B2 |
| W12 | **A false Boost on the pad is now permanent** (one-way states). It runs to Landed in about 31 s; NAND logging is off in Landed, so a real launch after that is not recorded, and a log slot is used. Unlikely with the new speed rule, total in effect. | V | 05 #4 |
| W13 | **Telemetry sends 0,0,0 three seconds after the last GPS fix.** The last good position is only kept by the Landed low-power mode, which is off. A GPS stall after landing loses the recovery position. | V | 06 G3 |
| W14 | **The 10 s GPS retry can disturb a working receiver.** It runs on any Failed status in any flight state, forces 115200 and re-probes the pins. One refused setup command can start the loop. (From last session's change.) | V (runs in flight); A (loop trigger) | 06 G2 |
| W15 | **Nothing watches the GPS once ready.** A receiver reset in flight is not detected; at the 88.4 kbaud fallback the UART is then deaf. | A | 06 G1 |
| W16 | **No IMU liveness check.** A dead IMU reads as success with zeros and the state machine is fed stale data marked valid. Earlier finding M8, still present. | A | 04 #2 |
| W17 | **The only "not logging" warning is one status bit.** Two HITL runs fill both slots. No buzzer, self-test or arming check. | A | 02 H4 |
| W18 | **The filter has no NaN recovery.** One NaN sample makes the altitude invalid for good. | T | 05 #5 |
| W19 | **Stored IMU calibration is not validated.** A scale of 0 or NaN from NVS would be applied in flight. | A | 04 |

### Commands (design decisions more than bugs)

- **`0x5250` radio power** is refused only in Boost, as you asked. In Pad Idle, Coast and Descent one frame can
  switch the front end off, and only a reset recovers it (03 F5). **V**
- **Camera and 5 V commands** go to Iris in every flight state (07 #2). **A**
- **Radio commands have no authentication or sequence number.** Replay or a deliberate transmitter can erase logs in
  Pad Idle or Landed, or start the maintenance image on the pad (03 F7, 07 #4). Earlier findings H2/M3. **A**
- **GPS assist** is refused only in Boost; record bodies are passed to the receiver unchecked, so a baud change or
  factory reset could pass (06 G4). **A**

### Medium (see the detail files)

- A full hex dump of every received radio packet is logged from the sensor task.
- `PM_ENABLE` and tickless idle are still on in the flight build (nothing sleeps today).
- Erase and clear wait on the logger with no timeout; if the logger is dead, the command task hangs.
- The IMU stream holds about 9 min at 960 Hz, then stops with only a status bit.
- The IMU stream has no per-record framing or CRC.
- One unreadable page fails the whole maintenance export.
- Stacks have not been measured since the Kalman filter went into the sensor task; `flash_capture` (3072 B) and the
  self-test task (3072 B) look tight.
- Boost is declared about 2 s after liftoff at 2 g net, against a 2 s pre-launch ring.
- The flight image erases all of NVS on `NO_FREE_PAGES`, losing the IMU calibration.

## 3. Iris (cameras)

Iris has **no failsafe that turns loads off**: no timeout, over-current, under-voltage or bus-off action. Loads
change only on a command or a reset. (All **A**; the file path the agent cited for the reset code did not resolve
when I checked, so finding I1 in particular should be confirmed on the bench.)

| # | Finding | Detail |
|---|---|---|
| I1 | **A cold reset of Iris leaves the cameras off.** Loads boot off. Only Witness re-sends the commanded states, from RAM it loses on its own reset, and it sends nothing at its own boot. A battery sag that resets both boards leaves the cameras off for the flight. | 09 #1 |
| I2 | **Any warm reset power-cycles the cameras** for hundreds of ms. Whether recording resumes depends on the camera. | 09 #2 |
| I3 | **The main stack may overflow** (3584 B against an estimated 3.3-3.9 KB). An overflow is a panic, so I2, possibly in a loop. Check the high-water mark. | 09 #3 (T) |
| I4 | Camera-off is never refused in flight, on Iris or Witness. | 09 |
| I5 | A camera-on command can be acked OK over radio although the CAN forward failed, and is then never re-sent. | 09 |
| I6 | Iris's CAN driver may leak transmit slots after bus-off, which would stop Iris telemetry for good. | 09 (T) |

## 4. Aperture (ground receiver)

Radio settings, `tx_slot.h` and `comms.h` match Witness exactly (byte-identical headers). All **A** unless marked.

| # | Finding | Detail |
|---|---|---|
| A1 | **A radio init failure is permanent**: a log loop with no retry or reboot. An absent radio is not detected. | 10 #1 |
| A2 | **No radio health check.** A failed transmit setup returns without going back to receive: deaf until the next successful transmit. | 10 #2 |
| A3 | **The BUSY wait sleeps a whole 10 ms tick** before every command: up to ~60 ms per received packet and ~120 ms deaf per uplink. Witness already fixed this. | 10 #3 (T) |
| A4 | **Queued commands go out back-to-back** (by design, **V**). With W10 this suppresses the downlink under a steady command stream. | 10 #4 |
| A5 | **A slow USB host stalls reception** (100 ms blocking writes in the one receive loop); drops are not counted. | 10 #5 (T) |
| A6 | ESP_LOG text shares the binary USB stream. | 10 |
| A7 | Boot TX power is -17 dBm; the docs say -9 dBm. | 10 |

## 5. Horizon (displays)

| # | Finding | Checked | Detail |
|---|---|---|---|
| H1 | **A dropped link never reconnects.** Any reader exception or TCP close ends that input and closes its raw log. TCP and manually connected serial ports stay down until someone clicks Connect. | V | 11 F1 |
| H2 | **An exception in a display timer aborts the whole app.** No `sys.excepthook` is installed anywhere in Horizon. | R, V (no hook) | 12 #1 |
| H3 | **The debug log is unbounded and unguarded**: 485 MB in 27 min, about 1 GB/hour, 94 % of it per-byte bad-packet lines. A full disk then kills the reader (H4) or the app (H2). | R (size) | 11 F4, 12 #2 |
| H4 | **The raw log write is in the same `try` as decode.** A log error kills the reader and closes the raw log. | V | 11 F3 |
| H5 | **A half-open TCP link shows "Connected" forever**: no keepalive, no no-data timeout. | A | 11 F2 |
| H6 | **Garbage bytes stall the parser**: one error event per byte and a quadratic resync. 16 KB of garbage took 8.4 s. This matches the one real GUI stall in `ui_stalls.log`. | R | 11 F5 |
| H7 | **Flight View binds to the first source name** and ignores the rest. After a port rename or a USB/LoRa/TCP switch, traces freeze until "Clear plots". | A | 12 #3 |
| H8 | **"Not logging", "resumed" and "maintenance" bits are not shown in Horizon proper**, only in the witnessOS monitor tab. | A | 12 #4 |
| H9 | **The landed position packet (0x09) is not in Flight View, the map or the feed.** One Horizon test already fails on this (`KeyError: 'landed_position'`); it failed at `b2f6610` too, so my note last session that the Horizon tests passed was wrong. | V | 12 #5 |
| H10 | The green "LIVE" shows if any one of ~45 traces is fresh; stale values are not greyed. | A | 12 |
| H11 | The local feed thread to the flight and attitude tabs dies on a socket error and is not restarted. | A | 11 F7 |

Not covered: the post-flight viewer files (`viewer.py`, `viewer_data.py`, `converter.py`, `any_csv.py`).

## 6. Cleared

- **Bricking:** every reset out of the maintenance image restores the flight slot first; rollback cannot loop.
- **HITL:** cannot start by accident (needs a full valid frame over USB in Pad Idle; times out after 1 s) and cannot
  be entered over radio.
- **Parsers:** every command and NMEA parser is bounded; the CAN reassembler is robust.
- **Buses:** the NAND is alone on its SPI bus; each SPI host has one user task; no deadlock or torn read found.
- **CAN:** a dead or shorted bus never blocks the sensor or radio path.
- **Heap:** no allocation after start-up. The bench "queue failed: NO_MEM" is a full forward queue, not heap.
- **Radio link:** modulation, frame formats and slot math agree on both ends; frequency, SF and BW cannot be changed
  over the air; GPS time loss falls back to free-running within 3 s.
- **GPS:** no built-in baud change, factory reset or receiver flash write.
- **Logging:** the pre-launch ring order is right; the USB export is read-only.
- **Panic:** panics and the interrupt watchdog reboot (not halt) and resume the flight state.

## 7. The four Kalman fixes (uncommitted)

A Sonnet agent implemented them; I read the diff. All 18 host tests and 91 HITL tests pass and the flight image
builds. Not flashed, not committed, not run on hardware.

| Fix | Result |
|---|---|
| One-way states | `set_state` refuses any lower state. `0x7300` in Landed acks, then restarts (0.4 s USB, 1.0 s radio). The pad hold is ungated: 75 m returns to 0 in 4 s. |
| GPS quality | The filter and heading alignment use only quality 1, 2, 4, 5. |
| Pad gravity | 1.01 g offset, 60 s coast without baro or GPS: 1 m error against 183 m. HITL replay descent error 11.9 % to 2.75 %. |
| Boost rule | Altitude > 30 m and speed >= 30 m/s, both for 300 ms. |

Consequences found by the audit:
- W12 (a false Boost is permanent) and W1 (the reboot returns at -17 dBm).
- After a Landed reboot with both slots full, the second flight is not logged.
- The agent's own choices: the Horizon `0x7300` button now asks for confirmation; pad gravity ignores jolts under
  0.5 s.
- Untested edge: `0x7300` in Landed while a maintenance switch is prepared.

## 8. Suggested order

**Before any flight**
1. W1: make the flight TX power the stored default, or persist it.
2. W2: on a resumed boot, keep appending to the in-flight slot.
3. W5: retry sensor init instead of aborting; start the radio regardless.
4. W4 + W3: a second launch trigger from axial acceleration, and let the filter start without a pad attitude after
   a reset.
5. W7: put the sensor and logger tasks on the task watchdog with panic on.
6. W6: retry NAND errors and skip bad blocks; tolerate a torn header page.
7. H1 + H2: auto-reconnect and a global exception hook in Horizon.

**Next**
8. W8 + W9 + A1 + A2: radio fault handling and re-init on both ends; A3.
9. W10 + A4: validate before arming the hold-off, and cap it.
10. W13: hold and flag the last good position.
11. I1: Witness re-sends load states at boot, or Iris stores them.
12. W11, W14, W17, H3-H9.

**Bench checks the audit asked for**
- Stack high-water marks on Witness, Iris and Aperture.
- Power loss during a session-header write.
- A radio reset without an MCU reset.
- Iris cold reset with cameras on.
- DTR/RTS reset of Aperture when ser2net reconnects.
