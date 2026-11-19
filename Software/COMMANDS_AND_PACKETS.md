# Witness commands and packet types (quick reference)

> **Changed 2026-11-16.** Sections changed: Status bytes (NAND-failed bit cleared on recovery; GPS_LOCKED stale-position convention); Radio scheduling (hold-off rules, Aperture pacing); Commands table (loads, `0x4741`-`0x4745` states and whitelist, `0x5250` rules, `0x6869`/`0x686A` two-step confirm, `0x7300`); Set-flight-state rules (three Boost routes); radio power rules; new sections "Ack details" and "NAND packet-stream records". Sources: `Software/AUDIT/2026-11-15/fixes/01_logging_fixes.md`, `02_radio_fixes.md`, `04_fsm_filter_fixes.md`, `05_gps_fixes.md`, `06_command_fixes.md`, `07_iris_fixes.md`, `08_aperture_fixes.md`. Constants checked in `main/command_policy.h`, `main/comms.h`, `main/usb_serial_echo.cpp`, `main/flight_state_machine.h`.

Generated from the firmware on 2026-11-15 (`Witness/softwarev1/main/comms.h`,
`command_policy.h`, `usb_serial_echo.cpp`, `command_bridge.cpp`, `sensors.cpp`,
Aperture `main.cpp`). The byte-level layouts are in `GROUND_STATION_PROTOCOL.md`;
this file is the overview. All frames are `ID | data | 0x0A`, multi-byte fields
big-endian.

Flight states: **PI** Pad Idle (0), **B** Boost (1), **C** Coast (2), **D**
Descent (3), **L** Landed (4). "any" = every state. The states are **one-way**:
Witness only moves forward (PI, B, C, D, L). The way back to Pad Idle is a restart
(`0x7300` while Landed, below).

## Packet types

| ID | Name | Frame bytes | Direction | Where it goes | Rate |
|---|---|---:|---|---|---|
| `0x01` | Sensors (filtered accel, gyro) | 23 | Witness -> ground | USB 10 Hz; LoRa 1 Hz in flight, every 3 s in Pad Idle; NAND | |
| `0x02` | Estimated state (quaternion, ENU position/velocity, GPS, Kalman altitude, battery, sats) | 65 | Witness -> ground | USB 1 Hz; LoRa 1 Hz in flight, every 3 s in Pad Idle; NAND | |
| `0x03` | Camera / Iris telemetry (relayed from Iris over CAN) | 19 | Iris -> Witness -> ground | USB and LoRa 1 Hz in every state | |
| `0x05` | Command | 10 | ground -> Witness (USB or LoRa via Aperture); also Iris | Witness acts locally and forwards to Iris over CAN | on demand |
| `0x06` | Command acknowledgement (opcode, 6 data bytes, result, detail) | 13 | Witness -> ground (USB or LoRa, same path as the command); Aperture on USB for ground radio power | immediate queue (never held by time slots) | per command |
| `0x07` | GPS assist transfer status (chunk bitmap) | 76 | Witness -> ground | reply to `0x4751` | on demand |
| `0x08` | Maintenance status beacon | 19 | Witness (maintenance image) -> ground | USB and LoRa every 5 s while in the maintenance image | 0.2 Hz |
| `0x09` | Landed position (1e-7 deg, fix age, flags, sats, battery) | 28 | Witness -> ground | only with the Landed low-power mode, which is **off** since 2026-11-15 (`landed::kLowPowerModeEnabled`); then once per minute at UTC :00 and as a ping reply | (off) |
| `0x0C` | HITL sensor input | 47 | host -> Witness, USB only | HITL mode | 100 Hz |
| `0x0E` | HITL 960 Hz IMU batch | 150 | host -> Witness, USB only | HITL mode | ~10 Hz |
| `0xE0` | Witness debug (full sensor and estimator snapshot) | 103 | Witness -> ground | USB 10 Hz after `0x00E0`; never on LoRa | |
| `0xE1` | Iris debug | 14 | Iris -> Witness -> ground | after `0x00E1` | |
| `0xE2` | Kalman filter log | 103 | NAND only | packet log | 50 Hz |
| `0xF0` | Ground-station status (Aperture RSSI, GPS) | 29 | Aperture -> host, USB only | 1 Hz | |
| `0xFF` | Heartbeat | 8 | Witness -> ground, and to Iris over CAN | USB 1 Hz; CAN 1 Hz (paused in Landed) | |

Radio scheduling (`main/tx_slot.h`, since 2026-11-14; hold-off rules changed 2026-11-16):
- With a GPS fix (UTC from GGA), Witness starts a telemetry packet only if it
  ends in the first 700 ms of the UTC second. Aperture, with a fix and a PPS
  edge, starts commands only between 800 and 950 ms.
- Hold-off: after Witness receives a command frame that passes validation, it
  holds its telemetry for 1 s. Frames that fail validation do not arm it. No hold
  is armed in Boost. A continuous hold is capped at 3 s, after which one full
  telemetry cycle is sent before another hold can start.
- Aperture pacing: at most 3 commands in a burst, then at least 1.5 s with nothing
  sent (`main/uplink_pacing.h`). The tx_slot rules still apply (800 ms after a
  command for follow-ups; the GPS window when there is a fix).
- Acks and replies (`0x06`, `0x07`, `0x09` ping replies) are never held by the time
  slots or the hold-off. Since 2026-11-18 they are paced for the half-duplex link
  (`main/radio_edge_logic.h` `AckPacer`): an ack waits until the radio has heard
  nothing for 40 ms (at most 300 ms), so a burst of commands is not cut by the
  ack of the first one, and they go out in order from a 16-deep queue. A frame
  that cannot be queued is logged (`radio reply ... not queued`), never silent.
  Only a complete 10-byte command frame arms the hold-off (a stray 3-byte frame
  does not). With the radio interrupt line (DIO1) stuck low, the radio IRQ status
  is still read every 200 ms, so commands are heard late instead of never.
- USB acks wait up to 50 ms for room in the USB queue (they were dropped at once
  when it was full) and are flushed to the host before a deliberate restart
  (`0x5746` on, `0x7300`, `0x4252`).
- Without GPS time on a side, that side sends whenever it can.

## Ack details

| Ack result | Detail | Meaning |
|---|---|---|
| BUSY (`0x04`) | `0xC0F1` (`IRIS_ACK_DETAIL_CONFIRM_NEEDED`, `comms.h`) | Radio erase/clear received once; send the same command again within 10 s to execute |
| BUSY | 0 | Load/5 V command not forwarded to Iris (CAN queue full); retry |
| BUSY | esp_err | Erase/clear timed out or the logger was busy; the detail is the `esp_err_t` value |
| REFUSED | esp_err | Erase/clear failed (for example `ESP_ERR_INVALID_STATE` when refused in flight) |
| REFUSED | flight state | `0x5250` refused by policy |
| INCOMPLETE | chunks received | Assist COMMIT with chunks missing |

The value `0xC0F1` is named `IRIS_ACK_DETAIL_CONFIRM_NEEDED` in `comms.h` (all four copies); `usb_serial_echo.cpp` uses it.

Stale position: while the GPS has no fix, `0x02` and the NAND state log carry the
last valid lat/lon/alt with GPS_LOCKED (status byte 1, bit 3) clear. A non-zero
position with GPS_LOCKED clear is the last known position, not a fix. Satellites is
the live count. Before the first fix ever, the position is 0,0,0. The age of the
held fix is not in the packet.

Aperture RSSI: a `0xF0` RSSI of **-128** means the receiver radio is down (init
failed, the radio faulted, or the RX front end is off). The normal range is
-127 to 0 dBm. The frame has no spare field.

## NAND packet-stream records

When Witness boots in an in-flight state it appends a resume marker to the same
flight session. It is a packet-stream record with source 6 (`kPacketSourceLogMarker`):

| Field | Size | Value |
|---|---|---|
| time | u32 BE | microseconds |
| source | u8 | 6 |
| length | u8 | 6 |
| kind | u8 | 1 = resumed |
| flight state | u8 | state at resume |
| session generation | u32 BE | |

After a resume marker, in the same session, the microsecond timestamps (IMU, WL,
packets) restart from about 0, and the IMU and WL sequence numbers restart at 0.
A reader must treat the marker as a time discontinuity. Readers that skip unknown
sources by their length byte are unaffected. Horizon and witnessOS parsers should
be checked for an unknown-source branch.

## Status bytes (in `0x01`, `0x02`, `0x09`, `0xFF`)

| Byte | Bit | Meaning |
|---|---|---|
| 0 | 0 | heartbeat toggle |
| 0 | 1 | low-g accel ready (drops while the IMU is down; see the IMU supervisor in FIRMWARE_HANDOFF.md) |
| 0 | 2 | gyro ready |
| 0 | 3 | IMU temperature ready |
| 0 | 4 | NAND init failed (set while the logger retries its init; cleared on success) |
| 0 | 5 | high-g accel ready |
| 0 | 6 | barometer ready |
| 0 | 7 | NAND logging failed (set while writes fail; cleared when writes recover) |
| 1 | 0-2 | flight state (0 PI, 1 B, 2 C, 3 D, 4 L) |
| 1 | 3 | GPS locked. **Clear while the position fields are non-zero = last known position** (no fix; lat/lon/alt are the last valid fix; satellites is the live count) |
| 1 | 4 | HITL active |
| 1 | 5 | MAINTENANCE (never set by the flight image; the maintenance image sends `0x08` instead) |
| 1 | 6 | RESUMED: an in-flight state was resumed after an MCU reset |
| 1 | 7 | **LOG SLOTS FULL** (new): both NAND slots hold a flight, nothing is logged until erase/clear |

## Commands (`0x05`, opcode in bytes 7-8, data d0-d5 in bytes 1-6)

Columns: **USB / LoRa** = accepted from that source; **States** = flight states
in which it is accepted (otherwise acked `REFUSED`); **Iris** = also forwarded
to Iris over CAN. Ack results: 0 OK, 1 REFUSED, 2 BAD_ARGUMENT, 3 UNKNOWN,
4 BUSY, 5 INCOMPLETE, 6 BAD_CRC, 7 BAD_FORMAT. The ack detail word and the
BUSY-with-0xC0F1 case are in "Ack details" below.

The "refused" rules are checked in the firmware (`command_policy.h`) and are the
same for USB and radio unless a row says otherwise.

| Opcode | Name | Data | USB | LoRa | States | Iris | Witness action / ack |
|---|---|---|---|---|---|---|---|
| `0x0020` / `0x0021` | Status flag set / clear | | yes | yes | any | yes | Iris only; Witness acks UNKNOWN over LoRa |
| `0x0022` | Ping | | yes | yes | any | yes | Iris answers PONG over CAN; Witness acks UNKNOWN (with the Landed low-power mode on: `0x09` reply in Landed, 1 per 5 s, else BUSY) |
| `0x0023` | Pong | | | | | from Iris | |
| `0x0030` / `0x0032` / `0x0034` | Iris channel 1/2/3 load **off** | | yes | yes | **PI, L** (refused in B, C, D) | yes | OK when the CAN forward is accepted; BUSY (detail 0 = not forwarded, retry) when the CAN queue is full; REFUSED by policy in flight. Refused commands are not re-sent over LoRa |
| `0x0031` / `0x0033` / `0x0035` | Iris channel 1/2/3 load **on** | | yes | yes | any | yes | as above; ON is always allowed |
| `0x0050` | Iris 5 V regulator **off** | | yes | yes | **PI, L** (refused in B, C, D) | yes | as for load off |
| `0x0051` | Iris 5 V regulator **on** | | yes | yes | any | yes | as for load on |
| `0x00E0` | Witness debug packets on (10 Hz, USB) | | yes | yes | any | yes | OK |
| `0x00E1` | Iris debug packets on | | yes | yes | any | yes | Iris |
| `0x43xx` | IMU calibration (gyro, face, clear, status) | xx | yes | no | PI, real sensors | no | compact USB form |
| `0x4741` | GPS assist BEGIN | size, chunks, CRC | yes | yes | **PI, L** (refused in B, C, D) | no | refused in flight |
| `0x4744` | GPS assist DATA (chunk) | chunk | yes | yes | **PI, L** | no | no ack per chunk (bitmap in `0x07`); refused in flight |
| `0x4745` | GPS assist COMMIT | | yes | yes | **PI, L** | no | OK / INCOMPLETE / BAD_CRC / BAD_FORMAT; BAD_FORMAT if a record is not on the whitelist (below); refused in flight |
| `0x4751` | GPS assist STATUS | window start | yes | yes | any | no | replies `0x07` (read only) |
| `0x4752` | GPS cold start | | yes | yes | PI (refused in B, C, D, L) | no | the state is checked when the request is made and again when the GPS task executes it (`UartDriver::set_cold_start_allowed`, refreshed by the sensor task; a request that is no longer in Pad Idle is dropped with a warning) |
| `0x4844` | Pad heading (body axis -> azimuth) | axis, 0.01 deg | yes | yes | PI | yes | |
| `0x4849` | HITL enter | | yes | no | PI | yes | canonical frame only |
| `0x4858` | HITL exit | | yes | yes | any | yes | |
| `0x4248` | Bench hang (test only) | d0-d5 ignored | yes | no | any, HITL active **or RESUMED after a warm reset** (USB only) | **never** | canonical USB frame only. The sensors task stops feeding the task watchdog and idles, so the real task watchdog trips (8 s, reset reason TASK_WDT, a warm reset that resumes the flight state). Acked OK before the hang; REFUSED when HITL is off; UNKNOWN over radio; never forwarded to Iris or over LoRa |
| `0x4249` | Bench Iris warm reset (test only) | d0 `0xA5`, d1 `0x5A` (arming bytes, set by Witness) | yes | no | any, HITL active **or RESUMED after a warm reset** (USB only) | restarts Iris | canonical USB frame only. Witness sends the armed frame to Iris over CAN (never from a received frame); Iris calls `esp_restart()` (reset reason SW, warm: load restore and expander reset-pin hold run). Iris ignores it from USB/UART or without the arming bytes. Acked OK when the CAN queue took the frame, BUSY otherwise; REFUSED when HITL is off; UNKNOWN over radio, and the radio copy is not forwarded to Iris |
| `0x4252` | Bench restart (test only) | d0: 0 software restart, 1 panic | yes | no | any, HITL active **or RESUMED after a warm reset** (USB only) | **never** | canonical frame only (the 3-byte and text forms are refused by the bridge too). Warm reset that resumes the flight state; refused when HITL is off, unknown over radio. The ack is flushed to the USB host before the reset; a software restart also flushes the flash log (shutdown handler), a panic cannot |
| `0x5250` | Radio power (drive dBm, front end) | target, dBm, FE | yes | yes | see the radio power rules below | no | refused in Boost. Coast/Descent: front end OFF and any lowering refused. Detail = FE bit 8, drive dBm now |
| `0x5746` | Wi-Fi / maintenance image (on, off, status) | on, TX dBm, minutes | yes | yes | PI, L | yes | "on" restarts into the maintenance image; refusal detail 0x0001-0x0008 |
| `0x6868` | USB export (maintenance image USB disk) | | yes | **no** | PI, L | yes | restarts into the maintenance image |
| `0x6869` | Erase all log sessions (every block) | | yes | yes (radio: **send twice within 10 s**) | PI, L | yes | frees a full store. Radio first send: BUSY, detail `0xC0F1`, nothing done; an identical second send within 10 s executes. USB is single shot. Since 2026-11-18 it answers in about 40 ms: the slot headers are cleared and a new session starts at once (the session number keeps counting up, it no longer restarts at 1), and the other slot is wiped in the background in 8-block chunks, Pad Idle and Landed only (a launch cancels the wipe). The USB task and the packet log are no longer held for 4.3 s |
| `0x686A` | Clear all log sessions (headers only, fast) | | yes | yes (radio: **send twice within 10 s**) | PI, L | yes | frees a full store; same confirm rule as `0x6869` |
| `0x7300` | **Reset to Pad Idle (Landed only)** | | yes | yes (Landed) | PI (no-op), L (restart) | yes | Landed: ack OK, then Witness restarts (about 0.4 s later over USB, 1.0 s over LoRa) and boots Pad Idle with a fresh filter and log session. The restart is checked again when it fires; a state change in that window abandons it. Pad Idle: ack OK, nothing happens. Boost, Coast, Descent: refused |
| `0x7301`-`0x7304` | Set flight state B/C/D/L | | yes | no | PI; from B/C/D only L | yes | forward only; `0x7301` from Pad Idle is a manual launch (USB) |

GPS assist whitelist (receiver side, `gps_assist::is_allowed_message`): the only
record accepted is SkyTraq `0x41` "Set GPS Ephemeris" with an exact body length of
87 bytes. Any other message (a baud change `0x05`, factory defaults `0x04`, and
so on) is refused at COMMIT with BAD_FORMAT, is checked again before each send, and
the transport refuses baud and factory-reset frames while the receiver baud is fixed.
Consequence: a host-initiated hot start through a single `0x01` record no longer
works; the cold start (`0x4752`) is unchanged.

Set-flight-state rules (`command_policy::allow_set_flight_state`). Flight states
never step back down (`FlightStateMachine::set_state` refuses a lower state with
`ESP_ERR_INVALID_STATE`; the ack is REFUSED). The same state is an accepted no-op.
- **LoRa:** only `0x7300` while Landed (restart) and `0x7300` in Pad Idle (no-op).
- **USB:** from Pad Idle any state 0 to 4 (1 = manual launch). From Boost, Coast
  or Descent only Landed (or the same state). From Landed only `0x7300` (restart).
  Any other backward command (for example `0x7302` in Descent, `0x7301` in
  Landed) is refused.
- A Landed record is never resumed after a reset. Boost, Coast and Descent are
  resumed after an in-flight reset (panic, watchdog, brownout, software restart).
- Automatic Boost (Pad Idle) has three routes; any one starts Boost:
  1. Kalman height above the pad over 30 m **and** Kalman vertical velocity at
     least 30 m/s, both true continuously for 300 ms. A slow climb (vehicle,
     hoist, aircraft) does not reach this rule.
  2. Filtered axial acceleration (body +X) at or above 2.5 g for 300 ms (a leaky
     budget; independent of the position filter, so it does not need an attitude).
  3. Airborne at boot: raw barometer rate of at least +20 m/s for 1 s. Used after a
     cold reset in flight or a missed launch.
  `0x7301` (manual launch, USB) is unchanged.
- GPS fixes feed the Kalman filter only with GGA fix quality 1, 2, 4 or 5.
  Qualities 3, 6 (dead reckoning), 7 and 8 do not; they still show as a valid fix
  in telemetry and the GPS-locked flag.

Radio power rules (`0x5250`, `command_policy::allow_radio_power_change`):
- Boost: refused from any source.
- Coast and Descent: switching the front end OFF is refused, and so is any drive
  below the level in use. Raising the level is allowed. An OFF that is allowed over
  radio is applied 600 ms after the ack, so the ack is on air first, and the flight
  state is checked again at that time.
- Pad Idle and Landed: everything.
- The chosen drive level is stored in NVS (namespace `radio`, key `tx_dbm`) on the
  ground only (Pad Idle and Landed), so it survives a reset. The front-end state is
  never stored. The boot default is **-17 dBm** (range -17 to +3 dBm). A level set in
  flight is RAM only.

Aperture-local: `0x5250` with d0 = 1 (ground target) sets Aperture's own radio
and is acked on USB; it is never transmitted.

Iris load re-send (Witness side): when a commanded load state is not confirmed by
Iris, Witness re-sends it with a backoff of 2, 4, 8, 16 and then 30 s (reset by a
match, a new command, or an Iris reboot). It never gives up.

### Maintenance image

The maintenance image accepts `0x5746` (on extends the window, off returns to
flight, d0 = 2 status) and `0x5250`. Everything else is acked UNKNOWN.


## Built-in self test (BIT), see BIT_PLAN_2026-11-18.md

| Opcode | Meaning | Data | Radio | State rule |
|---|---|---|---|---|
| `0x5442` | BIT run | d0 0 full / 1 passive, d1 options (bit0 IMU self-test, bit1 image+heap, bit2 not vertical, bit3 indoor, bit4 bench; bits 5-7 refused), d2-d5 nonce | yes | Pad Idle and Landed only; refused after a warm reset in flight; BUSY while running. Ack comes at the end of the run, detail = verdict (high byte) + number of blocking-failed groups. Gate refusal: detail 0x8000 + reason |
| `0x5452` | BIT report | d0 page (0xFF summary) | yes | any state except Boost |
| `0x5443` | reserved (camera option) | | | answered UNKNOWN |
| `0x4246` | BIT fault injection (bench, flight image) | d0 fault id, d1 on/off, d2-d3 ms (max 60 s), d4 heap level | never | HITL active, canonical USB only, never forwarded |

None of them is forwarded to Iris. Packets: `0x0B` summary (12 data bytes, 1 Hz on USB, on change and every 10 s while not GO on radio) and `0x10` detail (99 data bytes, 24 records), both logged as NAND packet source 7. Layouts are in `comms.h`.
