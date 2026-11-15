# Witness commands and packet types (quick reference)

Generated from the firmware on 2026-11-15 (`Witness/softwarev1/main/comms.h`,
`command_policy.h`, `usb_serial_echo.cpp`, `command_bridge.cpp`, `sensors.cpp`,
Aperture `main.cpp`). The byte-level layouts are in `GROUND_STATION_PROTOCOL.md`;
this file is the overview. All frames are `ID | data | 0x0A`, multi-byte fields
big-endian.

Flight states: **PI** Pad Idle (0), **B** Boost (1), **C** Coast (2), **D**
Descent (3), **L** Landed (4). "any" = every state.

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

Radio scheduling (`main/tx_slot.h`, since 2026-11-14):
- With a GPS fix (UTC from GGA), Witness starts a telemetry packet only if it
  ends in the first 700 ms of the UTC second. Aperture, with a fix and a PPS
  edge, starts commands only between 800 and 950 ms.
- After Witness hears any command frame it holds its telemetry for 1 s.
  Aperture sends follow-up commands at once for 800 ms after one went out.
- Acks and replies (`0x06`, `0x07`, `0x09` ping replies) are never held.
- Without GPS time on a side, that side sends whenever it can.

## Status bytes (in `0x01`, `0x02`, `0x09`, `0xFF`)

| Byte | Bit | Meaning |
|---|---|---|
| 0 | 0 | heartbeat toggle |
| 0 | 1 | low-g accel ready |
| 0 | 2 | gyro ready |
| 0 | 3 | IMU temperature ready |
| 0 | 4 | NAND init failed |
| 0 | 5 | high-g accel ready |
| 0 | 6 | barometer ready |
| 0 | 7 | NAND logging failed |
| 1 | 0-2 | flight state (0 PI, 1 B, 2 C, 3 D, 4 L) |
| 1 | 3 | GPS locked |
| 1 | 4 | HITL active |
| 1 | 5 | MAINTENANCE (never set by the flight image; the maintenance image sends `0x08` instead) |
| 1 | 6 | RESUMED: an in-flight state was resumed after an MCU reset |
| 1 | 7 | **LOG SLOTS FULL** (new): both NAND slots hold a flight, nothing is logged until erase/clear |

## Commands (`0x05`, opcode in bytes 7-8, data d0-d5 in bytes 1-6)

Columns: **USB / LoRa** = accepted from that source; **States** = flight states
in which it is accepted (otherwise acked `REFUSED`); **Iris** = also forwarded
to Iris over CAN. Ack results: 0 OK, 1 REFUSED, 2 BAD_ARGUMENT, 3 UNKNOWN,
4 BUSY, 5 INCOMPLETE, 6 BAD_CRC, 7 BAD_FORMAT.

| Opcode | Name | Data | USB | LoRa | States | Iris | Witness action / ack |
|---|---|---|---|---|---|---|---|
| `0x0020` / `0x0021` | Status flag set / clear | | yes | yes | any | yes | Iris only; Witness acks UNKNOWN over LoRa |
| `0x0022` | Ping | | yes | yes | any | yes | Iris answers PONG over CAN; Witness acks UNKNOWN (with the Landed low-power mode on: `0x09` reply in Landed, 1 per 5 s, else BUSY) |
| `0x0023` | Pong | | | | | from Iris | |
| `0x0030`-`0x0035` | Iris channel 1/2/3 load off/on | | yes | yes | any | yes | acked OK on receipt (Iris applies it; Witness re-sends after an Iris reboot) |
| `0x0050` / `0x0051` | Iris 5 V regulator off / on | | yes | yes | any | yes | as above |
| `0x00E0` | Witness debug packets on (10 Hz, USB) | | yes | yes | any | yes | OK |
| `0x00E1` | Iris debug packets on | | yes | yes | any | yes | Iris |
| `0x43xx` | IMU calibration (gyro, face, clear, status) | xx | yes | no | PI, real sensors | no | compact USB form |
| `0x4741` | GPS assist BEGIN | size, chunks, CRC | yes | yes | **all but B** | no | refused in Boost |
| `0x4744` | GPS assist DATA (chunk) | chunk | yes | yes | **all but B** | no | no ack per chunk (bitmap in `0x07`); refused in Boost |
| `0x4745` | GPS assist COMMIT | | yes | yes | **all but B** | no | OK / INCOMPLETE / BAD_CRC / BAD_FORMAT; refused in Boost |
| `0x4751` | GPS assist STATUS | window start | yes | yes | any | no | replies `0x07` |
| `0x4752` | GPS cold start | | yes | yes | PI | no | |
| `0x4844` | Pad heading (body axis -> azimuth) | axis, 0.01 deg | yes | yes | PI | yes | |
| `0x4849` | HITL enter | | yes | no | PI | yes | canonical frame only |
| `0x4858` | HITL exit | | yes | yes | any | yes | |
| `0x5250` | Radio power (drive dBm, front end) | target, dBm, FE | yes | yes | **all but B** | no | refused in Boost (new); detail = FE bit 8, dBm now |
| `0x5746` | Wi-Fi / maintenance image (on, off, status) | on, TX dBm, minutes | yes | yes | PI, L | yes | "on" restarts into the maintenance image; refusal detail 0x0001-0x0008 |
| `0x6868` | USB export (maintenance image USB disk) | | yes | **no** | PI, L | yes | restarts into the maintenance image |
| `0x6869` | Erase all log sessions (every block, slow) | | yes | **yes (new)** | PI, L | yes | frees a full store |
| `0x686A` | Clear all log sessions (headers only, fast) | | yes | **yes (new)** | PI, L | yes | frees a full store |
| `0x7300`-`0x7304` | Set flight state PI/B/C/D/L | | yes | Landed -> PI only | see below | yes | |

Set-flight-state rules (`command_policy::allow_set_flight_state`):
- **LoRa:** only Landed to Pad Idle.
- **USB:** any value from Pad Idle or Landed. From Boost, Coast or Descent only
  Landed (or the same state).

Aperture-local: `0x5250` with d0 = 1 (ground target) sets Aperture's own radio
and is acked on USB; it is never transmitted.

### Maintenance image

The maintenance image accepts `0x5746` (on extends the window, off returns to
flight, d0 = 2 status) and `0x5250`. Everything else is acked UNKNOWN.
