# Intercomm: shared Witness/Iris CAN contract

`can_transport.h` is the single header-only codec used by both Witness
(ESP32-S3) and Iris (ESP32-C3) firmware. Both include it by relative path, so
change it in one place and rebuild both boards together.

## Physical layer

- Transceiver: TI TCAN3414 (3.3 V) on both boards. STB (pin 8) low; SHDN
  (pin 5) low or left to its internal pulldown.
- Classical CAN, **500 kbit/s**, standard 11-bit identifiers. The transceiver
  supports CAN FD, but neither ESP32 TWAI controller uses it.
- CANH, CANL and a common ground between boards. Terminate 120 ohm at the two
  bus ends only (Iris: R39).
- Pins: Witness TX GPIO37 / RX GPIO36; Iris TX GPIO3 / RX GPIO1 (Iris PCB net
  labels are reversed relative to signal direction, see `Iris/.../hardware.h`).

## Identifiers

| ID | Direction | Carries |
|---|---|---|
| `0x120` | Witness → Iris | `0x05` commands, `0xFF` heartbeat (1 Hz) |
| `0x121` | Iris → Witness | `0x03` camera telemetry, `0xE1` debug, `0x05` pong |

## Fragmentation

A canonical application frame (`ID | payload | 0x0A`, max 103 bytes) is split
into Classical CAN frames:

| Byte | Meaning |
|---:|---|
| 0 | Sequence number (same for all fragments of one packet) |
| 1 | Zero-based fragment index |
| 2 | Total canonical packet length |
| 3..7 | Up to 5 packet bytes; final frame uses its exact DLC, no padding |

The receiver (`intercomm::Reassembler`) drops the transfer on a wrong ID,
extended/remote frame, bad DLC, inconsistent total or sequence, out-of-order
index, or if 250 ms pass since fragment 0. Callers must still validate the
packet ID, its fixed length from `comms.h`, and the final `0x0A`.

There is no delivery ACK or retransmission of whole packets; transport is
bounded best-effort. The fragment envelope never leaves the CAN bus — Witness
strips it before forwarding Iris packets to USB/LoRa, so Aperture and Horizon
see only canonical frames.

## Tests and tools

```sh
g++ -std=c++17 -Wall -Wextra -Werror can_transport_test.cpp -o can_transport_test
./can_transport_test
```

`bench_check.py` captures the three bench USB TCP streams on `prs2026hitl`
(7100 Iris, 7101 Aperture, 7102 Witness) and can send the harmless Iris ping
through Aperture. Captures go to `captures/`.
