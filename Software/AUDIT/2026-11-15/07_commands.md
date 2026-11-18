# Audit 07: command surface, USB, CAN/Iris (read-only, working tree of 2026-11-15)

Paths are under Software/Witness/softwarev1/main unless stated. No file was modified. IDF driver source was not available on this machine, so IDF behaviour is from memory and marked "needs test".
Code that looked mid-edit: usb_serial_echo.cpp (0x7300 reset path), command_policy.h, sensors.cpp. Findings were read from the tree as found.

## Findings table

| # | Sev | Conf | Where | Summary |
|---|-----|------|-------|---------|
| 1 | High | confirmed | hardware_pins.h:85, ra01.h:78, usb_serial_echo.cpp:1079-1094 | TX power is RAM only and every boot starts at -17 dBm. Any reset (the new Landed->Pad reboot, a warm reset resumed in flight, panic or WDT) silently drops the downlink about 20 dB until a ground 0x5250 is sent. After the 0x7300 reboot the weakened link may be too weak to send that command. The maintenance switch carries radio_dbm across (boot_manager.cpp:155-187); the plain restart and flight resets do not. |
| 2 | High | confirmed | usb_serial_echo.cpp:609-620, command_bridge.cpp:79-93, iris_load_tracker.h | Loads and 5 V (0x0030-0x0035, 0x0050/51) are forwarded to Iris from USB and LoRa in every state: no state gate, no arm interlock, no auth or replay protection. If OUT1-3 drive pyro this is the top safety hole. Acceptance of a radio frame needs only ID 0x05 plus either 10 bytes ending 0x0A or a bare 3-byte frame, plus the LoRa CRC. |
| 3 | High | confirmed | usb_serial_echo.cpp:945-974, command_policy.h:52 | 0x5250 is accepted in Coast and Descent from radio. FE OFF (d2=2) or -17 dBm kills downlink and uplink for the rest of the flight (RAM only; cannot be undone over the lost link). Refused only in Boost. No auth. |
| 4 | High | confirmed | usb_serial_echo.cpp:582-590, 1041-1068, flash_logger.cpp:847-865, 1008-1030 | Radio 0x6869/0x686A is accepted in Pad Idle and Landed, unauthenticated and replayable. In Landed it erases the flight just flown; in Pad Idle it erases pre-launch data, empties the prelaunch queue and switches capture off for the duration (seconds for the full erase of 1004 blocks). The state is checked once, the erase runs later on the logger task: a launch during it flies with logging off and restarts as a new session. |
| 5 | Medium | confirmed | usb_serial_echo.cpp:156-172 | echo_bytes uses portMAX_DELAY plus a taskYIELD() loop on a 0 return. A host that stops reading, or an unplug, blocks the usb task forever; it also runs radio command handling, so the 8-deep queue fills and later radio commands are dropped. If write returns 0 repeatedly it busy-loops at priority 3 and starves IDLE, so the 5 s task WDT resets the board (needs test). Earlier finding (witness_comms #12): still present. |
| 6 | Medium | confirmed | usb_serial_echo.cpp:774-794, flash_logger.cpp:847-857, 1071 | erase/clear wait on a semaphore with portMAX_DELAY. If the logger task deletes itself on error (flash_logger.cpp:1071) without giving erase_semaphore_, the usb task hangs permanently: no later USB or radio command is processed. |
| 7 | Medium | confirmed | maintenance_mode.cpp:104-114, usb_serial_echo.cpp:1030, 1086 | restart_after_ms never rechecks flight state. A 0x7300 reset (Landed), 0x5746 or 0x6868 restart fires 0.4 s (USB) or 1.0 s (radio) after the ack even if a launch happened in between (earlier M3, still present). No shutdown handler: up to 1 s of staged NAND data lost and a NAND page can be cut mid-program. |
| 8 | Medium | confirmed | usb_serial_echo.cpp:1079-1094, flash_logger.cpp:960-975 | The Landed->Pad reboot starts a fresh log session. If the other slot already holds a flight, both slots are full and nothing is logged (LOG SLOTS FULL) until an erase/clear. A second flight re-armed with 0x7300 alone is unlogged. Only status byte 1 bit 7 shows it. |
| 9 | Medium | confirmed | usb_serial_echo.cpp:461-465, 601-608, 640-680; command_policy.h:71-72 | The new reboot is reachable from USB text `05 73 00`, compact binary, canonical binary or ASCII frames, and from radio (a bare 3-byte `05 73 00` is enough), all in Landed. A false Landed while still falling (earlier M6) plus one stray or replayed 0x7300 reboots the computer mid-descent; Landed is not resumed, so it returns to Pad Idle with a 3 s telemetry rate. |
| 10 | Medium | plausible, needs test | twai_driver.cpp:100-141, 283-308 | TX slots are freed only in the tx-done callback. On bus-off and recovery, frames still queued in the controller may be dropped without a callback, leaking slots (8 total). Once all leak, transmit() returns ESP_ERR_TIMEOUT forever: no heartbeat to Iris, no load commands, no load re-send. Only suspend() resets slots. Not flight-data critical. |
| 11 | Medium | plausible | iris_load_tracker.h:59-92, can_link.cpp:109-117 | The load re-send has no retry cap. While Iris-reported OUT flags disagree with the commanded state, commands are re-sent every 2 s forever (every 500 ms after an apparent Iris reboot). If Iris trips a load (over-current) or refuses it, Witness keeps commanding it ON again. |
| 12 | Medium | plausible, needs test | sensors.cpp:1390-1420 (sensor task on core 1) | Every received radio packet is logged at INFO with a full ESP_LOG_BUFFER_HEX (up to 255 bytes) from the estimation task, plus a second warning when the packet is >105 bytes. A busy channel or a command flood adds console work to the sensor loop. Depends on how the USB console behaves with no host. |
| 13 | Medium | confirmed | gps_assist.cpp:19-32, command_policy.h:60 | Assist upload has no record whitelist and is allowed in Coast and Descent from radio: a CRC-valid blob can hold SkyTraq restart, baud or factory-default messages and break the GPS in flight. Needs a real upload, so noise cannot do it; deliberate or replayed only (earlier H2, part still open). |
| 14 | Low | confirmed | usb_serial_echo.cpp:341-380 | A binary frame stalled more than 20 ms is cut and treated as a compact command if exactly 3 bytes arrived: a canonical frame whose d0,d1 are 0x68 0x69 (e.g. a GPS assist DATA chunk) would act as erase (Pad Idle/Landed only). The tail is then parsed as text. Very rare. |
| 15 | Low | confirmed | usb_serial_echo.cpp:174-212 | A 0x0C or 0x0E byte outside a binary command makes the HITL assembler absorb the next 46 or 149 bytes, swallowing a following command (20 ms idle resets it). USB host only. |
| 16 | Low | confirmed | command_bridge.cpp:52-63, usb_serial_echo.cpp:364-369 | Every valid USB command is forwarded to Iris and re-transmitted on LoRa before the local policy runs, including commands then refused (e.g. 05 73 01 in Boost). |
| 17 | Low | confirmed | usb_serial_echo.cpp:535-541, command_bridge.cpp:21-28 | 3-byte radio frames are widened to canonical, so they are acked (args = uptime bytes) and run with those args; the compact radio branch is dead code. The doc says no ack for 3-byte frames. |
| 18 | Low | confirmed | main.cpp:59-89 | ESP_ERROR_CHECK on spi_data_forwarder, command_bridge, usb_serial_echo, flash_logger, sensors and heartbeat start: any failure aborts and reboot-loops. CAN start failure is handled (warning only). |
| 19 | Low | plausible | usb_serial_echo.h:46 | Usb task stack 6144. Idle used about 2.3 KB; the deepest path (prepare_switch: two image reads, NVS, esp_ota_set_boot_partition, plus ESP_LOG formatting) passed on the bench. The new reset path (nvs_open, erase_key, esp_timer create/start) is a subset. Margin not measured. Heartbeat (2048) and forwarder (3072) stacks suit their code. |

Earlier documented findings still present in this facet: M3 (restart without recheck), L1 (loads not gated), H2 in part (assist and 0x5250 allowed in Coast/Descent, no record whitelist), witness_comms #12 (echo blocks), no auth/sequence on radio commands, USB commands re-sent over LoRa. Fixed since: radio 0x6868 is USB-only, erase/clear gated to Pad Idle/Landed, Boost gating of 0x5250 and assist, one-way flight state, sets the radio 0x7301 launch refusal.

## Command matrix (from code)
Policy: command_policy.h and the dispatch in usb_serial_echo.cpp. U = USB, R = LoRa. CAN: nothing from Iris reaches the handlers (can_link.cpp:134-150 forwards to USB only; Iris command packets never enter CommandBridge). PI Pad Idle, B Boost, C Coast, D Descent, L Landed. "fwd" = also sent to Iris over CAN; USB frames are also re-sent over LoRa.

| Opcode | Src | PI | B | C | D | L | Notes (hazard in B/C/D) |
|---|---|---|---|---|---|---|---|
| 0x0020/21, 0x00E1, any unknown | U,R | fwd; ack OK (U) / UNKNOWN (R) | same | same | same | same (CAN wakes for 3 s) | forwarded unfiltered |
| 0x0022 ping | U,R | fwd; R ack UNKNOWN | same | same | same | R UNKNOWN (landed low-power mode off) | |
| 0x0030-35, 0x0050/51 | U,R | fwd, ack OK | same | same | same | same | no gate: actuator change in any state (finding 2) |
| 0x00E0 debug | U,R | OK | OK | OK | OK | OK | 10 Hz USB debug only |
| 0x43xx calibration | U (compact) | OK | REF | REF | REF | REF | R: UNKNOWN; status (0x53) always OK |
| 0x4741/44/45 assist | U,R | OK | REF | OK | OK | OK | records written to GPS in C/D (finding 13) |
| 0x4751 assist status | U,R | OK | OK | OK | OK | OK | replies 0x07 |
| 0x4752 cold start | U,R | OK | REF | REF | REF | REF | |
| 0x4844 pad heading | U,R | OK | REF | REF | REF | REF | R is also forwarded to Iris |
| 0x4849 HITL enter | U | OK | REF | REF | REF | REF | R: always REF |
| 0x4858 HITL exit | U,R | OK | OK | OK | OK | OK | acks OK even when inactive |
| 0x5250 radio power | U,R | OK | REF | OK | OK | OK | finding 3 |
| 0x5746 Wi-Fi on | U,R | switch+reboot | REF | REF | REF | switch+reboot | off/status always OK; bad args first |
| 0x6868 USB export | U | switch+reboot | REF | REF | REF | switch+reboot | R: REF |
| 0x6869 erase / 0x686A clear | U,R | runs | REF | REF | REF | runs | finding 4 |
| 0x7300 | U | OK no-op | REF | REF | REF | REBOOT | R: PI no-op OK, L reboot, else REF |
| 0x7301-0x7304 | U | PI: any 0..4 (7301 = launch) | B same, 7304 OK | C same, 7304 OK | D same, 7304 OK | 7304 same only | R: REF |
| 0x7305-0x73FF | U | REF | REF | REF | REF | REF | R: UNKNOWN |
| text LED, CAM, E0, TX hex | U | OK | OK | OK | OK | OK | TX forwards 0x05 frames to Iris and radio only |

Mismatches with COMMANDS_AND_PACKETS.md:
- Doc: USB set-state "any value from Pad Idle or Landed". Code: from Landed only 0x7304 (same) and 0x7300 (reboot); backwards moves are refused (command_policy.h:88-94). The doc's row also does not say 0x7300 reboots the board (USB after 0.4 s, radio after 1.0 s).
- Doc: 3-byte radio frames get no ack; code acks them (widened by the bridge).
- Doc: 0x43xx radio "no": code acks UNKNOWN and forwards to Iris.
- Doc row 0x7300 "Landed -> PI only" over LoRa: also allowed Pad Idle -> Pad Idle as a no-op (not documented).

## Details

### Authentication and noise
- Radio acceptance: byte0 == 0x05 (sensors.cpp:1419); bridge accepts length 10 with byte9 == 0x0A, or length 3 with no EOF (command_bridge.cpp:12-30). No application CRC, key, counter or timestamp. Only the SX1262 header and payload CRC16 and the default (private) sync word protect it (ra01.cpp never sets a sync word); a transmitter that clears the CRC flag in its header is not CRC-checked.
- Matching bits needed: 10-byte frame 8 (ID) + 8 (EOF) + 16 (opcode) = 32, plus extra argument bits for 0x5250 (d0=0, d1 range, d2=2) and 0x5746; 3-byte frame 24 bits plus length 3. The noise frame must also pass LoRa header detection and CRC16, so a random frame is accepted with probability below about 2^-45 (3-byte) and 2^-53 (10-byte). Noise is not the threat; replay and a deliberate or accidental transmission on a matching configuration are, and nothing prevents replay of a captured erase, FE-off or load command.
- USB: 0x05 always starts a binary command; 3-byte and 10-byte forms act with no checksum. Reachable only from a host.

### Destructive commands in B/C/D
- Reboot, erase, clear, maintenance/Wi-Fi, USB export, HITL enter and the cold start / pad heading are refused in Boost, Coast and Descent from both sources (command_policy.h:35-41, 71-95; wifi_ota_logic.h:14-17; sensors.cpp enter_hitl requires Pad Idle; set_pad_heading requires Pad Idle).
- Allowed in B/C/D and consequential: loads/5V forward (all), 0x5250 (C, D), assist upload (C, D), USB-only 0x7304 Landed.
- Long handlers on the usb task: full erase (seconds), prepare_switch (SHA-256 of maintenance image, otadata write stalls both cores), assist BEGIN (4 KB memset). All except assist are Pad Idle/Landed only.

### USB with no host
- Telemetry forwarder drops after a 20 ms timeout and checks is_connected (spi_data_forwarder.cpp:250-270): OK.
- ESP_LOG to the USB console: the bench ran for an hour with the cable attached but nothing reading (same as a full ring) without wedging tasks, so console logging is probably non-blocking. A fully unplugged cable on battery is not proven in this tree: test with USB unplugged and heavy logging.
- Echo path blocks (finding 5), but only runs when bytes arrive.

### CAN / Iris
- Dead bus (Iris absent, off, or shorted): frames get no ACK, fail_retry_cnt=3 drops them and frees the slot via callback; TEC climbs to bus-off after about 32 heartbeats (about 30 s), poll() starts recovery; repeats. transmit() uses timeout 0 and never blocks; at most one warning per second. RX queue 16 deep, drained every 1 ms. Nothing blocks the sensor or radio path. Slot-leak risk: finding 10.
- Reassembly (Intercomm/can_transport.h) checks identifier, DLC 4..8, total 2..103, index, offset+count, sequence and 250 ms timeout; buffer 103 bytes. CanLink forwards only 0x03/0xE1/0x05 of exact length with EOF. OK.
- Camera/debug relay cannot starve telemetry: the radio sends the latest cached frame per type on a schedule, the USB queue (16) is non-blocking and drops.
- TWAI ISR is not in IRAM (CONFIG_TWAI_ISR_IN_IRAM unset), so it is merely delayed during flash-cache-off windows (NVS/OTA/otadata writes, Pad Idle/Landed only); the IRAM_ATTR on the callbacks is harmless.
- Heartbeat task: non-blocking queue_packet, no loops, stack 2048 adequate.

## Checked and OK
- USB text parser bounds: command_buffer_ 313 bytes, writes limited to kCommandBufferSize-1 with an overflow flag; normalize_command terminates; parse_hex_byte never reads past the terminator; hex output arrays limited by byte_count.
- Binary parser writes capped at 10 bytes; frame_opcode used only for 10-byte frames; HITL/binary boundary logic in consume_input is correct.
- CommandBridge: canonical_command admits only length 3 or 10 with EOF; memcpy bounded; queue sends use timeout 0; receive checks capacity.
- CanLink and TwaiDriver: send_command argument checks, pending fragment cursor, 200 ms packet expiry, receive ring under critical section, ISR full-queue discard.
- gps_assist Assembler bounds (offset < length, 4096 blob, 128-byte bitmap, record parse) and the copy to the receiver under lock.
- send_ack, send_reply and send_transfer_status buffer sizes (11 and 74 byte payloads, 105-byte frame); radio power clamped -17..+3 (ra01.cpp:460); Wi-Fi arguments validated; Ra01 power/FE setters are atomics (no SPI from the usb task).
- set_flight_state is a CAS one-way transition, safe against the sensor task.
- maintenance_mode::consume honours a request only after ESP_RST_SW and erases it first; main.cpp cancels a stale request on each flight boot.
- 0x7300 reboot path logic: pending maintenance request cancelled, timer restart idempotent (repeated commands only postpone it), boots Pad Idle (Landed never resumed in flight_state_store.cpp).
- Iris load re-send: resends precede new user commands; reboot detection by Iris uptime decreasing; rate limited to 2 s and 500 ms.
