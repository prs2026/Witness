# Pre-flight audit 2026-11-19: Iris and Aperture firmware

Scope: `Software/Iris/softwarev1IRIS/main`, `Software/Aperture/softwareattempt2/main` (flight image; `softwarev1aperture` is the stale September tree), cross-checked against `Software/COMMANDS_AND_PACKETS.md`, `Witness/softwarev1/main` (comms.h, sensors.cpp, command_bridge.cpp, tx_slot.h) and ESP-IDF v6.1 sources for driver behaviour.
Read-only review; nothing built, flashed or run. History read: `iris_findings.md`, `aperture_findings.md`, `FIXES_REPORT_2026-11-16.md`.
Not re-raised (owner decisions): radio command authentication, Iris output restore at power-up, Iris BIT.

Result: no Critical or High finding. One Medium on Aperture that can silently corrupt telemetry after a radio re-init, two further Medium operational items, the rest Low.

## Medium

### M1. Aperture: any radio re-initialisation switches the DIO1 interrupt off for the rest of the session (CONFIRMED in code; trigger is rare)
- Where: `Aperture/softwareattempt2/main/ra01.cpp:113-114` (`inputs.intr_type = GPIO_INTR_DISABLE; gpio_config(&inputs)` covers BUSY and DIO1), `ra01.cpp:245-253` (the POSEDGE type is set only inside `if (!dio1_isr_installed_)`), `ra01.cpp:273` (`wait_for_event` trusts the flag).
- Why it matters: `initialize()` is re-entrant by design (health-check recovery in `health_check()`, retry loop in `main.cpp`). In ESP-IDF v6.1 `gpio_config()` calls `gpio_set_intr_type()` and `gpio_intr_disable()` for every pin in the mask (`esp_driver_gpio/src/gpio.c:448-453`). On the first init the ISR block then re-arms DIO1. On every later init `dio1_isr_installed_` is already true, so the block is skipped and DIO1 stays disabled, while `wait_for_event()` still believes the ISR works and just times out after one 10 ms tick.
- Scenario: a ground-side supply dip or ESD resets the SX1262 (`health: packet type ... re-initializing`) or 3 SPI/BUSY failures occur. Aperture recovers and looks healthy, but now polls the radio every 10 ms. Witness sends the 2 Hz sensors and state packets back to back (`sensors.cpp` `service_radio_transmit`: after a TX completes, the next due type starts on the next pass, a few ms later). The SX1262 has one RX buffer starting at address 0 and the next packet's payload overwrites it roughly 5 ms after the previous packet ends. With a read up to 10 ms late, `receive()` can read a mix of two packets. The overrun guard (`ra01.cpp:420`) only fires when the second packet's RxDone/CrcErr is already set, which it is not yet mid-packet. The mixed packet goes to the PC with the right length and EOF byte.
- Fix: after `gpio_config(&inputs)` in every `initialize()`, re-apply `gpio_set_intr_type(DIO1, GPIO_INTR_POSEDGE)` and `gpio_intr_enable(DIO1)` (or configure DIO1 separately, only once). Also drain a stale task notification. Cheap, local, no behaviour change on the first init.
- Bench check if time allows: force a `health_check()` re-init (briefly hold BUSY or RESET), then watch the 0xF0 stream and the rx_overrun counter with Witness running.

### M2. Aperture: uplink TX power is RAM-only and falls back to -17 dBm after any Aperture reset (CONFIRMED)
- Where: `Aperture/softwareattempt2/main/ra01.h:106` (default `IRIS_RADIO_TX_POWER_DBM` = -17), `main.cpp` `handle_ground_radio_power` (sets RAM only); Horizon does not re-send `0x5250` target ground on reconnect (`Horizon/main.py` `set_radio_power` is manual only). Witness persists its own level in NVS (fix W1); Aperture has no equivalent.
- Scenario: Aperture resets (5 s watchdog, 60-failure restart, USB port reopen reset). Nothing tells the operator; commands now go out at -17 dBm and are simply not acked beyond a short range. Downlink is unaffected, so the link looks alive.
- Fix (pick one): store the level in NVS like Witness; or have Horizon re-send `0x5250` (ground) whenever it sees the 0xF0 uptime go backwards; or at minimum add "ground TX power" to the pre-launch checklist after any Aperture reconnect.

### M3. Aperture: a command can be dropped with no indication to the PC (CONFIRMED, by design but easy to miss)
- Where: `Aperture/softwareattempt2/main/main.cpp:295-299` (`transmit()` failure is counted and logged, then `uplink.pop()` runs anyway); `main.cpp:177-186` (queue `Full` and `Duplicate` only bump counters). Log level is WARN and the console is the binary stream, so nobody sees these.
- Scenario: radio busy/SPI error at the moment a camera or Wi-Fi command is due; the frame is consumed and no ack ever comes. For a one-shot command such as `05 01 00 00 00 00 00 57 46 0a` (Wi-Fi on) the operator only sees silence; Horizon's retry is the only safety net.
- Fix: on transmit failure keep the frame at the head for one or two bounded retries before popping; emit a local 0x06 ack with `IRIS_ACK_BUSY` for dropped or queue-full frames so Horizon shows "RETRY".

## Low

### L1. Iris: no retry of CAN bring-up, and no back-off on repeated bus-off (PLAUSIBLE)
- Where: `Iris/softwarev1IRIS/main/can_link.cpp:8-12` and `usb_serial_echo.cpp:179` (CAN is initialised once; `initialized_ == false` makes `CanLink::poll()` return early, so `TwaiDriver::poll()`'s own "node == nullptr, retry every second" branch is never reached). `twai_driver.cpp:205-224` (every bus-off is recovered at once, forever).
- Scenario A: `twai_new_node_onchip` fails once at boot; Iris has no CAN for the whole flight (loads and USB still work). Scenario B: a marginal bus (noise, loose connector) makes Iris go bus-off repeatedly; each recovery re-joins at once and its error frames can corrupt Witness's own frames (heartbeat, forwarded commands).
- Fix: let `CanLink::poll()` call `driver_.poll()` even when not initialised; count bus-offs and after, say, 5 within 10 s stop transmitting for 10 s before trying again.

### L2. Iris: task watchdog does not reset on a hang; sdkconfig and sdkconfig.defaults disagree (CONFIRMED)
- Where: `Iris/softwarev1IRIS/sdkconfig` (`CONFIG_ESP_TASK_WDT_PANIC` not set, idle task CPU0 only; nothing calls `esp_task_wdt_add`); `sdkconfig.defaults` has no WDT, brown-out or log-level lines (Aperture's does); `CONFIG_LOG_DEFAULT_LEVEL_INFO=y`.
- Impact: a wedged main loop that still yields never trips the TWDT; Iris stays silent with loads in the last state. Not harmful to Witness: the TWAI controller keeps ACKing in hardware. Same as finding 11 in `iris_findings.md`, still open. Brown-out (level 7, interrupt mode) and INT WDT (300 ms, panic) are enabled; panic reboots with zero delay and the warm-reset restore applies.
- Fix: `CONFIG_ESP_TASK_WDT_PANIC=y`, subscribe the main task and feed it in the loop; copy the flight lines into `sdkconfig.defaults` so a regenerated sdkconfig cannot change them silently. Build the image from the committed sdkconfig.

### L3. Iris: PAC1931 is initialised only if the expander bus is up at boot (CONFIRMED)
- Where: `Iris/softwarev1IRIS/main/main.cpp:98-105`. If the first `bring_up_expander()` fails, the bus comes up later in the loop but `current_monitor.initialize()` is never retried: current telemetry stays flagged "monitor error" all flight. Loads still work. Fix: retry the PAC init in the main loop until it succeeds.

### L4. Docs: packet-rate documentation is stale after commit 0d3a403 (CONFIRMED)
- `COMMANDS_AND_PACKETS.md` rows for 0x01 and 0x02 (lines 20-21) say LoRa 1 Hz in flight and USB state 1 Hz; code is LoRa 2 Hz for both and USB state 2 Hz (`Witness/softwarev1/main/sensors.h` `kRadioSensorPacketRateHz`, `kRadioStatePacketRateHz`, `kStatePacketRateHz`). `Aperture/softwareattempt2/main/GROUND_STATION_PROTOCOL.md` lines 35-37, 741, 1045 also say 1 Hz. Pad Idle (3 s) and camera 1 Hz are correct. Any Horizon or witnessOS link-lost or staleness timeout derived from 1 Hz should be re-checked against 2 Hz (not reviewed here).

### L5. Aperture: stale tree and logging on the binary stream (CONFIRMED, low)
- `Aperture/softwarev1aperture` is the September version (no pacing, no ring, no watchdog). Easy to flash by mistake; consider deleting or renaming it before imaging.
- The app log tag is raised to INFO (start of `app_main`): one "stack_free_min" line per minute goes into the same USB stream as the frames. IDF v6.1 writes console characters one ring item at a time (`usb_serial_jtag_vfs.c:740`), so a frame can land in the middle of a log line. Frames are never split, and printable ASCII cannot be a valid packet ID, so Horizon's resync should cope, but the cleanest flight setting is WARN for that tag.

## Checked and found sound

Aperture
- Packet lengths: every Witness radio packet is at most 101 bytes (BIT detail 0x10) and Aperture's receive buffer is 128 (`IRIS_RADIO_MAX_PACKET_LENGTH`). 0x01=23, 0x02=65, 0x03=19, 0x06=13, 0x07=76, 0x08=19, 0x09=28, 0x0B=14, 0x10=101, 0xF0=29 match `comms.h` and COMMANDS_AND_PACKETS.md. Aperture does not parse radio payloads, it forwards them whole, so a new packet ID cannot be mishandled; oversize (more than 128) is dropped and counted.
- 2 Hz load: state 65 B is about 31 ms on air, sensors 23 B about 15 ms, camera 19 B about 14 ms at SF7/500/CR4-5, about 105 ms per second; the USB ring (3072 B, drop-oldest, whole frames, `pop_if` race handled) carries roughly 250 B/s plus echo, ample headroom.
- Radio parameters (926 MHz, SF7, BW500, CR4/5, preamble 8, explicit header, CRC, IQ normal) are identical on both ends; `comms.h` is byte-identical on Iris, Witness and Aperture; `tx_slot.h` is identical on Witness and Aperture.
- RX path: IRQ clear ordering, GetRxBufferStatus/GetPacketStatus offsets, 60-byte chunked ReadBuffer, drop-on-overrun and CRC handling are correct. One read per loop is enough at these rates while DIO1 works (see M1).
- Slot timing: Aperture sends commands only at phase 800-950 ms of the PPS second when GPS time is fresh; Witness downlink ends by 700 ms; the two-packet 2 Hz schedule fits the 700 ms window. A burst follow-up (up to 3, pacer) may start outside 800-950 but within 800 ms of the previous one and cannot reach the downlink window (worst case ends near phase 975). Without GPS or PPS Aperture sends at once (accepted behaviour, can collide with a downlink packet).
- Command uplink: `CommandAssembler` resyncs on 0x05 and is bounded; USB echo is byte-for-byte as documented; ground radio power (0x5250 target 1) is handled locally and acked, all other opcodes are queued; duplicate suppression applies only while a frame is still queued. The Wi-Fi frame `05 01 00 00 00 00 00 57 46 0a` assembles correctly (opcode bytes 7-8 = 0x5746, d0 = 1 = on), is forwarded unchanged and is not taken for a local command. On Witness it passes `check_policy`, is forwarded to Iris over CAN (unknown opcode, ignored there), and `handle_wifi` applies the Pad Idle / Landed rule.
- Reconnect and reset: 5 s task watchdog with panic reboot, loop never blocks that long (BUSY 100 ms then 5 ms, TX timeout 1 s), radio init retries with back-off and restarts after 60 failures, TX always ends in RX, health check every 1 s; USB TX is its own task with a 20 ms write timeout and retry, so an absent or stalled host never blocks the radio loop.
- GPS: NMEA parser is bounds-checked (160 B sentence, 24 fields, checksum required); PPS ISR is IRAM with an ISR-safe critical section; stacks are adequate (`main` 6144 B, usb 3072 B, gps 4096 B).

Iris
- No path found that lets Iris flood Witness: telemetry is 1 Hz (3 frames) plus the optional 10 Hz debug stream (2 frames) after an explicit `0x00E1`; ping replies are one packet; the TX queue holds 8 packets and drops new ones when full; `CanLink::poll()` sends at most 8 fragments per 10 ms loop; a stuck TX queue is rebuilt after 1 s; `fail_retry_cnt` is 3. A lone Iris with no ACK stays error-passive and cannot go bus-off from ACK errors alone.
- Bus-off recovery is wired (`on_state_change`, then `twai_node_recover` with re-arm on failure). The receive ISR queue (16) drops and counts rather than blocking.
- Output switching: commanded state is cached, read back every 100 ms (`verify_and_heal`) and re-pushed every 1 s; recovery never writes "all off"; the latch-cache bug from the earlier audit is gone (`write_pin` always writes). Warm-reset restore needs a warm reset reason and a valid magic/mask/CRC record; cold resets give all loads off. The bench reset opcode is honoured only from CAN with arming bytes, and Witness never forwards it from radio or USB data (`command_bridge.cpp` `is_bench_command`, `send_bench_iris_reset`).
- Crash behaviour: panic prints and reboots with no delay, main stack 6144 B (large objects static), stack canary on, brown-out detection on. The expander reset pad is held across CPU resets so a warm reset does not drop the loads (needs the planned bench check, not re-raised).
- Bounds: CAN reassembly, USB, UART1 and ASCII parsers are bounded (CAN packet 103 B; ASCII 256 B with overflow discard). Load/5 V OFF cannot arrive from radio in flight: Witness refuses them in Boost, Coast and Descent before they reach Iris.
- The TWAI ISR is not IRAM-placed, so it is deferred during flash writes rather than crashing; the only flash access after boot is the stale NVS key check about 3 s after power-up.
