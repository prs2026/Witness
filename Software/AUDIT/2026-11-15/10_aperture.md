# Aperture firmware audit (softwareattempt2) - read-only

Scope: Aperture\softwareattempt2\main (main.cpp, ra01.*, lc86_*, tx_slot.h, comms.h, hardware.h), sdkconfig, docs; compared with Witness\softwarev1\main (ra01.cpp, tx_slot.h, comms.h, sensors.cpp) and Horizon\witness_protocol.py. Paths below are relative to Aperture\softwareattempt2\main unless stated. Nothing was modified, built or flashed.

Context facts used: CONFIG_FREERTOS_HZ=100 (sdkconfig:2536), so `vTaskDelay(1)` = 10 ms. Main task stack 3584 B (sdkconfig:1997). Task WDT 5 s without panic (2019-2021). Panic = print + reboot (1975). Console = USB Serial/JTAG (sdkconfig.defaults, sdkconfig:1881), the same port as the binary bridge. Log level INFO (2707). Radio is polled over SPI; DIO1 is never read and no GPIO interrupt exists for the radio.

## Findings table

| # | Sev | Conf | Where | Finding |
|---|---|---|---|---|
| 1 | High | confirmed (reading) | main.cpp:57-67 | Radio init failure is permanent: infinite log loop, no retry, no reboot, no 0xF0 status either. Aperture is deaf and silent for the whole session. Witness has an SPI round-trip probe, Aperture does not |
| 2 | High | confirmed (reading) | ra01.cpp:319-332, 274-317, main.cpp:194-211 | No runtime recovery/health check. BUSY stuck high makes every receive() burn 500 ms and log, forever. SetRx failure after TX leaves radio in standby forever. RESET pin never used after init |
| 3 | High | confirmed by reading; magnitude needs test | ra01.cpp:323-330 | `wait_until_ready()` yields with `vTaskDelay(1)` (10 ms) whenever BUSY is high at the first check, before every SPI command. Per received packet up to ~6 commands, per uplink ~10 commands. Witness fixed this with a us spin (Witness ra01.cpp:364-372) |
| 4 | High | confirmed (reading) | main.cpp:161-192, tx_slot.h:93 | Uplink queue drains in a continuous burst: `last_uplink_ms` is refreshed on every send, so `now - last < 800` stays true while the queue is non-empty, regardless of GPS slots. With no PPS it is unconditional. A PC that keeps re-sending commands keeps Aperture transmitting; Witness additionally holds telemetry 1000 ms after each command it hears. Downlink is effectively off for the duration |
| 5 | High | plausible, needs test | main.cpp:91-93, 199-205 | Blocking USB writes (100 ms timeout each, 512 B TX ring) in the single RX loop. Host present but not reading (stalled Horizon, ser2net with no client, Pi port in over-current, DTR not asserted) blocks the loop 100 ms per write; radio packets are overwritten/lost meanwhile. No per-packet counter or log of drops |
| 6 | High | plausible (bench memory + code), needs test | ra01.cpp:206-272 | Uplink TX at raised drive makes the Ra-01SH-P pull USB current. Per bench memory, Pi ports trip on over-current when it transmits, dropping the USB device (re-enumeration, ser2net port order shift, telemetry lost until reconnect). Aperture has no mitigation; boot power -17 dBm hides it until 0x5250 is sent |
| 7 | Medium | confirmed (reading) | main.cpp:179,183,207,172; lc86_gps.cpp:230,269 | ESP_LOG text shares the binary stream (console on same USB Serial/JTAG). Every CRC error logs "Receive failed ... ESP_ERR_INVALID_CRC", each uplink logs, GPS logs every 30 s and on fix change. Text can land between or, on partial writes, inside frames. Horizon resyncs (drops the damaged frame) but can lose neighbours |
| 8 | Medium | confirmed (reading) | ra01.cpp:286-290, 297-300, 311-314 | RX race: IRQ is cleared after the read (`clear = {0x00,0xFF}`), so a second packet arriving between GetRxBufferStatus and ClearIrq has its RxDone flag cleared (lost), or ReadBuffer returns a mix of old and new payload bytes with a good radio CRC (corrupt frame passed to PC). Window is the 60+ ms of item 3 |
| 9 | Medium | confirmed (reading) | main.cpp:211 | 10-20 ms poll period with a single-packet radio buffer: two packets less than ~one poll apart (Witness immediate-queue acks/replies right after telemetry; 8-byte heartbeat is about 6 ms airtime) overwrite each other. First packet lost silently, no counter |
| 10 | Medium | plausible, needs test | main.cpp:34-213, sdkconfig:1997 | Main task stack 3584 B holds uplink queue (320 B), packet (128), status, usb_input (64), ack, plus Ra01 locals (read_command 2x258 B = 516 B, read_buffer 128 B), SPI driver and ESP_LOG formatting. Margin unknown; overflow = canary panic = reboot (radio reset to -17 dBm, GPS reset pulse) |
| 11 | Medium | plausible | sdkconfig:2019 | No watchdog on the main loop. TWDT only watches idle tasks, panic off. A wedged main task (item 2) is never rebooted |
| 12 | Medium | plausible, needs test | host side / USB-JTAG | Opening the port may reset the ESP32-S3 via DTR/RTS line sequences (USB Serial/JTAG auto-reset). Firmware cannot prevent or detect it; each Horizon/ser2net (re)connect could reboot Aperture (about 3.5 s GPS reboot + radio re-init, radio back to -17 dBm). Horizon never sets dtr/rts (no code found); ser2net.yaml (pi stuff) uses defaults |
| 13 | Medium | confirmed (reading) | main.cpp:97-155 | Command parser: 128 B USB RX ring read 64 B per 10-20 ms. A burst above ~128 B (GPS-assist upload, 0x4744 frames at 60 ms pace is OK; a faster PC loop is not) drops bytes; one lost byte destroys that frame and the next (resync only on 0x05, no checksum). A data byte 0x05 in misaligned data followed by 0x0A 9 bytes later forwards a garbage command over LoRa. Echo `write_bytes(&byte,1,0)` consumes 8+ B ring overhead per byte and is dropped when full |
| 14 | Medium | confirmed (reading) | hardware.h:24-26 vs Horizon/Witness protocol docs | TX drive boots at -17 dBm (low-power PA, ~+9 dBm out) and max +3; docs say "boot at -9 dBm, range -9..+3". Uplink range at boot is far lower than documented; commands silently fail until the user sends 0x5250 |
| 15 | Medium | confirmed (reading) | main.cpp:121-126 | A host 0x5250 command with front-end OFF powers the FEM down (RX attenuated). Accepted any time, no flight guard. Bench-only function reachable from the PC app = self-inflicted deafness |
| 16 | Low | confirmed (reading) | main.cpp:163-176,167-169; lc86_nmea.h:60 | Race between `now_us/now_ms` taken before `gps.state()/last_pps_us()`: a GGA/PPS arriving in between gives negative/wrapped ages. PPS case: `since_pps_ms` becomes ~2^64, phase = fmod of a huge double (arbitrary). GGA case: uint32 wrap makes `gga_fresh` false, `pps_known` false, command sent immediately at any phase. Probability about 1e-4 per evaluation, one stray command collides with downlink |
| 17 | Low | confirmed | ra01.cpp:309-310, ra01.h:23 | Only RSSI of last packet, shown once per second in 0xF0, not per frame; SNR not read or reported. Poor diagnostics, no effect on delivery |
| 18 | Low | confirmed (reading) | ra01.cpp:181 | Header-error and RX-timeout IRQs not enabled, no preamble-detect or sync IRQs; no way to count missed or bad-header packets. CRC errors are logged but not counted |
| 19 | Low | plausible | ra01.cpp (no SetRxBoostedGain/CalibrateImage) | RX boosted gain (reg 0x08AC=0x96) and image calibration for 902-928 not set; ~3 dB sensitivity left on the table; same on both ends |
| 20 | Low | confirmed | main.cpp:187-190 | Failed command TX is dequeued and never retried (relies on Horizon ack/resend, which exists in command_link.py send_reliable) |
| 21 | Low | confirmed | docs | Aperture GROUND_STATION_PROTOCOL.md and FIRMWARE_HANDOFF.md are stale (see Divergences) |

## Details

### 1. Permanent deafness after a single init failure (High)
main.cpp:55-67: if `radio.initialize()` returns anything but ESP_OK, `app_main` enters `for(;;) { ESP_LOGE(...); vTaskDelay(1000); }`. No retry, no `esp_restart()`, and the 0xF0 status loop (line 88) is never reached, so the PC gets no status frames either, only text lines. initialize() has many failure points: any `wait_until_ready` 500 ms BUSY timeout (ra01.cpp:323-330, e.g. radio still booting after the 10 ms post-reset delay at line 102/104, or brown-out glitch at power-up), any SPI error. Witness's copy added a packet-RAM round-trip probe (Witness ra01.cpp:134-141) because writes succeed with an absent radio; Aperture lacks it, so a missing/unpowered radio is not even detected (initialize returns OK, receive() then returns 0 bytes forever; `read_command` returns zeros, `irq=0`, ESP_ERR_NOT_FOUND, no log at all).
Scenario: cold power-up at the pad on a marginal supply (Pi USB, brownout level 7 = 2.8 V approx, sdkconfig:1691): one failed SPI command and nothing is received all flight. Fix idea (not applied): retry init with RESET pulse every few seconds, or `esp_restart()`.

### 2. No health check / re-init anywhere (High)
- Only init ever sets SetRx (ra01.cpp:200-201). After that the radio is assumed to stay in continuous RX. Nothing checks GetStatus (chip mode), nothing re-issues SetRx periodically. If the SX1262 resets (supply dip on the 3V3 rail during TX, which is documented on the bench as a USB over-current problem), it comes up in STDBY_RC with default modem config; Aperture keeps polling GetIrqStatus (returns 0) and never notices. Because the polarity of reset recovery is not checked, the receiver stays deaf until power cycle.
- transmit() (206-272): any failure in the setup sequence (lines 213-246) returns early WITHOUT re-entering RX (Witness's copy added "do not leave the receiver in standby", Witness ra01.cpp:275-279). Example: BUSY timeout inside `configure_pa_for`, `apply_tx_modulation_erratum`, `write_buffer`, or `SetTx`: radio left in STDBY (or, if SetTx succeeded then failed later, in TX until its 1 s timeout, then STDBY). Aperture then never calls SetRx again until the next *successful* transmit (which only occurs when another command is queued). Also if SetRx at line 269 fails, error is returned but no retry.
- BUSY stuck: `receive()` -> `read_command` -> `wait_until_ready` waits 500 ms (kBusyTimeoutMs, ra01.h:40) polling every 10 ms, logs ESP_LOGE, returns ESP_ERR_TIMEOUT; main loop logs "Receive failed" and loops; each iteration 500 ms + 10 ms. Infinite, with 2 log lines per half second into the data stream. RESET (GPIO12) is never toggled outside initialize().
- main.cpp:206-210: every error except NOT_FOUND is only logged. No error counter, no escalation.

### 3. BUSY wait yields a whole tick (High; magnitude needs test)
ra01.cpp:323-330 checks BUSY once; if high it calls `vTaskDelay(1)` which at 100 Hz sleeps up to 10 ms (until the next tick), then re-checks. BUSY goes high right after NSS rises for the command processing time (tens of microseconds for read commands, longer after SetStandby/SetPacketType/SetRfFrequency/Calibrate/SetTx/SetRx), and `spi_device_transmit` returns immediately after NSS rises, so the next command's `wait_until_ready` very likely sees BUSY high. The Witness copy replaced this with a busy-spin for exactly this reason (Witness ra01.cpp:364-372, comment "rather than sleeping a whole scheduler tick per command").
Cost, assuming every command hits the yield (worst case) :
- RX of one packet: GetIrqStatus, GetRxBufferStatus, GetPacketStatus, ReadBuffer (1-3 chunks), ClearIrqStatus = 5-7 commands = 50-70 ms. During this the radio buffer is exposed to the next packet (items 8, 9).
- transmit(): SetStandby, SetPaConfig (when changed), erratum read, WriteRegister, SetTxParams, SetPacketParams, WriteBuffer, ClearIrq, SetTx, then GetIrq poll every 10 ms, ClearIrq, SetRx = about 11 commands = ~110 ms setup + ~11 ms airtime + 10-20 ms poll granularity: radio not receiving for roughly 120-150 ms per uplink command. During a 32-command flush that is about 4 s of continuous deafness (item 4).
- Each idle poll (GetIrqStatus) may also take 10 ms, which with the 10 ms loop delay gives about 20 ms between polls.

### 4. Command queue flush transmits continuously (High)
Code (main.cpp:161-192, tx_slot.h:88-96):
```
if (!pps_known) return true;                          // no GPS: always
if (sent_before && now_ms - last_sent_ms < 800) return true;   // burst
phase in [800, 950-airtime]
```
`last_uplink_ms` is set after every transmit (main.cpp:190), so as long as the next send starts within 800 ms of the previous one, the burst condition holds again. Since each loop iteration sends one frame and takes only ~0.15 s, the entire queue (up to 32) is flushed back-to-back from the first slot-start, ignoring the 800-950 ms window for all but the first command. Overruns the window by up to 32*0.15 = ~4.8 s.
Consequences:
- No GPS fix/PPS (pad without sky view, LC86 failure, PPS wire off): `aperture_may_send` returns true on every loop, every queued command goes out immediately at any phase, colliding with Witness telemetry (no listen-before-talk), and Aperture is in TX or setup for most of the time.
- A PC app that continuously re-sends (Horizon send_reliable resends up to 5 times at 1 s; upload_assist paces 60 ms and sends chunks twice) keeps the queue non-empty. Queue never drains faster than arrivals if the PC sends at >~7 frames/s. With a stuck/flooding PC, uplink TX duty approaches 100% and RX is about 0.
- Witness side (tx_slot.h:80): Witness holds telemetry for 1000 ms after each command heard (explicit design). So any command stream of N frames spanning T seconds suppresses downlink for T + 1 s, even in perfect conditions. A stuck queue repeating a command every <1 s suppresses downlink entirely.
- Queue is FIFO with no coalescing or expiry: stale commands sent after the user gave up (flood from a retry loop) still go out later.
- Queue-full policy (main.cpp:146-148): drops the NEWEST command and logs; old stale commands win.
Tx_slot.h files are byte-identical (cmp) between Aperture and Witness, and comms.h identical (cmp), so no skew there.

### 5. Blocking USB writes in the RX loop (High; needs test)
main.cpp:91-93 (status, 100 ms), 138 (ack, 50 ms), 199-205 (packet, 100 ms, loop until written or count<=0). usb_config.tx_buffer_size = 512 (line 41). The USB Serial/JTAG driver writes into a ring buffer; if the host is not draining the endpoint (port not open by any process, Linux cdc_acm not being read, ser2net without a client, Windows app stalled, hub unpowered), the ring fills after ~512 B (about 6-8 frames, or 1 s of 0xF0 + one telemetry frame) and every subsequent write blocks for the full timeout. With the host absent the loop runs at about 10 iterations/s: radio polled every 100 ms; a 65 B frame arrives every ~0.5 s at normal rates so most are still read, but two close-together frames are lost, plus the 0xF0 status writes block 100 ms each second. When the host returns, the 512 B of stale queued data is delivered first (late data). Whether the driver drops instead of blocking when no host is connected depends on the IDF version (not verifiable offline). Drops are not counted or reported.
Also, no way for the PC to know frames were dropped (no sequence number from Aperture; Witness frames carry uptime so gaps are detectable by Horizon).

### 6. USB over-current / brown-out on transmit (High; from bench memory)
Bench memory "Pi ports trip when a Ra-01SH-P transmits; needs a powered hub". Aperture's `transmit()` is the trigger: uplink TX at higher drive draws a pulse. Effect on a flight day: the USB device resets or the port power cycles; Aperture reboots or is gone for seconds, ser2net port numbers shift (memory: port order shifts), Horizon's TCP stream ends; every packet during this is lost. Brown-out detector is on (sdkconfig:1675) and resets the MCU, so a sag during a command = reboot (back to -17 dBm, GPS reset pulse, radio init). Not visible in code beyond the missing mitigation; flagged because of its flight impact.

### 7. Log text interleaved in binary stream (Medium)
ESP-IDF console is USB Serial/JTAG, the same endpoint as `usb_serial_jtag_write_bytes`. Log sources during flight: "Receive failed: ESP_ERR_INVALID_CRC (0x109)" on every radio CRC error (main.cpp:207, a noisy-channel flood), "Sending command ..." (179), "Command send failed" (183), "uplink time slots ON/OFF" (172, flips whenever fix/PPS flaps), gps diagnostics every 30 s (lc86_gps.cpp:230), fix gained/lost, ra01 BUSY timeout errors. Text goes from other tasks (gps) as separate ring-buffer items, so it usually lands between frames; a partial write loop (main.cpp:199-205) can split a frame around a log line. Horizon's PacketDecoder (witness_protocol.py:366-419) discards unknown IDs byte by byte and resyncs on bad EOF. Text bytes such as 'E','I','W' are not valid IDs, so mostly harmless, but characters 0x01-0x0A/0x0D from color codes or the line terminator (0x0A is the EOF value, 0x0D) can fake structure; a corrupted frame is dropped. Recommendation: log only before flight or route to nothing (CONFIG_LOG_DEFAULT_LEVEL to WARN or ESP_CONSOLE_NONE) because the CRC-error log is attacker-free but channel-driven.
Also the echo (main.cpp:104) puts a copy of every PC byte into the downlink stream; documented behaviour, host must filter (Horizon only filters 0x0C).

### 8. Clear-after-read race (Medium)
receive(): GetIrq (281) -> RxBufferStatus (294) -> PacketStatus (306) -> ReadBuffer (312) -> ClearIrq(0x00FF) (313). In continuous RX the radio keeps listening and can start writing the next packet into the same buffer at offset 0 (`SetBufferBaseAddress 0,0`, ra01.cpp:191). Packet N+1 finishing before 314 gets RxDone cleared (never seen: lost), or its bytes partly overwrite the buffer while ReadBuffer is mid-way (corrupt but CRC-valid-looking data forwarded). The SX126x offers no atomicity here; a safe design clears RxDone immediately after RxBufferStatus and re-checks. The window is long because of item 3 (tens of ms).
Also CRC-error path (286-290) clears only the lower byte and returns without reading status: OK functionally (RxDone and CrcErr both cleared).

### 9. Poll latency vs. back-to-back packets (Medium)
Loop = SPI poll + `vTaskDelay(1)` (main.cpp:211). Witness sends acks/replies from an immediate queue right after telemetry (sensors.cpp:1446). Packet airtime at SF7/500: 8 B heartbeat about 6 ms, 13 B ack about 11 ms, 65 B state about 31 ms. With 10-20 ms between polls and a single RX buffer, back-to-back short frames are lost. Not provable without Witness timing; plausible.

### 10-11. Stack and watchdog (Medium)
No evidence of overflow, but margin is thin by inspection (see table). Canary stack check is enabled (sdkconfig:2542) so an overflow becomes a reboot, not corruption. TWDT (5 s) is not panicking and does not subscribe the main task, so a wedged main task is never reset. Panic handler reboots immediately (REBOOT_DELAY 0): safe for recovery but every reboot costs about 3 s GPS wait plus radio init, and drops the radio drive to -17 dBm.

### 12. DTR/RTS reset on port open (Medium; needs test)
USB Serial/JTAG implements the esptool auto-reset by watching DTR/RTS sequences. Linux cdc_acm raises DTR and RTS together (no reset sequence); Windows/pyserial and RFC2217 clients (ser2net 7000-7002 `telnet(rfc2217)` in ser2net.yaml) can toggle lines separately (rfc2217 SET_CONTROL, pyserial rfc2217.py:1256-1261 sets dtr). Bench memory notes esptool reset over rfc2217 is flaky; the raw TCP ports 7100-7102 do not pass control lines but ser2net still opens the tty. Firmware has no defense (cannot ignore DTR). If a reset happens at each reconnect, every ser2net reconnect costs ~5 s (GPS reset pulse + 3 s wait + init) of lost downlink and puts the radio back at -17 dBm. No Aperture-side code (e.g. DTR-gated writes) was found; main.cpp never examines USB connection state.

### 13. Command parser details (Medium)
- Accepts any 10 bytes starting with 0x05 and ending with 0x0A (main.cpp:108-116, 144-151). No validation of status/uptime fields or opcode; Witness `command_policy.h` is the only gate. 0x6869 (erase logs) over LoRa is allowed in Pad Idle/Landed per docs, so a corrupted-alignment frame is not harmless in principle (very low probability).
- USB RX ring 128 B, driver ISR drops overflow silently.
- Ground-power command (0x5250 with target ground) is consumed locally and acked on USB; the ack write blocks up to 50 ms (138).
- `command[7]/command[8]` opcode and `IRIS_RADIO_POWER_TARGET_GROUND` are only checked when `command.back()==0x0A`: OK.

### 14-15. Power/front-end
Hardware.h:24-26 boot -17 dBm, max +3, doc says -9..+3 and boot -9. Witness boots -17 too (hardware_pins.h:85). So both radios use the low-power PA at boot, link budget for uplink about 8 dB lower than the doc implies.

### 16. Time wrap and race (Low)
- uint64 ms math in tx_slot.h is fine; uint32 `now_ms - gga_ms` (lc86_nmea.h:60) is fine across wrap but gives a huge value if `gga_ms` is slightly in the future (race above). ESP uptime wraps at 49.7 days: irrelevant.
- PPS ISR (lc86_gps.cpp:127-134) is IRAM_ATTR, uses `esp_timer_get_time` (ISR-safe) and a spinlock with 64-bit store; safe. `gpio_install_isr_service(0)` is called in gps.start() before radio init; radio does not use GPIO interrupts, so no conflict. `last_pps_us` reader uses critical section: OK.
- `pps_known` requires GGA fresh (<3 s), fix_quality>0 and PPS age <3 s. PPS edge accuracy: PPS is the UTC second edge only when the LC86 has time; pulses may exist without fix (flagged by fix_quality). A PPS present but time-unlocked is excluded by the fix test. OK.
- Phase uses `since_pps_ms` of the most recent edge up to 3 s old: with a missed edge the phase is taken modulo 1000, correct as long as PPS period is exactly 1 s.

### 17-19. Diagnostics and radio config
RSSI formula `-packet_status[0]/2` (ra01.cpp:309) is correct (RssiPkt = -x/2 dBm). The comment at 303-304 lists a wrong layout but the offsets in read_command (skip opcode+status) make byte 0 = RssiPkt, byte 1 = SnrPkt, so the code is right. SNR unused. Frequency error: SX1262 crystal tolerance (no TCXO) 10-20 ppm = up to about 18 kHz at 926 MHz on each radio, well within SF7/500 kHz tolerance (about 25% of BW = 125 kHz); Doppler at 300 m/s = ~0.9 kHz: negligible.

## Divergences (docs/code) that matter
Aperture `main/GROUND_STATION_PROTOCOL.md` vs Horizon/Witness copies (diff):
- Missing 0x06 command-ack and 0x07 transfer-status rows and sections (Aperture itself emits 0x06 for 0x5250), 0x09 landed, 0x0E, 0xE2.
- Missing 0x5250 (radio power; Aperture implements it, main.cpp:114-143), 0x5746 maintenance reboot semantic, 0x4741-0x4752 GPS-assist, 0x4844, 0x686A, and the flight-state rules (one-way states, 0x7300 behaviour, LoRa refuses 0x7301).
- Status bits 5-7 (MAINTENANCE/RESUMED/LOG SLOTS FULL) described as reserved zero; Kalman altitude (offset 57) and Position fields documented as barometric/zero.
- Horizon vs Witness copies differ in 0x5746/0x6868 text only.
- FIRMWARE_HANDOFF.md says LoRa BW 125 kHz and TX -2 dBm; code is 500 kHz (hardware.h:37) and -17 dBm.
- Docs say "both radios boot at -9 dBm"; code -17 dBm.
- Docs say GPS slots/PPS are "diagnostic only" (handoff) while PPS now drives uplink slots.
- 0xF0 length: comms.h says 29 B; Horizon payload_lengths accepts (27, 11) payloads = 29/13 B frames: consistent.
- Horizon decoder accepts IDs 0x01,02,03,FF,F0,05,E0,E1,06,07,08,09 only. Witness 0x0B(?)/0x0A heartbeat or debug/Kalman packets other than those listed would be discarded as unknown ID by Horizon with error spam; not an Aperture defect.

## Checked and OK
- LoRa settings match Witness exactly: 926.0 MHz (same frequency word formula), SF7, BW code 0x06 (500 kHz), CR 4/5, preamble 8, explicit header, CRC on, normal IQ, LDRO off, default private sync word 0x1424 on both (neither sets it), PacketType LoRa, DIO2 as RF switch, regulator default on both. Packet params in RX use length 0xFF (ignored in explicit header); transmit() changes only the length field, which does not hamper RX after SetRx.
- Largest Witness frame 103 B (WITNESS_DEBUG, Kalman log never sent) < Aperture buffer 128 B (`IRIS_RADIO_MAX_PACKET_LENGTH`), ReadBuffer chunks 60 B + 3 = 63 B <= 64 B no-DMA limit (ra01.cpp:390); write_command up to 258 B only used with <=12 B. Over-length packets are dropped cleanly (297-301).
- Continuous RX (0xFFFFFF) restarts automatically after RxDone and after CRC error; TX path always ends in SetRx on the normal path (267-271).
- TX has a 1 s radio-side timeout and a host-side deadline (kTransmitTimeoutMs): TX cannot hang forever. SetTx timeout 0x00FA00 correct.
- TxModulation erratum (bit 2 for 500 kHz) applied before every TX, same as Witness.
- tx_slot.h and comms.h are byte-identical between Aperture and Witness (cmp). lora_airtime_ms matches Semtech formula for explicit header, CRC on, CR 4/5.
- 0xF0 builder: fields match comms.h offsets via macros, static_asserts on length; unsigned subtraction wraps are safe (now_ms - fix_ms).
- GPS task: priority 2 above main (priority 1) but blocks in `uart_read_bytes`, so cannot starve the main loop; 2 KiB UART RX buffer vs ~2 KB/s traffic fine; checksum-validated NMEA parser with bounded buffers; 4 KiB task stack adequate (256 B local buffer + 160 B sentence).
- PPS ISR safe (see 16). No ESP_ERROR_CHECK on runtime paths; the only one is `usb_serial_jtag_driver_install` at boot (main.cpp:44). No heap allocation in the loop (std::array on stack); no malloc/new found in main/ (only driver-internal), so no heap growth over a flight.
- Panic = reboot (not halt), brownout detector reset, interrupt WDT 300 ms: acceptable recovery posture.
- Frequency error/Doppler tolerance: fine (item 17-19).

## Suggested priority (not applied)
1) Retry/reset radio init and add an SPI probe; add periodic health check (GetStatus chip mode every 1-2 s, re-SetRx / full re-init on mismatch; pulse RESET on repeated BUSY timeouts). 2) Replace `vTaskDelay(1)` BUSY wait with a us spin as in Witness. 3) Make burst bounded (cap total burst, require slot per N frames, never exceed 950 ms phase; rate-limit and de-duplicate queue). 4) Drop USB writes without blocking when the host is not draining (count drops); stop logging to the data port in flight. 5) Clear RxDone immediately after RxBufferStatus; read packet and re-check. 6) Powered hub for TX.
