# Aperture firmware — audit findings

## Overview

Scope: `Software/Aperture/softwareattempt2/main` (main.cpp, ra01.*, lc86_gps.*, lc86_nmea.*, ground_station_status.*, hardware.h, comms.h), read against `FIRMWARE_HANDOFF.md`, `GROUND_STATION_PROTOCOL.md` and `sdkconfig` (CONFIG_FREERTOS_HZ=100, no PSRAM, USB Serial/JTAG console). Read-only; nothing built, flashed or run.

- The 0xF0 status packet matches comms.h: offsets, lengths, EOF, big-endian fields, RSSI int8, lat/lon 1e-7, HDOP 0.01, fix age. No scale or offset error found.
- The 0x05 command frame (10 bytes, opcode at frame[7..8], EOF at [9]) and the 0x06 ack layout (13 bytes) match comms.h. Frame assembly does span USB reads correctly.
- SX1262 IRQ bit assignments, GetRxBufferStatus/GetPacketStatus/ReadBuffer offsets, SetTx timeout (1 s) and the erratum 0x0889 handling look correct.
- Biggest risks: the radio can be left in standby after a failed transmit with no recovery (#1), and the USB frame parser can lose or fabricate commands (#2).
- Power limit: the software cap of +3 dBm is enforced only at runtime in set_tx_power_dbm. The boot default and everything below -9 dBm use an SX1261-style PA config on an SX1262 (#3).
- NMEA parsing looked robust (bounded buffers, checksum enforced, field-count cap, range checks). Only a minor semantic issue was found (#13).

## Findings

### 1. Radio left in standby after a failed transmit, never re-armed — high
- Where: `ra01.cpp:212-213` (SetStandby), early returns at `ra01.cpp:214`, `218`, `220`, `226`, `234`, `237`, `241`, `246`; the only SetRx restore is `ra01.cpp:268-271`; `receive()` at `ra01.cpp:274-317` never re-arms.
- What: `transmit()` puts the SX1262 in STDBY_RC before doing anything else. If any later step fails (SetPaConfig, erratum, SetTxParams, SetPacketParams, WriteBuffer, ClearIrq, SetTx, or a BUSY timeout), it returns without the `SetRx(continuous)` at line 268. `receive()` only reads IRQ status, so it returns ESP_ERR_NOT_FOUND forever.
- Scenario: one transient BUSY timeout (500 ms) or SPI error during a command uplink makes the ground station permanently deaf to Witness until reboot. The bridge keeps running and reports no error beyond the one-time TX log, so Horizon sees silence.
- Confidence: high.

### 2. USB frame parser cannot recover from a stray 0x05; can transmit a fabricated command — high
- Where: `main.cpp:97-100` (resync only when `command_length == 0`), `main.cpp:102-105` (fixed 10-byte window), `main.cpp:143` (`command_length = 0U` after a bad EOF, without re-scanning the discarded bytes), `main.cpp:133-135` (transmit whenever the window ends in 0x0A).
- What: once a 0x05 starts a window, the next 9 bytes are consumed unconditionally. A bad-EOF window is discarded with all its bytes, including any 0x05 that begins a real frame inside it. There is no inter-byte timeout, so a partial frame stays open until the next 9 bytes arrive.
- Scenario A (lost command): host noise or a partial frame leaves a stray 0x05. The next real 10-byte command is absorbed into the bad window, so it is lost and its trailing bytes are discarded until the next 0x05. FIRMWARE_HANDOFF.md line 118 says the parser "searches for packet ID 0x05 to resynchronize", which is not what the code does.
- Scenario B (fabricated command): if a stray 0x05 is followed by 9 bytes whose last byte is 0x0A, the bridge transmits that 10-byte junk over LoRa as a canonical command. Commands have no CRC (comms.h transport CRC length is 0), and Witness acts on opcodes such as Wi-Fi on, flight state and HITL.
- Fix direction (for the author): re-scan the window after a failed EOF check (or after a 0x05 appears inside a window), and add an inter-byte timeout that resets the window.
- Confidence: high for the mechanics; the impact depends on how much non-command or junk traffic reaches the USB input.

### 3. Low-power PA path uses SX1261 PA settings and an out-of-range power on the SX1262 — medium
- Where: `ra01.cpp:434-447` (configure_pa_for: `low_pa` deviceSel=1, `high_pa` deviceSel=0), `ra01.cpp:436-439` (comment: deviceSel 1 = low-power PA), `hardware.h:24-25` (boot default -17, min -17), `ra01.h:60`.
- What: the boot default is -17 dBm, so `initialize()` selects SetPaConfig deviceSel=1 and SetTxParams -17. The firmware's own comment cites deviceSel 1 as the SX1261 low-power PA. The SX1262 datasheet specifies SetTxParams power as -9..+22 dBm and gives deviceSel 0 for the SX1262 and 1 for the SX1261. So the boot-time transmit path, and every drive from -17 to -10, is outside the chip's specified configuration. Output power and FEM input are not established by datasheet.
- Scenario: with the FEM in circuit, the actual RF output at these settings may not match the register value. The +3 dBm FEM input limit then depends on an unverified mapping. Verify on a bench analyzer before any drive above boot level.
- Confidence: medium (depends on the SX1262 datasheet version and the Ra-01SH-P FEM behaviour, which were not checked against a document in this audit).

### 4. Blocking USB writes with 100 ms timeouts stall the loop and can emit truncated frames — medium
- Where: `main.cpp:80-82` (0xF0 status, single write, no retry), `main.cpp:152-158` (forwarded LoRa payload, `break` on count <= 0), `main.cpp:164` (1 tick delay per loop).
- What: each status write and each forwarded-packet write blocks up to 100 ms when the USB TX buffer is full (no host reading, or a slow host). While blocked, the loop does not poll the radio or read USB. A partial write leaves a truncated binary frame in the USB stream; the status write never retries at all.
- Scenario: with no host attached (or Horizon paused), each received packet costs up to 100 ms and the loop misses radio IRQs during that time. A half-written 0xF0 or forwarded packet is then not recoverable by the host except by fixed-length resync.
- Confidence: medium-high (timing and partial-write behaviour are standard for the ESP-IDF USB Serial/JTAG driver; not measured).

### 5. Echo writes use zero timeout and silently drop bytes — medium
- Where: `main.cpp:92-93` (`usb_serial_jtag_write_bytes(&byte, 1U, 0)`).
- What: the protocol requires byte-for-byte USB echo. With a zero timeout, if the TX buffer (512 bytes) is full the byte is discarded and the return value ignored. Nothing retries or flags the loss.
- Scenario: a host that is slow to drain while forwarding packets and status loses echo bytes. Horizon then sees an echo mismatch and may conclude the bridge is broken or misread the next frame.
- Confidence: medium.

### 6. Radio serviced once per loop; back-to-back packets overwrite each other — medium
- Where: `main.cpp:147-150` (one `receive()` call per iteration), `main.cpp:164` (`vTaskDelay(1)`, which is 10 ms at CONFIG_FREERTOS_HZ=100 per sdkconfig line 2536), `ra01.cpp:274-317` (single-packet read, IRQ polling only, DIO1 not used), `ra01.cpp:239-240` and `288-314` (IRQ cleared after read).
- What: the SX1262 holds one received packet in its buffer, and continuous RX reuses that buffer. The loop reads at most one packet per ~10 ms iteration, and only after draining up to 64 USB bytes and possibly blocking on USB writes (#4). A second packet arriving before the first is read can overwrite it, and the IRQ flag is then cleared for only one packet.
- Scenario: Witness sending packets in quick succession (or the ground station blocked by #4 or #2's TX path) loses the earlier packet with no error reported; the RSSI reported in 0xF0 then refers to the surviving packet.
- Confidence: medium (depends on Witness packet spacing and the SX1262 buffer reuse behaviour; not measured).

### 7. ESP-IDF logs share the binary USB stream and can splice into frames; receive errors log at loop rate — medium
- Where: `sdkconfig.defaults:4` (console on USB Serial/JTAG), `lc86_gps.cpp:230-241` (30 s diagnostic log), `lc86_gps.cpp:269-273` (fix logs from the GPS task at priority 2, above the main task), `main.cpp:152-158` (binary write loop), `main.cpp:159-163` (ESP_LOGE on every non-NOT_FOUND receive, including each CRC error at `ra01.cpp:286-289`).
- What: log text goes out on the same port as the binary bridge stream. The GPS task runs at a higher priority than app_main and can preempt it inside a blocked binary write, inserting ASCII text into a 0xF0 or forwarded packet. Persistent receive failures (for example BUSY timeouts or a CRC storm) log an error on every 10 ms iteration, which adds more console traffic to the same blocked port.
- Scenario: a fix-change log lands in the middle of a 29-byte status frame. The host discards that frame or, worse, accepts a corrupted field. The protocol doc (FIRMWARE_HANDOFF.md, "USB stream caveat") puts validation on the host; the firmware could avoid it by keeping logs off this port or serialising writes.
- Confidence: medium (the splice requires preemption inside a write; likely rare but real).

### 8. High-power drive has no supply or current guard on the 600 mA 3V3 rail — medium
- Where: `main.cpp:110-115` (USB command accepts any drive from -17 to +3 dBm and RF_EN on/off with no supply check), `ra01.cpp:440-446` (no current limiting), `hardware.h:26` (max +3 dBm), `ra01.h:25-29` (comment).
- What: the firmware has no measure of 3V3 headroom, no limit tied to supply, and no brownout handling. Raising the FEM drive increases peak current from the Feather's regulator, which also feeds the ESP32-S3 and the GPS module. FIRMWARE_HANDOFF.md (troubleshooting) already records USB disconnects from FEM transients.
- Scenario: a +3 dBm command issued while the GPS is acquiring and the MCU is active browns out the rail; the bridge resets and drops USB. Nothing in software records the cause.
- Confidence: low-medium (current draw not measured; the regulator rating is taken from the brief).

### 9. Radio init failure halts the whole bridge, including GPS status — medium
- Where: `main.cpp:54-66` (on `radio.initialize()` failure, an infinite loop that never enters the bridge loop at lines 76-165).
- What: when the radio fails to initialise, no USB echo, no command ack, no 0xF0 status and no forwarding run. The GPS task still runs, but its position is never published to the host, even though 0xF0 is the only way Horizon gets the pad position.
- Scenario: a transient SX1262 init failure (BUSY stuck at boot) leaves the pad position unavailable, and Horizon sees a silent device.
- Confidence: high (behaviour is explicit in the code).

### 10. transmit() drops a received packet that is pending in the radio — low
- Where: `ra01.cpp:212-213` (SetStandby discards RX in progress), `ra01.cpp:239-240` (ClearIrqStatus 0xFFFF clears any RxDone that was set but not yet read).
- What: a Witness packet that completed just before a USB command is transmitted is cleared without being read or forwarded.
- Scenario: a command that arrives in the same loop iteration as a packet loses that packet silently. Half-duplex operation makes this expected, but the loss is not reported.
- Confidence: high (mechanics); impact low.

### 11. Compile-time power limit in ra01.cpp is weaker than the hardware limit — low
- Where: `ra01.cpp:39-40` (`static_assert(... >= -17 && ... <= 22, "... between -17 and +22 dBm")`), versus `hardware.h:26-28` (max +3 dBm).
- What: the driver's assertion allows +22 dBm, which the FEM does not tolerate. The +3 dBm limit actually holds only because `hardware.h` asserts it and `set_tx_power_dbm` (`ra01.cpp:421-426`) checks against `IRIS_RADIO_TX_POWER_MAX_DBM`. A future edit to the driver assertion or a new setter would not be caught.
- Confidence: high.

### 12. transmit() accepts up to 255 bytes, but non-DMA SPI cannot carry more than 64 — low
- Where: `ra01.cpp:208` (length up to 255 accepted), `ra01.cpp:350-363` (write_buffer, 257-byte buffer, single `spi_device_transmit`), `ra01.cpp:69-71`/`ra01.cpp:116` (`SPI_DMA_DISABLED`), and the 64-byte limit noted in `ra01.cpp:388-389`.
- What: with DMA disabled, a single transfer longer than 64 bytes fails. `transmit()` for a payload over ~61 bytes returns an error from the SPI driver after the radio was already put into standby (see #1).
- Scenario: latent today, because the bridge only sends 10-byte command frames. It becomes a live bug if anyone reuses `transmit()` for larger packets.
- Confidence: medium (IDF non-DMA limit as used in the code's own comment; not tested).

### 13. Checksummed GGA with bad fields refreshes quality while the position stays old — low
- Where: `lc86_nmea.cpp:133-140` (state reset and `gga_ms` update before field validation), `lc86_nmea.cpp:153-154` (early return on bad quality or missing "M"), `ground_station_status.cpp:43-47` (quality and satellites taken from the latest GGA).
- What: a valid-checksum GGA whose position fields are unparseable still sets `gga` and `gga_ms`, and sets fix quality and satellites in the status. The position and `fix_ms` remain from an older fix.
- Scenario: 0xF0 can report quality 1 with a satellite count alongside a position that is seconds old, while fix age reads from the old fix. Horizon cannot tell the two apart from one packet.
- Confidence: medium.

### 14. GPS configured once at boot; no reconfiguration after a receiver reset — low
- Where: `lc86_gps.cpp:244-257` (reset and `configure()` only at task start), `lc86_gps.cpp:184-225` (volatile settings, RAM only).
- What: the LC86 keeps its PAIR settings only in RAM. If the module resets mid-operation (for example on the brownout in #8), it reverts to its default NMEA set and rate. The firmware never notices. At the default 1 Hz GGA the 3 s freshness window in `lc86_nmea.h:16` barely holds, so quality and satellites in 0xF0 may flicker to 0.
- Confidence: medium.

### 15. Documentation drift in FIRMWARE_HANDOFF.md — low
- Where: `FIRMWARE_HANDOFF.md:63` (Bandwidth 125 kHz), `:69` (SX1262 TX power -2 dBm), `:118` (parser resynchronizes on 0x05).
- What: code uses 500 kHz (`hardware.h:37`), boot drive -17 dBm (`hardware.h:24`), and does not re-scan windows (#2). A host implementer working from the handoff would configure the wrong bandwidth and assume a resync that does not happen.
- Confidence: high.

## Notes not raised as findings

- The 32-bit tick wrap in `main.cpp:77` and `ra01.cpp:248-265` only matters after about 497 days at 100 Hz.
- PPS ISR and the GPS spinlock usage (`lc86_gps.cpp:127-134`) look correct.
- Stack sizes (app_main 3584 B, GPS task 4 KiB) look adequate for the buffers in use.
