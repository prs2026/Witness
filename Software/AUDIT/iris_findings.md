# Iris firmware — audit findings

## Overview
- Scope: `Software/Iris/softwarev1IRIS/main` (main.cpp, usb_serial_echo.*, can_link.*, twai_driver.*, mcp23008.*, pac1931.*, battery_voltage.*, hardware.h, comms.h), plus `Software/Intercomm/can_transport.h`, `sdkconfig` and `FIRMWARE_AGENT_GUIDE.md`.
- Read-only review. Nothing was built, flashed or run; ESP-IDF library behaviour is taken from documented API use and should be confirmed against the IDF version in use.
- `comms.h` is byte-identical across the Iris, Witness, Horizon and Aperture copies.
- CAN reassembly (`intercomm::Reassembler`) and `CanLink` were checked for length, index, sequence, total and timeout handling. No buffer overrun was found in the CAN, TWAI, USB or UART parsers.
- Field offsets for packets 0x03, 0xE1, 0x05 (command), 0xFF (heartbeat) and 0x06 match `comms.h` and the compile-time asserts.
- Biggest concerns: (1) the MCP23008 output-latch cache can skip a required write, leaving a load in the wrong state; (2) current telemetry saturates at 1.00 A while the wire format advertises 5.10 A; (3) any Iris reset switches every load off and nothing restores it.
- There is no fail-safe on command silence, and load commands are not gated by sender or flight state.
- UART1 and USB command frames carry no integrity check, and Iris never acknowledges a command.
- Findings are ordered by severity within the Findings section.

## Findings

### 1. MCP23008 latch cache can skip a required write after a failed I2C transaction — high
- Where: `Software/Iris/softwarev1IRIS/main/mcp23008.cpp:124-126` (early return when `new_latch == output_latch_`), `mcp23008.cpp:128-131` (cache updated only on `ESP_OK`), `usb_serial_echo.cpp:366-406` (`set_expander_output` sets the status flag from the return value, which is `ESP_OK` on the skipped path).
- What: `write_pin` compares against the cached latch and returns `ESP_OK` without touching the bus when they match. The cache is updated only when the I2C write reports success. If a write times out after the expander has already latched it, the cache keeps the old value. Example: OUT1 cached off, `0x31` (ON) times out but the chip latched 1. Later `0x30` (OFF) computes the same value as the cache, skips the I2C write, returns `ESP_OK`, clears `IRIS_STATUS_OUT1_ENABLED_FLAG`, and the load stays on. The reverse also happens: a timed-out OFF leaves the load off while the status says ON. Status and load disagree and no error is reported.
- Scenario: intermittent I2C noise on the shared I2C0 bus (the PAC193x shares it) during a camera-off command.
- Confidence: medium (depends on ESP-IDF reporting a timeout for a transaction the slave actually completed; the code path itself is certain).
- Suggested check: always write OLAT on command, or mark the cache invalid on any failure and resync before the next compare.

### 2. Current telemetry saturates at 1.00 A while the wire format advertises 5.10 A — high
- Where: `Software/Iris/softwarev1IRIS/main/pac1931.cpp:74-88` (`raw_to_microamps`), `Software/Iris/softwarev1IRIS/main/hardware.h:71-74` (shunt and divisor 5), wire limit at `Software/Iris/softwarev1IRIS/main/comms.h:279-281` and `FIRMWARE_AGENT_GUIDE.md` section 6 "Current wire encoding".
- What: the conversion uses a 100 mV full-scale VSENSE over a 20 mOhm shunt, then divides by the empirical factor 5. At raw 65535 the result is about 1,000,000 uA (1.00 A). The camera field then tops out at `0x32` (50 counts). Anything above 1 A is clipped to 1.00 A, and the PAC1931 reports no saturation flag, so the clip is silent.
- Scenario: a camera drawing 1.5 A peak reports 1.00 A. Any over-current or power-budget conclusion from `0x03` is wrong, and the divisor is only correct if the true full scale is 1 A.
- Confidence: high on the arithmetic and the clip; medium on the physical interpretation (the 5x fudge is documented as empirical and the VSENSE range is assumed, not read back).
- Suggested check: calibrate with a known load above 1 A, read back the PAC1931 range configuration, and add a near-full-scale flag (for example raw >= 0xFFF0).

### 3. Any Iris reset switches every load off and nothing restores the state — high
- Where: `Software/Iris/softwarev1IRIS/main/mcp23008.cpp:29-33` (RESET pulse on every `initialize()`), `mcp23008.cpp:9-33` (OLAT and IODIR assumed to be power-on defaults), `main/main.cpp:22-32` (outputs forced low), `sdkconfig:1155` and `sdkconfig:2199-2201` (brownout reset at level 7), `sdkconfig:1327-1333` (task watchdog, 5 s).
- What: every boot pulses the MCP23008 reset, which clears OLAT and makes all ports inputs. `configure_output(..., false)` then forces OUT1, OUT2, OUT3 and 5V_EN off. Any ESP-only reset (brownout, panic, task-WDT starvation of the idle task) therefore drops camera and load power. Nothing re-asserts the pre-reset state, and Witness in this tree only forwards commands (`Software/Witness/softwarev1/main/command_bridge.cpp`), so no periodic re-send was found.
- Scenario: a brownout or panic during flight turns the camera off until an operator or Witness re-sends `0x31`/`0x33`/`0x35`.
- Confidence: high on the code path; severity depends on whether the camera must survive an Iris reset.
- Suggested check: skip the expander reset when the expander is already configured (or keep the last commanded state in RTC memory and re-apply it at boot), and decide the intended power-up state for each channel explicitly.

### 4. No fail-safe on command silence; load commands are not gated by sender or flight state — medium
- Where: `Software/Iris/softwarev1IRIS/main/usb_serial_echo.cpp:519-538` (`poll_can`, the only CAN consumer), `usb_serial_echo.cpp:582-647` (`execute_binary_command`, no state check), `usb_serial_echo.cpp:679-681` (heartbeat freshness used for telemetry only).
- What: when Witness or CAN goes silent, the three loads and 5V_EN keep their last state indefinitely. The heartbeat is used only to fill the `0x03` FC fields. Any valid 0x05 frame on CAN 0x120 switches loads in any flight state, from any node on the bus, with no sequence, sender or authentication check.
- Scenario: a stray or replayed command during descent switches the camera off, or a Witness reset leaves a load on indefinitely.
- Confidence: high (absence of logic). Whether a fail-safe is required is a design decision, not a code defect.
- Suggested check: define loss-of-link behaviour per channel (hold, or drop after N seconds without a Witness heartbeat) and make flight-state gating explicit in Iris or in the Witness bridge.

### 5. UART1 and USB command frames have no integrity check — medium
- Where: `Software/Iris/softwarev1IRIS/main/usb_serial_echo.cpp:564-580` (`process_binary_command_packet`: only packet ID and EOF are checked), `usb_serial_echo.cpp:540-562` (`consume_uart1_bytes`), `usb_serial_echo.cpp:243-291` (USB path). `Software/Iris/softwarev1IRIS/main/comms.h:10-13` (integrity pushed to the transport).
- What: UART1 has no framing integrity at all, and USB relies only on USB's own link CRC. The 0x05 command's status and uptime bytes (frame offsets 1-6) are ignored and could carry a check or sequence number. A single bit error in the opcode (bytes 7-8) can change `0x0030` (OUT1 OFF) into `0x0031` (OUT1 ON) and still pass the EOF test.
- Scenario: noise on the UART1 line (hardware wiring to the radio or a peer) toggles the wrong load.
- Confidence: medium (depends on how reliable the UART1 link is in practice; the code gap is certain).
- Suggested check: add a CRC or sequence in the unused bytes for UART1 and any non-USB transport, or require a two-frame confirm for power-switch opcodes.

### 6. Command parser has no resynchronisation and Iris never acknowledges commands — medium
- Where: `Software/Iris/softwarev1IRIS/main/usb_serial_echo.cpp:247-266` (USB binary window swallows exactly 10 bytes once started), `usb_serial_echo.cpp:546-560` (UART1 equivalent), `Software/Iris/softwarev1IRIS/main/comms.h:655-663` (0x06 acks are defined as Witness-to-host only).
- What: after a dropped or inserted byte, the 10-byte window can end on the wrong byte. The EOF check then fails and the next valid command is swallowed with no error. Iris sends no per-command result, so the sender learns the outcome only from the `0x03` status flags, about one second later. Commands from Witness over CAN are not acknowledged by Iris either (`FIRMWARE_AGENT_GUIDE.md` section 7, "Unknown binary commands have no effect").
- Scenario: a truncated USB frame from a host loses the following OUT2 OFF command; the host believes it was sent.
- Confidence: high on the code; impact depends on how the host retries.
- Suggested check: on a bad EOF, scan forward for the next 0x05 instead of discarding a full window, and send a short result frame for power-switch commands.

### 7. Partial USB writes leave truncated telemetry frames in the stream — low
- Where: `Software/Iris/softwarev1IRIS/main/usb_serial_echo.cpp:792-807` (`write_usb_packet` returns mid-frame on a zero-byte write), `usb_serial_echo.cpp:225-241` (`echo_bytes` truncates the same way).
- What: when the host stops reading, a 0x03 or 0xE1 frame can be cut short after part of its bytes have gone out. The next frame then starts in the middle of the previous one. The protocol has no sync marker beyond the packet ID and fixed length, so the host can misparse until it realigns.
- Scenario: a laptop sleeps during a flight log session; telemetry stays garbled after it wakes.
- Confidence: high on the code path; impact depends on the host parser.
- Suggested check: drop the whole frame on any partial write, or give the host an explicit resync rule.

### 8. Battery divider reading may saturate above about 9.8 V pack voltage — low
- Where: `Software/Iris/softwarev1IRIS/main/battery_voltage.h:23` (`ADC_ATTEN_DB_12`), `battery_voltage.cpp:103-111` (4:1 scaling), `Software/Iris/softwarev1IRIS/main/hardware.h:40-41` (15 kOhm / 5 kOhm).
- What: the 4:1 divider math is correct. The ADC pin is limited to the 12 dB range, roughly 2.45 V on the C3 (verify against the IDF range). Packs above about 9.8 V (for example 3S at 12.6 V) read as a flat maximum, with no flag.
- Scenario: a 3S pack reads 9.8 V while the battery is still well above the low-voltage threshold.
- Confidence: low (pack chemistry not in the tree; ADC range taken from memory of the C3 datasheet).
- Suggested check: confirm the pack size and the C3 ADC range; add a saturation flag if needed.

### 9. GPIO9 is a boot strapping pin shared with the MCP23008 reset — low
- Where: `Software/Iris/softwarev1IRIS/main/hardware.h:21` (`IRIS_PIN_LOAD_SWITCH_ENABLE GPIO_NUM_9`), `hardware.h:65` (`IRIS_PIN_MCP23008_RESET` aliases it).
- What: on the ESP32-C3, GPIO9 is sampled as a boot strap. If the LS_EN net is pulled low at power-up (for example to keep load switches off), the chip can enter download mode instead of booting the application.
- Scenario: a power cycle with the LS_EN net low leaves the board in the ROM downloader and the loads unpowered.
- Confidence: low (needs the schematic to confirm the pull direction).
- Suggested check: verify the LS_EN pull at power-up on the board.

### 10. Output pins float between expander reset and configuration — low
- Where: `Software/Iris/softwarev1IRIS/main/mcp23008.cpp:29-33` (reset makes all ports inputs), `main/main.cpp:22-32` (configuration happens after `initialize()`), `hardware.h:65` (GPIO9, which is also the expander reset, is driven high before the outputs are set low).
- What: between reset and the `configure_output` writes, the enable pins are high-impedance. Any external pull-up would briefly enable a load on every boot. The driver's OLAT-before-IODIR order (`mcp23008.cpp:90-99`) protects only the output-direction step.
- Scenario: a short load pulse at each boot, which could affect the camera or a sensor.
- Confidence: low (depends on the external pull network, not in the firmware).
- Suggested check: confirm the enable pulls, or configure the expander before the LS_EN net is asserted.

### 11. Main loop has no watchdog of its own — low
- Where: `Software/Iris/softwarev1IRIS/main/main.cpp:50-65` (`app_main` loop; no `esp_task_wdt_add` call anywhere in the tree), `sdkconfig:1327-1335` (task WDT watches only the idle task on CPU0).
- What: a stall in the main loop that still yields with `vTaskDelay(1)` never starves the idle task, so the task WDT does not fire. A blocking call that stops yielding (for example `uart_write_bytes` with no timeout, `usb_serial_echo.cpp:819-822`) is bounded by the UART baud rate, so the practical risk is small, but a hang leaves loads and CAN in their last state with no reset.
- Confidence: medium.
- Suggested check: subscribe the main task to the TWDT and feed it from the loop.

### 12. Bus-off and drop counters are never reported; stale telemetry floods after backpressure — low
- Where: `Software/Iris/softwarev1IRIS/main/can_link.cpp:46-48` (poll returns on transmit failure and leaves the head packet queued), `main/twai_driver.cpp:178-190` (`dropped_receive_frames`, `failed_transmit_frames`; no callers in the tree), `can_link.cpp:14-26` (queue full drops the new packet silently).
- What: CAN faults (bus-off, dropped RX frames, failed TX) are counted but never sent to Witness, so the operator cannot see a dead CAN link. After recovery, older camera or debug packets are sent first, which delays current telemetry.
- Confidence: high on the code path.
- Suggested check: add the counters to the `0x03` or `0xE1` status flags or to a dedicated field.

### 13. No bus recovery on the shared I2C0 bus — low
- Where: `Software/Iris/softwarev1IRIS/main/mcp23008.cpp:135-143` (`write_register`, 20 ms timeout, no recovery), `main/pac1931.cpp:26-61` (`read_currents_microamps`, shared bus), `main/mcp23008.cpp:9-60` (bus created with internal pull-ups disabled).
- What: a slave holding SDA low after an interrupted transaction would make every later MCP write fail. Output changes then return errors and are not applied, while the telemetry path keeps running. The firmware has no bus reset or retry.
- Confidence: low (depends on the hardware fault mode).
- Suggested check: add an I2C bus-clear sequence after repeated timeouts and report the failure in the status flags.

## Checked and found sound
- `Intercomm/can_transport.h` `Reassembler::accept`: rejects wrong identifier, extended or remote frames, bad DLC, inconsistent totals and out-of-order fragments; 250 ms timeout; packet buffer is 103 bytes and the write offsets are bounded by `total`.
- `can_link.cpp` queue and fragmentation: ring indexing and `fragment_` advance are correct; no stall path found.
- `twai_driver.cpp` receive path: slot ownership between ISR and task is consistent under `portMUX` locking.
- `usb_serial_echo.cpp` ASCII normalization and the `S` command parser: buffer bounds hold; overflow lines are discarded.
- Telemetry packets 0x03 and 0xE1: offsets, uptime encoding, and 20 mA per count encoding match `comms.h`.
