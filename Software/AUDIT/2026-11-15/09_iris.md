# Audit 09: Iris firmware (cameras must stay on in flight)

Read-only audit, nothing run on hardware. Paths: I = Software/Iris/softwarev1IRIS, W = Software/Witness/softwarev1.
"plausible" = needs bench test.

## Findings

| # | Sev | Conf | Finding | Where |
|---|-----|------|---------|-------|
| 1 | High | confirmed (reading) | Iris cold reset (power-on, EXT, USB, UNKNOWN, corrupt RTC record, e.g. battery sag) leaves all loads OFF. Only Witness's volatile load tracker can restore them. If Witness also reset in the same event (shared battery sag), nothing ever re-sends and the cameras stay off for the rest of the flight. | I/main/load_state.cpp:226-229, load_state_logic.h:346-352; W/main/iris_load_tracker.h:66 (known_==0 returns no action); W/main/can_link.cpp (no load command at boot) |
| 2 | High | confirmed (design, documented) | Any Iris warm reset (panic, WDT, brownout) drops all loads for ROM + bootloader + app start (hundreds of ms): GPIO9 is released, R55 pulls expander RESET low, outputs become inputs, EN pulldowns switch off. Cameras power-cycle. Iris has no record-trigger logic, so resuming recording depends on the camera. Restore code cannot shorten this gap. | I/main/main.cpp:58-62, mcp23008.cpp:17-20, FIRMWARE_AGENT_GUIDE.md "Reset recovery" |
| 3 | High | plausible, needs test | Main task stack is 3584 B. `UsbSerialEcho` (CanLink 8x112 B tx queue, TwaiDriver 8 tx + 16 rx slots, 256 B command buffer, Reassembler) is a local in app_main, and `poll()` adds a 512 B buffer plus nested `poll_uart1()` another 512 B. Rough sum 3.3-3.9 KB. Overflow causes canary panic or silent corruption then reboot (finding 2), possibly a loop. Measure `uxTaskGetStackHighWaterMark` in the running image. | I/sdkconfig CONFIG_ESP_MAIN_TASK_STACK_SIZE=3584; main.cpp:91; usb_serial_echo.h:276-313; usb_serial_echo.cpp:204,554; twai_driver.h:83-84; can_link.h:459 |
| 4 | Medium | confirmed | No flight-state gating of load OFF anywhere. Iris does not know the flight state. Witness forwards any radio/USB load-off to Iris unconditionally and acks OK; command_policy.h has no rule for 0x30/0x32/0x34/0x50. One LoRa frame (CRC16 only, no authentication, replayable by anyone on frequency) or a ground-station mis-click turns a camera off in Boost/Coast/Descent, and the tracker then enforces that OFF. | W/main/command_bridge.cpp:65-93; W/main/usb_serial_echo.cpp radio switch (CH*_LOAD_* acks OK); W/main/command_policy.h; I/main/usb_serial_echo.cpp:629-694 |
| 5 | Medium | confirmed | A load ON command can be lost while the ground is told OK. The radio switch acks IRIS_ACK_OK "already retransmitted" even if `forward_command` failed (commands_ queue of 8 full, CAN task not started). The tracker only learns a command at dequeue (`note_command`), so a command dropped before the queue is never re-sent. A 200 ms pending-deadline expiry after note_command does self-heal (mismatch resend after 0.9 s + 1.5 s grace). | W/main/can_link.cpp:42-54, 76-83, 203-207; W/main/command_bridge.cpp:79-92; W/main/usb_serial_echo.cpp radio load cases |
| 6 | Medium | plausible | Expander-only reset (3V3 dip, I2C glitch, GPIO9 glitch) turns IODIR back to inputs and the loads off until the 1 s resync. No I2C bus recovery: with SDA stuck every transaction costs a 20 ms timeout and resync never succeeds. | I/main/usb_serial_echo.cpp:234-247; mcp23008.cpp:144-154, 182-192; no SCL-clock bus clear anywhere |
| 7 | Medium | confirmed | Cold-class resets include ESP_RST_EXT, USB, UNKNOWN and any new reason: loads forced off. An EN-pin noise glitch or USB-JTAG reset becomes a camera outage until Witness sees the next 1 Hz camera packet (first one 1 s after boot) and re-sends: about 1.5-2.5 s. | I/main/load_state.cpp:215-230; main.cpp:103-115 |
| 8 | Medium | plausible | Iris TWAI driver never frees tx slots on bus-off (Witness driver clears them in suspend(); Iris has no equivalent). If frames dropped by bus-off produce no on_tx_done callback, all 8 slots stay in_use, `transmit` returns ESP_ERR_TIMEOUT forever and Iris telemetry/pong never reach Witness. Loads unaffected, but Witness goes blind (no reboot detection or mismatch resend). | I/main/twai_driver.cpp:85-100, 202-209, 273-281; can_link.cpp:400-402 |
| 9 | Low | confirmed | Parsers accept any 10-byte frame starting 0x05 ending 0x0A: no CRC, no resync after a bad EOF (10 bytes swallowed, can eat a real frame start), no sequence/timestamp check (replayed or stale frames act). Applies to USB, UART1 (GPIO6/7 line noise) and ASCII "S n 0". Corruption must hit a valid opcode, so very unlikely. | I/main/usb_serial_echo.cpp:284-304, 591-607, 611-627 |
| 10 | Low | confirmed | Load commands while the expander is not configured (boot bring-up failed) are rejected with INVALID_STATE and not stored; main-loop retry restores only the boot mask. Heals via Witness mismatch resend (2 s) only if Witness saw the command. | I/main/usb_serial_echo.cpp:410-417; main.cpp:117-122 |
| 11 | Low | plausible | USB with no host: `usb_serial_jtag_write_bytes` blocks up to 1 tick (10 ms) per write once the 512 B TX buffer is full (after ~25 s at 1 Hz; every write if the 10 Hz debug stream is on). Main loop slows to ~10-30 ms per iteration, CAN RX queue is 16 frames. Not a hang. Panic print to the USB console with no host may delay the reboot (needs test). | I/main/usb_serial_echo.cpp:259-278, 839-857; sdkconfig PANIC_PRINT_REBOOT |
| 12 | Low | confirmed | A Witness reboot (including the new 0x7300-in-Landed restart) wipes the load tracker, so a later Iris reboot is not auto-restored by Witness (Iris RTC restore only). Witness sends no load command at its own boot. Not a camera-off cause by itself. | W/main/iris_load_tracker.h; W/main/command_policy.h (is_reset_to_pad) |

## Every code path that can switch a load off

| Path | Condition | Gate | Notes |
|---|---|---|---|
| Iris binary cmd 0x30/0x32/0x34/0x50 | CAN (id 0x120), USB or UART1 frame, ID 0x05, EOF 0x0A | none | usb_serial_echo.cpp:648-682 |
| Iris ASCII "S 1/2/3/5 0" | USB text | none | :343-401 |
| Witness LoRa load-off | any CRC-valid LoRa command frame | none, no flight-state rule | command_bridge.cpp:65 |
| Witness USB load-off (binary, "05 30 ..", TX) | USB | none | usb_serial_echo.cpp:366,645,767,1145 |
| Witness tracker re-send of commanded OFF | Iris uptime backwards or flag mismatch > 0.9 s | enforces last forwarded command | iris_load_tracker.h:59-91 |
| Iris cold reset | POWERON/EXT/USB/UNKNOWN/invalid RTC | all off | load_state.cpp:244-247 |
| Iris warm reset window | panic/WDT/brownout/SW/lockup, until restore in app_main | R55 pulldown, expander reset | main.cpp:58 |
| Expander reset/glitch | LS_EN glitch, 3V3 dip | IODIR inputs until resync (<= 1 s) | usb_serial_echo.cpp:234 |

Absent (checked): heartbeat/ACK timeout failsafe, over-current limit, under-voltage cutoff, thermal, bus-off action, PAC1931 error action, command timeout. PAC/ADC errors only set status flags.

## Details

1. Boot defaults. Cold: `initialize(pulse_reset=true)` resets the expander (all inputs, EN pulldowns, off), then OLAT=0 before IODIR (mcp23008.cpp:125-130), so no wrong-level pulse. Warm with valid record: GPIO9 level is latched high before the pad driver is enabled (mcp23008.cpp:19-30), OLAT written first, then IODIR. Retried 25x, then main-loop retry every 250 ms; never aborts. The reset window itself is the unavoidable gap (finding 2). GPIO9 strapping is identical at cold boot, so warm resets boot the same way.
2. The RTC_NOINIT record has magic + CRC and is updated only in `set_expander_output` (usb_serial_echo.cpp:446-450). Brownout reset normally keeps RTC memory; if corrupted the CRC fails and loads go off (finding 1/7 path).
3. Panic: PANIC_PRINT_REBOOT, delay 0 (reboot, not halt). INT WDT 300 ms; task WDT 5 s without panic (log only). No ESP_ERROR_CHECK/abort/assert on any Iris runtime path (grep). No custom tasks, so only the main stack matters. FIRMWARE_AGENT_GUIDE.md's statement that MCP init failure is fatal via ESP_ERROR_CHECK is stale.
4. CAN. Iris applies no timeout on Witness traffic. With Witness paused (Landed) a lone Iris node goes error-passive (TEC stops rising on ACK errors), fail_retry_cnt=3 frees slots, bus-off not expected. The 8-packet CanLink queue fills with stale telemetry and is delivered after resume (harmless: tracker re-sends the commanded state, 1.5 s grace). Bus-off recovery runs from the main loop (twai_driver.cpp:141-166). "Hours without ACKs": no path switches loads; residual risks are findings 3, 8, 11 (long-uptime behaviour untested).
5. Camera record trigger: Iris only switches power. Nothing encodes auto-record on power-up, so every blip (findings 2, 6, 7) relies on the camera restarting recording itself.
6. Current/voltage: PAC1931 failure sets current_sample_error_ and a flag only; ADC failure sets the battery flag only; no thresholds, so inrush or sag cannot trip anything in firmware (positive). PAC read busy-waits 1.1 ms per 100 ms.

## Checked and OK

- No failsafe or timeout turns loads off; heartbeat only fills FC fields (3 s freshness).
- Loads change only on explicit command or reset.
- Expander cache logic: write_pin adopts the intended latch even if I2C fails; resync heals within 1 s; direction guard stops writes to inputs.
- Boot ordering OLAT-before-IODIR; reset pulse only on cold boot.
- Expander/ADC/USB/PAC/CAN init failures are non-fatal.
- CanLink fragment cursor retained across tx congestion; Reassembler validates ID, DLC, sequence, index, 250 ms timeout; RX ISR cannot overrun (drops counted).
- Witness: resends go before newer user commands and use the current commanded snapshot, so a stale OFF cannot overwrite a newer ON; Witness boot and Landed pause send no load commands; 0x7300 reboot does not touch Iris; LoRa CRC enabled (ra01.cpp:182).
- Reboot detection uses monotone Iris uptime; no false trigger found short of 49-day wrap.
