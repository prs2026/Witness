# Audit 03: LoRa telemetry downlink and command uplink

Scope: `ra01.cpp/.h`, radio parts of `sensors.cpp`, `sensors_landed.cpp`, `landed_*.h`, `tx_slot.h`, `command_bridge.*`, `command_policy.h`, `usb_serial_echo.cpp` (radio commands), `spi_data_forwarder.*` (queues), `comms.h`, `witness_status.h`; Aperture `main.cpp`, `ra01.cpp`, `tx_slot.h`, `comms.h`.
Working tree audited as found (read-only). `command_policy.h`, `usb_serial_echo.cpp`, `sensors.cpp` are in the "being edited" set: F4/F5/F7 sit in code that may change.
All paths are under `C:\Users\E\Documents\GitHub\Witness\Software\Witness\softwarev1\main\` unless noted. Aperture is `...\Software\Aperture\softwareattempt2\main\`.

## Findings table

| ID | Sev | Conf | Where | One line |
|---|---|---|---|---|
| F1 | Critical (low likelihood) | confirmed by reading | `ra01.cpp:364-385`, `sensors.cpp:2498-2501`, `ra01.cpp:290-330` | A radio whose BUSY pin sticks high blocks the 500 Hz sensor/estimator/state-machine task for 0.5 s per radio call (up to 1.5 s per failed TX), forever. No fault latch. |
| F2 | Critical (plausible) | plausible, needs test | `ra01.cpp:473-486`, `ra01.cpp:234-288`, `sensors.cpp:1436-1517` | No radio health check or re-init. After a radio-only reset (supply glitch) the cached PA config, packet type, modulation, frequency are lost but never re-applied; TX "succeeds" on the wrong settings, RX is dead, for the rest of the flight. |
| F3 | High | confirmed | `sensors.cpp:247-258` | Radio init failure at boot (also transient BUSY or probe failure) = radio disabled until reboot, no retry, no status. |
| F4 | High | confirmed | `sensors.cpp:1419-1423`, `tx_slot.h:80`, `spi_data_forwarder.cpp:145` | Uplink hold-off is armed by ANY received frame whose first byte is 0x05 (even 3 bytes, bad EOF, wrong length, other transmitter), in every flight state, with no cap. One frame per second mutes all scheduled telemetry indefinitely. |
| F5 | High | confirmed | `usb_serial_echo.cpp:945-974`, `command_policy.h:52-55`, `ra01.cpp:467-471` | Unauthenticated radio command 0x5250 can switch the RF front end OFF (TX and RX) or cut drive to -17 dBm. Refused only in Boost; accepted in Pad Idle (persists into flight), Coast, Descent. FEM is switched off before the ack is queued, so the ack goes out unpowered. Only a reset restores it. |
| F6 | High | confirmed | `hardware_pins.h:85`, `ra01.h:78`, `flight_state_store.*` | TX drive (RAM only) and FEM state are not persisted. Any warm reset in flight (WDT, brownout, panic, which the firmware is built to resume from) returns the downlink to the -17 dBm bench default (about 20 dB below flight power). A cold reset also drops to Pad Idle 3 s rate (earlier H3, still present). |
| F7 | High | confirmed (already documented as H2/M3, partly) | `usb_serial_echo.cpp:582-590, 555-558`, `command_policy.h:35-41` | Any LoRa frame on the channel (no auth, address or sequence) can, on the pad: erase/clear all flight logs (0x6869/0x686A), reboot into the maintenance image (0x5746) which has no flight sensing, or (Landed) restart (0x7300). |
| F8 | Medium | confirmed | `ra01.cpp:297-300`, `sensors.cpp:1368` | If DIO1 is open/stuck low: every TX takes the full 1.1 s deadline (about 1 packet/1.1 s vs 3 scheduled plus acks) and RX is never polled (no command can be received), silently. |
| F9 | Medium | confirmed | `sensors.cpp:1473-1476`, `spi_data_forwarder.cpp:98-120` | Cached radio packets carry a `generation` counter that is never compared. A stale cached 0x01/0x03 (IMU stalled, Iris/CAN dead) is retransmitted at 1 Hz as if fresh. |
| F10 | Medium | confirmed | `ra01.cpp:234-266`, `sensors.cpp:1507-1514, 1519-1529` | TX failure handling is log-and-count only. `radio_tx_failed_` is never acted on; early failures inside `start_transmit` leave the radio in standby (deaf) until the next successful TX; no consecutive-failure escalation to re-init. |
| F11 | Medium | confirmed | `landed_logic.h:19`, `usb_serial_echo.cpp:625-630`, `sensors.cpp:1465-1467` | With low-power Landed disabled, a landed Witness keeps TX at 1 Hz x3 types forever (no landed beacon rate reduction, ping acked UNKNOWN, 0x09 never sent). Battery drain and no recovery ping. Documented, owner decision. |
| F12 | Low | plausible | `sensors.cpp:1362-1370` (receive), `nmea_gps.cpp:112-113`, `sensors.cpp:1481-1485` | If GPS fixes keep arriving (fix.valid, new sequence) but the GGA time field stays constant (frozen or integer-second at 25 Hz), the extrapolated phase is pinned (anchor refreshed every fix), and if that phase is in 680-999 ms the slot gate returns Wait forever. 3 s age guard does not trigger. |
| F13 | Low | confirmed | `sensors.cpp:1402-1406` | Every received packet is logged as an INFO hex dump (up to 255 B, 16 lines) from the sensor task. Console is USB Serial/JTAG; stall risk with no host is plausible but untested. Also an amplifier for F4 floods. |
| F14 | Low | confirmed | `ra01.cpp:352-355` vs Aperture `ra01.cpp` receive | Witness `receive()` does not clear IRQ on oversize (Aperture does). Unreachable today (capacity 255 = max length byte). |
| F15 | Low | confirmed | Aperture `main.cpp:56-67` | Aperture radio init failure = infinite loop with no retry and no USB bridge. Ground side; a failed init at the pad kills the ground link until power cycle. |
| F16 | Low | confirmed | `sensors_landed.cpp` (whole file) | Compiled but unreachable (`kLowPowerModeEnabled=false`). If re-enabled: its radio path bypasses slots and hold-off, `schedule.on_sent` is called even when TX failed, TX power/FEM unchanged. Not a flight risk now. |

Earlier documented findings in this facet still present: H2(a) partly (F5), H3 (F6), M3 (F7), L6 (`set_radio_transmission_enabled` never called, still true, `sensors.cpp:624`), L7 (3 s Pad Idle rate), "radio commands carry no authentication" (F4/F5/F7).

---

## Details

### F1. Radio BUSY stuck high stalls the whole sensor task (Critical consequence, low likelihood)

Radio SPI is driven from the same 500 Hz task that reads the IMU FIFO, runs the attitude and position estimators and the flight state machine (`sensors.cpp:2264-2532`; radio at `:2498-2501`).

`Ra01::wait_until_ready` (`ra01.cpp:364-385`) spins 300 us, then loops with `vTaskDelay(1)` for up to `kBusyTimeoutMs = 500` ms per command, then returns `ESP_ERR_TIMEOUT`. Every command (`write_command`, `read_command`, `write_buffer`, `read_buffer`, the erratum register read) calls it.

Trace with BUSY stuck high (radio hung, latch-up, half-powered module, or a bad solder joint on BUSY):
- `service_radio_transmit` (`sensors.cpp:1436`) runs when a packet is due, at 1 Hz per type. It calls `start_transmit`. The first command (`SetStandby`, `ra01.cpp:242`) waits 500 ms, fails, returns. `radio_last_sent_[index]` was already set, so the next try for that type is 1 s later.
- Three types at 1 Hz, so 3 x 500 ms = 1.5 s of blocking per second, plus acks. The sensor task spends effectively all its time blocked in `vTaskDelay`.
- If a TX had been in flight when BUSY stuck: `poll_transmit` error path does `read_command` (500 ms) + `ClearIrq` (500 ms) + `SetRx` (500 ms) = 1.5 s in one call (`ra01.cpp:303, 320, 322`).
- IMU FIFO overruns (960 Hz three streams), state machine `update()` runs at a fraction of its rate (launch/burnout/landing dwell timers depend on timestamps but sampling gaps break the filters), flash logging of IMU data gaps. Task WDT (5 s) will not fire because the task yields.

Nothing marks the radio failed after a BUSY timeout. `radio_ready_` stays true.

Why it matters: the radio is the least reliable part of the board (bench notes: USB over-current when Ra-01SH-P transmits; swapped 3V3 LDO). A radio fault should cost telemetry, not flight data.

Fix idea (not applied): on the first `ESP_ERR_TIMEOUT` from `wait_until_ready` set a "radio faulted" flag, skip all radio work for N seconds, attempt a hardware reset + re-init (F2) from a low priority path; or run the radio from its own task so the estimator cannot be stalled.

### F2. No radio health check, re-init or config refresh (Critical, plausible)

- `Ra01::initialize` is the only place that configures the radio (`ra01.cpp:76-210`): packet type LoRa, RF frequency, PA config, DIO2 switch, modulation params, packet params, IRQ mapping, buffer base, continuous RX. It is called once from `Sensors::start` (`sensors.cpp:248`). There is no later call and no periodic check (grep: no other caller of `radio_.initialize`).
- `start_transmit` re-sends only SetStandby, (cached) SetPaConfig, the 0x0889 erratum, SetTxParams, SetPacketParams, buffer, SetTx (`ra01.cpp:241-274`). It assumes packet type, frequency, modulation, DIO/IRQ routing and DIO2 switch are still programmed.
- `configure_pa_for` caches `pa_low_configured_` (`ra01.cpp:473-486`, set at `:484`). After a radio reset the PA register is back to default but the cache says "configured", so it is not rewritten.

Scenario: the SX1262 (or its supply) resets while the ESP32-S3 does not (3V3 dip on a PA current step below the radio's POR but above the ESP brownout level 7 of about 2.5 V, ESD, a loose RESET line). The radio comes up in STDBY_RC, packet type GFSK (power-on default), default frequency, no IRQ mapping. Then:
- RX: DIO1 never rises, `poll_radio_commands` returns at `sensors.cpp:1368`. No command is ever received.
- TX: SetPacketParams is sent in a LoRa 6-byte layout to a GFSK modem; the radio transmits some GFSK/garbage carrier. TxDone may or may not set DIO1; either way the code returns Done or Timeout and calls SetRx, which now puts the radio into GFSK RX. No error is raised (`radio_tx_failed_` rises only on SPI errors/timeouts).
- Ground sees nothing for the rest of the flight; Witness reports healthy. Confidence is "plausible" because the power-on defaults are from the SX126x datasheet, not bench-tested here.

Fix idea: periodic (1 Hz) cheap probe (GetStatus or read a known register such as the sync word 0x0740/0x0741 or the packet-RAM probe used in init) and a full `initialize()` path when it mismatches; clear `pa_low_configured_`. Also count consecutive TX failures/timeouts and re-init after N.

### F3. Radio init failure at boot is final (High)

`sensors.cpp:247-258`: `radio_.initialize` failure sets `radio_ready_ = false`, calls `release()`, logs "radio disabled until reboot". Nothing retries. `radio_ready_` is never set true again (grep). `data_forwarder_.set_radio_available(false)` also stops the immediate queue. The init includes a strict RAM round-trip probe (`ra01.cpp:134-141`) and a 500 ms BUSY timeout per command; any transient (BUSY not yet low after a slow LDO start, SPI noise) disables telemetry for the whole session. The MS5607 gets a retry loop (`sensors.cpp:2332-2346`); the radio does not.

Impact: if it happens on the pad it is visible in the log/status only if someone looks (status byte has no radio-ready bit that I found). The flight then runs with no downlink and no commands (including no way to power-cycle remotely).

Fix idea: retry `initialize` every few seconds while in Pad Idle (and in flight, cheaply) until it succeeds.

### F4. Hold-off is unauthenticated, state-independent and unbounded (High)

Arming: `sensors.cpp:1419-1423`:
```
if (packet[0] == IRIS_PACKET_ID_COMMAND) { uplink_heard_ = true; last_uplink_ms_ = monotonic_ms(); ... submit_radio_command(...) }
```
This runs for any CRC-valid LoRa frame (same freq/SF/BW/sync) whose first byte is 0x05, before any validation. `canonical_command` (`command_bridge.cpp:12-30`) accepts a 3-byte frame `05 xx yy`, so the minimal hostile or erroneous frame is three bytes. Frames that later fail validation (wrong length, bad EOF) have already armed the hold-off.

Effect: `tx_slot::witness_telemetry` returns HoldOff while `now - last_uplink < 1000 ms` (`tx_slot.h:80`), consulted for every scheduled packet (`sensors.cpp:1483`). Only the immediate queue (acks/replies) bypasses it.

- Slot arithmetic makes one command per second enough to silence everything: Aperture sends in 800-950 ms; hold-off lasts 1000 ms from receipt, so it ends at 800-950 ms of the next second, i.e. after the whole 0-700 ms downlink window. One command per second in the uplink slot = zero telemetry.
- No state gating (Boost/Coast/Descent behave like Pad Idle), no cap on total silence, no minimum telemetry guarantee.
- Another team's Aperture or a second rocket's Aperture on the same channel, a ground tool in a retry loop, a buggy script, or GPS-assist uploads (allowed in Coast/Descent, `command_policy.h:60-64`) hold the downlink down for the whole stream. Also: each command is acked via the immediate queue (not held), so acks keep flowing while telemetry is silent; a stream of commands also produces back-to-back TX with no RX window (`poll_transmit` -> SetRx -> next `start_transmit` -> SetStandby in the same pass).
- Aperture itself never sends periodic commands (verified: Aperture `main.cpp` only sends queued USB commands; Horizon sends nothing periodically), so under normal operation this does not trigger.

Fix idea: do not arm hold-off in Boost/Coast/Descent (or cap it, e.g. at most 2 s of hold-off in any 10 s), arm it only for frames that pass `canonical_command`, and add a minimum telemetry rate guarantee.

### F5. 0x5250 can mute the downlink and uplink from the air (High)

`usb_serial_echo.cpp:945-974` (`handle_radio_power`), `command_policy.h:52-55`.

- Refused only when state == Boost. Accepted in Pad Idle, Coast, Descent, Landed, from radio.
- `front_end == IRIS_RADIO_FRONT_END_OFF` (0x02) drives RF_ENABLE low (`ra01.cpp:467-471`). Per `ra01.h:46-48` the PA/LNA are powered down: TX and RX both fail. Over radio it cannot be undone (the node can no longer hear). Recovery only by reset.
- `set_front_end_enabled` is called at `usb_serial_echo.cpp:960` before `send_ack` is queued (`:964`), so the ack is sent with the FEM already off. The ground may never see the refusal or acceptance.
- Drive can be set to -17 dBm (`ra01.cpp:460-465`): the downlink collapses by roughly 26 dB relative to +9 dBm drive. Range falls below usable at modest distance. Boost gating does not help: a pad-time command persists through launch.
- Sources: a stray command, another operator on the channel, or a mistaken click in Horizon (`Horizon/main.py:590`).
- Power also matters for the other direction: MAX is +3 dBm drive with a ~26 dB external PA (about +29 dBm). A raise in Coast/Descent loads the 3V3 rail (swapped LDO; bench USB over-current seen). A brownout reset in flight then returns at -17 dBm (F6).
- Possibly mid-edit (usb_serial_echo.cpp and command_policy.h are in the other agent's set).

Fix idea: refuse FEM-off and any power decrease outside Pad Idle/Landed (and require the front end to be on before Boost is allowed); ack before applying; or remove FEM-off from the flight image.

### F6. Radio power is RAM-only; any in-flight reset drops to bench power (High)

`hardware_pins.h:85`: boot default -17 dBm "bench-safe". `ra01.h:78-79` holds drive and FEM in `std::atomic` members; no NVS/RTC persistence; `flight_state_store` stores state/max altitude only (grep: no radio fields). The firmware deliberately resumes Boost/Coast/Descent after warm resets (WDT/panic/brownout/SW), see `FIRMWARE_STATE_MACHINE.md` section 4. After such a reset the resumed flight transmits at -17 dBm drive, and if the operator had raised it for launch that is lost without any ground-visible warning except the 0x5250 ack detail (which would need a new command).

Also from H3 (still present): cold reset -> Pad Idle -> 3 s radio period per type (`sensors.cpp:1465-1467`, `landed_logic.h:35-43`) at -17 dBm.

Fix idea: store drive/FEM in RTC memory (survives WDT/brownout resets except cold), or default to the flight power when `esp_reset_reason()` shows a warm in-flight reset; and make the Pad Idle throttle apply only after N seconds of stillness with no RESUMED flag.

### F7. Unauthenticated radio commands with flight-data consequences (High, documented)

No source id, address, sequence or key on LoRa commands. On the pad (Pad Idle) a single 10-byte frame can: erase or clear all flight logs (0x6869/0x686A, `command_policy.h:35-41`, handled inline in the USB task `usb_serial_echo.cpp:582-590`); start the maintenance image (0x5746 Wi-Fi on, `usb_serial_echo.cpp:555-558, 976-1012`), which has no flight sensing and times out only after 15 min (M3 earlier); switch the radio off (F5). In Landed 0x7300 restarts the board (H1 related). Armed rocket on the rail waiting for launch is exposed to every transmitter on 926 MHz SF7/500.

### F8. DIO1 dead or stuck low (Medium)

`poll_transmit` skips SPI while `DIO1 == 0 && !timed_out` (`ra01.cpp:297-300`). With DIO1 open: TX completion only detected at the 1.1 s deadline (then GetIrqStatus shows TxDone and reports Done). Throughput about one packet per 1.1 s versus 3 packets/s scheduled plus acks; telemetry is thinned, `radio_last_sent_` timing drifts. `poll_radio_commands` returns at `sensors.cpp:1368` when DIO1 is low, so no command is ever received and nothing logs it. If DIO1 is stuck high instead, `receive()` runs a SPI read every 2 ms pass (benign but noisy, F13).

Fix idea: poll GetIrqStatus at a low rate (e.g. 10 Hz) regardless of DIO1.

### F9. Stale cached packets presented as fresh (Medium)

`latest_radio_packet` returns `generation` but `service_radio_transmit` ignores it (`sensors.cpp:1473-1476`). The sensors packet cache is written only when `lsm_result == ESP_OK` (`sensors.cpp:2481-2485`); the state packet at 1 Hz; the camera packet only from the CAN relay. If the IMU read fails or Iris stops, the old frame (uptime field inside is the only clue) keeps going out at 1 Hz. Ground may display a frozen but "live" link. Low severity for survivability, relevant to trust.

### F10. TX failure paths do not escalate (Medium)

- `start_transmit` returns at the first failing SPI step after `SetStandby` without restoring RX (`ra01.cpp:243-266`); only the `SetTx` failure branch re-enters RX (`:275-280`). Radio stays in standby (no RX) until the next successful TX end.
- `poll_transmit` on `Error`/`Timeout` does ClearIrq + SetRx but nothing else; `++radio_tx_failed_` (`sensors.cpp:1524`) is only a counter. No limit, no re-init, no status bit.
- Combined with F1/F2, a failing radio is never recovered.

### F11. Landed with low-power mode off (Medium, documented owner decision)

`landed::kLowPowerModeEnabled = false` (`landed_logic.h:19`). After landing the board runs the full sensor loop and radio at 1 Hz per type indefinitely (`radio_period_ms` only throttles Pad Idle). Ping (0x0022) is acked UNKNOWN (`usb_serial_echo.cpp:625-630`); no 0x09 landed position packet is produced. Higher battery drain at the moment recovery needs the beacon to last. Bench report 2026-11-14 confirms this behaviour.

### F12. Frozen UTC with fresh fixes can pin the slot gate in Wait (Low, plausible)

`note_gps_fix` (`sensors_landed.cpp:118-132`) refreshes `last_utc_local_ms_` and the anchor on every new valid GGA, regardless of whether `utc_ms_of_day` advanced. If the receiver reports valid fixes at 25 Hz with a time field that does not advance (or whose fraction is stuck), `utc_clock_.utc_ms_at(now)` = anchor + at most 40 ms. If the frozen value has phase >= 680 ms (after the +20 ms latency add), `phase + airtime > 700` always: Decision::Wait forever. The 3 s age guard (`sensors.cpp:1482`) never expires because fixes are "fresh". Probability low (needs GPS misbehaviour), but the consequence is a silent total downlink loss. Cheap mitigation: require `utc_ms_of_day` to advance consistently with local time (reject if the learned rate leaves +-3 % for more than N fixes), or add a watchdog "no telemetry sent for 3 s -> ignore slots".

Other time-slot behaviour (traced, OK): UTC wrong/jumping only shifts the window; telemetry is still allowed 70 % of each second. GPS outage: `utc_known` false after 3 s -> free-running, as designed (`sensors.cpp:1481-1482`). Leap second: 1 s phase step, no lockout.

### F13. Hex dump logging of received packets (Low)

`sensors.cpp:1397-1406`: INFO log of each received frame plus `ESP_LOG_BUFFER_HEX_LEVEL`, in the sensor task. Log level default 3 (INFO). Console is USB Serial/JTAG (`sdkconfig`). With no host the IDF console normally drops output, but a stalled host could back-pressure. Plausible only; needs a bench test with a host that stops reading.

### F14-F16: see table.

---

## Answers to the specific questions

- **Every way telemetry can stop for the rest of a flight**: F1 (radio fault blocks the task), F2 (silent radio reset), F3 (init failure), F4 (hold-off from stray/hostile 0x05 frames), F5 (FEM off, drive down), F6 (reset in flight resets power), F8 (DIO1 dead throttles to about 1/s), F12 (frozen GPS time). Landed at launch is not an issue because low-power mode is off.
- **TX slot gate with wrong/stale time**: stale is bounded by `kTimeMaxAgeMs = 3000` (free-run after 3 s, `sensors.cpp:1481-1482`). Wrong or jumping time only moves the window. GPS time before fix valid: not used (needs `fix.valid` and `utc_valid`). Frozen UTC with live fixes is the only lockout (F12).
- **uint32 wrap / unsigned subtraction**: slot code uses 64-bit `monotonic_ms()`; `last_good_fix_local_ms_` conversion at `sensors_landed.cpp:125` is wrap-safe; `radio_last_sent_` uses 32-bit `TickType_t` differences (wrap-safe, 49-day wrap). `(now - last) < period` patterns are correct. No bug found.
- **Hold-off re-armed forever**: yes by any 0x05-first-byte frame (F4). Own echoes are not heard (half duplex, RX restarted only after TX done; clear IRQ at `poll_transmit`).
- **Radio driver stuck states**: BUSY stuck -> F1. TxDone/DIO1 -> deadline 1.1 s, then recovers (good), but see F8. Failed TX stays in standby only until the next TX (F10). No re-init watchdog (F2, F3).
- **PA/over-current/persistence**: drive and FEM RAM-only (F6); no soft limit on TX duty beyond the schedule; max +3 dBm.
- **Radio command able to mute/mis-configure downlink in flight**: 0x5250 (F5). Frequency/SF/BW are compile-time only; no runtime command changes them (good). Boost gate covers only Boost.
- **Packet build overflow**: payload arrays are sized by the comms.h macros; offsets are contiguous macros with `static_assert`s (`sensors.cpp:58-67`, `comms.h:1057-1066`); frame length max 105 B fits `RadioPacket` (`spi_data_forwarder.h:19-26`); Ra01 buffers 258 B vs max 255 + 3. No overflow found.
- **Queue starvation**: acks (immediate queue, depth 8) are served first on each pass and are not held; scheduled packets are cached-latest (no queue growth). Camera relay cannot starve state/sensors (each type has its own 1 Hz timer; rotation index advances on Send). Starvation comes only from hold-off (F4) and failing hardware.
- **Rates per state**: Pad Idle 1/3 Hz per type (sensors, state), camera 1 Hz; Boost/Coast/Descent/Landed 1 Hz per type (`sensors.h:78-81`, `landed_logic.h:35-43`). Total airtime about 65 ms/s at SF7/500 (fits the 700 ms window with slack).
- **Radio fail at boot**: F3.
- **Witness/Aperture agreement**: `tx_slot.h` and `comms.h` are byte-identical (diff clean). Modulation (SF7, 500 kHz code 0x06, CR 4/5, preamble 8, explicit header, CRC on, IQ normal), 926 MHz, default sync word match. Command frame (10 bytes, 0x0A EOF) and ack frame (13 bytes; Aperture's local ack detail byte layout matches Witness `handle_radio_power`) match. Aperture buffer 128 B >= largest Witness frame 105 B.

## Checked and OK

- `tx_slot.h`: `lora_airtime_ms` matches AN1200.13 for explicit header/CRC/no LDRO (SF7/500). `phase_ms` handles negative and wrap. At the second boundary, a phase of 980-999 plus the +20 ms latency maps to 0-19 and sends; real TX then spans the boundary into the next second's start, which is outside Aperture's 800-950 window. `phase + airtime <= 700` gives about 100 ms of real margin to 800 ms against +-50 ms time error. Aperture `aperture_may_send` window and burst rule (800 ms < 1000 ms hold-off) are consistent.
- `UtcClock::observe`: learned rate only accepted within +-3 %, after 300 s span; a single bad time cannot skew the rate by more than the bound; midnight wrap handled by fmod and day wrap.
- `note_gps_fix` local-time reconstruction is wrap-safe and cannot produce a future time.
- `Ra01::poll_transmit`: single-packet-at-a-time, deadline-based recovery from a missed TxDone, IRQ clear and RX restart on all exits. SetTx timeout (1 s) plus host deadline (1.1 s).
- `start_transmit` rejects length 0 or > 255, double start; `transmit()` blocking helper unused by flight code.
- `receive()` CRC error path clears IRQ; RxDone with CRC error returns CRC error; oversize unreachable.
- HITL cannot be entered over radio (`hitl_input.cpp:191`); log maintenance 0x6868 (USB export) is USB-only; flight-state radio commands are limited to Landed->Pad Idle reset and Pad Idle no-op (`command_policy.h:85-95`).
- CAN forwarding of radio commands is non-blocking (`can_link.cpp:42-54`, `xQueueSend(...,0)`); radio command queue overflow just drops (`command_bridge.cpp:88-92`).
- `radio_last_sent_` and `now` arithmetic: unsigned and wrap-safe; initial value 0 only delays the first packet by one period.
- Frequency/SF/BW cannot be changed at runtime (no command), so a mis-set cannot desynchronise the link.
- `set_radio_transmission_enabled` has no caller (earlier L6): radio cannot be disabled by software in flight.
