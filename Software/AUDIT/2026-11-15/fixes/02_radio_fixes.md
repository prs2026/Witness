# 02 Radio fixes (W1, W8, W9, W10, W13, F10/F13), 2026-11-15

Not committed, not flashed, not run on hardware. Host test: `tests/tx_slot_test.cpp` passes (-Wall -Wextra -Werror).
Firmware build (flight overlay, scratch dir `...\Temp\wf2`): full link NOT reached: `ninja -k 0` (3 runs) compiled ra01.cpp, sensors.cpp, radio_power_store.cpp and every other file without error except `uart_driver.cpp:472` (`std::min(const uint32_t&, unsigned)`, another agent's edit). Coordinator will do the final build.

## Status

| Finding | Status |
|---|---|
| W1 / F6 persistent TX power | Fixed on the Witness side. The 0x5250 handler (usb_serial_echo.cpp, not mine) must call `radio_power_store::save()`: not done yet. |
| W8 / F1 radio fault stalls sensor task | Fixed (fault latch, 2 ms BUSY budget) |
| W9 / F2, F3 re-init and health check | Fixed |
| W10 / F4 hold-off | Fixed (validated frames only, no hold in Boost, 3 s cap + forced cycle) |
| W13 / G3 last good fix in telemetry | Fixed |
| F13 hex dump | Fixed (one rate-limited line) |
| F10 / F8 TX failure path | F10 fixed. F8 (DIO1 stuck/open: RX never polled) not done. |

## What changed

- `main/radio_power_store.h/.cpp` (new, added to the flight source list): `bool load(int8_t&)`, `esp_err_t save(int8_t)`. NVS namespace "radio", key "tx_dbm" (i8). Range checked against IRIS_RADIO_TX_POWER_MIN/MAX_DBM on load and save; load returns false when NVS is not initialised; save skips the write if unchanged. Front end on/off is not stored.
- `sensors.cpp Sensors::start`: loads the stored level, applies it with `set_tx_power_dbm`, logs "restored from NVS" or "compiled default". Compiled default is still -17 dBm. NVS is initialised in main.cpp before `sensors.start()` (verified), and load tolerates its absence anyway.
- `ra01.h/.cpp`:
  - Real-time mode (`set_realtime(true)`, set by Sensors after boot init; the maintenance image does not enable it and behaves as before). BUSY wait is a spin of at most 2 ms, no vTaskDelay. A BUSY timeout, 3 consecutive SPI errors, or 3 consecutive failed transmissions latch `faulted_`; after that every start/receive/command returns ESP_ERR_INVALID_STATE immediately. Counters: `fault_count()`, `busy_timeout_count()`, `reinit_attempts()`, `last_fault()`.
  - `initialize()` split into `configure_step(0..11)` (same commands, same order) so the same sequence is used by the blocking init and the stepper.
  - `begin_reinit()` / `service_reinit()`: non-blocking hardware reset (2 ms reset, 10 ms, front end, 10 ms) then one configuration step per call; never waits on BUSY (returns InProgress); gives up after 500 ms. Clears the PA cache so power/PA/packet params are rewritten; TX power and front-end state are re-applied from the atomics.
  - `health_check()`: one 3-byte GetPacketType read; Lost if packet type is not LoRa (power-on default is GFSK) or chip mode is outside STBY_RC..TX: latches the fault. Idle in standby with settings intact: re-enters RX (Repaired).
  - `start_transmit`: every failure after the first command now restores continuous RX (F10) and counts towards the latch. `poll_transmit` counts Timeout/Error, resets on Done. `receive()` clears the IRQ on an oversize frame (F14).
- `sensors.cpp`: new `service_radio()` replaces the radio block in the task loop. Fault -> `radio_ready_=false`, `set_radio_available(false)`; first re-init attempt 2 s later, then every 5 s after a failed attempt; boot-time init failure is retried every 5 s. On success `radio_ready_` and `set_radio_available(transmission_enabled)` are restored. Health check every 2 s while idle.
- Hold-off (`tx_slot.h`, copied byte-identical to Aperture): new `UplinkHoldOff` class. Armed in `poll_radio_commands` only when `command_bridge_.submit_radio_command` did not return ESP_ERR_INVALID_ARG (its validation result); not armed in Boost; a hold lasts at most `kHoldOffMaxMs` = 3 s, then one telemetry cycle (all 3 packet types, or 2 s at most) is forced before another hold can start. Existing function signatures unchanged. Tests cover Boost, release after 1 s, cap under a steady stream, forced cycle completion and the 2 s fallback.
- G3: `Sensors::telemetry_gps_fix()` keeps the last valid lat/lon/alt and substitutes it whenever the current fix is not valid. Used for the 0x02 state packet (also the NAND state log) and the 0xE? debug packet.
- F13: per-packet hex dump removed; one summary line per 2 s at most, CRC/receive errors share the limit.

## What the ground sees (G3)

State packet lat/lon/alt now carry the last good fix when GPS is lost. Marker: the **GPS_LOCKED status bit** (status byte 1 bit 3) is clear while the position is the held one. Satellites is the live count (0 once the GGA itself is stale). Horizon should show lat/lon with GPS_LOCKED=0 and non-zero position as "last known / stale". The age of the held fix is not in the packet (no spare field; lengths unchanged). Before the first fix ever, 0,0,0 as before.

## Worst-case blocking in the sensor task

- Removed: 500 ms per radio command (1.5 s per failed TX).
- Now: normal pass unchanged (BUSY is microseconds). Hung BUSY: one 2 ms spin, once, at the moment the fault latches; faulted radio costs ~0. Re-init: at most one SPI command per pass (<= ~0.3 ms for a 103-byte buffer write) with no BUSY wait; total wall time about 50 ms spread over passes, 500 ms cap. Health check: ~10 us per 2 s. Hold-off governor: negligible.
- NVS `save()` (command task, not the sensor task) does a flash write, a few ms, ~50 ms worst case when NVS erases a page, which can stall the other core briefly. It is refused in Boost by command policy.

## Trickle-down / needed elsewhere

- **usb_serial_echo.cpp (command agent):** in `handle_radio_power`, after a successful `set_radio_tx_power_dbm`, call `radio_power_store::save(dbm)` (include "radio_power_store.h"); never persist the front-end state. Update the ack/doc text: power now survives resets.
- **Operators/docs:** a commanded power now persists across reboots (including a bench USB port tripping at a high level after an earlier flight setting). Set it back to -17 on the bench. FIRMWARE_HANDOFF / GROUND_STATION_PROTOCOL should say "RAM only" is no longer true, and the hold-off is capped at 3 s with no hold in Boost.
- **Aperture:** only tx_slot.h changed (additive class, copy is byte-identical). Aperture's burst of commands can now silence Witness for at most about 3 s, then at least one full telemetry set is sent.
- **Horizon:** show last-known position when GPS_LOCKED=0 and lat/lon non-zero.
- Aperture burst (3 commands, then 1.5 s quiet): the 1 s hold-off ends before the quiet period does, and the 3 s cap guarantees a telemetry cycle anyway.
- The command agent already calls `radio_power_store::save()` behind `__has_include`; the header exists with the agreed API, and `load()` is applied in `Sensors::start` before radio init.
- Short 3-byte command frames still pass `canonical_command` and so still arm the hold-off (command_bridge.cpp, not mine).
- Radio acks/replies are dropped while the radio is faulted (`set_radio_available(false)`).
- F8 (DIO1 dead: no RX polling) left as is; the health check does not cover it.
- Re-init resets the radio, so a command arriving during the ~50 ms re-init is lost.
