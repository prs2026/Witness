# Audit 06: GPS facet (read-only, working tree of 2026-11-15)

Files read in full: uart_driver.{h,cpp}, gps_receiver.{h,cpp}, nmea_gps.{h,cpp}, gps_assist.{h,cpp},
px1120s_gps.cpp, gps_i2c_driver.cpp (first half), skytraq_i2c_mailbox.cpp, sensors_landed.cpp, landed_logic.h,
tx_slot.h, command_policy.h, the GPS parts of sensors.cpp and usb_serial_echo.cpp (handle_gps_assist, cold start).
Mid-edit note: git shows nmea_gps.h, sensors.cpp, usb_serial_echo.cpp, command_policy.h modified and uncommitted
(usable_for_navigation, position_filter work). Nothing below depends on those hunks.
Nothing built or run; all findings are by reading and tracing.

Severity key: Critical = can lose a flight's data/telemetry or brick. High / Medium / Low.
Confidence: C = confirmed by reading, P = plausible, needs test.
No Critical found in this facet: no path hangs or bricks the computer. The GPS task cannot block another task,
no heap is used after start, all loops are bounded.

## Table of findings

| # | Sev | Conf | Where | One line |
|---|-----|------|-------|----------|
| G1 | High | C | gps_receiver.cpp:470-476, uart_driver.cpp:546-590 | Nothing supervises the receiver once `Ready`. A receiver reset or rate change in flight leaves the host deaf at the wrong baud for the rest of the flight |
| G2 | High | C | gps_receiver.cpp:448-468, uart_driver.cpp:580-590 | The 10 s retry of a `Failed` startup also runs against a receiver that is streaming fine: host baud forced to 115200, RX line unplugged 0.4 s, 3-9 s of garbage; repeats forever |
| G3 | High | C | uart_driver.cpp:654-663, sensors.cpp:1152-1157,1275-1280, landed_logic.h:20 | Telemetry never holds the last known position: 3 s after the last GGA, lat/lon/alt go out as 0,0,0. `last_good_fix_` is only used by the disabled Landed mode |
| G4 | High (still present) | C | usb_serial_echo.cpp:796-869, command_policy.h:57-61, gps_receiver.cpp:303-335, sensors.cpp:1419-1423 | GPS assist uploads are accepted in Coast/Descent/Landed, records are arbitrary SkyTraq messages (baud, factory default, restart, flash attribute), and every command frame silences telemetry for 1 s |
| G5 | Medium | P | uart_driver.cpp:208-246,225,260-269; hardware_pins.h:27 | Pin choice by a 400 ms edge count; default orientation is the wrong one for this board; a failed identification ends on the silent pin |
| G6 | Medium | C | main.cpp:72-74, uart_driver.cpp:61-126 | UART/GPS start failure (install needs 16 KB heap) is a log line only: no GPS for the whole flight, no retry, no status bit |
| G7 | Medium | C | gps_receiver.cpp:627-634, 47-50 | PX1120S Configure advances on timeout and Verify checks only mode and rate: `Ready` can be reported with the NMEA message set (0x08) never applied |
| G8 | Medium | P | uart_driver.cpp:361-371, usb_serial_echo.cpp:440-451,559-570, gps_receiver.cpp:337-360 | One unauthenticated radio frame in Pad Idle cold-starts the GPS; the request is not re-checked against flight state in the UART task, and the restart reconfigures at the host's current baud without re-identifying |
| G9 | Low | C | uart_driver.cpp:440-466, 580-590 | Fix timestamps are the read time, not the receive time: after the 0.8 s probe stall or any task stall, a burst of old fixes is stamped "now" |
| G10 | Low | C | usb_serial_echo.cpp:854-864, gps_assist.cpp:76-81 | COMMIT answered BUSY (hand-over slot full) is later answered OK by `AlreadyComplete`, but the blob was never submitted; a later refusal by the GPS is only logged |
| G11 | Low | P | uart_driver.cpp:484-529 | Bytes of SkyTraq binary replies (0x01/02/03/05, 0xE0, 0xE1, 0xFF) start phantom Iris frames on the GPS stream; one ending in 0x0A is queued as an immediate radio packet |
| G12 | Low | P | sdkconfig:1417 (UART_ISR_IN_IRAM not set) | UART ISR is not in IRAM: flash writes with cache disabled (NVS save at each state change) can overflow the 128 B FIFO and drop a few NMEA sentences |
| G13 | Low | C | nmea_gps.cpp, gps_receiver.cpp | No sanity gate (altitude/position jump) and no COCOM handling; behaviour above 18 km / 515 m/s depends entirely on the receiver |
| G14 | Info | C | main/CMakeLists.txt | px1120s_gps, gps_i2c_driver, skytraq_i2c_mailbox are not compiled into the flight image: no I2C path exists in flight |
| G15 | Info | P | BENCH_REPORT 2026-11-14/15 | The "88.4 kbaud" was measured at boot before any command was sent; a 25 Hz command cannot be the cause for that run. Host UART clock is a second suspect |

## Direct answers to the questions

* Stuck with no position for a whole flight? Yes, in two ways: (a) receiver changes rate or resets after `Ready` and the
  host baud no longer matches (G1); (b) the `Failed` retry loop keeps a working receiver half-deaf (G2). The parser itself
  recovers from garbage within one sentence.
* Receiver resets/browns out in flight and returns at 115200 with default config: if the host is at 115200 the NMEA
  parser keeps working (position still used, only at the default rate and without airborne mode and 25 Hz); this is not
  detected and not repaired. If the host is at the fallback rate (88,403, `baud_fallback_`) the UART is deaf forever.
  Status stays `Ready`, nothing re-probes.
* 10 s retry in flight: yes, `can_retry_identification()` is just `status_ == Failed` (gps_receiver.h:105), no flight-state
  or "receiver is streaming" test. It blocks only the UART task (vTaskDelay 2 x 400 ms), never another task, but it does
  flip baud and pins (G2, G5).
* Baud-change or factory-reset command path: the built-in ones (`SkytraqSwitchBaud` 0x05, `UbloxSwitchBaud` CFG-PRT) are
  unreachable because `kFixedBaud` is a constexpr true (gps_receiver.cpp:232-235). No built-in factory reset exists. The
  assist upload can carry any such message (G4). Nothing built-in is written to receiver flash: every command ends with
  attribute 0 (SRAM).
* I2C mailbox: not compiled into the flight image (G14), cannot hang the bus in flight.
* Event queue: none (queue size 0, uart_driver.cpp:115-121). RX overflow is silent; there is no counter.
* Heap: only `uart_driver_install` (2 x 8192 B, uart_driver.cpp:115-121; the 8 KB TX ring is not needed). Static: 4 KB
  buffers for assist in `UartDriver::assist_pending_`, `GpsReceiver::assist_blob_`, the function-static `blob` in `run()`,
  plus the Assembler in UsbSerialEcho.

## Details

### G1 (High, C). No supervision after Ready; deaf at a wrong baud forever
`GpsReceiver::service()` returns after `service_assist()` when `status_ == Ready` (gps_receiver.cpp:472-475) and
`consume()` ignores all bytes when `Ready` and no assist is active (gps_receiver.cpp:656-657). The only ways back to
`Configuring` are `cold_start`, `wake`, an acked assist restart, or `Failed` + retry (uart_driver.cpp:580). The UART task's
5 s diagnostic only logs "GPS UART input silent" (uart_driver.cpp:615-617); it takes no action. The `NmeaGps` 3 s timeout
only zeroes the position.
Scenario A: bench shows the receiver runs at 88,403 baud on this board (host `baud_fallback_`, gps_receiver.cpp:231).
A 3V3 dip that resets the GPS but not the ESP (the regulator is under suspicion: swapped SOT-89 LDO, supply instability
seen at power-up in BENCH_REPORT 11-15) brings it back at its power-up rate. If that differs from 88,403, no NMEA
decodes, status stays Ready, and the position is 0,0,0 for the rest of the flight.
Scenario B: receiver comes back at the right rate but in default configuration (no airborne mode, 1 Hz, full NMEA set).
GGA still parses, so position survives, but the dynamics mode is the receiver default and nobody notices.
Fix idea: while Ready and not asleep, if there is no checksum-valid GGA for N s (say 5 s) or the GGA rate is far below
25 Hz, drop to Configuring and run identification (current host baud and orientation first, then 115200, then the last
measured rate). Do it only when position is already lost, never against a good stream.

### G2 (High, C). The Failed retry disturbs a receiver that works
Trigger condition is only `status_ == Failed` (gps_receiver.h:105). Failed is reached by any NACK of a required command
(gps_receiver.cpp:809-810), a readback mismatch or timeout (817-826, 635-638), or a send failure. A receiver can
stream good NMEA the whole time in these cases (e.g. the readback times out because the ACK frames were lost).
Every retry (uart_driver.cpp:580-589):
1. `retry_identification` sets the host baud to 115200 immediately if it was anywhere else (gps_receiver.cpp:462-464). With
   a receiver working at the 88,403 fallback this makes every later byte garbage.
2. `select_gps_pins()` blocks the UART task for 2 x 400 ms (uart_driver.cpp:214-217), routes RX to the other pin for the
   second window (260-269), and `probe_rx` toggles `autobaud_en` on a live UART (189-190).
3. Identify then waits 3 s per baud/orientation stage (gps_receiver.cpp:543-600): 3 s at 115200, 3 s at the measured
   rate, possibly 3 s on the other orientation, then Configure rewrites mode/rate (SRAM) on a receiver in use.
Net effect: position lost for roughly 4-9 s out of every ~15 s, each cycle, forever (no retry limit,
`kIdentifyRetryMs` 10 s). It runs in Boost/Coast/Descent too: nothing tests flight state.
Fix idea: skip or abort the retry while `NmeaGps::gga_fresh()`; never change host baud or pins while valid NMEA is flowing;
treat "NMEA flowing but status Failed" as Ready-with-warning.

### G3 (High, C). No last-known position in telemetry
`latest_gps_fix()` returns `Fix{}` once the GGA is older than 3 s (uart_driver.cpp:659-662). `build_state_payload`
(sensors.cpp:1152-1157) and the debug packet (1275-1280) send it as is, so latitude/longitude/altitude are 0 and the
satellite count 0. `Sensors::last_good_fix_` is updated (sensors_landed.cpp:118-132) but only consumed inside
`run_landed()` (209-210), and `landed::kLowPowerModeEnabled = false` (landed_logic.h:20), so even the 0x09 packet with the
last position is never sent. After landing, any GPS stall (G1, G2, a bad antenna spot, COCOM exit) makes the recovery
telemetry read 0,0. The ground station may keep its own last value (not checked here), but the firmware gives no
"stale" flag either: only the GPS-locked bit drops.
Fix idea: in `build_state_payload`, fall back to `last_good_fix_` with the GPS-locked bit off (optionally with an age).

### G4 (High, C; previously documented H2, partly fixed). Assist upload scope
`allow_gps_assist` refuses BEGIN/DATA/COMMIT only in Boost (command_policy.h:57-61). In Coast, Descent and Landed any
radio frame sequence (no authentication) is accepted. After COMMIT, `GpsReceiver::begin_assist` writes each record body
verbatim as `A0 A1 .. body .. 0D 0A` (gps_receiver.cpp:163-180, 422-446). Record bodies are only length-checked (<=128 B,
gps_assist.cpp:19-32). A body 0x04 (factory default), 0x05 (set UART baud, attribute 1 = flash), 0x0E, 0x01 (restart),
0x64/0x17 etc. is passed through. An acked restart goes to `RestartWait` and Configure at the host's current baud
(gps_receiver.cpp:772-783). A baud change would leave the host deaf (G1: nothing re-detects). Even a legitimate upload
writes ephemerides into a navigating receiver in Coast/Descent.
Second effect: every received 0x05 radio frame sets `uplink_heard_` (sensors.cpp:1419-1423) and holds telemetry for
1 s (tx_slot.h:56-58, sensors.cpp:1483). A 4096 B upload is 1024 DATA frames: telemetry is silent for the whole transfer
(and any command stream or noise matching 0x05 does the same, in any state including Boost).
Fix idea: restrict assist to Pad Idle (and Landed if wanted), whitelist message ID 0x41 only, and make the hold-off
state-aware (never in Boost).

### G5 (Medium, P). Pin selection
`select_gps_pins()` picks the orientation by counting edges in two 400 ms windows (uart_driver.cpp:214-232). It swaps only
if the configured pin has <40 edges and the other has >=40. This board needs the swap (bench: receiver TX on GPIO5) but
`HW_MCU_UART_SWAP_RX_TX` is 0 (hardware_pins.h:27), so the default orientation is the wrong one. If the receiver is silent
or between 1 Hz bursts during the window (a freshly booted or reset receiver at default config sends ~45 ms of data per
second, so a 400 ms window misses it about half the time), no swap happens. Identify then spends 2 x 3 s, flips back
to the probe's orientation (gps_receiver.cpp:596-598) and fails on the silent pin; next try in 10 s. Not a flight loss when
the receiver is up at power-up, but it costs 10-30 s after a cold start, and inside G2's loop it can park the UART on the
wrong pin while a receiver is working. Fix idea: set the swap for this board, and make the last Identify fallback "the
orientation where NMEA was ever seen".

### G6 (Medium, C). UART start failure is silent
`gps_uart.start()` failure is `ESP_LOGW` and the boot continues (main.cpp:72-74). `initialize()` needs 2 x 8192 B of
internal heap (uart_driver.cpp:115-121) after the flash logger and queues are already allocated; bench saw 35 KB free.
There is no retry, no status bit set, and `Sensors` then reads an empty fix forever. Fix idea: TX ring 0 or small, and retry
`start()` from a timer.

### G7 (Medium, C). Ready does not mean configured
For the PX1120S an unanswered Configure command is skipped after 3 attempts (gps_receiver.cpp:627-631) and the
readbacks cover only navigation mode and update rate (47-50, 817-822). If the 0x08 NMEA interval message is the one lost,
the receiver streams its default sentence set at 25 Hz (GSV, GLL, VTG and more, very roughly 15 kB/s) over a link that
carries 11.5 kB/s at 115200 or 8.8 kB/s at 88.4k: sentences are truncated or dropped and GGA arrives well below 25 Hz while
status says Ready. Fix idea: count GGA per second after Ready and compare with the expected rate; verify 0x08 if the
receiver has a query for it.

### G8 (Medium, P). Cold start on the pad
Pad Idle only, USB or radio (usb_serial_echo.cpp:440-451, 559-570), but the flight state is checked in the USB task and the
action runs later in the UART task through a plain `volatile bool` (uart_driver.h:120, uart_driver.cpp:361-371), so a launch
detected between the two still cold-starts the receiver. In Pad Idle any radio frame suffices (unauthenticated): the
receiver then needs 30 s or more for a fix, which on a pad about to launch loses the position at the worst time. After
the restart the code does not re-identify (kFixedBaud, gps_receiver.cpp:484-486): if the receiver comes back at a different
rate than the host (the 88.4k case), Configure runs deaf (21 s of timeouts), Verify fails, retry after 10 s (G2 path).
Fix idea: re-check Pad Idle in the UART task; require confirmation for the radio source.

### G9 (Low, C). Timestamps of backlog
`last_rx_ms` is taken after `uart_read_bytes` (uart_driver.cpp:440) and becomes `Fix::received_ms` (nmea_gps.cpp:103).
When the task was away (select_gps_pins 0.8 s, NAND stalls, cache-disabled flash ops), up to 8 KB of old sentences are read
within a few ms and every GGA gets the same "now" stamp. Effects: stale fixes pass `gga_fresh`, the Kalman filter's GPS
lag handling sees a 0.8 s old fix as fresh, and UTC (`note_gps_fix`, sensors_landed.cpp:127-130) is anchored with an error
up to the backlog; the slot logic only needs +-50 ms (tx_slot.h:19-20). The slot logic cannot starve telemetry: some part of
every second is always `Send`.

### G10 (Low, C). Silent assist drops
`Assembler::handle` answers COMMIT with `AlreadyComplete` (acked OK) whenever the transfer completed earlier, even if the
first COMMIT got BUSY because `assist_pending_length_` was occupied (usb_serial_echo.cpp:854-864). The ground believes the
upload is done, the blob was never handed over. Also `GpsReceiver::begin_assist` refusals (not Ready, wrong model, upload
running, malformed) after an OK ack are only logged (uart_driver.cpp:353-358).

### G11 (Low, P). Phantom radio frames from the GPS stream
The loop that rebuilds Iris frames runs on every GPS byte (uart_driver.cpp:484-529). Start bytes are IDs 0x01, 0x02, 0x03,
0x05, 0xE0, 0xE1, 0xFF (canonical_frame_length). SkyTraq replies contain these values (ack ids, sub-ids, readback "mode 5"),
so during (re)configuration a phantom frame of 10-103 bytes swallows following NMEA and, if its last byte is a line feed
(0x0A, the Iris EOF value), is queued as a radio packet into the immediate queue that bypasses telemetry scheduling. Rare,
and only while configuring.

### G12 (Low, P). UART ISR not in IRAM
`CONFIG_UART_ISR_IN_IRAM` is not set (sdkconfig:1417). A cache-disabling internal flash write (NVS save at every state
change, calibration save) can stall the ISR for tens of ms; the hardware FIFO holds 128 B (11 ms at 115200). A few
sentences are lost; with 25 Hz output this costs a fix or two, but it coincides with Boost/Coast/Descent transitions.

### G13 (Low, C). No position sanity gate, COCOM unknown
`NmeaGps::parse` accepts any checksum-valid GGA with quality 1..8 and parseable numbers (nmea_gps.cpp:112-126): no limit
on altitude, jump from the previous fix or HDOP. If the receiver applies COCOM limits (18 km / 515 m/s) it may send quality 0
with empty fields, which parses correctly to "no fix, zeros" (`fix_ = {}` then early return, nmea_gps.cpp:101,115-116) and
then G3 applies. The code cannot tell and nothing documents the PX1120S behaviour; worth a HackRF simulation above the limits.

### G14 (Info). I2C path
`px1120s_gps.cpp`, `gps_i2c_driver.cpp`, `skytraq_i2c_mailbox.cpp` are only built for host tests (the flight source list in
main/CMakeLists.txt has none of them). GPIO 4/5 are shared with the UART option (hardware_pins.h:35-37) but the flight code
never touches I2C. No hang possible there in flight.

### G15 (Info). On the 88.4 kbaud
The 11-14 bench report says the pin probe measured about 88 kbaud at boot, before identification, i.e. before Witness had
sent anything, while an FTDI sees 115200. The 11-15 update then says the receiver answers at 115200 after power-up and
moves to 88.4k "during configuration", which blames the 25 Hz command. The two do not agree. Cheap tests: (1) scope the
receiver TX bit width on the Witness board; (2) print `uart_get_baudrate` and the autobaud `sclk_hz` at boot (the same
`sclk_hz` is used to measure and to set the rate, so a host clock that differs from what the driver assumes would make
115200 appear as 88.4k and still work when set to 88,398). `CONFIG_PM_ENABLE=y` with the UART on the default clock source
is worth checking if the Landed light-sleep mode is ever re-enabled.

## Earlier documented findings in this facet (FIRMWARE_STATE_MACHINE.md)
* H2 (assist/GPS messages unrestricted): STILL PRESENT in Coast/Descent/Landed, no record whitelist (G4).
* M9 (Failed terminal): FIXED by the 10 s retry, but the retry brings G2. The doc text at FIRMWARE_STATE_MACHINE.md:499 and
  the state diagram ("Failed --> Failed: terminal") are stale.
* "Assist and cold start never forwarded, but forwarded when typed as ASCII hex": not in this facet (usb_serial_echo text
  path), not re-checked.

## Checked and OK
* NMEA parser (nmea_gps.cpp): sentence buffer 160 B with `length_ + 1 >= size` guard, '$' restarts, non-printable aborts,
  CR/LF triggers parse; checksum over everything between '$' and the first '*', exactly 2 hex digits (either case) after it;
  `fields[20]` guarded by `count == 20`; GGA needs >= 11 fields and fields 6..10 are only touched after that; GSA needs >=18;
  empty fields give false/NaN, never garbage; `number()` rejects exponents, NaN, inf, whitespace, empty; latitude/longitude
  sign by hemisphere letter with S/W negative, wrong letter or empty hemisphere rejected, minutes >= 60 rejected, range
  checked; `M` unit required; altitude float cast finite-checked; quality 0 gives no position but keeps satellites.
* Binary/NMEA interleave: binary bytes either abort the NMEA collector or are skipped; a '$' inside binary only starts a
  sentence that fails its checksum.
* SkyTraq frame parser: payload size checked against the 64 B buffer (max 57), checksum and CR LF verified, idle timeout
  200 ms; UBX parser: payload stored only up to 48 B, length cap 1024, checksum checked.
* UTC: `utc_time` rejects hh >= 24, mm/ss >= 60 (a leap second is dropped, harmless), rounds to ms and rejects
  86,400,000; `UtcClock` wraps with `fmod(kDay)` and handles the midnight wrap in `observe`; no date is used so no week or
  month rollover; a receiver that still outputs GPS time (18 s offset before it knows the UTC offset) does not matter,
  only milliseconds within the second are used.
* Sequence counter: 32-bit, +1 per GGA (25 Hz: 5.4 years to wrap); consumers compare with `!=`.
* Fix lock: `portENTER_CRITICAL` around a ~56 B struct copy on both sides (sub-microsecond); `esp_timer_get_time` is called
  outside the lock after the copy, so `now >= received_ms` always (no negative age); 32-bit ms arithmetic is wrap-safe.
* RX ring: 8192 B at about 3.8 kB/s (25 Hz GGA+RMC plus GSA) is 2 s of margin; the task reads every 2 ms at priority 3;
  an overflow silently drops bytes (no event queue) but cannot hang.
* UART TX: `uart_write_bytes` returns after copying to an 8 KB ring; `uart_wait_tx_done` bounded at 100 ms; the TX pin is
  connected only after the receiver was heard (`tx_connected_`).
* Assist assembler: chunk offset checked (`offset < length_`, multiple of 4), `have_` sized for 4096/4 chunks, CRC-16 over
  the full blob, record framing validated twice (COMMIT and `begin_assist`), `assist_blob_` copy size-checked, hand-over
  slot guarded by a spinlock for a ~4 KB memcpy (a few microseconds); an aborted or partial upload cannot leave the receiver
  unusable by itself: records are SRAM ephemeris writes, each acked or timed out (2 attempts x 1 s), the upload ends at
  the last record, `cold_start` cancels it.
* Receiver state machine has no unbounded step: every step has a 3-attempt timeout (3 s) except RestartWait (2 s) and the
  unreachable SwitchBaud steps; no state waits forever.
* Nothing in the built-in sequence writes receiver flash (all commands end with attribute 0); no built-in baud change
  (kFixedBaud), no factory reset.
* Landed GPS sleep/wake is compiled in but off (`kLowPowerModeEnabled = false`), so no GPS sleep command is sent in any
  state; `cold_start` is refused outside Pad Idle by the command handlers.
* Heap: no allocation after `uart_driver_install`; the GPS task stack is 4096 B with a 64 B read buffer, a 103 B frame
  buffer and the `NmeaGps` object (about 300 B); depth is dominated by ESP_LOG formatting; no overflow path found, but there
  is no high-water-mark log for this task (the usb_serial_echo overflow on 11-14 was found by bench).
* HITL: `current_gps_fix()` takes the simulated fix only when HITL is active.
