# Bench report 2026-11-14: four-slot images, Pad Idle rate, Landed mode

Board: Witness v0.2 on the bench Pi (ser2net 7000 flash / 7100 data), Aperture on
7102, Iris on 7101. Firmware `a3bcd69-dirty` (uncommitted Landed/Pad Idle work plus
the stack fix below). Powered from the Pi's USB ports, so USB is always attached:
**light sleep never engages on this bench** (by design, USB holds a no-sleep lock).

## Summary

| Test | Result |
|---|---|
| USB migration flash (bootloader, table, otadata, flight_a, maint_a) | PASS, all hashes verified |
| Flight image boots from flight_a, partition table shows 4 app slots | PASS |
| Flight -> maintenance over USB (`0x5746` on) | **FAIL first, fixed**: stack overflow in `usb_serial_echo` (panic reset). PASS after raising its stack 3072 -> 6144 |
| Maintenance image: AP up, 0x08 beacon every 5 s, log store mounted (~12 s, 10 files) | PASS |
| Maintenance -> flight, `0x5746` off over USB | PASS (300 ms restart, flight slot 0) |
| Power loss while in maintenance (uhubctl cycle) | PASS, boots flight (POWERON) |
| Flight -> maintenance -> flight over LoRa (via Aperture) | PASS, ack over radio, beacons received by Aperture, 700 ms restart on off |
| `0x6868` over radio refused (USB only) | PASS, ack refused, detail 0 |
| Pad Idle LoRa rate | PASS: sensors and state every 3.0 s, Iris camera relayed at 1 Hz; USB stays at full rate |
| Landed entry (`0x7304`) | PASS: PM to 80 MHz with light sleep enabled, CAN paused, IMU stopped, core1 0.1 %, core0 4.7 % |
| 0x09 landed position packet | PASS: at entry, then every 60.0 s (free-running, no GPS time), on USB and LoRa, Horizon decoder OK |
| Ping (`0x0022` over radio) in Landed | PASS: immediate 0x09 with the ping flag, ack OK; second ping 2 s later acked BUSY, no extra packet |
| Radio command to Iris in Landed | PASS: CAN brought back up, Iris PONG received, CAN paused again 3 s later |
| GPS refresh timeout in Landed | PASS (logic): 90 s timeout, location kept as stale |
| Landed -> Pad Idle | PASS: PM back to 240 MHz no sleep, IMU back to 936 Hz, CAN resumed, 3 s radio rate resumed |

## Bugs found

1. **Stack overflow on the maintenance switch (fixed).** `0x5746` on crashed the
   flight image ("A stack overflow in task usb_serial_echo has been detected"),
   which rebooted into flight. `boot_manager::prepare_switch` (image-info reads of
   both slots, NVS, `esp_ota_set_boot_partition`) runs on that task, which had
   736 B free at idle. `kTaskStackSize` in `usb_serial_echo.h` is now 6144. Free
   heap dropped from 38.1 KB to 35.0 KB.
2. **GPS identification ends on the wrong pin** (pre-existing logic, not the
   Landed change). At boot the pin probe finds the receiver's TX on GPIO5 (this
   board's RX/TX are swapped) and measures about 88 kbaud. Identification at
   115200 then gets no reply in either orientation. Afterwards it keeps
   listening on GPIO4, the silent pin, so "GPS UART input silent" follows for the
   rest of the run. Even on the right pin, the bytes don't decode as NMEA at
   115200 (GGA count 0-1 over 19 KB to 800 KB). Either the receiver isn't at
   115200 on this board, or the line is degraded between the GPS and Witness. The
   GPS board itself works on an FTDI adapter. Firmware fix to make: after a
   failed identification, fall back to the orientation the probe found.
3. **Light-sleep CPU power-down fails at Landed entry** (harmless):
   `sleep_cpu_configure(236): Failed to enable CPU power down during light sleep`.
   The retention buffer can't be allocated, so the CPU stays powered in light
   sleep (a little more current). Either free about 8 KB or turn off
   `CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP`.
4. `CONFIG_PM_SLP_DISABLE_GPIO=y` is still set in `build/sdkconfig`, although
   `sdkconfig.flight` asks for it off (the existing sdkconfig won). The per-pad
   `gpio_sleep_sel_dis` calls should override it, but this needs checking in the
   battery test below.
5. Transient `sensor packet queue failed: ESP_ERR_NO_MEM` and `could not forward
   64 UART bytes` during the switch's flash writes. Expected, but noted.

## Hardware observations

- **MAC read flips:** esptool read `ac:c5:b9:15:60:24` on the first two runs and
  `84:c5:b9:15:60:24` after the rewire (Wi-Fi AP MAC `84:c5:b9:15:60:25`). Flaky
  eFuse/MAC reads were a symptom of the overvolted 3V3 rail. Confirm the
  TPL740F33 is fitted and the rail reads 3.3 V.
- One `last reset WDT` was seen during the GPS rewiring (cause unknown; not seen
  again).
- No USB over-current trips during the LoRa commands and pings.
- esptool resets over rfc2217 on 7000 failed with ConnectionResetError several
  times; `0x5746` on/off or `uhubctl` are reliable ways to reset.

## Update: GPS fix (same day)

`gps_receiver.cpp` with `kFixedBaud`: when 115200 gives no NMEA and the startup
probe measured a clearly different rate of at least 57.6 kbaud, Witness now
listens at the measured rate once, on the probed orientation. Only the host UART
rate changes; no baud command is sent to the receiver, and the configuration
stays RAM-only. If nothing is found anywhere, it flips back to the probed
orientation before giving up. Host test `tests/gps_receiver_test.cpp` passes,
with new cases for the measured-rate listen, configure-and-sleep there, no extra
rate near 115200 or below 57.6k, and the flip-back.

On the bench: `GPS UART host rate now 88398 baud` -> `GPS identified as PX1120S
at 88398 baud` -> `airborne 4 g mode and 25 Hz GGA+RMC confirmed`, then 125
GGA per 5 s (25 Hz). No fix indoors (no sky view, no HackRF sim). **Open
question:** on Witness this PX1120S talks at ~88.4 kbaud, but at 115200 on an
FTDI adapter. Witness sends it no baud command (fixed-baud build). Worth
checking the GPS on the FTDI while it is powered from the Witness 3V3 rail, or
scoping its TX bit width on the Witness board (115200 = 8.7 us, 88.4k = 11.3 us).

## Update 2026-11-15: FSM rework, gating, logging, slots, GPS retry

| Test | Result |
|---|---|
| HITL "Nominal 10 g" on Witness with the Kalman-AGL state machine | PASS: Boost 11.03 / Coast 12.50 / Descent 32.41 / Landed 148.21 s, all within 0.03 s of the reference; Kalman AGL error at most 2.8 m |
| Commanded Coast at the pad (maximum ~0 m) | found a false Descent (5 % of ~0 m); fixed with a 10 m minimum drop, then PASS |
| `0x5250` and GPS assist BEGIN in Boost, USB and LoRa | PASS: refused; assist STATUS still answered |
| Both NAND slots holding flights | PASS: "both log slots hold a flight: NOT LOGGING (nothing is erased)", no session, status byte 1 bit 7 set |
| `0x686A` clear over LoRa (Pad Idle) | PASS: acked OK, "log store freed: logging resumed", new session 848, bit 7 cleared |
| Uplink hold-off: 10 pings 0.5 s apart via Aperture | PASS: all acked; telemetry paused until 1 s after the last ping |
| GPS time slots | not exercised: no GPS fix indoors on either board |
| GPS after a cold power-up | found: the PX1120S answers at 115200 after power-up, is identified there, then changes to ~88.4 kbaud during configuration (Witness sends no baud command) and the readback times out. The receiver stayed failed for good. Fixed by re-probing and retrying any failed startup every 10 s; PASS after a cold USB power cycle (retries 1, then ready at 88,403 baud, 25 Hz GGA) |
| Landed low-power mode | GPS doze worked. Found that a light sleep with USB attached killed the USB link for good (fix: 5 s debounce + `CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION`, not bench verified). **The mode is now switched off** (`landed::kLowPowerModeEnabled = false`) |
| Landed with the mode off | PASS: normal loop, LoRa sensors/state/camera at 1 Hz, ping acked UNKNOWN, Iris PONG relayed |

Hardware observations:
- Once after a USB power cycle the bootloader read the eFuse block revision as v3.4 instead of v1.4. It refused every image and boot-looped, and an `espefuse summary` read failed with serial corruption. The next power cycle booted normally.
- This, together with the MAC that reads as `ac:` or `84:`, points at an unstable supply at power-up. Measure the 3V3 rail on Witness.
- After the Pi reboot, passwordless sudo covers only `/usr/sbin/uhubctl -l 2 -a 2` (a full cycle). That is enough to reset the boards.

Open question for the GPS: which configuration command moves the PX1120S from 115200 to 88.4 kbaud. The 25 Hz update rate (`0x0E`) is the prime suspect. To check, send the commands one at a time from the FTDI adapter at 115200.

## Not tested (needs hardware or access this bench doesn't have)

- **Light sleep and current**: needs Witness on battery with USB unplugged (meter
  in series). Includes DIO1 wake reliability, pad hold, NAND after a soak.
- **GPS sleep/wake, 10-min refresh with a real fix, UTC top-of-minute alignment**:
  blocked by the GPS link (bug 2). Landed currently logs "GPS sleep not possible
  (unknown GPS)".
- **Wi-Fi OTA (flight and maintenance upload), rollback after a failed self-test,
  maintenance confirm**: needs a host on the `witness-6025` AP; the desktop's
  Wi-Fi is off limits.
- USB mass-storage export (`0x6868` over USB).
- Refusal codes 0x0001-0x0008 (need a damaged or incompatible maintenance image).
- Iris behaviour after hours without Witness ACKs in Landed.
