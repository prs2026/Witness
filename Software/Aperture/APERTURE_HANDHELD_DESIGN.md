# Aperture as a standalone handheld: feasibility and design

Status: investigation only. No firmware, Horizon or bench changes were made.
Date: 2026-11-12. Sources: `softwareattempt2/main/*`, `Hardware/witnessv0.2/aperture/aperture.kicad_sch`,
`Software/Horizon/witness_protocol.py`, Witness `FIRMWARE_HANDOFF.md` (Wi-Fi OTA section), Adafruit's
ESP32-S3 TFT Feather pinout page. Items marked **(verify)** come from memory of Adafruit's pin
definitions or could not be confirmed from the repo, and need a check against the board/variant header
before code is written.

## 0. Verdict

**Viable, and mostly a firmware job.** The Feather TFT is footprint-compatible with the plain S3
Feather the Aperture PCB was designed for, the TFT's own pins do not collide with any radio or GPS pin,
and the packet decode is small. Four things need attention before it is a good field unit:

1. The wing is documented for the **plain S3 Feather (SKU 5477, no screen)**. A TFT Feather (5483) must be
   swapped in. The radio/GPS GPIO map is unchanged by the swap.
2. **The radio and the TFT share SPI2 pins (SCK 36 / MOSI 35 / MISO 37).** That is legal but
   `Ra01::initialize()` currently owns the whole bus (`spi_bus_initialize(SPI2_HOST, ..., SPI_DMA_DISABLED)`
   and `spi_bus_free` on failure). It must be refactored into a shared bus.
3. **GPIO3 (RF_EN) and GPIO4 (radio CS) are the Feather's I2C SDA/SCL.** The battery gauge, STEMMA QT port
   and any I2C sensor are therefore unusable (and CS toggling clocks the gauge). Battery % on the handheld
   needs a different source (see 1.2).
4. **Transmit from battery**: the 3V3 rail (AP2112, 600 mA) cannot carry +29 dBm TX peaks (~690 mA) on top of
   ESP32 + GPS + backlight. Keep a low drive or add bulk capacitance/own supply for the radio.

Protocol caveat: the Wi-Fi "status query" (`d0 = 2`) does **not exist** in Witness today (see 2.4).

## 1. Hardware

### 1.1 Which board is it, and pin conflicts

- Aperture schematic note (aperture.kicad_sch line ~1873): "Mates to ESP32-S3 feather Adafruit SKU 5477",
  using a HUZZAH32 *symbol* only as a stand-in (its pin names like IO36/IO19 are meaningless for the S3).
  So the intended host is the **plain Feather ESP32-S3 (5477)**, which has no display and no extra buttons.
  The screen requires **ESP32-S3 TFT Feather (5483)** (ST7789 240x135, BOOT + RESET buttons, NeoPixel).
  Same form factor and header pinout, so it is a drop-in swap. A reverse-TFT Feather (3 buttons) would need
  its own pin check; I did not evaluate it.
- Module: Adafruit lists 4 MB flash + 2 MB PSRAM (ESP32-S3-MINI-1 N4R2, quad PSRAM), so GPIO33-37 are not
  taken by PSRAM, which is why SPI on 35/36/37 and BUSY on 34 work. Current `sdkconfig` says
  `FLASHSIZE_2MB`, single-app partition table, PSRAM off: all under-using the part.
- Battery gauge differs by revision: LC709203F (0x0B, early 2023 and earlier) or MAX17048 (0x36, later).
  Check the silkscreen on the board you actually have.

| Function | Aperture today (hardware.h) | TFT Feather usage (verify GPIOs) | Verdict |
|---|---|---|---|
| SPI SCK/MOSI/MISO | 36 / 35 / 37 (radio) | TFT shares the same Feather SPI pins | **Shared bus**, separate CS. Needs driver refactor |
| Radio CS | 4 | I2C SCL (5k pull-up, shared with STEMMA + gauge) | **Conflict** with I2C |
| Radio RF_EN | 3 | I2C SDA | **Conflict** with I2C |
| Radio RESET/BUSY/DIO1/2/3 | 12 / 34 / 10 / 11 / 9 | not used by TFT | free. GPIO34 is not a normal Feather header pin; the PCB routes it, it works today, and 5483 uses the same module so it should be identical (verify on the new board) |
| GPS UART RX/TX | 38 / 39 (Feather RX/TX) | none | free |
| GPS PPS / RESET_N | 18 / 14 | none | free |
| TFT CS / DC / RST | n/a | 42 / 40 / 41 | free, not used by Aperture |
| TFT backlight | n/a | 45 (strapping pin, fine at runtime) | free |
| TFT_I2C_POWER | n/a | 21 (must be driven high for TFT *and* I2C pull-up rail) | free |
| NeoPixel | n/a | 33 | free, usable as link LED |
| BOOT button | n/a | GPIO0 | free at runtime |

Consequences of the I2C overlap:
- Radio CS (GPIO4) pulses look like SCL edges to the gauge/STEMMA devices, and RF_EN (GPIO3) low-level drive
  sinks current through the 5k SDA pull-up. Functionally the radio still works (it does today on the plain
  Feather, which has the same pull-ups and gauge), but **do not talk to I2C on this wing**. If the pull-ups
  are on the TFT_I2C_POWER rail, keeping that rail on (needed for the TFT) means pull-ups are live.
- A magnetometer or other I2C part cannot go on the default bus. If one is ever added, use a bit-banged or
  second I2C on free pins (D5/D6/A1-A3/A5: 5, 6, 17, 16, 15, 8 **(verify routing through the wing)**).
- A PCB respin that moved CS/RF_EN to free pins (e.g. D5/D6) would remove the conflict and free the
  gauge and STEMMA. Worth doing at the next spin; not required now.

### 1.2 Battery voltage (Aperture's own)
The fuel gauge is on I2C (blocked by 1.1). Alternatives: (a) read VBAT through the board's ADC divider if it
exists on this revision **(verify)** - the plain S3 Feather and the TFT Feather route VBAT/2 to an analog pin
on some revisions; (b) accept the I2C overlap and read the gauge anyway during a quiet moment (CS idle,
RF_EN high); I do not recommend that on shared lines; (c) respin. Start with (a); if unavailable, show only
USB/charging state and skip own-battery % in v1.

### 1.3 Input
- Only **BOOT (GPIO0)** is usable at runtime (RESET is not readable). One button gives: short = next page,
  long (0.7 s) = act/confirm, double = previous page. Enough for v1 (see UI).
- Two or three real buttons: wire tact switches to free Feather pins (D5/D6 plus one of A1-A3, active low,
  internal pull-ups) on a small perfboard/cap, or on the next PCB. Rotary encoders/Seesaw need I2C, blocked.
- No touch. Gloves/sunlight: BOOT is small and recessed on the Feather; plan a case with a plunger.

### 1.4 Battery life (ESTIMATES, nothing measured here)
Rough current, 3.3 V rail, receive-only: ESP32-S3 at 160 MHz no Wi-Fi 35-45 mA; SX1262 + Ra-01SH-P front end
in RX 10-20 mA; LC86 tracking ~20-25 mA at 10 Hz (could be 1 Hz); TFT + backlight 10-30 mA depending on level.
Total roughly 80-110 mA -> **a 500 mAh LiPo gives ~4-5 h, 1000 mAh ~8-10 h** (charger efficiency/boost loss
not counted; the Feather has no boost, the LDO drops directly so battery current ~ 3.3 V load current).
Levers: backlight auto-dim/off after 15 s with wake on button or new alarm (largest saver), 80 MHz CPU,
GPS to 1 Hz on battery, Wi-Fi off (it is, Aperture never uses it in the recommended design). Measure on the
bench before promising a number. For a typical pad + recovery day this is fine; tell the user to charge.

### 1.5 Transmit current
Aperture's TX is used rarely: a Wi-Fi-on command, maybe off/ping. Frame is 11 bytes at SF7/125 kHz, ~40 ms
airtime. At the boot default of -17 dBm drive the PA current is small, but so is the range (fine at the pad,
poor on a recovery walk 1 km away; the *downlink* from Witness is unaffected by Aperture's TX power).
At 0..+3 dBm drive the FEM makes ~+29 dBm and draws up to ~690 mA for those 40 ms. With ESP32+GPS+TFT already
~150 mA that exceeds the AP2112's 600 mA. Options, in order of preference: add a 470-1000 uF low-ESR cap
close to the Ra-01SH-P 3V3 pin (rides through 40 ms-ish bursts) and verify with a scope that 3V3 does not dip
below ~3.0 V while the TFT is on; send command repeats at a mid drive (e.g. -5..0 dBm) rather than max;
turn the backlight to minimum during TX; and make the TX drive a stored setting (NVS) rather than the
RAM-only value that resets to -17 dBm every boot. Do **not** leave +3 dBm as a handheld default on a bare
LDO until measured.

## 2. UI design (240x135, landscape)

### 2.1 Display and legibility
- ST7789 is a transmissive/IPS-ish LCD, not e-paper: it washes out in direct sun. Mitigations: black text on
  white or pure high-contrast on black? On a transflective-less IPS panel, **white background / black text at full
  backlight** reads better outdoors than dark themes; offer a "SUN" theme toggled by long press on a Settings
  page, default dark for night/pad. Use only 3 colors + red/green/amber for status; big numerals
  (32-48 px tall), bold sans, no thin fonts, no gradients. A matte screen protector/hood helps more than any UI.
- 135 px height means 3-4 lines of big text or a 2x3 grid of ~20 px items. Page indicator dots at bottom.

### 2.2 Pages (BOOT short = next, double = prev, long = page action)
Always-on top strip (14 px): link icon + age ("LINK 1s", amber at >5 s, red at >30 s or "NO LINK"),
flight state chip (PAD/BOOST/COAST/DESC/LANDED), USB/charging icon. The status strip also shows a flashing
"LANDED" banner the first time the state turns Landed and jumps to the Recovery page automatically.

**P1 Pad (go/no-go glance)** - one 2x3 grid, each cell colored green/amber/red:
- Witness battery V (0x02, payload[60:62] x 0.001) - thresholds to be set by the user (2S? voltage unknown).
- Iris battery V (0x03, payload[15:17] x 0.001); "--" if no 0x03 for >5 s (Iris absent).
- Witness GPS: lock bit (status byte 1 bit3) + satellites (0x02 payload[62]).
- Flight state (status byte 1 bits 0-2); a non-Pad-Idle state on the pad is red.
- Link: RSSI dBm and SNR dB of the last packet, age. (Aperture's SX1262 already reads RssiPkt/SnrPkt in
  `ra01.cpp` ~line 293 but stores only RSSI; store SNR too.)
- Status flags: sensors ready (low-g, gyro, baro bits) and flash init/log failed bits as a single OK / list
  of failing items. Detail screen on long press.
- "Witness last reset": **not transmitted**. The only available proxy is the 0x02/0x03 uptime going backwards
  (detect resets while the handheld is listening) plus uptime shown ("up 12m"). A real reset-cause field would
  need a Witness packet change; do not promise it.

**P2 Aperture/GPS**: own fix quality, sats, HDOP, lat/lon, altitude, UTC (already in `Lc86Nmea::State`), PPS ok,
radio power setting, own battery/USB state, version. Used to confirm the handheld's own GPS before the walk.

**P3 Recovery** (auto-selected on Landed or on long-press from any page):
- Last Witness position: lat/lon (from 0x02, **only 1e-4 deg = ~11 m resolution**, adequate to get within
  visual range, then Witness's buzzer/LED or the camera), GPS alt, age of the last valid 0x02 ("fix 3m 12s old").
  The last packet is persisted to NVS (rate-limited, only when state is Descent/Landed and position changed) so
  a reboot or battery swap in the field keeps it.
- Distance and bearing from Aperture's own GPS (haversine; equirectangular is enough under 5 km).
  Big number: distance (m / km). Compass rose, north-up, with "you" at center and a target dot/arrow.
- **Heading problem:** the Aperture has no compass and the NMEA parser currently drops RMC course/speed. The
  usable, no-hardware solution: add course-over-ground and speed to `Lc86Nmea::State` and draw the *relative*
  arrow (bearing minus COG) when speed > ~1 m/s, otherwise hold the last heading with a "walk to update"
  hint and the north-up bearing in degrees as the reliable readout. A magnetometer is a hardware add with the
  I2C caveat above; not needed for v1.
- Also shows Witness battery, Iris battery, and "last heard" time, which are useful on recovery.

**P4 Wi-Fi / logs**: see 2.4 and section 3.

**P5 Settings (optional)**: backlight level, theme, TX power (stored), GPS rate, Witness SSID suffix (see 2.4),
USB-bridge info.

### 2.3 What to keep from Horizon / what to drop
Keep: battery voltages, GPS lock/sats, flight state text, link RSSI/age, last position + range/bearing (the
"antenna pointer" math in `antenna_pointer_math.py` / `geodesy.py` is the reference for the bearing code),
command acks (0x06) as toasts. Drop: all plots, 3D attitude, orientation/accel/gyro, Kalman/position
vectors, maps and tiles, CSV logs, flight editor, spectrum, MQTT, HITL, ground-receiver multiplexing.
Packet 0x01 sensors needs no decode (status bytes only); 0x07 is not sent by Witness over LoRa anymore;
0x06 acks are worth decoding (Wi-Fi on result: OK/REFUSED/BAD_ARGUMENT/BUSY).

### 2.4 Commands from the handheld (whitelist)
Allow only: Wi-Fi on (0x5746 d0=1; d1 0=default +8 dBm; d2 minutes, suggest 30 for a landing recovery),
Wi-Fi off (d0=0), ping, and Aperture-local radio power. Do not expose set-flight-state, pad heading, GPS cold
start, load switches or mass-storage/erase from buttons. Every transmit needs a long-press confirm and shows
the ack result. The frame builder can be a straight port of `build_wifi_packet` (command layout:
`0x05, d0..d5, opcode hi, opcode lo, 0x0A`; `IRIS_PACKET_COMMAND_FRAME_LENGTH` in comms.h).

Gaps found in the Witness side that the brief assumed:
- **No status query.** `comms.h`, the protocol doc and `build_wifi_packet` define only d0 = 0 off / 1 on.
  `wifi_ota.cpp` has no `d0 == 2` handling. The handheld therefore cannot ask "is Wi-Fi on". Proxies: the
  on-ack (0x06) from Witness, followed by **telemetry going silent** (Witness stops all LoRa TX while Wi-Fi is
  on; ack is sent first, radio cut ~600 ms later), and a local countdown for the auto-off timer. A real
  status query (ack detail = on/off + minutes left + SSID bytes) is a small Witness change if wanted.
- **SSID is not known to Aperture.** AP is `witness-XXXX` (last two MAC bytes) unless
  `CONFIG_WITNESS_WIFI_SSID_OVERRIDE` is set. Easiest: set the override to a fixed name for the flight board, or
  have the Witness on-ack detail carry the two MAC bytes, or let the user enter the 4 hex digits in Settings
  once. Password is `CONFIG_WITNESS_WIFI_PASSWORD` (default `witness-ota-2026`, "change it"), IP 192.168.4.1.
- **Landed gating:** Wi-Fi turns on in Pad Idle or Landed (else REFUSED); `/logs` and `/logs/<name>` are
  refused (409) unless Landed, and the first `/logs` freezes logging until the Witness reboots (Wi-Fi off after
  that reboots Witness). The handheld should therefore only offer "download logs" when the last state was
  Landed and tell the user that logging is frozen afterward.
- `/logs` replies with JSON and there is **no web UI** ("no web UI" in the handoff), so a phone browser sees raw
  JSON. See section 3 for the fix.

## 3. Can Aperture itself download the logs?

Technically yes (ESP32-S3 can be a Wi-Fi STA, join `witness-XXXX`, and HTTP GET `/logs/<name>`), but it
does not fit the hardware:

- Storage: 4 MB flash (app + OTA ~1.5-2 MB), 2 MB PSRAM, **no SD**. The Witness logger writes to a 128 MiB NAND
  and its export files are likely several MB to tens of MB per flight (I did not size them). Aperture can
  hold none of that; LittleFS in the spare flash would be ~1-2 MB and wear flash.
- There would be nowhere to read the files out afterward except USB mass storage (extra work) or a second hop.
- Radio coexistence: STA Wi-Fi adds ~100+ mA, stops the Aperture from showing LoRa (Witness is silent anyway),
  and shares the 3V3 rail. Doable but pointless without storage.
- A microSD wing (SPI on the same bus, own CS on a free pin like D5/D6, and it stacks mechanically with the
  existing wing) would make an Aperture-side download real. That is a hardware add plus FAT/HTTP client work.
  It could also relay to a phone. Not recommended for v1.

**Recommendation: Aperture enables Witness's AP and guides the user; the phone/laptop does the download.**
1. Handheld sends Wi-Fi on (confirm), shows a Wi-Fi page with: SSID, password, `http://192.168.4.1/logs`,
   a countdown ("auto-off in 28:40"), "Witness is silent in Wi-Fi mode (expected)", and a **QR code**
   (`WIFI:T:WPA;S:witness-xxxx;P:...;;`). A version-3 QR (29x29 modules at 4 px = 116 px) fits the 135 px
   height. The phone joins by scanning; the laptop uses `witness_ota.py logs / get all`.
2. Small Witness-side change (not done here): serve a minimal HTML index at `/` or `/logs?html` with plain
   links to `/logs/<name>` so a phone can tap-download without scripts. Without that, use the laptop tool.
3. After download, send Wi-Fi off from the handheld (Witness reboots to thaw logging) or let auto-off run.
If the phone/laptop is not available, the handheld shows the LANDED coordinates and can still be used to find
the rocket; log download is a second step at the car.

## 4. Software architecture (ESP-IDF 6.1.0, ESP32-S3)

### 4.1 Rendering: LVGL vs custom
- 135x240x16 bpp = 64.8 KB full frame: fits internal RAM (no PSRAM needed). With a dirty-rect/strip push (e.g.
  16 lines = 7.7 KB DMA buffer, ~1.5-2 ms at 40 MHz SPI) the TFT cost to the system is tiny.
- **LVGL** (v9 via `esp_lvgl_port`): fast to prototype, fonts/anti-aliasing/themes, but adds a few hundred KB
  of flash, ~40-80 KB RAM, its own task/timer, and a second place for threads to touch the SPI bus. Overkill
  for 5 mostly-numeric pages and one arrow.
- **Custom minimal renderer** (rect/line/circle/text with embedded 1-bit or 4-bpp bitmap fonts at 2-3 sizes,
  triangle fill for the arrow, QR via a small public-domain encoder such as nayuki's QR-Code-generator C++
  or `qrcodegen`), drawn into a 16 bpp canvas and pushed with `esp_lcd` panel IO. It matches this repo's style
  (pure C++ modules with host tests, like `lc86_nmea` and `ground_station_status`), and the same renderer can
  be compiled on the PC to dump PNGs for UI review without hardware.
- **Recommend custom + `esp_lcd` ST7789.** In IDF 6.x the ST7789 panel driver is a managed component
  (`espressif/esp_lcd_st7789`) rather than part of IDF **(verify)**; add it via `idf_component.yml`. Keep LVGL
  as the fallback if the UI grows.

### 4.2 Memory/flash budget
Set `ESPTOOLPY_FLASHSIZE_4MB`, keep a custom `partitions.csv` (nvs 24 KB for the persisted last fix and
settings, one app ~512 KB-1 MB; add `ota_1` later if OTA of the Aperture is desired). Current app is small
(Wi-Fi component is compiled in via `CONFIG_ESP_WIFI_ENABLED=y` but unused). PSRAM is not needed; leave it off to
save power and the extra init. Fonts in flash as `const` tables (~20-60 KB depending on sizes).

### 4.3 Tasks and not disturbing the bridge
Today `app_main` is a single 10 ms loop (`CONFIG_FREERTOS_HZ=100`, `vTaskDelay(1)`) that does USB RX, command
parse, status frame, LoRa poll and USB write; GPS is a separate task. Plan:
- Keep that loop as the "bridge" and treat it as the highest priority application task. Add nothing slow to it.
  The only new work in it is `link.feed(packet, len, rssi, snr, now_ms)` (decode, copy under spinlock - same
  pattern as `Lc86Gps::state()`), and `ui_commands` dequeue (see below).
- New `ui` task, low priority (below GPS), 10-20 FPS only when dirty: reads snapshots of `LinkModel` and
  `Lc86Nmea::State`, runs the button state machine, redraws, and sets the backlight PWM (LEDC).
- Buttons-originated TX goes through a small queue handled by the bridge loop, which owns the radio. This
  avoids two tasks calling `Ra01::transmit` (the existing radio object is not thread-safe).
- SPI: one `spi_bus_initialize(SPI2_HOST, ..., SPI_DMA_CH_AUTO)` in a shared `board_spi` init (called before
  both radio and TFT), `Ra01` only does `spi_bus_add_device`. Radio transactions are short polling; TFT
  strips are DMA. ESP-IDF serializes per-transaction on the bus, so a 2 ms strip delays at most one radio
  command. Radio BUSY/NSS handling stays as is. Test with a worst-case 1 Hz packet burst to confirm no RX loss
  (SX1262 buffer holds a packet until read; polling at 100 Hz is far faster than the 1 Hz traffic).
- **Existing risk that becomes real on battery:** `usb_serial_jtag_write_bytes(..., 100 ms)` blocks when no host
  is reading. With no laptop the 512 B TX buffer fills and *every* subsequent write stalls 100 ms, stealing up
  to ~0.3 s of each second from the bridge loop (RX poll and command TX). Fix: only write when
  `usb_serial_jtag_is_connected()` is true or use a 0 timeout and drop on full. This must be done in phase 1.
  Console logging (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`) has the same property.

### 4.4 Decoder reuse
Write `witness_packets.{h,cpp}` as pure C++ using `comms.h` offsets (not magic numbers): fields needed are
state status bytes, uptime, lat/lon (int24 x 1e-4 deg), GPS alt (float32), battery (u16 x 1 mV), sats (u8)
from 0x02; Iris battery from 0x03; ack from 0x06 (verify the actual 0x06 layout in comms.h; the Aperture ack code
uses a 13-byte frame with opcode, echoed data, ack code and detail). Test against vectors generated from
`Horizon/witness_protocol.py` / `test_data/example_packets.json` (same host-test approach as
`tests/gps_status_test.cpp`). Note `comms.h` currently exists as four copies (Aperture, Iris, Witness,
Horizon), all modified in the working tree; verify they agree before porting, and consider a shared
component directory (`EXTRA_COMPONENT_DIRS`) for comms.h + decoder so Witness/Aperture cannot drift. Frame
length is variable in the wild (Horizon accepts 62/63-byte state payloads and 21/23-byte sensor payloads), so
the decoder should tolerate length variants like the Python one.

### 4.5 USB bridge mode must stay intact
The handheld features are additive. When USB is connected the bridge, byte-for-byte echo, 0xF0 status frame
and ground radio-power command remain exactly as now; the UI shows a "USB BRIDGE" strip and **disables
button-originated TX while the host has sent bytes in the last N seconds** (so Horizon owns the link). The
UI should be a build-time feature (`CONFIG_APERTURE_HANDHELD_UI`) so the plain Feather (5477) still builds the
bridge-only firmware. LoRa frames are forwarded unchanged to USB regardless of the UI decode.

## 5. Phased plan, effort, risks

| Phase | Work | Effort |
|---|---|---|
| 0 Bench check | Put the firmware on a 5483, confirm radio + GPS still work (pins), confirm 4 MB flash / BOOT button / TFT power rail; scope 3V3 on a TX; measure idle current | 0.5-1 d (needs hardware) |
| 1 Bridge hardening | Shared SPI bus refactor; DMA on; USB write when disconnected fix; store SNR; 4 MB flash + partitions; NVS | 1-2 d |
| 2 Screen bring-up | `esp_lcd` ST7789 (CS 42, DC 40, RST 41, BL 45, I2C power 21) **(verify)**, renderer, fonts, backlight PWM, host PNG preview | 2-3 d |
| 3 Decoder + Pad page | `witness_packets` + tests, link model, P1/P2, BOOT button state machine, auto-dim | 2-3 d |
| 4 Recovery page | RMC course/speed in parser, distance/bearing, compass UI, persisted last fix | 2 d |
| 5 Wi-Fi page | Whitelisted command TX, ack handling, countdown, QR, SSID entry; optional Witness `d0=2` status + HTML `/logs` index | 2-3 d on Aperture, +1-2 d Witness |
| 6 Field polish | Battery/thermal measurements, sunlight theme, case + button, brownout tuning, long soak | 2-3 d |
Total roughly **2-3 weeks part-time** for the full set; Phases 0-3 (a working pad checker) about one week.

Risks: (1) shared SPI regression in the radio path (mitigate with phase 1 test and RX counters); (2) 3V3
brownout on TX with the screen on; (3) battery life shorter than hoped (measure early); (4) outdoor
readability of an LCD in sun; (5) 11 m position quantization and missing heading make the arrow imprecise;
(6) assumptions about Witness behavior (Wi-Fi status, SSID, Landed gating, Wi-Fi not bench-verified per its
own handoff: "Not bench verified (written without hardware)"); (7) `comms.h` drift between copies;
(8) IDF 6.1 managed-component availability for the ST7789 driver; (9) Wi-Fi and log download untested with a
real board, and any log freeze requires a Witness reboot to resume flight logging, which is a flight-safety
procedure point.

## 6. Open questions for the user
1. Do you have, or will you buy, the 5483 (S3 TFT Feather) and are you OK being limited to one button, or do you
   want two or three external buttons / a respin that moves radio CS and RF_EN off I2C?
2. Which LiPo size, and what runtime target (pad only vs a full recovery day)? Charge from USB in the field?
3. Is a compass-free arrow (course-over-ground plus north-up bearing) acceptable, or should a magnetometer be added?
4. Is an 11 m position resolution acceptable, or should Witness send higher-resolution coordinates (needs a
   Witness/protocol change)?
5. Wi-Fi: set a fixed SSID override and change the default password? Add a `d0 = 2` status query and an HTML log
   index on Witness? Phone or laptop for downloads?
6. What battery voltage thresholds (green/amber/red) for Witness and Iris, and which chemistry/cell count?
7. Should the handheld ever transmit at higher power, and if so do you accept adding a bulk cap/own supply for the radio?
8. Should the handheld keep the plain Feather bridge build alive (build-time switch) for the pad box?
