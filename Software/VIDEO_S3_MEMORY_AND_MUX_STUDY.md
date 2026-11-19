# ESP32-S3 memory options for the video path, and two-camera switching with the TVP5151

Date: 2026-11-17. Research and write-up only; no code, hardware or bench touched. Follows `Software/VIDEO_S3_COMPUTE_STUDY.md` (sections 3, 6, 12) and `Software/VIDEO_DOWNLINK_STUDY.md`.

Tags: **[V]** read from a primary or vendor document this session. **[S]** secondary (forum, distributor snapshot, search-result summary). **[E]** my estimate or reasoning, not verified.

Terminology: the TVP5151 is a **video decoder** (analog NTSC/PAL/SECAM in, digital 8-bit ITU-R BT.656 or 4:2:2 with discrete syncs out), not an encoder [V, TI datasheet SLES241E]. "Encoder" in this report means the JPEG compressor running on the MCU.

## 1. Summary of recommendations

1. **Video runs on a separate ESP32-S3 board ("Iris v2"), not on the Witness flight MCU.** The flight MCU stays an ESP32-S3FN8 with no PSRAM.
2. **For the video board choose option (a): an octal-PSRAM part, ESP32-S3R8 (chip) or ESP32-S3-WROOM-1-N8R8 (module)**, enable PSRAM ECC (85 C instead of 65 C, costs 1/16 of the PSRAM), and still design the capture path to work from internal SRAM line buffers so PSRAM is a convenience, not a dependency.
3. **Option (b), external quad PSRAM beside the S3FN8's in-package flash, is not recommended.** No Espressif-documented wiring or IDF recipe exists; forum reports of trying GPIO26/CS1 on an FN8 failed [S]; a quad 80 MHz bus peaks near 40 MB/s and is shared with flash code fetches. If a no-pin-change PSRAM variant is wanted, the real part is **ESP32-S3FH4R2** (4 MB flash + 2 MB in-package quad PSRAM, -40 to 85 C), see 2.1.
4. **Camera switching: start with the TVP5151's own two-input mux (AIP1A/AIP1B, register 00h bit 0), no external parts.** Budget about 6 fields (TI FAQ, secondary) to roughly 0.2-0.3 s (forum measurements with unsynchronised sources) of lost video per switch. If that is too slow or crosstalk is too high, use **two TVP5151 on one bus with I2C-controlled output tri-state**. External analog switches do not shorten the re-lock, so they are not recommended unless more than two cameras are needed.

## 2. ESP32-S3 memory configuration

### 2.1 Real part numbers (datasheet v2.2 comparison table [V])

| Part number | In-package flash | In-package PSRAM | Ambient | VDD_SPI | Status / note |
|---|---|---|---|---|---|
| ESP32-S3 | none | none | -40 to 105 C | 3.3 / 1.8 V | external flash and PSRAM |
| **ESP32-S3FN8** | 8 MB quad | none | -40 to 85 C | 3.3 V | Witness today (`hardware_pins.h` header comment) |
| **ESP32-S3R8** | none | 8 MB **octal** | **-40 to 65 C** (85 C with PSRAM ECC, -1/16 capacity) | 3.3 V | needs external flash |
| ESP32-S3R8V | none | 8 MB octal, 1.8 V | -40 to 65 C | 1.8 V | **end of life** |
| ESP32-S3R16V | none | 16 MB octal, 1.8 V | -40 to 65 C | 1.8 V | needs 1.8 V flash |
| ESP32-S3RH2 (was S3R2, EOL) | none | 2 MB quad | -40 to 105 C | 3.3 V | needs external flash |
| **ESP32-S3FH4R2** | 4 MB quad | 2 MB quad | -40 to 85 C | 3.3 V | QFN56 7x7, bigger exposed pad than the other variants |

The names "S3R8" and "S3FN8" are the real orderable chips. "S3R8V" is EOL. Modules (secondary sources): ESP32-S3-WROOM-1-N8R8 (8 MB quad flash + 8 MB octal PSRAM, -40 to 65 C), ESP32-S3-WROOM-1-N16R8, and ESP32-S3-WROOM-1-N4R2-class parts [S]. There is no "WROVER" in the S3 family that I found; WROOM-1 carries the PSRAM variants. Chip-level snapshot prices/stock (LCSC, [S], verify live): S3FN8 about $3.56 at qty 1 (1554 in stock), S3R8 about $2.99 (1548 to 18394 in stock depending on snapshot), S3FH4R2 about $2.83-4.00 (2090-6552 in stock).

### 2.2 Pin impact [V datasheet Table 2-14 and GPIO notes; Witness map from `hardware_pins.h`]

- **In-package flash/PSRAM occupy GPIO26-32** (SPICS1 = GPIO26 = PSRAM CS1, SPIHD/SPIWP/SPICS0/SPICLK/SPIQ/SPID = GPIO27-32; CS0 is flash). Do not use them.
- **Octal PSRAM (S3R8) additionally uses GPIO33-37** (DQ4-DQ7 and DQS). The datasheet says these can be ordinary GPIO only if the chip is not in 8-line SPI mode with flash or PSRAM [V].
- **Witness uses all five of GPIO33-37**: LED red 33, LED green 34, LSM chip select 35, CAN RX 36, CAN TX 37. An S3R8 swap on Witness would force a respin moving the IMU chip select, the CAN pair and two LEDs. With S3FN8 or S3FH4R2 the pin map is unchanged.
- Witness also needs an external flash with S3R8 (it uses the in-package one today).
- Usable GPIO: S3 has 45 GPIO (0-21, 26-48). Minus 26-32 gives 38; minus 33-37 gives 33; minus USB 19/20 gives 31 [E]. The video board needs about 30 (8 data, PCLK, VSYNC, AVID/DE, optional HSYNC, I2C 2, decoder PDN/RESET 2, radio SPI/CS/BUSY/DIO/RESET about 8, CAN 2, camera-select, LEDs, UART), so an S3R8 video board is possible but tight; use an I/O expander or drop optional signals if needed [E].
- GPIO0, 3, 45, 46 are strapping pins; 47/48 run at 1.8 V on the "V" parts only.

### 2.3 Capture buffers and what needs PSRAM

TVP5151 gives 1440 bytes per active line (720 pixels x 2 bytes, YUV 4:2:2), AVID-gated, one byte per 27 MHz SCLK [V, TVP5151/5150 datasheets]. NTSC is 240 active lines per field, PAL 288 [E, standard values].

| Buffer | Bytes | Fits internal SRAM? |
|---|---|---|
| 720x480 NTSC full frame | 691,200 | No |
| 720x576 PAL full frame | 829,440 | No |
| 720x240 NTSC field | 345,600 | No (the S3 has 512 KB total, and IDF, caches, radio and stacks use most of it) [E] |
| 720x288 PAL field | 414,720 | No |
| 360x240 decimated field | 172,800 | Yes, on a dedicated video board (about 250-300 KB free heap with 32 KB caches and Wi-Fi off [E]) |
| 320x240 decimated field | 153,600 | Yes |
| JPEG output (4-80 KB per frame) | 4-80 KB | Yes |
| Radio TX queue (about 100 x 511 B) | about 52 KB | Yes |

So a **field buffer at 720 wide needs PSRAM**; any 320/360-wide decimated buffer does not. A no-PSRAM design is: GDMA drops AVID-gated lines into a ring of two 8-16 line chunks (about 23 KB) in internal SRAM, the CPU decimates each chunk 2:1 into a 154-173 KB buffer in internal SRAM, then JPEG encodes it (section 3 of the compute study: 22-25 ms). It is unproven on the S3 (custom LCD_CAM/GDMA driver) [E].

Another no-PSRAM idea for full-width pictures at 0.5 fps: capture one 30-line strip of the 720-wide picture per field (the TVP5151's AVID vertical window or a line counter), encode each strip as an independent small JPEG (also the right loss-tolerance structure), assemble the frame from 8 successive fields. Cost: up to 8 fields (133 ms) of temporal skew across the picture, visible as strip misalignment on fast motion [E].

### 2.4 Bandwidth comparison

| | (a) Octal PSRAM, S3R8 | (b) External quad PSRAM on S3FN8 | (c) S3FH4R2 in-package quad PSRAM | no PSRAM (SRAM only) |
|---|---|---|---|---|
| Bus | 8 data lines DDR, 80 MHz: 160 MB/s peak [S, forum arithmetic] | 4 data lines SDR, 80 MHz: about 40 MB/s peak [E, my arithmetic] | same as (b) | none |
| Measured sustained | 40-85 MB/s, contention-dependent [S] | not measured; typically well under peak because of command/dummy cycles [E] | same | n/a |
| vs DVP burst 22.7 MB/s average (27 MB/s within a line) | 25-70 % of measured bandwidth; works with margin if the CPU stays out of PSRAM during the 15 ms burst [E] | marginal to not feasible: burst alone is 55-70 % of peak, before flash fetches [E] | same as (b) | DMA into SRAM is unaffected |
| DMA to PSRAM | Espressif: bandwidth for DMA to external RAM "very limited", especially when a core accesses it [V] | same | same | n/a |
| Sharing | same MSPI controller and clock/data lines as the external flash [V, Table 2-14 shows shared SPICLK/DQ0-3] | shares SPICLK/SPID/SPIQ/WP/HD with the in-package flash; own CS1 on GPIO26 [V pin table; board wiring E] | in-package, same | flash only |
| Supported by IDF | yes (octal 80 MHz, 120 MHz is experimental and can crash if temperature drifts) [V] | **No documented support for adding PSRAM to an FN8**; IDF supports only Espressif-branded PSRAM, voltage must match flash; forum reports of CS1 on FN8 not detected or causing MD5 errors [S] | yes (standard SKU) | yes |
| Field-skipping eases it | yes: 1 field in 12 (5 fps) means a 15 ms burst per 200 ms (7.6 % duty), 0.76 % at 0.5 fps | yes | yes | n/a |

Field-skipping and sequencing (capture, then decimate/encode with no concurrent PSRAM traffic) is what makes PSRAM capture safe; it is independent of which PSRAM type is chosen.

### 2.5 Effect on flight firmware (Witness) and why video must be a separate S3

Facts about Witness: S3FN8, in-package flash, no PSRAM, flight build now `-Os`, 32 KB icache, 16 KB dcache, 240 MHz, about 42 KB free heap, core 1 limited by instruction-cache misses (your bench result; the committed `sdkconfig` in the tree is an older 160 MHz / 16 KB icache / 32 KB dcache build, so I used your numbers) [user + repo].

How PSRAM would change that picture [mix of V and E]:
- **Flash-code fetches and PSRAM data share the same MSPI0 bus and the same cache window** ("External RAM shares the same cache region as external flash" [V]; pins shared per Table 2-14 [V]). Every PSRAM cache miss or camera DMA burst can stall an instruction-cache refill from flash. With core 1 already limited by icache misses, adding PSRAM traffic makes flight-task timing worse and harder to predict [E]. Camera DMA into internal SRAM does not use that bus at all, so the no-PSRAM FN8 camera path leaves flash fetch bandwidth untouched.
- **Cache vs heap:** the cache sizes come out of the 512 KB internal SRAM; going from 16 KB to 32 KB dcache (needed to make PSRAM data accesses tolerable; Espressif recommends bigger caches with PSRAM) costs 16 KB of heap, and 64 KB costs 48 KB, on a 42 KB heap [E]. PSRAM would also add its own driver/DMA-reserve internal allocations (CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL, default tens of KB) [V option exists, default size E]. On the plus side, large buffers (more than 16 KB by default) move to PSRAM, freeing internal heap [V option].
- **Flash writes stall PSRAM:** when flash cache is disabled (code-flash writes, OTA, NVS) external RAM is inaccessible [V]. A camera DMA to PSRAM during such a window would overrun. Flight code that does OTA or NVS writes would need the video stream stopped, which is another coupling.
- **Contention with GDMA/CAN/LoRa timing:** the flight loop's deadlines depend on cache hits; a 22.7 MB/s DMA stream plus encoder CPU load is the largest new variable [E].
- **Pins:** the S3R8 needs the GPIO33-37 move (2.2).
- **Thermal/temperature:** S3R8 is 65 C ambient (85 C with ECC), versus 85 C for the FN8 [V].

**Recommendation: do not put video on the Witness MCU.** Keep Witness an FN8 without PSRAM, talk to a separate video S3 over CAN (control, status, GPS time) as the study already assumed.

### 2.6 Option comparison and choice

| | (a) S3R8 / WROOM-1-N8R8 | (b) S3FN8 + external quad PSRAM | (c) S3FH4R2 |
|---|---|---|---|
| PSRAM | 8 MB octal in package | 2-8 MB quad on shared bus, unsupported | 2 MB quad in package |
| Flash | external quad (add chip, 3.3 V) | in-package 8 MB | in-package 4 MB |
| Pins lost | 26-32 and 33-37 | 26-32 (shared bus plus GPIO26 as CS1) | 26-32 only |
| BOM add | external flash (about $0.5-1) | PSRAM chip (Espressif-branded only) | none |
| Temp grade | 65 C (85 C with ECC) | 85 C | 85 C |
| Availability | good [S] | n/a | good [S] |
| Power | PSRAM variants draw more than non-PSRAM [V datasheet note] | similar | similar |
| Capture options | PSRAM field buffer or SRAM ring | SRAM ring only in practice | 346 KB field fits 2 MB PSRAM but bandwidth marginal |
| Risk | respin if reused on Witness; 65 C limit | **high** (no recipe) | low-medium; footprint EPAD differs; 4 MB flash may be small for two images |

**Choice for the video board: (a).** It is the only well-supported way to get a lot of fast PSRAM; the capture path can still be SRAM-only. Mitigate the temperature limit with ECC (85 C, 7.5 MB usable) and by keeping the board off the hot side of the avionics bay. **If the video board must share Witness's footprint and pin map without a respin, use (c) S3FH4R2** (and design the capture through SRAM). **Do not use (b).**

Extra choice if PSRAM is not needed: the plain ESP32-S3FN8 as today with the SRAM-only 320x240/360x240 path, which is the lowest-risk option for 5 fps and also acceptable at 0.5 fps if 360x240 q85-90 is enough (the 720x240 field at q75 needs the strip or PSRAM route).

### 2.7 TVP5151 facts that affect the S3 pin plan [V, TI SLES241E]

- 27.000 MHz crystal (+/-50 ppm) or a 1.8-3.3 V external oscillator into XTAL1 or AVID/CLK_IN; **IO_DVDD 1.8-3.3 V**, core 1.8 V; 138 mW typ, 185 mW max, <1 mW power-down; TQFP-32 (PBS) or 48-PBGA; commercial 0-70 C or industrial -40 to 85 C (TVP5151I) [V]. TI product page: Active but "no ongoing design support for new projects" [V].
- **AVID is not enabled by default on the TVP5151:** pin 26 is AVID/CLK_IN and defaults to CLK_IN; **register 05h bit 1 = 1 selects AVID** [V]. The pin is an input at reset ("AVID/CLK_IN is input during RESETB"), so it needs a pull-down and must not be driven by the S3 DE input pad (use a series resistor or hold the S3 pad as input) until the register is written. Different from the TVP5150AM1 where AVID is output after enabling register 03h [V].
- Pin-compatible with the TVP5150AM1 apart from the clock source (27 MHz here, 14.31818 MHz on the AM1) and the I/O range [V].
- Same I2C addressing (I2CSEL on YOUT7: 0xB8/0xBA write) [V]; same register map for output formatter, sync enables and AVID crop, with the AVID-select addition above (as read in the TVP5150AM1 datasheet in section 12 of the compute study; the TVP5151 has identical register 03h/0Dh/11h-14h/18h/19h structure as listed in its register map) [V for the register list, E for identical behaviour].
- Output timing: propagation delay 3-8 ns after SCLK, measured with 22 ohm series resistors and 10 pF load [V]; add 22 ohm series resistors on YOUT/SCLK/AVID/HS/VS at the decoder for EMI/overshoot and sample on the falling edge or invert PCLK on the S3.
- TVP5151 temperature: Industrial grade is orderable (TVP5151IPBS/IPBSR, -40 to 85 C) [V].

## 3. Switching between two cameras

### 3.1 The TVP5151's own input mux [V unless noted]

- The chip has "an analog input channel that accepts two video inputs that are ac-coupled": **two selectable composite inputs, or one S-video** [V]. Selection is **register 00h bit 0: 0 = AIP1A (default), 1 = AIP1B**; bits [1] and the S-video setting must be 0 for composite [V]. Control is plain I2C, no hardware pin.
- **There is one front end** (clamp, PGA/AGC, ADC, line-locked PLL, comb filter). Selecting the other input does not keep two sources locked. After a switch the decoder must re-clamp, re-run AGC, re-lock the horizontal PLL to the new sync phase, re-detect the standard (if autoswitch is on) and re-acquire colour burst/vertical sync [E, from architecture].
- **The datasheet gives no switch or re-lock time and no input-to-input crosstalk spec** [V: neither is in the extracted text]. Secondary evidence: a TI TVP5150A FAQ puts re-lock after an input change at about 6 fields (about 100 ms at 59.94 fields/s) with fast lock enabled by default; a hobbyist measured 170-260 ms before a usable frame when the two sources were out of phase (170-180 ms when nearly in phase); another forum thread reports corrupted frames when switching with an external mux during acquisition; the related TVP5154 datasheet says fast lock is 3 fields or fewer for clean inputs when the standard is known [S, e2e threads, not independently opened: TI's forum returned 403 to my fetch, so these are search-result summaries].
- **Consequences at our frame rates:** 6 fields is 100 ms (0.5 frame at 5 fps) up to 260 ms (1-2 frames at 5 fps, a fraction of a frame at 0.5 fps). With two unsynchronised RunCam outputs expect the upper end [E]. At 0.5-5 fps this costs about 1-2 frames per switch and is acceptable for a mission-event switch (not continuous alternation).
- **Making it cleaner [E, uses documented registers]:** force the video standard (NTSC) and mask autoswitch (register 04h autoswitch mask) to remove standard-detect time; use the "black output" bit (register 00h bit 4) or discard the first N fields after a switch until Status register #1 shows lock (registers 87h-8Bh are status registers [V]); read the lost-lock bit; leave AGC enabled but allow settling.
- **Termination and levels [V]:** both inputs need an ac-coupling cap (0.1 uF), a recommended parallel termination of 75 ohm at the input, a maximum input of 0.75 Vpp sync-to-chroma amplitude (so a 1.5 Vpp source needs about 1/2 attenuation), the reference design's 37.4 ohm resistors in each input network, and a recommended anti-aliasing filter (AAF) and ESD protection on each analog input (Figure 6-1). An unused input is tied to AGND through 0.1 uF.
- **Impedance/crosstalk pitfalls [E]:** keep the unselected input's source terminated (75 ohm at the camera end) so it does not float or ring; route the two inputs apart; check crosstalk on the bench with one input carrying a colour-bar signal and the other grounded.

### 3.2 External options

Data is from distributor and aggregator pages [S]; switch/mux datasheets were not opened (only search-result summaries).

| Option | Bandwidth, DG/DP | Supply, package, control | Cost/availability | Glitch and lock | Termination/buffer |
|---|---|---|---|---|---|
| **TS5V330** (TI quad 2:1 video switch; "C" version 250 MHz) | 300 MHz min; DG 0.64 %, DP 0.1 deg; Ron about 3 ohm typ (10 ohm max) | 4-5.5 V (5 V only); 16-SOIC/TSSOP/SSOP/VQFN; one select pin (S) and enable (OE) | DigiKey lists SOIC/TSSOP variants; price not retrieved | Analog transient is sub-microsecond; decoder re-lock is unchanged (same two unsynchronised sources) | 75 ohm termination at each camera end; ac-couple after the switch; switch is passive (no gain) |
| **ADG794** (ADI low-voltage 300 MHz quad 2:1 mux) | 300 MHz; crosstalk about -70 dB, off-isolation about -65 dB at 10 MHz; Ron low; DG not retrieved | single 3.3 V or 5 V, TTL-compatible logic, 16-QSOP; break-before-make; tON 7 ns / tOFF 5 ns | Mouser listing exists; price not retrieved | same | same as above; 3.3 V logic matches the S3 |
| **TMUX1574** (TI quad SPDT) | Ron about 2 ohm, Con about 7.5 pF; bandwidth/crosstalk not retrieved | 1.5-5.5 V, powered-off protection to 3.6 V; select pin | available (TI) | same | same |
| **MAX4310/4311/4312** (Maxim/ADI 2/4/8-channel video mux-amp) and fixed-gain MAX4313-4315 | DG about 0.06 %, DP about 0.08 deg (garbled extraction); settling 25-42 ns | single supply (5 V tables, down to +4 V); SO/uMAX; pin or logic address select | ADI product pages exist; lifecycle not checked (older Maxim parts may be NRND) [E] | same | integrated buffer drives a 75 ohm back-terminated cable (use the +1 version to drive the decoder through its 75/37.4 network; the +2 versions need 75 ohm series and double the level) |
| **Second TVP5151 + digital selection** | n/a | 1.8 V + 1.8-3.3 V I/O as in 2.7; second I2C address by I2CSEL; shared 27 MHz oscillator into CLK_IN (supported [V]) | one more TQFP-32 and about 138 mW; chip price about $4-9 for the AM1 family, TVP5151 not checked | **Both decoders stay locked continuously.** Switch = I2C writes that tri-state one decoder's YCbCr/sync/SCLK outputs (register 03h bits 0, 2, 3 [V]) and enable the other's: under 1 ms of I2C plus up to one field (16.7 ms) to the next VSYNC. Frames lost: about 1 field. | each decoder keeps its own AAF/75 ohm network as above |

Notes:
- An external analog switch only changes where the switching happens; **it does not shorten the decoder's re-lock** because the decoder still sees a sudden sync-phase and burst change. The only real improvements are cleaner timing control, ability to select more than two cameras, and, for the mux-amps, a buffered/gain-corrected signal.
- Genlocking the two cameras (so a switch causes no sync jump) is not possible with the RunCam analog outputs [E].
- Break-before-make (ADG794) prevents the two cameras from being shorted together during the switch [S].
- A single-supply 5 V part such as the TS5V330 needs a 5 V rail on the video board (it already has the 5 V boost for the radio per the downlink study), with 3.3 V logic into a 5 V part needing check of VIH [E].

### 3.3 Recommended circuit approach

1. **Default: no external mux.** Camera 1 and camera 2 composite outputs go to separate input networks (75 ohm termination at the decoder, series 37.4 ohm/ac-coupling 0.1 uF/anti-alias filter per the TVP5151 reference design, plus a TVS), then to AIP1A and AIP1B. The S3 selects the camera with an I2C write to **register 00h bit 0**, then:
   - writes the autoswitch mask (04h) so only NTSC is allowed, and forces NTSC if the camera is known;
   - drops the next N fields (start with 12, tune on the bench) or until the lost-lock status clears;
   - re-asserts the AVID/DE and VSYNC capture setup if the S3 DMA aborted (arm only on VSYNC).
   Use a GPIO "camera select" line purely as a software flag or to control a camera power switch if the unused camera is to be powered down.
2. **If switch time must be under about 2 fields or crosstalk fails on the bench:** fit **two TVP5151 on one 8-bit bus**, one at I2C address 0xB8 and one at 0xBA, one 27.000 MHz oscillator feeding both CLK_IN pins, both with 22 ohm series resistors on the outputs; the host keeps one decoder's outputs enabled and the other's high-impedance (register 03h; disable first, then enable, to avoid contention). The unused decoder can be put in power-down (PDN) if the extra 138 mW matters, at the cost of full re-lock time on switch-back.
3. **If three or more cameras are ever needed:** add an ADG794 or TS5V330 (select by GPIO) in front of the AIP input, accepting the re-lock time of 3.1.
4. GPIO/I2C summary: I2C (SCL/SDA, 400 kHz max [V]) for decoder registers; one GPIO for decoder PDN, one for RESETB, optional GPIO for camera power or select; PCLK/VSYNC/AVID/data as in the compute study section 12.

## 4. Things I could not verify

- The ESP32-S3 TRM (LCD_CAM chapter) and the TVP5151's own register chapter for the AVID vertical window behaviour were not read in full; the TVP5151 AVID/CLK_IN selection (05h bit 1) and pin facts are from the datasheet text I extracted.
- No published switching/re-lock time for the TVP5151 input change; the 6-field and 170-260 ms numbers are from secondary summaries of TI forum threads (the forum pages themselves returned 403).
- S3R8 module temperature grades and WROOM-1 variants differ across datasheet revisions; chip-level temperature is from the v2.2 datasheet (65 C, 85 C with ECC) [V]; module-level figures are [S].
- Whether ESP-IDF can run an external quad PSRAM on a pin-shared bus with an S3FN8: not documented; forum reports say no [S]. ECC mode cost is stated by the datasheet [V], not measured.
- Switch/mux datasheets (TS5V330, ADG794, TMUX1574, MAX4310 family) were not opened; figures come from search-result summaries of their datasheets [S]; differential gain/phase, crosstalk and switching times for TMUX1574 were not retrieved.
- Prices and stock are snapshots; no live check at DigiKey/Mouser/LCSC was possible for the switches.
- The 50-85 MB/s PSRAM bandwidth figure is from forum posts, not an Espressif benchmark. Camera-DMA contention with instruction-cache fetches is reasoned from the shared MSPI bus and Espressif's "DMA to external RAM is very limited" statement, not measured.
- The Witness committed `sdkconfig` (160 MHz, 16 KB icache, 32 KB dcache) differs from the -Os/32 KB/16 KB/240 MHz build described by the owner; only `sdkconfig.flight` sets -Os in the tree.

## Sources

- TI TVP5151 datasheet SLES241E (inputs, AVID/CLK_IN, 27 MHz clock, power, register 00h/05h, application circuit): https://www.ti.com/lit/ds/symlink/tvp5151.pdf [V]; product page https://www.ti.com/product/TVP5151 [V]
- TI TVP5150AM1 datasheet SLES209E (register 03h/0Dh/11h-14h/18h/19h, AVID): https://www.ti.com/lit/ds/symlink/tvp5150am1.pdf [V]
- ESP32-S3 Series Datasheet v2.2 (Table 1-1 comparison, Table 2-14 flash/PSRAM pin mapping, GPIO26-37 notes, S3R8 ECC/temperature): https://documentation.espressif.com/esp32-s3_datasheet_en.pdf [V]
- ESP-IDF Support for External RAM (DMA to external RAM, shared cache region, flash-cache-disabled behaviour, config options, supported PSRAM chips): https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-guides/external-ram.html [V]
- ESP-IDF SPI flash and external SPI RAM configuration (quad PSRAM STR only, octal DTR only, 120 MHz experimental): https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-guides/flash_psram_config.html [V]
- Forum: PSRAM on ESP32-S3FN8 (GPIO26 CS1 not working): https://esp32.com/viewtopic.php?p=110084 , https://esp32.com/viewtopic.php?p=113080 , https://esp32.com/viewtopic.php?p=110198 [S]
- Forum: octal PSRAM bandwidth measurements: https://esp32.com/viewtopic.php?p=112250 , https://esp32.com/viewtopic.php?p=94255 , https://esp32.com/viewtopic.php?p=103811 [S]
- Espressif ESP-PSRAM64/64H datasheet (3.3 V, 133 MHz): https://www.espressif.com/sites/default/files/documentation/esp-psram64_esp-psram64h_datasheet_en.pdf [S]
- ESP32-S3-WROOM-1 datasheet: https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.html and variant listings https://stgstatic0425.lioncircuits.com/parts/ESP32-S3-WROOM-1-N8R8 [S]
- LCSC chip listings: https://www.lcsc.com/product-image/C2913196.html (S3FN8), https://www.lcsc.com/product-image/C2913194.html (S3R8), https://www.lcsc.com/product-image/C3013940.html (S3FH4R2) [S, snapshots]
- TVP5150/5151 input-switch and lock threads (search-result summaries only, 403 on direct fetch): https://e2e.ti.com/support/processors-group/processors/f/processors-forum/352944/switching-inputs-too-slow-with-tvp5150am1pbs , https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/194792/decreasing-the-lock-time-on-tv5150am1 , https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/724150/linux-am5728-image-corruption-when-switching-tvp5150-sources , https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/386803/tvp5151-channel-b-startup-issues , https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/137314/tvp5150-losing-lock [S]
- TVP5154 datasheet (fast-lock note): https://www.ti.com/lit/gpn/tvp5154 [S]
- Video switches: TS5V330 https://www.ti.com/product/TS5V330 ; ADG794 https://www.analog.com/en/products/adg794.html ; TMUX1574 https://www.ti.com/lit/gpn/tmux1574 ; MAX4310 family https://www.analog.com/en/products/max4310.html [S]
- Repo: `Software/Witness/softwarev1/main/hardware_pins.h`, `Software/Witness/softwarev1/sdkconfig`, `sdkconfig.flight`; `Software/VIDEO_S3_COMPUTE_STUDY.md`; `Software/VIDEO_DOWNLINK_STUDY.md`.
