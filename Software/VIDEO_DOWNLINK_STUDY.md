# Live video downlink viability study (Witness / Iris v2 / ESP32-P4 / LR2021)

Research only. No firmware, hardware, bench, Pi or serial port was touched, nothing was committed and no personal information was sent to any site. Date: 2026-11-14.
Calculation script (run with `python -P`): `C:\Users\E\AppData\Local\Temp\claude\a1adc47d057f1999\57c05c66-37ca-4475-bfbd-0c3c8c766392\scratchpad\vid.py`.

Convention: **UNVERIFIED** = not confirmed from a primary source (datasheet, vendor page) in this session. Frame sizes, JPEG/H.264 bit rates, board power, prices and regulatory readings are estimates unless stated.

## 1. Recommendation

1. **Do not put video on the existing LoRa telemetry link (Option A).** At 130 km it carries about 10 kbps at 50 % airtime (SF7/500), against 90 to 350 kbps needed for useful moving imagery. It also steals airtime from flight-safety telemetry and the command/ack uplink.
2. **Build video as a separate, independent link (Option B/C hybrid) using one LR2021 as a video-only radio, and leave Witness's SX1262 telemetry link untouched.** The LR2021 datasheet shows that **sub-GHz FLRC (902-928 MHz) closes the 130 km link far better than 2.4 GHz FLRC** (about 8.2 dB less path loss and a higher +22 dBm internal PA, against +12 dBm at 2.4 GHz).
3. **Treat the LR2021 "2.4 GHz video + sub-GHz LoRa telemetry on one chip" idea as not viable as stated:** the chip has one active modem and one PLL at a time, so the two bands cannot run simultaneously. It would also put safety telemetry on the video board. 2.4 GHz FLRC at 130 km only closes with an external +27 dBm PA and a 15 to 24 dBi ground antenna, and only at the lowest rates.
4. **Use JPEG, not H.264.** No retransmission and a lossy link mean frames must be independent. H.264 only pays off in a short-GOP variant (phase 2).
5. **Add a new CAN node ("Iris-V": ESP32-P4 + video decoder + LR2021) instead of replacing Iris.** Iris's CAN/load-control role (ESP32-C3, TCAN3414, MCP23008, PAC1931) is the safe, flown part. The video chain is the high-risk new part, and replacing both at once couples them.
6. Ground side: a dedicated LR2021 receiver (the Aperture SX1262 cannot demodulate FLRC) on an ESP32-S3 that forwards chunks over USB to Horizon.

The headline caveat: every number below that depends on image size, antenna gain, rocket antenna pattern and crystal accuracy is an assumption until measured. The first step should be a conducted-attenuator test (section 12), not a flight.

## 2. What was read from the repo (background)

- Current downlink: Witness ESP32-S3 + Ra-01SH-P (SX1262), 926 MHz, SF7/500 kHz, CR 4/5. Three packets per second (25, 65 and 19 bytes), 5.9 % airtime (`Software/HITL/LORA_MODULATION_OPTIONS.md`). That document assumes +29 dBm, 0 dBi on the rocket, a 13 dBi Yagi or 2 dBi duck on the ground, NF 6 dB, a 10 dB fade allowance, and 134.06 dB FSPL at 130 km and 926 MHz. I reuse those assumptions so numbers compare.
- Doppler: 4.1 kHz (4.4 ppm) at 926 MHz, peak rate 2.2 kHz/s (`LORA_DOPPLER_EVALUATION.md`). The same 1.33 km/s gives 10.8 kHz at 2.44 GHz. Crystals are ±20 ppm with no TCXO, so worst-case offset is about ±44 ppm including Doppler.
- Iris (`Software/Iris/softwarev1IRIS/main/hardware.h`): ESP32-C3FH4, TCAN3414, Classical CAN at 500 kbit/s, MCP23008 load switches, PAC1931 current sense, camera packet 0x03 (currents and status only, no imagery). The camera is a load Iris switches and monitors. Witness firmware handoff says the current RF macros are 926 MHz, -17 dBm default (low-power PA), SF7/500.
- Iris and Witness use classical CAN at 500 kbit/s. Usable payload throughput is well under 250 kbit/s, which matters only for Option A (video would have to cross CAN to Witness).

## 3. Component facts

### 3.1 RunCam Split 4 V2 (analog source)
Sources: https://www.getfpv.com/fpv/cameras/flight-cameras/runcam-split-4-v2-fpv-camera.html, https://shop.runcam.com/runcam-split-4 (retailer listings, secondary).
- 10.2 g, 29 x 29 mm PCB, 14 x 14 mm lens module, 5 to 20 V input, max 450 mA at 5 V, 140 degree FOV, 13 MP sensor, 4K30 recording. Analog FPV output, NTSC/PAL switchable and 16:9/4:3 selectable, about 55 ms latency (all retailer claims, **UNVERIFIED** against a RunCam manual).
- The analog out is a preview. The onboard SD recording is the data of record and should stay that way. Analog-out connector type and output level are **UNVERIFIED**.
- Resolution of the analog stream is standard definition (NTSC about 720x480 interlaced, 29.97 fps; PAL 720x576 at 25 fps). UNVERIFIED for this camera's actual line count. Interlace and vibration artefacts are a risk (section 10).

### 3.2 Analog decoder chips
- TI TVP5150AM1: NTSC/PAL/SECAM to 8-bit ITU-R BT.656, about 113 to 115 mW typical, power-down under 1 mW. Source: https://www.ti.com/product/TVP5150AM1. Retailers list it as NRND or discontinued: **verify availability before designing it in**.
- ADI ADV7180 (parallel BT.656) and ADV7280-M (MIPI CSI-2 output variant): characteristics here are from general knowledge only and are **UNVERIFIED**. I could not retrieve their datasheets in this session. https://www.analog.com/ADV7280-M, https://www.analog.com/ADV7180.
- The P4 parallel port does not list BT.656 embedded-sync decoding (see 3.3), so choose a decoder that also exposes discrete HS/VS/PCLK (UNVERIFIED for each part) or use MIPI.

### 3.3 ESP32-P4
Sources: ESP32-P4 Series Datasheet v0.4, English copy at https://cdn.compacttool.ru/downloads/ESP32_P4_Chip_Datasheet_EN.pdf (a third-party mirror of an early revision; check against the current Espressif datasheet https://www.espressif.com/en/products/socs/esp32-p4 and the v3.x chip-revision user guide https://documentation.espressif.com/esp32-p4-chip-revision-v3.x_user_guide_en.pdf). Camera guidance: https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/solution-introduction/camera/dvp-mipi-csi-camera-solution.html.
- Cores: dual-core RISC-V HP up to 400 MHz, LP core 40 MHz, 768 KB SRAM, external PSRAM.
- Camera inputs confirmed: MIPI CSI-2 2-lane x 1.5 Gbps, and a 16-bit (8/16-bit) parallel CAM port in the LCD_CAM block, pins from the GPIO matrix. The datasheet text gives no BT.656 embedded-sync mode, so **BT.656 decoding by hardware is UNVERIFIED (treat as absent)**; use HS/VS/PCLK sync lines.
- ISP: max 1920x1080, inputs MIPI CSI, DVP, DW-GDMA, but **input formats are RAW8/RAW10/RAW12 only**. A decoder's YUV output therefore bypasses the ISP and goes by GDMA to PSRAM.
- JPEG codec (baseline): encoder accepts RGB888, RGB565, YUV422, GRAY; outputs YUV444/422/420; configurable quantisation; 1080p at 34 fps or 720p at 88 fps. Plenty for 720x480 or smaller. Restart-marker (DRI) support is **UNVERIFIED**. The safe route is to encode independent horizontal strips as separate JPEGs.
- H.264 encoder: baseline, I and P frames, CAVLC, YUV420 progressive, 1080p30. Lots of headroom.
- TWAI: **three controllers** (ISO 11898-1), so CAN is available for Iris-V.
- The P4 has no radio. Power draw and thermals are **UNVERIFIED** (no figures in the datasheet pages read; budget about 0.5 to 1 W on a video workload as an assumption).

### 3.4 Semtech LR2021
Sources: Semtech LR2021/LR2022/LR2012 Datasheet **Final Rev. 2.2 (29 July 2026)**, read locally from a copy of https://components101.com/sites/default/files/2026-08/LR20xx-Datasheet-.pdf; also on Mouser https://lv.mouser.com/pdfDocs/63293790LR20xxDatasheet_V2_2.pdf; product page https://semtech.com/lr2021. Page numbers below are datasheet page numbers.

**Capabilities (primary source):**
- Bands: 150-960 MHz sub-GHz and 1.5-2.5 GHz; FLRC on **both** bands (FLRC is LR2021 only; LR2022 and LR2012 lack it). LoRa up to 125 kbps, FLRC up to 2.6 Mbps.
- Output power: **+22 dBm sub-GHz (PA_LF, 105 mA at +22 dBm, 27 mA at +14 dBm)**, **+12 dBm at 2.4 GHz (PA_HF, 23 mA)**, 0.5 dB steps.
- RX current: 5.7 to 10.4 mA sub-GHz depending on mode (FLRC 2.6 Mbps 10.4 mA), about 7 to 9 mA more at 2.4 GHz boosted.
- Supply 1.8 to 3.7 V, 5x5 mm QFN, MOQ 3000.
- **Simultaneity: not simultaneous.** Section 4.4: "only one modem can be active at a time" in both RX and TX, and the TX/RX path uses one PLL and one frequency synthesizer. Two PA paths and two matching networks exist (dual-band with or without switch), but you time-multiplex: switch packet type/frequency between FLRC video and LoRa telemetry. Switching time is **UNVERIFIED** (not extracted from the datasheet). Half-duplex: the chip cannot receive uplink commands while transmitting video.
- Packet length: FLRC payload 0 to 511 bytes (fixed) or 6 to 511 (variable); payloads over 256 bytes need RxFifo/TxFifo IRQ handling. CRC 0/2/3/4 bytes, syncword 0/2/4 bytes. LoRa max packet length on this chip: **UNVERIFIED** (assume 255 B as on the SX126x).
- FLRC coding (Table 18-2): CR 1 (no FEC), 3/4, 2/3, 1/2 convolutional. At raw 2.6 Mbps: 2.6 / 1.95 / 1.73 / 1.3 Mbps effective. Footnote under the sensitivity tables: using CR 1/2 improves receiver sensitivity (amount not tabulated here).
- Raw bit rate / bandwidth pairs (Table 18-1): 2.6 Mbps / 2.666 MHz, 2.08 / 2.222, 1.3 / 1.333, 1.04 / 1.333, 0.65 / 0.740, 0.52 / 0.571, 0.325 / 0.357, 0.26 / 0.307 MHz.
- TCXO: supported on XTA (VTCXO supply, `SetTcxoMode`). The datasheet also describes an XTAL temperature-compensation mechanism with an NTC.

**FLRC sensitivity at 1 % PER, CR 3/4 (Tables 3-12 and 3-13; 2.4 GHz figure is for a 31-byte packet; sub-GHz measured on a short payload):**

| Raw rate | Effective (CR 3/4) | Sub-GHz 915 MHz | 2.4 GHz | Tolerated freq. error |
|---|---|---|---|---|
| 2.6 Mbps | 1.95 Mbps | -100.5 dBm | -99 dBm | ±150 kHz |
| 2.08 Mbps | 1.56 Mbps | -101.5 | -100 | ±150 kHz |
| 1.3 Mbps | 0.975 Mbps | -104.5 | -101.5 | ±100 kHz |
| 1.04 Mbps | 0.78 Mbps | about -105 (**layout ambiguous in extracted text**) | -102.5 | ±100 kHz |
| 0.65 Mbps | 0.487 Mbps | about -107 (same caveat) | -104 | ±70 kHz |
| 0.52 Mbps | 0.39 Mbps | -108.5 | -105 | ±50 kHz |
| 0.325 Mbps | 0.243 Mbps | -110 | -108 | ±30 kHz |
| 0.26 Mbps | 0.195 Mbps | -111 | -108.5 | ±25 kHz |

Table 18-3 converts tolerances to ppm. At 915 MHz: ±164 ppm (2.6/2.08M), ±109 (1.3/1.04M), ±77 (0.65M), ±55 (0.52M), ±33 (0.325M), ±27 (0.26M). At 2450 MHz: ±61, ±41, ±29, ±20, ±12, ±10 ppm respectively. (The PDF text extraction is offset by one row in that table. I realigned it by checking kHz/MHz; confirm against the PDF.)

**Modules and kits:**
- Seeed Wio-LR2021 module, 863-928 MHz plus 2.4 GHz, 17 x 10.4 mm, FCC/CE pre-certified, separate sub-GHz and 2.4 GHz RF pads, SPI up to 16 MHz, max RF input +10 dBm, 116 to 119 mA at +22 dBm sub-GHz and 28 mA at +12 dBm 2.4 GHz: https://wiki.seeedstudio.com/wio_lr2021_introduction/. Price, stock and TCXO/crystal not stated there.
- LR2021 LoRa Plus board (LR2021 + Nordic nRF54L15): https://www.cnx-software.com/2026/02/16/lr2021-lora-plus-board-combines-semtech-lr2021-and-nordic-nrf54l15-for-high-speed-flrc-and-lora-connectivity/
- NiceRF LR2021 module https://www.nicerf.com/lora-module/lora2021.html and Elecrow module spec https://www.elecrow.com/download/product/LMM16121D/LR2021_Wireless_Communication_Module_Specifications.pdf (existence confirmed from search results only; specs and stock **UNVERIFIED**), plus a Semtech EVK (named on the Seeed wiki, not reviewed).
- Arduino/RadioLib/ESP-IDF driver support for LR2021 FLRC: **UNVERIFIED** (Semtech's own LoRa Plus driver is the safe assumption). This is real software effort.

## 4. Video payload sizing (assumptions, measure before committing)

The analog frame is SD (about 720x480). The P4 PPA or JPEG path can downscale; tiers below. JPEG sizes are my rule-of-thumb estimates for FPV-like scenes at 4:2:0 and **UNVERIFIED**; sky and plume content may compress far better. Bit rates include about 10 % packet overhead.

| Tier | Image | Estimated JPEG | 5 fps | 2 fps | 0.5 fps |
|---|---|---|---|---|---|
| T1 high | 320x240, about q50 | about 8 KB | 352 kbps | 141 kbps | 35 kbps |
| T2 mid | 320x240, about q25-30 | about 5 KB | 220 kbps | 88 kbps | 22 kbps |
| T3 | 240x180 | about 4 KB | 176 kbps | 70 kbps | 18 kbps |
| T4 low | 160x120 | about 2 KB | 88 kbps | 35 kbps | 8.8 kbps |

H.264 (baseline, CAVLC): for 320x240 at 5 fps with a 1-second GOP the rate is roughly 60 to 120 kbps (**UNVERIFIED rule of thumb**), about 3x lower than MJPEG, but one lost P-slice corrupts the rest of the GOP and there is no retransmission. At 0.5 fps every frame must be an IDR anyway, so it collapses back to about JPEG-sized frames. Decision: JPEG now, short-GOP H.264 (IDR at least every 5 frames) as a phase-2 option only on the high-rate link.

## 5. Link budgets (130 km and 10 km)

Assumptions: rocket antenna 0 dBi (conservative; a real wrap-around or patch near the nose or fin can has pattern nulls, see risks); fade allowance 10 dB; NF 6 dB for LoRa as in the repo, sensitivities for FLRC are datasheet values; polarization mismatch and ground-reflection ignored; ground antenna is fixed upward-pointing (rocket stays within a modest cone if downrange drift is small) or a small tracked dish. FSPL: 130 km: **134.1 dB at 926 MHz, 142.5 dB at 2440 MHz**; 10 km: 111.8 dB (926) and 120.2 dB (2440).

### 5.1 Option A: LoRa on the existing link (SX1262, +29 dBm per repo assumption)
Payload rates with 250-byte packets at 500 kHz BW, CR 4/5 (my airtime calc; SF5/SF6 SNR thresholds are approximations and **UNVERIFIED**, SF7+ are the repo numbers):

| SF | Packet airtime | Payload rate at 100 % / 50 % airtime | Sensitivity | Margin at 130 km (Yagi 13 dBi / duck 2 dBi) |
|---|---|---|---|---|
| 5 | 34 ms | 58.7 / 29.4 kbps | -113.5 dBm | 21.4 / 10.4 dB |
| 6 | 57 ms | 34.9 / 17.5 kbps | -116.0 | 23.9 / 12.9 |
| 7 | 97 ms | 20.5 / 10.3 kbps | -118.5 | 26.4 / 15.4 |
| 8 | 172 ms | 11.7 / 5.8 kbps | -121.0 | 28.9 / 17.9 |
| 9 | 308 ms | 6.5 / 3.3 kbps | -123.5 | 31.4 / 20.4 |

Result: the link is strong but the pipe is thin. At 130 km (SF7) about 10 kbps is available at 50 % airtime, which is just enough for **0.5 fps of 160x120 (8.8 kbps, tier T4)** and nothing else. At short range SF5 gives about 29 kbps, i.e. 160x120 at about 1.5 to 2 fps. The 5 fps / 320x240 goal (220 to 352 kbps) is **10x or more beyond** the LoRa pipe. Video also pushes telemetry airtime from 6 % toward 50 %, shrinks uplink availability and would have to cross CAN from Iris-V to Witness (fine at 30 kbps).

### 5.2 Option B: dedicated second radio
- B1: second SX1262 LoRa (same band): same numbers as 5.1 but no longer shares airtime with telemetry, so up to about 60 kbps at SF5 and 100 % duty (about 20 kbps at SF7). Still far below target. It also needs frequency separation and filtering from Witness's own 926 MHz radio.
- B2: SX1280-class 2.4 GHz FLRC: not researched, since LR2021 supersedes it. The 2.4 GHz numbers below apply with similar sensitivities (SX1280 figures **UNVERIFIED**).

### 5.3 Option C: LR2021
RSL = P_tx + 0 dBi (rocket) + G_ground - FSPL. Margin = RSL - sensitivity at 1 % PER, before the 10 dB fade allowance.

**Sub-GHz FLRC at 130 km (926 MHz, FSPL 134.1 dB), 13 dBi Yagi:**

| TX power | RSL | 2.6M (1.95M eff) | 1.3M (0.975M) | 0.52M (0.39M) | 0.26M (0.195M) |
|---|---|---|---|---|---|
| +22 dBm (LR2021 internal) | -99.1 | +1.4 dB | +5.4 | +9.4 | +11.9 |
| +29 dBm (external PA, as repo assumes) | -92.1 | +8.4 | +12.4 | +16.4 | +18.9 |

**2.4 GHz FLRC at 130 km (2440 MHz, FSPL 142.5 dB):**

| TX power / ground antenna | RSL | 2.6M | 1.3M | 0.52M | 0.26M |
|---|---|---|---|---|---|
| +12 dBm / 12 dBi patch | -118.5 | -19.5 | -17.0 | -13.5 | -10.0 |
| +12 dBm / 24 dBi dish | -106.5 | -7.5 | -5.0 | -1.5 | +2.0 |
| +27 dBm (ext PA) / 15 dBi | -100.5 | -1.5 | +1.0 | +4.5 | +8.0 |
| +27 dBm (ext PA) / 24 dBi | -91.5 | +7.5 | +10.0 | +13.5 | +17.0 |

Reading: **at 2.4 GHz the internal PA cannot close 130 km with any realistic ground antenna.** With an external PA (+27 dBm, part not selected, **UNVERIFIED**) and a pointed 24 dBi dish it reaches 13 to 17 dB margin at 390/195 kbps effective. At 900 MHz the internal +22 dBm PA with a 13 dBi Yagi gives 9 to 12 dB margin at 390/195 kbps effective, and 16 to 19 dB with a +29 dBm PA. At 10 km all of the above have 20+ dB more margin (path loss is 22 dB lower), so 1.3 to 2.6 Mbps works near the pad.

Packet-size note: sensitivity figures are for short (10 to 31 byte) packets. Long packets (200 to 250 bytes) have a higher PER at the same SNR, so budget 1 to 2 dB more margin or use FEC (section 7).

## 6. Per-option summary

Required throughput: 5 fps needs about 90 to 350 kbps (tiers T4 to T1); 0.5 fps needs about 9 to 35 kbps. Target is "5 fps ground degrading to 0.5 fps at apogee".

| | A: LoRa on primary link | B1: second LoRa radio | C-2.4: LR2021 2.4 GHz FLRC + sub-GHz LoRa | C-sub: LR2021 sub-GHz FLRC (video only) |
|---|---|---|---|---|
| Rate at 130 km | about 10 kbps (SF7, 50 %) | about 20 kbps (SF7, 100 %) | 195 kbps eff (0.26M raw) | 195 to 390 kbps eff (0.26M-0.52M raw) |
| Margin at 130 km | 26 dB Yagi / 15 dB duck (before 10 dB fade) | same | +2 dB (12 dBm, 24 dBi dish) to +17 dB (27 dBm PA, 24 dBi) | +9 to +12 dB (22 dBm, Yagi), +16 to +19 dB (29 dBm) |
| Rate near ground | about 29 kbps (SF5) | about 59 kbps (SF5) | up to 1.95 Mbps | up to 1.95 Mbps |
| Meets 5 fps ground | No | No | Yes (T1 to 320x240) | Yes |
| Meets 0.5 fps at apogee | Marginal (T4 only) | Marginal (T4 to T3) | Only with ext PA and pointed dish | **Yes, with margin; about 2 fps possible** |
| Telemetry coexistence | Competes for airtime | Independent, same band, desense risk | One modem and PLL: time-multiplexed with LoRa; telemetry shares failure domain | Independent from Witness; desense risk in same band |
| Ground RX | Aperture as is | Second SX1262 | LR2021 (2.4 GHz) | LR2021 (sub-GHz) |
| New RF hardware | None | Radio module | LR2021 + 2.4 GHz PA/filter | LR2021 (+ optional PA) |
| Overall | Not viable for video | Weak | Viable only with extra RF hardware; best for band isolation | **Recommended** |

## 7. Packet loss handling (no retransmission)

- Each JPEG frame is split into independent horizontal strips (each its own small baseline JPEG, or one JPEG with restart markers if the P4 encoder supports DRI, **UNVERIFIED**), so a lost packet costs one strip, not the frame. Each radio packet header: frame ID (8 bit), strip index, chunk index, chunk count, quality tier, CRC (FLRC and LoRa both have hardware CRC).
- Packet size 200 to 250 bytes balances airtime against PER; FLRC allows up to 511 bytes.
- Interleave chunks across strips so a burst (rocket spin, antenna null, motor transient) hits many strips lightly rather than one strip fully.
- FEC: FLRC convolutional coding at CR 3/4 or 2/3 for the long-range tiers (costs 25 to 33 % of the raw rate, improves sensitivity per the datasheet footnote). Optionally add an application-level erasure code (for example one XOR parity chunk per 4 to 6 data chunks, or Reed-Solomon over strips). At short range use CR 1 (no FEC) at 2.6 Mbps.
- Ground side: drop incomplete strips, hold the previous frame's strip (conceal), display per-strip age. Log raw chunks for post-flight reconstruction.
- Pad time is cheap: send a tiny low-quality frame immediately at the start of each tier change so something always displays.

## 8. Doppler and ppm tolerance

- Doppler at 1.33 km/s: 4.1 kHz at 926 MHz (4.4 ppm), 10.8 kHz at 2440 MHz (same 4.4 ppm). Rate 2.2 kHz/s at 926 MHz peak (about 5.8 kHz/s at 2440 MHz, scaled). FLRC correlators tolerate fixed offsets, not necessarily offset *rates* inside a packet; short packets (1 to 10 ms at 0.2 to 2 Mbps) make drift in-packet negligible.
- Crystal offsets: worst-case ±20 ppm per side = ±40 ppm plus 4.4 ppm Doppler = **±44 ppm total**.
  - At 915 MHz, FLRC tolerance (Table 18-3): 1.3M and faster ±109 to ±164 ppm: OK. 0.52M ±55 ppm: OK with 11 ppm to spare. **0.325M (±33 ppm) and 0.26M (±27 ppm) fail worst case**, passing only if each crystal is within about ±11 to ±14 ppm. So without a TCXO, **0.52M raw (0.39M effective) is the practical long-range rate**, not 0.26M.
  - At 2450 MHz tolerances are about 60 % smaller in ppm terms: ±41 ppm at 1.3M fails worst case (44 ppm), ±61 ppm at 2.08M/2.6M is OK, 0.52M (±20 ppm) and below fail badly. So 2.4 GHz long-range FLRC essentially **requires a TCXO** (LR2021 supports one) or a ground AFC/pre-compensation scheme. Whether Wio-LR2021 or other modules fit a TCXO is **UNVERIFIED**.
- Mitigations for either band: TCXO on the video radio (±1 to 2 ppm), calibrate each pair at room temperature and apply offset trim, ground receiver with wide correlator settings, broad frequency search during preamble.

## 9. Regulatory notes (verify before flight, none confirmed from primary sources here)

- US 902-928 MHz and 2400-2483.5 MHz under FCC Part 15.247: digital-modulation systems need a 6 dB bandwidth of at least 500 kHz, may run up to 1 W (+30 dBm) conducted with antenna gain up to 6 dBi (EIRP beyond that needs power reduction), and have a PSD limit of 8 dBm per 3 kHz. No duty-cycle limit applies to digital modulation, so 100 % video duty is allowed (the repo's LoRa documents already treat 500 kHz as the no-hopping bandwidth). The LR2021 datasheet lists FCC Part 15.247/249 as a compliance target. **UNVERIFIED reading.**
- FLRC raw 0.26 and 0.325 Mbps have occupied bandwidths of 307 and 357 kHz, under 500 kHz, so they probably do not qualify as 15.247 digital modulation. The 0.52M rate (0.571 MHz modulation bandwidth) is borderline (6 dB BW must be measured). Another reason to pick 0.52M or 0.65M as the lowest rate. PSD at +29 dBm over a 0.6 to 0.74 MHz signal is about +7 to +8 dBm per 3 kHz (at the limit), so +27 to +28 dBm is the safer cap. **UNVERIFIED calculation.**
- Rocket-specific: radio operation at 130 km altitude and launch-range coordination (FAA/range safety, a separate topic) are not covered here.
- Amateur alternatives: 33 cm (902-928 MHz) and 13 cm (2390-2450 MHz) amateur allocations allow more power and gain, but require licensed operators, station ID, no encryption, and secondary status (shared with ISM). Only relevant if the team holds amateur licences. **UNVERIFIED**.

## 10. Risks

- **Rocket antenna.** Biggest unknown. A 0 dBi assumption needs a good pattern; a roll-spinning rocket with a single-sided patch or blade can see 10 to 20 dB nulls. A circularly polarised wrap-around or two antennas with diversity helps; plume attenuation and a metal ogive/fairing or composite body shadowing are **UNVERIFIED**. The existing LoRa telemetry antenna already carries the same issue; the video link should not share it unless it passes a duplexer/diplexer test.
- **Co-site desense** if video and Witness telemetry are both in 902-928: the video TX at +22 to +29 dBm a few centimetres from the Witness RX (command uplink) could blind it. Separate by frequency (for example 910 MHz video vs 926 MHz telemetry), filter, or pause video during uplink windows. Moving video to 2.4 GHz removes this (the cost is 8.4 dB of path loss).
- **Analog video under vibration/shock.** Cable and connector sync loss, interlace tearing, decoder relock time after dropout (hundreds of ms, **UNVERIFIED**), ground loops injecting hum bars, switch-mode noise from the P4 supply into a 75 ohm line. Use a short shielded cable, a clamp or common-mode choke, camera on a separate LDO, and a decoder with strong sync detection (TVP5150 marketing claim). Boost g-loads were about 72 g axial in the repo's trajectory; test the cable and connector on the shake table.
- **Power and thermal.** Camera up to 450 mA at 5 V (2.25 W), decoder about 0.12 W, P4 about 0.5 to 1 W (**UNVERIFIED**), LR2021 TX 105 mA at +22 dBm (about 0.35 W), plus any external PA. Total about 3.5 to 4.5 W peak, about 0.7 to 1 A at 5 V. A 10 to 15 minute flight is under 1 Wh, so energy is minor, but at altitude (near-vacuum) there is no convection and the P4 and PA must conduct heat to the structure. Brownout on the shared battery during +22 dBm TX is a known repo concern (see the Pi bench USB over-current memory about Ra-01SH-P transmit current): add bulk capacitance and a separate regulator.
- **LR2021 maturity.** Final datasheet is dated July 2026, so silicon, drivers (RadioLib/ESP-IDF), modules and errata are young. Single-source vendor risk.
- **ESP32-P4 maturity.** The English datasheet read is v0.4; JPEG/H.264 limits, ISP and DVP behaviour have changed in chip revisions (see the v3.x user guide). BT.656 is not listed. TVP5150 is NRND.
- **CAN and loads.** Iris-V on the CAN bus must not disturb arming/load control: keep it a separate node with its own watchdog, and let Iris keep switching the camera and video board loads (video board off by default).
- **Command gating.** Any ground command to change video tiers rides the existing, already audited command path in Witness (see the radio-command gating fix in recent commits); do not add a second command receiver.
- **Weight and volume** of the P4 + decoder + LR2021 board are unknown until a layout exists.

## 11. Adaptive scheme

Open-loop on range (altitude and ground range from Witness GPS/baro state over CAN, with RSSI-based fallback if a return path exists) is simpler and safer than closed-loop, since uplink is scarce and the ground station gets no ack channel for video:

| Slant range | FLRC rate (raw/eff) | Image | Estimated fps |
|---|---|---|---|
| under 15 km | 2.6M / 1.95M (CR 3/4) or 1.3M | 320x240, T1 | 5 or more |
| 15-50 km | 1.3M / 0.975M | T1 to T2 | 5 to 3 |
| 50-100 km | 0.65M / 0.49M | T2 to T3 | 3 to 2 |
| 100-130+ km | 0.52M / 0.39M, CR 3/4 or 1/2 | T3 to T4, or T2 at lower rate | 2 to 0.5 |

All of the above is limited by section 5 margins and section 8 crystal tolerance. The rocket picks the tier from its own state. The ground station can also show an indicator of the tier received and chunk loss. A beacon chunk with tier ID in every packet header lets the ground receiver retune its FLRC settings without a command link (the receiver can also cycle through candidate rates). Frame rate is a simple output of frame size and rate: at 0.39 Mbps effective and about 4 KB frames the pipe supports about 10 fps airtime-wise, so the 0.5 fps apogee figure leaves much slack to spend on FEC or margin.

## 12. Ground station

- Aperture (ESP32-S3 + SX1262 Ra-01) **cannot receive FLRC**; FLRC exists only on SX128x (2.4 GHz) and LR2021. Need an LR2021 module (Wio-LR2021 or similar) on an SPI bus of an ESP32-S3 or Pi, plus an antenna on the matching band (900 MHz Yagi 13 dBi per the repo or a 2.4 GHz 15 to 24 dBi patch/dish; low-noise amplifier and band filter ahead of the radio would help, **UNVERIFIED benefit** of about 2 to 3 dB).
- SPI at 16 MHz handles 2.6 Mbps (about 325 KB/s) easily; USB CDC or UART bridging at 1 to 2 Mbps to the laptop; or Wi-Fi/UDP.
- Frames reach Horizon: new packet ID (for example 0x04 video chunk) in the existing `witness_protocol.py` framing, reassembled per frame ID and strip, shown in the existing GUI, and logged raw next to the existing `.bin` and CSV logs for post-flight replay. This is a ground-side Python task (about 1 to 2 weeks) plus ESP32-S3 receive firmware.
- Pointing: fixed zenith-pointing antennas suit a mostly vertical flight with small drift; for high gain (24 dBi dish, about 10 degree beam) a manual or motorised mount is needed, which is the "modest tracking" the task contemplated.

## 13. Iris v2 vs a new node

Iris currently: ESP32-C3 (single-core, no camera interface), TCAN3414, MCP23008 loads, PAC1931 sensing, USB. The P4 has three TWAI controllers, so the CAN role could be absorbed, but I recommend **not** replacing Iris first:
- Add Iris-V as a new CAN node (P4 + decoder + LR2021 + its own regulators), powered through an Iris load channel. Iris keeps load control and the camera current packet.
- If a single board is wanted later, port the Iris firmware to the P4 (IDF support for P4 and TWAI behaviour on it are **UNVERIFIED**) after the video chain is flight-proven.
- Scope for Iris-V firmware: decoder I2C config, GDMA capture of BT.656/parallel frames, PPA scaling, JPEG strip encode, packetiser, LR2021 FLRC driver and tier control, CAN client (state in, health out), watchdog, brownout handling.

## 14. Rough BOM and effort (estimates, **UNVERIFIED** prices)

Flight: RunCam Split 4 V2 (existing), analog decoder (ADV7180/ADV7280-M-class or similar, a few USD), ESP32-P4 module (about 15 to 40 USD), LR2021 module (about 10 to 20 USD), optional 2.4 GHz or 900 MHz PA/LNA and SAW filter (about 5 to 15 USD), TCXO (about 2 to 5 USD), regulators, CAN transceiver, connectors, antenna and shielded cable. Board and parts: about 80 to 150 USD for a prototype run. Ground: LR2021 module + ESP32-S3 + Yagi or patch: about 60 to 150 USD.

Effort (one engineer, no flight): ground capture/JPEG pipeline on a P4 dev kit 2 to 3 weeks; LR2021 driver and FLRC link bring-up with attenuator tests 2 to 3 weeks; Horizon viewer/protocol 1 to 2 weeks; Iris-V board design and bring-up 3 to 4 weeks; vibration/thermal/range test 2 to 3 weeks. Total roughly 10 to 15 weeks.

## 15. First validation steps (nothing run yet)

1. Record sample RunCam analog footage and measure JPEG sizes per tier on a laptop (replaces the UNVERIFIED sizes in section 4).
2. Bench P4 dev kit + decoder capture of the actual camera output: confirm DVP sync mode, relock time, JPEG strip workflow, PSRAM bandwidth.
3. Two LR2021 modules, conducted over fixed attenuators equal to 130 km minus antenna gains, to confirm margin, PER with 200 to 250 byte packets, FLRC CR 3/4 vs 1/2, and crystal-offset tolerance; measure the module crystal accuracy and Rx desense from the SX1262.
4. Measure TX-to-RX isolation to the Witness antenna on the airframe and the real pattern of the chosen antenna.

## 16. Open questions

- What is the real rocket antenna layout, and is the flight vertical enough for a fixed zenith ground antenna?
- Is +29 dBm available on the telemetry side already (Witness defaults to -17 dBm per macros), and would a second +29 dBm PA be accepted for video?
- Does the LR2021 module in hand have a TCXO or a plain crystal?
- Does the P4 encoder support restart markers, and can the P4 parallel port take BT.656 with embedded sync?
- LR2021 packet-type switching latency and LoRa max packet length (relevant only if the "one chip does both" idea is kept).
- Regulatory path (Part 15 vs amateur) and range/FAA coordination for the second transmitter.
- Is 0.5 fps at 160x120 acceptable as a fallback if the video radio fails (Option A as backup)?

## Sources

- Semtech LR2021/LR2022/LR2012 datasheet Final Rev 2.2: https://lv.mouser.com/pdfDocs/63293790LR20xxDatasheet_V2_2.pdf and https://components101.com/sites/default/files/2026-08/LR20xx-Datasheet-.pdf ; product page https://semtech.com/lr2021
- Seeed Wio-LR2021: https://wiki.seeedstudio.com/wio_lr2021_introduction/ ; CNX Software board article (URL above); NiceRF https://www.nicerf.com/lora-module/lora2021.html
- ESP32-P4 datasheet (English v0.4 mirror) https://cdn.compacttool.ru/downloads/ESP32_P4_Chip_Datasheet_EN.pdf ; Espressif https://www.espressif.com/en/products/socs/esp32-p4 ; camera notes https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/solution-introduction/camera/dvp-mipi-csi-camera-solution.html
- TI TVP5150AM1 https://www.ti.com/product/TVP5150AM1
- RunCam Split 4 V2 (retailer data): https://www.getfpv.com/fpv/cameras/flight-cameras/runcam-split-4-v2-fpv-camera.html , https://shop.runcam.com/runcam-split-4
- Repo: `Software/HITL/LORA_MODULATION_OPTIONS.md`, `Software/HITL/LORA_DOPPLER_EVALUATION.md`, `Software/Witness/softwarev1/FIRMWARE_HANDOFF.md`, `Software/Witness/softwarev1/main/comms.h`, `Software/Iris/softwarev1IRIS/main/hardware.h`

## 17. Update: NiceRF LoRa2021F33-2G4 on the rocket, 18 dBi dish on the ground (2026-11-14)

Assumptions from the user: one radio module on the rocket, the NiceRF LoRa2021F33-2G4 (https://www.nicerf.com/lora-module/lora2021f33-2g4.html), carrying 900 MHz LoRa telemetry and 2.4 GHz FLRC video; an 18 dBi parabolic 2.4 GHz antenna on the ground.

**Module facts (product page; datasheet V1.3 PDF could not be text-extracted):** LR2021 inside. Two RF pads, ANT (sub-GHz) and ANT-2G4. TX power 30 dBm typ at 868/915 MHz and at 1.9-2.5 GHz (the "2 W" / 33 dBm is the 433/470 MHz bands only). 2.4 GHz receive LNA with a bypass switch on DIO5 (vendor: bypass costs 12 dB sensitivity). Frequency error listed as 0.5 ppm (suggests a TCXO; **UNVERIFIED**). Supply 3.0-5.5 V, 5 V typ. TX current < 900 mA at 2.4 GHz 1 W and < 800 mA at 915 MHz 1 W (at 5 V); RX < 8 mA sub-GHz, < 42 mA at 2.4 GHz with the LNA on. 39 x 21 x 3.3 mm (image caption). No FLRC sensitivity given, no certification stated, no price.

**2.4 GHz FLRC at 130 km:** RSL = 30 dBm + 0 dBi (rocket) + 18 dBi - 142.5 dB = **-94.5 dBm**. Margins use the bare LR2021 sensitivities (section 3.4, CR 3/4); the module's LNA is extra, unquantified upside.

| Raw rate (effective) | Margin | Margin with 3 dB polarisation/pointing loss | Compliant with 15.247 DTS (6 dB BW >= 500 kHz)? |
|---|---|---|---|
| 2.6 M (1.95 M) | +4.5 dB | +1.5 | yes |
| 1.3 M (0.975 M) | +7.0 | +4.0 | yes |
| 0.65 M (0.487 M) | +9.5 | +6.5 | yes |
| 0.52 M (0.39 M) | +10.5 | +7.5 | yes (0.571 MHz) |
| 0.325 M (0.243 M) | +13.5 | +10.5 | no (0.357 MHz), unless amateur Part 97 |
| 0.26 M (0.195 M) | +14.0 | +11.0 | no (0.307 MHz), unless amateur Part 97 |

At 10 km the path loss is 22 dB lower (RSL about -72.5 dBm), so 2.6 Mbps has about +26 dB.

**Reading:** this combination changes the 2.4 GHz verdict from "does not close" to "closes". At apogee, 0.52 Mbps raw (about 390 kbps payload before framing, roughly 300 kbps of video after packet overhead and the telemetry slot) has 7-10 dB margin. That is about ten times the 0.5 fps target (9-35 kbps), so apogee could run 2-5 fps at 160x120-320x240, or keep 0.5 fps and spend the margin on CR 1/2 and erasure FEC. Near the pad 2.6 Mbps gives full 5 fps at 320x240 with lots of room. It does not meet the full 10 dB fade allowance of section 5 at 0.52 Mbps, so step down to 0.26-0.325 Mbps (amateur rules) or use CR 1/2 if fades show up.

**900 MHz LoRa telemetry on the same module:** 30 dBm instead of today's -17 dBm bench / +22 dBm chip limit, so the telemetry link gains margin as well (about +26 dB at SF7/500, 130 km, 13 dBi Yagi; section 5.1).

**Doppler and crystal:** 1.2 km/s at 2.44 GHz is 4 ppm (about 9.8 kHz). With the listed 0.5 ppm reference, even 0.26 Mbps (±10 ppm tolerance) works. If the module turns out to use a plain ±20 ppm crystal, the worst case is about ±44 ppm and only 2.08-2.6 Mbps (±61 ppm) tolerates it, which loses the long-range tiers. **Confirm the reference oscillator with NiceRF before buying.**

**One radio, two bands: what it costs.** The LR2021 runs one modem at a time (section 3.4), so the rocket time-multiplexes on a fixed GPS-timed frame, for example per 1 s: LoRa telemetry burst (3 packets, about 60-90 ms at SF7/500), a LoRa uplink listen window (about 100 ms), then 2.4 GHz FLRC video for the rest (about 80 %). Consequences:
- Uplink commands are only heard in the listen window, so command latency rises to up to about 1 s; the existing ack/retry system (command_link) covers this, but the window must be in the schedule in every flight state.
- The ground side is simplest with **two receivers** (one LR2021 or SX1262 on 900 MHz, one LR2021 on 2.4 GHz behind the dish): no band switching on the ground, and the existing Aperture keeps working for telemetry.
- Band-switch time is still **UNVERIFIED** and sets the overhead.

**Power and supply:** 2.4 GHz TX at 1 W is < 900 mA at 5 V (about 4.5 W) and runs about 80 % of the time while streaming, so about 3.5-4 W average, plus the P4. On the 1S battery that needs a 5 V boost good for at least 1 A peak, heat sinking of the module, and streaming only from launch detect (not in Pad Idle or Landed). Check whether the module keeps 30 dBm at 3.3-4.2 V or needs the full 5 V (**UNVERIFIED**).

**Ground antenna pointing:** an 18 dBi dish has about a 25 degree half-power beamwidth, so it needs pointing within about ±10 degrees. Near apogee over the site the angular rate is low, but boost is fast and close; either track (see the shelved antenna pointer design) or run a wide antenna for the first seconds while the 10 km budget has 20+ dB to spare. Use circular polarisation on the ground (or both ends) to survive rocket roll; the 3 dB column above covers that.

**Architecture decision this forces:** a single module means one board owns the only downlink. If the P4 (Iris v2) owns it, Witness telemetry, the Landed position beacon and recovery depend on the P4 board working, and Witness telemetry reaches the P4 over CAN. If Witness keeps it, the video stream (300+ kbps) has to reach Witness from the P4, which classic CAN at 1 Mbit/s cannot carry alongside Iris traffic (would need SPI or a dedicated link). The study recommends the P4 owning the module only if the Landed/recovery beacon also runs there with its own GPS and battery path; otherwise keep a small independent telemetry radio on Witness. User decision.

**Regulatory:** 30 dBm conducted at 2.4 GHz is the 15.247 limit with an antenna of 6 dBi or less on the rocket. Rates below 0.52 Mbps fail the DTS bandwidth rule and would need amateur operation (Part 97, licence, station ID, no encryption). The module lists no FCC certification.

**Updated open questions:** reference oscillator (TCXO?) and FLRC sensitivity of this module; output power at 3.3-4.2 V; LNA noise-figure gain at 2.4 GHz; band-switch time; datasheet V1.3 contents (PDF text not extractable here); availability and price from NiceRF.

## 18. Update: Witness keeps its Ra-01SH-P LoRa link, LR2021 module does 900 MHz video only (2026-11-14)

The LR2021's one-band-at-a-time limit stops mattering once it carries only video. Setup: Witness keeps the Ra-01SH-P (about +29 dBm) for telemetry, commands and the Landed beacon, unchanged. A NiceRF LoRa2021F33-2G4 on the video board transmits FLRC at 915 MHz, 30 dBm, into a 13 dBi Yagi on the ground (about 45 degree beam, far easier to point than the 18 dBi 2.4 GHz dish at about 25 degrees).

**900 MHz FLRC at 130 km:** RSL = 30 + 0 + 13 - 134.0 = **-91.0 dBm** (bare LR2021 sensitivities, CR 3/4).

| Raw rate (effective) | Margin | With 3 dB loss | Freq. tolerance at 915 MHz | OK with ±20 ppm crystals (±44 ppm worst case)? |
|---|---|---|---|---|
| 2.6 M (1.95 M) | +9.5 dB | +6.5 | ±164 ppm | yes |
| 1.3 M (0.975 M) | +13.5 | +10.5 | ±109 ppm | yes |
| 0.65 M (0.487 M) | +16.0 | +13.0 | ±77 ppm | yes |
| 0.52 M (0.39 M) | +17.5 | +14.5 | ±55 ppm | yes |
| 0.325 M (0.243 M) | +19.0 | +16.0 | ±33 ppm | no (needs the 0.5 ppm reference) |
| 0.26 M (0.195 M) | +20.0 | +17.0 | ±27 ppm | no (needs the 0.5 ppm reference) |

At 10 km RSL is about -68.7 dBm, so 2.6 Mbps has about 32 dB margin.

Compared with 2.4 GHz video on the 18 dBi dish (section 17): about **7 dB more margin** at every rate, it meets the full 10 dB fade allowance at 0.52-1.3 Mbps, it works with plain ±20 ppm crystals at 0.52 Mbps and faster (no TCXO dependency), every rate from 0.52 Mbps up meets the 15.247 DTS bandwidth rule, and a Yagi needs much less pointing. At apogee 0.52 Mbps gives about 300 kbps of video after overhead, enough for 5 fps at 320x240 rather than the 0.5 fps target; near the pad 1.3-2.6 Mbps.

**The cost: both transmitters are in 902-928 MHz on the same rocket.**
- Downlink is fine: put telemetry and video at opposite ends of the band (for example LoRa 903 MHz, video 925 MHz, about 22 MHz apart). The ground receivers see two distant signals of similar level and a 22 MHz gap; use separate ground antennas or a splitter.
- Uplink is the problem. With the video TX at +30 dBm and perhaps 20-30 dB of antenna isolation on the airframe, the Witness receiver sees roughly 0 to +10 dBm of video signal 22 MHz away, about 30 dB over what an SX1262 tolerates as a blocker, plus the video PA's wideband noise in the LoRa channel (estimated 30-40 dB desense, **UNVERIFIED**). A 26 MHz-wide band cannot be split by a practical filter. So **Witness cannot hear commands while video is transmitting.**
- Fix: GPS-timed slots (already approved for Witness/Aperture TX deconfliction). The video board pauses for a fixed command window every second (for example 100-150 ms aligned to the GPS second), told the schedule over CAN or by the shared PPS. Video loses 10-15 % of airtime; command latency rises to at most about 1 s, which the existing ack/retry covers. Witness's own telemetry TX also lands in the video receiver path only on the ground, which is not affected.
- Also stop video on any fault that could leave the video TX keyed (watchdog on the video board drops RF), and never stream in Pad Idle or Landed, so the Landed ping/beacon link is never desensed.

**Power:** 915 MHz 1 W TX is < 800 mA at 5 V (about 4 W peak, 3.4-3.6 W average while streaming at 85-90 % duty), on the video board's own 5 V boost.

**Verdict:** with a dedicated video module, 900 MHz video is the stronger link (more margin, no TCXO dependency, easy antenna) at the price of GPS-slotted command windows. 2.4 GHz video (section 17) avoids the co-site problem entirely (the bands are easy to filter apart and Witness can listen all the time) but needs the dish, pointing and the TCXO. Both are viable; the choice is uplink availability vs margin. Bench test the desense with the real modules before deciding.
