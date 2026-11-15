# ESP32-S3 vs ESP32-P4 for the analog video downlink: decoder, capture, JPEG and bitrate study

Date: 2026-11-15. Research only; no code, hardware or bench touched. Companion to `Software/VIDEO_DOWNLINK_STUDY.md` (sections 3.2, 4, 17, 18).

Labelling: **[V]** = read from a primary or vendor page in this session. **[S]** = secondary source (forum, distributor, search-result summary of a vendor page). **[E]** = my estimate or recollection, not verified. Treat [E] numbers as planning values to be replaced by a bench measurement.

## 1. Verdict

- **An ESP32-S3 can do the compute side.** Software JPEG at 320x240 takes about 22 ms/frame on one core [V], so 5 fps uses roughly 11 % of one core. Even a full 720x240 field at 5 fps is about 30 % of one core [E]. The link, not the CPU, sets the picture size.
- **The link sets the quality.** After overhead the study's rates leave about 3.75 KB/frame (0.26 Mbps raw) to 7.5 KB/frame (0.52 Mbps raw) at 5 fps, and 37.5 to 75 KB/frame at 0.5 fps. That means 320x240 or 360x240 at roughly q30-45 at 5 fps near the pad, and up to a full 720x240 field at q60-75 at 0.5 fps near apogee.
- **The real risk is the capture path, not the encode.** The S3 LCD_CAM camera port has no documented BT.656 embedded-sync (SAV/EAV) decoding. The decoder must drive discrete PCLK, VSYNC, HSYNC and a data-enable (DE) line. Espressif's own driver docs, FAQ and the TRM chapter I could reach do not describe an analog-decoder use. The only public attempt I found (esp32-camera issue 566, PAL decoder at 37 MHz PCLK) never reached a working result [V]. Budget a bench prototype before committing.
- **Recommended decoder for the S3 (revised, section 12): TI TVP5150AM1.** Its datasheet (read in full this session [V]) confirms discrete HSYNC, VSYNC, FID and a programmable **AVID** that can be a clean data-enable (active pixels only, low in vertical blanking), 3.3 V I/O, 27 MHz byte clock, and it is in stock. It has no scaler, so downscaling is done by AVID horizontal cropping plus a cheap software 2:1 decimation (about 3-15 ms per field, estimate). Fallback: **Renesas TW9910** (hardware scaler, 320x240 at a low pixel clock) when it can be sourced. The original TW9910-first recommendation was dropped because of availability.
- **Recommended decoder for the P4: ADI ADV7280A-M (or ADV7280-M)** over MIPI CSI-2, YUV422, straight into the hardware JPEG engine. Lifecycle is a concern (see 2.2). The DVP fallback is the same TW9910 as for the S3.
- **P4 is worth it** only if you want more than about 10 fps at 640x480 or above, H.264, on-chip scaling/overlay with zero CPU, or to avoid the S3 capture hack. It does not buy extra range or extra picture at 0.26-1.5 Mbps; those rates are radio-limited.
- **Update (section 11, ESP32-S31):** the S31 adds a hardware JPEG encoder (720p30), PPA scaler and fast PSRAM, but it has no MIPI CSI, no H.264, an unconfirmed BT.656/DE story on its LCD_CAM, and only preview ESP-IDF support. It does not change the first-board pick: build Iris v2 video on the S3 now with the TVP5150AM1 (section 12; TW9910 as fallback), treat the S31 as the upgrade once its camera and JPEG drivers are proven, and use the P4 only for the CSI-2 route.

## 2. Decoder ICs

### 2.1 Comparison table

| Part | Output | Pixel clock | Scaler / decimation | Power (typ.) | Supplies | Package | Status / price | Config |
|---|---|---|---|---|---|---|---|---|
| **TI TVP5150AM1** | 8-bit BT.656 with embedded sync, or 8-bit 4:2:2 with discrete syncs [V] | 27 MHz (crystal) [E] | None listed [V] | 115 mW; power-down under 1 mW [V] | 3.3 V I/O listed; core 1.8 V analog/digital [E] | 32-TQFP 7x7 (PBS); 48-PBGA (ZQC) [V] | TI page: ACTIVE, but "no ongoing design support for new projects" [V]; retailers list NRND (study 3.2). Price not on page. | I2C (0xB8/0xBA) [E] |
| TI TVP5151 | Same family, lower power, 32-VQFN [E] | 27 MHz | None | lower than 5150 [E] | 1.8 V / 3.3 V [E] | 32-QFN [E] | Not checked this session; verify | I2C |
| **ADI ADV7180** | 8-bit BT.656 (16-bit variants too) [S] | 27 MHz LLC [E] | None | about 240 mW for CVBS, derived from mirror currents: DVDD 77 mA, PLL 12 mA, AVDD 33 mA at 1.8 V; power-down 15 uW [S, my sum] | single 1.8 V possible, I/O 1.8-3.3 V [S] | LFCSP-40/32, LQFP-64 [S] | **Not recommended for new designs** (ADI page via search summary) [S] | I2C (0x40/0x42) [E] |
| **ADI ADV7280-M / ADV7280A-M** | **MIPI CSI-2, 1 data lane**, 8-bit YCrCb 4:2:2; non-M parts output 8-bit BT.656 [S] | CSI-2 serial; internal 27 MHz [E] | None known; has deinterlace? No (field-interleaved) [E] | mW not retrieved | Separate PVDD/AVDD/DVDD/MVDD(MIPI)/IOVDD rails, 1.8 V core [S] | LFCSP-32 5x5 [S] | ADV7280 points to ADV7280A; ADV7280A also shown NRND in a search summary [S]. LCSC lists ADV7280ABCPZ-M-RL (C662251) [S]. | I2C (0x40) [E] |
| ADI ADV7281/82/83 (-M) | More inputs (4/6), same decode core, BT.656 or CSI-2; automotive "A/W" grades [S] | 27 MHz | None | - | 1.8 V | LFCSP-40 [E] | Automotive pricing; not needed | I2C |
| **Renesas TW9910** | 8- or 16-bit 4:2:2 YCbCr, **ITU-R 601 (HS/VS) or 656** [S]; HS, VS, CLKx2 pins [S] | 27 MHz crystal (24.54/29.5 MHz for square pixel) [S] | **Built-in scaler: arbitrary downscale, packed output** [S] | <100 mW [S] | 1.8 V analog/digital, 3.3 V I/O [S] | 48-QFN 7x7 (LQFP-44 listed as obsolete) [S/V] | Renesas page: **Active**, TW9910-NB2-GR **$3.19**, in stock (page snapshot) [V] | I2C (0x88/0x8A) [E] |
| **Renesas TW9912** | **Progressive** ITU-R 656 via internal deinterlacer, also interlaced 656; component input [V] | 27 MHz crystal [S] | Scaler not mentioned on product page [V] | "low power", no figure [V] | 1.8 V + 3.3 V I/O [V] | 48-QFN 7x7x0.9 [V] | Active, TW9912-NA3-CR **$2.42** [V] | I2C |
| Renesas TW9990 | BT.656 (S-video and differential CVBS), AEC-Q100 [S] | - | not verified | - | - | 32-WFQFN 5x5 [S] | Full datasheet login-gated; datasheet FN8406 Rev 6.01 (2023) [S] | I2C |
| GM7150 (TVP5150 pin-clone, from memory) | BT.656 | 27 MHz | None | - | - | QFN | Not checked; common on cheap capture cards. [E] | I2C |

Notes on the table:
- TVP5150/5151 and ADV7180 are the best documented and widest supported on Linux (V4L2 drivers exist), which helps if you want to test the decoder on a Pi first. This is an observation from general knowledge [E].
- I could not download the TW9910, ADV7180 or ADV7280A datasheets directly (timeouts, 403). All pin-level facts marked [E] must be checked in the actual PDFs before schematic capture. Specifically verify: (a) the TVP5150's AVID/HSYNC/VSYNC/FID pins in discrete-sync mode, (b) the TW9910 HS/VS timing and the scaler's maximum horizontal/vertical ratio and whether it scales one field to 240 lines, (c) whether the ADV7180 can give a DE-like active-video signal at all (it has HS, VS, FIELD only [E]).

### 2.2 Which parts connect to which MCU

**ESP32-S3 (LCD_CAM, DVP input).**
- 8- or 16-bit DVP slave receive, signals CAM_PCLK, CAM_V_SYNC, CAM_H_SYNC, CAM_H_ENABLE (DE), data lines [S: esp-hal driver docs]. Two sync modes exist: VSYNC+HSYNC+DE (three signals wired) and VSYNC+DE [S]. **DE is required in both modes**, and no BT.656 mode is documented [S, V that it is absent from the driver and issue].
- PCLK: Espressif camera FAQ: "For ESP32-S3, the upper limit of PCLK is 40 MHz" [V]. 27 MHz is inside that.
- YUV422 is captured as raw bytes via GDMA (the esp32-camera framebuffer holds the 2-byte-per-pixel stream). Byte order in the stream will be CbYCrY (UYVY) from a BT.656-style decoder; esp_new_jpeg accepts CbYCrY as an input [V], so no swizzle is needed.
- Consequence for each decoder: the decoder must provide **a DE-type signal**. TVP5150AM1 has AVID, confirmed in its datasheet [V, section 12]. TW9910 provides HS/VS but whether it has an active-video (DE) output is not confirmed [E]. If neither does, the workaround is to feed an inverted HS (high except for the 4.7 us sync pulse) as DE and then crop porch bytes in software (extra roughly 6-19 % bytes on the bus, [E]). This is the single largest risk.
- IDF support: the ESP-IDF `esp_cam_ctlr` camera controller doc exists for the P4, and the same URL for the esp32s3 target returned 404. On the S3 you use the `esp32-camera` component's `cam_hal` or write a bare LCD_CAM + GDMA driver [V for 404, E for conclusion]. The decoder also needs its own small I2C init instead of a sensor driver.

**ESP32-P4.**
- MIPI CSI-2 supports RGB, YUV and RAW data [V, Espressif camera solution page]. ADV7280(A)-M outputs YUV422 over one CSI-2 lane, so the P4 receives it with no ISP (the ISP takes only RAW per the study 3.3).
- DVP exists (LCD_CAM DVP and an ISP DVP path); the ISP-DVP guideline is PCLK under 80 MHz (under 60 MHz with conversion; under 40 MHz at 16-bit) [S, ESP-IDF P4 camera_driver docs via search summary]. The same sync/DE question applies and I found no BT.656 support statement, so DVP on the P4 is not materially better than on the S3 for this job. The clean P4 path is CSI-2.
- A decoder with CSI-2 output removes DE-signal workarounds and the 27 MB/s byte-wide bus.

### 2.3 Recommendations

| MCU | Primary | Why | Fallback |
|---|---|---|---|
| ESP32-S3 | **TVP5150AM1PBSR** (TQFP-32, active, in stock; revised, section 12) | Discrete HS/VS plus programmable AVID as DE [V]; 3.3 V I/O; 115 mW; 14.31818 MHz crystal; software 2:1 decimation | **TW9910-NB2-GR** (QFN-48, $3.19 on Renesas, hard to get): scaler gives 320x240 at a low pixel clock; AVID/DE output unconfirmed |
| ESP32-P4 | **ADV7280A-M** (or -M) over CSI-2 | No parallel-sync risk; YUV422 into hardware JPEG; 1 lane | TW9910 on DVP (same risk as the S3) |

## 3. Compute and memory budget on the S3

### 3.1 Video geometry and bus rates

NTSC active video is 720x480 interlaced, 59.94 fields/s. BT.656 clock 27 MHz, 1716 clocks per 63.56 us line, 1440 of which are active (720 pixels x 2 bytes) [E, standard values].

| Capture mode | Bytes/frame (YUV422) | Burst rate on bus | Average to RAM at 5 fps | at 0.5 fps | Fits internal SRAM? |
|---|---|---|---|---|---|
| Full frame 720x480 (two fields woven) | 691 KB | 27 MB/s (22.7 MB/s average over a line) | 3.5 MB/s | 0.35 MB/s | No (needs PSRAM) |
| One field 720x240 | 346 KB | same | 1.7 MB/s | 0.17 MB/s | Marginal; no (leaves <100 KB) |
| Decoder-scaled 360x240 (TW9910) | 173 KB | about 13.5 MB/s [E] | 0.86 MB/s | 0.09 MB/s | Yes, but one buffer only |
| Decoder-scaled 320x240 (TW9910) | 154 KB | about 12 MB/s [E] | 0.77 MB/s | 0.08 MB/s | Yes |

The decoder always streams 30 frames/s; the S3 only arms its DMA for the field it wants (about 1 in 12 fields for 5 fps, 1 in 120 for 0.5 fps) so the average PSRAM traffic is the "average to RAM" columns. Software frame decimation is therefore free; the burst rate is what matters.

**Capture one field, not a frame.** A rocket that rolls or vibrates will comb badly if two fields are woven. One field is 240 lines and needs no deinterlacing. Pixel aspect on the ground: 720x240 is anamorphic (scale x2 vertically or 0.5 horizontally on display) [E].

### 3.2 Can the S3 absorb the burst?

- Octal PSRAM at 80 MHz DDR: theoretical 160 MB/s; community-measured sustained 40-85 MB/s, and DMA access to external RAM is described by Espressif as "very limited" when the CPU also touches it [S: esp32.com thread, IDF external-ram guide]. 27 MB/s for 15 ms bursts is within the measured range, but it is not guaranteed while a JPEG encode runs on the other core reading PSRAM. No clean GDMA-specific benchmark was found. **Estimate: feasible with margin 1.5-3x, unproven.**
- Quad-SPI PSRAM (not octal) is ruled out; issue 566 author asked whether quad PSRAM was the problem at 37 MHz PCLK [V]. Use the N8R8 or N16R8 (octal) module.
- ESP-IDF v6.1 docs mention a 120 MHz octal PSRAM option with restrictions [S]; not recommended for a flight board without evidence.
- If the bus rate or contention bites: (1) TW9910 scaler (cuts rate and size), (2) GDMA into internal-SRAM line buffers in a ring, with the CPU draining (cost: constant CPU load), (3) lower decoder output (e.g., 13.5 MHz mode, 360 px wide), (4) capture only while the encoder is idle (alternate: capture field N, then encode during the next 11 fields).
- The esp32-camera community noted `CAMERA_FB_IN_DRAM` as an option [V, issue 566]. Works for 154-173 KB frames only.
- Disk/flash: not relevant, the stream is not stored on the S3 (the RunCam SD card keeps the data of record).

### 3.3 Software JPEG speed on the S3

Espressif `esp_new_jpeg` encoder, ESP32-S3, quality 60, 4:2:0 output, single-task, YCbYCr input [V: https://components.espressif.com/components/espressif/esp_new_jpeg]:

| Resolution | Time/frame | fps | Input RGB888 fps |
|---|---|---|---|
| 320x240 | 22.1 ms | 45.3 | 30.4 |
| 480x320 | 41.1 ms | 24.4 | 15.8 |
| 640x480 | 75.5 ms | 13.2 | 8.3 |
| 800x480 | 92.4 ms | 10.8 | 6.7 |
| 1280x720 | 206.6 ms | 4.84 | 2.9 |

Interpolating by pixel count (about 0.25-0.29 us/pixel) [E]:

| Mode | Pixels | Est. encode time (one core) | Est. max fps one core | CPU at 5 fps | CPU at 0.5 fps |
|---|---|---|---|---|---|
| 160x120 | 19.2k | 6 ms | 170 | 3 % | 0.3 % |
| 320x240 | 76.8k | 22 ms [V] | 45 [V] | 11 % | 1 % |
| 360x240 | 86.4k | 25 ms | 40 | 12 % | 1.3 % |
| 720x240 (field) | 172.8k | 45-55 ms | 18-22 | 25-28 % | 3 % |
| 640x480 | 307k | 75 ms [V] | 13 [V] | 38 % | 4 % |
| 720x480 (frame) | 345.6k | 85-100 ms | 10-12 | 45-50 % | 5 % |

Caveats: the benchmark's memory placement (internal vs PSRAM input) is not stated on the page; reading from PSRAM would slow it, estimate +20-50 % [E]. Quality changes timing only slightly (entropy coding is a small part). A second core doubles throughput by running two single-task encoders on alternate frames (the library's dual-task encoder is RGB888-only [V]). Restart-marker (DRI) support in esp_new_jpeg is not documented on the page [V]; plan on strip encoding (see 5).

Other encoders: TJpgEnc, libjpeg-turbo ports and esp-dsp-based encoders exist; I did not find ESP32-S3 benchmark pages for them this session, so none are quoted. esp_new_jpeg is Espressif's own S3-optimised path and is the planning baseline.

### 3.4 S3 vs P4 throughput

| | ESP32-S3 (software) | ESP32-P4 (hardware) |
|---|---|---|
| JPEG | about 45 fps at 320x240, about 10-12 fps at 720x480 per core [V/E] | hardware baseline JPEG: 1080p 34 fps, 720p 88 fps (third-party datasheet mirror) = about 70 Mpixel/s, i.e. roughly 200 fps at 720x480 [S, scaled by me] |
| Input formats | YUV422 (CbYCrY), RGB, gray [V] | RGB888, RGB565, YUV422, gray [S, study 3.3] |
| H.264 | none hardware; software only, throughput not verified [E] | hardware baseline encoder, 1080p30, YUV420 progressive [S] |
| Scale/crop | software (CPU) or decoder scaler | PPA hardware |
| CPU free during encode | no | yes |
| Capture | LCD_CAM DVP + GDMA | CSI-2 (+DVP) + GDMA, ISP bypassed for YUV |

At 320x240 to 360x240 the P4 advantage does not matter because the S3 is already about 10x faster than needed.

## 4. Bitrate math

### 4.1 Net video payload per frame

Assumptions: FLRC at CR 3/4 (effective = 0.75 x raw, study 3.4). Video share of effective rate = 77 % (the study's 300 kbps at 0.52 Mbps raw covers packet headers, GPS-slotted command windows and a telemetry slot; section 17/18). Packet payload 511 B max for FLRC [study]. No retransmission.

| Raw rate | Effective CR 3/4 | Video net (x0.77) | KB/frame at 5 fps | KB/frame at 2 fps | KB/frame at 0.5 fps |
|---|---|---|---|---|---|
| 2.6 Mbps | 1.95 Mbps | 1.50 Mbps | 37.5 | 94 | 375 |
| 1.3 Mbps | 0.975 | 750 kbps | 18.8 | 47 | 188 |
| 0.65 Mbps | 0.4875 | 375 kbps | 9.4 | 23 | 94 |
| **0.52 Mbps** | 0.39 | **300 kbps** | **7.5** | 19 | **75** |
| 0.325 Mbps | 0.244 | 188 kbps | 4.7 | 11.7 | 47 |
| **0.26 Mbps** | 0.195 | **150 kbps** | **3.75** | 9.4 | **37.5** |

Which rate applies where (study 18, 900 MHz FLRC with 30 dBm and 13 dBi Yagi, 130 km): +17.5 dB at 0.52 Mbps, +20 dB at 0.26 Mbps, +9.5 dB at 2.6 Mbps. So near the pad 1.3-2.6 Mbps (19-37 KB/frame at 5 fps) and at apogee 0.26-0.52 Mbps.

### 4.2 What picture fits

JPEG sizes need a bench measurement on real RunCam footage. Rule of thumb for baseline 4:2:0: q50 about 0.8-1.3 bit/pixel on detailed scenes, 0.3-0.6 bit/pixel on sky/plume-dominated ones [E]. Required bits per pixel below = (KB x 8192) / pixels.

| Link / rate | Budget | 320x240 (76.8k px) | 360x240 (86.4k px) | 720x240 field (172.8k px) | 640x480 | 160x120 |
|---|---|---|---|---|---|---|
| 0.26 Mbps, 5 fps | 3.75 KB | 0.40 bpp: q about 12-18, poor | 0.36 bpp: poor | not viable | not viable | 1.6 bpp: q60+, small but clean |
| 0.26 Mbps, 2 fps | 9.4 KB | 1.0 bpp: q50 | 0.89 bpp: q45 | 0.45 bpp: q20 | - | - |
| **0.26 Mbps, 0.5 fps (apogee)** | **37.5 KB** | q95 (overkill) | q90 | **1.8 bpp: q75-85** | 1.0 bpp: q50 | - |
| 0.52 Mbps, 5 fps | 7.5 KB | 0.78 bpp: q35-45 | 0.69 bpp: q30-40 | 0.35 bpp: poor | - | - |
| **0.52 Mbps, 0.5 fps** | **75 KB** | q100 | q100 | 3.5 bpp: q90+ | 2 bpp: q75 | - |
| 1.3 Mbps, 5 fps | 18.8 KB | 2.0 bpp: q75-85 | 1.8 bpp: q75 | 0.9 bpp: q45 | 0.5 bpp: poor | - |
| 2.6 Mbps, 5 fps (near pad) | 37.5 KB | q95+ | q90+ | 1.8 bpp: q75-85 | 1.0 bpp: q50 | - |

Take-aways:
- **At the 0.5 fps apogee target the budget is generous (37.5-75 KB).** A 720x240 field at q70-85 fits at 0.26 Mbps. You could instead run 2 fps at 360x240 q45.
- **At 5 fps the practical size is 320x240 or 360x240 at about q35-45 at 0.52 Mbps raw**, and degrades quickly below that. At 0.26 Mbps raw hold 5 fps only at 160x120, or drop to 2-2.5 fps at 320x240.
- The study's table (section 4) tiers T1-T4 (320x240 about 8 KB, 160x120 about 2 KB) are consistent with these numbers.

### 4.3 Does H.264 change the answer?

- Not at 0.5 fps: every frame must be an IDR, so a frame is about JPEG-sized (study 4).
- At 5 fps, H.264 baseline would be roughly 2-3x smaller than MJPEG for static-camera-like content [E, study rule of thumb], but loss of any P slice corrupts a GOP and there is no retransmission. Rocket-camera content changes constantly (motion and roll), which cuts the gain.
- H.264 on the P4 is hardware; on the S3 it would be software (not verified, likely single-digit fps at 320x240) [E]. **Verdict: H.264 is not a reason to pick the P4 at these rates.** Mjpeg-with-strips is the more robust design for a lossy link.

## 5. Packetisation and loss (sizing only)

- 7.5 KB frame = 15 FLRC packets of 511 B; 37.5 KB = 74 packets; 75 KB = 147 packets.
- With 1 % PER (the datasheet sensitivity point) a 15-packet frame arrives intact about 86 % of the time; a 74-packet frame about 47 %; a 147-packet frame about 23 % (1-0.01)^n. **Whole-frame JPEGs at 0.5 fps will often have holes** unless the margin is large (study 18 gives +14 to +17 dB with polarisation loss, which should drive PER well below 1 %) or you add FEC.
- Remedy: encode **independent strips** (for example 4-8 horizontal strips of 720x30-60 lines, each its own small JPEG) or use restart markers if esp_new_jpeg supports DRI (not documented). A lost packet then costs one strip. Each strip carries a header of about 300-600 bytes (Huffman/quant tables, estimated) unless the ground side holds the tables and the rocket sends only scan data (custom framing, extra work). Account 4-8 % overhead for 4-8 strips [E].
- Add simple XOR parity every N packets (study section 7 covers erasure schemes).

## 6. RAM

| Item | Size | Where |
|---|---|---|
| Capture buffer 320x240 YUV422 | 154 KB | internal SRAM (S3 has about 512 KB; roughly 330-400 KB usable depending on Wi-Fi/BT off and heap config [E]) |
| Capture buffer 360x240 | 173 KB | internal SRAM |
| Capture buffer 720x240 | 346 KB | PSRAM |
| Capture buffer 720x480 | 691 KB | PSRAM |
| Double buffer (capture N+1 while encoding N) | 2x the above | PSRAM for anything above 360x240 |
| esp_new_jpeg mono encoder working memory | about 10 KB DRAM [V] | internal |
| JPEG output | 4-75 KB, size the buffer for worst case (for example 80 KB) | PSRAM OK |
| GDMA descriptors | small; must be in internal RAM (cannot live in PSRAM) [S, IDF external-ram guide] | internal |
| Radio TX queue (RingBuf of 511 B packets, about 100 packets) | about 50-60 KB | internal or PSRAM |
| Stack/heap/FreeRTOS/CAN/GPS tasks | 60-100 KB [E] | internal |

- **Iris v2 as a separate S3 board with 8 MB octal PSRAM (N8R8)**: all of the above fits easily; use internal SRAM for the 320x240 capture buffer if the TW9910 is used, PSRAM for the rest.
- **Same S3 as Witness**: not realistic. Witness currently has about 35 KB free heap (task statement) and runs the flight state machine, GPS, flash logger, CAN and the Ra-01SH-P radio. A 154 KB capture buffer alone exceeds that by 4x, and a PSRAM contention burst from the camera DMA would perturb the flight tasks' timing. Also, Witness's hardware may not have PSRAM; its module choice (flash/PSRAM variant) should be checked [E]. **Keep video on a separate board** and talk to Witness over CAN (low rate, already in Iris design), as study section 13 assumed.

## 7. Risks

| Risk | Severity | Notes / mitigation |
|---|---|---|
| S3 has no BT.656 mode; needs DE signal | **High** | Pick a decoder that has an active-video pin (TVP5150 AVID, verify) or use inverted HS as DE with software crop. Prototype with a TW9910/TVP5150 eval board plus a dev kit before any PCB. |
| PSRAM DMA contention with encode | Medium | Capture to internal SRAM (TW9910 scaler), or serialise capture and encode; measure. |
| No public reference for S3 + analog decoder | Medium | Issue 566 shows a failed attempt at 37 MHz PCLK [V]. Expect to write a bare LCD_CAM + GDMA driver. |
| Interlace combing, roll, vibration | Medium | One field only; accept 240 lines. |
| JPEG packet loss | Medium | Strip encoding plus FEC, see 5. |
| Decoder lifecycle | Low-Medium | TVP5150/ADV7180/ADV7280 are NRND or "not recommended" per ADI/TI pages [V/S]; TW9910/TW9912 listed Active on Renesas [V]. Buy last-time stock of the chosen part. |
| Thermal/power | Low | S3 with PSRAM at 240 MHz both cores busy is about 0.3-0.5 W [E]; decoder 0.1-0.25 W; video board is dominated by the 3.5-4 W PA (study 18). Flight is minutes. |
| RunCam analog level/format | Unknown | Output connector, level, NTSC timing and actual line count unverified (study 3.1). A 1 Vpp CVBS into 75 ohm is the usual expectation [E]. |

## 8. When the P4 is worth it

- You want 640x480 or 720x480 at 5+ fps, or any frame rate above about 10 fps at those sizes.
- You want H.264 for a high-rate (greater than 1.5 Mbps) near-pad link.
- You need zero-CPU scaling/overlay (PPA) or ISP/colour processing on a second camera.
- You want MIPI CSI-2 to eliminate the DE/sync workaround. Counter-point: the ADV7280(A)-M is marked not recommended for new designs, so this advantage rests on an ageing part.
- You are fine with a bigger board (P4 needs external PSRAM/flash and has no radio or Wi-Fi).
For the study's link rates (150-1500 kbps net), none of these change what the radio carries.

## 9. Bill of materials

**Option A: ESP32-S3 "Iris v2" video board (recommended first try).** Prices unverified except the decoder.
| Item | Part | Note |
|---|---|---|
| MCU module | ESP32-S3-WROOM-1 N8R8 (octal PSRAM) or N16R8 | octal PSRAM required |
| Decoder (revised) | TI TVP5150AM1PBSR, 32-TQFP, about $4-9 [V/S, section 12] | AVID as DE; needs 1.8 V + 3.3 V rails and a 14.31818 MHz crystal. Renesas TW9910-NB2-GR (48-QFN, $3.19) is the fallback |
| Decoder fallback | TI TVP5150AM1 (TQFP-32) | AVID as DE, 27 MHz crystal |
| Crystal | 14.31818 MHz (TVP5150AM1; datasheet requires this, not 27 MHz) [V] | the TW9910 fallback uses 27 MHz |
| Analog front end | 75 ohm termination, 0.1 uF coupling, TVS | RunCam composite in |
| Power | 1.8 V LDO (decoder core), 3.3 V buck, 5 V boost for the radio (study 18) | 1.8 V and 3.3 V rails |
| Radio | NiceRF LoRa2021F33-2G4 (study 17/18) | SPI |
| CAN | transceiver to Iris/Witness | as in the Iris design |
| Debug | USB, boot/reset, test pads for DVP bus | scope access to PCLK/VS/HS/DE |

**Option B: ESP32-P4 board.**
| Item | Part | Note |
|---|---|---|
| MCU | ESP32-P4 module with 16-32 MB PSRAM and flash | module choice and price unverified |
| Decoder | ADI ADV7280A-M (LFCSP-32, CSI-2 1 lane), LCSC C662251 [S] | 1.8 V rails, MIPI-routed 100 ohm pair; check lifecycle |
| Alternative decoder | TW9910 on DVP | same DE risk as S3 |
| Rest | same as option A | P4 has no radio; same SPI radio module |

## 10. Next steps (not done)

1. Download and read the real TW9910 (Renesas FN7773), TVP5150AM1 (TI SLES209) and ADV7280A datasheets; confirm AVID/DE availability, scaler limits and exact supply pins.
2. Read the ESP32-S3 TRM LCD_CAM chapter for exact CAM_H_ENABLE, VSYNC filter and byte-count constraints. I could not extract it in this session.
3. Buy a TW9910 or TVP5150 eval board and an S3 N8R8 dev kit; capture one field into PSRAM from a camera with a known NTSC pattern; measure bus integrity, PSRAM DMA throughput while a second core encodes, and JPEG sizes on real RunCam footage.
4. Measure esp_new_jpeg DRI/strip behaviour and per-strip overhead.

## 11. ESP32-S31 vs ESP32-S3 (added 2026-11-15)

### 11.1 What the S31 is

| Item | ESP32-S31 | ESP32-S3 (for comparison) |
|---|---|---|
| CPU | 2x RISC-V RV32IMAFC, 5-stage, up to 320 MHz, single-precision FPU; 128-bit data bus with SIMD instructions on **one** core only; LP RV32IMAC core 40 MHz [V datasheet v0.5 PRELIMINARY] | 2x Xtensa LX7, 240 MHz, PIE SIMD [E] |
| CoreMark | 2195 dual-core at 320 MHz (6.86/MHz) [V] | about 1180 dual at 240 MHz [E, recalled datasheet value] |
| SRAM | 512 KB HP SRAM, 32 KB LP SRAM, 320 KB ROM; 32 KB L1 I-cache, 64 KB L1 D-cache [V] | 512 KB, 32 KB caches [E] |
| PSRAM | In-package only (not pinned out): 16 MB (NRV16) or 32 MB (NRV32), octal 8-bit DDR at **250 MHz**, 1.8 V [V]. Peak about 500 MB/s theoretical [E, my arithmetic; Espressif gives no bandwidth figure]. Concurrent flash and PSRAM access [V] | external or in-module octal at 80 MHz, 40-85 MB/s measured [S] |
| Camera | LCD_CAM block with "Camera slave RX mode" and "Camera master RX mode", external DVP sensor, 8/16-bit, pins via GPIO matrix, data to internal or external memory via GDMA [V]. Pin table lists CAM_PCLK, CAM_XCLK, CAM_V_SYNC, CAM_H_SYNC [V]. **Max PCLK, BT.656/embedded sync and VSYNC/HSYNC/DE modes are not described in the datasheet text I could read** [V that it is absent] | same family of block; PCLK up to 40 MHz per FAQ [V] |
| MIPI CSI-2 | **None** (not mentioned in the datasheet or product page) [V] | none |
| JPEG | **Hardware codec, encoder and decoder.** Encode input RGB888, RGB565, YUV444, YUV422, YUV420, GRAY; compression for YUV444/422/420; still up to 4K; **moving images up to 720p@30fps** (excluding header time); 4 configurable quantisation tables, canonical Huffman [V] | software only |
| H.264 | **Not mentioned** anywhere [V]; treat as absent | none |
| PPA | Scale/rotate/mirror (SRM) and blend; SRM formats ARGB8888, RGB888, RGB565, YUV422, YUV420, GRAY [V]. 2D-DMA with macroblock reordering and colour conversion, supports PPA and JPEG [V] | none |
| ISP | not mentioned [V] | none |
| Radios | Wi-Fi 6 2.4 GHz 1T1R, BLE 5.4, BT Classic, 802.15.4 (Thread/Zigbee) [V]. No sub-GHz/FLRC; the LR2021 stays an external SPI part | Wi-Fi 4, BLE 5 |
| Other | CAN FD controller (ISO 11898-1:2015), 1000 Mbps Ethernet MAC, USB 2.0 HS OTG, 2 SDIO, 4 UART, 6 SPI, 2 I2C [V] | classic TWAI |
| Package | QFN80 8x8 mm, 60 GPIO, -40 to 85 C [V] | QFN56 |
| Status | Announced 2026-03-26 "upcoming" [V]; Espressif FAQ: "officially entered mass production", chips and modules in capacity ramp-up, chips/modules through channels July-August 2026, dev boards already purchasable [V]. LinuxGizmos 2026-07-28 reports mass production, ESP32-S31-WROOM-3 module about $5.84 and bare ESP32-S31NRV16 about $4.51 on AliExpress [S]. A Mouser-hosted ESP32-S31-WROOM-1 datasheet is a confidential pre-release v0.1 [S] | mature |
| ESP-IDF | Preview support on master and the v6.1 branch; v6.1-beta1/rc1 pre-releases; **first official supported tag is v6.1.1**; Espressif says to build from latest master, and a status page marks unsupported/unknown features [V/S]. I could not see a per-peripheral table (LCD_CAM, JPEG, PPA support status unknown) | mature (v5.x/v6.0.x) |

Check on arrival: whether v6.1.1 has shipped by your build date, and the per-peripheral support table at https://developer.espressif.com/hardware/esp32s31/ (loaded empty for me).

### 11.2 Does the S31 remove the BT.656 / DE risk?

**Not demonstrably.** The S31 LCD_CAM is the same style of block as the S3 (slave RX, DVP, 8/16-bit, VSYNC/HSYNC pads). Neither the datasheet text I read nor the product pages describe BT.656 embedded-sync decoding or whether a data-enable (H_ENABLE) input is mandatory [V that it is undocumented]. So the default assumption stays: **the decoder must drive PCLK + VSYNC + HSYNC/DE as discrete lines** [E]. Potential improvements that are also unconfirmed:
- The TRM for the S31 (not found; only the datasheet was reachable) may document sync modes and max PCLK. Read it before any commitment.
- A faster, better-documented master/slave camera mode might make the "inverted HS as DE" workaround easier, but nothing says so.
- The PPA can scale a captured 720x240 field to 360x240 (E, YUV422 SRM is listed) and the JPEG block takes YUV422, so the TW9910 scaler is no longer needed for size; it remains useful only to halve the DVP clock.

How each decoder would connect to the S31:

| Decoder | Connect to S31? | How |
|---|---|---|
| TW9910 | Yes, DVP | 8-bit data, PCLK/CLKx2, HS, VS; the same sync caveat as on the S3 |
| TVP5150AM1 | Yes, DVP | 8-bit, discrete HSYNC/VSYNC/AVID, AVID as DE (verify AVID availability) |
| ADV7180 | Yes, DVP | 8-bit, HS/VS/FIELD; no known DE, so the HS-as-DE workaround; not recommended for new designs [S] |
| ADV7280A-M | **No** | CSI-2 output; S31 has no CSI. A non-M ADV7280A (BT.656 parallel) would work in DVP mode with the same sync question |
| ADV7280-M/ADV7282-M (CSI) | No | same reason |

So the S31 gives no escape from the DE issue and removes the CSI option that the P4 has.

### 11.3 Encode and memory budget

Hardware JPEG at 720p30 is 27.6 Mpixel/s [V, peak spec, excluding headers]. Scaled by pixel count (E):

| Mode | Pixels | S3 software (one core, section 3.3) | S31 hardware JPEG (E) | S31 software (E) |
|---|---|---|---|---|
| 320x240 | 76.8k | 22 ms [V] | about 3 ms | about 12-22 ms |
| 360x240 | 86.4k | about 25 ms | about 3 ms | about 14-25 ms |
| 720x240 | 172.8k | about 45-55 ms | about 6-7 ms | about 25-50 ms |
| 720x480 | 345.6k | about 85-100 ms | about 13 ms | about 50-100 ms |

Notes:
- Hardware numbers assume the 720p30 peak applies at SD size and that you ignore header time (add roughly 1 ms for tables, E). The measured Espressif benchmark of the S31 JPEG engine was not found; no esp_new_jpeg S31 figure is published that I could reach. The P4 engine is 1080p34 [S]; the S31's is spec'd at 720p30, i.e. about 2.5x slower than the P4 on paper.
- S31 software encode is estimated from per-MHz scalar performance only (6.86 vs about 4.9 CoreMark/MHz dual-core per the figures above, at 320 vs 240 MHz, about 1.4x x 1.3 = 1.8x), but esp_new_jpeg's S3 speed relies on PIE SIMD assembly. Without an S31-optimised build, software encode should be assumed no faster than the S3 (1.0-1.8x). Do not plan on software encode on the S31; the hardware block makes it unnecessary.
- At the link budgets of section 4.1, the encode time is irrelevant on the S31 (under 7 ms for a 720x240 field):

| Link budget | KB/frame | S3 (SW, 360x240) CPU load | S31 (HW, 720x240) |
|---|---|---|---|
| 0.52 Mbps raw, 5 fps | 7.5 | about 12 % of one core | about 3 % of the codec, CPU nearly idle; but 720x240 at 7.5 KB is 0.35 bit/pixel, so you would scale to 360x240 (PPA) or 320x240 anyway |
| 0.26 Mbps raw, 0.5 fps | 37.5 | about 1.3 % | under 0.5 %; 720x240 field at q75-85 as before |
| 2.6 Mbps raw, 5 fps | 37.5 | 12-28 % | negligible; 720x240 at q75-85 |

- **Quality is bitrate-limited, not engine-limited.** The S31 hardware JPEG does not shrink frames at a given quality; it only frees CPU. Picture outcomes in section 4.2 apply unchanged. The hardware encoder's output may differ from esp_new_jpeg in table/header layout and restart-marker (DRI) support (unknown, E); strips would then be separate encodes, which costs 4-8 hardware job launches per frame, still trivial.
- **Memory:** same internal 512 KB; 320x240/360x240 buffers (154-173 KB) fit in SRAM, larger go to the in-package 16/32 MB PSRAM. PSRAM is in-package at 250 MHz DDR (about 500 MB/s peak, E), so the 27 MB/s capture burst should be a much smaller fraction of bandwidth than on the 80 MHz octal S3, which lowers the contention risk (E, unmeasured). Note 2D-DMA/JPEG/PPA reads also use that bus.
- Thermal/power: not published in the pages read; hardware JPEG should draw less than software encode at 320 MHz (E).

### 11.4 Verdict across S3, S31 and P4 for an Iris v2 video board

| | ESP32-S3 | ESP32-S31 | ESP32-P4 |
|---|---|---|---|
| Hardware JPEG | no (software, 22 ms at 320x240) | yes, 720p30 [V] | yes, 1080p34 [S] |
| H.264 | no | no [V absence] | yes [S] |
| Scaler | no (decoder or CPU) | PPA SRM [V] | PPA [S] |
| Camera input | DVP, 40 MHz max [V], BT.656 undocumented | DVP 8/16-bit, max PCLK and BT.656 undocumented | CSI-2 2-lane and DVP [V/S] |
| Decoder choices | TW9910, TVP5150, ADV7180, non-M ADV7280A | same as S3 | those plus ADV7280A-M via CSI |
| PSRAM | octal 80 MHz external, 40-85 MB/s | in-package octal 250 MHz DDR, about 500 MB/s peak | in-package/external, larger |
| IDF maturity | mature, stable, many examples | preview only (master/v6.1), first official tag v6.1.1 [V/S] | released; camera, JPEG, H.264 drivers exist [S] |
| Module availability | WROOM-1 N8R8/N16R8, plentiful | chip about $4.5, WROOM-3 about $5.8 on AliExpress [S]; official channels ramping July-Aug 2026; dev boards buyable [V] | modules/boards exist [E, not rechecked] |
| CAN | classic TWAI (CAN FD not supported) | CAN FD [V] | 3 TWAI [study] |
| Fits the link-limited job | yes | yes, with CPU to spare | overkill |

Recommendation:
1. **First Iris v2 video board: ESP32-S3 (N8R8) + TI TVP5150AM1 (revised in section 12; originally TW9910, now the fallback).** It is the only option with a mature SDK, abundant modules and a CPU budget that already exceeds the radio's needs (12 % of a core at 5 fps and 320x240). The one genuine unknown (DE/sync capture) exists on every DVP option, so there is no schedule or risk benefit in waiting for a different chip.
2. **Treat the S31 as the upgrade path, not the starting point.** What it adds for this job is hardware JPEG and a PPA scaler (so full 720x240 or 720x480 at 5 fps costs no CPU), 250 MHz PSRAM headroom, and CAN FD. Switch if (a) the S31 TRM documents the sync/DE modes and a max PCLK of at least 27 MHz, (b) an IDF tag (v6.1.1 or later) ships working LCD_CAM and JPEG-codec drivers (check the support table), and (c) the module is available from a normal channel. A board layout that keeps the DVP bus, I2C, SPI-to-radio and CAN on broken-out headers lets the S3 be swapped.
3. **P4 only if you want the CSI-2 route (ADV7280A-M) or H.264** (neither of which the link rates justify).
4. If the S31 TRM shows BT.656 embedded-sync support (not found), revisit: that would be the first option that removes the main risk, and TVP5150 (8-bit BT.656 is native, [V TI]) would become the simplest decoder.

Quick comparison of the S31's value for this mission: it removes CPU load, not the capture risk, and it does not add range or picture quality at the study's link rates.

## 12. TVP5150 with the ESP32-S3 (added 2026-11-15)

Primary source: TI TVP5150AM1 datasheet SLES209E (Nov 2007, rev. Oct 2011), https://www.ti.com/lit/ds/symlink/tvp5150am1.pdf, downloaded and text-extracted locally; all [V] items below are from it unless another source is named. TVP5151 facts come from TI's product page only.

### 12.1 Output modes and sync pins [V]

- **Two output formats, selected by register 0Dh bits [2:0]** (default 47h): `111` = 8-bit ITU-R BT.656 with embedded SAV/EAV (default), `000` = **8-bit 4:2:2 YCbCr with discrete syncs**. The datasheet's own example for discrete sync writes 0Dh = 40h (discrete sync, extended code range) and 03h = 0Dh. Bit 7 of 0Dh also picks BT.601 (16-235) vs extended (1-254) code range; bit 6 CbCr code format.
- **Pins (32-TQFP "PBS" numbering):** YOUT[6:0] pins 12-18, YOUT7/I2CSEL pin 11, **SCLK pin 9 (output, 2x the pixel clock = 27 MHz, i.e. one byte per clock)**, **HSYNC pin 25, VSYNC/PALI pin 24, FID/GLCO pin 23, AVID pin 26**, INTREQ/GPCL/VBLK pin 27, SCL pin 21, SDA pin 22, XTAL1 pin 5, XTAL2 pin 6, PDN pin 28, RESETB pin 8, IO_DVDD pin 10.
- **Power-up state:** HSYNC, VSYNC/PALI, AVID, FID and INTREQ are high-impedance by default; SCLK and YOUT stay high-Z after reset until the chip is initialised. Enable with **register 03h** (bit 0 SCLK enable, bit 2 sync outputs enable (HSYNC, VSYNC/PALI, AVID, FID), bit 3 YCbCr output enable, bit 5 INTREQ/GPCL/VBLK enable, bit 1 VBLK on/off). The datasheet's "write 0Dh to 03h" enables data, syncs and clock. A second enable bit, register C2h bit 2 (VDPOE), defaults to 1.
- **Pin sharing (register 0Fh, default 08h):** pin 24 = VSYNC (bit 2 = 0) or PALI (bit 2 = 1); pin 23 = FID (bit 3 = 0) or GLCO (bit 3 = 1; **the default is GLCO, so set bit 3 = 0 to get FID**); pin 27 = INTREQ, GPCL or VBLK (bit 1, plus 03h bit 7). A 10 kohm pull-down is mandatory on pin 27 whenever it is disabled.
- **AVID is a real data-enable.** "AVID ... is high during the horizontal active time of the video output." It crops active video horizontally and vertically (section 3.13): horizontal start/stop are programmable in registers **11h-12h (start, 10-bit signed, relative to default) and 13h-14h (stop)**; the stop/start also move the embedded SAV/EAV codes. The active pixel count must be even. **Vertical AVID gating follows the VBLK window (registers 18h start / 19h stop, line-programmable +-127/128 lines)**, and **register 12h bit 2 = 1 forces AVID inactive during vertical blanking** (default 0 = AVID toggles even in VBLK). So with 12h bit 2 = 1 AVID only pulses on active lines, which is exactly the S3's DE. Caveat: the cropping does not blank the data outside AVID, and the datasheet does not say if the 03h VBLK enable bit must also be set for the vertical window to apply [E, verify on the bench].
- **HSYNC** position is programmable (register 16h, steps of 4 pixel clocks; Nhbhs = 28 SCLKs and Nhb = 272 SCLKs for NTSC). VSYNC and FID are per field (FID = 1 for odd field). Timing of VSYNC/FID/VBLK relative to NTSC line numbers is in the datasheet figures 3-5/3-6.
- **Output timing at SCLK (15 pF load):** SCLK high/low 13.4-16.4 ns (about 14.5 typ.), rise/fall 2-5 ns, **output hold >= 2 ns, output delay 3-8 ns after the SCLK edge** (data and AVID/HS/VS change shortly after the rising edge). So sample on the falling edge, or invert PCLK on the S3 (the esp32-camera driver has a clock-invert setting, per issue 566 [V]). A 27 MHz period is 37 ns, leaving a valid window of roughly 29 ns (8 ns to 37 ns plus 2 ns hold) centred on the falling edge at about 18 ns.
- **Colour order:** BT.656 and the 4:2:2 mode both stream Cb Y Cr Y, which esp_new_jpeg accepts as CbYCrY [V, section 3.3].

### 12.2 Wiring to the S3 and capturing

Wiring (3.3 V logic on both sides, so no level shifters): YOUT[7:0] to the 8 CAM data pins, SCLK to CAM_PCLK, VSYNC to CAM_V_SYNC, AVID to CAM_H_ENABLE (DE), HSYNC to CAM_H_SYNC (optional if DE+VSYNC mode is used), FID to a plain GPIO (optional), SCL/SDA to an I2C port, PDN/RESETB to GPIOs. YOUT7 carries the I2C address pull resistor (greater than 1 kohm), so keep 3.3 V loading on that net high-impedance; avoid S3 strapping pins for the data bus.

Configuration of the S3 camera block (concept, since no driver exists for a decoder):
- **PCLK:** the S3's camera port accepts up to 40 MHz PCLK [V, Espressif FAQ]; 27 MHz is inside that. The S3 is in "camera slave RX" mode (the decoder drives the clock) [V, esp-hal docs]; 8-bit bus; PCLK inverted if needed (section 12.1 timing); VSYNC filter threshold can be set to ignore glitches [V, esp-hal].
- **Sync mode:** use **VSYNC + DE** or **VSYNC + HSYNC + DE**, with AVID on DE [S, esp-hal driver docs describe these two modes and require DE in both]. The AVID-as-DE assumption is what removes the "no DE" problem identified earlier. It is untested on an actual S3.
- **IDF/driver:** `esp_cam_ctlr` documents only the P4 (404 for the S3 target) [V]; use the `esp32-camera` component's `cam_hal` internals (the YUV422 path) or a thin bare LCD_CAM + GDMA driver, plus a small TVP5150 init routine over I2C (about 8 register writes: 03h, 0Dh, 0Fh, 12h/11h/13h/14h, 18h/19h, optionally 0Ah-0Ch picture controls).
- **I2C addresses [V]:** write 0xB8 / read 0xB9 when I2CSEL = 0 (7-bit 0x5C), write 0xBA / read 0xBB when I2CSEL = 1 (7-bit 0x5D); sampled at reset.
- **Capturing one field:** the decoder asserts VSYNC on every field (about 59.94 per second). The S3 arms its GDMA/camera RX to take one VSYNC-to-VSYNC window and ignores the others. FID can be read in a GPIO ISR on the VSYNC edge to pick odd or even; any single field is fine (the half-line offset between odd and even fields is cosmetic). Capture 1 field in N by simply not arming for the other N-1 (N about 12 for 5 fps, about 120 for 0.5 fps).
- **Horizontal crop:** use the AVID start/stop registers: 720 pixels = 1440 bytes per line is the default; 704 or 640 pixels (even) are legal [V, stop minus start must give even pixels]. A 640-pixel window (1280 bytes per line) trims the NTSC overscan and makes a 2:1 decimation land on exactly 320 pixels.
- **Capture size:** AVID-gated active-line data: 240 lines x 1440 B = 345.6 KB per field at 720 wide (NTSC has 240 active lines per field in the ITU-R 601 definition; some decoders give 243 [E]), or 240 x 1280 B = 307 KB at 640 wide.

### 12.3 Downscaling without a scaler, and cost [E]

The TVP5150 has no scaler. Options and estimated S3 cost at 240 MHz, per captured field:

| Method | Output | How | Cost (estimate) |
|---|---|---|---|
| AVID crop to 640, 2:1 horizontal decimate (average each pair of pixels, keep CbYCrY packing) | 320x240 | 4 input bytes pairs to 4 output bytes, using 32-bit loads and masked adds | about 2-4 ms per field if the source line is in internal SRAM, about 8-15 ms if read from PSRAM through the cache (cache-miss limited at 30-40 MB/s) |
| Full 720 crop, 2:1 decimate | 360x240 | same loop | same as above |
| Nearest-neighbour (drop every second macropixel) | 360x240 | copy 4 of every 8 bytes | about 1-2 ms SRAM, 5-10 ms PSRAM; adds aliasing, hurts little at JPEG q35-45 |
| No decimation, JPEG the whole field | 720x240 (anamorphic) | none | no extra step, encode about 45-55 ms (section 3.3); only sensible at 0.5 fps (37.5-75 KB budget) |
| Vertical decimate | 720x120 etc. | skip lines | trivial; not useful, resolution is already 240 lines |

Reading it: decimation costs less than the JPEG encode itself (22-25 ms at 320x240/360x240) and a fraction of a core at 5 fps (about 3-8 % of one core for decimation plus 11-12 % for encode). **At 0.5 fps decimation is irrelevant.**

A cleverer variant avoids PSRAM entirely: GDMA drops lines into a ring of two 8-16 line chunks in internal SRAM (about 23 KB), and the CPU decimates each chunk into a 154-173 KB buffer that also lives in internal SRAM. Feasible in principle (the S3 camera DMA supports line/length EOF events [S, esp-hal docs: byte-length EOF and line interrupt]) but needs a custom driver and a chunk-ready ISR roughly every 63.6 us x lines per chunk (for 16 lines about 1 ms) [E].

### 12.4 Memory and bandwidth [reusing section 3]

- AVID gating means the bus carries only active data: 1440 B per 63.56 us line = **22.7 MB/s average, 27 MB/s within the line**, for about 15.3 ms per 240-line field (of the 16.7 ms field period), and nothing during blanking.
- At 720x240 YUV422 the field is **346 KB** and goes to PSRAM (internal SRAM is too small once the stack and radio are in). The octal PSRAM at 80 MHz measured 40-85 MB/s sustained in forum reports [S], so the capture burst is about 25-70 % of the measured number, but contention with the CPU is uncharacterised.
- **Field skipping eases it a great deal:** with 1 field in 12 (5 fps) the bus is busy at 22.7 MB/s only 15 ms out of every 200 ms (7.6 % duty), and the second core can be held off the PSRAM (encode runs from a copy in SRAM, or is simply sequenced to start after the field ends: decimate -> JPEG -> radio queue, all in the remaining 185 ms). At 0.5 fps the duty is 0.76 %. Average PSRAM traffic 1.7 MB/s (5 fps) and 0.17 MB/s (0.5 fps), as in section 3.1.
- The no-PSRAM line-ring variant (section 12.3) removes the question; a bench measurement should compare the two.
- Memory budget with 640-wide crop: field 307 KB (PSRAM), decimated 320x240 154 KB (SRAM), JPEG output 8-80 KB, radio queue 50-60 KB; consistent with section 6.

### 12.5 Practicalities

| Item | TVP5150AM1 [V unless noted] | TVP5151 [V TI page, otherwise E] |
|---|---|---|
| Supplies | DVDD, PLL_AVDD, CH_AVDD 1.8 V (1.65-1.95 V); IO_DVDD 3.3 V (abs. max 4.5 V); absolute max on the 1.8 V rails 2.3 V | I/O 1.8-3.3 V; core 1.8 V [E] |
| Clock | **14.31818 MHz crystal** (or TTL oscillator into XTAL1), 100 kohm parallel resistor recommended; internal line-locked PLL makes the 27 MHz SCLK | **27.000 MHz** crystal or oscillator |
| Power | 115 mW typ., 150 mW max (the datasheet's supply-current rows extracted garbled, so per-rail currents are not quoted; budget about 60 mA on the 1.8 V rails and about 5 mA on 3.3 V [E]); power-down under 1 mW (PDN pin 28) | 138 mW typ., <1 mW power-down |
| Package | 32-TQFP "PBS", 7x7 mm per TI product page [V]; the DigiKey page lists 32-TQFP (5x5) [V], so measure the footprint against the datasheet's mechanical drawing; 48-PBGA (ZQC) | 32-terminal TQFP (PBS) 7x7 [V] |
| I2C | 0xB8/0xBA write, 400 kHz max, I2CSEL = YOUT7 pull | not checked |
| Temp | commercial 0-70 C; industrial (I suffix) -40 to 85 C; automotive Q1 variants listed | not checked |
| Orderable | TVP5150AM1PBSR (commercial, T&R 1000), TVP5150AM1PBS (tray), TVP5150AM1IPBSR (industrial, -40 to 85 C), IPBSRQ1 (AEC-Q100); datasheet table lists IPBSR "Active, Production" | similar |
| Lifecycle | TI page: ACTIVE, but "no ongoing design support for new projects" (same wording as the TVP5151 page) | same |
| Stock/price (snapshots) | DigiKey JP page: **729 in stock**, JPY 1365 (1), 893 (100), 761 (1000 reel), 26-week lead time, Active [V]; Mouser **3505 in stock at $4.16** (aggregator snapshot) [S]; LCSC has C10013 and a variant C2652020 (3 in stock, $6.50) [S]; counterfeit/grey-market risk on cheap marketplace chips [E] | not checked |
| Linux | mainline `drivers/media/i2c/tvp5150.c` supports TVP5150A/AM1 and TVP5151 [S] | same driver |

Existing designs and proof points:
- **STM32MP157 + TVP5151 over DCMI** (ST community): working PAL/NTSC capture, but with **embedded syncs** because the board lacked the discrete-sync wires; the author needed per-field DMA at 1440 bytes per line and notes the camera interface FIFO is only 4 words deep and cropping was unavailable in embedded mode [V, https://community.st.com/t5/stm32-mpus-solutions/dcmi-with-embedded-syncs-bt-656/m-p/582944]. That shows the chip's output timing works with a parallel capture block, not that the S3 does.
- **STM32F7 DCMI + TVP5150AM1** (TI E2E): black screen, because the DCMI expects progressive data and the decoder was interlaced [S, https://e2e.ti.com/support/data_converters/videoconverters/f/918/t/695140]; a reminder to capture per field.
- **Altium PB01** reference board with a TVP5150AM1 composite input [S, https://techdocs.altium.com/display/HWARE/PB01+Resources+-+Video+Input+Capture]; i.MX6 and BeagleBone-class Linux boards use the mainline driver [S].
- **No ESP32 or ESP32-S3 + TVP5150 project found** [S]. The only public S3 analog-decoder attempt (esp32-camera issue 566) used a different decoder and did not reach a result [V]. So the S3 side stays unproven.

### 12.6 Verdict: TVP5150AM1 + ESP32-S3

**Yes, it works on paper, and it is now the recommended first-board decoder.**

What it fixes: the single biggest S3 risk was "no data-enable". The TVP5150AM1 gives a programmable AVID that goes low in vertical blanking (12h bit 2), so CAM_H_ENABLE sees only active pixels, VSYNC per field, an HSYNC if wanted, a 27 MHz byte clock well under the S3's 40 MHz PCLK limit, 3.3 V I/O, and a horizontal crop in registers. It is in stock at DigiKey (729) and Mouser (about 3500, snapshot) at about $4-9, in a hand-solderable 32-TQFP, with a mainline Linux driver that lets you debug the decoder on a Pi or other Linux board first.

Remaining risks, in order:
1. **Untested on an S3 (High-Medium).** No known S3 + TVP5150 design; DE + VSYNC mode, PCLK polarity and byte-count EOF must be proved on the bench. Mitigation: breadboard a TVP5150 module (cheap USB capture dongles and eval boards use it or the GM7150 clone [E]) to an S3 dev kit before PCB.
2. **AVID vertical window not fully specified (Medium-Low).** Whether the VBLK enable bit must be set for the vertical window; AVID rising edge is 4 SCLKs early in BT.656 mode only (not our mode) [V]. Validate with a scope or logic analyser.
3. **No scaler (Low).** Costs 3-15 ms/field; fits comfortably (section 12.3).
4. **PSRAM contention (Medium-Low).** Eased by field skipping and sequencing (12.4), or by the SRAM line-ring variant.
5. **Lifecycle (Medium).** No design support from TI for new projects and a long (26 week) DigiKey lead time. Buy a lifetime quantity; 5-10 chips cover prototypes and flight spares. Mouser/LCSC counterfeit risk: buy from DigiKey or Mouser direct.
6. **Rails and clock (Low).** Needs a 1.8 V rail (about 60 mA) and a 14.31818 MHz crystal (cheap, common). Do not use a 27 MHz crystal (that is the TVP5151/TW99xx requirement); this was wrong in an earlier draft and in one vendor-page summary.
7. **Analog input level and format (Low-Medium).** The analog inputs (AIP1A/AIP1B) are ac-coupled (0.1 uF) and accept at most 0.75 Vpp; the datasheet says a 1.5 Vpp-class source "may require an attenuator" and its reference schematic (section 6.1) shows the termination/divider network [V]. Copy that network and check the RunCam's output level; the RunCam analog level is still unverified.

**Recommendation changes:** S3 decoder = **TVP5150AM1PBSR** (primary), TW9910 (fallback if you can get it; it adds a scaler but its AVID/DE output is still unconfirmed), TVP5151 only if you need a 27 MHz crystal system or 1.8 V I/O (same lifecycle note). BOM and sections 1, 2.3 and 11.4 updated accordingly.

**S31 later:** the same TVP5150 wiring applies (DVP, AVID as DE). The S31 datasheet lists CAM_V_SYNC and CAM_H_SYNC pads and "Camera slave RX mode" but no max PCLK or DE description [V]; confirm 27 MHz and a DE input before layout. Its PPA (scale/rotate with YUV422 input [V]) could do the 2:1 decimation in hardware (E) and its JPEG engine consumes YUV422, removing the 3-15 ms software step; this is a benefit only, not a prerequisite. Design the TVP5150 daughter-card with a header so the same decoder board moves from the S3 to the S31 or the P4's DVP port.

### 12.7 Other decoders with sync outputs (stock snapshots, all [S] unless noted)

| Part | Sync outputs / DE | Stock and price snapshot | Comment |
|---|---|---|---|
| TVP5150AM1 | HS, VS, FID, **AVID** [V] | DigiKey 729 (JPY 1365 at 1); Mouser 3505 at $4.16 | recommended |
| TVP5151 | discrete syncs; AVID not stated on TI page [V] | not checked | 27 MHz crystal, 138 mW |
| ADV7180 (BCPZ, parallel) | HS, VS, FIELD [E]; no AVID known | DigiKey 208 at $18.68 (BCPZ tray) | "not recommended for new designs"; about 4x the price; HS-as-DE workaround only |
| ADV7280A non-M (ADV7280AWBCPZ, parallel BT.656) | embedded sync, HS/VS available [E] | Mouser 218 at $27.9-31.1; DigiKey 0 in stock, 13-18 week lead | expensive, automotive WB grade; NRND per search summary |
| TW9912-NA3-CR | progressive BT.656 via deinterlacer; sync pins not checked | Mouser 260 at $5.41 (3.37 at 260 pcs); Renesas page $2.42 | progressive output at 27 MHz means 2x data and no benefit here; HS/VS/DE unknown |
| TW9910 | HS, VS, CLKx2 [S] | Renesas page $3.19 in stock; user reports hard to get | scaler; DE unconfirmed |
| TW9990 | BT.656; datasheet login-gated | not checked | 5x5 WFQFN, AEC-Q100; no |
| GM7150 (TVP5150 clone) | like TVP5150 [E] | marketplace only | grey-market; avoid for flight |

## Sources

- esp_new_jpeg component and ESP32-S3 benchmark table: https://components.espressif.com/components/espressif/esp_new_jpeg [V]
- Espressif camera FAQ (S3 PCLK upper limit 40 MHz, XCLK, 8-bit only): https://docs.espressif.com/projects/espressif-esp-faq/en/latest/application-solution/camera-application.html [V]
- Espressif camera solution page (DVP vs CSI-2, measured fps, no hardware JPEG on S3): https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/solution-introduction/camera/dvp-mipi-csi-camera-solution.html [V]
- esp-hal LCD_CAM camera driver source (sync modes, DE, DMA required): https://docs.rs/crate/esp-hal/1.1.1/source/src/lcd_cam/cam.rs [V]
- esp32-camera issue 566, PAL decoder on S3 (no working result): https://github.com/espressif/esp32-camera/issues/566 [V]
- ESP-IDF P4 camera controller driver doc (LCD_CAM DVP, ISP DVP PCLK guidance, via search summary): https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/camera_driver.html [S]
- ESP-IDF external RAM guide (DMA bandwidth to PSRAM limited, descriptors not in PSRAM, 120 MHz octal option): https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-guides/external-ram.html and https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-guides/external-ram.html [S]
- PSRAM bandwidth forum measurements: https://esp32.com/viewtopic.php?p=112250, https://esp32.com/viewtopic.php?p=94255, https://esp32.com/viewtopic.php?p=103811 [S]
- ESP32-S3 Technical Reference Manual (LCD_CAM chapter, not read): https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf [not verified]
- TI TVP5150AM1: https://www.ti.com/product/TVP5150AM1 [V]
- ADI ADV7180: https://www.analog.com/en/products/adv7180.html and datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/ADV7180.pdf; mirror with currents: https://html.alldatasheet.net/html-pdf/166576/AD/ADV7180/204/7/ADV7180.html [S]
- ADI ADV7280/ADV7280A and user guides: https://www.analog.com/en/products/adv7280.html, https://www.analog.com/en/products/adv7280a.html, https://www.analog.com/media/en/technical-documentation/user-guides/ADV7280A-7281A-7282A-UG-1176.pdf; LCSC listing https://www.lcsc.com/product-detail/C662251.html [S]
- Renesas TW9910: https://www.renesas.com/en/products/tw9910 [V]; TW9912: https://www.renesas.com/en/products/tw9912 [V]; TW9990: https://www.renesas.com/en/products/tw9990 [S]; TW9910 datasheet summaries (scaler, power) via https://datasheets.globalspec.com/ds/Renesas-Electronics/tw9910-nb2-gr/9f207943-43dc-46ce-b5b1-e33fc94587a2 and https://www.renesas.com/node/494726 [S]
- ESP32-P4 datasheet mirror (JPEG, H.264 figures, as used in the study): https://cdn.compacttool.ru/downloads/ESP32_P4_Chip_Datasheet_EN.pdf [S]
- Link budget and rates: `Software/VIDEO_DOWNLINK_STUDY.md` sections 3.4, 4, 17, 18.
- ESP32-S31 Series Datasheet v0.5 PRELIMINARY (CPU, memory, JPEG codec, LCD_CAM, PPA, radios, CAN FD): https://documentation.espressif.com/esp32-s31_datasheet_en.html [V; only the first ~200 KB of text read, LCD_CAM section lacks sync/PCLK detail]
- ESP32-S31 product page: https://www.espressif.com/en/products/socs/esp32-s31 [V]; announcement: https://www.espressif.com/en/news/ESP32_S31_Release [V]
- ESP32-S31 availability and ESP-IDF timeline FAQ: https://esp32-s31.espressif.com/en/faq [V]; developer status page: https://developer.espressif.com/hardware/esp32s31/ [V, peripheral table did not load]
- LinuxGizmos mass-production report with module/chip prices: https://linuxgizmos.com/espressif-begins-mass-production-of-esp32-s31-wireless-risc-v-soc/ [S]
- ESP32-S31-WROOM-1 pre-release datasheet (not read): https://www.mouser.com/catalog/specsheets/Espressif%20_07-27-2026_ESP32-S31-WROOM.pdf [S]
- ESP-IDF ESP32-S31 get-started: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s31/get-started/index.html [S]
- TI TVP5150AM1 datasheet SLES209E (output formatter, pins, registers 03h/0Dh/0Fh/11h-14h/16h/18h/19h, AVID, I2C addresses, supplies, timing): https://www.ti.com/lit/ds/symlink/tvp5150am1.pdf [V, text-extracted locally]; product page https://www.ti.com/product/TVP5150AM1 [V]; TVP5151 page https://www.ti.com/product/TVP5151 [V]
- DigiKey TVP5150AM1PBSR: https://www.digikey.jp/en/products/detail/texas-instruments/TVP5150AM1PBSR/657835 [V, snapshot]; Mouser/LCSC aggregators: https://origin-www.findchips.com/search/TVP5150AM1PBSR, https://www.lcsc.com/product-image/C10013.html [S]
- ST community STM32MP157 + TVP5151 DCMI: https://community.st.com/t5/stm32-mpus-solutions/dcmi-with-embedded-syncs-bt-656/m-p/582944 [V]; TI E2E STM32F7 + TVP5150: https://e2e.ti.com/support/data_converters/videoconverters/f/918/t/695140 [S]
- Linux driver mirror: https://elixir.free-electrons.com/linux/v5.4.26/source/drivers/media/i2c/tvp5150.c [S]
- ADI/Renesas stock snapshots: https://www.digikey.com.br/en/products/detail/analog-devices-inc/ADV7280AWBCPZ/7915249, https://azcus.digikey.com/en/products/detail/analog-devices-inc/ADV7180BCPZ/1217622 [S]
