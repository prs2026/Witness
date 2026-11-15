# Witness v0.2 U2 (3.3 V regulator) - replacement options

Research only. Nothing ordered, no hardware touched. Date of research: 2026-11-13.

## 1. What is wrong with the current part

- Schematic (`power.kicad_sch`): U2 symbol pins are 1 = GND, 2 = VOUT (+3V3), 3 = VIN (+BATT); footprint `Package_TO_SOT_SMD:SOT-89-3`, so pin 2 is the large tab = VOUT.
- Placed part: ME6118A33PG (MICRONE, LCSC C236721, SOT-89-3).
- ME6118 datasheet V10.0 (Micro One, copy at LCSC C236722 / C151064), "Pin Configuration" page 3, ME6118AXX table:

  | Package | GND | VOUT | VIN |
  |---|---|---|---|
  | SOT-223 (A, ...B3G) | 1 | 2 | 3 |
  | **SOT-89-3 (A, ...PG)** | **1** | **3** | **2** |
  | TO-252-2L | 1 | 2 | 3 |

  So the SOT-89 ME6118A33PG is **pin1 GND / pin2 VIN (tab) / pin3 VOUT**. On the board pin 2 is +3V3 and pin 3 is +BATT, so the regulator is installed backwards (input on the output net). The 4.3 V reading is consistent with the output net being driven through the part's body path from +BATT (a reverse-connected LDO, roughly a diode drop or pass-device path from the battery-side pin).
- Likely root cause: the schematic symbol is named `Witness:ME6118A33B3G` (the **SOT-223** pin order GND/VOUT/VIN, symbol description even says SOT-23-5), but the placed value/LCSC part is the SOT-89 `...PG`, whose pin order is different.
- Other ME6118 suffixes: the ordering table (page 2) lists the SOT-89 part only as ME6118A33PG. The "different pin" variants (E = GND/VIN/VOUT, H = VOUT/GND/VIN, L = VIN/GND/VOUT) exist only in SOT-223 / SOT-23-3 / TO-252-2L (ME6118E33B3G, ME6118H33M3G, ME6118L50K3G). **No ME6118 suffix gives GND/VOUT/VIN in SOT-89-3.** Same story for the sibling ME6209 (250 mA only, has a "P1G" different-pin SOT-89 but far too little current).
- ME6118A33PG key data for comparison (datasheet): Vin abs max 18 V, 800 mA, dropout typ 80 mV@100 mA, 400 mV@500 mA(typ), 1.3 V max @1 A, Iq 53 uA, Tj -40..+125 C, thermal shutdown 160 C, Ilimit 1.1 A, thetaJA SOT-89 = 56 C/W (datasheet figure), output cap "10 uF min if tantalum, ESR < 0.5 ohm" (ceramic not explicitly blessed). Electrically it is the best part for this job; it is just the wrong pin order.

## 2. Battery voltage (not documented anywhere; inferred)

- No schematic note/doc states the chemistry. Backplane J2 is just `J2_BATT_RAW` (JST B02B-PASK) through a P-FET (SQA403EJ) to +BATT.
- Evidence for 2S Li-ion/LiPo (6.0 - 8.4 V): HITL simulator default `battery_mv = 7800` draining 4 mV/min (`Software/HITL/sim_model.py:93`, `trajectory_csv.py:379`), fake Witness `batt: 7400`, tests use 7800/8123 mV; Aperture design doc says "thresholds ... 2S? voltage unknown". VSENSE divider is 15k/5k (x0.25), which would read up to ~12 V on a 12 dB ESP32-S3 ADC range, so 3S (12.6 V) is not excluded by the hardware; the Iris audit mentions 3S in passing. No firmware low-battery threshold exists.
- **Assumption used below: 2S (6.0 - 8.4 V), with 3S (up to 12.6 V) as a stretch case. 1S (3.0 - 4.2 V) is evaluated only to say that nothing in this package/pinout works.** Please confirm the actual pack.
- USB path: VBUS 5 V minus PMEG2020EH Schottky (~0.2-0.3 V) = ~4.7 V at the regulator input.

## 3. Thermal reality check (applies to every SOT-89 linear regulator)

Power = (Vin - 3.3 V) x I. SOT-89 thetaJA: 56 C/W (MICRONE figure, good copper), 150-170 C/W (Diodes/UTC datasheets, minimum pad). Real value depends on the copper under the tab (the tab is VOUT, so the +3V3 pour is the heat sink).

| Vin | 200 mA | 350 mA (Wi-Fi) | 500 mA |
|---|---|---|---|
| 4.7 V (USB) | 0.28 W | 0.49 W | 0.70 W |
| 6.0 V (2S empty) | 0.54 W | 0.95 W | 1.35 W |
| 7.4 V (2S nominal) | 0.82 W | 1.4 W | 2.05 W |
| 8.4 V (2S full) | 1.02 W | 1.8 W | 2.55 W |
| 12.6 V (3S full) | 1.9 W | 3.3 W | 4.65 W |

At theta = 60 C/W a 1 W dissipation is +60 C junction rise; 2 W is +120 C (thermal shutdown territory in a hot box). Conclusion: on 2S, a SOT-89 linear is fine for the idle/GPS/IMU load and short bursts but will thermally limit during sustained Wi-Fi (maintenance mode) or repeated LoRa TX at full pack voltage, whichever part is chosen. On 3S it is not viable. A buck regulator (or buck pre-regulator) is the real fix if the average current exceeds roughly 250 mA at 8.4 V; that is outside "drop-in" scope but is flagged here.

## 4. Candidate table

Pinout column: pin1 / pin2 / pin3 as printed in the datasheet top view; "tab" = pin 2. Required: GND / VOUT / VIN.

Distributor data: LCSC from the LCSC product pages / public product API (stock is a snapshot, prices USD). DigiKey from its search page. **Mouser could not be fetched (site timed out / blocked automated access); no Mouser stock or price was obtained for any part.** Where I give a Mouser number it is a guess and marked.

| Part (exact orderable) | Mfr | Pinout 1/2/3 (datasheet ref) | Iout max | Dropout @500 mA / @1 A | Vin max | Iq | Ceramic stable | LCSC # | DigiKey # | Mouser # | Price (qty 10) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **AZ1117CR-3.3TRG1** | Diodes Inc. | GND / OUT (tab) / IN. AZ1117C datasheet DS36676 Rev 5-2, p.2, "SOT89 ... R Package". **VERIFIED** | 0.8 A rated (1.35 A typ limit) | ~1.1 V @0.5 A (AZ1117-family typ 1.08 V max 1.18 V; C-variant table garbled in text extraction); ~1.2 V @0.8 A | abs 18 V, rec 15 V | 6 mA max | **Y, datasheet states "compatible with low ESR ceramic", needs >= 10 uF out, ESR < 20 (unit lost in extraction)** | C460428 (listing: ~1700 in stock, ~$0.12 @5+; stock/price not shown by page fetch) | AZ1117CR-3.3TRG1DICT-ND, 57,020 in stock | not obtained (likely 621-AZ1117CR-3.3TRG1, UNCONFIRMED) | DigiKey $0.176 | Tj -20..+125 C only. thetaJA SOT-89 150-170 C/W (datasheet). Marking G28N. **Best overall pick.** |
| AZ1117CR2-3.3TRG1 | Diodes Inc. | GND / **IN** / OUT (R2 package, same datasheet p.2). **WRONG ORDER, same as ME6118** | - | - | - | - | - | C362111 (do NOT order) | - | - | - | The "R2" suffix is the trap: swapped pins. |
| **LD1117AG-33-AB3-A-R** | UTC (Unisonic) | GND / OUT / IN. UTC LD1117/A datasheet QW-R102-006.AC: pin assignment code "A: GOI" for AB3 = SOT-89, "I:VIN O:VOUT G:GND". **VERIFIED (via suffix code)** | 0.8 A | ~1.2 V @0.8 A (datasheet); @1 A not rated | abs 18 V, rec 15 V | ~5 mA (LCSC) | datasheet lists "Support MLCC" as a feature; cap value details not extracted - treat as probable, test | C81768 (listing ~5,900 in stock; price not shown by page) | not checked | not checked | n/a | thetaJA SOT-89 165 C/W (datasheet). Tj -20..+125 C. **The "-C-" suffix (LD1117AG-33-AB3-C-R, LCSC C427616) is pin code "C: GIO" = GND/IN/OUT = WRONG ORDER.** Only the "-A-" suffix fits. |
| AMS1117-3.3 (SOT-89), UMW/YONGYUTAI datasheet | UMW | GND / OUTPUT / INPUT. Datasheet p.1 "1. GND 2. OUTPUT 3. INPUT". **VERIFIED for the UMW datasheet** | 1 A | 1.08-1.18 V @0.5 A, 1.15-1.25 V @1 A | abs 20 V, rec 15 V | 5-10 mA | **Uncertain/N-ish: ripple test uses 22 uF tantalum; no ceramic statement** | UMW part not checked | - | - | - | Tj 0..+125 C. thetaJA 46 to >90 C/W depending on copper. |
| AMS1117-3.3 (TWGMC) | TWGMC | (SOT-89 AMS1117 convention GND/OUT/IN) **UNVERIFIED for this vendor** | 1 A | ~1.15 V @1 A | 18 V | 5 mA | Uncertain | C47283698, 3,220 in stock | - | - | $0.048 | Tj -40..+125 C. |
| AMS1117-3.3V | MSKSEMI | **UNVERIFIED** (datasheet fetch returned an HTML redirect) | 1 A | ~1.15 V @1 A | 15 V | 12 mA | Uncertain | C5120796, 273,410 in stock | - | - | $0.054 | Tj 0..+125 C. |
| AMS1117-3.3 | FOSAN | **UNVERIFIED** | 1 A | ~1.15 V @1 A | 15 V | 2 mA | Uncertain | C20628883, 20,130 in stock | - | - | $0.039 | Tj -40..+125 C. |
| AMS1117-3.3(SOT-89) | ALJ | **UNVERIFIED** | 1 A | ~1.15 V @1 A | 15 V (listing) | 60 uA? (listing) | Uncertain | C22751496, 0 stock | - | - | $0.044 | Out of stock. |
| AMS1117-3.3(XBLW) | XBLW | **UNVERIFIED** | 1 A | ~1.15 V @1 A | 15 V | 4 mA | Uncertain | C18723569, 0 stock | - | - | - | Out of stock. |
| AMS1117-3.3S / "AMS1117-3.3 SOT-89" | EVVO / Slkor | **UNVERIFIED** | 1 A | ~1.15 V @1 A | ~15 V | ~5 mA | Uncertain | - | EVVO: 5,326 in stock (1 pc $0.30); Slkor: 361 in stock (1 pc $0.10); DK PNs not shown | - | n/a (page showed 1 and 1000 breaks only) | Fetch of DigiKey results page did not show DK numbers. |
| CJA1117B-3.3 | JSCJ (CJ) | **UNVERIFIED** | 1 A | ~1.2 V | 15 V | 5 mA | Uncertain | C24253, 103,170 in stock | - | - | $0.066 | Tj -25..+125 C. |
| KMA1117-3.3V | Kexin | **UNVERIFIED** | 1 A | ~1.2 V | 12 V | n/a | Uncertain | C489391, 290 in stock | - | - | $0.061 | 12 V max rules out 3S. |
| U1117-3.3V-SOT89 | Uni-Semic | **UNVERIFIED** | 1.1 A | ~1.2 V | 18 V | 1.8 mA | Uncertain | C22398252, 180 in stock | - | - | - | Tj -20..+85 C ambient. |
| LD1117AG-33-AB3-C-R | UTC | GND / **IN** / OUT (code "C: GIO"). **WRONG ORDER** | - | - | - | - | - | C427616 | - | - | - | Do not order. |
| ME6118A33PG (current) | MICRONE | GND / **IN (tab)** / OUT. ME6118 V10.0 p.3. **Confirmed wrong for this footprint** | 0.8 A | 0.4 V typ @0.5 A, 1.3 V max @1 A | abs 18 V | 53 uA | probably (10 uF, ESR < 0.5) | C236721, 17,510 in stock | - | - | - | Best electrical part, wrong order. |
| ME6209A33P1G | MICRONE | "different pin assignment" SOT-89-3 (see p.3 of ME6209 datasheet; not resolved) | **0.25 A only** | - | 18 V | ~uA | - | n/a | - | - | - | Rejected: current. |
| ELM85xxxxA | Elm Technology | CMOS 600/800 mA, ceramic-friendly, SOT-89 listed; **datasheet unreachable (connection refused), pinout UNVERIFIED** | 0.6-0.8 A | ~0.1 V @100 mA | ? (likely <= 6 V) | ? | Y (per summary) | not found | - | - | - | Not usable for 2S (low Vin) in any case. |

Rejected for the stated requirements (checked, not usable): AP2114H-3.3 (SOT-223 only, Vin 6 V), AP7215-33YG (600 mA, 5.5 V), MCP1700 / HT7533 / LR7533 / LR8341A / LTP3559 (100-350 mA), ME6209 (250 mA), SGM2013 (SOT-89 pin1 = OUT, wrong order), NIKO L1089 (tab = input, wrong), LD2981 (100 mA, VIN on pin 1).

## 5. Recommendations

Overall finding: the only SOT-89-3 parts I could find with the exact GND / VOUT(tab) / VIN order and >= 800 mA are the "1117" family (LM1117/AMS1117/AZ1117/LD1117 lineage, which has this order by convention on SOT-89). All have ~1.1-1.25 V dropout, mA-level Iq, and 15 V recommended Vin. No ceramic-stable CMOS 1 A LDO with this exact SOT-89 pin order was found.

### (a) 1S LiPo (3.0 - 4.2 V)
**No acceptable drop-in exists.** A 1117 needs Vin >= 4.4-4.6 V for 3.3 V at 0.5-1 A, so the rail would collapse below ~4.5 V of battery (most of the 1S discharge curve). The ME6118 itself (1.3 V max @1 A, 0.4 V typ @0.5 A) would only just work, but only in the wrong pin order. Options: a different footprint (e.g. SOT-223 ME6118A33B3G, which has the footprint's GND/VOUT/VIN order, or a boost-buck), or a boost to 5 V first. Nothing ranked here.

### (b) 2S or higher (6.0 - 8.4 V; 3S up to 12.6 V)
1. **AZ1117CR-3.3TRG1** (Diodes; LCSC C460428, DigiKey AZ1117CR-3.3TRG1DICT-ND, Mouser not obtained). Pinout verified, ceramic compatibility stated, Vin abs 18 V. Meets 2S; thermally limited (see section 3); not suitable for 3S continuous use.
2. **LD1117AG-33-AB3-A-R** (UTC; LCSC C81768; DK/Mouser not checked). Pinout verified via suffix code A (GOI); must be the "-A-" suffix, never "-C-". Advertises MLCC support. 800 mA.
3. **AMS1117-3.3 SOT-89, UMW** (pinout verified in UMW datasheet; cheap alternates TWGMC C47283698 in stock, MSKSEMI C5120796 in stock, FOSAN C20628883 in stock are UNVERIFIED). 1 A rating, but the datasheet qualifies stability with 22 uF tantalum only; with 2 x 10 uF ceramic (effective ~13 uF after DC bias) it may oscillate or ring. Needs bench check (scope the 3V3 rail at load steps) before flight.

Avoid: any "R2"/"-C-" suffix (swapped pin order), and any vendor whose datasheet shows only a generic 1117 SOT-89 drawing without a pin table.

### Caveats
- Dropout margin from USB (~4.7 V in): 3.3 V + 1.15-1.25 V = 4.45-4.55 V needed at 1 A; only ~0.15-0.25 V margin, so USB power with simultaneous Wi-Fi + LoRa TX may sag. Fine at <= 500 mA.
- The "ESR < 20" figure in AZ1117C Note 4 lost its unit in text extraction; confirm in the PDF (DS36676 p.3).
- AZ1117C dropout/Iq table was garbled by PDF text extraction; dropout figures given are the family values (AZ1117 DS36736, 1.08 V max 1.18 V @0.5 A, 1.15 V typ 1.25 V max @1 A). Operating Tj is only -20 C (AZ1117C/LD1117A) to +125 C, not -40 C, if cold-soak flights matter.
- Stock/prices are snapshots from 2026-11-13; LCSC price at qty 10 was not shown for the Diodes/UTC parts by the page fetch (only the public API prices for some AMS1117 clones).
- Mouser could not be queried for any part.

## 6. Sources
- ME6118 datasheet V10.0: https://atta.szlcsc.com/upload/public/pdf/source/20180709/C236722_8C795604A62DB63E4DCCB1C55FB5CFAC.pdf
- AZ1117C datasheet DS36676: https://www.diodes.com/assets/Datasheets/AZ1117C.pdf ; AZ1117 (older, NRND) DS36736: https://diodes.com/datasheet/download/AZ1117.pdf
- UTC LD1117/A: https://uploadcdn.oneyac.com/attachments/files/brand_pdf/utc/17/3D/UTC-LD1117A.pdf
- UMW AMS1117-3.3 SOT-89: https://uploadcdn.oneyac.com/attachments/files/brand_pdf/yongyutai/65/99/AMS1117-3.3.pdf
- ME6209 datasheet (P1G note): https://pf.unikeyic.com/datasheet/87/3b/d67d/873bd67d73420fdb2be8d9b4585ea32e.pdf
- DigiKey AZ1117CR-3.3TRG1 listing (search page), LCSC product pages C460428, C81768, C236721 and others as cited.

## 7. 1S LiPo options (dropout <= 400 mV @ 600 mA)

Research only, 2026-11-13. Nothing ordered. Requirement set: fixed 3.3 V, >= 600 mA (ideally 1 A), dropout <= 400 mV at 600 mA, ceramic-stable (2 x 10 uF X5R out, 10 uF in), Vin max >= 6 V, SOT-89-3 with pin 1 = GND, pin 2 (tab) = VOUT, pin 3 = VIN. Load: ESP32-S3 + Ra-01SH-P (SX1262/PA, ~690 mA TX peaks) + sensors/NAND. Input range 3.0-4.2 V (LiPo) or ~4.6-4.7 V (USB through Schottky).

### 7.1 Finding

**A true drop-in exists on paper: UTC LR1108G-33-AB3-A-R** (CMOS, SOT-89, pin code A = GND / VOUT / VIN, 1 A, 300 mV typ / 500 mV max at 1 A, Vin 2.5-7.0 V). It is the only part found that satisfies pin order, current, Vin and (derived) dropout. Caveats, in order of importance:

1. **Sourcing is unconfirmed.** I could not find LR1108G-33-AB3-A-R on LCSC, JLCPCB or DigiKey (DigiKey search for "LR1108G-33-AB3" returns nothing; Mouser not reachable). Only other UTC LR110x parts are on LCSC (LR1107, LR1108G-25-AA3-D-R C211651). It would have to come via UTC or a UTC-authorised distributor, probably with MOQ and lead time.
2. **Dropout at 600 mA is not a datasheet number.** The datasheet gives VD = 300 mV typ / 500 mV max at IL = 1 A only (TJ 25 C, VOUT >= 2.5 V, Note 8: output 2 % below nominal), plus a typical Dropout-vs-Iout graph (axis 0-350 mV vs 0-1000 mA; values not extractable from the PDF text). A PMOS pass device is resistive, so 600 mA is roughly 0.6 x: **~180 mV typ / ~300 mV max (my extrapolation, not a spec)**. That meets 400 mV with margin, but confirm on the bench.
3. Ceramic stability: test conditions COUT = 2.2 uF, typical application CIN 1 uF / COUT 4.7 uF, "low output capacitor required"; no ESR/stability chart. 2 x 10 uF X5R is probably fine but undocumented. Scope the rail on load steps.
4. Vin operating 2.5-7.0 V (survival 7.5 V): fine for USB (~4.6 V). Operating Tj 125 C max, Topr -40..+125 C. Ground-pin current 200 uA at no load, 300 uA at 1 A.
5. The SOT-89 version has no SD/ERROR pin (those exist only on the SOT-25 version), so nothing to tie off.
6. Datasheet QW-R102-031.I (copyright 2012) came from a mirror (datasheetq.com). I could not open the LCSC-hosted copy to check for a newer pin table; check UTC's current PDF before ordering.

**Suffix trap (UTC LR1108 SOT-89 pin codes, datasheet pin table):** A = GND(1)/VOUT(2)/VIN(3) = **correct**. C = GND/VIN/VOUT = wrong (same fault as ME6118). D = VIN/GND/VOUT = wrong. Order **-AB3-A-R** only. The sibling **LR1107G-33-AB3-C-R (LCSC C171498)** is the one that shows up on LCSC but is code C (GND/VIN/VOUT) = wrong; the LR1107 datasheet lists SOT-89 codes B (VOUT/GND/VIN), C, D, so it has **no** GND/VOUT/VIN SOT-89 option (the ordering table text was interleaved in extraction, so verify visually). It also fails on dropout: 500 mV max at 600 mA.

### 7.2 Candidate table

Mouser could not be fetched (timeouts), so no Mouser numbers for any part. LCSC page fetches did not return stock/price. "-" = not found / not applicable.

| Part (exact suffix) | Maker | Package + pin order (datasheet ref) | Iout | Dropout @ 600 mA (typ / max, conditions) | Vin max | Iq | Ceramic stable | LCSC # | DigiKey # | Mouser # | Price @10 | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **LR1108G-33-AB3-A-R** | UTC (Unisonic) | SOT-89, code A: 1 GND / 2 VOUT / 3 VIN. UTC LR1108 QW-R102-031.I "Pin Description". **DROP-IN** | 1 A DC (2 A short-circuit, thermal SD 165 C) | ~180 / ~300 mV **extrapolated** from 300 / 500 mV @ 1 A (TJ 25 C, VIN = VOUT+1 V, 2 % criterion) | 7.0 V op (7.5 survival) | 200 uA | Likely (2.2 uF test, 4.7 uF typ. app), no ESR chart | not listed | not listed | - | - | Sourcing unconfirmed. thetaJA SOT-89 165 C/W (datasheet). |
| LR1108G-33-AB3-C-R / -D-R | UTC | SOT-89, C: GND/VIN/VOUT, D: VIN/GND/VOUT. **WRONG ORDER** | - | - | - | - | - | - | - | - | - | Do not order. |
| LR1107G-33-AB3-C-R | UTC | SOT-89 C: GND/VIN/VOUT. **WRONG ORDER**. QW-R102-033.L | 600 mA | 300 / 500 mV @ 600 mA | 7.0 V | 200 uA | probably | C171498 | - | - | - | Wrong pins and fails 400 mV max. |
| AP7365-33YG-13 | Diodes | SOT-89 "Y": 1 **VOUT** / 2 (tab) **GND** / 3 VIN. DS32260 Rev 11-3 p.1-2. **WRONG ORDER** | 600 mA | 300 / **400 mV** @ 600 mA (VOUT >= 2.5 V, 2 % criterion, VIN = VOUT+1 V) | 6.0 V op (6.5 abs) | 35 uA | Yes, >= 1 uF, ESR > 15 mohm | not found | AP7365-33YG-13DICT-ND | - | $0.275 | **NRND, "use AP7366"** (AP7366 has no SOT-89). 40-week lead time on DigiKey. Tab is GND. |
| AP7365-33YRG-13 | Diodes | SOT-89 "YR": 1 GND / 2 (tab) **VIN** / 3 VOUT. **WRONG ORDER** (same as ME6118) | 600 mA | 300 / 400 mV | 6.0 V | 35 uA | Yes | - | 31-AP7365-33YRG-13TR-ND (2,674 listed) | - | - | Electrically a good match, wrong pins. |
| AP7361C-33Y5-13 | Diodes | **SOT-89-5**: 1 EN / 2 GND (tab) / 3 NC / 4 IN / 5 OUT. AP7361C datasheet Oct 2020 p.1 | 1 A | 360 mV typ @ 1 A (headline; 600 mA table garbled in extraction, est. 200-250 mV) | 6.0 V op (6.5 abs) | 60 uA | Yes, >= 1 uF in, >= 2.2 uF out | - | AP7361C-33Y5-13DICT-ND (92 in stock) | - | $0.403 | Bodge candidate (7.4). thetaJA 95 C/W. -40..+85 C. |
| TLV757P, 3.3 V version (confirm exact orderable on TI page) | TI | SOT-23-5 (DBV) / SOT-23 (DYD); not SOT-89 | 1 A | 300 / 450 mV @ 1 A (3.3 V, DYD); ~180 / ~270 mV @ 600 mA by linear scaling | 5.5 V (6.0 abs) | 25 uA | Yes, 1 uF | - | - | - | - | Next-rev or adapter part. thetaJA 60 C/W (DYD). 5.5 V max is OK for 4.2 V and 4.7 V inputs. |
| ME6118A33PG (installed) | MICRONE | SOT-89 GND / VIN / VOUT = wrong | 0.8 A | 0.4 V typ @ 0.5 A | 18 V | 53 uA | - | C236721 | - | - | - | Reference only. |
| ME6231A33PG | MICRONE | SOT-89-3: 1 VSS / 2 VIN / 3 VOUT = wrong | 0.5 A | 125 mV @ 100 mA only | 18 V | 1.8 uA | Yes | C2681684 | - | - | - | Rejected: wrong pins, 500 mA. |
| ME6217A33PG | MICRONE | SOT-89-3: VSS / VIN / VOUT = wrong | 0.8 A | 100 mV @ 300 mA only | 6.5 V (7.0 abs) | 100 uA | "small ceramic" | - | - | - | - | Rejected: wrong pins; dropout specified only at 300 mA. |
| ME6216A33PG | MICRONE | SOT-89 | 0.3 A | 240 mV @ 200 mA | 6 V | 4 uA | Yes | C126467 | - | - | - | Rejected: 300 mA. |
| ME6214A33PG | MICRONE | SOT-89-3 | 0.3 A | 160 mV @ 100 mA | 18 V | - | - | C427600 | - | - | - | Rejected: 300 mA. |

Also checked and rejected: AP2114H-3.3 (BCD/Diodes, SOT-223 only, 450 mV typ / 750 mV max @ 1 A); TPL820F33-89 (3PEAK, SOT-89-3 but 180 mA, 0.72 V, pin 1 = OUT); NCP694H33HT1G (onsemi, SOT-89-5, 1 A, ~180 mV typ, LCSC says discontinued); SGM2013 (300 mA); LP3993 (200 mA); HT73xx/HT75xx, LR7533, XC6201/XC6206/XC6214 (100-300 mA); XC6222 (700 mA but SOT-89-5: 1 CE, 4 VIN, 5 VOUT); UTC LR1106 (400 mA); the whole 1117 family (1.1-1.3 V dropout, useless at 1S). ELM85xxxxA (600/800 mA CMOS, SOT-89 option): the ELM datasheet host refused the connection, so pinout and Vin are still unverified. Nisshinbo NJM2884 / R1173 (650 mA - 1 A) appear only as SOT-89-5 (not checked in detail). No SOT-89-3 part from Torex, Richtek, onsemi, TI, Microchip, Rohm, ABLIC or Nisshinbo with 600 mA+ and the right order turned up.

### 7.3 Thermal check (1S, SOT-89)

P = (Vin - 3.3 V) x I.

| Condition | Vin | I | P | rise at 165 C/W (UTC min-pad figure) | rise at ~60 C/W (good copper under the VOUT tab) |
|---|---|---|---|---|---|
| Fresh LiPo, continuous LoRa TX | 4.2 V | 0.60 A | 0.54 W | +89 C | +32 C |
| Fresh LiPo, 0.3 A | 4.2 V | 0.30 A | 0.27 W | +45 C | +16 C |
| Nominal LiPo, TX | 3.7 V | 0.60 A | 0.24 W | +40 C | +14 C |
| USB, TX | 4.6 V | 0.60 A | 0.78 W | +129 C | +47 C |

Feasible, with care. The SOT-89 tab is VOUT, so a generous 3V3 pour under/around it is the heatsink. LoRa TX is bursty (average well under 200 mA), so average dissipation is a fraction of the table; the 0.54-0.78 W lines are worst-case continuous. LR1108 Tj max is 125 C, thermal shutdown 165 C. Avoid long continuous TX on USB power in a hot enclosure.

Droop at 1S: at 600 mA the LR1108 should hold 3.3 V while Vbat >= about 3.5 V (3.3 V + ~0.3 V max dropout). Below that the rail follows the battery minus the dropout, so at end of discharge under TX the 3V3 rail can sag to roughly 2.7-3.0 V. The ESP32-S3 brownout threshold, SX1262/Ra-01SH-P minimum supply and NAND minimum must tolerate that. Any LDO has this limit; only a buck-boost avoids it.

### 7.4 Recommendations

1. **Drop-in: UTC LR1108G-33-AB3-A-R**, if it can be sourced (ask UTC or an authorised distributor; the suffix must be exactly AB3-A). Bench checks before flight: scope 3V3 on a 0-600 mA load step with the real 2 x 10 uF; measure dropout at 600 mA (Vin about 3.5 V); measure temperature rise at 4.2 V in with TX on.
2. **Bodge if the LR1108 cannot be sourced: AP7361C-33Y5-13** (SOT-89-5, DigiKey AP7361C-33Y5-13DICT-ND, about $0.40 @10, 1 A, 360 mV typ @ 1 A, Vin 6 V, EN pin) on a small adapter (SOT-89-5 to three wires, or a cut piece of perfboard). Remove U2. Wire adapter OUT (pin 5) to the 3V3 pad (pad 2), IN (pin 4) to the +BATT pad (pad 3), GND (pin 2) to the GND pad (pad 1). Tie EN (pin 1) to IN, leave pin 3 NC. The SOT-89-5 tab is GND but the board tab pad is VOUT, so never solder the package tab onto pad 2; mount the adapter beside the footprint with flying leads and keep the tab insulated. The board copper is then not a heatsink, so use only at moderate TX duty. A TLV757P 3.3 V part (SOT-23-5, 1 A, no thermal tab) on the same style of adapter is the smaller equivalent.
   Poor bodge to avoid: AP7365-33YG-13 in place (pin 3 = VIN matches, but pins 1 and 2 are crossed and its GND tab would short the 3V3 pour to ground).
3. **Next board revision:** use a buck-boost so 3.3 V holds through 3.0 V cells: **TI TPS63021** (fixed 3.3 V, VSON-14 3 x 4 mm, Vin 1.8-5.5 V, 2 A at VIN > 2.5 V, 25 uA Iq, -40..+85 C). TPS63001 (3.3 V fixed, VSON-10 3 x 3 mm, 1.2 A buck mode at VIN 3.6-5.5 V, 800 mA boost mode, Iq < 50 uA) is smaller but marginal for LoRa TX plus ESP32 at 3.0 V cell voltage. A buck alone cannot hold 3.3 V below about 3.4 V input. If a linear regulator is still preferred for RF noise, use TLV757P 3.3 V (1 A, 450 mV max @ 1 A) or a leadless ~100-200 mV part (onsemi NCP186, 100 mV typ @ 1 A, XDFN8; ST LD39100PU33R, LCSC C155574); all have Vin max about 5.5 V, adequate for 1S and USB through a Schottky.

### 7.5 Sources
- UTC LR1108 datasheet QW-R102-031.I: https://datasheetq.com/pdf-view/LR1108-UTC.pdf (pin codes, VD 300/500 mV @ 1 A, Vin 2.5-7.0 V, thetaJA 165 C/W SOT-89)
- UTC LR1107/E datasheet QW-R102-033.L (LCSC C171497): https://atta.szlcsc.com/upload/public/pdf/source/20180111/C171497_15156386857951255601.pdf
- Diodes AP7365 DS32260: https://www.diodes.com/assets/Datasheets/AP7365.pdf ; AP7361C: https://www.diodes.com/assets/Datasheets/AP7361C.pdf ; AP2114 (BCD): https://www.diodes.com/assets/Datasheets/AP2114.pdf
- Micro One ME6217 V02 (C81592), ME6231 V03 (C2765183), ME6216 (C126467), ME6214 (C427600) via LCSC.
- TI TLV757P: https://www.ti.com/lit/ds/symlink/tlv757p.pdf ; TPS63001: https://www.ti.com/lit/ds/symlink/tps63001.pdf ; TPS63020/21: https://www.ti.com/lit/ds/symlink/tps63020.pdf ; onsemi NCP186: https://www.onsemi.com/pdf/datasheet/ncp186-d.pdf
- DigiKey search pages for AP7365-33YG-13, AP7361C-33Y5-13 and LR1108 (no results), 2026-11-13. Mouser and LCSC parametric/price pages were not reachable.

## 8. Wider search, 2026-11-13 (3 agents: SOT-89 drop-in, small part over the pads, lower current)

Load estimate (3V3 rail, datasheet figures, partly unverified): flight with Wi-Fi off about 100 mA peak at LoRa -17 dBm, about 220 mA at +22 dBm; Wi-Fi on 400-450 mA peak (Espressif asks for >= 500 mA supply). 500 mA is the practical minimum, 300 mA only for Wi-Fi off and low TX power. Check whether the Ra-01SH-P PA is on this rail: it can draw 750 mA at full power.

| Rank | Part | Package / mount | I max | Dropout | Notes |
|---|---|---|---|---|---|
| 1 | Diodes AP2112K-3.3TRG1 (LCSC C51118) | SOT-23-5 (1 VIN, 2 GND, 3 EN, 4 NC, 5 VOUT; DS39724 p.2). Wire pin 1 -> pad 3, pin 2 -> pad 1, pin 5 -> pad 2, EN to VIN | 600 mA | 250 typ / 400 max mV @ 600 mA | Meets the 1S spec on max dropout. Loses the tab heatsink: 0.54 W at 4.2 V / 600 mA is too hot continuous, fine at flight load. |
| 2 | 3PEAK TPL740F33-89TR | SOT-89-3, 1 GND / 2 OUT (tab) / 3 IN (TPL740 datasheet Rev A.4 p.6) - **exact drop-in** | 500 mA | 400 mV @ 500 mA | Vin 2.4-5.5 V, ceramic stable. DigiKey stock ~2-3k, not on LCSC. Holds 3.3 V only to ~3.7 V battery at 500 mA. |
| 3 | TI TLV75733PDBVR (LCSC C485517) | SOT-23-5, same wiring as AP2112K | 1 A | 300 typ / 425 max mV @ 1 A (lower at 600 mA) | Best current margin of the small parts. |
| 4 | TI TLV75533PDBVR (LCSC C404027) | SOT-23-5, same wiring | 500 mA | 150 typ / 238 max mV @ 500 mA | Lowest dropout, 500 mA ceiling. |
| 5 | Richtek RT9080-33GJ5 (LCSC C841192) | TSOT-23-5 | 600 mA | 310 typ / 530 max mV @ 600 mA | Iq 2 uA; fails the 400 mV limit at max. |

Traps confirmed in this pass (GND/VIN-tab/VOUT or other wrong order): ME6231A33PG, AP1115AY33G-13, RT9161A-33GX, XC6206 SOT-89 copies; TPL720F33-89TR reported GND/IN/OUT (unconfirmed). WL9005P3-33 pin order unreadable, treat as reject. Stock and prices are snapshots, recheck before ordering. Agent tables: scratchpad reg_search_sot89.md / reg_search_small.md / reg_search_lowcurrent.md.
