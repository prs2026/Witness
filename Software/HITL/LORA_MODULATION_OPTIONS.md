# LoRa modulation options for the Witness rocket telemetry link

Analysis only. No code, firmware or hardware was changed and nothing was committed.
Calculation scripts (run with `python -P`):
- `C:\Users\E\AppData\Local\Temp\claude\a1adc47d057f1999\57c05c66-37ca-4475-bfbd-0c3c8c766392\scratchpad\modulation\toa.py` (airtime, duty, uplink budget)
- `C:\Users\E\AppData\Local\Temp\claude\a1adc47d057f1999\57c05c66-37ca-4475-bfbd-0c3c8c766392\scratchpad\modulation\link.py` (sensitivity, link margin, crystal and Doppler checks)

Related: `LORA_DOPPLER_EVALUATION.md` (Doppler peak 4.1 kHz, peak rate 2.2 kHz/s during boost).

## 1. Recommendation

| Pick | Setting | DL duty | Uplink free | Margin at 130 km (Yagi 13 dBi / duck 2 dBi) | Crystal+Doppler vs tolerance | Doppler rate limit | Full power, no hopping |
|---|---|---|---|---|---|---|---|
| **Primary** | SF7 / 500 kHz / CR 4/5 | 5.9 % | 94.1 % | 26.5 dB / 15.5 dB | 41 kHz vs 125 kHz (33 %) | 63 kHz/s (vs 2.2) | Yes |
| Primary, FEC variant | SF7 / 500 kHz / CR 4/8 | 8.5 % | 91.5 % | 26.5 dB / 15.5 dB | 41 kHz vs 125 kHz | 42 kHz/s | Yes |
| **Fallback (weak link / rubber duck)** | SF9 / 500 kHz / CR 4/5 | 19.5 % | 80.5 % | 31.5 dB / 20.5 dB | 41 kHz vs 125 kHz | 5.0 kHz/s (vs 2.2) | Yes |
| Middle fallback | SF8 / 500 kHz / CR 4/5 | 10.8 % | 89.2 % | 29.0 dB / 18.0 dB | 41 kHz vs 125 kHz | 18 kHz/s | Yes |

Why SF7/500 CR 4/5:
- Downlink uses 5.9 % of airtime against a 66.7 % cap, which is about 11x the required capacity.
- Leaves 94 % of airtime for uplink and acks.
- Range is not the constraint: even the 2 dBi duck has 15.5 dB margin at 130 km, and the 10 dB fade allowance is covered with the Yagi out to about 860 km (the radio horizon at rocket altitude is lower; see section 6).
- The 500 kHz bandwidth gives crystal tolerance about 3x the worst-case offset and the largest Doppler-rate margin.
- A 500 kHz occupied bandwidth is the only option here that can use full power without frequency hopping under the FCC summary in the request (verify, see section 7).

Why the current SF7/125 is not the answer, despite passing the 50 % rule (23.6 % duty):
- Crystal tolerance fails: worst case 41.1 kHz against a 31.25 kHz tolerance (0.25 x 125 kHz). It only passes if both crystals are within about +/-14.7 ppm, not the +/-20 ppm worst case.
- Regulatory: 125 kHz needs hopping for +29 dBm.

## 2. Formulas

Symbol time and preamble (SX126x, Semtech AN1200.13):
- `Ts = 2^SF / BW`
- `Tpre = (N_pre + 4.25) * Ts`, with N_pre = 8

Payload symbols (explicit header, IH = 0; CRC on, CRC = 1; DE = LDRO flag; CR = 1..4 for 4/5..4/8):
- `nPay = 8 + max( ceil( (8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE)) ) * (CR + 4), 0 )`
- `ToA = Tpre + nPay * Ts`
- LDRO (DE = 1) when `Ts > 16 ms`. For the candidate set this is never on (only SF11 and SF12 at 125 kHz reach it).

Bit rate: `Rb = SF * (4 / (4 + CR)) / Ts`

Airtime duty (1 Hz, three downlink packets): `duty = ToA(25) + ToA(65) + ToA(19)`, in seconds per second.
- Headroom rule (50 %): `duty <= 2/3`, i.e. capacity >= 1.5x the current load.
- Uplink free time: `free = 1 - duty`.
- One command plus ack exchange: `X = ToA(10) + ToA(13) + G`, with G = 20 ms assumed turnaround and guard.
- Ideal command rate: `cmd/s = free / X`. Ideal ephemeris time for 712 commands: `712 / cmd/s`. This is a lower bound (no retries, no status traffic).

Link budget:
- `FSPL(d, f) = 20 log10(d_m) + 20 log10(f) + 20 log10(4*pi/c)`. At 130 km, 926 MHz: **134.06 dB** (matches 134.1).
- `RSL = P_tx + G_rocket + G_ground - FSPL`, with P_tx = 29 dBm, G_rocket = 0 dBi. RSL is -92.06 dBm with the Yagi and -103.06 dBm with the duck.
- `S = -174 + 10 log10(BW) + NF + SNR_req(SF)`, with NF = 6 dB and SNR_req = -7.5, -10, -12.5, -15, -17.5, -20 dB for SF7..SF12. No LNA credit.
- `margin = RSL - S`. Pass if margin >= 10 dB (this covers the 10 dB rocket fade allowance).
- Max range with 10 dB fade: `d_max = 10^((P_tx + G_r - (S + 10) - 20 log10 f - 20 log10(4*pi/c)) / 20)`.
- Off-boresight loss (Yagi): `L(theta) = 12 (theta / HPBW)^2`, HPBW = 35 deg assumed.

Frequency tolerance:
- Crystal: `dF_xtal = 2 * ppm * 1e-6 * f`. At +/-20 ppm each: 37.0 kHz. Plus Doppler peak 4.1 kHz: **41.1 kHz**.
- Criterion (from the request): `dF_total <= 0.25 * BW`. Break-even per-radio ppm: `(0.25*BW - 4.1 kHz) / (2 * f * 1e-6)`. That is 14.7 ppm at 125 kHz and 31.5 ppm at 250 kHz.

Doppler rate (conservative criterion from `LORA_DOPPLER_EVALUATION.md`, my own assumption):
- Drift during a packet must stay under half an FFT bin: `r_limit = 0.5 * (BW / 2^SF) / ToA(65)`.
- Peak rate 2.2 kHz/s must be below `r_limit`.

## 3. Airtime, duty and uplink (CR 4/5 and 4/8)

Times in ms, Rb in bit/s, duty in %. "50 % rule" is PASS when duty <= 66.7 %. "ToA10/13/76" are the uplink command, the ack and the transfer status.

| Config | CR | Ts ms | Rb bps | ToA25 | ToA65 | ToA19 | DL duty % | 50 % rule | Uplink free % | ToA10 | ToA13 | ToA76 | cmd/s (ideal) | Ephemeris 712 s (ideal) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SF7/125 | 4/5 | 1.024 | 5469 | 61.7 | 123.1 | 51.5 | 23.63 | PASS | 76.4 | 41.2 | 46.3 | 138.5 | 7.1 | 100 |
| SF7/125 | 4/8 | 1.024 | 3418 | 86.3 | 184.6 | 69.9 | 34.07 | PASS | 65.9 | 53.5 | 61.7 | 209.2 | 4.9 | 146 |
| SF8/125 | 4/5 | 2.048 | 3125 | 113.2 | 215.6 | 102.9 | 43.16 | PASS | 56.8 | 72.2 | 82.4 | 246.3 | 3.3 | 219 |
| SF8/125 | 4/8 | 2.048 | 1953 | 156.2 | 320.0 | 139.8 | 61.59 | PASS | 38.4 | 90.6 | 107.0 | 369.2 | 1.8 | 403 |
| SF9/125 | 4/5 | 4.096 | 1758 | 205.8 | 390.1 | 185.3 | 78.13 | FAIL | 21.9 | 144.4 | 164.9 | 451.6 | 0.7 | 1072 |
| SF9/125 | 4/8 | 4.096 | 1099 | 279.6 | 574.5 | 246.8 | 110.08 | FAIL | 0 (over) | 181.2 | 214.0 | 672.8 | 0 | n/a |
| SF10/125 | 4/5 | 8.192 | 977 | 411.6 | 739.3 | 329.7 | 148.07 | FAIL | 0 (over) | 288.8 | 288.8 | 821.2 | 0 | n/a |
| SF10/125 | 4/8 | 8.192 | 610 | 559.1 | 1083.4 | 428.0 | 207.05 | FAIL | 0 (over) | 362.5 | 362.5 | 1214.5 | 0 | n/a |
| SF7/250 | 4/5 | 0.512 | 10938 | 30.8 | 61.6 | 25.7 | 11.81 | PASS | 88.2 | 20.6 | 23.2 | 69.2 | 13.8 | 51 |
| SF7/250 | 4/8 | 0.512 | 6836 | 43.1 | 92.3 | 34.9 | 17.04 | PASS | 83.0 | 26.8 | 30.8 | 104.6 | 10.7 | 67 |
| SF8/250 | 4/5 | 1.024 | 6250 | 56.6 | 107.8 | 51.5 | 21.58 | PASS | 78.4 | 36.1 | 41.2 | 123.1 | 8.1 | 88 |
| SF8/250 | 4/8 | 1.024 | 3906 | 78.1 | 160.0 | 69.9 | 30.80 | PASS | 69.2 | 45.3 | 53.5 | 184.6 | 5.8 | 122 |
| SF9/250 | 4/5 | 2.048 | 3516 | 102.9 | 195.1 | 92.7 | 39.07 | PASS | 60.9 | 72.2 | 82.4 | 225.8 | 3.5 | 204 |
| SF9/250 | 4/8 | 2.048 | 2197 | 139.8 | 287.2 | 123.4 | 55.04 | PASS | 45.0 | 90.6 | 107.0 | 336.4 | 2.1 | 345 |
| **SF7/500** | **4/5** | 0.256 | 21875 | 15.4 | 30.8 | 12.9 | **5.91** | PASS | **94.1** | 10.3 | 11.6 | 34.6 | 22.5 | 32 |
| **SF7/500** | **4/8** | 0.256 | 13672 | 21.6 | 46.1 | 17.5 | **8.52** | PASS | **91.5** | 13.4 | 15.4 | 52.3 | 18.7 | 38 |
| SF8/500 | 4/5 | 0.512 | 12500 | 28.3 | 53.9 | 25.7 | 10.79 | PASS | 89.2 | 18.0 | 20.6 | 61.6 | 15.2 | 47 |
| SF8/500 | 4/8 | 0.512 | 7812 | 39.0 | 80.0 | 34.9 | 15.40 | PASS | 84.6 | 22.7 | 26.8 | 92.3 | 12.2 | 58 |
| SF9/500 | 4/5 | 1.024 | 7031 | 51.5 | 97.5 | 46.3 | 19.53 | PASS | 80.5 | 36.1 | 41.2 | 112.9 | 8.3 | 86 |
| SF9/500 | 4/8 | 1.024 | 4395 | 69.9 | 143.6 | 61.7 | 27.52 | PASS | 72.5 | 45.3 | 53.5 | 168.2 | 6.1 | 117 |
| SF10/500 | 4/5 | 2.048 | 3906 | 102.9 | 184.8 | 82.4 | 37.02 | PASS | 63.0 | 72.2 | 72.2 | 205.3 | 3.8 | 186 |
| SF10/500 | 4/8 | 2.048 | 2441 | 139.8 | 270.8 | 107.0 | 51.76 | PASS | 48.2 | 90.6 | 90.6 | 303.6 | 2.4 | 297 |
| SF11/500 | 4/5 | 4.096 | 2148 | 185.3 | 328.7 | 164.9 | 67.89 | FAIL | 32.1 | 123.9 | 144.4 | 369.7 | 1.1 | 639 |
| SF11/500 | 4/8 | 4.096 | 1343 | 246.8 | 476.2 | 214.0 | 93.70 | FAIL | 6.3 | 148.5 | 181.2 | 541.7 | 0.2 | 3950 |
| SF12/500 | 4/5 | 8.192 | 1172 | 370.7 | 616.4 | 329.7 | 131.69 | FAIL | 0 (over) | 247.8 | 288.8 | 698.4 | 0 | n/a |
| SF12/500 | 4/8 | 8.192 | 732 | 493.6 | 886.8 | 428.0 | 180.84 | FAIL | 0 (over) | 297.0 | 362.5 | 1017.9 | 0 | n/a |

Note on "cmd/s" and "ephemeris": these are ideal numbers that assume the full free time is used by uplink. Real throughput will be lower because of retries, ack timeouts and telemetry phase.

Packetisation options (CR 4/5, duty %):

| Config | Current (25+65+19 B) | Sensors+state merged (90 B) + camera (19 B) | State at 0.5 Hz |
|---|---|---|---|
| SF7/125 | 23.63 | 21.04 | 17.47 |
| SF8/125 | 43.16 | 37.99 | 32.38 |
| SF9/125 | 78.13 | 69.84 | 58.62 |
| SF7/250 | 11.81 | 10.52 | 8.74 |
| SF7/500 | 5.91 | 5.26 | 4.37 |
| SF8/500 | 10.79 | 9.50 | 8.10 |
| SF9/500 | 19.53 | 17.46 | 14.66 |

The merged-packet figure assumes no framing saving (conservative). Merging only buys about 0.6 duty points at SF7/500, so it is not needed. It matters more at SF9/500 (about 2 points). Halving the state rate is a larger saving than merging, but reduces the state telemetry rate.

## 4. Link margin, crystal tolerance and Doppler rate

RSL at 130 km: Yagi -92.06 dBm, duck -103.06 dBm. Sensitivity from the formula above (NF 6 dB, no LNA credit). Margin is for CR 4/5. Sensitivity does not depend on CR in this model; CR 4/8 Doppler limits are given in the footnote.

| Config | Sens dBm | Margin Yagi dB | Margin duck dB | Pass >= 10 dB (Yagi) | Rmax, 10 dB fade, Yagi km | Rmax, 10 dB fade, duck km | Crystal+Doppler kHz | Tol 0.25 BW kHz | Xtal OK | Doppler rate limit kHz/s | Doppler OK (2.2 kHz/s) | Full power w/o hopping (BW >= 500 kHz) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SF7/125 | -124.5 | 32.5 | 21.5 | PASS | 1728 | 487 | 41.1 | 31.25 | **FAIL (132 %)** | 3.97 | PASS | no |
| SF8/125 | -127.0 | 35.0 | 24.0 | PASS | 2304 | 649 | 41.1 | 31.25 | FAIL (132 %) | 1.13 | **FAIL** | no |
| SF9/125 | -129.5 | 37.5 | 26.5 | PASS | 3073 | 866 | 41.1 | 31.25 | FAIL | 0.31 | FAIL | no |
| SF10/125 | -132.0 | 40.0 | 29.0 | PASS | 4098 | 1155 | 41.1 | 31.25 | FAIL | 0.08 | FAIL | no |
| SF7/250 | -121.5 | 29.5 | 18.5 | PASS | 1222 | 344 | 41.1 | 62.50 | PASS (66 %) | 15.86 | PASS | no |
| SF8/250 | -124.0 | 32.0 | 21.0 | PASS | 1629 | 459 | 41.1 | 62.50 | PASS (66 %) | 4.53 | PASS | no |
| SF9/250 | -126.5 | 34.5 | 23.5 | PASS | 2173 | 612 | 41.1 | 62.50 | PASS (66 %) | 1.25 | **FAIL** | no |
| **SF7/500** | **-118.5** | **26.5** | **15.5** | **PASS** | 864 | 244 | 41.1 | 125.00 | PASS (33 %) | 63.45 | PASS | **yes** |
| SF8/500 | -121.0 | 29.0 | 18.0 | PASS | 1152 | 325 | 41.1 | 125.00 | PASS (33 %) | 18.12 | PASS | yes |
| SF9/500 | -123.5 | 31.5 | 20.5 | PASS | 1536 | 433 | 41.1 | 125.00 | PASS (33 %) | 5.01 | PASS | yes |
| SF10/500 | -126.0 | 34.0 | 23.0 | PASS | 2049 | 577 | 41.1 | 125.00 | PASS | 1.32 | **FAIL** | yes |
| SF11/500 | -128.5 | 36.5 | 25.5 | PASS | 2732 | 770 | 41.1 | 125.00 | PASS | 0.37 | FAIL | yes |
| SF12/500 | -131.0 | 39.0 | 28.0 | PASS | 3643 | 1027 | 41.1 | 125.00 | PASS | 0.10 | FAIL | yes |

Footnote, CR 4/8 Doppler rate limits (kHz/s): SF7/125 2.65, SF8/125 0.76, SF7/250 10.58, SF8/250 3.05, SF9/250 0.85, SF7/500 42.33, SF8/500 12.21, SF9/500 3.40, SF10/500 0.90. All other CR 4/8 margins match CR 4/5.

Yagi pointing at SF7/500 (margin 26.5 dB on boresight): 5 deg off gives 26.2 dB, 10 deg gives 25.5 dB, and 17.5 deg (the 3 dB point) gives 23.5 dB. Pointing is not a link-budget problem at this margin.

Crystal break-even: at 125 kHz a pair of crystals must both be within +/-14.7 ppm for the Doppler-inclusive total to fit. Common plain crystals are +/-10 to +/-20 ppm, so this is a real risk for any 125 kHz option.

Doppler note: all options with a 500 kHz bandwidth at SF9 or lower pass with margin. SF10/500 fails the rate criterion (1.3 kHz/s against 2.2 kHz/s), even though its airtime is fine.

## 5. Options by candidate (summary verdict)

| Config | Verdict | Reason |
|---|---|---|
| SF7/125 | Reject unless crystals are <= 14.7 ppm, or a TCXO is fitted | Crystal FAIL (132 %). Full power needs hopping. Current setting. |
| SF8/125 | Reject | Doppler FAIL (1.1 kHz/s), crystal FAIL |
| SF9/125, SF10/125 | Reject | Downlink duty 78 % and 148 %. Does not fit. |
| SF7/250 | Viable only with hopping or reduced power | Crystal 66 %, Doppler OK, duty 11.8 %. Not full power. |
| SF8/250 | Viable only with hopping or reduced power | Doppler 4.5 kHz/s limit (2x margin), duty 21.6 % |
| SF9/250 | Reject | Doppler FAIL (1.25 kHz/s) |
| **SF7/500 (4/5 or 4/8)** | **Recommended** | Duty 5.9 % (8.5 % at 4/8), crystal 33 %, Doppler OK, full power, margin 26.5 dB Yagi / 15.5 dB duck |
| SF8/500 | Good alternate | Duty 10.8 %, margin 29 dB / 18 dB, Doppler 18 kHz/s limit |
| SF9/500 | Fallback for weak link | +5 dB sensitivity over SF7/500, duty 19.5 %, Doppler 5.0 kHz/s limit (2.3x) |
| SF10/500 | Reject | Doppler FAIL (1.3 kHz/s). Duty 37 %. |
| SF11/500 | Reject | Duty 67.9 % (just over the 66.7 % cap). Doppler FAIL. |
| SF12/500 | Reject | Duty 132 %. Doppler FAIL. |

## 6. Splitting packets, command latency and half-duplex

- Half-duplex: while Witness transmits it cannot hear. An uplink command from Aperture can only succeed when Witness is in receive, so the uplink window is `1 - duty`. At SF7/500 this is 94 % of the second, with the telemetry bursts totalling about 59 ms. Commands sent during a Witness burst are lost (there is no CAD in the design, as far as I know), so Aperture should time commands to the telemetry phase.
- Command latency: with 94 % free time, a command normally goes out within the same second. The worst wait is one telemetry period (1 s) if telemetry bursts are scheduled poorly. At SF7/125 this worsens to a 76 % window with 41 ms commands and 46 ms acks, still usable.
- Ephemeris: 712 commands at SF7/500 is about 32 s ideal, or about 1 to 2 minutes realistically once acks, retries and transfer status (76 B, 35 ms at SF7/500) are included. The same upload at SF8/250 is about 88 s ideal.
- Splitting: merging sensors and state saves about 0.6 duty points at SF7/500. Halving the state rate saves about 1.5 points. Neither is needed for the recommended setting, but both help SF9/500 (the fallback).
- CR 4/5 vs 4/8: 4/8 costs 2.6 extra duty points at SF7/500 and gives extra error correction. Airtime is still far under the cap, so 4/8 is a reasonable choice if interference is the concern. The sensitivity change is not modelled.

Horizon check (rough): the 4/3-earth radio horizon is about `d = 3.57 sqrt(h_m)` km. A rocket at 30 km altitude sees about 620 km, so 130 km is not horizon-limited. This is a rough figure and I did not run it in the script.

## 7. Regulatory (summary from the request, not checked against 47 CFR 15.247)

- 500 kHz and above: digital modulation (DTS) with 6 dB bandwidth >= 500 kHz can use up to +30 dBm without hopping. SF7/500 meets this. Treat the 6 dB BW as the nominal BW (an assumption).
- Below 500 kHz (125 and 250 kHz): hopping is needed for up to +30 dBm or +24 dBm, with 50 or 25 channels respectively. Without hopping, limits are much lower (15.249 field-strength). The current 125 kHz setting falls in this class.
- Possible directional-gain reduction for the Yagi: 15.247(b)(4) may require reducing conducted power by 1 dB per dB of antenna gain over 6 dBi for fixed point-to-point use. For a 13 dBi Yagi that would be 7 dB. Margin at SF7/500 would then be 19.5 dB, still above 10 dB. Whether the rule applies to this mobile-to-fixed link needs checking.
- Amateur radio (33 cm, 902 to 928 MHz) is an alternative if licensed. Part 97 prohibits encryption, which telemetry does not use.

## 8. Firmware and hardware implications (read-only check)

- Witness `hardware_pins.h` sets SF 7, BW 125000 Hz, CR 4/5 (code 1), preamble 8. `ra01.cpp` has a `static_assert` that forces BW to 125000, so moving to 500 kHz needs a change in both Witness and Aperture firmware and a rebuild. No hardware change is needed.
- Boot TX drive is -17 dBm (low-power PA, bench-safe). The SX1262 drive is capped at +3 dBm (`IRIS_RADIO_TX_POWER_MAX_DBM`). The +29 dBm figure assumes an external PA gain of about 26 dB at that drive. Confirm the flight configuration reaches that level through `IRIS_COMMAND_RADIO_POWER`.
- No LNA credit is included. If the front end adds gain with a noise figure of about 1.5 to 2 dB, the sensitivity improves by a few dB, which only helps the margins above.

## 9. Assumptions and uncertainties

1. Sensitivity uses the formula with NF = 6 dB and the SNR table. I did not check it against the SX1262 datasheet table in this session (no datasheet access), so expect a difference of about 1 to 2 dB at 125 kHz.
2. Crystal error +/-20 ppm per radio (worst case, plain crystal). Ra-01SH-P reference type should be confirmed.
3. Doppler rate criterion (half an FFT bin over a 65 B packet) is my conservative assumption. The Semtech tolerance figures (about +/-25 % of BW for offset) and any in-packet drift limit were not verified in this session.
4. FCC rules are summarised from the request and not checked against the CFR text.
5. Uplink numbers assume a 20 ms guard per exchange, one ack per command, and no retries. They are lower bounds.
6. Witness telemetry burst timing inside the second is not known. Clustered bursts give the 94 % window; spread bursts reduce the usable gaps.
7. 65 B state packet is the longest; the drift check uses it.
8. Margins assume 0 dB implementation loss. Each dB of cable or connector loss comes straight off the margin. The 10 dB fade allowance is applied on the rocket side only.
9. The 130 km requirement is taken as given; the eayf trajectory in `LORA_DOPPLER_EVALUATION.md` only reaches 36 km, so Doppler at 130 km geometry was not re-derived (the speed bound still holds).
