# LoRa Doppler evaluation (Witness <-> Aperture, 926 MHz)

Analysis and plan only. No firmware, Horizon or HITL code was changed and no bench hardware was touched.
Calculation scripts: `C:\Users\E\AppData\Local\Temp\claude\a1adc47d057f1999\57c05c66-37ca-4475-bfbd-0c3c8c766392\scratchpad\doppler\dop.py` and `fine.py` (run with `python -P`).

## 1. Verdict in one paragraph

At the current SF7 / 125 kHz / CR4/5 configuration Doppler is **not a concern**. Peak shift is -4.1 kHz (4.4 ppm), about 3% of BW, and peak Doppler rate is about 2.2 kHz/s, which moves the signal by at most about 0.3 FFT bin over the longest (65 B) packet. The larger frequency uncertainty on this link is the two radios' own reference-clock error (up to roughly +/-37 kHz worst case if both are +/-20 ppm crystals), not Doppler. Doppler does become relevant as a **rate** problem if we move to SF10/125 (or SF8+/125 with the 65 B state packet): the drift within one packet then exceeds one FFT bin during boost. It is worth confirming empirically, but only if we plan to change SF/BW. A cheap static-offset test is worth doing regardless, because it measures the real crystal margin.

## 2. Physics, from the actual eayf trajectory

Method: `eayftrajectory.csv` (ft converted to m), ground station at the pad origin, slant range R = |(E,N,U)|, radial velocity v_r = r_hat . v, Doppler f_d = -f0 v_r / c (receding = negative), f0 = 926 MHz. Rate = finite difference of f_d over 0.05 to 1 s windows.

| Quantity | Value |
|---|---|
| Peak speed | 1327 m/s at t = 15.8 s, alt 6.2 km |
| Peak radial velocity | 1326 m/s (flight is nearly vertical over the pad, so v_r is about speed) |
| Peak Doppler | -4096 Hz = -4.42 ppm (3.3% of 125 kHz BW) at t = 15.8 s |
| Time with \|f_d\| above 3.9 kHz | 15.3 to 16.3 s only |
| Descent / apogee side | +1.06 kHz max (t about 137 s, falling) |
| Peak Doppler rate | 2.2 kHz/s (0.05 s window), 2.0 kHz/s (0.5 s), 1.76 kHz/s (1 s window); t about 14.5 s; about 72 g axial |
| Rate above 1 kHz/s | roughly 13.6 to 15.2 s (about 1.5 s) |
| Coast rates | 30 to 60 Hz/s from t = 20 s to apogee; below 25 Hz/s on descent |
| Max slant range in this CSV | 36 km at t = 89 s (apogee 35.9 km). The 130 km case is a different or hypothetical flight; Doppler is bounded by speed anyway, so 1.33 km/s gives at most 4.1 kHz for any geometry |

Other timing effects, all negligible: Doppler also stretches symbol time by 4.4 ppm, which over a 123 ms packet is about 0.5 us versus an 8 us chip at 125 kHz. Oscillator g-sensitivity (typically around 1 ppb/g, a datasheet-typical figure that needs checking for the actual part) at 72 g is about 70 ppb, about 65 Hz, well below one bin.

Rate per symbol at SF7/125 (Ts = 1.024 ms): 2.0 Hz, 0.0016% of BW.

## 3. Tolerances and how they stack

Primary sources checked in this session were limited; items marked (verify) must be confirmed against the SX1261/2 datasheet before relying on them.

- Semtech AN1200.22 (LoRa Modulation Basics), section 4.1.5 "Doppler Resistant": LoRa's frequency-offset tolerance relaxes reference-clock requirements and suits mobile links. It gives no numeric drift limit in the text I could extract.
- Semtech AN1200.14 (crystal guidance): LoRa is described as largely insensitive to initial Tx/Rx frequency offset; its PER-vs-offset plot for 125 kHz BW spans about +/-40 kHz (second-hand via search summary, verify). The commonly quoted rule of thumb is a receiver tolerance of about +/-25% of BW (about +/-31 kHz at 125 kHz) (verify in the datasheet).
- Low Data Rate Optimize is required when symbol time exceeds 16.38 ms (SF11/125, SF12/125). Its purpose is robustness to reference drift during long symbols. Not relevant to SF7 (1.02 ms) or SF10/125 (8.2 ms).
- I could not find an official numeric "maximum drift rate within a packet" for the SX126x. An unreferenced paper claims a CFO tolerance of 0.25 BW / 2^SF, which I could not confirm and would not rely on. So this analysis uses a **conservative engineering criterion**: the signal should not slide more than half an FFT bin (BW / 2^SF / 2) over the whole packet after the preamble-based offset estimate. That gives a limiting Doppler rate = 0.5 (BW / 2^SF) / ToA. The real receiver may tolerate more; that is exactly what the bench test measures.

Static stacking: Doppler (up to 4.1 kHz, 4.4 ppm) adds to the Tx crystal error plus the Rx crystal error. At 926 MHz, 10 ppm = 9.3 kHz, 20 ppm = 18.5 kHz. Worst case two +/-20 ppm parts give +/-37 kHz, which is about 30% of 125 kHz BW, so the crystal budget alone could approach the receiver tolerance, while Doppler adds only another 11%. With +/-10 ppm crystals: +/-18.5 kHz + 4.1 kHz = 18% of BW, comfortably inside. Temperature change on the rocket moves the Witness crystal by several ppm (AT-cut parts are typically within a few ppm over a modest range; actual curve unknown).

Hardware facts from the code (Witness `ra01.cpp`, Aperture `ra01.cpp`):
- SetModulationParams = {SF7, BW 0x04 (125 kHz), CR 1 (4/5), LDRO = 0x00}. LDRO is off, correct for SF7.
- No `SetDIO3AsTCXOCtrl` and no calibrate or regulator commands are issued, so the Ra-01SH-P is running on its own reference as wired (module crystal vs TCXO is not stated in the code; check the Ai-Thinker datasheet. If it is a plain crystal the ppm class above applies).
- Frequency word = freq << 25 / 32 MHz (so 32 MHz reference, step 0.95 Hz). `SetRfFrequency` is only issued at init; there is no runtime frequency control.
- Aperture reads `GetPacketStatus` (0x14) but keeps only byte 0 (RssiPkt, dBm = -value/2). SnrPkt (byte 1) and SignalRssiPkt (byte 2) are read and discarded. Witness `receive()` reads no packet status at all.
- Frequency error (FEI): I found no documented LoRa frequency-error command or register in the SX126x datasheet (verify; the SX128x has one). Treat FEI as unavailable until proven, and use SNR plus the static-offset scan plus the HackRF as the measurement paths (below).

## 4. Verdict by configuration

Packet airtime from the standard Semtech formula (preamble 8, explicit header, CRC on, CR4/5). "Limit rate" = 0.5 bin criterion above. Worst measured rate 2.2 kHz/s during t = 13.6 to 15.2 s; typical coast 30 to 60 Hz/s.

| Config | Bin (Hz) | ToA 25 B / 65 B | Drift in 65 B pkt at 2.2 kHz/s | Limit rate (65 B) | Static \|f_d\| / BW | Verdict |
|---|---|---|---|---|---|---|
| SF7 / 125 (current) | 977 | 62 / 123 ms | 243 Hz (0.25 bin) | about 4.0 kHz/s | 3.3% | OK, margin about 1.8x on rate, large on offset |
| SF8 / 125 | 488 | 113 / 216 ms | 426 Hz (0.9 bin) | about 1.1 kHz/s | 3.3% | Marginal during the boost burst (about 1.5 s), fine in coast |
| SF10 / 125 | 122 | 412 / 739 ms | 1460 Hz (12 bins) | about 80 Hz/s | 3.3% | Likely to fail packets during boost; coast (30 to 60 Hz/s) is borderline |
| SF11 / 125 (LDRO on) | 61 | 823 / 1561 ms | 3080 Hz (50 bins) | about 20 Hz/s | 3.3% | Fails in boost and likely most of the climb unless drift is tracked |
| SF7 / 250 | 1953 | 31 / 62 ms | 122 Hz | about 16 kHz/s | 1.6% | Fine |
| SF7 / 500 | 3906 | 15 / 31 ms | 61 Hz | about 63 kHz/s | 0.8% | Fine; Doppler is only 1% of BW |
| SF9 / 500 | 977 | 52 / 98 ms | 193 Hz | about 5 kHz/s | 0.8% | Fine |

Important qualifier: at long range the link budget, not Doppler, drives the SF choice. Higher SF raises sensitivity, which is why SF10 is attractive, and it is also exactly where drift tolerance shrinks. The boost phase (t < 20 s, range under 11 km) is a short-range, high-SNR part of the flight, so a packet lost in that window at SF10 may not matter for link margin but would still show as data loss in the flight record. The pre-compensation mitigation below removes the issue.

## 5. Test plan with current hardware (cheapest first)

Common rules: no live RF into antennas beyond the existing bench setup; Witness and Aperture radios stay at the -17 dBm default; do not run these concurrently with GPS RF replay on the same HackRF (HackRF is half-duplex, one session at a time).

### Step 0 (no firmware change, about 0.5 day): measure real crystal offsets with the HackRF

On the Pi, record each radio's transmissions with the HackRF at 926 MHz (2 Msps is enough for a 125 kHz signal) and estimate the carrier offset from the preamble (up-chirp stack / dechirp with numpy). This gives each radio's absolute frequency error (limited by the HackRF's own clock accuracy, which must be checked for the HackRF Pro). It tells us whether the combined static offset is a few kHz or tens of kHz before any Doppler is considered. Both radios must TX alone for this (use `set_front_end_enabled`, an existing hook, to silence the other FEM where needed).

### Step A (about 1 day firmware + 0.5 day test): static frequency-offset scan on the existing link

Emulate a fixed offset by retuning one radio's `SetRfFrequency` in firmware. Needed hooks:
1. A runtime command (new IRIS command, same style as `IRIS_COMMAND_RADIO_POWER`) to set a signed frequency offset in Hz on each of Witness and Aperture, applied in `Ra01` by recomputing the frequency word. The offset must be applied while the radio is in standby (SetRfFrequency is not valid mid-RX).
2. Report SnrPkt (already read on Aperture, just discarded) and RssiPkt per packet in the Aperture status or ground-station-status telemetry; add a per-run counter of RX OK vs CRC errors (the `ESP_ERR_INVALID_CRC` path already exists in `receive()`).
3. Optionally add the same status read to Witness `receive()` so the uplink direction can be tested.

Procedure: step Aperture RX offset 0, +/-2, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40 kHz, about 60 packets (60 s at 1 Hz per stream) per point. Record packet success rate, RSSI and SNR. Expected: a flat success curve to about +/-25 to 30 kHz, then a cliff. Result is the usable tolerance in kHz, to be compared with the Step 0 offsets and the 4.1 kHz Doppler. Repeat with the offset applied on Witness TX to confirm symmetry, and at a degraded link level (raise attenuation or drop to the lowest drive) because offset tolerance shrinks near sensitivity: run once at strong signal and once about 6 dB above the PER knee.

### Step B (Doppler rate within a packet, about 1.5 to 2 days): HackRF record-and-replay with a frequency ramp

The SX1262 cannot change frequency mid-packet, so a real in-packet ramp has to come from the HackRF. Options assessed:
- **Live re-TX channel emulator (RX Witness, TX with ramp): rejected.** Needs full-duplex streaming with latency control; the HackRF is half-duplex.
- **Generate LoRa from scratch in Python/gr-lora_sdr with a chirp offset: possible but larger.** Cost is writing or integrating a LoRa modulator and getting framing exactly compatible with the SX1262 (sync word, header, CRC).
- **Recommended: record real packets, ramp them offline, replay.** Capture a few hundred real Witness 0x01, 0x02, 0x03 packets with the HackRF (2 Msps), cut each out, multiply by exp(j 2 pi (f_off t + 0.5 r t^2)) in numpy for a chosen rate r, and replay with `hackrf_transfer -t` or the existing `gps_rf.py` plumbing into Aperture's RF input. This uses the real modulator, so there is no framing risk. Rate sweep: r = 0, 0.5, 1, 2, 4, 8, 16, 32 kHz/s (current worst case is 2.2 kHz/s; the sweep brackets it and also finds the SF7 cliff), each at several start offsets.
- RF path: a closed coax path HackRF TX -> fixed attenuation (we have about 60 dB) -> Aperture RF connector, with Witness's radio FEM off (`set_front_end_enabled(false)`) and Aperture's antenna replaced by the cable. HackRF TX gain can be set far lower than 0 dBm (its TX level is adjustable by VGA gain), so the attenuation budget should be sufficient; the received level should be set to a few dB above the PER knee found in Step A. Confirm the Ra-01SH-P connector type (u.FL vs SMA) and whether a conducted connection through the FEM LNA is safe (keep input below the LNA's rating; at -17 dBm drive class levels plus 60 dB attenuation it is far below).
- Success metric: packet success rate per (rate, offset) cell, plus Aperture SNR. Repeat for SF10/125 and SF7/500 by flashing the alternate configuration on both radios (recorded IQ is per configuration, so record again).
- Alternative worth a short spike: if the module uses a plain crystal (not a TCXO), the SX1262 crystal load trim registers (XTA/XTB) shift the reference by ppm per LSB and can be written over SPI during TX, which could give a crude stepped ramp with no HackRF. This is undocumented territory (verify) and steps may be coarse; treat as optional.

### Step C (about 1 to 1.5 days): live Doppler in the HITL run

During `hitl_session.py`, compute f_d(t) from the same trajectory the sim already follows (the radial velocity above) and push it to the radios once the Step A command exists. Design:
- Apply the offset on the **transmit side at TX start** (Witness before each of its 1 Hz packets; Aperture before each command with the opposite sign), where the radio is already in standby inside `start_transmit`. This avoids interrupting RX and never retunes mid-packet.
- Update rate: the offset must be right at the packet instant; refreshing at 5 to 10 Hz from the HITL host bounds the staleness error to 2.2 kHz/s * 0.1 s = 220 Hz, below one SF7 bin (977 Hz). So 5 Hz is sufficient; 10 Hz is comfortable.
- Latency/jitter: command is host -> USB -> firmware; for the uplink direction it adds one hop. Keep it as a state variable in firmware so a late update only delays the offset rather than corrupting a packet.
- It tests the full chain (real timing, real receive path) for the Doppler magnitude, but not the in-packet rate (Step B does that). It does add the real crystal offset on top, which is the combined case we actually care about.
- Limitation: with the crystal offset unknown, the emulated total is only meaningful with Step 0/A numbers beside it.

## 6. Mitigations if the tests say it matters

1. **Witness pre-compensation (best):** Witness knows its velocity (Kalman state) and the pad location, so it can shift its TX frequency by -f_d each packet (apply at TX start, so no mid-RX retune). This removes both shift and, to first order, the rate effect (rate error is then only the residual 2% velocity-estimate error times 2.2 kHz/s). It needs the pad position stored and uses the GPS-quality estimate already present; a 10% velocity error leaves 0.4 kHz.
2. **Ground-side tracking:** use per-packet frequency error (if a usable FEI can be found for the SX1262) or SNR-vs-offset to steer Aperture's RX centre between packets. Honest caveat: no documented LoRa FEI on the SX126x, so this may not be feasible without an alternative estimator.
3. **Wider BW / lower SF:** SF7/250 or SF7/500 removes the problem outright (Doppler is 0.8 to 1.6% of BW) at the cost of sensitivity (3 dB per BW doubling). SF9/500 keeps SF9 gain at 125 kHz rate parity with SF7/125... check the link budget first.
4. **Shorter packets at high SF:** split the 65 B state packet; drift is proportional to airtime.
5. **TCXO variant:** removes crystal uncertainty (not Doppler) if Step 0 shows large offsets.

## 7. Effort summary

| Item | Effort |
|---|---|
| Step 0: HackRF measurement of each radio's frequency error | 0.5 day |
| Step A: firmware hooks (offset command, SNR/CRC reporting) + scan | 1.5 days total |
| Step B: record, ramp, replay and sweep | 1.5 to 2 days, plus SF10 re-run +0.5 day |
| Step C: HITL integration | 1 to 1.5 days |
| Pre-compensation in Witness firmware | 1 day plus flight-day validation |

Recommendation: do Step 0 and Step A now (cheap, answers the crystal/tolerance question). Do Step B only if we decide to fly SF8+/125 for the long-range link; Step C only as a regression test once Step A hooks exist.

## 8. Open questions

1. Ra-01SH-P reference: crystal or TCXO, and its ppm class (Ai-Thinker datasheet; not determinable from the code, which never calls SetDIO3AsTCXOCtrl).
2. Does the SX1262 expose any LoRa frequency-error estimate? Not found in the datasheet; check RadioLib or Semtech's reference driver. If not, Step A relies on SNR and success cliffs.
3. Official numeric limits: the SX126x datasheet frequency-offset tolerance (about 25% of BW is assumed) and any intra-packet drift limit. The 0.5-bin criterion here is my own conservative assumption.
4. HackRF Pro reference clock accuracy (limits Step 0 absolute accuracy) and whether its TX gain range gives enough dynamic range through 60 dB of attenuation for the Aperture input.
5. Whether the SX1262 default image calibration covers 926 MHz (believed to be the 902 to 928 MHz default; verify). Retuning by tens of kHz is within the same image band, but check if SetRfFrequency changes in firmware should re-run CalibrateImage.
6. Which SF/BW is actually planned for the 130 km link: this decides whether Step B is needed at all.
7. The eayf CSV only reaches 36 km range. The 130 km slant-range trajectory (or a horizontal-flight case) is needed to check Doppler on other geometries, although the speed bound of 4.1 kHz still holds.
