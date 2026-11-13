# Audit summary — significant issues (2026-11-12)

First pass by Haiku agents (`*_findings.md`), each finding verified by a
Sonnet agent (`*_evaluation.md`). Only CONFIRMED/PLAUSIBLE high and
critical items are listed; medium/low items are in the evaluation files.
Nothing has been changed in code.

## Critical — fix before any flight

| # | Area | Issue | Fix |
|---|------|-------|-----|
| C1 | Witness sensing | One bad baro sample permanently raises the FSM's max altitude and forces an irreversible Descent 500 ms later. Seen in HITL run 20261111-145157 (Descent at sim 66.78 s while climbing at Mach 1.4, max 43 038 m vs 27.9 km true, right after corrupted 0x0C frames). | Slew/median-filter the FSM altitude input; checksum HITL frames. |
| C2 | Witness comms | Radio commands 0x6868 mass storage / 0x6869 erase / 0x686A clear are accepted over LoRa in any flight state with no confirmation (`usb_serial_echo.cpp` radio switch). In flight: logging frozen + LoRa silenced, or flight logs wiped. Verified. | USB-only, or Pad Idle/Landed + confirmation key. |
| C3 | Witness comms+sensing | Flight state is not persisted: any in-flight reset (brownout, panic, WDT) restarts in Pad Idle. | Keep flight state in RTC_NOINIT and restore after crash resets (or infer from baro/GPS). |
| C4 | Iris | Any Iris reset switches every load/camera off and nothing restores them (LS_EN GPIO9 also resets the MCP23008, 10k pulldown). | Save commanded load mask in RTC memory, re-apply on warm boot without resetting the expander; Witness re-sends when Iris uptime goes backwards. |

## High

| # | Area | Issue | Fix |
|---|------|-------|-----|
| H1 | Witness sensing | Coast→Descent uses un-gated baro through transonic flight (Mach 0.8–4.2 from 23–83 s); ~5 % altitude error would trigger Descent. | Skip max/Descent test while the position filter gates baro; require v_up < 0. |
| H2 | Witness comms | Unauthenticated radio 0x73xx can force flight state (e.g. Pad Idle) in flight. | Gate on state / persisted flight flag. |
| H3 | Witness comms | Armed maintenance request survives a cancelled reboot → a later crash boots into maintenance (logging off). | Arm right before esp_restart(); accept maintenance only if reset reason is ESP_RST_SW. |
| H4 | Witness comms | Failed Wi-Fi start in maintenance leaves Witness there, logging off. | Reboot to flight mode on failure. |
| H5 | Witness comms | OTA self-test never exercises NAND writes. | Run the startup NAND test in the validation path. |
| H6 | Witness comms | (plausible) No bad-block handling on the NAND write path; one error stops all logging. | Skip/retire bad blocks, keep logging. |
| H7 | Aperture | Failed transmit returns after SetStandby without re-arming RX → deaf until reboot. | Single exit path that always clears IRQ and sets RX. |
| H8 | Aperture | (plausible) SX1262 reset (e.g. 3V3 sag) never detected → silently deaf. | Poll GetStatus ~1 Hz, re-init if not in RX. |
| H9 | HITL | No board-identity check before commands/stream; bench_ports scans only 7100-7102 and can't tell the two Witnesses apart; port goes stale after hub cycles. | Verify identity on the live socket; scan 7103; add a board ID; re-identify after every cycle. |

## Notes

- HITL "Descent UNEXPECTED at ~130 s" = FSM not reset on HITL exit (bench baro vs HITL max). Medium; fix so it can't mask real failures.
- HITL altitude metric bias: receipt-time stamping ≈ 10–25 ms latency; the reported 4.52 % corrects to ~2.1–2.7 % (bias was conservative).
- Horizon: no high/critical; 5 medium fixes (decoder resync, BUSY on upload BEGIN, unbounded no-progress loop, other devices' uptime on the Witness time axis, P-log converter stops at first bad record).
- Iris current reading clips at 1.00 A from a firmware /5 divisor (hardware allows ~5 A); medium unless a load exceeds 1 A.
