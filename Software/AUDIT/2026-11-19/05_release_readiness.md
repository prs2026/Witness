# Release-readiness audit: configuration and process (2026-11-19)

Scope: Witness flight image (ESP32-S3, ESP-IDF 6.1), `Software/Witness/softwarev1`, plus a look at Iris.
Method: read-only review of the committed tree (HEAD a06d14d) and the working tree, host tests, and a clean build of the committed tree in a scratch directory. No hardware, no flash scripts, no git state changes.
Owner decisions not re-raised: no radio command authentication, false-flight auto reset disconnected, fault injection 0x4246 allowed in the flight image (HITL-gated), Iris not flight-critical.

Status tags: CONFIRMED = read in the file/artifact and checked; PLAUSIBLE = inferred.

## Summary

No Critical finding. The flight sdkconfig content is sound (panic reboots, task WDT 8 s, brownout detector on, rollback on, 32 KB data cache, 240 MHz, -Os, INFO log level). The release risk is **process**: the flight image that sits in `images/` cannot be tied to a commit, the default `idf.py build` produces a different (untuned) configuration, and two stale flash scripts would corrupt the 8 MB layout. The radio will transmit at the -17 dBm bench default unless the power is commanded and stored.

| # | Sev | Finding |
|---|-----|---------|
| 1 | High | Shipped images are stamped `168373c-dirty`, not the HEAD commit; "dirty" is permanent (tracked log file modified) so the stamp proves nothing |
| 2 | High | `flashit.ps1` / `flashit.sh` flash the old 2 MB single-app layout onto an 8 MB two-image board |
| 3 | High | A flight board boots at the -17 dBm compiled default; nothing in the imaging flow sets >= -3 dBm |
| 4 | Medium | Plain `idf.py build` uses a stale, untracked root `sdkconfig` (160 MHz, debug optimisation, 16 KB icache) |
| 5 | Medium | `images/` and `build_*` are untracked; image = whatever was last built, no checksum/manifest, build not bit-reproducible by design |
| 6 | Medium | `build_images.ps1` incremental build can keep stale sdkconfig values; no release path enforces `-Clean` |
| 7 | Medium | Default Wi-Fi OTA password is a public constant in the repo; any Wi-Fi OTA upload is unsigned (maintenance image, Pad Idle/Landed only) |
| 8 | Medium | Stash `stash@{0}` holds uncommitted Witness firmware work (bench) that is not in HEAD |
| 9 | Low | BIT limits: heap limit already fixed in HEAD, but several `[PROPOSED]`/`[PLACEHOLDER]` values remain |
| 10 | Low | No core dump configured; panic reboots leave no post-mortem |
| 11 | Low | Bench USB commands (0x4252/0x4248/0x4249) are compiled into the flight image; gated by USB + HITL/resumed |
| 12 | Low | flash_verified.ps1 always rewrites otadata and both images; verification is by app name only |
| 13 | Info | Iris builds with debug optimisation and default config (not flight-critical) |

## Findings

### 1. High (CONFIRMED): the images in `images/` are not provably the committed tree

- `build/project_description.json` and `build_maint/project_description.json` both say `"project_version": "168373c-dirty"`. The same string is embedded in `images/flight.bin` (esp_app_desc_t version). HEAD is `a06d14d`; two commits (0d3a403 GPS trust ramp, a06d14d) were made after the build.
- `git describe --dirty` is dirty right now because the tracked file `Software/Horizon/logs/ui_stalls.log` is modified. Any build made from this working tree will be stamped "-dirty" regardless of firmware state, so the version stamp cannot be used as a release check.
- Evidence the flight image does match the committed firmware sources: `images/flight.bin` is byte-identical to `build/softwarev1.bin` (built 22:44); no file under `main/` is newer than it, and the firmware files changed by 0d3a403 (`gps_trust.h`, `position_filter.*`, `sensors.h`) predate it. This is timestamp reasoning, not proof (PLAUSIBLE).
- `images/maint.bin` was built at 22:08, before those files changed. The maintenance image does not link them (flight-only sources in `main/CMakeLists.txt`), so it is functionally current.
- Fix: before imaging, `git status` must be clean for tracked files (move or ignore Horizon logs: `Software/Horizon/logs/` is partly tracked, partly untracked), tag the release commit, rebuild both images with `.\build_images.ps1 -Clean`, and record `git describe` plus SHA-256 of `flight.bin` and `maint.bin` in the release notes. Consider setting `PROJECT_VER` from the tag so the stamp does not depend on dirtiness.

### 2. High (CONFIRMED): stale flash scripts target the wrong layout

- `Software/Witness/softwarev1/flashit.ps1:1` and `flashit.sh:1` run `write_flash --flash_size 2MB` with the app at `0x10000`. In the current table (`partitions.csv`) `0x10000` is `otadata` and the app slots are `0x20000/0x220000`. Running either script writes the application over otadata and the head of flight_a, and rewrites the header with a 2 MB flash size on an 8 MB part. The result is an unbootable or wrongly-sized board.
- `flash_verified.ps1` is the correct path (uses `images/flash_args`, offsets from the table). It falls back to `build/flash_args` (flight only, no maintenance image) with only a warning if `images/` is missing (`flash_verified.ps1:80-88`, warning at the `argsDir -eq 'build'` check).
- Fix: delete or neutralise `flashit.ps1`/`flashit.sh`/`openserial.sh` (or make them print "use flash_verified.ps1" and exit); make `flash_verified.ps1` fail instead of warn when `images/flash_args` is absent.

### 3. High (CONFIRMED): radio TX power boots at the -17 dBm bench default

- `main/hardware_pins.h:85`: `IRIS_RADIO_TX_POWER_DBM -17`, `IRIS_RADIO_TX_POWER_MIN_DBM -17`.
- `main/sensors.cpp:283-292`: the radio uses the NVS value (`radio_power_store`, key written only by command 0x5250 in Pad Idle/Landed, `command_policy.h:~60-90`) and otherwise the compiled -17 dBm.
- `main/bit_limits.h:22`/`bit_logic.h:546`: BIT fails the radio check (Blocking class) below `kRadioMinFlightPowerDbm = -3`. Owner rule: minimum configured flight power -3 dBm.
- Consequence: a freshly imaged board, or one whose NVS was erased, flies at -17 dBm unless someone sends 0x5250 and sees BIT GO. Allowed range is -17..+3 dBm (FEM input limit +3 dBm; do not exceed).
- Fix (choose one): (a) add a mandatory step to the imaging checklist (below) to set 0x5250 to the flight value (>= -3, <= +3 dBm) and verify it survives a reboot and BIT is GO; or (b) make the compiled default a flight value in a flight-only build, keeping -17 for bench (needs an owner decision: the bench-safe default protects the radio when TX is keyed on a bench antenna port).

### 4. Medium (CONFIRMED): plain `idf.py build` is not the flight configuration

- `Software/Witness/softwarev1/sdkconfig` (untracked, ignored by `.gitignore:33`, dated 2026-11-15) has `CONFIG_COMPILER_OPTIMIZATION_DEBUG=y` (line 1003), `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160` (1916), `CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB=y` (1922). A plain `idf.py build` reuses it. The flight overlay (`sdkconfig.flight`) is only applied by `build_images.ps1` into `build/sdkconfig`. The handoff document states "`idf.py build` still builds the flight image" (FIRMWARE_HANDOFF.md ~line 1540), which is true for the image kind but not for the tuned configuration: 16 KB icache + -O0-ish code was measured at 96-100 % core-1 load (BENCH_REPORT_2026-11-17c).
- BIT would show `kExpectedCpuMhz = 240` (`bit_limits.h`) as a mismatch on such an image (PLAUSIBLE: depends on the BIT clock check being wired).
- Fix: delete the root `sdkconfig`, or add `set(SDKCONFIG_DEFAULTS ...)` to the top `CMakeLists.txt` so the default build equals flight; fix the handoff sentence.

### 5. Medium (CONFIRMED): artifacts are untracked and not reproducible bit for bit

- `images/`, `build_cmd/`, `build_fsm/`, `build_iris/`, `build_maint/` are untracked; `build/` is ignored. `build_cmd`, `build_fsm`, `build_iris` have no `sdkconfig` (leftover or partial configure dirs, not release inputs). Delete them to avoid flashing from the wrong directory.
- `main/CMakeLists.txt` sets `WITNESS_BUILD_ID` from the build clock (`string(TIMESTAMP ... "%s")`) and `CONFIG_APP_COMPILE_TIME_DATE=y`, so two builds of the same commit differ (image_info build_id, compile date/time). This is by design (build_id orders OTA images) but means "reproduces the images" can only be checked by size/section content, not hash.
- Fix: keep a release manifest (commit, build_id, SHA-256, sizes). Optional: `CONFIG_APP_REPRODUCIBLE_BUILD` plus a SOURCE_DATE_EPOCH-derived build_id for release builds.

### 6. Medium (PLAUSIBLE): incremental builds can carry stale config

- `build_images.ps1:25` passes `SDKCONFIG=<dir>/sdkconfig` and `SDKCONFIG_DEFAULTS`. ESP-IDF applies defaults only to options absent from an existing sdkconfig, so editing `sdkconfig.flight` after the first configure does not change `build/sdkconfig`. Current `build/sdkconfig` was checked and does match the overlay (240 MHz, -Os, 32 KB I/D cache, rollback, panic reboot, WDT 8 s). `-Clean` exists (`build_images.ps1:23`) but is optional.
- Fix: release procedure requires `-Clean`; add a post-build assertion in `build_images.ps1` that greps `build/sdkconfig` for the expected lines (see checklist step 6).

### 7. Medium (CONFIRMED): public default Wi-Fi password, unsigned OTA

- `main/Kconfig.projbuild:34-37`: `WITNESS_WIFI_PASSWORD` default `witness-ota-2026`, in a public repo. Anyone within Wi-Fi range with this password can upload firmware while Wi-Fi is on (the Kconfig help says so). The flight image links no Wi-Fi code (`main/CMakeLists.txt`), Wi-Fi never starts at boot, and it starts only in the maintenance image in Pad Idle/Landed, so exposure is pre-launch and post-recovery. No secure boot / flash encryption (`CONFIG_SECURE_BOOT*` off), and OTA images are checked for kind/compatibility only (`image_info.h`), not signed.
- This is not the radio-command-authentication topic; it is the Wi-Fi AP credential.
- Fix: build release images with a non-default password supplied at build time (not committed), or require an on-board confirmation before the AP starts.

### 8. Medium (CONFIRMED): uncommitted firmware work exists in a stash

- `git stash list`: `stash@{0}: On main: bench work before BIT merge` touches 19 firmware files (command_bridge, command_policy.h, comms.h, flash_logger, gps_receiver, heartbeat, sensors, spi_data_forwarder, usb_serial_echo, plus Iris comms.h/usb_serial_echo and tests). Also three Horizon-log stashes (`hitl-run-feedback`, `gps-rf-hitl`, `viewer-responsiveness`, log data only) and the unmerged branch `gps-detect-hitl`.
- Not applied to the working tree (no modified tracked firmware files), so the current images do not contain it. The risk is the owner believing a bench fix is in the image when it is only in the stash.
- Fix: before imaging, diff `stash@{0}` against HEAD (`git stash show -p`) and either land or drop each hunk deliberately.

### 9. Low (CONFIRMED): BIT limit values

- The heap premise is outdated: `main/bit_limits.h:50` is `kHeapMinDegradedBytes = 8 * 1024 // [OWNER]` and `kHeapLargestDegradedBytes = 5 * 1024` (:51). With heap ~19 KB free / 15 KB min the heap check passes. `bit_logic_test` passes.
- Still marked `[PROPOSED]`: `kHeapMinBlockingBytes` 6 KB, stack margins 256/512 B, IMU rate +-6 %, gravity bands, NAND bad blocks 10, free-seconds thresholds (`kImuFreeSecondsBlocking 150`), `kGpsReadyWithinMs 45 s`, pad AGL/Vz limits, baro noise. `[PLACEHOLDER]` (not from datasheets): `kMcuTempMaxC`, IMU self-test windows (0.0), battery cut-offs `kBatteryBlockingBelowMv 3300` / `kBatteryDegradedBelowMv 3600` / `kBatteryDegradedAboveMv 4300` (the battery checks ARE wired, so these are live values with no datasheet source), rail/current limits (monitors absent).
- Fix: before flight, bench-confirm each wired `[PROPOSED]` limit once with the flight image; replace the battery placeholders with pack datasheet values.

### 10. Low (CONFIRMED): no core dump

- Built `sdkconfig`: `CONFIG_ESP_COREDUMP_ENABLE_TO_NONE=y` (line 2396). Panic is `PRINT_REBOOT` with 0 s delay (1986-1992), so a flight panic resets and resumes but leaves nothing for post-flight analysis except the console text if USB is attached. Optional: core dump to flash needs a coredump partition (free space exists at 0x720000..0x800000).

### 11. Low (CONFIRMED): bench commands compiled into flight

- `command_policy.h:104-119`, `usb_serial_echo.cpp:536-575`. Gated to USB source and HITL-active or resumed-after-reset; radio and USB-forward paths refuse them (`command_bridge.cpp`); HITL entry requires USB and Pad Idle (`sensors.cpp:385-410`). A hang command in a resumed state needs a USB host attached, which a real flight does not have. Accepted per owner decision on HITL gating; listed for completeness. Fault injection is gated by `allow_bit_fault(source, hitl_active)` (`bit_monitor.cpp:563`) and expires (`kFaultMaxDurationMs`).

### 12. Low (CONFIRMED): flash_verified.ps1 limits

- Identity check compares only the app name `softwarev1` in whichever of six candidate offsets answers (`flash_verified.ps1:35-60`); `-Force` bypasses it. It always writes bootloader, partition table, otadata, flight and maintenance images (`images/flash_args`), which resets OTA state to flight_a and drops any newer OTA-installed slot. It does not touch NVS (0x9000, calibration and radio power) or the NAND log chip. It does not read back or verify the flash after writing (esptool verifies per block by MD5 on write_flash by default, so the write itself is checked).
- The script's default port is a bench-network address; it must be given the right port for each board. Verify the target by running with `-CheckOnly` first (owner flow; not run in this audit).

### 13. Info: Iris

- `Software/Iris/softwarev1IRIS/sdkconfig`: debug optimisation, task WDT 5 s, panic reboot, INFO logging, 4 MB flash, no overlay. `comms.h` is byte-identical between Witness and Iris (same MD5). Iris has its own `flashit.ps1` for a different board. Not flight-critical per owner.

## Compile-time flag inventory (flight image)

| Item | Where | In flight image? | Runtime gate |
|------|-------|------------------|--------------|
| `CONFIG_WITNESS_IMAGE_FLIGHT` / `_MAINTENANCE` choice | Kconfig.projbuild, sdkconfig.flight/.maint | Flight selected; CMake checks it matches the `WITNESS_IMAGE` env var (`main/CMakeLists.txt`) | build-time FATAL_ERROR on mismatch |
| HITL (hitl_input.cpp, Sensors HITL path) | always compiled | Yes | USB source only, Pad Idle only, times out without input (`sensors.cpp:385`, `hitl_input.cpp`) |
| Bench restart/hang/Iris reset 0x4252/0x4248/0x4249 | command_policy.h:104 | Yes | USB + (HITL or resumed); refused for radio/forward |
| BIT fault injection 0x4246 | bit_monitor.cpp:563 | Yes (owner-approved) | `allow_bit_fault`: USB + HITL; 60 s expiry |
| Simulated sensors (HITL packets 0x0C) | sensors.cpp | Yes | HITL active only; `IRIS_KALMAN_FLAG_HITL` set in telemetry (sensors.cpp:2263) |
| Wi-Fi / HTTP / TinyUSB MSC | main/CMakeLists.txt | No (maintenance image only) | not linked in flight |
| `HEARTBEAT_BUZZER_ENABLED` | heartbeat.cpp:76 | per its define (not a bench flag) | n/a |
| Log level | sdkconfig `LOG_DEFAULT_LEVEL=3` (INFO), max = default | INFO, no debug/verbose compiled | n/a |
| Heap poisoning / tracing | `HEAP_POISONING_DISABLED`, `HEAP_TRACING_OFF` | off | n/a |
| Assertions | `ASSERTION_LEVEL=2` enabled | on (intended; assert failure reboots) | n/a |

Flight sdkconfig (generated `build/sdkconfig`, verified): panic `PRINT_REBOOT` (halt off), task WDT 8 s panic on, idle-task WDT off (intended), interrupt WDT 300 ms, bootloader WDT 9 s, brownout detector on (level 7, interrupt), app rollback on, flash 8 MB DIO 80 MHz, partition table custom, I-cache 32 KB, D-cache 32 KB, 240 MHz, -Os, PM and tickless idle off, console on USB Serial/JTAG, Bluetooth off.

Partitions (`partitions.csv`): nvs 0x9000/0x6000, phy 0xF000, otadata 0x10000, flight_a 0x20000 (2 MB), flight_b 0x220000 (2 MB), maint_a 0x420000 (1.5 MB), maint_b 0x5A0000 (1.5 MB); ends 0x720000, inside 8 MB. Image sizes: flight 449,680 B (22 % of its slot), maintenance 851,728 B (55 % of its slot). Both fit with margin. Table offsets are consistent with `build_images.ps1` and `flash_verified.ps1`.

Constants reviewed and left as flight values: launch rules (`flight_state_machine.h:46-63`: 30 m AGL + 30 m/s for 300 ms, or 2.5 g axial for 300 ms), airborne 20 m/s 1 s, landing 30 s window, `kAutomaticTransitionsEnabled = true`, `kFalseFlightRestartEnabled = false`. GPS fixed-baud rules in `gps_receiver.cpp` are bench-derived but are flight settings.

## Checklist: image a flight board (from what the scripts do)

Preparation (once per release, on the build machine)
1. Decide the release commit. `git status`: no modified tracked files (Horizon log files are the current offender). `git stash list` reviewed (finding 8). Tag the commit.
2. Delete the stale root `sdkconfig` and the empty `build_cmd`, `build_fsm`, `build_iris` dirs; do not use `flashit.ps1`/`flashit.sh`.
3. Run `.\build_images.ps1 -Clean` from `Software/Witness/softwarev1` (loads the IDF 6.1 PowerShell profile, builds flight into `build/` with `sdkconfig.defaults;sdkconfig.flight`, maintenance into `build_maint/` with `WITNESS_IMAGE=maintenance` and `sdkconfig.maint`, copies `flight.bin`, `maint.bin`, `bootloader.bin`, `partition-table.bin`, `ota_data_initial.bin` into `images/`, writes `images/flash_args`).
4. Confirm the build output has no errors, and that `images/flash_args` reads: line 1 `--flash-mode dio --flash-freq 80m --flash-size 8MB`; then `0x0 bootloader.bin`, `0x8000 partition-table.bin`, `0x10000 ota_data_initial.bin`, `0x20000 flight.bin`, `0x420000 maint.bin`.
5. Confirm `build/project_description.json` `project_version` equals the tag with no `-dirty`.
6. Grep `build/sdkconfig`: `CONFIG_WITNESS_IMAGE_FLIGHT=y`, `COMPILER_OPTIMIZATION_SIZE=y`, `ESP_DEFAULT_CPU_FREQ_MHZ=240`, `ESP32S3_INSTRUCTION_CACHE_32KB=y`, `ESP32S3_DATA_CACHE_32KB=y`, `ESP_SYSTEM_PANIC_PRINT_REBOOT=y`, `ESP_TASK_WDT_TIMEOUT_S=8`, `BOOTLOADER_APP_ROLLBACK_ENABLE=y`, `ESP_BROWNOUT_DET=y`, `LOG_DEFAULT_LEVEL=3`.
7. Run the host tests (below). Record SHA-256 and size of the five images.

Per board (what `flash_verified.ps1` does)
8. Connect the board through the bench RFC2217 port; give `-Port` explicitly. Ports follow enumeration order and can swap after a replug or hub power-cycle, so confirm the board identity first.
9. `flash_verified.ps1 -CheckOnly`: reads the app descriptor at 0x20000, 0x220000, 0x420000, 0x5A0000, 0x320000, 0x10000; refuses unless chip is ESP32-S3 (exit 3) and app name is `softwarev1` (exit 4; `-Force` only for a blank Witness). Resets the board. The WIMG record (magic 0x474D4957) tells flight (1) or maintenance (2).
10. Run `flash_verified.ps1` (no `-CheckOnly`): same identity check, then `esptool write_flash` at 460800 baud with `images/flash_args`, `--before usb-reset --after hard-reset`. NVS (calibration, stored radio power), phy and the NAND log are not touched. otadata is rewritten, so the board boots flight_a and a pending OTA state is cleared.
11. Do not use `-Force` on a board that already runs Witness.

Post-flash acceptance (not performed by any script; add them)
12. Console/USB: confirm version string equals the tag, flight image kind, boot reaches Pad Idle, no panic/reset loop.
13. Confirm the IMU calibration in NVS is intact (or recalibrate).
14. Set the radio drive: send 0x5250 with the flight value (>= -3 dBm, <= +3 dBm) in Pad Idle; reboot and confirm "restored from NVS" in the log (`sensors.cpp:288`); never exceed +3 dBm into the FEM.
15. Run the on-command BIT; require GO (radio, heap, clocks 240 MHz, GPS, NAND, battery). Check heap min against the 8 KB limit (expected ~15 KB).
16. Confirm HITL is not entered, no fault injection active, Wi-Fi is off, the flight state is Pad Idle, and the log slot is empty/erased as intended (erase of bench logs is owner-approved).
17. Record serial/board ID, commit, SHA-256s, radio power, BIT result in the release log.

## Test results

Host C++ tests (`g++ -std=c++20 -Imain -Itests/stubs`, objects and binaries in a scratch dir, linking the matching `main/*.cpp` where the test needs it): **29 of 29 pass**.

PASS: accel_filter, attitude_estimator, bit_logic, bit_marker_scan, bit_policy, bit_stats, command_policy, erase_wipe, flight_state_machine, gps_assist, gps_receiver (with nmea_gps, px1120s_gps, gps_assist), gps_trust, heading_align (with attitude_estimator), hitl_input, image_info, imu_calibration, imu_calibration_store, imu_supervisor, iris_load_tracker, landed_logic, led_pattern, log_slot_logic, nmea_gps, position_filter, px1120s_gps, radio_edge_logic, skytraq_i2c_mailbox, tx_slot, wifi_ota_logic.

Note: the tests have no build script; the link-time source files must be guessed. A `tests/run_all.sh` would make this repeatable.

Software/HITL pytest: **not run**. `pytest` is not installed on this machine (`No module named pytest`) and installing it was outside the read-only scope. 14 files in `Software/HITL/tests`.

Clean build from the committed tree (git archive of HEAD into a scratch dir, `managed_components` copied): see the result block below.

Result (CONFIRMED): both images built cleanly from the committed tree (HEAD a06d14d, via `git archive`, flight and maintenance, same `SDKCONFIG_DEFAULTS` overlays as `build_images.ps1`, build dirs in the system temp folder). Notes:
- A first attempt in a deeply nested scratch path failed with a Windows path-length error in mbedtls (`fatal error: opening dependency file ... No such file`); a short build path fixed it. Keep the build directory path short on Windows.
- Sizes equal the shipped images exactly: flight 449,680 B, maintenance 851,728 B.
- Generated sdkconfig for both is identical to `build/sdkconfig` and `build_maint/sdkconfig` (no differences in any `CONFIG_` line).
- Binary comparison against `images/flight.bin` and `images/maint.bin`: 85 bytes differ in each, confined to two 4 KB regions (the image header/app descriptor area at the start and the trailing hash/checksum), i.e. the build timestamp, compile date/time and the digest that covers them. Same code and data otherwise. This supports (not proves) that the shipped flight image equals the committed firmware.
- Bootloader differs at byte 78 (build date); partition table is byte-identical.
- The archive build stamps `project_version` "1" (no git metadata); in the repo it is the `git describe` string (finding 1).
- Build warnings: none seen in the filtered output of the `error:` search; the only matches were the git-metadata message and the path-length failure above.
