# Imaging Witness for flight

This guide takes a Witness board from source code to a flashed, checked and configured flight unit. It covers Witness only. Iris and Aperture have their own projects.

Current flight candidate: **`a443972`**. It is verified on the bench on 2026-11-20 (`Software/HITL/runs/20261120-005819/verify_summary.md`).

## 0. Before you start

- **Board hardware:**
  - The 3.3 V regulator must be the corrected part, the TPL740F33, fitted the right way round. Measure the 3V3 rail at about 3.3 V before connecting anything. The first v0.2 boards had a swapped part that put about 4.3 V on the rail.
  - The battery must be charged.
  - The GPS and LoRa antennas must be fitted.
- **PC setup:**
  - Windows with ESP-IDF 6.1 installed.
  - A clone of this repository.
  - Horizon, for commands and checking telemetry.
  - Open a PowerShell window and load the IDF environment:

    ```powershell
    . C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1
    cd <repo>\Software\Witness\softwarev1
    ```

- **Never use any of these for Witness:**
  - `flashit.ps1` or `flashit.sh`. They write the old single-image layout over `otadata` and `flight_a`.
  - A plain `idf.py build` or `idf.py flash` from this folder. The `sdkconfig` in this folder is a stale debug config.

## 1. Build the images from a clean commit

1. Check out the commit you want to fly, normally the tip of `main`. Run `git status`: it must show no modified tracked files. Any modified tracked file makes the build stamp `-dirty`, and a dirty image should not fly.
2. Build both images from scratch:

   ```powershell
   .\build_images.ps1 -Clean
   ```

   This builds the **flight** image (`build/`, from `sdkconfig.defaults` + `sdkconfig.flight`) and the **maintenance** image (`build_maint/`). It collects them in `images/`:

   | File | Goes to | What it is |
   |---|---|---|
   | `bootloader.bin` | 0x0 | Second-stage bootloader |
   | `partition-table.bin` | 0x8000 | Partition table (`partitions.csv`) |
   | `ota_data_initial.bin` | 0x10000 | Selects `flight_a` as the boot slot |
   | `flight.bin` | 0x20000 (`flight_a`) | Flight image: all flight logic, no Wi-Fi |
   | `maint.bin` | 0x420000 (`maint_a`) | Maintenance image: Wi-Fi OTA, log download, USB log export |
   | `flash_args` | | The esptool arguments for all of the above |

3. Check the version stamp. It must be the short commit hash, with no `-dirty`:

   ```powershell
   Select-String build\project_description.json -Pattern '"project_version"'
   ```

## 2. Flash

Use only `flash_verified.ps1`. Before it writes anything, it reads the board's installed app descriptor and refuses a board that isn't a Witness (Aperture is `softwareattempt2`, Iris is an ESP32-C3). On the bench Pi, port numbers move around when boards are replugged, so this check matters.

`-Port` is required. There is no default, because a default once sent a flash to the bench board instead of the board on the PC. Find the board's COM port in Device Manager (USB JTAG/serial debug unit) before you start.

```powershell
.\flash_verified.ps1 -Port COM7 -CheckOnly   # a board on this PC's USB: identify only
.\flash_verified.ps1 -Port COM7              # flash it
.\flash_verified.ps1 -Port COM7 -Force       # blank board with no app yet (first flash)
.\flash_verified.ps1 -Port rfc2217://<bench Pi>:7000   # the bench Witness
```

`-Force` skips the "is this a Witness" check, so with `-Force` make sure the port is the board in front of you. Unplug other boards if in doubt.

- `-CheckOnly` must print `is a ESP32-S3 running 'softwarev1' ... OK to flash`.
- The flash writes 5 regions, and each must print **`Hash of data verified`**. If any is missing, flash again. Don't go on to step 3.
- If you get `Failed to set baud rate` while connecting, nothing was written. Run the same command again.
- The NVS partition (IMU calibration, radio power, boot bookkeeping) is not touched, so a reflash keeps the board's calibration.

## 3. Check the boot

After the flash the board resets into the flight image. Look at its USB console (any serial terminal at 115200, or Horizon's console) for these lines:

| Line | Must show |
|---|---|
| `app_init: App version:` | The commit you built, e.g. `a443972` |
| `app_init: ELF file SHA256:` | The first hex digits of the image's ELF hash. `verify_image.py` compares them with `images/flight.bin` for you. |
| `selftest: running flight slot 0 (<version>)` | The flight image, from `flight_a` |
| `uart: GPS identified as PX1120S at 115200 baud` | GPS found at 115200 |
| `uart: PX1120S already at 25 Hz: update-rate command not sent` | The GPS fix from `3a66505`. Without it the receiver can lock up at about 88.4 kbaud for minutes. |

There must be **no** `pending verification` line after about 30 s; a new image confirms itself once its self-test passes. There must also be **no** `GPS receiver is in its ~88.4 kbaud slow state` warning.

If `already at 25 Hz` doesn't appear, the GPS receiver doesn't have 25 Hz saved in its own flash. That can happen with a new or replaced receiver. Witness then sends the rate command, and the receiver can drop into its slow state. Set the receiver to 115200 baud and 25 Hz in its flash over a direct USB-UART link before flight.

## 4. Bench verification (HITL)

On the bench this is one command, about 13 minutes:

```powershell
cd <repo>\Software\HITL
py -3.14 verify_image.py --flash      # flashes images\, then checks; drop --flash if already flashed
```

It does the following:

1. Flashes with `flash_verified.ps1`.
2. Turns Wi-Fi on and off, which boots the maintenance image and then the flight image. It checks both image hashes against `images/`.
3. Erases the logs.
4. Flies one HITL flight on the eayf profile with RF GPS, using a 30 s lead-in.
5. After the log export power-cycles the board, it checks the GPS and the reset reason, runs a commanded BIT, and erases the logs again.

The result goes to `runs/<run>/verify_summary.md`.

| Pass criteria | |
|---|---|
| Result line | `boot OK, HITL flight OK, log is this run's OK, GPS power-up OK` |
| Transitions | Boost, Coast, Descent and Landed each within 0.3 s of truth (the tool allows 10 s) |
| GPS | First fix within about 10 s of RF start; after the power cycle `baud=115201 ... recoveries 0`, no slow-state warning |
| Health | Resets 0, panics 0, heap minimum about 14.9 KB |

`HITL frames rejected` in the summary, typically around 230–350, comes from the bench network link. It isn't a firmware problem and doesn't exist in flight. The one-time GPS scenario for the script is built with `py -3.14 verify_image.py --make-scenario`.

## 5. Configure the board for flight

These settings live in the board's NVS or are set on the day. Use Horizon's Witness monitor over USB (or radio through Aperture). Witness must be in **Pad Idle**.

1. **IMU calibration** (once per board, stored in NVS, kept across reflashes):
   - Close Horizon, then run `python tools/imu_calibrate.py --serial COMx`.
   - Do the gyro capture with the board still. Then do the six faces (+X, −X, +Y, −Y, +Z, −Z), with the board held still on each face for 2 s.
   - Check with `--status`.
   - Until this is done, BIT shows `1.3 nvs: FAIL value 2` and `4.6 calibration: FAIL`.
2. **Flight radio power:**
   - Use Horizon's **Radio power…** button, target Witness, and set **−3 to +3 dBm** with the front end on. Never above +3 dBm: the Ra-01SH-P front-end input limit.
   - The level is stored in NVS (in Pad Idle or Landed only). Reboot and confirm it is still set: BIT `8.3 radio power` must no longer FAIL.
   - The boot default is −17 dBm, which BIT reports as NO-GO.
3. **Aperture ground power:** Aperture keeps its power in RAM only. Re-send it from **Radio power…** with target ground after every Aperture reset.
4. **Erase the logs:** send `0x6869` (Horizon command entry `05 00 00 00 00 00 00 68 69 0a`). If both slots already hold a "flight", nothing is logged. HITL and bench runs that reach Boost count as flights.
5. **HITL off:** it is off after any reset. BIT `1.4 hitl inactive` must pass.
6. **Pad heading:** set it on the pad with **Set pad heading…**, or accept the `4.7 pad heading set` warning.

## 6. Final check before it leaves the bench

Run a commanded BIT (**Run BIT** in Pad Idle), then press **BIT detail**. Every check is listed in the BIT detail box under the BIT row.

| Expected on the bench, clears on the pad | Must not appear |
|---|---|
| `7.4 gps fix`, `7.5 utc valid`, `C.4 time of day` (indoors, no sky) | `1.2 task watchdog` (fixed in `a443972`) |
| `2.8 body orientation` (board not vertical) | `8.3 radio power` after step 5.2 |
| `4.7 pad heading set` until set on the pad | `1.3 nvs` or `4.6 calibration` after step 5.1 |
| `B.1 battery voltage` on USB or bench power | Any other blocking (red) FAIL |

The verdict on the pad must be **GO**, or **GO (warnings)** with only warnings you have read and understood.

## 7. Updating later without USB (Wi-Fi OTA)

Once a board has the two-image layout from step 2, both images can be updated over Wi-Fi:

1. In Pad Idle or Landed, press **Wi-Fi ON (maintenance)** in Horizon. Witness restarts into the maintenance image with its access point on.
2. Join the `witness-XXXX` network.
3. Upload `maint.bin` first if both images change, then `flight.bin`. Use **Upload image…** in the Wi-Fi / OTA box, or `python witness_ota.py upload <file>` from `Software/Horizon`.
4. The image is checked before anything is written, and only the inactive slot is written. A flight upload reboots straight into the new flight image, and a failed self-test rolls back to the previous one.
5. Confirm the version from the state packets or the boot log.
6. Wi-Fi turns itself off after 15 minutes idle, and any reset returns Witness to the flight image.

## Troubleshooting

| Symptom | Cause and action |
|---|---|
| Version ends in `-dirty` | A tracked file was modified when you built. Commit or restore it, then rebuild with `-Clean`. |
| `-CheckOnly` says not OK | The port reaches a different board. On the bench, ports move after replugs or hub power cycles. Find the Witness port, or use `-Port`. |
| `pending verification` stays, then the board reboots into the old image | The new image failed its self-test and was rolled back. Read its boot log for the failing check. |
| GPS at 88.4 kbaud, no fix for minutes | The receiver is in its slow power-up state. On images before `3a66505` this followed every power-up. On a current image, check that `already at 25 Hz` is printed (section 3). |
| Boot loops with `invalid header: 0xffffffff` and `rst:0x7 (TG0WDT_SYS_RST)` | The ROM reads only erased flash (0xFF) where the bootloader should be. Read it back: `esptool -p COMx read-flash 0x0 0x20 hdr.bin`. If `hdr.bin` starts `E9`, the flash is written but unreadable at boot: check `espefuse -p COMx summary` for the VDD_SPI setting and the GPIO45 strap (it must be low at reset for the 3.3 V in-package flash). If it is all `FF`, the write never reached this board: flash again and check for 5 × `Hash of data verified` on this port (`-Force` skips the identity check, so a wrong port flashes another board). |
| Nothing logged in a flight | Both log slots were full. Erase (`0x6869`) before flight and check BIT `6.1 log session`. |
