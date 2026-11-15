# witnessOS: flash Witness over Wi-Fi (field laptop)

Status: design plus `ota_workflow.py`, `ota_wifi.py` and `tests/test_ota_workflow.py` (fakes only; nothing here has run against a board or a real adapter). `main.py` is not touched.

## Constraint
The laptop running witnessOS does everything itself: it joins the Witness access point (`witness-XXXX`, WPA2, 192.168.4.1) with its own Wi-Fi and uploads with `witness_ota.Witness` directly. No Pi, no SSH, no bench setup. (`witness_ota.py` is stdlib and already works this way when run on any machine that has joined the AP; the bench Pi path stays a separate developer option.) The radio/USB link witnessOS already has carries only the commands that put Witness into maintenance.

## End-to-end flow (`OtaWorkflow.update(path, force)`)
| Step | What | Refuse / fail handling |
|---|---|---|
| inspect | read the .bin: `check_image` + the 48-byte image-info record at offset 288 (kind, version, store/command-set versions, LoRa settings, build id, size vs slot). A `flight.bin`/`maint.bin` sibling in the same folder is checked with a Python port of `check_maintenance` | not a Witness image, no image-info, too big, sibling incompatible: refused, nothing sent |
| precheck | fresh telemetry (<6 s) with flight state Pad Idle or Landed (`status & 7` in 0,4, same as `command_policy`). A fresh 0x08 beacon skips switch and precheck | no telemetry or any flight state: refused. Beacon in USB-export mode: refused |
| switch | `build_wifi_packet(True, 0, minutes)` through `Ports.send` (reliable send, `command_link.send_reliable`) | NAK: refused with `detail_name` (0x0001..0x0008) or "refused by flight state". Lost ack: still wait for the beacon |
| beacon | wait for 0x08 with `ap_up`; then wait while `unconfirmed` (unconfirmed maintenance refuses OTA) | acked but flight telemetry continues: "switch did not happen", nothing changed. Neither beacon nor telemetry: outcome `unknown` (any reset returns Witness to flight) |
| wifi | snapshot current network, find `witness-*` SSIDs, join (see below), poll `GET /info` until it answers | never reachable: failed, Wi-Fi restored, HTTP reboot sent if reachable |
| check | `/info`: kind must be maintenance, not unconfirmed; stage B compat against the running maintenance image's reported store/command-set (firmware 409 stays the authority) | refused before any byte is sent |
| upload | `Witness.upload` with progress, retried once from byte 0 if the connection drops | HTTP 4xx (409 incompatible, 422 invalid) is final: refused. A partial upload is harmless: the inactive slot is only booted after the full image validates |
| verify | flight image: Witness restarts into it, no Wi-Fi. Maintenance image: `wait_for_reboot` (new image self-tests 30 s with Wi-Fi up, then `valid`, not unconfirmed) | timeout: judge from beacon/telemetry after restoring Wi-Fi: `rolled_back` or `unknown` |
| leave | `POST /reboot` returns maintenance to flight (fallback: radio 0x5746 off) | |
| restore | reconnect the previous Wi-Fi (before waiting for telemetry, because the command link may itself run over that network) | failure is reported loudly with the old SSID |
| confirm | flight telemetry returns. Flight image: watch 45 s; a second uptime reset or a new beacon means the bootloader rolled back (`rolled_back`); status bits 0x10/0x80 give a warning | no telemetry: `unknown` |

"Update maintenance image" is the same flow with a maintenance .bin. "Download logs" is `download_logs(dir)`: same switch/join, `/logs` then `/logs/<name>`, leave, restore. USB export (`witness_download.py`) remains the alternative.

Outcome is always one of success, refused, rolled_back, unknown, cancelled, failed, with `Result.board` saying which image Witness is believed to run. The invariant that holds on every path, including cancel and power loss: the board is in the old image (nothing flashed), the new image (validated) or maintenance, and any reset returns maintenance to flight.

## Wi-Fi join (`ota_wifi.py`, injectable `WifiBackend`)
Interface: `snapshot()`, `visible_ssids()`, `join()`, `local_ip()`, `restore()`. Implementations:
- **NetshBackend (Windows, first target).** Read-only: `netsh wlan show interfaces|networks`, `netsh interface ip show address`. Changing: `wlan add profile` (temporary profile `witness-ota-tmp`, WPA2-PSK/AES, `connectionMode manual` so Windows never roams to it; the XML with the password is deleted straight after use), `wlan connect name= ssid= interface=`, later `wlan connect` to the previous profile, then `wlan delete profile` (deleting first would drop a connected network). Disconnected before: `wlan disconnect`.
- **ManualBackend (guided).** Changes nothing: the dialog shows SSID and password (default `witness-ota-2026`, `CONFIG_WITNESS_WIFI_PASSWORD`, make it a setting), and the workflow just polls `/info` until it answers. Also the fallback whenever the user declines automatic join, the OS is unsupported, or `join` fails.
- **Linux** would need `NmcliBackend`: `nmcli -t -f ACTIVE,SSID,DEVICE dev wifi` for the snapshot, `nmcli dev wifi connect <ssid> password <pw> ifname <if>` with a throwaway connection (`ipv4.never-default yes`, autoconnect no), `nmcli connection up <old>` and `connection delete` to restore; may need polkit rights.
- **macOS** would need `networksetup -getairportnetwork en0`, `-setairportnetwork en0 <ssid> <pw>`, restore by the same call with the old SSID (password from Keychain is not readable, so restore may re-prompt); verify on a real Mac.

Safety rules in the workflow: automatic join happens only if `auto_join` is on, the backend is automatic, and `confirm_join(text)` returns True. The text names the network being left and warns of internet loss. The previous network is written to a `RestoreJournal` file before the join and `ota_wifi.recover()` (call at witnessOS startup) undoes an interrupted update; restore also runs on success, failure and cancel (`_cleanup`). Windows may mark the AP "no internet" and, in some configurations, prefer another network; the profile is manual and `NetWitness` re-joins while waiting for the new maintenance image, but this needs a real-laptop check. Ethernet plus Wi-Fi: `NetWitness` binds its sockets to the Wi-Fi adapter's IPv4 (`source_address`), so traffic to 192.168.4.1 cannot leave over Ethernet (Windows picks the on-link /24 anyway unless Ethernet is also 192.168.4.x; warn on that overlap).

Caveat: if the command link is TCP over the same Wi-Fi (e.g. a networked ground station), joining the AP cuts it. The switch and the final telemetry confirm happen outside the joined window by design; the dialog should warn when the target is TCP-over-Wi-Fi.

## Module split
- `witness_ota.py`: unchanged library (`check_image`, `Witness`, `wait_for_reboot`). `NetWitness` subclasses `Witness` to bind the source address.
- `ota_wifi.py`: backends, journal, XML profile.
- `ota_workflow.py`: `inspect_image`, `check_pair`, `OtaWorkflow(Ports, WifiBackend, Callbacks, ...)`, runs on a worker thread, `cancel` is a `threading.Event`.
- `Ports` hooks for main.py: `send(frame)` = `core.command_link.send_reliable(write, frame, ...)` using the selected target (what `transmit_command` does); `beacon()` = last 0x08 packet and its age; `flight()` = (status, uptime, age) from the newest state/sensors packet (`on_decoded_packet` already sees every packet).

## UI
Dialog "Update Witness firmware": file picker, image card (kind, version, size, compatibility verdict, refusals in red), checkboxes "automatic Wi-Fi join" (off by default) and "force", step list (the `STEPS` tuple with running/done/failed/skipped marks), one progress bar (upload and log bytes), a log pane, Cancel (sets the event). Join confirmation and SSID choice are modal prompts raised from the worker via signals. It reuses the existing maintenance banner: banner on beacon stays, plus "Update in progress, do not power off". Menu entries: update flight image, update maintenance image, download logs.

## Remaining hook-up in main.py (few lines)
Menu action opening the dialog; a `QThread` running `OtaWorkflow`; signal bridges for `Callbacks`; `Ports` lambdas above (beacon and flight age tracking may need a timestamp on the latest packets); call `ota_wifi.recover(...)` at startup; disable normal command sending while the workflow runs.

## Only verifiable on a real laptop with the board
Locale-dependent `netsh` output (English parsed only); that Windows keeps the Witness profile connected without internet and does not hop back; eduroam (802.1X) reconnect by profile name; admin/permission needs of `add profile`; the Witness DHCP lease and `local_ip` parse; `/info` field names for per-slot store/command-set (stage B is tolerant and skips when absent); beacon timing over LoRa; USB serial re-enumeration when Witness restarts into maintenance (the selected port may change); whether flight self-test rollback shows as a beacon or a second uptime reset; throughput and the 5-minute confirm window.

## Open points
Flight packets carry no image id, so a flight update is confirmed by stability, not by version; a definitive check needs a firmware change (image id in a flight ack) or re-entering maintenance and reading `/info`. `GROUND_STATION_PROTOCOL.md` row for 0x5746 still says LoRa TX stops while Wi-Fi is up, contradicting FIRMWARE_HANDOFF. A refused switch because the installed maintenance image is incompatible can only be fixed by USB flashing.
