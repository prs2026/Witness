#!/usr/bin/env python3
"""Update Witness firmware and download flight logs over its Wi-Fi access point.

Standard library only, Python 3.9+, so it runs on the ground-station Raspberry
Pi. Join the "witness-XXXX" access point (WPA2, 192.168.4.1) first; Wi-Fi is
turned on with the 0x5746 command (witness_protocol.build_wifi_packet) in Pad
Idle or Landed and turns itself off after 15 minutes without HTTP activity.

    python3 witness_ota.py info
    python3 witness_ota.py upload softwarev1.bin
    python3 witness_ota.py logs
    python3 witness_ota.py get all -o ./witness_logs
    python3 witness_ota.py reboot
    python3 witness_ota.py wifi-off

Reading logs freezes Witness's flash logging until it reboots; wifi-off after
a download therefore reboots Witness a couple of seconds later. Witness only
allows log download in the Landed flight state (409 otherwise), so a launch
can never follow a download with logging frozen.
"""

from __future__ import annotations

import argparse
import http.client
import json
import os
import sys
import time
import urllib.parse

DEFAULT_BASE = "http://192.168.4.1"
PROJECT_NAME = "softwarev1"
CHIP_ID_ESP32S3 = 9
CHUNK = 4096

# esp_image_header_t (24) + first esp_image_segment_header_t (8) + esp_app_desc_t.
APP_DESC_OFFSET = 32
APP_DESC_MAGIC = 0xABCD5432
HEAD_LENGTH = APP_DESC_OFFSET + 256


class OtaError(Exception):
    pass


def check_image(head: bytes, project: str = PROJECT_NAME) -> dict:
    """Validate the start of an application image; returns its descriptor fields.

    Raises OtaError when it is not an ESP32-S3 image of this project."""
    if len(head) < HEAD_LENGTH:
        raise OtaError("file is too short to be an application image")
    if head[0] != 0xE9:
        raise OtaError("not an ESP32 application image (bad magic byte)")
    if int.from_bytes(head[12:14], "little") != CHIP_ID_ESP32S3:
        raise OtaError("image is not built for the ESP32-S3")
    desc = head[APP_DESC_OFFSET:APP_DESC_OFFSET + 256]
    if int.from_bytes(desc[0:4], "little") != APP_DESC_MAGIC:
        raise OtaError("application descriptor not found in the image")
    version = desc[16:48].split(b"\0", 1)[0].decode("ascii", "replace")
    name = desc[48:80].split(b"\0", 1)[0].decode("ascii", "replace")
    if name != project:
        raise OtaError("image is project %r, expected %r" % (name, project))
    return {"project": name, "version": version}


class Witness:
    """Minimal HTTP client for the Witness Wi-Fi endpoints."""

    def __init__(self, base: str = DEFAULT_BASE, timeout: float = 15.0):
        parsed = urllib.parse.urlparse(base if "//" in base else "http://" + base)
        if parsed.scheme != "http" or not parsed.hostname:
            raise OtaError("base URL must look like http://192.168.4.1")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.timeout = timeout

    def _connect(self, timeout: float | None = None) -> http.client.HTTPConnection:
        return http.client.HTTPConnection(self.host, self.port, timeout=timeout or self.timeout)

    def request_json(self, method: str, path: str, body: bytes | None = None,
                     timeout: float | None = None) -> dict:
        conn = self._connect(timeout)
        try:
            try:
                conn.request(method, path, body=body)
                response = conn.getresponse()
                data = response.read()
            except (OSError, http.client.HTTPException) as exc:
                raise OtaError("cannot reach Witness at %s:%d (%s)" % (self.host, self.port, exc))
        finally:
            conn.close()
        try:
            parsed = json.loads(data.decode("utf-8")) if data else {}
        except ValueError:
            parsed = {"ok": False, "error": data[:80].decode("utf-8", "replace")}
        if response.status != 200:
            raise OtaError("HTTP %d: %s" % (response.status, parsed.get("error", response.reason)))
        return parsed

    def info(self, timeout: float | None = None) -> dict:
        return self.request_json("GET", "/info", timeout=timeout)

    def logs(self) -> dict:
        return self.request_json("GET", "/logs", timeout=60.0)

    def reboot(self) -> dict:
        return self.request_json("POST", "/reboot")

    def wifi_off(self) -> dict:
        return self.request_json("POST", "/wifi/off")

    def upload(self, image: bytes, progress=None) -> dict:
        """POST the image to /ota with a Content-Length, streamed in chunks."""
        conn = self._connect(60.0)
        try:
            try:
                conn.putrequest("POST", "/ota")
                conn.putheader("Content-Type", "application/octet-stream")
                conn.putheader("Content-Length", str(len(image)))
                conn.endheaders()
                sent = 0
                try:
                    while sent < len(image):
                        piece = image[sent:sent + CHUNK]
                        conn.send(piece)
                        sent += len(piece)
                        if progress:
                            progress(sent, len(image))
                except ConnectionError:
                    pass   # Witness rejected the image early: read its reason below
                try:
                    response = conn.getresponse()
                    data = response.read()
                except (ConnectionError, http.client.HTTPException) as exc:
                    raise OtaError("connection closed during the upload (%s); Witness probably refused "
                                   "it: check 'info' (flight state must be PadIdle or Landed)" % exc)
            except (OSError, http.client.HTTPException) as exc:
                raise OtaError("upload failed: %s" % exc)
        finally:
            conn.close()
        try:
            parsed = json.loads(data.decode("utf-8")) if data else {}
        except ValueError:
            parsed = {}
        if response.status != 200:
            raise OtaError("Witness refused the update (HTTP %d): %s"
                           % (response.status, parsed.get("error", response.reason)))
        return parsed

    def download(self, name: str, destination: str, progress=None) -> int:
        """Stream /logs/<name> to a file; returns the byte count."""
        conn = self._connect(60.0)
        written = 0
        partial = destination + ".part"
        try:
            try:
                conn.request("GET", "/logs/" + urllib.parse.quote(name))
                response = conn.getresponse()
                if response.status != 200:
                    detail = response.read().decode("utf-8", "replace")[:100]
                    raise OtaError("HTTP %d for %s: %s" % (response.status, name, detail))
                expected = int(response.getheader("X-File-Size", "-1"))
                with open(partial, "wb") as handle:
                    while True:
                        chunk = response.read(CHUNK)
                        if not chunk:
                            break
                        handle.write(chunk)
                        written += len(chunk)
                        if progress:
                            progress(written, expected)
            except (OSError, http.client.HTTPException) as exc:
                raise OtaError("download of %s failed after %d bytes: %s" % (name, written, exc))
        finally:
            conn.close()
        if expected >= 0 and written != expected:
            raise OtaError("%s is %d bytes, expected %d" % (name, written, expected))
        os.replace(partial, destination)
        return written


def _progress_printer(label: str):
    state = {"last": 0.0}

    def show(done: int, total: int) -> None:
        now = time.monotonic()
        if done < total and now - state["last"] < 0.25:
            return
        state["last"] = now
        if total > 0:
            sys.stdout.write("\r%s %d / %d bytes (%d%%)" % (label, done, total, 100 * done // total))
        else:
            sys.stdout.write("\r%s %d bytes" % (label, done))
        sys.stdout.flush()
    return show


def wait_for_reboot(witness: Witness, expect_version: str | None, timeout: float = 90.0,
                    sleep=time.sleep, clock=time.monotonic) -> dict:
    """Wait for Witness to go down and come back, then for the new image to be
    marked valid (it self-tests for up to 30 s). Returns the final /info."""
    deadline = clock() + timeout
    rebooted = False
    last_uptime = None
    last = None
    while clock() < deadline:
        try:
            last = witness.info(timeout=3.0)
        except OtaError:
            rebooted = True          # the access point drops while Witness restarts
            sleep(1.0)
            continue
        uptime = last.get("uptime_ms")
        if last_uptime is not None and uptime is not None and uptime < last_uptime:
            rebooted = True
        last_uptime = uptime
        right_image = expect_version is None or last.get("version") == expect_version
        if rebooted and right_image and last.get("app_state") == "valid":
            return last
        sleep(1.0)
    if last is not None and not rebooted:
        raise OtaError("Witness did not reboot (still running %s)" % last.get("version"))
    raise OtaError("timed out waiting for the new image (app_state must become valid); "
                   "if Wi-Fi is off, turn it on again with the 0x5746 command and run 'info'")


def cmd_info(args, witness: Witness) -> int:
    print(json.dumps(witness.info(), indent=2))
    return 0


def cmd_upload(args, witness: Witness) -> int:
    with open(args.image, "rb") as handle:
        image = handle.read()
    desc = check_image(image[:HEAD_LENGTH])
    print("image %s version %s, %d bytes" % (desc["project"], desc["version"], len(image)))
    before = witness.info()
    print("Witness runs %s (%s), updating %s" % (before.get("version"), before.get("running"),
                                                 before.get("next")))
    result = witness.upload(image, _progress_printer("uploading"))
    print()
    print("uploaded %s bytes into %s; Witness reboots in %s ms" % (
        result.get("bytes"), result.get("slot"), result.get("reboot_ms")))
    if args.no_wait:
        return 0
    print("waiting for the reboot and the new image's self-test (the first boot after an "
          "update opens Wi-Fi for a few minutes so this can confirm it)")
    final = wait_for_reboot(witness, desc["version"], timeout=args.wait)
    print("now running %s from %s, app state %s" % (final.get("version"), final.get("running"),
                                                    final.get("app_state")))
    return 0


def cmd_logs(args, witness: Witness) -> int:
    listing = witness.logs()
    print("%d file(s); Witness flash logging is frozen until it reboots" % listing.get("count", 0))
    for entry in listing.get("files", []):
        print("%-14s %10d bytes  session %s" % (entry["name"], entry["size"], entry.get("session", "?")))
    return 0


def cmd_get(args, witness: Witness) -> int:
    os.makedirs(args.output, exist_ok=True)
    if args.name.lower() == "all":
        names = [entry["name"] for entry in witness.logs().get("files", [])]
    else:
        names = [args.name]
    for name in names:
        destination = os.path.join(args.output, os.path.basename(name))
        size = witness.download(name, destination, _progress_printer(name))
        print("\r%s: %d bytes -> %s" % (name, size, destination))
    return 0


def cmd_reboot(args, witness: Witness) -> int:
    print(json.dumps(witness.reboot()))
    return 0


def cmd_wifi_off(args, witness: Witness) -> int:
    print(json.dumps(witness.wifi_off()))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Witness Wi-Fi OTA update and log download")
    parser.add_argument("--base", default=DEFAULT_BASE, help="Witness URL (default %(default)s)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("info", help="firmware, partition and flight state").set_defaults(func=cmd_info)
    up = sub.add_parser("upload", help="flash a softwarev1.bin into the other OTA slot")
    up.add_argument("image")
    up.add_argument("--no-wait", action="store_true", help="do not wait for the reboot")
    up.add_argument("--wait", type=float, default=90.0, help="seconds to wait for the new image")
    up.set_defaults(func=cmd_upload)
    sub.add_parser("logs", help="list flight logs (freezes logging until reboot)").set_defaults(func=cmd_logs)
    get = sub.add_parser("get", help="download one log file, or 'all'")
    get.add_argument("name")
    get.add_argument("-o", "--output", default=".", help="destination directory")
    get.set_defaults(func=cmd_get)
    sub.add_parser("reboot", help="reboot Witness (Pad Idle or Landed)").set_defaults(func=cmd_reboot)
    sub.add_parser("wifi-off", help="turn the access point off").set_defaults(func=cmd_wifi_off)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args, Witness(args.base))
    except OtaError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1
    except OSError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
