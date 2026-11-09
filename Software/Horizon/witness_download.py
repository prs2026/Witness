"""Download Witness flash logs through the bench Pi, from this computer.

Witness exports its logs as a read-only USB disk (command 05 68 68). On the
bench that disk appears on the Pi, so this script:

  1. sends 05 68 68 to Witness over its raw TCP data port (ser2net, 7100),
  2. waits for the disk on the Pi and reads it over SSH with
     witness_pi_agent.py (piped to ``python3 -``; nothing is installed and
     nothing is mounted on the Pi),
  3. saves the files under logs/witness_<timestamp>/ here, and optionally
     converts the .BIN files to CSV and opens one in viewer.py.

Witness stays in export mode (no logging, no USB serial) until it is power
cycled; ``--restart`` power-cycles Pi USB hub 2 afterwards, which also resets
every other board on that hub.

Examples:
    python witness_download.py setup               # one time, asks for the Pi sudo password
    python witness_download.py list
    python witness_download.py get --all --convert
    python witness_download.py get I0000012.BIN --view
    python witness_download.py get --latest --convert --restart

SSH: ``--host`` (default prs2026hitl), ``--user``, ``--key``; defaults come
from environment variables WITNESS_PI_HOST / WITNESS_PI_USER / WITNESS_PI_KEY,
then from your ~/.ssh/config.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENT_SOURCE = (HERE / "witness_pi_agent.py").read_bytes()
EXPORT_COMMAND = bytes([0x05, 0x68, 0x68])
UDEV_RULE = (
    'SUBSYSTEM=="block", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4012", '
    'MODE="0660", GROUP="plugdev"'
)
UDEV_RULE_PATH = "/etc/udev/rules.d/99-witness-logs.rules"
HUB_RESTART = "sudo -n /usr/sbin/uhubctl -l 2 -a 2"


class DownloadError(Exception):
    pass


class Pi:
    def __init__(self, host: str, user: str | None, key: str | None):
        self.target = f"{user}@{host}" if user else host
        self.key_args = ["-i", key] if key else []
        self.base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", *self.key_args]

    def agent(self, *args: str, stdout=subprocess.PIPE) -> subprocess.Popen:
        """Runs witness_pi_agent.py on the Pi with the given arguments."""
        command = self.base + [self.target, "python3", "-", *args]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=stdout,
                                   stderr=subprocess.PIPE)
        process.stdin.write(AGENT_SOURCE)
        process.stdin.close()
        return process

    def agent_text(self, *args: str) -> tuple[int, str, str]:
        process = self.agent(*args)
        out, err = process.communicate()
        return process.returncode, out.decode(errors="replace"), err.decode(errors="replace")

    def run(self, remote: str, interactive: bool = False) -> int:
        """Runs a shell command on the Pi; interactive allows a sudo prompt."""
        base = (["ssh", "-t", "-o", "ConnectTimeout=10", *self.key_args]
                if interactive else self.base)
        return subprocess.call(base + [self.target, remote])


def send_export(host: str, port: int) -> None:
    try:
        with socket.create_connection((host, port), timeout=5) as link:
            link.settimeout(0.5)
            try:
                greeting = link.recv(256)
            except socket.timeout:
                greeting = b""
            if b"in use" in greeting:
                raise DownloadError(
                    f"{host}:{port} is busy (another client, e.g. Horizon, has it); close it first")
            link.sendall(EXPORT_COMMAND)
            time.sleep(0.5)
    except OSError as error:
        raise DownloadError(f"cannot reach Witness data port {host}:{port}: {error}") from error


def ensure_disk(pi: Pi, args) -> None:
    code, out, err = pi.agent_text("find")
    if code == 0:
        print(f"Witness log disk present on the Pi: {out.strip()}")
        return
    if args.no_export:
        raise DownloadError(err.strip() or "Witness log disk not found")
    print(f"Sending export command (05 68 68) to {args.host}:{args.port} ...")
    send_export(args.host, args.port)
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline:
        time.sleep(2)
        code, out, err = pi.agent_text("find")
        if code == 0:
            print(f"Witness log disk is up: {out.strip()}")
            time.sleep(1)  # let the kernel finish probing
            return
    raise DownloadError(
        "the Witness log disk did not appear on the Pi; check that Witness is in Pad "
        "Idle, the data port is right, and see `dmesg` on the Pi")


def list_files(pi: Pi) -> list[dict]:
    # A freshly attached disk is root-owned until udev applies the rule.
    for _attempt in range(5):
        code, out, err = pi.agent_text("list")
        if code != 4:
            break
        time.sleep(1)
    if code != 0:
        raise DownloadError(err.strip() or f"list failed ({code})")
    return [json.loads(line) for line in out.splitlines() if line.strip()]


def human(size: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024 or unit == "GB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return str(size)


def describe(name: str) -> str:
    upper = name.upper()
    if upper.startswith("I") and upper.endswith(".BIN"):
        return "IMU records (24-byte)"
    if upper.startswith("S") and upper.endswith(".BIN"):
        return "sensor records (48-byte WL)"
    if upper.startswith("S") and upper.endswith(".TXT"):
        return "diagnostic log text"
    if upper.startswith("P") and upper.endswith(".BIN"):
        return "packets: state 100 Hz, CAN in/out, radio commands"
    if upper.startswith("U") and upper.endswith(".BIN"):
        return "raw GPS UART bytes"
    return ""


def generation(name: str) -> int:
    """Session generation: Witness names files <type><7 hex digits>.<ext>."""
    match = re.match(r"[A-Za-z]([0-9A-Fa-f]{7})\.", name)
    return int(match.group(1), 16) if match else -1


def download(pi: Pi, name: str, size: int, destination: Path) -> None:
    started = time.monotonic()
    process = pi.agent("cat", name)
    received = 0
    last_report = 0.0
    with destination.open("wb") as out:
        while True:
            chunk = process.stdout.read(65536)
            if not chunk:
                break
            out.write(chunk)
            received += len(chunk)
            now = time.monotonic()
            if now - last_report > 0.5:
                last_report = now
                rate = received / max(now - started, 1e-3)
                percent = 100.0 * received / size if size else 100.0
                print(f"\r  {name}: {human(received)} / {human(size)} ({percent:.0f}%), "
                      f"{human(rate)}/s   ", end="", flush=True)
    error = process.stderr.read().decode(errors="replace").strip()
    code = process.wait()
    elapsed = time.monotonic() - started
    print(f"\r  {name}: {human(received)} in {elapsed:.1f} s" + " " * 30)
    if code != 0 or received != size:
        destination.unlink(missing_ok=True)
        raise DownloadError(f"{name}: download failed ({error or f'{received} of {size} bytes'})")


def convert(path: Path) -> Path | None:
    if path.suffix.upper() != ".BIN" or path.name.upper().startswith("U"):
        return None  # U*.BIN is raw NMEA text; open it in an editor
    output = path.with_suffix(".csv")
    result = subprocess.run([sys.executable, str(HERE / "converter.py"), str(path),
                             "--output", str(output)], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  convert {path.name} failed: {result.stderr.strip() or result.stdout.strip()}")
        return None
    print(f"  converted {path.name} -> {output.name}")
    return output


def command_setup(pi: Pi) -> int:
    print("Installing a udev rule on the Pi so your user can read the Witness log disk.")
    print(f"  {UDEV_RULE_PATH}: {UDEV_RULE}")
    print("You will be asked for the Pi sudo password.")
    remote = (
        f"echo '{UDEV_RULE}' | sudo tee {UDEV_RULE_PATH} >/dev/null && "
        "sudo udevadm control --reload-rules && "
        "sudo udevadm trigger --subsystem-match=block && "
        "id -nG | grep -qw plugdev && echo 'done (your user is in plugdev)' || "
        "echo 'rule installed; add your user to plugdev: sudo usermod -aG plugdev $USER'"
    )
    return pi.run(remote, interactive=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("--host", default=os.environ.get("WITNESS_PI_HOST", "prs2026hitl"))
    parser.add_argument("--user", default=os.environ.get("WITNESS_PI_USER"))
    parser.add_argument("--key", default=os.environ.get("WITNESS_PI_KEY"))
    parser.add_argument("--port", type=int, default=7100, help="Witness raw TCP data port")
    parser.add_argument("--wait", type=float, default=30, help="seconds to wait for the disk")
    parser.add_argument("--no-export", action="store_true",
                        help="do not send 05 68 68; the disk must already be up")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("setup", help="one-time Pi permission setup (udev rule)")
    sub.add_parser("list", help="list the files on Witness")
    get = sub.add_parser("get", help="download files")
    get.add_argument("names", nargs="*", help="file names, e.g. I0000012.BIN")
    get.add_argument("--all", action="store_true", help="every file")
    get.add_argument("--latest", action="store_true",
                     help="every file of the newest generation")
    get.add_argument("--skip-gps", action="store_true", help="leave out U*.BIN (large)")
    get.add_argument("--out", type=Path, help="destination folder")
    get.add_argument("--convert", action="store_true", help="convert .BIN to CSV")
    get.add_argument("--view", action="store_true",
                     help="open the first converted CSV in viewer.py (implies --convert)")
    for p in (get, sub.choices["list"]):
        p.add_argument("--restart", action="store_true",
                       help="afterwards power-cycle Pi USB hub 2 (resets ALL boards on it)")
    args = parser.parse_args()
    sys.stdout.reconfigure(line_buffering=True)
    pi = Pi(args.host, args.user, args.key)

    try:
        if args.command == "setup":
            return command_setup(pi)
        ensure_disk(pi, args)
        files = list_files(pi)
        if args.command == "list":
            for entry in files:
                print(f"  {entry['name']:<13} {human(entry['size']):>10}  {describe(entry['name'])}")
            if not files:
                print("  (no files)")
        else:
            wanted = files
            if args.latest:
                newest = max((generation(f["name"]) for f in files), default=-1)
                wanted = [f for f in files if generation(f["name"]) == newest]
            elif not args.all:
                names = {n.upper() for n in args.names}
                if not names:
                    raise DownloadError("name files to download, or use --all / --latest")
                wanted = [f for f in files if f["name"].upper() in names]
                missing = names - {f["name"].upper() for f in wanted}
                if missing:
                    raise DownloadError(f"not on Witness: {', '.join(sorted(missing))}")
            if args.skip_gps:
                wanted = [f for f in wanted if not f["name"].upper().startswith("U")]
            out_dir = args.out or HERE / "logs" / datetime.now().strftime("witness_%Y%m%d_%H%M%S")
            out_dir.mkdir(parents=True, exist_ok=True)
            print(f"Downloading {len(wanted)} file(s) to {out_dir}")
            converted: list[Path] = []
            for entry in wanted:
                path = out_dir / entry["name"]
                download(pi, entry["name"], entry["size"], path)
                if args.convert or args.view:
                    csv_path = convert(path)
                    if csv_path is not None:
                        converted.append(csv_path)
            if args.view and converted:
                imu_first = sorted(converted, key=lambda p: not p.name.upper().startswith("I"))
                subprocess.Popen([sys.executable, str(HERE / "viewer.py"), str(imu_first[0])])
                print(f"Opened {imu_first[0].name} in viewer.py")
        return 0
    except DownloadError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        if args.command != "setup":
            if args.restart:
                print("Power-cycling Pi USB hub 2 to return Witness to normal ...")
                result = subprocess.run(pi.base + [pi.target, HUB_RESTART],
                                        capture_output=True, text=True)
                if result.returncode != 0:
                    print("  hub restart failed; replug Witness instead: "
                          + (result.stderr.strip() or result.stdout.strip()))
                else:
                    print("  done; Witness reboots into normal logging")
            else:
                print("Witness stays in export mode until power-cycled (use --restart).")


if __name__ == "__main__":
    sys.exit(main())
