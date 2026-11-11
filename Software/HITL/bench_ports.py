"""Which ser2net port the Witness is on (the bench Pi), by what it sends.

ser2net ports follow /dev/ttyACM enumeration, which changes whenever boards
are replugged or the hub is power-cycled, and MACs change when a board is
swapped. So boards are recognised by their output on the raw TCP ports:

- Witness: ESP-IDF log text with its own tags (sensors, flash_logger, uart
  "GPS RX", iris_can, cpu) plus mirrored GPS NMEA. It prints "GPS RX" every
  5 s, so a 6 s listen always sees it.
- Iris: only binary 0x03 camera frames, no text.
- Aperture: binary 0x02/0xFF frames it forwards, no text.

Listening is passive (nothing is sent, nothing resets). For flashing, see
Witness/softwarev1/flash_verified.ps1, which checks the chip and the
installed application's project name instead.

    py -3.14 bench_ports.py            # prints {raw port: role}
"""

from __future__ import annotations

import re
import socket
import sys
import time

DEFAULT_HOST = "prs2026hitl"
RAW_PORTS = (7100, 7101, 7102)
WITNESS_TAGS = ("sensors", "flash_logger", "iris_can", "cpu", "serial_command", "data_forwarder")
ANSI = re.compile(rb"\x1b\[[0-9;]*m")


def classify(data: bytes) -> str:
    """'witness', 'iris', 'aperture' or 'unknown' from a few seconds of output."""
    text = ANSI.sub(b"", data).decode("latin1")
    tags = set(re.findall(r"[IWE] \(\d+\) (\w+):", text))
    if tags & set(WITNESS_TAGS) or "uart: GPS RX" in text:
        return "witness"
    # Witness also mirrors its GPS NMEA next to its own 0x01 sensor packets.
    if re.search(r"\$G[NPLAB]GGA,", text) and frame_counts(data).get(0x01, 0) >= 1:
        return "witness"
    if not data:
        return "unknown"
    counts = frame_counts(data)
    if counts.get(0x02, 0) + counts.get(0xFF, 0) >= 2:
        return "aperture"
    if counts.get(0x03, 0) >= 2:
        return "iris"
    return "unknown"


# Complete frame lengths (ID .. EOF) from GROUND_STATION_PROTOCOL.md.
FRAME_LENGTHS = {0x01: 23, 0x02: 65, 0x03: 19, 0xE0: 103, 0xE1: 14, 0xFF: 8}


def frame_counts(data: bytes) -> dict:
    """Frames that start right after an EOF (0x0A) and end with one at their
    known length, by ID (IDs inside payloads are not counted)."""
    counts: dict = {}
    for i in range(len(data) - 1):
        if data[i] != 0x0A:
            continue
        fid = data[i + 1]
        n = FRAME_LENGTHS.get(fid)
        if n and i + n < len(data) and data[i + n] == 0x0A:
            counts[fid] = counts.get(fid, 0) + 1
    return counts


def listen(host: str, port: int, seconds: float) -> bytes:
    buf = bytearray()
    try:
        sock = socket.create_connection((host, port), timeout=4)
    except OSError:
        return b""
    sock.settimeout(0.3)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                break
            buf.extend(data)
    finally:
        sock.close()
    return bytes(buf)


def identify(host: str = DEFAULT_HOST, ports=RAW_PORTS, seconds: float = 6.0) -> dict:
    """{raw port: role} for every port (one port at a time: ser2net serves
    one client per port)."""
    return {port: classify(listen(host, port, seconds)) for port in ports}


def witness_port(host: str = DEFAULT_HOST, ports=RAW_PORTS, seconds: float = 6.0) -> int:
    """The raw port of the only board that looks like Witness."""
    roles = identify(host, ports, seconds)
    found = [p for p, role in roles.items() if role == "witness"]
    if len(found) != 1:
        raise LookupError(f"expected one Witness on {host}, found {found or 'none'}: {roles}")
    return found[0]


def rfc2217_port(raw_port: int) -> int:
    """The bench maps rawTTYACMn (71xx) and ttyACMn RFC2217 (70xx) pairwise."""
    return raw_port - 100


if __name__ == "__main__":
    print(identify(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST))
