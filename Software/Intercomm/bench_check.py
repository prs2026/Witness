"""Capture the three bench USB TCP streams and optionally ping Iris via Aperture.

Only the harmless Iris ping command is transmitted. Raw captures remain available
for protocol diagnosis; the mixed-stream parser may have incidental false matches.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import socket
import time

LENGTHS = {1: 23, 2: 65, 3: 19, 5: 10, 0xE0: 103, 0xE1: 14, 0xFF: 8, 0xF0: 13}
PORTS = {"iris": 7101, "aperture": 7100, "witness": 7102}


def capture(host, board, port, duration, output, ping):
    counts = Counter()
    last = {}
    buffer = bytearray()
    total = 0
    pongs = 0
    result = {"board": board, "port": port}
    try:
        with socket.create_connection((host, port), timeout=5) as connection:
            connection.settimeout(0.25)
            start = time.monotonic()
            next_ping = start + 2
            with (output / f"{board}.bin").open("wb") as raw:
                while time.monotonic() - start < duration:
                    now = time.monotonic()
                    if ping and board == "aperture" and now >= next_ping:
                        connection.sendall(bytes.fromhex("05 00 00 00 00 00 00 00 22 0A"))
                        next_ping = now + 3
                    try:
                        data = connection.recv(16384)
                    except socket.timeout:
                        continue
                    if not data:
                        result["closed"] = True
                        break
                    raw.write(data)
                    total += len(data)
                    buffer.extend(data)
                    while buffer:
                        length = LENGTHS.get(buffer[0])
                        if length is None:
                            del buffer[0]
                            continue
                        if len(buffer) < length:
                            break
                        if buffer[length - 1] != 10:
                            del buffer[0]
                            continue
                        frame = bytes(buffer[:length])
                        del buffer[:length]
                        packet_id = frame[0]
                        counts[f"{packet_id:02X}"] += 1
                        last[f"{packet_id:02X}"] = frame.hex(" ")
                        if packet_id == 5 and frame[7:9] == b"\x00\x23":
                            pongs += 1
        result.update(bytes=total, packets=dict(counts), iris_pongs=pongs, last=last)
    except OSError as error:
        result["error"] = str(error)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="prs2026hitl")
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--ping", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or Path(__file__).parent / "captures" / datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = [pool.submit(capture, args.host, board, port, args.seconds, output, args.ping)
                   for board, port in PORTS.items()]
        results = [future.result() for future in futures]
    report = {"host": args.host, "ping_requested": args.ping, "results": results}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Capture directory: {output.resolve()}")
    return 1 if any("error" in item for item in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
