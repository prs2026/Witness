"""Guided Witness IMU calibration over the USB link (TCP bench port or COM port).

Usage:
    python imu_calibrate.py                      # prs2026hitl:7100 (bench Pi)
    python imu_calibrate.py --tcp HOST:PORT
    python imu_calibrate.py --serial COM5        # needs pyserial
    python imu_calibrate.py --gyro-only | --status | --clear

Witness must be in Pad Idle with real sensors (not HITL), and nothing else may
hold the port (close Horizon first). The script sends 05 43 xx commands and
prints Witness's `sensors` / `imu_cal` log lines. Calibration is stored in
Witness NVS and survives reboots.

Steps: gyro bias (4 s, board still in any orientation), then the six faces
(2 s each). Put the board flat with each axis pointing up and then down; the
order does not matter and faces need not be perfectly square (within about
14 degrees), because the solver fits every face to exactly 1 g.
"""
import argparse
import re
import socket
import sys
import time

GYRO = bytes([0x05, 0x43, 0x47])
FACE = bytes([0x05, 0x43, 0x41])
CLEAR = bytes([0x05, 0x43, 0x58])
STATUS = bytes([0x05, 0x43, 0x53])
LOG_LINE = re.compile(rb"[IWE] \(\d+\) (?:sensors|imu_cal): [^\r\n]*")
INTERESTING = re.compile(r"calibrat|face|NVS", re.I)


class Link:
    def __init__(self, args):
        self.buffer = bytearray()
        if args.serial:
            import serial  # pyserial
            self.port = serial.Serial(args.serial, 115200, timeout=0.2)
            self.read = lambda: self.port.read(4096)
            self.write = self.port.write
        else:
            host, port = args.tcp.rsplit(":", 1)
            self.sock = socket.create_connection((host, int(port)), timeout=5)
            self.sock.settimeout(0.2)
            self.write = self.sock.sendall

            def read():
                try:
                    return self.sock.recv(16384)
                except socket.timeout:
                    return b""
            self.read = read

    def wait_for(self, pattern, seconds):
        """Prints calibration log lines until one matches pattern; returns it."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.buffer += self.read()
            while True:
                match = LOG_LINE.search(self.buffer)
                if not match:
                    # Keep only a tail that could still hold a partial line.
                    del self.buffer[:max(0, len(self.buffer) - 512)]
                    break
                line = match.group().decode("ascii", "replace")
                del self.buffer[:match.end()]
                if INTERESTING.search(line):
                    print("   ", line)
                if pattern and re.search(pattern, line):
                    return line
        return None

    def command(self, data, pattern, seconds):
        self.buffer.clear()
        self.write(data)
        return self.wait_for(pattern, seconds)


def gyro(link):
    input("Gyro bias: leave the board still in any orientation, then press Enter.")
    line = link.command(GYRO, r"gyro calibration (done|failed)|rejected", 10)
    if line is None:
        print("No answer from Witness (is it in Pad Idle and the port free?)")
        return False
    return "done" in line


def faces(link):
    captured = set()
    while len(captured) < 6:
        missing = [f for f in ("+X", "-X", "+Y", "-Y", "+Z", "-Z") if f not in captured]
        input(f"Face {len(captured) + 1}/6: rest the board with one of {' '.join(missing)} "
              "pointing UP (that axis reads +1 g), keep it still, press Enter.")
        line = link.command(FACE, r"face \S+ captured|face capture failed|rejected", 8)
        if line is None:
            print("No answer from Witness; try again.")
            continue
        found = re.search(r"face (\S+) captured", line)
        if found:
            if found.group(1) in captured:
                print(f"{found.group(1)} was already captured (recaptured); turn the board.")
            captured.add(found.group(1))
    done = link.wait_for(r"accelerometer calibration (done|failed)", 5)
    return done is not None and "done" in done


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tcp", default="prs2026hitl:7100")
    parser.add_argument("--serial")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--gyro-only", action="store_true")
    group.add_argument("--status", action="store_true")
    group.add_argument("--clear", action="store_true")
    args = parser.parse_args()
    link = Link(args)
    time.sleep(0.5)

    if args.status:
        link.command(STATUS, r"faces captured", 5)
        return 0
    if args.clear:
        link.command(CLEAR, r"cleared|rejected", 5)
        link.wait_for(r"NVS", 3)
        return 0
    if not gyro(link):
        return 1
    if args.gyro_only:
        return 0
    if not faces(link):
        print("Accelerometer calibration did not complete; run the script again.")
        return 1
    link.command(STATUS, r"faces captured", 5)
    print("Calibration stored in Witness NVS.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
