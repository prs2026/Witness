"""Offline test bytes for main.py; this module never opens a device or socket.

JSON replay format (times are relative to the start of each playback):
    {"version": 1, "duration_ms": 2000, "frames": [
        {"at_ms": 0, "hex": "ff 00 00 00 00 00 00 0a"},
        {"at_ms": 1000, "hex": "ff 01 01 00 00 03 e8 0a"}
    ]}

Each hex entry may be a complete packet or an arbitrary stream fragment,
including deliberately malformed bytes. Payloads use comms.h wire encodings.
Duration must be later than the last entry; it controls the loop boundary.
Device uptimes in a file are replayed unchanged (a loop resembles a reboot).
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import time


@dataclass(frozen=True)
class TestFrame:
    at_ms: int
    data: bytes


@dataclass(frozen=True)
class TestScenario:
    name: str
    duration_ms: int
    frames: tuple[TestFrame, ...]


def load_scenario(filename):
    path = Path(filename)
    with path.open("rb") as stream:
        raw = stream.read(4 * 1024 * 1024 + 1)
    if len(raw) > 4 * 1024 * 1024:
        raise ValueError("Test source exceeds 4 MiB")
    try:
        document = json.loads(raw)
    except (ValueError, UnicodeDecodeError, RecursionError) as exc:
        raise ValueError(f"Test source is not valid JSON: {exc}") from exc
    if not isinstance(document, dict) or type(document.get("version")) is not int or document["version"] != 1:
        raise ValueError("Test source must contain version: 1")
    duration = document.get("duration_ms")
    if type(duration) is not int or not 1 <= duration <= 86400000:
        raise ValueError("duration_ms must be an integer between 1 and 86400000")
    rows = document.get("frames")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 10000:
        raise ValueError("Test source must contain 1 to 10000 frames")
    frames = []
    previous = -1
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"Frame {index}: expected an object")
        at_ms, hex_data = row.get("at_ms"), row.get("hex")
        if type(at_ms) is not int or not previous <= at_ms < duration or at_ms < 0:
            raise ValueError(f"Frame {index}: at_ms must be ordered and less than duration_ms")
        if not isinstance(hex_data, str):
            raise ValueError(f"Frame {index}: hex must be a string")
        try:
            data = bytes.fromhex(hex_data)
        except ValueError as exc:
            raise ValueError(f"Frame {index}: invalid hex bytes") from exc
        if not 1 <= len(data) <= 4096:
            raise ValueError(f"Frame {index}: expected 1 to 4096 bytes")
        frames.append(TestFrame(at_ms, data))
        previous = at_ms
    return TestScenario(path.stem, duration, tuple(frames))


def validate_speed(speed):
    if not math.isfinite(speed) or not 0.1 <= speed <= 10:
        raise ValueError("Test playback speed must be between 0.1 and 10")
    return speed


def play_scenario(scenario, on_bytes, stop_event, *, speed=1.0, loop=True,
                  on_cycle=None, clock=time.monotonic):
    """Play on a worker thread; Event.wait makes even long gaps cancellable."""
    validate_speed(speed)
    while not stop_event.is_set():
        start = clock()
        if on_cycle is not None:
            on_cycle()  # Reset deframer state; never splice a partial frame across loops.
        for frame in scenario.frames:
            delay = max(0, start + frame.at_ms / (1000 * speed) - clock())
            if stop_event.wait(delay):
                return
            on_bytes(frame.data)
            if stop_event.is_set():
                return
        if stop_event.wait(max(0, start + scenario.duration_ms / (1000 * speed) - clock())):
            return
        if not loop:
            return


def _integer(value, size, signed=False):
    return int(round(value)).to_bytes(size, "big", signed=signed)


def _vector(values, scale, size):
    return b"".join(_integer(value / scale, size, signed=True) for value in values)


def demo_packets(at_ms):
    """One illustrative flight sample, encoded as seven actual telemetry frames.

    This is a UI exercise, not a physically accurate flight dynamics model.
    Status/flight-state meanings follow GROUND_STATION_PROTOCOL.md and the UI.
    """
    seconds = at_ms / 1000
    if seconds < 5:
        state, altitude, velocity, acceleration = 0, 0, 0, 1
    elif seconds < 10:
        elapsed = seconds - 5
        state, altitude, velocity, acceleration = 1, 12 * elapsed ** 2, 24 * elapsed, 3.45
    elif seconds < 20:
        elapsed = seconds - 10
        state, altitude, velocity, acceleration = 2, 300 + 120 * elapsed - 6 * elapsed ** 2, 120 - 12 * elapsed, 0.05
    elif seconds < 50:
        state, altitude, velocity, acceleration = 3, 900 - 30 * (seconds - 20), -30, 1.1
    else:
        state, altitude, velocity, acceleration = 4, 0, 0, 1
    flags = 0x6E | (int(seconds) % 2)
    header = _integer((flags << 8) | state, 2) + _integer(at_ms, 4)
    accel = (0.05 * math.sin(seconds), 0.04 * math.cos(seconds), acceleration)
    accel_bytes = _vector(accel, 0.000040, 3)
    gyro = _vector((3 * math.sin(seconds), -2 * math.cos(seconds), 0.5), 0.070, 2)
    angle = 0.05 * math.sin(seconds / 3)
    orientation = struct.pack(">4f", 0, 0, math.sin(angle / 2), math.cos(angle / 2))
    position = struct.pack(">3f", seconds * 0.5, seconds * 0.25, altitude)
    velocity_bytes = struct.pack(">3f", 0.5, 0.25, velocity)
    latitude = _integer((34.0 + seconds * 0.00001) * 10000, 3, signed=True)
    longitude = _integer((-117.0 + seconds * 0.00001) * 10000, 3, signed=True)
    gps_altitude = struct.pack(">f", altitude + 100)
    barometric_altitude = struct.pack(">f", altitude)
    battery = _integer((4.2 - seconds * 0.002) * 1000, 2)
    iris_header = _integer(0x7800, 2) + _integer(at_ms, 4)
    currents = bytes((5 + int(seconds) % 3, 8, 12))
    iris_battery = _integer((7.8 - seconds * 0.003) * 1000, 2)
    debug = (header + _integer(30 + seconds / 10, 1, signed=True) + battery
             + accel_bytes + orientation + position + velocity_bytes
             + _vector(accel, 0.000488, 2) + _integer(2 * 256, 2, signed=True)
             + _vector(accel, 0.010417, 2) + gyro
             + _integer(101325 * math.exp(-altitude / 8400), 3)
             + _integer(2150, 2, signed=True) + barometric_altitude
             + latitude + longitude + _integer(1791849600 + int(seconds), 4) + gps_altitude)
    payloads = {
        0x01: header + accel_bytes + gyro,
        0x02: header + orientation + position + velocity_bytes + latitude + longitude
              + gps_altitude + barometric_altitude + battery,
        0x03: header + iris_header + currents + iris_battery,
        0xFF: header,
        0xF0: _integer(-65 - 10 * math.sin(seconds / 5), 1, signed=True) + _integer(at_ms, 4) + bytes(6),
        0xE0: debug,
        0xE1: iris_header + _integer(29, 1, signed=True) + iris_battery + currents,
    }
    return {packet_id: bytes((packet_id,)) + payload + b"\x0a"
            for packet_id, payload in payloads.items()}


def builtin_scenario():
    frames = []
    for at_ms in range(0, 60000, 100):
        packets = demo_packets(at_ms)
        frames.append(TestFrame(at_ms, packets.pop(0x01)))  # Sensors at 10 Hz.
        if at_ms % 1000 == 0:
            frames.extend(TestFrame(at_ms, data) for data in packets.values())
    return TestScenario("demo_flight", 60000, tuple(frames))
