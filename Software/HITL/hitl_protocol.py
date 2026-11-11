"""Witness wire protocol for the HITL tool (stdlib only).

Frames are ``packet ID | fixed-length payload | 0x0A``, big-endian. 0x0A is not
escaped, so frames are located by fixed length per ID with the EOF byte checked
only at the final position; on mismatch the decoder resynchronises one byte at
a time. The Witness USB stream also carries ESP-IDF log lines, raw GPS NMEA and
an echo of everything the host sends; the decoder yields text lines separately
and marks echoed 0x0C HITL frames so callers can ignore them.

The authoritative layout lives in Software/Witness/softwarev1/main/comms.h.
This tool may only ever transmit HITL ENTER, HITL EXIT, Witness debug start
(0x00E0, 10 Hz 0xE0 packets for attitude verification) and 0x0C sensor
frames.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field

EOF = 0x0A

ID_SENSORS = 0x01
ID_STATE = 0x02
ID_CAMERA = 0x03
ID_COMMAND = 0x05
ID_HITL = 0x0C
ID_HITL_IMU = 0x0E   # not 0x0D: that is CR, part of every text line
IMU_BATCH_SAMPLES = 8
ID_WITNESS_DEBUG = 0xE0
ID_IRIS_DEBUG = 0xE1
ID_HEARTBEAT = 0xFF

FRAME_LENGTHS = {
    ID_SENSORS: 23,
    ID_STATE: 65,
    ID_CAMERA: 19,
    ID_COMMAND: 10,
    ID_HITL: 47,
    ID_HITL_IMU: 150,   # only seen as an echo (the fake echoes everything)
    ID_WITNESS_DEBUG: 103,
    ID_IRIS_DEBUG: 14,
    ID_HEARTBEAT: 8,
}

PACKET_NAMES = {
    ID_SENSORS: "sensors",
    ID_STATE: "state",
    ID_CAMERA: "camera",
    ID_COMMAND: "command",
    ID_HITL: "hitl_echo",
    ID_HITL_IMU: "hitl_imu_echo",
    ID_WITNESS_DEBUG: "witness_debug",
    ID_IRIS_DEBUG: "iris_debug",
    ID_HEARTBEAT: "heartbeat",
}

# Packets whose bytes 1-2 are the Witness status field.
WITNESS_STATUS_PACKETS = (ID_SENSORS, ID_STATE, ID_COMMAND, ID_WITNESS_DEBUG,
                          ID_HEARTBEAT)
# Packets whose status is produced by Witness itself (not sender-supplied), so
# they are trusted for flight-state tracking.
TELEMETRY_STATUS_PACKETS = (ID_SENSORS, ID_STATE, ID_WITNESS_DEBUG,
                            ID_HEARTBEAT)

CMD_HITL_ENTER = 0x4849
CMD_HITL_EXIT = 0x4858
CMD_WITNESS_DEBUG_START = 0x00E0
# GPS receiver cold start (Witness: Pad Idle only; see comms.h).
CMD_GPS_COLD_START = 0x4752
# Safety: the only commands this tool will ever encode. Mass storage (0x6868),
# log erase (0x6869), log clear (0x686A) and flight-state commands (0x7300-0x7304) are
# deliberately impossible to build here.
# Export logs as a USB disk. Deliberately NOT in ALLOWED_COMMANDS: only the
# end-of-run FC log fetch sends it (hitl_outputs.fetch_fc_logs).
CMD_MASS_STORAGE = 0x6868
IMU_FRAME_LENGTH = 150
ALLOWED_COMMANDS = {CMD_HITL_ENTER: "HITL ENTER", CMD_HITL_EXIT: "HITL EXIT",
                    CMD_WITNESS_DEBUG_START: "WITNESS DEBUG START",
                    CMD_GPS_COLD_START: "GPS COLD START"}

FLIGHT_STATES = ("Pad idle", "Boost", "Coast", "Descent", "Landed")
PAD_IDLE, BOOST, COAST, DESCENT, LANDED = range(5)

STATUS0_BITS = (
    ("heartbeat", 0x01),
    ("low_g_ready", 0x02),
    ("gyro_ready", 0x04),
    ("imu_temp_ready", 0x08),
    ("nand_init_failed", 0x10),
    ("high_g_ready", 0x20),
    ("baro_ready", 0x40),
    ("flash_log_failed", 0x80),
)
STATUS1_GPS_LOCKED = 0x08
STATUS1_HITL_ACTIVE = 0x10
STATUS1_RESERVED = 0xE0

# Scale factors.
FILTERED_ACCEL_G_PER_COUNT = 0.000040
LOW_G_PER_COUNT = 0.000488
HIGH_G_PER_COUNT = 0.010417
GYRO_DPS_PER_COUNT = 0.070
LATLON_DEG_PER_COUNT = 0.0001
SEA_LEVEL_PRESSURE_MBAR = 1013.25

INT16_MIN, INT16_MAX = -32768, 32767
INT24_MIN, INT24_MAX = -(1 << 23), (1 << 23) - 1


# ---------------------------------------------------------------------------
# Primitive helpers
# ---------------------------------------------------------------------------

def clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def to_counts16(value: float, scale: float) -> int:
    """Engineering value -> saturated int16 count (round half away from 0)."""
    if math.isnan(value):
        return 0
    raw = value / scale
    raw = math.floor(raw + 0.5) if raw >= 0 else -math.floor(-raw + 0.5)
    return int(clamp(raw, INT16_MIN, INT16_MAX))


def i24_bytes(value: int) -> bytes:
    value = int(clamp(int(value), INT24_MIN, INT24_MAX))
    return (value & 0xFFFFFF).to_bytes(3, "big")


def u24_bytes(value: int) -> bytes:
    return int(clamp(int(value), 0, 0xFFFFFF)).to_bytes(3, "big")


def i24(data: bytes, offset: int) -> int:
    value = int.from_bytes(data[offset:offset + 3], "big")
    return value - (1 << 24) if value & 0x800000 else value


def u24(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 3], "big")


def i16(data: bytes, offset: int) -> int:
    return struct.unpack_from(">h", data, offset)[0]


def u16(data: bytes, offset: int) -> int:
    return struct.unpack_from(">H", data, offset)[0]


def u32(data: bytes, offset: int) -> int:
    return struct.unpack_from(">I", data, offset)[0]


def f32(data: bytes, offset: int) -> float:
    return struct.unpack_from(">f", data, offset)[0]


_ISA_ALT_LAYERS = (  # base geopotential m, base K, lapse K/m, base Pa
    (0.0, 288.15, -0.0065, 101325.0), (11000.0, 216.65, 0.0, 22632.06),
    (20000.0, 216.65, 0.001, 5474.889), (32000.0, 228.65, 0.0028, 868.0187),
    (47000.0, 270.65, 0.0, 110.9063))


def pressure_altitude_m(pressure_pa: float) -> float:
    """Barometric altitude exactly as the Witness MS5607 path computes it.

    Mirrors pressure_altitude.h: ISA layers to 47 km referenced to
    1013.25 mbar, converted from geopotential to geometric altitude.
    """
    mbar = pressure_pa / 100.0
    if mbar <= 0:
        return float("nan")
    pa = mbar * (101325.0 / (SEA_LEVEL_PRESSURE_MBAR * 100.0)) * 100.0
    i = 0
    while i + 1 < len(_ISA_ALT_LAYERS) and pa < _ISA_ALT_LAYERS[i + 1][3]:
        i += 1
    hb, tb, lapse, pb = _ISA_ALT_LAYERS[i]
    k = 287.05287 / 9.80665
    if lapse == 0.0:
        h = hb - k * tb * math.log(pa / pb)
    else:
        h = hb + tb / lapse * ((pa / pb) ** (-lapse * k) - 1.0)
    r = 6356766.0
    return r * h / (r - h)


def latlon_count(deg: float) -> int:
    return int(clamp(round(deg / LATLON_DEG_PER_COUNT), INT24_MIN, INT24_MAX))


# ---------------------------------------------------------------------------
# Encoders (host -> Witness)
# ---------------------------------------------------------------------------

def encode_command(command: int) -> bytes:
    """Canonical 10-byte command frame; only ALLOWED_COMMANDS are permitted."""
    if command not in ALLOWED_COMMANDS:
        raise ValueError(
            f"command 0x{command:04X} is not permitted by the HITL tool")
    return bytes((ID_COMMAND, 0, 0, 0, 0, 0, 0)) + struct.pack(
        ">H", command) + bytes((EOF,))


@dataclass
class HitlInput:
    """One HITL sensor sample in engineering units."""
    sim_time_ms: int = 0
    low_g: tuple = (1.0, 0.0, 0.0)        # g, specific force
    high_g: tuple = (1.0, 0.0, 0.0)       # g, specific force
    gyro_dps: tuple = (0.0, 0.0, 0.0)
    imu_temp_c: float = 25.0
    pressure_pa: float = 101325.0
    baro_temp_c: float = 25.0
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    gps_alt_m: float = 0.0
    satellites: int = 0
    fix_quality: int = 0
    battery_mv: int = 0


def encode_imu_batch(first_index: int, low_g, high_g, gyro) -> bytes:
    """Build a 150-byte 0x0E frame from eight samples of native counts.

    low_g, high_g, gyro: 8 x 3 int16 counts (numpy arrays or sequences).
    """
    out = bytearray((ID_HITL_IMU,))
    out += struct.pack(">I", int(first_index) & 0xFFFFFFFF)
    for k in range(IMU_BATCH_SAMPLES):
        for block in (low_g, high_g, gyro):
            for axis in range(3):
                out += struct.pack(">h", int(block[k][axis]))
    out.append(EOF)
    assert len(out) == IMU_FRAME_LENGTH == FRAME_LENGTHS[ID_HITL_IMU]
    return bytes(out)


# 0x0C status flag (comms.h IRIS_HITL_STATUS_GPS_FROM_RECEIVER): Witness takes
# GPS from its own receiver, which hears an RF simulation (gps_rf.py).
HITL_STATUS_GPS_FROM_RECEIVER = 0x01

# GPS assistance upload frames (Horizon/gps_assist.py; comms.h
# IRIS_COMMAND_GPS_ASSIST_*). The only other 0x05 frames this tool sends.
ASSIST_COMMANDS = {0x4741: "GPS ASSIST BEGIN", 0x4744: "GPS ASSIST DATA",
                   0x4745: "GPS ASSIST COMMIT", 0x4751: "GPS ASSIST STATUS"}


def is_assist_frame(frame: bytes) -> bool:
    """A canonical 10-byte 0x05 frame carrying a GPS assistance opcode."""
    return (len(frame) == 10 and frame[0] == ID_COMMAND and frame[9] == EOF
            and ((frame[7] << 8) | frame[8]) in ASSIST_COMMANDS)


def encode_hitl(sample: HitlInput, status: int = 0) -> bytes:
    """Build the 47-byte 0x0C frame. status: 0, or HITL_STATUS_GPS_FROM_RECEIVER."""
    if status not in (0, HITL_STATUS_GPS_FROM_RECEIVER):
        raise ValueError(f"unsupported 0x0C status 0x{status:02X}")
    out = bytearray((ID_HITL, status, 0))
    out += struct.pack(">I", int(sample.sim_time_ms) & 0xFFFFFFFF)
    for v in sample.low_g:
        out += struct.pack(">h", to_counts16(v, LOW_G_PER_COUNT))
    for v in sample.high_g:
        out += struct.pack(">h", to_counts16(v, HIGH_G_PER_COUNT))
    for v in sample.gyro_dps:
        out += struct.pack(">h", to_counts16(v, GYRO_DPS_PER_COUNT))
    out += struct.pack(">h", to_counts16(sample.imu_temp_c - 25.0, 1 / 256.0))
    out += u24_bytes(round(sample.pressure_pa))
    out += struct.pack(">h", to_counts16(sample.baro_temp_c, 0.01))
    out += i24_bytes(latlon_count(sample.lat_deg))
    out += i24_bytes(latlon_count(sample.lon_deg))
    out += struct.pack(">f", float(sample.gps_alt_m))
    out.append(int(clamp(int(sample.satellites), 0, 255)))
    out.append(int(clamp(int(sample.fix_quality), 0, 255)))
    out += struct.pack(">H", int(clamp(round(sample.battery_mv), 0, 0xFFFF)))
    out.append(EOF)
    assert len(out) == FRAME_LENGTHS[ID_HITL]
    return bytes(out)


# ---------------------------------------------------------------------------
# Decoders (Witness -> host)
# ---------------------------------------------------------------------------

def decode_status(b0: int, b1: int) -> dict:
    status = {name: bool(b0 & mask) for name, mask in STATUS0_BITS}
    state = b1 & 0x07
    status["flight_state"] = state
    status["flight_state_name"] = (FLIGHT_STATES[state] if state < 5
                                   else f"invalid({state})")
    status["gps_locked"] = bool(b1 & STATUS1_GPS_LOCKED)
    status["hitl_active"] = bool(b1 & STATUS1_HITL_ACTIVE)
    status["raw"] = (b0 << 8) | b1
    return status


def _status_plausible(b1: int) -> bool:
    return (b1 & STATUS1_RESERVED) == 0 and (b1 & 0x07) <= LANDED


def decode_hitl(frame: bytes) -> dict:
    """Decode a 0x0C frame back to engineering units (for echo/fake/tests)."""
    p = 1
    return {
        "sim_time_ms": u32(frame, p + 2),
        "low_g": [i16(frame, p + o) * LOW_G_PER_COUNT for o in (6, 8, 10)],
        "high_g": [i16(frame, p + o) * HIGH_G_PER_COUNT for o in (12, 14, 16)],
        "gyro_dps": [i16(frame, p + o) * GYRO_DPS_PER_COUNT
                     for o in (18, 20, 22)],
        "imu_temp_c": 25.0 + i16(frame, p + 24) / 256.0,
        "pressure_pa": u24(frame, p + 26),
        "baro_temp_c": i16(frame, p + 29) / 100.0,
        "lat_deg": i24(frame, p + 31) * LATLON_DEG_PER_COUNT,
        "lon_deg": i24(frame, p + 34) * LATLON_DEG_PER_COUNT,
        "gps_alt_m": f32(frame, p + 37),
        "satellites": frame[p + 41],
        "fix_quality": frame[p + 42],
        "battery_mv": u16(frame, p + 43),
    }


def decode_frame(frame: bytes) -> dict | None:
    """Decode one complete frame; None if implausible (caller resyncs)."""
    pid = frame[0]
    if len(frame) != FRAME_LENGTHS.get(pid, -1) or frame[-1] != EOF:
        return None
    pkt = {"id": pid, "name": PACKET_NAMES[pid]}
    if pid in TELEMETRY_STATUS_PACKETS or pid == ID_CAMERA:
        if not _status_plausible(frame[2]):
            return None
    if pid in WITNESS_STATUS_PACKETS or pid == ID_CAMERA:
        pkt["status"] = decode_status(frame[1], frame[2])
        pkt["uptime_ms"] = u32(frame, 3)

    if pid == ID_SENSORS:
        pkt["accel_g"] = [i24(frame, o) * FILTERED_ACCEL_G_PER_COUNT
                          for o in (7, 10, 13)]
        pkt["gyro_dps"] = [i16(frame, o) * GYRO_DPS_PER_COUNT
                           for o in (16, 18, 20)]
    elif pid == ID_STATE:
        pkt["quat_xyzw"] = [f32(frame, o) for o in (7, 11, 15, 19)]
        pkt["position_m"] = [f32(frame, o) for o in (23, 27, 31)]
        pkt["velocity_mps"] = [f32(frame, o) for o in (35, 39, 43)]
        pkt["lat_deg"] = i24(frame, 47) * LATLON_DEG_PER_COUNT
        pkt["lon_deg"] = i24(frame, 50) * LATLON_DEG_PER_COUNT
        pkt["gps_alt_m"] = f32(frame, 53)
        pkt["kalman_alt_m"] = f32(frame, 57)  # Kalman altitude above the pad
        pkt["battery_mv"] = u16(frame, 61)
        pkt["gps_satellites"] = frame[63]
    elif pid == ID_CAMERA:
        pkt["iris_status_raw"] = u16(frame, 7)
        pkt["iris_uptime_ms"] = u32(frame, 9)
        pkt["current_a"] = [frame[o] * 0.02 for o in (13, 14, 15)]
        pkt["iris_battery_mv"] = u16(frame, 16)
    elif pid == ID_COMMAND:
        pkt["command"] = u16(frame, 7)
        # Host-built frames (ours) have zero status and uptime; Witness echoes
        # them verbatim.
        pkt["probable_echo"] = frame[1:7] == b"\x00" * 6
    elif pid == ID_WITNESS_DEBUG:
        pkt["mcu_temp_c"] = struct.unpack_from(">b", frame, 7)[0]
        pkt["battery_mv"] = u16(frame, 8)
        pkt["accel_g"] = [i24(frame, o) * FILTERED_ACCEL_G_PER_COUNT
                          for o in (10, 13, 16)]
        pkt["quat_xyzw"] = [f32(frame, o) for o in (19, 23, 27, 31)]
        pkt["position_m"] = [f32(frame, o) for o in (35, 39, 43)]
        pkt["velocity_mps"] = [f32(frame, o) for o in (47, 51, 55)]
        pkt["low_g"] = [i16(frame, o) * LOW_G_PER_COUNT for o in (59, 61, 63)]
        pkt["imu_temp_c"] = 25.0 + i16(frame, 65) / 256.0
        pkt["high_g"] = [i16(frame, o) * HIGH_G_PER_COUNT
                         for o in (67, 69, 71)]
        pkt["gyro_dps"] = [i16(frame, o) * GYRO_DPS_PER_COUNT
                           for o in (73, 75, 77)]
        pkt["pressure_pa"] = u24(frame, 79)
        pkt["baro_temp_c"] = i16(frame, 82) / 100.0
        pkt["kalman_alt_m"] = f32(frame, 84)  # Kalman altitude above the pad
        pkt["baro_alt_m"] = (pressure_altitude_m(pkt["pressure_pa"])
                             if pkt["pressure_pa"] > 0 else None)
        pkt["lat_deg"] = i24(frame, 88) * LATLON_DEG_PER_COUNT
        pkt["lon_deg"] = i24(frame, 91) * LATLON_DEG_PER_COUNT
        pkt["gps_time_s"] = u32(frame, 94)
        pkt["gps_alt_m"] = f32(frame, 98)
    elif pid == ID_IRIS_DEBUG:
        pkt["iris_status_raw"] = u16(frame, 1)
        pkt["uptime_ms"] = u32(frame, 3)
        pkt["mcu_temp_c"] = struct.unpack_from(">b", frame, 7)[0]
        pkt["battery_mv"] = u16(frame, 8)
        pkt["current_a"] = [frame[o] * 0.02 for o in (10, 11, 12)]
    elif pid == ID_HITL:
        pkt.update(decode_hitl(frame))
    elif pid == ID_HEARTBEAT:
        pass
    return pkt


_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_TEXT_BYTES = frozenset(range(0x20, 0x7F)) | {0x09, 0x0D, 0x1B}
MAX_TEXT_LINE = 1024


@dataclass
class DecoderStats:
    bytes_in: int = 0
    frames: int = 0
    echo_frames: int = 0
    text_lines: int = 0
    junk_bytes: int = 0
    per_id: dict = field(default_factory=dict)


class StreamDecoder:
    """Resynchronising decoder for the mixed Witness USB byte stream.

    ``feed(data)`` returns a list of ``(kind, value)`` events where kind is
    ``"packet"`` (decoded dict) or ``"text"`` (one log/NMEA line, ANSI colour
    codes and CR stripped). Echoed 0x0C frames are returned as packets named
    ``hitl_echo`` with ``echo=True``; consumers should ignore them.
    """

    def __init__(self):
        self._buf = bytearray()
        self._text = bytearray()
        self.stats = DecoderStats()

    def feed(self, data: bytes) -> list:
        self.stats.bytes_in += len(data)
        self._buf += data
        events = []
        buf = self._buf
        i = 0
        n = len(buf)
        while i < n:
            b = buf[i]
            length = FRAME_LENGTHS.get(b)
            if length is not None:
                if n - i < length:
                    break  # wait for the rest of the candidate frame
                if buf[i + length - 1] == EOF:
                    pkt = decode_frame(bytes(buf[i:i + length]))
                    if pkt is not None:
                        if b == ID_HITL:
                            pkt["echo"] = True
                            self.stats.echo_frames += 1
                        self.stats.frames += 1
                        self.stats.per_id[b] = self.stats.per_id.get(b, 0) + 1
                        events.append(("packet", pkt))
                        i += length
                        continue
                self.stats.junk_bytes += 1
                i += 1
                continue
            if b == 0x0A:
                line = self._finish_text()
                if line is not None:
                    events.append(("text", line))
            elif b in _TEXT_BYTES:
                self._text.append(b)
                if len(self._text) >= MAX_TEXT_LINE:
                    line = self._finish_text()
                    if line is not None:
                        events.append(("text", line))
            else:
                self.stats.junk_bytes += 1
            i += 1
        del buf[:i]
        return events

    def _finish_text(self):
        raw = bytes(self._text)
        self._text.clear()
        line = _ANSI_RE.sub("", raw.decode("ascii", "replace")).strip("\r")
        line = line.rstrip()
        if not line:
            return None
        self.stats.text_lines += 1
        return line


# ESP-IDF flight-state log line, e.g. "I (1234) flight_state: pad idle -> boost"
_STATE_LOG_RE = re.compile(
    r"flight_state:\s*(pad idle|boost|coast|descent|landed)\s*->\s*"
    r"(pad idle|boost|coast|descent|landed)", re.IGNORECASE)
_LOG_STATE_NAMES = {"pad idle": PAD_IDLE, "boost": BOOST, "coast": COAST,
                    "descent": DESCENT, "landed": LANDED}


def parse_state_log(line: str):
    """Return (from_state, to_state) for a flight_state transition log line."""
    m = _STATE_LOG_RE.search(line)
    if not m:
        return None
    return (_LOG_STATE_NAMES[m.group(1).lower()],
            _LOG_STATE_NAMES[m.group(2).lower()])


def is_nmea(line: str) -> bool:
    return line.startswith("$") or line.startswith("!")
