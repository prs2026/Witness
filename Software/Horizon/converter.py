"""Convert Horizon binary telemetry logs to plotting-friendly CSV files.

Usage:
    python converter.py logs/serial_COM3_20260918_120000.bin
    python converter.py input.bin --output output.csv

The converter auto-detects both raw application-packet streams and the fixed
48-byte ``WL`` records exported by Witness flash mass storage. Neither format
contains host receive timestamps, so ``time_seconds`` is derived from device
uptime.

State packets (0x02) are 65-byte frames with ``gps_satellites``. Legacy 64-byte
state frames from older firmware (no satellite byte) are still accepted: the
65-byte form is tried first and the 64-byte form is used only when frame
offset 64 is not 0x0A (or the file ends). See ``frame_payload_length``.
Echoed host-to-Witness HITL input packets (0x0C, 47 bytes) are skipped.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from pathlib import Path
from typing import Any


EOF = 0x0A
PACKET_NAMES = {
    0x01: "sensors",
    0x02: "state",
    0x03: "camera",
    0x05: "command",
    0xFF: "heartbeat",
    0xE0: "witness_debug",
    0xE1: "iris_debug",
    0xF0: "ground_station_status",
    0xE2: "kalman_log",
}

# Host -> Witness HITL sensor input. Witness may echo it on USB; it is framed
# (fixed length + EOF) and skipped so it is never misparsed as telemetry.
HOST_PAYLOAD_LENGTHS = {
    0x0C: (45,),
}

# The current capture format uses a 23-byte sensor payload.  The 21-byte form
# declared by comms.h is also accepted for compatibility.
PAYLOAD_LENGTHS = {
    0x01: (23, 21),
    0x02: (63, 62),  # 65-byte frame with GPS satellites; legacy 64-byte frame
    0x03: (17,),
    0x05: (8,),
    0xFF: (6,),
    0xE0: (101,),
    0xE1: (12,),
    0xF0: (27, 11),  # 29-byte frame with Aperture GPS; legacy 13-byte frame
    0xE2: (101,),    # Kalman log, flash P*.BIN only
}

# Flash mass-storage .BIN files use fixed records rather than application
# packets. See FlashLogger::serialize_record in the Witness firmware.
FLASH_RECORD_MAGIC = b"WL"
FLASH_RECORD_VERSION = 1
FLASH_RECORD_SIZE = 48
# WL v2 (store v4 firmware, 100 Hz): status bytes 0-1, sequence, uptime, LSM6
# temperature, pressure, MS5607 temperature. IMU samples are in I*.BIN and
# baro altitude/battery in the P*.BIN state packets.
FLASH_RECORD_SIZES = {1: 48, 2: 24}
# Witness flash store v3 IMU stream (I*.BIN): 24-byte records, no magic.
# u32 timestamp_us (low 32 bits of esp_timer), int16 low-g xyz, int16 high-g
# xyz, int16 gyro xyz (native counts), u8 status, u8 sequence (wraps at 256).
IMU_RECORD_SIZE = 24
IMU_STATUS_STATE_MASK = 0x07
IMU_STATUS_HITL = 0x08
IMU_STATUS_LOW_G_FRESH = 0x10
IMU_STATUS_HIGH_G_FRESH = 0x20
IMU_STATUS_GYRO_FRESH = 0x40
IMU_STATUS_HIGH_G_ACTIVE = 0x80  # filtered accel taken from the high-g sensor

FILTERED_ACCEL_G_PER_COUNT = 0.000040
GYRO_DPS_PER_COUNT = 0.070
BATTERY_V_PER_COUNT = 0.001
CURRENT_A_PER_COUNT = 0.02
GPS_DEGREES_PER_COUNT = 0.0001
LSM6_TEMPERATURE_OFFSET_C = 25.0
LSM6_TEMPERATURE_C_PER_COUNT = 1.0 / 256.0
MS5607_TEMPERATURE_C_PER_COUNT = 0.01
LSM6_LOW_ACCEL_G_PER_COUNT = 0.000488
LSM6_HIGH_ACCEL_G_PER_COUNT = 0.010417


def unsigned(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=False)


def signed(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=True)


def signed_int24(data: bytes) -> int:
    value = unsigned(data)
    return value - (1 << 24) if value & (1 << 23) else value


def float32(data: bytes) -> float:
    return struct.unpack(">f", data)[0]


def float_vector(data: bytes, count: int) -> list[float]:
    return [float32(data[index:index + 4]) for index in range(0, count * 4, 4)]


GROUND_GPS_DEGREES_PER_COUNT = 1e-7


def parse_ground_station_status(payload: bytes) -> dict[str, Any]:
    """0xF0 from Aperture: RSSI, uptime and (27-byte form) Aperture's GPS.

    The legacy 11-byte payload has no GPS; its fields are reported as no fix.
    """
    packet: dict[str, Any] = {
        "packet": "ground_station_status",
        "rssi": signed(payload[0:1]),
        "uptime": unsigned(payload[1:5]),
        "gps_fix_quality": 0,
        "gps_satellites": 0,
        "latitude": None,
        "longitude": None,
        "gps_altitude": None,
        "gps_time": None,
        "gps_hdop": None,
        "gps_fix_age_ms": None,
    }
    if len(payload) == 27:
        hdop = unsigned(payload[23:25])
        age = unsigned(payload[25:27])
        has_position = age != 0xFFFF or unsigned(payload[7:15]) != 0
        packet.update({
            "gps_fix_quality": payload[5],
            "gps_satellites": payload[6],
            "latitude": signed(payload[7:11]) * GROUND_GPS_DEGREES_PER_COUNT if has_position else None,
            "longitude": signed(payload[11:15]) * GROUND_GPS_DEGREES_PER_COUNT if has_position else None,
            "gps_altitude": float32(payload[15:19]) if has_position else None,
            "gps_time": unsigned(payload[19:23]) or None,
            "gps_hdop": None if hdop == 0xFFFF else hdop / 100.0,
            "gps_fix_age_ms": None if age == 0xFFFF else age,
        })
    return packet


KALMAN_FLAGS = {
    "initialized": 0x01, "imu_ignored": 0x02, "pad_hold": 0x04, "baro_gated": 0x08,
    "baro_accepted": 0x10, "gps_accepted": 0x20, "gps_origin_set": 0x40, "hitl": 0x80,
}


def parse_kalman_log(payload: bytes) -> dict[str, Any]:
    """0xE2 Witness Kalman filter log (P*.BIN source 5), see comms.h."""
    flags = payload[1]
    packet: dict[str, Any] = {
        "packet": "kalman_log",
        "flight_state_code": payload[0],
        "kalman_flags": flags,
        **{f"kf_{name}": int(bool(flags & bit)) for name, bit in KALMAN_FLAGS.items()},
        "gps_satellites": payload[2],
        "position": float_vector(payload[3:15], 3),
        "velocity": float_vector(payload[15:27], 3),
        "position_variance": float_vector(payload[27:39], 3),
        "velocity_variance": float_vector(payload[39:51], 3),
        "position_velocity_covariance": float_vector(payload[51:63], 3),
        "baro_innovation": float32(payload[63:67]),
        "baro_innovation_variance": float32(payload[67:71]),
        "baro_sigma": float32(payload[71:75]),
        "gps_innovation": float_vector(payload[75:87], 3),
        "mach": float32(payload[87:91]),
        "baro_updates": unsigned(payload[91:93]),
        "baro_gated_count": unsigned(payload[93:95]),
        "baro_rejects": unsigned(payload[95:97]),
        "gps_updates": unsigned(payload[97:99]),
        "gps_rejects": unsigned(payload[99:101]),
    }
    # 1-sigma uncertainties (sqrt of the variances) for plotting.
    packet["position_sigma"] = [v ** 0.5 if v == v and v >= 0 else None
                                for v in packet["position_variance"]]
    packet["velocity_sigma"] = [v ** 0.5 if v == v and v >= 0 else None
                                for v in packet["velocity_variance"]]
    return packet


SEA_LEVEL_PRESSURE_PA = 101325.0
_ISA_LAYERS = (  # base geopotential m, base K, lapse K/m, base Pa
    (0.0, 288.15, -0.0065, 101325.0), (11000.0, 216.65, 0.0, 22632.06),
    (20000.0, 216.65, 0.001, 5474.889), (32000.0, 228.65, 0.0028, 868.0187),
    (47000.0, 270.65, 0.0, 110.9063))


def pressure_altitude_m(pressure_pa: float) -> float | None:
    """ISA barometric altitude (geometric m, 1013.25 mbar), as pressure_altitude.h."""
    if not pressure_pa or pressure_pa <= 0:
        return None
    i = 0
    while i + 1 < len(_ISA_LAYERS) and pressure_pa < _ISA_LAYERS[i + 1][3]:
        i += 1
    hb, tb, lapse, pb = _ISA_LAYERS[i]
    k = 287.05287 / 9.80665
    if lapse == 0.0:
        h = hb - k * tb * math.log(pressure_pa / pb)
    else:
        h = hb + tb / lapse * ((pressure_pa / pb) ** (-lapse * k) - 1.0)
    r = 6356766.0
    return r * h / (r - h)


def parse_packet(packet_id: int, payload: bytes) -> dict[str, Any] | None:
    """Decode one payload using the same field layout as main.py/comms.h."""
    if packet_id == 0x01 and len(payload) in (21, 23):
        packet: dict[str, Any] = {
            "packet": "sensors",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "filtered_accel": [
                signed_int24(payload[index:index + 3]) * FILTERED_ACCEL_G_PER_COUNT
                for index in (6, 9, 12)
            ],
            "gyro": [
                signed(payload[index:index + 2]) * GYRO_DPS_PER_COUNT
                for index in (15, 17, 19)
            ],
        }
        if len(payload) == 23:
            packet["sensor_trailing_u16"] = unsigned(payload[21:23])
        return packet

    if packet_id == 0x02 and len(payload) in (63, 62):
        packet = {
            "packet": "state",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "orientation": float_vector(payload[6:22], 4),
            "position": float_vector(payload[22:34], 3),
            "velocity": float_vector(payload[34:46], 3),
            "latitude": signed_int24(payload[46:49]) * GPS_DEGREES_PER_COUNT,
            "longitude": signed_int24(payload[49:52]) * GPS_DEGREES_PER_COUNT,
            "gps_altitude": float32(payload[52:56]),
            # Kalman altitude above the pad (raw baro MSL in pre-2026-11-10 logs).
            "kalman_altitude": float32(payload[56:60]),
            "witness_battery_voltage": unsigned(payload[60:62]) * BATTERY_V_PER_COUNT,
        }
        # Legacy 62-byte payloads leave gps_satellites blank in the CSV.
        if len(payload) == 63:
            packet["gps_satellites"] = payload[62]
        return packet

    if packet_id == 0x03 and len(payload) == 17:
        return {
            "packet": "camera",
            "fc_status": unsigned(payload[0:2]),
            "fc_uptime": unsigned(payload[2:6]),
            "status": unsigned(payload[6:8]),
            "uptime": unsigned(payload[8:12]),
            "current_sense": [payload[index] * CURRENT_A_PER_COUNT for index in (12, 13, 14)],
            "iris_battery_voltage": unsigned(payload[15:17]) * BATTERY_V_PER_COUNT,
        }

    if packet_id == 0x05 and len(payload) == 8:
        return {
            "packet": "command",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "command_value": unsigned(payload[6:8]),
        }

    if packet_id == 0xFF and len(payload) == 6:
        return {
            "packet": "heartbeat",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
        }

    if packet_id == 0xF0 and len(payload) in (11, 27):
        return parse_ground_station_status(payload)

    if packet_id == 0xE2 and len(payload) == 101:
        return parse_kalman_log(payload)

    if packet_id == 0xE0 and len(payload) == 101:
        return {
            "packet": "witness_debug",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "mcu_temperature": signed(payload[6:7]),
            "witness_battery_voltage": unsigned(payload[7:9]) * BATTERY_V_PER_COUNT,
            "filtered_accel": [
                signed_int24(payload[index:index + 3]) * FILTERED_ACCEL_G_PER_COUNT
                for index in (9, 12, 15)
            ],
            "orientation": float_vector(payload[18:34], 4),
            "position": float_vector(payload[34:46], 3),
            "velocity": float_vector(payload[46:58], 3),
            "low_accel": [
                signed(payload[index:index + 2]) * LSM6_LOW_ACCEL_G_PER_COUNT
                for index in (58, 60, 62)
            ],
            "lsm6_temperature": (
                LSM6_TEMPERATURE_OFFSET_C
                + signed(payload[64:66]) * LSM6_TEMPERATURE_C_PER_COUNT
            ),
            "high_accel": [
                signed(payload[index:index + 2]) * LSM6_HIGH_ACCEL_G_PER_COUNT
                for index in (66, 68, 70)
            ],
            "gyro": [
                signed(payload[index:index + 2]) * GYRO_DPS_PER_COUNT
                for index in (72, 74, 76)
            ],
            "pressure": unsigned(payload[78:81]),
            "ms5607_temperature": (
                signed(payload[81:83]) * MS5607_TEMPERATURE_C_PER_COUNT
            ),
            "kalman_altitude": float32(payload[83:87]),
            "latitude": signed_int24(payload[87:90]) * GPS_DEGREES_PER_COUNT,
            "longitude": signed_int24(payload[90:93]) * GPS_DEGREES_PER_COUNT,
            "gps_time": unsigned(payload[93:97]),
            "gps_altitude": float32(payload[97:101]),
        }

    if packet_id == 0xE1 and len(payload) == 12:
        return {
            "packet": "iris_debug",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "mcu_temperature": signed(payload[6:7]),
            "iris_battery_voltage": unsigned(payload[7:9]) * BATTERY_V_PER_COUNT,
            "current_sense": [payload[index] * CURRENT_A_PER_COUNT for index in (9, 10, 11)],
        }

    return None


VECTOR_LABELS = {
    "filtered_accel": ("x", "y", "z"),
    "gyro": ("x", "y", "z"),
    "orientation": ("x", "y", "z", "w"),
    "position": ("x", "y", "z"),
    "velocity": ("x", "y", "z"),
    "position_variance": ("x", "y", "z"),
    "velocity_variance": ("x", "y", "z"),
    "position_velocity_covariance": ("x", "y", "z"),
    "position_sigma": ("x", "y", "z"),
    "velocity_sigma": ("x", "y", "z"),
    "gps_innovation": ("x", "y", "z"),
    "low_accel": ("x", "y", "z"),
    "high_accel": ("x", "y", "z"),
    "current_sense": ("ch1", "ch2", "ch3"),
    "reserved": ("0", "1", "2", "3", "4", "5"),
}

FLIGHT_STATE_NAMES = {
    0: "Pad idle",
    1: "Boost",
    2: "Coast",
    3: "Descent",
    4: "Landed",
}

WITNESS_STATUS_PACKETS = {
    "sensors",
    "state",
    "command",
    "heartbeat",
    "witness_debug",
}


def clean_number(value: Any) -> Any:
    """Keep CSV numeric values finite so plotting programs can ingest them."""
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, float):
        return round(value, 9)
    return value


def flatten_packet(packet: dict[str, Any]) -> dict[str, Any]:
    flat: dict[str, Any] = {}
    for field, value in packet.items():
        if field == "packet":
            continue
        if isinstance(value, (list, tuple)):
            labels = VECTOR_LABELS.get(field)
            if labels is None or len(labels) != len(value):
                labels = tuple(str(index) for index in range(len(value)))
            for label, component in zip(labels, value):
                flat[f"{field}_{label}"] = clean_number(component)
        else:
            flat[field] = clean_number(value)

    packet_type = packet.get("packet")
    status_field = None
    if packet_type in WITNESS_STATUS_PACKETS:
        status_field = "status"
    elif packet_type == "camera":
        status_field = "fc_status"

    if status_field is not None and isinstance(packet.get(status_field), int):
        status = packet[status_field]
        status_byte0 = (status >> 8) & 0xFF
        status_byte1 = status & 0xFF
        state = status_byte1 & 0x07
        flat["status_byte0"] = status_byte0
        flat["status_byte1"] = status_byte1
        flat["flight_state"] = state
        flat["flight_state_name"] = FLIGHT_STATE_NAMES.get(
            state, f"Reserved ({state})"
        )
        # Status byte 1 bit 3: fresh GPS fix; bit 4: HITL (simulated sensors).
        flat["gps_locked"] = (status_byte1 >> 3) & 0x01
        flat["hitl_active"] = (status_byte1 >> 4) & 0x01
    return flat


def flash_record_size(data: bytes) -> int | None:
    """Record size of a WL file from its first record's version, or None."""
    if len(data) < 3 or data[0:2] != FLASH_RECORD_MAGIC:
        return None
    return FLASH_RECORD_SIZES.get(data[2])


def is_flash_record_log(data: bytes) -> bool:
    """Return True when the stream is predominantly aligned WL records (v1 or v2)."""
    size = flash_record_size(data)
    if size is None:
        return False
    complete_records = len(data) // size
    if complete_records == 0:
        return False
    sample_count = min(complete_records, 64)
    valid = 0
    for index in range(sample_count):
        offset = index * size
        if (data[offset:offset + 2] == FLASH_RECORD_MAGIC
                and FLASH_RECORD_SIZES.get(data[offset + 2]) == size):
            valid += 1
    return valid >= max(1, math.ceil(sample_count * 0.9))


def parse_flash_record_v2(record: bytes) -> dict[str, Any] | None:
    if len(record) != 24 or record[0:2] != FLASH_RECORD_MAGIC or record[2] != 2:
        return None
    status0, status1 = record[3], record[4]
    state = status1 & 0x07
    return {
        "packet": "flash_sample",
        "record_version": 2,
        "status_byte0": status0,
        "status_byte1": status1,
        "heartbeat_phase": int(bool(status0 & 0x01)),
        "low_g_valid": int(bool(status0 & 0x02)),
        "gyro_valid": int(bool(status0 & 0x04)),
        "imu_temperature_valid": int(bool(status0 & 0x08)),
        "flash_init_failed": int(bool(status0 & 0x10)),
        "high_g_valid": int(bool(status0 & 0x20)),
        "barometer_valid": int(bool(status0 & 0x40)),
        "logging_failed": int(bool(status0 & 0x80)),
        "flight_state": state,
        "flight_state_name": FLIGHT_STATE_NAMES.get(state, f"Reserved ({state})"),
        "gps_locked": int(bool(status1 & 0x08)),
        "hitl_active": int(bool(status1 & 0x10)),
        "sequence": unsigned(record[6:10]),
        "uptime": unsigned(record[10:14]),
        "lsm6_temperature": (
            LSM6_TEMPERATURE_OFFSET_C
            + signed(record[14:16]) * LSM6_TEMPERATURE_C_PER_COUNT
        ),
        "pressure": signed(record[16:20]),  # pascals
        "ms5607_temperature": signed(record[20:24]) * MS5607_TEMPERATURE_C_PER_COUNT,
        # Recomputed exactly as Witness does (the state packet now carries the
        # Kalman altitude instead).
        "barometric_altitude": pressure_altitude_m(signed(record[16:20])),
    }


def parse_flash_record(record: bytes) -> dict[str, Any] | None:
    if (len(record) != FLASH_RECORD_SIZE
            or record[0:2] != FLASH_RECORD_MAGIC
            or record[2] != FLASH_RECORD_VERSION):
        return None

    status = record[3]
    low_accel_raw = [signed(record[index:index + 2]) for index in (12, 14, 16)]
    high_accel_raw = [signed(record[index:index + 2]) for index in (18, 20, 22)]
    gyro_raw = [signed(record[index:index + 2]) for index in (24, 26, 28)]
    return {
        "packet": "flash_sample",
        "record_version": record[2],
        "status_byte0": status,
        "heartbeat_phase": int(bool(status & 0x01)),
        "low_g_valid": int(bool(status & 0x02)),
        "gyro_valid": int(bool(status & 0x04)),
        "imu_temperature_valid": int(bool(status & 0x08)),
        "flash_init_failed": int(bool(status & 0x10)),
        "high_g_valid": int(bool(status & 0x20)),
        "barometer_valid": int(bool(status & 0x40)),
        "logging_failed": int(bool(status & 0x80)),
        "sequence": unsigned(record[4:8]),
        "uptime": unsigned(record[8:12]),
        "low_accel": [value * LSM6_LOW_ACCEL_G_PER_COUNT for value in low_accel_raw],
        "high_accel": [value * LSM6_HIGH_ACCEL_G_PER_COUNT for value in high_accel_raw],
        "gyro": [value * GYRO_DPS_PER_COUNT for value in gyro_raw],
        "lsm6_temperature": (
            LSM6_TEMPERATURE_OFFSET_C
            + signed(record[30:32]) * LSM6_TEMPERATURE_C_PER_COUNT
        ),
        # The logger stores the driver's centi-mbar value. One centi-mbar is
        # numerically equal to one pascal.
        "pressure": unsigned(record[32:36]),
        "ms5607_temperature": (
            signed(record[36:40]) * MS5607_TEMPERATURE_C_PER_COUNT
        ),
        "barometric_altitude": float32(record[40:44]),
        "witness_battery_voltage": unsigned(record[44:48]) * BATTERY_V_PER_COUNT,
    }


class DecodeStats:
    """Filled in by the iter_* decoders: bytes not decoded, when finished."""

    def __init__(self) -> None:
        self.discarded = 0


def decode_flash_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    stats = DecodeStats()
    rows = list(iter_flash_log(data, stats))
    return rows, stats.discarded


def iter_flash_log(data: bytes, stats: DecodeStats | None = None):
    """WL flash records (v1 48-byte / v2 24-byte), one row dict at a time."""
    stats = stats or DecodeStats()
    count = 0
    size = flash_record_size(data) or FLASH_RECORD_SIZE
    parse = parse_flash_record_v2 if size == 24 else parse_flash_record
    discarded = len(data) % size
    complete_length = len(data) - discarded

    for offset in range(0, complete_length, size):
        record = data[offset:offset + size]
        packet = parse(record)
        if packet is None:
            discarded += size
            continue
        uptime_ms = packet["uptime"]
        row: dict[str, Any] = {
            "packet_index": count,
            "byte_offset": offset,
            "packet_id": "WL",
            "packet_type": packet["packet"],
            "frame_length": size,
            "time_seconds": uptime_ms / 1000.0,
        }
        row.update(flatten_packet(packet))
        row["raw_packet_hex"] = record.hex(" ")
        count += 1
        yield row

    stats.discarded = discarded


def frame_payload_length(data: bytes, offset: int, lengths: tuple[int, ...]) -> int | None:
    """Return the first candidate payload length whose EOF byte is 0x0A.

    Candidates are listed longest first. For the state packet this means a
    65-byte frame wins whenever frame offset 64 is 0x0A, even if offset 63 is
    also 0x0A (10 satellites). The legacy 64-byte frame is chosen only when
    offset 63 is 0x0A and offset 64 is not (or the data ends). No packet ID is
    0x0A, so a legacy frame followed by any valid packet is unambiguous; the
    remaining false match needs a stray 0x0A byte right after a legacy frame.
    """
    for candidate_length in lengths:
        eof_index = offset + 1 + candidate_length
        if eof_index < len(data) and data[eof_index] == EOF:
            return candidate_length
    return None


def decode_imu_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    """Decode a store-v3 I*.BIN IMU stream into rows.

    time_seconds is relative to the first record, unwrapping the 32-bit
    microsecond counter. ``missing_before`` counts records lost before this
    one, from the 8-bit sequence (it cannot see gaps of 256 or more).
    """
    stats = DecodeStats()
    rows = list(iter_imu_log(data, stats))
    return rows, stats.discarded


def iter_imu_log(data: bytes, stats: DecodeStats | None = None):
    """decode_imu_log, one row dict at a time (bounded memory)."""
    stats = stats or DecodeStats()
    count = 0
    discarded = len(data) % IMU_RECORD_SIZE
    stats.discarded = discarded
    first_us = None
    previous_raw = None
    wraps = 0
    previous_sequence = None
    for offset in range(0, len(data) - discarded, IMU_RECORD_SIZE):
        record = data[offset:offset + IMU_RECORD_SIZE]
        raw_us = unsigned(record[0:4])
        if previous_raw is not None and raw_us < previous_raw:
            wraps += 1
        previous_raw = raw_us
        timestamp_us = raw_us + (wraps << 32)
        if first_us is None:
            first_us = timestamp_us
        status = record[22]
        sequence = record[23]
        missing = 0 if previous_sequence is None else (sequence - previous_sequence - 1) % 256
        previous_sequence = sequence
        state = status & IMU_STATUS_STATE_MASK
        low = [signed(record[i:i + 2]) * LSM6_LOW_ACCEL_G_PER_COUNT for i in (4, 6, 8)]
        high = [signed(record[i:i + 2]) * LSM6_HIGH_ACCEL_G_PER_COUNT for i in (10, 12, 14)]
        gyro = [signed(record[i:i + 2]) * GYRO_DPS_PER_COUNT for i in (16, 18, 20)]
        count += 1
        yield {
            "packet_index": count - 1,
            "byte_offset": offset,
            "packet_id": "IMU",
            "packet_type": "imu_sample",
            "frame_length": IMU_RECORD_SIZE,
            "time_seconds": round((timestamp_us - first_us) / 1e6, 6),
            "timestamp_us": timestamp_us,
            "sequence": sequence,
            "missing_before": missing,
            "flight_state": state,
            "flight_state_name": FLIGHT_STATE_NAMES.get(state, f"Reserved ({state})"),
            "hitl_active": int(bool(status & IMU_STATUS_HITL)),
            "low_g_fresh": int(bool(status & IMU_STATUS_LOW_G_FRESH)),
            "high_g_fresh": int(bool(status & IMU_STATUS_HIGH_G_FRESH)),
            "gyro_fresh": int(bool(status & IMU_STATUS_GYRO_FRESH)),
            "high_g_active": int(bool(status & IMU_STATUS_HIGH_G_ACTIVE)),
            "low_accel_x_g": low[0], "low_accel_y_g": low[1], "low_accel_z_g": low[2],
            "high_accel_x_g": high[0], "high_accel_y_g": high[1], "high_accel_z_g": high[2],
            "gyro_x_dps": gyro[0], "gyro_y_dps": gyro[1], "gyro_z_dps": gyro[2],
            "raw_packet_hex": record.hex(" "),
        }


# Witness store-v4 packet stream (P*.BIN): records of
# [u32 timestamp_us][u8 source][u8 length][complete frame], big-endian.
PACKET_RECORD_HEADER = 6
PACKET_SOURCES = {1: "witness_state", 2: "can_rx", 3: "can_tx", 4: "radio_rx",
                  5: "kalman_log"}


def decode_packet_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    """Decode a P*.BIN packet stream into one row per logged frame.

    time_seconds is relative to the first record (32-bit microseconds
    unwrapped). Frames the wire parser does not know are kept with their
    raw bytes; a corrupt record ends the scan and counts as discarded.
    """
    stats = DecodeStats()
    rows = list(iter_packet_log(data, stats))
    return rows, stats.discarded


def iter_packet_log(data: bytes, stats: DecodeStats | None = None):
    """decode_packet_log, one row dict at a time (bounded memory)."""
    stats = stats or DecodeStats()
    count = 0
    offset = 0
    first_us = None
    previous_raw = None
    wraps = 0
    while offset + PACKET_RECORD_HEADER <= len(data):
        raw_us = unsigned(data[offset:offset + 4])
        source = data[offset + 4]
        length = data[offset + 5]
        start = offset + PACKET_RECORD_HEADER
        frame = data[start:start + length]
        if source not in PACKET_SOURCES or length < 2 or len(frame) != length \
                or frame[-1] != 0x0A:
            break
        if previous_raw is not None and raw_us < previous_raw:
            wraps += 1
        previous_raw = raw_us
        timestamp_us = raw_us + (wraps << 32)
        if first_us is None:
            first_us = timestamp_us
        packet_id = frame[0]
        try:
            packet = parse_packet(packet_id, frame[1:-1])
        except (IndexError, struct.error, ValueError):
            packet = None
        row: dict[str, Any] = {
            "packet_index": count,
            "byte_offset": offset,
            "packet_id": f"0x{packet_id:02X}",
            "packet_type": packet["packet"] if packet else "unknown",
            "frame_length": length,
            "time_seconds": round((timestamp_us - first_us) / 1e6, 6),
            "timestamp_us": timestamp_us,
            "log_source": PACKET_SOURCES[source],
        }
        if packet:
            row.update(flatten_packet(packet))
        row["raw_packet_hex"] = frame.hex(" ")
        count += 1
        offset = start + length
        yield row
    stats.discarded = len(data) - offset


def is_packet_log_name(path: Path) -> bool:
    """Witness exports the packet stream as P<generation>.BIN."""
    return path.name.upper().startswith("P") and path.suffix.upper() == ".BIN"


def is_imu_log_name(path: Path) -> bool:
    """Witness exports the IMU stream as I<generation>.BIN."""
    return path.name.upper().startswith("I") and path.suffix.upper() == ".BIN"


def decode_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    """Decode either a flash-record file or a possibly noisy packet stream."""
    stats = DecodeStats()
    rows = list(iter_log(data, stats))
    return rows, stats.discarded


def iter_log(data: bytes, stats: DecodeStats | None = None):
    """decode_log, one row dict at a time (bounded memory)."""
    stats = stats or DecodeStats()
    if is_flash_record_log(data):
        yield from iter_flash_log(data, stats)
        return

    offset = 0
    discarded = 0
    packet_index = 0

    while offset < len(data):
        packet_id = data[offset]
        host_lengths = HOST_PAYLOAD_LENGTHS.get(packet_id)
        if host_lengths is not None:
            payload_length = frame_payload_length(data, offset, host_lengths)
            if payload_length is not None:
                # Skip the echoed host packet whole; it is not telemetry and
                # is not counted as discarded noise.
                offset += 1 + payload_length + 1
                continue
        lengths = PAYLOAD_LENGTHS.get(packet_id)
        if lengths is None:
            offset += 1
            discarded += 1
            continue

        payload_length = frame_payload_length(data, offset, lengths)

        if payload_length is None:
            offset += 1
            discarded += 1
            continue

        frame_length = 1 + payload_length + 1
        frame = data[offset:offset + frame_length]
        payload = frame[1:-1]
        try:
            packet = parse_packet(packet_id, payload)
        except (IndexError, struct.error, ValueError):
            packet = None

        if packet is None:
            offset += 1
            discarded += 1
            continue

        uptime_ms = packet.get("fc_uptime", packet.get("uptime", ""))
        row: dict[str, Any] = {
            "packet_index": packet_index,
            "byte_offset": offset,
            "packet_id": f"0x{packet_id:02X}",
            "packet_type": packet["packet"],
            "frame_length": frame_length,
            "time_seconds": uptime_ms / 1000.0 if isinstance(uptime_ms, int) else "",
        }
        row.update(flatten_packet(packet))
        row["raw_packet_hex"] = frame.hex(" ")
        packet_index += 1
        offset += frame_length
        stats.discarded = discarded
        yield row

    stats.discarded = discarded


FORMAT_CHOICES = ("auto", "imu", "packets", "flash", "wire")


def select_decoder(path: Path, data: bytes, fmt: str = "auto"):
    """(input format description, iter_* decoder) for a raw log.

    auto: I*.BIN -> IMU records, P*.BIN -> packet log, WL magic -> flash
    records, anything else -> the wire packet scanner.
    """
    if fmt == "imu" or (fmt == "auto" and is_imu_log_name(path)):
        return "Witness IMU records (24-byte)", iter_imu_log
    if fmt == "packets" or (fmt == "auto" and is_packet_log_name(path)):
        return "Witness packet log (store v4 P*.BIN)", iter_packet_log
    if fmt == "flash" or (fmt == "auto" and is_flash_record_log(data)):
        return "Witness flash records", iter_flash_log
    return ("Witness flash records" if is_flash_record_log(data) else "wire packets"), iter_log


def csv_fieldnames(rows) -> list[str]:
    """The CSV header write_csv uses for these rows."""
    fieldnames = list(BASE_COLUMNS)
    seen = set(fieldnames)
    for row in rows:
        for field in row:
            if field not in seen and field != "raw_packet_hex":
                fieldnames.append(field)
                seen.add(field)
    fieldnames.append("raw_packet_hex")
    return fieldnames


BASE_COLUMNS = (
    "packet_index",
    "byte_offset",
    "packet_id",
    "packet_type",
    "frame_length",
    "time_seconds",
)


def write_csv_stream(make_rows, output_path: Path) -> int:
    """write_csv without holding every row: ``make_rows()`` is called twice
    (once for the header, once to write). Same file as write_csv. Returns
    the number of rows written."""
    fieldnames = csv_fieldnames(make_rows())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in make_rows():
            writer.writerow(row)
            count += 1
    return count


def write_csv(rows: list[dict[str, Any]], output_path: Path) -> None:
    fieldnames = csv_fieldnames(rows)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a Horizon raw .bin telemetry log to a flat CSV file."
    )
    parser.add_argument("input", type=Path, help="raw .bin log to convert")
    parser.add_argument(
        "-o", "--output", type=Path,
        help="output CSV path (default: input filename with a .csv extension)",
    )
    parser.add_argument(
        "--format", choices=FORMAT_CHOICES, default="auto",
        help="input format; auto treats I*.BIN as the Witness IMU stream",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    input_path: Path = args.input
    output_path: Path = args.output or input_path.with_suffix(".csv")

    if not input_path.is_file():
        print(f"error: input file does not exist: {input_path}", file=sys.stderr)
        return 2

    try:
        data = input_path.read_bytes()
        input_format, decoder = select_decoder(input_path, data, args.format)
        stats = DecodeStats()
        rows = list(decoder(data, stats))
        discarded = stats.discarded
        write_csv(rows, output_path)
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    counts: dict[str, int] = {}
    for row in rows:
        packet_type = str(row["packet_type"])
        counts[packet_type] = counts.get(packet_type, 0) + 1
    summary = ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))
    print(f"Input format: {input_format}")
    print(f"Wrote {len(rows)} record(s) to {output_path}")
    if summary:
        print(f"Packets: {summary}")
    print(f"Discarded/unframed bytes: {discarded}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
