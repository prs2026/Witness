"""Convert Horizon binary telemetry logs to plotting-friendly CSV files.

Usage:
    python converter.py logs/serial_COM3_20260918_120000.bin
    python converter.py input.bin --output output.csv

The converter auto-detects both raw application-packet streams and the fixed
48-byte ``WL`` records exported by Witness flash mass storage. Neither format
contains host receive timestamps, so ``time_seconds`` is derived from device
uptime.
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
}

# The current capture format uses a 23-byte sensor payload.  The 21-byte form
# declared by comms.h is also accepted for compatibility.
PAYLOAD_LENGTHS = {
    0x01: (23, 21),
    0x02: (62,),
    0x03: (17,),
    0x05: (8,),
    0xFF: (6,),
    0xE0: (101,),
    0xE1: (12,),
    0xF0: (11,),
}

# Flash mass-storage .BIN files use fixed records rather than application
# packets. See FlashLogger::serialize_record in the Witness firmware.
FLASH_RECORD_MAGIC = b"WL"
FLASH_RECORD_VERSION = 1
FLASH_RECORD_SIZE = 48

FILTERED_ACCEL_G_PER_COUNT = 0.000040
GYRO_DPS_PER_COUNT = 0.070
BATTERY_V_PER_COUNT = 0.001
CURRENT_A_PER_COUNT = 0.1
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

    if packet_id == 0x02 and len(payload) == 62:
        return {
            "packet": "state",
            "status": unsigned(payload[0:2]),
            "uptime": unsigned(payload[2:6]),
            "orientation": float_vector(payload[6:22], 4),
            "position": float_vector(payload[22:34], 3),
            "velocity": float_vector(payload[34:46], 3),
            "latitude": signed_int24(payload[46:49]) * GPS_DEGREES_PER_COUNT,
            "longitude": signed_int24(payload[49:52]) * GPS_DEGREES_PER_COUNT,
            "gps_altitude": float32(payload[52:56]),
            "barometric_altitude": float32(payload[56:60]),
            "witness_battery_voltage": unsigned(payload[60:62]) * BATTERY_V_PER_COUNT,
        }

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

    if packet_id == 0xF0 and len(payload) == 11:
        return {
            "packet": "ground_station_status",
            "rssi": signed(payload[0:1]),
            "uptime": unsigned(payload[1:5]),
            "reserved": list(payload[5:11]),
        }

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
            "barometric_altitude": float32(payload[83:87]),
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
    return flat


def is_flash_record_log(data: bytes) -> bool:
    """Return True when the stream is predominantly aligned 48-byte WL records."""
    complete_records = len(data) // FLASH_RECORD_SIZE
    if complete_records == 0:
        return False
    sample_count = min(complete_records, 64)
    valid = 0
    for index in range(sample_count):
        offset = index * FLASH_RECORD_SIZE
        if (data[offset:offset + 2] == FLASH_RECORD_MAGIC
                and data[offset + 2] == FLASH_RECORD_VERSION):
            valid += 1
    return valid >= max(1, math.ceil(sample_count * 0.9))


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


def decode_flash_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    discarded = len(data) % FLASH_RECORD_SIZE
    complete_length = len(data) - discarded

    for offset in range(0, complete_length, FLASH_RECORD_SIZE):
        record = data[offset:offset + FLASH_RECORD_SIZE]
        packet = parse_flash_record(record)
        if packet is None:
            discarded += FLASH_RECORD_SIZE
            continue
        uptime_ms = packet["uptime"]
        row: dict[str, Any] = {
            "packet_index": len(rows),
            "byte_offset": offset,
            "packet_id": "WL",
            "packet_type": packet["packet"],
            "frame_length": FLASH_RECORD_SIZE,
            "time_seconds": uptime_ms / 1000.0,
        }
        row.update(flatten_packet(packet))
        row["raw_packet_hex"] = record.hex(" ")
        rows.append(row)

    return rows, discarded


def decode_log(data: bytes) -> tuple[list[dict[str, Any]], int]:
    """Decode either a flash-record file or a possibly noisy packet stream."""
    if is_flash_record_log(data):
        return decode_flash_log(data)

    rows: list[dict[str, Any]] = []
    offset = 0
    discarded = 0
    packet_index = 0

    while offset < len(data):
        packet_id = data[offset]
        lengths = PAYLOAD_LENGTHS.get(packet_id)
        if lengths is None:
            offset += 1
            discarded += 1
            continue

        payload_length = None
        for candidate_length in lengths:
            eof_index = offset + 1 + candidate_length
            if eof_index < len(data) and data[eof_index] == EOF:
                payload_length = candidate_length
                break

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
        rows.append(row)
        packet_index += 1
        offset += frame_length

    return rows, discarded


BASE_COLUMNS = (
    "packet_index",
    "byte_offset",
    "packet_id",
    "packet_type",
    "frame_length",
    "time_seconds",
)


def write_csv(rows: list[dict[str, Any]], output_path: Path) -> None:
    fieldnames = list(BASE_COLUMNS)
    seen = set(fieldnames)
    for row in rows:
        for field in row:
            if field not in seen and field != "raw_packet_hex":
                fieldnames.append(field)
                seen.add(field)
    fieldnames.append("raw_packet_hex")

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
        input_format = (
            "Witness flash records" if is_flash_record_log(data) else "wire packets"
        )
        rows, discarded = decode_log(data)
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
