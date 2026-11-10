"""Telemetry field schemas and the display model behind Horizon's field trees.

No GUI here: flatten_fields() turns a packet into dotted field paths, and
TelemetryTable keeps per-row value / receive time / quality / age state that a
view (telemetry_view.TelemetryView) renders. WITNESS_FIELDS and
FEATHERWEIGHT_FIELDS list the rows created up front for each input.
"""

from __future__ import annotations

import datetime as dt
import json
import math
import time
from dataclasses import dataclass

WITNESS_FIELDS = {
    "sensors": ("status", "uptime", "filtered_accel[0]", "filtered_accel[1]", "filtered_accel[2]",
                "gyro[0]", "gyro[1]", "gyro[2]", "sensor_trailing_u16"),
    "state": ("status", "uptime", "orientation[0]", "orientation[1]", "orientation[2]", "orientation[3]",
              "position[0]", "position[1]", "position[2]", "velocity[0]", "velocity[1]", "velocity[2]",
              "latitude", "longitude", "gps_altitude", "kalman_altitude", "witness_battery_voltage",
              "gps_satellites"),
    "camera": ("fc_status", "fc_uptime", "status", "uptime", "current_sense[0]", "current_sense[1]",
               "current_sense[2]", "iris_battery_voltage"),
    "heartbeat": ("status", "uptime"),
    "command": ("status", "uptime", "command_value"),
    "ground_station_status": ("rssi", "uptime", "gps_fix_quality", "gps_satellites", "latitude",
                              "longitude", "gps_altitude", "gps_time", "gps_hdop", "gps_fix_age_ms"),
    "witness_debug": ("status", "uptime", "mcu_temperature", "witness_battery_voltage",
                      *(f"{name}[{i}]" for name, count in (("filtered_accel", 3), ("orientation", 4),
                        ("position", 3), ("velocity", 3), ("low_accel", 3), ("high_accel", 3), ("gyro", 3))
                        for i in range(count)), "lsm6_temperature", "pressure", "ms5607_temperature",
                      "kalman_altitude", "latitude", "longitude", "gps_time", "gps_altitude"),
    "iris_debug": ("status", "uptime", "mcu_temperature", "iris_battery_voltage",
                   "current_sense[0]", "current_sense[1]", "current_sense[2]"),
}
FEATHERWEIGHT_FIELDS = {"GPS_STAT": (
    "packet_type", "timestamp", "packet_length", "radio_crc_status", "text_crc_valid",
    "text_crc_expected", "text_crc_computed", "unit_type", "unit_type_raw", "unit_id", "altitude_ft", "latitude_deg",
    "longitude_deg", "horizontal_velocity_fps", "heading_deg", "vertical_velocity_fps", "fix_type",
    "satellites_total", "satellites_over_24db", "satellites_over_32db", "satellites_over_40db",
    *(f"satellite_details[{i}]" for i in range(5)),
)}
_FW_STATUS_COMMON = ("timestamp", "text_crc_valid", "text_crc_expected", "text_crc_computed")
FEATHERWEIGHT_FIELDS.update({
    "BATT_BLE": (*_FW_STATUS_COMMON, "battery_mv", "battery_voltage_v", "battery_valid",
                 "battery_scope", "ble_status", "ble_connected", "temperature_c"),
    "RX_NOMTK": (*_FW_STATUS_COMMON, "radio_crc_status", "unit_type", "unit_id", "packets_received",
                 "packets_sent", "rssi_dbm", "snr_db", "ack_received", "ack_sent", "ack_rssi_dbm",
                 "ack_snr_db", "spreading_factor", "frequency_hz", "tracker_battery_mv",
                 "tracker_battery_voltage_v", "tracker_battery_valid", "temperature_c"),
    "GS_STAT": (*_FW_STATUS_COMMON, "radio_crc_status", "unit_id", "packets_received", "packets_sent",
                "rssi_dbm", "ack_received", "ack_sent", "ack_rssi_dbm", "found_unit_id",
                "found_received", "found_sent", "found_rssi_dbm", "spreading_factor", "frequency_hz"),
    "TX_STAT": (*_FW_STATUS_COMMON, "unit_id", "apid", "tx_duration_ms", "spreading_factor", "frequency_hz"),
    "RX_TMOUT": (*_FW_STATUS_COMMON, "receive_timeout", "spreading_factor"),
})
FLIGHT_STATE_NAMES = ("Pad idle", "Boost", "Coast", "Descent", "Landed")
MAX_ROWS = 8192


def flatten_fields(packet, max_fields=512):
    fields = {}

    def visit(value, path, depth):
        if depth > 16 or len(fields) >= max_fields:
            raise ValueError("Telemetry field/depth limit exceeded")
        if isinstance(value, dict) and value:
            for key, item in value.items():
                key = key.replace("\\", "\\\\").replace(".", "\\.")
                key = key.replace("[", "\\[").replace("]", "\\]")
                visit(item, f"{path}.{key}" if path else key, depth + 1)
        elif isinstance(value, list) and value:
            for index, item in enumerate(value):
                visit(item, f"{path}[{index}]", depth + 1)
        else:
            fields[path] = value
    for key, value in packet.items():
        if key != "packet":
            visit(value, key, 0)
    return fields


def witness_status_fields(packet):
    """Derived Witness status bit fields (byte 0 bits, flight state, GPS lock, HITL)."""
    fields = {}
    for status_field in ("status", "fc_status"):
        status = packet.get(status_field)
        if not isinstance(status, int):
            continue
        for bit in range(8):
            fields[f"{status_field}.byte0.bit{bit}"] = (status >> (8 + bit)) & 1
        camera = status_field == "status" and packet["packet"] in ("camera", "iris_debug")
        if not camera:
            state = status & 7
            fields[f"{status_field}.flight_state"] = state
            fields[f"{status_field}.flight_state_name"] = FLIGHT_STATE_NAMES[state] if state < 5 else "Reserved/invalid"
            # Status byte 1 bit 3: GPS locked; bit 4: HITL active (simulated sensors).
            fields[f"{status_field}.gps_locked"] = (status >> 3) & 1
            fields[f"{status_field}.hitl_active"] = (status >> 4) & 1
    return fields


def age_tag(age, invalid=False):
    if invalid:
        return "invalid"
    return "fresh" if age < 2 else "yellow" if age < 5 else "orange" if age < 10 else "stale"


@dataclass
class Row:
    source: str
    packet: str
    field: str
    value: str = "null"
    received: str = "--"
    age: str = "--"
    quality: str = "--"
    tag: str = ""
    received_mono: float | None = None
    invalid: bool = False


class TelemetryTable:
    """Rows keyed (source, packet, field) in display order; bounded to MAX_ROWS."""

    def __init__(self, schema=None, *, max_rows=MAX_ROWS):
        self.schema = schema or {}
        self.max_rows = max_rows
        self.rows: dict[tuple, Row] = {}
        self.sources: set[str] = set()
        self.limit_reported = False

    def clear(self):
        self.rows.clear()
        self.sources.clear()
        self.limit_reported = False

    def row(self, source, packet, field):
        """Existing or new row (None once the row limit is reached)."""
        key = (source, packet, field)
        row = self.rows.get(key)
        if row is None:
            if len(self.rows) >= self.max_rows:
                return None
            row = self.rows[key] = Row(source, packet, field)
        return row

    def add_source(self, source):
        self.sources.add(source)
        return [row for packet, fields in self.schema.items() for field in fields
                if (row := self.row(source, packet, field)) is not None]

    def update_record(self, record, *, witness=False):
        """Apply one record; returns the rows that changed."""
        packet = record["packet"]
        fields = flatten_fields(packet)
        if witness:
            fields.update(witness_status_fields(packet))
        source = record["source"]
        changed = []
        if source not in self.sources:
            changed += self.add_source(source)
        received_at = dt.datetime.fromtimestamp(record["received_at"], dt.timezone.utc).isoformat(
            timespec="milliseconds").replace("T", " ").replace("+00:00", "")
        received_mono = record.get("received_mono", time.monotonic() - max(
            0, time.time() - record["received_at"]))
        bad_text = packet.get("text_crc_valid") is False
        bad_radio = packet.get("radio_crc_status", "CRC_OK") != "CRC_OK"
        invalid = bad_text or bad_radio
        quality = ("BAD TEXT CRC" if bad_text else "BAD RADIO CRC" if bad_radio else
                   "Snapshot" if record.get("replay") else "Live")
        for field, value in fields.items():
            row = self.row(source, packet["packet"], field)
            if row is None:
                continue
            changed.append(row)
            if invalid or value is None or (type(value) is float and not math.isfinite(value)):
                # Hold the last valid value and its original receive time.
                row.quality = quality if invalid else "INVALID - holding last valid"
                row.invalid = True
                row.tag = "invalid"
                continue
            if witness and field in ("status", "fc_status") and isinstance(value, int):
                row.value = f"0b{value:016b}"
            else:
                row.value = value if isinstance(value, str) else json.dumps(value, ensure_ascii=True)
            row.received, row.age, row.quality = received_at, "--", quality
            row.received_mono, row.invalid = received_mono, invalid
        return changed

    def update_ages(self, now=None):
        """Refresh age text and colour tag of rows that have data; returns them."""
        now = time.monotonic() if now is None else now
        rows = []
        for row in self.rows.values():
            if row.received_mono is None:
                continue
            age = max(0.0, now - row.received_mono)
            row.age = f"{age:.1f}s"
            row.tag = age_tag(age, row.invalid)
            rows.append(row)
        return rows

    @property
    def received(self):
        """{key: (received_mono, invalid)} for rows that have data (Tk-era view)."""
        return {key: (row.received_mono, row.invalid) for key, row in self.rows.items()
                if row.received_mono is not None}


@dataclass
class AimRow:
    topic: str
    field: str
    value: str = "null"
    received: str = "--"
    age: str = "--"
    delivery: str = "--"
    tag: str = ""
    received_mono: float | None = None
    retained: bool = False
    invalid: bool = False
    number: float | None = None     # latest numeric value (for the strip chart)


class AimFieldTable:
    """AIM MQTT topic/field rows: value as published, delivery, age (no GUI)."""

    def __init__(self, topic_fields, *, max_rows=4096):
        self.topic_fields = topic_fields
        self.max_rows = max_rows
        self.rows: dict[tuple, AimRow] = {}
        self.limit_reported = self.limit_notice = False
        self.reset()

    def reset(self):
        self.rows.clear()
        self.limit_reported = False
        for topic, fields in self.topic_fields.items():
            for field in fields:
                self.row(topic, field)

    def row(self, topic, field):
        key = (topic, field)
        row = self.rows.get(key)
        if row is None:
            # Keep a badly behaved publisher from creating unlimited rows.
            if len(self.rows) >= self.max_rows:
                return None
            row = self.rows[key] = AimRow(topic, field)
        return row

    def apply(self, updates):
        """updates: {topic: {field: (value, received_at_iso, received_mono, retained)}}."""
        changed = []
        for topic, fields in updates.items():
            for field, (value, received_at, received_mono, retained) in fields.items():
                row = self.row(topic, field)
                if row is None:
                    if not self.limit_reported:
                        self.limit_reported = self.limit_notice = True
                    continue
                changed.append(row)
                if value is None or (type(value) is float and not math.isfinite(value)):
                    row.invalid = True
                    row.delivery = "Invalid - holding last valid"
                    row.tag = "invalid"
                    continue
                row.invalid = False
                row.value = value if isinstance(value, str) else json.dumps(value, ensure_ascii=True)
                row.received = received_at.replace("T", " ").replace("+00:00", "")
                row.age, row.delivery = "0.0s", "Retained" if retained else "Live"
                row.received_mono, row.retained = received_mono, retained
                row.number = float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None
        return changed

    def update_ages(self, now=None):
        now = time.monotonic() if now is None else now
        rows = []
        for row in self.rows.values():
            if row.received_mono is None:
                continue
            age = max(0.0, now - row.received_mono)
            row.age = f"{age:.1f}s"
            row.tag = "invalid" if row.invalid else "retained" if row.retained else age_tag(age)
            rows.append(row)
        return rows
