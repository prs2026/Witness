"""Interactive viewer for CSV files produced by converter.py.

Usage:
    python viewer.py S0000434.csv
    python viewer.py S0000434.csv P0000434.csv   # overlay on Witness uptime
    python viewer.py

With no filename, the viewer opens a file-selection dialog. Several CSVs can
be open at once; drag traces from the list onto graphs to overlay them. The
same viewer is embedded as a witnessOS tab (CsvViewerPanel).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import queue
import threading
from dataclasses import dataclass, field
import math
import re
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import tkinter.font as tkfont

import numpy as np

import csv_columns


GRAPH_HEIGHT = 230
MIN_GRAPH_HEIGHT = 170
MAX_GRAPH_HEIGHT = 380
LAUNCH_ACCELERATION_G = 2.0
LAUNCH_REQUIRED_SAMPLES = 3
LAUNCH_MAX_SAMPLE_GAP_SECONDS = 0.5
PLOT_COLORS = (
    "#1565c0",
    "#c62828",
    "#2e7d32",
    "#6a1b9a",
    "#ef6c00",
    "#00838f",
)

ACCELERATION_VECTORS = (
    ("filtered_accel", "filtered acceleration"),
    ("low_accel", "low-g acceleration"),
    ("high_accel", "high-g acceleration"),
)

FLIGHT_STATE_NAMES = {
    0: "Pad idle",
    1: "Boost",
    2: "Coast",
    3: "Descent",
    4: "Landed",
}

FLIGHT_STATE_VALUES = {
    name.lower(): value for value, name in FLIGHT_STATE_NAMES.items()
}

FLIGHT_STATE_TRANSITION_RE = re.compile(
    r"^[A-Z]\s+\((\d+)\)\s+flight_state:\s*"
    r"([a-z ]+?)\s*->\s*([a-z ]+)\s*$",
    re.IGNORECASE | re.MULTILINE,
)

STATUS_BYTE0_BIT_FIELDS = (
    "status_byte0_bit0_heartbeat_phase",
    "status_byte0_bit1_low_g_valid",
    "status_byte0_bit2_gyro_valid",
    "status_byte0_bit3_imu_temperature_valid",
    "status_byte0_bit4_flash_init_failed",
    "status_byte0_bit5_high_g_valid",
    "status_byte0_bit6_barometer_valid",
    "status_byte0_bit7_logging_failed",
)

STATUS_BYTE1_BIT_FIELDS = (
    "status_byte1_bit0_flight_state_bit0",
    "status_byte1_bit1_flight_state_bit1",
    "status_byte1_bit2_flight_state_bit2",
    "status_byte1_bit3_gps_locked",
    "status_byte1_bit4_hitl_active",
    "status_byte1_bit5_reserved",
    "status_byte1_bit6_reserved",
    "status_byte1_bit7_reserved",
)

STATUS_BIT_FIELDS = STATUS_BYTE0_BIT_FIELDS + STATUS_BYTE1_BIT_FIELDS

# Converter columns decoded from Witness status byte 1 (bit 3, bit 4). Older
# CSVs without them get them derived from status/status_byte1 on load.
STATUS_FLAG_COLUMNS = {
    "gps_locked": 0x08,
    "hitl_active": 0x10,
}
# Columns drawn on a fixed 0..1 axis.
BINARY_COLUMNS = frozenset(STATUS_BIT_FIELDS) | frozenset(STATUS_FLAG_COLUMNS)


def numeric_value(text: str | None) -> float | None:
    if text is None or not text.strip():
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def nice_step(span: float, target_intervals: int = 5) -> float:
    if not math.isfinite(span) or span <= 0:
        return 1.0
    rough = span / max(1, target_intervals)
    magnitude = 10.0 ** math.floor(math.log10(rough))
    fraction = rough / magnitude
    if fraction <= 1.0:
        multiplier = 1.0
    elif fraction <= 2.0:
        multiplier = 2.0
    elif fraction <= 5.0:
        multiplier = 5.0
    else:
        multiplier = 10.0
    return multiplier * magnitude


def tick_text(value: float, step: float) -> str:
    if value == 0:
        return "0"
    if abs(value) >= 1_000_000 or abs(value) < 0.001:
        return f"{value:.3g}"
    if step >= 1 and abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    decimals = max(0, min(6, -math.floor(math.log10(step)) + 1))
    return f"{value:.{decimals}f}".rstrip("0").rstrip(".")


def row_time_seconds(row: dict[str, str]) -> float | None:
    value = numeric_value(row.get("time_seconds"))
    if value is not None:
        return value
    for field in ("uptime", "fc_uptime"):
        value = numeric_value(row.get(field))
        if value is not None:
            return value / 1000.0
    return None


def sensor_flag_is_valid(
        row: dict[str, str], mask: int, explicit_field: str,
) -> bool:
    explicit_raw = row.get(explicit_field)
    if explicit_raw is not None and explicit_raw.strip():
        explicit_value = numeric_value(explicit_raw)
        return explicit_value is None or bool(int(explicit_value))

    status_byte0 = numeric_value(row.get("status_byte0"))
    if status_byte0 is not None:
        return bool(int(status_byte0) & mask)

    packet_type = (row.get("packet_type") or "").strip().lower()
    status_field = "fc_status" if packet_type == "camera" else "status"
    combined_status = numeric_value(row.get(status_field))
    if combined_status is not None:
        return bool((int(combined_status) >> 8) & mask)
    return True


def acceleration_values(row: dict[str, str]) -> list[tuple[float, str]]:
    values: list[tuple[float, str]] = []
    for prefix, label in ACCELERATION_VECTORS:
        valid_mask, valid_field = {
            "filtered_accel": (0x02, "low_g_valid"),
            "low_accel": (0x02, "low_g_valid"),
            "high_accel": (0x20, "high_g_valid"),
        }[prefix]
        if not sensor_flag_is_valid(row, valid_mask, valid_field):
            continue
        components = [numeric_value(row.get(f"{prefix}_{axis}")) for axis in "xyz"]
        if all(component is not None for component in components):
            magnitude = math.sqrt(sum(component * component for component in components))
            values.append((magnitude, label))
    return values


def flight_state(row: dict[str, str]) -> int | None:
    """Return the Witness flight state, avoiding the camera's Iris status."""
    for field in ("flight_state", "global_flight_state", "status_byte1"):
        raw_value = (row.get(field) or "").strip()
        if raw_value.lower() == "boost":
            return 1
        value = numeric_value(raw_value)
        if value is not None:
            return int(value) & 0x07

    fc_status = numeric_value(row.get("fc_status"))
    if fc_status is not None:
        return int(fc_status) & 0x07

    packet_type = (row.get("packet_type") or "").strip().lower()
    if packet_type not in {"camera", "iris_debug"}:
        status = numeric_value(row.get("status"))
        if status is not None:
            return int(status) & 0x07
    return None


def add_derived_flight_state(
        rows: list[dict[str, str]], fieldnames: list[str],
) -> None:
    """Expose status byte 1 as columns in older converted packet CSVs."""
    found_state = False
    found_name = False
    for row in rows:
        state = flight_state(row)
        if state is None:
            continue
        if not (row.get("flight_state") or "").strip():
            row["flight_state"] = str(state)
        if not (row.get("flight_state_name") or "").strip():
            row["flight_state_name"] = FLIGHT_STATE_NAMES.get(
                state, f"Reserved ({state})"
            )
        found_state = True
        found_name = True

    if found_state and "flight_state" not in fieldnames:
        fieldnames.append("flight_state")
    if found_name and "flight_state_name" not in fieldnames:
        fieldnames.append("flight_state_name")


def add_paired_wl_flight_state(
        csv_path: Path,
        rows: list[dict[str, str]],
        fieldnames: list[str],
) -> Path | None:
    """Merge timestamped state transitions from the WL session's text log."""
    if not any(
            row.get("packet_id") == "WL" or row.get("packet_type") == "flash_sample"
            for row in rows
    ):
        return None

    text_path = csv_path.with_suffix(".TXT")
    if not text_path.exists():
        text_path = csv_path.with_suffix(".txt")
    if not text_path.exists():
        return None

    try:
        log_text = text_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None

    transitions: list[tuple[float, int, int]] = []
    for match in FLIGHT_STATE_TRANSITION_RE.finditer(log_text):
        from_name = " ".join(match.group(2).lower().split())
        to_name = " ".join(match.group(3).lower().split())
        if from_name not in FLIGHT_STATE_VALUES or to_name not in FLIGHT_STATE_VALUES:
            continue
        transitions.append((
            int(match.group(1)) / 1000.0,
            FLIGHT_STATE_VALUES[from_name],
            FLIGHT_STATE_VALUES[to_name],
        ))
    if not transitions:
        return None
    transitions.sort(key=lambda item: item[0])

    initial_state = transitions[0][1]
    populated = False
    for row in rows:
        if (row.get("flight_state") or "").strip():
            continue
        timestamp = row_time_seconds(row)
        if timestamp is None:
            continue
        state = initial_state
        for transition_time, _from_state, to_state in transitions:
            if timestamp < transition_time:
                break
            state = to_state
        row["flight_state"] = str(state)
        row["flight_state_name"] = FLIGHT_STATE_NAMES[state]
        populated = True

    if not populated:
        return None

    for transition_time, _from_state, to_state in transitions:
        rows.append({
            "packet_id": "TXT",
            "packet_type": "flight_state_transition",
            "time_seconds": f"{transition_time:.3f}",
            "flight_state": str(to_state),
            "flight_state_name": FLIGHT_STATE_NAMES[to_state],
        })
    for field in ("flight_state", "flight_state_name"):
        if field not in fieldnames:
            fieldnames.append(field)
    return text_path


def witness_status_bytes(row: dict[str, str]) -> tuple[int | None, int | None]:
    status_byte0_value = numeric_value(row.get("status_byte0"))
    status_byte1_value = numeric_value(row.get("status_byte1"))
    status_byte0 = (
        int(status_byte0_value) & 0xFF if status_byte0_value is not None else None
    )
    status_byte1 = (
        int(status_byte1_value) & 0xFF if status_byte1_value is not None else None
    )

    packet_type = (row.get("packet_type") or "").strip().lower()
    if packet_type == "camera":
        combined_status = numeric_value(row.get("fc_status"))
    elif packet_type == "iris_debug":
        combined_status = None
    else:
        combined_status = numeric_value(row.get("status"))

    if combined_status is not None:
        combined = int(combined_status) & 0xFFFF
        if status_byte0 is None:
            status_byte0 = (combined >> 8) & 0xFF
        if status_byte1 is None:
            status_byte1 = combined & 0xFF
    return status_byte0, status_byte1


def add_status_bit_traces(
        rows: list[dict[str, str]], fieldnames: list[str],
) -> None:
    """Add one numeric plotting field for every bit in both status bytes."""
    populated_fields: set[str] = set()
    for row in rows:
        status_byte0, status_byte1 = witness_status_bytes(row)
        for byte_value, bit_fields in (
                (status_byte0, STATUS_BYTE0_BIT_FIELDS),
                (status_byte1, STATUS_BYTE1_BIT_FIELDS),
        ):
            if byte_value is None:
                continue
            for bit, field in enumerate(bit_fields):
                row[field] = str((byte_value >> bit) & 0x01)
                populated_fields.add(field)

    for field in STATUS_BIT_FIELDS:
        if field in populated_fields and field not in fieldnames:
            fieldnames.append(field)


def add_status_flag_columns(
        rows: list[dict[str, str]], fieldnames: list[str],
) -> None:
    """Fill gps_locked/hitl_active from status byte 1 where they are blank."""
    populated_fields: set[str] = set()
    for row in rows:
        _status_byte0, status_byte1 = witness_status_bytes(row)
        for field, mask in STATUS_FLAG_COLUMNS.items():
            if (row.get(field) or "").strip():
                populated_fields.add(field)
                continue
            if status_byte1 is None:
                continue
            row[field] = "1" if status_byte1 & mask else "0"
            populated_fields.add(field)
    for field in STATUS_FLAG_COLUMNS:
        if field in populated_fields and field not in fieldnames:
            fieldnames.append(field)


def hitl_record_count(rows: list[dict[str, str]]) -> int:
    """Number of records whose Witness status reports HITL (simulated) mode."""
    count = 0
    for row in rows:
        value = numeric_value(row.get("hitl_active"))
        if value is None:
            _status_byte0, status_byte1 = witness_status_bytes(row)
            value = None if status_byte1 is None else status_byte1 & 0x10
        if value:
            count += 1
    return count


def calculate_summary(rows: list[dict[str, str]]) -> dict[str, str]:
    altitude_samples: list[tuple[float, float | None]] = []
    acceleration_samples: list[tuple[float | None, float, str]] = []
    timed_rows: list[tuple[float, dict[str, str]]] = []
    state_samples: list[tuple[float | None, int, int]] = []

    for row_index, row in enumerate(rows):
        timestamp = row_time_seconds(row)
        if timestamp is not None:
            timed_rows.append((timestamp, row))

        state = flight_state(row)
        if state is not None:
            state_samples.append((timestamp, row_index, state))

        altitude = numeric_value(row.get("barometric_altitude"))
        if (altitude is not None
                and sensor_flag_is_valid(row, 0x40, "barometer_valid")):
            altitude_samples.append((altitude, timestamp))

        row_accelerations = acceleration_values(row)
        if row_accelerations:
            magnitude, source = max(row_accelerations, key=lambda item: item[0])
            acceleration_samples.append((timestamp, magnitude, source))

    if altitude_samples:
        altitude, altitude_time = max(altitude_samples, key=lambda item: item[0])
        altitude_text = f"{altitude:.3f} m"
        if altitude_time is not None:
            altitude_text += f"\nat {altitude_time:.3f} s"
    else:
        altitude_text = "Not available"

    if acceleration_samples:
        acceleration_time, acceleration, acceleration_source = max(
            acceleration_samples, key=lambda item: item[1]
        )
        acceleration_text = f"{acceleration:.3f} g"
        if acceleration_time is not None:
            acceleration_text += f"\nat {acceleration_time:.3f} s"
        acceleration_text += f"\n{acceleration_source}"
    else:
        acceleration_text = "Not available"

    launch_time: float | None = None
    launch_method = ""
    for timestamp, row in sorted(timed_rows, key=lambda item: item[0]):
        if flight_state(row) == 1:
            launch_time = timestamp
            launch_method = "Boost flight state"
            break

    if launch_time is None:
        timed_acceleration = sorted(
            (sample for sample in acceleration_samples if sample[0] is not None),
            key=lambda item: item[0],
        )
        streak_start: float | None = None
        streak_count = 0
        previous_time: float | None = None
        for timestamp, magnitude, _source in timed_acceleration:
            if magnitude < LAUNCH_ACCELERATION_G:
                streak_start = None
                streak_count = 0
                previous_time = timestamp
                continue
            if (previous_time is None
                    or timestamp - previous_time <= LAUNCH_MAX_SAMPLE_GAP_SECONDS):
                if streak_count == 0:
                    streak_start = timestamp
                streak_count += 1
            else:
                streak_start = timestamp
                streak_count = 1
            if streak_count >= LAUNCH_REQUIRED_SAMPLES:
                launch_time = streak_start
                launch_method = f"{LAUNCH_ACCELERATION_G:g} g acceleration threshold"
                break
            previous_time = timestamp

    launch_text = "Not detected"
    if launch_time is not None:
        launch_text = f"{launch_time:.3f} s\n{launch_method}"

    if state_samples:
        ordered_states = [
            state for _timestamp, _row_index, state in sorted(
                state_samples,
                key=lambda item: (
                    item[0] is None,
                    item[0] if item[0] is not None else item[1],
                ),
            )
        ]
        state_history: list[int] = []
        for state in ordered_states:
            if not state_history or state_history[-1] != state:
                state_history.append(state)
        final_state = ordered_states[-1]
        final_name = FLIGHT_STATE_NAMES.get(final_state, f"Reserved ({final_state})")
        history_text = " -> ".join(
            FLIGHT_STATE_NAMES.get(state, f"Reserved ({state})")
            for state in state_history
        )
        flight_state_text = f"{final_name} ({final_state})\n{history_text}"
    else:
        flight_state_text = "Not available in this log"

    hitl_records = hitl_record_count(rows)
    return {
        "max_altitude": altitude_text,
        "launch_time": launch_text,
        "max_acceleration": acceleration_text,
        "flight_state": flight_state_text,
        "hitl": (
            f"HITL ACTIVE in {hitl_records:,} record(s): simulated sensor data"
            if hitl_records else "No HITL records"
        ),
    }


def nice_ticks(low: float, high: float, target_intervals: int = 5,
               inside: bool = False) -> tuple[float, list[float]]:
    """Return (step, ticks) of round values for the range low..high.

    With ``inside`` the ticks stay within the range (x axes, which span the
    data exactly); otherwise they extend to cover it (y axes). Ticks are
    computed from integer multiples, so they never accumulate float error.
    """
    if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
        return 1.0, [low]
    step = nice_step(high - low, target_intervals)
    if inside:
        first = math.ceil(low / step - 1e-9)
        last = math.floor(high / step + 1e-9)
    else:
        first = math.floor(low / step + 1e-9)
        last = math.ceil(high / step - 1e-9)
    last = min(last, first + 200)
    return step, [index * step for index in range(first, last + 1)]


class Series:
    """One trace: x/y numpy arrays sorted by x (no NaN).

    Indexes like the old list of (x, y) points (``s[0][0]``, ``len(s)``),
    but the drawing, zoom and hover paths use the arrays directly so a trace
    of millions of samples never becomes millions of Python objects.
    """
    __slots__ = ("xs", "ys")

    def __init__(self, xs, ys):
        self.xs = np.asarray(xs, dtype=np.float64)
        self.ys = np.asarray(ys, dtype=np.float64)

    def __len__(self) -> int:
        return int(self.xs.size)

    def __bool__(self) -> bool:
        return self.xs.size > 0

    def __getitem__(self, index):
        return (float(self.xs[index]), float(self.ys[index]))

    def __iter__(self):
        return iter(zip(self.xs.tolist(), self.ys.tolist()))

    def view(self, x_min: float, x_max: float) -> "Series":
        """Points inside [x_min, x_max] plus interpolated edge points."""
        xs, ys = self.xs, self.ys
        if xs.size == 0:
            return self
        first = int(np.searchsorted(xs, x_min, side="left"))
        last = int(np.searchsorted(xs, x_max, side="right"))
        out_x = [xs[first:last]]
        out_y = [ys[first:last]]

        def edge(i, x):
            xa, xb, ya, yb = xs[i - 1], xs[i], ys[i - 1], ys[i]
            return x, (ya if xb == xa else ya + (yb - ya) * (x - xa) / (xb - xa))

        if 0 < first < xs.size:
            ex, ey = edge(first, x_min)
            out_x.insert(0, np.array([ex]))
            out_y.insert(0, np.array([ey]))
        if 0 < last < xs.size:
            ex, ey = edge(last, x_max)
            out_x.append(np.array([ex]))
            out_y.append(np.array([ey]))
        return Series(np.concatenate(out_x), np.concatenate(out_y))

    def nearest(self, x: float):
        if self.xs.size == 0:
            return None
        index = int(np.searchsorted(self.xs, x))
        if index >= self.xs.size or (index > 0 and
                                     abs(self.xs[index - 1] - x) <= abs(self.xs[index] - x)):
            index -= 1
        return self[index]


def envelope(xs, ys, x_min: float, x_max: float, buckets: int):
    """Min/max of each of ``buckets`` x columns, in drawing order (numpy).

    Each column contributes its first and last sample's x with the column's
    min and max (ordered so the line stays continuous), so spikes survive
    and at most 2 * buckets points are drawn however long the trace is.
    """
    buckets = max(1, int(buckets))
    if xs.size <= 2 * buckets:
        return xs, ys
    span = (x_max - x_min) or 1.0
    column = ((xs - x_min) / span * buckets).astype(np.int64)
    np.clip(column, 0, buckets - 1, out=column)
    starts = np.flatnonzero(np.r_[True, column[1:] != column[:-1]])
    ends = np.r_[starts[1:], xs.size] - 1
    lows = np.minimum.reduceat(ys, starts)
    highs = np.maximum.reduceat(ys, starts)
    first = ys[starts]
    low_first = np.abs(first - lows) <= np.abs(first - highs)
    out_x = np.empty(starts.size * 2)
    out_y = np.empty(starts.size * 2)
    out_x[0::2] = xs[starts]
    out_x[1::2] = xs[ends]
    out_y[0::2] = np.where(low_first, lows, highs)
    out_y[1::2] = np.where(low_first, highs, lows)
    return out_x, out_y


def decimate_minmax(
        points: list[tuple[float, float]], x_min: float, x_max: float,
        buckets: int,
) -> list[tuple[float, float]]:
    """Reduce sorted points to the min and max of each pixel column.

    Keeps spikes visible (unlike stride sampling) while bounding the number
    of canvas coordinates to about 2 * buckets, however long the log is.
    """
    buckets = max(1, int(buckets))
    if isinstance(points, Series):
        return Series(*envelope(points.xs, points.ys, x_min, x_max, buckets))
    if len(points) <= 2 * buckets:
        return points
    span = (x_max - x_min) or 1.0
    output: list[tuple[float, float]] = []
    current = None
    low = high = None
    for point in points:
        bucket = int((point[0] - x_min) / span * buckets)
        bucket = min(max(bucket, 0), buckets - 1)
        if bucket != current:
            if current is not None:
                output.extend(sorted({low, high}, key=lambda item: item[0]))
            current = bucket
            low = high = point
        else:
            if point[1] < low[1]:
                low = point
            if point[1] > high[1]:
                high = point
    if current is not None:
        output.extend(sorted({low, high}, key=lambda item: item[0]))
    return output


def graph_height_for(count: int, view_height: int) -> int:
    """Per-graph height: fill the view for a few traces, scroll for many."""
    if count <= 0:
        return GRAPH_HEIGHT
    fit = int(view_height / count) if view_height > 1 else GRAPH_HEIGHT
    return max(MIN_GRAPH_HEIGHT, min(MAX_GRAPH_HEIGHT, fit))


def elide(text: str, max_width: float, measure) -> str:
    """Shorten ``text`` with an ellipsis so measure(text) <= max_width."""
    if measure(text) <= max_width:
        return text
    while text and measure(text + "…") > max_width:
        text = text[:-1]
    return text + "…" if text else ""


SYNC_X = "time [s] (per-file time axes)"
SYNC_SKIP_COLUMNS = frozenset(("uptime", "fc_uptime", "time_seconds", "uptime_ms",
                               "timestamp_us", "sim_s", "sim_ms", "wall_s"))
DATASET_DASHES = ((), (7, 3), (2, 2), (9, 3, 2, 3), (4, 4))
DROP_ZONE_HEIGHT = 64
DRAG_THRESHOLD_PX = 6
ZOOM_FACTOR = 0.8          # one Ctrl+wheel step in; its inverse zooms out
PAN_FRACTION = 0.1         # one Shift+wheel step pans this share of the view
MIN_ZOOM_DRAG_PX = 5       # a shorter drag across a plot is just a click


def view_points(
        points: list[tuple[float, float]], x_min: float, x_max: float,
) -> list[tuple[float, float]]:
    """The part of sorted ``points`` inside [x_min, x_max].

    Where the line crosses either edge, a point interpolated onto the edge is
    added, so a zoomed trace runs right to the plot border instead of
    stopping at the last sample inside it.
    """
    if isinstance(points, Series):
        return points.view(x_min, x_max)
    if not points:
        return []
    first = bisect.bisect_left(points, x_min, key=lambda point: point[0])
    last = bisect.bisect_right(points, x_max, key=lambda point: point[0])
    inside = points[first:last]

    def edge(a, b, x):
        (xa, ya), (xb, yb) = a, b
        if xb == xa:
            return (x, ya)
        return (x, ya + (yb - ya) * (x - xa) / (xb - xa))

    out = []
    if 0 < first < len(points):
        out.append(edge(points[first - 1], points[first], x_min))
    out.extend(inside)
    if 0 < last < len(points):
        out.append(edge(points[last - 1], points[last], x_max))
    return out


def zoom_range(view: tuple[float, float], full: tuple[float, float],
               center: float, factor: float) -> tuple[float, float] | None:
    """Scale ``view`` by ``factor`` about ``center``, kept inside ``full``.

    Returns None when the result covers the full range (zoom reset).
    """
    low, high = view
    full_low, full_high = full
    new_low = center - (center - low) * factor
    new_high = center + (high - center) * factor
    span = new_high - new_low
    full_span = full_high - full_low
    if span >= full_span or span <= 0:
        return None
    if new_low < full_low:
        new_low, new_high = full_low, full_low + span
    if new_high > full_high:
        new_low, new_high = full_high - span, full_high
    return (new_low, new_high)


def pan_range(view: tuple[float, float], full: tuple[float, float],
              delta: float) -> tuple[float, float]:
    """Shift ``view`` by ``delta`` without leaving ``full``."""
    low, high = view
    delta = max(full[0] - low, min(full[1] - high, delta))
    return (low + delta, high + delta)


def nearest_point(points: list[tuple[float, float]], x: float):
    """The sample of sorted ``points`` whose x is closest to ``x``."""
    if isinstance(points, Series):
        return points.nearest(x)
    if not points:
        return None
    index = bisect.bisect_left(points, x, key=lambda point: point[0])
    candidates = points[max(0, index - 1):index + 1]
    return min(candidates, key=lambda point: abs(point[0] - x))


def value_text(value: float) -> str:
    """Exact-looking readout of a CSV value (no float noise)."""
    if value == int(value) and abs(value) < 1e15:
        return str(int(value))
    return f"{value:.10g}"




def sync_time_seconds(row: dict[str, str]) -> float | None:
    """Witness uptime in seconds, the shared clock for overlaying logs.

    Packet and WL logs carry the board's ``uptime`` (ms); camera packets
    carry the flight computer's ``fc_uptime``. Logs without either (e.g.
    I*.BIN IMU records, whose time_seconds is relative to the first record)
    fall back to ``time_seconds`` and can be aligned with a manual offset.
    """
    for field in ("uptime", "fc_uptime"):
        value = numeric_value(row.get(field))
        if value is not None:
            return value / 1000.0
    return numeric_value(row.get("time_seconds"))


AUTO_TIME = "auto (Witness uptime)"
TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
# Default time column of a CSV, in order of preference. Witness-clock columns
# come first so flash logs and HITL comparison files line up by themselves;
# sim/wall time files (truth.csv, sent.csv) are aligned with an offset.
TIME_COLUMN_PREFERENCE = ("uptime_ms", "timestamp_us", "sim_s", "sim_ms",
                          "time_seconds", "wall_s")


def infer_time_unit(column: str) -> str:
    """Units implied by a time column's name (seconds when unsure)."""
    name = column.lower()
    if name in ("uptime", "fc_uptime") or name.endswith("_ms") or name == "ms":
        return "ms"
    if name.endswith("_us") or name == "us":
        return "us"
    if name.endswith("_ns") or name == "ns":
        return "ns"
    return "s"


# ---------------------------------------------------------------------------
# Column-wise (numpy) versions of the per-row derivations and summary above.
# load_dataset uses these; the row functions stay for callers with row dicts.
# ---------------------------------------------------------------------------

def _first_of(arrays, n: int) -> np.ndarray:
    """Per row, the first of ``arrays`` that is not NaN (None entries skipped)."""
    out = np.full(n, np.nan)
    for values in arrays:
        if values is None:
            continue
        blank = np.isnan(out)
        out[blank] = values[blank]
    return out


def _scaled(values, factor):
    return None if values is None else values * factor


def _int(values: np.ndarray, modulo: int) -> np.ndarray:
    """int(value) & (modulo - 1) per element, NaN kept (two's complement)."""
    return np.where(np.isnan(values), np.nan, np.mod(np.trunc(values), modulo))


def row_times_array(cols: dict, n: int) -> np.ndarray:
    """row_time_seconds for every row: time_seconds, else uptime/fc_uptime ms."""
    return _first_of((cols.get("time_seconds"), _scaled(cols.get("uptime"), 1e-3),
                      _scaled(cols.get("fc_uptime"), 1e-3)), n)


def sync_times_array(cols: dict, n: int) -> np.ndarray:
    """sync_time_seconds for every row: uptime, else fc_uptime, else time_seconds."""
    return _first_of((_scaled(cols.get("uptime"), 1e-3), _scaled(cols.get("fc_uptime"), 1e-3),
                      cols.get("time_seconds")), n)


def _packet_type(text: dict, n: int) -> np.ndarray:
    values = text.get("packet_type")
    if values is None:
        return csv_columns.TextColumn([""], np.zeros(n, dtype=np.uint16))
    return values.map(lambda value: value.strip().lower())


def status_bytes_array(cols: dict, text: dict, n: int):
    """witness_status_bytes for every row -> (byte0, byte1) float arrays."""
    nan = np.full(n, np.nan)
    byte0 = _int(cols["status_byte0"], 256) if "status_byte0" in cols else nan.copy()
    byte1 = _int(cols["status_byte1"], 256) if "status_byte1" in cols else nan.copy()
    kind = _packet_type(text, n)
    status = cols.get("status", nan)
    fc_status = cols.get("fc_status", nan)
    combined = np.where(kind == "camera", fc_status,
                        np.where(kind == "iris_debug", np.nan, status))
    combined = _int(combined, 65536)
    fill0 = np.isnan(byte0) & ~np.isnan(combined)
    byte0[fill0] = np.floor(combined[fill0] / 256) % 256
    fill1 = np.isnan(byte1) & ~np.isnan(combined)
    byte1[fill1] = combined[fill1] % 256
    return byte0, byte1


def flight_state_array(cols: dict, text: dict, n: int) -> np.ndarray:
    """flight_state() for every row (NaN where it has none)."""
    state = np.full(n, np.nan)
    for name in ("flight_state", "global_flight_state", "status_byte1", "fc_status"):
        values = cols.get(name)
        if values is not None:
            blank = np.isnan(state)
            state[blank] = _int(values, 8)[blank]
    status = cols.get("status")
    if status is not None:
        kind = _packet_type(text, n)
        usable = np.isnan(state) & ~kind.isin(("camera", "iris_debug"))
        state[usable] = _int(status, 8)[usable]
    return state


def _wl_transitions(csv_path: Path):
    """Timestamped state transitions from a WL session's paired .TXT log."""
    text_path = csv_path.with_suffix(".TXT")
    if not text_path.exists():
        text_path = csv_path.with_suffix(".txt")
    if not text_path.exists():
        return None, []
    try:
        log_text = text_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None, []
    transitions = []
    for match in FLIGHT_STATE_TRANSITION_RE.finditer(log_text):
        from_name = " ".join(match.group(2).lower().split())
        to_name = " ".join(match.group(3).lower().split())
        if from_name in FLIGHT_STATE_VALUES and to_name in FLIGHT_STATE_VALUES:
            transitions.append((int(match.group(1)) / 1000.0,
                                FLIGHT_STATE_VALUES[from_name], FLIGHT_STATE_VALUES[to_name]))
    transitions.sort(key=lambda item: item[0])
    return text_path, transitions


def derive_columns(csv_path: Path, cols: dict, text: dict, header: list[str]):
    """add_paired_wl_flight_state, add_derived_flight_state, add_status_bit_traces
    and add_status_flag_columns, on columns. Mutates ``cols``/``text``;
    returns (numeric column order, paired .TXT path or None, row count)."""
    n = len(next(iter(cols.values()))) if cols else 0
    order = [name for name in header if name in cols]

    def add(name):
        if name not in order:
            order.append(name)

    # Paired WL .TXT flight-state transitions (WL v1 has no state column).
    paired = None
    ids = text.get("packet_id")
    kinds = text.get("packet_type")
    is_wl = ((ids is not None and bool(np.any(ids == "WL"))) or
             (kinds is not None and bool(np.any(kinds == "flash_sample"))))
    if is_wl:
        text_path, transitions = _wl_transitions(csv_path)
        state = cols.get("flight_state")
        times = row_times_array(cols, n)
        target = (np.ones(n, bool) if state is None else np.isnan(state)) & ~np.isnan(times)
        if transitions and target.any():
            at = np.array([t for t, _f, _s in transitions])
            to = np.array([s for _t, _f, s in transitions], dtype=np.float64)
            index = np.searchsorted(at, times[target], side="right")
            new_state = np.full(n, np.nan) if state is None else state.copy()
            new_state[target] = np.where(index == 0, transitions[0][1],
                                         to[np.maximum(index - 1, 0)])
            cols["flight_state"] = new_state
            k = len(transitions)
            for name in list(cols):
                cols[name] = np.concatenate([cols[name], np.full(k, np.nan)])
            if "time_seconds" not in cols:
                cols["time_seconds"] = np.full(n + k, np.nan)
            cols["time_seconds"][n:] = np.round(at, 3)
            cols["flight_state"][n:] = to
            for name, value in (("packet_id", "TXT"), ("packet_type", "flight_state_transition")):
                old = text.get(name)
                if old is None:
                    old = csv_columns.TextColumn([""], np.zeros(n, dtype=np.uint16))
                text[name] = old.append(value, k)
            n += k
            paired = text_path
            add("time_seconds")
            add("flight_state")

    # Flight state from status byte 1 (or fc_status / status) where blank.
    derived = flight_state_array(cols, text, n)
    if not np.all(np.isnan(derived)):
        existing = cols.get("flight_state")
        cols["flight_state"] = derived if existing is None else np.where(
            np.isnan(existing), derived, existing)
        add("flight_state")

    # One 0/1 trace per status bit, and gps_locked / hitl_active flags.
    byte0, byte1 = status_bytes_array(cols, text, n)
    for byte, fields in ((byte0, STATUS_BYTE0_BIT_FIELDS), (byte1, STATUS_BYTE1_BIT_FIELDS)):
        have = ~np.isnan(byte)
        if not have.any():
            continue
        ints = np.where(have, byte, 0).astype(np.int64)
        for bit, name in enumerate(fields):
            values = cols.get(name, np.full(n, np.nan)).copy()
            values[have] = (ints[have] >> bit) & 1
            cols[name] = values
    for name in STATUS_BIT_FIELDS:
        if name in cols:
            add(name)
    have1 = ~np.isnan(byte1)
    ints1 = np.where(have1, byte1, 0).astype(np.int64)
    for name, mask in STATUS_FLAG_COLUMNS.items():
        values = cols.get(name, np.full(n, np.nan)).copy()
        fill = np.isnan(values) & have1
        values[fill] = (ints1[fill] & mask) != 0
        if not np.all(np.isnan(values)):
            cols[name] = values
            add(name)
    return order, paired, n


def _sensor_valid(cols: dict, text: dict, n: int, mask: int, explicit: str) -> np.ndarray:
    """sensor_flag_is_valid for every row."""
    valid = np.ones(n, bool)
    decided = np.zeros(n, bool)
    values = cols.get(explicit)
    if values is not None:
        have = ~np.isnan(values)
        valid[have] = np.trunc(values[have]) != 0
        decided |= have
    byte0 = cols.get("status_byte0")
    if byte0 is not None:
        have = ~decided & ~np.isnan(byte0)
        valid[have] = (np.trunc(byte0[have]).astype(np.int64) & mask) != 0
        decided |= have
    kind = _packet_type(text, n)
    combined = np.where(kind == "camera", cols.get("fc_status", np.full(n, np.nan)),
                        cols.get("status", np.full(n, np.nan)))
    have = ~decided & ~np.isnan(combined)
    valid[have] = ((np.trunc(combined[have]).astype(np.int64) >> 8) & mask) != 0
    return valid


def summarize_columns(cols: dict, text: dict, n: int) -> dict[str, str]:
    """calculate_summary on columns (same texts, vectorized)."""
    times = row_times_array(cols, n)
    timed = ~np.isnan(times)
    state = flight_state_array(cols, text, n)

    altitude_text = "Not available"
    altitude = cols.get("barometric_altitude")
    if altitude is not None:
        ok = ~np.isnan(altitude) & _sensor_valid(cols, text, n, 0x40, "barometer_valid")
        if ok.any():
            index = int(np.flatnonzero(ok)[np.argmax(altitude[ok])])
            altitude_text = f"{altitude[index]:.3f} m"
            if timed[index]:
                altitude_text += f"\nat {times[index]:.3f} s"

    magnitudes, labels = [], []
    for prefix, label in ACCELERATION_VECTORS:
        mask, field_name = {"filtered_accel": (0x02, "low_g_valid"),
                            "low_accel": (0x02, "low_g_valid"),
                            "high_accel": (0x20, "high_g_valid")}[prefix]
        parts = [cols.get(f"{prefix}_{axis}") for axis in "xyz"]
        if any(part is None for part in parts):
            magnitudes.append(np.full(n, -np.inf))
        else:
            magnitude = np.sqrt(parts[0] ** 2 + parts[1] ** 2 + parts[2] ** 2)
            ok = ~np.isnan(magnitude) & _sensor_valid(cols, text, n, mask, field_name)
            magnitudes.append(np.where(ok, magnitude, -np.inf))
        labels.append(label)
    stack = np.vstack(magnitudes)
    row_best = stack.argmax(axis=0)
    row_max = stack.max(axis=0)
    has_accel = np.isfinite(row_max)
    acceleration_text = "Not available"
    if has_accel.any():
        index = int(np.argmax(np.where(has_accel, row_max, -np.inf)))
        acceleration_text = f"{row_max[index]:.3f} g"
        if timed[index]:
            acceleration_text += f"\nat {times[index]:.3f} s"
        acceleration_text += f"\n{labels[row_best[index]]}"

    launch_time = None
    launch_method = ""
    boost = timed & (state == 1)
    if boost.any():
        launch_time = float(times[boost].min())
        launch_method = "Boost flight state"
    else:
        sample = has_accel & timed
        order = np.argsort(times[sample], kind="stable")
        sample_times = times[sample][order].tolist()
        sample_mags = row_max[sample][order].tolist()
        streak_start = None
        streak_count = 0
        previous_time = None
        for timestamp, magnitude in zip(sample_times, sample_mags):
            if magnitude < LAUNCH_ACCELERATION_G:
                streak_start = None
                streak_count = 0
                previous_time = timestamp
                continue
            if (previous_time is None
                    or timestamp - previous_time <= LAUNCH_MAX_SAMPLE_GAP_SECONDS):
                if streak_count == 0:
                    streak_start = timestamp
                streak_count += 1
            else:
                streak_start = timestamp
                streak_count = 1
            if streak_count >= LAUNCH_REQUIRED_SAMPLES:
                launch_time = streak_start
                launch_method = f"{LAUNCH_ACCELERATION_G:g} g acceleration threshold"
                break
            previous_time = timestamp
    launch_text = "Not detected"
    if launch_time is not None:
        launch_text = f"{launch_time:.3f} s\n{launch_method}"

    has_state = ~np.isnan(state)
    if has_state.any():
        rows = np.flatnonzero(has_state)
        primary = np.where(timed[rows], times[rows], rows.astype(np.float64))
        order = np.lexsort((rows, primary, ~timed[rows]))
        ordered = state[rows][order].astype(np.int64)
        changes = np.r_[True, ordered[1:] != ordered[:-1]]
        history = ordered[changes].tolist()
        final_state = int(ordered[-1])
        final_name = FLIGHT_STATE_NAMES.get(final_state, f"Reserved ({final_state})")
        history_text = " -> ".join(FLIGHT_STATE_NAMES.get(s, f"Reserved ({s})")
                                   for s in history)
        flight_state_text = f"{final_name} ({final_state})\n{history_text}"
    else:
        flight_state_text = "Not available in this log"

    _byte0, byte1 = status_bytes_array(cols, text, n)
    hitl = cols.get("hitl_active", np.full(n, np.nan)).copy()
    blank = np.isnan(hitl)
    hitl[blank] = np.where(np.isnan(byte1[blank]), 0,
                           np.where(np.isnan(byte1[blank]), 0, byte1[blank]).astype(np.int64)
                           & 0x10)
    hitl_records = int(np.count_nonzero(np.nan_to_num(hitl)))
    return {
        "max_altitude": altitude_text,
        "launch_time": launch_text,
        "max_acceleration": acceleration_text,
        "flight_state": flight_state_text,
        "hitl": (
            f"HITL ACTIVE in {hitl_records:,} record(s): simulated sensor data"
            if hitl_records else "No HITL records"
        ),
    }


@dataclass
class Dataset:
    """One converted CSV, held as numpy columns.

    ``numeric_data`` maps each numeric column (including derived flight
    state / status bits) to a float64 array, NaN where empty. Each CSV has
    its own time axis: ``time_column`` (a numeric column, or AUTO_TIME for
    the Witness uptime fallback chain), ``time_unit`` (a TIME_UNITS key) and
    ``offset`` in seconds, added after unit conversion so files can be slid
    into line with each other.
    """
    path: Path
    label: str
    fieldnames: list[str]
    numeric_columns: list[str]
    numeric_data: dict[str, np.ndarray]
    sync_times: np.ndarray
    row_times: np.ndarray
    packet_type: "csv_columns.TextColumn | None"
    summary: dict[str, str]
    row_count: int
    paired_state_log: Path | None = None
    is_wl: bool = False
    stride: int = 1
    total_rows: int = 0
    offset: float = 0.0
    time_column: str = AUTO_TIME
    time_unit: str = "s"
    _time_cache: tuple | None = field(default=None, repr=False, compare=False)

    @property
    def packet_types(self) -> list[str]:
        if self.packet_type is None:
            return []
        return sorted(value for value in self.packet_type.unique() if value)

    @property
    def has_auto_time(self) -> bool:
        return bool(np.any(~np.isnan(self.sync_times)))

    def time_choices(self) -> list[str]:
        """Columns that can be this CSV's time axis."""
        return ([AUTO_TIME] if self.has_auto_time else []) + list(self.numeric_columns)

    def set_time_axis(self, column: str | None = None, unit: str | None = None,
                      offset: float | None = None) -> None:
        if column is not None:
            if column not in self.time_choices():
                raise ValueError(f"{column!r} is not a numeric column of {self.label}")
            if column != self.time_column and column != AUTO_TIME and unit is None:
                unit = infer_time_unit(column)
            self.time_column = column
        if unit is not None:
            if unit not in TIME_UNITS:
                raise ValueError(f"unknown time unit {unit!r}")
            self.time_unit = unit
        if offset is not None:
            if not math.isfinite(offset):
                raise ValueError("the offset must be a finite number of seconds")
            self.offset = float(offset)

    def times(self) -> np.ndarray:
        """Per-row time in seconds (unit converted, before the offset; NaN = none)."""
        key = (self.time_column, self.time_unit)
        if self._time_cache is not None and self._time_cache[0] == key:
            return self._time_cache[1]
        if self.time_column == AUTO_TIME:
            values = self.sync_times      # already seconds
        else:
            raw = self.numeric_data.get(self.time_column)
            values = (np.full(self.row_count, np.nan) if raw is None
                      else raw * TIME_UNITS[self.time_unit])
        self._time_cache = (key, values)
        return values

    @property
    def has_sync_time(self) -> bool:
        return bool(np.any(~np.isnan(self.times())))

    def time_axis_text(self) -> str:
        if self.time_column == AUTO_TIME:
            text = "uptime auto"
        else:
            text = f"{self.time_column} {self.time_unit}"
        if self.offset:
            text += f", {self.offset:+g} s"
        return text

    def mask_for(self, packet_filter: str) -> np.ndarray | None:
        """Rows of one packet type (None = all rows)."""
        if packet_filter == "All" or self.packet_type is None:
            return None
        return self.packet_type == packet_filter


def default_time_axis(dataset: Dataset) -> tuple[str, str]:
    """(column, unit) a newly opened CSV starts with."""
    columns = set(dataset.numeric_columns)
    if {"uptime", "fc_uptime"} & columns:
        return AUTO_TIME, "s"
    for column in TIME_COLUMN_PREFERENCE:
        if column in columns:
            return column, infer_time_unit(column)
    if dataset.has_auto_time:
        return AUTO_TIME, "s"
    first = dataset.numeric_columns[0]
    return first, infer_time_unit(first)


def load_dataset(path: Path, label: str | None = None, progress=None,
                 cancel=None) -> Dataset:
    """Read a converted CSV into numpy columns (safe to run off the UI thread).

    ``progress(fraction)`` reports parsing progress; setting the ``cancel``
    threading.Event aborts with csv_columns.LoadCancelled. Raises OSError,
    csv.Error or ValueError for unreadable files.
    """
    path = Path(path)
    table = csv_columns.load_table(path, progress=progress, cancel=cancel)
    if not table.columns:
        raise ValueError("the CSV contains no numeric columns to graph")
    cols = dict(table.columns)
    text = dict(table.text)
    order, paired, n = derive_columns(path, cols, text, list(table.header))
    kinds = text.get("packet_type")
    ids = text.get("packet_id")
    dataset = Dataset(
        path=path, label=label or path.name, fieldnames=list(table.header),
        numeric_columns=order, numeric_data={name: cols[name] for name in order},
        sync_times=sync_times_array(cols, n), row_times=row_times_array(cols, n),
        packet_type=kinds,
        summary=summarize_columns(cols, text, n), row_count=n,
        paired_state_log=paired,
        is_wl=ids is not None and bool(np.any(ids == "WL")),
        stride=table.stride, total_rows=table.total_rows)
    dataset.time_column, dataset.time_unit = default_time_axis(dataset)
    return dataset


def unique_label(name: str, taken) -> str:
    """``name``, or ``name (2)``, ``name (3)``... if already used."""
    taken = set(taken)
    if name not in taken:
        return name
    index = 2
    while f"{name} ({index})" in taken:
        index += 1
    return f"{name} ({index})"


class GraphLayout:
    """Which series are drawn on which graph.

    A series is (dataset_id, column). Each graph holds one or more series
    that are overlaid on shared axes. Pure data: no Tk, so it is testable.
    """

    def __init__(self):
        self.graphs: list[list[tuple[int, str]]] = []

    def __len__(self) -> int:
        return len(self.graphs)

    def clear(self) -> None:
        self.graphs = []

    def add(self, refs, target: int | None = None) -> None:
        """Overlay ``refs`` on graph ``target``; None adds one graph each."""
        refs = list(refs)
        if target is None or not 0 <= target < len(self.graphs):
            for ref in refs:
                self.graphs.append([ref])
            return
        graph = self.graphs[target]
        for ref in refs:
            if ref not in graph:
                graph.append(ref)

    def move(self, source: int, ref, target: int | None) -> None:
        """Move one series to another graph (None: a new graph of its own)."""
        if not 0 <= source < len(self.graphs) or ref not in self.graphs[source]:
            return
        if target == source:
            return
        if target is not None and 0 <= target < len(self.graphs) \
                and ref in self.graphs[target]:
            self.remove_series(source, ref)
            return
        self.graphs[source].remove(ref)
        if target is None or not 0 <= target < len(self.graphs):
            self.graphs.append([ref])
        else:
            self.graphs[target].append(ref)
        if not self.graphs[source]:
            del self.graphs[source]

    def remove_series(self, index: int, ref) -> None:
        if 0 <= index < len(self.graphs) and ref in self.graphs[index]:
            self.graphs[index].remove(ref)
            if not self.graphs[index]:
                del self.graphs[index]

    def remove_graph(self, index: int) -> None:
        if 0 <= index < len(self.graphs):
            del self.graphs[index]

    def remove_dataset(self, dataset_id: int) -> None:
        self.graphs = [
            kept for kept in (
                [ref for ref in graph if ref[0] != dataset_id]
                for graph in self.graphs)
            if kept
        ]


class CsvViewerPanel(ttk.Frame):
    """The CSV viewer as an embeddable frame (summary and plot notebooks).

    Several CSVs can be open at once. The trace list groups numeric columns
    by CSV; drag a trace onto a graph to overlay it, or below the last graph
    for a new one. Drag a graph's legend entry onto another graph to move it,
    or back onto the list to remove it. With the default x axis every CSV is
    placed on Witness uptime, so logs of the same session line up.

    ``on_title`` is called with a window title when the files change.
    """

    def __init__(self, master, initial_files=None, *,
                 prompt_on_start: bool = False, on_title=None):
        super().__init__(master)
        self.on_title = on_title
        self.datasets: dict[int, Dataset] = {}
        self._next_dataset_id = 0
        self.layout = GraphLayout()
        self.redraw_job: str | None = None
        self.visible_job: str | None = None
        self._points_cache: dict[tuple, list[tuple[float, float]]] = {}
        self._graphs: list[dict] = []
        self._drawn: set[int] = set()
        self._tops: list[int] = []
        self._graph_width = 0
        self._last_size = (0, 0)
        self._legend_items: dict[int, tuple[int, tuple[int, str]]] = {}
        self._close_items: dict[int, int] = {}
        self._press = None
        self._drag = None
        self._ghost = None
        self._cache_bytes = 0
        self._cache_sizes: dict = {}
        self._load_queue: list[Path] = []
        self._load_events: queue.Queue = queue.Queue()
        self._load_thread: threading.Thread | None = None
        self._load_cancel = threading.Event()
        self._load_job: str | None = None
        self._load_errors: list[str] = []
        self.x_view: tuple[float, float] | None = None   # shared zoom
        self._full_x: tuple[float, float] | None = None
        self._zoom_press = None
        self.font = tkfont.nametofont("TkDefaultFont")
        self.bold_font = tkfont.Font(root=self, font=self.font)
        self.bold_font.configure(weight="bold")

        self.build_ui()
        self.bind("<Destroy>", self._on_destroy, add="+")
        if isinstance(initial_files, (str, Path)):
            initial_files = [initial_files]
        if initial_files:
            files = [Path(item) for item in initial_files]
            self.after(0, lambda: self.add_csvs(files))
        elif prompt_on_start:
            self.after(0, self.choose_csv)

    # ------------------------------------------------------------ compat
    @property
    def csv_path(self) -> Path | None:
        first = next(iter(self.datasets.values()), None)
        return None if first is None else first.path

    @property
    def numeric_columns(self) -> list[str]:
        first = next(iter(self.datasets.values()), None)
        return [] if first is None else first.numeric_columns

    # ---------------------------------------------------------------- UI
    def build_ui(self) -> None:
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")

        ttk.Button(controls, text="Open CSV…", command=self.choose_csv).pack(side="left")
        ttk.Button(controls, text="Add CSV…", command=self.choose_add_csv).pack(
            side="left", padx=(4, 0))
        self.load_cancel_button = ttk.Button(controls, text="Cancel",
                                             command=self.cancel_loads)
        self.load_bar = ttk.Progressbar(controls, length=140, maximum=1.0)
        self.load_label = ttk.Label(controls, text="", width=10)
        self.file_label = ttk.Label(controls, text="No file loaded", anchor="w",
                                    width=10)
        self.file_label.pack(side="left", fill="x", expand=True, padx=10)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self.summary_outer = ttk.Frame(self.notebook)
        self.plot_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.summary_outer, text="Summary")
        self.notebook.add(self.plot_tab, text="Plots")

        self.build_summary_tab()
        self.build_plot_tab()

    def build_summary_tab(self) -> None:
        # The summary scrolls vertically when the window is short; text
        # wraps to the current width.
        canvas = tk.Canvas(self.summary_outer, highlightthickness=0,
                           width=300, height=200)
        scrollbar = ttk.Scrollbar(self.summary_outer, orient="vertical",
                                  command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        self.summary_canvas = canvas
        self.summary_tab = ttk.Frame(canvas, padding=18)
        window = canvas.create_window((0, 0), window=self.summary_tab, anchor="nw")
        self._wrap_labels: list[tuple[ttk.Label, int]] = []

        def fit(_event=None):
            width = max(canvas.winfo_width(), 200)
            canvas.itemconfigure(window, width=width)
            for label, margin in self._wrap_labels:
                label.configure(wraplength=max(150, width - margin))
            canvas.configure(scrollregion=(
                0, 0, width, self.summary_tab.winfo_reqheight()))

        canvas.bind("<Configure>", fit)
        self.summary_tab.bind("<Configure>", lambda _event: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.bind("<MouseWheel>", lambda event: self._wheel(canvas, event))

        header = ttk.Frame(self.summary_tab)
        header.pack(fill="x", pady=(0, 4))
        ttk.Label(
            header,
            text="Flight Summary",
            font=("Segoe UI", 22, "bold"),
        ).pack(side="left")
        self.summary_combo = ttk.Combobox(header, state="readonly", width=24)
        self.summary_combo.pack(side="right", padx=(8, 0))
        self.summary_combo.bind("<<ComboboxSelected>>",
                                lambda _event: self.show_summary())
        ttk.Label(header, text="CSV:").pack(side="right")
        ttk.Label(
            self.summary_tab,
            text="Summary values calculated from the complete converted CSV.",
        ).pack(anchor="w", pady=(0, 20))

        cards = ttk.Frame(self.summary_tab)
        cards.pack(fill="x")
        for column in range(3):
            cards.columnconfigure(column, weight=1, uniform="summary")

        self.max_altitude_var = tk.StringVar(value="--")
        self.launch_time_var = tk.StringVar(value="--")
        self.max_acceleration_var = tk.StringVar(value="--")
        card_values = (
            ("Max barometric altitude", self.max_altitude_var),
            ("Launch detected at", self.launch_time_var),
            ("Max acceleration", self.max_acceleration_var),
        )
        for column, (title, value_var) in enumerate(card_values):
            card = ttk.LabelFrame(cards, text=title, padding=10)
            card.grid(row=0, column=column, sticky="nsew", padx=4)
            ttk.Label(
                card,
                textvariable=value_var,
                anchor="center",
                justify="center",
                font=("Segoe UI", 13, "bold"),
            ).pack(fill="both", expand=True, pady=8)

        self.flight_state_var = tk.StringVar(value="--")
        state_card = ttk.LabelFrame(
            self.summary_tab, text="Flight state (status byte 1 bits 2:0)", padding=12
        )
        state_card.pack(fill="x", padx=4, pady=(14, 0))
        state_label = ttk.Label(
            state_card,
            textvariable=self.flight_state_var,
            anchor="w",
            justify="left",
            font=("Segoe UI", 12, "bold"),
            wraplength=600,
        )
        state_label.pack(fill="x")
        self._wrap_labels.append((state_label, 90))

        self.summary_details_var = tk.StringVar(value="Open a CSV file to begin.")
        details = ttk.Label(
            self.summary_tab,
            textvariable=self.summary_details_var,
            padding=(6, 24, 6, 6),
            wraplength=600,
        )
        details.pack(anchor="w")
        self._wrap_labels.append((details, 60))

        note = ttk.Label(
            self.summary_tab,
            text=(
                "Launch detection uses the first Boost flight state when present. "
                "For older logs without flight state, it requires three consecutive "
                f"samples at or above {LAUNCH_ACCELERATION_G:g} g."
            ),
            wraplength=600,
            foreground="#555555",
        )
        note.pack(anchor="w", pady=8)
        self._wrap_labels.append((note, 60))

    def build_plot_tab(self) -> None:
        controls = ttk.Frame(self.plot_tab, padding=8)
        controls.pack(fill="x")

        ttk.Label(controls, text="X axis:").pack(side="left", padx=(0, 4))
        self.x_axis_combo = ttk.Combobox(controls, width=26, state="readonly")
        self.x_axis_combo.pack(side="left")
        self.x_axis_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_axis_changed())

        ttk.Label(controls, text="Packet type:").pack(side="left", padx=(10, 4))
        self.packet_filter_combo = ttk.Combobox(controls, width=14, state="readonly")
        self.packet_filter_combo.set("All")
        self.packet_filter_combo.pack(side="left")
        self.packet_filter_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self.on_axis_changed()
        )
        # Status line with the graph buttons on its right, so the toolbar
        # above stays narrow enough for small windows.
        status = ttk.Frame(self.plot_tab, padding=(8, 0, 8, 5))
        status.pack(fill="x")
        ttk.Button(status, text="Clear graphs", command=self.clear_graphs).pack(
            side="right", padx=(4, 0))
        ttk.Button(status, text="Reset zoom", command=self.reset_zoom).pack(
            side="right", padx=(4, 0))
        ttk.Button(status, text="Zoom out", command=self.zoom_out).pack(side="right")
        self.summary_label = ttk.Label(status, text="", width=10)
        self.summary_label.pack(side="left", fill="x", expand=True)

        plot_panes = ttk.Panedwindow(self.plot_tab, orient="horizontal")
        plot_panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        trace_panel = ttk.Frame(plot_panes, width=220)
        graph_frame = ttk.Frame(plot_panes)
        plot_panes.add(trace_panel, weight=0)
        plot_panes.add(graph_frame, weight=1)

        ttk.Label(trace_panel, text="Traces", font=("Segoe UI", 11, "bold")).pack(
            anchor="w", padx=4, pady=(2, 2)
        )
        self.help_label = ttk.Label(
            trace_panel, foreground="#555555", justify="left", wraplength=200,
            text=("Drag traces onto a graph to overlay them, or below the "
                  "last graph for a new one. Double-click adds a graph. Drag "
                  "a legend entry to another graph, or back here to remove it.\n"
                  "Zoom (all graphs): drag across a plot, or Ctrl+wheel; "
                  "Shift+wheel pans; double-click a plot to reset. Hover a "
                  "plot for exact values."))
        self.help_label.pack(fill="x", padx=4, pady=(0, 4))
        trace_panel.bind("<Configure>", self._trace_panel_resized)

        buttons = ttk.Frame(trace_panel)
        buttons.pack(fill="x", padx=2, pady=(0, 4))
        ttk.Button(buttons, text="Plot CSV", command=self.plot_selected_dataset).pack(
            side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(buttons, text="Remove CSV", command=self.remove_selected_dataset).pack(
            side="left", fill="x", expand=True, padx=(2, 0))

        # Time axis of the selected CSV: which column is time, its units and
        # an offset (seconds, after unit conversion). Changes redraw live.
        axis = ttk.LabelFrame(trace_panel, text="Time axis", padding=4)
        axis.pack(fill="x", padx=2, pady=(0, 4))
        self.time_axis_frame = axis
        axis.columnconfigure(1, weight=1)
        ttk.Label(axis, text="Column").grid(row=0, column=0, sticky="w")
        self.time_column_combo = ttk.Combobox(axis, state="readonly", width=14)
        self.time_column_combo.grid(row=0, column=1, sticky="ew", pady=1)
        self.time_column_combo.bind("<<ComboboxSelected>>",
                                    lambda _event: self._time_controls_changed())
        ttk.Label(axis, text="Units").grid(row=1, column=0, sticky="w")
        self.time_unit_combo = ttk.Combobox(axis, state="readonly", width=5,
                                            values=list(TIME_UNITS))
        self.time_unit_combo.grid(row=1, column=1, sticky="w", pady=1)
        self.time_unit_combo.bind("<<ComboboxSelected>>",
                                  lambda _event: self._time_controls_changed())
        ttk.Label(axis, text="Offset s").grid(row=2, column=0, sticky="w")
        self.offset_var = tk.StringVar(value="0")
        self.offset_entry = ttk.Spinbox(
            axis, textvariable=self.offset_var, from_=-1e9, to=1e9, increment=0.1,
            width=10, command=self.apply_offset)
        self.offset_entry.grid(row=2, column=1, sticky="ew", pady=1)
        self.offset_entry.bind("<Return>", lambda _event: self.apply_offset())
        self.offset_entry.bind("<FocusOut>", lambda _event: self.apply_offset())
        self.offset_entry.bind("<KeyRelease>", lambda _event: self._offset_typed())
        self._offset_job = None
        nudges = ttk.Frame(axis)
        nudges.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(2, 0))
        for text, delta in (("\u22121", -1.0), ("\u22120.1", -0.1),
                            ("+0.1", 0.1), ("+1", 1.0)):
            ttk.Button(nudges, text=text, width=4,
                       command=lambda d=delta: self.nudge_offset(d)).pack(
                side="left", fill="x", expand=True)
        self.time_axis_note = ttk.Label(axis, text="", foreground="#555555",
                                        justify="left", wraplength=190)
        self.time_axis_note.grid(row=4, column=0, columnspan=2, sticky="w", pady=(2, 0))

        tree_frame = ttk.Frame(trace_panel)
        tree_frame.pack(fill="both", expand=True)
        self.trace_tree = ttk.Treeview(tree_frame, show="tree", selectmode="extended",
                                       height=8)
        self.trace_tree.column("#0", width=200, minwidth=120, stretch=True)
        tree_scroll = ttk.Scrollbar(tree_frame, orient="vertical",
                                    command=self.trace_tree.yview)
        self.trace_tree.configure(yscrollcommand=tree_scroll.set)
        self.trace_tree.pack(side="left", fill="both", expand=True)
        tree_scroll.pack(side="right", fill="y")
        self.trace_tree.bind("<ButtonPress-1>", self._tree_press)
        self.trace_tree.bind("<B1-Motion>", self._tree_motion)
        self.trace_tree.bind("<ButtonRelease-1>", self._tree_release)
        self.trace_tree.bind("<Double-1>", self._tree_double)
        self.trace_tree.bind("<<TreeviewSelect>>",
                             lambda _event: self._sync_time_axis_controls())
        # Kept for callers of the single-file viewer.
        self.trace_vars: dict[str, tk.BooleanVar] = {}

        self.canvas = tk.Canvas(
            graph_frame,
            background="white",
            highlightthickness=1,
            highlightbackground="#b0b0b0",
            width=320,
            height=200,
        )
        self.graph_scrollbar = ttk.Scrollbar(
            graph_frame, orient="vertical", command=self.canvas.yview)

        def on_yscroll(first, last):
            self.graph_scrollbar.set(first, last)
            self.schedule_visible()

        self.canvas.configure(yscrollcommand=on_yscroll)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.graph_scrollbar.pack(side="right", fill="y")

        self.canvas.bind("<Configure>", self.schedule_redraw)
        self.canvas.bind("<ButtonPress-1>", self._canvas_press)
        self.canvas.bind("<B1-Motion>", self._canvas_motion)
        self.canvas.bind("<ButtonRelease-1>", self._canvas_release)
        self.canvas.bind("<MouseWheel>", lambda event: self._wheel(self.canvas, event))
        self.canvas.bind("<Button-4>", lambda event: self._wheel(self.canvas, event, -1))
        self.canvas.bind("<Button-5>", lambda event: self._wheel(self.canvas, event, 1))
        self.canvas.bind("<Control-MouseWheel>", self._zoom_wheel)
        self.canvas.bind("<Shift-MouseWheel>", self._pan_wheel)
        self.canvas.bind("<Double-Button-1>", self._double_click)
        self.canvas.bind("<Motion>", self._hover)
        self.canvas.bind("<Leave>", self.clear_hover)

    def _on_destroy(self, event) -> None:
        if event.widget is not self:
            return
        for job in (self.redraw_job, self.visible_job):
            if job is not None:
                try:
                    self.after_cancel(job)
                except tk.TclError:
                    pass
        self.redraw_job = self.visible_job = None
        self._end_drag()
        self.cancel_loads()

    @staticmethod
    def _wheel(widget, event, direction: int | None = None):
        """Scroll ``widget``; 'break' so an enclosing scroller does not also move."""
        if direction is None:
            if not event.delta:
                return None
            direction = -1 if event.delta > 0 else 1
            amount = max(1, abs(int(event.delta / 120)))
        else:
            amount = 1
        first, last = widget.yview()
        if (direction < 0 and first <= 0.0) or (direction > 0 and last >= 1.0):
            return None  # at the end: let an outer scroller take the wheel
        widget.yview_scroll(direction * amount, "units")
        return "break"

    def _set_title(self) -> None:
        if self.on_title is None:
            return
        names = [dataset.path.name for dataset in self.datasets.values()]
        if not names:
            self.on_title("Horizon CSV Viewer")
        elif len(names) == 1:
            self.on_title(f"Horizon CSV Viewer - {names[0]}")
        else:
            self.on_title(f"Horizon CSV Viewer - {names[0]} + {len(names) - 1} more")

    # ------------------------------------------------------------- files
    def choose_csv(self) -> None:
        filenames = filedialog.askopenfilenames(
            parent=self,
            title="Open converted telemetry CSV(s)",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if filenames:
            self.cancel_loads()
            self.clear_datasets()
            self.add_csvs([Path(name) for name in filenames])

    def choose_add_csv(self) -> None:
        filenames = filedialog.askopenfilenames(
            parent=self,
            title="Add converted telemetry CSV(s)",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if filenames:
            self.add_csvs([Path(name) for name in filenames])

    def load_csv(self, path: Path) -> bool:
        """Replace everything with this one CSV (single-file behaviour)."""
        self.cancel_loads()
        self.clear_datasets()
        return self.add_csv(path)

    @property
    def loading(self) -> bool:
        """True while CSVs are being read in the background."""
        return self._load_thread is not None or bool(self._load_queue)

    def add_csvs(self, paths) -> None:
        """Read CSVs on a worker thread; each is added when it finishes.

        The UI stays responsive (a progress bar and Cancel button show while
        files load) however large the files are.
        """
        self._load_queue.extend(Path(path) for path in paths)
        if self._load_thread is None:
            self._start_next_load()

    def _start_next_load(self) -> None:
        if not self._load_queue:
            self._show_load_progress(None)
            if self._load_errors:
                errors, self._load_errors = self._load_errors, []
                messagebox.showerror("Open CSV", "Could not open:\n\n" + "\n\n".join(errors),
                                     parent=self)
            return
        path = self._load_queue.pop(0)
        self._load_cancel = cancel = threading.Event()
        events = self._load_events

        def work():
            try:
                dataset = load_dataset(
                    path, progress=lambda f: events.put(("progress", path, f)),
                    cancel=cancel)
                events.put(("done", path, dataset))
            except csv_columns.LoadCancelled:
                events.put(("cancelled", path, None))
            except (OSError, csv.Error, ValueError, MemoryError) as exc:
                events.put(("error", path, f"{path}:\n{exc}"))
            except Exception as exc:  # never leave the panel stuck "loading"
                events.put(("error", path, f"{path}:\n{type(exc).__name__}: {exc}"))

        self._load_thread = threading.Thread(target=work, name="csv-load", daemon=True)
        self._show_load_progress(path, 0.0)
        self._load_thread.start()
        if self._load_job is None:
            self._load_job = self.after(80, self._poll_loads)

    def _poll_loads(self) -> None:
        self._load_job = None
        finished = False
        while True:
            try:
                kind, path, payload = self._load_events.get_nowait()
            except queue.Empty:
                break
            if kind == "progress":
                self._show_load_progress(path, payload)
                continue
            finished = True
            self._load_thread = None
            if kind == "done":
                self._add_loaded(payload)
            elif kind == "error":
                self._load_errors.append(payload)
        if finished:
            self._start_next_load()
        if self._load_thread is not None:
            self._load_job = self.after(80, self._poll_loads)

    def cancel_loads(self) -> None:
        """Stop the file being read and drop the queued ones."""
        self._load_queue.clear()
        self._load_cancel.set()

    def wait_for_loads(self, timeout: float = 60.0) -> bool:
        """Pump Tk until queued loads finish (tests and scripts)."""
        import time as _time
        deadline = _time.monotonic() + timeout
        while self.loading and _time.monotonic() < deadline:
            self.update()
            _time.sleep(0.01)
        return not self.loading

    def _show_load_progress(self, path, fraction: float = 0.0) -> None:
        if path is None:
            for widget in (self.load_cancel_button, self.load_bar, self.load_label):
                widget.pack_forget()
            return
        if not self.load_bar.winfo_manager():
            self.load_cancel_button.pack(side="right")
            self.load_bar.pack(side="right", padx=4)
            self.load_label.pack(side="right")
        pending = len(self._load_queue)
        self.load_label.configure(text=f"Loading {Path(path).name}" +
                                  (f" (+{pending})" if pending else ""), width=0)
        self.load_bar["value"] = fraction

    def add_csv(self, path: Path, report: bool = True) -> bool:
        """Read one more CSV now (blocking) and add it alongside the others."""
        try:
            dataset = load_dataset(path)
        except (OSError, csv.Error, ValueError) as exc:
            if not report:
                raise
            messagebox.showerror("Open CSV", f"Could not open {path}:\n{exc}",
                                 parent=self)
            return False
        self._add_loaded(dataset)
        return True

    def _add_loaded(self, dataset: Dataset) -> None:
        dataset.label = unique_label(dataset.path.name,
                                     (d.label for d in self.datasets.values()))
        dataset_id = self._next_dataset_id
        self._next_dataset_id += 1
        first = not self.datasets
        self.datasets[dataset_id] = dataset
        self._insert_tree(dataset_id, dataset)
        self._refresh_axis_choices(keep=not first)
        if first and not self.layout.graphs:
            self.plot_dataset(dataset_id)
        self._refresh_file_label()
        self._refresh_summary_choices(select=dataset_id)
        self._sync_time_axis_controls()
        self._set_title()
        if first:
            self.notebook.select(self.summary_outer)
            self.canvas.yview_moveto(0)
        self.draw_graphs()

    def clear_datasets(self) -> None:
        self.datasets.clear()
        self.x_view = None
        self.layout.clear()
        self._drop_cached()
        self.trace_tree.delete(*self.trace_tree.get_children())
        self._refresh_axis_choices(keep=False)
        self._refresh_file_label()
        self._refresh_summary_choices()
        self._sync_time_axis_controls()
        self.draw_graphs()

    def remove_dataset(self, dataset_id: int) -> None:
        if dataset_id not in self.datasets:
            return
        del self.datasets[dataset_id]
        self.layout.remove_dataset(dataset_id)
        self._drop_cached(dataset_id)
        if self.trace_tree.exists(f"d{dataset_id}"):
            self.trace_tree.delete(f"d{dataset_id}")
        self._refresh_axis_choices(keep=True)
        self._refresh_file_label()
        self._refresh_summary_choices()
        self._sync_time_axis_controls()
        self._set_title()
        self.draw_graphs()

    def _selected_dataset_id(self) -> int | None:
        for item in self.trace_tree.selection() or (self.trace_tree.focus(),):
            ref = self._tree_ref(item)
            if ref is not None:
                return ref[0]
            if item.startswith("d") and item[1:].isdigit():
                return int(item[1:])
        return next(iter(self.datasets), None)

    def remove_selected_dataset(self) -> None:
        dataset_id = self._selected_dataset_id()
        if dataset_id is not None:
            self.remove_dataset(dataset_id)

    def plot_dataset(self, dataset_id: int) -> None:
        """One graph per numeric column of a CSV (the classic view)."""
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        x_column = self.x_axis_combo.get()
        skip = (SYNC_SKIP_COLUMNS | {dataset.time_column} if x_column == SYNC_X
                else {x_column})
        self.layout.add([(dataset_id, column) for column in dataset.numeric_columns
                         if column not in skip])

    def plot_selected_dataset(self) -> None:
        dataset_id = self._selected_dataset_id()
        if dataset_id is not None:
            self.plot_dataset(dataset_id)
            self.draw_graphs()

    def clear_graphs(self) -> None:
        self.layout.clear()
        self.canvas.yview_moveto(0)
        self.draw_graphs()

    # ------------------------------------------------- per-file time axis
    def set_time_axis(self, dataset_id: int, column: str | None = None,
                      unit: str | None = None, offset: float | None = None) -> None:
        """Change one CSV's time column, units and/or offset (seconds).

        The shared x axis, zoom and hover all follow, after a short
        debounced redraw. Raises ValueError for an unknown column or unit.
        """
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        dataset.set_time_axis(column, unit, offset)
        self._drop_cached(dataset_id)
        if self.trace_tree.exists(f"d{dataset_id}"):
            self.trace_tree.item(f"d{dataset_id}", text=self._dataset_text(dataset))
        if self._selected_dataset_id() == dataset_id:
            self._sync_time_axis_controls()
        self.schedule_redraw()

    def apply_offset(self) -> None:
        """Use the offset typed (or spun) for the selected CSV."""
        if self._offset_job is not None:
            self.after_cancel(self._offset_job)
            self._offset_job = None
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        try:
            offset = float(self.offset_var.get())
        except ValueError:
            return  # half-typed ("-", "1e"): keep the last good value
        if not math.isfinite(offset) or offset == dataset.offset:
            return
        self.set_time_axis(dataset_id, offset=offset)

    def _offset_typed(self) -> None:
        # Live while typing, but only once the text has paused.
        if self._offset_job is not None:
            self.after_cancel(self._offset_job)
        self._offset_job = self.after(300, self.apply_offset)

    def nudge_offset(self, delta: float) -> None:
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is not None:
            self.set_time_axis(dataset_id, offset=round(dataset.offset + delta, 9))

    def _time_controls_changed(self) -> None:
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        column = self.time_column_combo.get()
        unit = self.time_unit_combo.get() or None
        if column != dataset.time_column:
            # A new column brings the units its name implies.
            self.set_time_axis(dataset_id, column=column)
        elif unit != dataset.time_unit and column != AUTO_TIME:
            self.set_time_axis(dataset_id, unit=unit)

    def _sync_time_axis_controls(self) -> None:
        """Show the selected CSV's time axis in the controls."""
        dataset = self.datasets.get(self._selected_dataset_id())
        if dataset is None:
            self.time_axis_frame.configure(text="Time axis")
            self.time_column_combo.configure(values=())
            self.time_column_combo.set("")
            self.time_unit_combo.set("")
            self.offset_var.set("0")
            self.time_axis_note.configure(text="")
            return
        self.time_axis_frame.configure(
            text=elide(f"Time axis: {dataset.label}", 220, self.font.measure))
        self.time_column_combo.configure(values=dataset.time_choices())
        self.time_column_combo.set(dataset.time_column)
        auto = dataset.time_column == AUTO_TIME
        self.time_unit_combo.set("s (auto)" if auto else dataset.time_unit)
        self.time_unit_combo.state(["disabled"] if auto else ["!disabled"])
        self.offset_var.set(f"{dataset.offset:g}")
        times = dataset.times()
        timed = times[~np.isnan(times)]
        if timed.size:
            note = (f"{value_text(round(float(timed.min()) + dataset.offset, 6))} to "
                    f"{value_text(round(float(timed.max()) + dataset.offset, 6))} s "
                    "on the shared axis")
        else:
            note = "No values in this column"
        self.time_axis_note.configure(text=note)

    def _trace_panel_resized(self, event) -> None:
        width = max(120, event.width - 10)
        self.help_label.configure(wraplength=width)
        self.time_axis_note.configure(wraplength=max(110, width - 12))

    def _dataset_text(self, dataset: Dataset) -> str:
        return f"{dataset.label}  [{dataset.time_axis_text()}]"

    def _insert_tree(self, dataset_id: int, dataset: Dataset) -> None:
        node = f"d{dataset_id}"
        self.trace_tree.insert("", "end", iid=node, text=self._dataset_text(dataset),
                               open=True)
        for column in dataset.numeric_columns:
            self.trace_tree.insert(node, "end", iid=f"{node}|{column}", text=column)

    def _tree_ref(self, item: str):
        if "|" not in item:
            return None
        node, column = item.split("|", 1)
        try:
            dataset_id = int(node[1:])
        except ValueError:
            return None
        return (dataset_id, column) if dataset_id in self.datasets else None

    def _refresh_file_label(self) -> None:
        paths = [dataset.path for dataset in self.datasets.values()]
        if not paths:
            text = "No file loaded"
        elif len(paths) == 1:
            text = str(paths[0])
        else:
            text = f"{len(paths)} CSVs: " + ", ".join(path.name for path in paths)
        self.file_label.configure(text=text)

    def _refresh_axis_choices(self, keep: bool) -> None:
        columns: list[str] = []
        seen = set()
        for dataset in self.datasets.values():
            for column in dataset.numeric_columns:
                if column not in seen:
                    seen.add(column)
                    columns.append(column)
        has_sync = any(dataset.has_sync_time for dataset in self.datasets.values())
        choices = ([SYNC_X] if has_sync else []) + columns
        self.x_axis_combo["values"] = choices
        current = self.x_axis_combo.get()
        if not (keep and current in choices):
            preferred = next((name for name in (SYNC_X, "time_seconds", "uptime",
                                                "packet_index") if name in choices),
                             choices[0] if choices else "")
            self.x_axis_combo.set(preferred)
        packet_types = sorted({kind for dataset in self.datasets.values()
                               for kind in dataset.packet_types})
        self.packet_filter_combo["values"] = ["All", *packet_types]
        if self.packet_filter_combo.get() not in ("All", *packet_types):
            self.packet_filter_combo.set("All")

    def _drop_cached(self, dataset_id: int | None = None) -> None:
        """Forget cached series (of one CSV, or all)."""
        if dataset_id is None:
            self._points_cache = {}
            self._cache_sizes = {}
            self._cache_bytes = 0
            return
        for key in [k for k in self._points_cache if k[0] == dataset_id]:
            self._points_cache.pop(key)
            self._cache_bytes -= self._cache_sizes.pop(key, 0)

    def on_axis_changed(self) -> None:
        self._drop_cached()
        self.x_view = None
        self.draw_graphs()

    # ----------------------------------------------------------- summary
    def _refresh_summary_choices(self, select: int | None = None) -> None:
        self._summary_ids = list(self.datasets)
        self.summary_combo["values"] = [self.datasets[i].label for i in self._summary_ids]
        if select is not None and select in self.datasets:
            self.summary_combo.set(self.datasets[select].label)
        elif self.summary_combo.get() not in self.summary_combo["values"]:
            self.summary_combo.set(self.datasets[self._summary_ids[0]].label
                                   if self._summary_ids else "")
        self.show_summary()

    def show_summary(self) -> None:
        label = self.summary_combo.get()
        dataset = next((d for d in self.datasets.values() if d.label == label), None)
        if dataset is None:
            for var in (self.max_altitude_var, self.launch_time_var,
                        self.max_acceleration_var, self.flight_state_var):
                var.set("--")
            self.summary_details_var.set("Open a CSV file to begin.")
            return
        summary = dataset.summary
        self.max_altitude_var.set(summary["max_altitude"])
        self.launch_time_var.set(summary["launch_time"])
        self.max_acceleration_var.set(summary["max_acceleration"])
        self.flight_state_var.set(summary["flight_state"])
        times = dataset.row_times[~np.isnan(dataset.row_times)]
        time_span = "unknown"
        if times.size:
            time_span = f"{times.min():.3f} to {times.max():.3f} s"
        details = (
            f"{dataset.path.name}: {dataset.row_count:,} rows | "
            f"{len(dataset.numeric_columns)} numeric fields | time span {time_span}"
        )
        if dataset.stride > 1:
            details += (f" | large file: every {dataset.stride}th of "
                        f"{dataset.total_rows:,} rows loaded (memory cap)")
        if dataset.paired_state_log is not None:
            details += f" | flight state from {dataset.paired_state_log.name}"
        elif dataset.is_wl:
            details += " | WL v1 does not contain flight state"
        if summary["hitl"].startswith("HITL ACTIVE"):
            details += f" | {summary['hitl']}"
        self.summary_details_var.set(details)

    # ------------------------------------------------------------ points
    SERIES_CACHE_BYTES = 768 * 1024 ** 2

    def _row_order(self, dataset_id: int, dataset: Dataset):
        """(sorted x, row indices) for the current x axis and packet filter."""
        x_column = self.x_axis_combo.get()
        packet_filter = self.packet_filter_combo.get()
        key = (dataset_id, x_column, packet_filter, None)
        cached = self._points_cache.get(key)
        if cached is not None:
            return cached
        if x_column == SYNC_X:
            x = dataset.times() + dataset.offset
        else:
            x = dataset.numeric_data.get(x_column)
        if x is None:
            result = (np.empty(0), np.empty(0, dtype=np.int64))
        else:
            keep = ~np.isnan(x)
            mask = dataset.mask_for(packet_filter)
            if mask is not None:
                keep &= mask
            if keep.all() and (x.size < 2 or bool(np.all(x[1:] >= x[:-1]))):
                # Logs are nearly always in time order with every row
                # timed: use the columns as they are (no sort, no copies).
                result = (x, None)
            else:
                rows = np.flatnonzero(keep)
                xs = x[rows]
                if xs.size > 1 and not bool(np.all(xs[1:] >= xs[:-1])):
                    order = np.argsort(xs, kind="stable")
                    rows, xs = rows[order], xs[order]
                result = (xs, rows)
        rows = result[1]
        self._cache_store(key, result, result[0].nbytes + (0 if rows is None else rows.nbytes))
        return result

    def _cache_store(self, key, value, nbytes: int) -> None:
        """Remember a computed series; forget the oldest beyond the budget."""
        self._points_cache[key] = value
        self._cache_sizes[key] = nbytes
        self._cache_bytes += nbytes
        while self._cache_bytes > self.SERIES_CACHE_BYTES and len(self._points_cache) > 1:
            old_key = next(iter(self._points_cache))
            if old_key == key:
                break
            self._points_cache.pop(old_key)
            self._cache_bytes -= self._cache_sizes.pop(old_key, 0)

    def series_points(self, ref) -> Series:
        """One trace as x-sorted arrays for the current x axis and filter."""
        dataset_id, column = ref
        dataset = self.datasets.get(dataset_id)
        key = (dataset_id, self.x_axis_combo.get(), self.packet_filter_combo.get(), column)
        points = self._points_cache.get(key)
        if points is not None:
            return points
        if dataset is None or column not in dataset.numeric_data:
            return Series([], [])
        xs, rows = self._row_order(dataset_id, dataset)
        column_values = dataset.numeric_data[column]
        ys = column_values if rows is None else column_values[rows]
        keep = ~np.isnan(ys)
        if keep.all():
            points = Series(xs, ys)          # shares the arrays
            nbytes = 0 if ys is column_values else ys.nbytes
        else:
            points = Series(xs[keep], ys[keep])
            nbytes = points.xs.nbytes + points.ys.nbytes
        self._cache_store(key, points, nbytes)
        return points

    def series_label(self, ref) -> str:
        dataset = self.datasets.get(ref[0])
        if dataset is None:
            return ref[1]
        if len(self.datasets) == 1:
            return ref[1]
        return f"{dataset.label}: {ref[1]}"

    # ----------------------------------------------------------- drawing
    def schedule_redraw(self, event=None) -> None:
        # A <Configure> with an unchanged size (e.g. a remap) needs no work.
        if event is not None:
            size = (event.width, event.height)
            if size == self._last_size:
                return
            self._last_size = size
        if self.redraw_job is not None:
            self.after_cancel(self.redraw_job)
        self.redraw_job = self.after(80, self.draw_graphs)

    def schedule_visible(self) -> None:
        if self.visible_job is None:
            self.visible_job = self.after_idle(self.draw_visible)

    def _legend_layout(self, labels: list[str], width: int) -> list[tuple[int, int]]:
        """(line, x) of each legend entry, wrapping to the graph width."""
        x, line = 14, 0
        out = []
        for label in labels:
            entry = 18 + min(self.font.measure(label), width - 60)
            if x + entry > width - 40 and x > 14:
                x, line = 14, line + 1
            out.append((line, x))
            x += entry + 14
        return out

    def draw_graphs(self) -> None:
        if self.redraw_job is not None:
            self.after_cancel(self.redraw_job)
        self.redraw_job = None
        canvas = self.canvas
        canvas.delete("all")
        self._graphs = []
        self._drawn = set()
        self._tops = []
        self._legend_items = {}
        self._close_items = {}
        width = canvas.winfo_width()
        if width <= 1:
            width = canvas.winfo_reqwidth()
        view_height = canvas.winfo_height()
        if view_height <= 1:
            view_height = canvas.winfo_reqheight()
        self._graph_width = width
        if not self.datasets:
            canvas.create_text(20, 20, anchor="nw", text="Open a converted CSV to begin.",
                               fill="#555555")
            canvas.configure(scrollregion=(0, 0, width, max(1, view_height)))
            self.summary_label.configure(text="")
            return

        line = self.font.metrics("linespace")
        graphs = []
        for index, refs in enumerate(self.layout.graphs):
            series = [(ref, self.series_points(ref)) for ref in refs]
            labels = [self.series_label(ref) for ref in refs]
            legend = self._legend_layout(labels, width) if len(refs) > 1 else []
            extra = (legend[-1][0] + 1) if legend else 0
            graphs.append({"index": index, "refs": refs, "series": series,
                           "labels": labels, "legend": legend,
                           "extra": extra * line})
        lows = [points[0][0] for graph in graphs for _ref, points in graph["series"]
                if points]
        highs = [points[-1][0] for graph in graphs for _ref, points in graph["series"]
                 if points]
        self._full_x = (min(lows), max(highs)) if lows else None
        if self.x_view is not None and self._full_x is not None:
            low = max(self.x_view[0], self._full_x[0])
            high = min(self.x_view[1], self._full_x[1])
            self.x_view = (low, high) if high > low else None
        elif self._full_x is None:
            self.x_view = None
        base = graph_height_for(len(graphs), view_height - DROP_ZONE_HEIGHT)
        top = 0
        for graph in graphs:
            graph["top"] = top
            graph["height"] = base + graph["extra"]
            self._tops.append(top)
            top += graph["height"]
        self._graphs = graphs
        self._drop_top = top
        drop_height = max(DROP_ZONE_HEIGHT, view_height - top) if graphs else max(
            view_height, DROP_ZONE_HEIGHT)
        canvas.configure(scrollregion=(0, 0, width, top + drop_height))
        x_column = self.x_axis_combo.get()
        zoom_text = ""
        if self.x_view is not None:
            zoom_text = (f" | zoom {value_text(round(self.x_view[0], 6))} to "
                         f"{value_text(round(self.x_view[1], 6))}")
        self.summary_label.configure(text=(
            f"{len(self.datasets)} CSV(s) | {len(graphs)} graph(s), "
            f"{sum(len(g['refs']) for g in graphs)} trace(s) | x = {x_column}"
            + zoom_text))
        self._draw_drop_zone(width, top, drop_height, empty=not graphs)
        self.draw_visible()

    def _draw_drop_zone(self, width, top, height, empty=False, active=False):
        self.canvas.delete("dropzone")
        text = ("Drag traces here from the list to plot them"
                if empty else "Drop a trace here for a new graph")
        self.canvas.create_rectangle(
            10, top + 8, width - 10, top + min(height, DROP_ZONE_HEIGHT) - 8,
            outline="#1565c0" if active else "#bdbdbd", dash=(4, 3),
            fill="#e3f2fd" if active else "", tags=("dropzone",))
        self.canvas.create_text(width / 2, top + min(height, DROP_ZONE_HEIGHT) / 2,
                                text=text, fill="#777777", tags=("dropzone",))

    def draw_visible(self) -> None:
        """Draw only the graphs in (or next to) the visible part of the canvas."""
        self.visible_job = None
        if not self._graphs:
            return
        canvas = self.canvas
        top = canvas.canvasy(0)
        bottom = top + max(canvas.winfo_height(), 1)
        for graph in self._graphs:
            index = graph["index"]
            if index in self._drawn:
                continue
            if graph["top"] + graph["height"] < top - 400 or graph["top"] > bottom + 400:
                continue
            self._drawn.add(index)
            self.draw_graph(graph)

    def series_style(self, graph: dict, series_index: int, ref) -> tuple[str, tuple]:
        """(colour, dash) of one trace: per-graph colour when alone, else
        one colour per trace and one dash pattern per CSV."""
        if len(graph["refs"]) == 1:
            return PLOT_COLORS[graph["index"] % len(PLOT_COLORS)], ()
        return (PLOT_COLORS[series_index % len(PLOT_COLORS)],
                DATASET_DASHES[ref[0] % len(DATASET_DASHES)])

    def x_range(self) -> tuple[float, float] | None:
        """The x range every graph shows: the zoom, else all plotted data."""
        return self.x_view or self._full_x

    def draw_graph(self, graph: dict) -> None:
        canvas = self.canvas
        font = self.font
        line_height = font.metrics("linespace")
        width = max(int(self._graph_width), 200)
        index = graph["index"]
        top = graph["top"]
        bottom = top + graph["height"]
        series = [(ref, points) for ref, points in graph["series"]]
        x_column = self.x_axis_combo.get()
        graph["geom"] = None

        canvas.create_rectangle(4, top + 4, width - 4, bottom - 4,
                                outline="#c8c8c8", fill="#ffffff")
        close = canvas.create_text(width - 14, top + 8 + line_height / 2, anchor="e",
                                   text="✕", fill="#888888",
                                   tags=("close", f"g{index}"))
        self._close_items[close] = index

        # Every graph shares one x range (the zoom, or all plotted data), so
        # the time axes line up and zooming one graph zooms them all.
        x_range = self.x_range()
        if x_range is None:
            x_min, x_max = 0.0, 1.0
        else:
            x_min, x_max = x_range
        if x_max <= x_min:
            x_min -= 0.5
            x_max += 0.5
        shown = [(ref, view_points(points, x_min, x_max)) for ref, points in series]

        # Header: a single trace keeps the name + statistics layout; several
        # traces get a wrapped, colour-keyed legend. Every name is draggable.
        header_y = top + 8 + line_height / 2
        non_empty = [points for _ref, points in shown if points]
        if len(series) == 1:
            ref, points = shown[0]
            name = elide(graph["labels"][0], width - 60, self.bold_font.measure)
            item = canvas.create_text(14, header_y, anchor="w", text=name,
                                      fill="#222222", font=self.bold_font,
                                      tags=("legend",))
            self._legend_items[item] = (index, ref)
            header_lines = 1
            if points:
                stats = (f"n={len(points):,}  latest={points[-1][1]:.6g}  "
                         f"min={points.ys.min():.6g}  max={points.ys.max():.6g}")
                if self.x_view is not None:
                    stats = "in view: " + stats
                if self.bold_font.measure(name) + 24 + font.measure(stats) <= width - 60:
                    canvas.create_text(width - 34, header_y, anchor="e", text=stats,
                                       fill="#444444", font=font)
                else:
                    canvas.create_text(14, header_y + line_height, anchor="w",
                                       text=elide(stats, width - 48, font.measure),
                                       fill="#444444", font=font)
                    header_lines = 2
        else:
            for series_index, ((ref, _points), label, (line, x)) in enumerate(
                    zip(series, graph["labels"], graph["legend"])):
                color, dash = self.series_style(graph, series_index, ref)
                y = header_y + line * line_height
                canvas.create_line(x, y, x + 14, y, fill=color, width=2, dash=dash)
                item = canvas.create_text(
                    x + 18, y, anchor="w", fill=color, font=font, tags=("legend",),
                    text=elide(label, width - 60, font.measure))
                self._legend_items[item] = (index, ref)
            header_lines = (graph["legend"][-1][0] + 1) if graph["legend"] else 1

        plot_top = top + 10 + header_lines * line_height + 6
        plot_bottom = bottom - (2 * line_height + 12)
        plot_height = plot_bottom - plot_top
        if not non_empty:
            message = ("No values in this time range" if self.x_view is not None
                       and any(points for _ref, points in series)
                       else f"No values for x = {x_column}")
            canvas.create_text(width / 2, (plot_top + plot_bottom) / 2,
                               text=message, fill="#777777")
            return

        # The y axis fits the data in view, so zooming in on time also
        # zooms in on the values.
        data_min = float(min(points.ys.min() for points in non_empty))
        data_max = float(max(points.ys.max() for points in non_empty))
        if all(ref[1] in BINARY_COLUMNS for ref, _points in series):
            y_step, y_ticks = 1.0, [0.0, 1.0]
            y_start, y_end = -0.1, 1.1
        else:
            if data_max <= data_min:
                padding = max(0.5, abs(data_min) * 0.05)
                low, high = data_min - padding, data_max + padding
            else:
                low, high = data_min, data_max
            target = max(2, min(8, int(plot_height // (line_height * 1.6))))
            y_step, y_ticks = nice_ticks(low, high, target)
            y_start, y_end = y_ticks[0], y_ticks[-1]
            if y_end <= y_start:
                y_end = y_start + y_step
        y_labels = [tick_text(value, y_step) for value in y_ticks]
        left = max(48, max(font.measure(text) for text in y_labels) + 18)
        right = width - 16
        if plot_height < 16 or right - left < 40:
            return

        def to_y(value: float) -> float:
            return plot_bottom - (value - y_start) / (y_end - y_start) * plot_height

        last_label_y = None
        for value, text in zip(y_ticks, y_labels):
            y = to_y(value)
            canvas.create_line(left, y, right, y, fill="#e8e8e8")
            if last_label_y is None or abs(last_label_y - y) >= line_height:
                canvas.create_text(left - 8, y, anchor="e", text=text,
                                   fill="#555555", font=font)
                last_label_y = y

        x_span = x_max - x_min
        x_target = max(2, min(10, int((right - left) // 90)))
        x_step, x_ticks = nice_ticks(x_min, x_max, x_target, inside=True)
        label_right = -1e9
        for value in x_ticks:
            x = left + (value - x_min) / x_span * (right - left)
            canvas.create_line(x, plot_top, x, plot_bottom, fill="#efefef")
            text = tick_text(value, x_step)
            half = font.measure(text) / 2
            # Skip a label that would overlap its neighbour or the edge.
            if x - half >= label_right + 8 and x + half <= width - 6:
                canvas.create_text(x, plot_bottom + 4, anchor="n", text=text,
                                   fill="#555555", font=font)
                label_right = x + half
        canvas.create_rectangle(left, plot_top, right, plot_bottom,
                                outline="#b8b8b8")

        styles = []
        for series_index, (ref, points) in enumerate(shown):
            color, dash = self.series_style(graph, series_index, ref)
            styles.append((color, dash))
            if not points:
                continue
            # Min/max envelope per pixel column of the visible span, so a
            # trace of millions of samples is drawn with ~2 points per pixel.
            rx, ry = envelope(points.xs, points.ys, x_min, x_max, int(right - left))
            coordinates = np.column_stack((
                left + (rx - x_min) / x_span * (right - left), to_y(ry))).ravel().tolist()
            if len(coordinates) >= 4:
                canvas.create_line(*coordinates, fill=color, width=1.5, dash=dash)
            else:
                x, y = coordinates
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)

        canvas.create_text(
            (left + right) / 2, plot_bottom + 6 + line_height,
            anchor="n", text=elide(x_column, right - left, font.measure),
            fill="#444444", font=font,
        )
        # Geometry for hover readouts and zoom selection.
        graph["geom"] = {
            "left": left, "right": right, "top": plot_top, "bottom": plot_bottom,
            "x_min": x_min, "x_max": x_max, "y_start": y_start, "y_end": y_end,
            "styles": styles,
        }

    # ------------------------------------------------------ zoom & hover
    def plot_hit(self, canvas_x: float, canvas_y: float):
        """(graph, geom) when the point is inside a drawn graph's plot area."""
        index = self.graph_at(canvas_y)
        if index is None:
            return None
        graph = self._graphs[index]
        geom = graph.get("geom")
        if (geom is None or not geom["left"] <= canvas_x <= geom["right"]
                or not geom["top"] <= canvas_y <= geom["bottom"]):
            return None
        return graph, geom

    @staticmethod
    def x_at(geom: dict, canvas_x: float) -> float:
        fraction = (canvas_x - geom["left"]) / (geom["right"] - geom["left"])
        return geom["x_min"] + fraction * (geom["x_max"] - geom["x_min"])

    def set_x_view(self, view: tuple[float, float] | None) -> None:
        """Zoom every graph to x in [low, high]; None shows everything."""
        full = self._full_x
        if view is not None and full is not None:
            low, high = sorted(view)
            low, high = max(low, full[0]), min(high, full[1])
            if high - low <= 0 or (low <= full[0] and high >= full[1]):
                view = None
            else:
                view = (low, high)
        self.x_view = view
        self.draw_graphs()

    def zoom(self, factor: float, center: float | None = None) -> None:
        full = self._full_x
        if full is None:
            return
        view = self.x_view or full
        if center is None:
            center = (view[0] + view[1]) / 2
        self.x_view = zoom_range(view, full, center, factor)
        self.draw_graphs()

    def zoom_out(self) -> None:
        self.zoom(2.0)

    def reset_zoom(self) -> None:
        self.set_x_view(None)

    def hover_readout(self, canvas_x: float, canvas_y: float) -> list[str] | None:
        """Text of the hover box at a canvas point (None outside a plot)."""
        rows = self._readout_rows(canvas_x, canvas_y)
        return None if rows is None else [text for text, _color in rows]

    def _readout_rows(self, canvas_x: float, canvas_y: float):
        """(text, colour) rows: the cursor x, then each trace's nearest sample."""
        hit = self.plot_hit(canvas_x, canvas_y)
        if hit is None:
            return None
        graph, geom = hit
        x_value = self.x_at(geom, canvas_x)
        rows = [(f"{self.x_axis_combo.get()}: {x_value:.6g}", "#222222")]
        for (ref, points), (color, _dash) in zip(graph["series"], geom["styles"]):
            # A log that has ended (or not started) at this time is left out
            # rather than showing its last sample from somewhere else.
            if not points or not points[0][0] <= x_value <= points[-1][0]:
                continue
            sample = nearest_point(points, x_value)
            rows.append((f"{self.series_label(ref)} = {value_text(sample[1])}"
                         f"  @ {value_text(sample[0])}", color))
        return rows

    def clear_hover(self, _event=None) -> None:
        self.canvas.delete("hover")

    def _hover(self, event) -> None:
        canvas = self.canvas
        canvas.delete("hover")
        if self._drag is not None or self._zoom_press is not None:
            return
        cx, cy = canvas.canvasx(event.x), canvas.canvasy(event.y)
        hit = self.plot_hit(cx, cy)
        if hit is None:
            return
        hovered, geom = hit
        x_value = self.x_at(geom, cx)
        hover = {"tags": ("hover",), "state": "disabled"}
        # A synced crosshair and the nearest sample of every trace, in every
        # drawn graph.
        for graph in self._graphs:
            g = graph.get("geom")
            if g is None or graph["index"] not in self._drawn:
                continue
            span = g["x_max"] - g["x_min"]
            if not g["x_min"] <= x_value <= g["x_max"]:
                continue
            x = g["left"] + (x_value - g["x_min"]) / span * (g["right"] - g["left"])
            canvas.create_line(x, g["top"], x, g["bottom"], fill="#9e9e9e",
                               dash=(3, 3), **hover)
            for (ref, points), (color, _dash) in zip(graph["series"], g["styles"]):
                if not points or not points[0][0] <= x_value <= points[-1][0]:
                    continue
                sample = nearest_point(points, x_value)
                if not g["x_min"] <= sample[0] <= g["x_max"]:
                    continue
                sx = g["left"] + (sample[0] - g["x_min"]) / span * (g["right"] - g["left"])
                height = g["bottom"] - g["top"]
                sy = g["bottom"] - (sample[1] - g["y_start"]) / (
                    g["y_end"] - g["y_start"]) * height
                if g["top"] - 1 <= sy <= g["bottom"] + 1:
                    canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, outline=color,
                                       width=2, fill="white", tags=("hover", "hovermark"),
                                       state="disabled")
        rows = self._readout_rows(cx, cy) or []
        if not rows:
            return
        line_height = self.font.metrics("linespace")
        box_width = max(self.font.measure(text) for text, _color in rows) + 12
        box_height = line_height * len(rows) + 8
        x = cx + 16
        if x + box_width > geom["right"]:
            x = cx - 16 - box_width
        x = max(4, x)
        y = min(max(cy - box_height / 2, hovered["top"] + 4),
                hovered["top"] + hovered["height"] - box_height - 4)
        canvas.create_rectangle(x, y, x + box_width, y + box_height, fill="#fffff4",
                                outline="#9e9e9e", **hover)
        for row, (text, color) in enumerate(rows):
            canvas.create_text(x + 6, y + 4 + row * line_height, anchor="nw", text=text,
                               fill=color, font=self.font, **hover)
        canvas.tag_raise("hovermark")  # sample markers stay visible over the box

    def _zoom_wheel(self, event) -> str:
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        hit = self.plot_hit(cx, cy)
        center = None if hit is None else self.x_at(hit[1], cx)
        if event.delta:
            self.zoom(ZOOM_FACTOR if event.delta > 0 else 1 / ZOOM_FACTOR, center)
        return "break"

    def _pan_wheel(self, event) -> str:
        view, full = self.x_view, self._full_x
        if view is not None and full is not None and event.delta:
            step = (view[1] - view[0]) * PAN_FRACTION
            self.x_view = pan_range(view, full, -step if event.delta > 0 else step)
            self.draw_graphs()
        return "break"

    def _double_click(self, event):
        if self.plot_hit(self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)):
            self.reset_zoom()
            return "break"
        return None

    # -------------------------------------------------------- drag & drop
    def graph_at(self, canvas_y: float) -> int | None:
        """Graph index at a canvas y, or None for the new-graph area."""
        index = bisect.bisect_right(self._tops, canvas_y) - 1
        if 0 <= index < len(self._graphs):
            graph = self._graphs[index]
            if canvas_y < graph["top"] + graph["height"]:
                return index
        return None

    def drop_target(self, x_root: int, y_root: int):
        """('graph', i) / ('new',) over the plots, ('list',) over the trace
        list, else None."""
        try:
            widget = self.winfo_containing(x_root, y_root)
        except (KeyError, tk.TclError):
            return None
        if widget is self.canvas:
            canvas_y = self.canvas.canvasy(y_root - self.canvas.winfo_rooty())
            index = self.graph_at(canvas_y)
            return ("new",) if index is None else ("graph", index)
        if widget is self.trace_tree:
            return ("list",)
        return None

    def drop(self, payload, target) -> None:
        """Apply a finished drag (also used by tests)."""
        if target is None:
            return
        if payload[0] == "traces":
            refs = payload[1]
            if target[0] == "graph":
                self.layout.add(refs, target[1])
            elif target[0] == "new":
                self.layout.add(refs)
            else:
                return
        elif payload[0] == "legend":
            source, ref = payload[1], payload[2]
            if target[0] == "graph":
                self.layout.move(source, ref, target[1])
            elif target[0] == "new":
                self.layout.move(source, ref, None)
            elif target[0] == "list":
                self.layout.remove_series(source, ref)
        self.draw_graphs()

    def _begin_drag(self, payload, text, event) -> None:
        self._drag = payload
        ghost = tk.Toplevel(self)
        ghost.overrideredirect(True)
        try:
            ghost.attributes("-topmost", True)
        except tk.TclError:
            pass
        tk.Label(ghost, text=text, background="#fffde7", relief="solid",
                 borderwidth=1, padx=4, pady=1).pack()
        ghost.geometry(f"+{event.x_root + 14}+{event.y_root + 10}")
        self._ghost = ghost

    def _drag_motion(self, event) -> None:
        if self._ghost is not None:
            self._ghost.geometry(f"+{event.x_root + 14}+{event.y_root + 10}")
        self.canvas.delete("drophint")
        target = self.drop_target(event.x_root, event.y_root)
        width = max(self._graph_width, 200)
        if target and target[0] == "graph":
            graph = self._graphs[target[1]]
            self.canvas.create_rectangle(
                2, graph["top"] + 2, width - 2, graph["top"] + graph["height"] - 2,
                outline="#1565c0", width=3, tags=("drophint",))
        self._draw_drop_zone(width, self._drop_top,
                             DROP_ZONE_HEIGHT, empty=not self._graphs,
                             active=bool(target and target[0] == "new"))
        # Auto-scroll when dragging near the top/bottom edge.
        if target and target[0] in ("graph", "new"):
            y = event.y_root - self.canvas.winfo_rooty()
            if y < 20:
                self.canvas.yview_scroll(-1, "units")
            elif y > self.canvas.winfo_height() - 20:
                self.canvas.yview_scroll(1, "units")

    def _end_drag(self) -> None:
        self._drag = None
        self._press = None
        if self._ghost is not None:
            try:
                self._ghost.destroy()
            except tk.TclError:
                pass
            self._ghost = None
        try:
            self.canvas.delete("drophint")
        except tk.TclError:
            pass

    def _finish_drag(self, event) -> None:
        payload = self._drag
        target = self.drop_target(event.x_root, event.y_root)
        self._end_drag()
        if payload is not None:
            self.drop(payload, target)

    def _moved_enough(self, event) -> bool:
        x, y = self._press[0], self._press[1]
        return abs(event.x_root - x) + abs(event.y_root - y) >= DRAG_THRESHOLD_PX

    def _tree_press(self, event):
        item = self.trace_tree.identify_row(event.y)
        self._press = (event.x_root, event.y_root, item, self.trace_tree.selection())

    def _tree_motion(self, event):
        if self._drag is not None:
            self._drag_motion(event)
            return "break"
        if self._press is None or not self._moved_enough(event):
            return None
        _x, _y, item, selection = self._press
        if item in selection:
            refs = [ref for ref in map(self._tree_ref, selection) if ref is not None]
        else:
            ref = self._tree_ref(item)
            refs = [] if ref is None else [ref]
        if not refs:
            return None
        text = (self.series_label(refs[0]) if len(refs) == 1
                else f"{len(refs)} traces")
        self._begin_drag(("traces", refs), text, event)
        self._drag_motion(event)
        return "break"

    def _tree_release(self, event):
        if self._drag is not None:
            self._finish_drag(event)
            return "break"
        self._press = None
        return None

    def _tree_double(self, event):
        ref = self._tree_ref(self.trace_tree.identify_row(event.y))
        if ref is not None:
            self.layout.add([ref])
            self.draw_graphs()
            self.canvas.yview_moveto(1.0)
            return "break"
        return None

    def _canvas_press(self, event):
        self._press = None
        self._zoom_press = None
        current = self.canvas.find_withtag("current")
        item = current[0] if current else None
        if item not in self._close_items and item not in self._legend_items:
            cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
            hit = self.plot_hit(cx, cy)
            if hit is not None:
                self.clear_hover()
                self._zoom_press = (cx, hit[0])
            return
        if item in self._close_items:
            self.layout.remove_graph(self._close_items[item])
            self.draw_graphs()
            return
        if item in self._legend_items:
            self._press = (event.x_root, event.y_root, item, ())

    def _canvas_motion(self, event):
        if self._zoom_press is not None:
            start, graph = self._zoom_press
            geom = graph.get("geom")
            self.canvas.delete("zoomband")
            if geom is not None:
                x = min(max(self.canvas.canvasx(event.x), geom["left"]), geom["right"])
                for g in self._graphs:   # show the span on every graph
                    gg = g.get("geom")
                    if gg is not None and g["index"] in self._drawn:
                        self.canvas.create_rectangle(
                            start, gg["top"], x, gg["bottom"], fill="#bbdefb",
                            outline="#1565c0", stipple="gray25",
                            tags=("zoomband",), state="disabled")
            return
        if self._drag is not None:
            self._drag_motion(event)
            return
        if self._press is None or not self._moved_enough(event):
            return
        source, ref = self._legend_items.get(self._press[2], (None, None))
        if ref is None:
            return
        self._begin_drag(("legend", source, ref), self.series_label(ref), event)
        self._drag_motion(event)

    def _canvas_release(self, event):
        if self._zoom_press is not None:
            start, graph = self._zoom_press
            self._zoom_press = None
            self.canvas.delete("zoomband")
            geom = graph.get("geom")
            end = self.canvas.canvasx(event.x)
            if geom is not None and abs(end - start) >= MIN_ZOOM_DRAG_PX:
                end = min(max(end, geom["left"]), geom["right"])
                self.set_x_view((self.x_at(geom, start), self.x_at(geom, end)))
            return
        if self._drag is not None:
            self._finish_drag(event)
        self._press = None


class CsvGraphViewer(tk.Tk):
    """Standalone window around CsvViewerPanel (python viewer.py)."""

    def __init__(self, initial_file=None):
        super().__init__()
        self.title("Horizon CSV Viewer")
        width = min(1100, self.winfo_screenwidth() - 40)
        height = min(800, self.winfo_screenheight() - 80)
        self.geometry(f"{width}x{height}")
        self.minsize(520, 360)
        self.panel = CsvViewerPanel(self, initial_file,
                                    prompt_on_start=not initial_file,
                                    on_title=self.title)
        self.panel.pack(fill="both", expand=True)

    def load_csv(self, path: Path) -> bool:
        return self.panel.load_csv(path)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Graph and overlay numeric columns of converted Horizon CSV files."
    )
    parser.add_argument("csv_files", nargs="*", type=Path,
                        help="converted CSV file(s); several are overlaid on Witness uptime")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    CsvGraphViewer(args.csv_files or None).mainloop()


if __name__ == "__main__":
    main()
