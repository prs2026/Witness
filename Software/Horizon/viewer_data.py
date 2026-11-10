"""Data side of the Horizon CSV viewer: loading, time axes, derivations.

Everything here is free of any GUI toolkit, so it runs on worker threads
and in tests. viewer.py is the PyQt6 / pyqtgraph user interface on top.

* ``load_dataset`` reads a converted CSV, any other logger's CSV (any_csv)
  or a raw Witness ``.BIN`` log (decoded by converter.py) into a
  ``Dataset`` of numpy float64 columns, after a memory check
  (``estimate_load`` / ``MemoryBudgetError``) made before anything big is
  allocated.
* Per-file time axes (``Dataset.set_time_axis``), the Witness status-byte
  derivations and the flight summary are computed column-wise with numpy.
* ``series_arrays`` gives one trace's x-sorted (x, y) arrays for a chosen
  x axis and packet filter; ``compute_spectrogram`` wraps spectrum.py for
  the spectrogram window.
"""

from __future__ import annotations

import bisect
import csv
import os
from dataclasses import dataclass, field
import math
import re
from pathlib import Path

import numpy as np

import any_csv
import converter
import csv_columns
import spectrum


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

        altitude = numeric_value(row.get("kalman_altitude"))
        if altitude is None:  # CSVs from before the Kalman altitude field
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
CSV_FILETYPES = (("CSV / text logs", "*.csv *.CSV *.txt *.tsv"), ("All files", "*.*"))
SYNC_SKIP_COLUMNS = frozenset(("uptime", "fc_uptime", "time_seconds", "uptime_ms",
                               "timestamp_us", "sim_s", "sim_ms", "wall_s"))
DATASET_DASHES = ((), (7, 3), (2, 2), (9, 3, 2, 3), (4, 4))
DROP_ZONE_HEIGHT = 64
DRAG_THRESHOLD_PX = 6
ZOOM_FACTOR = 0.8          # one Ctrl+wheel step in; its inverse zooms out
PAN_FRACTION = 0.1         # one Shift+wheel step pans this share of the view
MIN_ZOOM_DRAG_PX = 5       # a shorter drag across a plot is just a click
TEXT_READOUT_COLUMNS = 4   # text columns of a generic CSV shown on hover


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
ROW_INDEX = "row index"     # time axis of last resort: 0, 1, 2... per row
TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
# Default time column of a CSV, in order of preference. Witness-clock columns
# come first so flash logs and HITL comparison files line up by themselves;
# sim/wall time files (truth.csv, sent.csv) are aligned with an offset.
TIME_COLUMN_PREFERENCE = ("uptime_ms", "timestamp_us", "sim_s", "sim_ms",
                          "time_seconds", "wall_s")


def infer_time_unit(column: str) -> str:
    """Units implied by a time column's name (seconds when unsure).

    A unit in the header wins (``time (ms)``, ``Flight_Time_(s)``).
    """
    from_header = any_csv.time_unit_of(column)
    if from_header is not None:
        return from_header
    name = column.lower()
    if name in ("cts", "milliseconds", "millis", "millisecond"):
        return "ms"
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


def altitude_column(cols: dict, n: int) -> np.ndarray | None:
    """Per-row altitude for the summary: kalman_altitude (Witness now sends
    the Kalman altitude above the pad in the 0x02/0xE0 slot), falling back
    to barometric_altitude for older CSVs and rows without it."""
    kalman = cols.get("kalman_altitude")
    baro = cols.get("barometric_altitude")
    if kalman is None:
        return baro
    if baro is None:
        return kalman
    return np.where(np.isnan(kalman), baro, kalman)


def summarize_columns(cols: dict, text: dict, n: int) -> dict[str, str]:
    """calculate_summary on columns (same texts, vectorized)."""
    times = row_times_array(cols, n)
    timed = ~np.isnan(times)
    state = flight_state_array(cols, text, n)

    altitude_text = "Not available"
    altitude = altitude_column(cols, n)
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
    source_format: str = ""          # set for raw .bin logs decoded here
    discarded: int = 0               # .bin bytes the decoder could not use
    offset: float = 0.0
    time_column: str = AUTO_TIME
    time_unit: str = "s"
    # Generic (non-converter) CSVs: column -> "hex" / "tod" / "datetime" for
    # numeric columns decoded from text, the text columns kept for the hover
    # readout, and how the file was read.
    kinds: dict = field(default_factory=dict)
    text_columns: dict = field(default_factory=dict)
    format_note: str = ""
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
        return (([AUTO_TIME] if self.has_auto_time else []) + list(self.numeric_columns)
                + [ROW_INDEX])

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
        elif self.time_column == ROW_INDEX and ROW_INDEX not in self.numeric_data:
            values = np.arange(self.row_count, dtype=np.float64) * TIME_UNITS[self.time_unit]
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
    # Other loggers: Flight_Time_(s), time, t, timestamp, ms, GoPro cts...
    picked = any_csv.pick_time_column(dataset.numeric_columns, dataset.kinds)
    if picked is not None:
        return picked
    return ROW_INDEX, "s"


def load_dataset(path: Path, label: str | None = None, progress=None,
                 cancel=None, hex_as_numbers: bool = True, *, columns=None,
                 memory_budget: int | None = None, thin: bool = False) -> Dataset:
    """Read a CSV into numpy columns (safe to run off the UI thread).

    Converted CSVs (plain comma-separated, unique header) are read by
    csv_columns; any other CSV, from another flight computer or a GoPro
    export for example, by any_csv. That loader works out the delimiter,
    metadata lines, missing header, units row, duplicate names, hex,
    time-of-day and ISO datetime columns. ``hex_as_numbers`` plots hex
    columns as numbers; otherwise they are text for the readout only.

    Memory guard: before reading, the arrays the file needs are estimated
    (rows x numeric columns x 8 bytes, plus derived columns). Above
    ``memory_budget`` (default: ``default_memory_budget()``) this raises
    MemoryBudgetError without allocating, unless ``columns`` (a subset of
    the numeric columns to read) brings it under, or ``thin`` is set, which
    keeps every Nth row so the arrays fit (CSV files only).

    ``progress(fraction)`` reports parsing progress; setting the ``cancel``
    threading.Event aborts with csv_columns.LoadCancelled. Raises OSError,
    csv.Error or ValueError for unreadable files.
    """
    path = Path(path)
    budget = default_memory_budget() if memory_budget is None else int(memory_budget)
    estimate = estimate_load(path, hex_as_numbers, columns)
    max_bytes = csv_columns.MAX_TABLE_BYTES
    if estimate.bytes > budget:
        if not (thin and estimate.thinnable):
            raise MemoryBudgetError(path, estimate, budget)
        # Thin rows so the numeric arrays and their derived columns fit.
        share = max(1, estimate.n_columns) / max(1, estimate.n_columns + estimate.extra)
        max_bytes = max(1, int(budget * share / PEAK_FACTOR))
    max_bytes = min(max_bytes, csv_columns.MAX_TABLE_BYTES)
    if is_bin_path(path):
        return load_bin_dataset(path, label, progress, cancel, columns=columns)
    sniffed = any_csv.sniff(path)
    if sniffed.is_plain:
        table = csv_columns.load_table(path, progress=progress, cancel=cancel,
                                       max_bytes=max_bytes, columns=columns)
    else:
        table = any_csv.load(path, progress=progress, cancel=cancel,
                             hex_as_numbers=hex_as_numbers, sniffed=sniffed,
                             max_bytes=max_bytes, columns=columns)
    if not table.columns:
        raise ValueError("the CSV contains no numeric columns to graph")
    return dataset_from_table(path, table, label)


def is_bin_path(path) -> bool:
    return Path(path).suffix.lower() == ".bin"


def load_bin_dataset(path: Path, label: str | None = None, progress=None,
                     cancel=None, columns=None) -> Dataset:
    """Decode a raw Witness log (.bin) straight into a dataset.

    Uses converter.py's decoders (I*.BIN IMU records, P*.BIN packet log, WL
    flash records, else the wire packet scanner), so the columns, names and
    time-axis defaults are exactly those of the converted CSV. Rows are
    turned into numpy columns as they are decoded and never kept.
    """
    path = Path(path)
    data = path.read_bytes()
    if not data:
        raise ValueError("the file is empty")
    fmt, decoder = converter.select_decoder(path, data)
    stats = converter.DecodeStats()
    table = csv_columns.table_from_rows(path, decoder(data, stats), converter.BASE_COLUMNS,
                                        progress=progress, cancel=cancel, size=len(data),
                                        columns=columns)
    if table.row_count == 0 or not table.columns:
        if path.name.upper().startswith("U"):
            raise ValueError(
                "U*.BIN is the raw GPS (NMEA) ring; converter.py does not decode it "
                "into records, so there is nothing to plot")
        raise ValueError(
            f"no records could be decoded (tried: {fmt}; {len(data):,} bytes not "
            "recognised). Is this a Witness I*/P*/S*.BIN log or packet capture?")
    dataset = dataset_from_table(path, table, label)
    dataset.source_format = fmt
    dataset.discarded = stats.discarded
    return dataset


def dataset_from_table(path: Path, table, label: str | None = None) -> Dataset:
    """Derived columns, summary and time axis for a loaded table."""
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
        stride=table.stride, total_rows=table.total_rows,
        kinds=dict(getattr(table, "kinds", {})),
        text_columns={name: column for name, column in text.items()
                      if isinstance(table, any_csv.GenericTable)},
        format_note="; ".join(part for part in (getattr(table, "format", ""),
                                                *getattr(table, "notes", ())) if part))
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



# ---------------------------------------------------------------------------
# Memory guard: estimate what a file needs before allocating anything.
# ---------------------------------------------------------------------------

FALLBACK_MEMORY_BUDGET = 2 * 1024 ** 3     # without psutil
MEMORY_BUDGET_FRACTION = 0.25              # of physical RAM
PEAK_FACTOR = 1.5                          # concatenation / derivation peaks
BASE_EXTRA_ARRAYS = 4                      # sync/row times, the plot x axis, temporaries
STATUS_EXTRA_ARRAYS = len(STATUS_BIT_FIELDS) + len(STATUS_FLAG_COLUMNS) + 1
STATUS_SOURCE_COLUMNS = frozenset(("status", "fc_status", "status_byte0", "status_byte1"))
BIN_SAMPLE_ROWS = 4000


def default_memory_budget() -> int:
    """Bytes a single file may use: 25% of physical RAM (psutil), else 2 GB."""
    try:
        import psutil
        return int(psutil.virtual_memory().total * MEMORY_BUDGET_FRACTION)
    except Exception:  # psutil missing or unusable
        return FALLBACK_MEMORY_BUDGET


@dataclass
class LoadEstimate:
    """What loading a file would allocate (worked out from its head only)."""
    rows: int
    columns: list            # the numeric (plottable) columns that would be read
    extra: int               # derived / helper arrays of the same length
    thinnable: bool          # the loader can keep every Nth row instead

    @property
    def n_columns(self) -> int:
        return len(self.columns)

    @property
    def bytes(self) -> int:
        return int(self.rows * (self.n_columns + self.extra) * 8 * PEAK_FACTOR)

    def bytes_for(self, columns) -> int:
        """The estimate with only ``columns`` read."""
        chosen = [name for name in self.columns if name in set(columns)]
        extra = BASE_EXTRA_ARRAYS + (STATUS_EXTRA_ARRAYS if STATUS_SOURCE_COLUMNS & set(chosen)
                                     else 0)
        return int(self.rows * (len(chosen) + extra) * 8 * PEAK_FACTOR)


class MemoryBudgetError(ValueError):
    """A file would need more memory than the budget; nothing was allocated."""

    def __init__(self, path, estimate: LoadEstimate, budget: int):
        self.path = Path(path)
        self.estimate = estimate
        self.budget = int(budget)
        super().__init__(
            f"{self.path.name} would need about {format_bytes(estimate.bytes)} "
            f"({estimate.rows:,} rows x {estimate.n_columns} numeric columns), more than "
            f"the {format_bytes(self.budget)} allowed for one file "
            f"({MEMORY_BUDGET_FRACTION:.0%} of RAM). Load fewer columns"
            + (" or every Nth row." if estimate.thinnable else "."))


def format_bytes(count: float) -> str:
    for unit in ("B", "KB", "MB"):
        if abs(count) < 1024:
            return f"{count:.0f} {unit}" if unit == "B" else f"{count:.1f} {unit}"
        count /= 1024
    return f"{count:.1f} GB"


def _extra_arrays(names) -> int:
    return BASE_EXTRA_ARRAYS + (STATUS_EXTRA_ARRAYS if STATUS_SOURCE_COLUMNS & set(names)
                                else 0)


def estimate_load(path, hex_as_numbers: bool = True, columns=None) -> LoadEstimate:
    """Rows and numeric columns of a file, from its first bytes (fast).

    ``columns`` restricts the estimate to those numeric columns.
    """
    path = Path(path)
    size = os.path.getsize(path)
    wanted = None if columns is None else set(columns)
    if is_bin_path(path):
        names, rows = _estimate_bin(path, size)
        thinnable = False
    else:
        sniffed = any_csv.sniff(path)
        if sniffed.is_plain:
            header, sample, header_bytes, avg_line = csv_columns._read_head(str(path))
            names = [name for name, ok in zip(header, csv_columns._classify(header, sample))
                     if ok]
            rows = int(max(0, size - header_bytes) / max(avg_line, 1.0))
        else:
            names = [name for name, kind in zip(sniffed.names, sniffed.kinds)
                     if kind != "text" and (hex_as_numbers or kind != "hex")]
            rows = int(max(0, size - sniffed.data_offset) / max(sniffed.avg_line, 1.0))
        thinnable = True
    if wanted is not None:
        names = [name for name in names if name in wanted]
    return LoadEstimate(rows=max(0, rows), columns=list(names), extra=_extra_arrays(names),
                        thinnable=thinnable)


def _estimate_bin(path: Path, size: int):
    """(numeric column names, estimated records) of a raw log by decoding its start."""
    with open(path, "rb") as f:
        head = f.read(min(size, 4 * 1024 * 1024))
    if not head:
        return [], 0
    try:
        _fmt, decoder = converter.select_decoder(path, head)
        names: dict[str, None] = {}
        count = 0
        last_offset = 0
        for row in decoder(head, converter.DecodeStats()):
            for name, value in row.items():
                if name in csv_columns.TEXT_COLUMNS or name in names:
                    continue
                if isinstance(value, (int, float)):
                    names[name] = None
            count += 1
            last_offset = int(row.get("byte_offset", 0) or 0)
            if count >= BIN_SAMPLE_ROWS:
                break
    except Exception:  # an undecodable head: the loader reports it properly
        return [], 0
    if count == 0:
        return list(names), 0
    if last_offset and count > 1:
        per_row = max(1.0, last_offset / (count - 1))
    else:
        per_row = max(1.0, len(head) / count)
    return list(names), int(size / per_row) + 1


# ---------------------------------------------------------------------------
# Traces for the plots: x-sorted arrays per (dataset, x axis, filter, column).
# ---------------------------------------------------------------------------

def row_order(dataset: Dataset, x_column: str, packet_filter: str = "All"):
    """(sorted x, row indices or None) for an x axis and packet filter.

    With the per-file time axis (SYNC_X), x is the file's time in seconds
    *before* its offset: the plots move a file by its offset with
    ``curve.setPos``, so changing the offset never rebuilds these arrays.
    ``rows`` is None when every row is used in file order (no copy made).
    """
    if x_column == SYNC_X:
        x = dataset.times()
    else:
        x = dataset.numeric_data.get(x_column)
    if x is None:
        return np.empty(0), np.empty(0, dtype=np.int64)
    keep = np.isfinite(x)
    mask = dataset.mask_for(packet_filter)
    if mask is not None:
        keep &= mask
    if keep.all() and (x.size < 2 or bool(np.all(x[1:] >= x[:-1]))):
        # Logs are nearly always in time order with every row timed: use the
        # column as it is (no sort, no copy).
        return x, None
    rows = np.flatnonzero(keep)
    xs = x[rows]
    if xs.size > 1 and not bool(np.all(xs[1:] >= xs[:-1])):
        order = np.argsort(xs, kind="stable")
        rows, xs = rows[order], xs[order]
    return xs, rows


def series_arrays(dataset: Dataset, column: str, order) -> tuple[np.ndarray, np.ndarray]:
    """One trace's (x, y), finite values only, for a ``row_order`` result.

    Shares the dataset's arrays when nothing has to be dropped.
    """
    xs, rows = order
    values = dataset.numeric_data.get(column)
    if values is None:
        return np.empty(0), np.empty(0)
    ys = values if rows is None else values[rows]
    keep = np.isfinite(ys)
    if keep.all():
        return xs, ys
    return xs[keep], ys[keep]


def x_offset(dataset: Dataset, x_column: str) -> float:
    """How far a file's curves are shifted along x (its offset on time axes)."""
    return dataset.offset if x_column == SYNC_X else 0.0


def nearest_index(xs: np.ndarray, x: float) -> int | None:
    """Index of the sample of sorted ``xs`` nearest to ``x``."""
    if xs.size == 0:
        return None
    index = int(np.searchsorted(xs, x))
    if index >= xs.size or (index > 0 and abs(xs[index - 1] - x) <= abs(xs[index] - x)):
        index -= 1
    return index


def view_stats(xs: np.ndarray, ys: np.ndarray, low: float | None = None,
               high: float | None = None) -> dict | None:
    """n / latest / min / max of a trace, of the samples in [low, high] if given."""
    if low is not None and high is not None and xs.size:
        first = int(np.searchsorted(xs, low, side="left"))
        last = int(np.searchsorted(xs, high, side="right"))
        ys = ys[first:last]
    if ys.size == 0:
        return None
    return {"n": int(ys.size), "latest": float(ys[-1]), "min": float(ys.min()),
            "max": float(ys.max())}


def stats_text(stats: dict, zoomed: bool) -> str:
    text = (f"n={stats['n']:,}  latest={stats['latest']:.6g}  "
            f"min={stats['min']:.6g}  max={stats['max']:.6g}")
    return ("in view: " + text) if zoomed else text


def cell_text(dataset: Dataset | None, column: str, value: float) -> str:
    """A trace value as its CSV showed it: hex as 0x.., times of day as hh:mm:ss."""
    kind = dataset.kinds.get(column) if dataset is not None else None
    if kind == "hex" and value == int(value) and value >= 0:
        return f"0x{int(value):x}"
    if kind in ("tod", "datetime"):
        return f"{any_csv.format_time_of_day(value)} ({value_text(value)} s)"
    return value_text(value)


# ---------------------------------------------------------------------------
# Texts shown by the UI.
# ---------------------------------------------------------------------------

def title_for(datasets) -> str:
    names = [dataset.path.name for dataset in datasets]
    if not names:
        return "Horizon CSV Viewer"
    if len(names) == 1:
        return f"Horizon CSV Viewer - {names[0]}"
    return f"Horizon CSV Viewer - {names[0]} + {len(names) - 1} more"


def file_label_text(datasets) -> str:
    paths = [dataset.path for dataset in datasets]
    if not paths:
        return "No file loaded"
    if len(paths) == 1:
        return str(paths[0])
    return f"{len(paths)} files: " + ", ".join(path.name for path in paths)


def dataset_text(dataset: Dataset) -> str:
    return f"{dataset.label}  [{dataset.time_axis_text()}]"


def time_axis_note(dataset: Dataset) -> str:
    times = dataset.times()
    timed = times[np.isfinite(times)]
    if not timed.size:
        return "No values in this column"
    return (f"{value_text(round(float(timed.min()) + dataset.offset, 6))} to "
            f"{value_text(round(float(timed.max()) + dataset.offset, 6))} s "
            "on the shared axis")


def summary_details(dataset: Dataset) -> str:
    """The detail line under the summary cards."""
    summary = dataset.summary
    times = dataset.row_times[~np.isnan(dataset.row_times)]
    time_span = "unknown"
    if times.size:
        time_span = f"{times.min():.3f} to {times.max():.3f} s"
    elif dataset.time_column != ROW_INDEX:      # other loggers: their time axis
        times = dataset.times()
        times = times[~np.isnan(times)]
        if times.size:
            time_span = (f"{times.min():.3f} to {times.max():.3f} s "
                         f"({dataset.time_column})")
    details = (
        f"{dataset.path.name}: {dataset.row_count:,} rows | "
        f"{len(dataset.numeric_columns)} numeric fields | time span {time_span}"
    )
    if dataset.stride > 1:
        details += (f" | large file: every {dataset.stride}th of "
                    f"{dataset.total_rows:,} rows loaded (memory cap)")
    if dataset.source_format:
        details += f" | decoded from .bin ({dataset.source_format}"
        details += (f", {dataset.discarded:,} bytes not decoded)" if dataset.discarded
                    else ")")
    if dataset.format_note:
        details += f" | read as {dataset.format_note}"
    if dataset.kinds:
        details += " | decoded: " + ", ".join(
            f"{name} ({kind})" for name, kind in dataset.kinds.items())
    if dataset.text_columns:
        details += (" | text (readout only): "
                    + ", ".join(list(dataset.text_columns)[:8])
                    + (" ..." if len(dataset.text_columns) > 8 else ""))
    if dataset.paired_state_log is not None:
        details += f" | flight state from {dataset.paired_state_log.name}"
    elif dataset.is_wl:
        details += " | WL v1 does not contain flight state"
    if summary["hitl"].startswith("HITL ACTIVE"):
        details += f" | {summary['hitl']}"
    return details


LAUNCH_NOTE = ("Launch detection uses the first Boost flight state when present. "
               "For older logs without flight state, it requires three consecutive "
               f"samples at or above {LAUNCH_ACCELERATION_G:g} g.")


def default_plot_columns(dataset: Dataset, x_column: str) -> list[str]:
    """Columns ``Plot CSV`` graphs: all but the time axis columns."""
    skip = (SYNC_SKIP_COLUMNS | {dataset.time_column} if x_column == SYNC_X
            else {x_column})
    return [column for column in dataset.numeric_columns if column not in skip]


def axis_choices(datasets) -> list[str]:
    """X axis choices: the per-file time axis when any file has time, then columns."""
    columns: list[str] = []
    seen = set()
    for dataset in datasets:
        for column in dataset.numeric_columns:
            if column not in seen:
                seen.add(column)
                columns.append(column)
    has_sync = any(dataset.has_sync_time for dataset in datasets)
    return ([SYNC_X] if has_sync else []) + columns


def preferred_axis(choices) -> str:
    return next((name for name in (SYNC_X, "time_seconds", "uptime", "packet_index")
                 if name in choices), choices[0] if choices else "")


def save_bin_as_csv(source: Path, target: Path) -> str:
    """Decode a raw log again and stream it to a CSV (worker thread)."""
    data = Path(source).read_bytes()
    _fmt, decoder = converter.select_decoder(Path(source), data)
    count = converter.write_csv_stream(lambda: decoder(data), Path(target))
    return f"Wrote {count:,} rows to {target}"


# ---------------------------------------------------------------------------
# Spectrogram window glue (the maths is in spectrum.py).
# ---------------------------------------------------------------------------

SPECTRUM_MAX_COLUMNS = 2000
SPECTRUM_IMAGE_BINS = 1024       # frequency rows of the displayed image (max-pooled)


def spectrum_source(dataset: Dataset | None, column: str, packet_filter: str = "All"):
    """(time, values) of one trace for an FFT: the CSV's own time axis in
    shared-axis seconds (offset included), packet filter applied, rows
    without a value dropped."""
    if dataset is None or column not in dataset.numeric_data:
        return np.empty(0), np.empty(0)
    t = dataset.times() + dataset.offset
    y = dataset.numeric_data[column]
    keep = np.isfinite(t) & np.isfinite(y)
    mask = dataset.mask_for(packet_filter)
    if mask is not None:
        keep &= mask
    return t[keep], y[keep]


def spectrum_settings(start: str, end: str, nfft, overlap_percent, window: str,
                      detrend: str) -> dict:
    """Analysis arguments from the window's controls (ValueError if malformed)."""
    start, end = float(start), float(end)
    if not (math.isfinite(start) and math.isfinite(end)) or end <= start:
        raise ValueError("the end of the time window must be after its start")
    overlap = float(overlap_percent)
    if not 0 <= overlap <= 95:
        raise ValueError("overlap must be 0 to 95 %")
    return {"start": start, "end": end, "nfft": int(nfft), "overlap": overlap / 100.0,
            "window": window, "detrend": detrend}


@dataclass
class SpectrogramResult:
    """An analysis plus what the window draws, all prepared off the GUI thread."""
    analysis: "spectrum.Analysis"
    image: np.ndarray            # (freq rows, time columns) float32 dB, low f first
    auto_range: tuple


def compute_spectrogram(t, y, params: dict, max_columns: int = SPECTRUM_MAX_COLUMNS,
                        image_bins: int = SPECTRUM_IMAGE_BINS) -> SpectrogramResult:
    """spectrum.analyze plus the display image (max-pooled to ``image_bins``
    frequency rows so drawing stays cheap) and the auto colour range."""
    analysis = spectrum.analyze(t, y, max_columns=max_columns, **params)
    db = analysis.db                               # (columns, bins)
    if db.shape[1] > image_bins:
        db = spectrum._fit_axis(db, 1, image_bins)
    image = np.ascontiguousarray(db.T, dtype=np.float32)   # rows = frequency
    return SpectrogramResult(analysis=analysis, image=image,
                             auto_range=analysis.auto_range())


def spectrum_status_text(analysis) -> str:
    s, r = analysis.sampling, analysis.result
    df = r.fs / r.nfft
    text = (f"{s.values.size:,} samples ({s.duration:.6g} s) | fs = {s.fs:.6g} Hz "
            f"(median dt {s.median_dt * 1e3:.4g} ms) | FFT {r.nfft} "
            f"({df:.4g} Hz/bin), hop {r.hop}, {r.segments:,} segment(s)")
    if r.columns < r.segments:
        text += f" averaged into {r.columns:,} columns"
    if analysis.warnings:
        text += "\nWARNING: " + "\nWARNING: ".join(analysis.warnings)
    return text
