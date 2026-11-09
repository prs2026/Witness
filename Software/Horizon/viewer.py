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
from dataclasses import dataclass
import math
import re
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import tkinter.font as tkfont


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


def decimate_minmax(
        points: list[tuple[float, float]], x_min: float, x_max: float,
        buckets: int,
) -> list[tuple[float, float]]:
    """Reduce sorted points to the min and max of each pixel column.

    Keeps spikes visible (unlike stride sampling) while bounding the number
    of canvas coordinates to about 2 * buckets, however long the log is.
    """
    buckets = max(1, int(buckets))
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


SYNC_X = "uptime [s] (synced)"
SYNC_SKIP_COLUMNS = frozenset(("uptime", "fc_uptime", "time_seconds"))
DATASET_DASHES = ((), (7, 3), (2, 2), (9, 3, 2, 3), (4, 4))
DROP_ZONE_HEIGHT = 64
DRAG_THRESHOLD_PX = 6


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


@dataclass
class Dataset:
    """One converted CSV, parsed for plotting."""
    path: Path
    label: str
    rows: list[dict[str, str]]
    fieldnames: list[str]
    numeric_columns: list[str]
    numeric_data: dict[str, list[float | None]]
    sync_times: list[float | None]
    paired_state_log: Path | None = None
    offset: float = 0.0

    @property
    def packet_types(self) -> list[str]:
        return sorted({
            row.get("packet_type", "").strip()
            for row in self.rows if row.get("packet_type", "").strip()
        })

    @property
    def has_sync_time(self) -> bool:
        return any(value is not None for value in self.sync_times)


def load_dataset(path: Path, label: str | None = None) -> Dataset:
    """Read a converted CSV. Raises OSError/csv.Error/ValueError."""
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError("the CSV has no header row")
        rows = list(reader)
        fieldnames = list(reader.fieldnames)

    paired_state_log = add_paired_wl_flight_state(path, rows, fieldnames)
    add_derived_flight_state(rows, fieldnames)
    add_status_bit_traces(rows, fieldnames)
    add_status_flag_columns(rows, fieldnames)

    numeric_columns: list[str] = []
    numeric_data: dict[str, list[float | None]] = {}
    for field in fieldnames:
        values: list[float | None] = []
        has_number = False
        has_non_numeric_text = False
        for row in rows:
            raw_value = row.get(field, "")
            value = numeric_value(raw_value)
            values.append(value)
            if value is not None:
                has_number = True
            elif raw_value is not None and raw_value.strip():
                has_non_numeric_text = True
        if has_number and not has_non_numeric_text:
            numeric_columns.append(field)
            numeric_data[field] = values
    if not numeric_columns:
        raise ValueError("the CSV contains no numeric columns to graph")
    return Dataset(path=path, label=label or path.name, rows=rows,
                   fieldnames=fieldnames, numeric_columns=numeric_columns,
                   numeric_data=numeric_data,
                   sync_times=[sync_time_seconds(row) for row in rows],
                   paired_state_log=paired_state_log)


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
    def rows(self) -> list[dict[str, str]]:
        first = next(iter(self.datasets.values()), None)
        return [] if first is None else first.rows

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
        self.x_axis_combo = ttk.Combobox(controls, width=18, state="readonly")
        self.x_axis_combo.pack(side="left")
        self.x_axis_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_axis_changed())

        ttk.Label(controls, text="Packet type:").pack(side="left", padx=(10, 4))
        self.packet_filter_combo = ttk.Combobox(controls, width=14, state="readonly")
        self.packet_filter_combo.set("All")
        self.packet_filter_combo.pack(side="left")
        self.packet_filter_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self.on_axis_changed()
        )
        ttk.Button(controls, text="Clear graphs", command=self.clear_graphs).pack(
            side="left", padx=(10, 0))

        self.summary_label = ttk.Label(self.plot_tab, text="", padding=(8, 0, 8, 5),
                                       width=10)
        self.summary_label.pack(fill="x")

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
                  "a legend entry to another graph, or back here to remove it."))
        self.help_label.pack(fill="x", padx=4, pady=(0, 4))
        trace_panel.bind("<Configure>", lambda event: self.help_label.configure(
            wraplength=max(120, event.width - 10)))

        buttons = ttk.Frame(trace_panel)
        buttons.pack(fill="x", padx=2, pady=(0, 4))
        ttk.Button(buttons, text="Plot CSV", command=self.plot_selected_dataset).pack(
            side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(buttons, text="Remove CSV", command=self.remove_selected_dataset).pack(
            side="left", fill="x", expand=True, padx=(2, 0))

        offset_row = ttk.Frame(trace_panel)
        offset_row.pack(fill="x", padx=2, pady=(0, 4))
        ttk.Label(offset_row, text="Time offset s:").pack(side="left")
        self.offset_var = tk.StringVar(value="0")
        self.offset_entry = ttk.Entry(offset_row, textvariable=self.offset_var, width=8)
        self.offset_entry.pack(side="left", padx=4)
        self.offset_entry.bind("<Return>", lambda _event: self.apply_offset())
        ttk.Button(offset_row, text="Apply", command=self.apply_offset, width=6).pack(
            side="left")

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
        self.trace_tree.bind("<<TreeviewSelect>>", lambda _event: self._sync_offset_entry())
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
        self.clear_datasets()
        return self.add_csv(path)

    def add_csvs(self, paths) -> int:
        added = 0
        errors = []
        for path in paths:
            try:
                self.add_csv(Path(path), report=False)
                added += 1
            except (OSError, csv.Error, ValueError) as exc:
                errors.append(f"{path}:\n{exc}")
        if errors:
            messagebox.showerror("Open CSV", "Could not open:\n\n" + "\n\n".join(errors),
                                 parent=self)
        return added

    def add_csv(self, path: Path, report: bool = True) -> bool:
        """Open one more CSV alongside the ones already loaded."""
        try:
            dataset = load_dataset(
                path, unique_label(path.name, (d.label for d in self.datasets.values())))
        except (OSError, csv.Error, ValueError) as exc:
            if not report:
                raise
            messagebox.showerror("Open CSV", f"Could not open {path}:\n{exc}",
                                 parent=self)
            return False
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
        self._set_title()
        if first:
            self.notebook.select(self.summary_outer)
            self.canvas.yview_moveto(0)
        self.draw_graphs()
        return True

    def clear_datasets(self) -> None:
        self.datasets.clear()
        self.layout.clear()
        self._points_cache = {}
        self.trace_tree.delete(*self.trace_tree.get_children())
        self._refresh_axis_choices(keep=False)
        self._refresh_file_label()
        self._refresh_summary_choices()
        self.draw_graphs()

    def remove_dataset(self, dataset_id: int) -> None:
        if dataset_id not in self.datasets:
            return
        del self.datasets[dataset_id]
        self.layout.remove_dataset(dataset_id)
        self._points_cache = {key: value for key, value in self._points_cache.items()
                              if key[0] != dataset_id}
        if self.trace_tree.exists(f"d{dataset_id}"):
            self.trace_tree.delete(f"d{dataset_id}")
        self._refresh_axis_choices(keep=True)
        self._refresh_file_label()
        self._refresh_summary_choices()
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
        skip = SYNC_SKIP_COLUMNS if x_column == SYNC_X else {x_column}
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

    def apply_offset(self) -> None:
        dataset_id = self._selected_dataset_id()
        dataset = self.datasets.get(dataset_id)
        if dataset is None:
            return
        try:
            offset = float(self.offset_var.get())
        except ValueError:
            messagebox.showerror("Time offset", "Enter a number of seconds.", parent=self)
            return
        if not math.isfinite(offset):
            return
        dataset.offset = offset
        self._points_cache = {key: value for key, value in self._points_cache.items()
                              if key[0] != dataset_id}
        self.trace_tree.item(f"d{dataset_id}", text=self._dataset_text(dataset))
        self.draw_graphs()

    def _sync_offset_entry(self) -> None:
        dataset = self.datasets.get(self._selected_dataset_id())
        if dataset is not None:
            self.offset_var.set(f"{dataset.offset:g}")

    def _dataset_text(self, dataset: Dataset) -> str:
        text = dataset.label
        if dataset.offset:
            text += f"  ({dataset.offset:+g} s)"
        if not dataset.has_sync_time:
            text += "  [no uptime]"
        return text

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

    def on_axis_changed(self) -> None:
        self._points_cache = {}
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
        rows = dataset.rows
        summary = calculate_summary(rows)
        self.max_altitude_var.set(summary["max_altitude"])
        self.launch_time_var.set(summary["launch_time"])
        self.max_acceleration_var.set(summary["max_acceleration"])
        self.flight_state_var.set(summary["flight_state"])
        valid_times = [
            timestamp for timestamp in (row_time_seconds(row) for row in rows)
            if timestamp is not None
        ]
        time_span = "unknown"
        if valid_times:
            time_span = f"{min(valid_times):.3f} to {max(valid_times):.3f} s"
        details = (
            f"{dataset.path.name}: {len(rows):,} rows | "
            f"{len(dataset.numeric_columns)} numeric fields | time span {time_span}"
        )
        if dataset.paired_state_log is not None:
            details += f" | flight state from {dataset.paired_state_log.name}"
        elif any(row.get("packet_id") == "WL" for row in rows):
            details += " | WL v1 does not contain flight state"
        if summary["hitl"].startswith("HITL ACTIVE"):
            details += f" | {summary['hitl']}"
        self.summary_details_var.set(details)

    # ------------------------------------------------------------ points
    def filtered_indices(self, dataset: Dataset) -> list[int]:
        packet_filter = self.packet_filter_combo.get()
        if packet_filter == "All" or "packet_type" not in dataset.fieldnames:
            return list(range(len(dataset.rows)))
        return [
            index for index, row in enumerate(dataset.rows)
            if row.get("packet_type", "") == packet_filter
        ]

    def series_points(self, ref) -> list[tuple[float, float]]:
        """Sorted (x, y) points of one series for the current x axis/filter."""
        dataset_id, column = ref
        dataset = self.datasets.get(dataset_id)
        x_column = self.x_axis_combo.get()
        key = (dataset_id, x_column, self.packet_filter_combo.get(), column)
        points = self._points_cache.get(key)
        if points is not None:
            return points
        if dataset is None or column not in dataset.numeric_data:
            return []
        if x_column == SYNC_X:
            x_data = dataset.sync_times
            offset = dataset.offset
        else:
            x_data = dataset.numeric_data.get(x_column)
            offset = 0.0
        if x_data is None:
            points = []
        else:
            y_data = dataset.numeric_data[column]
            points = [
                (x_data[index] + offset, y_data[index])
                for index in self.filtered_indices(dataset)
                if x_data[index] is not None and y_data[index] is not None
            ]
            points.sort(key=lambda point: point[0])
        self._points_cache[key] = points
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
        self.summary_label.configure(text=(
            f"{len(self.datasets)} CSV(s) | {len(graphs)} graph(s), "
            f"{sum(len(g['refs']) for g in graphs)} trace(s) | x = {x_column}"))
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

        canvas.create_rectangle(4, top + 4, width - 4, bottom - 4,
                                outline="#c8c8c8", fill="#ffffff")
        close = canvas.create_text(width - 14, top + 8 + line_height / 2, anchor="e",
                                   text="✕", fill="#888888",
                                   tags=("close", f"g{index}"))
        self._close_items[close] = index

        # Header: a single trace keeps the name + statistics layout; several
        # traces get a wrapped, colour-keyed legend. Every name is draggable.
        header_y = top + 8 + line_height / 2
        non_empty = [points for _ref, points in series if points]
        if len(series) == 1:
            ref, points = series[0]
            name = elide(graph["labels"][0], width - 60, self.bold_font.measure)
            item = canvas.create_text(14, header_y, anchor="w", text=name,
                                      fill="#222222", font=self.bold_font,
                                      tags=("legend",))
            self._legend_items[item] = (index, ref)
            header_lines = 1
            if points:
                y_values = [point[1] for point in points]
                stats = (f"n={len(points):,}  latest={points[-1][1]:.6g}  "
                         f"min={min(y_values):.6g}  max={max(y_values):.6g}")
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
                color = PLOT_COLORS[series_index % len(PLOT_COLORS)]
                dash = DATASET_DASHES[ref[0] % len(DATASET_DASHES)]
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
            canvas.create_text(width / 2, (plot_top + plot_bottom) / 2,
                               text=f"No values for x = {x_column}", fill="#777777")
            return

        x_min = min(points[0][0] for points in non_empty)
        x_max = max(points[-1][0] for points in non_empty)
        data_min = min(point[1] for points in non_empty for point in points)
        data_max = max(point[1] for points in non_empty for point in points)
        if x_max <= x_min:
            x_min -= 0.5
            x_max += 0.5

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

        for series_index, (ref, points) in enumerate(series):
            if not points:
                continue
            reduced = decimate_minmax(points, x_min, x_max, int(right - left))
            coordinates: list[float] = []
            for x_value, y_value in reduced:
                coordinates.append(left + (x_value - x_min) / x_span * (right - left))
                coordinates.append(to_y(y_value))
            if len(series) == 1:
                color = PLOT_COLORS[index % len(PLOT_COLORS)]
                dash = ()
            else:
                color = PLOT_COLORS[series_index % len(PLOT_COLORS)]
                dash = DATASET_DASHES[ref[0] % len(DATASET_DASHES)]
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
        current = self.canvas.find_withtag("current")
        if not current:
            return
        item = current[0]
        if item in self._close_items:
            self.layout.remove_graph(self._close_items[item])
            self.draw_graphs()
            return
        if item in self._legend_items:
            self._press = (event.x_root, event.y_root, item, ())

    def _canvas_motion(self, event):
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
