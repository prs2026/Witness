"""Interactive viewer for CSV files produced by converter.py.

Usage:
    python viewer.py S0000434.csv
    python viewer.py

With no filename, the viewer opens a file-selection dialog. The same viewer
is embedded as a witnessOS tab (CsvViewerPanel).
"""

from __future__ import annotations

import argparse
import csv
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


class CsvViewerPanel(ttk.Frame):
    """The CSV viewer as an embeddable frame (summary and plot notebooks).

    ``on_title`` is called with a window title when a file is loaded.
    """

    def __init__(self, master, initial_file: Path | None = None, *,
                 prompt_on_start: bool = False, on_title=None):
        super().__init__(master)
        self.on_title = on_title
        self.csv_path: Path | None = None
        self.rows: list[dict[str, str]] = []
        self.fieldnames: list[str] = []
        self.numeric_columns: list[str] = []
        self.numeric_data: dict[str, list[float | None]] = {}
        self.trace_vars: dict[str, tk.BooleanVar] = {}
        self.redraw_job: str | None = None
        self.visible_job: str | None = None
        self._points_cache: dict[tuple, list[tuple[float, float]]] = {}
        self._graphs: list[tuple[str, list[tuple[float, float]]]] = []
        self._graph_height = GRAPH_HEIGHT
        self._graph_width = 0
        self._x_column = ""
        self._drawn: set[int] = set()
        self._last_size = (0, 0)
        self.font = tkfont.nametofont("TkDefaultFont")
        self.bold_font = tkfont.Font(root=self, font=self.font)
        self.bold_font.configure(weight="bold")

        self.build_ui()
        self.bind("<Destroy>", self._on_destroy, add="+")
        if initial_file is not None:
            self.after(0, lambda: self.load_csv(initial_file))
        elif prompt_on_start:
            self.after(0, self.choose_csv)

    def build_ui(self) -> None:
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")

        ttk.Button(controls, text="Open CSV", command=self.choose_csv).pack(side="left")
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
        # wraps to the current width instead of a fixed 850-900 px.
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

        ttk.Label(
            self.summary_tab,
            text="Flight Summary",
            font=("Segoe UI", 22, "bold"),
        ).pack(anchor="w", pady=(0, 4))
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

        ttk.Label(controls, text="X axis:").pack(side="left", padx=(8, 4))
        self.x_axis_combo = ttk.Combobox(controls, width=16, state="readonly")
        self.x_axis_combo.pack(side="left")
        self.x_axis_combo.bind("<<ComboboxSelected>>", lambda _event: self.draw_graphs())

        ttk.Label(controls, text="Packet type:").pack(side="left", padx=(10, 4))
        self.packet_filter_combo = ttk.Combobox(controls, width=14, state="readonly")
        self.packet_filter_combo.set("All")
        self.packet_filter_combo.pack(side="left")
        self.packet_filter_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self.draw_graphs()
        )

        self.summary_label = ttk.Label(self.plot_tab, text="", padding=(8, 0, 8, 5),
                                       width=10)
        self.summary_label.pack(fill="x")

        plot_panes = ttk.Panedwindow(self.plot_tab, orient="horizontal")
        plot_panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        trace_panel = ttk.Frame(plot_panes, width=200)
        graph_frame = ttk.Frame(plot_panes)
        plot_panes.add(trace_panel, weight=0)
        plot_panes.add(graph_frame, weight=1)

        ttk.Label(trace_panel, text="Traces", font=("Segoe UI", 11, "bold")).pack(
            anchor="w", padx=4, pady=(2, 5)
        )
        trace_buttons = ttk.Frame(trace_panel)
        trace_buttons.pack(fill="x", padx=2, pady=(0, 5))
        ttk.Button(trace_buttons, text="Select all", command=self.select_all_traces).pack(
            side="left", fill="x", expand=True, padx=(0, 2)
        )
        ttk.Button(trace_buttons, text="Clear all", command=self.clear_all_traces).pack(
            side="left", fill="x", expand=True, padx=(2, 0)
        )

        trace_list_frame = ttk.Frame(trace_panel)
        trace_list_frame.pack(fill="both", expand=True)
        self.trace_canvas = tk.Canvas(
            trace_list_frame,
            width=190,
            height=120,
            highlightthickness=1,
            highlightbackground="#c8c8c8",
        )
        trace_scrollbar = ttk.Scrollbar(
            trace_list_frame, orient="vertical", command=self.trace_canvas.yview
        )
        self.trace_canvas.configure(yscrollcommand=trace_scrollbar.set)
        self.trace_canvas.pack(side="left", fill="both", expand=True)
        trace_scrollbar.pack(side="right", fill="y")
        self.trace_inner = ttk.Frame(self.trace_canvas)
        self.trace_window = self.trace_canvas.create_window(
            (0, 0), window=self.trace_inner, anchor="nw"
        )
        self.trace_inner.bind(
            "<Configure>",
            lambda _event: self.trace_canvas.configure(
                scrollregion=self.trace_canvas.bbox("all")
            ),
        )
        self.trace_canvas.bind(
            "<Configure>",
            lambda event: self.trace_canvas.itemconfigure(
                self.trace_window, width=event.width
            ),
        )

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
        for widget in (self.canvas, self.trace_canvas):
            widget.bind("<MouseWheel>",
                        lambda event, w=widget: self._wheel(w, event))
            widget.bind("<Button-4>", lambda event, w=widget: self._wheel(w, event, -1))
            widget.bind("<Button-5>", lambda event, w=widget: self._wheel(w, event, 1))

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

    def _set_title(self, text: str) -> None:
        if self.on_title is not None:
            self.on_title(text)

    def choose_csv(self) -> None:
        filename = filedialog.askopenfilename(
            parent=self,
            title="Open converted telemetry CSV",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if filename:
            self.load_csv(Path(filename))

    def rebuild_trace_selector(self) -> None:
        for child in self.trace_inner.winfo_children():
            child.destroy()
        self.trace_vars = {}
        for column in self.numeric_columns:
            variable = tk.BooleanVar(value=True)
            self.trace_vars[column] = variable
            check = ttk.Checkbutton(
                self.trace_inner,
                text=column,
                variable=variable,
                command=self.draw_graphs,
            )
            check.pack(anchor="w", fill="x", padx=5, pady=1)
            check.bind("<MouseWheel>",
                       lambda event: self._wheel(self.trace_canvas, event))
        self.trace_canvas.yview_moveto(0)

    def select_all_traces(self) -> None:
        for variable in self.trace_vars.values():
            variable.set(True)
        self.draw_graphs()

    def clear_all_traces(self) -> None:
        for variable in self.trace_vars.values():
            variable.set(False)
        self.draw_graphs()

    def load_csv(self, path: Path) -> bool:
        try:
            with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
                reader = csv.DictReader(csv_file)
                if not reader.fieldnames:
                    raise ValueError("the CSV has no header row")
                rows = list(reader)
                fieldnames = list(reader.fieldnames)
        except (OSError, csv.Error, ValueError) as exc:
            messagebox.showerror("Open CSV", f"Could not open {path}:\n{exc}",
                                 parent=self)
            return False

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
            messagebox.showerror("Open CSV", "The CSV contains no numeric columns to graph.",
                                 parent=self)
            return False

        self.csv_path = path
        self.rows = rows
        self.fieldnames = fieldnames
        self.numeric_columns = numeric_columns
        self.numeric_data = numeric_data
        self._points_cache = {}
        self.rebuild_trace_selector()

        self.x_axis_combo["values"] = numeric_columns
        preferred_axis = next(
            (name for name in ("time_seconds", "uptime", "packet_index")
             if name in numeric_columns),
            numeric_columns[0],
        )
        self.x_axis_combo.set(preferred_axis)

        packet_types = sorted({
            row.get("packet_type", "").strip()
            for row in rows if row.get("packet_type", "").strip()
        })
        self.packet_filter_combo["values"] = ["All", *packet_types]
        self.packet_filter_combo.set("All")

        self.file_label.configure(text=str(path))
        self._set_title(f"Horizon CSV Viewer - {path.name}")
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
        summary_details = (
            f"{len(rows):,} rows | {len(numeric_columns)} numeric fields | "
            f"time span {time_span}"
        )
        if paired_state_log is not None:
            summary_details += f" | flight state from {paired_state_log.name}"
        elif any(row.get("packet_id") == "WL" for row in rows):
            summary_details += " | WL v1 does not contain flight state"
        if summary["hitl"].startswith("HITL ACTIVE"):
            summary_details += f" | {summary['hitl']}"
        self.summary_details_var.set(summary_details)
        self.notebook.select(self.summary_outer)
        self.canvas.yview_moveto(0)
        self.draw_graphs()
        return True

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

    def filtered_indices(self) -> list[int]:
        packet_filter = self.packet_filter_combo.get()
        if packet_filter == "All" or "packet_type" not in self.fieldnames:
            return list(range(len(self.rows)))
        return [
            index for index, row in enumerate(self.rows)
            if row.get("packet_type", "") == packet_filter
        ]

    def graph_points(self, x_column: str, column: str,
                     indices: list[int]) -> list[tuple[float, float]]:
        key = (x_column, self.packet_filter_combo.get(), column)
        points = self._points_cache.get(key)
        if points is None:
            x_data = self.numeric_data[x_column]
            y_data = self.numeric_data[column]
            points = [
                (x_data[index], y_data[index])
                for index in indices
                if x_data[index] is not None and y_data[index] is not None
            ]
            points.sort(key=lambda point: point[0])
            self._points_cache[key] = points
        return points

    def draw_graphs(self) -> None:
        if self.redraw_job is not None:
            self.after_cancel(self.redraw_job)
        self.redraw_job = None
        canvas = self.canvas
        canvas.delete("all")
        self._graphs = []
        self._drawn = set()
        width = canvas.winfo_width()
        if width <= 1:
            width = canvas.winfo_reqwidth()
        view_height = canvas.winfo_height()
        if not self.rows or not self.numeric_columns:
            canvas.create_text(20, 20, anchor="nw", text="Open a converted CSV to begin.",
                               fill="#555555")
            canvas.configure(scrollregion=(0, 0, width, max(1, view_height)))
            return

        x_column = self.x_axis_combo.get()
        if x_column not in self.numeric_data:
            return
        indices = self.filtered_indices()

        graphs: list[tuple[str, list[tuple[float, float]]]] = []
        for column in self.numeric_columns:
            variable = self.trace_vars.get(column)
            if column == x_column or variable is None or not variable.get():
                continue
            points = self.graph_points(x_column, column, indices)
            if points:
                graphs.append((column, points))

        graph_height = graph_height_for(len(graphs), view_height)
        total_height = max(1, len(graphs) * graph_height)
        canvas.configure(scrollregion=(0, 0, width, total_height))
        self.summary_label.configure(
            text=(f"{len(self.rows):,} rows | {len(indices):,} filtered rows | "
                  f"{len(graphs)} plotted traces | x = {x_column}")
        )

        if not graphs:
            canvas.create_text(
                width // 2, 40, text="No numeric values match the current filter.",
                fill="#555555", width=max(100, width - 20),
            )
            return

        self._graphs = graphs
        self._graph_height = graph_height
        self._graph_width = width
        self._x_column = x_column
        self.draw_visible()

    def draw_visible(self) -> None:
        """Draw only the graphs in (or next to) the visible part of the canvas."""
        self.visible_job = None
        if not self._graphs:
            return
        canvas = self.canvas
        top = canvas.canvasy(0)
        bottom = top + max(canvas.winfo_height(), 1)
        first = max(0, int(top // self._graph_height) - 1)
        last = min(len(self._graphs) - 1, int(bottom // self._graph_height) + 1)
        for graph_index in range(first, last + 1):
            if graph_index in self._drawn:
                continue
            self._drawn.add(graph_index)
            column, points = self._graphs[graph_index]
            self.draw_graph(graph_index, self._graph_width, self._x_column,
                            column, points, self._graph_height)

    def draw_graph(
            self,
            graph_index: int,
            width: int,
            x_column: str,
            column: str,
            points: list[tuple[float, float]],
            graph_height: int = GRAPH_HEIGHT,
    ) -> None:
        canvas = self.canvas
        font = self.font
        line_height = font.metrics("linespace")
        top = graph_index * graph_height
        bottom = top + graph_height
        width = max(int(width), 160)

        x_min = points[0][0]
        x_max = points[-1][0]
        y_values = [point[1] for point in points]
        data_min = min(y_values)
        data_max = max(y_values)
        if x_max <= x_min:
            x_min -= 0.5
            x_max += 0.5

        # Header: name on the left, statistics on the right. When both do
        # not fit on one line the statistics drop to a second line and are
        # shortened rather than drawn over the name.
        canvas.create_rectangle(
            4, top + 4, width - 4, bottom - 4, outline="#c8c8c8", fill="#ffffff",
        )
        stats = (f"n={len(points):,}  latest={points[-1][1]:.6g}  "
                 f"min={data_min:.6g}  max={data_max:.6g}")
        name = elide(column, width - 28, self.bold_font.measure)
        name_width = self.bold_font.measure(name)
        header_y = top + 8 + line_height / 2
        canvas.create_text(14, header_y, anchor="w", text=name,
                           fill="#222222", font=self.bold_font)
        if name_width + 24 + font.measure(stats) <= width - 28:
            canvas.create_text(width - 14, header_y, anchor="e", text=stats,
                               fill="#444444", font=font)
            header_lines = 1
        else:
            stats = elide(stats, width - 28, font.measure)
            canvas.create_text(14, header_y + line_height, anchor="w",
                               text=stats, fill="#444444", font=font)
            header_lines = 2

        plot_top = top + 10 + header_lines * line_height + 6
        plot_bottom = bottom - (2 * line_height + 12)
        plot_height = plot_bottom - plot_top

        if column in BINARY_COLUMNS:
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

        reduced = decimate_minmax(points, x_min, x_max, int(right - left))
        coordinates: list[float] = []
        for x_value, y_value in reduced:
            coordinates.append(left + (x_value - x_min) / x_span * (right - left))
            coordinates.append(to_y(y_value))

        color = PLOT_COLORS[graph_index % len(PLOT_COLORS)]
        if len(coordinates) >= 4:
            canvas.create_line(*coordinates, fill=color, width=1.5)
        else:
            x, y = coordinates
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)

        canvas.create_text(
            (left + right) / 2, plot_bottom + 6 + line_height,
            anchor="n", text=elide(x_column, right - left, font.measure),
            fill="#444444", font=font,
        )


class CsvGraphViewer(tk.Tk):
    """Standalone window around CsvViewerPanel (python viewer.py)."""

    def __init__(self, initial_file: Path | None = None):
        super().__init__()
        self.title("Horizon CSV Viewer")
        width = min(1100, self.winfo_screenwidth() - 40)
        height = min(800, self.winfo_screenheight() - 80)
        self.geometry(f"{width}x{height}")
        self.minsize(520, 360)
        self.panel = CsvViewerPanel(self, initial_file,
                                    prompt_on_start=initial_file is None,
                                    on_title=self.title)
        self.panel.pack(fill="both", expand=True)

    def load_csv(self, path: Path) -> bool:
        return self.panel.load_csv(path)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Graph every numeric column in a converted Horizon CSV file."
    )
    parser.add_argument("csv_file", nargs="?", type=Path, help="converted CSV file")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    CsvGraphViewer(args.csv_file).mainloop()


if __name__ == "__main__":
    main()
