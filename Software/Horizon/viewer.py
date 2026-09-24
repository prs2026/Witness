"""Interactive viewer for CSV files produced by converter.py.

Usage:
    python viewer.py S0000434.csv
    python viewer.py

With no filename, the viewer opens a file-selection dialog.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


GRAPH_HEIGHT = 230
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
    "status_byte1_bit3_reserved",
    "status_byte1_bit4_reserved",
    "status_byte1_bit5_reserved",
    "status_byte1_bit6_reserved",
    "status_byte1_bit7_reserved",
)

STATUS_BIT_FIELDS = STATUS_BYTE0_BIT_FIELDS + STATUS_BYTE1_BIT_FIELDS


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

    return {
        "max_altitude": altitude_text,
        "launch_time": launch_text,
        "max_acceleration": acceleration_text,
        "flight_state": flight_state_text,
    }


class CsvGraphViewer(tk.Tk):
    def __init__(self, initial_file: Path | None = None):
        super().__init__()
        self.title("Horizon CSV Viewer")
        self.geometry("1100x800")
        self.minsize(700, 450)

        self.csv_path: Path | None = None
        self.rows: list[dict[str, str]] = []
        self.fieldnames: list[str] = []
        self.numeric_columns: list[str] = []
        self.numeric_data: dict[str, list[float | None]] = {}
        self.trace_vars: dict[str, tk.BooleanVar] = {}
        self.redraw_job: str | None = None

        self.build_ui()
        if initial_file is not None:
            self.after(0, lambda: self.load_csv(initial_file))
        else:
            self.after(0, self.choose_csv)

    def build_ui(self) -> None:
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")

        ttk.Button(controls, text="Open CSV", command=self.choose_csv).pack(side="left")
        self.file_label = ttk.Label(controls, text="No file loaded", anchor="w")
        self.file_label.pack(side="left", fill="x", expand=True, padx=10)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self.summary_tab = ttk.Frame(self.notebook, padding=18)
        self.plot_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.summary_tab, text="Summary")
        self.notebook.add(self.plot_tab, text="Plots")

        self.build_summary_tab()
        self.build_plot_tab()

    def build_summary_tab(self) -> None:
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
            ("Max barometric altitude reached", self.max_altitude_var),
            ("Launch detected at time", self.launch_time_var),
            ("Max acceleration", self.max_acceleration_var),
        )
        for column, (title, value_var) in enumerate(card_values):
            card = ttk.LabelFrame(cards, text=title, padding=16)
            card.grid(row=0, column=column, sticky="nsew", padx=6)
            ttk.Label(
                card,
                textvariable=value_var,
                anchor="center",
                justify="center",
                font=("Segoe UI", 15, "bold"),
            ).pack(fill="both", expand=True, pady=12)

        self.flight_state_var = tk.StringVar(value="--")
        state_card = ttk.LabelFrame(
            self.summary_tab, text="Flight state (status byte 1 bits 2:0)", padding=12
        )
        state_card.pack(fill="x", padx=6, pady=(14, 0))
        ttk.Label(
            state_card,
            textvariable=self.flight_state_var,
            anchor="w",
            justify="left",
            font=("Segoe UI", 12, "bold"),
            wraplength=900,
        ).pack(fill="x")

        self.summary_details_var = tk.StringVar(value="Open a CSV file to begin.")
        ttk.Label(
            self.summary_tab,
            textvariable=self.summary_details_var,
            padding=(6, 24, 6, 6),
        ).pack(anchor="w")

        ttk.Label(
            self.summary_tab,
            text=(
                "Launch detection uses the first Boost flight state when present. "
                "For older logs without flight state, it requires three consecutive "
                f"samples at or above {LAUNCH_ACCELERATION_G:g} g."
            ),
            wraplength=850,
            foreground="#555555",
        ).pack(anchor="w", side="bottom", pady=8)

    def build_plot_tab(self) -> None:
        controls = ttk.Frame(self.plot_tab, padding=8)
        controls.pack(fill="x")

        ttk.Label(controls, text="X axis:").pack(side="left", padx=(8, 4))
        self.x_axis_combo = ttk.Combobox(controls, width=20, state="readonly")
        self.x_axis_combo.pack(side="left")
        self.x_axis_combo.bind("<<ComboboxSelected>>", lambda _event: self.draw_graphs())

        ttk.Label(controls, text="Packet type:").pack(side="left", padx=(10, 4))
        self.packet_filter_combo = ttk.Combobox(controls, width=20, state="readonly")
        self.packet_filter_combo.set("All")
        self.packet_filter_combo.pack(side="left")
        self.packet_filter_combo.bind(
            "<<ComboboxSelected>>", lambda _event: self.draw_graphs()
        )

        self.summary_label = ttk.Label(self.plot_tab, text="", padding=(8, 0, 8, 5))
        self.summary_label.pack(fill="x")

        plot_panes = ttk.Panedwindow(self.plot_tab, orient="horizontal")
        plot_panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        trace_panel = ttk.Frame(plot_panes, width=240)
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
            width=230,
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
        )
        scrollbar = ttk.Scrollbar(graph_frame, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.canvas.bind("<Configure>", self.schedule_redraw)
        self.canvas.bind(
            "<MouseWheel>",
            lambda event: self.canvas.yview_scroll(int(-event.delta / 120), "units"),
        )

    def choose_csv(self) -> None:
        filename = filedialog.askopenfilename(
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
            ttk.Checkbutton(
                self.trace_inner,
                text=column,
                variable=variable,
                command=self.draw_graphs,
            ).pack(anchor="w", fill="x", padx=5, pady=1)

    def select_all_traces(self) -> None:
        for variable in self.trace_vars.values():
            variable.set(True)
        self.draw_graphs()

    def clear_all_traces(self) -> None:
        for variable in self.trace_vars.values():
            variable.set(False)
        self.draw_graphs()

    def load_csv(self, path: Path) -> None:
        try:
            with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
                reader = csv.DictReader(csv_file)
                if not reader.fieldnames:
                    raise ValueError("the CSV has no header row")
                rows = list(reader)
                fieldnames = list(reader.fieldnames)
        except (OSError, csv.Error, ValueError) as exc:
            messagebox.showerror("Open CSV", f"Could not open {path}:\n{exc}")
            return

        paired_state_log = add_paired_wl_flight_state(path, rows, fieldnames)
        add_derived_flight_state(rows, fieldnames)
        add_status_bit_traces(rows, fieldnames)

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
            messagebox.showerror("Open CSV", "The CSV contains no numeric columns to graph.")
            return

        self.csv_path = path
        self.rows = rows
        self.fieldnames = fieldnames
        self.numeric_columns = numeric_columns
        self.numeric_data = numeric_data
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
        self.title(f"Horizon CSV Viewer - {path.name}")
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
        self.summary_details_var.set(summary_details)
        self.notebook.select(self.summary_tab)
        self.draw_graphs()

    def schedule_redraw(self, _event=None) -> None:
        if self.redraw_job is not None:
            self.after_cancel(self.redraw_job)
        self.redraw_job = self.after(100, self.draw_graphs)

    def filtered_indices(self) -> list[int]:
        packet_filter = self.packet_filter_combo.get()
        if packet_filter == "All" or "packet_type" not in self.fieldnames:
            return list(range(len(self.rows)))
        return [
            index for index, row in enumerate(self.rows)
            if row.get("packet_type", "") == packet_filter
        ]

    def draw_graphs(self) -> None:
        self.redraw_job = None
        canvas = self.canvas
        canvas.delete("all")
        if not self.rows or not self.numeric_columns:
            canvas.create_text(30, 30, anchor="nw", text="Open a converted CSV to begin.")
            canvas.configure(scrollregion=(0, 0, 1, 1))
            return

        x_column = self.x_axis_combo.get()
        if x_column not in self.numeric_data:
            return
        indices = self.filtered_indices()
        x_data = self.numeric_data[x_column]

        graphs: list[tuple[str, list[tuple[float, float]]]] = []
        selected_columns = [
            column for column in self.numeric_columns
            if column in self.trace_vars and self.trace_vars[column].get()
        ]
        for column in selected_columns:
            if column == x_column:
                continue
            y_data = self.numeric_data[column]
            points = [
                (x_data[index], y_data[index])
                for index in indices
                if x_data[index] is not None and y_data[index] is not None
            ]
            if points:
                points.sort(key=lambda point: point[0])
                graphs.append((column, points))

        width = max(canvas.winfo_width(), 650)
        total_height = max(1, len(graphs) * GRAPH_HEIGHT)
        canvas.configure(scrollregion=(0, 0, width, total_height))
        self.summary_label.configure(
            text=(f"{len(self.rows):,} rows | {len(indices):,} filtered rows | "
                  f"{len(graphs)} plotted traces | x = {x_column}")
        )

        if not graphs:
            canvas.create_text(
                width // 2, 40, text="No numeric values match the current filter.",
                fill="#555555",
            )
            return

        for graph_index, (column, points) in enumerate(graphs):
            self.draw_graph(graph_index, width, x_column, column, points)

    def draw_graph(
            self,
            graph_index: int,
            width: int,
            x_column: str,
            column: str,
            points: list[tuple[float, float]],
    ) -> None:
        canvas = self.canvas
        top = graph_index * GRAPH_HEIGHT
        left = 88
        right = max(left + 100, width - 20)
        plot_top = top + 38
        plot_bottom = top + GRAPH_HEIGHT - 42

        x_min = min(point[0] for point in points)
        x_max = max(point[0] for point in points)
        y_min = min(point[1] for point in points)
        y_max = max(point[1] for point in points)
        if x_max <= x_min:
            x_min -= 0.5
            x_max += 0.5
        if column in STATUS_BIT_FIELDS:
            y_start = 0.0
            y_end = 1.0
            y_step = 1.0
        else:
            if y_max <= y_min:
                padding = max(0.5, abs(y_min) * 0.05)
                y_min -= padding
                y_max += padding

            y_step = nice_step(y_max - y_min)
            y_start = math.floor(y_min / y_step) * y_step
            y_end = math.ceil(y_max / y_step) * y_step
            if y_end <= y_start:
                y_end = y_start + y_step

        canvas.create_rectangle(
            5, top + 5, width - 5, top + GRAPH_HEIGHT - 5,
            outline="#c8c8c8", fill="#ffffff",
        )
        canvas.create_text(
            14, top + 18, anchor="w", text=column,
            fill="#222222", font=("TkDefaultFont", 10, "bold"),
        )
        latest = points[-1][1]
        canvas.create_text(
            width - 14, top + 18, anchor="e",
            text=(f"samples={len(points):,}   latest={latest:.9g}   "
                  f"min={min(point[1] for point in points):.9g}   "
                  f"max={max(point[1] for point in points):.9g}"),
            fill="#444444",
        )

        y_tick = y_start
        tick_guard = 0
        while y_tick <= y_end + y_step * 0.001 and tick_guard < 20:
            y = plot_bottom - ((y_tick - y_start) / (y_end - y_start)) * (
                plot_bottom - plot_top
            )
            canvas.create_line(left, y, right, y, fill="#e4e4e4")
            canvas.create_text(
                left - 8, y, anchor="e", text=tick_text(y_tick, y_step),
                fill="#555555",
            )
            y_tick += y_step
            tick_guard += 1

        x_span = x_max - x_min
        for tick_index in range(6):
            fraction = tick_index / 5
            x_value = x_min + fraction * x_span
            x = left + fraction * (right - left)
            canvas.create_line(x, plot_top, x, plot_bottom, fill="#ededed")
            canvas.create_text(
                x, plot_bottom + 15, anchor="n",
                text=f"{x_value:.6g}", fill="#555555",
            )

        coordinates: list[float] = []
        for x_value, y_value in points:
            x = left + ((x_value - x_min) / x_span) * (right - left)
            y = plot_bottom - ((y_value - y_start) / (y_end - y_start)) * (
                plot_bottom - plot_top
            )
            coordinates.extend((x, y))

        color = PLOT_COLORS[graph_index % len(PLOT_COLORS)]
        if len(coordinates) >= 4:
            canvas.create_line(*coordinates, fill=color, width=1.5)
        elif coordinates:
            x, y = coordinates
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)

        canvas.create_text(
            (left + right) / 2, top + GRAPH_HEIGHT - 12,
            text=x_column, fill="#444444",
        )


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
