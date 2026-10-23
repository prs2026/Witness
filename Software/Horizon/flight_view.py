"""Horizon's single-pane launch dashboard. Nothing on this tab scrolls.

JSON: flight_view_default.json or Flight controls > Load layout / --flight-layout.
The compact Flight controls menu contains layout, pause, clear, pad, time-window,
fullscreen and display-details actions. Live/test/stale warnings remain visible
beside it. No separate title/footer rows take space away from the dashboard.
Power readouts may contain two traces (left/right in listed order); the default
pairs AIM CPU/ejection batteries. Voltage fields display three decimal places.
  version: 1; title; window_seconds; refresh_ms; columns; max_points; gap_seconds
  auto_scale: true (default) fits each Y axis to all visible valid samples;
    false honors explicit left-axis y_range limits. Bounds do not force zero
    or round outward to tick steps. Padding depends on span, not DC offset.
  plots: [{title, unit, right_unit: optional, y_range: null or [min,max], traces: [...]}]
  readouts: [{group: flight|state|gps|power, title, unit, traces: [...]}]
  trace: {label, input: witness|featherweight|aim, packet: name or [names], field,
          source: "*", color: "#RRGGBB", scale: 1, offset: 0, match: {},
          enabled: true, note: "", format: number|state|continuity, unit: override,
          axis: left|right}

Default six plots occupy the center 2x3 grid; flight readouts/state stay left,
GPS and power/continuity stay right. All dimensions follow the viewport.
Older plot-only JSON layouts also fit (no readouts). Legacy plot_height is
accepted but no longer fixes the physical height. Use at most six plots in
at most two columns and three rows; larger layouts are rejected, never hidden.

Fields match input tabs. AIM packet is its relative MQTT topic. '*' binds one
live source per input until Clear/reload. Exact sources are COM/feed names or
'host:port/base_path' for AIM. match can select a FW unit_id/type. Values are
decoded * scale + offset. Unknown AIM units remain labeled raw; unmapped
fields explicitly show N/A. To map continuity once topic/key is known, set
enabled=true and packet/field, format=continuity (0=OPEN, 1=CONTINUITY).

_derived.horizontal_distance_m uses Witness local X/Y or FW/AIM GPS lat/lon
degrees. First valid live position is the reference; Set pad releases origins
for the next live position. This is not slant range. Witness altitude remains
pressure-referenced, FW altitude is converted from ft ASL. Set pad does not
zero altitudes. AIM pressure_payload/v1 derives _derived.barometric_altitude_m
from pressure interpreted as Pa using a 101325 Pa standard atmosphere reference.
_derived.vertical_velocity_mps is its altitude difference / publisher time
difference, positive upwards, not AIM's fused velocity. A baro~ and A~ denote
these estimates. GPS altitude stays on a separate raw-unit graph axis and GPS
trace until publisher units are verified. Two fresh pressure samples within
gap_seconds are required for velocity; duplicate/retained/backwards times are
ignored and gaps/invalid pressure re-prime it. Clear/reload resets both AIM
estimators. Invalid samples hold the last valid displayed value and original
age, flag it INVALID, and leave a gap in graphs; they never plot held values
as new samples. Raw packet/error logs remain unchanged.

Featherweight RX_NOMTK supplies tracker_battery_voltage_v and rssi_dbm;
BATT_BLE supplies battery_voltage_v for the LOCAL serial-connected device
(normally the ground station). These batteries must not be combined. Zero
startup voltages remain unavailable. GPS unit type TRK is normalized to
TRACKER by the parser, with the original token retained as unit_type_raw.

Host receive time (UTC) is the graph clock. Cached values aren't live samples.
Pause freezes display only; receiving/logging continue. ! marks old/invalid
graph values; readouts show age. Clear resets history but never logs.
F11 toggles fullscreen; Escape exits fullscreen. No extra dependencies.
White panels and dark text are used throughout the dashboard. Pale custom or
legacy trace colors are darkened for readability without changing saved JSON.
Window selects the visible timespan without clearing history, including while
paused. Save layout persists it. History remains bounded by max_points / one
hour, so a longer selected window may contain empty space at the start.
AIM BASE 4.3.0 publishes Z acceleration but its units are unverified: it uses
the right raw axis. AIM gyro traces AX~/AY~/AZ~ are ESTIMATES from quaternion
differences, not raw measurements. They use orientation_payload/v1 and derived
fields _derived.gyro_dps[0..2], in deg/s on the same axis as Witness. The sample
timestamp is microseconds, independent of the graph's host receive clock.
The calculation assumes Hamilton body-to-world quaternions; AIM axis convention
and mounting relative to Witness have not been verified. Duplicates, retained
samples and backwards times produce no estimate; invalid data or a gap longer
than gap_seconds re-primes the estimator. No extrapolation across missing data.
Clear/reload resets the estimator; Pause/time-window changes do not. More than
180 degrees of rotation between samples is ambiguous. Witness uses solid lines;
AIM uses dashed lines. See aim_orientation.py for the calculation and limits.
"""

import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from flight_data import DEFAULT_LAYOUT, FlightData, load_layout, validate_layout
from flight_render import BG, TEXT, DashboardRenderer


class FlightView(ttk.Frame):
    def __init__(self, parent, *, layout_path=DEFAULT_LAYOUT):
        super().__init__(parent, padding=5)
        self._after_id = None
        self.paused, self.frozen = False, None
        self.wall_offset = time.time() - time.monotonic()
        self.layout_path = Path(layout_path)
        self.load_error = None
        try:
            layout = load_layout(self.layout_path)
        except (OSError, ValueError) as exc:
            self.load_error = str(exc)
            layout = validate_layout({"version": 1, "plots": [{"title": "Barometric altitude", "unit": "m",
                "traces": [{"label": "W", "input": "witness", "packet": "state", "field": "barometric_altitude"}]}]})
        self.data = FlightData(layout)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 2))
        toolbar.columnconfigure(1, weight=1)
        self.actions_button = ttk.Menubutton(toolbar, text="Flight controls", direction="below")
        self.actions_button.grid(row=0, column=0, padx=(0, 6))
        self.actions_menu = tk.Menu(self.actions_button, tearoff=False)
        self.actions_button.configure(menu=self.actions_menu)
        for label, command in (("Load layout...", self.choose_layout), ("Save layout...", self.save_layout),
                               ("Default layout", lambda: self.load_config(DEFAULT_LAYOUT))):
            self.actions_menu.add_command(label=label, command=command)
        self.actions_menu.add_separator()
        self.pause_menu_index = self.actions_menu.index("end") + 1
        self.actions_menu.add_command(label="Pause view", command=self.toggle_pause)
        self.actions_menu.add_command(label="Clear plots", command=self.clear)
        self.actions_menu.add_command(label="Set pad", command=self.set_pad)
        self.window_var = tk.StringVar()
        self.window_menu = tk.Menu(self.actions_menu, tearoff=False)
        self.actions_menu.add_cascade(label="Graph time window", menu=self.window_menu)
        self.sync_window()
        self.actions_menu.add_command(label="Fullscreen", accelerator="F11", command=self.toggle_fullscreen)
        self.actions_menu.add_separator()
        self.actions_menu.add_command(label="Layout / display details...", command=self.show_details)
        self.banner = tk.Label(toolbar, anchor="w", foreground=TEXT, font=("Segoe UI", 11, "bold"),
                               height=1, width=1, padx=8, pady=2)
        self.banner.grid(row=0, column=1, sticky="ew")
        self.canvas = tk.Canvas(self, background=BG, highlightthickness=0, width=1, height=1)
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.renderer = DashboardRenderer(self.canvas)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Map>", lambda _event: self.redraw())
        # Consume wheel events; there is no hidden content to scroll to.
        for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.canvas.bind(event, lambda _event: "break")
        self._top = self.winfo_toplevel()
        self._f11 = self._top.bind("<F11>", self.toggle_fullscreen, add="+")
        self._escape = self._top.bind("<Escape>", self.exit_fullscreen, add="+")
        self.redraw()
        self._poll()

    def toggle_fullscreen(self, _event=None):
        current = bool(self._top.tk.getboolean(self._top.attributes("-fullscreen")))
        self._top.attributes("-fullscreen", not current)
        return "break"

    def exit_fullscreen(self, _event=None):
        self._top.attributes("-fullscreen", False)
        return "break"

    def choose_layout(self):
        filename = filedialog.askopenfilename(parent=self, title="Flight View layout", filetypes=(("JSON layout", "*.json"),))
        if filename:
            self.load_config(filename)

    def sync_window(self):
        seconds = self.data.layout.window_seconds
        self.window_menu.delete(0, "end")
        for value in sorted({10, 30, 60, 120, 300, 600, 1800, 3600, seconds}):
            self.window_menu.add_radiobutton(label=f"{value:g} s", value=f"{value:g} s",
                                            variable=self.window_var, command=self.select_window)
        self.window_var.set(f"{seconds:g} s")

    def show_details(self):
        messagebox.showinfo("Flight View details", self.display_details(), parent=self)

    def display_details(self):
        return (f"{self.layout_path.name}\n{self.data.layout.window_seconds:g}s window | UTC receive time\n"
                "Solid W / dashed A | A gyro ~ = quaternion estimate\n"
                "! old/invalid | History bounded by max_points\n"
                "AIM GPS altitude remains on the Altitude graph. FW fix mode is on the Featherweight GPS tab.")

    def select_window(self, _event=None):
        try:
            self.data.set_window(float(self.window_var.get().split()[0]))
        except (ValueError, IndexError):
            self.sync_window()
            return
        self.redraw()

    def load_config(self, filename):
        try:
            layout = load_layout(filename)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Flight View layout", str(exc), parent=self)
            return False
        self.apply_layout(layout, Path(filename))
        return True

    def apply_layout(self, layout, path=None):
        """Apply a validated editor/file layout; receiver connections stay intact."""
        self.data.configure(layout)
        self.sync_window()
        if path is not None:
            self.layout_path = Path(path)
        self.load_error = None
        self.paused, self.frozen = False, None
        self.actions_menu.entryconfigure(self.pause_menu_index, label="Pause view")
        self.redraw()

    def save_layout(self):
        filename = filedialog.asksaveasfilename(parent=self, title="Save Flight View layout", defaultextension=".json",
                                              initialfile="flight_view_custom.json", filetypes=(("JSON layout", "*.json"),))
        if filename:
            try:
                Path(filename).write_text(json.dumps(self.data.layout.document, indent=2) + "\n", encoding="utf-8")
            except OSError as exc:
                messagebox.showerror("Save Flight View layout", str(exc), parent=self)

    def clear(self):
        self.data.clear()
        if self.paused:
            self.frozen = (time.monotonic(), self.data.snapshot())
        self.redraw()

    def set_pad(self):
        self.data.reset_pad()
        if self.paused:
            self.frozen = (time.monotonic(), self.data.snapshot())
        self.redraw()

    def toggle_pause(self):
        self.paused = not self.paused
        self.frozen = (time.monotonic(), self.data.snapshot()) if self.paused else None
        self.actions_menu.entryconfigure(self.pause_menu_index, label="Resume live" if self.paused else "Pause view")
        self.redraw()

    def redraw(self):
        end, histories = self.frozen if self.paused else (time.monotonic(), self.data.snapshot())
        flat = [(trace, history) for row, data in zip(self.data.layout.rows, histories)
                for trace, history in zip(row.traces, data) if trace.enabled]
        fresh = sum(h.quality == "LIVE" and h.received is not None and end - h.received < 2 for _, h in flat)
        simulated = any(h.source.startswith("TEST_") for _, h in flat)
        state = "PAUSED - DISPLAY FROZEN; receiving/logging continues" if self.paused else (
            f"LIVE - {fresh}/{len(flat)} readings fresh" if fresh else "WAITING / STALE - no fresh live readings")
        warning = "TEST DATA | " if simulated else ""
        if self.load_error:
            warning = "LAYOUT ERROR (fallback active) | " + warning
        self.banner.configure(text=f"{warning}{state} | {self.data.layout.window_seconds:g}s",
                              background="#fff2a8" if self.paused or simulated or self.load_error else "#d8efdf" if fresh else "#ffd5cf")
        self.renderer.draw(self.data.layout, histories, end, self.wall_offset)

    def _poll(self):
        try:
            if self.winfo_ismapped():
                self.redraw()
        finally:
            self._after_id = self.after(self.data.layout.refresh_ms, self._poll)

    def close(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        self._top.unbind("<F11>", self._f11)
        self._top.unbind("<Escape>", self._escape)
