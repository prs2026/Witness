"""Horizon's single-pane launch dashboard. Nothing on this tab scrolls.

JSON: flight_view_default.json or Load layout / --flight-layout.
  version: 1; title; window_seconds; refresh_ms; columns; max_points; gap_seconds
  plots: [{title, unit, y_range: null or [min,max], traces: [...]}]
  readouts: [{group: flight|state|gps|power, title, unit, traces: [...]}]
  trace: {label, input: witness|featherweight|aim, packet: name or [names], field,
          source: "*", color: "#RRGGBB", scale: 1, offset: 0, match: {},
          enabled: true, note: "", format: number|state|continuity, unit: override}

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
zero altitudes. AIM vertical velocity is unmapped, not guessed.

Host receive time (UTC) is the graph clock. Cached values aren't live samples.
Pause freezes display only; receiving/logging continue. ! marks old/invalid
graph values; readouts show age. Clear resets history but never logs.
F11 toggles fullscreen; Escape exits fullscreen. No extra dependencies.
White panels and dark text are used throughout the dashboard. Pale custom or
legacy trace colors are darkened for readability without changing saved JSON.
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
        self.rowconfigure(2, weight=1)
        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        for label, command in (("Load layout", self.choose_layout), ("Save layout", self.save_layout),
                               ("Default", lambda: self.load_config(DEFAULT_LAYOUT))):
            ttk.Button(toolbar, text=label, command=command, width=11).pack(side="left", padx=(0, 4))
        self.pause_button = ttk.Button(toolbar, text="Pause view", command=self.toggle_pause, width=12)
        self.pause_button.pack(side="left", padx=4)
        ttk.Button(toolbar, text="Clear plots", command=self.clear, width=11).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Set pad", command=self.set_pad, width=10).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Fullscreen (F11)", command=self.toggle_fullscreen, width=18).pack(side="right")
        self.banner = tk.Label(self, anchor="w", foreground=TEXT, font=("Segoe UI", 11, "bold"),
                               height=1, width=1, padx=8, pady=4)
        self.banner.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        self.canvas = tk.Canvas(self, background=BG, highlightthickness=0, width=1, height=1)
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.renderer = DashboardRenderer(self.canvas)
        self.info = ttk.Label(self, text="", width=1, anchor="w", font=("Segoe UI", 9))
        self.info.grid(row=3, column=0, sticky="ew", pady=(3, 0))
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

    def load_config(self, filename):
        try:
            layout = load_layout(filename)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Flight View layout", str(exc), parent=self)
            return False
        self.data.configure(layout)
        self.layout_path = Path(filename)
        self.load_error = None
        self.paused, self.frozen = False, None
        self.pause_button.configure(text="Pause view")
        self.redraw()
        return True

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
        self.pause_button.configure(text="Resume live" if self.paused else "Pause view")
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
        self.banner.configure(text=f"{warning}{state}",
                              background="#fff2a8" if self.paused or simulated or self.load_error else "#d8efdf" if fresh else "#ffd5cf")
        self.info.configure(text=f"{self.layout_path.name} | {self.data.layout.window_seconds:g}s window | UTC receive time | "
                            "! = old/invalid graph value | Input tabs show full source/quality details")
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
