"""Horizon's launch dashboard (Qt + pyqtgraph): readouts left/right, up to six plots.

The three panels sit in a resizable splitter; the side panels scroll only when
the window is too small for them. Plots are pyqtgraph, redrawn on a timer
(layout refresh_ms, at most 30 Hz) from FlightData snapshots, never per packet.

JSON: flight_view_default.json or Flight controls > Load layout / --flight-layout.
The Layout Editor tab offers form-based editing, trace/row ordering and JSON
save/load. Draft edits do not affect live telemetry until explicitly applied.
The compact Flight controls menu contains layout, pause, clear, pad, time-window,
fullscreen and display-details actions. Live/test/stale warnings remain visible
beside it. No separate title/footer rows take space away from the dashboard.
Power and GPS readouts may contain two traces (left/right in listed order); the
default pairs AIM CPU/ejection batteries and Witness GPS satellites with the GPS
lock bit ("GPS sats 7  lock 1"). The single state row may add a second trace
with format hitl (Witness status.hitl_active): it is drawn under the flight
state, and HITL ACTIVE - SIMULATED SENSORS also appears in the banner.
Voltage fields display three decimal places.
  version: 1; title; window_seconds; refresh_ms; columns; max_points; gap_seconds
  auto_scale: true (default) fits each Y axis to all visible valid samples;
    false honors explicit left-axis y_range limits. Bounds do not force zero
    or round outward to tick steps. Padding depends on span, not DC offset.
  plots: [{title, unit, right_unit: optional, y_range: null or [min,max], traces: [...]}]
  readouts: [{group: flight|state|gps|power, title, unit, traces: [...]}]
  trace: {label, input: witness|featherweight|aim, packet: name or [names], field,
          source: "*", color: "#RRGGBB", scale: 1, offset: 0, match: {},
          enabled: true, note: "", format: number|state|continuity|flag|hitl,
          unit: override,
          axis: left|right}

Default six plots occupy the center 2x3 grid; flight readouts/state stay left,
GPS and power stay right. All dimensions follow the viewport.
Older plot-only JSON layouts also fit (no readouts). Legacy plot_height is
accepted but no longer fixes the physical height. Use at most six plots in
at most two columns and three rows; larger layouts are rejected, never hidden.

Fields match input tabs. AIM packet is its relative MQTT topic. '*' binds one
live source per input until Clear/reload. Exact sources are COM/feed names or
'host:port/base_path' for AIM. match can select a FW unit_id/type. Values are
decoded * scale + offset. Unknown AIM units remain labeled raw; unmapped
fields explicitly show N/A. To map continuity once topic/key is known, set
enabled=true and packet/field, format=continuity (0=OPEN, 1=CONTINUITY).

_derived.horizontal_distance_m is distance from the pad, not slant range:
hypot(E, N) of the Witness Kalman ENU position (its origin is the pad), and
great-circle distance of FW/AIM GPS lat/lon from the shared pad position
(pad_reference.py) or, before one is known, from the first valid live fix.
Altitudes are metres above the pad: Witness kalman_altitude (0x02/0xE0) is
the Kalman filter's up estimate, already relative to the pad. MSL sources use
derived fields with the pad's MSL altitude subtracted (Featherweight GS,
Aperture LC86G GPS, Witness on-pad GPS, first tracker fix, in that order):
_derived.altitude_agl_m (FW altitude_ft x 0.3048) and
_derived.gps_altitude_agl_m (Witness GPS, only with the GPS-locked bit).
Without a pad altitude they use their own first live value. AIM pressure
altitude is not MSL, so _derived.barometric_altitude_agl_m always uses its
first live value. Set pad re-references distance and these AGL fields; the
Kalman altitude itself is zeroed by Witness on the pad, not by Horizon.
AIM pressure_payload/v1 derives _derived.barometric_altitude_m
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
F11 toggles fullscreen; Escape exits fullscreen; Ctrl+P pauses / resumes.
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

from __future__ import annotations

import json
from pathlib import Path
import time

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from flight_data import DEFAULT_LAYOUT, FlightData, load_layout, validate_layout
import flight_render as fr
from qt_common import message_error
from qt_widgets import light_axes, set_background

WINDOW_CHOICES = (10, 30, 60, 120, 300, 600, 1800, 3600)
MIN_REDRAW_MS = 33


def show_error(parent, title, text):
    """Error dialog (tests patch this)."""
    message_error(parent, title, text)


def _label(text="", size=10, bold=False, color=fr.TEXT, wrap=False):
    label = QtWidgets.QLabel(text)
    font = label.font()
    font.setPointSize(size)
    font.setBold(bold)
    label.setFont(font)
    label.setStyleSheet(f"color: {color};")
    label.setWordWrap(wrap)
    label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
    return label


def _set(label, text, color=None):
    if label.text() != text:
        label.setText(text)
    if color is not None and label.property("fg") != color:
        label.setProperty("fg", color)
        label.setStyleSheet(f"color: {color};")


class ReadoutPanel(QtWidgets.QFrame):
    """A white bordered panel of labels, rebuilt when the layout changes."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName("readoutPanel")
        self.setStyleSheet(f"#readoutPanel {{ background: {fr.PANEL}; border: 1px solid {fr.BORDER}; }} "
                           f"QLabel {{ color: {fr.TEXT}; background: transparent; }}")
        self.layout_ = QtWidgets.QVBoxLayout(self)
        self.layout_.setContentsMargins(8, 6, 8, 6)
        self.layout_.setSpacing(2)
        self.title = _label(title, 12, True)
        self.layout_.addWidget(self.title)
        self.body = QtWidgets.QVBoxLayout()
        self.body.setSpacing(2)
        self.layout_.addLayout(self.body)
        self.layout_.addStretch(1)
        self.labels = []

    def clear_body(self):
        def clear(layout):
            while layout.count():
                item = layout.takeAt(0)
                if item.widget() is not None:
                    item.widget().deleteLater()
                elif item.layout() is not None:
                    clear(item.layout())
        clear(self.body)
        self.labels = []

    def texts(self):
        return [label.text() for label in self.labels]


class StatusPanel(ReadoutPanel):
    """GPS or power readouts (flight_render.status_rows)."""

    def show_rows(self, rows):
        shape = [len(cells) for _title, cells in rows]
        if shape != getattr(self, "_shape", None):
            self._shape = shape
            self.clear_body()
            self.rows = []
            for _title, cells in rows:
                title = _label(size=9, color=fr.MUTED)
                values = QtWidgets.QHBoxLayout()
                cell_labels = []
                for _ in cells:
                    label = _label(size=11, bold=True)
                    values.addWidget(label)
                    cell_labels.append(label)
                self.body.addWidget(title)
                self.body.addLayout(values)
                self.rows.append((title, cell_labels))
                self.labels += [title, *cell_labels]
            if not rows:
                label = _label("No readouts configured", 9, color=fr.MUTED)
                self.body.addWidget(label)
                self.labels.append(label)
        for (title, cells), (title_label, cell_labels) in zip(rows, self.rows):
            _set(title_label, title)
            for (text, color), label in zip(cells, cell_labels):
                _set(label, text, color)


class FlightPanel(ReadoutPanel):
    """Flight overview metrics and the Witness flight state."""

    def __init__(self, parent=None):
        super().__init__("FLIGHT OVERVIEW", parent)
        self.layout_.insertWidget(1, _label("W = Witness    A = AIM    F = Featherweight", 9, color=fr.MUTED))

    def show_panel(self, panel):
        shape = [len(cells) for _t, cells in panel["metrics"]]
        if shape != getattr(self, "_shape", None):
            self._shape = shape
            self.clear_body()
            self.metric_labels = []
            for _title, cells in panel["metrics"]:
                title = _label(size=11, bold=True)
                self.body.addWidget(title)
                pairs = []
                for _ in cells:
                    value, status = _label(size=14, bold=True), _label(size=9)
                    row = QtWidgets.QHBoxLayout()
                    row.addWidget(value, 3)
                    row.addWidget(status, 1)
                    self.body.addLayout(row)
                    pairs.append((value, status))
                self.metric_labels.append((title, pairs))
                self.labels += [title, *(label for pair in pairs for label in pair)]
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            self.body.addWidget(line)
            heading = _label("WITNESS FLIGHT STATE", 11, True)
            self.body.addWidget(heading)
            self.state_value = _label(size=22, bold=True)
            self.state_status = _label(size=10, bold=True)
            self.state_source = _label(size=9, color=fr.MUTED)
            self.state_hitl = _label(size=10, wrap=True)
            for label in (self.state_value, self.state_status, self.state_source, self.state_hitl):
                self.body.addWidget(label)
            notes = [_label("Distance: from the pad (Set pad re-references)", 9, color=fr.MUTED, wrap=True),
                     _label("A baro / Vz: estimates; GPS units raw", 9, color=fr.WARN, wrap=True)]
            for label in notes:
                self.body.addWidget(label)
            self.labels += [heading, self.state_value, self.state_status, self.state_source,
                            self.state_hitl, *notes]
        for (title, cells), (title_label, pairs) in zip(panel["metrics"], self.metric_labels):
            _set(title_label, title)
            for (text, color, status, status_color), (value, status_label) in zip(cells, pairs):
                _set(value, text, color)
                _set(status_label, status, status_color)
        state = panel["state"]
        if state is None:
            _set(self.state_value, "NOT CONFIGURED", fr.MUTED)
            for label in (self.state_status, self.state_source, self.state_hitl):
                _set(label, "")
            return
        _set(self.state_value, state["value"], state["color"])
        _set(self.state_status, state["status"], state["status_color"])
        _set(self.state_source, state["source"])
        hitl = state["hitl"]
        _set(self.state_hitl, hitl[0] if hitl else "", hitl[1] if hitl else None)


class GraphPanel(QtWidgets.QFrame):
    """One layout plot: legend row + pyqtgraph plot with optional right axis."""

    def __init__(self, plot, wall_offset, parent=None):
        super().__init__(parent)
        self.plot = plot
        self.wall_offset = wall_offset
        self.setObjectName("graphPanel")
        # Light panel whatever the system theme: trace colours are tuned for white.
        self.setStyleSheet(f"#graphPanel {{ background: {fr.PANEL}; border: 1px solid {fr.BORDER}; }} "
                           f"QLabel {{ color: {fr.TEXT}; background: transparent; }}")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(1)
        units = plot.unit + (f" | {plot.right_unit}" if plot.right_unit else "")
        self.title = _label(f"{plot.title} [{units}]", 11, True)
        layout.addWidget(self.title)
        legend = QtWidgets.QGridLayout()
        legend.setSpacing(1)
        self.legend_labels = []
        for index, _trace in enumerate(plot.traces):
            label = _label(size=9)
            legend.addWidget(label, index // 3, index % 3)
            self.legend_labels.append(label)
        layout.addLayout(legend)
        self.notice = _label("", 9, color=fr.MUTED)
        layout.addWidget(self.notice)
        self.widget = pg.PlotWidget(axisItems={"bottom": pg.DateAxisItem(orientation="bottom", utcOffset=0)})
        self.widget.setBackground(fr.BG)
        item = self.widget.getPlotItem()
        item.showGrid(x=True, y=True, alpha=0.3)
        item.setMenuEnabled(False)
        item.setMouseEnabled(x=False, y=False)
        item.hideButtons()
        item.setLabel("left", plot.unit)
        light_axes(item, fr.MUTED)
        self.widget.setMinimumHeight(90)
        layout.addWidget(self.widget, 1)
        self.right_view = None
        if plot.right_unit:
            self.right_view = pg.ViewBox()
            self.right_view.setMouseEnabled(x=False, y=False)
            item.showAxis("right")
            item.scene().addItem(self.right_view)
            item.getAxis("right").linkToView(self.right_view)
            item.getAxis("right").setLabel(plot.right_unit, color=fr.BAD)
            self.right_view.setXLink(item)
            item.vb.sigResized.connect(self._sync_right)
        self.curves = []
        for trace in plot.traces:
            style = QtCore.Qt.PenStyle.DashLine if trace.input == "aim" else QtCore.Qt.PenStyle.SolidLine
            curve = pg.PlotDataItem(pen=pg.mkPen(fr.readable_color(trace.color), width=2, style=style),
                                    connect="finite")
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            if trace.axis == "right" and self.right_view is not None:
                self.right_view.addItem(curve)
            else:
                item.addItem(curve)
            self.curves.append(curve)

    def _sync_right(self):
        self.right_view.setGeometry(self.widget.getPlotItem().vb.sceneBoundingRect())

    def update_plot(self, histories, layout, end):
        for (text, color), label in zip(fr.legend(self.plot, histories, end), self.legend_labels):
            _set(label, text, color)
        start = end - layout.window_seconds
        series = [fr.trace_arrays(h.points, start, end, layout.gap_seconds) for h in histories]
        for curve, (x, y), trace in zip(self.curves, series, self.plot.traces):
            if trace.enabled and len(x):
                curve.setData(x + self.wall_offset, y)
            else:
                curve.setData([], [])
        ranges, clipped = fr.axis_ranges(self.plot, series, layout, max(60, self.widget.height()))
        item = self.widget.getPlotItem()
        item.setXRange(start + self.wall_offset, end + self.wall_offset, padding=0)
        item.setYRange(*ranges["left"][:2], padding=0)
        if self.right_view is not None:
            self.right_view.setYRange(*ranges["right"][:2], padding=0)
        has_values = any(bool((y == y).any()) for _x, y in series if len(y))
        notice = "OUT OF RANGE" if clipped else "" if has_values else "NO LIVE SAMPLES"
        _set(self.notice, notice, fr.BAD if clipped else fr.MUTED)

    def texts(self):
        return [self.title.text(), *(label.text() for label in self.legend_labels), self.notice.text()]


class FlightView(QtWidgets.QWidget):
    def __init__(self, parent=None, *, layout_path=DEFAULT_LAYOUT, pad=None):
        super().__init__(parent)
        self.paused, self.frozen = False, None
        self.wall_offset = time.time() - time.monotonic()
        self.layout_path = Path(layout_path)
        self.load_error = None
        self.redraws = 0
        try:
            layout = load_layout(self.layout_path)
        except (OSError, ValueError) as exc:
            self.load_error = str(exc)
            layout = validate_layout({"version": 1, "plots": [{"title": "Altitude above pad", "unit": "m",
                "traces": [{"label": "W", "input": "witness", "packet": "state", "field": "kalman_altitude"}]}]})
        self.data = FlightData(layout, pad=pad)
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(3)
        self.actions_button = QtWidgets.QToolButton()
        self.actions_button.setText("Flight controls")
        self.actions_button.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
        self.actions_menu = QtWidgets.QMenu(self)
        self.actions_button.setMenu(self.actions_menu)
        self._add_action("Load layout...", self.choose_layout)
        self._add_action("Save layout...", self.save_layout)
        self._add_action("Default layout", lambda: self.load_config(DEFAULT_LAYOUT))
        self.actions_menu.addSeparator()
        self.pause_action = self._add_action("Pause view", self.toggle_pause, "Ctrl+P")
        self._add_action("Clear plots", self.clear)
        self._add_action("Set pad", self.set_pad)
        self.window_menu = self.actions_menu.addMenu("Graph time window")
        self.window_group = QtGui.QActionGroup(self)
        self.window_group.setExclusive(True)
        self.sync_window()
        self.fullscreen_action = self._add_action("Fullscreen", self.toggle_fullscreen, "F11")
        self.actions_menu.addSeparator()
        self._add_action("Layout / display details...", self.show_details)
        escape = QtGui.QShortcut(QtGui.QKeySequence("Escape"), self)
        escape.setContext(QtCore.Qt.ShortcutContext.WindowShortcut)
        escape.activated.connect(self.exit_fullscreen)
        self.banner = _label(size=11, bold=True)
        self.banner.setContentsMargins(8, 2, 8, 2)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(self.actions_button)
        toolbar.addWidget(self.banner, 1)
        root.addLayout(toolbar)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.flight_panel = FlightPanel()
        self.gps_panel = StatusPanel("GPS / SATELLITES")
        self.power_panel = StatusPanel("POWER")
        self.graph_area = QtWidgets.QWidget()
        self.graph_grid = QtWidgets.QGridLayout(self.graph_area)
        self.graph_grid.setContentsMargins(0, 0, 0, 0)
        self.graph_grid.setSpacing(6)
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self.gps_panel)
        right_layout.addWidget(self.power_panel, 1)
        for widget, minimum in ((self.flight_panel, 170), (self.graph_area, 300), (right, 160)):
            if widget is not self.graph_area:
                scroll = QtWidgets.QScrollArea()
                scroll.setWidgetResizable(True)
                scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
                scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                scroll.setWidget(widget)
                widget = scroll
            widget.setMinimumWidth(minimum)
            self.splitter.addWidget(widget)
        for index, stretch in enumerate((26, 55, 19)):
            self.splitter.setStretchFactor(index, stretch)
        self.splitter.setSizes([260, 550, 190])
        root.addWidget(self.splitter, 1)
        self.graphs = []
        self._build_graphs()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(max(MIN_REDRAW_MS, self.data.layout.refresh_ms))
        self.redraw()

    def _add_action(self, label, slot, key=None):
        action = self.actions_menu.addAction(label)
        action.triggered.connect(lambda _checked=False: slot())
        if key:
            action.setShortcut(QtGui.QKeySequence(key))
            action.setShortcutContext(QtCore.Qt.ShortcutContext.WindowShortcut)
            self.addAction(action)
        return action

    def menu_labels(self):
        return [action.text() for action in self.actions_menu.actions() if not action.isSeparator()]

    def _build_graphs(self):
        for graph in self.graphs:
            self.graph_grid.removeWidget(graph)
            graph.deleteLater()
        layout = self.data.layout
        columns = max(1, min(layout.columns, len(layout.plots)))
        self.graphs = []
        for index, plot in enumerate(layout.plots):
            graph = GraphPanel(plot, self.wall_offset)
            row, column = divmod(index, columns)
            self.graph_grid.addWidget(graph, row, column)
            self.graphs.append(graph)
        rows = -(-len(layout.plots) // columns)
        for row in range(6):
            self.graph_grid.setRowStretch(row, 1 if row < rows else 0)

    # ------------------------------------------------------------ actions
    def toggle_fullscreen(self):
        window = self.window()
        if window.isFullScreen():
            window.showNormal()
        else:
            window.showFullScreen()

    def exit_fullscreen(self):
        if self.window().isFullScreen():
            self.window().showNormal()

    def choose_layout(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Flight View layout", "", "JSON layout (*.json)")
        if filename:
            self.load_config(filename)

    def sync_window(self):
        seconds = self.data.layout.window_seconds
        for action in self.window_group.actions():
            self.window_group.removeAction(action)
        self.window_menu.clear()
        for value in sorted({*WINDOW_CHOICES, seconds}):
            action = self.window_menu.addAction(f"{value:g} s")
            action.setCheckable(True)
            action.setChecked(value == seconds)
            action.triggered.connect(lambda _c=False, v=value: self.select_window(v))
            self.window_group.addAction(action)

    def window_label(self):
        checked = self.window_group.checkedAction()
        return checked.text() if checked else ""

    def select_window(self, seconds):
        """Change the visible time span; history, sources and the pause snapshot stay."""
        try:
            self.data.set_window(float(seconds))
        except (ValueError, TypeError):
            pass
        self.sync_window()
        self.redraw()

    def show_details(self):
        QtWidgets.QMessageBox.information(self, "Flight View details", self.display_details())

    def display_details(self):
        return (f"{self.layout_path.name}\n{self.data.layout.window_seconds:g}s window | UTC receive time\n"
                "Solid W / dashed A | A gyro ~ = quaternion estimate\n"
                "! old/invalid | History bounded by max_points\n"
                "AIM GPS altitude remains on the Altitude graph. FW fix mode is on the Featherweight GPS tab.")

    def load_config(self, filename):
        try:
            layout = load_layout(filename)
        except (OSError, ValueError) as exc:
            show_error(self, "Flight View layout", str(exc))
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
        self.pause_action.setText("Pause view")
        self._timer.setInterval(max(MIN_REDRAW_MS, layout.refresh_ms))
        self._build_graphs()
        self.redraw()

    def save_layout(self, filename=None):
        if filename is None:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save Flight View layout", "flight_view_custom.json", "JSON layout (*.json)")
        if not filename:
            return False
        try:
            Path(filename).write_text(json.dumps(self.data.layout.document, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            show_error(self, "Save Flight View layout", str(exc))
            return False
        return True

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
        self.pause_action.setText("Resume live" if self.paused else "Pause view")
        self.redraw()

    # ------------------------------------------------------------- render
    def redraw(self, end=None):
        """Render the current (or frozen) snapshot. ``end`` overrides 'now' (tests)."""
        if self.paused:
            frozen_end, histories = self.frozen
            end = frozen_end if end is None else end
        else:
            histories = self.data.snapshot()
            end = time.monotonic() if end is None else end
        self.redraws += 1
        layout = self.data.layout
        text, background = fr.banner(layout, histories, end, paused=self.paused, load_error=self.load_error)
        _set(self.banner, text)
        set_background(self.banner, background, fr.TEXT)
        self.flight_panel.show_panel(fr.flight_panel(layout, histories, end))
        self.gps_panel.show_rows(fr.status_rows(layout, histories, end, "gps"))
        self.power_panel.show_rows(fr.status_rows(layout, histories, end, "power"))
        for graph, data in zip(self.graphs, histories):
            graph.update_plot(data, layout, end)

    def texts(self):
        """Every readout / legend text currently shown (tests and diagnostics)."""
        out = [self.banner.text(), *self.flight_panel.texts(), *self.gps_panel.texts(), *self.power_panel.texts()]
        for graph in self.graphs:
            out += graph.texts()
        return [text for text in out if text]

    def _poll(self):
        if self.isVisible():
            self.redraw()

    def close(self):
        self._timer.stop()
        return super().close()
