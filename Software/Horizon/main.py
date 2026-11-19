"""Witness monitor widget (PyQt6 + pyqtgraph), embedded by witnessOS.

The standalone monitor window is gone: the monitor is the "Witness monitor"
tab of witnessOS (python Software/witnessOS/witnessos.py --tab monitor).
Running this file only prints that note.

MonitorPanel is the Qt view over monitor_core.MonitorCore (inputs, decoding,
logs, Horizon JSON feed, commands, test source; no Qt). The protocol helpers
(parse_packet, PacketDecoder, build_*, QUICK_COMMANDS, ...) live in
witness_protocol and are re-exported here for existing importers.

Keys inside the panel: Enter sends the typed hex command, Up/Down recall sent
commands, F5 refreshes the port lists, Ctrl+1 / Ctrl+2 switch Telemetry /
Graphs.
"""

from __future__ import annotations

import sys
import threading
import time
import traceback

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from link_health import RateLimited
from qt_common import Throttle, color, make_plot, message_error
import monitor_core
from monitor_core import (  # noqa: F401 - re-exported API
    BAUD_CHOICES, DEFAULT_TCP_1_HOST, DEFAULT_TCP_1_PORT, DEFAULT_TCP_2_HOST, DEFAULT_TCP_2_PORT,
    DEFAULT_WITNESS_FEED_HOST, DEFAULT_WITNESS_FEED_PORT, LOG_DIR, NO_SERIAL_PORT, ROOT, WINDOW_TITLE,
    RESET_TO_PAD_OPCODE, MonitorCore, RingHistory,
)
from flight_render import STRIP_COLORS
from qt_widgets import (LogConsole, RetilingGrid, age_color, bind_check, bind_combo, bind_enabled,
                        bind_label, bind_line_edit, hbox, scroll_area, set_background, wrap_label, _owned)
from witness_protocol import *  # noqa: F401,F403 - re-exported protocol API
from witness_protocol import status_maintenance  # noqa: E402
from witness_protocol import (  # explicit for linters / readers
    CAMERA_STATUS_FLAG_LABELS, FLIGHT_STATE_LABELS, IMU_CALIBRATION_COMMANDS, QUICK_COMMANDS,
    LOG_SLOTS_FULL_MASK, SECTION_SPECS, STATUS_BYTE1_FLAG_LABELS, STATUS_FLAG_LABELS, PacketDecoder,
    build_command_packet,
    build_imu_calibration_packet, decode_status, format_display_value, format_gps_satellites,
    is_status_field, parse_packet,
)

OFF_BG, OFF_FG, ON_FG = "#f7f7f7", "#777777", "#111111"
MAINTENANCE_STATUS_KEYS = ("packet.state.status", "packet.sensors.status")
# The maintenance image sends no state/sensors packets, only the 0x08 beacon:
# its arrival shows the banner, and the next flight packet clears it.
MAINTENANCE_BEACON_KEY = "packet.maintenance_status.uptime"
MAINTENANCE_TITLE_PREFIX = "MAINTENANCE - NOT FLIGHT READY | "
MAINTENANCE_BANNER_TEXT = ("WITNESS IN MAINTENANCE MODE (Wi-Fi OTA / log download)\n"
                           "FLIGHT LOGGING OFF - NOT READY FOR FLIGHT - turn Wi-Fi off to reboot into flight mode")
GOOD_BG, BAD_BG, STATE_BG, WARN_BG = "#74c476", "#fb6a4a", "#6baed6", "#fff2a8"
EVENT_MS = 30          # event pump: drains the worker queue (batched)
GRAPH_MS = 50          # graph redraw throttle (<= 20 Hz)
AGE_MS = 100
AUTO_MS = 250
GRAPH_MIN_HEIGHT = 210


# ---------------------------------------------------------------- safety net
# PyQt6 aborts the whole process (qFatal) when a slot raises and sys.excepthook is
# still the default. During a flight that must never happen (audit H2): install
# a hook that logs the traceback to a file and tells the operator instead.
EXCEPTION_LOG = LOG_DIR / "horizon_exceptions.log"
_exception_sinks = []
_exception_limit = RateLimited(5.0)
_hook_installed = False


def _record_exception(where, exc_type, exc, tb):
    """Log one uncaught exception; never raises. Repeats are rate-limited per type."""
    try:
        text = "".join(traceback.format_exception(exc_type, exc, tb))
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        try:
            EXCEPTION_LOG.parent.mkdir(parents=True, exist_ok=True)
            with EXCEPTION_LOG.open("a", encoding="utf-8") as handle:
                handle.write(f"[{stamp}] {where}\n{text}\n")
        except Exception:  # noqa: BLE001 - a full disk must not make this worse
            pass
        try:
            sys.stderr.write(f"[{stamp}] {where}\n{text}")
        except Exception:  # noqa: BLE001
            pass
        name = getattr(exc_type, "__name__", "?")
        allowed, skipped = _exception_limit.allow((where, name))
        if allowed:
            summary = f"UNCAUGHT {where}: {name}: {exc}"
            if skipped:
                summary += f" ({skipped} similar suppressed)"
            for sink in list(_exception_sinks):
                try:
                    sink(summary)
                except Exception:  # noqa: BLE001
                    pass
    except Exception:  # noqa: BLE001
        pass


def install_global_excepthook(sink=None):
    """Install sys.excepthook, threading.excepthook and sys.unraisablehook (once).
    `sink(summary_line)` is called (rate-limited, maybe on any thread) for the
    in-app console / status bar. Safe to call repeatedly; returns True the first time."""
    global _hook_installed
    if sink is not None and sink not in _exception_sinks:
        _exception_sinks.append(sink)
    if _hook_installed:
        return False
    _hook_installed = True
    sys.excepthook = lambda t, e, tb: _record_exception("GUI/main thread", t, e, tb)
    threading.excepthook = lambda a: _record_exception(
        f"thread {getattr(a.thread, 'name', '?')}", a.exc_type, a.exc_value, a.exc_traceback)
    sys.unraisablehook = lambda u: _record_exception(
        "unraisable", type(u.exc_value), u.exc_value, u.exc_traceback)
    return True


def show_error(parent, title, text):
    """Error dialog (tests patch this)."""
    message_error(parent, title, text)


def ask_yes_no(parent, title, text) -> bool:
    answer = QtWidgets.QMessageBox.question(
        parent, title, text,
        QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
        QtWidgets.QMessageBox.StandardButton.No)
    return answer == QtWidgets.QMessageBox.StandardButton.Yes


def _indicator(text, width=72):
    label = QtWidgets.QLabel(text)
    label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    label.setWordWrap(True)
    label.setMinimumWidth(width)
    label.setFrameShape(QtWidgets.QFrame.Shape.Box)
    set_background(label, OFF_BG, OFF_FG)
    return label


class StatusIndicator(QtWidgets.QWidget):
    """Decoded status bits: byte-0 flags, flight state, GPS lock and HITL."""

    def __init__(self, camera=False, parent=None):
        super().__init__(parent)
        self.camera = camera
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 0, 0, 4)
        layout.setSpacing(2)
        layout.addWidget(QtWidgets.QLabel("Byte 0: camera flags" if camera else "Byte 0: system flags"))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(2)
        self.flags = {}
        for index, (mask, text) in enumerate(CAMERA_STATUS_FLAG_LABELS if camera else STATUS_FLAG_LABELS):
            label = _indicator(text)
            grid.addWidget(label, index // 4, index % 4)
            self.flags[mask] = label
        layout.addLayout(grid)
        self.states, self.invalid, self.byte1 = {}, None, {}
        if camera:
            layout.addWidget(QtWidgets.QLabel("Byte 1: reserved"))
            return
        layout.addWidget(QtWidgets.QLabel("Byte 1 bits 2:0: flight state"))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(2)
        for index, (state, text) in enumerate(FLIGHT_STATE_LABELS):
            label = _indicator(text)
            grid.addWidget(label, index // 3, index % 3)
            self.states[state] = label
        self.invalid = _indicator("Invalid")
        grid.addWidget(self.invalid, 1, 2)
        layout.addLayout(grid)
        layout.addWidget(QtWidgets.QLabel("Byte 1 bit 3: GPS lock; bit 4: HITL; bit 5: maintenance"))
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(2)
        for mask, text in STATUS_BYTE1_FLAG_LABELS:
            label = _indicator(f"{text} 0", 100)
            row.addWidget(label)
            self.byte1[mask] = (text, label)
        row.addStretch(1)
        layout.addLayout(row)

    def show_value(self, value):
        if not isinstance(value, int):
            for label in (*self.flags.values(), *self.states.values()):
                set_background(label, OFF_BG, OFF_FG)
            if self.invalid is not None:
                self.invalid.setText("Invalid")
                set_background(self.invalid, OFF_BG, OFF_FG)
            for text, label in self.byte1.values():
                label.setText(f"{text} 0")
                set_background(label, OFF_BG, OFF_FG)
            return
        decoded = decode_status(value, self.camera)
        for mask, _text, active, error in decoded["flags"]:
            set_background(self.flags[mask], (BAD_BG if error else GOOD_BG) if active else OFF_BG,
                           ON_FG if active else OFF_FG)
        if self.camera:
            return
        for state, label in self.states.items():
            active = decoded["state"] == state
            set_background(label, STATE_BG if active else OFF_BG, ON_FG if active else OFF_FG)
        invalid = not decoded["state_valid"]
        self.invalid.setText(f"Invalid ({decoded['state']})" if invalid else "Invalid")
        set_background(self.invalid, BAD_BG if invalid else OFF_BG, ON_FG if invalid else OFF_FG)
        for mask, text, active, warning in decoded["byte1"]:
            label = self.byte1[mask][1]
            label.setText(f"{text} {int(active)}")
            set_background(label, (BAD_BG if warning else GOOD_BG) if active else OFF_BG,
                           ON_FG if active else OFF_FG)


class GraphPlot:
    """One selected field: a pyqtgraph plot with a curve per vector component."""

    COMPONENTS = {3: ("x", "y", "z"), 4: ("x", "y", "z", "w")}

    def __init__(self, key):
        self.key = key
        self.widget = make_plot(title=key, x_time=True, legend=True)
        self.widget.setMinimumHeight(GRAPH_MIN_HEIGHT)
        self.curves = []

    def _ensure_curves(self, width):
        if len(self.curves) == width:
            return
        item = self.widget.getPlotItem()
        for curve in self.curves:
            item.removeItem(curve)
        if item.legend is not None:
            item.legend.clear()
        names = self.COMPONENTS.get(width, tuple(f"[{i}]" for i in range(width)))
        self.curves = []
        for index in range(width):
            curve = item.plot(name=names[index] if width > 1 else None,
                              pen=pg.mkPen(color(index), width=1.5))
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            self.curves.append(curve)

    def update(self, history):
        field = self.key.rsplit(".", 1)[-1]
        if history is None or not len(history):
            for curve in self.curves:
                curve.setData([], [])
            self.widget.setTitle(f"{self.key} - waiting for data")
            return
        times, values = history.arrays()
        self._ensure_curves(values.shape[1])
        for index, curve in enumerate(self.curves):
            curve.setData(times, values[:, index], connect="finite")
        latest = format_display_value(field, history.last_value)
        self.widget.setTitle(f"{self.key}    Latest: {latest}")


class MonitorPanel(QtWidgets.QWidget):
    """The monitor as an embeddable widget (witnessOS) over a MonitorCore.

    The host owns the window: it calls close() on exit (stops inputs, the test
    source and the feed) and gets title changes through ``on_title``.
    Unknown attributes are looked up on ``self.core`` (values, check_vars,
    events, publisher, serials, start_test, stream_feeder, console_write, ...).
    """

    titleChanged = QtCore.pyqtSignal(str)
    # "LINK LOST - ..." text (empty when the link is fine): a host window can show it
    # in its own status bar; the panel also shows it as a banner and in the title.
    linkBannerChanged = QtCore.pyqtSignal(str)

    def __init__(self, parent=None, *, on_title=None, **options):
        super().__init__(parent)
        self.on_title = on_title
        self.core = MonitorCore(**options)
        self.core.alert = self._alert
        self.core.confirm = lambda title, text: ask_yes_no(self, title, text)
        self.graph_draws = 0
        self._plots = {}
        self._graph_layout_dirty = True
        self._shut_down = False
        self.age_labels = {}
        self.status_indicators = {}
        self._maintenance_on = False
        self._maintenance_beacon = False
        self._link_lost = False
        self._exception_text = ""
        self._exception_until = 0.0
        self._slot_errors = RateLimited(5.0)
        install_global_excepthook(self._note_exception)
        self._build()
        self.core.window_title.subscribe(self._title_changed)
        for key in MAINTENANCE_STATUS_KEYS:
            if key in self.core.values:
                self.core.values[key].subscribe(lambda _text, key=key: self._update_maintenance(key))
        if MAINTENANCE_BEACON_KEY in self.core.values:
            self.core.values[MAINTENANCE_BEACON_KEY].subscribe(lambda _text: self._update_maintenance_beacon())
        self.core.link_banner.subscribe(self._link_banner_changed, call_now=True)
        self.core.log_warning.subscribe(self._log_warning_changed, call_now=True)
        self._graph_throttle = Throttle(self.draw_graphs, GRAPH_MS, self)
        self._events_timer = self._timer(EVENT_MS, self._guarded(self.process_events, "event pump"))
        self._auto_timer = self._timer(AUTO_MS, self._guarded(self.core.auto_poll, "auto-connect"))
        self._age_timer = self._timer(AGE_MS, self._guarded(self.update_age_indicators, "age indicators"))
        self.process_events()
        if self.core._test_start_id is not None:
            QtCore.QTimer.singleShot(0, self._start_requested_test)

    # ---------------------------------------------- failure containment / link
    def _guarded(self, slot, name):
        """Timer slot wrapper: an exception is logged and reported (rate-limited),
        never raised into Qt, and the timer keeps running."""
        def run():
            try:
                return slot()
            except Exception as exc:  # noqa: BLE001
                _record_exception(f"timer slot {name}", type(exc), exc, exc.__traceback__)
                allowed, skipped = self._slot_errors.allow(name)
                if allowed:
                    self._note_exception(f"SLOT_ERROR[{name}] {type(exc).__name__}: {exc}"
                                         + (f" ({skipped} similar suppressed)" if skipped else ""))
        return run

    def _note_exception(self, summary):
        """Console + status-bar notice of an uncaught exception (any thread)."""
        try:
            self.core.console_write(summary)
        except Exception:  # noqa: BLE001
            pass
        self._exception_text = summary
        self._exception_until = time.monotonic() + 60.0

    def _link_banner_changed(self, text):
        self._link_lost = bool(text) and not text.startswith("WITNESS RESETTING")
        label = getattr(self, "link_banner_label", None)
        if label is not None:
            label.setText(text)
            label.setVisible(bool(text))
            colors = ("background:#ff8c00; color:black;" if text.startswith("WITNESS RESETTING")
                      else "background:#c00000; color:white;")
            label.setStyleSheet(colors + " font-size:16pt; font-weight:bold; padding:8px;"
                                "border:3px solid #ffdd00;")
        self._title_changed(self.core.window_title.get())
        self.linkBannerChanged.emit(text)

    def _log_warning_changed(self, text):
        label = getattr(self, "log_warning_label", None)
        if label is not None:
            label.setText(text)
            label.setVisible(bool(text))

    def _update_log_full(self):
        """LOG_SLOTS_FULL (status byte 1 bit 7): Witness is NOT logging. Banner on every tab."""
        full = any(isinstance(self.core.raw_values.get(key), int)
                   and bool(self.core.raw_values[key] & LOG_SLOTS_FULL_MASK)
                   for key in MAINTENANCE_STATUS_KEYS)
        label = getattr(self, "log_full_banner", None)
        if label is not None and label.isVisibleTo(self) != full:
            label.setVisible(full)
            if full:
                self.core.console_write("WITNESS LOG SLOTS FULL: NOT LOGGING until logs are cleared/erased")

    def _update_maintenance(self, key):
        """Show/hide the maintenance banner from the newest Witness status."""
        self._update_log_full()
        value = self.core.raw_values.get(key)
        if not isinstance(value, int):
            return
        # A flight packet means the flight image is running again.
        self._maintenance_beacon = False
        on = status_maintenance(value)
        self._set_maintenance(on)

    def _update_maintenance_beacon(self):
        """0x08 beacon: Witness is in the maintenance image (not flying, not logging)."""
        self._maintenance_beacon = True
        self._set_maintenance(True)

    def _set_maintenance(self, on):
        if on != self._maintenance_on:
            self._maintenance_on = on
            self.maintenance_banner.setVisible(on)
            self.core.maintenance_active = on
            self._title_changed(self.core.window_title.get())
            if on:
                self.core.console_write("WITNESS MAINTENANCE MODE (separate image): flight logging OFF, NOT FLIGHT READY")
            else:
                self.core.console_write("Witness left maintenance mode: back in flight mode")

    def __getattr__(self, name):
        if name in ("core", "__setstate__") or name.startswith("__"):
            raise AttributeError(name)
        try:
            core = object.__getattribute__(self, "core")
        except AttributeError:
            raise AttributeError(name) from None
        return getattr(core, name)

    def _timer(self, interval, callback):
        timer = QtCore.QTimer(self)
        timer.setInterval(interval)
        timer.timeout.connect(callback)
        timer.start()
        return timer

    # ----------------------------------------------------------------- UI
    def _build(self):
        core = self.core
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(4)

        self.port_a_combo, self.port_b_combo = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        bind_combo(core.port_a, self.port_a_combo, core.port_choices)
        bind_combo(core.port_b, self.port_b_combo, core.port_choices)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.setToolTip("Refresh serial ports (F5)")
        refresh.clicked.connect(core.refresh_ports)
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        bind_combo(core.baud, self.baud_combo, BAUD_CHOICES)
        self.connect_button = QtWidgets.QPushButton("Connect")
        bind_label(core.connect_text, self.connect_button)
        bind_enabled(core.inputs_locked, self.connect_button, invert=True)
        self.connect_button.clicked.connect(core.toggle_connection)
        self.status_label = QtWidgets.QLabel()
        bind_label(core.status_text, self.status_label)
        root.addLayout(hbox(QtWidgets.QLabel("Port A:"), self.port_a_combo, QtWidgets.QLabel("Port B:"),
                            self.port_b_combo, refresh, QtWidgets.QLabel("Baud:"), self.baud_combo,
                            self.connect_button, self.status_label, None))
        # Impossible to miss: Witness reports its maintenance (Wi-Fi) boot in
        # status byte 1 bit 5. It is not logging and must not fly like this.
        self.maintenance_banner = QtWidgets.QLabel(MAINTENANCE_BANNER_TEXT)
        self.maintenance_banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.maintenance_banner.setWordWrap(True)
        self.maintenance_banner.setStyleSheet(
            "background:#c00000; color:white; font-size:18pt; font-weight:bold; padding:10px;"
            "border:4px solid #ffdd00;")
        root.addWidget(self.maintenance_banner)
        # Hide only after parenting: reparenting a hidden widget un-hides it.
        self.maintenance_banner.setVisible(False)
        # LINK LOST (no data > 3 s / reconnecting), LOG FULL (not logging), log-file trouble.
        self.link_banner_label = QtWidgets.QLabel()
        self.link_banner_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.link_banner_label.setWordWrap(True)
        root.addWidget(self.link_banner_label)
        self.link_banner_label.setVisible(False)
        self.log_full_banner = QtWidgets.QLabel(
            "WITNESS LOG SLOTS FULL - NOT LOGGING. Clear or erase the logs (Clear Logs / Erase Logs).")
        self.log_full_banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.log_full_banner.setWordWrap(True)
        self.log_full_banner.setStyleSheet(
            "background:#c00000; color:white; font-size:16pt; font-weight:bold; padding:8px;"
            "border:3px solid #ffdd00;")
        root.addWidget(self.log_full_banner)
        self.log_full_banner.setVisible(False)
        self.log_warning_label = QtWidgets.QLabel()
        self.log_warning_label.setWordWrap(True)
        self.log_warning_label.setStyleSheet("background:#fff2a8; color:#600000; font-weight:bold; padding:4px;")
        root.addWidget(self.log_warning_label)
        self.log_warning_label.setVisible(False)
        self.link_label = wrap_label(muted=True)
        bind_label(core.link_summary, self.link_label)
        root.addWidget(self.link_label)
        self.exception_label = wrap_label()
        self.exception_label.setStyleSheet("background:#ffd0d0; color:#600000; padding:2px;")
        root.addWidget(self.exception_label)
        self.exception_label.setVisible(False)

        self.tcp_entries = []
        tcp_widgets = []
        for index, (host_cell, port_cell) in enumerate(zip(core.tcp_hosts, core.tcp_ports), start=1):
            host, port = QtWidgets.QLineEdit(), QtWidgets.QLineEdit()
            host.setPlaceholderText("host")
            port.setPlaceholderText("port")
            port.setMaximumWidth(80)
            bind_line_edit(host_cell, host)
            bind_line_edit(port_cell, port)
            self.tcp_entries.append((host, port))
            tcp_widgets += [QtWidgets.QLabel(f"TCP {index} host:"), host, QtWidgets.QLabel("port:"), port]
        root.addLayout(hbox(*tcp_widgets))

        self.auto_button = QtWidgets.QCheckBox("Auto-connect USB")
        bind_check(core.auto_connect, self.auto_button, on_user=core._auto_changed)
        root.addLayout(hbox(self.auto_button, wrap_label(
            "Known Witness devices only; select generic adapters manually once.", muted=True)))

        self.test_file_entry = QtWidgets.QLineEdit()
        self.test_file_entry.setPlaceholderText("blank = built-in simulated flight")
        bind_line_edit(core.test_file, self.test_file_entry)
        self.test_browse_button = QtWidgets.QPushButton("Browse JSON")
        self.test_browse_button.clicked.connect(self.browse_test_source)
        self.test_speed_entry = QtWidgets.QLineEdit()
        self.test_speed_entry.setMaximumWidth(60)
        bind_line_edit(core.test_speed, self.test_speed_entry)
        self.test_loop_button = QtWidgets.QCheckBox("Loop")
        bind_check(core.test_loop, self.test_loop_button)
        self.test_button = QtWidgets.QPushButton("Start test")
        bind_label(core.test_button_text, self.test_button)
        self.test_button.clicked.connect(core.toggle_test)
        for widget in (self.test_file_entry, self.test_browse_button, self.test_speed_entry,
                       self.test_loop_button):
            bind_enabled(core.inputs_locked, widget, invert=True)
        test_row = hbox(QtWidgets.QLabel("Test source:"), self.test_file_entry, self.test_browse_button,
                        QtWidgets.QLabel("Speed:"), self.test_speed_entry, self.test_loop_button,
                        self.test_button)
        test_row.setStretch(1, 1)
        root.addLayout(test_row)
        self.test_status_label = wrap_label()
        bind_label(core.test_status, self.test_status_label)
        core.test_warning.subscribe(
            lambda warn: set_background(self.test_status_label, WARN_BG if warn else None), call_now=True)
        root.addWidget(self.test_status_label)
        self.feed_label = wrap_label()
        bind_label(core.feed_status, self.feed_label)
        self.nmea_label = wrap_label()
        bind_label(core.nmea_status, self.nmea_label)
        root.addWidget(self.feed_label)
        # Remote feed: a second, independent server on this PC's Tailscale address.
        self.remote_check = QtWidgets.QCheckBox("Share over Tailscale")
        self.remote_check.setToolTip(
            "Publish the same receive-only telemetry feed on this PC's Tailscale address "
            "(port 8767) so Horizon on another tailnet machine can watch. Independent of the "
            "local 127.0.0.1 feed: a slow remote viewer is dropped, never the local one.")
        bind_check(core.remote_feed_enabled, self.remote_check,
                   on_user=lambda: core.set_remote_feed(core.remote_feed_enabled.get()))
        self.remote_label = wrap_label()
        bind_label(core.remote_feed_status, self.remote_label)
        remote_row = hbox(self.remote_check, self.remote_label)
        remote_row.setStretch(1, 1)
        root.addLayout(remote_row)
        root.addWidget(self.nmea_label)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        self.tabs = QtWidgets.QTabWidget()
        self.telemetry_grid = RetilingGrid()
        for packet, rows in core.section_keys.items():
            self.telemetry_grid.add_tile(self._section(packet, rows))
        self.telemetry_scroll = scroll_area(self.telemetry_grid)
        self.tabs.addTab(self.telemetry_scroll, "Telemetry")
        self.tabs.addTab(self._graph_page(), "Graphs")
        self.tabs.currentChanged.connect(lambda _i: self._graph_throttle.request())
        splitter.addWidget(self.tabs)
        splitter.addWidget(self._console_box())
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

        for sequence, slot in (("F5", core.refresh_ports),
                               ("Ctrl+1", lambda: self.tabs.setCurrentIndex(0)),
                               ("Ctrl+2", lambda: self.tabs.setCurrentIndex(1))):
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(sequence), self)
            shortcut.setContext(QtCore.Qt.ShortcutContext.WidgetWithChildrenShortcut)
            shortcut.activated.connect(slot)

    def _section(self, packet, rows):
        box = QtWidgets.QGroupBox(packet.capitalize())
        layout = QtWidgets.QGridLayout(box)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(2)
        layout.addWidget(QtWidgets.QLabel("Data age:"), 0, 0, 1, 2)
        age = QtWidgets.QLabel("--")
        age.setFrameShape(QtWidgets.QFrame.Shape.Panel)
        age.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        age.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        age.setMinimumWidth(70)
        layout.addWidget(age, 0, 2, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.age_labels[f"packet.{packet}"] = age
        row = 1
        for key, field, label_text in rows:
            check = QtWidgets.QCheckBox()
            check.setToolTip("Plot this field on the Graphs tab")
            bind_check(self.core.check_vars[key], check)
            self.core.check_vars[key].subscribe(lambda _v: self._selection_changed())
            label = QtWidgets.QLabel(label_text + ":")
            label.setWordWrap(True)
            label.setMinimumWidth(130)
            value = QtWidgets.QLabel()
            value.setWordWrap(True)
            value.setMinimumWidth(130)
            value.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            bind_label(self.core.values[key], value)
            layout.addWidget(check, row, 0)
            layout.addWidget(label, row, 1)
            layout.addWidget(value, row, 2)
            row += 1
            if is_status_field(field):
                indicator = StatusIndicator(camera=(packet == "camera" and field == "status"))
                layout.addWidget(indicator, row, 0, 1, 3)
                self.status_indicators[key] = indicator
                self.core.values[key].subscribe(
                    lambda _text, key=key: self.status_indicators[key].show_value(self.core.raw_values.get(key)))
                row += 1
        layout.setColumnStretch(2, 1)
        return box

    def _graph_page(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        buttons = []
        for text, slot in (("Clear graph history", self.clear_history), ("Save setup", self.save_graph_setup),
                           ("Load setup", self.load_graph_setup)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            buttons.append(button)
        layout.addLayout(hbox(*buttons, wrap_label(
            "Select fields using the checkboxes on the Telemetry tab.", muted=True)))
        self.graph_container = QtWidgets.QWidget()
        self.graph_layout = QtWidgets.QVBoxLayout(self.graph_container)
        self.graph_layout.setContentsMargins(0, 0, 0, 0)
        self.no_fields_label = QtWidgets.QLabel("No fields selected")
        self.no_fields_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.graph_layout.addWidget(self.no_fields_label)
        self.graph_scroll = scroll_area(self.graph_container)
        layout.addWidget(self.graph_scroll, 1)
        self.graph_page = page
        return page

    def _console_box(self):
        core = self.core
        box = QtWidgets.QGroupBox("Console")
        layout = QtWidgets.QVBoxLayout(box)
        self.console = LogConsole(max_lines=core.max_console_lines)
        self.console.setMinimumHeight(60)
        layout.addWidget(self.console, 1)
        self.command_port_combo = QtWidgets.QComboBox()
        self.command_port_combo.setMinimumWidth(110)
        bind_combo(core.command_target, self.command_port_combo, core.command_targets)
        self.command_entry = QtWidgets.QLineEdit()
        self.command_entry.setPlaceholderText("hex bytes, e.g. 05 00 00 00 00 00 00 00 31 0a")
        bind_line_edit(core.command_text, self.command_entry)
        self.command_entry.returnPressed.connect(self.send_hex_command)
        self.command_entry.installEventFilter(self)
        self.send_command_button = QtWidgets.QPushButton("Send")
        self.send_command_button.clicked.connect(self.send_hex_command)
        layout.addLayout(hbox(QtWidgets.QLabel("Send hex to:"), self.command_port_combo,
                              QtWidgets.QLabel("Command:"), self.command_entry, self.send_command_button))
        grid_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(3)
        columns = 4
        self.quick_command_buttons = []
        self.reset_witness_button = None
        for index, (label, opcode, confirm) in enumerate(QUICK_COMMANDS):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _c=False, n=label, v=opcode, c=confirm:
                                   core.send_quick_command(n, v, c))
            grid.addWidget(button, index // columns, index % columns)
            self.quick_command_buttons.append(button)
            if opcode == RESET_TO_PAD_OPCODE:
                self.reset_witness_button = button
                button.setToolTip("Witness reached Landed on the pad without a real flight? This "
                                  "restarts it into Pad Idle (fresh filter, new log session). "
                                  "Enabled only while Witness reports Landed; asks first.")
        offset = len(QUICK_COMMANDS)
        offset += (-offset) % columns
        for index, (label, argument) in enumerate(IMU_CALIBRATION_COMMANDS, start=offset):
            button = QtWidgets.QPushButton(label + " (USB)")
            button.clicked.connect(lambda _c=False, n=label, a=argument: core.send_imu_calibration(n, a))
            grid.addWidget(button, index // columns, index % columns)
            self.quick_command_buttons.append(button)
        index = offset + len(IMU_CALIBRATION_COMMANDS)
        eph = QtWidgets.QPushButton("Upload GPS ephemeris…")
        eph.setToolTip("RINEX broadcast ephemeris -> Witness -> PX1120S (works over USB and radio)")
        eph.clicked.connect(lambda _c=False: self.upload_gps_ephemeris())
        grid.addWidget(eph, index // columns, index % columns)
        self.quick_command_buttons.append(eph)
        index += 1
        heading = QtWidgets.QPushButton("Set pad heading…")
        heading.setToolTip("On the rail: which body axis points to which azimuth (Witness has no "
                           "magnetometer). Pad Idle only; works over USB and radio.")
        heading.clicked.connect(lambda _c=False: self.set_pad_heading())
        grid.addWidget(heading, index // columns, index % columns)
        self.quick_command_buttons.append(heading)
        index += 1
        power = QtWidgets.QPushButton("Radio power…")
        power.setToolTip("Set the Witness or ground-station (Aperture) radio drive, -17..+3 dBm "
                         "(below -9 uses the SX1262 low-power PA). Both boot at -17 dBm for the "
                         "bench; not saved.")
        power.clicked.connect(lambda _c=False: self.set_radio_power())
        grid.addWidget(power, index // columns, index % columns)
        self.quick_command_buttons.append(power)
        index += 1
        ping = QtWidgets.QPushButton("Ping Witness (Landed)")
        ping.setToolTip("Ask a landed Witness for its position now (0x09 landed position packet, "
                        "rate limited to one per 5 s). Radio only; Witness must be in the Landed state.")
        ping.clicked.connect(lambda _c=False: self.ping_witness())
        grid.addWidget(ping, index // columns, index % columns)
        self.quick_command_buttons.append(ping)
        index += 1
        for widget in (self.command_entry, self.send_command_button, *self.quick_command_buttons):
            if widget is self.reset_witness_button:
                bind_enabled(core.reset_witness_enabled, widget)
            else:
                bind_enabled(core.commands_enabled, widget)
        layout.addLayout(hbox(QtWidgets.QLabel("Commands:"), grid_widget))
        layout.addLayout(self._bit_row())
        return box

    def _bit_row(self):
        """Compact built-in self test (BIT) status with its two buttons. Run BIT (0x5442) is enabled
        only in Pad Idle and asks first; BIT detail (0x5452) only asks Witness to re-send.
        Horizon never runs BIT by itself."""
        core = self.core
        self.bit_label = QtWidgets.QLabel()
        self.bit_label.setWordWrap(True)
        self.bit_label.setMargin(2)
        bind_label(core.bit_text, self.bit_label)
        _owned(self.bit_label, core.bit_tooltip.subscribe(self.bit_label.setToolTip, call_now=True))
        _owned(self.bit_label, core.bit_kind.subscribe(self._show_bit_kind, call_now=True))
        self.run_bit_button = QtWidgets.QPushButton("Run BIT")
        self.run_bit_button.setToolTip("Run Witness's built-in self test (0x5442). Pad Idle only; asks first. "
                                       "Never run automatically.")
        self.run_bit_button.clicked.connect(lambda _c=False: core.send_bit_run())
        self.bit_detail_button = QtWidgets.QPushButton("BIT detail")
        self.bit_detail_button.setToolTip("Ask Witness to re-send the BIT detail pages (0x5452). "
                                          "Does not run a test.")
        self.bit_detail_button.clicked.connect(lambda _c=False: core.request_bit_report())
        bind_enabled(core.bit_run_enabled, self.run_bit_button)
        bind_enabled(core.bit_detail_enabled, self.bit_detail_button)
        return hbox(QtWidgets.QLabel("BIT:"), self.bit_label, self.run_bit_button, self.bit_detail_button,
                    stretch_last=False)

    def _show_bit_kind(self, kind):
        background, foreground = STRIP_COLORS.get(kind, STRIP_COLORS["muted"])
        set_background(self.bit_label, background, foreground)

    def eventFilter(self, obj, event):  # noqa: N802 - Qt API
        if obj is self.command_entry and event.type() == QtCore.QEvent.Type.KeyPress:
            if event.key() == QtCore.Qt.Key.Key_Up:
                self.recall_command(-1)
                return True
            if event.key() == QtCore.Qt.Key.Key_Down:
                self.recall_command(1)
                return True
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------- actions
    def _alert(self, title, text):
        self.core.last_alert = (title, text)
        show_error(self, title, text)

    def _title_changed(self, text):
        if self._maintenance_on:
            text = MAINTENANCE_TITLE_PREFIX + text
        if self._link_lost:
            text = "LINK LOST | " + text
        self.titleChanged.emit(text)
        if self.on_title is not None:
            self.on_title(text)

    def recall_command(self, direction):
        text = self.core.recall_command(direction)
        self.command_entry.setCursorPosition(len(self.command_entry.text()))
        return text

    def send_hex_command(self):
        return self.core.send_hex_command()

    def set_pad_heading(self, axis=None, azimuth_deg=None):
        """Ask which body axis points where on the rail and send the pad
        heading command (comms.h IRIS_COMMAND_PAD_HEADING)."""
        if axis is None:
            choices = list(PAD_HEADING_AXES) + ["Clear (GPS alignment only)"]
            axis, ok = QtWidgets.QInputDialog.getItem(
                self, "Pad heading", "Body axis across the rocket that you can see:", choices, 0, False)
            if not ok:
                return False
            if axis.startswith("Clear"):
                axis = None
        if axis is not None and azimuth_deg is None:
            azimuth_deg, ok = QtWidgets.QInputDialog.getDouble(
                self, "Pad heading", f"Azimuth that body {axis} points to (deg from true north):",
                0.0, 0.0, 359.99, 1)
            if not ok:
                return False
        frame = build_pad_heading_packet(axis, azimuth_deg or 0.0)
        label = "pad heading cleared" if axis is None else f"pad heading: body {axis} at {azimuth_deg:.1f} deg"
        return self.core.transmit_command(self.core.command_target.get(), frame, label)

    def set_radio_power(self, target=None, drive_dbm=None):
        """Ask which radio and what drive level, then send IRIS_COMMAND_RADIO_POWER.
        The console shows ACK/NAK; the ack's detail is the level now in use."""
        from witness_protocol import (RADIO_FRONT_END_GAIN_DB, RADIO_POWER_MAX_DBM,
                                      RADIO_POWER_MIN_DBM, build_radio_power_packet)
        labels = {"Witness (rocket)": "witness", "Ground station (Aperture)": "ground"}
        if target is None:
            choice, ok = QtWidgets.QInputDialog.getItem(
                self, "Radio power", "Radio:", list(labels), 0, False)
            if not ok:
                return False
            target = labels[choice]
        if drive_dbm is None:
            drive_dbm, ok = QtWidgets.QInputDialog.getInt(
                self, "Radio power",
                f"SX1262 drive, dBm ({RADIO_POWER_MIN_DBM}..{RADIO_POWER_MAX_DBM}; output is about "
                f"+{RADIO_FRONT_END_GAIN_DB} dB more; below -9 the low-power PA is used).\n"
                "Bench: keep -17 unless the boards have their "
                "own supply; the transmit current can brown them out on USB power.",
                RADIO_POWER_MIN_DBM, RADIO_POWER_MIN_DBM, RADIO_POWER_MAX_DBM, 1)
            if not ok:
                return False
        frame = build_radio_power_packet(target, drive_dbm)
        label = (f"{'Witness' if target == 'witness' else 'ground'} radio drive {drive_dbm:+d} dBm "
                 f"(~{drive_dbm + RADIO_FRONT_END_GAIN_DB:+d} dBm out)")
        return self.core.transmit_command(self.core.command_target.get(), frame, label)

    def ping_witness(self):
        """Send IRIS_COMMAND_PING; a landed Witness answers with its position packet."""
        from witness_protocol import build_ping_packet
        return self.core.transmit_command(self.core.command_target.get(), build_ping_packet(),
                                          "ping (landed position request)")

    def upload_gps_ephemeris(self, path=None):
        """Pick a RINEX nav file and upload the ephemerides valid now to
        Witness's GPS (SkyTraq 0x41 via the chunked 0x05 command frames)."""
        import gps_assist
        if path is None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "RINEX navigation file (broadcast ephemeris)", "",
                "RINEX nav (*.??n *.??N *.rnx *.nav);;All files (*.*)")
            if not path:
                return False
        try:
            nav = gps_assist.parse_rinex_nav(path)
            blob, prns = gps_assist.ephemeris_upload(nav)
        except (OSError, ValueError) as exc:
            self._alert("GPS ephemeris", f"Cannot use {path}: {exc}")
            return False
        return self.core.upload_gps_assist(blob, f"GPS ephemeris upload ({len(prns)} SVs)")

    def browse_test_source(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Timed test packets", "", "Test source JSON (*.json)")
        if filename:
            self.core.test_file.set(filename)

    def _start_requested_test(self):
        if not self._shut_down and self.core._test_start_id is not None:
            self.core._start_requested_test()

    def clear_history(self):
        self.core.clear_history()
        self._graph_throttle.flush()

    def save_graph_setup(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save graph setup", "horizon_graph_setup.json", "JSON files (*.json);;All files (*)")
        if not filename:
            return
        try:
            self.core.save_graph_setup(filename)
        except (OSError, TypeError) as exc:
            show_error(self, "Save graph setup", f"Could not save setup:\n{exc}")

    def load_graph_setup(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load graph setup", "", "JSON files (*.json);;All files (*)")
        if not filename:
            return
        try:
            self.core.load_graph_setup(filename)
        except (OSError, ValueError) as exc:
            show_error(self, "Load graph setup", f"Could not load setup:\n{exc}")
        self._graph_throttle.request()

    # -------------------------------------------------------------- pump
    def process_events(self):
        """Drain the worker queue (one batch), show console lines, schedule graphs."""
        handled = self.core.process_events()
        self.console.append_lines(self.core.take_console())
        if self.core.graphs_dirty:
            self._graph_throttle.request()
        return handled

    def update_age_indicators(self):
        self.core.update_feed_status()
        self.core.update_link_health()
        show = bool(self._exception_text) and time.monotonic() < self._exception_until
        if self.exception_label.isVisibleTo(self) != show:
            self.exception_label.setVisible(show)
        if show:
            self.exception_label.setText(self._exception_text)
        for key, age in self.core.data_ages().items():
            label = self.age_labels.get(key)
            if label is None:
                continue
            label.setText("--" if age is None else f"{age:.1f}s")
            set_background(label, age_color(age) or "#f0f0f0")

    def _selection_changed(self):
        self._graph_layout_dirty = True
        self._graph_throttle.request()

    def schedule_graph_draw(self):
        self._graph_throttle.request()

    def draw_graphs(self, force=False):
        """Redraw selected fields (throttled; skipped while the Graphs tab is hidden)."""
        if self._shut_down:
            return
        if not force and self.tabs.currentWidget() is not self.graph_page:
            self.core.graphs_dirty = True
            return
        self.graph_draws += 1
        self.core.graphs_dirty = False
        selected = self.core.selected_fields()
        if self._graph_layout_dirty:
            self._graph_layout_dirty = False
            for key in list(self._plots):
                if key not in selected:
                    plot = self._plots.pop(key)
                    self.graph_layout.removeWidget(plot.widget)
                    plot.widget.deleteLater()
            for index, key in enumerate(selected):
                plot = self._plots.get(key)
                if plot is None:
                    plot = self._plots[key] = GraphPlot(key)
                self.graph_layout.insertWidget(index, plot.widget)
            self.no_fields_label.setVisible(not selected)
        for key in selected:
            self._plots[key].update(self.core.history.get(key))

    # ------------------------------------------------------------ shutdown
    def shutdown(self):
        """Stop timers, inputs, the test source and the feed (idempotent)."""
        if self._shut_down:
            return
        self._shut_down = True
        for timer in (self._events_timer, self._auto_timer, self._age_timer):
            timer.stop()
        self._graph_throttle.stop()
        self.core.close()

    def close(self):  # noqa: D401 - Tk-era API: stop everything, keep host window
        self.shutdown()
        return super().close()


MOVED_NOTE = ("The Witness monitor is now the \"Witness monitor\" tab of witnessOS: "
              "python Software/witnessOS/witnessos.py --tab monitor")


def main(argv=None) -> int:
    print(MOVED_NOTE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
