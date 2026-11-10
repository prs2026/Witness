"""witnessOS "HITL bench" tab: PyQt6 widgets with pyqtgraph plots.

    from wos_hitl_tab import HitlWidget

The bench logic lives in Software/HITL (hitl_session.py, sim_model.py,
trajectory_csv.py, ...); HITL/hitl_gui.py holds the Qt-free helpers
(settings, vibration choice, prediction) and the headless runner
(``witnessos.py --hitl-headless``). This module is only the GUI.

Threading: the HitlSession runs its own reader/sender threads and reports
through ``session.events`` (a queue.Queue) and RunState lists. A 100 ms
QTimer on the GUI thread drains the events and reads the run state; plots
redraw through a qt_common.Throttle. CSV parsing, prediction, 960 Hz IMU
synthesis and connecting run through qt_common.run_in_thread and report
back on the GUI thread. No widget is touched from another thread.
"""

from __future__ import annotations

import dataclasses
import os
import queue
import sys
import threading

import wos_paths  # noqa: F401  (HITL and Horizon on sys.path)

import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: E402

import hitl_gui as hg  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402
import trajectory_csv as tc  # noqa: E402
from qt_common import (add_curve, ensure_app, make_plot,  # noqa: E402
                       process_events_until, run_in_thread, Throttle)

STATUS_LAMPS = (
    ("HITL active", lambda s: s["hitl_active"], "good"),
    ("GPS locked", lambda s: s["gps_locked"], "good"),
    ("Low-g", lambda s: s["low_g_ready"], "good"),
    ("High-g", lambda s: s["high_g_ready"], "good"),
    ("Gyro", lambda s: s["gyro_ready"], "good"),
    ("IMU temp", lambda s: s["imu_temp_ready"], "good"),
    ("Baro", lambda s: s["baro_ready"], "good"),
    ("Heartbeat", lambda s: s["heartbeat"], "good"),
    ("NAND fail", lambda s: s["nand_init_failed"], "bad"),
    ("Flash log fail", lambda s: s["flash_log_failed"], "bad"),
)
TABLE_COLUMNS = ("State", "Expected", "Witness", "Delta", "Result")
LOG_MAX_LINES = 3000
POLL_MS = 100
PLOT_MS = 150


class ParamField:
    """One parameter editor with a uniform get()/set() (str/bool values)."""

    def __init__(self, widget: QtWidgets.QWidget):
        self.widget = widget

    def get(self):
        w = self.widget
        if isinstance(w, QtWidgets.QCheckBox):
            return w.isChecked()
        if isinstance(w, QtWidgets.QComboBox):
            return w.currentText()
        return w.text()

    def set(self, value) -> None:
        w = self.widget
        if isinstance(w, QtWidgets.QCheckBox):
            w.setChecked(bool(value))
        elif isinstance(w, QtWidgets.QComboBox):
            text = str(value)
            index = w.findText(text)
            if index < 0:          # e.g. a saved profile folder that is gone
                w.addItem(text)
                index = w.count() - 1
            w.setCurrentIndex(index)
        else:
            w.setText(str(value))


class _Series:
    """numpy copy of a growing RunState list of (t, value) pairs.

    Only the new tail is converted on each update, so a 436 s run never
    re-copies tens of thousands of points per redraw.
    """

    def __init__(self):
        self.reset(None)

    def reset(self, source):
        self.source = source
        self.n = 0
        self.buf = np.empty((0, 2))

    def update(self, source: list) -> bool:
        if source is not self.source:
            self.reset(source)
        n = len(source)
        if n <= self.n:
            return False
        new = np.asarray(source[self.n:n], dtype=float).reshape(-1, 2)
        if n > len(self.buf):
            grown = np.empty((max(n, 2 * len(self.buf), 256), 2))
            grown[:self.n] = self.buf[:self.n]
            self.buf = grown
        self.buf[self.n:n] = new
        self.n = n
        return True

    @property
    def x(self):
        return self.buf[:self.n, 0]

    @property
    def y(self):
        return self.buf[:self.n, 1]


class HitlWidget(QtWidgets.QWidget):
    """The HITL bench: connection, source/parameters, live state and plots.

    The host (witnessOS) calls shutdown() before the widget goes away
    (stops any run, sends HITL EXIT).
    """

    # While a worker runs CPU-bound Python (IMU synthesis), the GUI thread
    # waits up to sys.getswitchinterval() (5 ms) for the GIL after every
    # release; a short interval while busy keeps the UI smooth.
    BUSY_SWITCH_INTERVAL_S = 0.0005

    def __init__(self, settings: dict | None = None,
                 settings_path: str = hg.SETTINGS_PATH,
                 runs_dir: str | None = hs.DEFAULT_RUNS_DIR,
                 parent: QtWidgets.QWidget | None = None):
        ensure_app()
        super().__init__(parent)
        self.settings_path = settings_path
        self.settings = settings if settings is not None else hg.load_settings(settings_path)
        for key, value in hg.default_settings().items():
            self.settings.setdefault(key, value)
        self.session = hs.HitlSession(runs_dir=runs_dir)
        self.session.fc_log_config = hg.fc_log_config(self.settings)
        self._closing = False
        self._closing_evt = threading.Event()
        self._busy: list[str] = []
        self._jobs: dict = {}
        self._job_id = 0
        self._preparing = False
        self._connecting = False
        self._predict_token = 0
        self._pending_log: list[str] = []
        self._table_rows = None
        self._lamp_colors: list = [None] * len(STATUS_LAMPS)
        self._switch_interval = None
        self._plot_sig = None
        self._plot_run = None
        self.csv_path = None
        self.param_cls = None
        self.param_vars: dict[str, ParamField] = {}
        self.param_types: dict[str, str] = {}
        self.vibration_picker = None

        self._build_ui()
        self.load_preset()
        self._refresh_buttons()
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self.poll)
        self._poll_timer.start()
        self._plot_throttle = Throttle(self._draw_plot, PLOT_MS, self)
        self._plot_throttle.request()

    # ------------------------------------------------------------ layout
    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        # Connection bar (two rows so the window can be narrow).
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Host"))
        self.host_edit = QtWidgets.QLineEdit(str(self.settings["host"]))
        self.host_edit.setMaximumWidth(160)
        bar.addWidget(self.host_edit)
        bar.addWidget(QtWidgets.QLabel("Port"))
        self.port_edit = QtWidgets.QLineEdit(str(self.settings["port"]))
        self.port_edit.setMaximumWidth(60)
        bar.addWidget(self.port_edit)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        bar.addWidget(self.connect_btn)
        self.conn_label = QtWidgets.QLabel("Disconnected")
        self.conn_label.setMinimumWidth(90)
        bar.addWidget(self.conn_label)
        self.bytes_label = QtWidgets.QLabel("rx 0 / tx 0 B")
        bar.addWidget(self.bytes_label)
        bar.addStretch(1)
        outer.addLayout(bar)

        bar2 = QtWidgets.QHBoxLayout()
        self.btn_enter = QtWidgets.QPushButton("Enter HITL")
        self.btn_enter.clicked.connect(self.enter_hitl)
        self.btn_start = QtWidgets.QPushButton("Start sim")
        self.btn_start.clicked.connect(self.start_sim)
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_stop.clicked.connect(self.stop_sim)
        self.btn_exit = QtWidgets.QPushButton("Exit HITL")
        self.btn_exit.setToolTip("Stops (aborts) a running sim, or sends HITL EXIT")
        self.btn_exit.clicked.connect(self.exit_hitl)
        for b in (self.btn_enter, self.btn_start, self.btn_stop, self.btn_exit):
            bar2.addWidget(b)
        bar2.addSpacing(8)
        bar2.addWidget(QtWidgets.QLabel("Tolerance s"))
        self.tol_edit = QtWidgets.QLineEdit(str(self.settings["tolerance_s"]))
        self.tol_edit.setMaximumWidth(50)
        bar2.addWidget(self.tol_edit)
        # Busy indicator for worker-thread jobs (CSV parsing, prediction,
        # 960 Hz IMU synthesis, connecting): the window never freezes.
        self.busy_bar = QtWidgets.QProgressBar()
        self.busy_bar.setRange(0, 0)
        self.busy_bar.setFixedWidth(90)
        self.busy_bar.setMaximumHeight(14)
        self.busy_bar.setTextVisible(False)
        self.busy_label = QtWidgets.QLabel("")
        self.busy_label.setStyleSheet("color: #555;")
        bar2.addSpacing(10)
        bar2.addWidget(self.busy_bar)
        bar2.addWidget(self.busy_label)
        self.busy_bar.hide()
        self.busy_label.hide()
        bar2.addStretch(1)
        outer.addLayout(bar2)

        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        self.banner.setStyleSheet("background: #b00020; color: white; "
                                  "font-weight: bold; font-size: 11pt; padding: 3px;")
        self.banner.hide()
        outer.addWidget(self.banner)

        self.main_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.main_split.setChildrenCollapsible(False)
        outer.addWidget(self.main_split, 1)
        self.main_split.addWidget(self._build_left())
        self.main_split.addWidget(self._build_right())
        self.main_split.setStretchFactor(0, 0)
        self.main_split.setStretchFactor(1, 1)
        self.main_split.setSizes([300, 900])

    def _build_left(self) -> QtWidgets.QWidget:
        left = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(left)
        lay.setContentsMargins(0, 0, 4, 0)
        lay.addWidget(QtWidgets.QLabel("Preset"))
        self.preset_combo = QtWidgets.QComboBox()
        self.preset_combo.addItems(list(sm.PRESETS))
        self.preset_combo.setSizeAdjustPolicy(
            QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.preset_combo.setMinimumContentsLength(14)
        preset = self.settings.get("preset")
        self.preset_combo.setCurrentText(preset if preset in sm.PRESETS else "Nominal 10 g")
        self.preset_combo.activated.connect(lambda _i: self.load_preset())
        lay.addWidget(self.preset_combo)

        csv_row = QtWidgets.QHBoxLayout()
        self.load_csv_btn = QtWidgets.QPushButton("Load CSV…")
        self.load_csv_btn.clicked.connect(lambda: self.load_csv())
        csv_row.addWidget(self.load_csv_btn)
        self.csv_label = QtWidgets.QLabel("(preset source)")
        self.csv_label.setMinimumWidth(80)
        csv_row.addWidget(self.csv_label, 1)
        lay.addLayout(csv_row)

        att_row = QtWidgets.QHBoxLayout()
        att_row.addWidget(QtWidgets.QLabel("Final tilt max deg"))
        self.att_tilt_edit = QtWidgets.QLineEdit(
            str(self.settings.get("att_final_tilt_deg", 5.0)))
        self.att_tilt_edit.setMaximumWidth(60)
        att_row.addWidget(self.att_tilt_edit)
        att_row.addStretch(1)
        lay.addLayout(att_row)

        box = QtWidgets.QGroupBox("Parameters")
        box_lay = QtWidgets.QVBoxLayout(box)
        box_lay.setContentsMargins(2, 2, 2, 2)
        self.param_scroll = QtWidgets.QScrollArea()
        self.param_scroll.setWidgetResizable(True)
        self.param_scroll.setMinimumSize(250, 120)
        box_lay.addWidget(self.param_scroll)
        lay.addWidget(box, 1)
        self._build_param_grid(sm.SimParams)

        lay.addWidget(self._build_fc_box())

        # Which vibration profile the next run would use / the last run used.
        self.vibration_label = QtWidgets.QLabel("")
        self.vibration_label.setWordWrap(True)
        self.vibration_label.setStyleSheet("color: #1f5fbf;")
        lay.addWidget(self.vibration_label)
        self.predict_btn = QtWidgets.QPushButton("Predict transitions")
        self.predict_btn.clicked.connect(self.predict)
        lay.addWidget(self.predict_btn)
        # The prediction (with the CSV report) can be long: it scrolls in a
        # bounded box so the parameter list keeps the space.
        self.predict_label = QtWidgets.QLabel("")
        self.predict_label.setWordWrap(True)
        self.predict_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop
                                        | QtCore.Qt.AlignmentFlag.AlignLeft)
        self.predict_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        predict_scroll = QtWidgets.QScrollArea()
        predict_scroll.setWidgetResizable(True)
        predict_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        predict_scroll.setWidget(self.predict_label)
        predict_scroll.setMinimumHeight(60)
        predict_scroll.setMaximumHeight(170)
        lay.addWidget(predict_scroll)
        return left

    def _build_fc_box(self) -> QtWidgets.QGroupBox:
        """FC flash log fetch through the bench Pi after each run."""
        box = QtWidgets.QGroupBox("Fetch FC logs after run")
        box.setToolTip("Download the FC flash logs through the bench Pi after each run")
        box.setCheckable(True)
        box.setChecked(bool(self.settings.get("fetch_fc_logs", True)))
        form = QtWidgets.QFormLayout(box)
        form.setContentsMargins(4, 2, 4, 2)
        self.pi_host_edit = QtWidgets.QLineEdit(str(self.settings.get("pi_host") or ""))
        self.pi_host_edit.setPlaceholderText("Witness host / WITNESS_PI_HOST")
        self.pi_user_edit = QtWidgets.QLineEdit(str(self.settings.get("pi_user") or ""))
        self.pi_user_edit.setPlaceholderText("WITNESS_PI_USER")
        self.pi_key_edit = QtWidgets.QLineEdit(str(self.settings.get("pi_key") or ""))
        self.pi_key_edit.setPlaceholderText("SSH key file / WITNESS_PI_KEY")
        self.restart_hub_check = QtWidgets.QCheckBox("Then power-cycle USB hub 2")
        self.restart_hub_check.setChecked(bool(self.settings.get("fetch_restart_hub", True)))
        form.addRow("Pi host", self.pi_host_edit)
        form.addRow("Pi user", self.pi_user_edit)
        form.addRow("SSH key", self.pi_key_edit)
        form.addRow(self.restart_hub_check)
        for edit in (self.pi_host_edit, self.pi_user_edit, self.pi_key_edit):
            edit.editingFinished.connect(self._fc_changed)
        box.toggled.connect(lambda _on: self._fc_changed())
        self.restart_hub_check.toggled.connect(lambda _on: self._fc_changed())
        self.fc_box = box
        return box

    def _build_right(self) -> QtWidgets.QWidget:
        right = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(right)
        lay.setContentsMargins(4, 0, 0, 0)
        st = QtWidgets.QHBoxLayout()
        self.state_label = QtWidgets.QLabel("State: ?")
        font = self.state_label.font()
        font.setPointSize(16)
        font.setBold(True)
        self.state_label.setFont(font)
        self.state_label.setMinimumWidth(170)
        st.addWidget(self.state_label, 0, QtCore.Qt.AlignmentFlag.AlignTop)
        lamps = QtWidgets.QGridLayout()
        lamps.setHorizontalSpacing(4)
        self.lamps: list[QtWidgets.QLabel] = []
        for i, (name, _fn, _kind) in enumerate(STATUS_LAMPS):
            lamp = QtWidgets.QLabel()
            lamp.setFixedSize(12, 12)
            row, col = i // 5, (i % 5) * 2
            lamps.addWidget(lamp, row, col)
            lamps.addWidget(QtWidgets.QLabel(name), row, col + 1)
            self.lamps.append(lamp)
        self._paint_lamps(["#888"] * len(STATUS_LAMPS))
        st.addLayout(lamps)
        st.addStretch(1)
        lay.addLayout(st)

        self.info_label = QtWidgets.QLabel("")
        self.info_label.setWordWrap(True)
        self.info_label.setFont(QtGui.QFontDatabase.systemFont(
            QtGui.QFontDatabase.SystemFont.FixedFont))
        self.info_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self.info_label)

        # Plots and table/log share a draggable vertical split.
        vsplit = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        vsplit.setChildrenCollapsible(False)
        lay.addWidget(vsplit, 1)
        vsplit.addWidget(self._build_plots())
        lower = QtWidgets.QWidget()
        low = QtWidgets.QHBoxLayout(lower)
        low.setContentsMargins(0, 0, 0, 0)
        self.table = QtWidgets.QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(TABLE_COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.table.setMinimumWidth(300)
        self.table.setMaximumWidth(420)
        low.addWidget(self.table)
        self.logbox = QtWidgets.QPlainTextEdit()
        self.logbox.setReadOnly(True)
        self.logbox.setMaximumBlockCount(LOG_MAX_LINES)
        self.logbox.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.logbox.setFont(QtGui.QFontDatabase.systemFont(
            QtGui.QFontDatabase.SystemFont.FixedFont))
        low.addWidget(self.logbox, 1)
        vsplit.addWidget(lower)
        vsplit.setStretchFactor(0, 3)
        vsplit.setStretchFactor(1, 1)
        vsplit.setSizes([560, 190])
        self.table.setMinimumHeight(110)
        return right

    def _build_plots(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.plot_alt = make_plot(title="Altitude MSL (m)", x_label="sim time (s)")
        self.plot_acc = make_plot(title="X accel (g)", x_label="sim time (s)")
        self.plot_att = make_plot(title="Tilt / attitude error (deg)",
                                  x_label="sim time (s)")
        self.plots = (self.plot_alt, self.plot_acc, self.plot_att)
        for p in self.plots:
            p.setMinimumHeight(110)
            lay.addWidget(p)
        self.plot_acc.setXLink(self.plot_alt)
        self.plot_att.setXLink(self.plot_alt)
        self.plot_att.hide()
        empty = np.empty(0)
        # (curve, RunState list attribute)
        self.curves = {
            "plot_true": add_curve(self.plot_alt, empty, empty, name="sim true", index=0, width=1.5),
            "plot_baro": add_curve(self.plot_alt, empty, empty, name="Witness Kalman", index=3, width=1.5),
            "plot_accel": add_curve(self.plot_acc, empty, empty, name="sim low-g X", index=0, width=1.5),
            "plot_wacc": add_curve(self.plot_acc, empty, empty, name="Witness accel X", index=3, width=1.5),
            "plot_tilt_truth": add_curve(self.plot_att, empty, empty, name="truth tilt", index=0, width=1.5),
            "plot_tilt_w": add_curve(self.plot_att, empty, empty, name="Witness tilt", index=3, width=1.5),
            "plot_tilt_err": add_curve(self.plot_att, empty, empty, name="tilt err", index=2, width=1.5),
            "plot_att_err": add_curve(self.plot_att, empty, empty, name="att err", index=1, width=1.5),
        }
        self.series = {key: _Series() for key in self.curves}
        self.cursors = []
        for p in self.plots:
            line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(
                "#999", style=QtCore.Qt.PenStyle.DashLine))
            line.hide()
            p.addItem(line, ignoreBounds=True)
            self.cursors.append(line)
        self.plot_alt.setTitle("Altitude MSL (m) - start a sim to plot")
        return box

    # ------------------------------------------------------------ params
    def _build_param_grid(self, cls):
        """(Re)build the parameter editors for SimParams or CsvParams."""
        if self.param_cls is cls:
            return
        self.param_cls = cls
        self.param_vars = {}
        self.param_types = {}
        self.vibration_picker = None
        grid = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(grid)
        form.setContentsMargins(2, 2, 2, 2)
        form.setVerticalSpacing(2)
        for f in dataclasses.fields(cls):
            if f.name == "csv_path":
                continue
            t = str(f.type)
            self.param_types[f.name] = t
            if "bool" in t:
                w = QtWidgets.QCheckBox()
            elif f.name == "after_apogee_attitude":
                w = QtWidgets.QComboBox()
                w.addItems(list(tc.ATTITUDE_MODES))
            elif f.name == "vibration_profile":
                w = QtWidgets.QComboBox()
                w.addItems([hg.VIB_AUTO, hg.VIB_NONE, *hg.vibration_profiles()])
                w.activated.connect(lambda _i: self._vibration_changed())
                self.vibration_picker = w
            else:
                w = QtWidgets.QLineEdit()
                w.setMaximumWidth(110)
                if f.name == "vibration_scale":
                    w.editingFinished.connect(self._vibration_changed)
            form.addRow(f.name, w)
            self.param_vars[f.name] = ParamField(w)
        self.param_scroll.setWidget(grid)   # deletes the previous grid

    def _fill_params(self, values: dict):
        for name, value in values.items():
            field = self.param_vars.get(name)
            if field is None:
                continue
            if "bool" in self.param_types[name]:
                field.set(bool(value))
            elif name == "vibration_profile":
                field.set(hg.vibration_display(value))
            else:
                field.set("" if value is None else str(value))

    def load_preset(self):
        self.csv_path = None
        self.csv_label.setText("(preset source)")
        self._build_param_grid(sm.SimParams)
        self._fill_params(sm.preset_params(self.preset_combo.currentText()).to_dict())
        self.predict()

    def load_csv(self, path: str | None = None):
        """Switch the source to a trajectory CSV (file dialog if no path)."""
        if path is None:
            last = self.settings.get("csv_path") or ""
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Load trajectory CSV",
                os.path.dirname(last) if last else hg.TRAJECTORY_DIR,
                "CSV (*.csv);;All files (*.*)")
            if not path:
                return
        keep = {}
        if self.param_cls is tc.CsvParams:
            try:
                keep = self.read_params().to_dict()
            except ValueError:
                keep = {}
        self._build_param_grid(tc.CsvParams)
        if not keep:
            # A new CSV source starts from the saved vibration choice.
            keep = tc.CsvParams().to_dict()
            keep["vibration_profile"] = self.settings.get("vibration_profile", "")
            keep["vibration_scale"] = self.settings.get("vibration_scale", 1.0)
        self._fill_params(keep)
        self.csv_path = path
        self.csv_label.setText(os.path.basename(path))
        self.csv_label.setToolTip(path)
        self.settings["csv_path"] = path
        hg.save_settings(self.settings, self.settings_path)
        self.predict()

    def read_params(self):
        values = {}
        for name, field in self.param_vars.items():
            t = self.param_types[name]
            if "bool" in t:
                values[name] = bool(field.get())
            elif name == "vibration_profile":
                values[name] = hg.vibration_value(field.get())
            else:
                raw = str(field.get())
                try:
                    values[name] = hg.coerce(t, raw.strip())
                except ValueError:
                    raise ValueError(f"bad value for {name}: {raw!r}")
        if self.param_cls is tc.CsvParams:
            return tc.CsvParams(csv_path=self.csv_path or "", **values)
        return sm.SimParams(**values)

    def _att_thresholds(self) -> dict:
        raw = self.att_tilt_edit.text().strip()
        return {"final_tilt_max_deg": None if raw.lower() in ("", "none") else float(raw)}

    def _running(self) -> bool:
        return self.session.run is not None and self.session.run.running

    def predict(self):
        """Expected transitions for the current source, computed off the GUI
        thread (a CSV takes ~1-2 s to build). Returns the params, or None."""
        try:
            params = self.read_params()
        except ValueError as exc:
            self.predict_label.setText(str(exc))
            return None
        if self._running():
            # Never compete with the streaming loop for the CPU.
            self.predict_label.setText("(prediction paused during the run)")
            return params
        self._predict_token += 1
        token = self._predict_token
        self.predict_label.setText("Predicting…")
        self._run_in_worker("Predicting", lambda: hg.compute_prediction(params),
                            lambda result, error: self._show_prediction(token, result, error))
        return params

    def _show_prediction(self, token, result, error):
        if token != self._predict_token:
            return  # a newer prediction is on its way
        if error is not None:
            prefix = "CSV error: " if isinstance(error, (OSError, ValueError)) else ""
            self.predict_label.setText(f"{prefix}{error}")
            self.vibration_label.setText("")
            return
        self.predict_label.setText(result["text"])
        self.vibration_label.setText(result["vibration"])
        if not self._running():
            self._fill_table(dict(result["expected"]), {})

    def _vibration_changed(self):
        try:
            params = self.read_params()
        except ValueError:
            return
        if isinstance(params, tc.CsvParams):
            self.settings.update(vibration_profile=params.vibration_profile,
                                 vibration_scale=params.vibration_scale)
            hg.save_settings(self.settings, self.settings_path)
        self.predict()

    def _fc_changed(self):
        self.settings.update(
            fetch_fc_logs=self.fc_box.isChecked(),
            pi_host=self.pi_host_edit.text().strip(),
            pi_user=self.pi_user_edit.text().strip(),
            pi_key=self.pi_key_edit.text().strip(),
            fetch_restart_hub=self.restart_hub_check.isChecked())
        self.session.fc_log_config = hg.fc_log_config(self.settings)
        hg.save_settings(self.settings, self.settings_path)

    # ------------------------------------------------------------ workers
    def _run_in_worker(self, label: str, fn, done):
        """Run ``fn`` on Qt's thread pool; ``done(result, error)`` runs on the
        GUI thread. Shows the busy indicator meanwhile. ``fn`` must not hold
        widgets."""
        self._busy.append(label)
        self._job_id += 1
        job = self._job_id
        self._show_busy()
        worker = run_in_thread(
            fn, on_done=lambda result: self._job_finished(job, result, None),
            on_error=lambda exc: self._job_finished(job, None, exc))
        self._jobs[job] = (label, done, worker)   # keeps the signals alive

    def _job_finished(self, job, result, error):
        entry = self._jobs.pop(job, None)
        if entry is None or self._closing:
            return
        label, done, _worker = entry
        if label in self._busy:
            self._busy.remove(label)
        self._show_busy()
        try:
            done(result, error)
        except Exception as exc:  # never break the GUI
            self.append_log(f"[tool] {exc}")

    def wait_idle(self, timeout: float = 60.0) -> bool:
        """Pump Qt until worker jobs finish (tests and scripts)."""
        ok = process_events_until(lambda: not self._busy and not self._preparing,
                                  timeout)
        self._flush_log()
        return ok

    def _show_busy(self):
        if self._busy and self._switch_interval is None:
            self._switch_interval = sys.getswitchinterval()
            sys.setswitchinterval(self.BUSY_SWITCH_INTERVAL_S)
        elif not self._busy and self._switch_interval is not None:
            sys.setswitchinterval(self._switch_interval)
            self._switch_interval = None
        if self._busy:
            self.busy_label.setText(self._busy[-1] + "…")
        self.busy_bar.setVisible(bool(self._busy))
        self.busy_label.setVisible(bool(self._busy))
        self._refresh_buttons()

    @property
    def busy(self) -> bool:
        """True while the busy indicator is shown."""
        return bool(self._busy)

    # ----------------------------------------------------------- actions
    def toggle_connect(self):
        if self._connecting:
            return
        if self.session.connected:
            self.session.disconnect()
            self._refresh_buttons()
            return
        host = self.host_edit.text().strip()
        try:
            port = int(self.port_edit.text())
        except ValueError:
            self.append_log("[tool] invalid port")
            return
        self.settings.update(host=host, port=port)
        hg.save_settings(self.settings, self.settings_path)
        self._connecting = True
        session, closing = self.session, self._closing_evt

        def connect():
            session.connect(host, port, timeout=5)
            if closing.is_set():     # the widget closed while connecting
                session.disconnect()

        def done(_result, error):
            self._connecting = False
            if error is not None:
                self.append_log(f"[tool] connect to {host}:{port} failed: {error}")
            self._refresh_buttons()

        self._run_in_worker(f"Connecting to {host}:{port}", connect, done)

    def enter_hitl(self):
        self._safe(lambda: self.session.send_command(hp.CMD_HITL_ENTER))

    def exit_hitl(self):
        if self._running():
            self.session.stop_run("Exit HITL pressed")
        else:
            self._safe(lambda: self.session.send_command(hp.CMD_HITL_EXIT))

    def start_sim(self):
        if self._preparing:
            return
        try:
            params = self.read_params()
            tol = float(self.tol_edit.text())
            th = self._att_thresholds()
        except ValueError as exc:
            self.append_log(f"[tool] {exc}")
            return
        self.settings.update(preset=self.preset_combo.currentText(), tolerance_s=tol,
                             att_final_tilt_deg=th["final_tilt_max_deg"])
        if isinstance(params, tc.CsvParams):
            self.settings.update(vibration_profile=params.vibration_profile,
                                 vibration_scale=params.vibration_scale)
        hg.save_settings(self.settings, self.settings_path)
        label = (hg.CSV_SOURCE + " " + os.path.basename(self.csv_path)
                 if self.csv_path else self.preset_combo.currentText())
        self._preparing = True
        self.append_log(f"[tool] building run: {label}")
        # CSV parsing, 960 Hz IMU synthesis and 0x0E frame encoding take
        # seconds; they run before streaming starts, off the GUI thread.
        session = self.session
        self._run_in_worker(
            "Building run (960 Hz IMU synthesis)",
            lambda: session.prepare_run(params, preset=label, tolerance_s=tol,
                                        att_thresholds=th),
            self._run_prepared)

    def _run_prepared(self, run, error):
        self._preparing = False
        if error is not None:
            self.append_log(f"[tool] cannot build run: {error}")
            self._refresh_buttons()
            return
        if run.csv_report:
            for line in tc.format_report(run.csv_report).splitlines():
                self.append_log("[csv] " + line)
            for line in hs.HitlSession.imu_summary(run):
                self.append_log("[imu] " + line)
            self.vibration_label.setText(hg.vibration_used(run))
        self.hide_banner()
        self._safe(lambda: self.session.start_run(run))
        self._plot_throttle.request()
        self._refresh_buttons()

    def stop_sim(self):
        self.session.stop_run("stopped by user")
        self._refresh_buttons()

    def _safe(self, fn):
        try:
            fn()
        except (OSError, RuntimeError, ValueError) as exc:
            self.append_log(f"[tool] {exc}")

    def shutdown(self):
        """Stop timers and disconnect (stops any run / sends EXIT)."""
        if self._closing:
            return
        self._closing = True
        self._closing_evt.set()
        # Results of unfinished jobs are dropped (_job_finished checks
        # _closing); the entries stay so their workers' signals stay alive.
        self._busy.clear()
        if self._switch_interval is not None:
            sys.setswitchinterval(self._switch_interval)
            self._switch_interval = None
        self._poll_timer.stop()
        self._plot_throttle.stop()
        if self.session.connected:
            self.session.disconnect()

    # ----------------------------------------------------------- display
    def show_banner(self, text):
        self.banner.setText(text)
        self.banner.show()

    def hide_banner(self):
        self.banner.hide()

    def append_log(self, line: str):
        """Queue a log line; poll() writes queued lines in one batch."""
        self._pending_log.append(line)

    def _flush_log(self):
        if not self._pending_log:
            return
        lines, self._pending_log = self._pending_log[-LOG_MAX_LINES:], []
        self.logbox.appendPlainText("\n".join(lines))
        bar = self.logbox.verticalScrollBar()
        bar.setValue(bar.maximum())

    def log_text(self) -> str:
        return self.logbox.toPlainText()

    def _refresh_buttons(self):
        conn = self.session.connected
        running = self._running()
        busy = self._preparing
        self.connect_btn.setText("Disconnect" if conn else "Connect")
        self.connect_btn.setEnabled(not self._connecting)
        self.conn_label.setText("Connecting…" if self._connecting else
                                "Connected" if conn else "Disconnected")
        # Source changes (CSV parse, prediction) wait while a run streams or
        # is being built, so they never compete with the 0x0E send loop.
        for b, ok in ((self.btn_enter, conn and not running and not busy),
                      (self.btn_start, conn and not running and not busy),
                      (self.btn_stop, running),
                      (self.btn_exit, conn),
                      (self.load_csv_btn, not running and not busy),
                      (self.preset_combo, not running and not busy),
                      (self.predict_btn, not running and not busy)):
            if b.isEnabled() != bool(ok):
                b.setEnabled(bool(ok))

    def _paint_lamps(self, colors):
        for i, (lamp, color) in enumerate(zip(self.lamps, colors)):
            if self._lamp_colors[i] != color:
                self._lamp_colors[i] = color
                lamp.setStyleSheet(f"background: {color}; border-radius: 6px;")

    def _fill_table(self, expected: dict, observed: dict, tol=None):
        rows = []
        for st in hs.TRANSITION_STATES:
            e = expected.get(st)
            o = observed.get(st)
            if e is None and o is None:
                continue
            d = None if e is None or o is None else o - e
            if tol is None:
                res = ""
            elif e is None:
                res = "UNEXPECTED"
            elif o is None:
                res = "pending"
            else:
                res = "OK" if abs(d) <= tol else ("LATE" if d > 0 else "EARLY")
            fmt = (lambda v: "-" if v is None else f"{v:.2f}")
            rows.append((hp.FLIGHT_STATES[st], fmt(e), fmt(o), fmt(d), res))
        if rows == self._table_rows:
            return  # unchanged: no widget churn every poll
        self._table_rows = rows
        self.table.setRowCount(len(rows))
        for r, values in enumerate(rows):
            for c, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                if c == 4 and value in ("LATE", "EARLY", "UNEXPECTED"):
                    item.setForeground(QtGui.QColor("#d02020"))
                elif c == 4 and value == "OK":
                    item.setForeground(QtGui.QColor("#1a9c3a"))
                self.table.setItem(r, c, item)

    def table_rows(self) -> list:
        return list(self._table_rows or [])

    def poll(self):
        if self._closing:
            return
        s = self.session
        for _ in range(5000):
            try:
                kind, payload = s.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("log", "text"):
                self.append_log(payload)
            elif kind == "warning":
                self.show_banner("WARNING: " + payload)
            elif kind == "run" and payload == "finished":
                self._run_finished(s.run)
        v = s.view
        state = "?" if v.flight_state is None else hp.FLIGHT_STATES[v.flight_state]
        self.state_label.setText(f"State: {state}")
        status = v.last_status
        stale = s.now() - v.last_status_time > 3.0
        colors = []
        for _name, fn, kind in STATUS_LAMPS:
            if status is None or stale:
                colors.append("#888")
            elif fn(status):
                colors.append("#1a9c3a" if kind == "good" else "#d02020")
            else:
                colors.append("#555" if kind == "good" else "#c8c8c8")
        self._paint_lamps(colors)
        fmt = (lambda x, f: "-" if x is None else format(x, f))
        run = s.run
        sim = (f"sim {run.sim_ms / 1000:7.2f} s  sent {run.frames_sent}"
               f"  late {run.late_ticks}" if run else "")
        info = (f"sats {fmt(v.gps_sats, 'd')}  lock {'Y' if v.gps_locked else 'N'}"
                f"  KF alt {fmt(v.kalman_alt_m, '.1f')} m AGL  gps {fmt(v.gps_alt_m, '.1f')}"
                f" m  accX {fmt(v.accel_x_g, '.2f')} g  batt "
                f"{fmt(v.battery_mv, 'd')} mV  uptime "
                f"{fmt(None if v.uptime_ms is None else v.uptime_ms / 1000, '.1f')}"
                f" s  echo {v.echo_frames}  nmea {v.nmea_lines}   {sim}")
        if self.info_label.text() != info:
            self.info_label.setText(info)
        self.bytes_label.setText(f"rx {v.rx_bytes} / tx {v.tx_bytes} B")
        if run is not None:
            self._fill_table(run.expected, run.observed, run.tolerance_s)
        if self._plot_signature() != self._plot_sig:
            self._plot_throttle.request()
        self._flush_log()
        self._refresh_buttons()

    def _run_finished(self, run):
        if not run or not run.result:
            return
        result = run.result
        for line in hs.format_table(result).splitlines():
            self.append_log("[result] " + line)
        for line in hg.result_lines(run, result):
            self.append_log("[result] " + line)
        if not result["passed"]:
            self.show_banner("Run FAILED: " + "; ".join(
                result["problems"] or
                [r["state"] + " " + r["result"]
                 for r in result["rows"] if r["result"] != "OK"]))

    # -------------------------------------------------------------- plots
    def _plot_signature(self):
        run = self.session.run
        return (id(run),) + (() if run is None else (
            run.sim_ms, len(run.plot_true), len(run.plot_baro), len(run.plot_wacc),
            len(run.plot_tilt_w), len(run.plot_tilt_err), len(run.plot_att_err),
            len(run.att_records) > 0))

    def showEvent(self, event):
        super().showEvent(event)
        if hasattr(self, "_plot_throttle"):
            self._plot_sig = None
            self._plot_throttle.request()

    def redraw_plot(self):
        """Redraw the plots now (normally the throttle does this)."""
        self._plot_sig = None
        self._draw_plot()

    def _draw_plot(self):
        if self._closing:
            return
        if not self.isVisible() and self._plot_sig is not None:
            return   # hidden tab: drawn again on show
        sig = self._plot_signature()
        if sig == self._plot_sig:
            return
        self._plot_sig = sig
        run = self.session.run
        if run is not self._plot_run:
            self._new_plot_run(run)
        if run is None:
            return
        now = run.sim_ms / 1000
        for key, series in self.series.items():
            if key == "plot_tilt_truth":
                continue
            if series.update(getattr(run, key)):
                self.curves[key].setData(series.x, series.y)
        truth = self.series["plot_tilt_truth"]
        truth.update(run.plot_tilt_truth)
        shown = int(np.searchsorted(truth.x, now, side="right"))
        self.curves["plot_tilt_truth"].setData(truth.x[:shown], truth.y[:shown])
        title = "Tilt / attitude error (deg)"
        if run.att_records and not run.plot_tilt_w:
            title += " - Witness attitude not available (all zero)"
        self.plot_att.setTitle(title)
        t_end = max(run.samples[-1].t_ms / 1000 if run.samples else 1, 1)
        for line in self.cursors:
            line.setPos(min(now, t_end))
            line.show()

    def _new_plot_run(self, run):
        self._plot_run = run
        empty = np.empty(0)
        for key, series in self.series.items():
            series.reset(None)
            self.curves[key].setData(empty, empty)
        if run is None:
            self.plot_alt.setTitle("Altitude MSL (m) - start a sim to plot")
            self.plot_att.hide()
            for line in self.cursors:
                line.hide()
            return
        self.plot_alt.setTitle("Altitude MSL (m)")
        self.plot_att.setVisible(run.truth_quats is not None)
        t_end = max(run.samples[-1].t_ms / 1000 if run.samples else 1, 1)
        self.plot_alt.setXRange(0, t_end, padding=0.01)
        for p in self.plots:
            p.enableAutoRange(axis="y")
