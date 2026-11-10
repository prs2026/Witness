"""Independent AIM XTRA MQTT tab for Horizon (Qt).

Receive only: subscribes to <base path>/# on the broker the vendor AIM BASE
app publishes to. The embedded local-only broker (mqtt_broker.py) runs by
default. Field state is telemetry_model.AimFieldTable; the connection
lifecycle is auto_connect.ReceiverControl. Click a numeric field to plot it
live in the strip chart (pyqtgraph, redrawn by a throttle, not per message).
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
import time

import numpy as np
from PyQt6 import QtCore, QtWidgets

from aim_mqtt import AIM_TOPIC_FIELDS, AimMqttReceiver, AimSettings
from auto_connect import ReceiverControl
from ground_settings import DEFAULT_MQTT_BROKER_ENABLED, DEFAULT_MQTT_BROKER_PORT
from mqtt_broker import EmbeddedMqttBroker
from observable import Cell
from qt_common import Throttle, add_curve, make_plot, message_error
from qt_widgets import LogConsole, bind_check, bind_label, bind_line_edit, hbox, wrap_label
from telemetry_model import AimFieldTable
from telemetry_view import configure_tree, paint_row

COLUMNS = (("Topic / field", 340), ("Latest value", 200), ("Received at (UTC)", 205),
           ("Receive age", 95), ("Delivery", 80))
POLL_MS = 100
CHART_POINTS = 3000


class AimMqttTab(ReceiverControl, QtWidgets.QWidget):
    stats_format = "Messages: {messages}   Bytes: {bytes}   Errors: {errors}"

    def __init__(self, parent=None, *, log_dir: Path, host, port, client_uuid, base_path, on_sample=None,
                 broker_enabled=DEFAULT_MQTT_BROKER_ENABLED, broker_port=DEFAULT_MQTT_BROKER_PORT):
        QtWidgets.QWidget.__init__(self, parent)
        self.init_receiver_control()
        self.log_dir = log_dir
        self.on_sample = on_sample
        self.model = AimFieldTable(AIM_TOPIC_FIELDS)
        self._groups, self._rows = {}, {}
        self.host = Cell(host)
        self.port = Cell(str(port))
        self.client_uuid = Cell(client_uuid)
        self.base_path = Cell(base_path)
        self.broker = EmbeddedMqttBroker(log_dir, port=broker_port)
        self.broker_enabled = Cell(bool(broker_enabled))
        self.broker_status = Cell("Embedded broker off")
        self._last_broker_status = None
        self.chart_key = None
        self.chart_points = deque(maxlen=CHART_POINTS)
        self._build_ui()
        self.reset_view()
        self._chart_throttle = Throttle(self._draw_chart, 50, self)
        self.broker_changed()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self.entries = []
        widgets = []
        for label, cell, width in (("MQTT host:", self.host, None), ("Port:", self.port, 70),
                                   ("Subscriber UUID:", self.client_uuid, 110)):
            edit = QtWidgets.QLineEdit()
            if width:
                edit.setMaximumWidth(width)
            bind_line_edit(cell, edit)
            self.entries.append(edit)
            widgets += [QtWidgets.QLabel(label), edit]
        self.connect_button = QtWidgets.QPushButton("Connect")
        bind_label(self.connect_text, self.connect_button)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.auto_button = QtWidgets.QCheckBox("Auto-connect")
        bind_check(self.auto_connect, self.auto_button, on_user=self.auto_changed)
        row = hbox(*widgets, self.connect_button, self.auto_button)
        row.setStretch(1, 1)
        layout.addLayout(row)
        base = QtWidgets.QLineEdit()
        bind_line_edit(self.base_path, base)
        self.entries.append(base)
        layout.addLayout(hbox(QtWidgets.QLabel("Base path:"), base, QtWidgets.QLabel("/#  (receive only)")))
        self.broker_button = QtWidgets.QCheckBox("Run embedded MQTT broker (local only)")
        bind_check(self.broker_enabled, self.broker_button, on_user=self.broker_changed)
        layout.addLayout(hbox(self.broker_button, wrap_label(
            "Turning this off disconnects its clients; an external broker is unaffected.", muted=True)))
        for cell in (self.status, self.stats, self.broker_status):
            label = wrap_label()
            bind_label(cell, label)
            layout.addWidget(label)
        layout.addWidget(wrap_label(
            "Values and timestamps are shown as published. Retained messages may be old. "
            "Drag column dividers to resize; click a numeric field to plot it below.", muted=True))
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        self.tree = QtWidgets.QTreeWidget()
        configure_tree(self.tree, COLUMNS)
        self.tree.currentItemChanged.connect(self._select_field)
        splitter.addWidget(self.tree)
        self.chart = make_plot(title="Selected field: none", x_label="age [s]", legend=False)
        self.chart.setMinimumHeight(120)
        self.chart_curve = add_curve(self.chart, np.empty(0), np.empty(0), width=1.5)
        splitter.addWidget(self.chart)
        box = QtWidgets.QGroupBox("MQTT messages and diagnostics")
        box_layout = QtWidgets.QVBoxLayout(box)
        box_layout.setContentsMargins(4, 4, 4, 4)
        self.raw = LogConsole(max_lines=1000)
        box_layout.addWidget(self.raw)
        splitter.addWidget(box)
        for index, stretch in enumerate((4, 2, 1)):
            splitter.setStretchFactor(index, stretch)
        layout.addWidget(splitter, 1)
        log_label = wrap_label(muted=True)
        bind_label(self.log_status, log_label)
        layout.addWidget(log_label)

    # -------------------------------------------------------------- broker
    def broker_changed(self):
        if self.broker_enabled.get():
            self.broker.start()
        else:
            self.broker.stop()

    def _poll_broker(self):
        state, message, path = self.broker.snapshot()
        # Honor a rapid off/on toggle once the previous listener has closed.
        if self.broker_enabled.get() and state == "stopped" and not self.broker.alive:
            self.broker.start()
            state, message, path = self.broker.snapshot()
        self.broker_status.set(message)
        status = (state, message, path)
        if status != self._last_broker_status:
            self._last_broker_status = status
            self.show_lines([f"BROKER: {message}" + (f" | Log: {path}" if path else "")])

    # ---------------------------------------------------------------- rows
    def _item(self, row):
        key = (row.topic, row.field)
        item = self._rows.get(key)
        if item is None:
            group = self._groups.get(row.topic)
            if group is None:
                group = self._groups[row.topic] = QtWidgets.QTreeWidgetItem(self.tree, [row.topic])
                group.setExpanded(True)
            item = self._rows[key] = QtWidgets.QTreeWidgetItem(group, [row.field, "null", "--", "--", "--"])
            item.setData(1, QtCore.Qt.ItemDataRole.UserRole, key)
        return item

    def _show(self, row):
        item = self._item(row)
        for column, text in ((1, row.value), (2, row.received), (3, row.age), (4, row.delivery)):
            if item.text(column) != text:
                item.setText(column, text)
        if item.data(0, QtCore.Qt.ItemDataRole.UserRole) != row.tag:
            paint_row(item, row.tag)

    def _reset_fields(self):
        self.tree.clear()
        self._groups.clear()
        self._rows.clear()
        self.model.reset()
        for row in self.model.rows.values():
            self._show(row)

    def value(self, topic, field):
        return self._rows[(topic, field)].text(1)

    # ------------------------------------------------------ receiver hooks
    def reset_view(self):
        self._reset_fields()

    def alert(self, title, text):
        message_error(self, "AIM XTRA MQTT", text)

    def make_receiver(self):
        settings = AimSettings(str(self.host.get()).strip(), int(self.port.get()),
                               str(self.client_uuid.get()).strip(),
                               str(self.base_path.get()).strip().rstrip("/"))
        self._settings = settings
        return AimMqttReceiver(settings, self.log_dir, on_sample=self.on_sample)

    def connecting_status(self, receiver):
        self.stats.set(self.stats_format.format(messages=0, bytes=0, errors=0))
        return f"Connecting to {self._settings.host}:{self._settings.port}..."

    def stop_receiver(self, receiver):
        receiver.stop()

    def disconnect(self):
        super().disconnect()
        self.status.set("Disconnected — latest values retained")

    def set_inputs_enabled(self, enabled):
        for entry in self.entries:
            entry.setEnabled(enabled)

    def show_records(self, updates, diagnostics):
        for row in self.model.apply(updates):
            self._show(row)
            if (row.topic, row.field) == self.chart_key and row.number is not None and not row.invalid:
                self.chart_points.append((row.received_mono, row.number))
                self._chart_throttle.request()
        if self.model.limit_notice:
            self.model.limit_notice = False
            diagnostics = [*diagnostics, "Display field limit reached; further fields remain in the log."]
        self.show_lines(diagnostics)

    def show_lines(self, lines):
        self.raw.append_lines(lines)

    _append_lines = show_lines

    # --------------------------------------------------------------- chart
    def _select_field(self, item, _previous=None):
        key = item.data(1, QtCore.Qt.ItemDataRole.UserRole) if item is not None else None
        if key is None or key == self.chart_key:
            return
        self.chart_key = key
        self.chart_points.clear()
        row = self.model.rows.get(key)
        if row is not None and row.number is not None and row.received_mono is not None:
            self.chart_points.append((row.received_mono, row.number))
        self.chart.setTitle(f"Selected field: {key[0]} / {key[1]}")
        self._chart_throttle.flush()

    def _draw_chart(self):
        if not self.chart_points:
            self.chart_curve.setData([], [])
            return
        data = np.array(self.chart_points, dtype=float)
        self.chart_curve.setData(data[:, 0] - time.monotonic(), data[:, 1])

    # ---------------------------------------------------------------- poll
    def _poll(self):
        try:
            self._poll_broker()
            self.poll()
        finally:
            for row in self.model.update_ages():
                self._show(row)

    def close(self):
        self._timer.stop()
        self._chart_throttle.stop()
        self.disconnect()
        self.broker.stop(wait=True)
        return QtWidgets.QWidget.close(self)
