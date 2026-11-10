"""Witness feed and Featherweight serial tabs for Horizon Unified (Qt).

Connection logic is auto_connect.ReceiverControl and the receivers are
ground_receivers (worker threads); these widgets only show their state.
"""

from __future__ import annotations

from PyQt6 import QtCore, QtWidgets

from auto_connect import ReceiverControl
from featherweight import FEATHERWEIGHT_BAUD
from ground_receivers import FeatherweightReceiver, WitnessReceiver
from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT
from observable import Cell
from qt_common import message_error
from qt_widgets import bind_check, bind_combo, bind_label, bind_line_edit, hbox, wrap_label
from serial_discovery import SerialDiscovery
from telemetry_model import FEATHERWEIGHT_FIELDS, WITNESS_FIELDS  # noqa: F401 - re-exported
from telemetry_view import TelemetryView

NO_PORT = "(no port)"
POLL_MS = 100


class ReceiverTab(ReceiverControl, QtWidgets.QWidget):
    def __init__(self, parent=None, log_dir=None, *, witness=False, schema=None, on_record=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.init_receiver_control()
        self.log_dir, self.witness = log_dir, witness
        self.on_record = on_record
        self.entries = []
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self.controls = QtWidgets.QHBoxLayout()
        layout.addLayout(self.controls)
        self.status_label, self.stats_label = wrap_label(), wrap_label()
        bind_label(self.status, self.status_label)
        bind_label(self.stats, self.stats_label)
        self.note = wrap_label(muted=True)
        self.log_label = wrap_label(muted=True)
        bind_label(self.log_status, self.log_label)
        for widget in (self.status_label, self.stats_label, self.note):
            layout.addWidget(widget)
        self.view = TelemetryView(schema=schema)
        layout.addWidget(self.view, 1)
        layout.addWidget(self.log_label)
        self.connect_button = QtWidgets.QPushButton("Connect")
        bind_label(self.connect_text, self.connect_button)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.auto_button = QtWidgets.QCheckBox("Auto-connect")
        bind_check(self.auto_connect, self.auto_button, on_user=self.auto_changed)
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

    # ReceiverControl view hooks.
    def alert(self, title, text):
        message_error(self, title, text)

    def reset_view(self):
        self.view.clear()

    def show_records(self, updates, diagnostics):
        for record in updates:
            self.remember_source(record)
            try:
                self.view.update_record(record, witness=self.witness)
            except (ValueError, TypeError, OverflowError, OSError) as exc:
                diagnostics.append(f"DISPLAY_ERROR: {exc}")
        self.view.append(diagnostics)

    def set_inputs_enabled(self, enabled):
        for entry in self.entries:
            entry.setEnabled(enabled)

    def _poll(self):
        try:
            self.poll()
        finally:
            self.view.update_ages()

    def close(self):
        self._timer.stop()
        self.disconnect()
        return QtWidgets.QWidget.close(self)


class WitnessTab(ReceiverTab):
    def __init__(self, parent=None, *, log_dir, on_record=None):
        super().__init__(parent, log_dir, witness=True, schema=WITNESS_FIELDS, on_record=on_record)
        self.host = Cell(WITNESS_FEED_HOST)
        self.port = Cell(str(WITNESS_FEED_PORT))
        host_edit, port_edit = QtWidgets.QLineEdit(), QtWidgets.QLineEdit()
        port_edit.setMaximumWidth(80)
        bind_line_edit(self.host, host_edit)
        bind_line_edit(self.port, port_edit)
        self.entries += [host_edit, port_edit]
        self.controls.addLayout(hbox(QtWidgets.QLabel("Witness app host:"), host_edit,
                                     QtWidgets.QLabel("Feed port:"), port_edit, self.connect_button,
                                     self.auto_button, None))
        self.note.setText(
            "Receive only: start main.py to supply decoded Witness telemetry. "
            "Auto-connect retries while enabled; snapshots retain original age. Units: g, dps, m, m/s, V, A, C, Pa; "
            "uptime in ms. kalman_altitude and position/velocity are the Kalman filter's ENU estimate "
            "relative to the pad (m, m/s); gps_altitude is m MSL and GPS fields read 0 without "
            "status.gps_locked. An all-zero orientation means attitude not available (not yet "
            "initialised, or lost after gyro saturation in flight). ground_station_status carries "
            "Aperture's own LC86G GPS (pad position). Live graphs are on Flight View, attitude/map in "
            "Windows > Visualizer; commands remain in main.py.")

    def make_receiver(self):
        return WitnessReceiver(str(self.host.get()).strip(), int(self.port.get()), self.log_dir,
                               on_record=self.on_record)


class FeatherweightTab(ReceiverTab):
    def __init__(self, parent=None, *, log_dir, on_record=None):
        super().__init__(parent, log_dir, schema=FEATHERWEIGHT_FIELDS, on_record=on_record)
        self.discovery = SerialDiscovery("featherweight")
        self.port = Cell(NO_PORT)
        self.port_choices = Cell((NO_PORT,))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(120)
        bind_combo(self.port, self.port_combo, self.port_choices)
        self.entries.append(self.port_combo)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.controls.addLayout(hbox(QtWidgets.QLabel("Featherweight COM port:"), self.port_combo, refresh,
                                     self.connect_button, self.auto_button, None))
        self.note.setText(
            f"{FEATHERWEIGHT_BAUD} baud, 8-N-1, receive only. "
            "Altitude: ft ASL; velocity: ft/s; coordinates/heading: degrees. "
            "Fix: 0 = no fix, 2 = 2D, 3 = 3D. Bad text/radio CRC is shown in red. "
            "BATT_BLE = local/GS battery; RX_NOMTK = tracker battery. Voltages: V and raw mV; "
            "temperature: C; RSSI: dBm; SNR: dB; TX airtime: ms; frequency: Hz. "
            "Binary FWT chatter without a tracker is ignored; raw bytes are logged. "
            "Auto-connect finds Featherweight-named USB devices or learned USB identities. "
            "For generic adapters select the port and Connect once to learn it.")
        self.refresh_ports()
        self.view.add_source(NO_PORT)

    def refresh_ports(self):
        self.discovery.refresh(force=True)
        self._update_ports()

    def _update_ports(self):
        if self.receiver is not None:
            return
        ports = [port.device for port in self.discovery.ports]
        current = self.port.get()
        self.port_choices.set((NO_PORT, *ports))
        self.port.set(current if current in ports else NO_PORT)

    def auto_tick(self):
        self.discovery.refresh()
        self._update_ports()
        if self.discovery.error:
            self.view.append([self.discovery.error])
            self.discovery.error = None
        super().auto_tick()

    def prepare_auto_connect(self):
        candidates = self.discovery.candidates()
        if not candidates:
            self.status.set("Auto-connect: waiting for a known Featherweight USB device; "
                            "select generic adapters manually once")
            return False
        self.port.set(candidates[0])
        self.discovery.attempted(candidates[0])
        return True

    def remember_source(self, record):
        if record["packet"].get("text_crc_valid"):
            self.discovery.remember(record["source"])

    def make_receiver(self):
        return FeatherweightReceiver(self.port.get(), self.log_dir, on_record=self.on_record)

    def prepare_view(self):
        self.view.add_source(self.port.get())
