"""witnessOS "Witness monitor" tab: Horizon main.py's monitor, embedded.

The tab is main.MonitorPanel, the same telemetry readout, status decoders,
graphs, console, commands, test source and Horizon JSON feed that
``python Horizon/main.py`` shows, plus one witnessOS addition: mirroring the
HITL bench's Witness connection.

Witness's raw TCP port (ser2net, e.g. prs2026hitl:7100) serves one client at
a time. While the HITL tab holds it, the monitor cannot open a second
connection, so "Mirror HITL tab connection" feeds the bytes the HITL session
receives (HitlSession.rx_tap) through the monitor's own decoder instead.
Mirroring only reads; nothing is sent to Witness.

What this tab needs from main.MonitorPanel (a QWidget):
    MonitorPanel(on_title=callable, **monitor_options)
    .stream_feeder(source_name) -> feed(bytes)   called on the HITL reader
                                                 thread; the panel marshals
                                                 to its GUI thread itself
    .console_write(line)                         GUI thread
    .shutdown() (or a non-QWidget .close())      stops inputs and the feed
"""

from __future__ import annotations

from PyQt6 import QtWidgets

import wos_paths  # noqa: F401


def shutdown_panel(panel) -> None:
    """Call a hosted panel's shutdown(); fall back to a Tk-style close()."""
    stop = getattr(panel, "shutdown", None)
    if stop is None:
        stop = getattr(panel, "close", None)
    if stop is not None:
        stop()


class MonitorTab(QtWidgets.QWidget):
    def __init__(self, parent=None, hitl_session=None, *, mirror: bool = True,
                 on_title=None, **monitor_options):
        super().__init__(parent)
        import main as horizon_main   # imported here so tests can stand in
        self.hitl_session = hitl_session
        self._tap = None
        self._closed = False
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        bar.setContentsMargins(8, 6, 8, 0)
        self.mirror_check = QtWidgets.QCheckBox("Mirror HITL tab connection")
        bar.addWidget(self.mirror_check)
        self.mirror_note = QtWidgets.QLabel(
            "Witness's raw TCP port serves one client; while the HITL tab is "
            "connected, mirror it here instead of opening TCP to the same "
            "port. Read only.")
        self.mirror_note.setWordWrap(True)
        self.mirror_note.setStyleSheet("color: #555;")
        bar.addWidget(self.mirror_note, 1)
        lay.addLayout(bar)
        if hitl_session is None:
            self.mirror_check.setEnabled(False)
        self.monitor = horizon_main.MonitorPanel(on_title=on_title, **monitor_options)
        lay.addWidget(self.monitor, 1)
        if mirror and hitl_session is not None:
            self.mirror_check.setChecked(True)
            self._mirror_changed()
        self.mirror_check.toggled.connect(lambda _on: self._mirror_changed())

    @property
    def mirroring(self) -> bool:
        return self._tap is not None

    def set_mirroring(self, on: bool) -> None:
        self.mirror_check.setChecked(bool(on))
        self._mirror_changed()

    def _mirror_changed(self):
        session = self.hitl_session
        if session is None:
            return
        if self.mirror_check.isChecked():
            if self._tap is None:
                self._tap = self.monitor.stream_feeder("HITL")
                session.rx_tap = self._tap
                self.monitor.console_write(
                    "MIRROR decoding the HITL tab's Witness connection "
                    "(read only)")
        elif self._tap is not None:
            if session.rx_tap is self._tap:
                session.rx_tap = None
            self._tap = None
            self.monitor.console_write("MIRROR stopped")

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self.mirror_check.blockSignals(True)
        self.mirror_check.setChecked(False)
        self._mirror_changed()
        shutdown_panel(self.monitor)
