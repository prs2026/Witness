"""Shared helpers for the Qt viewer tests (headless, offscreen platform)."""

import os
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6 import QtCore, QtWidgets  # noqa: E402

import qt_common  # noqa: E402
import viewer  # noqa: E402

UPLOADS = Path(r"C:\Users\E\.claude\uploads\57c05c66-37ca-4475-bfbd-0c3c8c766392")
RAVEN_LR = UPLOADS / "eb0d6a14-Sohn_B_Raven_LR_07-19-2025_12_26_24.csv"
RAVEN_HR = UPLOADS / "c90785de-Sohn_B_Raven_HR_07-19-2025_12_26_24.csv"


def pump(seconds: float = 0.0) -> None:
    app = qt_common.ensure_app()
    deadline = time.monotonic() + seconds
    while True:
        app.processEvents()
        if time.monotonic() >= deadline:
            break
        QtCore.QThread.msleep(5)


class StallProbe(QtCore.QObject):
    """A 5 ms QTimer; the longest gap between its ticks is the longest time
    the GUI thread was blocked while the probe ran."""

    def __init__(self, interval_ms: int = 5):
        super().__init__()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._tick)
        self.last = None
        self.max_gap = 0.0

    def _tick(self):
        now = time.perf_counter()
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        self.last = now

    def __enter__(self):
        self.last = time.perf_counter()
        self.max_gap = 0.0
        self.timer.start()
        return self

    def __exit__(self, *exc):
        self._tick()
        self.timer.stop()
        return False


class ViewerCase(unittest.TestCase):
    """A temp directory and CsvViewerWidgets that are shut down afterwards."""

    def setUp(self):
        self.app = qt_common.ensure_app()
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def make(self, width=1000, height=700, **kwargs) -> "viewer.CsvViewerWidget":
        widget = viewer.CsvViewerWidget(**kwargs)
        widget.show_dialogs = False
        widget.resize(width, height)
        widget.show()
        self.addCleanup(self._dispose, widget)
        pump()
        return widget

    def _dispose(self, widget):
        widget.shutdown()
        widget.close()
        widget.deleteLater()
        pump()

    def plots(self, widget):
        widget.notebook.setCurrentWidget(widget.plot_tab)
        pump()
        widget.realize_visible_now()
        pump()

    def write_text(self, name, text) -> Path:
        path = Path(self.tmp) / name
        path.write_text(text)
        return path
