"""Record where the GUI thread is stuck when the window freezes.

A QTimer on the GUI thread stamps a heartbeat every HEARTBEAT_MS. A daemon
thread watches it; when the GUI thread has not run for ``threshold_s`` it
appends the GUI thread's Python stack (and every other thread's) to
``ui_stalls.log`` once per stall, then a line with the total duration when
the stall ends. Nothing is sent anywhere; the log stays on this machine.

    from stall_watchdog import install
    install(log_dir)            # after the QApplication exists
"""

from __future__ import annotations

import datetime as dt
import sys
import threading
import time
import traceback
from pathlib import Path

from PyQt6 import QtCore

HEARTBEAT_MS = 100
DEFAULT_THRESHOLD_S = 1.0
LOG_NAME = "ui_stalls.log"
MAX_LOG_BYTES = 2_000_000

_installed = None


class StallWatchdog(QtCore.QObject):
    def __init__(self, log_path: Path, threshold_s: float = DEFAULT_THRESHOLD_S, parent=None):
        super().__init__(parent)
        self.log_path = Path(log_path)
        self.threshold_s = float(threshold_s)
        self.gui_thread_id = threading.get_ident()
        self.beat = time.monotonic()
        self.stalls = 0
        self._stop = threading.Event()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(HEARTBEAT_MS)
        self._timer.timeout.connect(self._heartbeat)
        self._timer.start()
        self._thread = threading.Thread(target=self._watch, name="ui-stall-watchdog", daemon=True)
        self._thread.start()

    def _heartbeat(self):
        self.beat = time.monotonic()

    def stop(self):
        self._stop.set()
        self._timer.stop()

    def _write(self, text: str):
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            if self.log_path.exists() and self.log_path.stat().st_size > MAX_LOG_BYTES:
                self.log_path.replace(self.log_path.with_suffix(".old.log"))
            with open(self.log_path, "a", encoding="utf-8") as handle:
                handle.write(text)
        except OSError:
            pass

    def snapshot(self) -> str:
        """Every thread's Python stack, the GUI thread first."""
        frames = sys._current_frames()
        names = {t.ident: t.name for t in threading.enumerate()}
        order = sorted(frames, key=lambda ident: ident != self.gui_thread_id)
        parts = []
        for ident in order:
            if ident == threading.get_ident():
                continue
            label = "GUI thread" if ident == self.gui_thread_id else names.get(ident, str(ident))
            parts.append(f"--- {label} ---\n" + "".join(traceback.format_stack(frames[ident])))
        return "\n".join(parts)

    def _watch(self):
        stalled_since = None
        while not self._stop.wait(0.1):
            age = time.monotonic() - self.beat
            if age >= self.threshold_s and stalled_since is None:
                stalled_since = self.beat
                self.stalls += 1
                stamp = dt.datetime.now().isoformat(timespec="seconds")
                self._write(f"\n=== {stamp} GUI thread blocked for {age:.1f} s (still blocked) ===\n"
                            + self.snapshot())
            elif age >= 5 * self.threshold_s and stalled_since is not None and age < 5 * self.threshold_s + 0.1:
                # A second sample a few seconds in shows where a long stall spends its time.
                self._write(f"--- still blocked after {age:.1f} s ---\n" + self.snapshot())
            elif age < self.threshold_s and stalled_since is not None:
                self._write(f"=== stall ended after {time.monotonic() - stalled_since:.1f} s ===\n")
                stalled_since = None


def install(log_dir, threshold_s: float = DEFAULT_THRESHOLD_S) -> StallWatchdog:
    """Start the process-wide watchdog once (later calls return the same one)."""
    global _installed
    if _installed is None:
        _installed = StallWatchdog(Path(log_dir) / LOG_NAME, threshold_s)
    return _installed
