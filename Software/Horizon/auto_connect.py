"""Shared Tk tab auto-connect controls (network/serial I/O stays in workers)."""

import time
import tkinter as tk
from tkinter import ttk

from serial_discovery import RETRY_SECONDS


class AutoConnectTab:
    def init_auto_connect(self, parent, **grid):
        self.auto_connect = tk.BooleanVar(value=False)
        self._next_connect = 0
        self.auto_button = ttk.Checkbutton(parent, text="Auto-connect", variable=self.auto_connect,
                                          command=self.auto_changed)
        self.auto_button.grid(**grid)

    def set_auto_connect(self, enabled):
        self.auto_connect.set(enabled)
        self.auto_changed()

    def auto_changed(self):
        self._next_connect = 0
        if self.receiver is not None:
            self.receiver.auto_retry = self.auto_connect.get()

    def auto_tick(self):
        if self.receiver is not None and not self.receiver.running:
            # Preserve the final diagnostic and displayed telemetry after loss.
            status = self.receiver.status
            self.disconnect()
            self.status.set(status)
            self._next_connect = time.monotonic() + RETRY_SECONDS
        if not self.auto_connect.get() or self.receiver is not None:
            return
        now = time.monotonic()
        if now < self._next_connect:
            return
        self._next_connect = now + RETRY_SECONDS
        if self.prepare_auto_connect():
            self.connect(automatic=True)

    def prepare_auto_connect(self):
        return True
