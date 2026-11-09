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
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import wos_paths  # noqa: F401
from main import MonitorPanel


class MonitorTab(ttk.Frame):
    def __init__(self, master, hitl_session=None, *, mirror: bool = True,
                 on_title=None, **monitor_options):
        super().__init__(master)
        self.hitl_session = hitl_session
        self._tap = None
        bar = ttk.Frame(self, padding=(8, 6, 8, 0))
        bar.pack(fill="x")
        self.mirror_var = tk.BooleanVar(value=False)
        self.mirror_check = ttk.Checkbutton(
            bar, text="Mirror HITL tab connection", variable=self.mirror_var,
            command=self._mirror_changed)
        self.mirror_check.pack(side="left")
        self.mirror_note = ttk.Label(
            bar, foreground="#555", justify="left", wraplength=420,
            text=("Witness's raw TCP port serves one client; while the HITL "
                  "tab is connected, mirror it here instead of opening TCP "
                  "to the same port. Read only."))
        self.mirror_note.pack(side="left", fill="x", expand=True, padx=8)
        bar.bind("<Configure>", lambda e: self.mirror_note.configure(
            wraplength=max(150, e.width - 230)))
        if hitl_session is None:
            self.mirror_check.state(["disabled"])
        self.monitor = MonitorPanel(self, on_title=on_title, **monitor_options)
        self.monitor.pack(fill="both", expand=True)
        if mirror and hitl_session is not None:
            self.mirror_var.set(True)
            self._mirror_changed()

    @property
    def mirroring(self) -> bool:
        return self._tap is not None

    def _mirror_changed(self):
        session = self.hitl_session
        if session is None:
            return
        if self.mirror_var.get():
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

    def close(self):
        self.mirror_var.set(False)
        self._mirror_changed()
        self.monitor.close()
