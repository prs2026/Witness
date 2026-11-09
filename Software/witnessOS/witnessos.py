"""witnessOS: the Witness bench tools in one tabbed window.

    python witnessos.py                  # all three tabs
    python witnessos.py flight.csv       # open a converted CSV in the viewer
    python witnessos.py a.csv b.csv      # overlay several CSVs
    python witnessos.py --tab monitor    # start on a given tab
    python witnessos.py --no-feed        # do not start the Horizon feed
    python witnessos.py --no-auto-connect  # monitor: no automatic USB opens

Tabs (each reuses the original tool by import):
  HITL bench     Software/HITL/hitl_gui.py   (HitlApp embedded in a frame)
  CSV viewer     Software/Horizon/viewer.py  (CsvViewerPanel)
  Witness monitor  Software/Horizon/main.py   (MonitorPanel: the monitor's
                 telemetry readout, graphs, console, commands and Horizon
                 JSON feed), plus mirroring of the HITL tab's connection

The standalone tools and their CLIs (hitl_gui.py --headless, viewer.py,
main.py, converter.py) are unchanged in behaviour and still run on their own.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import wos_paths  # noqa: F401  (must precede the tool imports)
import tkinter as tk
from tkinter import ttk

import hitl_gui
import hitl_session as hs
import viewer
from wos_monitor_tab import MonitorTab
from wos_widgets import ScrollableFrame, WheelRouter

APP_NAME = "witnessOS"
TABS = ("hitl", "viewer", "monitor")
TAB_TITLES = {"hitl": "HITL bench", "viewer": "CSV viewer",
              "monitor": "Witness monitor"}
TAB_ALIASES = {"relay": "monitor"}


class WitnessOS:
    """Owns the root window, the notebook and the three tool panels."""

    def __init__(self, root, *, csv_files=None,
                 start_tab: str = "hitl",
                 hitl_settings: dict | None = None,
                 hitl_settings_path: str = hitl_gui.SETTINGS_PATH,
                 hitl_runs_dir: str | None = hs.DEFAULT_RUNS_DIR,
                 monitor_options: dict | None = None):
        self.root = root
        self._closed = False
        self._viewer_title = ""
        root.title(APP_NAME)
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{max(480, min(1320, sw - 40))}x"
                      f"{max(360, min(900, sh - 80))}")
        root.minsize(420, 300)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.router = WheelRouter(root)

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)
        self.frames: dict[str, ScrollableFrame] = {}
        for key in TABS:
            frame = ScrollableFrame(self.notebook, router=self.router)
            self.frames[key] = frame
            self.notebook.add(frame, text=TAB_TITLES[key])

        self.hitl = hitl_gui.HitlApp(
            root, settings=hitl_settings, settings_path=hitl_settings_path,
            runs_dir=hitl_runs_dir, parent=self.frames["hitl"].inner)
        self.router.register(self.hitl.param_canvas)

        csv_files = [Path(p) for p in (csv_files or [])]
        self.viewer = viewer.CsvViewerPanel(
            self.frames["viewer"].inner, csv_files or None,
            on_title=self._viewer_titled)
        self.viewer.pack(fill="both", expand=True)

        self._monitor_title = ""
        self.monitor = MonitorTab(self.frames["monitor"].inner,
                                  hitl_session=self.hitl.session,
                                  on_title=self._monitor_titled,
                                  **(monitor_options or {}))
        self.monitor.pack(fill="both", expand=True)

        self.notebook.bind("<<NotebookTabChanged>>", self._tab_changed)
        start_tab = TAB_ALIASES.get(start_tab, start_tab)
        self.select("viewer" if csv_files else start_tab)

    def select(self, key: str):
        self.notebook.select(self.frames[key])

    def current_tab(self) -> str:
        widget = self.root.nametowidget(self.notebook.select())
        return next(k for k, f in self.frames.items() if f is widget)

    def _viewer_titled(self, text: str):
        self._viewer_title = text
        self._tab_changed()

    def _monitor_titled(self, text: str):
        self._monitor_title = ("" if text == "Witness Communications Monitor"
                               else text)
        self._tab_changed()

    def _tab_changed(self, _event=None):
        try:
            key = self.current_tab()
        except (StopIteration, KeyError, tk.TclError):
            return
        suffix = TAB_TITLES[key]
        if key == "viewer" and self._viewer_title:
            suffix = self._viewer_title
        elif key == "monitor" and self._monitor_title:
            suffix = self._monitor_title
        self.root.title(f"{APP_NAME} - {suffix}")
        # A tab hidden while the window was resized redraws on show.
        if key == "hitl":
            self.hitl._schedule_plot()
        elif key == "viewer":
            self.viewer.schedule_redraw()

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.monitor.close()
        finally:
            try:
                self.hitl.shutdown()  # stops any run, sends HITL EXIT
            finally:
                self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv_files", nargs="*", type=Path,
                    help="converted CSV(s) to open (overlaid) in the viewer")
    ap.add_argument("--tab", choices=TABS + tuple(TAB_ALIASES),
                    default="hitl", help="tab to show first")
    ap.add_argument("--no-feed", action="store_true",
                    help="do not start the Horizon Witness feed at launch")
    ap.add_argument("--no-auto-connect", action="store_true",
                    help="monitor: do not open Witness USB ports "
                    "automatically")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    root = tk.Tk()
    WitnessOS(root, csv_files=args.csv_files, start_tab=args.tab,
              monitor_options={"start_feed": not args.no_feed,
                               "auto_connect": not args.no_auto_connect})
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
