"""witnessOS: the Witness bench tools in one tabbed window.

    python witnessos.py                  # all three tabs
    python witnessos.py flight.csv       # open a converted CSV in the viewer
    python witnessos.py --tab relay      # start on a given tab
    python witnessos.py --no-feed        # do not start the Horizon feed

Tabs (each reuses the original tool by import):
  HITL bench     Software/HITL/hitl_gui.py   (HitlApp embedded in a frame)
  CSV viewer     Software/Horizon/viewer.py  (CsvViewerPanel)
  Witness relay  Witness bytes -> Horizon JSON feed, using Horizon/main.py's
                 PacketDecoder and witness_link.WitnessPublisher

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
from wos_relay import RelayEngine
from wos_relay_tab import RelayTab
from wos_widgets import ScrollableFrame, WheelRouter

APP_NAME = "witnessOS"
TABS = ("hitl", "viewer", "relay")
TAB_TITLES = {"hitl": "HITL bench", "viewer": "CSV viewer",
              "relay": "Witness relay"}


class WitnessOS:
    """Owns the root window, the notebook and the three tool panels."""

    def __init__(self, root, *, csv_file: Path | None = None,
                 start_tab: str = "hitl",
                 hitl_settings: dict | None = None,
                 hitl_settings_path: str = hitl_gui.SETTINGS_PATH,
                 hitl_runs_dir: str | None = hs.DEFAULT_RUNS_DIR,
                 relay_engine: RelayEngine | None = None,
                 start_feed: bool = True):
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

        self.viewer = viewer.CsvViewerPanel(
            self.frames["viewer"].inner, csv_file, on_title=self._viewer_titled)
        self.viewer.pack(fill="both", expand=True)

        self.relay = RelayTab(self.frames["relay"].inner,
                              hitl_session=self.hitl.session,
                              engine=relay_engine, start_feed=start_feed)
        self.relay.pack(fill="both", expand=True)

        self.notebook.bind("<<NotebookTabChanged>>", self._tab_changed)
        self.select(start_tab if csv_file is None else "viewer")

    def select(self, key: str):
        self.notebook.select(self.frames[key])

    def current_tab(self) -> str:
        widget = self.root.nametowidget(self.notebook.select())
        return next(k for k, f in self.frames.items() if f is widget)

    def _viewer_titled(self, text: str):
        self._viewer_title = text
        self._tab_changed()

    def _tab_changed(self, _event=None):
        try:
            key = self.current_tab()
        except (StopIteration, KeyError, tk.TclError):
            return
        suffix = TAB_TITLES[key]
        if key == "viewer" and self._viewer_title:
            suffix = self._viewer_title
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
            self.relay.close()
        finally:
            try:
                self.hitl.shutdown()  # stops any run, sends HITL EXIT
            finally:
                self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv_file", nargs="?", type=Path,
                    help="converted CSV to open in the viewer tab")
    ap.add_argument("--tab", choices=TABS, default="hitl",
                    help="tab to show first")
    ap.add_argument("--no-feed", action="store_true",
                    help="do not start the Horizon Witness feed at launch")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    root = tk.Tk()
    WitnessOS(root, csv_file=args.csv_file, start_tab=args.tab,
              start_feed=not args.no_feed)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
