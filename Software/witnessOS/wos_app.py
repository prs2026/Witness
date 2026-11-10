"""witnessOS main window (PyQt6): a QTabWidget with the three tool panels.

Started by witnessos.py; see that module for the command line.
"""

from __future__ import annotations

from pathlib import Path

import wos_paths  # noqa: F401  (must precede the tool imports)
from PyQt6 import QtGui, QtWidgets

import hitl_gui
import hitl_session as hs
from qt_common import ensure_app
from witnessos import APP_NAME, TAB_ALIASES, TAB_TITLES, TABS
from wos_hitl_tab import HitlWidget
from wos_monitor_tab import MonitorTab, shutdown_panel
from wos_widgets import scroll_tab


class WitnessOS(QtWidgets.QMainWindow):
    """The main window: a QTabWidget holding the three tool panels."""

    def __init__(self, *, csv_files=None,
                 start_tab: str = "hitl",
                 hitl_settings: dict | None = None,
                 hitl_settings_path: str = hitl_gui.SETTINGS_PATH,
                 hitl_runs_dir: str | None = hs.DEFAULT_RUNS_DIR,
                 monitor_options: dict | None = None):
        ensure_app()
        super().__init__()
        import viewer   # Horizon; imported here so tests can stand in
        self._closed = False
        self._viewer_title = ""
        self._monitor_title = ""
        self.setWindowTitle(APP_NAME)
        screen = QtGui.QGuiApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            sw, sh = avail.width(), avail.height()
        else:  # pragma: no cover
            sw, sh = 1360, 980
        self.resize(max(480, min(1320, sw - 40)), max(360, min(900, sh - 80)))
        self.setMinimumSize(420, 300)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.setCentralWidget(self.tabs)

        self.hitl = HitlWidget(
            settings=hitl_settings, settings_path=hitl_settings_path,
            runs_dir=hitl_runs_dir)
        csv_files = [Path(p) for p in (csv_files or [])]
        self.viewer = viewer.CsvViewerWidget(files=tuple(csv_files),
                                             on_title=self._viewer_titled)
        self.monitor = MonitorTab(hitl_session=self.hitl.session,
                                  on_title=self._monitor_titled,
                                  **(monitor_options or {}))
        self.panels = {"hitl": self.hitl, "viewer": self.viewer,
                       "monitor": self.monitor}
        self.pages: dict[str, QtWidgets.QScrollArea] = {}
        for key in TABS:
            page = scroll_tab(self.panels[key])
            self.pages[key] = page
            self.tabs.addTab(page, TAB_TITLES[key])

        self.tabs.currentChanged.connect(lambda _i: self._tab_changed())
        start_tab = TAB_ALIASES.get(start_tab, start_tab)
        self.select("viewer" if csv_files else start_tab)
        self._tab_changed()

    def select(self, key: str):
        self.tabs.setCurrentWidget(self.pages[TAB_ALIASES.get(key, key)])

    def current_tab(self) -> str:
        page = self.tabs.currentWidget()
        return next(k for k, p in self.pages.items() if p is page)

    def _viewer_titled(self, text: str):
        self._viewer_title = text or ""
        self._tab_changed()

    def _monitor_titled(self, text: str):
        self._monitor_title = ("" if text == "Witness Communications Monitor"
                               else text or "")
        self._tab_changed()

    def _tab_changed(self):
        # on_title can fire while the panels are still being built.
        if not getattr(self, "pages", None):
            return
        try:
            key = self.current_tab()
        except StopIteration:
            return
        suffix = TAB_TITLES[key]
        if key == "viewer" and self._viewer_title:
            suffix = self._viewer_title
        elif key == "monitor" and self._monitor_title:
            suffix = self._monitor_title
        self.setWindowTitle(f"{APP_NAME} - {suffix}")

    def shutdown(self):
        """Shut all three panels down (monitor, HITL, viewer); idempotent."""
        if self._closed:
            return
        self._closed = True
        try:
            self.monitor.shutdown()          # stops mirroring and the feed
        finally:
            try:
                self.hitl.shutdown()         # stops any run, sends HITL EXIT
            finally:
                shutdown_panel(self.viewer)  # stops CSV loader threads

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


def run(args) -> int:
    """Open the window for parsed witnessos.py arguments; run the event loop."""
    app = ensure_app()
    import stall_watchdog   # Horizon: logs where the UI freezes (Horizon/logs/ui_stalls.log)
    stall_watchdog.install(Path(wos_paths.HORIZON_DIR) / "logs")
    window = WitnessOS(csv_files=args.csv_files, start_tab=args.tab,
                       monitor_options={"start_feed": not args.no_feed,
                                        "remote_feed": getattr(args, "remote_feed", "auto"),
                                        "remote_feed_port": getattr(args, "remote_feed_port", None),
                                        "auto_connect": not args.no_auto_connect})
    window.show()
    return app.exec()
