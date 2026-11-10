"""witnessOS shell tests (Qt, offscreen).

The CSV viewer and Witness monitor tabs are Horizon's widgets. The shell
tests run them as lightweight stand-ins (unittest.mock.patch.dict on
sys.modules) that implement only the embedding contract; the integration
test at the bottom runs the real Horizon widgets once they are Qt.
"""

import os
import shutil
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import wos_paths  # noqa: E402,F401
from PyQt6 import QtWidgets  # noqa: E402

import witnessos  # noqa: E402
from qt_common import ensure_app, process_events_until  # noqa: E402

HITL_SETTINGS = {"host": "127.0.0.1", "port": 1,
                 "preset": "Nominal 10 g", "tolerance_s": 1.0}


class FakeViewer(QtWidgets.QWidget):
    """viewer.CsvViewerWidget contract."""

    def __init__(self, parent=None, files=(), *, on_title=None):
        super().__init__(parent)
        self.files = [Path(p) for p in files]
        self.on_title = on_title
        self.shut = False
        self.setMinimumSize(200, 150)
        if files and on_title:
            on_title(Path(files[0]).name)

    def open_files(self, paths):
        self.files = [Path(p) for p in paths]

    def add_files(self, paths):
        self.files += [Path(p) for p in paths]

    def close_all(self):
        self.files = []

    def shutdown(self):
        self.shut = True


class FakeMonitor(QtWidgets.QWidget):
    """main.MonitorPanel contract (the parts witnessOS uses)."""

    def __init__(self, parent=None, *, on_title=None, **options):
        super().__init__(parent)
        self.on_title = on_title
        self.options = options
        self.console = []
        self.fed = []
        self._closed = False

    def stream_feeder(self, source_name):
        def feed(data):
            self.fed.append((source_name, bytes(data)))
        return feed

    def console_write(self, line):
        self.console.append(line)

    def shutdown(self):
        self._closed = True


def stand_ins():
    return mock.patch.dict(sys.modules, {
        "viewer": types.SimpleNamespace(CsvViewerWidget=FakeViewer),
        "main": types.SimpleNamespace(MonitorPanel=FakeMonitor)})


class ShellTests(unittest.TestCase):
    def setUp(self):
        self.qapp = ensure_app()
        self.tmp = tempfile.mkdtemp(prefix="witnessos-")
        patcher = stand_ins()
        patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def make(self, **kwargs):
        kwargs.setdefault("hitl_settings", dict(HITL_SETTINGS))
        kwargs.setdefault("hitl_settings_path", os.path.join(self.tmp, "s.json"))
        kwargs.setdefault("hitl_runs_dir", self.tmp)
        kwargs.setdefault("monitor_options", {"auto_connect": False, "feed_port": 0})
        app = witnessos.WitnessOS(**kwargs)
        self.addCleanup(app.deleteLater)
        self.addCleanup(app.shutdown)
        return app

    def test_builds_all_tabs_and_titles(self):
        csv = Path(self.tmp) / "flight.csv"
        app = self.make(csv_files=[csv, Path(self.tmp) / "second.csv"])
        self.assertEqual([app.tabs.tabText(i) for i in range(app.tabs.count())],
                         ["HITL bench", "CSV viewer", "Witness monitor"])
        self.assertEqual((app.minimumWidth(), app.minimumHeight()), (420, 300))
        self.assertGreaterEqual(app.width(), 420)
        # CSV files open the viewer tab; its title shows in the window title.
        self.assertEqual(app.current_tab(), "viewer")
        self.assertEqual(app.viewer.files[0], csv)
        self.assertEqual(app.windowTitle(), "witnessOS - flight.csv")
        app.select("hitl")
        self.assertEqual(app.windowTitle(), "witnessOS - HITL bench")
        app.select("monitor")
        self.assertEqual(app.windowTitle(), "witnessOS - Witness monitor")
        app.monitor.monitor.on_title("Witness Communications Monitor")
        self.assertEqual(app.windowTitle(), "witnessOS - Witness monitor")
        app.monitor.monitor.on_title("Witness - COM5")
        self.assertEqual(app.windowTitle(), "witnessOS - Witness - COM5")
        self.assertIsInstance(app.hitl, QtWidgets.QWidget)
        self.assertEqual(app.monitor.monitor.options,
                         {"auto_connect": False, "feed_port": 0})

    def test_start_tab_and_alias(self):
        self.assertEqual(self.make(start_tab="relay").current_tab(), "monitor")
        self.assertEqual(self.make().current_tab(), "hitl")

    def test_monitor_mirrors_the_hitl_connection(self):
        app = self.make()
        monitor = app.monitor.monitor
        self.assertTrue(app.monitor.mirroring)
        self.assertIs(app.hitl.session.rx_tap, app.monitor._tap)
        app.hitl.session.rx_tap(b"\xff\x00\x01")
        self.assertEqual(monitor.fed, [("HITL", b"\xff\x00\x01")])
        self.assertIn("MIRROR", monitor.console[0])
        app.monitor.mirror_check.setChecked(False)      # user unticks
        self.assertFalse(app.monitor.mirroring)
        self.assertIsNone(app.hitl.session.rx_tap)
        app.monitor.set_mirroring(True)
        self.assertTrue(app.monitor.mirroring)

    def test_small_window_scrolls_instead_of_clipping(self):
        app = self.make()
        app.resize(420, 300)
        app.show()
        page = app.pages["hitl"]
        process_events_until(lambda: page.viewport().width() > 0, 2)
        hint = app.hitl.minimumSizeHint()
        self.assertGreaterEqual(app.hitl.height(), min(hint.height(), 10_000))
        if hint.height() > page.viewport().height():
            self.assertTrue(page.verticalScrollBar().isVisible())
        app.resize(1300, 900)
        process_events_until(lambda: app.hitl.width() >= page.viewport().width(), 2)
        self.assertGreaterEqual(app.hitl.width(), page.viewport().width())

    def test_close_shuts_every_panel_down(self):
        app = self.make()
        app.show()
        session = app.hitl.session
        viewer, monitor = app.viewer, app.monitor.monitor
        app.close()
        self.assertTrue(monitor._closed)
        self.assertTrue(viewer.shut)
        self.assertTrue(app.hitl._closing)
        self.assertIsNone(session.rx_tap)
        app.shutdown()            # idempotent

    def test_command_line(self):
        args = witnessos.build_parser().parse_args(
            ["a.csv", "b.csv", "--tab", "relay", "--no-feed", "--no-auto-connect"])
        self.assertEqual(args.csv_files, [Path("a.csv"), Path("b.csv")])
        self.assertEqual(args.tab, "relay")
        self.assertTrue(args.no_feed and args.no_auto_connect)

    def test_main_builds_the_window(self):
        captured = {}

        def fake_exec():
            win = next(w for w in QtWidgets.QApplication.topLevelWidgets()
                       if isinstance(w, witnessos.WitnessOS) and w.isVisible())
            captured["win"] = win
            win.close()
            return 0

        # main() uses the real hitl_settings.json; building the window only
        # reads it (settings are written on user actions).
        with mock.patch.object(QtWidgets.QApplication, "exec", staticmethod(fake_exec)):
            code = witnessos.main(["--tab", "monitor", "--no-feed", "--no-auto-connect"])
        self.assertEqual(code, 0)
        win = captured["win"]
        self.addCleanup(win.deleteLater)
        self.assertEqual(win.current_tab(), "monitor")
        self.assertEqual(win.monitor.monitor.options,
                         {"start_feed": False, "remote_feed": "auto", "remote_feed_port": None,
                          "auto_connect": False})
        self.assertTrue(win._closed)


def real_horizon_widgets() -> str:
    """'' when Horizon's viewer/monitor are the Qt widgets, else why not."""
    try:
        import main
        import viewer
    except Exception as exc:  # noqa: BLE001
        return f"Horizon import failed: {exc}"
    if not isinstance(getattr(viewer, "CsvViewerWidget", None), type) or \
            not issubclass(viewer.CsvViewerWidget, QtWidgets.QWidget):
        return "viewer.CsvViewerWidget is not a QWidget yet"
    if not issubclass(getattr(main, "MonitorPanel", object), QtWidgets.QWidget):
        return "main.MonitorPanel is not a QWidget yet"
    return ""


class IntegrationTests(unittest.TestCase):
    """The real Horizon widgets (needs the Qt ports of viewer.py / main.py)."""

    def setUp(self):
        why = real_horizon_widgets()
        if why:
            self.skipTest(why)
        self.qapp = ensure_app()
        self.tmp = tempfile.mkdtemp(prefix="witnessos-")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_app_builds_all_tabs_loads_csv_mirrors_and_closes(self):
        csv_path = Path(self.tmp) / "flight.csv"
        with csv_path.open("w", newline="") as f:
            f.write("time_seconds,uptime,barometric_altitude,packet_type,status\n")
            for i in range(2000):
                f.write(f"{i * 0.01:.2f},{i * 10},{(i % 500) * 1.5:.1f},state,"
                        f"{0x4201 if i > 100 else 0x4200}\n")
        app = witnessos.WitnessOS(
            csv_files=[csv_path], hitl_settings=dict(HITL_SETTINGS),
            hitl_settings_path=os.path.join(self.tmp, "s.json"),
            hitl_runs_dir=self.tmp,
            monitor_options={"auto_connect": False, "feed_port": 0})
        self.addCleanup(app.deleteLater)
        app.show()
        self.assertEqual(app.current_tab(), "viewer")
        for size in ((1200, 800), (500, 350)):
            app.resize(*size)
            for key in witnessos.TABS:
                app.select(key)
                process_events_until(lambda: False, 0.1)
                self.assertTrue(app.windowTitle().startswith("witnessOS - "))
        self.assertTrue(app.monitor.mirroring)
        self.assertIs(app.hitl.session.rx_tap, app.monitor._tap)
        frame = bytes([0xFF]) + (0x0001).to_bytes(2, "big") \
            + (4242).to_bytes(4, "big") + b"\n"
        import socket
        from witness_link import decode_record
        monitor = app.monitor.monitor
        client = socket.create_connection(("127.0.0.1", monitor.publisher.port))
        client.settimeout(3)
        try:
            app.hitl.session.rx_tap(frame)
            app.select("monitor")
            cell = monitor.values["packet.heartbeat.uptime"]
            self.assertTrue(process_events_until(lambda: cell.get() == "4242", 5))
            record = decode_record(client.makefile("rb").readline())
            self.assertEqual(record["source"], "HITL")
            self.assertEqual(record["packet"]["uptime"], 4242)
        finally:
            client.close()
        app.monitor.set_mirroring(False)
        self.assertIsNone(app.hitl.session.rx_tap)
        app.close()
        self.assertTrue(app._closed)


if __name__ == "__main__":
    unittest.main()
