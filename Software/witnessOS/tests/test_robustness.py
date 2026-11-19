"""witnessOS: crash hook, link banner on every tab, serial ports opened quietly."""

import os
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import wos_paths  # noqa: E402,F401
from PyQt6 import QtCore, QtWidgets  # noqa: E402

import witnessos  # noqa: E402
import wos_app  # noqa: E402
from qt_common import ensure_app  # noqa: E402
from test_app import FakeMonitor, FakeViewer, HITL_SETTINGS  # noqa: E402


class BannerMonitor(FakeMonitor):
    linkBannerChanged = QtCore.pyqtSignal(str)


class Base(unittest.TestCase):
    hook = None

    def setUp(self):
        ensure_app()
        self.tmp = tempfile.mkdtemp(prefix="witnessos-")
        main_stand_in = types.SimpleNamespace(MonitorPanel=BannerMonitor)
        if self.hook is not None:
            main_stand_in.install_global_excepthook = self.hook
        patcher = mock.patch.dict(sys.modules, {
            "viewer": types.SimpleNamespace(CsvViewerWidget=FakeViewer), "main": main_stand_in})
        patcher.start()
        self.addCleanup(patcher.stop)

    def make(self):
        app = witnessos.WitnessOS(
            hitl_settings=dict(HITL_SETTINGS), hitl_settings_path=os.path.join(self.tmp, "s.json"),
            hitl_runs_dir=self.tmp, monitor_options={"auto_connect": False, "feed_port": 0})
        self.addCleanup(app.deleteLater)
        self.addCleanup(app.shutdown)
        return app


class CrashHook(Base):
    def setUp(self):
        self.sinks = []
        Base.hook = staticmethod(lambda sink=None: self.sinks.append(sink) or True)
        self.addCleanup(setattr, Base, "hook", None)
        super().setUp()

    def test_hook_is_installed_before_the_tabs_exist_and_reports_to_the_status_bar(self):
        app = self.make()
        self.assertTrue(app.crash_hook_installed)
        self.assertEqual(len(self.sinks), 1)
        # An exception on a worker thread reaches the status bar through the sink.
        worker = threading.Thread(target=self.sinks[0], args=("UNCAUGHT thread x: ValueError: boom",))
        worker.start()
        worker.join()
        for _ in range(50):
            QtWidgets.QApplication.processEvents()
            if app.statusBar().currentMessage():
                break
            QtCore.QThread.msleep(10)
        self.assertIn("boom", app.statusBar().currentMessage())

    def test_no_hook_offered_is_not_an_error(self):
        with mock.patch.dict(sys.modules, {"main": types.SimpleNamespace(MonitorPanel=BannerMonitor)}):
            self.assertFalse(wos_app.install_crash_hook())


class RealHook(unittest.TestCase):
    def test_install_replaces_the_default_excepthooks(self):
        import main as horizon_main
        saved = (sys.excepthook, threading.excepthook, sys.unraisablehook, horizon_main._hook_installed,
                 list(horizon_main._exception_sinks))
        self.addCleanup(lambda: (setattr(sys, "excepthook", saved[0]),
                                 setattr(threading, "excepthook", saved[1]),
                                 setattr(sys, "unraisablehook", saved[2]),
                                 setattr(horizon_main, "_hook_installed", saved[3]),
                                 horizon_main._exception_sinks.__init__(saved[4])))
        horizon_main._hook_installed = False
        sys.excepthook = sys.__excepthook__
        self.assertTrue(wos_app.install_crash_hook())
        self.assertIsNot(sys.excepthook, sys.__excepthook__)
        self.assertIsNot(threading.excepthook, threading.__excepthook__)


class LinkBanner(Base):
    def test_banner_shows_on_other_tabs_and_not_twice_on_the_monitor_tab(self):
        app = self.make()
        monitor = app.monitor.monitor
        app.select("hitl")
        self.assertFalse(app.link_banner.isVisibleTo(app))
        monitor.linkBannerChanged.emit("LINK LOST - no data from COM5")
        self.assertTrue(app.link_banner.isVisibleTo(app))
        self.assertIn("LINK LOST", app.link_banner.text())
        app.select("viewer")
        self.assertTrue(app.link_banner.isVisibleTo(app))
        app.select("monitor")                     # the monitor shows its own
        self.assertFalse(app.link_banner.isVisibleTo(app))
        app.select("hitl")
        self.assertTrue(app.link_banner.isVisibleTo(app))
        monitor.linkBannerChanged.emit("")
        self.assertFalse(app.link_banner.isVisibleTo(app))

    def test_witness_resetting_has_its_own_colour(self):
        app = self.make()
        app.select("hitl")
        app.monitor.monitor.linkBannerChanged.emit("WITNESS RESETTING - back in a few seconds")
        self.assertIn("#ff8c00", app.link_banner.styleSheet())
        app.monitor.monitor.linkBannerChanged.emit("LINK LOST")
        self.assertIn("#c00000", app.link_banner.styleSheet())


class SerialOpenedQuietly(unittest.TestCase):
    def test_auto_connect_open_drops_dtr_and_rts_before_the_port_opens(self):
        import serial_discovery
        order = []

        class FakeSerial:
            def __setattr__(self, name, value):
                if name in ("dtr", "rts"):
                    order.append((name, value))
                object.__setattr__(self, name, value)

            def open(self):
                order.append(("open", None))

        with mock.patch("serial.Serial", FakeSerial):
            job = serial_discovery.SerialOpenJob("COM_FAKE", 115200)
            job.thread.join(2)
        stream, error = job.take()
        self.assertIsNone(error)
        self.assertIsInstance(stream, FakeSerial)
        self.assertLess(order.index(("dtr", False)), order.index(("open", None)))
        self.assertLess(order.index(("rts", False)), order.index(("open", None)))

    def test_monitor_connect_uses_the_quiet_opener(self):
        import monitor_core
        with mock.patch.object(monitor_core, "open_serial_quiet") as opener:
            monitor_core.MonitorCore._serial_opener("COM_FAKE", 115200)()
        opener.assert_called_once()


if __name__ == "__main__":
    unittest.main()
