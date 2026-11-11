"""witnessOS HITL bench tab (HitlWidget): workers, plots, settings."""

import json
import os
import shutil
import sys
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import wos_paths  # noqa: E402,F401
sys.path.insert(0, os.path.join(wos_paths.HITL_DIR, "tests"))   # fake_witness

from PyQt6 import QtWidgets  # noqa: E402

import hitl_gui  # noqa: E402
from qt_common import ensure_app, process_events_until  # noqa: E402
from wos_hitl_tab import HitlWidget  # noqa: E402

TRAJ = os.path.join(wos_paths.HITL_DIR, "trajectories")
EAYF = os.path.join(TRAJ, "eayftrajectory.csv")
OTHER = os.path.join(TRAJ, "a3241569-trajectory.csv")
BUNDLED = OTHER


def synthesis_available() -> bool:
    try:
        import imu_synth
        return not imu_synth.available()
    except ImportError:
        return False


class GuiWorkerTests(unittest.TestCase):
    def setUp(self):
        try:
            self.qapp = ensure_app()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Qt unavailable: {exc}")
        self.tmp = tempfile.mkdtemp(prefix="hitl-gui-")
        self.settings_path = os.path.join(self.tmp, "s.json")
        self.switch = sys.getswitchinterval()   # before the first prediction
        self.app = HitlWidget(
            settings=hitl_gui.load_settings(self.settings_path),
            settings_path=self.settings_path, runs_dir=self.tmp)

    def tearDown(self):
        self.app.shutdown()
        self.app.deleteLater()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_load_csv_predicts_off_the_ui_thread_with_vibration_picker(self):
        app = self.app
        self.assertTrue(app.wait_idle(60))
        started = time.perf_counter()
        app.load_csv(EAYF)
        self.assertLess(time.perf_counter() - started, 0.5)   # did not block
        self.assertEqual(app.predict_label.text(), "Predicting…")
        self.assertTrue(app.busy)
        self.assertFalse(app.busy_bar.isHidden())               # indicator shown
        self.assertTrue(app.wait_idle(60))
        self.assertTrue(app.busy_bar.isHidden())
        self.assertEqual(sys.getswitchinterval(), self.switch)  # restored
        self.assertIn("Boost", app.predict_label.text())
        picker = app.vibration_picker
        values = [picker.itemText(i) for i in range(picker.count())]
        self.assertEqual(values[:2], [hitl_gui.VIB_AUTO, hitl_gui.VIB_NONE])
        self.assertIn("eayftrajectory", values)
        self.assertEqual(app.read_params().vibration_profile, "")
        if synthesis_available():
            self.assertIn("'eayftrajectory'", app.vibration_label.text())
        # Picking a profile saves it and goes into the run parameters.
        app.param_vars["vibration_profile"].set(hitl_gui.VIB_NONE)
        app.param_vars["vibration_scale"].set("0.5")
        app._vibration_changed()
        self.assertTrue(app.wait_idle(60))
        params = app.read_params()
        self.assertEqual((params.vibration_profile, params.vibration_scale), ("none", 0.5))
        with open(self.settings_path) as f:
            saved = json.load(f)
        self.assertEqual((saved["vibration_profile"], saved["vibration_scale"]), ("none", 0.5))
        # A later CSV source starts from the saved choice.
        app.load_preset()
        app.load_csv(OTHER)
        self.assertEqual(app.param_vars["vibration_profile"].get(), hitl_gui.VIB_NONE)
        self.assertTrue(app.wait_idle(60))

    def test_stale_prediction_is_ignored(self):
        app = self.app
        app.load_csv(EAYF)
        app.load_preset()              # a newer prediction supersedes the CSV one
        self.assertTrue(app.wait_idle(60))
        self.assertNotIn("CSV", app.predict_label.text())

    def test_start_sim_builds_the_run_on_a_worker_and_reports_vibration(self):
        from fake_witness import FakeWitness
        fake = FakeWitness().start()
        self.addCleanup(fake.close)
        app = self.app
        app.load_csv(OTHER)
        self.assertTrue(app.wait_idle(60))
        app.param_vars["imu_synthesis"].set(False)   # keep the test quick
        app.host_edit.setText("127.0.0.1")
        app.port_edit.setText(str(fake.port))
        app.toggle_connect()                          # connects on a worker
        self.assertTrue(app.wait_idle(10))
        self.assertTrue(app.session.connected)
        app.poll()
        self.assertEqual(app.connect_btn.text(), "Disconnect")
        started = time.perf_counter()
        app.start_sim()
        self.assertLess(time.perf_counter() - started, 0.5)
        self.assertTrue(app._preparing)
        self.assertFalse(app.btn_start.isEnabled())
        self.assertFalse(app.load_csv_btn.isEnabled())
        self.assertTrue(app.wait_idle(60))
        self.assertIsNotNone(app.session.run)
        self.assertTrue(app.session.run.running)
        self.assertTrue(app.btn_stop.isEnabled())
        self.assertIn("synthesis off", app.vibration_label.text())
        log = app.log_text()
        self.assertIn("[csv]", log)
        self.assertIn("[imu]", log)
        # Live data reaches the pyqtgraph curves through the throttle.
        self.assertTrue(process_events_until(
            lambda: len(app.session.run.plot_true) > 5, 10))
        app.redraw_plot()
        x, _y = app.curves["plot_true"].getData()
        self.assertGreater(len(x), 5)
        self.assertFalse(app.plot_att.isHidden())     # CSV run: attitude panel
        app.stop_sim()
        app.session.wait_run(5)

    def test_connect_failure_is_logged_without_blocking(self):
        app = self.app
        app.host_edit.setText("127.0.0.1")
        app.port_edit.setText("1")
        app.toggle_connect()
        self.assertTrue(app.wait_idle(10))
        self.assertFalse(app.session.connected)
        self.assertIn("connect to 127.0.0.1:1 failed", app.log_text())

    def test_log_lines_are_written_in_batches(self):
        app = self.app
        before = app.log_text()
        for i in range(2000):
            app.append_log(f"line {i}")
        self.assertEqual(app.log_text(), before)   # nothing yet
        app._flush_log()
        text = app.log_text()
        self.assertIn("line 0\n", text)
        self.assertTrue(text.endswith("line 1999"))
        self.assertEqual(app._pending_log, [])

    def test_tool_and_comms_logs_are_separate_and_timestamped(self):
        import time as _time
        import hitl_session as hs_mod
        app = self.app
        s = app.session
        s._event("text", "I (123) uart: GPS RX: baud=115201")
        s._event("log", "[tool] HackRF streaming")
        stamped = hs_mod.Stamped("I (456) flight_state: pad idle -> boost")
        stamped.t = _time.time() + 2.5
        s.events.put(("text", stamped))
        app.poll()
        tool, comms = app.logbox.toPlainText(), app.commsbox.toPlainText()
        self.assertIn("[tool] HackRF streaming", tool)
        self.assertNotIn("uart: GPS RX", tool)
        self.assertIn("uart: GPS RX", comms)
        second = comms.splitlines()[-1]
        self.assertRegex(second, r"^\d\d:\d\d:\d\d\.\d{3} +\+ *2\.\d{3}  I \(456\)")
        # A run is in progress while the session reports an activity.
        s.set_activity("FC logs: downloading P0000001.BIN (1/4, 0.2 MB)")
        app.poll()
        self.assertTrue(app.activity_label.isVisibleTo(app))
        self.assertIn("FC logs: downloading", app.activity_label.text())
        self.assertTrue(app._running())
        s.set_activity("")
        app.poll()
        self.assertFalse(app.activity_label.isVisibleTo(app))

    def test_fc_log_settings_are_saved_and_applied(self):
        app = self.app
        app.pi_user_edit.setText("bench")
        app.pi_key_edit.setText("C:/keys/id")
        app.fc_box.setChecked(False)          # toggled -> saved
        cfg = app.session.fc_log_config
        self.assertFalse(cfg["enabled"])
        self.assertEqual((cfg["pi_user"], cfg["pi_key"]), ("bench", "C:/keys/id"))
        with open(self.settings_path) as f:
            saved = json.load(f)
        self.assertFalse(saved["fetch_fc_logs"])
        self.assertEqual(saved["pi_user"], "bench")


class QtGuiTests(unittest.TestCase):
    def setUp(self):
        self.app = ensure_app()
        self.tmp = tempfile.mkdtemp(prefix="hitl-gui-")
        self.settings = {"host": "127.0.0.1", "port": 1,
                         "preset": "Nominal 10 g", "tolerance_s": 1.0}

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_embedded_widget_does_not_own_the_window(self):
        host = QtWidgets.QMainWindow()
        host.setWindowTitle("host")
        widget = HitlWidget(
            settings=dict(self.settings),
            settings_path=os.path.join(self.tmp, "s.json"), runs_dir=self.tmp)
        host.setCentralWidget(widget)
        host.resize(900, 600)
        host.show()
        self.app.processEvents()
        self.assertEqual(host.windowTitle(), "host")
        widget.show_banner("test warning " * 20)
        self.assertFalse(widget.banner.isHidden())
        widget.hide_banner()
        self.assertTrue(widget.banner.isHidden())
        widget.redraw_plot()
        self.assertTrue(widget.wait_idle(30))
        widget.shutdown()       # embedded: shuts down, the host survives
        self.assertTrue(host.isVisible())
        host.close()
        host.deleteLater()

    def test_build_gui_presets_csv_and_attitude_plot(self):
        app = HitlWidget(
            settings=dict(self.settings),
            settings_path=os.path.join(self.tmp, "s.json"), runs_dir=self.tmp)
        app.resize(1200, 800)
        app.show()
        app.preset_combo.setCurrentText("Pad abort - no launch")
        app.load_preset()
        self.assertFalse(app.read_params().ignite)
        app.load_csv(BUNDLED)
        params = app.read_params()
        self.assertEqual(params.csv_path, BUNDLED)
        self.assertEqual(params.after_apogee_attitude, "hold")
        self.assertTrue(app.wait_idle(60))
        self.assertTrue(app.table_rows())          # expected transitions shown
        # Plot path for an attitude run (not started, nothing sent).
        run = app.session.prepare_run(params)
        run.sim_ms = 60_000
        run.plot_true.extend([(0.0, 100.0), (30.0, 900.0), (60.0, 1500.0)])
        run.plot_tilt_w.append((50.0, 10.0))
        run.plot_tilt_err.append((50.0, 0.5))
        app.session.run = run
        app.redraw_plot()
        self.assertFalse(app.plot_att.isHidden())
        x, _y = app.curves["plot_true"].getData()
        self.assertEqual(len(x), 3)
        tx, _ty = app.curves["plot_tilt_truth"].getData()
        self.assertTrue(len(tx) > 0 and tx[-1] <= 60.0)   # truth up to sim time
        run.plot_true.append((61.0, 1510.0))              # incremental append
        app.redraw_plot()
        self.assertEqual(len(app.curves["plot_true"].getData()[0]), 4)
        app.session.run = None
        app.preset_combo.setCurrentText("Nominal 10 g")
        app.load_preset()
        app.poll()
        app.redraw_plot()
        self.assertTrue(app.plot_att.isHidden())
        self.assertEqual(len(app.curves["plot_true"].getData()[0] or []), 0)
        self.assertTrue(app.wait_idle(30))
        app.shutdown()
        self.assertTrue(app._closing)
        app.deleteLater()



if __name__ == "__main__":
    unittest.main()
