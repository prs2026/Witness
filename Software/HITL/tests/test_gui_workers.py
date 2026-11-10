"""HITL GUI: vibration picker, worker-thread prepare, batched log."""

import json
import os
import shutil
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import hitl_gui  # noqa: E402
import trajectory_csv as tc  # noqa: E402

TRAJ = os.path.join(os.path.dirname(HERE), "trajectories")
EAYF = os.path.join(TRAJ, "eayftrajectory.csv")
OTHER = os.path.join(TRAJ, "a3241569-trajectory.csv")


def synthesis_available() -> bool:
    try:
        import imu_synth
        return not imu_synth.available()
    except ImportError:
        return False


class VibrationHelperTests(unittest.TestCase):
    def test_profiles_and_display_round_trip(self):
        self.assertIn("eayftrajectory", hitl_gui.vibration_profiles())
        for value in ("", "none", "eayftrajectory"):
            self.assertEqual(hitl_gui.vibration_value(hitl_gui.vibration_display(value)),
                             value)
        self.assertEqual(hitl_gui.vibration_display(""), hitl_gui.VIB_AUTO)
        self.assertEqual(hitl_gui.vibration_display("NONE"), hitl_gui.VIB_NONE)
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        os.makedirs(os.path.join(tmp, "b", "x"))
        os.makedirs(os.path.join(tmp, "a"))
        open(os.path.join(tmp, "a", "profile.json"), "w").close()
        self.assertEqual(hitl_gui.vibration_profiles(tmp), ["a"])

    @unittest.skipUnless(synthesis_available(), "numpy/scipy/soundfile missing")
    def test_describe_vibration(self):
        self.assertEqual(hitl_gui.describe_vibration(EAYF, "", 1.0),
                         "Vibration: Auto -> profile 'eayftrajectory' x1")
        self.assertIn("no profile for this trajectory",
                      hitl_gui.describe_vibration(OTHER, "", 1.0))
        self.assertEqual(hitl_gui.describe_vibration(OTHER, "eayftrajectory", 0.5),
                         "Vibration: profile 'eayftrajectory' x0.5")
        self.assertIn("none", hitl_gui.describe_vibration(EAYF, "none", 1.0))
        self.assertIn("scale 0", hitl_gui.describe_vibration(EAYF, "", 0.0))
        self.assertIn("no profile.json", hitl_gui.describe_vibration(EAYF, "missing", 1.0))

    def test_settings_have_vibration_defaults(self):
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        settings = hitl_gui.load_settings(os.path.join(tmp, "absent.json"))
        self.assertEqual((settings["vibration_profile"], settings["vibration_scale"]),
                         ("", 1.0))


class GuiWorkerTests(unittest.TestCase):
    def setUp(self):
        try:
            import tkinter as tk
            self.root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        self.root.withdraw()
        self.tmp = tempfile.mkdtemp(prefix="hitl-gui-")
        self.settings_path = os.path.join(self.tmp, "s.json")
        self.switch = sys.getswitchinterval()   # before the first prediction
        self.app = hitl_gui.HitlApp(
            self.root, settings=hitl_gui.load_settings(self.settings_path),
            settings_path=self.settings_path, runs_dir=self.tmp)

    def tearDown(self):
        self.app.on_close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_load_csv_predicts_off_the_ui_thread_with_vibration_picker(self):
        app = self.app
        self.assertTrue(app.wait_idle(60))
        started = time.perf_counter()
        app.load_csv(EAYF)
        self.assertLess(time.perf_counter() - started, 0.5)   # did not block
        self.assertEqual(app.predict_label.cget("text"), "Predicting…")
        self.assertTrue(app._busy)
        self.assertTrue(app.busy_bar.winfo_manager())          # indicator shown
        self.assertTrue(app.wait_idle(60))
        self.assertFalse(app.busy_bar.winfo_manager())
        self.assertEqual(sys.getswitchinterval(), self.switch)  # restored
        self.assertIn("Boost", app.predict_label.cget("text"))
        values = app.vibration_picker.cget("values")
        self.assertEqual(list(values[:2]), [hitl_gui.VIB_AUTO, hitl_gui.VIB_NONE])
        self.assertIn("eayftrajectory", values)
        self.assertEqual(app.read_params().vibration_profile, "")
        if synthesis_available():
            self.assertIn("'eayftrajectory'", app.vibration_label.cget("text"))
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
        self.assertNotIn("CSV", app.predict_label.cget("text"))

    def test_start_sim_builds_the_run_on_a_worker_and_reports_vibration(self):
        from fake_witness import FakeWitness
        fake = FakeWitness().start()
        self.addCleanup(fake.close)
        app = self.app
        app.load_csv(OTHER)
        self.assertTrue(app.wait_idle(60))
        app.param_vars["imu_synthesis"].set(False)   # keep the test quick
        app.host_var.set("127.0.0.1")
        app.port_var.set(str(fake.port))
        app.toggle_connect()
        self.assertTrue(app.session.connected)
        started = time.perf_counter()
        app.start_sim()
        self.assertLess(time.perf_counter() - started, 0.5)
        self.assertTrue(app._preparing)
        self.assertIn("disabled", app.btn_start.state())
        self.assertIn("disabled", app.load_csv_btn.state())
        self.assertTrue(app.wait_idle(60))
        self.assertIsNotNone(app.session.run)
        self.assertTrue(app.session.run.running)
        self.assertIn("synthesis off", app.vibration_label.cget("text"))
        log = app.logbox.get("1.0", "end")
        self.assertIn("[csv]", log)
        self.assertIn("[imu]", log)
        app.stop_sim()
        app.session.wait_run(5)

    def test_log_lines_are_written_in_batches(self):
        app = self.app
        before = app.logbox.get("1.0", "end")
        for i in range(2000):
            app.append_log(f"line {i}")
        self.assertEqual(app.logbox.get("1.0", "end"), before)   # nothing yet
        app._flush_log()
        text = app.logbox.get("1.0", "end")
        self.assertIn("line 0\n", text)
        self.assertIn("line 1999\n", text)
        self.assertEqual(app._pending_log, [])


if __name__ == "__main__":
    unittest.main()
