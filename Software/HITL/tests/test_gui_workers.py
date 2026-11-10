"""HITL vibration helpers (the GUI tests are in witnessOS/tests/test_hitl_tab.py)."""

import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import hitl_gui  # noqa: E402

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


if __name__ == "__main__":
    unittest.main()
