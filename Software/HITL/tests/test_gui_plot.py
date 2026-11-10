import os
import sys
import unittest

HITL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HITL_DIR)

import hitl_gui  # noqa: E402
import hitl_session as hs  # noqa: E402


class PlotTickTests(unittest.TestCase):
    def test_ticks_cover_range(self):
        ticks = hitl_gui._nice_ticks(-3.2, 47.9, 6)
        self.assertLessEqual(ticks[0], -3.2)
        self.assertGreaterEqual(ticks[-1], 47.9)
        self.assertEqual(ticks[1] - ticks[0], 10)

    def test_labels(self):
        self.assertEqual(hitl_gui._tick_label(1500.0, 500), "1500")
        self.assertEqual(hitl_gui._tick_label(0.25, 0.05), "0.25")
        self.assertEqual(hitl_gui._tick_label(1e-12, 0.5), "0")

    def test_degenerate(self):
        self.assertEqual(len(hitl_gui._nice_ticks(3, 3)), 2)


class RxTapTests(unittest.TestCase):
    def test_session_has_tap_hook(self):
        self.assertIsNone(hs.HitlSession(runs_dir=None).rx_tap)


if __name__ == "__main__":
    unittest.main()
