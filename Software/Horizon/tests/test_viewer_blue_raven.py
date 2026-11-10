"""The Qt viewer on real Blue Raven logs (LR ~100 columns, HR 192k rows x 19).

Skipped when the sample files are not on this machine (they are not in the
repository). Also measures how long the GUI thread is blocked by each step
(a 5 ms QTimer stall probe) and prints the numbers.
"""

import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from viewer_qt_support import RAVEN_HR, RAVEN_LR, StallProbe, ViewerCase, pump  # noqa: E402

import qt_common  # noqa: E402
import viewer  # noqa: E402

# A generous ceiling that still catches a real freeze; the measured values
# are printed (see the report) and are far lower.
STALL_LIMIT_S = 0.5


@unittest.skipUnless(RAVEN_HR.exists() and RAVEN_LR.exists(), "Blue Raven sample logs absent")
class BlueRavenViewerTests(ViewerCase):
    def test_load_both_files_lazily(self):
        panel = self.make(1200, 900)
        panel.add_files([RAVEN_LR, RAVEN_HR])
        self.assertTrue(panel.wait_for_loads(120))
        self.assertEqual(panel.messages, [])
        lr, hr = panel.datasets.values()
        self.assertEqual(hr.row_count, 192000)
        self.assertEqual(len(hr.numeric_columns), 19)
        self.assertGreater(len(lr.numeric_columns), 90)
        self.assertEqual(hr.time_column, "Flight_Time_(s)")
        self.plots(panel)
        # ~100 LR columns: one placeholder each, a handful of real plots.
        self.assertGreater(len(panel.slots), 90)
        self.assertLess(len(panel.live_slots()), 15)
        panel.plot_dataset(list(panel.datasets)[1])
        panel.rebuild_graphs()
        panel.realize_visible_now()
        self.assertLessEqual(len(panel.live_slots()), viewer.MAX_LIVE_PLOTS)

    def test_gui_stalls_on_the_hr_file(self):
        panel = self.make(1200, 900)
        stalls = {}
        with StallProbe() as probe:
            panel.add_files([RAVEN_HR])
            qt_common.process_events_until(lambda: not panel.loading, 120)
            pump(0.2)
        stalls["load (background) + first summary"] = probe.max_gap
        self.assertEqual(len(panel.datasets), 1)

        with StallProbe() as probe:
            panel.notebook.setCurrentWidget(panel.plot_tab)
            qt_common.process_events_until(
                lambda: not panel._realize_timer.isActive() and panel.live_slots(), 10)
            pump(0.3)
        stalls["show plots (lazy graph creation)"] = probe.max_gap
        live = panel.live_slots()
        self.assertTrue(live)

        with StallProbe() as probe:
            panel.set_x_view((100.0, 100.5))
            pump(0.3)
        stalls["zoom in to 0.5 s"] = probe.max_gap

        with StallProbe() as probe:
            panel.reset_zoom()
            pump(0.3)
        stalls["reset zoom (whole 192k rows)"] = probe.max_gap

        panel.set_x_view((100.0, 100.5))
        pump(0.1)
        dataset_id = next(iter(panel.datasets))
        panel.select_dataset(dataset_id)
        curve = live[0].curves[0][1]
        x_before = curve.xData
        worst = 0.0
        for value in (0.1, 0.2, 1.0, -3.0, 12.5, 0.0):
            with StallProbe() as probe:
                panel.offset_spin.setValue(value)
                pump(0.1)
            worst = max(worst, probe.max_gap)
        stalls["offset change while zoomed (worst of 6)"] = worst
        self.assertIs(curve.xData, x_before)

        with StallProbe() as probe:
            panel.set_time_axis(dataset_id, unit="ms")
            pump(0.3)
        stalls["time unit change (x rebuilt)"] = probe.max_gap
        panel.set_time_axis(dataset_id, unit="s")

        with StallProbe() as probe:
            for i in range(40):
                panel.show_hover(0, 100.0 + i * 0.01)
                pump()
        stalls["hover x40"] = probe.max_gap

        panel.reset_zoom()
        ref = (dataset_id, "Accel_X")
        with StallProbe() as probe:
            window = panel.open_spectrogram(ref)
            window.nfft_combo.setCurrentText("4096")
            self.assertTrue(window.wait(60))
            pump(0.3)
        stalls["spectrogram whole trace (FFT 4096)"] = probe.max_gap
        self.assertIsNotNone(window.analysis, window.status_label.text())
        window.close()

        print("\nGUI-thread stalls on the HR file (192k rows x 19 cols):")
        for name, seconds in stalls.items():
            print(f"  {name:45s} {seconds * 1000:7.1f} ms")
        for name, seconds in stalls.items():
            self.assertLess(seconds, STALL_LIMIT_S, name)


if __name__ == "__main__":
    unittest.main()
