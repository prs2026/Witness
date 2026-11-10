"""Viewer responsiveness: coalesced wheel zoom, thin pens on dense traces,
a bounded number of live plots, and the GUI stall recorder."""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from PyQt6 import QtCore, QtGui  # noqa: E402

from viewer_qt_support import ViewerCase, pump  # noqa: E402

import qt_common  # noqa: E402
import viewer  # noqa: E402
import viewer_data as vd  # noqa: E402


def noisy_csv(path, rows=20000, columns=3, rate=960.0):
    rng = np.random.default_rng(1)
    t = np.arange(rows) / rate
    data = np.column_stack([t] + [rng.normal(0, 1, rows) for _ in range(columns)])
    header = "time_s," + ",".join(f"accel_{i}" for i in range(columns))
    np.savetxt(path, data, delimiter=",", header=header, comments="", fmt="%.6f")
    return path


class WheelAndPens(ViewerCase):
    def _panel(self, rows=20000, columns=3):
        panel = self.make(1000, 700)
        panel.add_files([noisy_csv(Path(self.tmp) / "imu.csv", rows, columns)])
        self.assertTrue(panel.wait_for_loads(60))
        self.plots(panel)
        return panel

    def test_wheel_burst_is_applied_once(self):
        panel = self._panel()
        calls = []
        original = panel.set_x_view
        panel.set_x_view = lambda view: (calls.append(view), original(view))[1]
        full = panel.current_x_range(full=True)
        center = (full[0] + full[1]) / 2
        for _ in range(50):                           # a fast spin / touchpad burst
            panel.queue_wheel(zoom_steps=12 / 120.0, center=center)   # small smooth-scroll deltas
        self.assertEqual(calls, [])                   # nothing redrawn per event
        panel.flush_wheel()
        self.assertEqual(len(calls), 1)               # one redraw for the whole burst
        span = panel.x_view[1] - panel.x_view[0]
        expected = (full[1] - full[0]) * vd.ZOOM_FACTOR ** 5    # 50 x 0.1 notch = 5 notches
        self.assertAlmostEqual(span, expected, delta=expected * 1e-6)
        panel.queue_wheel(pan_steps=1.0)
        panel.flush_wheel()
        self.assertAlmostEqual(panel.x_view[1] - panel.x_view[0], span, delta=span * 1e-6)
        panel.queue_wheel(zoom_steps=-1000.0, center=center)    # far out: back to the full view
        panel.flush_wheel()
        self.assertIsNone(panel.x_view)

    def test_redraw_waits_until_input_stops(self):
        panel = self._panel()
        full = panel.current_x_range(full=True)
        center = (full[0] + full[1]) / 2
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:             # wheel keeps turning for 0.5 s
            panel.queue_wheel(zoom_steps=0.25, center=center)
            pump(0.03)
            self.assertIsNone(panel.x_view)            # no redraw while it moves
        pump(viewer.SETTLE_MS / 1000 + 0.15)
        self.assertIsNotNone(panel.x_view)             # applied once it stopped

    def test_drag_pan_applies_on_pause_and_release(self):
        panel = self._panel()
        panel.set_x_view((5.0, 7.0))
        panel.drag_pan(6.0, 5.5, finished=False)
        panel.drag_pan(6.0, 5.0, finished=False)
        self.assertEqual(panel.x_view, (5.0, 7.0))     # nothing while the mouse moves
        panel.drag_pan(6.0, 5.0, finished=True)
        self.assertAlmostEqual(panel.x_view[0], 6.0)   # dragged 1 s left -> view 1 s later
        self.assertAlmostEqual(panel.x_view[1], 8.0)
        self.assertIsNone(panel._drag_pan)

    def test_page_scroll_builds_graphs_after_scrolling_stops(self):
        panel = self._panel(rows=2000, columns=30)
        bar = panel.graph_scroll.verticalScrollBar()
        bar.setValue(bar.maximum())
        pump(0.02)
        self.assertTrue(panel._scroll_settle.pending())
        pump(viewer.SETTLE_MS / 1000 + 0.3)
        bottom = panel.slots[-1]
        self.assertIsNotNone(bottom.plot)

    def test_ctrl_wheel_event_is_queued(self):
        panel = self._panel()
        vb = panel.live_slots()[0].vb

        class Wheel:
            def modifiers(self):
                return QtCore.Qt.KeyboardModifier.ControlModifier

            def delta(self):
                return 120

            def pos(self):
                return vb.boundingRect().center()

            def accept(self):
                pass

            def ignore(self):
                pass

        for _ in range(3):
            vb.wheelEvent(Wheel())
        self.assertIsNone(panel.x_view)
        self.assertAlmostEqual(panel._wheel_zoom, 3.0)
        panel.flush_wheel()
        self.assertIsNotNone(panel.x_view)

    def test_dense_traces_use_thin_pens(self):
        panel = self._panel()
        curve = panel.live_slots()[0].curves[0][1]
        self.assertEqual(curve.opts["pen"].widthF(), 1.0)
        small = self.make(800, 500)
        small.add_files([noisy_csv(Path(self.tmp) / "small.csv", rows=500)])
        self.assertTrue(small.wait_for_loads(60))
        self.plots(small)
        self.assertEqual(small.live_slots()[0].curves[0][1].opts["pen"].widthF(), 1.5)

    def test_live_plots_stay_bounded_while_scrolling(self):
        panel = self._panel(rows=3000, columns=40)
        bar = panel.graph_scroll.verticalScrollBar()
        for value in range(0, bar.maximum() + 1, max(1, bar.maximum() // 20)):
            bar.setValue(value)
            panel.realize_visible_now()
            self.assertLessEqual(len(panel.live_slots()), viewer.MAX_LIVE_PLOTS)


class TilingAndResize(ViewerCase):
    def test_graphs_tile_side_by_side(self):
        panel = self.make(1200, 800)
        panel.add_files([noisy_csv(Path(self.tmp) / "imu.csv", rows=2000, columns=6)])
        self.assertTrue(panel.wait_for_loads(60))
        self.plots(panel)
        grid = panel.graph_layout
        panel.set_columns(3)
        self.assertEqual(panel.columns_spin.value(), 3)
        positions = [grid.getItemPosition(grid.indexOf(slot))[:2] for slot in panel.slots]
        self.assertEqual(positions[:4], [(0, 0), (0, 1), (0, 2), (1, 0)])
        pump(0.1)
        first, second = panel.slots[0], panel.slots[1]
        self.assertEqual(first.y(), second.y())
        self.assertGreater(second.x(), first.x() + first.width() / 2)
        panel.realize_visible_now()
        self.assertTrue(panel.live_slots())
        panel.set_columns(1)
        positions = [grid.getItemPosition(grid.indexOf(slot))[:2] for slot in panel.slots]
        self.assertEqual(positions[:2], [(0, 0), (1, 0)])

    def test_resize_one_graph_and_its_row(self):
        panel = self.make(1200, 800)
        panel.add_files([noisy_csv(Path(self.tmp) / "imu.csv", rows=2000, columns=4)])
        self.assertTrue(panel.wait_for_loads(60))
        self.plots(panel)
        auto = panel.slots[0].height()
        panel.set_graph_height(1, 420)
        self.assertEqual(panel.slots[1].height(), 420)
        self.assertEqual(panel.slots[0].height(), auto)          # one column: only that graph
        panel.set_columns(2)
        self.assertEqual(panel.slots[0].height(), 420)           # tiled: the row lines up
        panel.set_graph_height(2, 50)
        self.assertEqual(panel.slots[2].height(), viewer.MIN_RESIZE_HEIGHT)
        panel.rebuild_graphs()                                   # survives a rebuild
        self.assertEqual(panel.slots[1].height(), 420)
        panel.reset_graph_heights()
        self.assertTrue(all(slot.height() == panel._graph_height() for slot in panel.slots))

    def test_grip_resizes_on_release_only(self):
        from PyQt6 import QtTest
        panel = self.make(1000, 800)
        panel.add_files([noisy_csv(Path(self.tmp) / "imu.csv", rows=2000, columns=2)])
        self.assertTrue(panel.wait_for_loads(60))
        self.plots(panel)
        slot = panel.slots[0]
        before = slot.height()
        grip = slot.grip
        self.assertEqual(grip.y(), slot.height() - viewer.GRIP_HEIGHT)
        QtTest.QTest.mousePress(grip, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(20, 3))
        grip.mouseMoveEvent(QtGui.QMouseEvent(
            QtCore.QEvent.Type.MouseMove, QtCore.QPointF(20, 83), grip.mapToGlobal(QtCore.QPointF(20, 83)),
            QtCore.Qt.MouseButton.LeftButton, QtCore.Qt.MouseButton.LeftButton,
            QtCore.Qt.KeyboardModifier.NoModifier))
        self.assertEqual(slot.height(), before)                  # only the guide moved
        QtTest.QTest.mouseRelease(grip, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(20, 83))
        self.assertEqual(slot.height(), before + 80)
        QtTest.QTest.mouseDClick(grip, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(20, 3))
        self.assertEqual(slot.height(), panel._graph_height())


class StallRecorder(unittest.TestCase):
    def test_records_where_the_gui_thread_was_stuck(self):
        import stall_watchdog
        qt_common.ensure_app()
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "ui_stalls.log"
            dog = stall_watchdog.StallWatchdog(log, threshold_s=0.3)
            try:
                pump(0.2)

                def stuck_in_a_slow_redraw():
                    time.sleep(0.8)

                stuck_in_a_slow_redraw()
                pump(0.4)
            finally:
                dog.stop()
            text = log.read_text(encoding="utf-8")
            self.assertIn("GUI thread blocked", text)
            self.assertIn("stuck_in_a_slow_redraw", text)
            self.assertIn("stall ended", text)
            self.assertEqual(dog.stalls, 1)


if __name__ == "__main__":
    unittest.main()
