import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import viewer  # noqa: E402


class TickTests(unittest.TestCase):
    def test_ticks_cover_range_without_float_drift(self):
        step, ticks = viewer.nice_ticks(0.03, 0.97, 5)
        self.assertAlmostEqual(step, 0.2)
        self.assertLessEqual(ticks[0], 0.03)
        self.assertGreaterEqual(ticks[-1], 0.97)
        self.assertEqual([viewer.tick_text(t, step) for t in ticks],
                         ["0", "0.2", "0.4", "0.6", "0.8", "1"])

    def test_inside_ticks_stay_in_range(self):
        _step, ticks = viewer.nice_ticks(13.7, 128.2, 5, inside=True)
        self.assertTrue(all(13.7 <= t <= 128.2 for t in ticks))
        self.assertEqual(ticks, [50.0, 100.0])

    def test_degenerate_range(self):
        self.assertEqual(viewer.nice_ticks(5.0, 5.0)[1], [5.0])
        self.assertEqual(len(viewer.nice_ticks(float("nan"), 1.0)[1]), 1)


class DecimationTests(unittest.TestCase):
    def test_short_series_unchanged(self):
        pts = [(i, i * i) for i in range(10)]
        self.assertIs(viewer.decimate_minmax(pts, 0, 9, 100), pts)

    def test_spikes_survive_and_size_is_bounded(self):
        pts = [(i * 0.001, math.sin(i * 0.01)) for i in range(100_000)]
        pts[54_321] = (pts[54_321][0], 50.0)
        out = viewer.decimate_minmax(pts, pts[0][0], pts[-1][0], 400)
        self.assertLessEqual(len(out), 800)
        self.assertIn(50.0, [y for _x, y in out])
        self.assertEqual(out, sorted(out, key=lambda p: p[0]))


class LayoutTests(unittest.TestCase):
    def test_graph_height_fills_then_scrolls(self):
        self.assertEqual(viewer.graph_height_for(2, 700), 350)
        self.assertEqual(viewer.graph_height_for(1, 2000),
                         viewer.MAX_GRAPH_HEIGHT)
        self.assertEqual(viewer.graph_height_for(40, 700),
                         viewer.MIN_GRAPH_HEIGHT)

    def test_elide(self):
        measure = len
        self.assertEqual(viewer.elide("short", 10, measure), "short")
        self.assertEqual(viewer.elide("a long column name", 8, measure),
                         "a long …")


class PanelTests(unittest.TestCase):
    def test_panel_draws_visible_graphs_at_any_size(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        tmp = tempfile.mkdtemp()
        path = Path(tmp) / "x.csv"
        with path.open("w", newline="") as f:
            f.write("time_seconds,a,b,c,d,e,f,gps_locked\n")
            for i in range(5000):
                f.write(f"{i / 100},{i},{-i},{i % 7},{i * 1e6},0.5,{i % 2},1\n")
        try:
            titles = []
            panel = viewer.CsvViewerPanel(root, on_title=titles.append)
            panel.pack(fill="both", expand=True)
            self.assertTrue(panel.load_csv(path))
            panel.notebook.select(panel.plot_tab)
            for size in ("1000x700", "420x300"):
                root.geometry(size)
                for _ in range(5):
                    root.update()
                panel.draw_graphs()
                root.update()
                self.assertTrue(panel._drawn)
                # Lazy drawing: off-screen graphs wait until scrolled to.
                self.assertLess(len(panel._drawn), len(panel._graphs))
            panel.canvas.yview_moveto(1.0)
            panel.draw_visible()
            self.assertIn(len(panel._graphs) - 1, panel._drawn)
            self.assertEqual(titles, ["Horizon CSV Viewer - x.csv"])
        finally:
            root.destroy()
            os.remove(path)
            os.rmdir(tmp)


if __name__ == "__main__":
    unittest.main()
