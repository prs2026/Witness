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


class LayoutModelTests(unittest.TestCase):
    def test_add_overlay_move_remove(self):
        layout = viewer.GraphLayout()
        a, b, c = (0, "alt"), (1, "alt"), (0, "accel")
        layout.add([a, c])                      # two new graphs
        self.assertEqual(layout.graphs, [[a], [c]])
        layout.add([b], target=0)               # overlay on graph 0
        layout.add([b], target=0)               # duplicates ignored
        self.assertEqual(layout.graphs, [[a, b], [c]])
        layout.move(0, b, 1)                    # legend dragged to graph 1
        self.assertEqual(layout.graphs, [[a], [c, b]])
        layout.move(0, a, 1)                    # emptied graph disappears
        self.assertEqual(layout.graphs, [[c, b, a]])
        layout.move(0, a, None)                 # dragged to the new-graph area
        self.assertEqual(layout.graphs, [[c, b], [a]])
        layout.remove_dataset(1)
        self.assertEqual(layout.graphs, [[c], [a]])
        layout.remove_series(0, c)
        self.assertEqual(layout.graphs, [[a]])

    def test_move_onto_graph_already_showing_it_merges(self):
        layout = viewer.GraphLayout()
        a = (0, "x")
        layout.add([a])
        layout.add([a])
        layout.move(1, a, 0)
        self.assertEqual(layout.graphs, [[a]])

    def test_unique_label(self):
        self.assertEqual(viewer.unique_label("a.csv", []), "a.csv")
        self.assertEqual(viewer.unique_label("a.csv", ["a.csv", "a.csv (2)"]),
                         "a.csv (3)")

    def test_sync_time_prefers_witness_uptime(self):
        self.assertEqual(viewer.sync_time_seconds(
            {"uptime": "15528", "time_seconds": "0.5"}), 15.528)
        self.assertEqual(viewer.sync_time_seconds(
            {"fc_uptime": "2000", "time_seconds": ""}), 2.0)
        self.assertEqual(viewer.sync_time_seconds({"time_seconds": "3.25"}), 3.25)
        self.assertIsNone(viewer.sync_time_seconds({}))


class OverlayPanelTests(unittest.TestCase):
    def setUp(self):
        try:
            import tkinter as tk
            self.root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        self.root.destroy()
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, name, start_ms, scale):
        path = Path(self.tmp) / name
        with path.open("w", newline="") as f:
            f.write("packet_type,uptime,time_seconds,altitude\n")
            for i in range(200):
                f.write(f"state,{start_ms + i * 50},{i * 0.05},{i * scale}\n")
        return path

    def test_two_csvs_overlay_on_uptime(self):
        a = self.write("a.csv", 10_000, 1.0)
        b = self.write("b.csv", 12_000, 2.0)
        titles = []
        panel = viewer.CsvViewerPanel(self.root, on_title=titles.append)
        panel.pack(fill="both", expand=True)
        self.root.geometry("900x600")
        self.assertTrue(panel.load_csv(a))
        self.assertTrue(panel.add_csv(b))
        self.assertEqual(titles[-1], "Horizon CSV Viewer - a.csv + 1 more")
        self.assertEqual(panel.x_axis_combo.get(), viewer.SYNC_X)
        ids = list(panel.datasets)
        # Tree is grouped by CSV with the columns underneath.
        self.assertEqual(panel.trace_tree.get_children(), tuple(f"d{i}" for i in ids))
        self.assertIn(f"d{ids[1]}|altitude", panel.trace_tree.get_children(f"d{ids[1]}"))
        # The first CSV auto-plots one graph per column (time columns skipped).
        self.assertEqual(panel.layout.graphs, [[(ids[0], "altitude")]])
        panel.drop(("traces", [(ids[1], "altitude")]), ("graph", 0))
        self.assertEqual(panel.layout.graphs,
                         [[(ids[0], "altitude"), (ids[1], "altitude")]])
        # Points sit on Witness uptime: b starts 2 s after a.
        pa = panel.series_points((ids[0], "altitude"))
        pb = panel.series_points((ids[1], "altitude"))
        self.assertAlmostEqual(pa[0][0], 10.0)
        self.assertAlmostEqual(pb[0][0], 12.0)
        # Per-CSV offset shifts that CSV only.
        panel.trace_tree.selection_set(f"d{ids[1]}")
        panel.offset_var.set("-2")
        panel.apply_offset()
        self.assertAlmostEqual(panel.series_points((ids[1], "altitude"))[0][0], 10.0)
        panel.notebook.select(panel.plot_tab)
        for _ in range(5):
            self.root.update()
        panel.draw_graphs()
        self.root.update()
        self.assertEqual(len(panel._legend_items), 2)
        self.assertEqual(panel.graph_at(5), 0)
        self.assertIsNone(panel.graph_at(panel._drop_top + 5))
        # Legend dragged to the new-graph area, then back to the list.
        panel.drop(("legend", 0, (ids[1], "altitude")), ("new",))
        self.assertEqual(len(panel.layout), 2)
        panel.drop(("legend", 1, (ids[1], "altitude")), ("list",))
        self.assertEqual(panel.layout.graphs, [[(ids[0], "altitude")]])
        panel.remove_dataset(ids[0])
        self.assertEqual(panel.layout.graphs, [])
        self.assertEqual(panel.csv_path, b)


if __name__ == "__main__":
    unittest.main()
