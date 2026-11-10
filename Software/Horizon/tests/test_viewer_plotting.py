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


class ZoomMathTests(unittest.TestCase):
    def test_view_points_interpolates_onto_the_edges(self):
        pts = [(0.0, 0.0), (10.0, 100.0), (20.0, 0.0)]
        self.assertEqual(viewer.view_points(pts, 5.0, 15.0),
                         [(5.0, 50.0), (10.0, 100.0), (15.0, 50.0)])
        # Zoomed between two samples: still a line across the view.
        self.assertEqual(viewer.view_points(pts, 2.0, 4.0), [(2.0, 20.0), (4.0, 40.0)])
        self.assertEqual(viewer.view_points(pts, 30.0, 40.0), [])
        self.assertEqual(viewer.view_points(pts, -5.0, 25.0), pts)

    def test_zoom_and_pan_stay_inside_the_data(self):
        full = (0.0, 100.0)
        self.assertEqual(viewer.zoom_range(full, full, 50.0, 0.5), (25.0, 75.0))
        # Zooming about a point keeps that point under the cursor.
        self.assertEqual(viewer.zoom_range(full, full, 10.0, 0.5), (5.0, 55.0))
        self.assertEqual(viewer.zoom_range((0.0, 10.0), full, 0.0, 0.5), (0.0, 5.0))
        self.assertIsNone(viewer.zoom_range((40.0, 60.0), full, 50.0, 10.0))
        self.assertEqual(viewer.zoom_range((90.0, 100.0), full, 100.0, 2.0), (80.0, 100.0))
        self.assertEqual(viewer.pan_range((10.0, 20.0), full, 5.0), (15.0, 25.0))
        self.assertEqual(viewer.pan_range((10.0, 20.0), full, -50.0), (0.0, 10.0))

    def test_nearest_point_and_value_text(self):
        pts = [(1.0, 10.0), (2.0, 20.0), (4.0, 40.0)]
        self.assertEqual(viewer.nearest_point(pts, 2.9), (2.0, 20.0))
        self.assertEqual(viewer.nearest_point(pts, 3.1), (4.0, 40.0))
        self.assertEqual(viewer.nearest_point(pts, -9), (1.0, 10.0))
        self.assertIsNone(viewer.nearest_point([], 1))
        self.assertEqual(viewer.value_text(15528.0), "15528")
        self.assertEqual(viewer.value_text(0.1 + 0.2), "0.3")
        self.assertEqual(viewer.value_text(-0.375012), "-0.375012")


class TimeAxisModelTests(unittest.TestCase):
    def write(self, name, header, rows):
        tmp = tempfile.mkdtemp()
        self.addCleanup(lambda: __import__("shutil").rmtree(tmp, ignore_errors=True))
        path = Path(tmp) / name
        with path.open("w", newline="") as f:
            f.write(header + "\n")
            for row in rows:
                f.write(",".join(str(v) for v in row) + "\n")
        return viewer.load_dataset(path)

    def test_infer_time_unit(self):
        for column, unit in (("uptime", "ms"), ("fc_uptime", "ms"), ("uptime_ms", "ms"),
                             ("timestamp_us", "us"), ("sim_ms", "ms"), ("t_ns", "ns"),
                             ("sim_s", "s"), ("time_seconds", "s"), ("wall_s", "s")):
            self.assertEqual(viewer.infer_time_unit(column), unit, column)

    def test_defaults_match_hitl_and_flash_files(self):
        truth = self.write("truth.csv", "sim_s,alt_msl_m", [(0.0, 1400), (0.5, 1401)])
        comparison = self.write("comparison.csv", "sim_s,uptime_ms,error_up_m",
                                [(0.05, 674451, 0.1)])
        imu = self.write("I000004A.csv", "time_seconds,timestamp_us,low_accel_x_g",
                         [(0.0, 692770390, 1.01)])
        packets = self.write("P000004A.csv", "time_seconds,timestamp_us,uptime,status",
                             [(0.0, 174272, 174, 9728)])
        self.assertEqual((truth.time_column, truth.time_unit), ("sim_s", "s"))
        self.assertEqual((comparison.time_column, comparison.time_unit), ("uptime_ms", "ms"))
        self.assertEqual((imu.time_column, imu.time_unit), ("timestamp_us", "us"))
        self.assertEqual(packets.time_column, viewer.AUTO_TIME)
        # All on one clock in seconds: comparison 674.451 s, IMU 692.77039 s.
        self.assertAlmostEqual(comparison.times()[0], 674.451)
        self.assertAlmostEqual(imu.times()[0], 692.77039)
        self.assertAlmostEqual(packets.times()[0], 0.174)

    def test_set_time_axis_converts_then_offsets(self):
        data = self.write("x.csv", "sim_ms,wall_s,value", [(1000, 5.0, 1), (2000, 6.0, 2)])
        self.assertEqual((data.time_column, data.time_unit), ("sim_ms", "ms"))
        self.assertEqual(data.times(), [1.0, 2.0])
        data.set_time_axis(unit="s")              # the user says it is seconds
        self.assertEqual(data.times(), [1000.0, 2000.0])
        data.set_time_axis(column="wall_s")       # new column: its own units
        self.assertEqual(data.time_unit, "s")
        self.assertEqual(data.times(), [5.0, 6.0])
        data.set_time_axis(offset=-5.0)
        self.assertEqual(data.time_axis_text(), "wall_s s, -5 s")
        self.assertNotIn(viewer.AUTO_TIME, data.time_choices())
        with self.assertRaises(ValueError):
            data.set_time_axis(column="nope")
        with self.assertRaises(ValueError):
            data.set_time_axis(unit="minutes")
        with self.assertRaises(ValueError):
            data.set_time_axis(offset=float("nan"))


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

    def test_per_file_time_axes_line_up_and_update_live(self):
        truth = Path(self.tmp) / "truth.csv"
        with truth.open("w", newline="") as f:
            f.write("sim_s,alt\n")
            for i in range(100):
                f.write(f"{i * 0.1:.1f},{i}\n")
        flash = Path(self.tmp) / "I000004A.csv"
        with flash.open("w", newline="") as f:
            f.write("time_seconds,timestamp_us,alt\n")
            for i in range(100):
                f.write(f"{i * 0.1:.1f},{692_000_000 + i * 100_000},{i}\n")
        panel = viewer.CsvViewerPanel(self.root)
        panel.pack(fill="both", expand=True)
        self.root.geometry("900x700")
        panel.load_csv(truth)
        panel.add_csv(flash)
        t_id, f_id = list(panel.datasets)
        self.assertEqual(panel.x_axis_combo.get(), viewer.SYNC_X)
        self.assertEqual(panel.series_points((t_id, "alt"))[0][0], 0.0)
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 692.0)
        self.assertIn("[timestamp_us us]", panel.trace_tree.item(f"d{f_id}", "text"))

        # Slide sim time onto the flash clock: offset applies after units.
        panel.set_time_axis(t_id, offset=692.0)
        self.assertAlmostEqual(panel.series_points((t_id, "alt"))[0][0], 692.0)
        self.assertIn("+692 s", panel.trace_tree.item(f"d{t_id}", "text"))
        # Wrong units are fixed in place, and the redraw is live (debounced).
        panel.set_time_axis(f_id, unit="ms")
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 692000.0)
        self.assertIsNotNone(panel.redraw_job)
        panel.set_time_axis(f_id, unit="us")

        # The controls follow the selected CSV and edit it.
        panel.trace_tree.selection_set(f"d{f_id}")
        self.root.update()
        self.assertEqual(panel.time_column_combo.get(), "timestamp_us")
        self.assertEqual(panel.time_unit_combo.get(), "us")
        self.assertIn(viewer.AUTO_TIME, panel.time_column_combo["values"])
        panel.time_column_combo.set("time_seconds")
        panel._time_controls_changed()
        self.assertEqual(panel.datasets[f_id].time_column, "time_seconds")
        self.assertEqual(panel.series_points((f_id, "alt"))[0][0], 0.0)
        panel.nudge_offset(0.1)
        panel.nudge_offset(1.0)
        self.assertAlmostEqual(panel.datasets[f_id].offset, 1.1)
        self.assertEqual(panel.offset_var.get(), "1.1")
        panel.offset_var.set("-")            # half-typed: ignored, not an error
        panel.apply_offset()
        self.assertAlmostEqual(panel.datasets[f_id].offset, 1.1)
        panel.offset_var.set("2.5")
        panel.apply_offset()
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 2.5)
        # Settings stay with the file while it is loaded.
        panel.trace_tree.selection_set(f"d{t_id}")
        self.root.update()
        self.assertEqual(panel.offset_var.get(), "692")
        panel.trace_tree.selection_set(f"d{f_id}")
        self.root.update()
        self.assertEqual(panel.offset_var.get(), "2.5")

    def test_shared_zoom_and_hover_values(self):
        a = self.write("a.csv", 10_000, 1.0)
        b = self.write("b.csv", 12_000, 2.0)
        panel = viewer.CsvViewerPanel(self.root)
        panel.pack(fill="both", expand=True)
        self.root.geometry("900x700")
        panel.load_csv(a)
        panel.add_csv(b)
        ids = list(panel.datasets)
        panel.clear_graphs()
        panel.layout.add([(ids[0], "altitude")])
        panel.layout.add([(ids[1], "altitude")])
        panel.notebook.select(panel.plot_tab)
        for _ in range(5):
            self.root.update()
        panel.draw_graphs()
        self.root.update()
        g0, g1 = (graph["geom"] for graph in panel._graphs)
        # Both graphs share one time axis: the union of the two logs.
        self.assertEqual((g0["x_min"], g0["x_max"]), (10.0, 21.95))
        self.assertEqual((g1["x_min"], g1["x_max"]), (10.0, 21.95))

        # Hover: exact sample values at the cursor, nearest sample in time.
        x = g0["left"] + (12.0 - 10.0) / (21.95 - 10.0) * (g0["right"] - g0["left"])
        lines = panel.hover_readout(x, (g0["top"] + g0["bottom"]) / 2)
        self.assertEqual(lines[0], f"{viewer.SYNC_X}: 12")
        self.assertEqual(lines[1], "a.csv: altitude = 40  @ 12")
        self.assertIsNone(panel.hover_readout(2, 2))
        # b.csv starts at 12 s: at 11 s its graph shows no (stale) value.
        x11 = g1["left"] + (11.0 - 10.0) / (21.95 - 10.0) * (g1["right"] - g1["left"])
        self.assertEqual(len(panel.hover_readout(x11, (g1["top"] + g1["bottom"]) / 2)), 1)
        panel.event_generate("<Motion>", x=0, y=0)  # no crash off-plot
        panel._hover(type("E", (), {"x": int(x), "y": int(g1["top"] + 5)})())
        self.assertTrue(panel.canvas.find_withtag("hover"))
        panel.clear_hover()

        # Drag-select on one graph zooms every graph (and their y axes).
        start = g1["left"] + 0.25 * (g1["right"] - g1["left"])
        end = g1["left"] + 0.5 * (g1["right"] - g1["left"])
        y = (g1["top"] + g1["bottom"]) / 2
        event = type("E", (), {"x": int(start), "y": int(y), "x_root": 0, "y_root": 0})
        panel._canvas_press(event())
        event.x = int(end)
        panel._canvas_motion(event())
        self.assertTrue(panel.canvas.find_withtag("zoomband"))
        panel._canvas_release(event())
        self.root.update()
        low, high = panel.x_view
        self.assertAlmostEqual(low, 10.0 + 0.25 * 11.95, delta=0.05)
        self.assertAlmostEqual(high, 10.0 + 0.5 * 11.95, delta=0.05)
        z0, z1 = (graph["geom"] for graph in panel._graphs)
        self.assertEqual((z0["x_min"], z0["x_max"]), (z1["x_min"], z1["x_max"]))
        self.assertLess(z0["y_end"], g0["y_end"])  # y refits to the data in view
        # Ctrl+wheel zooms about the cursor; zooming out past the data resets.
        panel.zoom(0.5)
        self.assertAlmostEqual(panel.x_view[1] - panel.x_view[0], (high - low) / 2)
        panel.zoom_out()
        panel.zoom_out()
        panel.zoom_out()
        self.assertIsNone(panel.x_view)
        panel.set_x_view((12.0, 13.0))
        panel.on_axis_changed()            # a new x axis drops the zoom
        self.assertIsNone(panel.x_view)


if __name__ == "__main__":
    unittest.main()
