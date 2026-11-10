import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import viewer_data as viewer  # noqa: E402  (data logic)
from viewer_qt_support import ViewerCase, pump  # noqa: E402


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
        self.assertEqual(data.times().tolist(), [1.0, 2.0])
        data.set_time_axis(unit="s")              # the user says it is seconds
        self.assertEqual(data.times().tolist(), [1000.0, 2000.0])
        data.set_time_axis(column="wall_s")       # new column: its own units
        self.assertEqual(data.time_unit, "s")
        self.assertEqual(data.times().tolist(), [5.0, 6.0])
        data.set_time_axis(offset=-5.0)
        self.assertEqual(data.time_axis_text(), "wall_s s, -5 s")
        self.assertNotIn(viewer.AUTO_TIME, data.time_choices())
        with self.assertRaises(ValueError):
            data.set_time_axis(column="nope")
        with self.assertRaises(ValueError):
            data.set_time_axis(unit="minutes")
        with self.assertRaises(ValueError):
            data.set_time_axis(offset=float("nan"))


import viewer as qt_viewer  # noqa: E402


class PanelTests(ViewerCase):
    def test_panel_builds_visible_graphs_lazily(self):
        path = Path(self.tmp) / "x.csv"
        with path.open("w", newline="") as f:
            f.write("time_seconds,a,b,c,d,e,f,gps_locked\n")
            for i in range(5000):
                f.write(f"{i / 100},{i},{-i},{i % 7},{i * 1e6},0.5,{i % 2},1\n")
        titles = []
        panel = self.make(1000, 700, on_title=titles.append)
        self.assertTrue(panel.load_csv(path))
        self.plots(panel)
        self.assertEqual(len(panel.slots), 7)
        self.assertTrue(panel.live_slots())
        self.assertEqual(titles[-1], "Horizon CSV Viewer - x.csv")
        # A binary column sits on a fixed 0..1 axis.
        slot = panel.slots[-1]
        bar = panel.graph_scroll.verticalScrollBar()
        bar.setValue(bar.maximum())
        panel.realize_visible_now()
        self.assertTrue(slot.realized)
        self.assertEqual(slot.refs, [(0, "gps_locked")])
        y_low, y_high = slot.vb.viewRange()[1]
        self.assertAlmostEqual(y_low, -0.1)
        self.assertAlmostEqual(y_high, 1.1)


class OverlayPanelTests(ViewerCase):
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
        panel = self.make(900, 600, on_title=titles.append)
        self.assertTrue(panel.load_csv(a))
        self.assertTrue(panel.add_csv(b))
        self.assertEqual(titles[-1], "Horizon CSV Viewer - a.csv + 1 more")
        self.assertEqual(panel.x_column(), viewer.SYNC_X)
        ids = list(panel.datasets)
        # The list is grouped by CSV with the columns underneath.
        tree = panel.trace_tree
        self.assertEqual(tree.topLevelItemCount(), 2)
        children = [tree.topLevelItem(1).child(i).text(0)
                    for i in range(tree.topLevelItem(1).childCount())]
        self.assertIn("altitude", children)
        # The first CSV auto-plots one graph per column (time columns skipped).
        self.assertEqual(panel.layout_model.graphs, [[(ids[0], "altitude")]])
        panel.drop(("traces", [(ids[1], "altitude")]), ("graph", 0))
        self.assertEqual(panel.layout_model.graphs,
                         [[(ids[0], "altitude"), (ids[1], "altitude")]])
        # Points sit on Witness uptime: b starts 2 s after a.
        self.assertAlmostEqual(panel.series_points((ids[0], "altitude"))[0][0], 10.0)
        self.assertAlmostEqual(panel.series_points((ids[1], "altitude"))[0][0], 12.0)
        # Per-CSV offset shifts that CSV only.
        panel.select_dataset(ids[1])
        panel.offset_spin.setValue(-2.0)
        self.assertAlmostEqual(panel.datasets[ids[1]].offset, -2.0)
        self.assertAlmostEqual(panel.series_points((ids[1], "altitude"))[0][0], 10.0)
        self.assertAlmostEqual(panel.series_points((ids[0], "altitude"))[0][0], 10.0)
        self.plots(panel)
        slot = panel.slots[0]
        self.assertEqual(len(slot.curves), 2)
        self.assertEqual(len(slot.findChildren(qt_viewer.LegendButton)), 2)
        # Legend dragged to the new-graph area, then back to the list.
        panel.drop(("legend", 0, (ids[1], "altitude")), ("new",))
        self.assertEqual(len(panel.layout_model), 2)
        panel.drop(("legend", 1, (ids[1], "altitude")), ("list",))
        self.assertEqual(panel.layout_model.graphs, [[(ids[0], "altitude")]])
        panel.remove_dataset(ids[0])
        self.assertEqual(panel.layout_model.graphs, [])
        self.assertEqual([d.path for d in panel.datasets.values()], [b])
        self.assertEqual(titles[-1], "Horizon CSV Viewer - b.csv")

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
        panel = self.make(900, 700)
        panel.load_csv(truth)
        panel.add_csv(flash)
        t_id, f_id = list(panel.datasets)
        self.assertEqual(panel.x_column(), viewer.SYNC_X)
        self.assertEqual(panel.series_points((t_id, "alt"))[0][0], 0.0)
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 692.0)
        self.assertIn("[timestamp_us us]", panel._tree_item(f_id).text(0))
        # Slide sim time onto the flash clock: offset applies after units.
        panel.set_time_axis(t_id, offset=692.0)
        self.assertAlmostEqual(panel.series_points((t_id, "alt"))[0][0], 692.0)
        self.assertIn("+692 s", panel._tree_item(t_id).text(0))
        # Wrong units are fixed in place.
        panel.set_time_axis(f_id, unit="ms")
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 692000.0)
        panel.set_time_axis(f_id, unit="us")
        # The controls follow the selected CSV and edit it.
        panel.select_dataset(f_id)
        self.assertEqual(panel.time_column_combo.currentText(), "timestamp_us")
        self.assertEqual(panel.time_unit_combo.currentText(), "us")
        choices = [panel.time_column_combo.itemText(i)
                   for i in range(panel.time_column_combo.count())]
        self.assertIn(viewer.AUTO_TIME, choices)
        panel.time_column_combo.setCurrentText("time_seconds")
        panel._time_controls_changed()
        self.assertEqual(panel.datasets[f_id].time_column, "time_seconds")
        self.assertEqual(panel.series_points((f_id, "alt"))[0][0], 0.0)
        panel.nudge_offset(0.1)
        panel.nudge_offset(1.0)
        self.assertAlmostEqual(panel.datasets[f_id].offset, 1.1)
        self.assertEqual(panel.offset_spin.text(), "1.1")
        panel.offset_spin.setValue(2.5)            # typed / spun: applied live
        self.assertAlmostEqual(panel.series_points((f_id, "alt"))[0][0], 2.5)
        self.assertIn("2.5 to", panel.time_axis_note.text())
        # Settings stay with the file while it is loaded.
        panel.select_dataset(t_id)
        self.assertEqual(panel.offset_spin.text(), "692")
        panel.select_dataset(f_id)
        self.assertEqual(panel.offset_spin.text(), "2.5")

    def test_shared_zoom_and_hover_values(self):
        a = self.write("a.csv", 10_000, 1.0)
        b = self.write("b.csv", 12_000, 2.0)
        panel = self.make(900, 700)
        panel.load_csv(a)
        panel.add_csv(b)
        ids = list(panel.datasets)
        panel.clear_graphs()
        panel.layout_model.add([(ids[0], "altitude")])
        panel.layout_model.add([(ids[1], "altitude")])
        panel.rebuild_graphs()
        self.plots(panel)
        v0, v1 = (slot.vb for slot in panel.slots)
        # Both graphs share one time axis: the union of the two logs.
        self.assertEqual(panel.current_x_range(), (10.0, 21.95))
        for vb in (v0, v1):
            low, high = vb.viewRange()[0]
            self.assertAlmostEqual(low, 10.0)
            self.assertAlmostEqual(high, 21.95)
        # Hover: exact sample values at the cursor, nearest sample in time.
        lines = panel.hover_readout(0, 12.0)
        self.assertEqual(lines[0], f"{viewer.SYNC_X}: 12")
        self.assertEqual(lines[1], "a.csv: altitude = 40  @ 12")
        # b.csv starts at 12 s: at 11 s its graph shows no (stale) value.
        self.assertEqual(len(panel.hover_readout(1, 11.0)), 1)
        panel.show_hover(0, 12.0)
        self.assertTrue(panel.slots[1].hover_line.isVisible())     # synced crosshair
        panel.clear_hover()
        y0_before = v0.viewRange()[1]
        # Zooming one graph zooms every graph (links), y refits to the view.
        v1.setXRange(10.0 + 0.25 * 11.95, 10.0 + 0.5 * 11.95, padding=0)
        pump(0.2)
        low, high = panel.x_view
        self.assertAlmostEqual(low, 10.0 + 0.25 * 11.95, delta=0.05)
        self.assertAlmostEqual(high, 10.0 + 0.5 * 11.95, delta=0.05)
        self.assertAlmostEqual(v0.viewRange()[0][0], low, delta=1e-6)
        self.assertLess(v0.viewRange()[1][1], y0_before[1])
        self.assertIn("in view:", panel.slots[0].stats_label.text())
        # Zoom about the centre; zooming out past the data resets.
        panel.zoom(0.5)
        self.assertAlmostEqual(panel.x_view[1] - panel.x_view[0], (high - low) / 2, delta=1e-6)
        panel.zoom_out()
        panel.zoom_out()
        panel.zoom_out()
        self.assertIsNone(panel.x_view)
        panel.set_x_view((12.0, 13.0))
        self.assertEqual(panel.current_time_zoom(), (12.0, 13.0))
        panel.on_axis_changed()            # a new x axis drops the zoom
        self.assertIsNone(panel.x_view)


class OffsetMovesCurvesTests(ViewerCase):
    """Changing an offset must only move curves: same arrays, no replot."""

    def test_offset_change_keeps_arrays_and_clips_correctly(self):
        path = self.write_text("p.csv", "uptime,alt,speed\n" + "".join(
            f"{i * 10},{i},{i * 2}\n" for i in range(5000)))
        panel = self.make(900, 700)
        self.assertTrue(panel.load_csv(path))
        self.plots(panel)
        dataset_id = next(iter(panel.datasets))
        slot = panel.slots[0]
        curve = slot.curves[0][1]
        x_before, y_before = curve.xData, curve.yData
        cached = panel.series(slot.refs[0])
        calls = []
        original = panel.rebuild_graphs
        panel.rebuild_graphs = lambda *a: calls.append(a) or original(*a)
        panel.set_x_view((10.0, 12.0))
        panel.select_dataset(dataset_id)
        panel.offset_spin.setValue(5.0)
        pump()
        self.assertEqual(calls, [])                         # nothing rebuilt
        self.assertIs(curve.xData, x_before)                 # same arrays
        self.assertIs(curve.yData, y_before)
        self.assertIs(panel.series(slot.refs[0])[0], cached[0])
        self.assertIs(panel.slots[0].curves[0][1], curve)    # same curve item
        self.assertEqual(curve.pos().x(), 5.0)
        # Clipping honours the shift: shared 10..12 s is file time 5..7 s.
        shown = curve._getDisplayDataset()
        self.assertGreaterEqual(shown.x.min(), 4.9)
        self.assertLessEqual(shown.x.max(), 7.1)
        self.assertGreater(shown.x.size, 10)
        # The zoom is kept; the full range follows the file.
        self.assertEqual(panel.x_view, (10.0, 12.0))
        self.assertAlmostEqual(panel._full_x[0], 5.0)

    def test_time_unit_change_rebuilds_x_once(self):
        path = self.write_text("q.csv", "t_ms,alt\n" + "".join(
            f"{i * 10},{i}\n" for i in range(1000)))
        panel = self.make(900, 700)
        panel.load_csv(path)
        self.plots(panel)
        dataset_id = next(iter(panel.datasets))
        curve = panel.slots[0].curves[0][1]
        panel.set_time_axis(dataset_id, column="t_ms")      # units from the name: ms
        self.assertEqual(float(curve.xData[-1]), 9.99)
        y_before = curve.yData
        panel.set_time_axis(dataset_id, unit="s")
        self.assertTrue(np.shares_memory(curve.yData, y_before))   # values untouched
        self.assertEqual(float(curve.xData[-1]), 9990.0)    # x rebuilt in seconds
        self.assertEqual(panel._full_x, (0.0, 9990.0))


class LazyPlotTests(ViewerCase):
    def test_many_columns_create_few_plots(self):
        columns = [f"c{i}" for i in range(100)]
        path = self.write_text("wide.csv", "time_seconds," + ",".join(columns) + "\n" + "".join(
            f"{i / 100}," + ",".join(str(i * k) for k in range(100)) + "\n"
            for i in range(300)))
        panel = self.make(1000, 700)
        self.assertTrue(panel.load_csv(path))
        self.plots(panel)
        self.assertEqual(len(panel.slots), 100)
        live = panel.live_slots()
        self.assertGreater(len(live), 0)
        self.assertLess(len(live), 15)                       # only what is in view
        self.assertFalse(panel.slots[-1].realized)
        # Scrolling builds what comes into view and frees what is far away.
        bar = panel.graph_scroll.verticalScrollBar()
        for step in range(1, 21):
            fraction = step / 20
            bar.setValue(int(bar.maximum() * fraction))
            panel.realize_visible_now()
            self.assertLessEqual(len(panel.live_slots()), qt_viewer.MAX_LIVE_PLOTS)
        self.assertTrue(panel.slots[-1].realized)
        self.assertFalse(panel.slots[0].realized)
        # Linked x axes survive freeing the first (link root) plot.
        live = panel.live_slots()
        live[0].vb.setXRange(1.0, 2.0, padding=0)
        pump(0.1)
        for slot in live:
            low, high = slot.vb.viewRange()[0]
            self.assertAlmostEqual(low, 1.0, delta=1e-6)
            self.assertAlmostEqual(high, 2.0, delta=1e-6)
        bar.setValue(0)
        panel.realize_visible_now()
        self.assertAlmostEqual(panel.slots[0].vb.viewRange()[0][0], 1.0, delta=1e-6)


class MemoryGuardTests(ViewerCase):
    def big(self):
        return self.write_text("big.csv", "t,a,b,c,d\n" + "".join(
            f"{i},{i},{i},{i},{i}\n" for i in range(20000)))

    def test_refuses_before_allocating(self):
        import csv_columns
        path = self.big()
        estimate = viewer.estimate_load(path)
        self.assertEqual(estimate.columns, ["t", "a", "b", "c", "d"])
        self.assertAlmostEqual(estimate.rows, 20000, delta=2000)
        original = csv_columns.load_table
        csv_columns.load_table = lambda *a, **k: self.fail("allocated despite the guard")
        try:
            with self.assertRaises(viewer.MemoryBudgetError) as caught:
                viewer.load_dataset(path, memory_budget=estimate.bytes // 2)
        finally:
            csv_columns.load_table = original
        self.assertIn("would need about", str(caught.exception))
        self.assertTrue(caught.exception.estimate.thinnable)
        # Fewer columns fit; so does keeping every Nth row.
        budget = estimate.bytes_for(["t", "a"]) + 1
        data = viewer.load_dataset(path, memory_budget=budget, columns={"t", "a"})
        self.assertEqual(data.numeric_columns, ["t", "a"])
        thinned = viewer.load_dataset(path, memory_budget=estimate.bytes // 3, thin=True)
        self.assertGreater(thinned.stride, 1)
        self.assertLess(thinned.row_count, 20000)
        self.assertGreater(viewer.default_memory_budget(), 0)

    def test_viewer_offers_column_selection(self):
        path = self.big()
        panel = self.make()
        panel.memory_budget = viewer.estimate_load(path).bytes_for(["t", "a"]) + 1
        prompts = []
        panel.memory_prompt = lambda error, request: prompts.append(error)
        panel.add_files([path])
        self.assertTrue(panel.wait_for_loads(30))
        self.assertEqual(panel.datasets, {})
        self.assertEqual(len(prompts), 1)
        self.assertTrue(any(kind == "memory" for kind, _t, _m in panel.messages))
        # The dialog lists the columns and only accepts a selection that fits.
        dialog = qt_viewer.ColumnPickerDialog(prompts[0], panel)
        self.addCleanup(dialog.deleteLater)
        for i in range(dialog.list.count()):
            item = dialog.list.item(i)
            item.setCheckState(qt_viewer.QtCore.Qt.CheckState.Checked)
        self.assertFalse(dialog.load_button.isEnabled())
        for i in range(2, dialog.list.count()):
            dialog.list.item(i).setCheckState(qt_viewer.QtCore.Qt.CheckState.Unchecked)
        self.assertTrue(dialog.load_button.isEnabled())
        dialog.accept()
        self.assertEqual(dialog.choice, ("columns", ["t", "a"]))
        panel._load_queue.append(qt_viewer.LoadRequest(path, columns=["t", "a"]))
        panel._start_next_load()
        self.assertTrue(panel.wait_for_loads(30))
        self.assertEqual(next(iter(panel.datasets.values())).numeric_columns, ["t", "a"])


if __name__ == "__main__":
    unittest.main()
