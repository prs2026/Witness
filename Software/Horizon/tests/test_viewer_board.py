"""Separate y axes per trace, the dashboard with docked spectrograms, and
saving / loading data boards (viewer_board.py)."""

import json
import os
import sys
import unittest
from pathlib import Path

import numpy as np
from PyQt6 import QtCore

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import viewer  # noqa: E402
import viewer_board  # noqa: E402
from viewer_qt_support import ViewerCase, pump  # noqa: E402


def write_log(path: Path, columns=("alt", "accel", "temp"), seconds=20.0, rate=100.0):
    """A CSV on a time_s axis: alt ramps 0..2000, accel is a +-1 sine whose
    amplitude jumps to 50 after 10 s, temp sits near 20."""
    t = np.arange(int(seconds * rate)) / rate
    data = {
        "alt": t * 100.0,
        "accel": np.where(t < 10.0, 1.0, 50.0) * np.sin(2 * np.pi * 5.0 * t),
        "temp": 20.0 + 0.1 * np.sin(t),
        "extra": t * 0.0 + 3.0,
    }
    with path.open("w") as f:
        f.write("time_s," + ",".join(columns) + "\n")
        for i, ti in enumerate(t):
            f.write(f"{ti:.3f}," + ",".join(f"{data[c][i]:.5f}" for c in columns) + "\n")
    return path


class BoardCase(ViewerCase):
    def open_log(self, name="a.csv", **kwargs):
        panel = getattr(self, "panel", None) or self.make(1100, 800)
        self.panel = panel
        before = set(panel.datasets)
        self.assertTrue(panel.add_csv(write_log(Path(self.tmp) / name, **kwargs)))
        return panel, next(iter(set(panel.datasets) - before))

    def settle(self, panel):
        self.plots(panel)
        pump(0.05)
        panel.realize_visible_now()


class SeparateAxesTests(BoardCase):
    def test_layout_model_flag_follows_the_graph(self):
        layout = viewer.GraphLayout()
        a, b, c = (0, "alt"), (0, "accel"), (0, "temp")
        layout.add([a])
        layout.add([b], 0, separate=True)
        self.assertTrue(layout.is_separate(0))
        layout.add([c])
        layout.move(1, c, 0)
        self.assertEqual(layout.graphs, [[a, b, c]])
        layout.remove_dataset(5)
        self.assertTrue(layout.is_separate(0))
        layout.set_separate(0, False)
        self.assertFalse(layout.is_separate(0))
        self.assertEqual(layout.graphs, [[a, b, c]])      # plain list equality still holds

    def test_axes_are_created_per_trace_and_coloured(self):
        panel, did = self.open_log()
        panel.layout_model.add([(did, "alt"), (did, "accel"), (did, "temp")])
        # all three on one graph, own axes
        panel.layout_model.graphs = [viewer.GraphRefs([(did, "alt"), (did, "accel"),
                                                       (did, "temp")])]
        panel.layout_model.set_separate(0, True)
        panel.rebuild_graphs()
        self.settle(panel)
        slot = panel.slots[0]
        self.assertTrue(slot.realized and slot.separate)
        self.assertEqual(len(slot.axes), 3)
        self.assertEqual(set(slot.aux_views), {(did, "accel"), (did, "temp")})
        colours = [panel.series_style(0, i, 3, (did, "x"))[0] for i in range(3)]
        self.assertEqual([a.labelText for a in slot.axes], ["alt", "accel", "temp"])
        self.assertEqual([a.labelStyle["color"] for a in slot.axes], colours)
        self.assertEqual([a.orientation for a in slot.axes], ["left", "right", "right"])
        self.assertEqual(len(slot.curves), 3)
        for ref, curve in slot.curves:
            self.assertIs(curve.getViewBox(), slot.view_for(ref))
        self.assertEqual(panel._side_counts(), (1, 2))

    def test_each_axis_autoranges_on_its_own_for_the_visible_x(self):
        panel, did = self.open_log()
        panel.layout_model.graphs = [viewer.GraphRefs([(did, "alt"), (did, "accel"),
                                                       (did, "temp")])]
        panel.layout_model.set_separate(0, True)
        panel.rebuild_graphs()
        self.settle(panel)
        slot = panel.slots[0]
        views = [slot.vb, slot.view_for((did, "accel")), slot.view_for((did, "temp"))]

        def y_ranges():
            pump(0.05)
            for view in views:
                view.updateAutoRange()
            return [view.viewRange()[1] for view in views]

        panel.set_x_view((0.0, 5.0))
        alt, accel, temp = y_ranges()
        self.assertLess(alt[1], 600)             # alt reaches 500 by t=5
        self.assertLess(accel[1], 3)             # accel is +-1 before t=10
        self.assertAlmostEqual((temp[0] + temp[1]) / 2, 20.0, delta=0.5)
        self.assertLess(temp[1] - temp[0], 1.0)
        panel.set_x_view((12.0, 18.0))
        alt, accel, temp = y_ranges()
        self.assertGreater(alt[0], 1000)         # alt only spans 1200..1800 now
        self.assertGreater(accel[1], 40)         # the 50 g section
        self.assertLess(temp[1] - temp[0], 1.0)
        # Shared x: every box shows the same time range.
        for view in views:
            self.assertAlmostEqual(view.viewRange()[0][0], 12.0, places=6)
            self.assertAlmostEqual(view.viewRange()[0][1], 18.0, places=6)

    def test_shared_axis_mode_is_unchanged_and_toggle_rebuilds(self):
        panel, did = self.open_log()
        panel.layout_model.add([(did, "alt"), (did, "accel")])      # two graphs
        panel.drop(("legend", 1, (did, "accel")), ("graph", 0))     # overlay on graph 0
        self.settle(panel)
        slot = panel.slots[0]
        self.assertFalse(slot.separate)
        self.assertEqual(slot.aux_views, {})
        self.assertEqual(len(slot.axes), 1)
        panel.set_separate_axes(0, True)
        pump(0.05)
        self.settle(panel)
        self.assertTrue(panel.slots[0].separate)
        self.assertEqual(len(panel.slots[0].axes), 2)
        panel.set_separate_axes(0, False)
        pump(0.05)
        self.settle(panel)
        self.assertEqual(len(panel.slots[0].axes), 1)

    def test_new_overlays_follow_the_default_checkbox(self):
        panel, did = self.open_log()
        panel.layout_model.add([(did, "alt")])
        panel.own_axes_check.setChecked(True)
        panel.drop(("traces", [(did, "accel")]), ("graph", 0))
        self.assertTrue(panel.layout_model.is_separate(0))

    def test_hover_marker_of_an_own_axis_trace_sits_on_the_curve(self):
        panel, did = self.open_log()
        panel.layout_model.graphs = [viewer.GraphRefs([(did, "alt"), (did, "temp")])]
        panel.layout_model.set_separate(0, True)
        panel.rebuild_graphs()
        self.settle(panel)
        slot = panel.slots[0]
        slot.view_for((did, "temp")).updateAutoRange()
        panel.show_hover(0, 7.0)
        spots = slot.hover_marks.points()
        self.assertEqual(len(spots), 2)
        low, high = slot.vb.viewRange()[1]
        for spot in spots:
            self.assertTrue(low - 1 <= spot.pos().y() <= high + 1)


class BoardFileTests(BoardCase):
    def build(self):
        panel, did = self.open_log()
        panel.layout_model.graphs = [
            viewer.GraphRefs([(did, "alt"), (did, "accel")]),
            viewer.GraphRefs([(did, "temp")])]
        panel.layout_model.set_separate(0, True)
        panel.rebuild_graphs()
        panel.set_columns(1)
        dock = panel.add_board_spectrogram((did, "accel"), 1)
        dock.window.nfft_combo.setCurrentText("256")
        dock.window.window_combo.setCurrentText("Blackman")
        dock.window.welch_check.setChecked(True)
        self.settle(panel)
        return panel, did, dock

    def test_spectrogram_sits_in_the_board_linked_in_time(self):
        panel, did, dock = self.build()
        self.assertTrue(dock.window.wait(30))
        self.assertIsNotNone(dock.window.analysis, dock.window.status_label.text())
        # Between the two graphs in the grid.
        grid = panel.graph_layout
        rows = {grid.getItemPosition(grid.indexOf(w))[0]: w
                for w in (*panel.slots, dock)}
        self.assertEqual([rows[r] for r in sorted(rows)], [panel.slots[0], dock, panel.slots[1]])
        # The time axes line up on screen (graph with 2 extra y axes vs the colour bar).
        pump(0.2)
        spec_rect = dock.window.p1.vb.sceneBoundingRect()
        graphics = dock.window.graphics
        spec_edges = [graphics.mapToGlobal(graphics.mapFromScene(p)).x()
                      for p in (spec_rect.topLeft(), spec_rect.topRight())]
        for slot in panel.slots:
            rect = slot.vb.sceneBoundingRect()
            edges = [slot.plot.mapToGlobal(slot.plot.mapFromScene(p)).x()
                     for p in (rect.topLeft(), rect.topRight())]
            for got, want in zip(edges, spec_edges):
                self.assertAlmostEqual(got, want, delta=2)
        # Zooming the graphs moves the spectrogram's time axis, and back.
        panel.set_x_view((4.0, 9.0))
        pump(0.05)
        low, high = dock.window.p1.vb.viewRange()[0]
        self.assertAlmostEqual(low, 4.0, places=6)
        self.assertAlmostEqual(high, 9.0, places=6)
        dock.window.p1.vb.setXRange(6.0, 8.0, padding=0)
        pump(0.05)
        self.assertAlmostEqual(panel.slots[1].vb.viewRange()[0][0], 6.0, places=6)
        self.assertEqual(panel.x_view, (6.0, 8.0))
        # Closing it takes it off the board.
        panel.remove_board_spectrogram(dock)
        self.assertEqual(panel.docks, [])

    def test_save_load_round_trip(self):
        panel, did, dock = self.build()
        panel.set_graph_height(1, 300)
        path = Path(self.tmp) / "my.board.json"
        panel.save_board(path)
        data = json.loads(path.read_text())
        self.assertEqual(data["format"], viewer_board.BOARD_FORMAT)
        self.assertEqual(data["version"], 2)
        self.assertEqual([p["type"] for p in data["panels"]], ["plot", "spectrogram", "plot"])
        self.assertEqual([a["side"] for a in data["panels"][0]["axes"]], ["left", "right"])
        self.assertEqual([[t["column"] for t in a["traces"]] for a in data["panels"][0]["axes"]],
                         [["alt"], ["accel"]])
        self.assertEqual(data["version"], 2)
        self.assertEqual([t["column"] for t in data["panels"][0]["traces"]], ["alt", "accel"])
        self.assertEqual(data["panels"][1]["nfft"], 256)
        self.assertEqual(data["panels"][1]["window"], "Blackman")
        self.assertEqual(data["panels"][1]["time_window"], "full")
        self.assertEqual(data["panels"][2]["height"], 300)

        other = self.make(1100, 800)
        other_id = None
        self.assertTrue(other.load_csv(write_log(Path(self.tmp) / "b.csv", seconds=12.0)))
        other_id = next(iter(other.datasets))
        report = other.load_board(path)
        self.assertTrue(report.ok, report.text())
        self.assertEqual((report.plots, report.spectrograms), (2, 1))
        self.assertEqual([list(g) for g in other.layout_model.graphs],
                         [[(other_id, "alt"), (other_id, "accel")], [(other_id, "temp")]])
        self.assertTrue(other.layout_model.is_separate(0))
        self.assertFalse(other.layout_model.is_separate(1))
        self.assertEqual(len(other.docks), 1)
        window = other.docks[0].window
        self.assertEqual(other.docks[0].after, 1)
        self.assertEqual(window.nfft_combo.currentText(), "256")
        self.assertEqual(window.window_combo.currentText(), "Blackman")
        self.assertTrue(window.wait(30))
        self.assertLess(window.analysis.t_end, 12.0)       # the new, shorter log
        self.assertGreater(window.analysis.t_end, 10.5)
        self.settle(other)
        self.assertEqual(len(other.slots[0].axes), 2)
        # Saving again from the loaded board gives the same description.
        again = other.board_dict()
        for key in ("panels", "x_axis", "graphs_per_row"):
            self.assertEqual(again[key], panel.board_dict()[key], key)

    def test_columns_missing_from_the_new_log_are_reported_and_skipped(self):
        panel, did, dock = self.build()
        board = panel.board_dict()
        other = self.make(1100, 800)
        self.assertTrue(other.load_csv(write_log(Path(self.tmp) / "c.csv",
                                                 columns=("alt", "extra"))))
        other_id = next(iter(other.datasets))
        report = other.load_board(board)
        self.assertEqual(sorted(report.missing), ["accel", "temp"])
        self.assertEqual([list(g) for g in other.layout_model.graphs], [[(other_id, "alt")]])
        self.assertEqual(other.docks, [])                    # its column was missing
        self.assertTrue(any("Graph 3" in n for n in report.notes))
        self.assertTrue(any("Spectrogram 2" in n for n in report.notes))
        self.assertFalse(report.ok)
        self.assertEqual(other.messages[-1][1], "Data board")
        self.settle(other)                                    # and it still draws
        self.assertTrue(other.slots[0].realized)

    def test_unusable_boards_are_refused_not_crashed(self):
        panel, did, dock = self.build()
        bad = Path(self.tmp) / "bad.json"
        bad.write_text("{not json")
        self.assertIsNone(panel.open_board(bad))
        self.assertIn("not valid JSON", panel.messages[-1][2])
        bad.write_text(json.dumps({"format": "other"}))
        self.assertIsNone(panel.open_board(bad))
        future = panel.board_dict()
        future["version"] = 99
        bad.write_text(json.dumps(future))
        self.assertIsNone(panel.open_board(bad))
        self.assertIn("version 99", panel.messages[-1][2])
        self.assertEqual(len(panel.layout_model.graphs), 2)          # untouched
        empty = self.make()
        report = empty.load_board(panel.board_dict())
        self.assertEqual(report.plots, 0)
        self.assertTrue(report.notes)

    def test_time_axis_and_second_log_map_by_position(self):
        panel, did = self.open_log("one.csv")
        panel, did2 = self.open_log("two.csv")
        panel.set_time_axis(did2, offset=2.5)
        panel.layout_model.graphs = [viewer.GraphRefs([(did, "alt"), (did2, "alt")])]
        panel.rebuild_graphs()
        board = panel.board_dict()
        self.assertEqual([s["offset"] for s in board["sources"]], [0.0, 2.5])
        fresh = self.make()
        fresh.add_csv(write_log(Path(self.tmp) / "x.csv"))
        fresh.add_csv(write_log(Path(self.tmp) / "y.csv"))
        ids = sorted(fresh.datasets)
        report = fresh.load_board(board)
        self.assertTrue(report.ok, report.text())
        self.assertEqual(fresh.datasets[ids[1]].offset, 2.5)
        self.assertEqual(list(fresh.layout_model.graphs[0]), [(ids[0], "alt"), (ids[1], "alt")])
        # With one log open, the second source is reported, not an error.
        single = self.make()
        single.load_csv(write_log(Path(self.tmp) / "z.csv"))
        report = single.load_board(board)
        self.assertEqual(report.plots, 1)
        self.assertTrue(any("no matching open log" in n for n in report.notes))

    def test_schema_validation(self):
        good = {"format": viewer_board.BOARD_FORMAT, "version": 1, "panels": [
            {"type": "plot", "traces": [{"column": "a"}]}]}
        out = viewer_board.validate(good)
        self.assertEqual([a["side"] for a in out["panels"][0]["axes"]], ["left"])
        self.assertEqual(out["panels"][0]["traces"][0]["source"], 0)
        for broken in ({"format": viewer_board.BOARD_FORMAT, "version": 1,
                        "panels": [{"type": "pie"}]},
                       {"format": viewer_board.BOARD_FORMAT, "version": 1,
                        "panels": [{"type": "plot", "traces": [{"source": 0}]}]},
                       {"format": viewer_board.BOARD_FORMAT, "version": "1", "panels": []}):
            with self.assertRaises(viewer_board.BoardError):
                viewer_board.validate(broken)


def write_imu_log(path: Path, seconds=20.0, rate=100.0):
    """Low-g x,y,z, high-g x,y,z and a velocity: the acceleration + velocity overlay."""
    t = np.arange(int(seconds * rate)) / rate
    cols = {"low_x": np.sin(t), "low_y": np.cos(t), "low_z": 1.0 + 0 * t,
            "high_x": 20 * np.sin(t), "high_y": 20 * np.cos(t), "high_z": 20 + 0 * t,
            "vel_x": 300 * t, "vel_y": 100 * t, "vel_z": 50 * t}
    with path.open("w") as f:
        f.write("time_s," + ",".join(cols) + "\n")
        for i, ti in enumerate(t):
            f.write(f"{ti:.3f}," + ",".join(f"{cols[c][i]:.4f}" for c in cols) + "\n")
    return path


class AxisGroupModelTests(unittest.TestCase):
    def setUp(self):
        self.refs = [(0, name) for name in ("lx", "ly", "hx", "vx", "vy")]

    def graph(self):
        return viewer.GraphRefs(self.refs)

    def test_shared_by_default_and_merge_split_move(self):
        g = self.graph()
        self.assertFalse(g.custom)
        self.assertEqual([x.refs for x in g.axis_groups()], [self.refs])
        # a new axis in the middle: side with fewer axes, right when tied
        g.assign(self.refs[3], side=None)
        self.assertEqual([x.side for x in g.axis_groups()], ["left", "right"])
        g.assign(self.refs[4], group=1)                      # merge onto the velocity axis
        self.assertEqual(g.axis_groups()[1].refs, self.refs[3:])
        self.assertEqual(g.axis_groups()[0].refs, self.refs[:3])
        g.assign(self.refs[2], side="left")                  # a new axis on the left
        self.assertEqual([x.side for x in g.axis_groups()], ["left", "right", "left"])
        g.merge(2, 0)                                        # axis merge
        self.assertEqual(len(g.axis_groups()), 2)
        self.assertEqual(sorted(g.axis_groups()[0].refs), sorted(self.refs[:3]))
        g.move_group(1, "left")
        self.assertEqual([x.side for x in g.axis_groups()], ["left", "left"])
        g.move_group(1, "right")
        # emptied axes vanish; a single left axis is the shared state again
        g.merge(1, 0)
        self.assertEqual(len(g.axis_groups()), 1)
        self.assertFalse(g.custom)
        g.split()
        self.assertEqual(len(g.axis_groups()), 5)

    def test_layout_add_move_with_axis_targets(self):
        layout = viewer.GraphLayout()
        layout.add(self.refs[:1])
        layout.add(self.refs[1:3], 0, axis=("group", 0))      # onto the existing axis
        self.assertFalse(layout.graphs[0].custom)
        layout.add(self.refs[3:], 0, axis=("new", "right"))   # both share one new axis
        groups = layout.graphs[0].axis_groups()
        self.assertEqual([g.side for g in groups], ["left", "right"])
        self.assertEqual(groups[1].refs, self.refs[3:])
        layout.move(0, self.refs[4], 0, axis=("new", "left"))  # within the graph
        self.assertEqual([g.refs for g in layout.graphs[0].axis_groups()][-1], [self.refs[4]])
        layout.add([self.refs[2]], 0, axis=("group", 1))
        self.assertIn(self.refs[2], layout.graphs[0].axis_groups()[1].refs)
        layout.remove_dataset(9)
        self.assertEqual(len(layout.graphs[0].axis_groups()), 3)
        layout.remove_series(0, self.refs[4])
        self.assertEqual(len(layout.graphs[0].axis_groups()), 2)

    def test_axis_dropped_on_axis_merges_and_across_graphs(self):
        layout = viewer.GraphLayout()
        layout.add(self.refs[:1])
        layout.add(self.refs[1:2], 0)
        layout.add(self.refs[2:3])
        layout.set_separate(0, True)
        layout.move_axis(0, 1, 0, ("group", 0))
        self.assertFalse(layout.graphs[0].custom)
        layout.set_separate(0, True)
        layout.move_axis(0, 1, 0, ("new", "left"))            # same graph, edge: moves the axis
        self.assertEqual(layout.graphs[0].axis_groups()[1].side, "left")
        layout.move_axis(0, 1, 1, ("new", "right"))           # to another graph
        self.assertIn(self.refs[1], layout.graphs[1])
        self.assertEqual(layout.graphs[1].axis_groups()[-1].side, "right")

    def test_drop_zone_classification(self):
        spans = [(0.0, 64.0, 0), (936.0, 1000.0, 1)]
        plot = (64.0, 936.0)

        def at(x, left=1, right=1):
            return viewer.classify_drop(x, spans, *plot, left, right)

        self.assertEqual((at(30).kind, at(30).value, at(30).zone), ("group", 0, "axis"))
        self.assertEqual((at(990).kind, at(990).value), ("group", 1))
        for x, side in ((70, "left"), (930, "right")):
            t = at(x)
            self.assertEqual((t.kind, t.value, t.zone), ("new", side, "edge"), x)
        self.assertEqual((at(500, 1, 1).value, at(500, 1, 1).zone), ("right", "middle"))
        self.assertEqual(at(500, 1, 2).value, "left")
        self.assertEqual(at(500, 0, 0).value, "right")
        self.assertEqual(at(500, 2, 1).value, "right")
        self.assertEqual(at(500).axis, ("new", "right"))
        # a blank pad left of the plot, outside every axis, is the left edge
        far = viewer.classify_drop(-30.0, [(10.0, 74.0, 0)], 74.0, 900.0, 1, 0)
        self.assertEqual((far.kind, far.value), ("new", "left"))


class AxisGroupViewTests(BoardCase):
    def imu(self):
        panel = self.make(1200, 800)
        self.assertTrue(panel.add_csv(write_imu_log(Path(self.tmp) / "imu.csv")))
        did = next(iter(panel.datasets))
        self.panel = panel
        return panel, did

    def test_group_autoranges_over_all_its_traces(self):
        panel, did = self.imu()
        low = [(did, c) for c in ("low_x", "low_y", "low_z", "high_x", "high_y", "high_z")]
        vel = [(did, c) for c in ("vel_x", "vel_y", "vel_z")]
        panel.layout_model.add(low[:1])
        panel.drop(("traces", low[1:]), ("graph", 0, ("group", 0)))
        panel.drop(("traces", vel), ("graph", 0, ("new", "right")))
        self.settle(panel)
        slot = panel.slots[0]
        self.assertEqual([a.orientation for a in slot.axes], ["left", "right"])
        self.assertEqual(len(slot.spec[0][1]), 6)
        self.assertEqual(slot.spec[1][1], vel)
        self.assertEqual([a.labelText for a in slot.axes], ["low_x +5", "vel_x +2"])
        self.assertEqual(slot.axes[0].labelStyle["color"], viewer.NEUTRAL_AXIS)
        for ref in vel:
            self.assertIs(slot.view_for(ref), slot.group_views[1])
        for ref in low:
            self.assertIs(slot.view_for(ref), slot.vb)
        panel.set_x_view((0.0, 4.0))
        pump(0.05)
        for view in slot.group_views:
            view.updateAutoRange()
        left, right = (v.viewRange()[1] for v in slot.group_views)
        self.assertGreater(left[1], 15)          # the high-g traces set the left range
        self.assertLess(left[1], 30)
        self.assertGreater(right[1], 1000)       # vel_x reaches 1200 by t=4
        self.assertLess(right[1], 1400)

    def test_slot_drop_target_follows_the_geometry(self):
        panel, did = self.imu()
        panel.layout_model.add([(did, "low_x")])
        panel.layout_model.add([(did, "vel_x")], 0, axis=("new", "right"))
        panel.rebuild_graphs()
        self.settle(panel)
        slot = panel.slots[0]

        def slot_pos(scene_x):
            vb = slot.vb.sceneBoundingRect()
            point = slot.plot.mapFromScene(QtCore.QPointF(scene_x, vb.center().y()))
            return QtCore.QPointF(slot.plot.mapTo(slot, point))

        vb = slot.vb.sceneBoundingRect()
        left_axis = slot.axes[0].parentItem().mapRectToScene(slot.axes[0].geometry())
        right_axis = slot.axes[1].parentItem().mapRectToScene(slot.axes[1].geometry())
        t = slot.drop_target(slot_pos(left_axis.center().x()))
        self.assertEqual((t.kind, t.value), ("group", 0))
        t = slot.drop_target(slot_pos(right_axis.center().x()))
        self.assertEqual((t.kind, t.value), ("group", 1))
        t = slot.drop_target(slot_pos(vb.left() + 5))
        self.assertEqual((t.kind, t.value, t.zone), ("new", "left", "edge"))
        t = slot.drop_target(slot_pos(vb.center().x()))
        self.assertEqual((t.kind, t.zone), ("new", "middle"))
        self.assertIsNone(slot.drop_target(QtCore.QPointF(5, 5)))      # the header
        slot._show_hint(t)
        self.assertFalse(slot.hint.rect_f.isEmpty())
        slot._show_hint(None)
        self.assertFalse(slot.hint.isVisible())

    def test_drop_payloads_and_axis_menu_operations(self):
        panel, did = self.imu()
        panel.layout_model.add([(did, "low_x")])
        panel.drop(("traces", [(did, "vel_x")]), ("graph", 0, ("new", "left")))
        self.assertEqual([g.side for g in panel.layout_model.graphs[0].axis_groups()],
                         ["left", "left"])
        panel.drop(("legend", 0, (did, "vel_x")), ("graph", 0, ("group", 0)))
        self.assertFalse(panel.layout_model.graphs[0].custom)
        panel.drop(("legend", 0, (did, "vel_x")), ("graph", 0, ("new", "right")))
        panel.drop(("axis", 0, 1), ("graph", 0, ("new", "left")))     # axis dragged to the left
        self.assertEqual([g.side for g in panel.layout_model.graphs[0].axis_groups()],
                         ["left", "left"])
        panel.drop(("axis", 0, 1), ("graph", 0, ("group", 0)))        # onto another axis
        self.assertFalse(panel.layout_model.graphs[0].custom)
        panel.layout_model.add([(did, "vel_x")], 0, axis=("new", "right"))
        panel.move_axis_side(0, 1, "left")
        pump(0.05)
        self.assertEqual(panel.layout_model.graphs[0].axis_groups()[1].side, "left")
        panel.merge_axes(0, 1, 0)
        pump(0.05)
        self.assertFalse(panel.layout_model.graphs[0].custom)
        panel.drop(("traces", [(did, "vel_x"), (did, "vel_y")]),
                   ("graph", 0, ("new", "right")))
        panel.split_axis(0, 1)
        pump(0.05)
        self.assertEqual(len(panel.layout_model.graphs[0].axis_groups()), 3)
        self.settle(panel)
        self.assertEqual(len(panel.slots[0].axes), 3)

    def test_groups_round_trip_and_v1_boards_load(self):
        panel, did = self.imu()
        lows = [(did, c) for c in ("low_x", "low_y", "high_x")]
        vel = [(did, c) for c in ("vel_x", "vel_y")]
        panel.layout_model.add(lows[:1])
        panel.layout_model.add(lows[1:], 0, axis=("group", 0))
        panel.layout_model.add(vel, 0, axis=("new", "right"))
        panel.rebuild_graphs()
        board = panel.board_dict()
        axes = board["panels"][0]["axes"]
        self.assertEqual([(a["side"], [t["column"] for t in a["traces"]]) for a in axes],
                         [("left", ["low_x", "low_y", "high_x"]),
                          ("right", ["vel_x", "vel_y"])])
        path = Path(self.tmp) / "g.board.json"
        panel.save_board(path)
        other = self.make(1200, 800)
        other.add_csv(write_imu_log(Path(self.tmp) / "imu2.csv", seconds=8.0))
        report = other.load_board(path)
        self.assertTrue(report.ok, report.text())
        self.assertEqual(other.board_dict()["panels"], board["panels"])
        # Missing columns thin a group; a group left empty disappears.
        thin = json.loads(path.read_text())
        thin["panels"][0]["traces"][3]["column"] = "nope"
        thin["panels"][0]["traces"][4]["column"] = "nope2"
        thin["panels"][0]["axes"][1]["traces"][0]["column"] = "nope"
        thin["panels"][0]["axes"][1]["traces"][1]["column"] = "nope2"
        third = self.make(1200, 800)
        third.add_csv(write_imu_log(Path(self.tmp) / "imu3.csv", seconds=3.0))
        report = third.load_board(thin)
        self.assertEqual(sorted(report.missing), ["nope", "nope2"])
        self.assertEqual(len(third.layout_model.graphs[0].axis_groups()), 1)
        # Version 1: "shared" and "separate" words.
        v1 = {"format": viewer_board.BOARD_FORMAT, "version": 1, "sources": [], "panels": [
            {"type": "plot", "axes": "shared", "traces": [{"column": "low_x"},
                                                           {"column": "vel_x"}]},
            {"type": "plot", "axes": "separate", "traces": [{"column": "low_x"},
                                                             {"column": "vel_x"},
                                                             {"column": "vel_y"}]}]}
        fourth = self.make(1200, 800)
        fourth.add_csv(write_imu_log(Path(self.tmp) / "imu4.csv", seconds=3.0))
        report = fourth.load_board(v1)
        self.assertTrue(report.ok, report.text())
        first, second = fourth.layout_model.graphs
        self.assertEqual([g.side for g in first.axis_groups()], ["left"])
        self.assertEqual([g.side for g in second.axis_groups()], ["left", "right", "right"])
        self.settle(fourth)
        self.assertEqual(len(fourth.slots[1].axes), 3)
        with self.assertRaises(viewer_board.BoardError):
            viewer_board.validate({**v1, "version": 3})
        with self.assertRaises(viewer_board.BoardError):
            viewer_board.validate({**v1, "panels": [{"type": "plot", "traces": [],
                                                     "axes": [{"side": "top"}]}]})



def write_state_log(path: Path, boost_at: float, coast_at=None, seconds=30.0, rate=20.0,
                    with_state=True, glitch_at=None, t0=0.0):
    """A log whose flight_state goes 0 -> 1 at ``boost_at`` (-> 2 at ``coast_at``)."""
    t = np.arange(int(seconds * rate)) / rate + t0
    state = np.zeros(t.size)
    state[t >= boost_at] = 1
    if coast_at is not None:
        state[t >= coast_at] = 2
    if glitch_at is not None:                         # one-sample blip into Boost
        state[int(np.searchsorted(t, glitch_at))] = 1
    with path.open("w") as f:
        f.write("time_s,alt" + (",flight_state" if with_state else "") + "\n")
        for ti, si in zip(t, state):
            f.write(f"{ti:.3f},{ti * 2:.2f}" + (f",{int(si)}" if with_state else "") + "\n")
    return path


class SyncTests(BoardCase):
    def load(self, panel, *paths):
        before = set(panel.datasets)
        for path in paths:
            self.assertTrue(panel.add_csv(path))
        return sorted(set(panel.datasets) - before)

    def test_transition_detection_direct_column(self):
        panel = self.make()
        (did,) = self.load(panel, write_state_log(Path(self.tmp) / "a.csv", 7.0, 15.0))
        data = panel.datasets[did]
        self.assertAlmostEqual(viewer.state_transition_time(data, 0, 1), 7.0, places=3)
        self.assertAlmostEqual(viewer.state_transition_time(data, 1, 2), 15.0, places=3)
        self.assertIsNone(viewer.state_transition_time(data, 2, 3))
        self.assertIsNone(viewer.state_transition_time(data, 3, 4))

    def test_single_sample_glitch_is_not_an_edge(self):
        panel = self.make()
        (did,) = self.load(panel, write_state_log(Path(self.tmp) / "g.csv", 20.0,
                                                  glitch_at=5.0))
        self.assertAlmostEqual(viewer.state_transition_time(panel.datasets[did], 0, 1),
                               20.0, places=3)

    def test_multiple_logs_align_on_the_transition(self):
        panel = self.make()
        a, b = self.load(panel, write_state_log(Path(self.tmp) / "a.csv", 7.0),
                         write_state_log(Path(self.tmp) / "b.csv", 12.5, t0=3.0))
        result = panel.sync_logs((0, 1))
        self.assertEqual(result["skipped"], [])
        self.assertEqual(panel.sync_transition, (0, 1))
        self.assertEqual(panel.sync_combo.currentText(), "Pad idle → Boost")
        self.assertAlmostEqual(panel.datasets[a].offset, -7.0)
        self.assertAlmostEqual(panel.datasets[b].offset, -12.5)
        # On the shared axis both rise into Boost at t = 0.
        for did in (a, b):
            data = panel.datasets[did]
            t = data.times() + data.offset
            first = t[np.flatnonzero(data.numeric_data["flight_state"] == 1)[0]]
            self.assertAlmostEqual(float(first), 0.0, places=3)
        # The offsets are the ordinary, editable per-file ones.
        panel.set_time_axis(a, offset=-6.0)
        self.assertAlmostEqual(panel.datasets[a].offset, -6.0)

    def test_not_found_is_reported_and_offset_kept(self):
        panel = self.make()
        a, b = self.load(panel, write_state_log(Path(self.tmp) / "a.csv", 7.0, 15.0),
                         write_state_log(Path(self.tmp) / "b.csv", 9.0))
        panel.set_time_axis(b, offset=1.25)
        result = panel.sync_logs((1, 2))
        self.assertEqual(result["synced"], ["a.csv"])
        self.assertEqual(result["skipped"], ["b.csv"])
        self.assertAlmostEqual(panel.datasets[b].offset, 1.25)       # unchanged
        self.assertAlmostEqual(panel.datasets[a].offset, -15.0)
        self.assertIn("b.csv", panel.messages[-1][2])
        # A log with no state column at all is skipped too.
        (c,) = self.load(panel, write_state_log(Path(self.tmp) / "c.csv", 1.0,
                                                with_state=False))
        self.assertEqual(panel.sync_logs((0, 1))["skipped"], ["c.csv"])

    def test_log_without_state_uses_its_sessions_text_log(self):
        panel = self.make()
        imu = write_state_log(Path(self.tmp) / "I00000F7.csv", 1.0, with_state=False)
        (Path(self.tmp) / "S00000F7.TXT").write_text(
            "I (100) boot: ok\nI (8250) flight_state: pad idle -> boost\n"
            "I (16000) flight_state: boost -> coast\n")
        (did,) = self.load(panel, imu)
        self.assertAlmostEqual(viewer.text_log_transition_time(imu, 0, 1), 8.25)
        self.assertEqual(panel.sync_logs((0, 1))["synced"], ["I00000F7.csv"])
        self.assertAlmostEqual(panel.datasets[did].offset, -8.25)
        self.assertAlmostEqual(panel.sync_logs((1, 2)) and panel.datasets[did].offset, -16.0)
        self.assertEqual(panel.sync_logs((2, 3))["skipped"], ["I00000F7.csv"])

    def test_log_without_state_uses_the_paired_open_log(self):
        panel = self.make()
        imu, state = self.load(
            panel, write_state_log(Path(self.tmp) / "I0000A01.csv", 1.0, with_state=False),
            write_state_log(Path(self.tmp) / "S0000A01.csv", 11.0))
        other, = self.load(panel, write_state_log(Path(self.tmp) / "S0000B02.csv", 4.0))
        panel.sync_logs((0, 1))
        self.assertAlmostEqual(panel.datasets[state].offset, -11.0)
        self.assertAlmostEqual(panel.datasets[imu].offset, -11.0)     # via the session's S log
        self.assertAlmostEqual(panel.datasets[other].offset, -4.0)
        self.assertEqual(viewer.session_key(Path("I0000A01.BIN")), "0000a01")
        self.assertIsNone(viewer.session_key(Path("flight.csv")))

    def test_sync_is_saved_in_the_board_and_reapplied(self):
        panel = self.make(1100, 800)
        a, b = self.load(panel, write_state_log(Path(self.tmp) / "a.csv", 7.0),
                         write_state_log(Path(self.tmp) / "b.csv", 12.5))
        panel.sync_logs((0, 1))
        board = panel.board_dict()
        self.assertEqual(board["sync"], [0, 1])
        self.assertEqual([s["offset"] for s in board["sources"]], [-7.0, -12.5])
        path = Path(self.tmp) / "s.board.json"
        panel.save_board(path)
        other = self.make(1100, 800)
        c, d = self.load(other, write_state_log(Path(self.tmp) / "c.csv", 3.0),
                         write_state_log(Path(self.tmp) / "d.csv", 5.0, 9.0))
        report = other.load_board(path)
        self.assertTrue(report.ok, report.text())
        self.assertEqual(other.sync_transition, (0, 1))
        self.assertAlmostEqual(other.datasets[c].offset, -3.0)        # re-synced, not -7
        self.assertAlmostEqual(other.datasets[d].offset, -5.0)
        # A new log opened afterwards joins the sync.
        (e,) = self.load(other, write_state_log(Path(self.tmp) / "e.csv", 2.0))
        self.assertAlmostEqual(other.datasets[e].offset, -2.0)
        # A board without a sync clears it; a log with no Boost is reported.
        plain = other.board_dict()
        plain["sync"] = None
        other.load_board(plain)
        self.assertIsNone(other.sync_transition)
        late = self.make()
        self.load(late, write_state_log(Path(self.tmp) / "f.csv", 2.0, with_state=False))
        report = late.load_board(path)
        self.assertTrue(any("Pad idle" in n for n in report.notes))
        with self.assertRaises(viewer_board.BoardError):
            viewer_board.validate({**board, "sync": [1]})


if __name__ == "__main__":
    unittest.main()
