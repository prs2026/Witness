"""Column-wise CSV loading (csv_columns) and the viewer's numpy data path."""

import math
import os
import shutil
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import csv_columns as cc  # noqa: E402
import viewer  # noqa: E402


class LoadTableTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, name, text):
        path = Path(self.tmp) / name
        path.write_bytes(text.encode("utf-8"))
        return path

    def test_numeric_columns_text_columns_and_blanks(self):
        path = self.write("a.csv", "packet_type,t,v,hex,empty\r\n"
                                   "state,0.5,1,0a 0b,\r\n"
                                   "camera,1.5,,ff,\r\n"
                                   ",2.5,-3e2,,\r\n")
        table = cc.load_table(path)
        self.assertEqual(table.row_count, 3)
        self.assertEqual(table.total_rows, 3)
        self.assertEqual(table.columns["t"].tolist(), [0.5, 1.5, 2.5])
        np.testing.assert_array_equal(table.columns["v"], [1.0, np.nan, -300.0])
        self.assertNotIn("hex", table.columns)        # text column
        self.assertNotIn("empty", table.columns)      # never held a number
        kinds = table.text["packet_type"]
        self.assertEqual([kinds[i] for i in range(3)], ["state", "camera", ""])
        self.assertEqual((kinds == "camera").tolist(), [False, True, False])
        self.assertEqual(kinds.isin(("state", "camera")).tolist(), [True, True, False])

    def test_fast_path_matches_robust_fallback(self):
        rows = ["packet_type,a,b,c"] + [f"x{i % 3},{i},{'' if i % 7 else i * 0.5},{-i}"
                                        for i in range(5000)]
        path = self.write("b.csv", "\n".join(rows) + "\n")
        fast = cc.load_table(path)
        header, _sample, _hb, _avg = cc._read_head(str(path))
        robust = cc._load_robust(str(path), header, 1, path.stat().st_size, None, None)
        for name in ("a", "b", "c"):
            np.testing.assert_array_equal(fast.columns[name], robust.columns[name])
        self.assertEqual(fast.text["packet_type"].unique(), robust.text["packet_type"].unique())

    def test_odd_files_fall_back_to_csv_reader(self):
        # Text appears in a column only after the sampled head; and a short row.
        lines = ["t,v"] + [f"{i},{i}" for i in range(20000)] + ["20000,oops", "20001"]
        path = self.write("c.csv", "\n".join(lines) + "\n")
        table = cc.load_table(path)
        self.assertEqual(table.row_count, 20002)
        self.assertNotIn("v", table.columns)           # turned out to be text
        self.assertEqual(table.columns["t"][-1], 20001)

    def test_memory_cap_thins_rows(self):
        lines = ["t,v"] + [f"{i},{i * 2}" for i in range(10000)]
        path = self.write("d.csv", "\n".join(lines) + "\n")
        table = cc.load_table(path, max_bytes=40_000)
        self.assertGreater(table.stride, 1)
        self.assertEqual(table.total_rows, 10000)
        self.assertEqual(table.columns["t"][:3].tolist(),
                         [0.0, float(table.stride), float(2 * table.stride)])
        self.assertTrue(np.all(table.columns["v"] == 2 * table.columns["t"]))

    def test_progress_and_cancel(self):
        lines = ["t,v"] + [f"{i},{i}" for i in range(300000)]
        path = self.write("e.csv", "\n".join(lines) + "\n")
        seen = []
        cc.load_table(path, progress=seen.append)
        self.assertEqual(seen[-1], 1.0)
        self.assertTrue(all(0 <= f <= 1 for f in seen))
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(cc.LoadCancelled):
            cc.load_table(path, cancel=cancel)

    def test_no_header(self):
        with self.assertRaises(ValueError):
            cc.load_table(self.write("f.csv", ""))


class SeriesTests(unittest.TestCase):
    def test_series_view_matches_list_version(self):
        pts = [(0.0, 0.0), (10.0, 100.0), (20.0, 0.0)]
        series = viewer.Series([p[0] for p in pts], [p[1] for p in pts])
        for lo, hi in ((5.0, 15.0), (2.0, 4.0), (30.0, 40.0), (-5.0, 25.0)):
            self.assertEqual(list(viewer.view_points(series, lo, hi)),
                             viewer.view_points(pts, lo, hi))
        self.assertEqual(viewer.nearest_point(series, 7.0), (10.0, 100.0))
        self.assertEqual(series[0], (0.0, 0.0))
        self.assertEqual(len(series), 3)

    def test_envelope_keeps_spikes_and_bounds_points(self):
        xs = np.arange(1_000_000, dtype=float) * 1e-3
        ys = np.sin(xs)
        ys[543_210] = 50.0
        ys[123_456] = -40.0
        rx, ry = viewer.envelope(xs, ys, xs[0], xs[-1], 500)
        self.assertLessEqual(rx.size, 1000)
        self.assertIn(50.0, ry)
        self.assertIn(-40.0, ry)
        self.assertTrue(np.all(np.diff(rx) >= 0))
        # Few points: unchanged.
        sx, sy = viewer.envelope(xs[:10], ys[:10], 0, 1, 500)
        np.testing.assert_array_equal(sx, xs[:10])
        np.testing.assert_array_equal(sy, ys[:10])


class AsyncLoadTests(unittest.TestCase):
    def setUp(self):
        try:
            import tkinter as tk
            self.root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        self.root.destroy()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_background_load_progress_errors_and_cancel(self):
        good = Path(self.tmp) / "good.csv"
        good.write_text("uptime,alt\n" + "".join(f"{i * 10},{i}\n" for i in range(200000)))
        bad = Path(self.tmp) / "bad.csv"
        bad.write_text("")
        panel = viewer.CsvViewerPanel(self.root)
        panel.pack(fill="both", expand=True)
        errors = []
        original = viewer.messagebox.showerror
        viewer.messagebox.showerror = lambda *a, **k: errors.append(a)
        try:
            panel.add_csvs([good, bad])
            self.assertTrue(panel.loading)
            self.root.update()
            self.assertTrue(panel.load_bar.winfo_ismapped())   # progress shown
            self.assertTrue(panel.wait_for_loads(60))
        finally:
            viewer.messagebox.showerror = original
        self.assertEqual([d.path.name for d in panel.datasets.values()], ["good.csv"])
        self.assertEqual(len(errors), 1)                       # bad.csv reported once
        self.assertFalse(panel.load_bar.winfo_ismapped())
        dataset = next(iter(panel.datasets.values()))
        self.assertIsInstance(dataset.numeric_data["alt"], np.ndarray)
        # Cancel drops the file being read and the queue.
        panel.add_csvs([good, good])
        panel.cancel_loads()
        self.assertTrue(panel.wait_for_loads(60))
        self.assertLessEqual(len(panel.datasets), 2)

    def test_summary_matches_row_based_code(self):
        path = Path(self.tmp) / "p.csv"
        lines = ["packet_type,status,fc_status,uptime,barometric_altitude,"
                 "low_accel_x,low_accel_y,low_accel_z,hitl_active"]
        for i in range(400):
            state = 1 if 100 <= i < 200 else (2 if i >= 200 else 0)
            status = (0x6E << 8) | state | (0x10 if i % 50 == 0 else 0)
            kind = "camera" if i % 9 == 0 else "state"
            lines.append(f"{kind},{status},{status},{i * 10},{i * 1.5},"
                         f"{0.1 * (i % 40)},0,{1.0 if i % 3 else ''},")
        path.write_text("\n".join(lines) + "\n")
        dataset = viewer.load_dataset(path)
        import csv as _csv
        with path.open(newline="") as f:
            reader = _csv.DictReader(f)
            rows = list(reader)
            fields = list(reader.fieldnames)
        viewer.add_derived_flight_state(rows, fields)
        viewer.add_status_bit_traces(rows, fields)
        viewer.add_status_flag_columns(rows, fields)
        self.assertEqual(dataset.summary, viewer.calculate_summary(rows))
        for name in dataset.numeric_columns:
            ref = [viewer.numeric_value(r.get(name)) for r in rows]
            ref = np.array([math.nan if v is None else v for v in ref])
            np.testing.assert_array_equal(dataset.numeric_data[name], ref, err_msg=name)


if __name__ == "__main__":
    unittest.main()
