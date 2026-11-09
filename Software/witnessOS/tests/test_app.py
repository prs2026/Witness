import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import wos_widgets as ww  # noqa: E402


class PureLayoutTests(unittest.TestCase):
    def test_fit_inner_fills_large_windows(self):
        self.assertEqual(ww.fit_inner(1000, 800, 600, 400),
                         (1000, 800, False, False))

    def test_fit_inner_scrolls_small_windows(self):
        self.assertEqual(ww.fit_inner(500, 300, 600, 400),
                         (600, 400, True, True))
        self.assertEqual(ww.fit_inner(700, 300, 600, 400),
                         (700, 400, False, True))

    def test_can_scroll(self):
        self.assertFalse(ww.can_scroll((0.0, 1.0), 1))
        self.assertFalse(ww.can_scroll((0.0, 0.5), -1))
        self.assertTrue(ww.can_scroll((0.0, 0.5), 1))
        self.assertTrue(ww.can_scroll((0.5, 1.0), -1))


class TkTests(unittest.TestCase):
    def setUp(self):
        try:
            import tkinter as tk
            self.tk = tk
            self.root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        self.root.geometry("+0+0")
        self.tmp = tempfile.mkdtemp(prefix="witnessos-")

    def tearDown(self):
        try:
            self.root.destroy()
        except Exception:
            pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    def pump(self, n=8):
        for _ in range(n):
            self.root.update()

    def test_scrollable_frame_shows_bars_only_when_needed(self):
        tk = self.tk
        frame = ww.ScrollableFrame(self.root)
        frame.pack(fill="both", expand=True)
        tk.Frame(frame.inner, width=500, height=400).pack()
        self.root.geometry("800x600")
        self.pump()
        frame.relayout()
        self.assertFalse(frame.vbar.winfo_ismapped())
        self.assertFalse(frame.hbar.winfo_ismapped())
        self.root.geometry("300x200")
        self.pump()
        frame.relayout()
        self.pump()
        self.assertTrue(frame.vbar.winfo_ismapped())
        self.assertTrue(frame.hbar.winfo_ismapped())
        # Content keeps its requested size, so nothing is squashed/clipped.
        self.assertGreaterEqual(frame.inner.winfo_width(), 500)
        self.assertGreaterEqual(frame.inner.winfo_height(), 400)

    def test_wheel_router_prefers_inner_scroller(self):
        tk = self.tk
        router = ww.WheelRouter(self.root)
        outer = ww.ScrollableFrame(self.root, router=router)
        outer.pack(fill="both", expand=True)
        inner_canvas = tk.Canvas(outer.inner, width=200, height=100,
                                 scrollregion=(0, 0, 200, 1000))
        inner_canvas.pack()
        tk.Frame(outer.inner, width=300, height=900).pack()
        router.register(inner_canvas)
        text = tk.Text(outer.inner, height=3)
        text.pack()
        self.root.geometry("400x300")
        self.pump()
        outer.relayout()
        inner_canvas.yview_moveto(0.0)
        self.pump()
        self.assertIs(router.target_for(inner_canvas, 1), inner_canvas)
        # At its top the inner canvas cannot scroll up: the outer one may not
        # either (also at top) so nothing takes the wheel.
        self.assertIsNone(router.target_for(inner_canvas, -1))
        inner_canvas.yview_moveto(1.0)
        self.pump()
        self.assertIs(router.target_for(inner_canvas, 1), outer.canvas)
        # Text widgets keep their native wheel handling.
        self.assertIsNone(router.target_for(text, 1))

    def test_app_builds_all_tabs_loads_csv_and_closes(self):
        import witnessos
        csv_path = Path(self.tmp) / "flight.csv"
        with csv_path.open("w", newline="") as f:
            f.write("time_seconds,barometric_altitude,packet_type,status\n")
            for i in range(2000):
                f.write(f"{i * 0.01:.2f},{(i % 500) * 1.5:.1f},state,"
                        f"{0x4201 if i > 100 else 0x4200}\n")
        engine = witnessos.RelayEngine(feed_port=0, log_dir=None)
        app = witnessos.WitnessOS(
            self.root, csv_file=csv_path,
            hitl_settings={"host": "127.0.0.1", "port": 1,
                           "preset": "Nominal 10 g", "tolerance_s": 1.0},
            hitl_settings_path=os.path.join(self.tmp, "s.json"),
            hitl_runs_dir=self.tmp, relay_engine=engine)
        self.pump()
        self.assertEqual(self.root.nametowidget(app.notebook.select()),
                         app.frames["viewer"])
        self.assertEqual(app.viewer.csv_path, csv_path)
        app.viewer.notebook.select(app.viewer.plot_tab)
        for size in ("1200x800", "500x350"):
            self.root.geometry(size)
            for key in witnessos.TABS:
                app.select(key)
                self.pump()
                self.assertTrue(self.root.title().startswith("witnessOS - "))
        app.select("viewer")
        app.viewer.draw_graphs()
        self.pump()
        self.assertTrue(app.viewer._drawn)
        self.assertGreater(len(app.viewer.canvas.find_all()), 10)
        self.assertIsNotNone(engine.publisher)
        app.relay.source_var.set("hitl")
        app.relay.toggle_connect()
        self.assertIs(app.hitl.session.rx_tap.__class__,
                      (lambda: None).__class__)
        app.close()
        self.assertIsNone(engine.publisher)
        self.assertIsNone(app.hitl.session.rx_tap)


if __name__ == "__main__":
    unittest.main()
