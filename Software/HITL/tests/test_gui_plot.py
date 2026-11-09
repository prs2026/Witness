import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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


class EmbeddedGuiTests(unittest.TestCase):
    def test_embedded_app_does_not_own_root(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        import tempfile
        tmp = tempfile.mkdtemp()
        try:
            root.title("host")
            frame = tk.Frame(root)
            frame.pack(fill="both", expand=True)
            app = hitl_gui.HitlApp(
                root, settings={"host": "127.0.0.1", "port": 1,
                                "preset": "Nominal 10 g", "tolerance_s": 1.0},
                settings_path=os.path.join(tmp, "s.json"), runs_dir=tmp,
                parent=frame)
            root.geometry("900x600")
            root.update()
            self.assertEqual(root.title(), "host")
            app.show_banner("test warning " * 20)
            root.update()
            app.hide_banner()
            app._redraw_now()
            app.on_close()  # embedded: shuts down, root survives
            self.assertTrue(root.winfo_exists())
        finally:
            root.destroy()
            import shutil
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
