"""End-to-end tests against the local fake Witness (127.0.0.1 only)."""

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import fake_witness as fw  # noqa: E402
import hitl_gui  # noqa: E402
import hitl_protocol as hp  # noqa: E402

# The fake clocks its state machine from 0x0C sim time, so the test can send
# 20 sim ticks per 10 ms wall tick (Quick hop: ~60 s sim in ~3 s).
SCALE = "20"


class EndToEndTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="hitl-test-")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self, fake, preset="Quick hop (test)", extra=()):
        out = io.StringIO()
        argv = ["--headless", "--preset", preset, "--host", "127.0.0.1",
                "--port", str(fake.port), "--time-scale", SCALE,
                "--settle", "0.5", "--runs-dir", self.tmp, *extra]
        with contextlib.redirect_stdout(out):
            code = hitl_gui.main(argv)
        return code, out.getvalue()

    def test_headless_pass(self):
        fake = fw.FakeWitness().start()
        try:
            code, text = self._run(fake, extra=["--tolerance", "1.0"])
        finally:
            fake.close()
        self.assertEqual(code, 0, text)
        self.assertIn("PASS", text)
        for name in ("Boost", "Coast", "Descent", "Landed"):
            self.assertRegex(text, name + r" .* OK")
        self.assertEqual([t[2] for t in fake.transitions],
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        # Safety: only ENTER and EXIT were ever sent.
        self.assertTrue(fake.commands)
        self.assertTrue(set(fake.commands) <= {hp.CMD_HITL_ENTER,
                                               hp.CMD_HITL_EXIT})
        self.assertEqual(fake.commands[0], hp.CMD_HITL_ENTER)
        self.assertEqual(fake.commands[-1], hp.CMD_HITL_EXIT)
        self.assertGreater(fake.hitl_frames, 5000)
        runs = os.listdir(self.tmp)
        self.assertEqual(len(runs), 1)
        rd = os.path.join(self.tmp, runs[0])
        for f in ("params.json", "sent.csv", "received.jsonl", "log.txt",
                  "transitions.csv", "summary.json"):
            self.assertTrue(os.path.exists(os.path.join(rd, f)), f)
        with open(os.path.join(rd, "summary.json")) as f:
            self.assertTrue(json.load(f)["passed"])
        with open(os.path.join(rd, "received.jsonl")) as f:
            names = {json.loads(line)["name"] for line in f}
        self.assertIn("state", names)
        self.assertNotIn("hitl_echo", names)

    def test_pad_abort_passes_with_no_transitions(self):
        fake = fw.FakeWitness().start()
        try:
            code, text = self._run(fake, preset="Pad abort - no launch")
        finally:
            fake.close()
        self.assertEqual(code, 0, text)
        self.assertEqual(fake.transitions, [])

    def test_no_hitl_bit_fails_with_warning(self):
        fake = fw.FakeWitness(set_hitl_bit=False).start()
        try:
            code, text = self._run(fake)
        finally:
            fake.close()
        self.assertNotEqual(code, 0, text)
        self.assertIn("HITL-active bit not seen", text)
        self.assertIn("FAIL", text)
        self.assertEqual(fake.commands[-1], hp.CMD_HITL_EXIT)

    def test_tight_tolerance_reports_late(self):
        # Fake reports Boost correctly; a negative-width window must fail.
        fake = fw.FakeWitness().start()
        try:
            code, text = self._run(fake, extra=["--tolerance", "-1"])
        finally:
            fake.close()
        self.assertEqual(code, 1, text)
        self.assertRegex(text, r"LATE|EARLY")

    def test_not_pad_idle_fails(self):
        fake = fw.FakeWitness(start_state=hp.LANDED).start()
        try:
            code, text = self._run(fake)
        finally:
            fake.close()
        self.assertEqual(code, 1, text)
        self.assertIn("not in Pad idle", text)

    def test_connection_refused(self):
        fake = fw.FakeWitness()
        port = fake.port
        fake.srv.close()
        out = io.StringIO()
        with contextlib.redirect_stdout(out), \
                contextlib.redirect_stderr(io.StringIO()):
            code = hitl_gui.main(["--headless", "--host", "127.0.0.1",
                                  "--port", str(port), "--no-save",
                                  "--connect-timeout", "1"])
        self.assertEqual(code, 2)


class GuiSmokeTest(unittest.TestCase):
    def test_build_gui(self):
        try:
            import tkinter as tk
            root = tk.Tk()
        except Exception as exc:  # pragma: no cover - environment specific
            self.skipTest(f"Tk unavailable: {exc}")
        tmp = tempfile.mkdtemp(prefix="hitl-gui-")
        try:
            root.withdraw()
            app = hitl_gui.HitlApp(
                root, settings={"host": "127.0.0.1", "port": 1,
                                "preset": "Nominal 10 g",
                                "tolerance_s": 1.0},
                settings_path=os.path.join(tmp, "s.json"), runs_dir=tmp)
            root.update()
            app.preset_var.set("Pad abort - no launch")
            app.load_preset()
            self.assertFalse(app.read_params().ignite)
            app.poll()
            app.redraw_plot()
            root.update()
            app.on_close()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
