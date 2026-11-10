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

import csv  # noqa: E402

import fake_witness as fw  # noqa: E402
import hitl_gui  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import synthetic_csv as syn  # noqa: E402

BUNDLED = os.path.join(os.path.dirname(HERE), "trajectories",
                       "a3241569-trajectory.csv")
CSV_SPEED = "40"
HORIZON_ATT_HEADER = ["time_s", "packet_id", "witness_qx", "witness_qy",
                      "witness_qz", "witness_qw", "truth_qx", "truth_qy",
                      "truth_qz", "truth_qw", "flight_state", "hitl_active"]

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


class CsvEndToEndTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="hitl-csv-e2e-")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self, fake, path, extra=()):
        out = io.StringIO()
        argv = ["--headless", "--csv", path, "--host", "127.0.0.1",
                "--port", str(fake.port), "--speed", CSV_SPEED,
                "--settle", "0.5", "--tolerance", "2.0",
                "--runs-dir", self.tmp, *extra]
        with contextlib.redirect_stdout(out):
            code = hitl_gui.main(argv)
        return code, out.getvalue()

    def _run_dir(self):
        runs = os.listdir(self.tmp)
        self.assertEqual(len(runs), 1)
        return os.path.join(self.tmp, runs[0])

    def test_bundled_csv_with_attitude(self):
        fake = fw.FakeWitness(heading_offset_deg=37.0).start()
        try:
            code, text = self._run(fake, BUNDLED)
        finally:
            fake.close()
        self.assertEqual(code, 0, text)
        self.assertEqual([t[2] for t in fake.transitions],
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        # Safety: only ENTER, EXIT and one debug start were sent.
        self.assertTrue(set(fake.commands) <= {
            hp.CMD_HITL_ENTER, hp.CMD_HITL_EXIT, hp.CMD_WITNESS_DEBUG_START})
        self.assertEqual(fake.commands.count(hp.CMD_WITNESS_DEBUG_START), 1)
        self.assertEqual(fake.commands[0], hp.CMD_HITL_ENTER)
        self.assertEqual(fake.commands[-1], hp.CMD_HITL_EXIT)
        self.assertGreater(fake.debug_packets, 20)
        self.assertIn("MS5607", text)
        self.assertIn("Attitude PASS", text)
        rd = self._run_dir()
        with open(os.path.join(rd, "summary.json")) as f:
            summary = json.load(f)
        a = summary["attitude"]
        self.assertTrue(a["available"])
        self.assertTrue(a["passed"])
        self.assertAlmostEqual(a["heading_offset_deg"], 37.0, delta=1.0)
        self.assertLess(a["final"]["tilt_err_deg"], 2.0)
        self.assertLess(a["final"]["att_err_deg"], 2.0)
        with open(os.path.join(rd, "attitude.csv"), newline="") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0][:12], HORIZON_ATT_HEADER)
        self.assertGreater(len(rows), 20)
        self.assertIn(rows[1][1], ("2", "224"))

    def test_attitude_unavailable_is_warning(self):
        path = os.path.join(self.tmp, "hop.csv")
        syn.write(path, syn.simulate())
        runs = os.path.join(self.tmp, "runs")
        os.makedirs(runs)
        fake = fw.FakeWitness(attitude=False).start()
        try:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = hitl_gui.main([
                    "--headless", "--csv", path, "--host", "127.0.0.1",
                    "--port", str(fake.port), "--speed", CSV_SPEED,
                    "--settle", "0.5", "--tolerance", "2.0",
                    "--runs-dir", runs, "--set", "pad_hold_s=5",
                    "--set", "after_apogee_attitude=pendulum"])
            text = out.getvalue()
        finally:
            fake.close()
        self.assertEqual(code, 0, text)
        self.assertIn("attitude not available", text)
        self.assertEqual([t[2] for t in fake.transitions],
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        rd = os.path.join(runs, os.listdir(runs)[0])
        with open(os.path.join(rd, "attitude.csv"), newline="") as f:
            rows = list(csv.reader(f))
        self.assertTrue(rows[1:])
        self.assertTrue(all(r[2:6] == ["0.0000000"] * 4 for r in rows[1:]))

    def test_speed_refused_for_hardware_host(self):
        err = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()),                 contextlib.redirect_stderr(err):
            code = hitl_gui.main(["--headless", "--csv", BUNDLED,
                                  "--host", "prs2026hitl", "--speed", "5",
                                  "--no-save"])
        self.assertEqual(code, 2)
        self.assertIn("only allowed", err.getvalue())

    def test_dry_run(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = hitl_gui.main(["--headless", "--csv", BUNDLED,
                                  "--dry-run", "--no-save"])
        self.assertEqual(code, 0)
        self.assertIn("Expected: Boost", out.getvalue())


if __name__ == "__main__":
    unittest.main()
