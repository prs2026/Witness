"""RF GPS simulation: ephemeris encoding and upload frames, scenario files,
NMEA parsing, the RF run mode (0x0C flag, lead-in, ephemeris upload), the
vibration file overrides and the report PDF."""

import dataclasses
import datetime as dt
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent.parent / "Horizon"))

import gps_assist as ga  # noqa: E402
import gps_rf  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402

RINEX = gps_rf.GPS_SDR_SIM_SOURCE / "brdc0010.22n"


@unittest.skipUnless(RINEX.exists(), "gps-sdr-sim checkout (bundled RINEX) not found")
class Ephemeris(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gps-test-"))
        self.nav = ga.parse_rinex_nav(RINEX)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_parse_and_round_trip(self):
        self.assertGreater(len(self.nav.ephemerides), 300)
        e = self.nav.ephemerides[0]
        self.assertEqual(e.prn, 1)
        self.assertAlmostEqual(e.sqrta, 5153.67499542, places=6)
        self.assertEqual(e.week, 2190)
        start = dt.datetime(2026, 11, 10, 23, 20, 10)
        shifted, target = ga.shift_to(self.nav, start)
        self.assertEqual(target, dt.datetime(2026, 11, 10, 23, 0))
        path = ga.write_rinex2(shifted, self.tmp / "nav.n")
        back = ga.parse_rinex_nav(path)
        self.assertEqual(len(back.ephemerides), len(self.nav.ephemerides))
        b = back.ephemerides[0]
        self.assertEqual(b.toc, target)
        self.assertEqual(b.week, ga.gps_from_datetime(target)[0])
        for name in ("sqrta", "ecc", "m0", "omg0", "inc0", "aop", "af0", "af1", "idot", "omgdot"):
            self.assertAlmostEqual(getattr(b, name), getattr(e, name), delta=abs(getattr(e, name)) * 1e-11 + 1e-20)
        self.assertEqual(back.header["leap"], 18)

    def test_each_scenario_gets_new_issue_of_data(self):
        a, _ = ga.shift_to(self.nav, dt.datetime(2026, 11, 11, 5, 20))
        b, _ = ga.shift_to(self.nav, dt.datetime(2026, 11, 11, 6, 20))
        c, _ = ga.shift_to(self.nav, dt.datetime(2026, 11, 11, 20, 59))   # 15 h later
        keys = lambda nav: {(e.prn, int(e.iode)) for e in nav.ephemerides}  # noqa: E731
        self.assertFalse(keys(a) & keys(b))
        self.assertFalse(keys(a) & keys(c))
        for nav in (a, b):
            self.assertTrue(all(e.iode == e.iodc and 0 <= e.iode < 256 for e in nav.ephemerides))
        # Consecutive sets of one satellite differ.
        prn1 = sorted((e for e in a.ephemerides if e.prn == 1), key=lambda e: e.toc)
        self.assertEqual(len({e.iode for e in prn1}), len(prn1))
        # What is uploaded carries the same IODE as what is transmitted.
        e = ga.select_for(a, dt.datetime(2026, 11, 11, 5, 20))[1]
        d = ga.decode_skytraq_ephemeris(ga.skytraq_set_ephemeris(e))
        self.assertEqual((d["iode"], d["iodc"]), (int(e.iode), int(e.iode)))

    def test_skytraq_body_decodes_to_the_ephemeris(self):
        shifted, target = ga.shift_to(self.nav, dt.datetime(2026, 11, 10, 23, 20))
        selected = ga.select_for(shifted, target + dt.timedelta(minutes=20))
        self.assertGreaterEqual(len(selected), 24)
        for prn, e in list(selected.items())[:5]:
            body = ga.skytraq_set_ephemeris(e)
            self.assertEqual(len(body), 87)
            self.assertEqual(body[0], 0x41)
            self.assertEqual(body[3], 0)          # reserved byte per subframe
            d = ga.decode_skytraq_ephemeris(body)
            self.assertEqual(d["prn"], prn)
            self.assertEqual(d["subframe_ids"], [1, 2, 3])
            self.assertEqual(d["week"], e.week % 1024)
            self.assertEqual(d["iode"], int(e.iode))
            self.assertEqual(d["toe"], int(e.toe_sec / 16) * 16)
            self.assertAlmostEqual(d["sqrta"], e.sqrta, delta=2 ** -19)
            self.assertAlmostEqual(d["ecc"], e.ecc, delta=2 ** -33)
            self.assertAlmostEqual(d["m0"], e.m0, delta=2 ** -31 * ga.PI * 2)
            self.assertAlmostEqual(d["af0"], e.af0, delta=2 ** -31)

    def test_upload_frames(self):
        self.assertEqual(ga.crc16(b"123456789"), 0x29B1)
        shifted, target = ga.shift_to(self.nav, dt.datetime(2026, 11, 10, 23, 20))
        blob, prns = ga.ephemeris_upload(shifted, target)
        self.assertEqual(len(blob), len(prns) * 89)
        frames = ga.assist_frames(blob, transfer_id=5)
        self.assertEqual(len(frames), 2 + (len(blob) + 3) // 4)
        self.assertTrue(all(len(f) == 10 and f[0] == 5 and f[9] == 0x0A for f in frames))
        self.assertTrue(all(hp.is_assist_frame(f) for f in frames))
        self.assertEqual(frames[0][1:7], bytes((len(blob) >> 8, len(blob) & 0xFF,
                                                ga.crc16(blob) >> 8, ga.crc16(blob) & 0xFF, 1, 5)))
        # Reassemble from the DATA frames.
        rebuilt = bytearray(len(blob) + 4)
        for f in frames[1:-1]:
            off = (f[1] << 8) | f[2]
            rebuilt[off:off + 4] = f[3:7]
        self.assertEqual(bytes(rebuilt[:len(blob)]), blob)
        self.assertFalse(hp.is_assist_frame(hp.encode_command(hp.CMD_HITL_ENTER)))
        with self.assertRaises(ValueError):
            ga.build_blob([b"\x41" * 87] * 60)    # beyond the 4 KB transfer limit


class Nmea(unittest.TestCase):
    def test_gga_gsa(self):
        g = gps_rf.parse_gga("$GNGGA,123519.00,3259.418,N,10658.500,W,1,09,0.9,1400.2,M,-22.6,M,,*47")
        self.assertEqual(g["quality"], 1)
        self.assertEqual(g["sats"], 9)
        self.assertAlmostEqual(g["lat"], 32.99030, places=5)
        self.assertAlmostEqual(g["lon"], -106.975, places=5)
        self.assertEqual(g["geoid_sep"], -22.6)
        self.assertEqual(gps_rf.parse_gga("$GNGGA,123519,,,,,0,00,99.99,,,,,,*74")["quality"], 0)
        s = gps_rf.parse_gsa("$GNGSA,A,3,01,02,03,,,,,,,,,,1.8,1.0,1.5,1*00")
        self.assertEqual((s["fix_type"], s["pdop"], s["hdop"], s["vdop"]), (3, 1.8, 1.0, 1.5))
        self.assertEqual(s["sv"], [1, 2, 3])


class HackRFOutput(unittest.TestCase):
    def test_rate_lines_mark_streaming(self):
        self.assertTrue(gps_rf.RATE_LINE.search(
            "5.2 MB / 1.014 sec =  5.2 MB/second, average power -11.5 dBfs"))
        self.assertTrue(gps_rf.RATE_LINE.search("5.0 MiB / 1.000 sec =  5.0 MiB/second"))
        self.assertFalse(gps_rf.RATE_LINE.search("call hackrf_set_sample_rate(2600000 Hz/2.600 MHz)"))


class Scenarios(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gps-sc-"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_motion_rows(self):
        samples = sm.generate(sm.preset_params("Quick hop (test)"))
        rows = gps_rf.motion_rows(samples, 2.0, -20.0)
        self.assertEqual(rows[0][0], 0.0)
        self.assertEqual(rows[20][0], 2.0)            # lead-in is 20 rows at 10 Hz
        self.assertEqual(len(rows), 20 + len(samples[::10]))
        self.assertAlmostEqual(rows[0][3], samples[0].alt_msl_m - 20.0)
        self.assertAlmostEqual(rows[-1][0], 2.0 + samples[::10][-1].t_ms / 1000)

    @unittest.skipUnless(gps_rf.gps_sdr_sim_path() and RINEX.exists(), "gps-sdr-sim not built")
    def test_static_scenario_iq_contains_the_visible_satellites(self):
        sc = gps_rf.create_static(32.9903, -106.975, 1400.0, 4, base=self.tmp, rinex=RINEX,
                                  geoid_sep_m=-23.0, start_gps=dt.datetime(2026, 11, 10, 23, 20, 10))
        self.assertTrue(sc.iq.exists())
        self.assertAlmostEqual(sc.iq_bytes, 4 * gps_rf.SAMPLE_RATE * 2, delta=0.2 * gps_rf.SAMPLE_RATE * 2)
        self.assertGreaterEqual(len(sc.visible_prns), 6)
        r = gps_rf.verify_iq(sc.iq, sc.sample_rate, offset_s=1.0, ms=8)
        self.assertEqual(r["detected"], sc.visible_prns)
        loaded = gps_rf.Scenario.load(sc.folder)
        self.assertEqual(loaded.visible_prns, sc.visible_prns)
        frames, prns = gps_rf.ephemeris_frames(loaded)
        self.assertTrue(set(sc.visible_prns) <= set(prns))
        self.assertTrue(all(hp.is_assist_frame(f) for f in frames))


class FakePlayer:
    """Stands in for HackRFPlayer: streams immediately, never transmits."""
    instances = []

    def __init__(self, iq_path, **kwargs):
        self.kwargs = kwargs
        self.t0 = None
        self.stopped = False
        self.tx_vga_db = kwargs.get("tx_vga_db", 0)
        self.amp = kwargs.get("amp", False)
        FakePlayer.instances.append(self)

    def start(self):
        self.t0 = time.monotonic()

    def wait_streaming(self, timeout=10):
        return True

    def elapsed_s(self):
        return time.monotonic() - self.t0

    @property
    def running(self):
        return not self.stopped

    def output(self):
        return ["fake"]

    def stop(self):
        self.stopped = True


class RfRunMode(unittest.TestCase):
    def _scenario(self, folder, samples, **kw):
        first = samples[0].hitl
        base = dict(folder=Path(folder), kind="flight", start_gps="2026-11-10T23:20:10",
                    duration_s=1.0 + samples[-1].t_ms / 1000 + 0.5, lead_in_s=1.0,
                    lat_deg=first.lat_deg, lon_deg=first.lon_deg, alt_msl_m=samples[0].alt_msl_m,
                    geoid_sep_m=0.0, sample_rate=gps_rf.SAMPLE_RATE, prns=[1, 2])
        base.update(kw)
        return gps_rf.Scenario(**base)

    def test_prepare_marks_frames_and_checks_the_scenario(self):
        session = hs.HitlSession(runs_dir=None)
        params = sm.preset_params("Quick hop (test)")
        run = session.prepare_run(params)
        self.assertTrue(all(f[1] == 0 for f in run.frames))
        sc = self._scenario("x", run.samples)
        rf = session.prepare_run(params, gps={"scenario": sc, "tx_vga_db": 3})
        self.assertEqual(rf.gps_mode, "rf")
        self.assertEqual(rf.gps_tx["tx_vga_db"], 3)
        self.assertTrue(all(f[1] == hp.HITL_STATUS_GPS_FROM_RECEIVER for f in rf.frames))
        with self.assertRaises(ValueError):          # too short for lead-in + run
            session.prepare_run(params, gps={"scenario": self._scenario("x", run.samples, duration_s=5)})
        with self.assertRaises(ValueError):          # another launch site
            session.prepare_run(params, gps={"scenario": self._scenario("x", run.samples, lat_deg=10.0)})
        with self.assertRaises(ValueError):          # a static scenario
            session.prepare_run(params, gps={"scenario": self._scenario("x", run.samples, kind="static")})
        with self.assertRaises(ValueError):
            hp.encode_hitl(run.samples[0].hitl, 0x80)

    def test_lead_in_uploads_ephemeris_then_waits(self):
        session = hs.HitlSession(runs_dir=None)
        sent = []
        session._send = lambda data: sent.append(bytes(data))
        params = sm.preset_params("Quick hop (test)")
        run = session.prepare_run(params)
        sc = self._scenario("x", run.samples, lead_in_s=1.2)
        session._prepare_gps(run, {"scenario": sc, "upload_ephemeris": True})
        session.view.gps_locked = True
        session.view.gps_sats = 9
        upload = [ga.command_frame(ga.CMD_BEGIN, b"\0\4\0\0\1\1")]
        with patch.object(gps_rf, "HackRFPlayer", FakePlayer), \
                patch.object(gps_rf, "ephemeris_frames", return_value=(upload, [3, 7])):
            t = time.perf_counter()
            t0 = session._gps_lead_in(run)
        self.assertAlmostEqual(t0 - t, 1.2, delta=0.15)       # sim t = 0 after the lead-in
        self.assertGreaterEqual(time.perf_counter() - t, 0.6)  # waited (minus 0.5 s margin)
        self.assertEqual(sent, upload)
        self.assertEqual(run.gps_report["ephemeris_prns"], [3, 7])
        self.assertIsNotNone(run.gps_report["first_fix_s"])
        with self.assertRaises(ValueError):
            session.upload_assist([hp.encode_command(hp.CMD_HITL_ENTER)])


class ColdStart(unittest.TestCase):
    def test_waits_for_the_reconfiguration_line(self):
        session = hs.HitlSession(runs_dir=None)
        sent = []

        def fake_send(data):
            sent.append(bytes(data))
            threading.Timer(0.2, session._on_text, args=(
                "I (100) uart: PX1120S airborne 4 g mode and 25 Hz GGA+RMC confirmed at 115200 baud",)).start()
        session._send = fake_send
        self.assertTrue(session.cold_start_gps(timeout=3))
        self.assertEqual(sent, [hp.encode_command(hp.CMD_GPS_COLD_START)])
        self.assertEqual(sent[0][7:9], bytes((0x47, 0x52)))
        session._send = lambda data: None
        self.assertFalse(session.cold_start_gps(timeout=0.3))


class VibrationFiles(unittest.TestCase):
    def test_files_override_the_profile(self):
        import imu_synth as ims
        folder = HERE.parent / "vibration" / "eayftrajectory"
        thrust = str(folder / "vibration_flight_thrust.ogg")
        lateral = str(folder / "vibration_flight_lateral.ogg")
        p = ims.resolve_profile("whatever.csv", "none", thrust, lateral, 100.0)
        self.assertEqual(p["thrust_path"], thrust)
        self.assertEqual(p["lateral_path"], lateral)
        self.assertEqual(p["full_scale_g"], 100.0)
        p = ims.resolve_profile("whatever.csv", "", thrust)
        self.assertEqual(p["lateral_path"], thrust)
        self.assertEqual(p["full_scale_g"], ims.DEFAULT_FILE_FULL_SCALE_G)
        with self.assertRaises(ValueError):
            ims.resolve_profile("x.csv", "", str(folder / "missing.ogg"))
        self.assertIsNone(ims.resolve_profile("x.csv", "none"))


@unittest.skipUnless((HERE.parent / "runs").exists(), "no saved runs")
class Report(unittest.TestCase):
    def test_report_from_a_saved_run_layout(self):
        import hitl_report
        tmp = Path(tempfile.mkdtemp(prefix="hitl-report-"))
        try:
            run = tmp / "run"
            run.mkdir()
            samples = sm.generate(sm.preset_params("Quick hop (test)"))
            with open(run / "truth.csv", "w") as f:
                f.write("sim_s,expected_state,east_m,north_m,up_m,alt_msl_m,speed_mps\n")
                for s in samples[::10]:
                    f.write(f"{s.t_ms / 1000},{'Pad idle'},0,0,0,{s.alt_msl_m},{abs(s.vel_mps)}\n")
            with open(run / "nmea.txt", "w") as f:
                for k, s in enumerate(samples[::10]):
                    f.write(f"{100 + k * 0.1:.3f} sim= {s.t_ms / 1000:7.2f} "
                            f"$GNGGA,000000,3259.418,N,10658.500,W,1,09,0.9,{s.alt_msl_m:.1f},M,-22.6,M,,*00\n")
                    if k % 5 == 0:
                        f.write(f"{100 + k * 0.1:.3f} sim= {s.t_ms / 1000:7.2f} "
                                "$GNGSA,A,3,01,02,03,04,,,,,,,,,1.8,0.9,1.5,1*00\n")
            (run / "summary.json").write_text(json.dumps({
                "passed": True, "rows": [], "gps": {"mode": "rf", "scenario": "x", "lead_in_s": 90}}))
            (run / "params.json").write_text(json.dumps({
                "preset": "Quick hop (test)", "params": {"lat_deg": 32.9903, "lon_deg": -106.975}}))
            path = hitl_report.build_report(run)
            self.assertTrue(path.read_bytes().startswith(b"%PDF"))
            nmea = hitl_report.parse_nmea_file(run / "nmea.txt")
            self.assertEqual(len(nmea["gga"]), len(samples[::10]))
            g = hitl_report.gnss_series(nmea, hitl_report._read_csv(run / "truth.csv"),
                                        json.loads((run / "params.json").read_text()), [], True)
            self.assertTrue(any("availability" in line for line in g["summary"]))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
