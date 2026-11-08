"""CSV trajectory conversion and attitude-metric tests (stdlib only)."""

import math
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import attitude as att  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402
import synthetic_csv as syn  # noqa: E402
import trajectory_csv as tc  # noqa: E402

BUNDLED = os.path.join(ROOT, "trajectories", "a3241569-trajectory.csv")
QUIET = dict(accel_noise_g=0.0, high_g_noise_g=0.0, gyro_noise_dps=0.0,
             baro_noise_pa=0.0)


def integrate_sent_gyro(samples):
    """Integrate the gyro as decoded from the 0x0C frames (q*exp(w dt))."""
    q = samples[0].q_truth
    worst = 0.0
    for s in samples[1:]:
        g = hp.decode_hitl(hp.encode_hitl(s.hitl))["gyro_dps"]
        rv = tuple(math.radians(c) * sm.DT_S for c in g)
        q = att.qnormalize(att.qmul(q, att.from_rotvec(rv)))
        worst = max(worst, math.degrees(att.angle_between(q, s.q_truth)))
    return q, worst


class SyntheticBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="hitl-csv-")
        cls.rows = syn.simulate()
        cls.path_ft = os.path.join(cls.tmp, "hop_ft.csv")
        cls.path_m = os.path.join(cls.tmp, "hop_m.csv")
        cls.path_novel = os.path.join(cls.tmp, "hop_novel.csv")
        syn.write(cls.path_ft, cls.rows, units="ft")
        syn.write(cls.path_m, cls.rows, units="m")
        syn.write(cls.path_novel, cls.rows, units="m", velocities=False)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)


class ParseTests(SyntheticBase):
    def test_units_duplicates_nans(self):
        tf = tc.load_trajectory(self.path_ft)
        tm = tc.load_trajectory(self.path_m)
        self.assertGreaterEqual(tf.duplicates_merged, 3)
        self.assertEqual(len(tf.t), len(set(tf.t)))
        self.assertTrue(all(b > a for a, b in zip(tf.t, tf.t[1:])))
        apo_f = max(tf.up)
        apo_m = max(tm.up)
        true_apo = max(r[3] for r in self.rows)
        self.assertAlmostEqual(apo_f, true_apo, delta=true_apo * 1e-4)
        self.assertAlmostEqual(apo_m, true_apo, delta=true_apo * 1e-4)
        self.assertAlmostEqual(max(tf.vu), max(r[6] for r in self.rows),
                               delta=0.1)
        # rev/s -> rad/s
        self.assertAlmostEqual(max(tf.roll_rate), 0.1 * 2 * math.pi,
                               places=4)
        # angle-of-attack column turns nan after apogee
        t_apo = self.rows[max(range(len(self.rows)),
                              key=lambda k: self.rows[k][3])][0]
        self.assertIsNotNone(tf.aero_nan_t)
        self.assertAlmostEqual(tf.aero_nan_t, t_apo, delta=0.5)
        self.assertEqual(tf.derived, [])

    def test_missing_velocities_are_derived(self):
        tr = tc.load_trajectory(self.path_novel)
        self.assertTrue(any(d.startswith("vu") for d in tr.derived))
        ref = tc.load_trajectory(self.path_m)
        # compare derived vs given vertical velocity mid-coast
        i = next(k for k, t in enumerate(ref.t) if t > 10.0)
        j = next(k for k, t in enumerate(tr.t) if t >= ref.t[i])
        self.assertAlmostEqual(tr.vu[j], ref.vu[i], delta=1.0)

    def test_header_errors(self):
        bad = os.path.join(self.tmp, "bad.csv")
        with open(bad, "w") as f:
            f.write("time [s],east [ft]\n0,0\n1,1\n")
        with self.assertRaises(ValueError):
            tc.load_trajectory(bad)
        with open(bad, "w") as f:
            f.write("time [s],altitude [furlong]\n0,0\n1,1\n")
        with self.assertRaisesRegex(ValueError, "unsupported unit"):
            tc.load_trajectory(bad)

    def test_bundled_csv_facts(self):
        tr = tc.load_trajectory(BUNDLED)
        self.assertEqual(tr.rows, 3595)
        self.assertEqual(tr.duplicates_merged, 4)
        self.assertAlmostEqual(max(tr.up) / syn.FT, 124564, delta=1)
        self.assertAlmostEqual(tr.aero_nan_t, 91.62, places=2)
        self.assertAlmostEqual(tr.t_end, 376.051, places=2)


class ConversionTests(SyntheticBase):
    def build(self, **kw):
        args = dict(QUIET)
        args.update(kw)
        return tc.build(tc.CsvParams(csv_path=self.path_ft, **args))

    def test_timeline_exact_100hz(self):
        b = self.build(pad_hold_s=5.0, post_landing_s=10.0, touchdown_s=0.2)
        ts = [s.t_ms for s in b.samples]
        self.assertEqual(ts, list(range(0, 10 * len(ts), 10)))
        tr = b.trajectory
        expect = 5.0 + (tr.t_end - tr.t[0]) + 0.2 + 10.0
        self.assertAlmostEqual(len(ts) * 0.01, expect, delta=0.02)
        self.assertEqual(b.samples[0].alt_msl_m, 1400.0)
        self.assertEqual(b.samples[-1].phase, "landed")

    def test_specific_force_signs(self):
        b = self.build(pad_hold_s=5.0)
        pad = b.samples[100].hitl
        self.assertAlmostEqual(pad.low_g[0], 1.0, places=6)
        self.assertAlmostEqual(abs(pad.low_g[1]) + abs(pad.low_g[2]), 0.0,
                               places=6)
        # mid burn: thrust 15 g along the nose
        burn = b.samples[int((5.0 + 1.0) / 0.01)].hitl
        self.assertGreater(burn.high_g[0], 10.0)
        self.assertAlmostEqual(burn.high_g[0], 15.0, delta=0.2)
        self.assertLess(abs(burn.high_g[1]) + abs(burn.high_g[2]), 0.2)
        # drag-free coast: free fall reads ~0 g
        coast = b.samples[int((5.0 + 8.0) / 0.01)].hitl
        self.assertLess(math.sqrt(sum(c * c for c in coast.low_g)), 0.01)
        # steady descent: 1 g magnitude
        desc = b.samples[int((5.0 + 60.0) / 0.01)].hitl
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in desc.low_g)),
                               1.0, delta=0.01)
        # post landing: stationary 1 g, zero gyro
        last = b.samples[-1].hitl
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in last.low_g)),
                               1.0, delta=0.001)
        self.assertEqual(tuple(last.gyro_dps), (0.0, 0.0, 0.0))

    def test_truth_attitude_follows_velocity_then_holds(self):
        b = self.build(pad_hold_s=5.0)
        s = b.samples[int((5.0 + 1.0) / 0.01)]
        self.assertAlmostEqual(att.tilt_from_vertical_deg(s.q_truth), 5.0,
                               delta=0.3)
        hold = b.report["timeline"]["attitude_hold_sim_s"]
        qa = b.samples[int(hold / 0.01) + 50].q_truth
        qb = b.samples[-1].q_truth
        self.assertLess(math.degrees(att.angle_between(qa, qb)), 1e-6)
        # pad attitude for a vertical rail = Horizon's shortest-arc reference
        q0 = b.samples[0].q_truth
        for c, r in zip(q0, (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))):
            self.assertAlmostEqual(c, r, places=9)

    def test_gyro_integrates_to_truth(self):
        for mode in ("hold", "pendulum"):
            b = self.build(pad_hold_s=2.0, post_landing_s=5.0,
                           after_apogee_attitude=mode)
            q, worst = integrate_sent_gyro(b.samples)
            self.assertLess(worst, 0.1, mode)

    def test_gyro_with_noise_stays_close(self):
        b = self.build(pad_hold_s=2.0, post_landing_s=5.0,
                       gyro_noise_dps=0.05)
        _q, worst = integrate_sent_gyro(b.samples)
        self.assertLess(worst, 1.0)

    def test_expected_transitions(self):
        b = self.build(pad_hold_s=5.0, accel_noise_g=0.003)
        track = sm.predict_state_track(b.samples)
        exp = hs.expected_from_track(b.samples, track)
        self.assertEqual(sorted(exp, key=exp.get),
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        self.assertAlmostEqual(exp[hp.BOOST], 5.3, delta=0.15)
        ref = sm.predict_transitions(b.samples)
        self.assertEqual(set(exp), set(ref))
        for k in exp:
            self.assertAlmostEqual(exp[k], ref[k], places=6)
        td = b.report["timeline"]["touchdown_sim_s"]
        self.assertAlmostEqual(exp[hp.LANDED], td + 30.0, delta=1.0)


class BundledConversionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = tc.build(tc.CsvParams(csv_path=BUNDLED))

    def test_report(self):
        r = self.b.report
        self.assertTrue(r["sensors"]["low_g_saturates"])
        self.assertFalse(r["sensors"]["gyro_saturates"])
        self.assertFalse(r["sensors"]["high_g_saturates"])
        self.assertGreater(r["sensors"]["max_specific_force_mag_g"], 16)
        self.assertTrue(r["baro"]["below_ms5607_range_windows_sim_s"])
        ed = r["edges"]
        self.assertAlmostEqual(ed["sent_high_g_x_above_10g_sim_s"],
                               ed["csv_thrust_10g_sim_s"], delta=0.05)
        for a, b in zip(ed["sent_low_g_x_below_1g_sim_s"],
                        ed["csv_burnout_1g_sim_s"]):
            self.assertAlmostEqual(a, b, delta=0.05)
        # every sent value respects the sensor limits
        for s in self.b.samples[::7]:
            self.assertTrue(all(abs(v) <= 16.0 for v in s.hitl.low_g))
            self.assertTrue(all(abs(v) <= 2000.0 for v in s.hitl.gyro_dps))

    def test_transitions_no_reboost(self):
        track = sm.predict_state_track(self.b.samples)
        exp = hs.expected_from_track(self.b.samples, track)
        self.assertEqual(sorted(exp, key=exp.get),
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        self.assertAlmostEqual(exp[hp.BOOST], 20.3, delta=0.1)
        self.assertLess(exp[hp.COAST], 25.0)          # first burnout
        changes = sum(1 for a, b in zip(track, track[1:]) if a != b)
        self.assertEqual(changes, 4)                  # no re-entry to Boost
        apo = self.b.report["timeline"]["attitude_hold_sim_s"]
        self.assertGreater(exp[hp.DESCENT], apo)
        td = self.b.report["timeline"]["touchdown_sim_s"]
        self.assertAlmostEqual(exp[hp.LANDED], td + 30.0, delta=1.0)

    def test_gyro_integrates_to_truth(self):
        _q, worst = integrate_sent_gyro(self.b.samples)
        self.assertLess(worst, 1.0)


class IsaTests(unittest.TestCase):
    def test_troposphere_matches_sim_model(self):
        # sim_model uses the altitude directly; isa() converts geometric to
        # geopotential altitude first.
        for z in (0.0, 1400.0, 8000.0):
            h = 6356766.0 * z / (6356766.0 + z)
            self.assertAlmostEqual(tc.isa(z)[0], sm.isa_pressure_pa(h),
                                   delta=0.5)

    def test_ms5607_limit_altitude(self):
        p, t = tc.isa(31000.0)
        self.assertAlmostEqual(p, 1000.0, delta=120.0)
        self.assertAlmostEqual(tc.isa(11000.0)[1], 216.65, delta=0.5)


class AttitudeMetricTests(unittest.TestCase):
    def setUp(self):
        tilt = att.from_axis_angle((0.3, 1.0, 0.0), math.radians(20))
        self.qt = att.qmul(tilt, att.shortest_arc(att.EX, att.EZ))
        self.qt = att.qmul(self.qt, att.from_axis_angle(att.EX, 0.7))

    def test_known_yaw_offset_removed(self):
        psi = math.radians(123.0)
        qw = att.qmul(att.yaw_quat(psi), self.qt)
        est = att.heading_offset_rad(self.qt, qw)
        self.assertAlmostEqual(att.wrap_deg(math.degrees(est)), 123.0,
                               places=6)
        self.assertLess(att.attitude_error_deg(self.qt, qw, est), 1e-6)
        self.assertLess(att.nose_error_deg(self.qt, qw, est), 1e-5)
        self.assertLess(att.tilt_deg_error(self.qt, qw), 1e-6)
        # negated quaternion is the same attitude
        neg = tuple(-c for c in qw)
        self.assertLess(att.attitude_error_deg(self.qt, neg, est), 1e-6)

    def test_real_error_survives_heading_removal(self):
        psi = math.radians(-40.0)
        err = att.from_axis_angle((0.0, 1.0, 0.0), math.radians(3.0))
        qw = att.qmul(att.yaw_quat(psi), att.qmul(self.qt, err))
        self.assertAlmostEqual(att.attitude_error_deg(self.qt, qw, psi), 3.0,
                               places=6)

    def test_summary_unavailable_is_warning(self):
        recs = [{"sim_s": 1.0, "phase": "Pad idle", "valid": False,
                 "q_truth": att.IDENTITY, "q_witness": (0, 0, 0, 0),
                 "tilt_err_deg": None}]
        s = att.summarize(recs, None)
        self.assertFalse(s["available"])
        self.assertTrue(s["passed"])
        self.assertIn("attitude not available", s["warnings"][0])
        self.assertFalse(att.is_valid((0.0, 0.0, 0.0, 0.0)))

    def test_summary_threshold(self):
        qw = att.qmul(self.qt, att.from_axis_angle((0, 1, 0),
                                                   math.radians(8)))
        recs = [{"sim_s": 1.0, "phase": "Landed", "valid": True,
                 "q_truth": self.qt, "q_witness": qw,
                 "tilt_err_deg": att.tilt_deg_error(self.qt, qw)}]
        att.compute_records(recs, 0.0)
        self.assertFalse(att.summarize(recs, 0.0)["passed"])
        self.assertTrue(att.summarize(
            recs, 0.0, {"final_tilt_max_deg": 20.0})["passed"])


if __name__ == "__main__":
    unittest.main()
