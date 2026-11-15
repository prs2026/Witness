import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hitl_protocol as hp  # noqa: E402
import sim_model as sm  # noqa: E402


class SimModelTests(unittest.TestCase):
    def test_nominal_transitions_in_order(self):
        p = sm.preset_params("Nominal 10 g")
        samples = sm.generate(p)
        exp = sm.predict_transitions(samples)
        self.assertEqual(sorted(exp, key=exp.get),
                         [hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED])
        # Launch qualifies 0.3 s after the vehicle passes 30 m AGL, burnout
        # ~0.5 s after burn.
        site = samples[0].alt_msl_m
        above = next(s.t_ms / 1000 for s in samples if s.alt_msl_m - site > 30.0)
        self.assertAlmostEqual(exp[hp.BOOST], above + 0.3, delta=0.05)
        self.assertAlmostEqual(exp[hp.COAST],
                               p.pad_time_s + p.burn_time_s + 0.5, delta=0.15)
        summ = sm.summarize(samples)
        self.assertGreater(summ["apogee_agl_m"], 500)
        self.assertLess(exp[hp.COAST], summ["apogee_t_s"])
        self.assertGreater(exp[hp.DESCENT], summ["apogee_t_s"])
        touchdown = next(s.t_ms for s in samples if s.phase == "landed")
        self.assertAlmostEqual(exp[hp.LANDED], touchdown / 1000 + 30,
                               delta=1.0)

    def test_pad_abort_has_no_transitions(self):
        samples = sm.generate(sm.preset_params("Pad abort - no launch"))
        self.assertEqual(sm.predict_transitions(samples), {})
        self.assertTrue(all(s.phase == "pad" for s in samples))
        for s in samples[::100]:
            self.assertAlmostEqual(s.hitl.low_g[0], 1.0, delta=0.05)
        self.assertIs(sm.preset_params("Pad abort — no launch").ignite,
                      False)

    def test_all_presets_sane(self):
        for name in sm.PRESETS:
            with self.subTest(name):
                samples = sm.generate(sm.preset_params(name))
                exp = sm.predict_transitions(samples)
                order = sorted(exp, key=exp.get)
                self.assertEqual(order, sorted(order))
                for s in samples[:50]:
                    f = hp.encode_hitl(s.hitl)
                    self.assertEqual(len(f), 47)
                    self.assertEqual(f[-1], 0x0A)

    def test_deterministic_with_seed(self):
        a = sm.generate(sm.preset_params("Quick hop (test)"))
        b = sm.generate(sm.preset_params("Quick hop (test)"))
        self.assertEqual([hp.encode_hitl(s.hitl) for s in a],
                         [hp.encode_hitl(s.hitl) for s in b])
        c = sm.generate(sm.SimParams(**{**sm.PRESETS["Quick hop (test)"],
                                        "seed": 2}))
        self.assertNotEqual(hp.encode_hitl(a[5].hitl),
                            hp.encode_hitl(c[5].hitl))

    def test_pad_conditions_and_pressure(self):
        p = sm.preset_params("Nominal 10 g")
        s = sm.FlightSim(p).step()
        self.assertAlmostEqual(s.hitl.low_g[0], 1.0, delta=0.02)
        alt = hp.pressure_altitude_m(s.hitl.pressure_pa)
        self.assertAlmostEqual(alt, p.site_alt_msl_m, delta=2.0)
        self.assertEqual(s.hitl.fix_quality, 1)

    def test_gps_dropout(self):
        p = sm.preset_params("GPS dropout")
        samples = sm.generate(p)
        mid = [s for s in samples
               if p.gps_dropout_start_s <= s.t_ms / 1000 < p.gps_dropout_end_s]
        self.assertTrue(mid)
        self.assertTrue(all(s.hitl.fix_quality == 0 and s.hitl.lat_deg == 0
                            for s in mid))
        after = samples[int(p.gps_dropout_end_s * 100) + 5]
        self.assertEqual(after.hitl.fix_quality, 1)

    def test_reference_fsm_launch_needs_300ms_above_30m(self):
        fsm = sm.ReferenceStateMachine()
        for i in range(1, 100):  # 29 m AGL: never
            fsm.update(i * 10_000, 11.0, [11, 0, 0], [0, 0, 0], 29.0, 50.0)
        self.assertEqual(fsm.state, hp.PAD_IDLE)
        for i in range(100, 130):  # 290 ms above 30 m
            fsm.update(i * 10_000, 11.0, [11, 0, 0], [0, 0, 0], 31.0, 50.0)
        self.assertEqual(fsm.state, hp.PAD_IDLE)
        fsm.update(130 * 10_000, 11.0, [11, 0, 0], [0, 0, 0], 31.0, 50.0)
        self.assertEqual(fsm.state, hp.BOOST)


if __name__ == "__main__":
    unittest.main()
