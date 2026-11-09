"""Tests for the desktop 960 Hz IMU synthesis and the 0x0E frame."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hitl_outputs as ho  # noqa: E402
import hitl_protocol as hp  # noqa: E402

try:
    import numpy as np
    import imu_synth as ims
    MISSING = ims.available()
except ImportError as exc:  # pragma: no cover
    MISSING = str(exc)


class FrameTests(unittest.TestCase):
    def test_imu_batch_layout(self):
        low = [[i, -i, 2 * i] for i in range(8)]
        high = [[100 + i, 0, -1] for i in range(8)]
        gyro = [[-32768, 32767, i] for i in range(8)]
        f = hp.encode_imu_batch(123456, low, high, gyro)
        self.assertEqual(len(f), 150)
        self.assertEqual(f[0], 0x0E)
        self.assertEqual(f[-1], 0x0A)
        self.assertEqual(struct.unpack_from(">I", f, 1)[0], 123456)
        # Sample 3: low xyz, high xyz, gyro xyz.
        vals = struct.unpack_from(">9h", f, 5 + 18 * 3)
        self.assertEqual(vals, (3, -3, 6, 103, 0, -1, -32768, 32767, 3))

    def test_echo_is_skipped_by_decoder(self):
        f = hp.encode_imu_batch(0, [[0, 0, 0]] * 8, [[0, 0, 0]] * 8, [[0, 0, 0]] * 8)
        events = hp.StreamDecoder().feed(b"I (1) x: y\r\n" + f + b"I (2) z: w\r\n")
        texts = [v for k, v in events if k == "text"]
        names = [v["name"] for k, v in events if k == "packet"]
        self.assertEqual(texts, ["I (1) x: y", "I (2) z: w"])
        self.assertEqual(names, ["hitl_imu_echo"])


@unittest.skipIf(MISSING, f"IMU synthesis unavailable: {MISSING}")
class SynthesisTests(unittest.TestCase):
    def still(self, seconds=6.0, **kw):
        t = np.arange(0, seconds, 0.01)
        f = np.tile([1.0, 0.0, 0.0], (len(t), 1))
        w = np.zeros((len(t), 3))
        return ims.synthesize(t, f, w, None, ims.ImuSynthParams(**kw))

    def test_rate_and_noise_match_datasheet(self):
        s = self.still(offset_scale=0.0)
        self.assertEqual(len(s), int((5.99 * 7680 + 1) // 8))
        self.assertAlmostEqual(float(np.median(np.diff(s.time_s))), 1 / 960, places=9)
        low = s.low_g[200:] * ims.LOW_G_PER_COUNT
        gyro = s.gyro[200:] * ims.GYRO_DPS_PER_COUNT
        # White noise through a 2nd-order Butterworth: ENBW ~ 1.11 fc.
        exp_low = ims.LOW_G_NOISE_DENSITY * np.sqrt(1.11 * 384.0)
        exp_gyro = ims.GYRO_NOISE_DENSITY * np.sqrt(1.11 * 407.0)
        self.assertAlmostEqual(float(low[:, 1].std()), exp_low, delta=0.15 * exp_low)
        self.assertAlmostEqual(float(gyro[:, 0].std()), exp_gyro, delta=0.2 * exp_gyro)
        self.assertAlmostEqual(float(low[:, 0].mean()), 1.0, delta=0.002)

    def test_worst_case_offsets_applied(self):
        s = self.still(noise_scale=0.0)
        low = s.low_g.astype(float) * ims.LOW_G_PER_COUNT
        gyro = s.gyro.astype(float) * ims.GYRO_DPS_PER_COUNT
        for axis in range(3):
            self.assertAlmostEqual(abs(low[-1, axis] - (1.0 if axis == 0 else 0.0)), 0.012, delta=0.0006)
            self.assertAlmostEqual(abs(gyro[-1, axis]), 1.0, delta=0.071)
        self.assertEqual(s.report["low_g_offset_g"], [o for o in s.report["low_g_offset_g"]])

    def test_adc_clips_before_the_filter(self):
        t = np.arange(0, 2.0, 0.01)
        f = np.tile([30.0, 0.0, 0.0], (len(t), 1))
        s = ims.synthesize(t, f, np.zeros((len(t), 3)), None,
                           ims.ImuSynthParams(noise_scale=0.0, offset_scale=0.0))
        self.assertEqual(int(s.low_g[-1, 0]), 32767)  # 16 g is past int16: 15.99 g
        self.assertAlmostEqual(s.high_g[-1, 0] * ims.HIGH_G_PER_COUNT, 30.0, delta=0.02)
        self.assertGreater(s.report["raw_clipped_fraction"]["low_g"][0], 0.99)

    def test_vibration_starts_at_launch(self):
        t = np.arange(0, 8.0, 0.01)
        f = np.tile([1.0, 0.0, 0.0], (len(t), 1))
        s = ims.synthesize(t, f, np.zeros((len(t), 3)), 4.0,
                           ims.ImuSynthParams(noise_scale=0.0, offset_scale=0.0))
        hg = s.high_g * ims.HIGH_G_PER_COUNT
        before = s.time_s < 3.9
        after = s.time_s > 4.5
        self.assertLess(float(np.abs(hg[before, 1]).max()), 0.02)
        self.assertGreater(float(hg[after, 1].std()), 0.5)
        self.assertEqual(s.report["vibration_start_s"], 4.0)


class ComparisonSummaryTests(unittest.TestCase):
    def test_summary_by_phase(self):
        rows = [
            {"expected_state": "Boost", "error_up_m": 3.0, "error_horizontal_m": 4.0,
             "error_speed_mps": 1.0, "error_attitude_deg": 2.0},
            {"expected_state": "Boost", "error_up_m": -4.0, "error_horizontal_m": 0.0,
             "error_speed_mps": -1.0, "error_attitude_deg": None},
            {"expected_state": "Coast", "error_up_m": 1.0, "error_horizontal_m": 1.0,
             "error_speed_mps": 0.0, "error_attitude_deg": 0.5},
        ]
        s = ho.summarize(rows)
        self.assertEqual(s["Boost"]["packets"], 2)
        self.assertAlmostEqual(s["Boost"]["up_rms_m"], (12.5) ** 0.5)
        self.assertEqual(s["Boost"]["up_max_m"], 4.0)
        self.assertEqual(s["Boost"]["attitude_rms_deg"], 2.0)
        self.assertEqual(s["Coast"]["horizontal_max_m"], 1.0)


if __name__ == "__main__":
    unittest.main()
