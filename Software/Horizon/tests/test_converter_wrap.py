"""32-bit microsecond unwrapping in the flash-log decoders (converter.py).

Run 20261111-200145: single IMU records logged a few ms out of order were
counted as counter wraps, adding 4294.967 s blocks to the time axis.
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import converter  # noqa: E402


def imu_record(us, sequence):
    return struct.pack(">I9hBB", us & 0xFFFFFFFF, *([0] * 9), 0, sequence & 0xFF)


def times(raw):
    data = b"".join(imu_record(us, i) for i, us in enumerate(raw))
    return [row["timestamp_us"] for row in converter.iter_imu_log(data)]


class WrapTests(unittest.TestCase):
    def test_small_backward_step_is_not_a_wrap(self):
        raw = [947_390_000 + 1042 * i for i in range(10)]
        raw[5] = raw[4] - 7_600          # one record 7.6 ms early (seen on hardware)
        t = times(raw)
        self.assertLess(max(t) - min(t), 20_000)
        self.assertEqual(t[-1] - t[0], raw[-1] - raw[0])

    def test_real_wrap_still_unwrapped(self):
        raw = [(2**32 - 3_000 + 1042 * i) % 2**32 for i in range(8)]
        t = times(raw)
        self.assertTrue(all(b > a for a, b in zip(t, t[1:])))
        self.assertEqual(t[-1] - t[0], 7 * 1042)

    def test_straggler_just_after_wrap(self):
        raw = [(2**32 - 3_000 + 1042 * i) % 2**32 for i in range(8)]
        raw.insert(5, (raw[4] - 2_000) % 2**32)   # logged before the wrap, written after
        t = times(raw)
        self.assertLess(max(t) - min(t), 20_000)
        self.assertEqual(t[-1] - t[0], 7 * 1042)


if __name__ == "__main__":
    unittest.main()
