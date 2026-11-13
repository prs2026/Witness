import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import witness_protocol as wp  # noqa: E402


class PadHeadingFrame(unittest.TestCase):
    def test_layout(self):
        f = wp.build_pad_heading_packet("+Y", 270.0)
        self.assertEqual(len(f), 10)
        self.assertEqual(f[0], 0x05)
        self.assertEqual(f[1], 0)                                   # +Y
        self.assertEqual(int.from_bytes(f[2:4], "big"), 27000)      # 0.01 deg
        self.assertEqual(f[4:7], bytes(3))
        self.assertEqual(int.from_bytes(f[7:9], "big"), 0x4844)
        self.assertEqual(f[9], 0x0A)

    def test_axes_wrap_and_clear(self):
        self.assertEqual(wp.build_pad_heading_packet("-Z", 360.0)[1:4], bytes((3, 0, 0)))
        self.assertEqual(wp.build_pad_heading_packet("-Y", -90.0)[1:4], bytes((1,)) + (27000).to_bytes(2, "big"))
        self.assertEqual(wp.build_pad_heading_packet(None)[1], 0xFF)
        with self.assertRaises(ValueError):
            wp.build_pad_heading_packet("+X", 10.0)


if __name__ == "__main__":
    unittest.main()
