import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import command_link as cl  # noqa: E402
import witness_protocol as wp  # noqa: E402


class RadioPowerFrame(unittest.TestCase):
    def test_layout(self):
        f = wp.build_radio_power_packet("witness", -9)
        self.assertEqual(f, bytes((0x05, 0x00, 0xF7, 0, 0, 0, 0, 0x52, 0x50, 0x0A)))
        lp = wp.build_radio_power_packet("witness", -17, front_end=False)
        self.assertEqual(lp[1:4], bytes((0x00, 0xEF, 0x02)))
        self.assertEqual(wp.build_radio_power_packet("ground", 0, front_end=True)[3], 0x01)
        g = wp.build_radio_power_packet("ground", 3)
        self.assertEqual(g[1:3], bytes((0x01, 0x03)))
        self.assertTrue(cl.CommandLink.is_acked_command(f))

    def test_limits(self):
        for target, dbm in (("witness", 4), ("ground", -18), ("iris", 0)):
            with self.assertRaises(ValueError):
                wp.build_radio_power_packet(target, dbm)

    def test_matches_comms_h(self):
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        text = open(os.path.join(here, "comms.h")).read()
        self.assertIn("#define IRIS_COMMAND_RADIO_POWER 0x5250U", text)
        self.assertIn("#define IRIS_RADIO_POWER_TARGET_GROUND 0x01U", text)


if __name__ == "__main__":
    unittest.main()
