"""The Witness pad estimate updates only in Pad Idle and freezes for the flight."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pad_reference import PadReference  # noqa: E402

PAD_IDLE_LOCKED = 0x0008
BOOST_LOCKED = 0x0009
LANDED_LOCKED = 0x000C


def state(status, lat, lon, uptime, alt=609.0):
    return {"packet": {"packet": "state", "status": status, "uptime": uptime,
                       "latitude": lat, "longitude": lon, "gps_altitude": alt},
            "received_mono": uptime / 1000.0}


class WitnessPad(unittest.TestCase):
    def test_pad_freezes_in_flight_and_after_landing(self):
        pad = PadReference()
        for k in range(10):
            pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.3470, -117.8090, 1000 + k * 100))
        start = pad.position()
        self.assertEqual(start.source, "witness")
        self.assertAlmostEqual(start.lat, 35.3470)
        # Flight: positions far from the pad never move it.
        for k in range(50):
            pad.accept_record("witness", state(BOOST_LOCKED, 35.3470 + k * 0.001, -117.80, 3000 + k * 100))
        self.assertTrue(pad.witness_frozen)
        # Landed somewhere else, still the same boot: frozen.
        pad.accept_record("witness", state(LANDED_LOCKED, 35.40, -117.70, 9000))
        # A Pad Idle packet without a reboot (e.g. a commanded state) stays frozen.
        pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.40, -117.70, 9500))
        self.assertAlmostEqual(pad.position().lat, start.lat)
        self.assertAlmostEqual(pad.position().lon, start.lon)

    def test_reboot_or_operator_starts_a_new_pad(self):
        pad = PadReference()
        pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.3470, -117.8090, 50_000))
        pad.accept_record("witness", state(BOOST_LOCKED, 35.36, -117.80, 60_000))
        pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.50, -117.50, 2_000))   # rebooted
        self.assertFalse(pad.witness_frozen)
        self.assertAlmostEqual(pad.position().lat, 35.50)
        pad.accept_record("witness", state(BOOST_LOCKED, 35.51, -117.50, 3_000))
        self.assertTrue(pad.witness_frozen)
        pad.clear_manual()                                                            # "Auto pad"
        self.assertFalse(pad.witness_frozen)

    def test_unlocked_pad_idle_is_ignored(self):
        pad = PadReference()
        pad.accept_record("witness", state(0x0000, 35.0, -117.0, 1000))
        self.assertIsNone(pad.position())


if __name__ == "__main__":
    unittest.main()
