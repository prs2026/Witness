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

    def test_reboot_before_any_flight_or_operator_starts_a_new_pad(self):
        pad = PadReference()
        pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.3470, -117.8090, 50_000))
        pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.50, -117.50, 2_000))   # rebooted on the pad
        self.assertFalse(pad.witness_frozen)
        self.assertAlmostEqual(pad.position().lat, 35.50)
        pad.accept_record("witness", state(BOOST_LOCKED, 35.51, -117.50, 3_000))
        self.assertTrue(pad.witness_frozen)
        pad.clear_manual()                                                            # "Auto pad"
        self.assertFalse(pad.witness_frozen)
        self.assertFalse(pad.flight_seen)

    def test_landed_reboot_keeps_flight_pad_until_a_new_boost(self):
        """0x7300 in Landed reboots Witness into Pad Idle at the landing site (audit #12)."""
        pad = PadReference()
        for k in range(10):
            pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.3470, -117.8090, 50_000 + k * 100))
        pad.accept_record("witness", state(BOOST_LOCKED, 35.36, -117.80, 60_000))
        pad.accept_record("witness", state(LANDED_LOCKED, 35.40, -117.70, 90_000))
        version = pad.version
        for k in range(30):                                                           # rebooted, still at landing site
            pad.accept_record("witness", state(PAD_IDLE_LOCKED, 35.40, -117.70, 1_000 + k * 100))
        self.assertTrue(pad.witness_frozen)
        self.assertAlmostEqual(pad.position().lat, 35.3470)
        self.assertEqual(pad.version, version)
        self.assertEqual(pad.segment, 0)
        # Recovered, carried to pad 2: idle fixes there, then Boost = a new flight segment.
        for k in range(20):
            pad.accept_record("witness", state(PAD_IDLE_LOCKED, 36.0, -117.0, 10_000 + k * 100))
        pad.accept_record("witness", state(BOOST_LOCKED, 36.0, -117.0, 20_000))
        self.assertEqual(pad.segment, 1)
        self.assertAlmostEqual(pad.position().lat, 36.0)
        self.assertAlmostEqual(pad.witness_launch_site(0).lat, 35.3470)   # old flight keeps its origin
        # A Boost with no Pad Idle period (e.g. immediately after a reboot) is not a new flight.
        pad.accept_record("witness", state(LANDED_LOCKED, 36.1, -117.0, 40_000))
        pad.accept_record("witness", state(BOOST_LOCKED, 36.2, -117.0, 41_000))
        self.assertEqual(pad.segment, 1)

    def test_landed_packet_and_stale_aperture_fix(self):
        pad = PadReference()
        landed = {"packet": {"packet": "landed_position", "latitude": 35.5, "longitude": -117.6, "gps_altitude": 650.0,
                             "position_valid": True, "stale": False}, "received_mono": 5.0}
        self.assertTrue(pad.accept_record("witness", landed))
        self.assertEqual(pad.landed().lat, 35.5)
        self.assertIsNone(pad.position())            # informational: never selected as the pad
        self.assertTrue(pad.flight_seen and pad.witness_frozen)
        aperture = {"packet": {"packet": "ground_station_status", "latitude": 35.1, "longitude": -117.1,
                               "gps_altitude": 1.0, "gps_fix_quality": 0}, "received_mono": 6.0}
        self.assertFalse(pad.accept_record("witness", aperture))   # fix lost: last known position only
        self.assertIsNone(pad.position())

    def test_unlocked_pad_idle_is_ignored(self):
        pad = PadReference()
        pad.accept_record("witness", state(0x0000, 35.0, -117.0, 1000))
        self.assertIsNone(pad.position())


if __name__ == "__main__":
    unittest.main()
