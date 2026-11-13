import math
import unittest

import antenna_pointer_math as m


class LookAngleTests(unittest.TestCase):
    GS = (32.9, -105.0, 1200.0)

    def test_cardinal_directions(self):
        lat, lon, alt = self.GS
        d = 0.01
        self.assertAlmostEqual(m.wrap180(m.look_angles(*self.GS, lat + d, lon, alt).azimuth), 0.0, delta=0.05)
        self.assertAlmostEqual(m.look_angles(*self.GS, lat, lon + d, alt).azimuth, 90.0, delta=0.05)
        self.assertAlmostEqual(m.look_angles(*self.GS, lat - d, lon, alt).azimuth, 180.0, delta=0.05)
        self.assertAlmostEqual(m.look_angles(*self.GS, lat, lon - d, alt).azimuth, 270.0, delta=0.05)

    def test_overhead_and_elevation(self):
        lat, lon, alt = self.GS
        up = m.look_angles(*self.GS, lat, lon, alt + 3000)
        self.assertAlmostEqual(up.elevation, 90.0, places=3)
        self.assertAlmostEqual(up.range_m, 3000.0, places=2)
        la = m.look_angles(*self.GS, lat + 3000 / 111_132.0, lon, alt + 3000)
        self.assertAlmostEqual(la.elevation, 45.0, delta=0.1)

    def test_below_horizon_over_long_range(self):
        lat, lon, alt = self.GS
        self.assertLess(m.look_angles(*self.GS, lat + 0.9, lon, alt).elevation, 0.0)

    def test_predict(self):
        lat, lon, alt = self.GS
        p = m.predict_lla(lat, lon, alt, 0.0, 100.0, 50.0, 10.0)
        la = m.look_angles(lat, lon, alt, *p)
        self.assertAlmostEqual(la.range_m, math.hypot(1000.0, 500.0), delta=2.0)
        self.assertEqual(m.predict_lla(lat, lon, alt, 1, 2, 3, 0), (lat, lon, alt))


class AzimuthTests(unittest.TestCase):
    def test_shortest_path_in_wide_range(self):
        az, ok = m.choose_azimuth(190.0, 170.0, -270.0, 270.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(az, 190.0)
        az, ok = m.choose_azimuth(350.0, -10.0, -270.0, 270.0)
        self.assertAlmostEqual(az, -10.0)

    def test_cable_wrap_forces_long_way(self):
        az, ok = m.choose_azimuth(190.0, 170.0, -180.0, 180.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(az, -170.0)

    def test_out_of_range_clamps_to_nearest_limit(self):
        az, ok = m.choose_azimuth(200.0, 0.0, 0.0, 180.0)
        self.assertFalse(ok)
        self.assertEqual(az, 180.0)
        az, ok = m.choose_azimuth(300.0, 0.0, 0.0, 180.0)
        self.assertFalse(ok)
        self.assertEqual(az, 0.0)

    def test_boundary_inclusive(self):
        az, ok = m.choose_azimuth(360.0, 10.0, 0.0, 360.0)
        self.assertTrue(ok)
        self.assertIn(az, (0.0, 360.0))

    def test_bad_limits(self):
        with self.assertRaises(ValueError):
            m.choose_azimuth(0, 0, 10, 0)

    def test_wrap_helpers(self):
        self.assertAlmostEqual(m.wrap180(190), -170)
        self.assertAlmostEqual(m.wrap360(-10), 350)


class MountTests(unittest.TestCase):
    def test_offsets_and_declination(self):
        la = m.LookAngles(100.0, 20.0, 1000.0)
        self.assertEqual(m.mount_command(la), (100.0, 20.0))
        az, el = m.mount_command(la, az_offset=2.0, el_offset=-1.0, declination=8.0)
        self.assertAlmostEqual(az, 94.0)
        self.assertAlmostEqual(el, 19.0)
        self.assertAlmostEqual(m.mount_command(m.LookAngles(2.0, 0, 1), declination=8.0)[0], 354.0)

    def test_keyhole(self):
        self.assertTrue(m.in_keyhole(88.0))
        self.assertFalse(m.in_keyhole(60.0))


class MotionTests(unittest.TestCase):
    def test_rate_limit(self):
        self.assertAlmostEqual(m.rate_limit(0.0, 100.0, 30.0, 0.5), 15.0)
        self.assertAlmostEqual(m.rate_limit(0.0, 5.0, 30.0, 0.5), 5.0)
        self.assertAlmostEqual(m.rate_limit(0.0, -100.0, 30.0, 0.5), -15.0)

    def test_feed(self):
        self.assertAlmostEqual(m.feed_for_move(3.0, 0.0, 0.2, margin=1.0), 900.0)
        self.assertEqual(m.feed_for_move(0.0, 0.0, 0.2), 6.0)
        self.assertEqual(m.feed_for_move(1000.0, 0.0, 0.1), 3600.0)

    def test_gcode_lines(self):
        self.assertEqual(m.format_jog(123.4567, 45.0, 900.4), "$J=G21G90X123.457Y45.000F900")
        self.assertEqual(m.format_move(-10.0, 5.5, 600), "G1 X-10.000 Y5.500 F600")
        self.assertEqual(m.GRBL_JOG_CANCEL, b"\x85")

    def test_parse_status(self):
        self.assertEqual(m.parse_grbl_status("<Run|MPos:10.500,20.000,0.000|FS:300,0>"),
                         ("Run", {"x": 10.5, "y": 20.0, "z": 0.0, "kind": "MPos"}))
        self.assertEqual(m.parse_grbl_status("<Idle:0|WPos:1,2,3>")[0], "Idle")
        self.assertIsNone(m.parse_grbl_status("ok"))
        self.assertIsNone(m.parse_grbl_status("<Idle|FS:0,0>"))


if __name__ == "__main__":
    unittest.main()
