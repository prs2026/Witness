"""attitude_math: quaternion conventions, angle sets, alignment and projection."""

import math
import random
import unittest

import attitude_math as am

S = math.sqrt(0.5)


class VectorAssertions(unittest.TestCase):
    def assertVec(self, actual, expected, places=9):
        self.assertEqual(len(actual), len(expected))
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=places, msg=f"{actual} != {expected}")

    def assertSameRotation(self, a, b, tolerance_deg=1e-6):
        self.assertLess(am.angle_between_deg(a, b), tolerance_deg, f"{a} vs {b}")


class KnownRotations(VectorAssertions):
    def test_identity_body_x_is_east(self):
        self.assertVec(am.rotate(am.IDENTITY, (1, 0, 0)), (1, 0, 0))
        self.assertVec(am.nose(am.IDENTITY), (1, 0, 0))
        self.assertAlmostEqual(am.tilt_deg(am.IDENTITY), 90.0)
        self.assertAlmostEqual(am.azimuth_deg(am.IDENTITY), 90.0)  # east
        yaw, pitch, roll, gimbal = am.to_euler_zyx(am.IDENTITY)
        self.assertEqual((yaw, pitch, roll, gimbal), (0.0, 0.0, 0.0, False))

    def test_plus_90_about_world_y_points_nose_down(self):
        # Right hand about +Y carries +Z to +X and +X to -Z.
        q = am.from_axis_angle((0, 1, 0), math.radians(90))
        self.assertVec(q, (0.0, S, 0.0, S))
        self.assertVec(am.rotate(q, (1, 0, 0)), (0, 0, -1))
        self.assertVec(am.rotate(q, (0, 0, 1)), (1, 0, 0))
        self.assertAlmostEqual(am.tilt_deg(q), 180.0)
        self.assertAlmostEqual(am.to_euler_zyx(q)[1], 90.0)  # pitch > 0 = nose down

    def test_upright_pad_is_minus_90_about_world_y(self):
        q = am.from_axis_angle((0, 1, 0), math.radians(-90))
        self.assertVec(q, (0.0, -S, 0.0, S))
        self.assertVec(q, am.Q_UPRIGHT)
        self.assertVec(am.rotate(q, (1, 0, 0)), (0, 0, 1))   # nose up
        self.assertVec(am.rotate(q, (0, 1, 0)), (0, 1, 0))   # body +Y north
        self.assertVec(am.rotate(q, (0, 0, 1)), (-1, 0, 0))  # body +Z west
        angles = am.attitude_angles(q)
        self.assertAlmostEqual(angles["tilt"], 0.0)
        self.assertIsNone(angles["azimuth"])
        self.assertAlmostEqual(angles["spin"], 0.0)
        self.assertAlmostEqual(angles["pitch"], -90.0)
        self.assertTrue(angles["gimbal"])

    def test_wire_order_from_comms_example(self):
        # Wire order x, y, z, w: a pure 90 deg yaw (about up) is (0, 0, s, s).
        q = (0.0, 0.0, S, S)
        self.assertVec(am.rotate(q, (1, 0, 0)), (0, 1, 0))  # east -> north
        self.assertAlmostEqual(am.to_euler_zyx(q)[0], 90.0)
        self.assertAlmostEqual(am.azimuth_deg(q), 0.0)

    def test_tilted_rocket_azimuth_and_tilt(self):
        # Upright, then tilt 10 deg toward east: rotate about world +Y by -(90-10)... use
        # the helper and verify the nose geometry directly.
        q = am.from_tilt_azimuth_spin(10.0, 90.0, 0.0)
        e, n, u = am.nose(q)
        self.assertAlmostEqual(u, math.cos(math.radians(10)))
        self.assertAlmostEqual(e, math.sin(math.radians(10)))
        self.assertAlmostEqual(n, 0.0)
        self.assertAlmostEqual(am.azimuth_deg(q), 90.0)
        # Tilt toward north instead.
        q = am.from_tilt_azimuth_spin(30.0, 0.0, 0.0)
        self.assertVec(am.nose(q), (0.0, 0.5, math.cos(math.radians(30))))
        self.assertAlmostEqual(am.attitude_angles(q)["elevation"], 60.0)

    def test_spin_is_right_handed_about_nose(self):
        roll = am.from_axis_angle((1, 0, 0), math.radians(30))
        q = am.multiply(am.Q_UPRIGHT, roll)     # roll in body, nose still up
        self.assertAlmostEqual(am.tilt_deg(q), 0.0)
        self.assertAlmostEqual(am.spin_deg(q), 30.0)
        # Right hand about up (the nose) carries body +Y (north) toward west.
        self.assertVec(am.rotate(q, (0, 1, 0)), (-0.5, math.cos(math.radians(30)), 0.0))


class ProductsAndRoundTrips(VectorAssertions):
    def test_multiply_composes_rotations(self):
        a = am.from_axis_angle((0, 0, 1), 0.7)
        b = am.from_axis_angle((1, 2, 3), -1.1)
        v = (0.3, -0.4, 2.0)
        self.assertVec(am.rotate(am.multiply(a, b), v), am.rotate(a, am.rotate(b, v)))
        self.assertVec(am.multiply(a, am.conjugate(a)), am.IDENTITY)

    def test_matrix_columns_are_body_axes(self):
        q = am.normalize((0.1, -0.5, 0.3, 0.8))
        m = am.to_matrix(q)
        for column, axis in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
            self.assertVec([m[row][column] for row in range(3)], am.rotate(q, axis))

    def test_euler_round_trip_including_gimbal_lock(self):
        rng = random.Random(4)
        for _ in range(500):
            pitch = rng.choice((90.0, -90.0, rng.uniform(-89.0, 89.0)))
            q = am.from_euler_zyx(rng.uniform(-180, 180), pitch, rng.uniform(-180, 180))
            yaw2, pitch2, roll2, _ = am.to_euler_zyx(q)
            self.assertSameRotation(am.from_euler_zyx(yaw2, pitch2, roll2), q, 1e-4)
        yaw, pitch, roll, gimbal = am.to_euler_zyx(am.from_euler_zyx(20, 35, -40))
        self.assertAlmostEqual(yaw, 20)
        self.assertAlmostEqual(pitch, 35)
        self.assertAlmostEqual(roll, -40)
        self.assertFalse(gimbal)

    def test_tilt_azimuth_spin_round_trip(self):
        rng = random.Random(7)
        for _ in range(500):
            tilt, az, spin = rng.uniform(0.5, 179), rng.uniform(0, 360), rng.uniform(-179, 179)
            a = am.attitude_angles(am.from_tilt_azimuth_spin(tilt, az, spin))
            self.assertAlmostEqual(a["tilt"], tilt, places=6)
            self.assertAlmostEqual(am.wrap_deg(a["azimuth"] - az), 0.0, places=5)
            self.assertAlmostEqual(am.wrap_deg(a["spin"] - spin), 0.0, places=5)

    def test_sign_flip_is_same_attitude(self):
        q = am.from_tilt_azimuth_spin(20, 45, 10)
        neg = tuple(-c for c in q)
        self.assertAlmostEqual(am.tilt_deg(neg), 20.0)
        self.assertAlmostEqual(am.spin_deg(neg), 10.0)
        self.assertAlmostEqual(am.angle_between_deg(q, neg), 0.0)


class Validity(unittest.TestCase):
    def test_zero_quaternion_is_not_available(self):
        q, n, reason = am.checked((0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(q)
        self.assertEqual(reason, "not available")
        self.assertEqual(n, 0.0)
        with self.assertRaises(ValueError):
            am.normalize((0, 0, 0, 0))

    def test_missing_nonfinite_and_short(self):
        self.assertEqual(am.checked(None)[2], "missing")
        self.assertEqual(am.checked([None, 0, 0, 1])[2], "missing")
        self.assertEqual(am.checked([0, 0, 1])[2], "missing")
        self.assertEqual(am.checked([math.nan, 0, 0, 1])[2], "non-finite")
        self.assertEqual(am.checked([1e-9, 0, 0, 0])[2], "degenerate")

    def test_norm_is_reported_and_normalised(self):
        q, n, reason = am.checked((0, 0, 0, 1.05))
        self.assertEqual(reason, "")
        self.assertEqual(q, (0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(n, 1.05)
        self.assertTrue(am.norm_bad(n))
        self.assertFalse(am.norm_bad(1.005))


class AlignmentAndInterpolation(VectorAssertions):
    def test_heading_offset_removed(self):
        truth = am.from_tilt_azimuth_spin(25, 300, 40)
        estimate = am.apply_heading(truth, -73.0)   # arbitrary initial heading
        offset = am.heading_offset_deg(estimate, truth)
        self.assertAlmostEqual(offset, 73.0)
        tilt_err, att_err = am.attitude_errors(estimate, truth, offset)
        self.assertAlmostEqual(tilt_err, 0.0)
        self.assertAlmostEqual(att_err, 0.0, places=5)
        _, unaligned = am.attitude_errors(estimate, truth, 0.0)
        self.assertGreater(unaligned, 10.0)

    def test_errors_after_alignment(self):
        truth = am.from_tilt_azimuth_spin(10, 0, 0)
        estimate = am.apply_heading(am.from_tilt_azimuth_spin(12, 0, 0), 50)
        offset = am.heading_offset_deg(estimate, truth)
        tilt_err, att_err = am.attitude_errors(estimate, truth, offset)
        self.assertAlmostEqual(tilt_err, 2.0)
        self.assertAlmostEqual(att_err, 2.0, places=3)

    def test_slerp(self):
        a = am.IDENTITY
        b = am.from_axis_angle((0, 0, 1), math.radians(90))
        self.assertSameRotation(am.slerp(a, b, 0.5), am.from_axis_angle((0, 0, 1), math.radians(45)))
        self.assertSameRotation(am.slerp(a, b, 0.0), a)
        self.assertSameRotation(am.slerp(a, b, 1.0), b)
        # Shortest path across the sign flip.
        neg_b = tuple(-c for c in b)
        self.assertSameRotation(am.slerp(a, neg_b, 0.5), am.from_axis_angle((0, 0, 1), math.radians(45)))
        # Nearly identical inputs fall back to normalised lerp.
        c = am.from_axis_angle((0, 0, 1), 1e-5)
        self.assertAlmostEqual(am.norm(am.slerp(a, c, 0.3)), 1.0)


class Projection(VectorAssertions):
    def test_named_views(self):
        right, up, toward = am.camera_basis(90, 0)       # from east, looking west
        self.assertVec(right, (0, 1, 0))
        self.assertVec(up, (0, 0, 1))
        self.assertVec(toward, (1, 0, 0))
        right, up, toward = am.camera_basis(0, 0)        # from north, looking south
        self.assertVec(right, (-1, 0, 0))
        self.assertVec(up, (0, 0, 1))
        right, up, toward = am.camera_basis(0, 90)       # from above, north up
        self.assertVec(right, (1, 0, 0))
        self.assertVec(up, (0, 1, 0))
        self.assertVec(toward, (0, 0, 1))

    def test_basis_orthonormal_and_project(self):
        basis = am.camera_basis(135, 35.26)
        for a in basis:
            self.assertAlmostEqual(sum(c * c for c in a), 1.0)
        for a, b in ((0, 1), (1, 2), (0, 2)):
            self.assertAlmostEqual(sum(x * y for x, y in zip(basis[a], basis[b])), 0.0)
        sx, sy, depth = am.project((0, 0, 1), basis)
        self.assertAlmostEqual(sx, 0.0)
        self.assertGreater(sy, 0.0)  # up is up on screen
        self.assertGreater(depth, 0.0)


if __name__ == "__main__":
    unittest.main()
