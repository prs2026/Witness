"""Quaternion-derived angular rates; synthetic rotations have known answers."""

import math
import unittest

from aim_orientation import AimAngularRateEstimator, GYRO_FIELDS
from flight_data import DEFAULT_LAYOUT, FlightData, load_layout


def rotation(axis, degrees):
    half = math.radians(degrees) / 2
    length = math.hypot(*axis)
    return (math.cos(half), *(v / length * math.sin(half) for v in axis))


def product(q, r):
    a, b, c, d = q
    w, x, y, z = r
    return (a*w-b*x-c*y-d*z, a*x+b*w+c*z-d*y,
            a*y-b*z+c*w+d*x, a*z+b*y-c*x+d*w)


def sample(quaternion, seconds):
    return dict(zip(('w', 'x', 'y', 'z'), quaternion),
                timestamp=1_792_741_990_000_000 + round(seconds * 1_000_000))


class AngularRateTests(unittest.TestCase):
    def test_each_axis_and_arbitrary_initial_attitude(self):
        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 2, 3)):
            for angle in (-90, -1, 0, 1, 90):
                with self.subTest(axis=axis, angle=angle):
                    estimator = AimAngularRateEstimator()
                    initial = rotation((0, 1, 1), 75)
                    self.assertEqual(estimator.update('A', sample(initial, 0), 10), dict.fromkeys(GYRO_FIELDS))
                    result = estimator.update('A', sample(product(initial, rotation(axis, angle)), .5), 10.001)
                    for key, direction in zip(GYRO_FIELDS, axis):
                        self.assertAlmostEqual(result[key], direction / math.hypot(*axis) * angle / .5, places=8)

    def test_normalization_sign_flip_and_shortest_wrap(self):
        estimator = AimAngularRateEstimator()
        q = rotation((0, 0, 1), 170)
        estimator.update('A', sample(tuple(v * 2 for v in q), 0), 1)
        result = estimator.update('A', sample(tuple(-v * 3 for v in q), .1), 1.1)
        self.assertEqual(tuple(result.values()), (0, 0, 0))
        result = estimator.update('A', sample(rotation((0, 0, 1), -170), .2), 1.2)
        self.assertAlmostEqual(result[GYRO_FIELDS[2]], 200)

    def test_timestamp_duplicates_out_of_order_and_burst_delivery(self):
        estimator = AimAngularRateEstimator()
        estimator.update('A', sample(rotation((1, 0, 0), 0), 0), 100)
        for seconds in (0, -.1, .0001):
            self.assertEqual(estimator.update('A', sample(rotation((1, 0, 0), 100), seconds), 100), {})
        result = estimator.update('A', sample(rotation((1, 0, 0), 9), .1), 100)
        self.assertAlmostEqual(result[GYRO_FIELDS[0]], 90)
        # A backwards host clock sample cannot replace the stored baseline either.
        self.assertEqual(estimator.update('A', sample(rotation((1, 0, 0), 99), 2), 99), {})
        result = estimator.update('A', sample(rotation((1, 0, 0), 18), .2), 100)
        self.assertAlmostEqual(result[GYRO_FIELDS[0]], 90)

    def test_invalid_retained_gaps_and_clock_restart(self):
        invalids = ({'w': 0, 'x': 0, 'y': 0, 'z': 0}, {'w': float('nan')},
                    {'x': float('inf')}, {'z': True}, {'w': 10**1000},
                    {'timestamp': None}, {'timestamp': 1.5})
        for bad in invalids:
            estimator = AimAngularRateEstimator()
            estimator.update('A', sample((1, 0, 0, 0), 0), 0)
            self.assertEqual(estimator.update('A', {**sample((1, 0, 0, 0), .1), **bad}, .1),
                             dict.fromkeys(GYRO_FIELDS))
            self.assertFalse(estimator.previous)
        estimator = AimAngularRateEstimator()
        self.assertEqual(estimator.update('A', sample((1, 0, 0, 0), 0), 0, retained=True), {})
        self.assertFalse(estimator.previous)
        estimator.update('A', sample((1, 0, 0, 0), 0), 0)
        self.assertEqual(estimator.update('A', {'timestamp': 123}, .1), {})
        # Device-time gap, then host-time gap, both re-prime without a large spike.
        self.assertEqual(estimator.update('A', sample((1, 0, 0, 0), 5), .2), dict.fromkeys(GYRO_FIELDS))
        self.assertEqual(estimator.update('A', sample((1, 0, 0, 0), 5.1), 10), dict.fromkeys(GYRO_FIELDS))
        # Restarted source clock: recover after a reception gap.
        self.assertEqual(estimator.update('A', sample((1, 0, 0, 0), 0), 20), dict.fromkeys(GYRO_FIELDS))
        result = estimator.update('A', sample(rotation((0, 1, 0), 10), .1), 20.1)
        self.assertAlmostEqual(result[GYRO_FIELDS[1]], 100)

    def test_sources_are_isolated_and_bounded(self):
        estimator = AimAngularRateEstimator()
        estimator.update('A', sample((1, 0, 0, 0), 0), 0)
        estimator.update('B', sample(rotation((0, 0, 1), 90), 0), 0)
        self.assertEqual(tuple(estimator.update('B', sample(rotation((0, 0, 1), 90), .1), .1).values()), (0, 0, 0))
        result = estimator.update('A', sample(rotation((1, 0, 0), 10), .1), .1)
        self.assertAlmostEqual(result[GYRO_FIELDS[0]], 100)
        for i in range(200):
            estimator.update(str(i), sample((1, 0, 0, 0), 0), 0)
        self.assertLessEqual(len(estimator.previous), 128)

    def test_flight_routing_reset_and_no_duplicate_freshness(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        def accept(q, seconds, received, retained=False):
            model.accept_aim('A', 'orientation_payload/v1', sample(q, seconds), received, retained)
        accept((1, 0, 0, 0), 0, 1)
        self.assertIsNone(model.snapshot()[2][3].latest)
        accept(rotation((1, 0, 0), 9), .1, 1.1)
        self.assertAlmostEqual(model.snapshot()[2][3].latest, 90)
        self.assertEqual(model.snapshot()[2][4].latest, 0)
        before = model.snapshot()[2][3:]
        accept(rotation((1, 0, 0), 50), .1, 1.2)
        accept(rotation((1, 0, 0), 50), .2, 1.3, True)
        self.assertEqual(model.snapshot()[2][3:], before)
        model.set_window(60)
        model.reset_pad()
        accept(rotation((1, 0, 0), 18), .2, 1.4)
        self.assertAlmostEqual(model.snapshot()[2][3].latest, 90)
        self.assertTrue(all(t.axis == 'left' for t in model.layout.plots[2].traces))
        self.assertIsNone(model.layout.plots[2].right_unit)
        model.clear()
        accept(rotation((1, 0, 0), 27), .3, 1.5)
        self.assertIsNone(model.snapshot()[2][3].latest)
        accept(rotation((1, 0, 0), 36), .4, 1.6)
        self.assertAlmostEqual(model.snapshot()[2][3].latest, 90)
        model.configure(load_layout(DEFAULT_LAYOUT))
        self.assertFalse(model.aim_rates.previous)


if __name__ == '__main__':
    unittest.main()
