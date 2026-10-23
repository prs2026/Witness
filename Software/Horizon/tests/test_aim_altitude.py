"""Altitude-derived AIM velocity, display holding and tight graph bounds."""

import math
import unittest

from aim_altitude import AimAltitudeEstimator, BARO_ALTITUDE, BARO_VELOCITY, pressure_altitude
from flight_data import DEFAULT_LAYOUT, FlightData, graph_range, load_layout
from flight_render import quality_text, value_text


def sample(altitude, seconds):
    pressure = 101325 * (1 - .0065 * altitude / 288.15) ** (9.80665 / (287.05287 * .0065))
    return {"pressure": pressure, "timestamp": 1790000000000000 + round(seconds * 1e6)}


class AltitudeTests(unittest.TestCase):
    def test_pressure_units_reference_and_standard_layers(self):
        for altitude in (-1000, 0, 100, 1000, 10000, 11000):
            self.assertAlmostEqual(pressure_altitude(sample(altitude, 0)["pressure"]), altitude, places=7)
        for pressure, altitude in ((22632.04, 11000), (5474.88, 20000), (868.016, 32000)):
            self.assertAlmostEqual(pressure_altitude(pressure), altitude, delta=.1)
        for pressure in (None, True, "bad", 0, -1, 10**1000, float("nan"), float("inf"), 99, 120001):
            self.assertIsNone(pressure_altitude(pressure))

    def test_velocity_rising_falling_and_burst_receive_times(self):
        estimator = AimAltitudeEstimator()
        self.assertIsNone(estimator.update("A", sample(100, 0), 1)[BARO_VELOCITY])
        # Receive spacing is irrelevant: use sample timestamps, not MQTT bursts.
        self.assertAlmostEqual(estimator.update("A", sample(110, .5), 1)[BARO_VELOCITY], 20)
        self.assertAlmostEqual(estimator.update("A", sample(90, 1), 1.01)[BARO_VELOCITY], -40)
        self.assertAlmostEqual(estimator.update("A", sample(90, 2), 2)[BARO_VELOCITY], 0)
        self.assertIsNone(estimator.update("B", sample(500, 2), 2)[BARO_VELOCITY])

    def test_retained_duplicate_out_of_order_invalid_and_gap(self):
        estimator = AimAltitudeEstimator()
        self.assertEqual(estimator.update("A", sample(10, 0), 1, retained=True), {})
        self.assertFalse(estimator.previous)
        estimator.update("A", sample(10, 0), 1)
        for fields, received in ((sample(999, 0), 1.1), (sample(999, -.1), 1.2), (sample(999, 1), .5)):
            self.assertEqual(estimator.update("A", fields, received), {})
        self.assertAlmostEqual(estimator.update("A", sample(20, 1), 2)[BARO_VELOCITY], 10)
        self.assertIsNone(estimator.update("A", {"pressure": 0, "timestamp": sample(0, 2)["timestamp"]}, 3)[BARO_VELOCITY])
        self.assertIsNone(estimator.update("A", sample(30, 3), 4)[BARO_VELOCITY])
        self.assertIsNone(estimator.update("A", sample(50, 5), 8)[BARO_VELOCITY])
        self.assertIsNone(estimator.update("A", sample(100, 10), 9)[BARO_VELOCITY])
        self.assertAlmostEqual(estimator.update("A", sample(110, 11), 10)[BARO_VELOCITY], 10)
        for index in range(150):
            estimator.update(str(index), sample(0, 0), 1)
        self.assertLessEqual(len(estimator.previous), 128)

    def test_flight_routes_gps_separate_hold_age_and_recovery(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        model.accept_aim("A", "pressure_payload/v1", sample(100, 0), 1, False)
        model.accept_aim("A", "pressure_payload/v1", sample(110, 1), 2, False)
        model.accept_aim("A", "gnss_payload/v1", {"gnss_coordinate.altitude": 200, "satellites_used": 6}, 2, False)
        altitude, velocity = model.snapshot()[0][2], model.snapshot()[3][2]
        self.assertAlmostEqual(altitude.latest, 110)
        self.assertAlmostEqual(velocity.latest, 10)
        self.assertEqual(model.snapshot()[0][3].latest, 200)
        model.accept_aim("A", "pressure_payload/v1", {"pressure": None, "timestamp": sample(0, 2)["timestamp"]}, 3, False)
        velocity = model.snapshot()[3][2]
        self.assertAlmostEqual(velocity.latest, 10)
        self.assertEqual(velocity.received, 2)
        self.assertEqual(velocity.quality, "INVALID")
        self.assertEqual(velocity.points[-1], (3, None))
        self.assertEqual(quality_text(velocity, 15)[0], "INVALID 13.0s")
        self.assertEqual(value_text(model.layout.plots[3].traces[2], velocity), "10")
        # Do not revive an out-of-order valid reading after newer bad data.
        model.accept_aim("A", "pressure_payload/v1", sample(500, 2), 2.5, False)
        self.assertAlmostEqual(model.snapshot()[0][2].latest, 110)
        self.assertNotIn("A", model.aim_altitude.previous)
        model.accept_aim("A", "pressure_payload/v1", sample(120, 3), 4, False)
        self.assertEqual(model.snapshot()[3][2].quality, "INVALID")
        model.accept_aim("A", "pressure_payload/v1", sample(130, 4), 5, False)
        self.assertAlmostEqual(model.snapshot()[3][2].latest, 10)
        self.assertEqual(model.snapshot()[3][2].quality, "LIVE")
        model.accept_aim("A", "gnss_payload/v1", {"gnss_coordinate.altitude": None}, 3, False)
        self.assertEqual(model.snapshot()[0][3].latest, 200)
        model.accept_aim("A", "gnss_payload/v1", {"gnss_coordinate.altitude": 0, "satellites_used": 0}, 4, False)
        self.assertEqual(model.snapshot()[0][3].latest, 200)
        model.clear()
        self.assertFalse(model.aim_altitude.previous)
        self.assertIsNone(model.snapshot()[0][2].latest)


class RangeTests(unittest.TestCase):
    def test_rssi_span_small_signals_and_large_offsets(self):
        for values in ((-60, 0), (-60.01, -60), (0, .000001), (1e6, 1e6 + .01), (-1e6, -1e6 + .01)):
            low, high, step = graph_range(values, 35)
            span = max(values) - min(values)
            self.assertLess(low, min(values))
            self.assertGreater(high, max(values))
            self.assertGreater(span / (high - low), .85)
            self.assertGreater(step, 0)

    def test_empty_constant_fixed_and_default_auto(self):
        self.assertTrue(load_layout(DEFAULT_LAYOUT).auto_scale)
        self.assertEqual(graph_range([], 50)[:2], (0, 1))
        for value in (0, -60, 1e12):
            low, high, step = graph_range([value], 50)
            self.assertLess(low, value)
            self.assertGreater(high, value)
            self.assertTrue(math.isfinite(step))
        self.assertEqual(graph_range([-60, 0], 50, (-200, 200))[:2], (-200, 200))


if __name__ == "__main__":
    unittest.main()
