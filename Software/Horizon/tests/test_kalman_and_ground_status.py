"""Tests for the 0xE2 Kalman log packet and the GPS-carrying 0xF0 status."""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import converter  # noqa: E402
import main  # noqa: E402


def kalman_frame() -> bytes:
    nan = float("nan")
    payload = bytes([2, 0x01 | 0x10 | 0x40, 11])
    payload += struct.pack(">3f", 1.0, 2.0, 300.0)       # position
    payload += struct.pack(">3f", 0.1, 0.2, 150.0)       # velocity
    payload += struct.pack(">3f", 4.0, 9.0, 0.25)        # P00
    payload += struct.pack(">3f", 1.0, 1.0, 0.04)        # P11
    payload += struct.pack(">3f", 0.5, 0.5, 0.01)        # P01
    payload += struct.pack(">3f", 0.7, 0.9, 1.2)         # baro innovation, S, sigma
    payload += struct.pack(">3f", nan, nan, nan)         # GPS not offered
    payload += struct.pack(">f", 0.44)                   # Mach
    payload += struct.pack(">5H", 1000, 3, 2, 400, 70000 & 0xFFFF)
    assert len(payload) == 101
    return bytes([0xE2]) + payload + b"\x0a"


class KalmanLogTest(unittest.TestCase):
    def test_decodes_in_packet_log(self):
        frame = kalman_frame()
        self.assertEqual(len(frame), 103)
        data = struct.pack(">IBB", 1000, 5, len(frame)) + frame
        rows, discarded = converter.decode_packet_log(data)
        self.assertEqual(discarded, 0)
        row = rows[0]
        self.assertEqual(row["log_source"], "kalman_log")
        self.assertEqual(row["packet_type"], "kalman_log")
        self.assertEqual(row["kf_initialized"], 1)
        self.assertEqual(row["kf_baro_accepted"], 1)
        self.assertEqual(row["kf_gps_origin_set"], 1)
        self.assertEqual(row["kf_baro_gated"], 0)
        self.assertEqual(row["gps_satellites"], 11)
        self.assertAlmostEqual(row["position_variance_z"], 0.25)
        self.assertAlmostEqual(row["position_sigma_x"], 2.0)
        self.assertAlmostEqual(row["velocity_sigma_z"], 0.2, places=6)
        self.assertAlmostEqual(row["baro_innovation"], 0.7, places=6)
        self.assertAlmostEqual(row["mach"], 0.44, places=6)
        self.assertEqual(row["gps_updates"], 400)
        gps = row["gps_innovation_x"]
        self.assertTrue(gps is None or gps == "" or (isinstance(gps, float) and math.isnan(gps)))


class GroundStationStatusTest(unittest.TestCase):
    def payload(self, age=250):
        return (struct.pack(">bI", -70, 123456) + bytes([1, 9])
                + struct.pack(">ii", 345678901, -1171234567) + struct.pack(">f", 610.5)
                + struct.pack(">IHH", 1791849600, 85, age))

    def test_gps_fields(self):
        for parse in (lambda p: converter.parse_packet(0xF0, p),
                      lambda p: main.parse_packet(0xF0, p)):
            packet = parse(self.payload())
            self.assertEqual(packet["rssi"], -70)
            self.assertEqual(packet["uptime"], 123456)
            self.assertEqual(packet["gps_satellites"], 9)
            self.assertAlmostEqual(packet["latitude"], 34.5678901, places=7)
            self.assertAlmostEqual(packet["longitude"], -117.1234567, places=7)
            self.assertAlmostEqual(packet["gps_altitude"], 610.5)
            self.assertAlmostEqual(packet["gps_hdop"], 0.85)
            self.assertEqual(packet["gps_fix_age_ms"], 250)

    def test_no_fix_and_legacy(self):
        empty = struct.pack(">bI", -127, 5) + bytes(18) + struct.pack(">HH", 0xFFFF, 0xFFFF)
        packet = converter.parse_packet(0xF0, empty)
        self.assertIsNone(packet["latitude"])
        self.assertIsNone(packet["gps_fix_age_ms"])
        legacy = converter.parse_packet(0xF0, struct.pack(">bI", -80, 7) + bytes(6))
        self.assertEqual(legacy["rssi"], -80)
        self.assertEqual(legacy["gps_fix_quality"], 0)
        self.assertIsNone(legacy["latitude"])


if __name__ == "__main__":
    unittest.main()
