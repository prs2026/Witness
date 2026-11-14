"""0x08 maintenance status beacon (two-image boot) decode and framing."""
import unittest

import witness_protocol as wp

# Frame built exactly as main/maint_beacon.h encodes it.
PAYLOAD = bytes([1, 0, 0x25, 4, 1, 1, 0xBE, 0xEF, 0x12, 0x34, 0x1D, 0x14, 0x01, 0x02, 0x03, 0x04, 12])


class MaintenanceBeaconTests(unittest.TestCase):
    def test_decode(self):
        pkt = wp.decode_maintenance_status(PAYLOAD)
        self.assertEqual(pkt["packet"], "maintenance_status")
        self.assertEqual(pkt["mode_name"], "Wi-Fi")
        self.assertTrue(pkt["ap_up"])
        self.assertTrue(pkt["store_mounted"])
        self.assertTrue(pkt["radio_ok"])
        self.assertFalse(pkt["recovery"])
        self.assertEqual(pkt["maint_image_id"], 0xBEEF)
        self.assertEqual(pkt["flight_image_id"], 0x1234)
        self.assertAlmostEqual(pkt["battery_voltage"], 7.444)
        self.assertEqual(pkt["uptime"], 0x01020304)
        self.assertEqual(pkt["log_files"], 12)

    def test_bad_version_or_length(self):
        self.assertEqual(wp.decode_maintenance_status(bytes([2]) + PAYLOAD[1:])["beacon_version"], 2)
        self.assertIsNone(wp.decode_maintenance_status(PAYLOAD[:-1]))

    def test_parse_packet_and_stream(self):
        self.assertEqual(wp.parse_packet(0x08, PAYLOAD)["packet"], "maintenance_status")
        got = []
        decoder = wp.PacketDecoder(got.append)
        decoder.feed(bytes([0x08]) + PAYLOAD + bytes([0x0A]))
        self.assertEqual([p["packet"] for p in got], ["maintenance_status"])

    def test_refusal_reason(self):
        payload = bytes([0x57, 0x46, 1, 0, 8, 0, 0, 0, 1, 0, 4])
        pkt = wp.parse_packet(0x06, payload)
        self.assertEqual(pkt["detail_name"], "LoRa settings differ")
        ok = wp.parse_packet(0x06, bytes([0x57, 0x46, 1, 0, 8, 0, 0, 0, 0, 0, 0]))
        self.assertNotIn("detail_name", ok)


if __name__ == "__main__":
    unittest.main()
