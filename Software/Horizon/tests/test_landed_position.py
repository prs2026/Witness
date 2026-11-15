"""0x09 landed position packet decode and framing (comms.h IRIS_PACKET_ID_LANDED_POSITION)."""
import struct
from pathlib import Path
import unittest

import witness_protocol as wp


def payload(lat=-33.8688197, lon=151.2092955, alt=123.5, age=300, flags=0x09, sats=9, mv=7444,
            status=(0x80, 0x1C), uptime=0x01020304):
    return (bytes(status) + struct.pack(">I", uptime) + struct.pack(">i", round(lat * 1e7)) +
            struct.pack(">i", round(lon * 1e7)) + struct.pack(">f", alt) + struct.pack(">I", age) +
            bytes([flags, sats]) + struct.pack(">H", mv))


class LandedPositionTests(unittest.TestCase):
    def test_decode(self):
        pkt = wp.decode_landed_position(payload())
        self.assertEqual(pkt["packet"], "landed_position")
        self.assertEqual(pkt["flight_state"], 4)
        self.assertAlmostEqual(pkt["latitude"], -33.8688197, places=7)
        self.assertAlmostEqual(pkt["longitude"], 151.2092955, places=7)
        self.assertAlmostEqual(pkt["gps_altitude"], 123.5, places=3)
        self.assertEqual(pkt["fix_age"], 300)
        self.assertTrue(pkt["position_valid"])
        self.assertTrue(pkt["utc_aligned"])
        self.assertFalse(pkt["stale"])
        self.assertFalse(pkt["ping_reply"])
        self.assertEqual(pkt["satellites"], 9)
        self.assertAlmostEqual(pkt["battery_voltage"], 7.444)
        self.assertEqual(pkt["uptime"], 0x01020304)

    def test_stale_ping_and_no_fix(self):
        pkt = wp.decode_landed_position(payload(flags=0x03 | 0x04))
        self.assertTrue(pkt["stale"] and pkt["ping_reply"] and not pkt["utc_aligned"])
        none = wp.decode_landed_position(payload(lat=0, lon=0, alt=0, age=0xFFFFFFFF, flags=0, sats=0))
        self.assertIsNone(none["latitude"])
        self.assertIsNone(none["fix_age"])
        self.assertFalse(none["position_valid"])

    def test_length(self):
        self.assertIsNone(wp.decode_landed_position(payload()[:-1]))

    def test_parse_packet_and_stream(self):
        self.assertEqual(wp.parse_packet(0x09, payload())["packet"], "landed_position")
        got = []
        decoder = wp.PacketDecoder(got.append)
        decoder.feed(bytes([0x09]) + payload() + bytes([0x0A]))
        self.assertEqual([p["packet"] for p in got], ["landed_position"])

    def test_ping_frame(self):
        frame = wp.build_ping_packet()
        self.assertEqual(frame, bytes([0x05, 0, 0, 0, 0, 0, 0, 0x00, 0x22, 0x0A]))

    def test_comms_header_layout(self):
        text = open(Path(__file__).resolve().parents[1] / "comms.h", encoding="utf-8").read()
        for needle in ("IRIS_PACKET_ID_LANDED_POSITION 0x09U", "IRIS_PACKET_LANDED_POSITION_DATA_LENGTH 26U",
                       "IRIS_FIELD_LANDED_COORDINATE_SCALE 10000000L"):
            self.assertIn(needle, text)


if __name__ == "__main__":
    unittest.main()
