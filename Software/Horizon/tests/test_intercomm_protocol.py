"""Cross-project schema and actual wire vectors for the shared protocol."""

from pathlib import Path
import unittest

from converter import parse_packet as convert_packet
from main import PacketDecoder, SerialMonitor, parse_packet


class IntercommProtocolTests(unittest.TestCase):
    def test_schema_copies_match(self):
        root = Path(__file__).resolve().parents[2]
        paths = (
            "Horizon/comms.h", "Iris/softwarev1IRIS/main/comms.h",
            "Witness/softwarev1/main/comms.h", "Aperture/softwareattempt2/main/comms.h",
        )
        expected = (root / paths[0]).read_text()
        for path in paths[1:]:
            with self.subTest(path=path):
                self.assertEqual((root / path).read_text(), expected)

    def test_iris_camera_and_debug_twenty_milliamp_wire_vectors(self):
        # 20 mA, 1 A, 5.1 A exercise resolution and full-scale saturation.
        camera = bytes.fromhex("03 00 00 00 00 00 01 48 00 00 00 00 02 01 32 ff 10 68 0a")
        debug = bytes.fromhex("e1 48 00 00 00 00 02 00 10 68 01 32 ff 0a")
        packets, errors = [], []
        decoder = PacketDecoder(packets.append, errors.append)
        for frame in (camera, debug):
            for byte in frame:
                decoder.feed(bytes((byte,)))
            for decode in (parse_packet, convert_packet):
                packet = decode(frame[0], frame[1:-1])
                for actual, expected in zip(packet["current_sense"], (0.02, 1.0, 5.1)):
                    self.assertAlmostEqual(actual, expected)
                self.assertAlmostEqual(packet["iris_battery_voltage"], 4.2)
        self.assertFalse(errors)
        self.assertEqual(len(packets), 2)

    def test_all_shortcuts_cross_aperture_as_canonical_commands(self):
        for opcode in (0x22, 0x31, 0x51, 0xE0, 0xE1, 0x6868, 0x6869, 0x7300):
            with self.subTest(opcode=opcode):
                frame = SerialMonitor.build_quick_command_packet(opcode)
                self.assertEqual(len(frame), 10)
                self.assertEqual(frame[0], 0x05)
                self.assertEqual(frame[-1], 0x0A)
                self.assertEqual(int.from_bytes(frame[7:9], "big"), opcode)


if __name__ == "__main__":
    unittest.main()
