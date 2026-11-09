"""Tests for decoding the Witness store-v4 packet stream (P*.BIN)."""

import os
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import converter  # noqa: E402


def record(us: int, source: int, frame: bytes) -> bytes:
    return struct.pack(">IBB", us, source, len(frame)) + frame


STATE = bytes([0x02]) + bytes(63) + bytes([0x0A])
COMMAND = bytes([0x05, 0, 0, 0, 0, 0, 0, 0x73, 0x01, 0x0A])


class PacketLogTest(unittest.TestCase):
    def test_sources_times_and_wrap(self):
        data = (record(0xFFFFFF00, 1, STATE) + record(0x00000100, 2, COMMAND)
                + record(0x00002810, 4, COMMAND))
        rows, discarded = converter.decode_packet_log(data)
        self.assertEqual(discarded, 0)
        self.assertEqual([r["log_source"] for r in rows],
                         ["witness_state", "can_rx", "radio_rx"])
        self.assertEqual([r["packet_type"] for r in rows], ["state", "command", "command"])
        # 0xFFFFFF00 -> 0x100 crosses the 32-bit wrap: 512 us later.
        self.assertAlmostEqual(rows[1]["time_seconds"], 0.000512)
        self.assertAlmostEqual(rows[2]["time_seconds"], 0.010512)

    def test_truncated_tail_is_discarded(self):
        data = record(10, 1, STATE) + record(20, 3, COMMAND)[:-3]
        rows, discarded = converter.decode_packet_log(data)
        self.assertEqual(len(rows), 1)
        self.assertEqual(discarded, 16 - 3)

    def test_unknown_frame_is_kept_raw(self):
        frame = bytes([0x7E, 1, 2, 0x0A])
        rows, _ = converter.decode_packet_log(record(5, 2, frame))
        self.assertEqual(rows[0]["packet_type"], "unknown")
        self.assertEqual(rows[0]["raw_packet_hex"], frame.hex(" "))

    def test_name_detection(self):
        self.assertTrue(converter.is_packet_log_name(Path("P0000032.BIN")))
        self.assertFalse(converter.is_packet_log_name(Path("I0000032.BIN")))


class FlashRecordV2Test(unittest.TestCase):
    def record(self, sequence, uptime_ms, state):
        return (b"WL" + bytes([2, 0x46, 0x08 | state, 0])
                + struct.pack(">IIhii", sequence, uptime_ms, 512, 85602, 2150))

    def test_v2_records_decode(self):
        data = self.record(7, 1000, 1) + self.record(8, 1010, 2)
        self.assertTrue(converter.is_flash_record_log(data))
        rows, discarded = converter.decode_flash_log(data)
        self.assertEqual(discarded, 0)
        self.assertEqual([r["sequence"] for r in rows], [7, 8])
        self.assertEqual(rows[0]["flight_state_name"], "Boost")
        self.assertEqual(rows[1]["flight_state"], 2)
        self.assertEqual(rows[0]["gps_locked"], 1)
        self.assertEqual(rows[0]["pressure"], 85602)
        self.assertAlmostEqual(rows[0]["ms5607_temperature"], 21.5)
        self.assertAlmostEqual(rows[1]["time_seconds"], 1.01)
        self.assertEqual(rows[0]["frame_length"], 24)

    def test_v1_still_detected(self):
        v1 = b"WL" + bytes([1]) + bytes(45)
        self.assertTrue(converter.is_flash_record_log(v1 * 3))
        rows, _ = converter.decode_flash_log(v1 * 3)
        self.assertEqual(len(rows), 3)


if __name__ == "__main__":
    unittest.main()
