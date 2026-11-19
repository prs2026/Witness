"""HITL readers of a flight-log session that was resumed after a reset in flight.

Streams are built byte for byte from the firmware layouts in
Software/Witness/softwarev1/main/flash_logger.h: packet records are
[u32 us BE][u8 source][u8 length][frame]; source 6 is the resumed marker with
frame [u8 kind=1][u8 state][u32 generation BE].
"""

import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
HORIZON = os.path.join(os.path.dirname(os.path.dirname(HERE)), "Horizon")
sys.path.insert(0, HORIZON)

import converter  # noqa: E402
import hitl_report  # noqa: E402

SEC = 1_000_000


def packet(us, source, frame):
    return struct.pack(">IBB", us & 0xFFFFFFFF, source, len(frame)) + frame


def kalman_frame(gps_utc=0):
    return bytes([0xE2]) + bytes(93) + struct.pack(">I", gps_utc) + bytes(4) + bytes([0x0A])


def marker(us, state=2, generation=7):
    return packet(us, 6, bytes([1, state]) + struct.pack(">I", generation))


class ResumedSession(unittest.TestCase):
    def build(self):
        before = [packet(1 * SEC + i * 40_000, 1, kalman_frame()) for i in range(20)]
        # Device clock restarts at ~0 after the reset: the counter went back.
        after = [marker(100_000)] + [packet(150_000 + i * 40_000, 1, kalman_frame()) for i in range(20)]
        return b"".join(before + after)

    def test_converter_marks_the_session_and_kalman_rows_skip_the_restarted_clock(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "fc_logs").mkdir()
            (run / "fc_logs" / "P0000007.BIN").write_bytes(self.build())
            rows = converter.iter_packet_log((run / "fc_logs" / "P0000007.BIN").read_bytes())
            all_rows = list(rows)
            self.assertEqual(sum(1 for r in all_rows if r["packet_type"] == "log_resumed"), 1)
            self.assertEqual(sum(1 for r in all_rows if r["packet_type"] == "kalman_log"), 40)
            kept = hitl_report.kalman_rows(run)
            self.assertEqual(len(kept), 20)                       # only the pre-reset segment
            self.assertTrue(all(not r.get("segment") for r in kept))

    def test_session_without_a_reset_keeps_every_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "fc_logs").mkdir()
            (run / "fc_logs" / "P0000001.BIN").write_bytes(
                b"".join(packet(SEC + i * 40_000, 1, kalman_frame()) for i in range(25)))
            self.assertEqual(len(hitl_report.kalman_rows(run)), 25)


if __name__ == "__main__":
    unittest.main()
