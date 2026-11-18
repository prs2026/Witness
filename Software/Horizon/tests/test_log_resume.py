"""NAND flight-log readers after the logger fixes (audit 2026-11-15, fix 01).

The streams are built byte for byte from the firmware layouts in
Software/Witness/softwarev1/main/flash_logger.{h,cpp}:

* P*.BIN packet records: [u32 us BE][u8 source][u8 length][frame]; source 6 is
  the "resumed" marker, frame = [u8 kind=1][u8 flight state][u32 generation BE].
* I*.BIN IMU records: 24 bytes, u32 us, 9 x int16, u8 status, u8 sequence.
* S*.BIN WL v2 records: 24 bytes, "WL", version 2, ..., u32 sequence, u32 ms.
* The exported file is the page payloads (2036 bytes per full page, no page
  headers) concatenated; a missing page is simply absent from the stream.
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import converter  # noqa: E402

PAGE_PAYLOAD = 2036            # kPagePayloadSize = 2048 - 12
MARKER_SOURCE = 6
SEC = 1_000_000


def packet(us, source, frame):
    return struct.pack(">IBB", us & 0xFFFFFFFF, source, len(frame)) + frame


def state_frame(uptime_ms, status=0x0008):
    # 0x02 + 63-byte payload (status, uptime, ...) + EOF: a 65-byte frame.
    return bytes([0x02]) + struct.pack(">HI", status, uptime_ms) + bytes(57) + bytes([0x0A])


def kalman_frame(gps_utc):
    # 0xE2 + 101-byte payload (gps_time at payload[93:97]) + EOF: 103 bytes.
    return bytes([0xE2]) + bytes(93) + struct.pack(">I", gps_utc) + bytes(4) + bytes([0x0A])


def marker(us, state=2, generation=7, kind=1):
    return packet(us, MARKER_SOURCE, bytes([kind, state]) + struct.pack(">I", generation))


def imu(us, sequence, state=2):
    return struct.pack(">I9hBB", us & 0xFFFFFFFF, *([0] * 9), 0x70 | state, sequence & 0xFF)


def wl(sequence, uptime_ms, state=2):
    return (b"WL" + bytes([2, 0x46, 0x08 | state, 0])
            + struct.pack(">IIhii", sequence, uptime_ms, 512, 85602, 2150))


def drop_page(data, page):
    pages = [data[i:i + PAGE_PAYLOAD] for i in range(0, len(data), PAGE_PAYLOAD)]
    del pages[page]
    return b"".join(pages)


def run_packets(data):
    stats = converter.DecodeStats()
    return list(converter.iter_packet_log(data, stats)), stats


def run_imu(data):
    stats = converter.DecodeStats()
    return list(converter.iter_imu_log(data, stats)), stats


def run_wl(data):
    stats = converter.DecodeStats()
    return list(converter.iter_flash_log(data, stats)), stats


def increasing(values):
    return all(b > a for a, b in zip(values, values[1:]))


class NormalSessionTest(unittest.TestCase):
    """No resume: output and columns are exactly the old ones."""

    def test_packets_have_no_resume_columns(self):
        data = b"".join(packet(1000 + i * 20_000, 1, state_frame(i * 20)) for i in range(50))
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 50)
        self.assertEqual(stats.segments, 1)
        self.assertEqual((stats.discarded, stats.gap_events, stats.unknown_records), (0, 0, 0))
        extra = {"segment", "time_after_reset_estimated", "resumed", "device_timestamp_us",
                 "gap_before_bytes"}
        for row in rows:
            self.assertFalse(extra & set(row))
        self.assertAlmostEqual(rows[-1]["time_seconds"], 49 * 0.02, places=6)

    def test_imu_has_no_resume_columns(self):
        data = b"".join(imu(5_000_000 + i * 1042, i) for i in range(400))
        rows, stats = run_imu(data)
        self.assertEqual(len(rows), 400)
        self.assertEqual(stats.segments, 1)
        self.assertEqual(stats.gap_events, 0)
        self.assertTrue(all(r["missing_before"] == 0 for r in rows))
        self.assertFalse({"segment", "resumed"} & set(rows[0]))
        self.assertAlmostEqual(rows[-1]["time_seconds"], 399 * 0.001042, places=6)

    def test_wl_uptime_is_time(self):
        data = b"".join(wl(i, 1000 + 20 * i) for i in range(100))
        rows, stats = run_wl(data)
        self.assertEqual(len(rows), 100)
        self.assertEqual(rows[5]["time_seconds"], 1.1)
        self.assertEqual(stats.segments, 1)
        self.assertNotIn("segment", rows[0])


class ResumeMarkerTest(unittest.TestCase):
    def build(self, first_us=40 * SEC, count_before=30, count_after=30):
        """Segment 0: state packets 20 ms apart from first_us. Reset. Marker at
        4.0 s of the new clock, then packets again."""
        before = [packet(first_us + i * 20_000, 1, state_frame(first_us // 1000 + i * 20))
                  for i in range(count_before)]
        after = [marker(4 * SEC)]
        after += [packet(4 * SEC + 20_000 + i * 20_000, 1, state_frame(4000 + 20 + i * 20))
                  for i in range(count_after)]
        return b"".join(before + after), first_us + (count_before - 1) * 20_000

    def test_marker_row_and_continuous_time(self):
        data, last_raw = self.build()
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 30 + 1 + 30)
        self.assertEqual(stats.segments, 2)
        self.assertEqual((stats.discarded, stats.unknown_records), (0, 0))
        marker_rows = [r for r in rows if r["packet_type"] == "log_resumed"]
        self.assertEqual(len(marker_rows), 1)
        m = marker_rows[0]
        self.assertEqual(m["packet_id"], "LOG")
        self.assertEqual(m["log_source"], "log_marker")
        self.assertEqual(m["resumed"], 1)
        self.assertEqual(m["resume_flight_state"], 2)
        self.assertEqual(m["resume_flight_state_name"], "Coast")
        self.assertEqual(m["resume_generation"], 7)
        self.assertEqual(m["resume_gap_s"], 1.0)
        self.assertEqual(m["resume_gap_source"], "nominal")
        self.assertEqual(m["time_after_reset_estimated"], 1)
        self.assertEqual(m["segment"], 1)
        self.assertEqual(m["raw_packet_hex"], "01 02 00 00 00 07")
        # Monotonic time axis across the reset; the old clock's last time plus
        # the nominal 1.0 s is where the restarted clock's zero lands.
        times = [r["time_seconds"] for r in rows]
        self.assertTrue(increasing(times))
        last_before = rows[29]["time_seconds"]
        self.assertAlmostEqual(last_before, 29 * 0.02, places=6)
        self.assertAlmostEqual(m["time_seconds"], last_before + 1.0 + 4.0, places=6)
        self.assertAlmostEqual(rows[-1]["time_seconds"], last_before + 1.0 + 4.0 + 30 * 0.02, places=6)
        # Rows of the first segment have no resume columns, later rows do.
        self.assertNotIn("segment", rows[0])
        self.assertEqual(rows[-1]["segment"], 1)
        self.assertEqual(rows[-1]["time_after_reset_estimated"], 1)
        self.assertEqual(rows[-1]["device_timestamp_us"], 4 * SEC + 20_000 + 29 * 20_000)
        # The summary a CLI/viewer shows.
        self.assertEqual(len(stats.resumes), 1)
        self.assertEqual(stats.resumes[0]["generation"], 7)
        self.assertTrue(any("Resumed" in line and "ESTIMATED" in line
                            for line in converter.describe_stats(stats)))

    def test_gps_utc_gives_the_true_gap(self):
        utc0 = 1_800_000_000
        pre = []
        for i in range(40):                                  # raw 10 s .. 49.5 s, 0.5 s steps
            t = 10 * SEC + i * 500_000
            pre.append(packet(t, 5, kalman_frame(utc0 + (t + 499_999) // SEC)))
        last_raw_s = (10 * SEC + 39 * 500_000) / SEC          # 29.5
        true_gap = 6.0                                        # clock zero is 6 s after the last record
        utc_zero = utc0 + last_raw_s + true_gap               # 1_800_000_035.5
        post = [marker(2 * SEC)]
        for i in range(10):                                   # raw 3 s ...
            t = 3 * SEC + i * 500_000
            post.append(packet(t, 5, kalman_frame(int(round(utc_zero + t / SEC)))))
        rows, stats = run_packets(b"".join(pre + post))
        m = next(r for r in rows if r["packet_type"] == "log_resumed")
        self.assertEqual(m["resume_gap_source"], "gps_utc")
        self.assertAlmostEqual(m["resume_gap_s"], true_gap, delta=1.0)   # UTC is whole seconds
        self.assertEqual(m["time_after_reset_estimated"], 0)
        self.assertEqual(rows[-1]["time_after_reset_estimated"], 0)
        self.assertEqual(stats.resumes[0]["gap_source"], "gps_utc")
        # Segment 0 spans raw 10 s..29.5 s => 19.5 s on the axis; the marker is
        # gap + 2 s of the new clock later.
        self.assertAlmostEqual(m["time_seconds"], 19.5 + m["resume_gap_s"] + 2.0, places=5)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))

    def test_gps_time_held_across_the_reset_falls_back(self):
        # The same UTC before and after: an implausible (negative) gap.
        pre = [packet(10 * SEC + i * 500_000, 5, kalman_frame(1_800_000_000)) for i in range(10)]
        post = [marker(2 * SEC)] + [packet(3 * SEC + i * 500_000, 5, kalman_frame(1_800_000_000))
                                    for i in range(5)]
        rows, _ = run_packets(b"".join(pre + post))
        m = next(r for r in rows if r["packet_type"] == "log_resumed")
        self.assertEqual(m["resume_gap_source"], "nominal")
        self.assertEqual(m["resume_gap_s"], 1.0)

    def test_two_resets(self):
        data = (b"".join(packet(30 * SEC + i * 100_000, 1, state_frame(i)) for i in range(10))
                + marker(3 * SEC, state=3, generation=9)
                + b"".join(packet(5 * SEC + i * 100_000, 1, state_frame(i)) for i in range(10))
                + marker(2 * SEC, state=3, generation=9)
                + b"".join(packet(4 * SEC + i * 100_000, 1, state_frame(i)) for i in range(10)))
        rows, stats = run_packets(data)
        self.assertEqual(stats.segments, 3)
        self.assertEqual(len(stats.resumes), 2)
        self.assertEqual([r["segment"] for r in rows if "segment" in r][-1], 2)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))

    def test_marker_first_in_file_does_not_raise(self):
        # The pre-reset pages may be gone (ring overwrite): nothing to continue from.
        data = marker(3 * SEC) + b"".join(packet(3 * SEC + 20_000 * (i + 1), 1, state_frame(i))
                                          for i in range(5))
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 6)
        self.assertEqual(rows[0]["resume_gap_source"], "no_previous_data")
        self.assertEqual(rows[0]["resume_gap_s"], "")
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))
        self.assertEqual(stats.resumes[0]["gap_s"], None)

    def test_marker_with_unknown_kind_is_skipped_not_a_boundary(self):
        data = (packet(1000, 1, state_frame(1)) + marker(2000, kind=9)
                + packet(3000, 1, state_frame(2)))
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 2)
        self.assertEqual(stats.segments, 1)
        self.assertEqual(stats.unknown_sources, {"6:kind 9": 1})

    def test_imu_restart_is_a_segment_and_sequence_restart_is_not_missing(self):
        before = [imu(120 * SEC + i * 1042, 250 + i) for i in range(20)]       # sequence wraps 255 -> 0
        after = [imu(6 * SEC + i * 1042, i) for i in range(20)]                 # clock and sequence restart
        rows, stats = run_imu(b"".join(before + after))
        self.assertEqual(len(rows), 40)
        self.assertEqual(stats.segments, 2)
        self.assertEqual(stats.resumes[0]["gap_source"], "nominal")
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))
        first_after = rows[20]
        self.assertEqual(first_after["resumed"], 1)
        self.assertEqual(first_after["missing_before"], 0)
        self.assertEqual(first_after["segment"], 1)
        self.assertEqual(first_after["time_after_reset_estimated"], 1)
        self.assertTrue(all(r["missing_before"] == 0 for r in rows))
        self.assertAlmostEqual(first_after["time_seconds"],
                               rows[19]["time_seconds"] + 1.0 + 6.0, places=5)

    def test_wl_restart_is_a_segment(self):
        before = [wl(i, 90_000 + 20 * i) for i in range(50)]
        after = [wl(i, 3_000 + 20 * i, state=3) for i in range(50)]
        rows, stats = run_wl(b"".join(before + after))
        self.assertEqual(len(rows), 100)
        self.assertEqual(stats.segments, 2)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))
        self.assertEqual(rows[50]["resumed"], 1)
        self.assertAlmostEqual(rows[50]["time_seconds"], rows[49]["time_seconds"] + 1.0 + 3.0)
        self.assertEqual(rows[50]["segment"], 1)


class UnknownRecordTest(unittest.TestCase):
    def test_unknown_sources_are_skipped_by_length(self):
        unknown_a = packet(2000, 9, bytes(range(7)))           # not ending in 0x0A
        unknown_b = packet(2500, 0x42, b"\x0a" * 3 + b"\x00")
        data = (packet(1000, 1, state_frame(1)) + unknown_a + unknown_b
                + packet(3000, 2, bytes([0x05, 0, 0, 0, 0, 0, 0, 0x73, 0x01, 0x0A]))
                + unknown_a + packet(4000, 1, state_frame(2)))
        rows, stats = run_packets(data)
        self.assertEqual([r["log_source"] for r in rows], ["witness_state", "can_rx", "witness_state"])
        self.assertEqual(stats.unknown_records, 3)
        self.assertEqual(stats.unknown_sources, {"9": 2, "66": 1})
        self.assertEqual(stats.discarded, 0)
        self.assertEqual(stats.gap_events, 0)
        self.assertTrue(any("unknown" in line for line in converter.describe_stats(stats)))

    def test_unknown_source_zero_length_at_the_end(self):
        data = packet(1000, 1, state_frame(1)) + struct.pack(">IBB", 2000, 12, 0)
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 1)
        self.assertEqual(stats.unknown_records, 1)
        self.assertEqual(stats.discarded, 0)


class WrapTest(unittest.TestCase):
    def test_packet_clock_wraps_without_a_boundary(self):
        start = 2**32 - 3 * SEC                                    # wraps 3 s in
        data = b"".join(packet(start + i * 500_000, 1, state_frame(i)) for i in range(12))
        rows, stats = run_packets(data)
        self.assertEqual(stats.segments, 1)
        self.assertEqual(stats.time_wraps, 1)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))
        self.assertAlmostEqual(rows[-1]["time_seconds"], 11 * 0.5, places=6)
        self.assertNotIn("segment", rows[-1])

    def test_imu_clock_wraps_without_a_boundary(self):
        start = 2**32 - 5000
        data = b"".join(imu(start + i * 1042, i) for i in range(20))
        rows, stats = run_imu(data)
        self.assertEqual(stats.segments, 1)
        self.assertEqual(stats.time_wraps, 1)
        self.assertEqual(rows[-1]["timestamp_us"] - rows[0]["timestamp_us"], 19 * 1042)
        self.assertTrue(all(r["missing_before"] == 0 for r in rows))

    def test_restart_after_a_wrap_is_still_a_restart(self):
        # 71.6 min of flight, wrapped once, then a reset (clock back to ~3 s).
        pre = [imu((2**32 - 3000 + i * 100_000) % 2**32, i) for i in range(100)]   # 10 s past the wrap
        post = [imu(3 * SEC + i * 1042, i) for i in range(10)]
        rows, stats = run_imu(b"".join(pre + post))
        self.assertEqual(stats.segments, 2)
        self.assertEqual(stats.time_wraps, 1)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))

    def test_restart_from_near_the_top_of_the_counter(self):
        # The last record before a reset is just under 2^32 and the new clock
        # starts at 1 s: forward by ~1 s, i.e. a wrap, not a restart.
        pre = [imu(2**32 - 2 * SEC + i * 1042, i) for i in range(5)]
        post = [imu(SEC + i * 1042, 5 + i) for i in range(5)]
        rows, stats = run_imu(b"".join(pre + post))
        self.assertEqual(stats.segments, 1)           # indistinguishable from a wrap
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))


class MissingPageTest(unittest.TestCase):
    def test_imu_stream_realigns_after_a_missing_page(self):
        data = b"".join(imu(10 * SEC + i * 1042, i) for i in range(3000))
        self.assertNotEqual(PAGE_PAYLOAD % 24, 0)
        damaged = drop_page(data, 1)
        rows, stats = run_imu(damaged)
        self.assertEqual(stats.gap_events, 1)
        self.assertGreater(stats.discarded, 0)
        self.assertLess(stats.discarded, 3 * 24)
        # Every decoded record is a real one: its timestamp is a multiple of 1042 from
        # the start and its sequence matches its index.
        for row in rows:
            index = (row["timestamp_us"] - 10 * SEC) // 1042
            self.assertEqual((row["timestamp_us"] - 10 * SEC) % 1042, 0)
            self.assertEqual(row["sequence"], index % 256)
        # Only the page's records (about 85), the two torn ones and nothing else are lost.
        lost = 3000 - len(rows)
        self.assertTrue(PAGE_PAYLOAD // 24 <= lost <= PAGE_PAYLOAD // 24 + 3, lost)
        self.assertEqual(rows[-1]["timestamp_us"], 10 * SEC + 2999 * 1042)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))
        self.assertTrue(any(r.get("gap_before_bytes") for r in rows))

    def test_imu_missing_page_then_reset(self):
        pre = b"".join(imu(80 * SEC + i * 1042, i) for i in range(1500))
        post = b"".join(imu(5 * SEC + i * 1042, i) for i in range(1000))
        rows, stats = run_imu(drop_page(pre, 0) + post)
        self.assertEqual(stats.segments, 2)
        self.assertEqual(stats.gap_events, 1)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))

    def test_packet_stream_realigns_after_a_missing_page(self):
        records = [packet(1 * SEC + i * 20_000, 1, state_frame(i * 20)) for i in range(120)]
        data = b"".join(records)
        rows, stats = run_packets(drop_page(data, 1))
        self.assertEqual(stats.gap_events, 1)
        self.assertGreater(stats.discarded, 0)
        self.assertLess(stats.discarded, 2 * 71)
        uptimes = [struct.unpack(">I", bytes.fromhex(r["raw_packet_hex"])[3:7])[0] for r in rows]
        self.assertTrue(increasing(uptimes))
        self.assertEqual(uptimes[-1], 119 * 20)
        self.assertEqual(uptimes[0], 0)
        self.assertTrue(all(r["packet_type"] == "state" for r in rows))
        lost = 120 - len(rows)
        self.assertTrue(PAGE_PAYLOAD // 71 <= lost <= PAGE_PAYLOAD // 71 + 2, lost)
        self.assertTrue(any(r.get("gap_before_bytes") for r in rows))

    def test_packet_missing_page_keeps_the_resume(self):
        records = [packet(40 * SEC + i * 20_000, 1, state_frame(i)) for i in range(60)]
        records.append(marker(3 * SEC))
        records += [packet(3 * SEC + 20_000 * (i + 1), 1, state_frame(i)) for i in range(60)]
        rows, stats = run_packets(drop_page(b"".join(records), 1))
        self.assertEqual(stats.segments, 2)
        self.assertEqual(stats.gap_events, 1)
        self.assertTrue(increasing([r["time_seconds"] for r in rows]))

    def test_wl_stream_realigns_after_a_missing_page(self):
        data = b"".join(wl(i, 1000 + 20 * i) for i in range(400))
        rows, stats = run_wl(drop_page(data, 1))
        self.assertEqual(stats.gap_events, 1)
        sequences = [r["sequence"] for r in rows]
        self.assertTrue(increasing(sequences))
        self.assertEqual(sequences[-1], 399)
        for row in rows:                                     # no torn record decoded
            self.assertEqual(row["uptime"], 1000 + 20 * row["sequence"])
        lost = 400 - len(rows)
        self.assertTrue(PAGE_PAYLOAD // 24 <= lost <= PAGE_PAYLOAD // 24 + 2, lost)

    def test_truncated_tail_is_still_discarded(self):
        data = packet(10, 1, state_frame(1)) + packet(20, 3, state_frame(2))[:-3]
        rows, stats = run_packets(data)
        self.assertEqual(len(rows), 1)
        self.assertEqual(stats.discarded, 71 - 3)
        self.assertEqual(stats.gap_events, 0)


class CsvOutputTest(unittest.TestCase):
    def test_resume_columns_in_the_csv_header(self):
        data = (packet(1000, 1, state_frame(1)) + marker(500) + packet(900, 1, state_frame(2)))
        rows, _ = run_packets(data)
        fields = converter.csv_fieldnames(rows)
        for name in ("segment", "time_after_reset_estimated", "resumed", "resume_gap_s",
                     "resume_flight_state", "resume_generation", "device_timestamp_us"):
            self.assertIn(name, fields)
        self.assertEqual(fields[-1], "raw_packet_hex")

    def test_plain_log_header_has_no_new_columns(self):
        rows, _ = run_packets(packet(1000, 1, state_frame(1)) + packet(2000, 1, state_frame(2)))
        fields = converter.csv_fieldnames(rows)
        for name in ("segment", "time_after_reset_estimated", "resumed", "gap_before_bytes"):
            self.assertNotIn(name, fields)


class ViewerAndCliTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        from pathlib import Path
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(self.tmp, ignore_errors=True))

    def resumed_packets(self):
        return (b"".join(packet(40 * SEC + i * 20_000, 1, state_frame(40_000 + i * 20))
                         for i in range(40))
                + marker(4 * SEC)
                + b"".join(packet(4 * SEC + 20_000 * (i + 1), 1, state_frame(4_000 + i * 20))
                           for i in range(40))
                + packet(5 * SEC, 9, b""))

    def test_viewer_loads_a_resumed_log_on_one_axis(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import viewer_data
        path = self.tmp / "P0000001.BIN"
        path.write_bytes(self.resumed_packets())
        dataset = viewer_data.load_bin_dataset(path)
        self.assertEqual(dataset.row_count, 81)
        self.assertIn("Resumed after an in-flight reset", dataset.decode_notes)
        self.assertIn("unknown type", dataset.decode_notes)
        self.assertIn("segment", dataset.numeric_columns)
        times = dataset.numeric_data["time_seconds"]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))

    def test_cli_prints_the_resume_and_writes_the_csv(self):
        import csv
        import subprocess
        path = self.tmp / "P0000002.BIN"
        path.write_bytes(self.resumed_packets())
        out = self.tmp / "out.csv"
        result = subprocess.run([sys.executable, "-I", os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "converter.py"),
            str(path), "--output", str(out)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Resumed after an in-flight reset", result.stdout)
        self.assertIn("log_resumed=1", result.stdout)
        self.assertIn("Skipped 1 record(s) of unknown type (9=1)", result.stdout)
        with out.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 81)
        self.assertEqual(rows[40]["packet_type"], "log_resumed")
        self.assertEqual(rows[40]["resume_generation"], "7")
        self.assertEqual(rows[0]["segment"], "")


if __name__ == "__main__":
    unittest.main()
