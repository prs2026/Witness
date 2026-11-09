"""The Witness monitor keeps up with fast streams and captures NMEA."""

import gc
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from nmea_capture import NmeaExtractor, nmea_checksum, summarize_gga


def sentence(body: str) -> bytes:
    return f"${body}*{nmea_checksum(body.encode()):02X}\r\n".encode()


GGA = "GNGGA,123519.00,3349.27782,N,11308.25745,W,1,08,0.9,680.5,M,0.0,M,,"
RMC = "GNRMC,123519.00,A,3349.27782,N,11308.25745,W,000.0,000.0,081126,,,A"


class NmeaExtractorTests(unittest.TestCase):
    def test_sentences_split_across_chunks(self):
        extractor = NmeaExtractor()
        data = sentence(GGA) + sentence(RMC)
        found = []
        for i in range(0, len(data), 7):
            found += extractor.feed(data[i:i + 7])
        self.assertEqual([s for s, _ in found], [f"${GGA}*{nmea_checksum(GGA.encode()):02X}",
                                                 f"${RMC}*{nmea_checksum(RMC.encode()):02X}"])
        self.assertTrue(all(ok for _, ok in found))
        self.assertEqual((extractor.valid, extractor.bad), (2, 0))

    def test_bad_checksum_is_reported_not_trusted(self):
        extractor = NmeaExtractor()
        bad = sentence(GGA).replace(b",08,", b",09,")
        found = extractor.feed(bad)
        self.assertEqual(len(found), 1)
        self.assertFalse(found[0][1])
        self.assertEqual(extractor.bad, 1)

    def test_binary_frame_inside_sentence_is_damaged(self):
        # A 0x01 telemetry frame spliced into the middle of a sentence.
        raw = sentence(GGA)
        spliced = raw[:20] + bytes([0x01, 0x6E, 0x00, 0x0A]) + raw[20:]
        found = NmeaExtractor().feed(spliced)
        self.assertFalse(any(ok for _, ok in found))

    def test_interleaved_binary_and_log_text(self):
        stream = (bytes([0x01]) + bytes(21) + b"\n" + b"I (123) uart: GPS RX: baud=115200\n"
                  + sentence(GGA) + bytes([0xFF, 0x00, 0x01, 0, 0, 0, 5, 0x0A]))
        found = NmeaExtractor().feed(stream)
        self.assertEqual(len(found), 1)
        self.assertTrue(found[0][1])

    def test_gga_summary(self):
        text = sentence(GGA).decode().strip()
        self.assertEqual(summarize_gga(text), "GGA 12:35:19 UTC, fix 1, sats 8, alt 680.5 m")
        self.assertIsNone(summarize_gga(sentence(RMC).decode().strip()))


class MonitorThroughputTests(unittest.TestCase):
    def make_monitor(self, log_dir):
        from main import SerialMonitor
        with patch("main.DEFAULT_WITNESS_FEED_PORT", 0), patch("main.LOG_DIR", Path(log_dir)):
            monitor = SerialMonitor(auto_connect=False)
        monitor.withdraw()
        return monitor

    def close_monitor(self, monitor):
        for callback in monitor.tk.call("after", "info"):
            monitor.after_cancel(callback)
        monitor.close()

    def tearDown(self):
        # Finalize this test's Tk variables now, on the main thread. Left to
        # the garbage collector they can be deleted from a background thread
        # later, which blocks on Tk mid-import and stalls later GUI tests.
        gc.collect()

    def test_backlog_drains_in_one_tick_and_shows_newest(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = self.make_monitor(directory)
            try:
                base = time.time() - 10.0
                for i in range(3000):
                    packet = {"packet": "heartbeat", "status": 1, "uptime": i}
                    monitor.events.put(("packet", ("COM_TEST", packet, base + i * 0.001)))
                    monitor.events.put(("bytes", ("COM_TEST", b"\xff\x00\x01")))
                started = time.monotonic()
                monitor.process_events()
                elapsed = time.monotonic() - started
                self.assertTrue(monitor.events.empty(), "one tick must drain the backlog")
                self.assertLess(elapsed, 5.0)
                self.assertEqual(monitor.values["packet.heartbeat.uptime"].get(), "2999")
                history = monitor.history["packet.heartbeat.uptime"]
                self.assertEqual(len(history), monitor.max_history)
                # Graph time is the arrival time, not when the GUI processed it.
                self.assertAlmostEqual(history[-1][0], base + 2.999, places=6)
                lines = int(monitor.console.index("end-1c").split(".")[0])
                self.assertLessEqual(lines, monitor.max_console_lines + 2)
            finally:
                self.close_monitor(monitor)

    def test_console_stays_bounded_over_many_ticks(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = self.make_monitor(directory)
            try:
                for tick in range(20):
                    for i in range(300):
                        monitor.events.put(("bytes", ("COM_TEST", bytes([tick, i % 256]))))
                    monitor.process_events()
                lines = int(monitor.console.index("end-1c").split(".")[0])
                self.assertLessEqual(lines, monitor.max_console_lines + 2)
            finally:
                self.close_monitor(monitor)

    def test_nmea_sentences_are_logged_and_summarized(self):
        with tempfile.TemporaryDirectory() as directory:
            monitor = self.make_monitor(directory)
            try:
                with patch("main.LOG_DIR", Path(directory)):
                    extractor = NmeaExtractor()
                    monitor._queue_nmea("TCP_bench_7100", extractor, sentence(GGA) + sentence(RMC))
                    monitor.process_events()
                    files = list(Path(directory).glob("nmea_TCP_bench_7100_*.csv"))
                    self.assertEqual(len(files), 1)
                    monitor._close_nmea_file("TCP_bench_7100")
                    rows = files[0].read_text(encoding="ascii").splitlines()
                self.assertEqual(rows[0], "received_utc,checksum_ok,sentence")
                self.assertEqual(len(rows), 3)
                self.assertTrue(rows[1].endswith(sentence(GGA).decode().strip()))
                self.assertIn("2 sentences (0 damaged)", monitor.nmea_status.get())
                self.assertIn("sats 8", monitor.nmea_status.get())
            finally:
                self.close_monitor(monitor)


if __name__ == "__main__":
    unittest.main()
