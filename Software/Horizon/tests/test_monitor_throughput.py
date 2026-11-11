"""The Witness monitor keeps up with fast streams, batches redraws and captures NMEA."""

import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from nmea_capture import NmeaExtractor, nmea_checksum, summarize_gga  # noqa: E402


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


class RingHistoryTests(unittest.TestCase):
    def test_bounded_contiguous_and_vector_width(self):
        from monitor_core import RingHistory
        history = RingHistory(5)
        for i in range(23):
            history.append(float(i), [i, -i, None])
        times, values = history.arrays()
        self.assertEqual(list(times), [18.0, 19.0, 20.0, 21.0, 22.0])
        self.assertEqual(values.shape, (5, 3))
        self.assertTrue(all(v != v for v in values[:, 2]))     # None -> NaN
        self.assertEqual(history[-1][1][:2], [22.0, -22.0])
        history.append(23.0, 7)                                 # width change resets
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0], (23.0, 7.0))


class MonitorThroughputTests(unittest.TestCase):
    """The Qt monitor (main.MonitorPanel over monitor_core.MonitorCore)."""

    def setUp(self):
        from horizon_qt_support import make_monitor
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        log_patch = patch("monitor_core.LOG_DIR", Path(self.directory.name))
        log_patch.start()
        self.addCleanup(log_patch.stop)
        self.monitor = make_monitor(self.directory.name)
        self.addCleanup(self.monitor.close)

    def test_backlog_drains_in_one_tick_and_shows_newest(self):
        monitor = self.monitor
        base = time.time() - 10.0
        for i in range(3000):
            packet = {"packet": "heartbeat", "status": 0x0102, "uptime": i}
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
        _times, values = history.arrays()
        self.assertEqual(values.shape, (monitor.max_history, 1))
        self.assertEqual(values[-1, 0], 2999)
        self.assertLessEqual(monitor.console.line_count(), monitor.max_console_lines + 2)
        # The status decoder shows the newest status word (Coast, HB phase flag).
        indicator = monitor.status_indicators["packet.heartbeat.status"]
        self.assertIn("#6baed6", indicator.states[2].styleSheet())
        self.assertIn("#74c476", indicator.flags[0x01].styleSheet())

    def test_console_stays_bounded_over_many_ticks(self):
        monitor = self.monitor
        for tick in range(20):
            for i in range(300):
                monitor.events.put(("bytes", ("COM_TEST", bytes([tick, i % 256]))))
            monitor.process_events()
        self.assertLessEqual(monitor.console.line_count(), monitor.max_console_lines + 2)

    def test_nmea_sentences_are_logged_and_summarized(self):
        monitor = self.monitor
        extractor = NmeaExtractor()
        monitor._queue_nmea("TCP_bench_7100", extractor, sentence(GGA) + sentence(RMC))
        monitor.process_events()
        files = list(Path(self.directory.name).glob("nmea_TCP_bench_7100_*.csv"))
        self.assertEqual(len(files), 1)
        monitor._close_nmea_file("TCP_bench_7100")
        rows = files[0].read_text(encoding="ascii").splitlines()
        self.assertEqual(rows[0], "received_utc,checksum_ok,sentence")
        self.assertEqual(len(rows), 3)
        self.assertTrue(rows[1].endswith(sentence(GGA).decode().strip()))
        self.assertIn("2 sentences (0 damaged)", monitor.nmea_status.get())
        self.assertIn("sats 8", monitor.nmea_status.get())
        self.assertIn("sats 8", monitor.nmea_label.text())

    def test_high_rate_live_packets_are_batched_not_redrawn_per_packet(self):
        """N packets from a worker thread produce far fewer graph redraws."""
        from horizon_qt_support import pump
        monitor = self.monitor
        monitor.check_vars["packet.sensors.filtered_accel"].set(True)
        monitor.tabs.setCurrentWidget(monitor.graph_page)
        pump(0.1)
        draws_before = monitor.graph_draws
        count = 4000

        def produce():
            for i in range(count):
                monitor.on_decoded_packet("COM_FAST", {"packet": "sensors", "status": 0x0201, "uptime": i,
                                                       "filtered_accel": [i * 1e-3, 0.0, 1.0],
                                                       "gyro": [0.0, 0.0, 0.0]})
        started = time.monotonic()
        worker = threading.Thread(target=produce)
        worker.start()
        while worker.is_alive() or not monitor.events.empty():
            pump(0.02)
        pump(0.2)
        worker.join()
        elapsed = time.monotonic() - started
        draws = monitor.graph_draws - draws_before
        self.assertEqual(monitor.values["packet.sensors.uptime"].get(), str(count - 1))
        self.assertGreaterEqual(draws, 1)
        # Throttled to <= 20 Hz: bounded by elapsed time, never one per packet.
        self.assertLessEqual(draws, elapsed / 0.05 + 3)
        self.assertLess(draws, count / 20)
        plot = monitor._plots["packet.sensors.filtered_accel"]
        self.assertEqual(len(plot.curves), 3)
        self.assertEqual(len(plot.curves[0].xData), monitor.max_history)

    def test_graphs_tab_hidden_skips_redraw(self):
        monitor = self.monitor
        monitor.check_vars["packet.heartbeat.uptime"].set(True)
        monitor.tabs.setCurrentIndex(0)
        draws = monitor.graph_draws
        monitor.draw_graphs()
        self.assertEqual(monitor.graph_draws, draws)
        self.assertTrue(monitor.core.graphs_dirty)
        monitor.draw_graphs(force=True)
        self.assertEqual(monitor.graph_draws, draws + 1)
        self.assertIn("packet.heartbeat.uptime", monitor._plots)
        monitor.check_vars["packet.heartbeat.uptime"].set(False)
        monitor.draw_graphs(force=True)
        self.assertNotIn("packet.heartbeat.uptime", monitor._plots)
        self.assertFalse(monitor.no_fields_label.isHidden())

    def test_graph_setup_save_load_and_commands(self):
        monitor = self.monitor
        monitor.check_vars["packet.state.kalman_altitude"].set(True)
        path = Path(self.directory.name) / "setup.json"
        monitor.core.save_graph_setup(path)
        monitor.check_vars["packet.state.kalman_altitude"].set(False)
        path.write_text(path.read_text().replace('"packet.state.kalman_altitude"',
                                                 '"packet.state.kalman_altitude", "nope"'))
        self.assertEqual(monitor.core.load_graph_setup(path), ["nope"])
        self.assertTrue(monitor.check_vars["packet.state.kalman_altitude"].get())
        # Commands: disabled without a target, typed hex sent to the selected input.
        self.assertFalse(monitor.command_entry.isEnabled())
        sent = []
        port = type("Port", (), {"write": lambda self, data: sent.append(data), "close": lambda self: None})()
        monitor.serials["COM_X"] = port
        monitor.refresh_command_targets()
        self.assertTrue(monitor.command_entry.isEnabled())
        self.assertEqual(monitor.command_port_combo.currentText(), "COM_X")
        monitor.command_entry.setText("0x05, 43 53")
        monitor.send_hex_command()
        self.assertEqual(sent, [b"\x05\x43\x53"])
        self.assertEqual(monitor.command_entry.text(), "")
        monitor.recall_command(-1)
        self.assertEqual(monitor.command_entry.text(), "05 43 53")
        monitor.recall_command(1)
        self.assertEqual(monitor.command_entry.text(), "")
        with patch("main.ask_yes_no", return_value=False):
            erase = next(b for b in monitor.quick_command_buttons if b.text() == "Erase Logs (full, slow)")
            erase.click()
        self.assertEqual(len(sent), 1)          # declined confirmation sends nothing
        monitor.quick_command_buttons[1].click()   # CH1 On
        self.assertEqual(int.from_bytes(sent[-1][7:9], "big"), 0x0031)
        # GPS ephemeris upload: chunked assist frames, on a worker thread.
        rinex = Path(__file__).resolve().parents[4] / "gps-sdr-sim" / "brdc0010.22n"
        if rinex.exists():
            import datetime as dt
            import gps_assist
            from horizon_qt_support import pump_until
            before = len(sent)
            when = dt.datetime(2022, 1, 1, 0, 30)
            with patch("gps_assist.gps_now", return_value=when):
                self.assertTrue(monitor.upload_gps_ephemeris(str(rinex)))
            nav = gps_assist.parse_rinex_nav(rinex)
            blob, _prns = gps_assist.ephemeris_upload(nav, when)
            expected = len(gps_assist.assist_frames(blob)) + 1      # + STATUS
            self.assertTrue(pump_until(lambda: len(sent) - before >= expected, 20))
            frames = sent[before:]
            self.assertEqual(frames[0][7:9], b"GA")
            self.assertEqual(frames[-1][7:9], b"GQ")
            self.assertTrue(all(len(f) == 10 and f[-1] == 0x0A for f in frames))
        monitor.serials.clear()

    def test_panel_api_used_by_witnessos(self):
        """MonitorPanel keeps the API witnessOS mirrors the HITL link with."""
        from horizon_qt_support import pump_until
        import main
        titles = []
        with patch("monitor_core.DEFAULT_WITNESS_FEED_PORT", 0):
            panel = main.MonitorPanel(None, on_title=titles.append, auto_connect=False, start_feed=True)
        self.addCleanup(panel.close)
        self.assertIn("receive only", panel.feed_status.get())
        self.assertGreater(panel.publisher.port, 0)
        feed = panel.stream_feeder("HITL")
        feed(bytes.fromhex("ff 6e 00 00 00 10 92 0a"))     # heartbeat, uptime 4242
        self.assertTrue(pump_until(lambda: panel.values["packet.heartbeat.uptime"].get() == "4242"))
        panel.console_write("MIRROR test line")
        panel.process_events()
        self.assertIn("MIRROR test line", panel.console.toPlainText())
        panel.check_vars["packet.heartbeat.uptime"].set(True)
        panel.draw_graphs()
        self.assertTrue(panel.start_test())
        self.assertEqual(titles[-1], main.WINDOW_TITLE + " [TEST DATA]")
        self.assertFalse(panel.connect_button.isEnabled())
        panel.close()
        self.assertTrue(panel._closed)
        feed(b"\xff")        # ignored once closed
        self.assertFalse(panel.publisher._thread.is_alive())

    def test_main_script_points_to_witnessos(self):
        import main
        with patch("builtins.print") as printed:
            self.assertEqual(main.main([]), 0)
        self.assertIn("witnessos.py --tab monitor", printed.call_args[0][0])
        self.assertFalse(hasattr(main, "SerialMonitor"))
        for name in ("parse_packet", "PacketDecoder", "build_command_packet", "build_quick_command_packet",
                     "build_imu_calibration_packet", "QUICK_COMMANDS", "IMU_CALIBRATION_COMMANDS"):
            self.assertTrue(hasattr(main, name), name)


if __name__ == "__main__":
    unittest.main()
