"""Link resilience (audit 2026-11-15, 11_horizon_data.md): reconnect, half-open
watchdog, raw-log survival, bounded debug log, parser robustness, exception hook,
command pacing. Loopback sockets on 127.0.0.1 and fakes only: no hardware, no real
serial port, no network host."""
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import link_health as lh  # noqa: E402
import witness_protocol as wp  # noqa: E402
from witness_protocol import PacketDecoder  # noqa: E402

_APP = None
CAPTURE = Path(__file__).resolve().parents[1] / "logs" / "TCP_prs2026hitl_7100_20261111_201412.bin"


UP = 0x20202020     # uptimes avoid bytes that look like packet IDs, so a false ID candidate in
                    # a garbage flood cannot (legitimately) swallow the frames after it


def heartbeat(n=1):
    return bytes((0xFF, 0, 0)) + struct.pack(">I", UP + n) + b"\x0a"


def wait_for(predicate, timeout=5.0, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


class ParserRobustness(unittest.TestCase):
    def decoder(self):
        self.packets, self.errors = [], []
        return PacketDecoder(self.packets.append, self.errors.append)

    def test_text_between_frames_is_skipped_without_errors(self):
        d = self.decoder()
        nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
        esp = b"I (12345) witness: boot ok\r\nW (12346) gps: no fix\r\n"
        stream = heartbeat(1) + nmea + heartbeat(2) + esp + heartbeat(3) + nmea * 50 + heartbeat(4)
        d.feed(stream)
        self.assertEqual([p.get("uptime", 0) - UP for p in self.packets], [1, 2, 3, 4])
        self.assertEqual(self.errors, [])
        self.assertGreater(d.stats()["text_bytes_skipped"], len(nmea) * 50)

    def test_garbage_flood_is_fast_and_one_event_per_run(self):
        for size in (4096, 16384):
            d = self.decoder()
            blob = bytes((i * 7 + 3) % 256 for i in range(size))
            start = time.perf_counter()
            d.feed(blob + b"".join(heartbeat(n) for n in range(9, 30)))
            elapsed = time.perf_counter() - start
            self.assertLess(elapsed, 0.5, f"{size} B flood took {elapsed:.2f}s (audit: 8.4 s)")
            self.assertEqual([p.get("uptime", 0) - UP for p in self.packets][-5:], [25, 26, 27, 28, 29])
            self.assertLess(len(self.errors), 50)            # not one per byte
            self.assertGreater(d.stats()["garbage_bytes_skipped"], size // 2)

    def test_random_flood_16k_and_resync(self):
        import random
        rng = random.Random(5)
        d = self.decoder()
        junk = bytes(rng.randrange(256) for _ in range(16384))
        start = time.perf_counter()
        d.feed(junk)
        d.feed(b"".join(heartbeat(n) for n in range(5, 30)))
        self.assertLess(time.perf_counter() - start, 0.5)
        self.assertEqual([p.get("uptime", 0) - UP for p in self.packets][-5:], [25, 26, 27, 28, 29])

    def test_callback_exception_does_not_eat_next_frames(self):
        seen = []

        def on_packet(packet):
            seen.append(packet["uptime"])
            if packet["uptime"] == UP + 1:
                raise RuntimeError("boom")
        errors = []
        d = PacketDecoder(on_packet, errors.append)
        d.feed(heartbeat(1) + heartbeat(2) + heartbeat(3))
        seen[:] = [u - UP for u in seen]
        self.assertEqual(seen, [1, 2, 3])          # audit: 3 good frames delivered 0
        self.assertEqual(d.stats()["callback_errors"], 1)
        self.assertEqual(len(errors), 1)

    def test_parse_exception_does_not_eat_next_frames(self):
        d = self.decoder()
        original = wp.parse_packet
        calls = []

        def flaky(packet_id, payload):
            calls.append(1)
            if len(calls) == 1:
                raise ValueError("bad payload")
            return original(packet_id, payload)
        with patch("witness_protocol.parse_packet", flaky):
            d.feed(heartbeat(1) + heartbeat(2) + heartbeat(3))
        self.assertEqual([p.get("uptime", 0) - UP for p in self.packets], [2, 3])

    def test_counters_for_bad_eof_and_split_frames(self):
        d = self.decoder()
        bad = bytearray(heartbeat(1))
        bad[-1] = 0x55                                   # known id, wrong EOF
        d.feed(bytes(bad) + heartbeat(2))
        self.assertEqual([p.get("uptime", 0) - UP for p in self.packets], [2])
        self.assertEqual(d.stats()["bad_frames"], {0xFF: 1})
        d2 = self.decoder()
        frame = heartbeat(7)
        for byte in frame:                               # one byte at a time
            d2.feed(bytes((byte,)))
        self.assertEqual([p.get("uptime", 0) - UP for p in self.packets], [7])

    @unittest.skipUnless(CAPTURE.exists(), "real capture not present")
    def test_real_capture_is_linear_and_chunk_independent(self):
        data = CAPTURE.read_bytes()
        counts = {}
        for chunk in (4096, 64):
            packets = []
            d = PacketDecoder(packets.append, lambda _m: None)
            start = time.process_time()
            for i in range(0, len(data), chunk):
                d.feed(data[i:i + chunk])
            elapsed = time.process_time() - start
            counts[chunk] = len(packets)
            self.assertLess(elapsed, 3.0, f"chunk {chunk}: {elapsed:.2f}s (before the fix: 4.6 s / 15.7 s)")
        self.assertEqual(counts[4096], counts[64])


class Helpers(unittest.TestCase):
    def test_backoff_schedule(self):
        self.assertEqual([lh.backoff_delay(n) for n in (1, 2, 3, 4, 5, 6, 50)], [0.5, 1, 2, 4, 5, 5, 5])

    def test_raw_log_survives_write_errors_and_reopens(self):
        class Flaky:
            def __init__(self, fail):
                self.fail, self.data = fail, bytearray()

            def write(self, data):
                if self.fail:
                    raise OSError(28, "No space left on device")
                self.data += data

            def flush(self):
                pass

            def close(self):
                pass
        files = [Flaky(True), Flaky(False)]
        now = [0.0]
        log = lh.RawLog("unused.bin", opener=lambda: files.pop(0), clock=lambda: now[0])
        self.assertFalse(log.write(b"abc"))           # OSError: counted, never raised
        self.assertTrue(log.failing)
        self.assertIn("RAW LOG NOT WRITING", log.warning)
        self.assertEqual((log.errors, log.bytes_lost), (1, 3))
        self.assertFalse(log.write(b"def"))           # inside the retry interval: dropped, counted
        now[0] += lh.RAW_REOPEN_S + 0.1
        self.assertTrue(log.write(b"ghi"))            # re-opened by itself
        self.assertFalse(log.failing)
        self.assertEqual(log.reopens, 1)

    def test_raw_log_appends_to_the_same_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.bin"
            first = lh.RawLog(path)
            first.write(b"one")
            first.fsync()
            first.close()
            self.assertFalse(first.write(b"ignored"))
            second = lh.RawLog(path)
            second.write(b"two")
            second.close()
            self.assertEqual(path.read_bytes(), b"onetwo")

    def test_debug_log_rotates_and_is_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = lh.DebugLog(Path(tmp) / "debug.log", max_bytes=1000, files=3)
            for _ in range(200):
                log.write("x" * 99 + "\n")
            log.flush()
            log.close()
            names = sorted(p.name for p in Path(tmp).iterdir())
            self.assertEqual(names, ["debug.log", "debug.log.1", "debug.log.2"])
            self.assertLessEqual(sum(p.stat().st_size for p in Path(tmp).iterdir()), 3 * 1100)
            self.assertGreater(log.rotations, 2)

    def test_debug_log_never_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = lh.DebugLog(Path(tmp) / "debug.log")
            log._fh.close()                              # writing to a closed file raises ValueError
            log.write("x")
            log.flush()
            log.close()
            self.assertGreaterEqual(log.errors, 1)

    def test_coalescer_one_line_per_interval(self):
        now = [0.0]
        co = lh.Coalescer(1.0, clock=lambda: now[0])
        out = [co.add(f"m{i}") for i in range(1000)]
        self.assertEqual([o for o in out if o], ["m0"])      # first goes out at once
        now[0] = 1.5
        summary = co.flush_due()
        self.assertIn("999 events", summary)

    def test_bounded_events_drop_oldest_display_event_only(self):
        q = lh.BoundedEvents(limit=5)
        q.put(("error", "keep"))
        for i in range(10):
            q.put(("packet", i))
        items = [q.get_nowait() for _ in range(q.qsize())]
        self.assertEqual(len(items), 5)
        self.assertEqual(items[0], ("error", "keep"))
        self.assertEqual([i[1] for i in items[1:]], [6, 7, 8, 9])
        self.assertEqual(q.dropped, 6)

    def test_link_status_states(self):
        now = [100.0]
        st = lh.LinkStatus("TCP_x", "tcp", clock=lambda: now[0])
        st.connected()
        st.note_bytes(500)
        now[0] += 1.0
        self.assertEqual(st.snapshot()["state"], "connected")
        now[0] += 4.0
        snap = st.snapshot()
        self.assertEqual(snap["state"], "stale")
        self.assertGreater(snap["seconds_since_last_byte"], 3.0)
        st.lost("closed", 2)
        self.assertEqual(st.snapshot()["text"], "Reconnecting to TCP_x (attempt 2)")
        st.connected()
        self.assertEqual(st.snapshot()["reconnects"], 1)

    def test_serial_opened_with_dtr_rts_deasserted_before_open(self):
        order = []

        class FakeSerial:
            def __init__(self):
                self._dtr = self._rts = True

            def __setattr__(self, name, value):
                if name in ("dtr", "rts"):
                    order.append((name, value))
                object.__setattr__(self, name, value)

            def open(self):
                order.append(("open", None))

        class Module:
            Serial = FakeSerial
        lh.open_serial_quiet("COM9", 115200, serial_module=Module)
        self.assertEqual(order, [("dtr", False), ("rts", False), ("open", None)])

    def test_rssi_minus_128_label(self):
        self.assertIn("RECEIVER RADIO FAILED", wp.format_display_value("rssi", -128))
        self.assertEqual(wp.format_display_value("rssi", -90), "-90")


class FakeServer:
    """Loopback TCP server standing in for ser2net."""

    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(4)
        self.port = self.listener.getsockname()[1]
        self.connections = []
        self.accepted = 0
        self._stop = False
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        self.listener.settimeout(0.1)
        while not self._stop:
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.accepted += 1
            self.connections.append(conn)

    def current(self):
        return self.connections[-1]

    def close(self):
        self._stop = True
        for conn in self.connections:
            try:
                conn.close()
            except OSError:
                pass
        self.listener.close()


class ReconnectTests(unittest.TestCase):
    def tmp(self):
        """Temp dir removed AFTER the core closed its log files (cleanups run LIFO)."""
        import shutil
        path = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, path, True)
        return path

    def tmp(self):
        """Temp dir removed AFTER the core closed its log files (cleanups run LIFO)."""
        import shutil
        path = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, path, True)
        return path

    def make_core(self, tmp, port):
        import monitor_core
        patches = [patch.object(monitor_core, "LOG_DIR", Path(tmp)),
                   patch.object(monitor_core, "backoff_delay", lambda attempt: 0.05)]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        core = monitor_core.MonitorCore(auto_connect=False, start_feed=False)
        self.addCleanup(core.close)
        core.tcp_hosts[0].set("127.0.0.1")
        core.tcp_ports[0].set(str(port))
        return core

    def pump(self, core, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            core.process_events()
            if predicate():
                return True
            time.sleep(0.02)
        core.process_events()
        return predicate()

    def test_tcp_link_heals_after_server_drop_and_continues_the_same_raw_log(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            name = f"TCP_127.0.0.1_{server.port}"
            self.assertTrue(wait_for(lambda: server.accepted == 1))
            server.current().sendall(heartbeat(1) + heartbeat(2))
            self.assertTrue(self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 2)))
            logs = list(Path(tmp).glob("TCP_*.bin"))
            self.assertEqual(len(logs), 1)
            # Server closes the connection (ser2net restart / Pi blip).
            server.current().close()
            self.assertTrue(self.pump(core, lambda: server.accepted == 2))   # reconnected by itself
            server.current().sendall(heartbeat(3))
            self.assertTrue(self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 3)))
            self.assertEqual(list(Path(tmp).glob("TCP_*.bin")), logs)        # same file, not a new one
            self.assertEqual(logs[0].read_bytes(), heartbeat(1) + heartbeat(2) + heartbeat(3))
            link = core.links[name]
            self.assertEqual(link.status.reconnects, 1)
            self.assertIn(name, core.tcp_sockets)                            # commands possible again
            self.assertEqual(core.link_banner.get(), "")
            self.assertIn("Connected", core.status_text.get())

    def test_reconnects_forever_while_the_server_is_down_and_shows_state(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 1))
            server.current().sendall(heartbeat(1))
            self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 1))
            server.listener.close()                  # nothing listens any more
            server._stop = True
            server.current().close()
            self.assertTrue(self.pump(core, lambda: "Reconnecting to TCP 127.0.0.1" in core.link_banner.get()))
            self.assertTrue(core.link_banner.get().startswith("LINK LOST"))
            self.assertFalse(core.tcp_sockets)                               # no commands while down
            self.assertFalse(core.transmit_command(next(iter(core.links)), b"\x05" + bytes(8) + b"\x0a"))
            self.assertTrue(self.pump(core, lambda: next(iter(core.links.values())).status.attempt >= 3))
            core.disconnect()                                               # operator stops it for good
            self.assertFalse(core.links)
            self.assertEqual(core.link_banner.get(), "")

    def test_half_open_link_is_detected_by_the_no_data_watchdog(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            import monitor_core
            core = self.make_core(tmp, server.port)
            with patch.object(monitor_core, "TCP_NO_DATA_RECONNECT_S", 0.5):
                self.assertTrue(core.connect())
                self.assertTrue(wait_for(lambda: server.accepted == 1))
                server.current().sendall(heartbeat(1))        # delivers data ...
                self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 1))
                # ... then goes silent WITHOUT closing (half-open): no FIN, no RST.
                self.assertTrue(self.pump(core, lambda: server.accepted == 2, timeout=6))
                link = next(iter(core.links.values()))
                self.assertEqual(link.status.reconnects, 1)
                self.assertIn("no data", link.last_reason)

    def test_stale_banner_after_3_seconds_without_data(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 1))
            server.current().sendall(heartbeat(1))
            self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 1))
            link = next(iter(core.links.values()))
            core.update_link_health(time.monotonic())
            self.assertEqual(core.link_banner.get(), "")
            core.update_link_health(time.monotonic() + lh.LINK_LOST_AFTER_S + 1)
            self.assertIn("LINK LOST", core.link_banner.get())
            snap = core.link_snapshots[0]
            self.assertEqual(snap["state"], "stale")
            self.assertGreater(snap["seconds_since_last_byte"], 3.0)
            # An expected 0x7300 reset shows a calmer message.
            core.expected_reset_until = time.monotonic() + 60
            core.update_link_health(time.monotonic() + lh.LINK_LOST_AFTER_S + 1)
            self.assertIn("RESETTING", core.link_banner.get())
            self.assertIs(link, core.links[link.name])

    def test_raw_log_failure_never_stops_reader_or_decoder(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 1))
            link = next(iter(core.links.values()))

            def broken():
                raise OSError(28, "No space left on device")
            link.raw_log.close()
            link.raw_log = core.log_files[link.name] = lh.RawLog("x.bin", opener=broken)
            server.current().sendall(heartbeat(1) + heartbeat(2))
            self.assertTrue(self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 2)))
            self.assertTrue(self.pump(core, lambda: "RAW LOG NOT WRITING" in core.log_warning.get()))
            self.assertEqual(link.status.state, "connected")      # the reader is still running

    def test_decode_failure_never_stops_the_raw_log(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 1))

            def boom(*_a, **_k):
                raise RuntimeError("publisher exploded")
            with patch.object(core.publisher, "publish", boom):
                server.current().sendall(heartbeat(1) + heartbeat(2) + heartbeat(3))
                path = next(Path(tmp).glob("TCP_*.bin"))
                self.assertTrue(wait_for(lambda: path.exists() and path.read_bytes() == heartbeat(1) + heartbeat(2) + heartbeat(3)))
            self.assertTrue(self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 3)))

    def test_debug_log_is_closed_on_reconnect_and_decoder_noise_is_aggregated(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            first = core.debug_log_file
            core.disconnect()
            self.assertTrue(first._closed)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 2))
            junk = bytes((i * 13 + 5) % 251 + 1 for i in range(16384))
            server.current().sendall(junk + b"".join(heartbeat(n) for n in range(1, 30)))
            self.assertTrue(self.pump(core, lambda: core.values["packet.heartbeat.uptime"].get() == str(UP + 29)))
            text = "".join(p.read_text() for p in Path(tmp).glob("debug_*.log"))
            self.assertLess(text.count("BAD_PACKET"), 100)          # was one line per byte

    def test_reset_command_marks_link_drop_as_expected(self):
        server = FakeServer()
        self.addCleanup(server.close)
        if True:
            tmp = self.tmp()
            core = self.make_core(tmp, server.port)
            self.assertTrue(core.connect())
            self.assertTrue(wait_for(lambda: server.accepted == 1))
            name = next(iter(core.links))
            self.assertTrue(core.transmit_command(name, wp.build_command_packet(0x7300), "reset"))
            self.assertGreater(core.expected_reset_until, time.monotonic() + 10)
            # whole frame arrived at the server (atomic send)
            conn = server.current()
            conn.settimeout(2)
            self.assertEqual(conn.recv(64), wp.build_command_packet(0x7300))

    def test_busy_confirm_ack_is_answered_with_a_second_send(self):
        import monitor_core
        core = monitor_core.MonitorCore(auto_connect=False, start_feed=False)
        self.addCleanup(core.close)
        sent = []

        def write(data):
            sent.append(data)
            result, detail = (4, 0xC0F1) if len(sent) == 1 else (0, 0)
            core.command_link.feed(wp.parse_packet(0x06, struct.pack(">H", 0x686A) + data[1:7]
                                                   + bytes((result,)) + struct.pack(">H", detail)))
        port = type("Port", (), {"write": staticmethod(write), "close": lambda self: None})()
        core.serials["COM_X"] = port
        with patch.object(monitor_core, "BUSY_CONFIRM_GAP_S", 0.05):
            self.assertTrue(core.transmit_command("COM_X", wp.build_command_packet(0x686A), "Clear Logs"))
            self.assertTrue(wait_for(lambda: len(sent) == 2))
        core.process_events()
        self.assertTrue(wait_for(lambda: any("ACK[COM_X]" in line for line in list(core.console_pending))))
        self.assertTrue(any("CONFIRM[COM_X]" in line for line in core.console_pending))

    def test_busy_without_confirm_detail_says_retry(self):
        import monitor_core
        core = monitor_core.MonitorCore(auto_connect=False, start_feed=False)
        self.addCleanup(core.close)

        def write(data):
            core.command_link.feed(wp.parse_packet(0x06, struct.pack(">H", 0x0031) + data[1:7]
                                                   + bytes((4,)) + struct.pack(">H", 0x0001)))
        core.serials["COM_X"] = type("Port", (), {"write": staticmethod(write), "close": lambda self: None})()
        core.transmit_command("COM_X", wp.build_command_packet(0x0031), "CH1 On")
        self.assertTrue(wait_for(lambda: any("RETRY" in line for line in list(core.console_pending))))


class AssistUploadBounds(unittest.TestCase):
    def test_upload_aborts_on_a_dead_link_instead_of_flooding(self):
        import command_link
        link = command_link.CommandLink()
        writes = []
        start = time.monotonic()
        ok, message = link.upload_assist(lambda frame: writes.append(frame), b"\x00" * 100,
                                         pace_s=0, timeout=0.05, resume_deadline_s=600, max_dead_rounds=2)
        self.assertFalse(ok)
        self.assertLess(time.monotonic() - start, 15)
        self.assertLess(len(writes), 10)                       # BEGIN x3 per round, 2 rounds, no more
        self.assertIn("link", message.lower())

    def test_upload_stops_when_the_link_drops(self):
        import command_link
        link = command_link.CommandLink()
        cancel = threading.Event()
        cancel.set()
        ok, message = link.upload_assist(lambda f: None, b"\x00" * 100, pace_s=0, timeout=0.05, cancel=cancel)
        self.assertFalse(ok)


class PublisherSupervision(unittest.TestCase):
    def test_feed_thread_restarts_after_any_exception(self):
        from witness_link import WitnessPublisher
        pub = WitnessPublisher("127.0.0.1", 0)
        original = pub._serve
        calls = []

        def flaky():
            calls.append(1)
            if len(calls) == 1:
                raise ValueError("not an OSError")        # used to end the thread silently
            return original()
        pub._serve = flaky
        pub.start()
        self.addCleanup(pub.close)
        self.assertTrue(wait_for(lambda: pub.restarts >= 1, timeout=5))
        self.assertTrue(wait_for(lambda: pub.healthy, timeout=5))
        client = socket.create_connection(("127.0.0.1", pub.port), timeout=2)
        self.addCleanup(client.close)
        pub.publish("COM1", {"packet": "heartbeat", "status": 1, "uptime": 5})
        client.settimeout(3)
        self.assertIn(b'"uptime":5', client.recv(4096))
        self.assertIn("ValueError", pub.last_error)


class ExceptionHookTests(unittest.TestCase):
    def test_hook_logs_traceback_and_reports_without_raising(self):
        import main
        saved = (sys.excepthook, main._hook_installed, list(main._exception_sinks), main.EXCEPTION_LOG)
        threading_hook, unraisable = threading.excepthook, sys.unraisablehook
        try:
            with tempfile.TemporaryDirectory() as tmp:
                main.EXCEPTION_LOG = Path(tmp) / "horizon_exceptions.log"
                main._hook_installed = False
                main._exception_sinks.clear()
                main._exception_limit = lh.RateLimited(5.0)
                lines = []
                self.assertTrue(main.install_global_excepthook(lines.append))
                self.assertFalse(main.install_global_excepthook())       # idempotent
                try:
                    raise RuntimeError("slot blew up")
                except RuntimeError as exc:
                    with patch("sys.stderr"):
                        sys.excepthook(type(exc), exc, exc.__traceback__)
                self.assertIn("slot blew up", main.EXCEPTION_LOG.read_text())
                self.assertIn("UNCAUGHT", lines[0])
                done = threading.Event()

                def crash():
                    try:
                        raise KeyError("in a thread")
                    finally:
                        done.set()
                with patch("sys.stderr"):
                    t = threading.Thread(target=crash, name="t-crash")
                    t.start()
                    t.join()
                self.assertTrue(wait_for(lambda: len(lines) >= 2))
        finally:
            sys.excepthook, main._hook_installed = saved[0], saved[1]
            main._exception_sinks[:] = saved[2]
            main.EXCEPTION_LOG = saved[3]
            threading.excepthook, sys.unraisablehook = threading_hook, unraisable

    def test_panel_timer_slot_exception_is_contained(self):
        from PyQt6 import QtWidgets
        import main
        global _APP
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])   # keep a reference
        with tempfile.TemporaryDirectory() as tmp, patch.object(main, "EXCEPTION_LOG", Path(tmp) / "e.log"), \
                patch("monitor_core.LOG_DIR", Path(tmp)):
            panel = main.MonitorPanel(None, auto_connect=False, start_feed=False)
            self.addCleanup(panel.close)

            def bad():
                raise OSError(28, "disk full")
            with patch("sys.stderr"):
                panel._guarded(bad, "unit")()                 # must not raise
            self.assertIn("SLOT_ERROR[unit]", panel._exception_text)
            panel.core.link_banner.set("LINK LOST - test")
            self.assertTrue(panel.link_banner_label.isVisibleTo(panel))
            self.assertTrue(panel._link_lost)
            panel.core.link_banner.set("")
            self.assertFalse(panel.link_banner_label.isVisibleTo(panel))


class TailscaleProbeOffThread(unittest.TestCase):
    def test_slow_probe_does_not_block_construction(self):
        import monitor_core
        release = threading.Event()

        def slow():
            release.wait(5)
            return None
        with patch.object(monitor_core, "tailscale_ipv4", slow):
            start = time.monotonic()
            core = monitor_core.MonitorCore(auto_connect=False, feed_port=0, remote_feed="auto")
            try:
                core.update_feed_status()
                self.assertLess(time.monotonic() - start, 1.0)
                self.assertIn("checking", core.remote_feed_status.get())
            finally:
                release.set()
                core.close()


class FeatherweightReconnect(unittest.TestCase):
    def test_reader_reconnects_and_appends_to_the_same_raw_log(self):
        from ground_receivers import FeatherweightReceiver
        attempts = []

        class FakePort:
            def __init__(self, fail, payload, die):
                self.fail, self.payload, self.die, self.in_waiting = fail, payload, die, 0

            def __enter__(self):
                if self.fail:
                    raise OSError("device disconnected")
                return self

            def __exit__(self, *_a):
                return False

            def read(self, _n):
                if self.payload:
                    data, self.payload = self.payload, b""
                    return data
                if self.die:
                    raise OSError("device unplugged")
                time.sleep(0.05)
                return b""

        def factory(**_kw):
            attempts.append(1)
            n = len(attempts)
            return FakePort(n == 1, b"zz" if n == 2 else b"yy", die=(n == 2))
        with tempfile.TemporaryDirectory() as tmp, patch("ground_receivers.backoff_delay", lambda a: 0.05):
            receiver = FeatherweightReceiver("COM_F", tmp, serial_factory=factory)
            receiver.auto_retry = True
            receiver.start()
            try:
                self.assertTrue(wait_for(lambda: receiver.bytes_received >= 4, timeout=8))
                self.assertGreaterEqual(receiver.reconnects, 1)
                raw = receiver.raw_path
            finally:
                receiver.stop(wait=True)
            self.assertEqual(raw.read_bytes(), b"zzyy")


if __name__ == "__main__":
    unittest.main()
