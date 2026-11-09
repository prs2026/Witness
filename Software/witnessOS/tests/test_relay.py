import os
import socket
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import wos_relay as wr  # noqa: E402
from witness_link import WitnessPublisher, decode_record  # noqa: E402


def heartbeat(uptime_ms: int, status: int = 0x0042) -> bytes:
    return (bytes([0xFF]) + status.to_bytes(2, "big")
            + uptime_ms.to_bytes(4, "big") + b"\n")


class FakePublisher:
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.published = []
        self.error = None
        self.sequence = 0
        self.dropped = 0

    def start(self):
        pass

    def publish(self, source, packet, received_at=None):
        self.sequence += 1
        self.published.append((source, packet))

    def close(self):
        pass


class FakeSession:
    def __init__(self):
        self.rx_tap = None
        self.connected = True
        self._host = "bench"


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class RelayEngineTests(unittest.TestCase):
    def engine(self):
        eng = wr.RelayEngine(feed_port=0, log_dir=None,
                             publisher_factory=FakePublisher)
        self.assertTrue(eng.start_feed())
        self.addCleanup(eng.close)
        return eng

    def test_split_frames_decode_and_publish(self):
        eng = self.engine()
        session = FakeSession()
        eng.share_session(session)
        self.assertEqual(eng.source, "HITL_bench")
        data = heartbeat(1000) + heartbeat(2000)
        for i in range(0, len(data), 3):  # arbitrary split reads
            session.rx_tap(data[i:i + 3])
        pub = eng.publisher
        self.assertEqual([p["uptime"] for _s, p in pub.published],
                         [1000, 2000])
        self.assertEqual(pub.published[0][0], "HITL_bench")
        snap = eng.snapshot()
        self.assertEqual(snap["packets"]["heartbeat"]["count"], 2)
        self.assertEqual(snap["rx_bytes"], len(data))
        eng.disconnect()
        self.assertIsNone(session.rx_tap)
        self.assertIsNone(eng.source)

    def test_stale_reader_bytes_are_ignored(self):
        eng = self.engine()
        session = FakeSession()
        eng.share_session(session)
        tap = session.rx_tap
        eng.disconnect()
        tap(heartbeat(5))
        self.assertEqual(eng.snapshot()["rx_bytes"], 0)

    def test_decode_errors_are_rate_limited(self):
        eng = self.engine()
        eng.share_session(FakeSession())
        eng.feed_bytes(b"\x99" * 200)  # unknown packet IDs
        self.assertGreater(eng.decode_errors, 0)
        logged = 0
        while not eng.events.empty():
            kind, _msg = eng.events.get_nowait()
            logged += kind == "decode"
        self.assertLessEqual(logged, wr.MAX_ERROR_LOGS_PER_S)

    def test_feed_port_in_use_reports_error(self):
        first = wr.RelayEngine(feed_port=0, log_dir=None)
        self.assertTrue(first.start_feed())
        self.addCleanup(first.close)
        second = wr.RelayEngine(feed_port=first.publisher.port, log_dir=None)
        self.assertFalse(second.start_feed())
        self.assertIn("cannot listen", second.feed_status()["error"])

    def test_tcp_input_to_real_feed_end_to_end(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        self.addCleanup(server.close)

        def fake_witness():
            conn, _ = server.accept()
            with conn:
                time.sleep(0.2)
                conn.sendall(heartbeat(4242))
                time.sleep(0.5)

        threading.Thread(target=fake_witness, daemon=True).start()
        tmp_log = os.path.join(HERE, "_relay_log_tmp")
        eng = wr.RelayEngine(feed_port=0, log_dir=tmp_log)
        self.addCleanup(lambda: __import__("shutil").rmtree(
            tmp_log, ignore_errors=True))
        self.addCleanup(eng.close)
        self.assertTrue(eng.start_feed())
        client = socket.create_connection(("127.0.0.1", eng.publisher.port))
        client.settimeout(3)
        self.addCleanup(client.close)
        eng.connect_tcp("127.0.0.1", server.getsockname()[1])
        line = client.makefile("rb").readline()
        record = decode_record(line)
        self.assertEqual(record["packet"]["packet"], "heartbeat")
        self.assertEqual(record["packet"]["uptime"], 4242)
        self.assertEqual(record["source"],
                         f"TCP_127.0.0.1_{server.getsockname()[1]}")
        # Remote close is noticed and the input is released.
        self.assertTrue(wait_for(lambda: eng.source is None))
        logs = os.listdir(tmp_log)
        self.assertEqual(len(logs), 1)
        with open(os.path.join(tmp_log, logs[0]), "rb") as f:
            self.assertEqual(f.read(), heartbeat(4242))

    def test_serial_input_loopback(self):
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest("pyserial not installed")
        eng = self.engine()
        eng.connect_serial("loop://")
        eng._input["handle"].write(heartbeat(777))
        self.assertTrue(wait_for(lambda: eng.publisher.published))
        source, packet = eng.publisher.published[0]
        self.assertEqual((source, packet["uptime"]), ("loop://", 777))
        eng.disconnect()
        self.assertIsNone(eng.source)

    def test_flatten_packet(self):
        rows = dict(wr.flatten_packet({"packet": "sensors", "status": 0x1234,
                                       "gyro": [1.5, -2.0, 0.25],
                                       "uptime": 7}))
        self.assertEqual(rows["status"], "0x1234")
        self.assertEqual(rows["gyro"], "1.5, -2, 0.25")
        self.assertEqual(rows["uptime"], "7")

    def test_publisher_is_the_horizon_one(self):
        self.assertIs(wr.WitnessPublisher, WitnessPublisher)


if __name__ == "__main__":
    unittest.main()
