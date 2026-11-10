"""Remote (Tailscale) Witness feed: host checks, independence from the local
feed, runtime toggle, and Horizon reading it over a real Tailscale address."""

import json
import socket
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import witness_link
from ground_receivers import WitnessReceiver, resolve_feed_host
from witness_link import WitnessPublisher, is_tailscale_address, tailscale_ipv4

PACKET = {"packet": "state", "status": 1, "uptime": 1000, "orientation": [0, 0, 0, 1]}


def read_records(sock, count, timeout=10.0):
    """Read ``count`` newline-delimited records (fails on timeout)."""
    sock.settimeout(timeout)
    buffer, records = b"", []
    deadline = time.monotonic() + timeout
    while len(records) < count and time.monotonic() < deadline:
        data = sock.recv(65536)
        if not data:
            break
        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            records.append(json.loads(line))
    return records


def loopback_counts_as_tailscale(address):
    return str(address).startswith("127.") or witness_link.TAILSCALE_NETWORKS[0].__contains__(
        __import__("ipaddress").ip_address(str(address)))


class HostRules(unittest.TestCase):
    def test_tailscale_ranges(self):
        self.assertTrue(is_tailscale_address("100.101.102.103"))
        self.assertTrue(is_tailscale_address("fd7a:115c:a1e0::1"))
        for address in ("192.168.1.20", "10.0.0.5", "8.8.8.8", "127.0.0.1", "100.63.255.255", "bench"):
            self.assertFalse(is_tailscale_address(address), address)

    def test_remote_publisher_binds_tailscale_only(self):
        for host in ("0.0.0.0", "192.168.1.20", "127.0.0.1"):
            with self.assertRaises(ValueError):
                WitnessPublisher(host, 0, remote=True)
        with self.assertRaises(ValueError):
            WitnessPublisher("100.101.102.103", 0)          # the local feed stays loopback

    def test_horizon_accepts_loopback_and_tailscale_only(self):
        self.assertEqual(resolve_feed_host("localhost"), ("127.0.0.1", False))
        self.assertEqual(resolve_feed_host("127.0.0.1"), ("127.0.0.1", False))
        self.assertEqual(resolve_feed_host("100.101.102.103"), ("100.101.102.103", True))
        with self.assertRaises(ValueError):
            resolve_feed_host("192.168.1.20")
        with patch("socket.getaddrinfo", return_value=[(0, 0, 0, "", ("100.90.1.2", 0))]):
            self.assertEqual(resolve_feed_host("benchpc"), ("100.90.1.2", True))     # MagicDNS name
        with patch("socket.getaddrinfo", return_value=[(0, 0, 0, "", ("93.184.216.34", 0))]):
            with self.assertRaises(ValueError):
                resolve_feed_host("example.com")
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                WitnessReceiver("192.168.1.20", 8767, tmp)
            WitnessReceiver("benchpc.tail1234.ts.net", 8767, tmp)    # names resolve on the worker


class Independence(unittest.TestCase):
    def test_stuck_remote_viewer_never_delays_the_local_feed(self):
        from monitor_core import MonitorCore
        with patch("witness_link.is_tailscale_address", side_effect=loopback_counts_as_tailscale), \
                patch("monitor_core.is_tailscale_address", side_effect=loopback_counts_as_tailscale), \
                patch("monitor_core.tailscale_ipv4", return_value="127.0.0.1"):
            core = MonitorCore(auto_connect=False, feed_port=0, remote_feed="auto", remote_feed_port=0)
        try:
            remote = core.remote_publisher
            self.assertIsNotNone(remote, core.remote_feed_status.get())
            remote.max_pending = 256 * 1024          # make the stuck viewer overflow quickly
            self.assertNotEqual(remote.port, core.publisher.port)
            # A remote viewer that connects and then never reads (a dead cellular link).
            stuck = socket.socket()
            stuck.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            stuck.connect(("127.0.0.1", remote.port))
            local = socket.create_connection(("127.0.0.1", core.publisher.port))
            time.sleep(0.2)
            # ~1.5 kHz for 3 s: faster than Witness's real telemetry. (A tight
            # unpaced burst overflows the local feed's own limits with or
            # without a remote viewer, so it would not test independence.)
            total = 4500
            received = []
            reader = threading.Thread(target=lambda: received.extend(read_records(local, total, 20)))
            reader.start()
            publish_s = 0.0
            for i in range(total):
                start = time.perf_counter()
                core.on_decoded_packet("COM7", dict(PACKET, uptime=i))
                publish_s += time.perf_counter() - start
                if i % 3 == 2:
                    time.sleep(0.002)
            reader.join(25)
            self.assertEqual(len(received), total)                  # local viewer got everything
            self.assertEqual([r["packet"]["uptime"] for r in received], list(range(total)))
            self.assertLess(publish_s / total, 0.002)               # publishing stays cheap
            deadline = time.monotonic() + 5
            while remote.disconnected_slow == 0 and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertGreaterEqual(remote.disconnected_slow, 1)    # only the stuck remote viewer is dropped
            core.update_feed_status()
            self.assertIn("slow viewer", core.remote_feed_status.get())
        finally:
            for sock in ("stuck", "local"):
                if sock in locals():
                    locals()[sock].close()
            core.close()

    def test_remote_feed_toggles_without_touching_local(self):
        from monitor_core import MonitorCore
        with patch("witness_link.is_tailscale_address", side_effect=loopback_counts_as_tailscale), \
                patch("monitor_core.is_tailscale_address", side_effect=loopback_counts_as_tailscale), \
                patch("monitor_core.tailscale_ipv4", return_value="127.0.0.1"):
            core = MonitorCore(auto_connect=False, feed_port=0, remote_feed="off", remote_feed_port=0)
            try:
                self.assertIsNone(core.remote_publisher)
                local_port = core.publisher.port
                core.set_remote_feed(True)
                self.assertIsNotNone(core.remote_publisher)
                self.assertIn("viewers connected", core.remote_feed_status.get())
                core.set_remote_feed(False)
                self.assertIsNone(core.remote_publisher)
                self.assertIn("off", core.remote_feed_status.get())
                self.assertEqual(core.publisher.port, local_port)
                self.assertIsNone(core.publisher.error)
            finally:
                core.close()

    def test_waits_and_retries_when_tailscale_is_down(self):
        from monitor_core import MonitorCore
        with patch("monitor_core.tailscale_ipv4", return_value=None):
            core = MonitorCore(auto_connect=False, feed_port=0, remote_feed="auto", remote_feed_port=0)
        try:
            self.assertIsNone(core.remote_publisher)
            self.assertIn("waiting for Tailscale", core.remote_feed_status.get())
            self.assertIsNotNone(core.publisher.port)            # local feed is up regardless
        finally:
            core.close()


TS_IP = tailscale_ipv4()


@unittest.skipUnless(TS_IP, "Tailscale is not running on this machine")
class OverRealTailscale(unittest.TestCase):
    def test_horizon_reads_the_feed_over_tailscale(self):
        publisher = WitnessPublisher(TS_IP, 0, remote=True)
        publisher.start()
        records = []
        with tempfile.TemporaryDirectory() as tmp:
            receiver = WitnessReceiver(TS_IP, publisher.port, tmp, on_record=records.append)
            receiver.start()
            try:
                deadline = time.monotonic() + 10
                while receiver.remote is False and time.monotonic() < deadline:
                    time.sleep(0.05)
                time.sleep(0.5)
                for i in range(20):
                    publisher.publish("COM7", dict(PACKET, uptime=i))
                    time.sleep(0.01)
                while len(records) < 20 and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertTrue(receiver.remote)
                self.assertEqual([r["packet"]["uptime"] for r in records if not r["replay"]][-20:],
                                 list(range(20)))
                self.assertEqual(publisher.clients, 1)
            finally:
                receiver.stop()
                publisher.close()


if __name__ == "__main__":
    unittest.main()
