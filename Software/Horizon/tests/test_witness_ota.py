"""witness_ota.py against a fake Witness HTTP server, and the 0x5746 Wi-Fi frame."""

import io
import json
import os
import sys
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import witness_ota as wo  # noqa: E402
import witness_protocol as wp  # noqa: E402


def make_image(project="softwarev1", version="v2", chip=9, size=2000):
    image = bytearray(size)
    image[0] = 0xE9
    image[12:14] = chip.to_bytes(2, "little")
    image[32:36] = (0xABCD5432).to_bytes(4, "little")
    image[32 + 16:32 + 16 + len(version)] = version.encode()
    image[32 + 48:32 + 48 + len(project)] = project.encode()
    return bytes(image)


class FakeState:
    def __init__(self):
        self.version = "v1"
        self.uptime = 100000
        self.app_state = "valid"
        self.uploaded = None
        self.calls = []
        self.logs = {"I0001.BIN": bytes(range(256)) * 40, "S0001.TXT": b"hello log\n"}
        self.reject_upload = False
        self.lie_about_size = False
        self.polls_after_upload = 0
        self.uploaded_version = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    state = None

    def log_message(self, *args):
        pass

    def _json(self, status, obj):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        s = self.state
        s.calls.append(("GET", self.path))
        if self.path == "/info":
            if s.uploaded is not None:
                s.polls_after_upload += 1
                if s.polls_after_upload > 1:       # rebooted: new version, fresh uptime
                    s.version = s.uploaded_version
                    s.uptime = 2000 * s.polls_after_upload
                    s.app_state = "pending" if s.polls_after_upload == 2 else "valid"
            self._json(200, {"project": "softwarev1", "version": s.version, "running": "ota_0",
                             "next": "ota_1", "app_state": s.app_state, "uptime_ms": s.uptime,
                             "flight_state": "PadIdle", "free_heap": 1})
        elif self.path == "/logs":
            self._json(200, {"logging_frozen": True, "count": len(s.logs), "files": [
                {"name": n, "size": len(d), "session": 1, "slot": 0} for n, d in s.logs.items()]})
        elif self.path.startswith("/logs/"):
            name = self.path[6:]
            if name not in s.logs:
                self._json(404, {"ok": False, "error": "no such log"})
                return
            data = s.logs[name]
            self.send_response(200)
            self.send_header("X-File-Size", str(len(data) + (5 if s.lie_about_size else 0)))
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self._json(404, {"ok": False, "error": "unknown path"})

    def do_POST(self):
        s = self.state
        s.calls.append(("POST", self.path))
        if self.path == "/ota":
            length = int(self.headers.get("Content-Length", "0"))
            if s.reject_upload:
                self.rfile.read(length)   # a real Witness closes early; keep the test deterministic
                self._json(422, {"ok": False, "error": "image is a different project"})
                self.close_connection = True
                return
            body = self.rfile.read(length)
            s.uploaded = body
            s.uploaded_version = body[48:80].split(b"\0")[0].decode()
            self._json(200, {"ok": True, "bytes": len(body), "slot": "ota_1",
                             "version": s.uploaded_version, "reboot_ms": 1000})
        elif self.path in ("/reboot", "/wifi/off"):
            self._json(200, {"ok": True})
        else:
            self._json(404, {"ok": False, "error": "unknown path"})


class WitnessOtaTests(unittest.TestCase):
    def setUp(self):
        self.state = FakeState()
        handler = type("H", (Handler,), {"state": self.state})
        self.server = HTTPServer(("127.0.0.1", 0), handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = "http://127.0.0.1:%d" % self.server.server_address[1]
        self.client = wo.Witness(self.base, timeout=5.0)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(5)

    def run_cli(self, *argv):
        out = io.StringIO()
        with redirect_stdout(out):
            code = wo.main(["--base", self.base] + list(argv))
        return code, out.getvalue()

    def test_check_image(self):
        self.assertEqual(wo.check_image(make_image())["version"], "v2")
        with self.assertRaises(wo.OtaError):
            wo.check_image(make_image(project="softwareattempt2"))
        with self.assertRaises(wo.OtaError):
            wo.check_image(make_image(chip=5))
        with self.assertRaises(wo.OtaError):
            wo.check_image(b"\0" * 400)
        with self.assertRaises(wo.OtaError):
            wo.check_image(make_image()[:100])

    def test_info(self):
        code, out = self.run_cli("info")
        self.assertEqual(code, 0)
        self.assertEqual(json.loads(out)["app_state"], "valid")

    def test_upload_waits_for_valid_new_version(self):
        path = os.path.join(tempfile.mkdtemp(), "softwarev1.bin")
        image = make_image(version="v2")
        with open(path, "wb") as handle:
            handle.write(image)
        original = wo.time.sleep
        wo.time.sleep = lambda s: None
        try:
            code, out = self.run_cli("upload", path)
        finally:
            wo.time.sleep = original
        self.assertEqual(code, 0, out)
        self.assertEqual(self.state.uploaded, image)
        self.assertIn("now running v2", out)
        self.assertIn("app state valid", out)

    def test_upload_refuses_wrong_project_locally(self):
        path = os.path.join(tempfile.mkdtemp(), "other.bin")
        with open(path, "wb") as handle:
            handle.write(make_image(project="softwareattempt2"))
        code, _ = self.run_cli("upload", path)
        self.assertEqual(code, 1)
        self.assertNotIn(("POST", "/ota"), self.state.calls)

    def test_upload_reports_device_rejection(self):
        self.state.reject_upload = True
        with self.assertRaises(wo.OtaError) as raised:
            self.client.upload(make_image(size=3_000_000))
        self.assertIn("different project", str(raised.exception))

    def test_wait_for_reboot_times_out_without_reboot(self):
        ticks = iter(range(0, 1000))
        with self.assertRaises(wo.OtaError) as raised:
            wo.wait_for_reboot(self.client, "v2", timeout=5, sleep=lambda s: None,
                               clock=lambda: float(next(ticks)))
        self.assertIn("did not reboot", str(raised.exception))

    def test_logs_and_get(self):
        code, out = self.run_cli("logs")
        self.assertEqual(code, 0)
        self.assertIn("I0001.BIN", out)
        directory = tempfile.mkdtemp()
        code, out = self.run_cli("get", "all", "-o", directory)
        self.assertEqual(code, 0, out)
        with open(os.path.join(directory, "I0001.BIN"), "rb") as handle:
            self.assertEqual(handle.read(), self.state.logs["I0001.BIN"])
        with open(os.path.join(directory, "S0001.TXT"), "rb") as handle:
            self.assertEqual(handle.read(), b"hello log\n")
        self.assertFalse([f for f in os.listdir(directory) if f.endswith(".part")])

    def test_get_detects_truncation_and_missing(self):
        self.state.lie_about_size = True
        directory = tempfile.mkdtemp()
        code, _ = self.run_cli("get", "S0001.TXT", "-o", directory)
        self.assertEqual(code, 1)
        self.assertFalse(os.path.exists(os.path.join(directory, "S0001.TXT")))
        code, _ = self.run_cli("get", "NOPE.BIN", "-o", directory)
        self.assertEqual(code, 1)

    def test_reboot_and_wifi_off(self):
        self.assertEqual(self.run_cli("reboot")[0], 0)
        self.assertEqual(self.run_cli("wifi-off")[0], 0)
        self.assertIn(("POST", "/reboot"), self.state.calls)
        self.assertIn(("POST", "/wifi/off"), self.state.calls)

    def test_unreachable(self):
        client = wo.Witness("http://127.0.0.1:1", timeout=1.0)
        with self.assertRaises(wo.OtaError):
            client.info()


class WifiPacketTests(unittest.TestCase):
    def test_layout(self):
        self.assertEqual(wp.build_wifi_packet(True),
                         bytes((0x05, 1, 0, 0, 0, 0, 0, 0x57, 0x46, 0x0A)))
        self.assertEqual(wp.build_wifi_packet(False), bytes((0x05, 0, 0, 0, 0, 0, 0, 0x57, 0x46, 0x0A)))
        self.assertEqual(wp.build_wifi_packet(True, 12, 30)[1:4], bytes((1, 12, 30)))

    def test_limits(self):
        for kwargs in ({"tx_dbm": 1}, {"tx_dbm": 21}, {"tx_dbm": -4}, {"minutes": 241}, {"minutes": -1}):
            with self.assertRaises(ValueError):
                wp.build_wifi_packet(True, **kwargs)

    def test_matches_comms_h(self):
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(here, "comms.h")) as handle:
            self.assertIn("#define IRIS_COMMAND_WIFI 0x5746U", handle.read())


if __name__ == "__main__":
    unittest.main()
