"""Wi-Fi / OTA box: Wi-Fi ON/OFF through the command path, HTTP buttons on a worker (mocked)."""
import os
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import witness_ota  # noqa: E402
import witness_protocol as wp  # noqa: E402
from horizon_qt_support import make_monitor, pump_until  # noqa: E402


def fake_image(kind=2, version="v9"):
    head = bytearray(witness_ota.HEAD_LENGTH + witness_ota.INFO_LENGTH)
    head[0] = 0xE9
    head[12:14] = struct.pack("<H", witness_ota.CHIP_ID_ESP32S3)
    d = witness_ota.APP_DESC_OFFSET
    head[d:d + 4] = struct.pack("<I", witness_ota.APP_DESC_MAGIC)
    head[d + 16:d + 16 + len(version)] = version.encode()
    name = witness_ota.PROJECT_NAME.encode()
    head[d + 48:d + 48 + len(name)] = name
    o = witness_ota.HEAD_LENGTH
    head[o:o + 4] = struct.pack("<I", witness_ota.INFO_MAGIC)
    head[o + 5] = kind
    return bytes(head)


class FakeWitness:
    gate = None
    calls = []
    fail = None

    def __init__(self, *a, **k):
        pass

    def _go(self, name):
        FakeWitness.calls.append(name)
        if FakeWitness.gate is not None:
            FakeWitness.gate.wait(5)
        if FakeWitness.fail:
            raise witness_ota.OtaError(FakeWitness.fail)

    def info(self, timeout=None):
        self._go("info")
        return {"version": "v1", "running": "ota_0", "kind": "maintenance"}

    def logs(self):
        self._go("logs")
        return {"count": 1, "files": [{"name": "a.bin", "size": 3, "session": 1}]}

    def reboot(self):
        self._go("reboot")
        return {"ok": True}

    def upload(self, image, progress=None, force=False):
        self._go("upload")
        FakeWitness.force = force
        if progress:
            progress(len(image), len(image))
        return {"bytes": len(image), "slot": "ota_1", "reboot_ms": 500, "kind": "flight"}

    def download(self, name, dest, progress=None):
        self._go("download")
        Path(dest).write_bytes(b"abc")
        return 3


class WifiOtaBoxTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        p = patch("monitor_core.LOG_DIR", Path(self.dir.name))
        p.start()
        self.addCleanup(p.stop)
        self.monitor = make_monitor(self.dir.name)
        self.addCleanup(self.monitor.close)
        self.core = self.monitor.core
        self.box = self.monitor.wifi_ota_box
        self.core.commands_enabled.set(True)
        self.sent = []
        self.core.transmit_command = lambda t, c, label="": self.sent.append((c, label)) or True
        self.answer = True
        self.asked = []
        self.core.confirm = lambda title, text: self.asked.append(text) or self.answer
        self.lines = []
        self.core.console_write = self.lines.append
        FakeWitness.gate = None
        FakeWitness.calls = []
        FakeWitness.fail = None
        p2 = patch("witness_ota.Witness", FakeWitness)
        p2.start()
        self.addCleanup(p2.stop)

    def finish(self):
        self.box.wait_finished()
        self.assertTrue(pump_until(lambda: not self.box.busy))

    def text(self):
        return "\n".join(self.lines)

    def test_wifi_buttons_send_exact_frames(self):
        self.box.wifi_on_button.click()
        self.box.wifi_off_button.click()
        self.assertEqual(self.sent[0][0], bytes.fromhex("05 01 00 00 00 00 00 57 46 0a"))
        self.assertEqual(self.sent[1][0], bytes.fromhex("05 00 00 00 00 00 00 57 46 0a"))
        self.assertIn("Pad Idle or Landed", self.asked[0])
        self.assertEqual(len(self.asked), 1)          # OFF does not ask

    def test_wifi_on_declined_sends_nothing(self):
        self.answer = False
        self.box.wifi_on_button.click()
        self.assertEqual(self.sent, [])

    def test_wifi_on_enabled_only_in_pad_idle_landed_or_unknown(self):
        self.assertTrue(self.box.wifi_on_button.isEnabled())
        for state, enabled in ((1, False), (4, True), (0, True)):
            with patch.object(self.core, "witness_flight_state", return_value=state):
                self.core._refresh_reset_enabled()
            self.assertEqual(self.box.wifi_on_button.isEnabled(), enabled, state)

    def test_info_and_logs_run_on_worker_and_report(self):
        main_thread = threading.current_thread()
        seen = []
        orig = FakeWitness._go
        FakeWitness._go = lambda s, n: seen.append(threading.current_thread()) or orig(s, n)
        self.addCleanup(setattr, FakeWitness, "_go", orig)
        self.box.info_button.click()
        self.finish()
        self.box.logs_button.click()
        self.finish()
        self.assertTrue(all(t is not main_thread for t in seen) and seen)
        self.assertIn("ota_0", self.text())
        self.assertIn("a.bin", self.text())

    def test_buttons_disabled_while_running(self):
        FakeWitness.gate = threading.Event()
        self.box.info_button.click()
        self.assertTrue(self.box.busy)
        for b in self.box.http_buttons:
            self.assertFalse(b.isEnabled())
        self.assertTrue(self.box.wifi_off_button.isEnabled())
        self.assertFalse(self.box.ota_info())          # second op refused
        FakeWitness.gate.set()
        self.finish()
        for b in self.box.http_buttons:
            self.assertTrue(b.isEnabled())

    def test_errors_are_reported_not_raised(self):
        FakeWitness.fail = "HTTP 409: not Landed"
        self.box.logs_button.click()
        self.finish()
        self.assertIn("OTA_ERROR HTTP 409: not Landed", self.text())
        with patch.object(FakeWitness, "info", side_effect=OSError("boom")):
            self.box.info_button.click()
            self.finish()
        self.assertIn("OSError: boom", self.text())

    def test_reboot_confirms_first(self):
        self.answer = False
        self.box.reboot_button.click()
        self.assertEqual(FakeWitness.calls, [])
        self.answer = True
        self.box.reboot_button.click()
        self.finish()
        self.assertEqual(FakeWitness.calls, ["reboot"])

    def test_upload_checks_confirms_and_forces(self):
        path = Path(self.dir.name) / "x.bin"
        path.write_bytes(fake_image(kind=1))
        self.box.force_check.setChecked(True)
        self.box.upload_image(str(path))
        self.finish()
        self.assertIn("flight image", self.asked[0])
        self.assertTrue(FakeWitness.force)
        self.assertIn("uploaded", self.text())
        bad = Path(self.dir.name) / "bad.bin"
        bad.write_bytes(b"\0" * 400)
        FakeWitness.calls = []
        self.assertFalse(self.box.upload_image(str(bad)))
        self.assertEqual(FakeWitness.calls, [])
        self.assertIn("OTA_ERROR", self.text())

    def test_download_all(self):
        out = Path(self.dir.name) / "dl"
        self.box.download_logs(str(out))
        self.finish()
        self.assertEqual((out / "a.bin").read_bytes(), b"abc")
        self.assertIn("done", self.text())


if __name__ == "__main__":
    unittest.main()
