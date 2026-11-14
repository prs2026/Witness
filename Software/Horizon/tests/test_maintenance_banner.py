"""Witness maintenance boot (status byte 1 bit 5) is impossible to miss in Horizon."""
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import witness_protocol as wp  # noqa: E402


class MaintenanceBitTests(unittest.TestCase):
    def test_decoder(self):
        self.assertTrue(wp.status_maintenance(0x0120))
        self.assertFalse(wp.status_maintenance(0x0118))
        self.assertFalse(wp.status_maintenance(None))
        byte1 = {text: (on, error) for _m, text, on, error in wp.decode_status(0x0120)["byte1"]}
        self.assertEqual(byte1["MAINTENANCE"], (True, True))


class MaintenanceBannerTests(unittest.TestCase):
    def setUp(self):
        from horizon_qt_support import make_monitor
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        log_patch = patch("monitor_core.LOG_DIR", Path(self.directory.name))
        log_patch.start()
        self.addCleanup(log_patch.stop)
        self.monitor = make_monitor(self.directory.name)
        self.addCleanup(self.monitor.close)

    def feed(self, status):
        packet = {"packet": "sensors", "status": status, "uptime": 1,
                  "filtered_accel": [0.0, 0.0, 1.0], "gyro": [0.0, 0.0, 0.0]}
        self.monitor.events.put(("packet", ("COM_TEST", packet, time.time())))
        self.monitor.process_events()

    def test_banner_follows_status_bit(self):
        banner = self.monitor.maintenance_banner
        self.assertFalse(banner.isVisibleTo(self.monitor))
        self.feed(0x0120)      # Pad Idle + maintenance
        self.assertTrue(banner.isVisibleTo(self.monitor))
        self.assertIn("NOT READY FOR FLIGHT", banner.text())
        self.feed(0x0100)      # back in flight mode
        self.assertFalse(banner.isVisibleTo(self.monitor))

    def test_banner_follows_maintenance_beacon(self):
        banner = self.monitor.maintenance_banner
        beacon = {"packet": "maintenance_status", "mode": 0, "mode_name": "Wi-Fi", "flags": 0x25,
                  "ap_up": True, "usb_disk": False, "store_mounted": True, "recovery": False,
                  "unconfirmed": False, "radio_ok": True, "store_version": 4, "command_set": 1,
                  "radio_settings": 1, "maint_image_id": 1, "flight_image_id": 2,
                  "battery_voltage": 7.4, "uptime": 5000, "log_files": 3}
        self.monitor.events.put(("packet", ("COM_TEST", beacon, time.time())))
        self.monitor.process_events()
        self.assertTrue(banner.isVisibleTo(self.monitor))
        self.feed(0x0100)      # flight packets again: the flight image is back
        self.assertFalse(banner.isVisibleTo(self.monitor))


if __name__ == "__main__":
    unittest.main()
