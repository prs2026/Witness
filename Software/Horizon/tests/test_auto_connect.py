"""Automatic connection tests use synthetic ports and loopback/fake receivers only."""

import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from aim_mqtt import AimMqttReceiver, AimSettings
from ground_receivers import WitnessReceiver
from serial_discovery import SerialDiscovery, SerialOpenJob


def port(device, description="USB Serial", serial="serial-1", location="1-2"):
    return SimpleNamespace(device=device, description=description, manufacturer=None, product=None,
                           vid=0x1234, pid=0x5678, serial_number=serial, location=location)


class DiscoveryTests(unittest.TestCase):
    def test_only_named_or_learned_devices_and_com_renumbering(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fw.json"
            discovery = SerialDiscovery("featherweight", path=path)
            discovery.ports = [port("COM1", "Featherweight GPS"), port("COM2", serial="generic"),
                               port("COM3", "Witness Ground Station", serial="other")]
            self.assertEqual(discovery.candidates(), ["COM1"])
            discovery.remember("COM2")
            self.assertEqual(discovery.candidates(), ["COM1", "COM2"])
            self.assertNotIn("COM2", path.read_text())  # USB identity, not COM number.
            discovery.attempted("COM1")
            self.assertEqual(discovery.candidates(), ["COM2", "COM1"])
            self.assertEqual(discovery.candidates(exclude=["COM2"]), ["COM1"])
            reloaded = SerialDiscovery("featherweight", path=path)
            reloaded.ports = [port("COM99", serial="generic"), port("COM2", serial="unrelated")]
            self.assertEqual(reloaded.candidates(), ["COM99"])

    def test_location_fallback_missing_identity_bad_config_and_save_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "devices.json"
            discovery = SerialDiscovery("witness", path=path)
            discovery.ports = [port("COM1", serial=None), port("COM2", serial=None, location=None)]
            discovery.remember("COM1")
            discovery.remember("COM2")
            self.assertEqual(discovery.known, [{"vid": 0x1234, "pid": 0x5678, "location": "1-2"}])
            discovery.ports = [port("COM8", serial=None), port("COM9", serial=None, location="2-3")]
            self.assertEqual(discovery.candidates(), ["COM8"])
            with patch.object(Path, "read_text", return_value="not json"):
                bad = SerialDiscovery("witness", path=path)
            self.assertIsNotNone(bad.error)
            self.assertEqual(bad.known, [])
            with patch.object(Path, "write_text", side_effect=OSError("read only")):
                discovery.remember("COM9")
            self.assertIn("read only", discovery.error)

    def test_scan_is_background_and_failures_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            discovery = SerialDiscovery("witness", path=Path(directory) / "devices.json")
            with patch("serial_discovery.enumerate_ports", return_value=[port("COM1", "Witness")]) as scan:
                discovery.refresh()
                discovery._thread.join(1)
                discovery.refresh()
                self.assertEqual(discovery.candidates(), ["COM1"])
                self.assertEqual(scan.call_count, 1)
            with patch("serial_discovery.enumerate_ports", side_effect=OSError("device list failed")):
                discovery.refresh(force=True)
                discovery._thread.join(1)
                discovery.refresh()
            self.assertEqual(discovery.ports, [])
            self.assertIn("device list failed", discovery.error)

    def test_cancellation_closes_late_serial_open_without_writing(self):
        entered, release = threading.Event(), threading.Event()
        stream = Mock()

        def open_serial(*args, **kwargs):
            entered.set()
            release.wait(2)
            return stream

        with patch("serial.Serial", side_effect=open_serial):
            job = SerialOpenJob("COM_FAKE", 115200)
            self.assertTrue(entered.wait(1))
            self.assertIsNone(job.take())
            job.cancel()
            release.set()
            job.thread.join(2)
        stream.close.assert_called_once()
        stream.write.assert_not_called()
        self.assertIsNone(job.take())

    def test_success_cancel_completed_and_busy_port(self):
        stream = Mock()
        with patch("serial.Serial", return_value=stream) as factory:
            job = SerialOpenJob("COM_FAKE", 115200)
            job.thread.join(1)
            self.assertEqual(job.take(), (stream, None))
            factory.assert_called_once_with("COM_FAKE", 115200, timeout=0.2)
            job.cancel()
            stream.close.assert_not_called()  # Ownership transferred to the UI.
            job = SerialOpenJob("COM_FAKE", 115200)
            job.thread.join(1)
            job.cancel()
            stream.close.assert_called_once()
        with patch("serial.Serial", side_effect=OSError("busy")):
            job = SerialOpenJob("COM_FAKE", 115200)
            job.thread.join(1)
            self.assertEqual(job.take(), (None, "busy"))


class RetryTests(unittest.TestCase):
    def test_witness_disabled_does_not_retry_failed_connect(self):
        with tempfile.TemporaryDirectory() as directory:
            receiver = WitnessReceiver("localhost", 8766, directory)
            receiver.auto_retry = False
            with patch("ground_receivers.socket.create_connection", side_effect=OSError("offline")) as connect:
                receiver.start()
                receiver._thread.join(2)
                self.assertFalse(receiver.running)
                self.assertEqual(connect.call_count, 1)
            receiver.stop(wait=True)
            self.assertIn("auto-connect off", receiver.status)

    def test_aim_disabled_no_retry_and_stop_during_open(self):
        receiver = AimMqttReceiver(AimSettings("localhost", 1883, "124", "entacore/aim/1234"), Path("unused"))
        receiver.client = Mock()
        receiver.client.reconnect.side_effect = OSError("offline")
        receiver.auto_retry = False
        receiver._network_loop()
        receiver.client.reconnect.assert_called_once()
        receiver.client.loop_forever.assert_not_called()
        self.assertIn("auto-connect off", receiver.status)
        receiver.client.reset_mock()
        receiver.client.reconnect.side_effect = receiver._stopped.set
        receiver._network_loop()
        receiver.client.disconnect.assert_called_once()
        receiver.client.loop_forever.assert_not_called()


class AutoGuiTests(unittest.TestCase):
    def make_horizon(self, directory, **kwargs):
        import tkinter as tk
        from horizon import HorizonGroundStation
        try:
            app = HorizonGroundStation(log_dir=Path(directory), broker_enabled=False, **kwargs)
        except tk.TclError as exc:
            self.skipTest(f"Tk display unavailable: {exc}")
        app.withdraw()
        return app

    def test_horizon_defaults_manual_disconnect_and_off_preserves_connection(self):
        with tempfile.TemporaryDirectory() as directory, patch("serial_discovery.enumerate_ports", return_value=[]):
            app = self.make_horizon(directory)
            try:
                for tab in (app.witness, app.aim, app.featherweight):
                    self.assertTrue(tab.auto_connect.get())
                with patch.object(app.witness, "connect") as witness, patch.object(app.aim, "connect") as aim, \
                        patch.object(app.featherweight, "connect") as fw:
                    app.witness.auto_tick()
                    app.aim.auto_tick()
                    app.featherweight.auto_tick()
                    witness.assert_called_once_with(automatic=True)
                    aim.assert_called_once_with(automatic=True)
                    fw.assert_not_called()
                    app.witness.auto_tick()
                    self.assertEqual(witness.call_count, 1)  # Rate limited.
                for tab in (app.witness, app.aim, app.featherweight):
                    receiver = Mock(running=True, auto_retry=True)
                    receiver.drain.return_value = ({} if tab is app.aim else [], ("Connected", 0, 0, 0), [])
                    tab.receiver = receiver
                    tab.set_auto_connect(False)
                    self.assertIs(tab.receiver, receiver)
                    self.assertFalse(receiver.auto_retry)
                    receiver.stop.assert_not_called()
                    tab.set_auto_connect(True)
                    self.assertTrue(receiver.auto_retry)
                    tab.toggle_connection()
                    self.assertFalse(tab.auto_connect.get())
                    self.assertIsNone(tab.receiver)
            finally:
                app.close()

    def test_featherweight_discovery_retry_and_manual_learning(self):
        with tempfile.TemporaryDirectory() as directory, patch("serial_discovery.enumerate_ports", return_value=[]):
            app = self.make_horizon(directory, auto_connect=False)
            try:
                tab = app.featherweight
                discovery = SerialDiscovery("featherweight", path=Path(directory) / "fw.json")
                discovery.ports = [port("COM_FAKE", "Featherweight GPS"), port("COM_GENERIC", serial="generic")]
                tab.discovery = discovery
                tab.set_auto_connect(True)
                with patch.object(discovery, "refresh"), patch.object(tab, "connect") as connect:
                    tab.auto_tick()
                    self.assertEqual(tab.port_combo.get(), "COM_FAKE")
                    connect.assert_called_once_with(automatic=True)
                    failed = Mock(running=False, status="Device unplugged")
                    failed.drain.return_value = ([], ("Device unplugged", 0, 0, 0), [])
                    tab.receiver = failed
                    tab.auto_tick()
                    self.assertIsNone(tab.receiver)
                    self.assertEqual(tab.status.get(), "Device unplugged")
                    self.assertEqual(connect.call_count, 1)
                    tab._next_connect = 0
                    tab.auto_tick()
                    self.assertEqual(connect.call_count, 2)
                tab.remember_source({"source": "COM_GENERIC", "packet": {"text_crc_valid": False}})
                self.assertFalse(discovery.path.exists())
                tab.remember_source({"source": "COM_GENERIC", "packet": {"text_crc_valid": True}})
                self.assertEqual(json.loads(discovery.path.read_text())[0]["serial_number"], "generic")
            finally:
                app.close()

    def test_main_auto_open_error_reconnect_disable_and_test_isolation(self):
        import tkinter as tk
        from main import SerialMonitor
        with tempfile.TemporaryDirectory() as directory, \
                patch("main.DEFAULT_WITNESS_FEED_PORT", 0), patch("main.LOG_DIR", Path(directory)), \
                patch("serial_discovery.enumerate_ports", return_value=[]), \
                patch("main.SerialOpenJob") as open_job:
            try:
                app = SerialMonitor()
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            app.withdraw()
            try:
                self.assertTrue(app.auto_connect.get())
                app.discovery = SerialDiscovery("witness", path=Path(directory) / "witness.json")
                app.discovery.ports = [port("COM_W", "Witness"), port("COM_F", "Featherweight", serial="fw")]
                stream = Mock()
                open_job.return_value.port = "COM_W"
                open_job.return_value.take.return_value = None
                with patch.object(app.discovery, "refresh"), patch.object(app, "read_serial"):
                    app._auto_poll()
                    open_job.assert_called_once_with("COM_W", 115200)
                    open_job.return_value.take.return_value = (stream, None)
                    app._auto_poll()
                    self.assertIs(app.serials["COM_W"], stream)
                    self.assertIn("COM_W", app.command_port_combo["values"])
                    # Unplug releases only this input; a later scan can reopen it.
                    app.events.put(("error", ("COM_W", "unplugged", stream)))
                    app.process_events()
                    self.assertFalse(app.serials)
                    self.assertTrue(app.auto_connect.get())
                    app._next_auto_open = 0
                    app._auto_poll()
                    self.assertEqual(open_job.call_count, 2)
                    app.toggle_connection()  # Cancel pending open, do not re-open.
                    self.assertFalse(app.auto_connect.get())
                    open_job.return_value.cancel.assert_called()
                    app._auto_poll()
                    self.assertEqual(open_job.call_count, 2)
                    app.auto_connect.set(True)
                    self.assertTrue(app.start_test())
                    self.assertFalse(app.auto_connect.get())
                    app.auto_connect.set(True)
                    app._auto_changed()
                    self.assertFalse(app.auto_connect.get())
                    app._auto_poll()
                    self.assertEqual(open_job.call_count, 2)
            finally:
                for timer in app.tk.call("after", "info"):
                    app.after_cancel(timer)
                app.close()


if __name__ == "__main__":
    unittest.main()
