"""Exercise the embedded broker over loopback; never use the live AIM port."""

import json
from pathlib import Path
import queue
import socket
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import paho.mqtt.client as mqtt

from aim_mqtt import AimMqttReceiver, AimSettings
from mqtt_broker import EmbeddedMqttBroker


def wait_for(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("Timed out waiting for broker/client")


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class BrokerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.broker = EmbeddedMqttBroker(self.directory.name, free_port())
        self.addCleanup(lambda: self.broker.stop(wait=True))

    def start_broker(self):
        self.broker.start()
        wait_for(lambda: self.broker.snapshot()[0] in ("running", "failed"))
        self.assertEqual(self.broker.snapshot()[0], "running", self.broker.snapshot()[1])

    def client(self, subscription=None):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv311)
        ready, subscribed = threading.Event(), threading.Event()
        messages = queue.Queue()
        client.on_connect = lambda *_: ready.set()
        client.on_subscribe = lambda *_: subscribed.set()
        client.on_message = lambda _c, _u, message: messages.put(message)
        client.connect("127.0.0.1", self.broker.port, keepalive=5)
        client.loop_start()
        def close():
            client.disconnect()
            client.loop_stop()
        self.addCleanup(close)
        self.assertTrue(ready.wait(5))
        if subscription:
            client.subscribe(subscription, qos=2)
            self.assertTrue(subscribed.wait(5))
        return client, messages

    def test_qos_wildcards_retained_and_aim_pipeline(self):
        self.start_broker()
        _, messages = self.client("entacore/aim/1234/+/v1")
        publisher, _ = self.client()
        samples = queue.Queue()
        receiver = AimMqttReceiver(AimSettings("127.0.0.1", self.broker.port, "test", "entacore/aim/1234"),
                                   Path(self.directory.name), on_sample=lambda *args: samples.put(args))
        receiver.start()
        self.addCleanup(lambda: receiver.stop(wait=True))
        wait_for(lambda: receiver.status.startswith("Subscribed"))
        for qos in (0, 1, 2):
            payload = json.dumps({"timestamp": qos + 1, "pressure": 100000 + qos}).encode()
            info = publisher.publish("entacore/aim/1234/pressure_payload/v1", payload, qos=qos, retain=True)
            info.wait_for_publish(5)
            self.assertTrue(info.is_published())
            self.assertEqual(messages.get(timeout=5).payload, payload)
            sample = samples.get(timeout=5)
            self.assertEqual(sample[1], "pressure_payload/v1")
            self.assertEqual(sample[2]["pressure"], 100000 + qos)
            self.assertFalse(sample[-1])
        _, retained = self.client("entacore/aim/1234/#")
        message = retained.get(timeout=5)
        self.assertTrue(message.retain)
        self.assertEqual(json.loads(message.payload)["pressure"], 100002)
        self.assertEqual(receiver.errors, 0)

    def test_restart_shutdown_and_logs(self):
        self.start_broker()
        self.broker.start()  # Idempotent.
        first_thread = self.broker._thread
        client, _ = self.client()
        disconnected = threading.Event()
        client.on_disconnect = lambda *_: disconnected.set()
        self.broker.stop(wait=True)
        self.assertTrue(disconnected.wait(3))
        self.assertFalse(first_thread.is_alive())
        self.assertEqual(self.broker.snapshot()[0], "stopped")
        log = self.broker.snapshot()[2].read_text(encoding="utf-8")
        self.assertIn("Listening on 127.0.0.1", log)
        self.assertIn("Broker stopped", log)
        self.start_broker()
        self.assertIsNot(first_thread, self.broker._thread)

    def test_occupied_port_reports_failure_and_preserves_other_server(self):
        with socket.socket() as owner:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                owner.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            owner.bind(("127.0.0.1", self.broker.port))
            owner.listen()
            self.broker.start()
            wait_for(lambda: not self.broker.alive)
            self.assertEqual(self.broker.snapshot()[0], "failed")
            self.assertIn("occupied", self.broker.snapshot()[1])
            self.broker.stop(wait=True)
            with socket.create_connection(owner.getsockname(), timeout=1):
                accepted, _ = owner.accept()
                accepted.close()
        self.start_broker()

    def test_missing_dependency_and_immediate_stop_are_safe(self):
        import builtins
        real_import = builtins.__import__
        def missing(name, *args, **kwargs):
            if name == "amqtt.broker":
                raise ImportError("test missing dependency")
            return real_import(name, *args, **kwargs)
        with patch("builtins.__import__", side_effect=missing):
            self.broker.start()
            wait_for(lambda: not self.broker.alive)
        self.assertEqual(self.broker.snapshot()[0], "failed")
        self.assertIn("requirements.txt", self.broker.snapshot()[1])
        self.broker.start()
        self.broker.stop(wait=True)
        self.assertFalse(self.broker.alive)
        self.assertEqual(self.broker.snapshot()[0], "stopped")


class BrokerGuiTests(unittest.TestCase):
    def test_default_enabled_checkbox_restart_and_close(self):
        from horizon import HorizonGroundStation
        with tempfile.TemporaryDirectory() as directory:
            port = free_port()
            try:
                # Broker default remains on even if subscriber auto-connect is off.
                app = HorizonGroundStation(log_dir=Path(directory), auto_connect=False, broker_port=port)
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            app.withdraw()
            try:
                tab = app.aim
                self.assertTrue(tab.broker_enabled.get())
                wait_for(lambda: tab.broker.snapshot()[0] == "running")
                tab._poll_broker()
                self.assertIn(str(port), tab.broker_status.get())
                tab.broker_button.invoke()
                self.assertFalse(tab.broker_enabled.get())
                wait_for(lambda: not tab.broker.alive)
                tab._poll_broker()
                self.assertFalse(tab.broker.alive)
                tab.broker_button.invoke()
                wait_for(lambda: tab.broker.snapshot()[0] == "running")
                # Stopping the subscriber does not stop the broker.
                tab.disconnect()
                self.assertEqual(tab.broker.snapshot()[0], "running")
            finally:
                app.close()
            self.assertFalse(tab.broker.alive)


if __name__ == "__main__":
    unittest.main()
