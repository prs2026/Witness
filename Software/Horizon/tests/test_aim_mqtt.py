"""Protocol and transport checks: python -B -m unittest discover -s tests -v."""

import base64
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from aim_mqtt import AimMqttReceiver, AimSettings, decode_payload, relative_topic


class PayloadTests(unittest.TestCase):
    def test_installed_publisher_gnss_schema(self):
        fields = decode_payload(json.dumps({
            "timestamp": 1791500000,
            "gnss_coordinate": {"latitude": 39.55612, "longitude": -105.1032, "altitude": 1723.7},
            "satellites_used": 9,
        }).encode())
        self.assertEqual(fields["gnss_coordinate.longitude"], -105.1032)
        self.assertEqual(fields["gnss_coordinate.altitude"], 1723.7)
        self.assertEqual(fields["satellites_used"], 9)
        self.assertEqual(fields["timestamp"], 1791500000)

    def test_extensions_arrays_null_and_literal_dotted_keys(self):
        fields = decode_payload(b'{"extra": [0, false, null], "a.b": 1, "a": {"b": 2}}')
        self.assertEqual(fields, {
            "extra[0]": 0, "extra[1]": False, "extra[2]": None,
            r"a\.b": 1, "a.b": 2,
        })

    def test_malformed_and_nonfinite_values(self):
        for payload in (b'\xff', b'{broken', b'{"value": NaN}', b'{"value": 1e999}'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                decode_payload(payload)

    def test_base_path_is_authoritative_not_subscriber_uuid(self):
        settings = AimSettings("localhost", 1883, "124", "entacore/aim/1234")
        self.assertEqual(settings.subscription, "entacore/aim/1234/#")
        self.assertEqual(relative_topic("entacore/aim/1234/gnss_payload/v1", settings.base_path),
                         "gnss_payload/v1")
        with self.assertRaises(ValueError):
            relative_topic("entacore/aim/12345/gnss_payload/v1", settings.base_path)
        for base in ("", "#", "entacore/+/test"):
            with self.subTest(base=base), self.assertRaises(ValueError):
                AimSettings("localhost", 1883, "124", base)

    def test_partial_updates_coalesce_by_field_and_bad_packet_does_not_replace_values(self):
        receiver = AimMqttReceiver(AimSettings("localhost", 1883, "124", "entacore/aim/1234"), Path("logs"))
        for payload in (b'{"voltage": 4.2}', b'{"timestamp": 100}', b'bad JSON', b'{"voltage": 4.1}'):
            receiver._on_message(None, None, SimpleNamespace(
                topic="entacore/aim/1234/computer_battery_payload/v1",
                payload=payload, qos=0, retain=False,
            ))
        updates, stats, diagnostics = receiver.drain()
        fields = updates["computer_battery_payload/v1"]
        self.assertEqual(fields["voltage"][0], 4.1)
        self.assertEqual(fields["timestamp"][0], 100)
        self.assertEqual(stats[1], 4)
        self.assertEqual(stats[3], 1)
        self.assertTrue(any("BAD_MESSAGE" in message for message in diagnostics))


def mqtt_string(text):
    data = text.encode()
    return len(data).to_bytes(2, "big") + data


def mqtt_packet(header, body):
    length = len(body)
    encoded = bytearray()
    while True:
        digit = length % 128
        length //= 128
        encoded.append(digit | (0x80 if length else 0))
        if not length:
            return bytes([header]) + encoded + body


def read_exact(connection, count):
    data = bytearray()
    while len(data) < count:
        chunk = connection.recv(count - len(data))
        if not chunk:
            raise EOFError("MQTT peer closed")
        data.extend(chunk)
    return bytes(data)


def read_packet(connection):
    header = read_exact(connection, 1)[0]
    length = 0
    for shift in range(0, 28, 7):
        digit = read_exact(connection, 1)[0]
        length |= (digit & 0x7f) << shift
        if not digit & 0x80:
            return header, read_exact(connection, length)
    raise ValueError("Invalid MQTT remaining length")


class TransportTests(unittest.TestCase):
    def test_real_paho_subscription_reconnect_logging_and_receive_only(self):
        # This fixture listens only on an ephemeral loopback port. It does not
        # publish to the user's broker or communicate with flight hardware.
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        listener.settimeout(8)
        topic = "entacore/aim/1234/computer_battery_payload/v1"
        server_errors = []
        commands = []
        client_ids = []
        completed = threading.Event()

        def serve():
            try:
                # Drop the first connection to exercise Paho's re-subscription.
                for attempt in range(2):
                    connection, _address = listener.accept()
                    with connection:
                        connection.settimeout(8)
                        header, body = read_packet(connection)
                        commands.append(header >> 4)
                        self.assertEqual(header, 0x10)  # CONNECT
                        self.assertEqual(body[:7], b'\x00\x04MQTT\x04')
                        self.assertEqual(body[7], 2)  # clean session, no will or credentials
                        length = int.from_bytes(body[10:12], "big")
                        client_ids.append(body[12:12 + length].decode())
                        connection.sendall(b'\x20\x02\x00\x00')  # CONNACK
                        header, body = read_packet(connection)
                        commands.append(header >> 4)
                        self.assertEqual(header, 0x82)
                        self.assertEqual(body[2:], mqtt_string("entacore/aim/1234/#") + b'\x00')
                        connection.sendall(mqtt_packet(0x90, body[:2] + b'\x00'))
                        if attempt == 0:
                            continue
                        # Split a long MQTT PUBLISH over arbitrary TCP boundaries.
                        payload = json.dumps({"timestamp": 1791500000, "voltage": 4.2,
                                              "extra": "x" * 150}).encode()
                        wire = mqtt_packet(0x31, mqtt_string(topic) + payload)
                        connection.sendall(wire[:3])
                        connection.sendall(wire[3:29])
                        connection.sendall(wire[29:])
                        connection.sendall(mqtt_packet(0x30, mqtt_string(topic) + b'broken JSON'))
                        connection.sendall(mqtt_packet(0x30, mqtt_string(topic) + b'{"voltage":4.1}'))
                        while True:
                            header, _body = read_packet(connection)
                            commands.append(header >> 4)
                            if header == 0xe0:
                                break
                            self.assertEqual(header, 0xc0)  # Only keepalives permitted
                            connection.sendall(b'\xd0\x00')
            except Exception as exc:
                server_errors.append(exc)
            finally:
                completed.set()

        server = threading.Thread(target=serve, daemon=True)
        server.start()
        with tempfile.TemporaryDirectory(prefix="horizon-aim-test-") as temp:
            receiver = AimMqttReceiver(AimSettings(
                "127.0.0.1", listener.getsockname()[1], "124", "entacore/aim/1234",
            ), Path(temp))
            try:
                receiver.start()
                deadline = time.monotonic() + 8
                while receiver.messages < 3 and time.monotonic() < deadline:
                    if server_errors:
                        raise server_errors[0]
                    time.sleep(0.02)
                updates, stats, _diagnostics = receiver.drain()
                self.assertEqual(stats[1], 3)
                self.assertEqual(stats[3], 1)
                fields = updates["computer_battery_payload/v1"]
                self.assertEqual(fields["voltage"][0], 4.1)
                self.assertFalse(fields["voltage"][3])
                self.assertTrue(fields["timestamp"][3])  # remains marked as retained
            finally:
                receiver.stop(wait=True)
                completed.wait(3)
                listener.close()
            self.assertFalse(server_errors, repr(server_errors))
            self.assertNotIn(3, commands)  # Client never issued a PUBLISH
            self.assertEqual(commands.count(8), 2)  # SUBSCRIBE after each connect
            self.assertTrue(client_ids[0].startswith("horizon-aim-124-"))
            self.assertEqual(client_ids[0], client_ids[1])
            records = [json.loads(line) for line in receiver.log_path.read_text().splitlines()]
            messages = [record for record in records if record["event"] == "message"]
            self.assertEqual(len(messages), 3)
            self.assertEqual(base64.b64decode(messages[1]["payload_base64"]), b'broken JSON')
            self.assertIn("error", messages[1])
            self.assertEqual(messages[-1]["fields"]["voltage"], 4.1)


if __name__ == "__main__":
    unittest.main()
