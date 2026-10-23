"""Read-only integration tests; no real COM ports, MQTT broker or receiver needed."""

from collections import deque
import json
from pathlib import Path
import queue
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from featherweight import crc16_buypass
from ground_receivers import FeatherweightReceiver, WitnessReceiver
from ground_tabs import WITNESS_FIELDS
from main import PacketDecoder, SerialMonitor, parse_packet
from telemetry_view import flatten_fields
from witness_link import SCHEMA, WitnessPublisher, decode_record, encode_record


def wait_for(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for input")


def record(sequence=1, *, source="COM7", received_at=None, replay=False):
    return {"schema": SCHEMA, "type": "telemetry", "session": "test-session",
            "sequence": sequence, "source": source,
            "received_at": time.time() if received_at is None else received_at,
            "replay": replay, "packet": {"packet": "sensors", "status": 0x0201,
                                         "uptime": 1234, "filtered_accel": [-1.2, 2.0, 3.0]}}


def receive_line(connection):
    wire = bytearray()
    while not wire.endswith(b"\n"):
        chunk = connection.recv(1)
        if not chunk:
            raise EOFError("Feed closed")
        wire.extend(chunk)
    return decode_record(wire)


class FeedProtocolTests(unittest.TestCase):
    def test_record_validation(self):
        self.assertEqual(decode_record(encode_record(record()))["packet"]["uptime"], 1234)
        for changes in ({"schema": "raw"}, {"received_at": float("nan")},
                        {"source": ""}, {"replay": 1}, {"sequence": True},
                        {"packet": {"packet": "sensors", "value": float("inf")}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_record(json.dumps(dict(record(), **changes)).encode())
        with self.assertRaises(ValueError):
            decode_record(b"x" * 65537)

    def test_fields_cover_all_existing_decoder_packets(self):
        for packet_id, lengths in PacketDecoder.payload_lengths.items():
            for length in lengths:
                packet = parse_packet(packet_id, bytes(length))
                self.assertIsNotNone(packet)
                self.assertLessEqual(set(flatten_fields(packet)), set(WITNESS_FIELDS[packet["packet"]]))

    def test_server_snapshots_sources_and_no_command_api(self):
        publisher = WitnessPublisher(port=0)
        publisher.start()
        self.addCleanup(publisher.close)
        timestamp = time.time() - 30
        publisher.publish("COM7", record()["packet"], timestamp)
        publisher.publish("TCP_test_7000", {"packet": "heartbeat", "status": 1, "uptime": 22})
        with socket.create_connection((publisher.host, publisher.port), timeout=2) as client:
            first, second = receive_line(client), receive_line(client)
            self.assertEqual(first["source"], "COM7")
            self.assertEqual(first["received_at"], timestamp)
            self.assertTrue(first["replay"])
            self.assertEqual(second["source"], "TCP_test_7000")
            publisher.publish("COM7", {"packet": "sensors", "uptime": 1300, "value": float("nan")})
            live = receive_line(client)
            self.assertFalse(live["replay"])
            self.assertEqual(live["sequence"], 3)
            self.assertIsNone(live["packet"]["value"])
            # Protocol deliberately rejects client data instead of forwarding commands.
            client.sendall(b'05 68 69\n')
            try:
                self.assertEqual(client.recv(1), b"")
            except ConnectionResetError:
                pass  # Windows resets a connection closed with unread client data.
        self.assertIsNone(publisher.error)

    def test_slow_client_is_closed_without_stopping_publisher(self):
        publisher = WitnessPublisher(port=0)
        publisher.start()
        self.addCleanup(publisher.close)
        publisher.publish("COM7", {"packet": "heartbeat", "uptime": 1})
        with socket.create_connection((publisher.host, publisher.port), timeout=2) as client:
            receive_line(client)
            with patch("witness_link.MAX_PENDING_BYTES", 100):
                publisher.publish("COM7", {"packet": "heartbeat", "text": "x" * 200})
                self.assertEqual(client.recv(1), b"")
        publisher.publish("COM7", {"packet": "heartbeat", "uptime": 2})
        with socket.create_connection((publisher.host, publisher.port), timeout=2) as client:
            self.assertEqual(receive_line(client)["packet"]["uptime"], 2)
        self.assertIsNone(publisher.error)

    def test_bind_conflict_does_not_steal_existing_feed(self):
        publisher = WitnessPublisher(port=0)
        publisher.start()
        self.addCleanup(publisher.close)
        duplicate = WitnessPublisher(port=publisher.port)
        with self.assertRaises(OSError):
            duplicate.start()
        duplicate.close()
        with self.assertRaises(ValueError):
            WitnessPublisher(host="0.0.0.0")


class ReceiverTests(unittest.TestCase):
    def test_live_decoder_to_receiver_preserves_units_and_logs(self):
        publisher = WitnessPublisher(port=0)
        publisher.start()
        self.addCleanup(publisher.close)
        with tempfile.TemporaryDirectory() as directory:
            receiver = WitnessReceiver(publisher.host, publisher.port, directory)
            receiver.start()
            try:
                wait_for(lambda: receiver.status.startswith("Connected"))
                monitor = SimpleNamespace(publisher=publisher, events=queue.Queue())
                errors = []
                decoder = PacketDecoder(lambda packet: SerialMonitor.on_decoded_packet(
                    monitor, "COM4", packet), errors.append)
                payload = (b'\x02\x01' + (1234).to_bytes(4, "big")
                           + (25000).to_bytes(3, "big") + bytes(6)
                           + (-100).to_bytes(2, "big", signed=True) + bytes(4))
                wire = b'\x01' + payload + b'\x0a'
                for offset in range(0, len(wire), 3):
                    decoder.feed(wire[offset:offset + 3])
                wait_for(lambda: receiver.messages == 1)
                updates, _stats, _diagnostics = receiver.drain()
                self.assertEqual(updates[0]["packet"]["filtered_accel"], [1.0, 0.0, 0.0])
                self.assertEqual(updates[0]["packet"]["gyro"], [-7.0, 0.0, 0.0])
                self.assertEqual(updates[0]["packet"]["status"], 0x0201)
                self.assertEqual(monitor.events.get_nowait()[0], "packet")
                self.assertFalse(errors)
            finally:
                receiver.stop(wait=True)
            lines = [json.loads(line) for line in receiver.log_path.read_text().splitlines()]
            self.assertEqual(len([line for line in lines if line["event"] == "packet"]), 1)
            self.assertFalse(receiver._thread.is_alive())

    def test_fragmented_malformed_messages_and_auto_reconnect(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(2)
        listener.settimeout(4)
        self.addCleanup(listener.close)
        done = threading.Event()
        errors = []
        observed = []
        def server():
            try:
                for attempt in range(2):
                    connection, _ = listener.accept()
                    with connection:
                        connection.settimeout(0.15)
                        wire = encode_record(record(attempt + 1))
                        connection.sendall(b"bad json\n" if attempt == 0 else b"")
                        connection.sendall(wire[:5])
                        connection.sendall(wire[5:])
                        try:
                            observed.append(connection.recv(10))
                        except socket.timeout:
                            observed.append(b"")
            except Exception as exc:
                errors.append(exc)
            finally:
                done.set()
        threading.Thread(target=server, daemon=True).start()
        with tempfile.TemporaryDirectory() as directory:
            receiver = WitnessReceiver("localhost", listener.getsockname()[1], directory)
            receiver.start()
            try:
                wait_for(lambda: receiver.messages == 2)
            finally:
                receiver.stop(wait=True)
                done.wait(4)
            self.assertFalse(errors, repr(errors))
            self.assertEqual(observed, [b"", b""])  # Receiver sends no bytes.
            self.assertEqual(receiver.errors, 1)
            updates, _stats, _diagnostics = receiver.drain()
            self.assertEqual(len(updates), 1)  # UI coalescing, logs retain both.
            records = [json.loads(line) for line in receiver.log_path.read_text().splitlines()]
            self.assertEqual(len([line for line in records if line["event"] == "packet"]), 2)

    def test_featherweight_read_only_crc_noise_and_raw_logging(self):
        body = ("@ GPS_STAT 200 2026 10 12 12:00:00 CRC_OK TRACKER 12 "
                "Alt 1000 lt 34.2 ln -117.5 Vel 10 20 30 Fix 3 # 8 7 6 5 123_45_40")
        crc = crc16_buypass(body.encode("ascii"))
        good = f"{body} CRC: {crc:04X}\r\n".encode()
        bad = f"{body} CRC: {crc ^ 1:04X}\r\n".encode()
        noise = b'FWT\x00@\xff\r\n\x00\xff'
        chunks = deque([noise, good[:14], good[14:], bad])
        config = {}
        class FakeSerial:
            in_waiting = 4096
            closed = False
            def __init__(self, **kwargs):
                config.update(kwargs)
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                self.closed = True
            def read(self, _count):
                if chunks:
                    return chunks.popleft()
                time.sleep(0.01)
                return b""
            def write(self, _data):
                raise AssertionError("Featherweight must never write")
        with tempfile.TemporaryDirectory() as directory:
            receiver = FeatherweightReceiver("COM_FAKE", directory, serial_factory=FakeSerial)
            receiver.start()
            try:
                wait_for(lambda: receiver.messages == 2)
            finally:
                receiver.stop(wait=True)
            updates, stats, _diagnostics = receiver.drain()
            self.assertEqual(stats[3], 1)  # Bad CRC, not binary chatter.
            self.assertEqual(updates[0]["packet"]["latitude_deg"], 34.2)
            self.assertFalse(updates[0]["packet"]["text_crc_valid"])
            self.assertEqual(receiver.raw_path.read_bytes(), noise + good + bad)
            self.assertEqual(config["baudrate"], 115200)
            self.assertFalse(config["xonxoff"])
            self.assertFalse(receiver._thread.is_alive())


class GuiTests(unittest.TestCase):
    def test_live_witness_window_feeds_unified_window(self):
        import tkinter as tk
        from horizon import HorizonGroundStation
        with tempfile.TemporaryDirectory() as directory:
            try:
                unified = HorizonGroundStation(log_dir=Path(directory), auto_connect=False)
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            unified.withdraw()
            witness = None
            try:
                with patch("main.DEFAULT_WITNESS_FEED_PORT", 0):
                    witness = SerialMonitor(auto_connect=False)
                witness.withdraw()
                unified.witness.port.set(str(witness.publisher.port))
                unified.witness.connect()
                # Both serial and raw TCP decoders use this callback.
                witness.on_decoded_packet("COM_LIVE", {"packet": "state", "status": 0x0B02,
                    "uptime": 2100, "barometric_altitude": 125.5, "witness_battery_voltage": 4.125})
                key = ("COM_LIVE", "state", "witness_battery_voltage")
                def displayed():
                    unified.update()
                    witness.update()
                    row = unified.witness.view._rows.get(key)
                    return row and unified.witness.view.tree.set(row, "value") == "4.125"
                wait_for(displayed)
                self.assertIsNone(unified.aim.receiver)
                self.assertIsNone(unified.featherweight.receiver)
                receiver = unified.witness.receiver
                unified.witness.disconnect()
                self.assertFalse(receiver._thread.is_alive())
                self.assertTrue(witness.publisher._thread.is_alive())
                # Closing Horizon's feed does not close or command Witness ports.
                self.assertEqual(witness.values["packet.state.barometric_altitude"].get(), "125.5")
            finally:
                if witness is not None:
                    for callback in witness.tk.call("after", "info"):
                        witness.after_cancel(callback)
                    witness.close()
                unified.close()

    def test_unified_tabs_and_witness_feed_widgets(self):
        import tkinter as tk
        from horizon import HorizonGroundStation
        with tempfile.TemporaryDirectory() as directory:
            try:
                unified = HorizonGroundStation(log_dir=Path(directory), auto_connect=False)
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            unified.withdraw()
            try:
                titles = [unified.notebook.tab(tab, "text") for tab in unified.notebook.tabs()]
                self.assertEqual(titles, ["Flight View", "Witness", "Featherweight GPS", "AIM XTRA MQTT", "Layout Editor"])
                replay = record(received_at=time.time() - 20, replay=True)
                unified.witness.view.update_record(replay, witness=True)
                unified.witness.view.update_ages()
                view = unified.witness.view
                row = view._rows[("COM7", "sensors", "status")]
                self.assertEqual(view.tree.set(row, "value"), "0b0000001000000001")
                self.assertEqual(view.tree.set(row, "quality"), "Snapshot")
                self.assertEqual(view.tree.item(row, "tags"), ("stale",))
                row = view._rows[("COM7", "sensors", "status.flight_state_name")]
                self.assertEqual(view.tree.set(row, "value"), "Boost")
                row = view._rows[("COM7", "state", "witness_battery_voltage")]
                self.assertEqual(view.tree.set(row, "value"), "null")
                self.assertEqual(unified.aim.host.get(), "localhost")
                self.assertEqual(unified.aim.base_path.get(), "entacore/aim/1234")
                unified.update()
            finally:
                unified.close()
            with patch("main.DEFAULT_WITNESS_FEED_PORT", 0):
                witness = SerialMonitor(auto_connect=False)
            witness.withdraw()
            try:
                self.assertFalse(hasattr(witness, "aim_page"))
                self.assertFalse(hasattr(witness, "featherweight_serial"))
                self.assertIn("receive only", witness.feed_status.get())
                witness.on_decoded_packet("COM_TEST", {"packet": "heartbeat", "status": 1, "uptime": 5})
                witness.process_events()
                self.assertEqual(witness.values["packet.heartbeat.uptime"].get(), "5")
                # Cancel existing monitor timers before destroying this test Tcl interpreter.
                for callback in witness.tk.call("after", "info"):
                    witness.after_cancel(callback)
            finally:
                witness.close()
            self.assertFalse(witness.publisher._thread.is_alive())


if __name__ == "__main__":
    unittest.main()
