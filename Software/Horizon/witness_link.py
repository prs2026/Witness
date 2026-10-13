"""Local, one-way Witness telemetry feed (versioned newline-delimited JSON).

Publishing never performs socket I/O on a decoder or Tk thread. Slow clients
are disconnected, not allowed to stall Witness. New clients get latest-packet
snapshots with original receive times and an explicit replay marker.
"""

from __future__ import annotations

from collections import OrderedDict, deque
import json
import math
import select
import socket
import threading
import time
import uuid

from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT

SCHEMA = "horizon.witness.v1"
MAX_LINE_BYTES = 65536
MAX_PENDING_BYTES = 512 * 1024
MAX_PACKET_TYPES = 128


def json_safe(value):
    """Preserve decoded values; invalid float samples are explicitly null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def encode_record(record):
    wire = json.dumps(record, ensure_ascii=True, allow_nan=False,
                      separators=(",", ":")).encode("utf-8") + b"\n"
    if len(wire) > MAX_LINE_BYTES:
        raise ValueError("Witness feed record exceeds 64 KiB")
    return wire


def decode_record(line):
    if len(line) > MAX_LINE_BYTES:
        raise ValueError("Witness feed record exceeds 64 KiB")
    def invalid_number(value):
        raise ValueError(f"Non-finite JSON number: {value}")
    try:
        record = json.loads(line, parse_constant=invalid_number)
    except (ValueError, UnicodeDecodeError, RecursionError) as exc:
        raise ValueError(f"Invalid Witness JSON: {exc}") from exc
    if not isinstance(record, dict) or record.get("schema") != SCHEMA:
        raise ValueError("Not a Horizon Witness v1 feed; do not use a raw telemetry port")
    if record.get("type") != "telemetry":
        raise ValueError("Unknown Witness feed record type")
    for name in ("source", "session"):
        if not isinstance(record.get(name), str) or not 1 <= len(record[name]) <= 256:
            raise ValueError(f"Invalid Witness {name}")
    if type(record.get("sequence")) is not int or record["sequence"] < 1:
        raise ValueError("Invalid Witness sequence")
    received = record.get("received_at")
    if type(received) not in (int, float) or not math.isfinite(received) or received < 0:
        raise ValueError("Invalid Witness receive timestamp")
    if type(record.get("replay")) is not bool:
        raise ValueError("Missing Witness replay flag")
    packet = record.get("packet")
    if (not isinstance(packet, dict) or not isinstance(packet.get("packet"), str)
            or not 1 <= len(packet["packet"]) <= 100):
        raise ValueError("Invalid Witness packet")
    # Validate the entire value tree before accepting or displaying it. json.loads
    # can produce infinity from e.g. 1e999 without invoking parse_constant.
    count = 0
    def validate(value, depth=0):
        nonlocal count
        count += 1
        if depth > 16 or count > 1024:
            raise ValueError("Witness packet nesting/field limit exceeded")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Non-finite Witness packet value")
        if isinstance(value, dict):
            for item in value.values():
                validate(item, depth + 1)
        elif isinstance(value, list):
            for item in value:
                validate(item, depth + 1)
    validate(packet)
    return record


class WitnessPublisher:
    def __init__(self, host=WITNESS_FEED_HOST, port=WITNESS_FEED_PORT):
        if host != "127.0.0.1":
            raise ValueError("The Witness feed must bind to 127.0.0.1")
        self.host, self.port = host, port
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.dropped = 0
        self.error = None
        self._lock = threading.Lock()
        self._pending = deque()
        self._latest = OrderedDict()
        self._stop = threading.Event()
        self._listener = None
        self._thread = None

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Witness feed already started")
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Never steal an existing application's listening endpoint on Windows.
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            listener.bind((self.host, self.port))
            listener.listen(4)
            listener.setblocking(False)
        except Exception:
            listener.close()
            raise
        self.port = listener.getsockname()[1]
        self._listener = listener
        self._thread = threading.Thread(target=self._run, name="witness-feed", daemon=True)
        self._thread.start()

    def publish(self, source, packet, received_at=None):
        if self._thread is None or self._stop.is_set():
            return
        with self._lock:
            self.sequence += 1
            record = {"schema": SCHEMA, "type": "telemetry", "session": self.session,
                      "sequence": self.sequence, "source": source,
                      "received_at": time.time() if received_at is None else received_at,
                      "replay": False, "packet": json_safe(packet)}
            wire = encode_record(record)
            key = (source, packet["packet"])
            self._latest[key] = record
            self._latest.move_to_end(key)
            if len(self._latest) > MAX_PACKET_TYPES:
                self._latest.popitem(last=False)
            if len(self._pending) >= 4096:
                self._pending.popleft()
                self.dropped += 1
            self._pending.append((self.sequence, wire))

    def _run(self):
        clients = {}
        def close_client(client):
            clients.pop(client, None)
            client.close()
        try:
            while not self._stop.is_set():
                readable, writable, _ = select.select(
                    [self._listener, *clients],
                    [client for client, (pending, _seq) in clients.items() if pending],
                    [], 0.02,
                )
                for client in readable:
                    if client is self._listener:
                        connection, _ = self._listener.accept()
                        connection.setblocking(False)
                        if len(clients) >= 8:
                            connection.close()
                            continue
                        with self._lock:
                            latest = sorted(self._latest.values(), key=lambda item: item["sequence"])
                            snapshot = b"".join(encode_record(dict(item, replay=True)) for item in latest)
                            sequence = self.sequence
                        if len(snapshot) > MAX_PENDING_BYTES:
                            connection.close()
                        else:
                            clients[connection] = (bytearray(snapshot), sequence)
                    else:
                        # Clients have no command API. Disconnect on inbound data or EOF.
                        try:
                            client.recv(1)
                        except BlockingIOError:
                            continue
                        except OSError:
                            pass
                        close_client(client)
                with self._lock:
                    outgoing = list(self._pending)
                    self._pending.clear()
                for client, (pending, sequence) in list(clients.items()):
                    for record_sequence, wire in outgoing:
                        if record_sequence > sequence:
                            pending.extend(wire)
                            sequence = record_sequence
                        if len(pending) > MAX_PENDING_BYTES:
                            close_client(client)
                            break
                    else:
                        clients[client] = pending, sequence
                for client in writable:
                    if client not in clients:
                        continue
                    pending, _ = clients[client]
                    try:
                        sent = client.send(pending)
                        if sent == 0:
                            close_client(client)
                        else:
                            del pending[:sent]
                    except BlockingIOError:
                        continue
                    except OSError:
                        close_client(client)
        except OSError as exc:
            if not self._stop.is_set():
                self.error = str(exc)
        finally:
            for client in list(clients):
                close_client(client)
            self._listener.close()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
