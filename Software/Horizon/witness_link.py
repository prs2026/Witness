"""One-way Witness telemetry feed (versioned newline-delimited JSON).

Publishing never performs socket I/O on a decoder or GUI thread. Slow clients
are disconnected, not allowed to stall Witness. New clients get latest-packet
snapshots with original receive times and an explicit replay marker.

Two independent publishers can run at once: the local feed (127.0.0.1) and a
remote feed bound to this PC's Tailscale address. Each has its own socket,
thread, queue and client buffers, so a slow or dead remote client can only
ever be disconnected from the remote feed; the local feed never waits on it.
Only Tailscale addresses (100.64.0.0/10, fd7a:115c:a1e0::/48) are accepted for
the remote feed, so it is never exposed on the LAN or the internet; the
tailnet's own authentication and ACLs decide who can connect.
"""

from __future__ import annotations

from collections import OrderedDict, deque
import ipaddress
import json
import math
import select
import shutil
import socket
import subprocess
import threading
import time
import uuid

from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT

SCHEMA = "horizon.witness.v1"
MAX_LINE_BYTES = 65536
MAX_PENDING_BYTES = 512 * 1024
MAX_PACKET_TYPES = 128
# Remote links (Tailscale over cellular/Wi-Fi) stall for seconds at a time;
# give them a larger buffer before a slow client is dropped.
REMOTE_MAX_PENDING_BYTES = 8 * 1024 * 1024
TAILSCALE_NETWORKS = (ipaddress.ip_network("100.64.0.0/10"), ipaddress.ip_network("fd7a:115c:a1e0::/48"))
KEEPALIVE_IDLE_MS, KEEPALIVE_INTERVAL_MS = 5000, 1000


def is_tailscale_address(address) -> bool:
    try:
        ip = ipaddress.ip_address(str(address).split("%")[0])
    except ValueError:
        return False
    return any(ip in network for network in TAILSCALE_NETWORKS)


def is_loopback_address(address) -> bool:
    try:
        return ipaddress.ip_address(str(address)).is_loopback
    except ValueError:
        return False


def tailscale_ipv4():
    """This PC's Tailscale IPv4 address, or None when Tailscale is not up."""
    try:
        import psutil
        for addresses in psutil.net_if_addrs().values():
            for item in addresses:
                if item.family == socket.AF_INET and is_tailscale_address(item.address):
                    return item.address
    except Exception:  # noqa: BLE001 - fall back to the CLI
        pass
    exe = shutil.which("tailscale")
    if exe:
        try:
            out = subprocess.run([exe, "ip", "-4"], capture_output=True, text=True, timeout=3,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            for line in out.stdout.split():
                if is_tailscale_address(line.strip()):
                    return line.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    return None


def enable_keepalive(sock) -> None:
    """Detect a silently dead remote peer within seconds (not hours)."""
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "SIO_KEEPALIVE_VALS"):
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, KEEPALIVE_IDLE_MS, KEEPALIVE_INTERVAL_MS))
        else:
            for name, value in (("TCP_KEEPIDLE", KEEPALIVE_IDLE_MS // 1000),
                                ("TCP_KEEPINTVL", KEEPALIVE_INTERVAL_MS // 1000), ("TCP_KEEPCNT", 5)):
                if hasattr(socket, name):
                    sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), max(1, value))
    except OSError:
        pass


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
    def __init__(self, host=WITNESS_FEED_HOST, port=WITNESS_FEED_PORT, *, remote=False,
                 max_pending=None, max_clients=8):
        if remote:
            if not is_tailscale_address(host):
                raise ValueError(f"The remote Witness feed binds to a Tailscale address only, not {host!r}")
        elif host != "127.0.0.1":
            raise ValueError("The Witness feed must bind to 127.0.0.1")
        self.host, self.port = host, port
        self.remote = remote
        self._max_pending = max_pending
        self.max_clients = max_clients
        self.clients = 0
        self.disconnected_slow = 0
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

    @property
    def max_pending(self):
        """Per-client buffer limit (read live, so the module setting applies)."""
        if self._max_pending:
            return self._max_pending
        return REMOTE_MAX_PENDING_BYTES if self.remote else MAX_PENDING_BYTES

    @max_pending.setter
    def max_pending(self, value):
        self._max_pending = value

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Witness feed already started")
        family = socket.AF_INET6 if ":" in str(self.host) else socket.AF_INET
        listener = socket.socket(family, socket.SOCK_STREAM)
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
            self.clients = len(clients)
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
                        if len(clients) >= self.max_clients:
                            connection.close()
                            continue
                        if self.remote:
                            enable_keepalive(connection)
                        with self._lock:
                            latest = sorted(self._latest.values(), key=lambda item: item["sequence"])
                            snapshot = b"".join(encode_record(dict(item, replay=True)) for item in latest)
                            sequence = self.sequence
                        if len(snapshot) > self.max_pending:
                            connection.close()
                        else:
                            clients[connection] = (bytearray(snapshot), sequence)
                            self.clients = len(clients)
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
                        if len(pending) > self.max_pending:
                            self.disconnected_slow += 1
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
