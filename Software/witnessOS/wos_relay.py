"""Witness packet relay: one Witness byte stream -> the Horizon JSON feed.

This is the non-GUI half of the witnessOS "Witness relay" tab. It reuses the
Horizon monitor's pieces unchanged:

* ``main.PacketDecoder`` (Horizon/main.py) deframes and decodes the comms.h
  packets, exactly as the monitor's serial/TCP readers do;
* ``witness_link.WitnessPublisher`` serves every decoded packet as
  ``horizon.witness.v1`` newline-delimited JSON on 127.0.0.1 (default port
  8766), the same feed main.py publishes for horizon.py / ground receivers.

Inputs: a raw TCP port (ser2net on the bench Pi, e.g. prs2026hitl:7100), a
USB serial port (needs pyserial), or the HITL tab's existing connection.
Witness's raw TCP port serves one client at a time, so while the HITL bench
is connected the relay mirrors that connection's received bytes instead of
opening a second one. The relay never transmits anything to Witness.
"""

from __future__ import annotations

from collections import deque
import datetime as dt
import os
import queue
import re
import socket
import threading
import time

import wos_paths  # noqa: F401  (adds Horizon/ and HITL/ to sys.path)
from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT
from main import PacketDecoder
from witness_link import WitnessPublisher

LOG_DIR = os.path.join(wos_paths.HORIZON_DIR, "logs")
RATE_WINDOW_S = 2.0
MAX_ERROR_LOGS_PER_S = 10


def source_name_for_tcp(host: str, port: int) -> str:
    """Same naming as main.py, so feed consumers see the same source."""
    return f"TCP_{host}_{port}"


def safe_file_part(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_") or "witness"


def flatten_packet(packet: dict) -> list[tuple[str, str]]:
    """Decoded packet -> (field, display value) rows for a table."""
    rows = []
    for key, value in packet.items():
        if isinstance(value, (list, tuple)):
            rows.append((key, ", ".join(f"{v:.6g}" if isinstance(v, float)
                                        else str(v) for v in value)))
        elif isinstance(value, float):
            rows.append((key, f"{value:.6g}"))
        elif key in ("status", "fc_status") and isinstance(value, int):
            rows.append((key, f"0x{value:04x}"))
        else:
            rows.append((key, str(value)))
    return rows


class PacketStats:
    __slots__ = ("count", "times", "last_time", "latest")

    def __init__(self):
        self.count = 0
        self.times: deque = deque()
        self.last_time = 0.0
        self.latest: dict = {}


class RelayEngine:
    """Decode one input and publish it to the Horizon Witness feed.

    Thread-safe: inputs feed bytes from their reader threads; the GUI reads
    ``snapshot()`` and drains ``events`` (kind, message) on its own thread.
    """

    def __init__(self, feed_host: str = WITNESS_FEED_HOST,
                 feed_port: int = WITNESS_FEED_PORT,
                 log_dir: str | None = LOG_DIR,
                 publisher_factory=WitnessPublisher, clock=time.time):
        self.feed_host = feed_host
        self.feed_port = feed_port
        self.log_dir = log_dir
        self.record_raw = log_dir is not None
        self.publisher_factory = publisher_factory
        self.clock = clock
        self.events: queue.Queue = queue.Queue()
        self.publisher = None
        self.feed_error: str | None = None
        self._lock = threading.RLock()
        self._input: dict | None = None
        self._decoder: PacketDecoder | None = None
        self._log_file = None
        self._error_window = (0.0, 0)
        self.reset_stats()

    # ------------------------------------------------------------- stats
    def reset_stats(self):
        with self._lock:
            self.rx_bytes = 0
            self.decode_errors = 0
            self.host_packets = 0
            self.published = 0
            self.stats: dict[str, PacketStats] = {}

    def _event(self, kind: str, message: str):
        self.events.put((kind, message))

    # -------------------------------------------------------------- feed
    def start_feed(self, port: int | None = None) -> bool:
        """Start serving the JSON feed; False (and feed_error) on failure."""
        self.stop_feed()
        if port is not None:
            self.feed_port = int(port)
        publisher = self.publisher_factory(self.feed_host, self.feed_port)
        try:
            publisher.start()
        except OSError as exc:
            self.feed_error = (f"cannot listen on {self.feed_host}:"
                               f"{self.feed_port}: {exc} (is Horizon main.py "
                               "already serving the feed?)")
            self._event("error", self.feed_error)
            return False
        self.publisher = publisher
        self.feed_error = None
        self._event("log", f"Horizon feed listening on {publisher.host}:"
                           f"{publisher.port} (receive only)")
        return True

    def stop_feed(self):
        publisher, self.publisher = self.publisher, None
        if publisher is not None:
            publisher.close()
            self._event("log", "Horizon feed stopped")

    def feed_status(self) -> dict:
        p = self.publisher
        if p is None:
            return {"running": False, "error": self.feed_error,
                    "port": self.feed_port}
        return {"running": p.error is None, "error": p.error or None,
                "host": p.host, "port": p.port, "sequence": p.sequence,
                "dropped": p.dropped}

    # ------------------------------------------------------------ inputs
    @property
    def connected(self) -> bool:
        inp = self._input
        if inp is None:
            return False
        if inp["kind"] == "hitl":
            return bool(inp["session"].connected)
        return not inp["stop"].is_set()

    @property
    def source(self) -> str | None:
        inp = self._input
        return None if inp is None else inp["source"]

    def _begin(self, kind: str, source: str, **extra) -> dict:
        if self._input is not None:
            raise RuntimeError("relay input already connected")
        with self._lock:
            self._decoder = PacketDecoder(
                lambda pkt: self._on_packet(source, pkt),
                self._on_decode_error,
                self._on_host_packet)
            inp = {"kind": kind, "source": source,
                   "stop": threading.Event(), **extra}
            self._input = inp
        if self.record_raw and self.log_dir:
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
                path = os.path.join(self.log_dir,
                                    f"{safe_file_part(source)}_{stamp}.bin")
                self._log_file = open(path, "wb")
                self._event("log", f"recording raw bytes to {path}")
            except OSError as exc:
                self._event("error", f"raw log disabled: {exc}")
        return inp

    def connect_tcp(self, host: str, port: int, timeout: float = 3.0):
        sock = socket.create_connection((host, int(port)), timeout=timeout)
        sock.settimeout(0.2)
        inp = self._begin("tcp", source_name_for_tcp(host, int(port)),
                          handle=sock)
        threading.Thread(target=self._read_tcp, args=(inp,),
                         name="relay-tcp", daemon=True).start()
        self._event("log", f"connected to {host}:{port}")

    def connect_serial(self, port: str, baud: int = 115200):
        import serial  # pyserial; optional dependency
        # serial_for_url accepts COM3 / /dev/ttyACM0 and pyserial URLs.
        handle = serial.serial_for_url(port, int(baud), timeout=0.2)
        inp = self._begin("serial", port, handle=handle)
        threading.Thread(target=self._read_serial, args=(inp,),
                         name="relay-serial", daemon=True).start()
        self._event("log", f"opened {port} at {baud} baud")

    def share_session(self, session, label: str = "HITL"):
        """Mirror the bytes a HitlSession receives (no second connection)."""
        source = label
        host = getattr(session, "_host", None)
        if host:
            source = f"HITL_{host}"
        inp = self._begin("hitl", source, session=session)
        tap = (lambda data, inp=inp: self.feed_bytes(data, inp))
        inp["tap"] = tap
        session.rx_tap = tap
        self._event("log", "mirroring the HITL tab connection" +
                    ("" if session.connected else
                     " (not connected yet; connect on the HITL tab)"))

    def disconnect(self):
        inp, self._input = self._input, None
        if inp is None:
            return
        inp["stop"].set()
        if inp["kind"] == "hitl":
            session = inp["session"]
            if getattr(session, "rx_tap", None) is inp.get("tap"):
                session.rx_tap = None
        else:
            try:
                inp["handle"].close()
            except Exception:
                pass
        log, self._log_file = self._log_file, None
        if log is not None:
            try:
                log.close()
            except OSError:
                pass
        self._event("log", f"disconnected {inp['source']}")

    def close(self):
        self.disconnect()
        self.stop_feed()

    def _reader_failed(self, inp: dict, message: str):
        if self._input is inp and not inp["stop"].is_set():
            self._event("error", f"{inp['source']}: {message}")
            self.disconnect()

    def _read_tcp(self, inp: dict):
        sock = inp["handle"]
        while not inp["stop"].is_set():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                self._reader_failed(inp, str(exc))
                return
            if not data:
                self._reader_failed(inp, "TCP server closed the connection")
                return
            self.feed_bytes(data, inp)

    def _read_serial(self, inp: dict):
        port = inp["handle"]
        while not inp["stop"].is_set():
            try:
                data = port.read(port.in_waiting or 1)
            except Exception as exc:  # pyserial raises several types
                self._reader_failed(inp, str(exc))
                return
            if data:
                self.feed_bytes(data, inp)

    # ----------------------------------------------------------- decoding
    def feed_bytes(self, data: bytes, inp: dict | None = None):
        """Decode bytes from the current input (any thread)."""
        with self._lock:
            if inp is not None and inp is not self._input:
                return  # a reader that outlived its connection
            decoder = self._decoder
            if decoder is None:
                return
            self.rx_bytes += len(data)
            log = self._log_file
            if log is not None:
                try:
                    log.write(data)
                    log.flush()
                except (OSError, ValueError):
                    pass
            decoder.feed(data)

    def _on_packet(self, source: str, packet: dict):
        now = self.clock()
        publisher = self.publisher
        if publisher is not None:
            try:
                publisher.publish(source, packet, received_at=now)
                self.published += 1
            except Exception as exc:
                self._event("error", f"Horizon feed: {exc}")
        kind = packet.get("packet", "?")
        st = self.stats.get(kind)
        if st is None:
            st = self.stats[kind] = PacketStats()
        st.count += 1
        st.last_time = now
        st.latest = packet
        st.times.append(now)
        while st.times and now - st.times[0] > RATE_WINDOW_S:
            st.times.popleft()

    def _on_decode_error(self, message: str):
        self.decode_errors += 1
        now = self.clock()
        start, count = self._error_window
        if now - start >= 1.0:
            start, count = now, 0
        if count < MAX_ERROR_LOGS_PER_S:
            self._event("decode", message)
        self._error_window = (start, count + 1)

    def _on_host_packet(self, _packet_id: int, count: int):
        self.host_packets = count

    def snapshot(self) -> dict:
        """Copy of the counters for display."""
        now = self.clock()
        with self._lock:
            packets = {}
            for kind, st in self.stats.items():
                recent = [t for t in st.times if now - t <= RATE_WINDOW_S]
                packets[kind] = {
                    "count": st.count,
                    "rate_hz": len(recent) / RATE_WINDOW_S,
                    "age_s": now - st.last_time,
                    "latest": dict(st.latest),
                }
            return {"connected": self.connected, "source": self.source,
                    "rx_bytes": self.rx_bytes,
                    "decode_errors": self.decode_errors,
                    "host_packets": self.host_packets,
                    "published": self.published,
                    "packets": packets, "feed": self.feed_status()}
