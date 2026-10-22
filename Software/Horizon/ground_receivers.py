"""Background read-only inputs for the unified ground station (no Tk calls)."""

from collections import OrderedDict, deque
import datetime as dt
import json
from pathlib import Path
import socket
import threading
import time

from featherweight import FEATHERWEIGHT_BAUD, FeatherweightStreamParser
from witness_link import MAX_LINE_BYTES, decode_record, json_safe


class LoggedReceiver:
    """Log every input record, but coalesce bounded latest-packet GUI updates."""

    def __init__(self, log_dir, prefix, *, on_record=None):
        self.log_dir = Path(log_dir)
        self.prefix = prefix
        self.log_path = self.raw_path = None
        self._log = self._raw = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._latest = OrderedDict()
        self._diagnostics = deque(maxlen=200)
        self.status = "Disconnected"
        self.messages = self.bytes_received = self.errors = 0
        self.on_record = on_record

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Receiver already started")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.log_path = self.log_dir / f"{self.prefix}_{stamp}.jsonl"
        self._log = self.log_path.open("x", encoding="utf-8")
        self._thread = threading.Thread(target=self._run_guarded, daemon=True,
                                        name=self.prefix)
        self._thread.start()

    def _log_event(self, event):
        if self._log is None:
            return
        try:
            self._log.write(json.dumps(json_safe(event), allow_nan=False) + "\n")
            self._log.flush()
        except (OSError, ValueError) as exc:
            with self._lock:
                self.errors += 1
                self._diagnostics.append(f"LOG_ERROR: {exc}")
            try:
                self._log.close()
            except OSError:
                pass
            self._log = None

    def report(self, message, *, error=False, status=False):
        with self._lock:
            if error:
                self.errors += 1
            if status:
                self.status = message
            self._diagnostics.append(message)
        self._log_event({"event": "error" if error else "status",
                         "received_at": time.time(), "message": message})

    def accept(self, record):
        self._log_event({**record, "event": "packet"})
        key = (record["source"], record["packet"]["packet"])
        # Convert original receive age to a monotonic time once. Snapshots must
        # not appear fresh merely because Horizon just connected.
        record = dict(record, received_mono=time.monotonic() - max(
            0, time.time() - record["received_at"]))
        # History consumers receive EVERY sample, before latest-value coalescing.
        # They run on this worker and must not touch Tk or block on rendering.
        if self.on_record is not None:
            try:
                self.on_record(record)
            except Exception as exc:
                self.report(f"FLIGHT_VIEW_ERROR: {exc}", error=True)
        with self._lock:
            self.messages += 1
            self._latest[key] = record
            self._latest.move_to_end(key)
            if len(self._latest) > 128:
                self._latest.popitem(last=False)
            self._diagnostics.append(
                f"{'SNAPSHOT' if record.get('replay') else 'PACKET'}[{record['source']}] "
                + json.dumps(json_safe(record["packet"]), ensure_ascii=True))

    def drain(self):
        with self._lock:
            updates = list(self._latest.values())
            self._latest.clear()
            diagnostics = list(self._diagnostics)
            self._diagnostics.clear()
            stats = (self.status, self.messages, self.bytes_received, self.errors)
        return updates, stats, diagnostics

    def _run_guarded(self):
        try:
            self.run()
        except Exception as exc:
            if not self._stop.is_set():
                self.report(f"Input stopped: {exc}", error=True, status=True)
        finally:
            for stream in (self._raw, self._log):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass
            self._raw = self._log = None

    def stop(self, wait=False):
        self._stop.set()
        if wait and self._thread is not None:
            self._thread.join(timeout=4)


class WitnessReceiver(LoggedReceiver):
    def __init__(self, host, port, log_dir, *, on_record=None):
        super().__init__(log_dir, "horizon_witness", on_record=on_record)
        # Keep the unauthenticated bridge on the local machine only.
        if host not in ("localhost", "127.0.0.1"):
            raise ValueError("Witness feed host must be localhost or 127.0.0.1")
        if not 1 <= port <= 65535:
            raise ValueError("Witness feed port must be between 1 and 65535")
        self.host, self.port = "127.0.0.1", port

    def run(self):
        while not self._stop.is_set():
            try:
                self.report(f"Connecting to Witness at {self.host}:{self.port}...", status=True)
                with socket.create_connection((self.host, self.port), timeout=1) as connection:
                    connection.settimeout(0.2)
                    self.report("Connected to Witness (receive only)", status=True)
                    buffer = bytearray()
                    previous = None
                    while not self._stop.is_set():
                        try:
                            data = connection.recv(16384)
                        except socket.timeout:
                            continue
                        if not data:
                            if buffer:
                                self.report("Discarded incomplete Witness JSON record", error=True)
                            raise ConnectionError("Witness app disconnected")
                        with self._lock:
                            self.bytes_received += len(data)
                        buffer.extend(data)
                        while b"\n" in buffer:
                            line, _, remainder = buffer.partition(b"\n")
                            buffer = bytearray(remainder)
                            try:
                                record = decode_record(line)
                            except ValueError as exc:
                                self.report(f"PARSE_ERROR: {exc}; preview={line[:160]!r}", error=True)
                                continue
                            if not record["replay"]:
                                current = (record["session"], record["sequence"])
                                if previous and previous[0] == current[0] and current[1] > previous[1] + 1:
                                    self.report(f"Witness sequence gap: {previous[1]} -> {current[1]}", error=True)
                                previous = current
                            self.accept(record)
                        if len(buffer) > MAX_LINE_BYTES:
                            raise ValueError("Witness feed record exceeds 64 KiB without a newline")
            except (OSError, ValueError) as exc:
                if not self._stop.is_set():
                    self.report(f"Waiting for Witness: {exc}; retrying in 1s",
                                error=isinstance(exc, ValueError), status=True)
                    self._stop.wait(1)


class FeatherweightReceiver(LoggedReceiver):
    def __init__(self, port, log_dir, *, serial_factory=None, on_record=None):
        super().__init__(log_dir, "horizon_featherweight", on_record=on_record)
        if not port or port == "(no port)":
            raise ValueError("Select a Featherweight serial port")
        if serial_factory is None:
            try:
                from serial import Serial
            except ImportError as exc:
                raise RuntimeError("Featherweight requires pyserial: pip install -r requirements.txt") from exc
            serial_factory = Serial
        self.port, self.serial_factory = port, serial_factory

    def run(self):
        self.raw_path = self.log_path.with_suffix(".bin")
        self._raw = self.raw_path.open("xb")
        self.report(f"Opening {self.port}: {FEATHERWEIGHT_BAUD} baud, 8-N-1, read only", status=True)
        # Deliberately no write(), break, or protocol command calls anywhere.
        with self.serial_factory(port=self.port, baudrate=FEATHERWEIGHT_BAUD,
                                 bytesize=8, parity="N", stopbits=1, timeout=0.2,
                                 xonxoff=False, rtscts=False, dsrdtr=False) as connection:
            self.report(f"Connected: {self.port} (receive only)", status=True)
            parser = FeatherweightStreamParser(
                self._packet, lambda message: self.report(f"PARSE_ERROR: {message}", error=True))
            while not self._stop.is_set():
                data = connection.read(min(connection.in_waiting or 1, 16384))
                if data:
                    with self._lock:
                        self.bytes_received += len(data)
                    self._raw.write(data)
                    self._raw.flush()
                    parser.feed(data)

    def _packet(self, packet):
        if not packet["text_crc_valid"]:
            self.report(f"TEXT_CRC_ERROR: received {packet['text_crc_expected']:04X}, "
                        f"computed {packet['text_crc_computed']:04X}", error=True)
        self.accept({"source": self.port, "received_at": time.time(), "replay": False,
                     "packet": {**packet, "packet": packet["packet_type"]}})
