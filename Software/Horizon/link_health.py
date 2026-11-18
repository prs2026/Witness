"""Link-health helpers shared by Horizon's live inputs (no GUI, no hardware).

Why this exists (audit 2026-11-15, 11_horizon_data.md): a link blip must heal
by itself, the raw log must survive everything, the debug log must be bounded,
and the operator must be told when data stops. Pieces:

- backoff_delay(): reconnect schedule 0.5, 1, 2, 4, 5, 5 ... s, forever.
- RawLog: append-only raw byte log. write() never raises; a failure is counted,
  shown as a warning, and the file is re-opened periodically. fsync() runs from
  a separate thread so the reader never waits for the disk.
- DebugLog: size-capped, rotating text log; every operation guarded.
- LinkStatus: per-source state (connected / reconnecting / stale), seconds since
  the last byte, bytes/s, reconnect count, bad-byte count.
- Coalescer: collapses a burst of decoder messages into one line per interval.
- BoundedEvents: queue.Queue that drops the OLDEST display event when full.
- RateLimited: report a repeating exception at most once per interval.
- tcp_send_all / SendLock helpers: a send completes or raises, never leaves
  half a frame without the caller dropping the connection.
"""

from __future__ import annotations

import os
import queue
import socket
import threading
import time
from collections import deque
from pathlib import Path

BACKOFF_STEPS = (0.5, 1.0, 2.0, 4.0, 5.0)     # capped at 5 s, retried forever
LINK_LOST_AFTER_S = 3.0           # banner: a link that delivered data is silent this long
NO_DATA_SINCE_CONNECT_S = 5.0     # banner: connected but never any byte
TCP_NO_DATA_RECONNECT_S = 5.0     # half-open watchdog: force a TCP reconnect
SEND_TIMEOUT_S = 1.0              # a command write completes or fails within this
RAW_REOPEN_S = 2.0                # retry a failed raw-log open/write this often
RAW_FSYNC_S = 5.0
DEBUG_LOG_MAX_BYTES = 50 * 1024 * 1024
DEBUG_LOG_FILES = 3               # current + 2 rotated (.1, .2)


def open_serial_quiet(port, baud, *, timeout=0.2, write_timeout=2.0, serial_module=None):
    """Open a serial port with DTR and RTS de-asserted BEFORE the open, so opening
    (or re-opening after a blip) does not reset an ESP32 board wired to them."""
    if serial_module is None:
        import serial as serial_module
    ser = serial_module.Serial()
    ser.port, ser.baudrate = port, baud
    ser.timeout, ser.write_timeout = timeout, write_timeout
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def backoff_delay(attempt: int) -> float:
    """Delay before reconnect attempt number `attempt` (1-based)."""
    return BACKOFF_STEPS[min(max(attempt, 1), len(BACKOFF_STEPS)) - 1]


class RawLog:
    """Raw byte log that outlives every error. write() never raises."""

    def __init__(self, path, *, opener=None, clock=time.monotonic):
        self.path = Path(path)
        self._opener = opener or self._default_open
        self._clock = clock
        self._lock = threading.Lock()
        self._fh = None
        self._closed = False
        self._dirty = False
        self._next_retry = 0.0
        self.errors = 0
        self.last_error = None
        self.bytes_written = 0
        self.bytes_lost = 0
        self.reopens = 0
        self._open_locked()

    def _default_open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        return self.path.open("ab")

    def _open_locked(self):
        try:
            self._fh = self._opener()
            return True
        except Exception as exc:  # noqa: BLE001 - logging must not kill the reader
            self._fail(exc)
            return False

    def _fail(self, exc):
        self.errors += 1
        self.last_error = str(exc)
        self._next_retry = self._clock() + RAW_REOPEN_S
        fh, self._fh = self._fh, None
        if fh is not None:
            try:
                fh.close()
            except Exception:  # noqa: BLE001
                pass

    @property
    def failing(self):
        return self._fh is None and not self._closed

    @property
    def warning(self):
        if not self.failing:
            return ""
        return (f"RAW LOG NOT WRITING ({self.path.name}): {self.last_error}; "
                f"{self.bytes_lost} byte(s) not saved, retrying")

    def write(self, data) -> bool:
        with self._lock:
            if self._closed:
                return False
            if self._fh is None:
                if self._clock() < self._next_retry or not self._open_locked():
                    self.bytes_lost += len(data)
                    return False
                self.reopens += 1
            try:
                self._fh.write(data)
                self._fh.flush()          # at least once per chunk: well inside 1 s
            except Exception as exc:  # noqa: BLE001
                self._fail(exc)
                self.bytes_lost += len(data)
                return False
            self.bytes_written += len(data)
            self._dirty = True
            return True

    def tick(self):
        """Retry a failed log without waiting for data (call from idle loops)."""
        with self._lock:
            if self._fh is None and not self._closed and self._clock() >= self._next_retry:
                if self._open_locked():
                    self.reopens += 1

    def fsync(self):
        """Force the OS to commit the file; called from the sync thread, not the reader."""
        with self._lock:
            if self._fh is None or not self._dirty:
                return
            try:
                self._fh.flush()
                fd = self._fh.fileno()
                self._dirty = False
            except Exception:  # noqa: BLE001
                return
        try:
            os.fsync(fd)
        except Exception:  # noqa: BLE001 - closed meanwhile, or a fake file in tests
            pass

    def close(self):
        with self._lock:
            self._closed = True
            fh, self._fh = self._fh, None
        if fh is not None:
            try:
                fh.flush()
                fh.close()
            except Exception:  # noqa: BLE001
                pass


class DebugLog:
    """Rotating debug text log (file-like write/flush/close). Never raises."""

    def __init__(self, path, max_bytes=DEBUG_LOG_MAX_BYTES, files=DEBUG_LOG_FILES):
        self.path = Path(path)
        self.max_bytes = max_bytes
        self.files = max(1, files)
        self.errors = 0
        self.last_error = None
        self.rotations = 0
        self._fh = None
        self._size = 0
        self._closed = False
        self._open()

    def _open(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = self.path.open("a", encoding="utf-8")
            self._size = self.path.stat().st_size
        except Exception as exc:  # noqa: BLE001
            self._fh = None
            self.errors += 1
            self.last_error = str(exc)

    def _rotate(self):
        try:
            if self._fh is not None:
                self._fh.close()
        except Exception:  # noqa: BLE001
            pass
        self._fh = None
        try:
            for index in range(self.files - 1, 0, -1):
                older = self.path.with_name(f"{self.path.name}.{index}")
                newer = self.path if index == 1 else self.path.with_name(f"{self.path.name}.{index - 1}")
                if newer.exists():
                    os.replace(newer, older)
            self.rotations += 1
        except Exception as exc:  # noqa: BLE001
            self.errors += 1
            self.last_error = str(exc)
        self._open()

    @property
    def warning(self):
        return f"DEBUG LOG ERROR: {self.last_error}" if self.errors and self._fh is None else ""

    def write(self, text):
        if self._closed:
            return
        if self._fh is None:
            self._open()
            if self._fh is None:
                return
        try:
            self._fh.write(text)
            self._size += len(text)      # characters ~ bytes; the cap is approximate
            if self._size >= self.max_bytes:
                self._rotate()
        except Exception as exc:  # noqa: BLE001 - disk full etc: drop debug text, keep running
            self.errors += 1
            self.last_error = str(exc)
            try:
                self._fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._fh = None

    def flush(self):
        try:
            if self._fh is not None:
                self._fh.flush()
        except Exception as exc:  # noqa: BLE001
            self.errors += 1
            self.last_error = str(exc)

    def close(self):
        self._closed = True
        try:
            if self._fh is not None:
                self._fh.close()
        except Exception:  # noqa: BLE001
            pass
        self._fh = None


class LinkStatus:
    """Per-source link state, written by the reader thread, read by the GUI."""

    def __init__(self, name, kind, clock=time.monotonic):
        self.name, self.kind, self._clock = name, kind, clock
        self.label = name                   # operator-facing name (e.g. "TCP host:port")
        self._lock = threading.Lock()
        self.state = "connecting"           # connecting | connected | reconnecting | stopped
        self.attempt = 0                    # reconnect attempt in progress (0 when connected)
        self.reconnects = 0                 # successful re-connections since the first one
        self.bytes_total = 0
        self.bad_bytes = 0                  # bytes the decoder skipped as non-packet data
        self.text_bytes = 0
        self.ever_received = False
        self.conn_received = False          # this connection delivered data (watchdog arms)
        self.last_byte = None
        self.connected_at = None
        self.last_error = ""
        self._samples = deque(maxlen=64)    # (time, cumulative bytes) for bytes/s

    def connected(self):
        with self._lock:
            if self.connected_at is not None:
                self.reconnects += 1
            self.state, self.attempt = "connected", 0
            self.connected_at = self._clock()
            self.conn_received = False
            self.last_error = ""

    def lost(self, reason, attempt):
        with self._lock:
            self.state, self.attempt, self.last_error = "reconnecting", attempt, str(reason)

    def note_bytes(self, count):
        now = self._clock()
        with self._lock:
            self.bytes_total += count
            self.last_byte = now
            self.ever_received = self.conn_received = True
            self._samples.append((now, self.bytes_total))

    def silent_for(self, now=None):
        """Seconds without a byte (since connect if none yet); None when never connected."""
        now = self._clock() if now is None else now
        with self._lock:
            ref = self.last_byte if self.last_byte is not None else self.connected_at
            return None if ref is None else max(0.0, now - ref)

    def rate(self, now=None, window=5.0):
        now = self._clock() if now is None else now
        with self._lock:
            pts = [p for p in self._samples if now - p[0] <= window]
            if len(pts) < 2:
                return 0.0
            span = pts[-1][0] - pts[0][0]
            return (pts[-1][1] - pts[0][1]) / span if span > 0 else 0.0

    def snapshot(self, now=None):
        now = self._clock() if now is None else now
        silent = self.silent_for(now)
        with self._lock:
            state, attempt, ever = self.state, self.attempt, self.ever_received
        if state == "connected":
            if ever and silent is not None and silent > LINK_LOST_AFTER_S:
                shown = "stale"
            elif not ever and silent is not None and silent > NO_DATA_SINCE_CONNECT_S:
                shown = "no data"
            else:
                shown = "connected"
        else:
            shown = state
        if shown == "reconnecting":
            text = f"Reconnecting to {self.label} (attempt {attempt})"
        elif shown == "stale":
            text = f"{self.label}: no data for {silent:.1f} s"
        elif shown == "no data":
            text = f"{self.label}: connected, no data yet ({silent:.0f} s)"
        else:
            text = f"{self.label}: {shown}"
        return {"name": self.name, "kind": self.kind, "state": shown, "text": text,
                "attempt": attempt, "reconnects": self.reconnects, "seconds_since_last_byte": silent,
                "bytes_per_s": self.rate(now), "bytes_total": self.bytes_total,
                "bad_bytes": self.bad_bytes, "text_bytes": self.text_bytes,
                "last_error": self.last_error}


class Coalescer:
    """Collapse bursts of messages into one summary per `interval` seconds."""

    def __init__(self, interval=1.0, clock=time.monotonic):
        self.interval, self._clock = interval, clock
        self._count = 0
        self._first = ""
        self._since = None
        self._last_emit = -1e9

    def add(self, message):
        """Record one message; returns a summary string when one is due, else None."""
        if self._count == 0:
            self._first, self._since = str(message), self._clock()
        self._count += 1
        return self.flush_due()

    def flush_due(self, force=False):
        if not self._count:
            return None
        now = self._clock()
        if not force and now - self._last_emit < self.interval:
            return None
        count, first = self._count, self._first
        span = max(now - (self._since or now), 0.0)
        self._count, self._first, self._last_emit = 0, "", now
        if count == 1:
            return first
        return f"{count} events in {max(span, 0.0):.1f} s, first: {first}"


class BoundedEvents(queue.Queue):
    """Unbounded-looking queue with a cap: when full, the oldest DISPLAY event is
    dropped. Link/error events are kept. Raw bytes for the log never pass here."""

    PROTECTED = frozenset(("error", "link_up", "link_down", "link_note"))

    def __init__(self, limit=20000):
        super().__init__()
        self.limit = limit
        self.dropped = 0

    def put(self, item, block=True, timeout=None):
        with self.mutex:
            if len(self.queue) >= self.limit:
                for index, old in enumerate(self.queue):
                    if old[0] not in self.PROTECTED:
                        del self.queue[index]
                        self.dropped += 1
                        break
                else:
                    self.dropped += 1
                    return
            self.queue.append(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()


class RateLimited:
    """Let a repeating failure through at most once per `interval` seconds."""

    def __init__(self, interval=5.0, clock=time.monotonic):
        self.interval, self._clock = interval, clock
        self._last = {}
        self.suppressed = {}

    def allow(self, key):
        now = self._clock()
        if now - self._last.get(key, -1e9) >= self.interval:
            self._last[key] = now
            skipped = self.suppressed.pop(key, 0)
            return True, skipped
        self.suppressed[key] = self.suppressed.get(key, 0) + 1
        return False, 0


def tcp_send_all(sock, data, timeout=SEND_TIMEOUT_S):
    """Send every byte or raise TimeoutError/OSError. The caller holds the link's
    send lock and MUST drop the connection on failure (a partial frame is on the wire)."""
    deadline = time.monotonic() + timeout
    view = memoryview(data)
    sent = 0
    while sent < len(view):
        try:
            count = sock.send(view[sent:])
        except socket.timeout:
            count = 0
        if count == 0 and time.monotonic() >= deadline:
            raise TimeoutError(f"send timed out after {sent}/{len(view)} byte(s)")
        if count == 0:
            time.sleep(0.01)
        sent += count


class AnyEvent:
    """is_set() is true when any of the events is set (cancel for command retries)."""

    def __init__(self, *events):
        self.events = events

    def is_set(self):
        return any(event.is_set() for event in self.events)
