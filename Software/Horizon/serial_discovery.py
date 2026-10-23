"""Conservative USB discovery: named devices or explicitly learned identities.

Never probe arbitrary serial ports. A generic USB adapter must be selected once
manually; a valid received packet then enrolls its USB identity. Serial numbers
survive COM renumbering; adapters without one are tied to their USB location.
Enumeration and automatic opens run off the Tk thread. No probe bytes are sent.
"""

import json
from pathlib import Path
import threading
import time


ROOT = Path(__file__).resolve().parent
RETRY_SECONDS = 5.0


def enumerate_ports():
    try:
        from serial.tools import list_ports
        return list(list_ports.comports())
    except ImportError:
        return []


def identity(port):
    vid, pid = getattr(port, "vid", None), getattr(port, "pid", None)
    if vid is None or pid is None:
        return None
    for key in ("serial_number", "location"):
        value = getattr(port, key, None)
        if value:
            return {"vid": vid, "pid": pid, key: value}
    return None


class SerialDiscovery:
    def __init__(self, role, *, path=None):
        self.role = role
        self.path = Path(path) if path is not None else ROOT / f"auto_connect_{role}.json"
        self.known = []
        self.ports = []
        self.error = None
        self._thread = None
        self._next_scan = 0
        self._attempts = {}
        self._lock = threading.Lock()
        self._result = None
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, list) or not all(isinstance(item, dict) for item in data):
                raise ValueError("expected a list of USB identities")
            self.known = data
        except FileNotFoundError:
            pass
        except (OSError, ValueError) as exc:
            self.error = f"Could not load {self.path.name}: {exc}"

    def refresh(self, *, force=False):
        with self._lock:
            if self._result is not None:
                self.ports, error = self._result
                self._result = None
                if error:
                    self.error = error
        now = time.monotonic()
        if self._thread is not None and self._thread.is_alive():
            return
        if not force and now < self._next_scan:
            return
        self._next_scan = now + RETRY_SECONDS

        def scan():
            try:
                result = (enumerate_ports(), None)
            except Exception as exc:
                result = ([], f"Serial discovery failed: {exc}")
            with self._lock:
                self._result = result

        self._thread = threading.Thread(target=scan, daemon=True, name=f"discover-{self.role}")
        self._thread.start()

    def candidates(self, *, exclude=()):
        found = []
        for port in self.ports:
            description = " ".join(str(getattr(port, key, "") or "") for key in
                                   ("description", "manufacturer", "product")).lower()
            signature = identity(port)
            if port.device not in exclude and (self.role in description or
                                               (signature and signature in self.known)):
                found.append(port.device)
        return sorted(found, key=lambda device: (self._attempts.get(device, 0), device))

    def attempted(self, device):
        self._attempts[device] = time.monotonic()

    def remember(self, device):
        port = next((p for p in self.ports if p.device == device), None)
        signature = identity(port) if port is not None else None
        if signature is None or signature in self.known:
            return
        self.known.append(signature)
        try:
            # Each app/role has its own file; atomic replacement prevents partial
            # configuration reads after a crash. Never save telemetry here.
            temporary = self.path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(self.known, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self.path)
        except OSError as exc:
            self.error = f"Could not remember {device}: {exc}"


class SerialOpenJob:
    """One cancellable background open; late success is closed, never leaked."""
    def __init__(self, port, baud):
        self.port = port
        self._lock = threading.Lock()
        self._cancelled = False
        self._result = None

        def open_port():
            try:
                from serial import Serial
                result = (Serial(port, baud, timeout=0.2), None)
            except Exception as exc:
                result = (None, str(exc))
            with self._lock:
                if self._cancelled:
                    if result[0] is not None:
                        result[0].close()
                else:
                    self._result = result

        self.thread = threading.Thread(target=open_port, daemon=True, name=f"open-{port}")
        self.thread.start()

    def take(self):
        with self._lock:
            result, self._result = self._result, None
            return result

    def cancel(self):
        with self._lock:
            self._cancelled = True
            if self._result is not None and self._result[0] is not None:
                self._result[0].close()
            self._result = None
