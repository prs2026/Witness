"""Receive-only AIM BASE MQTT telemetry, independent of Tk and Witness framing.

AIM BASE 4.3.0's publisher uses <base>/<payload_name>/v1. Its JSON keys and
topic names below were checked against the installed AIM BASE executable.
The GUI's illustrative 'gnss/v1' text differs from the actual 'gnss_payload/v1'
publisher. Subscribe to the configured base subtree to also capture new topics.
Values are preserved as published; no unverified unit conversions are applied.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import datetime as dt
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid


AIM_TOPIC_FIELDS = {
    "gnss_payload/v1": (
        "timestamp", "gnss_coordinate.latitude", "gnss_coordinate.longitude",
        "gnss_coordinate.altitude", "satellites_used",
    ),
    "z_acceleration_payload/v1": ("timestamp", "z_acceleration"),
    "pressure_payload/v1": ("timestamp", "pressure"),
    "orientation_payload/v1": ("timestamp", "x", "y", "z", "w"),
    "rssi_payload/v1": ("timestamp", "rssi"),
    "computer_battery_payload/v1": ("timestamp", "voltage"),
    "ejection_battery_payload/v1": ("timestamp", "voltage"),
    "temperature_payload/v1": ("timestamp", "temperature"),
    "received_packet_signal_quality_payload/v1": ("timestamp", "rssi", "snr"),
}
MAX_PAYLOAD_BYTES = 1024 * 1024
MAX_FIELDS = 256


@dataclass(frozen=True)
class AimSettings:
    host: str
    port: int
    client_uuid: str
    base_path: str

    def __post_init__(self):
        if not self.host or self.host != self.host.strip():
            raise ValueError("Enter an MQTT broker hostname.")
        if not 1 <= self.port <= 65535:
            raise ValueError("MQTT port must be between 1 and 65535.")
        if not self.client_uuid or "\x00" in self.client_uuid:
            raise ValueError("Enter a subscriber UUID.")
        if (not self.base_path or self.base_path.endswith("/")
                or any(char in self.base_path for char in "#+\x00")):
            raise ValueError("Enter a base topic path without wildcards or a trailing slash.")

    @property
    def subscription(self):
        return self.base_path + "/#"


def relative_topic(topic: str, base_path: str) -> str:
    if topic == base_path:
        return "(base)"
    if not topic.startswith(base_path + "/"):
        raise ValueError("Message is outside the configured AIM base topic.")
    return topic[len(base_path) + 1:]


def decode_payload(payload: bytes) -> dict:
    """Flatten JSON objects/arrays into named fields, preserving every leaf.

    Unknown topic schemas remain readable. Known schemas are prepopulated by
    the UI, but are not used as an allowlist that could hide additional fields.
    """
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("MQTT payload exceeds the 1 MiB display limit.")
    try:
        value = json.loads(payload.decode("utf-8-sig"))
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ValueError(f"Expected UTF-8 JSON: {exc}") from exc
    fields = {}

    def visit(item, path, depth):
        if depth > 16:
            raise ValueError("MQTT JSON nesting exceeds 16 levels.")
        if isinstance(item, dict) and item:
            for key, child in item.items():
                # Escape path separators to avoid collisions with nested keys.
                key = key.replace("\\", "\\\\").replace(".", "\\.")
                key = key.replace("[", "\\[").replace("]", "\\]")
                visit(child, f"{path}.{key}" if path else key, depth + 1)
        elif isinstance(item, list) and item:
            for index, child in enumerate(item):
                visit(child, f"{path}[{index}]", depth + 1)
        else:
            if isinstance(item, float) and not math.isfinite(item):
                raise ValueError("MQTT JSON contains a non-finite number.")
            if len(fields) >= MAX_FIELDS:
                raise ValueError(f"MQTT JSON exceeds {MAX_FIELDS} displayed fields.")
            fields[path or "(value)"] = item

    visit(value, "", 0)
    return fields


class AimMqttReceiver:
    """Background Paho subscriber with raw logging and bounded GUI updates.

    There is deliberately no publish method, will message, or command topic.
    MQTT connection, subscription, acknowledgements and keepalives are the only
    outbound traffic. Disk logging precedes UI coalescing, so rapid updates do
    not cause received payloads to be omitted from the log.
    """

    def __init__(self, settings: AimSettings, log_dir: Path, *, on_sample=None):
        self.settings = settings
        self.log_dir = Path(log_dir)
        self.client = None
        self.client_id = f"horizon-aim-{settings.client_uuid}-{uuid.uuid4().hex[:8]}"
        self.log_path = None
        self._log = None
        self._log_lock = threading.Lock()
        self._lock = threading.Lock()
        self._latest = {}
        self._diagnostics = queue.Queue(maxsize=200)
        self._stopped = threading.Event()
        self._closed = threading.Event()
        self._closed.set()
        self.status = "Disconnected"
        self.messages = self.bytes_received = self.errors = 0
        self.last_received = None
        self._last_status = None
        self.on_sample = on_sample

    def start(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError(
                "AIM MQTT requires paho-mqtt 2.x. Install the project dependencies "
                "with: python -m pip install -r requirements.txt"
            ) from exc
        if not hasattr(mqtt, "CallbackAPIVersion"):
            raise RuntimeError("Upgrade paho-mqtt to 2.x: python -m pip install 'paho-mqtt>=2.1,<3'")
        if self.client is not None:
            raise RuntimeError("This MQTT receiver has already been started.")
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id, clean_session=True, protocol=mqtt.MQTTv311,
        )
        self.client.connect_timeout = 3.0
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        self.client.on_connect = self._on_connect
        self.client.on_connect_fail = self._on_connect_fail
        self.client.on_disconnect = self._on_disconnect
        self.client.on_subscribe = self._on_subscribe
        self.client.on_message = self._on_message
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.log_path = self.log_dir / f"aim_mqtt_{stamp}.jsonl"
            self._log = self.log_path.open("x", encoding="utf-8")
            self._closed.clear()
            self._set_status(f"Connecting to {self.settings.host}:{self.settings.port}...")
            self.client.connect_async(self.settings.host, self.settings.port, keepalive=30)
            result = self.client.loop_start()
            if result != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"Could not start MQTT receiver: {result}")
        except Exception:
            self.stop(wait=True)
            raise

    def _write_log(self, record):
        try:
            with self._log_lock:
                if self._log is not None:
                    self._log.write(json.dumps(record, ensure_ascii=True, allow_nan=False) + "\n")
                    self._log.flush()
        except (OSError, ValueError) as exc:
            self._diagnostic(f"LOG_ERROR: {exc}")
            # Close once on failure, instead of repeatedly flooding the UI.
            with self._log_lock:
                if self._log is not None:
                    try:
                        self._log.close()
                    except OSError:
                        pass
                    self._log = None

    def _diagnostic(self, text):
        line = f"[{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {text}"
        try:
            self._diagnostics.put_nowait(line)
        except queue.Full:
            pass  # Full history stays in the JSONL log; keep GUI memory bounded.

    def _set_status(self, text):
        with self._lock:
            self.status = text
        if text != self._last_status:
            self._last_status = text
            self._write_log({"event": "status", "received_at": self._utc_now(), "message": text})
            self._diagnostic(text)

    @staticmethod
    def _utc_now():
        return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        if self._stopped.is_set():
            return
        if reason_code.is_failure:
            self._set_status(f"MQTT connection rejected: {reason_code}")
            return
        result, _mid = client.subscribe(self.settings.subscription, qos=0)
        self._set_status(
            f"Subscribing to {self.settings.subscription}..." if result == 0
            else f"MQTT subscription failed: {result}"
        )

    def _on_subscribe(self, client, _userdata, _mid, reason_codes, _properties):
        if self._stopped.is_set():
            return
        if any(code.is_failure for code in reason_codes):
            self._set_status("Broker rejected the AIM topic subscription.")
        else:
            self._set_status(f"Subscribed to {self.settings.subscription}; waiting for telemetry")

    def _on_connect_fail(self, client, _userdata):
        if not self._stopped.is_set():
            self._set_status(
                f"Broker unavailable at {self.settings.host}:{self.settings.port}; retrying. "
                "AIM BASE publishes to a broker; the broker must be running."
            )

    def _on_disconnect(self, client, _userdata, _flags, reason_code, _properties):
        if not self._stopped.is_set():
            self._set_status(f"MQTT disconnected ({reason_code}); reconnecting...")

    def _on_message(self, client, _userdata, message):
        if self._stopped.is_set():
            return
        received_at = self._utc_now()
        received_mono = time.monotonic()
        payload = bytes(message.payload)
        record = {
            "event": "message", "received_at": received_at, "topic": message.topic,
            "qos": message.qos, "retained": bool(message.retain),
            "payload_base64": base64.b64encode(payload).decode("ascii"),
        }
        try:
            topic = relative_topic(message.topic, self.settings.base_path)
            fields = decode_payload(payload)
            record["fields"] = fields
        except (ValueError, TypeError, RecursionError) as exc:
            record["error"] = str(exc)
        self._write_log(record)
        if "error" not in record and self.on_sample is not None:
            try:
                source = f"{self.settings.host}:{self.settings.port}/{self.settings.base_path}"
                self.on_sample(source, topic, fields, received_mono, bool(message.retain))
            except Exception as exc:
                # A visualization error must never stop MQTT reception/logging.
                self._diagnostic(f"FLIGHT_VIEW_ERROR: {exc}")
                self._write_log({"event": "display_error", "received_at": received_at, "error": str(exc)})
        with self._lock:
            self.messages += 1
            self.bytes_received += len(payload)
            if "error" in record:
                self.errors += 1
            else:
                # Coalesce each FIELD so partial messages keep all latest values.
                if topic not in self._latest and len(self._latest) >= 256:
                    self._latest.pop(next(iter(self._latest)))
                pending = self._latest.setdefault(topic, {})
                for key, value in fields.items():
                    if key not in pending and len(pending) >= MAX_FIELDS:
                        pending.pop(next(iter(pending)))
                    pending[key] = (value, received_at, received_mono, bool(message.retain))
                self.last_received = received_mono
                self.status = f"Receiving AIM telemetry from {self.settings.host}:{self.settings.port}"
        if "error" in record:
            self._diagnostic(f"BAD_MESSAGE[{message.topic}] {record['error']}")
        else:
            preview = payload.decode("utf-8-sig")[:600]
            self._diagnostic(
                f"RX[{message.topic}]{' [retained]' if message.retain else ''} "
                + preview.replace("\n", " ").replace("\r", " ")
            )

    def drain(self):
        """Transfer pending latest fields and status to the UI without blocking."""
        with self._lock:
            updates, self._latest = self._latest, {}
            stats = (self.status, self.messages, self.bytes_received, self.errors)
        diagnostics = []
        for _ in range(200):
            try:
                diagnostics.append(self._diagnostics.get_nowait())
            except queue.Empty:
                break
        return updates, stats, diagnostics

    def stop(self, wait=False):
        """Disconnect independently; keep network-thread joins off the Tk thread."""
        if not self._stopped.is_set():
            self._stopped.set()
            if self.client is not None:
                self.client.disconnect()

            def finish():
                try:
                    if self.client is not None:
                        self.client.loop_stop()
                    self._set_status("Disconnected")
                finally:
                    try:
                        with self._log_lock:
                            if self._log is not None:
                                self._log.close()
                                self._log = None
                    finally:
                        self._closed.set()

            threading.Thread(target=finish, name="aim-mqtt-close", daemon=True).start()
        if wait:
            self._closed.wait(timeout=5)
