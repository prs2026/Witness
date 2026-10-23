"""Horizon-owned, local-only MQTT 3.1.1 broker on a background asyncio loop.

Uses aMQTT's protocol implementation (QoS, wildcards, retained messages).
No service installation, subprocess, firewall changes or device writes.
Sessions/retained messages are in memory and reset when the broker stops.
"""

import asyncio
from datetime import datetime
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import threading

from ground_settings import DEFAULT_MQTT_BROKER_PORT


class EmbeddedMqttBroker:
    host = "127.0.0.1"

    def __init__(self, log_dir, port=DEFAULT_MQTT_BROKER_PORT):
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("Broker port must be 1..65535")
        self.port = port
        self.log_dir = Path(log_dir)
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status = ("stopped", "Embedded broker off", None)

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    def snapshot(self):
        with self._lock:
            return self._status

    def _set_status(self, state, message, path=None):
        with self._lock:
            self._status = (state, message, path or self._status[2])

    def start(self):
        """Called by the UI thread; never blocks Tk on imports or socket binding."""
        if self.alive:
            return
        self._stop.clear()
        self._set_status("starting", f"Starting embedded broker on {self.host}:{self.port}...")
        self._thread = threading.Thread(target=self._run, name="Horizon MQTT broker", daemon=True)
        self._thread.start()

    def stop(self, *, wait=False):
        self._stop.set()
        if wait and self._thread is not None:
            self._thread.join(timeout=10)

    def _run(self):
        handler = None
        logger = logging.getLogger(f"horizon.mqtt_broker.{id(self)}")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        library_logger = logging.getLogger("amqtt")
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path = self.log_dir / f"mqtt_broker_{datetime.now():%Y%m%d_%H%M%S_%f}.log"
            handler = RotatingFileHandler(path, maxBytes=5_000_000, backupCount=2, encoding="utf-8")
            thread_id = threading.get_ident()
            # Don't capture other broker instances or flood disk with packet dumps.
            handler.addFilter(lambda record: record.thread == thread_id and
                              (record.name == logger.name or record.levelno >= logging.WARNING))
            handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
            logger.addHandler(handler)
            library_logger.addHandler(handler)
            self._set_status("starting", self.snapshot()[1], path)
            asyncio.run(self._serve(logger))
        except ImportError:
            logger.exception("Embedded broker dependency missing")
            self._set_status("failed", "Broker unavailable: install requirements.txt in this Python environment.")
        except Exception as exc:
            logger.exception("Embedded broker failed")
            self._set_status("failed", f"Broker failed on {self.host}:{self.port}: {exc}. "
                             "If the port is occupied, use the existing broker or free the port, then toggle this checkbox to retry.")
        else:
            self._set_status("stopped", "Embedded broker off (local clients disconnected)")
        finally:
            if handler is not None:
                library_logger.removeHandler(handler)
                logger.removeHandler(handler)
                handler.close()

    async def _serve(self, logger):
        from amqtt.broker import Broker

        if self._stop.is_set():
            return
        broker = Broker({
            "listeners": {"default": {"type": "tcp", "bind": f"{self.host}:{self.port}",
                                      "max_connections": 32}},
            "session_expiry_interval": 60,
            "plugins": {"amqtt.plugins.authentication.AnonymousAuthPlugin": {"allow_anonymous": True}},
        })
        await broker.start()
        try:
            logger.info("Listening on %s:%s (local-only, MQTT 3.1.1)", self.host, self.port)
            self._set_status("running", f"Broker running: {self.host}:{self.port} (local-only, MQTT 3.1.1)")
            while not self._stop.is_set():
                await asyncio.sleep(0.1)
        finally:
            self._set_status("stopping", "Stopping embedded broker...")
            await asyncio.wait_for(broker.shutdown(), timeout=8)
            logger.info("Broker stopped; sessions and retained messages cleared")
