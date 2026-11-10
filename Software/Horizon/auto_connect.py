"""Receiver-tab connection logic shared by Horizon's input tabs (no GUI).

AutoConnectLogic: the Auto-connect checkbox state and retry policy. Turning it
off keeps a healthy connection but stops future retries; Disconnect turns it
off. ReceiverControl adds connect / disconnect / periodic draining of a
ground_receivers receiver; the Qt tab subclasses it and implements the view
hooks (show_records, show_lines, set_inputs_enabled, alert). Network and
serial I/O stay in the receivers' worker threads.
"""

from __future__ import annotations

import time

from observable import Cell
from serial_discovery import RETRY_SECONDS


class AutoConnectLogic:
    receiver = None

    def init_auto_connect(self, enabled=False):
        self.auto_connect = Cell(bool(enabled))
        self._next_connect = 0

    def set_auto_connect(self, enabled):
        self.auto_connect.set(bool(enabled))
        self.auto_changed()

    def auto_changed(self):
        self._next_connect = 0
        if self.receiver is not None:
            self.receiver.auto_retry = self.auto_connect.get()

    def auto_tick(self):
        if self.receiver is not None and not self.receiver.running:
            # Preserve the final diagnostic and displayed telemetry after loss.
            status = self.receiver.status
            self.disconnect()
            self.status.set(status)
            self._next_connect = time.monotonic() + RETRY_SECONDS
        if not self.auto_connect.get() or self.receiver is not None:
            return
        now = time.monotonic()
        if now < self._next_connect:
            return
        self._next_connect = now + RETRY_SECONDS
        if self.prepare_auto_connect():
            self.connect(automatic=True)

    def prepare_auto_connect(self):
        return True


class ReceiverControl(AutoConnectLogic):
    """Connection lifecycle of one receive-only input."""

    stats_format = "Packets: {messages}   Bytes: {bytes}   Errors: {errors}"
    connect_errors = (ValueError, RuntimeError, OSError)

    def init_receiver_control(self):
        self.receiver = None
        self.init_auto_connect(False)
        self.status = Cell("Disconnected")
        self.stats = Cell(self.stats_format.format(messages=0, bytes=0, errors=0))
        self.log_status = Cell("")
        self.connect_text = Cell("Connect")

    # Hooks for the view.
    def make_receiver(self):
        raise NotImplementedError

    def prepare_view(self):
        pass

    def reset_view(self):
        pass

    def show_records(self, updates, diagnostics):
        pass

    def show_lines(self, lines):
        pass

    def set_inputs_enabled(self, enabled):
        pass

    def alert(self, title, text):
        pass

    def remember_source(self, record):
        pass

    def connecting_status(self, receiver):
        return "Connecting..."

    # Logic.
    def toggle_connection(self):
        if self.receiver is not None:
            self.set_auto_connect(False)
            self.disconnect()
        else:
            self.connect()

    def connect(self, *, automatic=False):
        if self.receiver is not None:
            return False
        try:
            receiver = self.make_receiver()
            receiver.auto_retry = self.auto_connect.get()
            receiver.start()
        except self.connect_errors as exc:
            self.status.set(f"Connection failed: {exc}")
            if not automatic:
                self.alert("Input connection", str(exc))
            return False
        self.receiver = receiver
        if not automatic:
            self.reset_view()
        self.prepare_view()
        self.status.set(self.connecting_status(receiver))
        self.log_status.set(f"Log: {receiver.log_path}")
        self.set_inputs_enabled(False)
        self.connect_text.set("Disconnect")
        return True

    def consume(self, receiver):
        updates, stats, diagnostics = receiver.drain()
        status, messages, byte_count, errors = stats
        self.status.set(status)
        self.stats.set(self.stats_format.format(messages=messages, bytes=byte_count, errors=errors))
        self.show_records(updates, diagnostics)

    _consume = consume

    def poll(self):
        """Periodic step (the view calls it every 100 ms)."""
        if self.receiver is not None:
            self.consume(self.receiver)
        self.auto_tick()

    def stop_receiver(self, receiver):
        # Wait for the old reader to release its COM port/log files before
        # allowing another connection (serial reads time out after 0.2 s).
        receiver.stop(wait=True)

    def disconnect(self):
        receiver, self.receiver = self.receiver, None
        if receiver is not None:
            self.consume(receiver)
            self.stop_receiver(receiver)
        self.status.set("Disconnected - latest values retained")
        self.connect_text.set("Connect")
        self.set_inputs_enabled(True)


# Tk-era name for the mixin.
AutoConnectTab = AutoConnectLogic
