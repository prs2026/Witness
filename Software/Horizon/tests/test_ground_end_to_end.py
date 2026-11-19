"""Witness-to-Horizon ground path at flight rate with injected loss (no hardware).

A loopback TCP server stands in for the Aperture bridge (or ser2net): it streams
real Witness frames, in random fragments, with dropped frames, corrupted frames
and one blackout longer than the staleness limit. They go through the real
receive path: monitor core (TCP link, decoder, raw log) -> Horizon feed ->
Horizon receiver -> Flight View data. Checks that Horizon stays live, that the
reported age tracks the real silence (stale during the blackout, live again
after it) and that the Qt event loop never stalls.
"""
import os
import random
import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

STATE_FLIGHT = 0x0002          # Coast, status byte 1 bits 2:0
GPS_LOCK = 0x0008
SENSOR_HZ = 100                # 10x the USB rate, so a short test covers many frames
STATE_HZ = 25                  # the Kalman-log rate; USB state is 1 Hz
LIVE_S = 2.0
BLACKOUT_S = 3.6               # longer than STALE_S (3 s)
TAIL_S = 2.0
LOSS = 0.15
CORRUPT = 0.03
MAX_UI_GAP_S = 0.5


def sensors_frame(uptime):
    accel = b"".join((int(1.0 / 0.00004) & 0xFFFFFF).to_bytes(3, "big") for _ in range(3))
    gyro = struct.pack(">hhh", 0, 0, 0)
    return b"\x01" + struct.pack(">HI", 0x0100 | STATE_FLIGHT, uptime) + accel + gyro + b"\x0a"


def state_frame(uptime, altitude):
    payload = struct.pack(">HI", 0x0100 | GPS_LOCK | STATE_FLIGHT, uptime)
    payload += struct.pack(">4f", 0.0, 0.0, 0.0, 1.0) + struct.pack(">3f", 0.0, 0.0, altitude)
    payload += struct.pack(">3f", 0.0, 0.0, 50.0) + bytes(6) + struct.pack(">ff", 600.0 + altitude, altitude)
    payload += struct.pack(">H", 7400) + bytes((9,))
    assert len(payload) == 63, len(payload)
    return b"\x02" + payload + b"\x0a"


class Sender(threading.Thread):
    def __init__(self, connection, seed=3):
        super().__init__(daemon=True)
        self.connection = connection
        self.rng = random.Random(seed)
        self.dropped = self.corrupted = 0
        self.delivered_uptimes = []
        self.phase = "live"
        self.stop = threading.Event()
        self.error = None

    def _emit(self, frame, is_state, uptime):
        if self.rng.random() < LOSS:
            self.dropped += 1
            return b""
        if self.rng.random() < CORRUPT:
            self.corrupted += 1
            damaged = bytearray(frame)
            damaged[self.rng.randrange(1, len(damaged) - 1)] ^= 0xFF
            return bytes(damaged)
        if is_state:
            self.delivered_uptimes.append(uptime)
        return frame

    def _send(self, data):
        # Random fragmentation: 1..40 byte writes, like a radio bridge.
        index = 0
        while index < len(data):
            size = self.rng.randint(1, 40)
            self.connection.sendall(data[index:index + size])
            index += size

    def run(self):
        try:
            t0 = time.monotonic()
            tick = 0
            period = 1.0 / SENSOR_HZ
            while not self.stop.is_set():
                elapsed = time.monotonic() - t0
                if elapsed > LIVE_S + BLACKOUT_S + TAIL_S:
                    break
                blackout = LIVE_S <= elapsed < LIVE_S + BLACKOUT_S
                self.phase = "blackout" if blackout else ("live" if elapsed < LIVE_S else "tail")
                if not blackout:
                    uptime = 100_000 + int(elapsed * 1000)
                    chunk = self._emit(sensors_frame(uptime), False, uptime)
                    if tick % (SENSOR_HZ // STATE_HZ) == 0:
                        chunk += self._emit(state_frame(uptime, 10.0 + elapsed), True, uptime)
                    if chunk:
                        self._send(chunk)
                tick += 1
                delay = t0 + tick * period - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            self.phase = "done"
        except OSError as exc:
            self.error = exc
            self.phase = "done"


class GroundEndToEnd(unittest.TestCase):
    def test_flight_rate_stream_with_loss_stays_live_and_ages_correctly(self):
        from horizon_qt_support import make_horizon, pump
        from monitor_core import MonitorCore
        from PyQt6 import QtCore

        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(10)
        self.addCleanup(listener.close)
        with tempfile.TemporaryDirectory() as directory:
            log_dir = Path(directory)
            with patch("monitor_core.LOG_DIR", log_dir):
                core = MonitorCore(auto_connect=False, feed_port=0)
                core.tcp_hosts[0].set("127.0.0.1")
                core.tcp_ports[0].set(str(listener.getsockname()[1]))
                unified = make_horizon(directory)
                try:
                    self.assertTrue(core.connect())
                    connection, _ = listener.accept()
                    self.addCleanup(connection.close)
                    unified.witness.port.set(str(core.publisher.port))
                    unified.witness.connect()

                    # A 10 ms heartbeat timer measures the longest Qt event-loop stall.
                    gaps, last = [], [time.monotonic()]
                    ticker = QtCore.QTimer()
                    ticker.setInterval(10)

                    def tick():
                        now = time.monotonic()
                        gaps.append(now - last[0])
                        last[0] = now
                    ticker.timeout.connect(tick)
                    ticker.start()

                    sender = Sender(connection)
                    sender.start()
                    flight = unified.flight
                    samples = {"live": [], "blackout": [], "tail": []}
                    while sender.phase != "done":
                        core.process_events()
                        pump(0.02)
                        phase = sender.phase
                        if phase in samples:
                            seen = flight.data.status_snapshot()["essential"].get("witness")
                            age = None if seen is None else time.monotonic() - seen
                            flight.redraw()
                            samples[phase].append((age, flight.banner.text()))
                    sender.join(5)
                    ticker.stop()
                    self.assertIsNone(sender.error)

                    # The injected loss happened and Horizon still received the rest.
                    self.assertGreater(sender.dropped, 5)
                    self.assertGreater(sender.corrupted, 1)
                    self.assertGreater(len(sender.delivered_uptimes), 30)
                    self.assertIsNotNone(core.last_packet_time.get("packet.state"))

                    def ages(phase, skip=0):
                        return [a for a, _t in samples[phase][skip:] if a is not None]

                    # Live: fresh the whole time (a state frame gets through within a few
                    # frames even at 15 % loss) and the banner says LIVE.
                    live = ages("live", 10)
                    self.assertTrue(live)
                    self.assertLess(max(live), 1.0, "live data must show a small age")
                    self.assertTrue(all("LIVE" in text for _a, text in samples["live"][-20:]),
                                    samples["live"][-1])
                    # Blackout: the age grows with the real silence, then the view goes stale.
                    black = ages("blackout")
                    self.assertGreater(max(black), 3.0)
                    self.assertGreater(black[-1], black[0] + 2.0)
                    self.assertTrue(any("STALE" in text for _a, text in samples["blackout"]),
                                    samples["blackout"][-1])
                    # Tail: live again, ages small, banner LIVE.
                    tail = ages("tail", 25)
                    self.assertLess(max(tail), 1.0)
                    self.assertIn("LIVE", samples["tail"][-1][1])
                    # Nothing was invented or reordered.
                    self.assertEqual(sender.delivered_uptimes, sorted(sender.delivered_uptimes))
                    # No UI stall while 100+ frames/s were decoded.
                    self.assertLess(max(gaps), MAX_UI_GAP_S, f"event loop stalled {max(gaps):.2f}s")
                finally:
                    try:
                        unified.close()
                    finally:
                        core.close()


if __name__ == "__main__":
    unittest.main()
