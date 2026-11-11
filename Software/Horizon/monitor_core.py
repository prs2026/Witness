"""Witness monitor logic without any GUI: inputs, decoding, logs, feed, commands.

MonitorCore is everything the monitor does except drawing. The Qt monitor
(main.MonitorPanel) is a thin view over it; tests drive it headless.

Data flow (unchanged from the Tk monitor):
- Input workers (serial / raw TCP / test source / a mirrored stream) decode
  bytes with PacketDecoder, publish each packet to the Horizon JSON feed at
  once (WitnessPublisher, independent of the GUI) and put events on
  ``self.events``.
- The GUI calls process_events() on a timer. One call drains the whole
  backlog: every sample goes into the graph ring buffers (stamped with its
  arrival time), console lines are written in one batch, and readouts show
  only the newest packet of each type.

User-visible state lives in observable Cells (see observable.py), so views can
bind to it and tests can read ``core.values[key].get()``. Anything that needs
a dialog goes through the ``alert(title, text)`` / ``confirm(title, text)``
hooks, which a GUI replaces; headless defaults log to the console and decline.

Offline demo / timed replay: start_test(); bytes go through PacketDecoder and
are never written to hardware. USB auto-connect opens Witness-named devices and
learned USB identities (auto_connect_witness.json), never generic adapters.
"""

from __future__ import annotations

import datetime as _dt
import json
import queue
import socket
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT, WITNESS_REMOTE_FEED_PORT
from nmea_capture import NmeaExtractor, summarize_gga
from observable import Cell
from serial_discovery import RETRY_SECONDS, SerialDiscovery, SerialOpenJob
from witness_link import WitnessPublisher, is_tailscale_address, tailscale_ipv4
from witness_protocol import (
    IMU_CALIBRATION_COMMANDS, QUICK_COMMANDS, SECTION_SPECS, PacketDecoder,
    build_command_packet, build_imu_calibration_packet, format_display_value,
    format_gps_satellites, format_packet, is_status_field, parse_hex_command,
)
from witness_test_source import builtin_scenario, load_scenario, play_scenario, validate_speed

ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / "logs"

# Default network endpoints shown at startup.
DEFAULT_TCP_1_HOST = ""
DEFAULT_TCP_1_PORT = ""
DEFAULT_TCP_2_HOST = ""
DEFAULT_TCP_2_PORT = ""
DEFAULT_WITNESS_FEED_HOST = WITNESS_FEED_HOST
DEFAULT_WITNESS_FEED_PORT = WITNESS_FEED_PORT
DEFAULT_REMOTE_FEED_PORT = WITNESS_REMOTE_FEED_PORT
REMOTE_RETRY_S = 10.0       # look for Tailscale again while the remote feed is waiting
NO_SERIAL_PORT = "(no port)"
BAUD_CHOICES = ("115200", "57600", "38400", "9600")
WINDOW_TITLE = "Witness Communications Monitor"
TEST_STATUS_IDLE = "Test source: blank = built-in 60s simulated flight. No hardware writes."


def _stamp(fmt="%Y%m%d_%H%M%S_%f"):
    return _dt.datetime.now().strftime(fmt)


def _safe(name, extra="_"):
    return "".join(char if char.isalnum() or char in extra else "_" for char in name)


class RingHistory:
    """Bounded (time, value) history kept in numpy arrays.

    Values may be scalars or fixed-length vectors (x, y, z ...). Storage is a
    2x-capacity buffer that is compacted when full, so arrays() is a cheap
    contiguous view and append is amortized O(1). Non-numeric components
    are stored as NaN. len() and indexing return (time, value) pairs like the
    old list-of-tuples history.
    """

    def __init__(self, capacity: int):
        self.capacity = max(1, int(capacity))
        self._t = np.empty(2 * self.capacity)
        self._v: np.ndarray | None = None
        self._start = self._end = 0
        self.width = 0
        self.last_value = None

    @staticmethod
    def _components(value):
        parts = value if isinstance(value, (list, tuple)) else (value,)
        out = []
        for part in parts:
            try:
                number = float(part)
            except (TypeError, ValueError):
                number = float("nan")
            out.append(number)
        return out

    def append(self, t: float, value) -> None:
        parts = self._components(value)
        if self._v is None or len(parts) != self.width:
            self.width = len(parts)
            self._v = np.empty((2 * self.capacity, self.width))
            self._start = self._end = 0
        if self._end == len(self._t):
            keep = self.capacity - 1
            self._t[:keep] = self._t[self._end - keep:self._end]
            self._v[:keep] = self._v[self._end - keep:self._end]
            self._start, self._end = 0, keep
        self._t[self._end] = t
        self._v[self._end] = parts
        self._end += 1
        if self._end - self._start > self.capacity:
            self._start = self._end - self.capacity
        self.last_value = value

    def arrays(self):
        """(times, values[n, width]) views of the stored samples."""
        if self._v is None:
            return np.empty(0), np.empty((0, 0))
        return self._t[self._start:self._end], self._v[self._start:self._end]

    def __len__(self):
        return self._end - self._start

    def __getitem__(self, index):
        n = len(self)
        if index < 0:
            index += n
        if not 0 <= index < n:
            raise IndexError(index)
        row = self._v[self._start + index]
        value = float(row[0]) if self.width == 1 else [float(v) for v in row]
        return float(self._t[self._start + index]), value

    def __iter__(self):
        for index in range(len(self)):
            yield self[index]


# Confirmation text for quick commands that ask first (default: log erase).
CONFIRM_TEXT = {
    0x4752: "the GPS receiver forgets its ephemerides, almanac, position and time and "
            "needs a new fix (30 s or more). Witness accepts it only in Pad Idle.",
}


class MonitorCore:
    """The Witness monitor's state and behaviour, minus widgets."""

    max_history = 300
    max_events_per_tick = 20000
    max_console_lines = 2000
    max_console_lines_per_tick = 400

    def __init__(self, *, test_mode=False, test_source="", test_speed=1.0, test_loop=True,
                 auto_connect=True, start_feed=True, feed_port=None,
                 remote_feed="off", remote_feed_port=None):
        self._closed = False
        self.serials = {}
        self.tcp_sockets = {}
        self.discovery = SerialDiscovery("witness")
        self._auto_job = None
        self._next_auto_open = 0
        self.stop_event = threading.Event()
        self.test_active = False
        self._test_source = None
        self._test_stop = None
        self._test_thread = None
        self._test_token = None
        self._test_start_id = True if test_mode else None   # pending start request
        self._test_display = False
        self.events = queue.Queue()
        self.log_files = {}
        self.nmea_files = {}
        self.nmea_counts = {}       # source -> [valid, bad]
        self.nmea_last_gga = {}
        self._nmea_dirty = False
        self.debug_log_file = None
        self.history = {}
        self.raw_values = {}        # key -> newest raw value (status decoders)
        self.last_packet_time = {}
        self.command_history = []
        self.command_history_index = 0
        self.command_history_draft = ""
        self.graphs_dirty = False
        self.console_pending = deque()
        self.last_alert = None
        # Hooks a GUI replaces.
        self.alert = self._default_alert
        self.confirm = lambda _title, _text: False

        # Observable state.
        self.auto_connect = Cell(bool(auto_connect and not test_mode))
        self.test_file = Cell(test_source)
        self.test_speed = Cell(str(test_speed))
        self.test_loop = Cell(bool(test_loop))
        self.test_status = Cell(TEST_STATUS_IDLE)
        self.test_warning = Cell(False)            # yellow banner while test values are shown
        self.test_button_text = Cell("Start test")
        self.inputs_locked = Cell(False)           # test running: no connect / test edits
        self.feed_status = Cell("Horizon feed: starting...")
        self.remote_feed_status = Cell("Remote feed: off")
        self.remote_publisher = None
        # remote_feed: "auto" (this PC's Tailscale IPv4), "off", or a Tailscale address.
        self.remote_feed_mode = "off" if not start_feed else str(remote_feed or "off")
        self.remote_feed_port = DEFAULT_REMOTE_FEED_PORT if remote_feed_port is None else int(remote_feed_port)
        self.remote_feed_enabled = Cell(self.remote_feed_mode != "off")
        self._remote_retry_at = 0.0
        self.nmea_status = Cell("NMEA: none received")
        self.status_text = Cell("Disconnected")
        self.connect_text = Cell("Connect")
        self.window_title = Cell(WINDOW_TITLE)
        self.port_choices = Cell((NO_SERIAL_PORT,))
        self.port_a = Cell(NO_SERIAL_PORT)
        self.port_b = Cell(NO_SERIAL_PORT)
        self.baud = Cell("115200")
        self.tcp_hosts = [Cell(DEFAULT_TCP_1_HOST), Cell(DEFAULT_TCP_2_HOST)]
        self.tcp_ports = [Cell(DEFAULT_TCP_1_PORT), Cell(DEFAULT_TCP_2_PORT)]
        self.command_targets = Cell(())
        self.command_target = Cell("")
        self.command_text = Cell("")
        self.commands_enabled = Cell(False)
        self.values = {}
        self.check_vars = {}
        self.section_keys = {}      # packet -> [(key, field, label)]
        for packet, fields in SECTION_SPECS.items():
            rows = []
            for field, label in fields:
                key = f"packet.{packet}.{field}"
                self.values[key] = Cell("null")
                self.check_vars[key] = Cell(False)
                rows.append((key, field, label))
            self.section_keys[packet] = rows

        self.publisher = WitnessPublisher(
            DEFAULT_WITNESS_FEED_HOST, DEFAULT_WITNESS_FEED_PORT if feed_port is None else feed_port)
        if start_feed:
            try:
                self.publisher.start()
                self.feed_status.set(f"Horizon feed: {self.publisher.host}:{self.publisher.port} (receive only)")
            except OSError as exc:
                self.feed_status.set(f"Horizon feed unavailable: {exc}")
                self.console_write(f"FEED_ERROR {exc}; Witness input still works")
        else:
            self.feed_status.set("Horizon feed: off")
        self.start_remote_feed()
        self.refresh_ports()

    # ------------------------------------------------------------ dialogs
    def _default_alert(self, title, text):
        self.last_alert = (title, text)
        self.console_write(f"ALERT[{title}] {text}")

    # -------------------------------------------------------------- ports
    def refresh_ports(self):
        self.discovery.refresh(force=True)
        self._update_port_choices()

    def _update_port_choices(self):
        ports = [p.device for p in self.discovery.ports]
        self.port_choices.set((NO_SERIAL_PORT, *ports))
        for cell in (self.port_a, self.port_b):
            current = cell.get()
            cell.set(current if current in ports or current in self.serials else NO_SERIAL_PORT)
        self.refresh_command_targets()

    def _connected_label(self):
        return "Disconnect" if self.serials or self.tcp_sockets else "Connect"

    def _auto_changed(self):
        self._next_auto_open = 0
        if self.test_active or self._test_start_id is not None:
            self.auto_connect.set(False)
        if not self.auto_connect.get() and self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
            self.connect_text.set(self._connected_label())

    def set_auto_connect(self, enabled):
        self.auto_connect.set(bool(enabled))
        self._auto_changed()

    def auto_poll(self):
        """One auto-connect step (the GUI calls this every 250 ms)."""
        try:
            self.discovery.refresh()
            self._update_port_choices()
            if self.discovery.error:
                self.console_write(f"AUTO_CONNECT {self.discovery.error}")
                self.discovery.error = None
            if not self.auto_connect.get() or self.test_active or self._test_start_id is not None:
                if self._auto_job is not None:
                    self._auto_job.cancel()
                    self._auto_job = None
                return
            if self._auto_job is not None:
                result = self._auto_job.take()
                if result is None:
                    return
                port = self._auto_job.port
                self._auto_job = None
                self.connect_text.set(self._connected_label())
                stream, error = result
                if error:
                    self.console_write(f"AUTO_CONNECT[{port}] {error}; will retry")
                    self.status_text.set(f"Waiting for {port}")
                else:
                    self._accept_auto_port(port, stream)
                self._next_auto_open = time.monotonic() + RETRY_SECONDS
            if len(self.serials) >= 2 or time.monotonic() < self._next_auto_open:
                return
            candidates = self.discovery.candidates(exclude=self.serials)
            if not candidates:
                return
            port = candidates[0]
            self.discovery.attempted(port)
            self._next_auto_open = time.monotonic() + RETRY_SECONDS
            try:
                baud = int(self.baud.get())
                if baud <= 0:
                    raise ValueError("baud must be positive")
            except ValueError as exc:
                self.console_write(f"AUTO_CONNECT invalid baud: {exc}")
                return
            self.console_write(f"AUTO_CONNECT[{port}] opening known Witness USB device")
            self._auto_job = SerialOpenJob(port, baud)
            self.connect_text.set("Cancel connect")
        except Exception as exc:
            self.console_write(f"AUTO_CONNECT_ERROR {exc}")

    _auto_poll = auto_poll   # Tk-era name

    def _leave_test_display(self, message):
        if self._test_display:
            self._clear_telemetry()
            self._test_display = False
            self.window_title.set(WINDOW_TITLE)
            self.test_status.set(message)
            self.test_warning.set(False)

    def _accept_auto_port(self, port, stream):
        if port in self.serials or len(self.serials) >= 2:
            stream.close()
            return
        log_file = None
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = _stamp()
            log_file = (LOG_DIR / f"serial_{_safe(port, '')}_{stamp}.bin").open("ab")
            if self.debug_log_file is None:
                self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        except OSError:
            stream.close()
            if log_file is not None:
                log_file.close()
            raise
        self._leave_test_display("Live mode. Test values cleared.")
        self.serials[port] = stream
        self.log_files[port] = log_file
        self.stop_event.clear()
        for cell in (self.port_a, self.port_b):
            if cell.get() not in self.serials or cell.get() == port:
                cell.set(port)
                break
        self.connect_text.set("Disconnect")
        self.status_text.set(f"Connected: {', '.join(self.serials)}")
        self.refresh_command_targets()
        self.console_write(f"AUTO_CONNECT[{port}] connected")
        threading.Thread(target=self.read_serial, args=(port, stream), daemon=True).start()

    # ----------------------------------------------------------- commands
    def refresh_command_targets(self):
        targets = () if self.test_active else (*self.serials, *self.tcp_sockets)
        self.command_targets.set(tuple(targets))
        if self.command_target.get() not in targets:
            self.command_target.set(targets[0] if targets else "")
        self.commands_enabled.set(bool(targets))

    def transmit_command(self, target_name, command, description=None):
        if self.test_active:
            self.console_write("TX_BLOCKED test mode never sends commands to hardware")
            return False
        serial_port = self.serials.get(target_name)
        tcp_socket = self.tcp_sockets.get(target_name)
        if serial_port is None and tcp_socket is None:
            self.console_write("TX_ERROR no connected serial/TCP target selected")
            return False
        try:
            if serial_port is not None:
                serial_port.write(command)
            else:
                tcp_socket.sendall(command)
            sent_text = command.hex(" ")
            label = f" {description}:" if description else ""
            self.console_write(f"TX[{target_name}]{label}  {sent_text}")
            self.command_history.append(sent_text)
            self.command_history_index = len(self.command_history)
            self.command_history_draft = ""
            return True
        except Exception as exc:
            self.console_write(f"TX_ERROR[{target_name}] {exc}")
            return False

    def send_hex_command(self):
        target_name = self.command_target.get()
        try:
            command = parse_hex_command(self.command_text.get())
        except ValueError as exc:
            self.console_write(f"TX_ERROR[{target_name}] invalid hex command: {exc}")
            return False
        if self.transmit_command(target_name, command):
            self.command_text.set("")
            return True
        return False

    build_command_packet = staticmethod(build_command_packet)
    build_quick_command_packet = staticmethod(build_command_packet)

    def send_quick_command(self, name, opcode, confirm=False):
        text = CONFIRM_TEXT.get(opcode, "delete all stored log sessions? This cannot be undone.")
        if confirm and not self.confirm("Confirm command", f"{name}: {text}"):
            return False
        return self.transmit_command(self.command_target.get(), build_command_packet(opcode), name)

    def send_gps_assist(self, frames, description="GPS ephemeris upload", pace_s=None):
        """Send GPS assistance frames (gps_assist.assist_frames) to the
        selected target on a background thread, one console line per upload.
        Serial (USB) is paced at 3 ms per frame; TCP (Aperture/radio bridge)
        at 60 ms so the radio queue keeps up."""
        import gps_assist
        target_name = self.command_target.get()
        if self.test_active:
            self.console_write("TX_BLOCKED test mode never sends commands to hardware")
            return False
        serial_port = self.serials.get(target_name)
        tcp_socket = self.tcp_sockets.get(target_name)
        if serial_port is None and tcp_socket is None:
            self.console_write("TX_ERROR no connected serial/TCP target selected")
            return False
        write = serial_port.write if serial_port is not None else tcp_socket.sendall
        pace = pace_s if pace_s is not None else (0.003 if serial_port is not None else 0.06)
        frames = list(frames) + [gps_assist.status_frame()]

        def worker():
            try:
                gps_assist.send_frames(write, frames, pace)
                self.console_write(f"TX[{target_name}] {description}: {len(frames)} frames sent; "
                                   "Witness logs 'GPS assist done: N/N acknowledged'")
            except Exception as exc:  # noqa: BLE001
                self.console_write(f"TX_ERROR[{target_name}] {description}: {exc}")
        threading.Thread(target=worker, name="gps-assist", daemon=True).start()
        return True

    def send_imu_calibration(self, name, argument):
        """Compact USB-only calibration command; Aperture does not forward it."""
        return self.transmit_command(self.command_target.get(), build_imu_calibration_packet(argument), name)

    def recall_command(self, direction):
        """Step through successfully transmitted commands (Up = -1, Down = +1)."""
        if not self.command_history:
            return self.command_text.get()
        history_end = len(self.command_history)
        if direction < 0:
            if self.command_history_index == history_end:
                self.command_history_draft = self.command_text.get()
            self.command_history_index = max(0, self.command_history_index - 1)
        else:
            self.command_history_index = min(history_end, self.command_history_index + 1)
        if self.command_history_index == history_end:
            recalled = self.command_history_draft
        else:
            recalled = self.command_history[self.command_history_index]
        self.command_text.set(recalled)
        return recalled

    # -------------------------------------------------------- connections
    def toggle_connection(self):
        if self.serials or self.tcp_sockets or self._auto_job is not None:
            self.auto_connect.set(False)
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
        if self.test_active:
            self.alert("Test mode", "Stop the test before connecting real inputs.")
            return False
        ports = [port for port in (self.port_a.get(), self.port_b.get())
                 if port and port != NO_SERIAL_PORT]
        tcp_endpoints = []
        for index, (host_cell, port_cell) in enumerate(zip(self.tcp_hosts, self.tcp_ports), start=1):
            host = str(host_cell.get()).strip()
            port_text = str(port_cell.get()).strip()
            if not host:
                continue
            try:
                port = int(port_text)
            except ValueError:
                self.alert("Horizon monitor", f"TCP {index} port must be a number.")
                return False
            if not 1 <= port <= 65535:
                self.alert("Horizon monitor", f"TCP {index} port must be 1-65535.")
                return False
            tcp_endpoints.append((index, host, port))
        if len({(host, port) for _, host, port in tcp_endpoints}) != len(tcp_endpoints):
            self.alert("Horizon monitor", "TCP endpoints must be different.")
            return False
        if not ports and not tcp_endpoints:
            self.alert("Horizon monitor", "Select a serial port or enter a TCP host.")
            return False
        if len(set(ports)) != len(ports):
            self.alert("Serial monitor", "Port A and Port B must be different.")
            return False
        self._leave_test_display("Live mode. Test values cleared; source file remains available for replay.")
        try:
            if ports:
                import serial
        except ImportError:
            self.alert("Serial monitor", "pyserial is required: pip install pyserial")
            return False
        try:
            baud = int(self.baud.get())
        except ValueError:
            self.alert("Serial monitor", "Baud must be a number.")
            return False
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = _stamp("%Y%m%d_%H%M%S")
        self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        self.stop_event.clear()
        try:
            for port in ports:
                serial_port = serial.Serial(port, baud, timeout=0.2)
                self.serials[port] = serial_port
                self.log_files[port] = (LOG_DIR / f"serial_{_safe(port, '')}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_serial, args=(port, serial_port), daemon=True).start()
            self.refresh_command_targets()
            for _index, tcp_host, tcp_port in tcp_endpoints:
                tcp_socket = socket.create_connection((tcp_host, tcp_port), timeout=3.0)
                tcp_socket.settimeout(0.2)
                tcp_source = f"TCP_{tcp_host}_{tcp_port}"
                self.tcp_sockets[tcp_source] = tcp_socket
                self.log_files[tcp_source] = (LOG_DIR / f"{_safe(tcp_source, '')}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_tcp, args=(tcp_source, tcp_socket), daemon=True).start()
            self.refresh_command_targets()
        except Exception as exc:
            self.disconnect()
            self.alert("Serial monitor", f"Could not open serial port: {exc}")
            return False
        self.connect_text.set("Disconnect")
        sources = ports + [f"TCP {host}:{port}" for _, host, port in tcp_endpoints]
        self.status_text.set(f"Connected: {', '.join(sources)}")
        return True

    def disconnect(self):
        if self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
        self.stop_event.set()
        for stream in (*self.serials.values(), *self.tcp_sockets.values()):
            try:
                stream.close()
            except Exception:
                pass
        self.serials.clear()
        self.tcp_sockets.clear()
        self.refresh_command_targets()
        for log_file in self.log_files.values():
            try:
                log_file.close()
            except Exception:
                pass
        self.log_files.clear()
        for source in list(self.nmea_files):
            try:
                self._close_nmea_file(source)
            except Exception:
                pass
        if self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None
        self.connect_text.set("Connect")
        self.status_text.set("Disconnected")

    # ---------------------------------------------------------- test mode
    def _start_requested_test(self):
        self._test_start_id = None
        return self.start_test()

    def toggle_test(self):
        return self.stop_test() if self.test_active else self.start_test()

    def _clear_telemetry(self):
        for cell in self.values.values():
            cell.set("null")
        self.raw_values.clear()
        self.history.clear()
        self.last_packet_time.clear()
        self.graphs_dirty = True

    def start_test(self):
        if self.test_active:
            return False
        if self.serials or self.tcp_sockets:
            self.alert("Test mode", "Disconnect real serial/TCP inputs before starting a test.")
            return False
        self.auto_connect.set(False)
        self._auto_changed()
        raw_log = None
        try:
            speed = validate_speed(float(self.test_speed.get()))
            filename = str(self.test_file.get()).strip()
            scenario = load_scenario(filename) if filename else builtin_scenario()
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = _stamp()
            raw_log = (LOG_DIR / f"test_{stamp}.bin").open("xb")
            self.debug_log_file = (LOG_DIR / f"test_debug_{stamp}.log").open("x", encoding="utf-8")
        except (OSError, ValueError) as exc:
            if raw_log is not None:
                raw_log.close()
            self.alert("Test mode", str(exc))
            return False
        self._clear_telemetry()
        self._test_display = self.test_active = True
        self._test_token = token = object()
        self._test_stop = stop = threading.Event()
        self._test_source = source = "TEST_" + _safe(scenario.name, "_-")[:100]
        self.window_title.set(WINDOW_TITLE + " [TEST DATA]")
        self.test_warning.set(True)
        self.test_status.set(f"TEST DATA: {scenario.name} at {speed:g}x; source {source}. No hardware writes.")
        self.status_text.set("TEST MODE - simulated input")
        self.test_button_text.set("Stop test")
        self.inputs_locked.set(True)
        self.refresh_command_targets()
        loop = bool(self.test_loop.get())
        self.console_write(f"TEST_START[{source}] {len(scenario.frames)} chunks, "
                           f"duration={scenario.duration_ms}ms, speed={speed:g}x, "
                           f"loop={loop}; raw log={raw_log.name}")
        self._test_thread = threading.Thread(
            target=self.read_test, args=(scenario, source, token, stop, raw_log, speed, loop),
            name="witness-test-source", daemon=True)
        self._test_thread.start()
        return True

    def read_test(self, scenario, source, token, stop, raw_log, speed, loop):
        nmea = NmeaExtractor()

        def event(kind, payload):
            self.events.put(("test_event", (token, kind, payload)))
        decoder = PacketDecoder(
            lambda packet: self.on_decoded_packet(source, packet, test_token=token),
            lambda message: event("decoder_error", (source, message)),
            lambda packet_id, count: event("host_packet", (source, packet_id, count)))

        def receive(data):
            # Backpressure is safe for an offline source; keep the GUI queue bounded.
            while self.events.qsize() >= 1000:
                if stop.wait(0.01):
                    return
            if stop.is_set():
                return
            raw_log.write(data)
            raw_log.flush()
            event("bytes", (source, data))
            decoder.feed(data)
            for sentence, ok in nmea.feed(data):
                event("nmea", (source, sentence, ok, time.time()))

        def reset_decoder():
            if decoder.buffer:
                event("decoder_error", (source, f"test loop discarded {len(decoder.buffer)} trailing bytes"))
            decoder.buffer.clear()
        result = "completed"
        try:
            with raw_log:
                play_scenario(scenario, receive, stop, speed=speed, loop=loop, on_cycle=reset_decoder)
            if decoder.buffer and not stop.is_set():
                event("decoder_error", (source, f"test ended with {len(decoder.buffer)} trailing bytes"))
        except Exception as exc:
            result = f"failed: {exc}"
            event("decoder_error", (source, result))
        finally:
            event("test_done", result)

    def stop_test(self, result="stopped"):
        if not self.test_active:
            return False
        self._test_stop.set()
        if self._test_thread is not None:
            self._test_thread.join(timeout=2)
        self.test_active = False
        self._test_token = None  # Ignore queued display/completion events from an old run.
        self._test_thread = None
        self.test_button_text.set("Start test")
        self.inputs_locked.set(False)
        self.refresh_command_targets()
        self.status_text.set("TEST MODE - " + result)
        self.test_status.set(f"TEST {result}: displayed values are simulated. Start test to replay again.")
        self.console_write(f"TEST_END[{self._test_source}] {result}")
        if self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None
        return True

    # ------------------------------------------------------------ workers
    def on_decoded_packet(self, source, packet, *, test_token=None):
        # Called by input workers: export immediately, independent of GUI backlog.
        # The arrival time travels with the packet so graphs plot when it was
        # received, not when the GUI got round to it.
        received_at = time.time()
        try:
            self.publisher.publish(source, packet, received_at=received_at)
        except Exception as exc:
            self.events.put(("decoder_error", (source, f"Horizon feed: {exc}")))
        remote = getattr(self, "remote_publisher", None)
        if remote is not None:
            # Separate publisher, thread and buffers: never delays the local feed.
            try:
                remote.publish(source, packet, received_at=received_at)
            except Exception as exc:  # noqa: BLE001
                self.events.put(("decoder_error", (source, f"Remote feed: {exc}")))
        if test_token is None:
            self.events.put(("packet", (source, packet, received_at)))
        else:
            self.events.put(("test_event", (test_token, "packet", (source, packet, received_at))))

    def _queue_nmea(self, source, extractor, data):
        sentences = extractor.feed(data)
        if sentences:
            received_at = time.time()
            for sentence, ok in sentences:
                self.events.put(("nmea", (source, sentence, ok, received_at)))

    def _decoder(self, source):
        return PacketDecoder(
            lambda p: self.on_decoded_packet(source, p),
            lambda message: self.events.put(("decoder_error", (source, message))),
            lambda packet_id, count: self.events.put(("host_packet", (source, packet_id, count))),
        )

    def _log_input(self, source, data):
        log_file = self.log_files.get(source)
        if log_file is not None:
            log_file.write(data)
            log_file.flush()

    def read_serial(self, port_name, serial_port):
        nmea = NmeaExtractor()
        decoder = self._decoder(port_name)
        while not self.stop_event.is_set() and self.serials.get(port_name) is serial_port:
            try:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    self._log_input(port_name, data)
                    self.events.put(("bytes", (port_name, bytes(data))))
                    decoder.feed(data)
                    self._queue_nmea(port_name, nmea, data)
            except Exception as exc:
                self.events.put(("error", (port_name, str(exc), serial_port)))
                break

    def read_tcp(self, source_name, tcp_socket):
        nmea = NmeaExtractor()
        decoder = self._decoder(source_name)
        while not self.stop_event.is_set() and self.tcp_sockets.get(source_name) is tcp_socket:
            try:
                data = tcp_socket.recv(4096)
                if not data:
                    self.events.put(("error", (source_name, "TCP server closed the connection", tcp_socket)))
                    break
                self._log_input(source_name, data)
                self.events.put(("bytes", (source_name, bytes(data))))
                decoder.feed(data)
                self._queue_nmea(source_name, nmea, data)
            except socket.timeout:
                continue
            except Exception as exc:
                self.events.put(("error", (source_name, str(exc), tcp_socket)))
                break

    def stream_feeder(self, source_name):
        """Return feed(bytes) that decodes a byte stream owned by someone else.

        Used to mirror a connection another tool holds (witnessOS shares the
        HITL bench's Witness TCP connection, since ser2net serves one client).
        Decoding, publishing and display are exactly as for read_tcp. Call
        feed from one thread at a time; nothing is ever sent to Witness.
        """
        nmea = NmeaExtractor()
        decoder = self._decoder(source_name)

        def feed(data):
            if self._closed:
                return
            self.events.put(("bytes", (source_name, bytes(data))))
            decoder.feed(data)
            self._queue_nmea(source_name, nmea, data)

        return feed

    # ------------------------------------------------------- event pump
    def process_events(self):
        """Drain every queued event, then update the readouts once.

        Graph history keeps every sample (stamped with its arrival time),
        console lines are written in one batch, and values show only the
        newest packet of each type. Returns the number of events handled.
        """
        latest = {}              # packet type -> (port, packet), newest wins
        ports_seen = set()
        console_lines = []
        handled = 0
        while handled < self.max_events_per_tick:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            handled += 1
            try:
                self._handle_event(kind, payload, latest, ports_seen, console_lines)
            except Exception as exc:
                # A malformed packet or display value must never stop the pump.
                console_lines.append(self._console_stamp(f"GUI_ERROR {exc}"))
        for port_name in ports_seen:
            if port_name in self.serials:
                self.discovery.remember(port_name)
        for _port_name, packet in latest.values():
            try:
                self.update_values("packet", packet, record=False)
            except Exception as exc:
                console_lines.append(self._console_stamp(f"GUI_ERROR {exc}"))
        self._write_console_lines(console_lines)
        self._refresh_nmea_status()
        return handled

    @staticmethod
    def _console_stamp(line):
        timestamp = _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        return f"[{timestamp}] {line}"

    def _handle_event(self, kind, payload, latest, ports_seen, console_lines):
        """Apply one queued event. Display work is deferred to the caller."""
        if kind == "test_event":
            token, kind, payload = payload
            if token is not self._test_token:
                return
            if kind == "test_done":
                self._write_console_lines(console_lines)
                console_lines.clear()
                self.stop_test(result=payload)
                return
        elif self.test_active:
            # Ignore late events from a previously disconnected live input;
            # never mix those values with this test run.
            return
        if kind == "packet":
            port_name, packet = payload[0], payload[1]
            received_at = payload[2] if len(payload) > 2 else time.time()
            ports_seen.add(port_name)
            self.record_history("packet", packet, received_at)
            latest[packet.get("packet")] = (port_name, packet)
            console_lines.append(self._console_stamp(f"PACKET[{port_name}] {format_packet(packet)}"))
        elif kind == "nmea":
            self._handle_nmea(*payload)
        elif kind == "bytes":
            port_name, data = payload
            console_lines.append(self._console_stamp(f"RX[{port_name}]  {data.hex(' ')}"))
        elif kind == "decoder_error":
            port_name, message = payload
            console_lines.append(self._console_stamp(f"BAD_PACKET[{port_name}] {message}"))
        elif kind == "host_packet":
            # Echoed host->Witness packet (0x0C HITL input): framed and
            # skipped, never shown as telemetry. Log sparsely.
            port_name, packet_id, count = payload
            if count == 1 or count % 100 == 0:
                console_lines.append(self._console_stamp(
                    f"HOST_ECHO[{port_name}] skipped 0x{packet_id:02x} host packet "
                    f"(HITL input; {count} so far)"))
        elif kind == "error":
            # Rare and stateful: flush what came before so the log stays in order.
            self._write_console_lines(console_lines)
            console_lines.clear()
            self._handle_input_error(payload)

    # --------------------------------------------------------------- NMEA
    def _nmea_file(self, source):
        handle = self.nmea_files.get(source)
        if handle is None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            safe = _safe(source, "-_.")
            handle = (LOG_DIR / f"nmea_{safe}_{_stamp('%Y%m%d_%H%M%S')}.csv").open(
                "a", encoding="ascii", newline="")
            if handle.tell() == 0:
                handle.write("received_utc,checksum_ok,sentence\n")
            self.nmea_files[source] = handle
        return handle

    def _handle_nmea(self, source, sentence, ok, received_at):
        """Log one NMEA sentence (valid or damaged) and refresh the summary."""
        counts = self.nmea_counts.setdefault(source, [0, 0])
        counts[0 if ok else 1] += 1
        handle = self._nmea_file(source)
        stamp = _dt.datetime.fromtimestamp(received_at, _dt.timezone.utc).isoformat(timespec="milliseconds")
        handle.write(f"{stamp},{int(ok)},{sentence}\n")
        self._nmea_dirty = True
        if ok:
            gga = summarize_gga(sentence)
            if gga:
                self.nmea_last_gga[source] = gga

    def _refresh_nmea_status(self):
        if not self._nmea_dirty:
            return
        self._nmea_dirty = False
        parts = []
        for source, (valid, bad) in self.nmea_counts.items():
            text = f"{source}: {valid} sentences ({bad} damaged)"
            if source in self.nmea_last_gga:
                text += f", last {self.nmea_last_gga[source]}"
            parts.append(text)
        self.nmea_status.set("NMEA " + "; ".join(parts) + f" - logged to {LOG_DIR.name}/nmea_*.csv")
        for handle in self.nmea_files.values():
            handle.flush()

    def _close_nmea_file(self, source):
        handle = self.nmea_files.pop(source, None)
        if handle is not None:
            handle.close()

    def _handle_input_error(self, payload):
        port_name, message, stream = payload
        inputs = self.serials if port_name in self.serials else self.tcp_sockets
        if inputs.get(port_name) is not stream:
            return  # A late failure from a replaced/disconnected worker.
        inputs.pop(port_name)
        stream.close()
        log_file = self.log_files.pop(port_name, None)
        if log_file is not None:
            log_file.close()
        self._close_nmea_file(port_name)
        self.refresh_command_targets()
        if not self.serials and not self.tcp_sockets:
            self.connect_text.set("Connect")
        self.status_text.set(f"Serial error on {port_name}: {message}")
        self.console_write(f"ERROR[{port_name}] {message}")
        if not self.serials and not self.tcp_sockets and self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None

    # ------------------------------------------------------------ readouts
    # ------------------------------------------------------------ remote feed
    def start_remote_feed(self):
        """Start the Tailscale feed if enabled; returns True when it is running."""
        if self.remote_publisher is not None or not self.remote_feed_enabled.get():
            if not self.remote_feed_enabled.get():
                self.remote_feed_status.set("Remote feed (Tailscale): off")
            return self.remote_publisher is not None
        self._remote_retry_at = time.monotonic() + REMOTE_RETRY_S
        mode = self.remote_feed_mode
        host = tailscale_ipv4() if mode in ("auto", "on") else mode
        if not host:
            self.remote_feed_status.set("Remote feed (Tailscale): waiting for Tailscale "
                                        "(not connected on this PC); retrying")
            return False
        if not is_tailscale_address(host):
            self.remote_feed_status.set(f"Remote feed (Tailscale): {host} is not a Tailscale address; off")
            return False
        publisher = WitnessPublisher(host, self.remote_feed_port, remote=True)
        try:
            publisher.start()
        except OSError as exc:
            self.remote_feed_status.set(f"Remote feed (Tailscale) unavailable: {exc}; retrying")
            return False
        self.remote_publisher = publisher
        self._remote_status_text()
        self.console_write(f"Remote feed on {publisher.host}:{publisher.port} (Tailscale, receive only)")
        return True

    def stop_remote_feed(self):
        publisher, self.remote_publisher = self.remote_publisher, None
        if publisher is not None:
            publisher.close()

    def set_remote_feed(self, enabled):
        """Turn the Tailscale feed on/off at runtime (the local feed is untouched)."""
        self.remote_feed_enabled.set(bool(enabled))
        if enabled:
            if self.remote_feed_mode == "off":
                self.remote_feed_mode = "auto"
            self.start_remote_feed()
        else:
            self.stop_remote_feed()
            self.remote_feed_status.set("Remote feed (Tailscale): off")

    def _remote_status_text(self):
        p = self.remote_publisher
        if p is None:
            return
        if p.error:
            self.remote_feed_status.set(f"Remote feed (Tailscale) stopped: {p.error}; restarting")
            return
        text = (f"Remote feed (Tailscale): {p.host}:{p.port}, {p.clients} viewer"
                f"{'' if p.clients == 1 else 's'} connected")
        if p.disconnected_slow:
            text += f"; {p.disconnected_slow} slow viewer(s) dropped (local feed unaffected)"
        self.remote_feed_status.set(text)

    def update_feed_status(self):
        p = self.remote_publisher
        if p is not None and p.error:
            self.stop_remote_feed()
        if self.remote_publisher is None and self.remote_feed_enabled.get() \
                and time.monotonic() >= self._remote_retry_at:
            self.start_remote_feed()
        self._remote_status_text()
        if self.publisher.error:
            self.feed_status.set(f"Horizon feed stopped: {self.publisher.error}")
        elif self.publisher.dropped:
            self.feed_status.set(
                f"Horizon feed: {self.publisher.host}:{self.publisher.port}; "
                f"overload: {self.publisher.dropped} updates dropped (raw Witness logs unaffected)")

    def data_ages(self, now=None):
        """{"packet.<type>": seconds since its newest packet, or None}."""
        now = time.monotonic() if now is None else now
        return {f"packet.{packet}": (None if self.last_packet_time.get(f"packet.{packet}") is None
                                     else max(0.0, now - self.last_packet_time[f"packet.{packet}"]))
                for packet in SECTION_SPECS}

    format_display_value = staticmethod(format_display_value)
    format_gps_satellites = staticmethod(format_gps_satellites)
    format_packet = staticmethod(format_packet)

    def console_write(self, line):
        self._write_console_lines([self._console_stamp(line)])

    def _write_console_lines(self, lines):
        """Append stamped lines: all to the debug log, a bounded tail to the console."""
        if not lines:
            return
        if self.debug_log_file is not None:
            self.debug_log_file.write("\n".join(lines) + "\n")
            self.debug_log_file.flush()
        shown = lines
        if len(lines) > self.max_console_lines_per_tick:
            skipped = len(lines) - self.max_console_lines_per_tick
            shown = [self._console_stamp(
                f"CONSOLE skipped {skipped} line(s) to keep up; all are in the debug log")]
            shown += lines[-self.max_console_lines_per_tick:]
        self.console_pending.extend(shown)
        while len(self.console_pending) > self.max_console_lines:
            self.console_pending.popleft()

    def take_console(self):
        """Console lines written since the last call (the GUI shows them)."""
        lines = list(self.console_pending)
        self.console_pending.clear()
        return lines

    def record_history(self, kind, packet, received_at):
        """Add one packet's numeric fields to the graph history at its arrival time."""
        name = packet.get("packet")
        for field, value in packet.items():
            if field == "packet" or not isinstance(value, (int, float, list, tuple)):
                continue
            key = f"{kind}.{name}.{field}"
            if key not in self.values:
                continue
            history = self.history.get(key)
            if history is None:
                history = self.history[key] = RingHistory(self.max_history)
            history.append(received_at, value)
        self.graphs_dirty = True

    def update_values(self, kind, packet, *, record=True, draw=None):
        """Show a packet's fields (the pump calls this for the newest packet per type)."""
        if record:
            self.record_history(kind, packet, time.time())
        packet_key = f"{kind}.{packet['packet']}"
        self.last_packet_time[packet_key] = time.monotonic()
        for field, value in packet.items():
            if field == "packet":
                continue
            key = f"{packet_key}.{field}"
            if key not in self.values:
                continue
            if is_status_field(field):
                self.raw_values[key] = value
            if field == "gps_satellites" and packet["packet"] == "state":
                self.values[key].set(format_gps_satellites(packet))
            else:
                self.values[key].set(format_display_value(field, value))

    def selected_fields(self):
        return [key for key, cell in self.check_vars.items() if cell.get()]

    def clear_history(self):
        self.history.clear()
        self.graphs_dirty = True

    def save_graph_setup(self, filename):
        selected = self.selected_fields()
        setup = {"format": "horizon-graph-setup", "version": 1, "selected_fields": selected}
        Path(filename).write_text(json.dumps(setup, indent=2) + "\n", encoding="utf-8")
        self.console_write(f"GRAPH_SETUP saved {len(selected)} field(s) to {filename}")

    def load_graph_setup(self, filename):
        """Select the fields saved in filename. Raises OSError/ValueError."""
        setup = json.loads(Path(filename).read_text(encoding="utf-8"))
        if not isinstance(setup, dict):
            raise ValueError("the top-level JSON value must be an object")
        selected_fields = setup.get("selected_fields")
        if not isinstance(selected_fields, list) or not all(isinstance(key, str) for key in selected_fields):
            raise ValueError("selected_fields must be a list of field names")
        selected_set = set(selected_fields)
        unknown_fields = sorted(selected_set.difference(self.check_vars))
        for key, cell in self.check_vars.items():
            cell.set(key in selected_set)
        self.graphs_dirty = True
        self.console_write(f"GRAPH_SETUP loaded {len(selected_set) - len(unknown_fields)} field(s) "
                           f"from {filename}")
        if unknown_fields:
            self.console_write("GRAPH_SETUP ignored unknown field(s): " + ", ".join(unknown_fields))
        return unknown_fields

    # ------------------------------------------------------------ shutdown
    def close(self):
        if self._closed:
            return
        self._closed = True
        self._test_start_id = None
        self.stop_test()
        self.disconnect()
        self.publisher.close()
        self.stop_remote_feed()


__all__ = [
    "MonitorCore", "RingHistory", "LOG_DIR", "NO_SERIAL_PORT", "WINDOW_TITLE", "BAUD_CHOICES",
    "QUICK_COMMANDS", "IMU_CALIBRATION_COMMANDS",
]
