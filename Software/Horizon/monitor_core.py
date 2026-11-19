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
from link_health import (
    LINK_LOST_AFTER_S, TCP_NO_DATA_RECONNECT_S, AnyEvent, BoundedEvents, Coalescer, DebugLog,
    LinkStatus, RateLimited, RawLog, RAW_FSYNC_S, backoff_delay, open_serial_quiet, tcp_send_all,
)
from nmea_capture import NmeaExtractor, summarize_gga
from observable import Cell
from serial_discovery import RETRY_SECONDS, SerialDiscovery, SerialOpenJob
import bit_model
from command_link import CommandLink
from witness_link import WitnessPublisher, enable_keepalive, is_tailscale_address, tailscale_ipv4
from witness_protocol import (
    IMU_CALIBRATION_COMMANDS, QUICK_COMMANDS, SECTION_SPECS, PacketDecoder,
    BIT_REPORT_SUMMARY, BIT_RUN_COMMAND, build_bit_report_packet, build_bit_run_packet,
    build_command_packet, build_imu_calibration_packet, format_display_value,
    format_gps_satellites, format_packet, is_status_field, parse_hex_command, FLIGHT_STATE_MASK,
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
SERIAL_WRITE_TIMEOUT_S = 2.0
# Commands that Witness acknowledges are resent only this often (the uplink is a
# half-duplex radio and Witness pauses telemetry for 1 s after every command).
COMMAND_RESEND_S = 2.5
# Aperture paces the radio uplink (3 commands per burst, then 1.5 s quiet): a 32-frame
# GPS-assist upload takes 20-28 s, so the STATUS answer is awaited at least this long.
ASSIST_STATUS_TIMEOUT_S = 30.0
BUSY_CONFIRM_DETAIL = 0xC0F1     # erase/clear over radio: "send again within 10 s to confirm"
# BUSY acks Aperture sends itself (never from Witness): the command did not go out.
APERTURE_BUSY_DETAILS = {0xA901: "Aperture could not transmit it (3 radio send failures)",
                         0xA902: "Aperture's uplink queue is full"}
BUSY_CONFIRM_GAP_S = 1.0
RESET_TO_PAD_OPCODE = 0x7300        # reboots Witness when Landed: the link drops for a few s
LANDED_STATE = 4                    # witness flight state (status byte 1 bits 2:0)
WITNESS_STATE_PACKETS = ("state", "sensors", "heartbeat")   # carry the Witness status word
RESET_EXPECTED_S = 20.0
PAD_IDLE_STATE = 0                  # witness flight state; BIT runs only here (from the button, never automatic)
BOOST_STATE = 1                     # Witness refuses a BIT report in Boost
BIT_RUN_ACK_WAIT_S = 30.0           # the run ack comes at the END of the run (about 6-12 s); never resent
BIT_DETAIL_PAGE_WAIT_S = 5.0        # wait for a re-sent detail page before asking for the next
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


class Link:
    """One live input (serial port or TCP endpoint) that reconnects by itself.

    The reader thread owns the stream: when it fails for any reason other than
    the operator pressing Disconnect it keeps the Link (same source name, same
    raw log file) in a "reconnecting" state and re-opens with backoff, forever.
    """

    def __init__(self, name, kind, opener, raw_log, *, stream=None, auto=False):
        self.name, self.kind, self.opener, self.raw_log, self.auto = name, kind, opener, raw_log, auto
        self.stream = stream
        self.stop = threading.Event()        # operator Disconnect (or core close)
        self.down = threading.Event()        # set while there is no usable stream
        if stream is None:
            self.down.set()
        self.status = LinkStatus(name, kind)
        self.send_lock = threading.Lock()
        self.last_reason = ""
        self.thread = None
        self.cancel = AnyEvent(self.stop, self.down)   # for command retry / upload loops

    def drop_stream(self):
        """Close the current stream; the reader notices and reconnects."""
        stream = self.stream
        if stream is not None:
            try:
                stream.close()
            except Exception:  # noqa: BLE001
                pass

    def write(self, data):
        """Write a whole command frame or raise; on failure the connection is
        dropped so a half-written frame can never be followed by another."""
        stream = self.stream
        if stream is None:
            raise ConnectionError(f"{self.name} link is down (reconnecting)")
        with self.send_lock:
            try:
                if self.kind == "tcp":
                    tcp_send_all(stream, data)
                else:
                    stream.write(data)
            except Exception:
                self.drop_stream()
                raise


# Confirmation text for quick commands that ask first (default: log erase).
CONFIRM_TEXT = {
    BIT_RUN_COMMAND: "runs the Witness built-in self test now (about 10 s): it writes and reads back a test "
                     "page in NAND and checks the sensors, logger and radio, so keep the rocket still on "
                     "the pad. Witness accepts it only in Pad Idle (or Landed). It does not change the "
                     "flight state.",
    0x7300: "use this only for a FALSE FLIGHT ON THE PAD (Witness reached Landed without a real "
            "flight). Witness restarts now: the link drops for a few seconds, it boots into Pad Idle "
            "with a fresh filter and a new log session. It accepts this only in Landed.",
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
        # Display events only; raw bytes are logged by the reader thread itself.
        # When a long GUI stall fills it, the OLDEST display event is dropped.
        self.events = BoundedEvents(self.max_events_per_tick)
        self.links = {}             # source name -> Link (serial or TCP, reconnecting)
        self.log_files = {}         # source name -> RawLog
        self._sync_thread = None
        self._pump_errors = RateLimited(5.0)
        self._last_health = 0.0
        self.expected_reset_until = 0.0     # monotonic: 0x7300 sent, a link drop is expected
        self._probe_thread = None           # Tailscale CLI probe runs off the GUI thread
        self._probe_done = False
        self._probe_result = None
        self.nmea_files = {}
        self.nmea_counts = {}       # source -> [valid, bad]
        self.nmea_last_gga = {}
        self.nmea_log_errors = 0
        self._nmea_dirty = False
        self.debug_log_file = None
        self.history = {}
        self.raw_values = {}        # key -> newest raw value (status decoders)
        self.last_packet_time = {}
        self.command_history = []
        self.command_link = CommandLink()     # 0x06 acks / 0x07 transfer status
        self.ack_attempts = 3       # each resend waits COMMAND_RESEND_S (uplink is half duplex)
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
        self.link_banner = Cell("")         # "LINK LOST ..." (empty = link fine); GUI thread only
        self.link_summary = Cell("")        # one line per source: state, age, rate
        self.log_warning = Cell("")         # raw/debug log trouble, visible to the operator
        self.link_snapshots = []            # per-source dicts, see LinkStatus.snapshot()
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
        # "False flight on pad: reset Witness" (0x7300): only while Witness reports Landed.
        self.reset_witness_enabled = Cell(False)
        self.wifi_on_enabled = Cell(False)
        self.commands_enabled.subscribe(lambda _value: self._refresh_reset_enabled())
        # Built-in self test (BIT): the newest summary/detail Witness sent, a one-line status for the
        # readouts, and the two buttons. "Run BIT" (0x5442) is enabled only in Pad Idle and is sent
        # only by the operator; "BIT detail" (0x5452) only asks Witness to re-send.
        self.bit = bit_model.BitModel()
        self.bit_text = Cell("BIT: no data yet")
        self.bit_kind = Cell("muted")
        self.bit_tooltip = Cell("No BIT summary (0x0B) received from Witness yet.")
        self.bit_run_enabled = Cell(False)
        self.bit_detail_enabled = Cell(False)
        self._bit_report_busy = threading.Lock()
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
        return "Disconnect" if self.serials or self.tcp_sockets or self.links else "Connect"

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
            occupied = {*self.serials, *self.links}   # reconnecting ports stay occupied
            if len(occupied) >= 2 or time.monotonic() < self._next_auto_open:
                return
            candidates = self.discovery.candidates(exclude=occupied)
            if not candidates:
                return
            # A USB device that came back under a NEW port name: the old name will
            # never return, so release its (auto-opened, reconnecting) link.
            present = {p.device for p in self.discovery.ports}
            for name, link in list(self.links.items()):
                if link.auto and link.down.is_set() and name not in present:
                    self._remove_link(name, "device re-enumerated under another port name")
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
        if port in self.serials or port in self.links or len(self.links) >= 2:
            stream.close()
            return
        try:
            stream.write_timeout = SERIAL_WRITE_TIMEOUT_S   # SerialOpenJob opens without one
        except Exception:  # noqa: BLE001
            pass
        stamp = _stamp()
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            if self.debug_log_file is None:
                self.debug_log_file = DebugLog(LOG_DIR / f"debug_{stamp}.log")
        except OSError as exc:
            self.console_write(f"DEBUG_LOG_ERROR {exc}")
        self._leave_test_display("Live mode. Test values cleared.")
        try:
            baud = int(self.baud.get())
        except ValueError:
            baud = 115200
        self.stop_event.clear()
        self._start_link(port, "serial", self._serial_opener(port, baud),
                         LOG_DIR / f"serial_{_safe(port, '')}_{stamp}.bin", stream, auto=True)
        for cell in (self.port_a, self.port_b):
            if cell.get() not in self.serials or cell.get() == port:
                cell.set(port)
                break
        self.connect_text.set("Disconnect")
        self.refresh_command_targets()
        self._refresh_status_text()
        self.console_write(f"AUTO_CONNECT[{port}] connected")

    # ----------------------------------------------------------- commands
    def refresh_command_targets(self):
        targets = () if self.test_active else (*self.serials, *self.tcp_sockets)
        self.command_targets.set(tuple(targets))
        if self.command_target.get() not in targets:
            self.command_target.set(targets[0] if targets else "")
        self.commands_enabled.set(bool(targets))
        self._refresh_reset_enabled()

    def witness_flight_state(self):
        """Flight state (0..4) from the newest Witness status word seen, or None."""
        best = None
        for name in WITNESS_STATE_PACKETS:
            value = self.raw_values.get(f"packet.{name}.status")
            when = self.last_packet_time.get(f"packet.{name}")
            if isinstance(value, int) and when is not None and (best is None or when > best[0]):
                best = (when, value & FLIGHT_STATE_MASK)
        return None if best is None else best[1]

    def _refresh_reset_enabled(self):
        self.reset_witness_enabled.set(bool(self.commands_enabled.get())
                                       and self.witness_flight_state() == LANDED_STATE)
        state = self.witness_flight_state()
        self.wifi_on_enabled.set(bool(self.commands_enabled.get())
                                 and state in (None, PAD_IDLE_STATE, LANDED_STATE))
        self.bit_run_enabled.set(bool(self.commands_enabled.get()) and state == PAD_IDLE_STATE)
        self.bit_detail_enabled.set(bool(self.commands_enabled.get()) and state != BOOST_STATE)

    def _note_bit(self, packet, received_mono):
        """Every BIT summary/detail packet, not only the newest per tick: the three detail pages of a
        run arrive together."""
        if self.bit.feed(packet, received_mono):
            self._refresh_bit_text()

    def _refresh_bit_text(self, now=None):
        shown = self.bit.status(time.monotonic() if now is None else now)
        if shown is None:
            return
        text, kind, tooltip = shown
        self.bit_text.set(text)
        self.bit_kind.set(kind)
        self.bit_tooltip.set(tooltip)

    def _target_writer(self, target_name):
        """(write, link) for a connected target, or (None, None). Writes through a
        Link are serialized, time-limited and drop the connection on failure."""
        link = self.links.get(target_name)
        if link is not None and link.stream is not None:
            return link.write, link
        serial_port = self.serials.get(target_name)
        if serial_port is not None:
            return serial_port.write, None
        tcp_socket = self.tcp_sockets.get(target_name)
        if tcp_socket is not None:
            return tcp_socket.sendall, None
        return None, None

    def _no_target_message(self, target_name):
        link = self.links.get(target_name)
        if link is not None and link.stream is None:
            return f"TX_ERROR {target_name} is reconnecting; nothing sent"
        return "TX_ERROR no connected serial/TCP target selected"

    def transmit_command(self, target_name, command, description=None):
        if self.test_active:
            self.console_write("TX_BLOCKED test mode never sends commands to hardware")
            return False
        write, link = self._target_writer(target_name)
        if write is None:
            self.console_write(self._no_target_message(target_name))
            return False
        is_bit_run = len(command) == 10 and (command[7] << 8 | command[8]) == BIT_RUN_COMMAND
        # A BIT run is acknowledged only when it ends, so it is NOT resent (a resend would start a
        # second run): one write, then wait for the ack.
        acked = CommandLink.is_acked_command(command) and not is_bit_run
        try:
            if acked or is_bit_run:
                self.command_link.forget(command)
            write(command)
            sent_text = command.hex(" ")
            label = f" {description}:" if description else ""
            self.console_write(f"TX[{target_name}]{label}  {sent_text}")
            self.command_history.append(sent_text)
            self.command_history_index = len(self.command_history)
            self.command_history_draft = ""
        except Exception as exc:
            self.console_write(f"TX_ERROR[{target_name}] {exc}")
            return False
        if len(command) == 10 and (command[7] << 8 | command[8]) == RESET_TO_PAD_OPCODE:
            # Witness acks, then reboots (Landed only): the link drops for a few
            # seconds and returns in Pad Idle. Expected, not an error.
            self.expected_reset_until = time.monotonic() + RESET_EXPECTED_S
        if is_bit_run:
            cancel = link.cancel if link is not None else self.stop_event
            threading.Thread(target=self._await_bit_run_ack, name="bit-run-ack", daemon=True,
                             args=(target_name, command, description or "BIT run", cancel)).start()
        if acked:
            # Radio round trips (Aperture -> LoRa -> Witness -> back) take far
            # longer than USB, and the uplink is half duplex: resend until
            # Witness acknowledges, but never faster than COMMAND_RESEND_S.
            cancel = link.cancel if link is not None else self.stop_event
            threading.Thread(target=self._await_ack, name="command-ack", daemon=True,
                             args=(target_name, write, command, description or sent_text,
                                   COMMAND_RESEND_S, cancel)).start()
        return True

    def _await_bit_run_ack(self, target_name, command, label, cancel):
        try:
            ack = self.command_link.send_reliable(None, command, timeout=BIT_RUN_ACK_WAIT_S, attempts=1,
                                                  cancel=cancel, first_sent=True)
        except Exception as exc:  # noqa: BLE001
            self.console_write(f"TX_ERROR[{target_name}] {label}: {exc}")
            return
        if self.stop_event.is_set():
            return
        if ack is None:
            self.console_write(f"NO_ACK[{target_name}] {label}: no result within {BIT_RUN_ACK_WAIT_S:g} s "
                               "(not resent: check the BIT status, or use BIT detail)")
            return
        result, detail = ack.get("result"), ack.get("detail") or 0
        verdict, groups = bit_model.ack_verdict(detail)
        if result == 0:
            self.console_write(f"ACK[{target_name}] {label}: {verdict}, {groups} blocking group(s) failed")
        elif result == 4:
            self.console_write(f"BUSY[{target_name}] {label}: a BIT run is already in progress")
        elif result == 1:
            self.console_write(f"REFUSED[{target_name}] {label}: Witness refused (detail {detail:#06x}; "
                               "0x8000 + reason = test gate: not still, state changed or moving)")
        else:
            self.console_write(f"NAK[{target_name}] {label}: {ack.get('result_name')} (detail {detail})")

    def _await_ack(self, target_name, write, command, label, timeout, cancel=None):
        cancel = self.stop_event if cancel is None else cancel
        try:
            ack = self.command_link.send_reliable(write, command, timeout=timeout, attempts=self.ack_attempts,
                                                  cancel=cancel, first_sent=True)
        except Exception as exc:  # noqa: BLE001
            self.console_write(f"TX_ERROR[{target_name}] {label}: resend failed: {exc}")
            return
        if self.stop_event.is_set():
            return
        if (ack is not None and ack.get("result") == 4 and ack.get("detail") == BUSY_CONFIRM_DETAIL
                and not cancel.is_set()):
            # Radio erase/clear needs the same command twice within 10 s. The operator
            # already confirmed in the dialog, so send it again once, ~1 s later.
            self.console_write(f"CONFIRM[{target_name}] {label}: Witness asks for the command again "
                               f"(radio safety); sending it again in {BUSY_CONFIRM_GAP_S:g} s")
            if self.stop_event.wait(BUSY_CONFIRM_GAP_S) or cancel.is_set():
                return
            try:
                self.command_link.forget(command)
                write(command)
                ack = self.command_link.send_reliable(write, command, timeout=timeout,
                                                      attempts=self.ack_attempts, cancel=cancel,
                                                      first_sent=True)
            except Exception as exc:  # noqa: BLE001
                self.console_write(f"TX_ERROR[{target_name}] {label}: confirm resend failed: {exc}")
                return
            if self.stop_event.is_set():
                return
        if ack is None and cancel.is_set():
            self.console_write(f"NO_ACK[{target_name}] {label}: link dropped before the acknowledgement "
                               "(no more resends; check the log/state after it reconnects)")
        elif ack is None:
            self.console_write(f"NO_ACK[{target_name}] {label}: no acknowledgement after "
                               f"{self.ack_attempts} tries (link down or Witness off?)")
        elif ack.get("result") == 0:
            self.console_write(f"ACK[{target_name}] {label}: ok")
        elif ack.get("result") == 4 and ack.get("detail") in APERTURE_BUSY_DETAILS:
            self.console_write(f"NOT_SENT[{target_name}] {label}: "
                               f"{APERTURE_BUSY_DETAILS[ack.get('detail')]}; Witness never got it. RETRY")
        elif ack.get("result") == 4:
            # Busy: e.g. the camera/load CAN queue was full. Nothing was done: retry.
            self.console_write(f"BUSY[{target_name}] {label}: Witness is busy (detail {ack.get('detail'):#06x}); "
                               "RETRY the command")
        else:
            self.console_write(f"NAK[{target_name}] {label}: {ack.get('result_name')} "
                               f"(detail {ack.get('detail')})")

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
        if opcode == RESET_TO_PAD_OPCODE:
            state = self.witness_flight_state()
            if state != LANDED_STATE:
                shown = "no state telemetry yet" if state is None else f"state {state}, not Landed"
                self.console_write(f"TX_BLOCKED {name}: Witness is not Landed ({shown}); "
                                   "it only accepts this in Landed")
                return False
        text = CONFIRM_TEXT.get(opcode, "delete all stored log sessions? This cannot be undone.")
        if confirm and not self.confirm("Confirm command", f"{name}: {text}"):
            return False
        return self.transmit_command(self.command_target.get(), build_command_packet(opcode), name)

    WIFI_ON_CONFIRM = ("Witness restarts into its maintenance image: flight logging OFF, not "
                       "flight-ready, until Wi-Fi is turned off or it resets. Only accepted in "
                       "Pad Idle or Landed.")

    def send_wifi(self, on):
        """Wi-Fi ON (maintenance) asks first; OFF (back to flight) does not. Same send path
        (target, ack resend) as every other command button."""
        from witness_protocol import build_wifi_packet
        if on and not self.confirm("Confirm command", f"Wi-Fi ON (maintenance): {self.WIFI_ON_CONFIRM}"):
            return False
        name = "Wi-Fi ON (maintenance)" if on else "Wi-Fi OFF (back to flight)"
        return self.transmit_command(self.command_target.get(), build_wifi_packet(bool(on)), name)

    def send_bit_run(self):
        """Run BIT button: 0x5442 (full run, no options) after a confirmation, Pad Idle only.
        Called by nothing but that button."""
        state = self.witness_flight_state()
        if state != PAD_IDLE_STATE:
            shown = "no state telemetry yet" if state is None else f"state {state}, not Pad Idle"
            self.console_write(f"TX_BLOCKED Run BIT: Witness is not in Pad Idle ({shown})")
            return False
        if not self.confirm("Confirm command", f"Run BIT: {CONFIRM_TEXT[BIT_RUN_COMMAND]}"):
            return False
        nonce = int(time.time() * 1000) & 0xFFFFFFFF
        return self.transmit_command(self.command_target.get(), build_bit_run_packet(nonce), "Run BIT")

    def request_bit_report(self):
        """BIT detail button: ask Witness to re-send the detail pages not held yet, or the summary when
        all are held (0x5452; it never runs a test). One request at a time on a worker thread: each
        page is awaited (or times out) before the next goes out, so a radio uplink is not flooded."""
        state = self.witness_flight_state()
        if state == BOOST_STATE:
            self.console_write("TX_BLOCKED BIT detail: Witness refuses reports in Boost")
            return False
        if self.test_active:
            self.console_write("TX_BLOCKED test mode never sends commands to hardware")
            return False
        target_name = self.command_target.get()
        write, link = self._target_writer(target_name)
        if write is None:
            self.console_write(self._no_target_message(target_name))
            return False
        if not self._bit_report_busy.acquire(blocking=False):
            self.console_write("BIT detail: a request is already in progress")
            return False
        cancel = link.cancel if link is not None else self.stop_event
        threading.Thread(target=self._bit_report_worker, name="bit-report", daemon=True,
                         args=(target_name, cancel)).start()
        return True

    def _bit_report_worker(self, target_name, cancel):
        try:
            pages = self.bit.missing_pages()
            if self.bit.summary is None:
                requests = [BIT_REPORT_SUMMARY, *pages]
            else:
                requests = pages or [BIT_REPORT_SUMMARY]
            for page in requests:
                if cancel.is_set() or self.stop_event.is_set():
                    return
                name = "BIT summary" if page == BIT_REPORT_SUMMARY else f"BIT detail page {page}"
                if not self.transmit_command(target_name, build_bit_report_packet(page), name):
                    return
                if page != BIT_REPORT_SUMMARY:
                    deadline = time.monotonic() + BIT_DETAIL_PAGE_WAIT_S
                    while page in self.bit.missing_pages() and time.monotonic() < deadline:
                        if cancel.wait(0.1):
                            return
        finally:
            self._bit_report_busy.release()

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
        write, link = self._target_writer(target_name)
        if write is None:
            self.console_write(self._no_target_message(target_name))
            return False
        is_serial = link.kind == "serial" if link is not None else target_name in self.serials
        pace = pace_s if pace_s is not None else (0.003 if is_serial else 0.06)
        frames = list(frames) + [gps_assist.status_frame()]

        def worker():
            try:
                gps_assist.send_frames(write, frames, pace)    # a dead link raises (Link.write)
                self.console_write(f"TX[{target_name}] {description}: {len(frames)} frames sent; "
                                   "Witness logs 'GPS assist done: N/N acknowledged'")
            except Exception as exc:  # noqa: BLE001
                self.console_write(f"TX_ERROR[{target_name}] {description}: {exc}")
        threading.Thread(target=worker, name="gps-assist", daemon=True).start()
        return True

    def upload_gps_assist(self, blob, description="GPS ephemeris upload", transfer_id=1, pace_s=None,
                          resume_deadline_s=600.0):
        """Acknowledged, resumable GPS-assist upload (command_link): BEGIN,
        chunks, STATUS bitmap, resend missing, COMMIT. Survives link drops by
        waiting (up to resume_deadline_s) and resuming where Witness left off."""
        target_name = self.command_target.get()
        if self.test_active:
            self.console_write("TX_BLOCKED test mode never sends commands to hardware")
            return False
        write, link = self._target_writer(target_name)
        if write is None:
            self.console_write(self._no_target_message(target_name))
            return False
        is_serial = link.kind == "serial" if link is not None else target_name in self.serials
        pace = pace_s if pace_s is not None else (0.003 if is_serial else 0.06)
        timeout = COMMAND_RESEND_S          # never resend faster than this on a half-duplex uplink
        cancel = link.cancel if link is not None else self.stop_event   # also stops when the link drops
        last = [0.0, None]

        def progress(received, expected, note):
            now = time.monotonic()
            if note != last[1] or now - last[0] > 5.0:
                got = "?" if received is None else received
                self.console_write(f"TX[{target_name}] {description}: {got}/{expected} chunks ({note})")
                last[0], last[1] = now, note

        def worker():
            try:
                ok, message = self.command_link.upload_assist(
                    write, blob, transfer_id=transfer_id, pace_s=pace, timeout=timeout,
                    status_timeout=ASSIST_STATUS_TIMEOUT_S if not is_serial else None,
                    resume_deadline_s=resume_deadline_s, progress=progress, cancel=cancel)
            except Exception as exc:  # noqa: BLE001
                ok, message = False, str(exc)
            if not self.stop_event.is_set():
                tag = "TX" if ok else "TX_ERROR"
                self.console_write(f"{tag}[{target_name}] {description}: {message}")
        self.console_write(f"TX[{target_name}] {description}: {len(blob)} bytes, acknowledged upload started")
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
        if self.serials or self.tcp_sockets or self.links or self._auto_job is not None:
            self.auto_connect.set(False)
            self.disconnect()
        else:
            self.connect()

    @staticmethod
    def _serial_opener(port, baud):
        def open_port():
            return open_serial_quiet(port, baud, write_timeout=SERIAL_WRITE_TIMEOUT_S)
        return open_port

    @staticmethod
    def _tcp_opener(host, port):
        def open_socket():
            sock = socket.create_connection((host, port), timeout=3.0)
            sock.settimeout(0.2)
            # Half-open links (Wi-Fi drop without FIN) die within seconds, not hours.
            enable_keepalive(sock, idle_ms=3000, interval_ms=1000)
            return sock
        return open_socket

    def _open_debug_log(self, stamp):
        if self.debug_log_file is not None:
            self.debug_log_file.close()          # connect() used to leak the previous handle
        self.debug_log_file = DebugLog(LOG_DIR / f"debug_{stamp}.log")

    def _start_link(self, name, kind, opener, log_path, stream, *, auto=False, label=None):
        raw = RawLog(log_path)
        link = Link(name, kind, opener, raw, stream=stream, auto=auto)
        link.status.label = label or name
        link.status.connected()
        self.links[name] = link
        self.log_files[name] = raw
        (self.serials if kind == "serial" else self.tcp_sockets)[name] = stream
        reader = self.read_serial if kind == "serial" else self.read_tcp
        link.thread = threading.Thread(target=reader, args=(name, stream), name=f"link-{name}", daemon=True)
        link.thread.start()
        self._ensure_sync_thread()
        return link

    def _remove_link(self, name, why=""):
        """Forget a link for good (operator Disconnect, or it can never return)."""
        link = self.links.pop(name, None)
        self.serials.pop(name, None)
        self.tcp_sockets.pop(name, None)
        if link is not None:
            link.stop.set()
            link.down.set()
            link.drop_stream()
            link.raw_log.close()
        raw = self.log_files.pop(name, None)
        if raw is not None:
            raw.close()
        self._close_nmea_file(name)
        if why:
            self.console_write(f"LINK[{name}] removed: {why}")
        self.refresh_command_targets()

    def _ensure_sync_thread(self):
        """fsync the raw logs every RAW_FSYNC_S from a separate thread (never the reader)."""
        if self._sync_thread is not None:
            return
        self._sync_stop = threading.Event()

        def run():
            while not self._sync_stop.wait(RAW_FSYNC_S):
                for raw in list(self.log_files.values()):
                    try:
                        raw.fsync()
                    except Exception:  # noqa: BLE001
                        pass
        self._sync_thread = threading.Thread(target=run, name="rawlog-fsync", daemon=True)
        self._sync_thread.start()

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
        self._open_debug_log(stamp)
        self.stop_event.clear()
        try:
            for port in ports:
                serial_port = open_serial_quiet(port, baud, write_timeout=SERIAL_WRITE_TIMEOUT_S,
                                                serial_module=serial)
                self._start_link(port, "serial", self._serial_opener(port, baud),
                                 LOG_DIR / f"serial_{_safe(port, '')}_{stamp}.bin", serial_port)
            self.refresh_command_targets()
            for _index, tcp_host, tcp_port in tcp_endpoints:
                tcp_socket = self._tcp_opener(tcp_host, tcp_port)()
                tcp_source = f"TCP_{tcp_host}_{tcp_port}"
                self._start_link(tcp_source, "tcp", self._tcp_opener(tcp_host, tcp_port),
                                 LOG_DIR / f"{_safe(tcp_source, '')}_{stamp}.bin", tcp_socket,
                                 label=f"TCP {tcp_host}:{tcp_port}")
            self.refresh_command_targets()
        except Exception as exc:
            self.disconnect()
            self.alert("Serial monitor", f"Could not open serial port: {exc}")
            return False
        self.connect_text.set("Disconnect")
        self._refresh_status_text()
        return True

    def disconnect(self):
        """Operator Disconnect (or shutdown): the ONLY thing that stops reconnecting."""
        if self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
        self.stop_event.set()
        for link in list(self.links.values()):
            link.stop.set()
            link.down.set()
            link.drop_stream()
        for stream in (*self.serials.values(), *self.tcp_sockets.values()):
            try:
                stream.close()
            except Exception:
                pass
        self.serials.clear()
        self.tcp_sockets.clear()
        self.links.clear()
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
        self.link_banner.set("")
        self.link_summary.set("")
        self.link_snapshots = []
        self.expected_reset_until = 0.0
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
        self._refresh_reset_enabled()
        self.graphs_dirty = True

    def start_test(self):
        if self.test_active:
            return False
        if self.serials or self.tcp_sockets or self.links:
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
            self.debug_log_file = DebugLog(LOG_DIR / f"test_debug_{stamp}.log")
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
        link = getattr(self, "command_link", None)
        if link is not None and test_token is None and isinstance(packet, dict):
            try:
                link.feed(packet)     # a failure here must not lose the packet for the displays
            except Exception as exc:  # noqa: BLE001
                self.events.put(("decoder_error", (source, f"command link: {exc}")))
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
        # Decoder messages are coalesced to one summary line per second per
        # source (the audit saw 3 M one-byte BAD_PACKET lines in 485 MB of log).
        coalescer = Coalescer(1.0)

        def on_error(message):
            summary = coalescer.add(message)
            if summary:
                self.events.put(("decoder_error", (source, summary)))
        decoder = PacketDecoder(
            lambda p: self.on_decoded_packet(source, p), on_error,
            lambda packet_id, count: self.events.put(("host_packet", (source, packet_id, count))),
        )
        decoder.coalescer = coalescer
        return decoder

    def _flush_decoder_errors(self, source, decoder):
        summary = decoder.coalescer.flush_due()
        if summary:
            self.events.put(("decoder_error", (source, summary)))

    def _log_input(self, source, data):
        raw = self.log_files.get(source)
        if raw is not None:
            raw.write(data)       # RawLog.write never raises

    def _link_for(self, name, kind, stream):
        link = self.links.get(name)
        if link is None or link.stream is not stream:
            # A caller that registered the stream itself (no reconnect possible).
            link = Link(name, kind, None, self.log_files.get(name), stream=stream)
            link.status.connected()
        return link

    def read_serial(self, port_name, serial_port):
        self._run_link(self._link_for(port_name, "serial", serial_port))

    def read_tcp(self, source_name, tcp_socket):
        self._run_link(self._link_for(source_name, "tcp", tcp_socket))

    def _run_link(self, link):
        """Reader thread of one input. Ends ONLY on operator Disconnect (link.stop);
        any other failure re-opens the same source with backoff (0.5, 1, 2, 4, 5 s ... forever)."""
        name, status = link.name, link.status
        attempt = 0
        try:
            while not link.stop.is_set():
                stream = link.stream
                if stream is None:
                    attempt += 1
                    status.lost(link.last_reason, attempt)
                    if link.stop.wait(backoff_delay(attempt)):
                        break
                    if link.raw_log is not None:
                        link.raw_log.tick()
                    try:
                        stream = link.opener()
                    except Exception as exc:  # noqa: BLE001 - port busy / gone / host down: retry
                        link.last_reason = f"{type(exc).__name__}: {exc}"
                        continue
                    if link.stop.is_set():
                        try:
                            stream.close()
                        except Exception:  # noqa: BLE001
                            pass
                        break
                    link.stream = stream
                    status.connected()
                    link.down.clear()
                    attempt = 0
                    self.events.put(("link_up", (link, stream)))
                reason = self._read_loop(link, stream)
                link.down.set()
                try:
                    stream.close()
                except Exception:  # noqa: BLE001
                    pass
                if link.stream is stream:
                    link.stream = None
                if link.stop.is_set():
                    break
                link.last_reason = reason
                status.lost(reason, 1)
                if link.opener is None:      # cannot reconnect: legacy behaviour
                    self.events.put(("error", (name, reason, stream)))
                    break
                self.events.put(("link_down", (link, stream, reason)))
        except Exception as exc:  # noqa: BLE001 - a bug here must be loud, never a silent dead input
            self.events.put(("error", (name, f"link supervisor failed: {exc!r}", link.stream)))

    def _read_loop(self, link, stream):
        """Read one connection until it fails; returns the reason. The raw log is
        written FIRST and independently of decoding: a decode exception never
        stops the log, and a log OSError never stops the reader or the decoder."""
        name, status, tcp = link.name, link.status, link.kind == "tcp"
        nmea = NmeaExtractor()
        decoder = self._decoder(name)
        while not link.stop.is_set():
            try:
                if tcp:
                    try:
                        data = stream.recv(4096)
                    except socket.timeout:
                        data = None
                    else:
                        if not data:
                            return "TCP server closed the connection"
                else:
                    data = stream.read(stream.in_waiting or 1)
            except Exception as exc:  # noqa: BLE001
                return f"{type(exc).__name__}: {exc}"
            if data:
                if link.raw_log is not None:
                    link.raw_log.write(data)
                status.note_bytes(len(data))
                self.events.put(("bytes", (name, bytes(data))))
                try:
                    decoder.feed(data)
                except Exception as exc:  # noqa: BLE001 - PacketDecoder should not raise; belt and braces
                    self.events.put(("decoder_error", (name, f"decoder failure: {exc!r}")))
                    decoder.buffer.clear()
                try:
                    self._queue_nmea(name, nmea, data)
                except Exception as exc:  # noqa: BLE001
                    self.events.put(("decoder_error", (name, f"NMEA extractor failure: {exc!r}")))
                status.bad_bytes = decoder.garbage_bytes_skipped
                status.text_bytes = decoder.text_bytes_skipped
            else:
                if link.raw_log is not None:
                    link.raw_log.tick()
            self._flush_decoder_errors(name, decoder)
            if tcp and status.conn_received and status.last_byte is not None:
                now = time.monotonic()
                silent = now - status.last_byte
                # Half-open link (no FIN, e.g. Wi-Fi drop): force a reconnect. Not
                # while Witness is expected to be rebooting (0x7300). Serial ports
                # are NOT cycled on silence: reopening may toggle DTR/RTS.
                if silent > TCP_NO_DATA_RECONNECT_S and now > self.expected_reset_until:
                    return f"no data for {silent:.1f} s (half-open link?)"
        return "stopped"

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
            try:
                decoder.feed(data)
                self._queue_nmea(source_name, nmea, data)
            except Exception as exc:  # noqa: BLE001 - the HITL reader thread must survive
                self.events.put(("decoder_error", (source_name, f"mirror decode failure: {exc!r}")))
            self._flush_decoder_errors(source_name, decoder)

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
                self._pump_error(f"event {kind}", exc, console_lines)
        for port_name in ports_seen:
            try:
                if port_name in self.serials:
                    self.discovery.remember(port_name)
            except Exception as exc:  # noqa: BLE001 - e.g. read-only learned-device file
                self._pump_error("discovery.remember", exc, console_lines)
        for _port_name, packet in latest.values():
            try:
                self.update_values("packet", packet, record=False)
            except Exception as exc:
                self._pump_error("update_values", exc, console_lines)
        try:
            self._write_console_lines(console_lines)
        except Exception as exc:  # noqa: BLE001 - must never escape into the Qt slot
            self._pump_error("console", exc, None)
        try:
            self._refresh_nmea_status()
        except Exception as exc:  # noqa: BLE001
            self._pump_error("nmea status", exc, None)
        now = time.monotonic()
        if now - self._last_health >= 0.25:
            self._last_health = now
            try:
                self._refresh_bit_text(now)       # the summary ages: STALE after 5 minutes
            except Exception as exc:  # noqa: BLE001
                self._pump_error("bit status", exc, None)
            try:
                self.update_link_health(now)
            except Exception as exc:  # noqa: BLE001
                self._pump_error("link health", exc, None)
        return handled

    def _pump_error(self, where, exc, console_lines):
        """Rate-limited report of a failure in the GUI-thread pump (one line / 5 s per kind)."""
        allowed, skipped = self._pump_errors.allow((where, type(exc).__name__))
        if allowed:
            more = f" ({skipped} similar suppressed)" if skipped else ""
            line = self._console_stamp(f"GUI_ERROR[{where}] {type(exc).__name__}: {exc}{more}")
            if console_lines is not None:
                console_lines.append(line)
            else:
                self.console_pending.append(line)

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
            if packet.get("packet") in ("bit_summary", "bit_detail"):
                self._note_bit(packet, time.monotonic())
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
        elif kind == "link_up":
            link, stream = payload
            if self.links.get(link.name) is link and not link.stop.is_set():
                (self.serials if link.kind == "serial" else self.tcp_sockets)[link.name] = stream
                self.refresh_command_targets()
                self._refresh_status_text()
                console_lines.append(self._console_stamp(
                    f"LINK_UP[{link.name}] reconnected (reconnect #{link.status.reconnects}); "
                    "raw log continues in the same file"))
        elif kind == "link_down":
            link, stream, reason = payload
            inputs = self.serials if link.kind == "serial" else self.tcp_sockets
            if inputs.get(link.name) is stream:
                inputs.pop(link.name)
            self.refresh_command_targets()
            self._refresh_status_text()
            expected = " (expected: Witness is restarting)" if time.monotonic() < self.expected_reset_until else ""
            console_lines.append(self._console_stamp(
                f"LINK_DOWN[{link.name}] {reason}{expected}; reconnecting automatically"))
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
        stamp = _dt.datetime.fromtimestamp(received_at, _dt.timezone.utc).isoformat(timespec="milliseconds")
        try:
            self._nmea_file(source).write(f"{stamp},{int(ok)},{sentence}\n")
        except (OSError, ValueError) as exc:     # the NMEA csv is secondary: count it, keep going
            self.nmea_log_errors += 1
            self._nmea_files_drop(source)
            allowed, _ = self._pump_errors.allow(("nmea csv", type(exc).__name__))
            if allowed:
                self.console_write(f"NMEA_LOG_ERROR {exc} ({self.nmea_log_errors} sentence(s) not logged)")
        self._nmea_dirty = True
        if ok:
            gga = summarize_gga(sentence)
            if gga:
                self.nmea_last_gga[source] = gga

    def _nmea_files_drop(self, source):
        handle = self.nmea_files.pop(source, None)
        if handle is not None:
            try:
                handle.close()
            except Exception:  # noqa: BLE001
                pass

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
        for source, handle in list(self.nmea_files.items()):
            try:
                handle.flush()
            except (OSError, ValueError):
                self._nmea_files_drop(source)

    def _close_nmea_file(self, source):
        handle = self.nmea_files.pop(source, None)
        if handle is not None:
            handle.close()

    def _handle_input_error(self, payload):
        """An input that cannot reconnect (no opener) or whose supervisor failed."""
        port_name, message, stream = payload
        inputs = self.serials if port_name in self.serials else self.tcp_sockets
        if inputs.get(port_name) is not stream and port_name not in self.links:
            return  # A late failure from a replaced/disconnected worker.
        inputs.pop(port_name, None)
        try:
            if stream is not None:
                stream.close()
        except Exception:  # noqa: BLE001
            pass
        link = self.links.pop(port_name, None)
        if link is not None:
            link.stop.set()
            link.down.set()
        log_file = self.log_files.pop(port_name, None)
        if log_file is not None:
            log_file.close()
        self._close_nmea_file(port_name)
        self.refresh_command_targets()
        if not self.serials and not self.tcp_sockets and not self.links:
            self.connect_text.set("Connect")
        self.status_text.set(f"Serial error on {port_name}: {message}")
        self.console_write(f"ERROR[{port_name}] {message}")
        if not self.serials and not self.tcp_sockets and not self.links and self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None

    # -------------------------------------------------------- link health
    def _refresh_status_text(self):
        """Status line from the links: "Connected: A, B; Reconnecting to X (attempt n)"."""
        if not self.links:
            return
        snaps = [link.status.snapshot() for link in self.links.values()]
        up = [link.status.label for link, snap in zip(self.links.values(), snaps)
              if snap["state"] not in ("reconnecting", "connecting")]
        parts = ([f"Connected: {', '.join(up)}"] if up else [])
        parts += [snap["text"] for snap in snaps if snap["state"] == "reconnecting"]
        self.status_text.set("; ".join(parts))

    def update_link_health(self, now=None):
        """Refresh per-source link state, the LINK LOST banner and log warnings.
        Runs from process_events (GUI thread; throttled to 4 Hz). Read the result
        from link_snapshots / link_banner / link_summary / log_warning."""
        now = time.monotonic() if now is None else now
        links = list(self.links.values())
        snaps = [link.status.snapshot(now) for link in links]
        self.link_snapshots = snaps
        problems = [snap for snap in snaps if snap["state"] in ("reconnecting", "stale", "no data")]
        if problems and now < self.expected_reset_until:
            banner = ("WITNESS RESETTING - the link drops for a few seconds and returns in Pad Idle; "
                      "it reconnects by itself")
        elif problems:
            banner = "LINK LOST - " + "; ".join(snap["text"] for snap in problems)
        else:
            banner = ""
        self.link_banner.set(banner)
        self.link_summary.set(" | ".join(
            f"{link.status.label}: {snap['state']}"
            + ("" if snap["seconds_since_last_byte"] is None else f", last byte {snap['seconds_since_last_byte']:.1f} s ago")
            + f", {snap['bytes_per_s']:.0f} B/s, reconnects {snap['reconnects']}, bad bytes {snap['bad_bytes']}"
            for link, snap in zip(links, snaps)))
        warnings = [raw.warning for raw in self.log_files.values() if getattr(raw, "warning", "")]
        debug_warning = getattr(self.debug_log_file, "warning", "")
        if debug_warning:
            warnings.append(debug_warning)
        if self.events.dropped:
            warnings.append(f"{self.events.dropped} display event(s) dropped after a GUI stall "
                            "(raw logs are complete)")
        self.log_warning.set(" | ".join(warnings))
        self._refresh_status_text()

    def source_silence(self, source=None):
        """Seconds since the last byte for a source (None = no data yet / unknown)."""
        link = self.links.get(source) if source is not None else next(iter(self.links.values()), None)
        return None if link is None else link.status.silent_for()

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
        if mode in ("auto", "on"):
            # tailscale_ipv4() may run the CLI (3 s timeout): never on the GUI thread.
            if not self._probe_done:
                if self._probe_thread is None or not self._probe_thread.is_alive():
                    self._probe_thread = threading.Thread(target=self._probe_tailscale,
                                                          name="tailscale-probe", daemon=True)
                    self._probe_thread.start()
                self._remote_retry_at = time.monotonic() + 0.2     # collect the answer soon
                self.remote_feed_status.set("Remote feed (Tailscale): checking for Tailscale...")
                return False
            host, self._probe_done = self._probe_result, False
        else:
            host = mode
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

    def _probe_tailscale(self):
        try:
            self._probe_result = tailscale_ipv4()
        except Exception:  # noqa: BLE001
            self._probe_result = None
        self._probe_done = True

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
            self.feed_status.set(f"Horizon feed stopped: {self.publisher.error}; restarting")
        elif self.publisher.restarts:
            self.feed_status.set(
                f"Horizon feed: {self.publisher.host}:{self.publisher.port}; restarted "
                f"{self.publisher.restarts} time(s), last failure: {self.publisher.last_error}")
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
            try:
                self.debug_log_file.write("\n".join(lines) + "\n")
                self.debug_log_file.flush()
            except Exception:  # noqa: BLE001 - the debug log is optional; DebugLog.warning reports it
                pass
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
        if packet["packet"] in WITNESS_STATE_PACKETS:
            self._refresh_reset_enabled()

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
        stop = getattr(self, "_sync_stop", None)
        if stop is not None:
            stop.set()
        self.publisher.close()
        self.stop_remote_feed()


__all__ = [
    "MonitorCore", "RingHistory", "LOG_DIR", "NO_SERIAL_PORT", "WINDOW_TITLE", "BAUD_CHOICES",
    "QUICK_COMMANDS", "IMU_CALIBRATION_COMMANDS",
]
