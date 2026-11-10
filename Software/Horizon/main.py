"""Witness monitor and Horizon telemetry feed.

Offline demo: python main.py --test
Timed hex replay: python main.py --test-file test_data/example_packets.json
Use --test-speed 2 for 2x playback and --test-once to disable looping.
Test bytes pass through PacketDecoder; they are never written to hardware.
USB auto-connect is on by default. Witness-named devices are discovered; for
generic adapters select a port manually once. After a decoded packet, its USB
identity is saved in auto_connect_witness.json (not its changing COM number).
The Auto-connect USB checkbox stops discovery connections, not healthy inputs.
Manual Disconnect turns it off. Test mode always disables automatic USB opens.
"""

from __future__ import annotations

import datetime as _dt
import argparse
import json
import socket
import struct
import queue
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

from ground_settings import WITNESS_FEED_HOST, WITNESS_FEED_PORT
from witness_link import WitnessPublisher
from nmea_capture import NmeaExtractor, summarize_gga
from witness_test_source import builtin_scenario, load_scenario, play_scenario, validate_speed
from serial_discovery import RETRY_SECONDS, SerialDiscovery, SerialOpenJob
from viewer import decimate_minmax, elide, nice_ticks, tick_text


ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / "logs"

# Default network endpoints shown by the GUI at startup.
DEFAULT_TCP_1_HOST = ""
DEFAULT_TCP_1_PORT = ""
DEFAULT_TCP_2_HOST = ""
DEFAULT_TCP_2_PORT = ""
DEFAULT_WITNESS_FEED_HOST = WITNESS_FEED_HOST
DEFAULT_WITNESS_FEED_PORT = WITNESS_FEED_PORT
NO_SERIAL_PORT = "(no port)"

# Commands currently defined by comms.h. Most command packets are sent as:
# ID (0x05) | status (2) | uptime (4) | command (2) | EOF (0x0A).
# Every shortcut uses a full frame so it works directly or through Aperture.
COMMAND_PACKET_ID = 0x05
COMMAND_PACKET_EOF = 0x0A
QUICK_COMMANDS = (
    ("CH1 Off", 0x0030, False),
    ("CH1 On", 0x0031, False),
    ("CH2 Off", 0x0032, False),
    ("CH2 On", 0x0033, False),
    ("CH3 Off", 0x0034, False),
    ("CH3 On", 0x0035, False),
    ("5V Reg Off", 0x0050, False),
    ("5V Reg On", 0x0051, False),
    ("Witness Debug 10 Hz", 0x00E0, False),
    ("Iris Debug 10 Hz", 0x00E1, False),
    ("Iris Ping", 0x0022, False),
    ("USB Mass Storage", 0x6868, False),
    ("Erase Logs", 0x6869, True),
    ("State: Pad", 0x7300, False),
    ("State: Boost", 0x7301, False),
    ("State: Coast", 0x7302, False),
    ("State: Descent", 0x7303, False),
    ("State: Landed", 0x7304, False),
    # HITL: Witness accepts ENTER only over USB in Pad Idle and then uses host
    # 0x0C sensor packets. These buttons send one command each; the monitor
    # never sends HITL commands or 0x0C packets automatically.
    ("HITL Enter (USB, pad only)", 0x4849, False),
    ("HITL EXIT -> real sensors", 0x4858, False),
)

STATUS_FLAG_LABELS = (
    (0x01, "HB phase"),
    (0x02, "Low-g"),
    (0x04, "Gyro"),
    (0x08, "IMU temp"),
    (0x10, "Flash init FAIL"),
    (0x20, "High-g"),
    (0x40, "Barometer"),
    (0x80, "Logging FAIL"),
)
FLIGHT_STATE_LABELS = (
    (0x00, "Pad idle"),
    (0x01, "Boost"),
    (0x02, "Coast"),
    (0x03, "Descent"),
    (0x04, "Landed"),
)
# Witness status byte 1 (low byte of the combined 16-bit status value).
FLIGHT_STATE_MASK = 0x07
GPS_LOCKED_MASK = 0x08   # bit 3: fresh GPS fix
HITL_ACTIVE_MASK = 0x10  # bit 4: HITL active, sensor values are simulated
STATUS_BYTE1_FLAG_LABELS = (
    (GPS_LOCKED_MASK, "GPS lock"),
    (HITL_ACTIVE_MASK, "HITL (SIM)"),
)


def status_gps_locked(status: int) -> int:
    return 1 if status & GPS_LOCKED_MASK else 0


CAMERA_STATUS_FLAG_LABELS = (
    (0x01, "Current error"),
    (0x02, "Battery invalid"),
    (0x04, "Command flag"),
    (0x08, "Output 1"),
    (0x10, "Output 2"),
    (0x20, "Output 3"),
    (0x40, "5V regulator"),
    (0x80, "Reserved"),
)


def number(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=False)


def signed_number(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=True)


def signed_int24(data: bytes) -> int:
    value = int.from_bytes(data, byteorder="big", signed=False)
    return value - (1 << 24) if value & (1 << 23) else value


def float32(data: bytes) -> float:
    return struct.unpack(">f", data)[0]


def float_vector(data: bytes, count: int) -> list[float]:
    return [round(float32(data[i:i + 4]), 6) for i in range(0, count * 4, 4)]


# comms.h canonical field scales.
FILTERED_ACCEL_G_PER_COUNT = 0.000040
GYRO_DPS_PER_LSB = 0.070
BATTERY_V_PER_COUNT = 0.001
CURRENT_A_PER_COUNT = 0.02
GPS_DEGREES_PER_COUNT = 0.0001
LSM6_TEMPERATURE_OFFSET_C = 25.0
LSM6_TEMPERATURE_C_PER_COUNT = 1.0 / 256.0
MS5607_TEMPERATURE_C_PER_COUNT = 0.01
LSM6_LOW_ACCEL_G_PER_COUNT = 0.000488
LSM6_HIGH_ACCEL_G_PER_COUNT = 0.010417


def parse_packet(packet_id: int, payload: bytes):
    if packet_id == 0x01 and len(payload) in (21, 23):
        result = {"packet": "sensors", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "filtered_accel": [round(signed_int24(payload[i:i + 3]) * FILTERED_ACCEL_G_PER_COUNT, 6)
                          for i in (6, 9, 12)],
                "gyro": [round(signed_number(payload[i:i + 2]) * GYRO_DPS_PER_LSB, 3)
                         for i in (15, 17, 19)]}
        # The live stream currently contains two additional bytes on sensor
        # frames even though the checked-in comms.h totals 21 payload bytes.
        # Preserve them numerically until the producer/header discrepancy is
        # resolved, rather than rejecting an otherwise valid sensor sample.
        if len(payload) == 23:
            result["sensor_trailing_u16"] = number(payload[21:23])
        return result
    if packet_id == 0x02 and len(payload) in (63, 62):
        # 63-byte payload (65-byte frame) is current; 62 is the legacy form
        # without GPS satellites. See PacketDecoder for how they are told apart.
        result = {"packet": "state", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "orientation": float_vector(payload[6:22], 4),
                "position": float_vector(payload[22:34], 3),
                "velocity": float_vector(payload[34:46], 3),
                "latitude": round(signed_int24(payload[46:49]) * GPS_DEGREES_PER_COUNT, 4),
                "longitude": round(signed_int24(payload[49:52]) * GPS_DEGREES_PER_COUNT, 4),
                "gps_altitude": round(float32(payload[52:56]), 6),
                "kalman_altitude": round(float32(payload[56:60]), 6),
                "witness_battery_voltage": round(number(payload[60:62]) * BATTERY_V_PER_COUNT, 3)}
        if len(payload) == 63:
            result["gps_satellites"] = payload[62]
        return result
    if packet_id == 0x03 and len(payload) == 17:
        result = {"packet": "camera", "fc_status": number(payload[0:2]), "fc_uptime": number(payload[2:6]),
                "status": number(payload[6:8]), "uptime": number(payload[8:12]),
                "current_sense": [round(payload[i] * CURRENT_A_PER_COUNT, 2) for i in (12, 13, 14)],
                "iris_battery_voltage": round(number(payload[15:17]) * BATTERY_V_PER_COUNT, 3)}
        return result
    if packet_id == 0xFF and len(payload) == 6:
        return {"packet": "heartbeat", "status": number(payload[0:2]), "uptime": number(payload[2:6])}
    if packet_id == 0xF0 and len(payload) in (11, 27):
        from converter import parse_ground_station_status
        return parse_ground_station_status(payload)
    if packet_id == 0x05 and len(payload) == 8:
        return {"packet": "command", "status": number(payload[0:2]), "uptime": number(payload[2:6]),
                "command_value": number(payload[6:8])}
    if packet_id == 0xE0 and len(payload) == 101:
        return {
            "packet": "witness_debug", "status": number(payload[0:2]),
            "uptime": number(payload[2:6]), "mcu_temperature": signed_number(payload[6:7]),
            "witness_battery_voltage": round(number(payload[7:9]) * BATTERY_V_PER_COUNT, 3),
            "filtered_accel": [round(signed_int24(payload[i:i + 3]) * FILTERED_ACCEL_G_PER_COUNT, 6)
                               for i in (9, 12, 15)],
            "orientation": float_vector(payload[18:34], 4),
            "position": float_vector(payload[34:46], 3),
            "velocity": float_vector(payload[46:58], 3),
            "low_accel": [round(signed_number(payload[i:i + 2]) * LSM6_LOW_ACCEL_G_PER_COUNT, 6)
                          for i in (58, 60, 62)],
            "lsm6_temperature": round(LSM6_TEMPERATURE_OFFSET_C +
                                       signed_number(payload[64:66]) * LSM6_TEMPERATURE_C_PER_COUNT, 3),
            "high_accel": [round(signed_number(payload[i:i + 2]) * LSM6_HIGH_ACCEL_G_PER_COUNT, 6)
                           for i in (66, 68, 70)],
            "gyro": [round(signed_number(payload[i:i + 2]) * GYRO_DPS_PER_LSB, 3)
                     for i in (72, 74, 76)],
            "pressure": number(payload[78:81]),
            "ms5607_temperature": round(signed_number(payload[81:83]) * MS5607_TEMPERATURE_C_PER_COUNT, 2),
            "kalman_altitude": round(float32(payload[83:87]), 6),
            "latitude": round(signed_int24(payload[87:90]) * GPS_DEGREES_PER_COUNT, 4),
            "longitude": round(signed_int24(payload[90:93]) * GPS_DEGREES_PER_COUNT, 4),
            "gps_time": number(payload[93:97]),
            "gps_altitude": round(float32(payload[97:101]), 6),
        }
    if packet_id == 0xE1 and len(payload) == 12:
        return {
            "packet": "iris_debug", "status": number(payload[0:2]),
            "uptime": number(payload[2:6]), "mcu_temperature": signed_number(payload[6:7]),
            "iris_battery_voltage": round(number(payload[7:9]) * BATTERY_V_PER_COUNT, 3),
            "current_sense": [round(payload[i] * CURRENT_A_PER_COUNT, 2) for i in (9, 10, 11)],
        }
    return None


class PacketDecoder:
    """Decode the single ID-prefixed serial protocol from comms.h."""
    # Sensor frames in the supplied capture are 25 bytes total: ID + 23-byte
    # payload + EOF.  Keep the 21-byte header-defined form as a fallback so
    # older producers remain readable while the firmware/header are reconciled.
    # State: 63-byte payload (65-byte frame, with GPS satellites) is current;
    # 62 bytes is the legacy 64-byte frame from older firmware/logs.
    payload_lengths = {0x01: (23, 21), 0x02: (63, 62), 0x03: (17,), 0xFF: (6,),
                       0xF0: (27, 11), 0x05: (8,), 0xE0: (101,), 0xE1: (12,)}
    # Host -> Witness packets that Witness may echo on USB. They are deframed
    # (fixed length + EOF) and skipped, never decoded or shown as telemetry.
    # 0x0C = HITL sensor input, 47-byte frame.
    host_payload_lengths = {0x0C: (45,)}
    # Packets whose shorter (legacy) payload is accepted only once the longer
    # candidate's EOF position has arrived and is not 0x0A. A state frame
    # with 10 satellites has 0x0A at frame offset 63, so a split read must not
    # be mistaken for a complete legacy frame.
    confirm_shorter = frozenset((0x02,))
    eof = 0x0A

    def __init__(self, on_packet, on_error=None, on_host_packet=None):
        self.buffer = bytearray()
        self.on_packet = on_packet
        self.on_error = on_error or (lambda _message: None)
        self.on_host_packet = on_host_packet or (lambda _packet_id, _count: None)
        self.frame_lengths = {**self.payload_lengths, **self.host_payload_lengths}
        self.host_packet_count = 0

    def find_valid_frame(self, start=0):
        """Return the next offset whose fixed-length frame has a valid EOF."""
        for offset in range(start, len(self.buffer)):
            packet_id = self.buffer[offset]
            payload_lengths = self.frame_lengths.get(packet_id)
            if payload_lengths is None:
                continue
            for payload_length in payload_lengths:
                eof_index = offset + 1 + payload_length
                if eof_index < len(self.buffer) and self.buffer[eof_index] == self.eof:
                    return offset
        return None

    def frame_payload_length(self, packet_id: int):
        """Return the matching payload length, or None if more bytes are needed."""
        payload_lengths = self.frame_lengths.get(packet_id)
        if payload_lengths is None:
            return None
        for index, payload_length in enumerate(payload_lengths):
            eof_index = 1 + payload_length
            if len(self.buffer) > eof_index and self.buffer[eof_index] == self.eof:
                if index and packet_id in self.confirm_shorter:
                    longest = 1 + max(payload_lengths)
                    if len(self.buffer) <= longest:
                        return None  # Wait: the longer form may still complete.
                return payload_length
        return None

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while self.buffer:
            try:
                packet_id = self.buffer[0]
                if packet_id not in self.frame_lengths:
                    next_frame = self.find_valid_frame(1)
                    discard_count = next_frame if next_frame is not None else 1
                    self.on_error(
                        f"unknown packet ID 0x{packet_id:02x}; discarded {discard_count} byte(s)"
                    )
                    del self.buffer[:discard_count]
                    continue

                payload_length = self.frame_payload_length(packet_id)
                if payload_length is None:
                    # Do not reject a split frame. For the sensor packet this
                    # also allows us to wait for the longer observed form
                    # before deciding that its EOF is bad.
                    maximum_length = max(self.frame_lengths[packet_id])
                    if len(self.buffer) < 1 + maximum_length + 1:
                        return
                    payload_length = maximum_length
                frame_length = 1 + payload_length + 1
                if len(self.buffer) < frame_length:
                    return
                frame = bytes(self.buffer[1:1 + payload_length])
                eof_byte = self.buffer[1 + payload_length]
                del self.buffer[:frame_length]
                if eof_byte != self.eof:
                    next_frame = self.find_valid_frame(1)
                    self.on_error(
                        f"bad EOF for 0x{packet_id:02x}: expected 0x0a, got 0x{eof_byte:02x}; "
                        f"resync={'offset ' + str(next_frame) if next_frame is not None else 'discard 1 byte'}"
                    )
                    if next_frame is not None:
                        del self.buffer[:next_frame]
                    else:
                        # The current frame is invalid, but leave later bytes
                        # buffered so a split TCP read can complete a frame.
                        del self.buffer[:1]
                    continue
                if packet_id in self.host_payload_lengths:
                    self.host_packet_count += 1
                    self.on_host_packet(packet_id, self.host_packet_count)
                    continue
                parsed = parse_packet(packet_id, frame)
                if parsed:
                    self.on_packet(parsed)
            except Exception as exc:
                bad = bytes(self.buffer[: min(len(self.buffer), 32)])
                self.on_error(f"decoder rejected packet: {exc}; buffer={bad.hex(' ')}")
                # Always make progress after malformed input.
                del self.buffer[0]


WINDOW_TITLE = "Witness Communications Monitor"


class MonitorBase:
    """The monitor's widgets and logic.

    SerialMonitor runs it as its own window (python main.py). MonitorPanel
    embeds the same monitor in another window as a frame (witnessOS).
    """

    standalone = True
    on_title = None

    def _init_monitor(self, *, test_mode=False, test_source="", test_speed=1.0, test_loop=True,
                      auto_connect=True, start_feed=True, feed_port=None):
        self._closed = False
        self.serials = {}
        self.tcp_sockets = {}
        self.discovery = SerialDiscovery("witness")
        self.auto_connect = tk.BooleanVar(value=auto_connect and not test_mode)
        self._auto_job = None
        self._auto_after = None
        self._next_auto_open = 0
        self.stop_event = threading.Event()
        self.test_active = False
        self._test_source = None
        self._test_stop = None
        self._test_thread = None
        self._test_token = None
        self._test_start_id = None
        self._test_display = False
        self.test_file = tk.StringVar(value=test_source)
        self.test_speed = tk.StringVar(value=str(test_speed))
        self.test_loop = tk.BooleanVar(value=test_loop)
        self.test_status = tk.StringVar(value="Test source: blank = built-in 60s simulated flight. No hardware writes.")
        self.events = queue.Queue()
        self.log_files = {}
        self.nmea_files = {}
        self.nmea_counts = {}       # source -> [valid, bad]
        self.nmea_last_gga = {}
        self.nmea_status = tk.StringVar(value="NMEA: none received")
        self.debug_log_file = None
        self.values = {}
        self.check_vars = {}
        self.history = {}
        self.last_packet_time = {}
        self.age_labels = {}
        self.status_indicators = {}
        self.section_groups = {}
        self.section_column_counts = {}
        self.command_history = []
        self.command_history_index = 0
        self.command_history_draft = ""
        self.max_history = 300
        # Event-pump limits. Workers can produce events far faster than Tk
        # can display them one at a time, so each tick drains the queue,
        # keeps every graph sample, and redraws only the newest state.
        self.max_events_per_tick = 20000
        self.max_console_lines = 2000
        self.max_console_lines_per_tick = 400
        self.graph_interval_s = 0.1
        self._last_graph_draw = 0.0
        self._graphs_dirty = False
        self._graph_job = None
        self._graph_size = (0, 0)
        self.publisher = WitnessPublisher(
            DEFAULT_WITNESS_FEED_HOST, DEFAULT_WITNESS_FEED_PORT if feed_port is None else feed_port)
        self.feed_status = tk.StringVar(value="Horizon feed: starting...")
        self.build_ui()
        if start_feed:
            try:
                self.publisher.start()
                self.feed_status.set(f"Horizon feed: {self.publisher.host}:{self.publisher.port} (receive only)")
            except OSError as exc:
                self.feed_status.set(f"Horizon feed unavailable: {exc}")
                self.console_write(f"FEED_ERROR {exc}; Witness input still works")
        else:
            self.feed_status.set("Horizon feed: off")
        self.refresh_ports()
        self._auto_after = self.after(250, self._auto_poll)
        self._events_after = self.after(50, self.process_events)
        self._graph_job = self.after(100, self.draw_graphs)
        self._age_after = self.after(100, self.update_age_indicators)
        if self.standalone:
            self.protocol("WM_DELETE_WINDOW", self.close)
        if test_mode:
            self._test_start_id = self.after(0, self._start_requested_test)

    def build_ui(self):
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")
        ttk.Label(controls, text="Port A:").pack(side="left")
        self.port_a_combo = ttk.Combobox(controls, width=13, state="readonly")
        self.port_a_combo.pack(side="left", padx=(4, 8))
        ttk.Label(controls, text="Port B:").pack(side="left")
        self.port_b_combo = ttk.Combobox(controls, width=13, state="readonly")
        self.port_b_combo.pack(side="left", padx=(4, 4))
        ttk.Button(controls, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Label(controls, text="Baud:").pack(side="left", padx=(14, 4))
        self.baud_combo = ttk.Combobox(controls, width=9, values=("115200", "57600", "38400", "9600"))
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left")
        self.connect_button = ttk.Button(controls, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=(14, 0))
        self.status_label = ttk.Label(controls, text="Disconnected", width=34, anchor="w")
        self.status_label.pack(side="left", padx=12)
        tcp_controls = ttk.Frame(self, padding=(8, 0, 8, 8))
        tcp_controls.pack(fill="x")
        self.tcp_entries = []
        for index, (host_default, port_default) in enumerate(
                ((DEFAULT_TCP_1_HOST, DEFAULT_TCP_1_PORT),
                 (DEFAULT_TCP_2_HOST, DEFAULT_TCP_2_PORT)), start=1):
            ttk.Label(tcp_controls, text=f"TCP {index} host:").pack(side="left",
                                                                      padx=(0 if index == 1 else 14, 4))
            host_entry = ttk.Entry(tcp_controls, width=16)
            host_entry.insert(0, host_default)
            host_entry.pack(side="left")
            ttk.Label(tcp_controls, text="port:").pack(side="left", padx=(4, 2))
            port_entry = ttk.Entry(tcp_controls, width=7)
            port_entry.insert(0, port_default)
            port_entry.pack(side="left")
            self.tcp_entries.append((host_entry, port_entry))
        auto_controls = ttk.Frame(self, padding=(8, 0, 8, 4))
        auto_controls.pack(fill="x")
        self.auto_button = ttk.Checkbutton(auto_controls, text="Auto-connect USB", variable=self.auto_connect,
                                           command=self._auto_changed)
        self.auto_button.pack(side="left")
        ttk.Label(auto_controls, text="Known Witness devices only; select generic adapters manually once.").pack(
            side="left", padx=8)
        test_controls = ttk.Frame(self, padding=(8, 0, 8, 4))
        test_controls.pack(fill="x")
        test_controls.columnconfigure(1, weight=1)
        ttk.Label(test_controls, text="Test source:").grid(row=0, column=0, padx=(0, 4))
        self.test_file_entry = ttk.Entry(test_controls, textvariable=self.test_file)
        self.test_file_entry.grid(row=0, column=1, sticky="ew")
        self.test_browse_button = ttk.Button(test_controls, text="Browse JSON", command=self.browse_test_source)
        self.test_browse_button.grid(row=0, column=2, padx=4)
        ttk.Label(test_controls, text="Speed:").grid(row=0, column=3, padx=(8, 4))
        self.test_speed_entry = ttk.Entry(test_controls, textvariable=self.test_speed, width=5)
        self.test_speed_entry.grid(row=0, column=4)
        self.test_loop_button = ttk.Checkbutton(test_controls, text="Loop", variable=self.test_loop)
        self.test_loop_button.grid(row=0, column=5, padx=8)
        self.test_button = ttk.Button(test_controls, text="Start test", command=self.toggle_test)
        self.test_button.grid(row=0, column=6)
        self.test_status_label = tk.Label(test_controls, textvariable=self.test_status,
                                         anchor="w", justify="left", wraplength=800)
        self.test_status_label.grid(row=1, column=0, columnspan=7, sticky="ew", pady=(3, 0))
        self._status_background = self.test_status_label.cget("background")
        test_controls.bind("<Configure>", lambda event: self.test_status_label.configure(
            wraplength=max(120, event.width - 16)))
        ttk.Label(self, textvariable=self.feed_status, wraplength=800).pack(fill="x", padx=8, pady=(0, 5))
        ttk.Label(self, textvariable=self.nmea_status, wraplength=800).pack(fill="x", padx=8, pady=(0, 5))
        main_pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        main_pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        notebook = ttk.Notebook(main_pane)
        telemetry_page = ttk.Frame(notebook)
        notebook.add(telemetry_page, text="Telemetry")
        telemetry_canvas = tk.Canvas(telemetry_page, highlightthickness=0, width=320, height=200)
        telemetry_scrollbar = ttk.Scrollbar(telemetry_page, orient="vertical",
                                            command=telemetry_canvas.yview)
        self.telemetry_frame = ttk.Frame(telemetry_canvas, padding=8)
        telemetry_window = telemetry_canvas.create_window(
            (0, 0), window=self.telemetry_frame, anchor="nw"
        )
        telemetry_canvas.configure(yscrollcommand=telemetry_scrollbar.set)
        telemetry_canvas.pack(side="left", fill="both", expand=True)
        telemetry_scrollbar.pack(side="right", fill="y")
        self.telemetry_frame.bind(
            "<Configure>",
            lambda _event: telemetry_canvas.configure(
                scrollregion=telemetry_canvas.bbox("all")
            ),
        )
        telemetry_canvas.bind(
            "<Configure>",
            lambda event: telemetry_canvas.itemconfigure(
                telemetry_window, width=event.width
            ),
        )
        self.graph_frame = ttk.Frame(notebook, padding=8)
        notebook.add(self.graph_frame, text="Graphs")
        self.make_sections(self.telemetry_frame, "packet")
        self.bind_scroll_wheel(telemetry_canvas)
        graph_controls = ttk.Frame(self.graph_frame)
        graph_controls.pack(fill="x", pady=(0, 5))
        ttk.Button(graph_controls, text="Clear graph history", command=self.clear_history).pack(side="left")
        ttk.Button(graph_controls, text="Save setup", command=self.save_graph_setup).pack(
            side="left", padx=(8, 0)
        )
        ttk.Button(graph_controls, text="Load setup", command=self.load_graph_setup).pack(
            side="left", padx=(4, 0)
        )
        ttk.Label(graph_controls, text="Select fields using the checkboxes on the Telemetry tab.").pack(side="left", padx=10)
        graph_area = ttk.Frame(self.graph_frame)
        graph_area.pack(fill="both", expand=True)
        self.graph_canvas = tk.Canvas(graph_area, background="white", height=240, width=320,
                                      highlightthickness=1, highlightbackground="#b0b0b0")
        graph_scrollbar = ttk.Scrollbar(graph_area, orient="vertical", command=self.graph_canvas.yview)
        self.graph_canvas.configure(yscrollcommand=graph_scrollbar.set)
        graph_scrollbar.pack(side="right", fill="y")
        self.graph_canvas.pack(side="left", fill="both", expand=True)
        self.graph_canvas.bind("<Configure>", self._graph_configured)
        self.graph_canvas.bind("<Map>", lambda _event: self.schedule_graph_draw())
        self.bind_scroll_wheel(self.graph_canvas)

        console_frame = ttk.LabelFrame(main_pane, text="Console", padding=5)
        self.console = scrolledtext.ScrolledText(
            console_frame, height=6, wrap="word", state="disabled",
            font=("Consolas", 9)
        )
        self.console.pack(fill="both", expand=True)
        command_frame = ttk.Frame(console_frame, padding=(0, 6, 0, 0))
        command_frame.pack(fill="x")
        ttk.Label(command_frame, text="Send hex to:").pack(side="left")
        self.command_port_combo = ttk.Combobox(command_frame, width=13, state="readonly")
        self.command_port_combo.pack(side="left", padx=(5, 8))
        ttk.Label(command_frame, text="Command:").pack(side="left")
        self.command_entry = ttk.Entry(command_frame)
        self.command_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.command_entry.bind("<Return>", lambda _event: self.send_hex_command())
        self.command_entry.bind("<Up>", lambda _event: self.recall_command(-1))
        self.command_entry.bind("<Down>", lambda _event: self.recall_command(1))
        self.send_command_button = ttk.Button(
            command_frame, text="Send", command=self.send_hex_command
        )
        self.send_command_button.pack(side="left")
        quick_command_frame = ttk.Frame(console_frame, padding=(0, 5, 0, 0))
        quick_command_frame.pack(fill="x")
        ttk.Label(quick_command_frame, text="Commands:").pack(side="left", anchor="n")
        quick_command_grid = ttk.Frame(quick_command_frame)
        quick_command_grid.pack(side="left", fill="x", expand=True)
        self.quick_command_buttons = []
        command_columns = 4
        for index, (label, opcode, confirm) in enumerate(QUICK_COMMANDS):
            button = ttk.Button(
                quick_command_grid,
                text=label,
                command=lambda name=label, value=opcode, needs_confirm=confirm:
                    self.send_quick_command(
                        name, value, needs_confirm
                ),
            )
            button.grid(
                row=index // command_columns, column=index % command_columns,
                sticky="ew", padx=(5, 0), pady=(0, 3),
            )
            self.quick_command_buttons.append(button)
        for column in range(command_columns):
            quick_command_grid.columnconfigure(column, weight=1)
        main_pane.add(notebook, weight=4)
        main_pane.add(console_frame, weight=1)

    def make_sections(self, parent, prefix):
        # Field lists mirror comms.h exactly, in wire order, per packet type.
        specs = {
            "sensors": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("filtered_accel", "Filtered accel [g; x, y, z]"),
                ("gyro", "Gyro [dps; x, y, z]"),
                ("sensor_trailing_u16", "Sensor trailing bytes [u16]"),
            ],
        "state": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("orientation", "Orientation [x, y, z, w]"),
                ("position", "Position [x, y, z]"),
                ("velocity", "Velocity [x, y, z]"),
                ("latitude", "Latitude"), ("longitude", "Longitude"),
                ("gps_altitude", "GPS altitude"),
                ("kalman_altitude", "Kalman altitude above pad [m]"),
                ("witness_battery_voltage", "Witness battery voltage [V]"),
                # Shown as "<sats>  lock <0|1>": lock is status byte 1 bit 3.
                ("gps_satellites", "GPS sats"),
            ],
            "camera": [
                ("fc_status", "FC status"), ("fc_uptime", "FC uptime"),
                ("status", "Camera status"), ("uptime", "Camera uptime"),
                ("current_sense", "Current [ch1, ch2, ch3]"),
                ("iris_battery_voltage", "Iris battery voltage [V]"),
            ],
            "heartbeat": [("status", "Status"), ("uptime", "Uptime")],
            "ground_station_status": [
                ("rssi", "Last valid LoRa RSSI [dBm]"),
                ("uptime", "Ground-station uptime [ms]"),
                ("gps_fix_quality", "Aperture GPS fix quality"),
                ("gps_satellites", "Aperture GPS satellites"),
                ("latitude", "Aperture latitude [deg]"),
                ("longitude", "Aperture longitude [deg]"),
                ("gps_altitude", "Aperture GPS altitude [m MSL]"),
                ("gps_hdop", "Aperture GPS HDOP"),
                ("gps_fix_age_ms", "Aperture GPS fix age [ms]"),
            ],
            "command": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("command_value", "Command value"),
            ],
            "witness_debug": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("mcu_temperature", "MCU temperature [C]"),
                ("witness_battery_voltage", "Witness battery voltage [V]"),
                ("filtered_accel", "Filtered accel [g; x, y, z]"),
                ("orientation", "Orientation [x, y, z, w]"),
                ("position", "Position [m; x, y, z]"),
                ("velocity", "Velocity [m/s; x, y, z]"),
                ("low_accel", "LSM6 low accel [g; x, y, z]"),
                ("lsm6_temperature", "LSM6 temperature [C]"),
                ("high_accel", "LSM6 high accel [g; x, y, z]"),
                ("gyro", "Gyro [dps; x, y, z]"),
                ("pressure", "MS5607 pressure [Pa]"),
                ("ms5607_temperature", "MS5607 temperature [C]"),
                ("kalman_altitude", "Kalman altitude above pad [m]"),
                ("latitude", "Latitude [deg]"), ("longitude", "Longitude [deg]"),
                ("gps_time", "GPS time [Unix s]"), ("gps_altitude", "GPS altitude [m]"),
            ],
            "iris_debug": [
                ("status", "Status"), ("uptime", "Uptime"),
                ("mcu_temperature", "MCU temperature [C]"),
                ("iris_battery_voltage", "Iris battery voltage [V]"),
                ("current_sense", "Current [A; ch1, ch2, ch3]"),
            ],
        }
        sections = []
        for row, (packet, fields) in enumerate(specs.items()):
            section = ttk.LabelFrame(parent, text=packet.capitalize(), padding=6)
            section.grid(row=row // 3, column=row % 3, sticky="nsew", padx=4, pady=4)
            sections.append(section)
            section.columnconfigure(0, weight=1)
            header = ttk.Frame(section)
            header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
            ttk.Label(header, text="Data age:").pack(side="left")
            age_key = f"{prefix}.{packet}"
            age_label = tk.Label(header, text="--", width=10, anchor="center",
                                 relief="sunken", bd=1)
            age_label.pack(side="left", padx=(5, 0))
            self.age_labels[age_key] = age_label
            field_row = 1
            for field, label in fields:
                key = f"{prefix}.{packet}.{field}"
                self.values[key] = tk.StringVar(value="null")
                self.check_vars[key] = tk.BooleanVar(value=False)
                row_frame = ttk.Frame(section)
                row_frame.grid(row=field_row, column=0,
                               sticky="ew", pady=(0, 2))
                row_frame.columnconfigure(2, weight=1)
                ttk.Checkbutton(row_frame, variable=self.check_vars[key]).grid(
                    row=0, column=0, sticky="w")
                ttk.Label(row_frame, text=label + ":", width=20, wraplength=145,
                          justify="left", anchor="w").grid(row=0, column=1,
                                                             sticky="w", padx=(2, 4))
                # Fixed-width value column: resizing the window changes the
                # section tiling, not the size of the displayed value cells.
                ttk.Label(row_frame, textvariable=self.values[key], width=18,
                          wraplength=135, justify="left", anchor="w").grid(
                              row=0, column=2, sticky="w")
                field_row += 1
                if field == "status" or field.endswith("_status"):
                    self.make_status_decoder(
                        section, key, field_row,
                        camera_status=(packet == "camera" and field == "status"),
                    )
                    field_row += 1
        self.section_groups[parent] = sections
        parent.bind("<Configure>", lambda _event, frame=parent: self.retile_sections(frame))
        self.retile_sections(parent)

    def make_status_decoder(self, parent, key, row, camera_status=False):
        decoder = ttk.Frame(parent)
        decoder.grid(row=row, column=0, sticky="ew", padx=(24, 0), pady=(0, 5))

        ttk.Label(
            decoder,
            text="Byte 0: camera flags" if camera_status else "Byte 0: system flags",
        ).pack(anchor="w")
        flags_frame = ttk.Frame(decoder)
        flags_frame.pack(anchor="w")
        flag_labels = []
        flag_specs = CAMERA_STATUS_FLAG_LABELS if camera_status else STATUS_FLAG_LABELS
        for index, (mask, text) in enumerate(flag_specs):
            label = tk.Label(
                flags_frame, text=text, width=9, height=2, wraplength=65,
                background="#f7f7f7", foreground="#777777",
                relief="solid", borderwidth=1,
            )
            label.grid(row=index // 4, column=index % 4, sticky="nsew", padx=1, pady=1)
            flag_labels.append((mask, label))

        if camera_status:
            ttk.Label(decoder, text="Byte 1: reserved").pack(anchor="w", pady=(3, 0))
            self.status_indicators[key] = {
                "flags": flag_labels,
                "states": None,
                "invalid": None,
                "camera": True,
            }
            return

        ttk.Label(decoder, text="Byte 1 bits 2:0: flight state").pack(
            anchor="w", pady=(3, 0)
        )
        states_frame = ttk.Frame(decoder)
        states_frame.pack(anchor="w")
        state_labels = {}
        for index, (state, text) in enumerate(FLIGHT_STATE_LABELS):
            label = tk.Label(
                states_frame, text=text, width=9, height=1,
                background="#f7f7f7", foreground="#777777",
                relief="solid", borderwidth=1,
            )
            label.grid(row=index // 3, column=index % 3, sticky="nsew", padx=1, pady=1)
            state_labels[state] = label
        invalid_label = tk.Label(
            states_frame, text="Invalid", width=9, height=1,
            background="#f7f7f7", foreground="#777777",
            relief="solid", borderwidth=1,
        )
        invalid_label.grid(row=1, column=2, sticky="nsew", padx=1, pady=1)
        ttk.Label(decoder, text="Byte 1 bit 3: GPS lock; bit 4: HITL").pack(
            anchor="w", pady=(3, 0)
        )
        byte1_frame = ttk.Frame(decoder)
        byte1_frame.pack(anchor="w")
        byte1_labels = []
        for index, (mask, text) in enumerate(STATUS_BYTE1_FLAG_LABELS):
            label = tk.Label(
                byte1_frame, text=f"{text} 0", width=13, height=1,
                background="#f7f7f7", foreground="#777777",
                relief="solid", borderwidth=1,
            )
            label.grid(row=0, column=index, sticky="nsew", padx=1, pady=1)
            byte1_labels.append((mask, text, label))
        self.status_indicators[key] = {
            "flags": flag_labels,
            "states": state_labels,
            "invalid": invalid_label,
            "byte1_flags": byte1_labels,
            "camera": False,
        }

    def retile_sections(self, parent):
        sections = self.section_groups.get(parent, [])
        if not sections:
            return
        # Value columns remain fixed-width; only the number of tiles changes.
        # Tiles are as wide as the widest section needs (its status-flag
        # grid), so a column is never squeezed and its flags clipped; a
        # narrow window just gets fewer columns.
        tile_width = max(290, max(section.winfo_reqwidth() for section in sections) + 8)
        columns = max(1, min(len(sections), parent.winfo_width() // tile_width))
        if columns == 0:
            columns = 1
        if self.section_column_counts.get(parent) == columns:
            return
        self.section_column_counts[parent] = columns
        for column in range(len(sections)):
            parent.columnconfigure(column, weight=1 if column < columns else 0,
                                   minsize=0)
        for index, section in enumerate(sections):
            section.grid_configure(row=index // columns, column=index % columns)

    def refresh_ports(self):
        self.discovery.refresh(force=True)
        self._update_port_choices()

    def _update_port_choices(self):
        ports = [p.device for p in self.discovery.ports]
        choices = [NO_SERIAL_PORT, *ports]
        for combo in (
            self.port_a_combo,
            self.port_b_combo,
        ):
            current = combo.get()
            combo["values"] = choices
            combo.set(current if current in ports or current in self.serials else NO_SERIAL_PORT)
        self.refresh_command_targets()

    def _auto_changed(self):
        self._next_auto_open = 0
        if self.test_active or self._test_start_id is not None:
            self.auto_connect.set(False)
        if not self.auto_connect.get() and self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
            self.connect_button.configure(text="Disconnect" if self.serials or self.tcp_sockets else "Connect")

    def _auto_poll(self):
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
                self.connect_button.configure(text="Disconnect" if self.serials or self.tcp_sockets else "Connect")
                stream, error = result
                if error:
                    self.console_write(f"AUTO_CONNECT[{port}] {error}; will retry")
                    self.status_label.configure(text=f"Waiting for {port}")
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
                baud = int(self.baud_combo.get())
                if baud <= 0:
                    raise ValueError("baud must be positive")
            except ValueError as exc:
                self.console_write(f"AUTO_CONNECT invalid baud: {exc}")
                return
            self.console_write(f"AUTO_CONNECT[{port}] opening known Witness USB device")
            self._auto_job = SerialOpenJob(port, baud)
            self.connect_button.configure(text="Cancel connect")
        except Exception as exc:
            self.console_write(f"AUTO_CONNECT_ERROR {exc}")
        finally:
            self._auto_after = self.after(250, self._auto_poll)

    def _accept_auto_port(self, port, stream):
        if port in self.serials or len(self.serials) >= 2:
            stream.close()
            return
        log_file = None
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            safe_port = "".join(char if char.isalnum() else "_" for char in port)
            log_file = (LOG_DIR / f"serial_{safe_port}_{stamp}.bin").open("ab")
            if self.debug_log_file is None:
                self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        except OSError:
            stream.close()
            if log_file is not None:
                log_file.close()
            raise
        if self._test_display:
            self._clear_telemetry()
            self._test_display = False
            self.set_window_title(WINDOW_TITLE)
            self.test_status.set("Live mode. Test values cleared.")
            self.test_status_label.configure(background=self._status_background)
        self.serials[port] = stream
        self.log_files[port] = log_file
        self.stop_event.clear()
        for combo in (self.port_a_combo, self.port_b_combo):
            if combo.get() not in self.serials or combo.get() == port:
                combo.set(port)
                break
        self.connect_button.configure(text="Disconnect")
        self.status_label.configure(text=f"Connected: {', '.join(self.serials)}")
        self.refresh_command_targets()
        self.console_write(f"AUTO_CONNECT[{port}] connected")
        threading.Thread(target=self.read_serial, args=(port, stream), daemon=True).start()

    def refresh_command_targets(self):
        targets = [] if self.test_active else list(self.serials) + list(self.tcp_sockets)
        self.command_port_combo["values"] = targets
        if self.command_port_combo.get() not in targets:
            self.command_port_combo.set(targets[0] if targets else "")
        state = "normal" if targets else "disabled"
        self.command_entry.configure(state=state)
        self.send_command_button.configure(state=state)
        for button in self.quick_command_buttons:
            button.configure(state=state)

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
        target_name = self.command_port_combo.get()
        text = self.command_entry.get().strip().replace(",", " ")
        text = text.replace("0x", "").replace("0X", "")
        if not text:
            self.console_write(f"TX_ERROR[{target_name}] command is empty")
            return
        try:
            command = bytes.fromhex(text)
        except ValueError as exc:
            self.console_write(f"TX_ERROR[{target_name}] invalid hex command: {exc}")
            return
        if self.transmit_command(target_name, command):
            self.command_entry.delete(0, "end")

    @staticmethod
    def build_command_packet(opcode):
        if not 0 <= opcode <= 0xFFFF:
            raise ValueError("command opcode must fit in 16 bits")
        return (
            bytes((COMMAND_PACKET_ID,))
            + bytes(6)  # Zero status and uptime; the receiver ignores these.
            + opcode.to_bytes(2, byteorder="big")
            + bytes((COMMAND_PACKET_EOF,))
        )

    @staticmethod
    def build_quick_command_packet(opcode):
        return SerialMonitor.build_command_packet(opcode)

    def send_quick_command(self, name, opcode, confirm=False):
        if confirm and not messagebox.askyesno(
                "Confirm command",
                "Erase all stored log sessions? This cannot be undone.",
                icon="warning"):
            return
        packet = self.build_quick_command_packet(opcode)
        self.transmit_command(self.command_port_combo.get(), packet, name)

    def recall_command(self, direction):
        """Navigate successfully transmitted commands with Up and Down."""
        if not self.command_history:
            return "break"

        history_end = len(self.command_history)
        if direction < 0:
            if self.command_history_index == history_end:
                self.command_history_draft = self.command_entry.get()
            self.command_history_index = max(0, self.command_history_index - 1)
        else:
            self.command_history_index = min(
                history_end, self.command_history_index + 1
            )

        if self.command_history_index == history_end:
            recalled = self.command_history_draft
        else:
            recalled = self.command_history[self.command_history_index]
        self.command_entry.delete(0, "end")
        self.command_entry.insert(0, recalled)
        self.command_entry.icursor("end")
        return "break"

    def toggle_connection(self):
        if self.serials or self.tcp_sockets or self._auto_job is not None:
            self.auto_connect.set(False)
            self.disconnect()
        else:
            self.connect()

    def _start_requested_test(self):
        self._test_start_id = None
        self.start_test()

    def browse_test_source(self):
        filename = filedialog.askopenfilename(
            parent=self, title="Timed test packets", filetypes=(("Test source JSON", "*.json"),))
        if filename:
            self.test_file.set(filename)

    def toggle_test(self):
        self.stop_test() if self.test_active else self.start_test()

    def _clear_telemetry(self):
        for variable in self.values.values():
            variable.set("null")
        self.history.clear()
        self.last_packet_time.clear()
        for indicators in self.status_indicators.values():
            labels = [label for _mask, label in indicators["flags"]]
            if indicators["states"] is not None:
                labels.extend(indicators["states"].values())
                labels.append(indicators["invalid"])
                indicators["invalid"].configure(text="Invalid")
            for _mask, text, label in indicators.get("byte1_flags", ()):
                label.configure(text=f"{text} 0")
                labels.append(label)
            for label in labels:
                label.configure(background="#f7f7f7", foreground="#777777", relief="solid")

    def start_test(self):
        if self.test_active:
            return False
        if self.serials or self.tcp_sockets:
            messagebox.showerror("Test mode", "Disconnect real serial/TCP inputs before starting a test.", parent=self)
            return False
        self.auto_connect.set(False)
        self._auto_changed()
        raw_log = None
        try:
            speed = validate_speed(float(self.test_speed.get()))
            filename = self.test_file.get().strip()
            scenario = load_scenario(filename) if filename else builtin_scenario()
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            raw_log = (LOG_DIR / f"test_{stamp}.bin").open("xb")
            self.debug_log_file = (LOG_DIR / f"test_debug_{stamp}.log").open("x", encoding="utf-8")
        except (OSError, ValueError) as exc:
            if raw_log is not None:
                raw_log.close()
            messagebox.showerror("Test mode", str(exc), parent=self)
            return False
        self._clear_telemetry()
        self._test_display = self.test_active = True
        self._test_token = token = object()
        self._test_stop = stop = threading.Event()
        self._test_source = source = "TEST_" + "".join(
            char if char.isalnum() or char in "_-" else "_" for char in scenario.name)[:100]
        self.set_window_title(WINDOW_TITLE + " [TEST DATA]")
        self.test_status_label.configure(background="#fff2a8")
        self.test_status.set(f"TEST DATA: {scenario.name} at {speed:g}x; source {source}. No hardware writes.")
        self.status_label.configure(text="TEST MODE - simulated input")
        self.test_button.configure(text="Stop test")
        for widget in (self.test_file_entry, self.test_browse_button, self.test_speed_entry,
                       self.test_loop_button, self.connect_button):
            widget.configure(state="disabled")
        self.refresh_command_targets()
        self.console_write(f"TEST_START[{source}] {len(scenario.frames)} chunks, "
                           f"duration={scenario.duration_ms}ms, speed={speed:g}x, "
                           f"loop={self.test_loop.get()}; raw log={raw_log.name}")
        self._test_thread = threading.Thread(
            target=self.read_test, args=(scenario, source, token, stop, raw_log, speed, self.test_loop.get()),
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
            return
        self._test_stop.set()
        if self._test_thread is not None:
            self._test_thread.join(timeout=2)
        self.test_active = False
        self._test_token = None  # Ignore queued display/completion events from an old run.
        self._test_thread = None
        self.test_button.configure(text="Start test")
        for widget in (self.test_file_entry, self.test_browse_button, self.test_speed_entry,
                       self.test_loop_button, self.connect_button):
            widget.configure(state="normal")
        self.refresh_command_targets()
        self.status_label.configure(text="TEST MODE - " + result)
        self.test_status.set(f"TEST {result}: displayed values are simulated. Start test to replay again.")
        self.console_write(f"TEST_END[{self._test_source}] {result}")
        if self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None

    def connect(self):
        if self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
        if self.test_active:
            messagebox.showerror("Test mode", "Stop the test before connecting real inputs.", parent=self)
            return
        ports = [
            port for port in (self.port_a_combo.get(), self.port_b_combo.get())
            if port and port != NO_SERIAL_PORT
        ]
        tcp_endpoints = []
        for index, (host_entry, port_entry) in enumerate(self.tcp_entries, start=1):
            host = host_entry.get().strip()
            port_text = port_entry.get().strip()
            if not host:
                continue
            try:
                port = int(port_text)
            except ValueError:
                messagebox.showerror("Horizon monitor", f"TCP {index} port must be a number.")
                return
            if not 1 <= port <= 65535:
                messagebox.showerror("Horizon monitor", f"TCP {index} port must be 1-65535.")
                return
            tcp_endpoints.append((index, host, port))
        if len({(host, port) for _, host, port in tcp_endpoints}) != len(tcp_endpoints):
            messagebox.showerror("Horizon monitor", "TCP endpoints must be different.")
            return
        if not ports and not tcp_endpoints:
            messagebox.showerror("Horizon monitor", "Select a serial port or enter a TCP host.")
            return
        if len(set(ports)) != len(ports):
            messagebox.showerror("Serial monitor", "Port A and Port B must be different.")
            return
        if self._test_display:
            self._clear_telemetry()
            self._test_display = False
            self.test_status.set("Live mode. Test values cleared; source file remains available for replay.")
            self.test_status_label.configure(background=self._status_background)
            self.set_window_title(WINDOW_TITLE)
        try:
            if ports:
                import serial
        except ImportError:
            messagebox.showerror("Serial monitor", "pyserial is required: pip install pyserial")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.debug_log_file = (LOG_DIR / f"debug_{stamp}.log").open("a", encoding="utf-8")
        self.stop_event.clear()
        try:
            for port in ports:
                serial_port = serial.Serial(port, int(self.baud_combo.get()), timeout=0.2)
                self.serials[port] = serial_port
                safe_port = "".join(char if char.isalnum() else "_" for char in port)
                self.log_files[port] = (LOG_DIR / f"serial_{safe_port}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_serial, args=(port, serial_port), daemon=True).start()
            self.refresh_command_targets()
            for index, tcp_host, tcp_port in tcp_endpoints:
                tcp_socket = socket.create_connection((tcp_host, tcp_port), timeout=3.0)
                tcp_socket.settimeout(0.2)
                tcp_source = f"TCP_{tcp_host}_{tcp_port}"
                self.tcp_sockets[tcp_source] = tcp_socket
                safe_source = "".join(char if char.isalnum() else "_" for char in tcp_source)
                self.log_files[tcp_source] = (LOG_DIR / f"{safe_source}_{stamp}.bin").open("ab")
                threading.Thread(target=self.read_tcp, args=(tcp_source, tcp_socket), daemon=True).start()
            self.refresh_command_targets()
        except Exception as exc:
            self.disconnect()
            messagebox.showerror("Serial monitor", f"Could not open serial port: {exc}")
            return
        self.connect_button.configure(text="Disconnect")
        sources = ports + [f"TCP {host}:{port}" for _, host, port in tcp_endpoints]
        self.status_label.configure(text=f"Connected: {', '.join(sources)}")

    def on_decoded_packet(self, source, packet, *, test_token=None):
        # Called by input workers: export immediately, independent of GUI backlog.
        # The arrival time travels with the packet so graphs plot when it was
        # received, not when the GUI got round to it.
        received_at = time.time()
        try:
            self.publisher.publish(source, packet, received_at=received_at)
        except Exception as exc:
            self.events.put(("decoder_error", (source, f"Horizon feed: {exc}")))
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

    def read_serial(self, port_name, serial_port):
        nmea = NmeaExtractor()
        decoder = PacketDecoder(
            lambda p: self.on_decoded_packet(port_name, p),
            lambda message: self.events.put(("decoder_error", (port_name, message))),
            lambda packet_id, count: self.events.put(("host_packet", (port_name, packet_id, count))),
        )
        while not self.stop_event.is_set() and self.serials.get(port_name) is serial_port:
            try:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    log_file = self.log_files.get(port_name)
                    if log_file is not None:
                        log_file.write(data)
                        log_file.flush()
                    self.events.put(("bytes", (port_name, bytes(data))))
                    decoder.feed(data)
                    self._queue_nmea(port_name, nmea, data)
            except Exception as exc:
                self.events.put(("error", (port_name, str(exc), serial_port)))
                break

    def read_tcp(self, source_name, tcp_socket):
        nmea = NmeaExtractor()
        decoder = PacketDecoder(
            lambda p: self.on_decoded_packet(source_name, p),
            lambda message: self.events.put(("decoder_error", (source_name, message))),
            lambda packet_id, count: self.events.put(("host_packet", (source_name, packet_id, count))),
        )
        while not self.stop_event.is_set() and self.tcp_sockets.get(source_name) is tcp_socket:
            try:
                data = tcp_socket.recv(4096)
                if not data:
                    self.events.put(("error", (source_name, "TCP server closed the connection", tcp_socket)))
                    break
                log_file = self.log_files.get(source_name)
                if log_file is not None:
                    log_file.write(data)
                    log_file.flush()
                self.events.put(("bytes", (source_name, bytes(data))))
                decoder.feed(data)
                self._queue_nmea(source_name, nmea, data)
            except socket.timeout:
                continue
            except Exception as exc:
                self.events.put(("error", (source_name, str(exc), tcp_socket)))
                break

    def process_events(self):
        """Drain every queued event, then update the display once.

        Older code did all display work per event inside a 20 ms budget, so a
        fast stream queued faster than it drained and the monitor fell
        further behind real time. Now each tick takes everything waiting:
        graph history keeps every sample (stamped with its arrival time),
        console lines are written in one batch, and labels, status
        indicators and graphs show only the newest packet of each type.
        """
        latest = {}              # packet type -> (port, packet), newest wins
        ports_seen = set()
        console_lines = []
        handled = 0
        try:
            while handled < self.max_events_per_tick:
                try:
                    kind, payload = self.events.get_nowait()
                except queue.Empty:
                    break
                handled += 1
                try:
                    self._handle_event(kind, payload, latest, ports_seen, console_lines)
                except Exception as exc:
                    # A malformed packet or display value must never terminate
                    # the Tk event pump. Keep the failure visible in the log.
                    console_lines.append(self._console_stamp(f"GUI_ERROR {exc}"))
            for port_name in ports_seen:
                if port_name in self.serials:
                    self.discovery.remember(port_name)
            for port_name, packet in latest.values():
                try:
                    self.update_values("packet", packet, record=False, draw=False)
                except Exception as exc:
                    console_lines.append(self._console_stamp(f"GUI_ERROR {exc}"))
            self._write_console_lines(console_lines)
            self._refresh_nmea_status()
            self._maybe_draw_graphs()
        finally:
            # Come back sooner while a backlog remains; otherwise idle at 50 ms.
            # Never ~1 ms: Tk's update() keeps running timers that are already
            # due, so a 1 ms reschedule under a steady stream would never let
            # update() return (one tick already drains up to 20,000 events).
            if not self._closed:
                self._events_after = self.after(15 if not self.events.empty() else 50, self.process_events)

    def _console_stamp(self, line):
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
            # Ignore any late events from a previously disconnected live
            # input; never mix those values with this test run.
            return
        if kind == "packet":
            port_name, packet = payload[0], payload[1]
            received_at = payload[2] if len(payload) > 2 else time.time()
            ports_seen.add(port_name)
            self.record_history("packet", packet, received_at)
            latest[packet.get("packet")] = (port_name, packet)
            console_lines.append(self._console_stamp(
                f"PACKET[{port_name}] {self.format_packet(packet)}"))
            return
        if kind == "nmea":
            self._handle_nmea(*payload)
            return
        if kind == "bytes":
            port_name, data = payload
            console_lines.append(self._console_stamp(f"RX[{port_name}]  {data.hex(' ')}"))
            return
        if kind == "decoder_error":
            port_name, message = payload
            console_lines.append(self._console_stamp(f"BAD_PACKET[{port_name}] {message}"))
            return
        if kind == "host_packet":
            # Echoed host->Witness packet (0x0C HITL input): framed
            # and skipped, never shown as telemetry. Log sparsely.
            port_name, packet_id, count = payload
            if count == 1 or count % 100 == 0:
                console_lines.append(self._console_stamp(
                    f"HOST_ECHO[{port_name}] skipped 0x{packet_id:02x} host packet "
                    f"(HITL input; {count} so far)"))
            return
        if kind == "error":
            # Rare and stateful: flush what came before so the log stays in order.
            self._write_console_lines(console_lines)
            console_lines.clear()
            self._handle_input_error(payload)
            return

    def _nmea_file(self, source):
        handle = self.nmea_files.get(source)
        if handle is None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            safe = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in source)
            handle = (LOG_DIR / f"nmea_{safe}_{stamp}.csv").open("a", encoding="ascii", newline="")
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
        if not getattr(self, "_nmea_dirty", False):
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
            self.connect_button.configure(text="Connect")
        self.status_label.configure(text=f"Serial error on {port_name}: {message}")
        self.console_write(f"ERROR[{port_name}] {message}")
        if not self.serials and not self.tcp_sockets and self.debug_log_file is not None:
            self.debug_log_file.close()
            self.debug_log_file = None

    def update_age_indicators(self):
        if self.publisher.error:
            self.feed_status.set(f"Horizon feed stopped: {self.publisher.error}")
        elif self.publisher.dropped:
            self.feed_status.set(
                f"Horizon feed: {self.publisher.host}:{self.publisher.port}; "
                f"overload: {self.publisher.dropped} updates dropped (raw Witness logs unaffected)"
            )
        now = time.monotonic()
        for key, label in self.age_labels.items():
            received = self.last_packet_time.get(key)
            if received is None:
                label.configure(text="--", background="#f0f0f0")
                continue
            age = max(0.0, now - received)
            if age < 2.0:
                background = "#f0f0f0"
            elif age < 5.0:
                background = "#fff2a8"
            elif age < 10.0:
                background = "#ffd27f"
            else:
                background = "#ff8a80"
            label.configure(text=f"{age:.1f}s", background=background)
        if not self._closed:
            self._age_after = self.after(100, self.update_age_indicators)

    @staticmethod
    def format_display_value(field, value):
        if value is None:
            return "null"
        if isinstance(value, int) and (field == "status" or field.endswith("_status")):
            return f"0b{value:016b}"
        return str(value)

    @staticmethod
    def format_gps_satellites(packet):
        """Satellite count with the status-byte GPS lock flag beside it."""
        satellites = packet.get("gps_satellites")
        status = packet.get("status")
        lock = status_gps_locked(status) if isinstance(status, int) else "?"
        return f"{'null' if satellites is None else satellites}  lock {lock}"

    def update_status_decoder(self, key, value):
        indicators = self.status_indicators.get(key)
        if indicators is None or not isinstance(value, int):
            return

        system_flags = (value >> 8) & 0xFF
        flight_state = value & 0x07
        for mask, label in indicators["flags"]:
            active = bool(system_flags & mask)
            error_masks = {0x01, 0x02, 0x80} if indicators["camera"] else {0x10, 0x80}
            active_color = "#fb6a4a" if mask in error_masks else "#74c476"
            label.configure(
                background=active_color if active else "#f7f7f7",
                foreground="#111111" if active else "#777777",
                relief="sunken" if active else "solid",
            )

        if indicators["states"] is None:
            return
        for state, label in indicators["states"].items():
            active = flight_state == state
            label.configure(
                background="#6baed6" if active else "#f7f7f7",
                foreground="#111111" if active else "#777777",
                relief="sunken" if active else "solid",
            )
        invalid = flight_state not in indicators["states"]
        indicators["invalid"].configure(
            text=f"Invalid ({flight_state})" if invalid else "Invalid",
            background="#fb6a4a" if invalid else "#f7f7f7",
            foreground="#111111" if invalid else "#777777",
            relief="sunken" if invalid else "solid",
        )
        for mask, text, label in indicators.get("byte1_flags", ()):
            active = bool(value & mask)
            # HITL active is a warning (simulated sensors), GPS lock is good.
            active_color = "#fb6a4a" if mask == HITL_ACTIVE_MASK else "#74c476"
            label.configure(
                text=f"{text} {int(active)}",
                background=active_color if active else "#f7f7f7",
                foreground="#111111" if active else "#777777",
                relief="sunken" if active else "solid",
            )

    @classmethod
    def format_packet(cls, packet):
        fields = ", ".join(
            f"{name}={cls.format_display_value(name, value)}"
            for name, value in packet.items() if name != "packet"
        )
        return f"{packet['packet']}: {fields}"

    def console_write(self, line):
        self._write_console_lines([self._console_stamp(line)])

    def _write_console_lines(self, lines):
        """Append already-stamped lines: all to the debug log, a bounded tail to the console."""
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
        self.console.configure(state="normal")
        self.console.insert("end", "\n".join(shown) + "\n")
        # Keep the widget bounded: an ever-growing Text slows every insert.
        excess = int(self.console.index("end-1c").split(".")[0]) - 1 - self.max_console_lines
        if excess > 0:
            self.console.delete("1.0", f"{excess + 1}.0")
        self.console.see("end")
        self.console.configure(state="disabled")

    def record_history(self, kind, packet, received_at):
        """Add one packet's numeric fields to the graph history at its arrival time."""
        name = packet.get("packet")
        for field, value in packet.items():
            if field == "packet" or not isinstance(value, (int, float, list, tuple)):
                continue
            key = f"{kind}.{name}.{field}"
            if key not in self.values:
                continue
            samples = self.history.setdefault(key, [])
            samples.append((received_at, value))
            if len(samples) > self.max_history:
                del samples[:-self.max_history]
        self._graphs_dirty = True

    def update_values(self, kind, packet, *, record=True, draw=True):
        """Show a packet's fields. The event pump records history separately
        and calls this only for the newest packet of each type per tick."""
        if record:
            self.record_history(kind, packet, time.time())
        packet_key = f"{kind}.{packet['packet']}"
        self.last_packet_time[packet_key] = time.monotonic()
        for field, value in packet.items():
            if field != "packet":
                key = f"{kind}.{packet['packet']}.{field}"
                if key in self.values:
                    if field == "gps_satellites" and packet["packet"] == "state":
                        self.values[key].set(self.format_gps_satellites(packet))
                    else:
                        self.values[key].set(self.format_display_value(field, value))
                    if field == "status" or field.endswith("_status"):
                        self.update_status_decoder(key, value)
        if draw:
            self.draw_graphs()

    def _maybe_draw_graphs(self):
        """Redraw at most every graph_interval_s, and only when data changed."""
        if not self._graphs_dirty:
            return
        now = time.monotonic()
        if now - self._last_graph_draw < self.graph_interval_s:
            return
        self._last_graph_draw = now
        self._graphs_dirty = False
        self.draw_graphs()

    def clear_history(self):
        self.history.clear()
        self.draw_graphs()

    def save_graph_setup(self):
        selected_fields = [
            key for key, variable in self.check_vars.items() if variable.get()
        ]
        filename = filedialog.asksaveasfilename(
            title="Save graph setup",
            defaultextension=".json",
            initialfile="horizon_graph_setup.json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not filename:
            return
        setup = {
            "format": "horizon-graph-setup",
            "version": 1,
            "selected_fields": selected_fields,
        }
        try:
            Path(filename).write_text(json.dumps(setup, indent=2) + "\n", encoding="utf-8")
            self.console_write(
                f"GRAPH_SETUP saved {len(selected_fields)} field(s) to {filename}"
            )
        except (OSError, TypeError) as exc:
            messagebox.showerror("Save graph setup", f"Could not save setup:\n{exc}")

    def load_graph_setup(self):
        filename = filedialog.askopenfilename(
            title="Load graph setup",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not filename:
            return
        try:
            setup = json.loads(Path(filename).read_text(encoding="utf-8"))
            if not isinstance(setup, dict):
                raise ValueError("the top-level JSON value must be an object")
            selected_fields = setup.get("selected_fields")
            if not isinstance(selected_fields, list) or not all(
                    isinstance(key, str) for key in selected_fields):
                raise ValueError("selected_fields must be a list of field names")
            selected_set = set(selected_fields)
            unknown_fields = sorted(selected_set.difference(self.check_vars))
            for key, variable in self.check_vars.items():
                variable.set(key in selected_set)
            self.draw_graphs()
            self.console_write(
                f"GRAPH_SETUP loaded {len(selected_set) - len(unknown_fields)} field(s) "
                f"from {filename}"
            )
            if unknown_fields:
                self.console_write(
                    "GRAPH_SETUP ignored unknown field(s): " + ", ".join(unknown_fields)
                )
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            messagebox.showerror("Load graph setup", f"Could not load setup:\n{exc}")

    GRAPH_COLORS = ("#1565c0", "#c62828", "#2e7d32", "#6a1b9a", "#ef6c00")
    GRAPH_MIN_HEIGHT = 210
    GRAPH_MAX_HEIGHT = 330

    def _graph_configured(self, event):
        size = (event.width, event.height)
        if size != self._graph_size:
            self._graph_size = size
            self.schedule_graph_draw()

    def schedule_graph_draw(self):
        """Redraw the graphs shortly (debounced, e.g. while resizing)."""
        if self._closed:
            return
        if self._graph_job is not None:
            try:
                self.after_cancel(self._graph_job)
            except tk.TclError:
                pass
        self._graph_job = self.after(60, self.draw_graphs)

    @staticmethod
    def _component_names(count):
        if count == 3:
            return ["x", "y", "z"]
        if count == 4:
            return ["x", "y", "z", "w"]
        return [f"[{index}]" for index in range(count)]

    @staticmethod
    def _time_label(timestamp, step):
        text = _dt.datetime.fromtimestamp(timestamp).strftime("%H:%M:%S.%f")
        if step >= 1:
            return text[:8]
        decimals = 1 if step >= 0.1 else 2 if step >= 0.01 else 3
        return text[:9 + decimals]

    def draw_graphs(self):
        """Draw every selected field: one graph each, sized to the view.

        Graphs fill the canvas when few are selected and scroll (scrollbar
        or wheel) when many are. Axes use round ticks; labels that would
        overlap are skipped; vector components get a coloured legend.
        Skipped while the Graphs tab is hidden (drawn again when shown).
        """
        self._graph_job = None
        if not hasattr(self, "graph_canvas") or self._closed:
            return
        canvas = self.graph_canvas
        if not canvas.winfo_ismapped():
            self._graphs_dirty = True
            return
        canvas.delete("all")
        selected = [key for key, variable in self.check_vars.items() if variable.get()]
        width = canvas.winfo_width()
        if width <= 1:
            width = canvas.winfo_reqwidth()
        view_height = max(canvas.winfo_height(), 1)
        if not selected:
            canvas.configure(scrollregion=(0, 0, width, view_height))
            canvas.create_text(width // 2, 40, text="No fields selected", fill="#555555",
                               width=max(100, width - 20))
            return
        graph_height = max(self.GRAPH_MIN_HEIGHT,
                           min(self.GRAPH_MAX_HEIGHT, view_height // len(selected)))
        canvas.configure(scrollregion=(0, 0, width, graph_height * len(selected)))
        for graph_index, key in enumerate(selected):
            self._draw_graph(canvas, key, self.history.get(key, []),
                             graph_index * graph_height, width, graph_height)

    def _draw_graph(self, canvas, key, samples, top, width, height):
        import tkinter.font as tkfont
        font = tkfont.nametofont("TkDefaultFont")
        line = font.metrics("linespace")
        width = max(int(width), 200)
        bottom = top + height
        canvas.create_rectangle(4, top + 4, width - 4, bottom - 4, outline="#cccccc")
        title = elide(key, width - 28, font.measure)
        header_y = top + 8 + line / 2
        canvas.create_text(14, header_y, anchor="w", text=title, fill="#222222")
        if not samples:
            canvas.create_text(width // 2, top + height // 2, text="Waiting for data", fill="#777777")
            return
        times = [sample[0] for sample in samples]
        values = [sample[1] for sample in samples]
        components = len(values[-1]) if isinstance(values[-1], (list, tuple)) else 1
        series = [[] for _ in range(components)]
        for timestamp, value in samples:
            parts = value if isinstance(value, (list, tuple)) else [value]
            for component in range(min(components, len(parts))):
                try:
                    number = float(parts[component])
                except (TypeError, ValueError):
                    continue
                if number == number and abs(number) != float("inf"):
                    series[component].append((timestamp, number))
        flat = [number for points in series for _t, number in points]
        if not flat:
            canvas.create_text(width // 2, top + height // 2, text="No numeric data", fill="#777777")
            return

        # Legend for vector components, right-aligned on the header line
        # when it fits beside the title.
        if components > 1:
            names = self._component_names(components)
            legend_width = sum(font.measure(name) + 26 for name in names)
            if font.measure(title) + legend_width + 40 <= width - 28:
                x = width - 14 - legend_width
                for component, name in enumerate(names):
                    color = self.GRAPH_COLORS[component % len(self.GRAPH_COLORS)]
                    canvas.create_line(x, header_y, x + 12, header_y, fill=color, width=2)
                    canvas.create_text(x + 16, header_y, anchor="w", text=name, fill=color)
                    x += font.measure(name) + 26

        plot_top = top + 14 + line
        plot_bottom = bottom - (3 * line + 22)
        plot_height = plot_bottom - plot_top
        low, high = min(flat), max(flat)
        if high <= low:
            pad = max(0.5, abs(low) * 0.05)
            low, high = low - pad, high + pad
        target = max(2, min(8, int(plot_height // (line * 1.6))))
        value_step, value_ticks = nice_ticks(low, high, target)
        low, high = value_ticks[0], value_ticks[-1]
        if high <= low:
            high = low + value_step
        labels = [tick_text(value, value_step) for value in value_ticks]
        left = max(48, max(font.measure(text) for text in labels) + 16)
        right = width - 16
        if plot_height < 16 or right - left < 40:
            return

        def to_y(value):
            return plot_bottom - (value - low) / (high - low) * plot_height

        last_y = None
        for value, text in zip(value_ticks, labels):
            y = to_y(value)
            canvas.create_line(left, y, right, y, fill="#e6e6e6")
            if last_y is None or abs(last_y - y) >= line:
                canvas.create_text(left - 6, y, anchor="e", text=text, fill="#555555")
                last_y = y

        start, end = times[0], times[-1]
        span = max(end - start, 0.001)
        time_target = max(2, min(8, int((right - left) // 110)))
        time_step, time_ticks = nice_ticks(start, start + span, time_target, inside=True)
        label_right = -1e9
        for tick in time_ticks:
            x = left + (tick - start) / span * (right - left)
            canvas.create_line(x, plot_top, x, plot_bottom, fill="#eeeeee")
            text = self._time_label(tick, time_step)
            half = font.measure(text) / 2
            if x - half >= label_right + 8 and x + half <= width - 6:
                canvas.create_text(x, plot_bottom + 4, anchor="n", text=text, fill="#555555")
                label_right = x + half
        canvas.create_rectangle(left, plot_top, right, plot_bottom, outline="#bbbbbb")

        for component, points in enumerate(series):
            reduced = decimate_minmax(points, start, start + span, int(right - left))
            coordinates = []
            for timestamp, number in reduced:
                coordinates.append(left + (timestamp - start) / span * (right - left))
                coordinates.append(to_y(number))
            color = self.GRAPH_COLORS[component % len(self.GRAPH_COLORS)]
            if len(coordinates) >= 4:
                canvas.create_line(*coordinates, fill=color, width=1.5)
            elif coordinates:
                x, y = coordinates
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)

        # Exact first/last sample times, then the latest value (one line,
        # shortened rather than spilling into the next graph).
        endpoints_y = plot_bottom + 8 + line
        start_label = _dt.datetime.fromtimestamp(start).strftime("%H:%M:%S.%f")[:-3]
        end_label = _dt.datetime.fromtimestamp(end).strftime("%H:%M:%S.%f")[:-3]
        if font.measure(start_label) + font.measure(end_label) + 20 <= right - left:
            canvas.create_text(left, endpoints_y, anchor="nw", text=start_label, fill="#555555")
        canvas.create_text(right, endpoints_y, anchor="ne", text=end_label, fill="#555555")
        latest_field = key.rsplit(".", 1)[-1]
        latest = self.format_display_value(latest_field, values[-1])
        canvas.create_text(14, endpoints_y + line + 2, anchor="nw", fill="#222222",
                           text=elide(f"Latest: {latest}", width - 28, font.measure))

    def disconnect(self):
        if self._auto_job is not None:
            self._auto_job.cancel()
            self._auto_job = None
        self.stop_event.set()
        for serial_port in self.serials.values():
            try:
                serial_port.close()
            except Exception:
                pass
        self.serials.clear()
        self.refresh_command_targets()
        for tcp_socket in self.tcp_sockets.values():
            try:
                tcp_socket.close()
            except Exception:
                pass
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
        self.connect_button.configure(text="Connect")
        self.status_label.configure(text="Disconnected")

    def set_window_title(self, text):
        if self.standalone:
            self.title(text)
        elif self.on_title is not None:
            self.on_title(text)

    def bind_scroll_wheel(self, canvas):
        """Wheel-scroll ``canvas`` from anywhere over it or its contents.

        Binds the canvas and every descendant instead of bind_all, so the
        monitor never takes over the wheel of a host app. At either end the
        event passes on (e.g. to an outer scroller). Widget bindings (not
        bind_class) so Tk drops them, and the monitor, when it is destroyed.
        """

        def scroll(event, direction=None):
            if direction is None:
                if not event.delta:
                    return None
                direction = -1 if event.delta > 0 else 1
            first, last = canvas.yview()
            if (direction < 0 and first <= 0.0) or (direction > 0 and last >= 1.0):
                return None
            canvas.yview_scroll(direction, "units")
            return "break"

        def add(widget):
            widget.bind("<MouseWheel>", scroll, add="+")
            widget.bind("<Button-4>", lambda event: scroll(event, -1), add="+")
            widget.bind("<Button-5>", lambda event: scroll(event, 1), add="+")
            for child in widget.winfo_children():
                add(child)

        add(canvas)

    def stream_feeder(self, source_name):
        """Return feed(bytes) that decodes a byte stream owned by someone else.

        Used to mirror a connection another tool holds (witnessOS shares the
        HITL bench's Witness TCP connection, since ser2net serves one client).
        Decoding, publishing and display are exactly as for read_tcp. Call
        feed from one thread at a time; nothing is ever sent to Witness.
        """
        nmea = NmeaExtractor()
        decoder = PacketDecoder(
            lambda p: self.on_decoded_packet(source_name, p),
            lambda message: self.events.put(("decoder_error", (source_name, message))),
            lambda packet_id, count: self.events.put(("host_packet", (source_name, packet_id, count))),
        )

        def feed(data):
            if self._closed:
                return
            self.events.put(("bytes", (source_name, bytes(data))))
            decoder.feed(data)
            self._queue_nmea(source_name, nmea, data)

        return feed

    def close(self):
        self._closed = True
        for job in (self._auto_after, self._test_start_id, self._events_after,
                    self._age_after, self._graph_job):
            if job is not None:
                try:
                    self.after_cancel(job)
                except tk.TclError:
                    pass
        self.stop_test()
        self.disconnect()
        self.publisher.close()
        if self.standalone:
            self.destroy()
        self._release_tk_variables()

    def _release_tk_variables(self):
        """Free the monitor's Tk variables now, on the Tk thread.

        The monitor is kept alive by reference cycles (bound-method
        callbacks), so otherwise its ~200 Variables are freed whenever the
        cyclic GC runs, possibly on a worker thread, where each Variable's
        cleanup blocks for about a second ("main thread is not in main loop").
        """
        for mapping in (self.values, self.check_vars):
            mapping.clear()
        for name, value in list(vars(self).items()):
            if isinstance(value, tk.Variable):
                delattr(self, name)


class SerialMonitor(MonitorBase, tk.Tk):
    """The monitor in its own window."""

    def __init__(self, **options):
        tk.Tk.__init__(self)
        self.title(WINDOW_TITLE)
        self.geometry(f"{min(920, self.winfo_screenwidth() - 40)}x"
                      f"{min(720, self.winfo_screenheight() - 80)}")
        self._init_monitor(**options)


class MonitorPanel(MonitorBase, ttk.Frame):
    """The monitor as a frame inside another window (witnessOS).

    The host owns the window: it calls close() on exit (which stops inputs,
    the test source and the feed but leaves the window to the host) and
    gets title changes through ``on_title``.
    """

    standalone = False

    def __init__(self, master, *, on_title=None, **options):
        ttk.Frame.__init__(self, master)
        self.on_title = on_title
        self._init_monitor(**options)


def command_line(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--test", action="store_true", help="start the built-in simulated flight (no hardware)")
    parser.add_argument("--test-file", metavar="JSON", help="start timed hex-packet replay from a JSON file")
    parser.add_argument("--test-speed", type=float, default=1.0, help="playback multiplier, 0.1 to 10 (default: 1)")
    parser.add_argument("--test-once", action="store_true", help="stop at end of test instead of looping")
    parser.add_argument("--no-auto-connect", action="store_true", help="disable automatic Witness USB connections")
    args = parser.parse_args(argv)
    try:
        validate_speed(args.test_speed)
    except ValueError as exc:
        parser.error(str(exc))
    return args


if __name__ == "__main__":
    args = command_line()
    SerialMonitor(test_mode=args.test or args.test_file is not None,
                  test_source=args.test_file or "", test_speed=args.test_speed,
                  test_loop=not args.test_once, auto_connect=not args.no_auto_connect).mainloop()
