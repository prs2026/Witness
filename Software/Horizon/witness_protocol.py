"""Witness serial protocol (comms.h): packet decoding and command frames.

Pure Python, no GUI: used by the monitor core (monitor_core.py), the Qt
monitor (main.py re-exports everything here), tests and HITL tooling.
"""

from __future__ import annotations

import re
import struct

import bit_model

# Commands currently defined by comms.h. Most command packets are sent as:
# ID (0x05) | status (2) | uptime (4) | command (2) | EOF (0x0A).
# Every shortcut uses a full frame so it works directly or through Aperture.
COMMAND_PACKET_ID = 0x05
COMMAND_PACKET_EOF = 0x0A
RESET_WITNESS_LABEL = "False flight on pad: reset Witness"
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
    ("Clear Logs (fast)", 0x686A, True),
    ("Erase Logs (full, slow)", 0x6869, True),
    # Receiver forgets ephemerides, almanac, position and time (Pad Idle only).
    ("GPS cold start", 0x4752, True),
    # One-way states: 0x7300 acks, then restarts Witness, and only while Landed
    # (Pad Idle: no-op; refused in Boost, Coast and Descent). The monitor enables
    # this button only while Witness reports Landed.
    (RESET_WITNESS_LABEL, 0x7300, True),
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

# IMU calibration (comms.h IRIS_COMMAND_IMU_CALIBRATION): compact 3-byte USB
# commands 05 43 <arg>, accepted only on a direct Witness USB link, in Pad Idle
# with real sensors (not through Aperture/LoRa). Results are Witness log lines.
IMU_CALIBRATION_COMMAND = 0x43
IMU_CALIBRATION_COMMANDS = (
    ("IMU cal: gyro bias (keep still)", 0x47),
    ("IMU cal: capture face", 0x41),
    ("IMU cal: clear", 0x58),
    ("IMU cal: status", 0x53),
)


def build_imu_calibration_packet(argument: int) -> bytes:
    if argument not in {value for _label, value in IMU_CALIBRATION_COMMANDS}:
        raise ValueError(f"unknown IMU calibration argument 0x{argument:02X}")
    return bytes((COMMAND_PACKET_ID, IMU_CALIBRATION_COMMAND, argument))


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
MAINTENANCE_MASK = 0x20  # bit 5: reserved; the flight image never sets it (maintenance is a separate image: 0x08 beacon)
RESUMED_MASK = 0x40      # bit 6: resumed an in-flight state after an MCU reset
LOG_SLOTS_FULL_MASK = 0x80  # bit 7: both NAND log slots hold a flight: NOT LOGGING until erase/clear (0x6869/0x686A)
STATUS_BYTE1_FLAG_LABELS = (
    (GPS_LOCKED_MASK, "GPS lock"),
    (HITL_ACTIVE_MASK, "HITL (SIM)"),
    (MAINTENANCE_MASK, "MAINTENANCE"),
    (RESUMED_MASK, "RESUMED AFTER RESET"),
    (LOG_SLOTS_FULL_MASK, "LOG FULL: NOT LOGGING"),
)


def status_maintenance(status) -> bool:
    """Witness is in its maintenance (Wi-Fi) boot: not logging, not flight ready."""
    return isinstance(status, int) and bool(status & MAINTENANCE_MASK)


MAINT_MODES = {0: "Wi-Fi", 1: "USB export"}
MAINT_FLAGS = ((0x01, "ap_up"), (0x02, "usb_disk"), (0x04, "store_mounted"), (0x08, "recovery"),
               (0x10, "unconfirmed"), (0x20, "radio_ok"))
MAINT_BEACON_PAYLOAD_LENGTH = 17


def decode_maintenance_status(payload) -> dict | None:
    """0x08 maintenance status beacon (comms.h IRIS_PACKET_ID_MAINTENANCE_STATUS), 17-byte payload."""
    if len(payload) != MAINT_BEACON_PAYLOAD_LENGTH:
        return None
    flags = payload[2]
    return {"packet": "maintenance_status", "beacon_version": payload[0], "mode": payload[1],
            "mode_name": MAINT_MODES.get(payload[1], f"0x{payload[1]:02X}"), "flags": flags,
            **{name: bool(flags & mask) for mask, name in MAINT_FLAGS},
            "store_version": payload[3], "command_set": payload[4], "radio_settings": payload[5],
            "maint_image_id": (payload[6] << 8) | payload[7], "flight_image_id": (payload[8] << 8) | payload[9],
            "battery_voltage": round(((payload[10] << 8) | payload[11]) / 1000.0, 3),
            "uptime": int.from_bytes(bytes(payload[12:16]), "big"), "log_files": payload[16]}


LANDED_FLAGS = ((0x01, "position_valid"), (0x02, "stale"), (0x04, "ping_reply"), (0x08, "utc_aligned"))
LANDED_PAYLOAD_LENGTH = 26
LANDED_FIX_AGE_NONE = 0xFFFFFFFF


def decode_landed_position(payload) -> dict | None:
    """0x09 landed position packet (comms.h IRIS_PACKET_ID_LANDED_POSITION), 26-byte payload."""
    if len(payload) != LANDED_PAYLOAD_LENGTH:
        return None
    flags = payload[22]
    age = number(payload[18:22])
    valid = bool(flags & 0x01)
    return {"packet": "landed_position", "status": number(payload[0:2]), "flight_state": payload[1] & 0x07,
            "uptime": number(payload[2:6]),
            "latitude": round(signed_number(payload[6:10]) * 1e-7, 7) if valid else None,
            "longitude": round(signed_number(payload[10:14]) * 1e-7, 7) if valid else None,
            "gps_altitude": round(float32(payload[14:18]), 3) if valid else None,
            "fix_age": None if age == LANDED_FIX_AGE_NONE else age,
            "flags": flags, **{name: bool(flags & mask) for mask, name in LANDED_FLAGS},
            "satellites": payload[23], "battery_voltage": round(number(payload[24:26]) / 1000.0, 3)}


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


# 0x06 command acknowledgement result codes (comms.h IRIS_ACK_*).
ACK_RESULTS = {0: "ok", 1: "refused", 2: "bad argument", 3: "unknown command", 4: "busy",
               5: "incomplete", 6: "bad CRC", 7: "bad format"}
# 0x07 transfer status states.
# Ack detail of a refused 0x5746 "on" / 0x6868 (switch to the maintenance image).
MAINT_REFUSE_REASONS = {
    0x0001: "no valid maintenance image installed", 0x0002: "maintenance image-info missing/corrupt",
    0x0003: "maintenance image cannot read the log storage format", 0x0004: "LoRa settings differ",
    0x0005: "command set differs", 0x0006: "image-info layout differs",
    0x0007: "could not select the maintenance boot slot", 0x0008: "switch already pending",
}
TRANSFER_STATES = {0: "idle", 1: "receiving", 2: "complete", 3: "failed"}


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
    if packet_id == 0x06 and len(payload) == 11:
        # Command acknowledgement (comms.h IRIS_PACKET_ID_COMMAND_ACK).
        return {"packet": "command_ack", "command_value": number(payload[0:2]),
                "command_data": bytes(payload[2:8]).hex(" "), "result": payload[8],
                "result_name": ACK_RESULTS.get(payload[8], f"0x{payload[8]:02X}"),
                "detail": number(payload[9:11]),
                **({"detail_name": MAINT_REFUSE_REASONS[number(payload[9:11])]}
                   if payload[8] == 1 and number(payload[0:2]) in (0x5746, 0x6868)
                   and number(payload[9:11]) in MAINT_REFUSE_REASONS else {})}
    if packet_id == 0x07 and len(payload) == 74:
        # GPS assist transfer status (comms.h IRIS_PACKET_ID_TRANSFER_STATUS).
        start = number(payload[8:10])
        bitmap = payload[10:74]
        return {"packet": "transfer_status", "transfer_id": payload[0],
                "state": TRANSFER_STATES.get(payload[1], payload[1]), "length": number(payload[2:4]),
                "chunks_expected": number(payload[4:6]), "chunks_received": number(payload[6:8]),
                "window_start": start, "bitmap": bytes(bitmap).hex()}
    if packet_id == 0x08:
        return decode_maintenance_status(payload)
    if packet_id == 0x09:
        return decode_landed_position(payload)
    if packet_id == 0x0B:
        return bit_model.decode_summary(payload)
    if packet_id == 0x10:
        return bit_model.decode_detail(payload)
    if packet_id == 0xE1 and len(payload) == 12:
        return {
            "packet": "iris_debug", "status": number(payload[0:2]),
            "uptime": number(payload[2:6]), "mcu_temperature": signed_number(payload[6:7]),
            "iris_battery_voltage": round(number(payload[7:9]) * BATTERY_V_PER_COUNT, 3),
            "current_sense": [round(payload[i] * CURRENT_A_PER_COUNT, 2) for i in (9, 10, 11)],
        }
    return None


# Bytes that can appear in the text Witness mirrors onto the same stream
# (NMEA "$...\r\n", ESP-IDF log lines, ANSI colour codes). A skipped run of
# mostly these is text, not corruption: it is counted but raises no error event.
_TEXT_BYTES = bytes(range(0x20, 0x7F)) + b"\t\n\r\x1b"


class PacketDecoder:
    """Decode the single ID-prefixed serial protocol from comms.h.

    Resync is linear: every input byte is examined a bounded number of times.
    A byte that is not a packet ID, or an ID whose fixed-length frame does not
    end in EOF 0x0A, is skipped on its own (never "the whole frame"), and a
    run of skipped bytes produces ONE summary on_error call (none at all when
    the run is text). A parse/callback exception loses only that frame.
    Counters (see stats()): packets, packets_by_id, text_bytes_skipped,
    garbage_bytes_skipped, unknown_ids (skipped byte value -> count),
    bad_frames (known packet id -> EOF/length mismatches), undecoded (framed
    but not decoded), callback_errors, bytes_in.
    """
    # Sensor frames in the supplied capture are 25 bytes total: ID + 23-byte
    # payload + EOF.  Keep the 21-byte header-defined form as a fallback so
    # older producers remain readable while the firmware/header are reconciled.
    # State: 63-byte payload (65-byte frame, with GPS satellites) is current;
    # 62 bytes is the legacy 64-byte frame from older firmware/logs.
    payload_lengths = {0x01: (23, 21), 0x02: (63, 62), 0x03: (17,), 0xFF: (6,),
                       0xF0: (27, 11), 0x05: (8,), 0xE0: (101,), 0xE1: (12,),
                       0x06: (11,), 0x07: (74,), 0x08: (17,), 0x09: (26,),
                       # BIT summary / detail: framed by length so their data bytes (which can equal
                       # any packet ID) are never scanned for other frames.
                       0x0B: (12,), 0x10: (99,)}
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
        # Linear scan: jump straight to the next byte that could start a frame.
        self._id_search = re.compile(
            b"[" + b"".join(re.escape(bytes((i,))) for i in sorted(self.frame_lengths)) + b"]").search
        self.bytes_in = 0
        self.packets = 0
        self.packets_by_id = {}
        self.text_bytes_skipped = 0
        self.garbage_bytes_skipped = 0
        self.unknown_ids = {}        # skipped non-text byte value -> count
        self.bad_frames = {}         # known packet id whose EOF/length did not match -> count
        self.undecoded = {}          # framed ok but parse_packet returned None -> count
        self.callback_errors = 0

    def stats(self):
        """Counters for the UI / logs (copies; safe to read from another thread)."""
        return {"bytes_in": self.bytes_in, "packets": self.packets,
                "packets_by_id": dict(self.packets_by_id),
                "text_bytes_skipped": self.text_bytes_skipped,
                "garbage_bytes_skipped": self.garbage_bytes_skipped,
                "unknown_ids": dict(self.unknown_ids), "bad_frames": dict(self.bad_frames),
                "undecoded": dict(self.undecoded), "callback_errors": self.callback_errors}

    def _error(self, message):
        try:
            self.on_error(message)
        except Exception:  # noqa: BLE001 - reporting must never break decoding
            pass

    def _match(self, buf, pos, end, packet_id):
        """(payload_length, False) for a valid frame at pos, (None, True) when more
        bytes are needed to decide, (None, False) when the EOF never matches."""
        lengths = self.frame_lengths[packet_id]
        for index, payload_length in enumerate(lengths):
            eof_index = pos + 1 + payload_length
            if end > eof_index and buf[eof_index] == self.eof:
                if index and packet_id in self.confirm_shorter:
                    if end <= pos + 1 + max(lengths):
                        return None, True  # Wait: the longer form may still complete.
                return payload_length, False
        if end < pos + 2 + max(lengths):
            return None, True              # Do not reject a split frame.
        return None, False

    def _flush_gap(self, buf, start, stop):
        """Account for the skipped run buf[start:stop]: counters, one event if binary."""
        count = stop - start
        if count <= 0:
            return
        run = bytes(buf[start:stop])
        binary = run.translate(None, _TEXT_BYTES)         # only the non-text bytes remain
        if len(binary) * 10 <= count:                     # >= 90 % text: NMEA / log lines
            self.text_bytes_skipped += count
            return
        self.garbage_bytes_skipped += count
        for value in binary:
            self.unknown_ids[value] = self.unknown_ids.get(value, 0) + 1
        self._error(f"discarded {count} byte(s) of non-packet data ({len(binary)} non-text), "
                    f"first bytes {run[:8].hex(' ')}")

    def feed(self, data: bytes):
        buf = self.buffer
        buf.extend(data)
        self.bytes_in += len(data)
        end = len(buf)
        pos = 0
        gap = None                  # start of the current run of skipped bytes
        try:
            while pos < end:
                packet_id = buf[pos]
                if packet_id not in self.frame_lengths:
                    found = self._id_search(buf, pos)
                    if gap is None:
                        gap = pos
                    pos = end if found is None else found.start()
                    continue
                payload_length, need_more = self._match(buf, pos, end, packet_id)
                if payload_length is None:
                    if need_more:
                        break              # split frame: wait for the rest
                    # EOF never matches: skip just this ID byte and rescan.
                    self.bad_frames[packet_id] = self.bad_frames.get(packet_id, 0) + 1
                    if gap is None:
                        gap = pos
                    pos += 1
                    continue
                if gap is not None:
                    self._flush_gap(buf, gap, pos)
                    gap = None
                frame = bytes(buf[pos + 1:pos + 1 + payload_length])
                pos += payload_length + 2   # consumed BEFORE any callback can raise
                try:
                    if packet_id in self.host_payload_lengths:
                        self.host_packet_count += 1
                        self.on_host_packet(packet_id, self.host_packet_count)
                        continue
                    parsed = parse_packet(packet_id, frame)
                    if parsed:
                        self.packets += 1
                        self.packets_by_id[packet_id] = self.packets_by_id.get(packet_id, 0) + 1
                        self.on_packet(parsed)
                    else:
                        self.undecoded[packet_id] = self.undecoded.get(packet_id, 0) + 1
                except Exception as exc:  # noqa: BLE001 - lose this frame only, never the next
                    self.callback_errors += 1
                    self._error(f"decoder rejected packet 0x{packet_id:02x}: {exc}; "
                                f"frame={frame[:32].hex(' ')}")
            if gap is not None:
                self._flush_gap(buf, gap, pos)
        finally:
            del buf[:pos]


def build_command_packet(opcode: int) -> bytes:
    """Full 10-byte command frame: ID | status(2) | uptime(4) | opcode(2) | EOF."""
    if not 0 <= opcode <= 0xFFFF:
        raise ValueError("command opcode must fit in 16 bits")
    return (
        bytes((COMMAND_PACKET_ID,))
        + bytes(6)  # Zero status and uptime; the receiver ignores these.
        + opcode.to_bytes(2, byteorder="big")
        + bytes((COMMAND_PACKET_EOF,))
    )


# Pad heading (comms.h IRIS_COMMAND_PAD_HEADING): which body axis points to
# which azimuth while the rocket is on the rail. Witness has no magnetometer,
# so without it the heading is unknown until GPS shows the course.
PAD_HEADING_COMMAND = 0x4844
PAD_HEADING_AXES = {"+Y": 0, "-Y": 1, "+Z": 2, "-Z": 3}
PAD_HEADING_CLEAR = 0xFF


def build_pad_heading_packet(axis: str | None, azimuth_deg: float = 0.0) -> bytes:
    """Command frame setting the pad heading: `axis` ("+Y", "-Y", "+Z", "-Z")
    points to `azimuth_deg` (clockwise from true north). axis None clears it.
    Data bytes ride in the status/uptime fields: d0 axis code, d1-d2
    azimuth in 0.01 deg (u16 BE), d3-d5 zero."""
    if axis is None:
        code, centideg = PAD_HEADING_CLEAR, 0
    else:
        if axis not in PAD_HEADING_AXES:
            raise ValueError(f"axis must be one of {sorted(PAD_HEADING_AXES)}")
        code = PAD_HEADING_AXES[axis]
        centideg = int(round((azimuth_deg % 360.0) * 100.0)) % 36000
    data = bytes((code,)) + centideg.to_bytes(2, "big") + bytes(3)
    return (bytes((COMMAND_PACKET_ID,)) + data + PAD_HEADING_COMMAND.to_bytes(2, "big")
            + bytes((COMMAND_PACKET_EOF,)))


# comms.h IRIS_COMMAND_RADIO_POWER: RAM-only radio drive level. Both radios
# boot at the bench-safe minimum (-17 dBm drive on the SX1262 low-power PA).
RADIO_POWER_COMMAND = 0x5250
RADIO_POWER_TARGETS = {"witness": 0x00, "ground": 0x01}
RADIO_POWER_MIN_DBM, RADIO_POWER_MAX_DBM = -17, 3   # below -9: SX1262 low-power PA
RADIO_FRONT_END = {None: 0x00, True: 0x01, False: 0x02}   # unchanged / on / off (RF_EN)
RADIO_FRONT_END_GAIN_DB = 26     # Ra-01SH-P PA, ~+29 dBm saturated


def build_radio_power_packet(target: str, drive_dbm: int, front_end: bool | None = None) -> bytes:
    """Command frame setting a radio's SX1262 drive (dBm, -17..+3).
    front_end: None leaves RF_EN as is, True turns the Ra-01SH-P front end on,
    False powers it down (bench only: no PA current, signal passes attenuated).
    target "witness" (Witness applies and acks, USB or radio) or "ground"
    (Aperture applies it to its own radio and acks on USB; not transmitted).
    d0 target, d1 drive as int8, d2 front end, d3-d5 zero."""
    if target not in RADIO_POWER_TARGETS:
        raise ValueError(f"target must be one of {sorted(RADIO_POWER_TARGETS)}")
    drive = int(drive_dbm)
    if not RADIO_POWER_MIN_DBM <= drive <= RADIO_POWER_MAX_DBM:
        raise ValueError(f"drive must be {RADIO_POWER_MIN_DBM}..{RADIO_POWER_MAX_DBM} dBm "
                         "(Ra-01SH-P front-end input limit)")
    data = bytes((RADIO_POWER_TARGETS[target], drive & 0xFF, RADIO_FRONT_END[front_end])) + bytes(3)
    return (bytes((COMMAND_PACKET_ID,)) + data + RADIO_POWER_COMMAND.to_bytes(2, "big")
            + bytes((COMMAND_PACKET_EOF,)))


# comms.h IRIS_COMMAND_PING (0x0022): Iris answers PONG over CAN; Witness in
# Landed (radio only) also replies with a 0x09 landed position packet, at most
# once per 5 s (anything else gets IRIS_ACK_UNKNOWN as before).
PING_COMMAND = 0x0022


def build_ping_packet() -> bytes:
    """Standard 10-byte command frame for IRIS_COMMAND_PING (no arguments)."""
    return (bytes((COMMAND_PACKET_ID,)) + bytes(6) + PING_COMMAND.to_bytes(2, "big")
            + bytes((COMMAND_PACKET_EOF,)))


# comms.h IRIS_COMMAND_WIFI: Wi-Fi access point for OTA firmware update and log
# download. Pad Idle or Landed only; LoRa transmission stops while it is on.
WIFI_COMMAND = 0x5746
WIFI_TX_MIN_DBM, WIFI_TX_MAX_DBM = 2, 20
WIFI_AUTO_OFF_MAX_MINUTES = 240


def build_wifi_packet(on: bool, tx_dbm: int = 0, minutes: int = 0) -> bytes:
    """Command frame turning Witness's Wi-Fi access point on or off.
    d0 0 off / 1 on, d1 Wi-Fi maximum TX power in dBm as int8 (0 keeps the
    firmware default, +8; otherwise 2..20), d2 auto-off minutes (0 = default
    15, else 1..240), d3-d5 zero. USB or radio; Witness acks (0x06)."""
    tx = int(tx_dbm)
    if tx != 0 and not WIFI_TX_MIN_DBM <= tx <= WIFI_TX_MAX_DBM:
        raise ValueError(f"tx_dbm must be 0 (default) or {WIFI_TX_MIN_DBM}..{WIFI_TX_MAX_DBM} dBm")
    mins = int(minutes)
    if not 0 <= mins <= WIFI_AUTO_OFF_MAX_MINUTES:
        raise ValueError(f"minutes must be 0 (default) or 1..{WIFI_AUTO_OFF_MAX_MINUTES}")
    data = bytes((1 if on else 0, tx & 0xFF, mins)) + bytes(3)
    return (bytes((COMMAND_PACKET_ID,)) + data + WIFI_COMMAND.to_bytes(2, "big")
            + bytes((COMMAND_PACKET_EOF,)))


# comms.h IRIS_COMMAND_BIT_RUN / IRIS_COMMAND_BIT_REPORT (built-in self test). Horizon sends BIT_RUN only
# from the "Run BIT" button (Pad Idle), never by itself; BIT_REPORT only asks for a re-send.
BIT_RUN_COMMAND = 0x5442
BIT_REPORT_COMMAND = 0x5452
BIT_REPORT_SUMMARY = 0xFF


def build_bit_run_packet(nonce: int = 0) -> bytes:
    """Command frame for a full commanded BIT with no options: d0 0 (full), d1 0, d2-d5 nonce (u32 BE,
    echoed in the 0x06 ack so a late ack of an earlier run is not taken for this one)."""
    return (bytes((COMMAND_PACKET_ID, 0x00, 0x00)) + (int(nonce) & 0xFFFFFFFF).to_bytes(4, "big")
            + BIT_RUN_COMMAND.to_bytes(2, "big") + bytes((COMMAND_PACKET_EOF,)))


def build_bit_report_packet(page: int = BIT_REPORT_SUMMARY) -> bytes:
    """Command frame asking Witness to re-send the BIT summary (page 0xFF) or detail page `page`."""
    if not 0 <= page <= 0xFF:
        raise ValueError("BIT report page must fit in one byte")
    return (bytes((COMMAND_PACKET_ID, page)) + bytes(5) + BIT_REPORT_COMMAND.to_bytes(2, "big")
            + bytes((COMMAND_PACKET_EOF,)))


def build_quick_command_packet(opcode: int) -> bytes:
    return build_command_packet(opcode)


def parse_hex_command(text: str) -> bytes:
    """Bytes from a typed hex command ("05 00 ..", "0x05,0x00", ...)."""
    text = text.strip().replace(",", " ").replace("0x", "").replace("0X", "")
    if not text:
        raise ValueError("command is empty")
    return bytes.fromhex(text)


# ----------------------------------------------------------- display helpers

RSSI_RADIO_DOWN = -128


def format_display_value(field, value):
    if value is None:
        return "null"
    if isinstance(value, int) and (field == "status" or field.endswith("_status")):
        return f"0b{value:016b}"
    if field == "rssi" and value == RSSI_RADIO_DOWN:
        return "RECEIVER RADIO FAILED (-128)"   # Aperture 0xF0: -128 means its LoRa radio is down
    return str(value)


def format_gps_satellites(packet):
    """Satellite count with the status-byte GPS lock flag beside it."""
    satellites = packet.get("gps_satellites")
    status = packet.get("status")
    lock = status_gps_locked(status) if isinstance(status, int) else "?"
    return f"{'null' if satellites is None else satellites}  lock {lock}"


def format_packet(packet):
    fields = ", ".join(
        f"{name}={format_display_value(name, value)}"
        for name, value in packet.items() if name != "packet"
    )
    return f"{packet['packet']}: {fields}"


# Field lists mirror comms.h exactly, in wire order, per packet type.
SECTION_SPECS = {
    "sensors": [
        ("status", "Status"), ("uptime", "Uptime"),
        ("filtered_accel", "Filtered accel [g; x, y, z]"),
        ("gyro", "Gyro [dps; x, y, z]"),
        ("sensor_trailing_u16", "Sensor trailing bytes [u16]"),
    ],
    "state": [
        ("status", "Status"), ("uptime", "Uptime"),
        ("orientation", "Orientation [x, y, z, w; 0 = n/a]"),
        ("position", "Kalman position [m; E, N, U from pad]"),
        ("velocity", "Kalman velocity [m/s; E, N, U]"),
        ("latitude", "Latitude [deg]"), ("longitude", "Longitude [deg]"),
        ("gps_altitude", "GPS altitude [m MSL]"),
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
        ("orientation", "Orientation [x, y, z, w; 0 = n/a]"),
        ("position", "Kalman position [m; E, N, U from pad]"),
        ("velocity", "Kalman velocity [m/s; E, N, U]"),
        ("low_accel", "LSM6 low accel [g; x, y, z]"),
        ("lsm6_temperature", "LSM6 temperature [C]"),
        ("high_accel", "LSM6 high accel [g; x, y, z]"),
        ("gyro", "Gyro [dps; x, y, z]"),
        ("pressure", "MS5607 pressure [Pa]"),
        ("ms5607_temperature", "MS5607 temperature [C]"),
        ("kalman_altitude", "Kalman altitude above pad [m]"),
        ("latitude", "Latitude [deg]"), ("longitude", "Longitude [deg]"),
        ("gps_time", "GPS time [Unix s]"), ("gps_altitude", "GPS altitude [m MSL]"),
    ],
    "maintenance_status": [
        ("mode_name", "Maintenance mode"), ("uptime", "Uptime"), ("ap_up", "Wi-Fi AP up"),
        ("usb_disk", "USB disk"), ("store_mounted", "Log store mounted"), ("recovery", "Recovery (no flight image)"),
        ("unconfirmed", "Image unconfirmed"), ("radio_ok", "Radio ok"), ("log_files", "Log files"),
        ("battery_voltage", "Battery [V]"), ("store_version", "Store version"),
        ("command_set", "Command set"), ("radio_settings", "Radio settings"),
        ("maint_image_id", "Maintenance image id"), ("flight_image_id", "Flight image id"),
    ],
    "landed_position": [
        ("uptime", "Uptime"), ("latitude", "Latitude [deg]"), ("longitude", "Longitude [deg]"),
        ("gps_altitude", "GPS altitude [m MSL]"), ("fix_age", "Fix age [s]"), ("stale", "Stale"),
        ("ping_reply", "Ping reply"), ("utc_aligned", "UTC aligned"), ("satellites", "Satellites"),
        ("battery_voltage", "Battery [V]"),
    ],
    "iris_debug": [
        ("status", "Status"), ("uptime", "Uptime"),
        ("mcu_temperature", "MCU temperature [C]"),
        ("iris_battery_voltage", "Iris battery voltage [V]"),
        ("current_sense", "Current [A; ch1, ch2, ch3]"),
    ],
}


def is_status_field(field: str) -> bool:
    return field == "status" or field.endswith("_status")


def decode_status(value: int, camera: bool = False) -> dict:
    """What the status indicators show for a 16-bit status value.

    flags: [(mask, text, active, is_error)] for byte 0 (system or camera flags);
    state / state_valid: byte 1 bits 2:0 (not for camera status);
    byte1: [(mask, text, active, is_warning)] for GPS lock and HITL.
    """
    system_flags = (value >> 8) & 0xFF
    specs = CAMERA_STATUS_FLAG_LABELS if camera else STATUS_FLAG_LABELS
    error_masks = {0x01, 0x02, 0x80} if camera else {0x10, 0x80}
    result = {"flags": [(mask, text, bool(system_flags & mask), mask in error_masks)
                        for mask, text in specs]}
    if camera:
        return result
    state = value & FLIGHT_STATE_MASK
    result["state"] = state
    result["state_valid"] = state in dict(FLIGHT_STATE_LABELS)
    result["byte1"] = [(mask, text, bool(value & mask), mask in (HITL_ACTIVE_MASK, MAINTENANCE_MASK, RESUMED_MASK, LOG_SLOTS_FULL_MASK))
                       for mask, text in STATUS_BYTE1_FLAG_LABELS]
    return result
