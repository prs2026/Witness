"""65-byte state frames, GPS lock / HITL status bits, 0x0C echo skipping."""

import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest

import converter
from flight_data import DEFAULT_LAYOUT, FlightData, load_layout, validate_layout
from flight_render import hitl_line, value_text
from main import QUICK_COMMANDS, PacketDecoder, SerialMonitor, parse_packet
import viewer
from witness_test_source import builtin_scenario, demo_packets, load_scenario

ROOT = Path(__file__).resolve().parents[1]
HEARTBEAT = bytes.fromhex("ff 6e 00 00 00 00 01 0a")


def state_frame(status=0x6E00, satellites=7, legacy=False):
    payload = (status.to_bytes(2, "big") + (1234).to_bytes(4, "big")
               + struct.pack(">4f", 0, 0, 0, 1) + struct.pack(">3f", 1, 2, 3)
               + struct.pack(">3f", 0, 0, -1)
               + (340000).to_bytes(3, "big", signed=True)
               + (-1170000).to_bytes(3, "big", signed=True)
               + struct.pack(">ff", 101.5, 99.25) + (4123).to_bytes(2, "big"))
    if not legacy:
        payload += bytes((satellites,))
    return b"\x02" + payload + b"\x0a"


def hitl_frame():
    # 0x0C host -> Witness HITL input: 45-byte payload. Fill it with bytes that
    # look like EOFs and packet IDs so a misframe would be visible.
    payload = bytes((0x0A, 0x02, 0xFF, 0x01) * 11) + b"\x0a"
    return b"\x0c" + payload + b"\x0a"


def decode(data, chunk=1):
    packets, errors, hosts = [], [], []
    decoder = PacketDecoder(packets.append, errors.append, lambda pid, count: hosts.append((pid, count)))
    for offset in range(0, len(data), chunk):
        decoder.feed(data[offset:offset + chunk])
    return packets, errors, hosts, decoder


class StateFrameTests(unittest.TestCase):
    def test_frame_lengths_and_satellites_field(self):
        self.assertEqual(len(state_frame()), 65)
        self.assertEqual(len(state_frame(legacy=True)), 64)
        self.assertEqual(len(hitl_frame()), 47)
        for decode_packet in (parse_packet, converter.parse_packet):
            with self.subTest(decoder=decode_packet.__module__):
                current = decode_packet(0x02, state_frame(satellites=7)[1:-1])
                legacy = decode_packet(0x02, state_frame(legacy=True)[1:-1])
                self.assertEqual(current["gps_satellites"], 7)
                self.assertNotIn("gps_satellites", legacy)
                self.assertAlmostEqual(current["witness_battery_voltage"], 4.123)
                self.assertAlmostEqual(legacy["witness_battery_voltage"], 4.123)
                self.assertAlmostEqual(current["latitude"], 34.0)
                self.assertAlmostEqual(current["longitude"], -117.0)

    def test_ten_satellites_puts_eof_byte_in_payload_and_split_reads_wait(self):
        frame = state_frame(status=0x6E08, satellites=10)
        self.assertEqual(frame[63], 0x0A)
        for chunk in (1, 3, 64, 65):
            with self.subTest(chunk=chunk):
                packets, errors, _hosts, decoder = decode(frame, chunk)
                self.assertEqual(errors, [])
                self.assertEqual(len(packets), 1)
                self.assertEqual(packets[0]["gps_satellites"], 10)
                self.assertFalse(decoder.buffer)
        # Exactly the first 64 bytes look like a legacy frame; never decide yet.
        packets, errors, _hosts, decoder = decode(frame[:64])
        self.assertEqual((packets, errors), ([], []))
        decoder.feed(frame[64:])
        self.assertEqual(packets[0]["gps_satellites"], 10)

    def test_legacy_64_byte_state_frames_still_decode(self):
        stream = state_frame(legacy=True) + HEARTBEAT + state_frame(legacy=True) + state_frame(satellites=3)
        packets, errors, _hosts, decoder = decode(stream)
        self.assertEqual(errors, [])
        self.assertEqual([p["packet"] for p in packets], ["state", "heartbeat", "state", "state"])
        self.assertNotIn("gps_satellites", packets[0])
        self.assertNotIn("gps_satellites", packets[2])
        self.assertEqual(packets[3]["gps_satellites"], 3)
        self.assertFalse(decoder.buffer)
        # A legacy frame at the very end waits for one more byte (it may still
        # be a 65-byte frame with 10 satellites).
        packets, errors, _hosts, decoder = decode(state_frame(legacy=True))
        self.assertEqual((packets, errors), ([], []))
        decoder.feed(HEARTBEAT)
        self.assertEqual([p["packet"] for p in packets], ["state", "heartbeat"])

    def test_echoed_hitl_input_is_framed_and_skipped(self):
        stream = HEARTBEAT + hitl_frame() + state_frame() + hitl_frame() + HEARTBEAT
        for chunk in (1, 5, len(stream)):
            with self.subTest(chunk=chunk):
                packets, errors, hosts, decoder = decode(stream, chunk)
                self.assertEqual(errors, [])
                self.assertEqual([p["packet"] for p in packets], ["heartbeat", "state", "heartbeat"])
                self.assertEqual(hosts, [(0x0C, 1), (0x0C, 2)])
                self.assertEqual(decoder.host_packet_count, 2)
                self.assertFalse(decoder.buffer)
        self.assertNotIn(0x0C, PacketDecoder.payload_lengths)  # never decoded as telemetry
        self.assertIsNone(parse_packet(0x0C, hitl_frame()[1:-1]))

    def test_converter_mixed_log_columns_and_legacy(self):
        data = (state_frame(status=0x6E1A, satellites=10) + hitl_frame()
                + state_frame(status=0x6E03, legacy=True) + HEARTBEAT)
        rows, discarded = converter.decode_log(data)
        self.assertEqual(discarded, 0)
        self.assertEqual([(r["packet_type"], r["frame_length"]) for r in rows],
                         [("state", 65), ("state", 64), ("heartbeat", 8)])
        first, legacy, heartbeat = rows
        self.assertEqual((first["gps_satellites"], first["gps_locked"], first["hitl_active"],
                          first["flight_state"]), (10, 1, 1, 2))
        self.assertNotIn("gps_satellites", legacy)
        self.assertEqual((legacy["gps_locked"], legacy["hitl_active"], legacy["flight_state"]), (0, 0, 3))
        self.assertEqual((heartbeat["gps_locked"], heartbeat["hitl_active"]), (0, 0))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "out.csv"
            converter.write_csv(rows, path)
            header = path.read_text(encoding="utf-8").splitlines()[0].split(",")
        for column in ("gps_satellites", "gps_locked", "hitl_active"):
            self.assertIn(column, header)

    def test_status_bits_decoded_alongside_flight_state(self):
        flat = converter.flatten_packet({"packet": "heartbeat", "status": 0x6E00 | 0x08 | 0x10 | 4, "uptime": 0})
        self.assertEqual((flat["flight_state"], flat["gps_locked"], flat["hitl_active"]), (4, 1, 1))
        flat = converter.flatten_packet({"packet": "camera", "fc_status": 0x6E08, "fc_uptime": 0,
                                         "status": 0x7818, "uptime": 0})
        self.assertEqual((flat["gps_locked"], flat["hitl_active"]), (1, 0))  # Witness fc_status only

    def test_monitor_readout_and_hitl_shortcuts(self):
        self.assertEqual(SerialMonitor.format_gps_satellites({"gps_satellites": 7, "status": 0x6E08}),
                         "7  lock 1")
        self.assertEqual(SerialMonitor.format_gps_satellites({"gps_satellites": 0, "status": 0x6E10}),
                         "0  lock 0")
        commands = {opcode: label for label, opcode, _confirm in QUICK_COMMANDS}
        self.assertIn("EXIT", commands[0x4858])
        self.assertIn("Enter", commands[0x4849])
        for opcode in (0x4849, 0x4858):
            frame = SerialMonitor.build_quick_command_packet(opcode)
            self.assertEqual((len(frame), frame[0], frame[-1]), (10, 0x05, 0x0A))
            self.assertEqual(int.from_bytes(frame[7:9], "big"), opcode)


class FlightViewGpsHitlTests(unittest.TestCase):
    def rows(self, model, title):
        index = next(i for i, row in enumerate(model.layout.rows) if row.title == title)
        return model.layout.rows[index], model.snapshot()[index]

    def test_default_layout_sats_lock_and_hitl(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        gps_row, gps = self.rows(model, "Witness")
        self.assertEqual(gps_row.group, "gps")
        self.assertEqual([t.label for t in gps_row.traces], ["GPS sats", "lock"])
        state_row, state = self.rows(model, "Witness flight state")
        self.assertEqual([t.format for t in state_row.traces], ["state", "hitl"])
        self.assertEqual(hitl_line(state_row, state, 0), ("HITL: --", "#444444"))
        packet = parse_packet(0x02, state_frame(status=0x6E08 | 0x10 | 1, satellites=9)[1:-1])
        model.accept_record("witness", {"source": "W", "received_mono": 5.0, "packet": packet})
        gps_row, gps = self.rows(model, "Witness")
        self.assertEqual([value_text(t, h) for t, h in zip(gps_row.traces, gps)], ["9", "1"])
        state_row, state = self.rows(model, "Witness flight state")
        self.assertEqual(value_text(state_row.traces[0], state[0]), "BOOST")
        self.assertEqual(hitl_line(state_row, state, 5.0)[0], "HITL ACTIVE - SIMULATED SENSORS")
        # Lock and HITL also follow status-only packets, e.g. heartbeats.
        model.accept_record("witness", {"source": "W", "received_mono": 6.0,
                                        "packet": {"packet": "heartbeat", "status": 0x6E01, "uptime": 1}})
        gps_row, gps = self.rows(model, "Witness")
        self.assertEqual(value_text(gps_row.traces[1], gps[1]), "0")
        state_row, state = self.rows(model, "Witness flight state")
        self.assertTrue(hitl_line(state_row, state, 6.0)[0].startswith("HITL off"))

    def test_readout_limits_and_formats(self):
        document = json.loads(DEFAULT_LAYOUT.read_text(encoding="utf-8"))
        validate_layout(document)
        gps = next(r for r in document["readouts"] if r["title"] == "Witness")
        bad = copy.deepcopy(document)
        next(r for r in bad["readouts"] if r["title"] == "Witness")["traces"].append(gps["traces"][0])
        with self.assertRaises(ValueError):
            validate_layout(bad)
        bad = copy.deepcopy(document)
        next(r for r in bad["readouts"] if r["group"] == "state")["traces"][1]["format"] = "bogus"
        with self.assertRaises(ValueError):
            validate_layout(bad)
        counts = {}
        for row in document["readouts"]:
            counts[row["group"]] = counts.get(row["group"], 0) + 1
        self.assertLessEqual(counts["gps"], 6)
        self.assertEqual(counts["state"], 1)


class ReplayFixtureTests(unittest.TestCase):
    def test_fixtures_and_demo_use_65_byte_state_with_lock(self):
        for name, frames in (("example", load_scenario(ROOT / "test_data/example_packets.json").frames),
                             ("ballistic", load_scenario(ROOT / "test_data/ballistic_10g.json").frames),
                             ("demo", builtin_scenario().frames)):
            with self.subTest(source=name):
                packets, errors, hosts, decoder = decode(b"".join(f.data for f in frames), 11)
                self.assertEqual((errors, hosts), ([], []))
                self.assertFalse(decoder.buffer)
                states = [p for p in packets if p["packet"] == "state"]
                self.assertTrue(states)
                self.assertTrue(all("gps_satellites" in p for p in states))
                satellites = {p["gps_satellites"] for p in states}
                self.assertIn(0, satellites)
                self.assertIn(10, satellites)  # 0x0A inside the payload
                locks = {(p["status"] >> 3) & 1 for p in states}
                self.assertEqual(locks, {0, 1})
                self.assertTrue(all(p["status"] & 0x10 == 0 for p in packets if "status" in p
                                    and p["packet"] not in ("camera", "iris_debug")))
                for p in states:
                    self.assertEqual(bool(p["gps_satellites"]), bool(p["status"] & 0x08))
        self.assertEqual(len(demo_packets(0)[0x02]), 65)


class ViewerStatusTests(unittest.TestCase):
    def test_status_bits_flag_columns_and_hitl_summary(self):
        rows = [
            {"packet_type": "state", "status": str(0x6E00 | 0x18 | 2), "time_seconds": "1"},
            {"packet_type": "heartbeat", "status": str(0x6E00), "time_seconds": "2"},
            {"packet_type": "iris_debug", "status": str(0x78FF), "time_seconds": "3"},
            {"packet_type": "state", "status": "0", "gps_locked": "1", "hitl_active": "0"},
        ]
        fieldnames = ["packet_type", "status", "time_seconds", "gps_locked", "hitl_active"]
        viewer.add_status_bit_traces(rows, fieldnames)
        viewer.add_status_flag_columns(rows, fieldnames)
        self.assertEqual(rows[0]["status_byte1_bit3_gps_locked"], "1")
        self.assertEqual(rows[0]["status_byte1_bit4_hitl_active"], "1")
        self.assertEqual((rows[0]["gps_locked"], rows[0]["hitl_active"]), ("1", "1"))
        self.assertEqual((rows[1]["gps_locked"], rows[1]["hitl_active"]), ("0", "0"))
        self.assertNotIn("gps_locked", rows[2])  # Iris status has no Witness bits
        self.assertEqual(rows[3]["gps_locked"], "1")  # explicit converter value kept
        self.assertIn("gps_locked", viewer.BINARY_COLUMNS)
        self.assertIn("hitl_active", viewer.BINARY_COLUMNS)
        self.assertEqual(viewer.calculate_summary(rows)["hitl"],
                         "HITL ACTIVE in 1 record(s): simulated sensor data")
        self.assertEqual(viewer.calculate_summary(rows[1:])["hitl"], "No HITL records")


if __name__ == "__main__":
    unittest.main()
