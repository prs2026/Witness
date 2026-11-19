"""Witness BIT packets 0x0B (summary) and 0x10 (detail): framing, decoding, status and the buttons.

The golden summary bytes are the firmware's (Witness/softwarev1/tests/bit_logic_test.cpp).
"""
import os
import re
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import bit_model as bm  # noqa: E402
import converter  # noqa: E402
import flight_render as fr  # noqa: E402
import witness_protocol as wp  # noqa: E402

GOLDEN_SUMMARY = bytes([0x0B, 0x1C, 0xE1, 0x01, 0x00, 0x00, 0x90, 0x02, 0x00, 0x83, 0x07, 0x01, 0x2C, 0x0A])
BIT_TYPES_H = Path(__file__).resolve().parents[2] / "Witness" / "softwarev1" / "main" / "bit_types.h"


def summary_frame(verdict=bm.VERDICT_GO, bench=False, kind=0, state=0, core=True, settled=True, active=False,
                  blocking=0, degraded=0, not_eval=0, first=0, seq=1, age=0, version=1):
    d = bytes([(version << 4) | verdict | (0x08 if bench else 0),
               kind | (state << 2) | (0x20 if core else 0) | (0x40 if settled else 0) | (0x80 if active else 0),
               blocking >> 8, blocking & 0xFF, degraded >> 8, degraded & 0xFF, not_eval >> 8, not_eval & 0xFF,
               first, seq, age >> 8, age & 0xFF])
    return bytes([0x0B]) + d + bytes([0x0A])


def detail_frame(page, seq, records, pages=3):
    """records: [(id, result, cls, bench, value)] (at most 24); the rest are unused (0xFF)."""
    body = bytearray([page, pages, seq])
    for i in range(24):
        if i < len(records):
            cid, result, cls, bench, value = records[i]
            body += bytes([cid, result | (cls << 3) | (0x20 if bench else 0), value >> 8, value & 0xFF])
        else:
            body += bytes([0xFF, 0, 0, 0])
    assert len(body) == 99
    return bytes([0x10]) + bytes(body) + bytes([0x0A])


def sensors_frame(uptime, status=0x0100):
    return bytes([0x01]) + status.to_bytes(2, "big") + uptime.to_bytes(4, "big") + bytes(17) + bytes([0x0A])


def heartbeat_frame(uptime, status=0x0100):
    return bytes([0xFF]) + status.to_bytes(2, "big") + uptime.to_bytes(4, "big") + bytes([0x0A])


def decode(stream, chunk=None):
    got = []
    decoder = wp.PacketDecoder(got.append)
    if chunk is None:
        decoder.feed(stream)
    else:
        for i in range(0, len(stream), chunk):
            decoder.feed(stream[i:i + chunk])
    return got, decoder


class Decoding(unittest.TestCase):
    def test_golden_summary(self):
        p = wp.parse_packet(0x0B, GOLDEN_SUMMARY[1:-1])
        self.assertEqual(p["packet"], "bit_summary")
        self.assertEqual((p["version"], p["verdict"], p["bench"], p["run_kind"], p["flight_state"]),
                         (1, bm.VERDICT_NO_GO, True, 1, 0))
        self.assertTrue(p["core_done"] and p["settled"] and p["active_ran"])
        self.assertEqual((p["blocking_mask"], p["degraded_mask"], p["not_evaluated_mask"]), (0x0100, 0x0090, 0x0200))
        self.assertEqual((p["first_fail_id"], p["sequence"], p["age_s"]), (0x83, 7, 300))
        self.assertEqual(bm.group_names(p["blocking_mask"]), ["radio"])
        self.assertEqual(bm.group_names(p["degraded_mask"]), ["attitude", "GPS"])
        self.assertEqual(bm.check_label(p["first_fail_id"]), "8.3 radio power")
        self.assertEqual(summary_frame(verdict=4, bench=True, kind=1, core=True, settled=True, active=True,
                                       blocking=0x0100, degraded=0x0090, not_eval=0x0200, first=0x83, seq=7,
                                       age=300), GOLDEN_SUMMARY)

    def test_detail_records(self):
        frame = detail_frame(1, 7, [(0x83, bm.RESULT_FAIL, 2, True, 0xFFEF), (0x81, bm.RESULT_PASS, 2, False, 0)])
        p = wp.parse_packet(0x10, frame[1:-1])
        self.assertEqual((p["packet"], p["page"], p["page_count"], p["sequence"]), ("bit_detail", 1, 3, 7))
        self.assertEqual(len(p["records"]), 2)
        self.assertEqual(p["records"][0], {"id": 0x83, "result": 1, "class": 2, "bench": True, "value": 0xFFEF})

    def test_wrong_lengths_are_not_decoded(self):
        self.assertIsNone(bm.decode_summary(bytes(11)))
        self.assertIsNone(bm.decode_detail(bytes(98)))

    def test_check_table_matches_the_firmware_header(self):
        if not BIT_TYPES_H.exists():
            self.skipTest("firmware header not in this checkout")
        text = BIT_TYPES_H.read_text(encoding="utf-8")
        rows = re.findall(r'make_id\((\d+), (\d+)\), Class::(\w+), Stage::\w+, \w+, (?:true|false), "([^"]+)"', text)
        firmware = [((int(g) << 4) | int(c), cls, name) for g, c, cls, name in rows]
        self.assertEqual(list(bm.CHECKS), firmware)
        self.assertEqual(bm.DETAIL_PAGE_COUNT, 3)


class Framing(unittest.TestCase):
    """The BIT frames are framed by length: their data bytes (any value, including 0x0A and packet IDs)
    must never be scanned for other frames. Before 0x0B/0x10 were known the decoder skipped the ID byte,
    then found a packet ID inside the data whose EOF lined up (0x05 + 8 bytes + the summary's own EOF)
    and reported a bogus command / sensors packet."""

    def names(self, stream, chunk=None):
        got, decoder = decode(stream, chunk)
        return [p["packet"] for p in got], got, decoder

    def test_blocking_mask_byte_5_is_not_a_command_packet(self):
        # blocking groups 0 and 2 (mask low byte 0x05): 0x05 + 8 bytes ends exactly on the summary EOF.
        frame = summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0005, first=0x03, seq=3)
        stream = sensors_frame(1000) + frame + sensors_frame(2000)
        names, got, decoder = self.names(stream)
        self.assertEqual(names, ["sensors", "bit_summary", "sensors"])
        self.assertEqual([p["uptime"] for p in got if p["packet"] == "sensors"], [1000, 2000])
        self.assertEqual(decoder.stats()["garbage_bytes_skipped"], 0)

    def test_summary_data_that_looks_like_a_sensors_frame(self):
        # run kind 1 / Pad Idle with no flags = d1 0x01 (a sensors ID).
        frame = summary_frame(core=False, settled=False, kind=1, seq=9)
        self.assertEqual(frame[2], 0x01)
        stream = sensors_frame(5) + frame + heartbeat_frame(77) + sensors_frame(6)
        names, got, _ = self.names(stream)
        self.assertEqual(names, ["sensors", "bit_summary", "heartbeat", "sensors"])
        self.assertEqual(got[2]["uptime"], 77)

    def test_detail_records_hold_packet_ids_and_eof_bytes(self):
        records = [(0x01, 1, 1, False, 0x0A0A), (0x02, 0, 0, False, 0x0A00), (0x03, 0, 2, False, 0x000A),
                   (0x05, 1, 2, False, 0x0A0A)] + [(0x11, 0, 0, False, 0x0A0A)] * 20
        frame = detail_frame(0, 1, records)
        stream = heartbeat_frame(1) + frame + heartbeat_frame(2) + frame + heartbeat_frame(3)
        names, got, decoder = self.names(stream)
        self.assertEqual(names, ["heartbeat", "bit_detail", "heartbeat", "bit_detail", "heartbeat"])
        self.assertEqual([p["uptime"] for p in got if p["packet"] == "heartbeat"], [1, 2, 3])
        self.assertEqual(decoder.stats()["garbage_bytes_skipped"], 0)

    def test_every_summary_byte_value_keeps_the_neighbours(self):
        for position in range(12):
            for value in range(256):
                payload = bytearray(summary_frame()[1:-1])
                payload[position] = value
                frame = bytes([0x0B]) + bytes(payload) + bytes([0x0A])
                got, _ = decode(heartbeat_frame(10) + frame + heartbeat_frame(20))
                self.assertEqual([p["packet"] for p in got], ["heartbeat", "bit_summary", "heartbeat"],
                                 (position, value))

    def test_split_feeds_give_the_same_result(self):
        stream = (sensors_frame(1) + summary_frame(blocking=0x0005) + detail_frame(2, 1, [(0xC4, 0, 0, False, 0)])
                  + sensors_frame(2))
        whole, _, _ = self.names(stream)
        for chunk in (1, 2, 3, 7, 13):
            self.assertEqual(self.names(stream, chunk)[0], whole, chunk)
        self.assertEqual(whole, ["sensors", "bit_summary", "bit_detail", "sensors"])

    def test_a_damaged_summary_loses_only_itself(self):
        frame = bytearray(summary_frame(blocking=0x0100))
        frame[13] = 0x55                          # EOF gone
        names, _, decoder = self.names(heartbeat_frame(1) + bytes(frame) + heartbeat_frame(2) + sensors_frame(3) * 2)
        self.assertEqual(names.count("heartbeat"), 2)
        self.assertNotIn("bit_summary", names)
        self.assertEqual(decoder.stats()["bad_frames"].get(0x0B), 1)

    def test_converter_skips_bit_frames_whole(self):
        frame = summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0005, first=0x03)
        data = sensors_frame(1000) + frame + detail_frame(0, 1, [(0x01, 0, 0, False, 0)]) + sensors_frame(2000)
        rows, discarded = converter.decode_log(data)
        self.assertEqual([r["packet_type"] for r in rows], ["sensors", "sensors"])
        self.assertEqual(discarded, 0)

    def test_converter_without_the_bit_table_finds_a_bogus_command(self):
        # The same bytes without the BIT length table (what the wire scanner did) give a bogus command.
        frame = summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0005, first=0x03)
        with patch.object(converter, "SKIPPED_PAYLOAD_LENGTHS", {}):
            rows, _ = converter.decode_log(sensors_frame(1000) + frame + sensors_frame(2000))
        self.assertIn("command", [r["packet_type"] for r in rows])

    def test_nand_log_source_7_is_skipped_by_length(self):
        def record(source, frame, t=0):
            return t.to_bytes(4, "big") + bytes([source, len(frame)]) + frame
        data = (record(1, sensors_frame(1000), 1) + record(7, summary_frame(blocking=0x0005), 2)
                + record(7, detail_frame(0, 1, []), 3) + record(1, sensors_frame(2000), 4))
        rows, _ = converter.decode_packet_log(data)
        self.assertEqual([r["packet_type"] for r in rows], ["sensors", "sensors"])


class Status(unittest.TestCase):
    def feed(self, model, raw, now=100.0, **kw):
        return model.feed(wp.parse_packet(raw[0], raw[1:-1]), now, **kw)

    def test_nothing_before_a_summary(self):
        self.assertIsNone(bm.BitModel().status(0.0))

    def test_go_degraded_nogo_and_bench(self):
        model = bm.BitModel()
        self.feed(model, summary_frame(verdict=bm.VERDICT_GO))
        self.assertEqual(model.status(100.5)[:2], ("BIT GO", "ok"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_GO_WARN, degraded=0x0090, first=0x71, seq=2))
        self.assertEqual(model.status(100.5)[:2], ("BIT GO with warnings: 7.1 gps receiver (+1 more)", "warn"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0100, first=0x83, seq=3))
        self.assertEqual(model.status(100.5)[:2], ("BIT NO-GO: 8.3 radio power", "bad"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_NO_GO, bench=True, blocking=0x0100, first=0x83, seq=4))
        text, kind, tip = model.status(100.5)
        self.assertEqual((text, kind), ("BIT NO-GO (bench): 8.3 radio power", "bench"))
        self.assertIn("Bench condition", tip)

    def test_running_not_run_and_core_pending(self):
        model = bm.BitModel()
        self.feed(model, summary_frame(verdict=bm.VERDICT_RUNNING, core=False, settled=False))
        self.assertEqual(model.status(100.0)[:2], ("BIT running", "muted"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_NOT_RUN, core=False, settled=False))
        self.assertEqual(model.status(100.0)[:2], ("BIT not run", "muted"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_GO, core=False))
        self.assertEqual(model.status(100.0)[:2], ("BIT GO (core pending)", "warn"))

    def test_stale_after_five_minutes_and_for_snapshots(self):
        model = bm.BitModel()
        self.feed(model, summary_frame(verdict=bm.VERDICT_GO, age=10))
        self.assertEqual(model.status(100.0 + 200)[1], "ok")
        self.assertEqual(model.status(100.0 + 295)[:2], ("BIT GO - STALE", "warn"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_GO), snapshot=True)
        self.assertTrue(model.status(100.0)[0].endswith("STALE"))
        self.feed(model, summary_frame(verdict=bm.VERDICT_NO_GO, blocking=1, first=3, age=400))
        self.assertEqual(model.status(100.0)[1], "bad")       # a failure never greys out

    def test_other_version_is_flagged_not_trusted(self):
        model = bm.BitModel()
        self.feed(model, summary_frame(version=2))
        text, kind, _ = model.status(100.0)
        self.assertEqual(kind, "warn")
        self.assertIn("version 2", text)

    def test_failing_checks_come_from_detail_pages_of_the_same_run(self):
        model = bm.BitModel()
        self.feed(model, summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0100, degraded=0x0080, first=0x83, seq=5))
        self.assertEqual(model.failing(), [])
        self.assertEqual(model.missing_pages(), [0, 1, 2])
        self.feed(model, detail_frame(1, 4, [(0x83, 1, 2, False, 7)]))           # an older run
        self.assertEqual(model.failing(), [])
        self.feed(model, detail_frame(2, 5, [(0x83, bm.RESULT_FAIL, 2, False, 0xFFEF),
                                             (0x71, bm.RESULT_FAIL, 1, False, 3),
                                             (0x81, bm.RESULT_PASS, 2, False, 0)]))
        self.assertEqual(model.missing_pages(), [0, 1])
        fails = model.failing()
        self.assertEqual([f[1] for f in fails], ["8.3 radio power", "7.1 gps receiver"])      # blocking first
        self.assertEqual(fails[0][2:], ("blocking", "FAIL", 0xFFEF))
        tip = model.status(100.0)[2]
        self.assertIn("8.3 radio power", tip)
        self.assertNotIn("Check names not received", tip)
        self.feed(model, detail_frame(0, 5, []))
        self.feed(model, detail_frame(1, 5, []))
        self.assertEqual(model.missing_pages(), [])

    def test_ack_verdict(self):
        self.assertEqual(bm.ack_verdict(0x0403), ("NO-GO", 3))
        self.assertEqual(bm.ack_verdict(0x0200), ("GO", 0))


class Builders(unittest.TestCase):
    def test_run_frame_layout(self):
        self.assertEqual(wp.build_bit_run_packet(0x01020304),
                         bytes([0x05, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x54, 0x42, 0x0A]))
        self.assertEqual(len(wp.build_bit_run_packet()), 10)

    def test_report_frame_layout(self):
        self.assertEqual(wp.build_bit_report_packet(), bytes([0x05, 0xFF, 0, 0, 0, 0, 0, 0x54, 0x52, 0x0A]))
        self.assertEqual(wp.build_bit_report_packet(2)[1], 2)
        with self.assertRaises(ValueError):
            wp.build_bit_report_packet(256)

    def test_no_bit_command_is_in_the_quick_list(self):
        self.assertFalse([c for c in wp.QUICK_COMMANDS if c[1] in (0x5442, 0x5452, 0x5443, 0x4246)])


class Strip(unittest.TestCase):
    def test_chip_in_the_status_strip(self):
        from telemetry_model import witness_status_fields
        model = bm.BitModel()
        status = {"vehicle": {"status": 0x7E00, "received": 100.0, "snapshot": False, "position": None,
                              "bits": witness_status_fields({"packet": "state", "status": 0x7E00})},
                  "bit": model}
        self.assertFalse([t for t, _ in fr.status_strip(status, 100.5) if t.startswith("BIT")])
        raw = summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0100, first=0x83)
        model.feed(wp.parse_packet(0x0B, raw[1:-1]), 100.0)
        chips = [c for c in fr.status_strip(status, 100.5) if c[0].startswith("BIT")]
        self.assertEqual(chips, [("BIT NO-GO: 8.3 radio power", "bad")])
        self.assertIn("bench", fr.STRIP_COLORS)

    def test_flight_data_keeps_the_bit_model_from_the_feed(self):
        from flight_data import DEFAULT_LAYOUT, FlightData, load_layout
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        raw = summary_frame(verdict=bm.VERDICT_GO)
        data.accept_record("witness", {"source": "COM1", "received_at": time.time(),
                                       "packet": wp.parse_packet(0x0B, raw[1:-1])})
        self.assertEqual(data.status_snapshot()["bit"].status(time.monotonic())[:2], ("BIT GO", "ok"))


class Monitor(unittest.TestCase):
    PAD, BOOST, LANDED = 0, 1, 4

    def setUp(self):
        from horizon_qt_support import make_monitor
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        patcher = patch("monitor_core.LOG_DIR", Path(self.directory.name))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.monitor = make_monitor(self.directory.name)
        self.addCleanup(self.monitor.close)
        self.core = self.monitor.core
        self.core.commands_enabled.set(True)
        self.sent = []
        self.core.transmit_command = lambda target, command, label="": self.sent.append((command, label)) or True
        self.asked = []
        self.answer = True
        self.core.confirm = lambda title, text: self.asked.append(text) or self.answer

    def push(self, packet):
        self.monitor.events.put(("packet", ("COM_TEST", packet, time.time())))
        self.monitor.process_events()

    def state(self, state, packet="sensors"):
        self.push({"packet": packet, "status": 0x0100 | state, "uptime": 1,
                   "filtered_accel": [0.0, 0.0, 1.0], "gyro": [0.0, 0.0, 0.0]})

    def bit_packet(self, raw):
        self.push(wp.parse_packet(raw[0], raw[1:-1]))

    def test_run_bit_enabled_only_in_pad_idle_with_a_link(self):
        button = self.monitor.run_bit_button
        self.assertEqual(button.text(), "Run BIT")
        self.assertFalse(button.isEnabled())                       # no telemetry yet
        for state, enabled in ((self.PAD, True), (self.BOOST, False), (2, False), (3, False),
                               (self.LANDED, False), (self.PAD, True)):
            self.state(state)
            self.assertEqual(button.isEnabled(), enabled, f"state {state}")
        self.core.commands_enabled.set(False)
        self.assertFalse(button.isEnabled())

    def test_run_bit_asks_first_and_sends_only_0x5442(self):
        self.state(self.PAD)
        self.answer = False
        self.assertFalse(self.core.send_bit_run())
        self.assertEqual(self.sent, [])
        self.assertIn("built-in self test", self.asked[-1])
        self.answer = True
        self.assertTrue(self.core.send_bit_run())
        command, label = self.sent[-1]
        self.assertEqual((command[0], command[7:9], command[9], label), (0x05, bytes([0x54, 0x42]), 0x0A, "Run BIT"))
        self.assertEqual(command[1:3], bytes(2))                   # full run, no options
        self.assertEqual(len(self.sent), 1)

    def test_run_bit_is_refused_by_the_monitor_outside_pad_idle(self):
        for state in (self.BOOST, self.LANDED):
            self.state(state)
            self.assertFalse(self.core.send_bit_run())
        self.assertEqual((self.sent, self.asked), ([], []))

    def test_bit_packets_never_cause_a_command(self):
        self.state(self.PAD)
        for raw in (summary_frame(verdict=bm.VERDICT_NOT_RUN), summary_frame(verdict=bm.VERDICT_NO_GO, blocking=1),
                    summary_frame(verdict=bm.VERDICT_GO), detail_frame(0, 1, [])):
            self.bit_packet(raw)
        self.core.update_link_health()
        self.assertEqual(self.sent, [])

    def test_status_text_follows_the_summary_and_names_failures(self):
        self.assertEqual(self.core.bit_text.get(), "BIT: no data yet")
        self.bit_packet(summary_frame(verdict=bm.VERDICT_NO_GO, blocking=0x0100, first=0x83, seq=4))
        self.assertEqual((self.core.bit_text.get(), self.core.bit_kind.get()), ("BIT NO-GO: 8.3 radio power", "bad"))
        # all three detail pages arrive in one tick: every one is kept, not only the newest
        for page in (0, 1, 2):
            recs = [(0x83, bm.RESULT_FAIL, 2, False, 1)] if page == 1 else []
            raw = detail_frame(page, 4, recs)
            self.monitor.events.put(("packet", ("COM_TEST", wp.parse_packet(0x10, raw[1:-1]), time.time())))
        self.monitor.process_events()
        self.assertEqual(self.core.bit.missing_pages(), [])
        self.assertIn("8.3 radio power: blocking FAIL", self.core.bit_tooltip.get())
        self.assertEqual(self.monitor.bit_label.text(), "BIT NO-GO: 8.3 radio power")

    def test_detail_request_asks_for_missing_pages_one_at_a_time(self):
        self.core.command_target.set("COM_TEST")
        self.core._target_writer = lambda name: (lambda data: None, None)
        self.state(self.PAD)
        self.bit_packet(summary_frame(verdict=bm.VERDICT_NO_GO, blocking=1, first=3, seq=6))
        feed = []

        def answer(target, command, label=""):
            self.sent.append((command, label))
            raw = detail_frame(command[1], 6, [])          # Witness answers each request with that page
            self.core.bit.feed(wp.parse_packet(0x10, raw[1:-1]), time.monotonic())
            feed.append(command[1])
            return True
        self.core.transmit_command = answer
        self.assertTrue(self.core.request_bit_report())
        deadline = time.time() + 5
        while len(self.sent) < 3 and time.time() < deadline:
            time.sleep(0.02)
        self.assertEqual([c[1] for c, _ in self.sent], [0, 1, 2])
        self.assertTrue(all(c[7:9] == bytes([0x54, 0x52]) for c, _ in self.sent))

    def test_detail_request_is_refused_in_boost_and_the_button_follows(self):
        self.state(self.BOOST)
        self.assertFalse(self.monitor.bit_detail_button.isEnabled())
        self.assertFalse(self.core.request_bit_report())
        self.assertEqual(self.sent, [])
        self.state(self.PAD)
        self.assertTrue(self.monitor.bit_detail_button.isEnabled())

    def test_run_ack_is_not_resent_and_is_reported(self):
        # The real transmit path: a BIT run is written once and never resent while waiting for the ack.
        from monitor_core import MonitorCore
        del self.core.transmit_command                              # use the class method again
        writes, lines = [], []
        self.core._target_writer = lambda name: (writes.append, None)
        self.core.console_write = lines.append
        frame = wp.build_bit_run_packet(0x0A0B0C0D)
        with patch("monitor_core.BIT_RUN_ACK_WAIT_S", 3.0):
            self.assertTrue(self.core.transmit_command("COM_TEST", frame, "Run BIT"))
            time.sleep(0.2)
            self.core.command_link.feed({"packet": "command_ack", "command_value": 0x5442,
                                         "command_data": frame[1:7].hex(" "), "result": 0,
                                         "result_name": "ok", "detail": 0x0403})
            deadline = time.time() + 3
            while not any("blocking group" in str(line) for line in lines) and time.time() < deadline:
                time.sleep(0.02)
        self.assertEqual(writes, [frame])
        self.assertTrue(any("NO-GO, 3 blocking group(s) failed" in str(line) for line in lines), lines)


if __name__ == "__main__":
    unittest.main()
