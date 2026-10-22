"""Offline test-source validation, timing, decoding and GUI-to-Horizon checks."""

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from main import PacketDecoder, SerialMonitor, command_line, parse_packet
from witness_test_source import (
    TestFrame, TestScenario, builtin_scenario, demo_packets, load_scenario,
    play_scenario, validate_speed,
)

ROOT = Path(__file__).resolve().parents[1]


class SourceTests(unittest.TestCase):
    def test_entire_demo_decodes_all_types_and_phases(self):
        packets, errors = [], []
        decoder = PacketDecoder(packets.append, errors.append)
        scenario = builtin_scenario()
        for frame in scenario.frames:
            # Deliberately arbitrary chunk boundaries, as on serial/TCP.
            for offset in range(0, len(frame.data), 3):
                decoder.feed(frame.data[offset:offset + 3])
        self.assertFalse(errors, repr(errors[:5]))
        self.assertFalse(decoder.buffer)
        self.assertEqual(len(packets), len(scenario.frames))
        self.assertEqual(len(packets), 960)
        self.assertEqual({p["packet"] for p in packets}, {
            "sensors", "state", "camera", "heartbeat", "ground_station_status", "witness_debug", "iris_debug"})
        states = [p for p in packets if p["packet"] == "state"]
        self.assertEqual({p["status"] & 7 for p in states}, {0, 1, 2, 3, 4})
        self.assertEqual(max(p["barometric_altitude"] for p in states), 900)
        self.assertEqual(states[0]["witness_battery_voltage"], 4.2)
        self.assertLess(states[-1]["witness_battery_voltage"], 4.2)
        self.assertTrue(all(p["longitude"] < 0 for p in states))

    def test_demo_wire_lengths_and_engineering_units(self):
        expected = {0x01: 23, 0x02: 64, 0x03: 19, 0xFF: 8, 0xF0: 13, 0xE0: 103, 0xE1: 14}
        frames = demo_packets(7000)
        self.assertEqual({key: len(frame) for key, frame in frames.items()}, expected)
        packet = parse_packet(0xE0, frames[0xE0][1:-1])
        self.assertEqual(packet["uptime"], 7000)
        self.assertEqual(packet["status"] & 7, 1)
        self.assertAlmostEqual(packet["filtered_accel"][2], 3.45, places=4)
        self.assertAlmostEqual(packet["low_accel"][2], 3.45, places=3)
        self.assertEqual(packet["barometric_altitude"], 48.0)
        self.assertEqual(packet["witness_battery_voltage"], 4.186)
        self.assertEqual(packet["lsm6_temperature"], 27.0)

    def test_example_source_and_invalid_sources(self):
        scenario = load_scenario(ROOT / "test_data/example_packets.json")
        self.assertEqual(len(scenario.frames), 35)
        self.assertEqual(scenario.duration_ms, 6000)
        valid = {"version": 1, "duration_ms": 2000,
                 "frames": [{"at_ms": 0, "hex": "ff"}, {"at_ms": 1000, "hex": "00 0a"}]}
        invalid = [[], {**valid, "version": True}, {**valid, "version": 2},
                   {**valid, "duration_ms": 0}, {**valid, "frames": []},
                   {**valid, "frames": [{"at_ms": 2000, "hex": "ff"}]},
                   {**valid, "frames": [{"at_ms": 1.2, "hex": "ff"}]},
                   {**valid, "frames": [{"at_ms": 0, "hex": "zz"}]},
                   {**valid, "frames": [{"at_ms": 0, "hex": ""}]},
                   {**valid, "frames": list(reversed(valid["frames"]))}]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source.json"
            for document in invalid:
                path.write_text(json.dumps(document))
                with self.subTest(document=document), self.assertRaises(ValueError):
                    load_scenario(path)
            path.write_text(json.dumps(valid))
            self.assertEqual(load_scenario(path).frames[0].data, b"\xff")

    def test_schedule_speed_loop_and_cancellation(self):
        class ClockStop:
            now = 0
            stopped = False
            def wait(self, delay):
                self.now += delay
                return self.stopped
            def is_set(self):
                return self.stopped
        clock = ClockStop()
        scenario = TestScenario("timing", 2000, (TestFrame(0, b"one"), TestFrame(1000, b"two")))
        received, cycles = [], []
        play_scenario(scenario, lambda data: received.append((clock.now, data)), clock,
                      speed=2, loop=False, on_cycle=lambda: cycles.append(clock.now), clock=lambda: clock.now)
        self.assertEqual(received, [(0, b"one"), (0.5, b"two")])
        self.assertEqual(clock.now, 1.0)
        self.assertEqual(cycles, [0])
        clock.now = 0
        received.clear()
        cycles.clear()
        def receive(data):
            received.append(data)
            if len(received) == 3:
                clock.stopped = True
        play_scenario(scenario, receive, clock, loop=True, on_cycle=lambda: cycles.append(clock.now),
                      clock=lambda: clock.now)
        self.assertEqual(received, [b"one", b"two", b"one"])
        self.assertEqual(cycles, [0, 2])

    def test_stop_interrupts_long_gap(self):
        stop = threading.Event()
        delivered = []
        scenario = TestScenario("long gap", 60000, (TestFrame(59000, b"no"),))
        thread = threading.Thread(target=play_scenario, args=(scenario, delivered.append, stop), daemon=True)
        thread.start()
        stop.set()
        thread.join(timeout=0.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(delivered, [])

    def test_cli_and_speed_validation(self):
        args = command_line(["--test-file", "example.json", "--test-speed", "2", "--test-once"])
        self.assertEqual(args.test_file, "example.json")
        self.assertTrue(args.test_once)
        self.assertEqual(args.test_speed, 2)
        self.assertTrue(command_line(["--test"]).test)
        for speed in (0, -1, 11, float("nan"), float("inf")):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                validate_speed(speed)


class TestModeGuiTests(unittest.TestCase):
    def test_replay_flows_to_horizon_and_never_opens_hardware(self):
        import tkinter as tk
        from horizon import HorizonGroundStation
        with tempfile.TemporaryDirectory() as directory, patch("main.LOG_DIR", Path(directory)), \
                patch("main.DEFAULT_WITNESS_FEED_PORT", 0), patch("main.messagebox.showerror") as error_box:
            try:
                app = SerialMonitor(test_source=str(ROOT / "test_data/example_packets.json"),
                                    test_speed=10, test_loop=False)
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            app.withdraw()
            unified = HorizonGroundStation(log_dir=Path(directory), auto_connect=False)
            unified.withdraw()
            def pump_until(predicate, timeout=5):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    app.update()
                    unified.update()
                    if predicate():
                        return
                    time.sleep(0.01)
                self.fail("GUI test timed out")
            try:
                unified.witness.port.set(str(app.publisher.port))
                unified.witness.connect()
                pump_until(lambda: unified.witness.receiver.status.startswith("Connected"))
                # Existing real inputs must be explicitly disconnected first.
                live_port = Mock()
                app.serials["COM_REAL"] = live_port
                self.assertFalse(app.start_test())
                self.assertTrue(error_box.called)
                app.serials.clear()
                error_box.reset_mock()
                with patch("serial.Serial", side_effect=AssertionError("Opened real serial port")), \
                        patch("main.socket.create_connection", side_effect=AssertionError("Opened raw TCP")):
                    self.assertTrue(app.start_test())
                    self.assertFalse(app.transmit_command("COM_REAL", b"\x05\x68\x69"))
                    app.connect()
                    self.assertTrue(error_box.called)
                    self.assertEqual(app.command_port_combo["values"], "")
                    pump_until(lambda: not app.test_active)
                live_port.write.assert_not_called()
                pump_until(lambda: unified.witness.receiver.messages >= 35)
                source = "TEST_example_packets"
                view = unified.witness.view
                row_key = (source, "state", "witness_battery_voltage")
                pump_until(lambda: row_key in view._rows and view.tree.set(view._rows[row_key], "value") == "4.09")
                self.assertEqual(app.values["packet.state.witness_battery_voltage"].get(), "4.09")
                flight = unified.flight.data.snapshot()
                self.assertEqual(flight[0][0].source, source)
                self.assertEqual(len(flight[0][0].points), 10)  # State + equivalent debug samples.
                self.assertEqual(len(flight[1][0].points), 10)  # Sensors + equivalent debug samples.
                self.assertEqual(flight[0][0].latest, 0)  # Landed in the final replay frame.
                unified.flight.redraw()
                self.assertIn("TEST DATA", unified.flight.banner.cget("text"))
                self.assertIn("simulated", app.test_status.get())
                self.assertEqual(app.serials, {})
                self.assertEqual(app.tcp_sockets, {})
                expected = b"".join(frame.data for frame in load_scenario(ROOT / "test_data/example_packets.json").frames)
                raw_logs = list(Path(directory).glob("test_*.bin"))
                self.assertEqual(len(raw_logs), 1)
                self.assertEqual(raw_logs[0].read_bytes(), expected)
                debug = next(Path(directory).glob("test_debug_*.log")).read_text()
                self.assertIn("TEST_START", debug)
                self.assertIn("TEST_END", debug)
                self.assertIn("PACKET[TEST_example_packets]", debug)
                self.assertNotIn("BAD_PACKET", debug)
                # Repeated start/stop must not leave a worker or stale completion event.
                app.test_file.set("")
                self.assertTrue(app.start_test())
                worker = app._test_thread
                app.stop_test()
                self.assertFalse(worker.is_alive())
                self.assertFalse(app.test_active)
                self.assertEqual(app.test_button.cget("text"), "Start test")
            finally:
                app.stop_test()
                for callback in app.tk.call("after", "info"):
                    app.after_cancel(callback)
                app.close()
                unified.close()


if __name__ == "__main__":
    unittest.main()
