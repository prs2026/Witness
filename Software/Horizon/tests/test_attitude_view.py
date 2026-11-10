"""Attitude tab: live source selection, CSV loading/replay and a GUI smoke test."""

import csv
from pathlib import Path
import struct
import tempfile
import time
import unittest

import attitude_math as am
from attitude_data import E0_FRESH_S, LiveAttitude, load_attitude_csv
from converter import decode_log, parse_packet, write_csv

PAD = am.Q_UPRIGHT
TILT_EAST_10 = am.from_tilt_azimuth_spin(10.0, 90.0, 0.0)


def status_word(state=0, gps=0, hitl=0):
    return state | (gps << 3) | (hitl << 4)


def state_frame(q, *, status=0, uptime=1000, sats=7):
    payload = (struct.pack(">HI", status, uptime) + struct.pack(">4f", *q) + bytes(24)
               + bytes(6) + struct.pack(">ff", 0, 0) + struct.pack(">H", 3900) + bytes([sats]))
    assert len(payload) == 63
    return bytes([0x02]) + payload + b"\n"


def debug_frame(q, *, status=0, uptime=1000):
    payload = bytearray(101)
    payload[0:6] = struct.pack(">HI", status, uptime)
    payload[18:34] = struct.pack(">4f", *q)
    return bytes([0xE0]) + bytes(payload) + b"\n"


def record(frame, *, received=None, source="COM7", replay=False):
    packet = parse_packet(frame[0], frame[1:-1])
    received = time.monotonic() if received is None else received
    return {"source": source, "received_at": time.time(), "received_mono": received,
            "replay": replay, "packet": packet}


class LiveSelection(unittest.TestCase):
    def test_prefers_fresh_e0_then_falls_back_to_state(self):
        live = LiveAttitude()
        t = 1000.0
        self.assertTrue(live.accept_record(record(state_frame(PAD, status=status_word(1, 1, 1)), received=t)))
        self.assertEqual(live.current(t).source, "0x02")
        self.assertEqual(live.status, (1, 1, 1))
        live.accept_record(record(debug_frame(TILT_EAST_10), received=t + 0.1))
        current = live.current(t + 0.2)
        self.assertEqual(current.source, "0xE0")
        self.assertAlmostEqual(am.tilt_deg(current.q), 10.0, places=4)
        # A later 0x02 does not displace fresh 0xE0, and is not added to history.
        live.accept_record(record(state_frame(PAD), received=t + 0.5))
        self.assertEqual(live.current(t + 0.6).source, "0xE0")
        self.assertEqual([s.source for s in live.history_list()], ["0x02", "0xE0"])
        # Once 0xE0 is stale the newest sample (0x02) is used.
        self.assertEqual(live.current(t + 0.1 + E0_FRESH_S + 0.1).source, "0x02")

    def test_zero_quaternion_and_non_witness_packets(self):
        live = LiveAttitude()
        live.accept_record(record(state_frame((0, 0, 0, 0)), received=5.0))
        sample = live.current(5.0)
        self.assertIsNone(sample.q)
        self.assertEqual(sample.reason, "not available")
        self.assertFalse(live.accept_record({"source": "x", "packet": {"packet": "camera", "status": 3}}))
        self.assertFalse(live.accept_record({"bad": 1}))
        self.assertFalse(live.accept_record(record(bytes([0xFF]) + struct.pack(">HI", status_word(2), 9) + b"\n")))
        self.assertEqual(live.status[0], 2)   # heartbeat still updates flight state


class CsvLoading(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.dir = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def test_converter_csv_prefers_e0_rows(self):
        data = b"".join([
            state_frame(PAD, uptime=0, status=status_word(0, 1, 1)),
            debug_frame(PAD, uptime=0, status=status_word(0, 1, 1)),
            debug_frame(TILT_EAST_10, uptime=100, status=status_word(1, 1, 1)),
            bytes([0xFF]) + struct.pack(">HI", 0, 150) + b"\n",
            state_frame(TILT_EAST_10, uptime=1000),
            debug_frame((0, 0, 0, 0), uptime=200),
        ])
        rows, discarded = decode_log(data)
        self.assertEqual(discarded, 0)
        path = self.dir / "flight.csv"
        write_csv(rows, path)
        track = load_attitude_csv(path)
        self.assertEqual(len(track), 3)
        self.assertEqual({s.source for s in track.samples}, {"0xE0"})
        self.assertEqual(track.columns["estimate"], ("orientation_x", "orientation_y", "orientation_z", "orientation_w"))
        self.assertEqual(track.columns["time"], "time_seconds")
        self.assertFalse(track.has_truth)
        self.assertEqual(track.samples[1].flight_state, 1)
        self.assertEqual(track.samples[1].hitl, 1)
        self.assertIsNone(track.samples[2].q)
        mid = track.sample_at(0.05)
        self.assertTrue(mid.interpolated)
        self.assertAlmostEqual(am.tilt_deg(mid.q), 5.0, places=3)
        self.assertTrue(track.warnings)

    def write_hitl(self, name, header, rows):
        path = self.dir / name
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)
        return path

    def test_hitl_csv_with_truth_and_heading_offset(self):
        header = ["time_s", "packet_id", "witness_qx", "witness_qy", "witness_qz", "witness_qw",
                  "truth_qx", "truth_qy", "truth_qz", "truth_qw", "flight_state", "hitl_active"]
        rows = []
        for i in range(11):
            truth = am.from_tilt_azimuth_spin(i * 1.0, 45.0, 0.0)
            estimate = am.apply_heading(truth, -30.0)
            rows.append([i * 0.1, "0xE0", *estimate, *truth, 1, 1])
        rows[0][2:6] = [0, 0, 0, 0]   # estimate not yet available
        track = load_attitude_csv(self.write_hitl("attitude.csv", header, rows))
        self.assertTrue(track.has_truth)
        self.assertIsNone(track.samples[0].q)
        offset, at = track.first_heading_offset()
        self.assertAlmostEqual(offset, 30.0, places=4)
        self.assertAlmostEqual(at, 0.1)
        sample = track.sample_at(0.55)
        tilt_err, att_err = am.attitude_errors(sample.q, sample.truth, offset)
        self.assertAlmostEqual(tilt_err, 0.0, places=4)
        self.assertAlmostEqual(att_err, 0.0, places=3)

    def test_generic_columns_ms_time_and_bad_files(self):
        path = self.write_hitl("plain.csv", ["uptime_ms", "qx", "qy", "qz", "qw"],
                               [[0, *PAD], [500, *TILT_EAST_10], [100, *PAD]])
        track = load_attitude_csv(path)
        self.assertEqual(track.samples[1].time, 0.5)
        self.assertGreater(track.samples[2].time, 0.5)   # backwards time shifted
        self.assertEqual(track.samples[0].source, "file")
        with self.assertRaises(ValueError):
            load_attitude_csv(self.write_hitl("none.csv", ["a", "b"], [[1, 2]]))


class AttitudeTabGui(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Tk display unavailable: {exc}")
        self.root.withdraw()
        from attitude_view import AttitudeTab
        self.tab = AttitudeTab(self.root)
        self.tab.pack(fill="both", expand=True)
        self.root.geometry("1200x800")
        self.root.update_idletasks()

    def tearDown(self):
        self.tab.close()
        self.root.destroy()

    def value(self, key):
        return self.tab.values[key].get()

    def test_live_readouts(self):
        self.tab.refresh()
        self.assertIn("no attitude", self.value("age"))
        now = time.monotonic()
        self.tab.accept_record(record(state_frame(PAD, status=status_word(0, 1, 1)), received=now - 0.5))
        self.tab.refresh(now)
        self.assertEqual(self.value("source"), "0x02 (COM7)")
        self.assertEqual(self.value("tilt"), "0.0°")
        self.assertEqual(self.value("state"), "PAD IDLE")
        self.assertIn("ACTIVE", self.value("hitl"))
        self.assertEqual(self.value("gps"), "1")
        self.assertEqual(self.value("qy"), "-0.70711")
        self.assertEqual(self.value("pitch"), "-90.0°")
        self.assertIn("gimbal", self.value("roll"))
        self.assertTrue(self.value("age").startswith("0.5 s"))
        self.tab.accept_record(record(debug_frame(TILT_EAST_10, status=status_word(1, 1, 0)), received=now))
        self.tab.refresh(now + 0.05)
        self.assertEqual(self.value("source"), "0xE0 (COM7)")
        self.assertEqual(self.value("tilt"), "10.0°")
        self.assertEqual(self.value("azimuth"), "90.0°")
        self.assertEqual(self.value("state"), "BOOST")
        self.assertEqual(self.value("hitl"), "off")
        self.assertGreater(len(self.tab.canvas.find_all()), 60)
        self.assertGreater(len(self.tab.chart.find_all()), 10)
        # Non-unit norm flagged; zero quaternion not drawn.
        self.tab.accept_record(record(debug_frame((0, 0, 0, 1.05)), received=now + 0.1))
        self.tab.refresh(now + 0.15)
        self.assertIn("CHECK NORM", self.value("norm"))
        self.tab.accept_record(record(debug_frame((0, 0, 0, 0)), received=now + 0.2))
        self.tab.refresh(now + 0.25)
        self.assertIn("not available", self.value("note"))
        self.assertEqual(self.value("tilt"), "--")
        texts = [self.tab.canvas.itemcget(i, "text") for i in self.tab.canvas.find_all()
                 if self.tab.canvas.type(i) == "text"]
        self.assertIn("ATTITUDE NOT AVAILABLE", texts)
        for name in ("From East", "From North", "From Above", "Iso"):
            self.tab.set_view(name)

    def test_file_replay_with_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "attitude.csv"
            with path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["time_s", "witness_qx", "witness_qy", "witness_qz", "witness_qw",
                                 "truth_qx", "truth_qy", "truth_qz", "truth_qw", "hitl_active"])
                for i in range(21):
                    truth = am.from_tilt_azimuth_spin(i * 0.5, 0.0, 0.0)
                    est = am.apply_heading(am.from_tilt_azimuth_spin(i * 0.5 + 1.0, 0.0, 0.0), 40.0)
                    writer.writerow([i * 0.1, *est, *truth, 1])
            self.tab.load_file(path)
        self.assertEqual(self.tab.mode, "file")
        self.assertIn("attitude.csv", self.value("mode"))
        self.assertIn("auto", self.value("offset"))
        self.assertEqual(self.value("tilt_err"), "1.00°")
        self.assertEqual(self.value("att_err"), "1.00°")
        self.assertIn("ACTIVE", self.value("hitl"))
        self.tab.seek(1.0)
        self.assertEqual(self.value("truth_tilt"), "5.0°")
        self.tab.align_here()
        self.assertIn("manual", self.value("offset"))
        self.tab.toggle_play()
        self.assertTrue(self.tab.playing)
        self.tab.play_time = 5.0
        self.tab._tick()
        self.assertFalse(self.tab.playing)
        self.assertAlmostEqual(self.tab.play_time, 2.0)
        self.tab.use_live()
        self.assertEqual(self.tab.mode, "live")

    def test_horizon_has_attitude_tab_fed_by_witness(self):
        import tkinter as tk
        try:
            from horizon import HorizonGroundStation
        except ImportError as exc:
            self.skipTest(f"Horizon dependency unavailable: {exc}")
        with tempfile.TemporaryDirectory() as directory:
            try:
                app = HorizonGroundStation(log_dir=Path(directory), auto_connect=False, broker_enabled=False)
            except tk.TclError as exc:
                self.skipTest(f"Tk display unavailable: {exc}")
            app.withdraw()
            try:
                titles = [app.notebook.tab(tab, "text") for tab in app.notebook.tabs()]
                self.assertEqual(titles[:2], ["Flight View", "Attitude"])
                app._witness_record(record(state_frame(TILT_EAST_10)))
                app.attitude.refresh()
                self.assertEqual(app.attitude.values["tilt"].get(), "10.0°")
            finally:
                app.close()


if __name__ == "__main__":
    unittest.main()
