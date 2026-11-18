"""Display audit fixes (H2, H7-H10 and the medium items): source following, status strip,
landed fix, staleness, one-way flight segments, decimated history, slot guards."""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

import flight_render as fr  # noqa: E402
import qt_common  # noqa: E402
from flight_data import DEFAULT_LAYOUT, FlightData, PointBuffer, load_layout  # noqa: E402
from map_data import MapData  # noqa: E402
from pad_reference import PadReference  # noqa: E402

LOG_FULL = 0x0080 | 0x0008          # byte 1: log slots full + GPS lock
RESUMED = 0x0040
MAINT = 0x0020
LOGGING_FAILED = 0x8000
FLASH_FAILED = 0x1000
SENSORS = 0x6E00                    # byte 0 bits 1,2,3,5,6


def rec(source, packet, t, **kw):
    return {"source": source, "received_at": time.time(), "received_mono": t, "packet": packet, **kw}


def state(status, t, uptime=1000, **extra):
    return {"packet": "state", "status": status, "uptime": uptime, "kalman_altitude": 10.0, **extra}


class SourceFollow(unittest.TestCase):
    def test_follows_freshest_source_with_one_series(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        for i in range(5):
            data.accept_record("witness", rec("COM7", state(0x08, 0, i), 100 + i))
        self.assertEqual(data.status_snapshot()["sources"]["witness"], "COM7")
        # COM8 appears while COM7 is still fresh: ignored.
        data.accept_record("witness", rec("COM8", state(0x08, 0, 1), 105))
        self.assertEqual(data.status_snapshot()["sources"]["witness"], "COM7")
        # COM7 silent > 3 s, COM8 delivers: switch, series continues in one trace.
        data.accept_record("witness", rec("COM8", state(0x08, 0, 2), 110))
        snap = data.status_snapshot()
        self.assertEqual(snap["sources"]["witness"], "COM8")
        self.assertEqual(snap["switch"][1:3], ("COM7", "COM8"))
        history = data.snapshot()[0][0]
        self.assertEqual(history.source, "COM8")
        times = [t for t, _ in history.points]
        self.assertEqual(times, sorted(times))
        self.assertGreaterEqual(len(times), 6)          # nothing cleared at the switch
        text, _bg = fr.banner(data.layout, data.snapshot(), 111, status=snap)
        self.assertIn("COM8", text)


class StatusStrip(unittest.TestCase):
    def strip(self, status_word, **extra):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", state(status_word, 0, **extra), 100))
        return fr.status_strip(data.status_snapshot(), 100.5)

    def test_not_logging_is_red(self):
        chips = dict(self.strip(LOG_FULL | SENSORS))
        self.assertTrue(any("NOT LOGGING" in c for c, k in self.strip(LOG_FULL | SENSORS) if k == "bad"))
        self.assertNotIn("LOGGING OK", chips)

    def test_failures_resumed_maintenance_hitl(self):
        chips = self.strip(LOGGING_FAILED | FLASH_FAILED | MAINT | RESUMED | 0x10 | SENSORS)
        bad = [c for c, k in chips if k == "bad"]
        warn = [c for c, k in chips if k == "warn"]
        self.assertTrue(any("LOGGING FAILED" in c for c in bad))
        self.assertTrue(any("FLASH INIT FAILED" in c for c in bad))
        self.assertTrue(any("MAINTENANCE" in c for c in bad))
        self.assertTrue(any("RESUMED" in c for c in warn))
        self.assertTrue(any("HITL" in c for c in warn))

    def test_healthy_and_old_status(self):
        chips = self.strip(0x08 | SENSORS)
        self.assertIn(("LOGGING OK", "ok"), chips)
        self.assertIn(("GPS LOCKED", "ok"), chips)
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", state(0x08 | SENSORS, 0), 100))
        old = fr.status_strip(data.status_snapshot(), 130)
        self.assertIn(("LOGGING OK", "muted"), old)       # stale green goes grey
        self.assertTrue(any("OLD" in c for c, _ in old))
        self.assertEqual(fr.status_strip(None, 1), [("NO WITNESS STATUS RECEIVED", "muted")])

    def test_last_known_position_when_no_fix(self):
        chips = self.strip(SENSORS, latitude=35.1, longitude=-117.1)
        self.assertTrue(any("NO FIX" in c and "last known" in c for c, _ in chips))

    def test_derived_status_fields_exist(self):
        from telemetry_model import witness_status_fields
        fields = witness_status_fields({"packet": "state", "status": LOG_FULL | RESUMED | MAINT | LOGGING_FAILED})
        self.assertEqual((fields["status.log_slots_full"], fields["status.resumed"], fields["status.maintenance"],
                          fields["status.logging_failed"], fields["status.flash_init_failed"]), (1, 1, 1, 1, 0))


class LandedAndStale(unittest.TestCase):
    LANDED = {"packet": "landed_position", "status": 0x0004, "flight_state": 4, "uptime": 5, "latitude": 35.5,
              "longitude": -117.6, "gps_altitude": 650.0, "fix_age": 12, "flags": 1, "position_valid": True,
              "stale": False, "ping_reply": True, "utc_aligned": False, "satellites": 7, "battery_voltage": 7.4}

    def test_landed_line_and_map_marker(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", dict(self.LANDED), 50))
        text, kind = fr.landed_line(data.status_snapshot(), 60)
        self.assertIn("35.500000", text)
        self.assertIn("PING-REPLY", text)
        self.assertEqual(kind, "ok")
        pad = PadReference()
        pad.accept_record("witness", rec("COM7", dict(self.LANDED), 50))
        mapd = MapData(pad)
        self.assertTrue(mapd.accept_record("witness", rec("COM7", dict(self.LANDED), 50)))
        snap = mapd.snapshot()
        self.assertEqual(len(snap["tracks"]["land"]), 1)
        self.assertEqual(snap["reference"][2], "first land fix")
        none = dict(self.LANDED, position_valid=False, latitude=None, longitude=None)
        self.assertFalse(mapd.accept_record("witness", rec("COM7", none, 51)))

    def test_no_fix_position_is_last_known_not_track(self):
        mapd = MapData(PadReference())
        pkt = {"packet": "state", "status": 0, "uptime": 1, "position": [0, 0, 0], "orientation": [0, 0, 0, 0],
               "latitude": 35.2, "longitude": -117.2, "gps_altitude": 600.0}
        mapd.accept_record("witness", rec("COM7", pkt, 10))
        snap = mapd.snapshot()
        self.assertEqual(snap["tracks"]["wgps"], [])
        self.assertIsNotNone(mapd.last_known)
        zero = dict(pkt, latitude=0.0, longitude=0.0)
        mapd2 = MapData(PadReference())
        mapd2.accept_record("witness", rec("COM7", zero, 10))
        self.assertIsNone(mapd2.last_known)

    def test_live_means_essential_telemetry(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", state(0x08, 0), 100))
        text, bg = fr.banner(data.layout, data.snapshot(), 101, status=data.status_snapshot())
        self.assertTrue(text.startswith("LIVE"))
        self.assertEqual(bg, "#d8efdf")
        # A heartbeat alone does not keep it green once state/position went quiet.
        data.accept_record("witness", rec("COM7", {"packet": "heartbeat", "status": 8, "uptime": 2}, 108))
        text, bg = fr.banner(data.layout, data.snapshot(), 109, status=data.status_snapshot())
        self.assertTrue(text.startswith("WAITING / STALE"))
        self.assertEqual(bg, "#ffd5cf")

    def test_stale_readouts_grey(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", state(0x08, 0), 100))
        panel = fr.flight_panel(data.layout, data.snapshot(), 101)
        live = [c[1] for _t, cells in panel["metrics"] for c in cells][0]
        stale_panel = fr.flight_panel(data.layout, data.snapshot(), 140)
        stale = [c[1] for _t, cells in stale_panel["metrics"] for c in cells][0]
        self.assertNotEqual(live, fr.STALE_GREY)
        self.assertEqual(stale, fr.STALE_GREY)

    def test_aperture_rssi_goes_stale_without_rocket_packets(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        rssi = lambda t: rec("COM7", {"packet": "ground_station_status", "rssi": -70, "uptime": 1}, t)
        data.accept_record("witness", rec("COM7", state(0x08, 0), 100))
        data.accept_record("witness", rssi(101))
        history = [h for row, hs in zip(data.layout.rows, data.snapshot()) for t, h in zip(row.traces, hs)
                   if t.field == "rssi"][0]
        self.assertEqual((history.quality, history.latest, history.received), ("LIVE", -70, 101))
        data.accept_record("witness", rssi(120))        # no rocket packet for 19 s: Aperture repeats the old value
        history = [h for row, hs in zip(data.layout.rows, data.snapshot()) for t, h in zip(row.traces, hs)
                   if t.field == "rssi"][0]
        self.assertEqual(history.quality, "INVALID")
        self.assertEqual(history.received, 101)         # original age is kept, so it ages out

    def test_future_record_time_is_clamped(self):
        data = FlightData(load_layout(DEFAULT_LAYOUT))
        data.accept_record("witness", rec("COM7", state(0x08, 0), time.monotonic() + 500))
        self.assertLessEqual(data.snapshot()[0][0].received, time.monotonic() + 0.1)


class OneWayFlight(unittest.TestCase):
    def test_track_not_reprojected_after_landed_reboot(self):
        pad = PadReference()
        mapd = MapData(pad)

        def send(status, up, t, pos=(0, 0, 0), lat=35.0, lon=-117.0):
            pkt = {"packet": "state", "status": status, "uptime": up, "position": list(pos),
                   "orientation": [0, 0, 0, 1], "latitude": lat, "longitude": lon, "gps_altitude": 600.0}
            r = rec("COM7", pkt, t)
            pad.accept_record("witness", r)
            mapd.accept_record("witness", r)
        for k in range(10):
            send(0x08, 50_000 + k * 100, 10 + k * .1)
        send(0x09, 60_000, 20, pos=(100.0, 200.0, 300.0))
        before = [(p.east, p.north) for p in mapd.snapshot()["tracks"]["kf"]]
        send(0x0C, 90_000, 30, lat=35.01, lon=-117.01)              # landed away from the pad
        for k in range(30):                                          # rebooted: Pad Idle at the landing site
            send(0x08, 1_000 + k * 100, 40 + k * .1, lat=35.01, lon=-117.01)
        after = [(p.east, p.north) for p in mapd.snapshot()["tracks"]["kf"]]
        self.assertEqual(after[0], before[0])
        self.assertEqual(after[1], before[1])
        self.assertAlmostEqual(pad.position().lat, 35.0)


class History(unittest.TestCase):
    def test_older_points_are_decimated_not_dropped(self):
        buf = PointBuffer(1000)
        for k in range(5000):
            buf.append((k * 0.02, float(k % 7)))     # 50 Hz for 100 s
        self.assertEqual(len(buf), 1000)             # recent tier keeps full rate
        t, v = buf.arrays()
        self.assertLess(t[0], 5.0)                   # window still reaches back ~100 s
        self.assertTrue(np.all(np.diff(t) >= 0))
        self.assertLess(len(t), 3000)                # ...in far fewer points than 5000

    def test_trace_arrays_matches_reference_semantics(self):
        pts = [(1, 1.0), (2, None), (3, 3.0), (8, 8.0), (9, None), (10, None), (11, 4.0)]
        x, y = fr.trace_arrays(pts, 0, 20, 3)
        self.assertEqual(list(x), [1, 2, 3, 3, 8, 9, 11])
        self.assertTrue(np.array_equal(y, [1, np.nan, 3, np.nan, 8, np.nan, 4], equal_nan=True))
        buf = PointBuffer(50)
        for p in pts:
            buf.append(p)
        x2, y2 = fr.trace_arrays(buf.snapshot(), 0, 20, 3)
        self.assertTrue(np.array_equal(x, x2) and np.array_equal(y, y2, equal_nan=True))


class Guards(unittest.TestCase):
    def test_guarded_slot_swallows_and_rate_limits(self):
        calls = []

        @qt_common.guarded_slot
        def boom():
            calls.append(1)
            raise KeyError("same error")
        before = len(qt_common.GUARD_ERRORS)
        for _ in range(5):
            boom()
        self.assertEqual(len(calls), 5)
        self.assertEqual(len(qt_common.GUARD_ERRORS), before + 1)    # logged once per 10 s
        self.assertFalse(qt_common.guard_call("x", lambda: 1 / 0))
        self.assertTrue(qt_common.guard_call("x", lambda: None))

    def test_log_console_burst_is_bounded(self):
        from qt_widgets import LogConsole
        qt_common.ensure_app()
        console = LogConsole(max_lines=50)
        console.append_lines(f"line {i}" for i in range(5000))
        self.assertEqual(console.line_count(), 50)
        self.assertIn("line 4999", console.toPlainText())


class FlightViewGui(unittest.TestCase):
    def test_bad_layout_falls_back_to_default_and_one_bad_widget_does_not_blank_rest(self):
        qt_common.ensure_app()
        import flight_view
        with tempfile.TemporaryDirectory() as directory:
            bad = Path(directory) / "old.json"
            bad.write_text('{"version": 1, "refresh_ms": 50, "plots": []}', encoding="utf-8")
            view = flight_view.FlightView(layout_path=bad)
            try:
                self.assertEqual(len(view.data.layout.plots), 5)         # built-in default, not one plot
                self.assertTrue(view.data.layout.readouts)
                view.data.accept_record("witness", rec("COM7", state(LOG_FULL | SENSORS, 0), time.monotonic()))
                view.graphs[0].update_plot = lambda *a, **k: 1 / 0         # one failing widget
                view.redraw()
                texts = " | ".join(view.texts())
                self.assertIn("NOT LOGGING", texts)                        # strip still drawn
                self.assertIn("LAYOUT ERROR", view.banner.text())
                view._poll()                                               # slot never raises
            finally:
                view.close()


if __name__ == "__main__":
    unittest.main()
