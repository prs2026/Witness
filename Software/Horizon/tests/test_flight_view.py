"""Flight View configuration, full-rate ingestion, drawing and GUI integration."""

import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from aim_mqtt import AimMqttReceiver, AimSettings  # noqa: E402
from flight_data import DEFAULT_LAYOUT, FlightData, load_layout, nice_step, plot_segments, validate_layout
from ground_receivers import LoggedReceiver
from flight_render import BG, PANEL, TEXT, MUTED, WARN, BAD, readable_color


def contrast_on_white(color):
    channels = [int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    linear = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4 for v in channels]
    return 1.05 / (.05 + sum(v * w for v, w in zip(linear, (.2126, .7152, .0722))))


def document(**changes):
    return {"version": 1, "max_points": 100, "plots": [{"title": "Acceleration", "unit": "g", "traces": [
        {"label": "X", "input": "witness", "packet": "sensors", "field": "filtered_accel[0]"}]}], **changes}


class LayoutTests(unittest.TestCase):
    def test_compact_power_rows_and_voltage_precision(self):
        from flight_data import History
        from flight_render import value_text
        layout = load_layout(DEFAULT_LAYOUT)
        self.assertNotIn("AIM GPS altitude", [row.title for row in layout.readouts])
        self.assertNotIn("Featherweight fix", [row.title for row in layout.readouts])
        pair = next(row for row in layout.readouts if row.title == "AIM CPU / eject battery")
        self.assertEqual([trace.label for trace in pair.traces], ["CPU", "Eject"])
        for row in layout.readouts:
            for trace in row.traces:
                if "voltage" in trace.field:
                    self.assertEqual(value_text(trace, History([], latest=4.123456)), "4.123")
                    self.assertEqual(value_text(trace, History([], latest=12)), "12.000")
                    self.assertEqual(value_text(trace, History([])), "--")
        doc = document(readouts=[{"group": "power", "title": "Pair", "unit": "V", "traces": [
            {"label": "CPU", "input": "aim", "packet": "computer_battery_payload/v1", "field": "voltage"},
            {"label": "Eject", "input": "aim", "packet": "ejection_battery_payload/v1", "field": "voltage"}]}])
        validate_layout(doc)
        doc["readouts"][0]["traces"].append(dict(doc["readouts"][0]["traces"][0]))
        with self.assertRaises(ValueError):
            validate_layout(doc)

    def test_secondary_axis_validation(self):
        doc = document()
        trace = doc['plots'][0]['traces'][0]
        trace['axis'] = 'right'
        with self.assertRaises(ValueError):
            validate_layout(doc)
        doc['plots'][0]['right_unit'] = 'raw'
        trace['unit'] = 'g'
        with self.assertRaises(ValueError):
            validate_layout(doc)
        trace['unit'] = 'raw'
        layout = validate_layout(doc)
        self.assertEqual(layout.plots[0].traces[0].axis, 'right')
        trace['axis'] = 'invalid'
        with self.assertRaises(ValueError):
            validate_layout(doc)

    def test_light_palette_and_legacy_color_readability(self):
        self.assertEqual(BG, "#ffffff")
        self.assertEqual(PANEL, "#ffffff")
        layout = load_layout(DEFAULT_LAYOUT)
        colors = [TEXT, MUTED, WARN, BAD] + [t.color for row in layout.rows for t in row.traces]
        colors += [t.color for row in validate_layout(document()).rows for t in row.traces]
        for color in colors:
            with self.subTest(color=color):
                self.assertGreaterEqual(contrast_on_white(color), 7)
                self.assertEqual(readable_color(color), color)
        for color in ("#ffffff", "#edf3fa", "#ff7979", "#70df9b", "#5cc8ff", "#ffb454", "#77dd99"):
            with self.subTest(legacy=color):
                self.assertGreaterEqual(contrast_on_white(readable_color(color)), 7)
        doc = document()
        doc["plots"][0]["traces"][0]["color"] = "#ffffff"
        custom = validate_layout(doc)
        readable_color(custom.plots[0].traces[0].color)
        self.assertEqual(custom.document["plots"][0]["traces"][0]["color"], "#ffffff")

    def test_default_and_strict_validation(self):
        layout = load_layout(DEFAULT_LAYOUT)
        self.assertEqual(len(layout.plots), 6)
        self.assertEqual(len(layout.readouts), 12)
        self.assertFalse(any("continuity" in row.title.lower() for row in layout.readouts))
        self.assertEqual({t.input for p in layout.plots for t in p.traces}, {"witness", "aim", "featherweight"})
        for changes in ({"version": True}, {"version": 2}, {"columns": 1.5}, {"columns": 0},
                        {"window_seconds": float("nan")}, {"refresh_ms": 0}, {"max_points": 20001},
                        {"gap_seconds": 0}, {"plots": []}, {"column": 2}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_layout(document(**changes))
        for key, value in (("color", "red"), ("input", "serial"), ("field", ""), ("scale", float("inf")),
                           ("match", []), ("match", {"a": []}), ("source", 12)):
            doc = document()
            doc["plots"][0]["traces"][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_layout(doc)
        for limits in ([1, 1], [2, 1], [0], "auto", [0, float("inf")]):
            doc = document()
            doc["plots"][0]["y_range"] = limits
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                validate_layout(doc)
        doc = document(max_points=20000)
        doc["plots"] *= 16
        doc["plots"][0]["traces"] *= 8
        with self.assertRaises(ValueError):
            validate_layout(doc)

    def test_bad_or_oversized_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "layout.json"
            for content in (b"not json", b"x" * 262145, b"[]"):
                path.write_bytes(content)
                with self.assertRaises(ValueError):
                    load_layout(path)


class HistoryTests(unittest.TestCase):
    def test_time_window_keeps_history_sources_and_origins(self):
        model = FlightData(validate_layout(document()))
        for stamp in range(0, 500, 10):
            model.accept('witness', 'W', 'sensors', {'filtered_accel[0]': stamp}, stamp)
        before = model.snapshot()
        bindings = dict(model.selected)
        for seconds in (10, 30, 60, 120, 300, 600, 1800, 3600):
            model.set_window(seconds)
            self.assertEqual(model.layout.window_seconds, seconds)
            self.assertEqual(model.layout.document['window_seconds'], seconds)
            self.assertEqual(model.snapshot(), before)
            self.assertEqual(model.selected, bindings)
        for bad in (True, 0, 3601, float('nan'), '60'):
            with self.assertRaises(ValueError):
                model.set_window(bad)

    def test_default_aim_acceleration_uses_independent_raw_axis(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        for value in (1, 9):
            model.accept_aim('A', 'z_acceleration_payload/v1', {'z_acceleration': value}, value, False)
        model.accept('witness', 'W', 'sensors', {'filtered_accel[0]': 2, 'gyro[0]': 3}, 10)
        self.assertEqual(model.snapshot()[1][0].latest, 2)
        self.assertEqual(model.snapshot()[1][3].latest, 9)
        self.assertEqual(model.layout.plots[1].traces[3].axis, 'right')
        self.assertEqual(model.layout.plots[1].right_unit, 'A raw')
        # Partial orientation without a timestamp cannot produce angular rates.
        model.accept_aim('A', 'orientation_payload/v1', {'x': 1, 'y': 2, 'z': 3}, 11, False)
        self.assertEqual(model.snapshot()[2][3].quality, 'INVALID')
        self.assertIsNone(model.snapshot()[2][3].latest)

    def test_peak_is_preserved_before_receiver_coalescing(self):
        model = FlightData(validate_layout(document()))
        receiver = LoggedReceiver("unused", "test", on_record=lambda r: model.accept_record("witness", r))
        for index in range(50):
            receiver.accept({"source": "COM1", "received_at": time.time(), "packet": {
                "packet": "sensors", "filtered_accel": [99 if index == 19 else 1, 0, 0]}})
        self.assertEqual(len(receiver.drain()[0]), 1)
        history = model.snapshot()[0][0]
        self.assertEqual(len(history.points), 50)
        self.assertEqual(max(value for _, value in history.points), 99)

    def test_auto_source_lock_explicit_source_and_scaling(self):
        doc = document()
        second = dict(doc["plots"][0]["traces"][0], source="COM2", scale=2, offset=3)
        doc["plots"][0]["traces"].append(second)
        model = FlightData(validate_layout(doc))
        model.accept("witness", "COM1", "sensors", {"filtered_accel[0]": 5}, 1)
        model.accept("witness", "COM2", "sensors", {"filtered_accel[0]": 7}, 2)
        auto, exact = model.snapshot()[0]
        self.assertEqual((auto.latest, auto.source), (5, "COM1"))
        self.assertEqual((exact.latest, exact.source), (17, "COM2"))
        model.accept("witness", "TEST_demo", "sensors", {"filtered_accel[0]": 40}, 3)
        self.assertEqual(model.snapshot()[0][0].latest, 5)
        model.clear()
        model.accept("witness", "TEST_demo", "sensors", {"filtered_accel[0]": 40}, 3)
        self.assertEqual(model.snapshot()[0][0].source, "TEST_demo")

    def test_snapshots_invalid_out_of_order_and_bounds(self):
        model = FlightData(validate_layout(document()))
        def accept(value, stamp, **kw):
            model.accept("witness", "COM1", "sensors", {"filtered_accel[0]": value}, stamp, **kw)
        accept(1, 1, snapshot=True)
        self.assertEqual(model.snapshot()[0][0].quality, "SNAPSHOT")
        self.assertFalse(model.snapshot()[0][0].points)
        accept(3, 2)
        accept(7, 1.5)
        accept(8, 4, snapshot=True)
        self.assertEqual(model.snapshot()[0][0].latest, 3)
        for i, value in enumerate((None, "bad", float("nan"), float("inf"), 10**1000)):
            accept(value, 3 + i)
            self.assertEqual(model.snapshot()[0][0].quality, "INVALID")
            self.assertIsNone(model.snapshot()[0][0].points[-1][1])
            self.assertEqual(model.snapshot()[0][0].latest, 3)
            self.assertEqual(model.snapshot()[0][0].received, 2)
        for stamp in range(10, 300):
            accept(2, stamp)
        self.assertEqual(len(model.snapshot()[0][0].points), 100)
        accept(2, 1000)
        self.assertEqual(len(model.snapshot()[0][0].points), 100)  # Zooming back out still has older samples.
        accept(2, 5000)
        self.assertEqual(len(model.snapshot()[0][0].points), 1)

    def test_featherweight_crc_fix_and_tracker_filter(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        packet = {"packet": "GPS_STAT", "unit_type": "BASE", "altitude_ft": 1000,
                  "fix_type": 3, "text_crc_valid": True, "radio_crc_status": "CRC_OK"}
        def accept(stamp):
            model.accept_record("featherweight", {"source": "COM22", "received_mono": stamp,
                                                 "packet": dict(packet)})
        accept(1)
        self.assertIsNone(model.snapshot()[0][1].latest)
        packet["unit_type"] = "TRACKER"
        accept(2)
        self.assertEqual(model.snapshot()[0][1].latest, 0)   # AGL: first tracker fix without a pad altitude
        packet["altitude_ft"] = 2000
        accept(2.5)
        self.assertAlmostEqual(model.snapshot()[0][1].latest, 304.8)
        packet["text_crc_valid"] = False
        accept(3)
        self.assertAlmostEqual(model.snapshot()[0][1].latest, 304.8)
        self.assertEqual(model.snapshot()[0][1].received, 2.5)
        packet["text_crc_valid"], packet["fix_type"] = True, 0
        accept(4)
        self.assertAlmostEqual(model.snapshot()[0][1].latest, 304.8)
        self.assertFalse(any(row.title == "Featherweight fix" for row in model.layout.readouts))

    def test_mqtt_full_rate_retained_and_partial_updates(self):
        doc = document()
        doc["plots"][0]["traces"] = [{"label": "AIM", "input": "aim", "packet": "z_acceleration_payload/v1",
                                     "field": "z_acceleration"}]
        model = FlightData(validate_layout(doc))
        receiver = AimMqttReceiver(AimSettings("localhost", 1883, "124", "entacore/aim/1234"),
                                   Path("unused"), on_sample=model.accept_aim)
        for payload, retained in ((b'{"z_acceleration": 20}', True), (b'{"z_acceleration": 1}', False),
                                  (b'{"z_acceleration": 30}', False), (b'{"timestamp": 100}', False),
                                  (b'bad', False), (b'{"z_acceleration": 2}', False)):
            receiver._on_message(None, None, SimpleNamespace(topic="entacore/aim/1234/z_acceleration_payload/v1",
                payload=payload, qos=0, retain=retained))
        history = model.snapshot()[0][0]
        self.assertEqual([v for _, v in history.points], [1, 30, 2])
        self.assertEqual(history.source, "localhost:1883/entacore/aim/1234")
        self.assertEqual(receiver.errors, 1)

    def test_thread_safety_during_reload(self):
        layout = validate_layout(document())
        model = FlightData(layout)
        errors = []
        def receive():
            try:
                for i in range(10000):
                    model.accept("witness", "COM1", "sensors", {"filtered_accel[0]": i}, i / 100)
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=receive)
        thread.start()
        for _ in range(50):
            model.configure(layout)
            model.snapshot()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)

    def test_decimation_and_gap_boundaries(self):
        points = [(i / 10000, 100 if i == 2345 else -20 if i == 2346 else 1) for i in range(10000)]
        segments = plot_segments(points, 0, 1, 100, 1)
        self.assertLessEqual(len(segments[0]), 400)
        self.assertEqual(max(v for _, v in segments[0]), 100)
        self.assertEqual(min(v for _, v in segments[0]), -20)
        segments = plot_segments([(1, 1), (2, None), (3, 3), (8, 8)], 0, 10, 100, 3)
        self.assertEqual(segments, [[(1, 1)], [(3, 3)], [(8, 8)]])
        self.assertEqual(nice_step(120), 25)

    def test_distance_origin_reset_and_cached_data(self):
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        def witness(x, y, t, snapshot=False):
            model.accept("witness", "W", "state", {"position[0]": x, "position[1]": y}, t, snapshot=snapshot)
        witness(1000, 2000, 1, snapshot=True)
        self.assertIsNone(model.snapshot()[4][0].latest)   # cached data never counts
        # Witness Kalman ENU has its origin at the pad: distance = hypot(E, N).
        witness(3, 4, 2)
        self.assertEqual(model.snapshot()[4][0].latest, 5)
        witness(0, 0, 1.5)  # Out-of-order updates do not move the latest reading.
        self.assertEqual(model.snapshot()[4][0].latest, 5)
        model.reset_pad()
        self.assertIsNone(model.snapshot()[4][0].latest)
        witness(6, 8, 4)
        self.assertEqual(model.snapshot()[4][0].latest, 10)
        self.assertFalse(model.origins)
        model.accept_aim("A", "gnss_payload/v1", {"gnss_coordinate.latitude": 0,
                         "gnss_coordinate.longitude": 0, "satellites_used": 0}, 6, False)
        self.assertEqual(len(model.origins), 0)  # No bogus origin with no satellites.
        for longitude, stamp in ((0, 7), (.001, 8)):
            model.accept_aim("A", "gnss_payload/v1", {"gnss_coordinate.latitude": 0,
                             "gnss_coordinate.longitude": longitude, "satellites_used": 9}, stamp, False)
        self.assertAlmostEqual(model.snapshot()[4][1].latest, 111.1949, places=3)

    def test_shared_pad_reference_sets_agl_and_distance(self):
        from pad_reference import PadReference
        pad = PadReference()
        model = FlightData(load_layout(DEFAULT_LAYOUT), pad=pad)
        base = {"packet": "GPS_STAT", "unit_type": "BASE", "altitude_ft": 1000, "fix_type": 3,
                "latitude_deg": 34.0, "longitude_deg": -117.0, "text_crc_valid": True,
                "radio_crc_status": "CRC_OK"}
        record = {"source": "COM22", "received_mono": 1, "packet": base}
        pad.accept_record("featherweight", record)
        model.accept_record("featherweight", record)
        tracker = dict(base, unit_type="TRACKER", altitude_ft=1500, latitude_deg=34.001)
        model.accept_record("featherweight", {"source": "COM22", "received_mono": 2, "packet": tracker})
        altitude = next(i for i, row in enumerate(model.layout.plots) if row.title == "Altitude AGL")
        self.assertAlmostEqual(model.snapshot()[altitude][1].latest, 500 * 0.3048)
        self.assertAlmostEqual(model.snapshot()[4][2].latest, 111.19, places=1)
        # Witness GPS AGL needs the GPS-locked bit; pad MSL = 304.8 m.
        fields = {"gps_altitude": 404.8, "status.gps_locked": 1}
        derived = model._derive_agl("witness", "W", "state", fields, False, True)
        self.assertAlmostEqual(derived["_derived.gps_altitude_agl_m"], 100.0)
        derived = model._derive_agl("witness", "W", "state", dict(fields, **{"status.gps_locked": 0}), False, True)
        self.assertIsNone(derived["_derived.gps_altitude_agl_m"])

    def test_readout_unavailable_state_and_optional_continuity_mapping(self):
        from flight_render import value_text
        model = FlightData(load_layout(DEFAULT_LAYOUT))
        for row, histories in zip(model.layout.rows, model.snapshot()):
            for trace, history in zip(row.traces, histories):
                if not trace.enabled:
                    self.assertEqual(history.quality, "UNAVAILABLE")
                    self.assertEqual(value_text(trace, history), "N/A")
        for state in range(8):
            model.accept_record("witness", {"source": "W", "received_mono": state, "packet": {
                "packet": "heartbeat", "status": 0x6E00 | state}})
            row_index = next(i for i, row in enumerate(model.layout.rows) if row.group == "state")
            trace = model.layout.rows[row_index].traces[0]
            text = value_text(trace, model.snapshot()[row_index][0])
            self.assertEqual(text, ("PAD IDLE", "BOOST", "COAST", "DESCENT", "LANDED", "INVALID STATE", "INVALID STATE", "INVALID STATE")[state])
        doc = document()
        doc["readouts"] = [{"group": "power", "title": "Mapped test continuity", "unit": "state", "traces": [
            {"label": "A", "input": "aim", "packet": "test/v1", "field": "continuity", "format": "continuity"}]}]
        model = FlightData(validate_layout(doc))
        for value, expected in ((0, "OPEN"), (1, "CONTINUITY"), (2, "UNKNOWN")):
            model.accept_aim("A", "test/v1", {"continuity": value}, value, False)
            self.assertEqual(value_text(model.layout.readouts[0].traces[0], model.snapshot()[1][0]), expected)


class FlightGuiTests(unittest.TestCase):
    def test_render_reload_pause_resizable_layout_and_receiver_wiring(self):
        from PyQt6 import QtCore
        from horizon_qt_support import make_horizon, pump
        from witness_protocol import PacketDecoder
        from witness_test_source import demo_packets
        with tempfile.TemporaryDirectory() as directory:
            app = make_horizon(directory)
            try:
                flight = app.flight
                for label in ("Load layout...", "Save layout...", "Default layout", "Pause view",
                              "Clear plots", "Set pad", "Graph time window", "Fullscreen"):
                    self.assertIn(label, flight.menu_labels())
                flight.pause_action.trigger()
                self.assertTrue(flight.paused)
                self.assertEqual(flight.pause_action.text(), "Resume live")
                flight.pause_action.trigger()
                self.assertFalse(flight.paused)
                self.assertIs(app.notebook.currentWidget(), flight)
                self.assertEqual(flight.splitter.orientation(), QtCore.Qt.Orientation.Horizontal)
                self.assertEqual(flight.splitter.count(), 3)
                self.assertEqual(len(flight.graphs), 6)
                receiver = app.witness.make_receiver()  # No connection opened.
                decoder = PacketDecoder(lambda packet: receiver.accept({"source": "TEST_flight",
                    "received_at": time.time(), "packet": packet}))
                for stamp in range(0, 3000, 100):
                    for frame in demo_packets(stamp).values():
                        decoder.feed(frame)
                flight.redraw()
                self.assertIn("TEST DATA", flight.banner.text())
                self.assertEqual(len(flight.data.snapshot()[1][0].points), 60)
                # Resizable: shown at small and large sizes, every plot stays visible.
                app.show()
                for width, height in ((1000, 650), (1366, 768), (800, 500)):
                    app.resize(width, height)
                    pump(0.05)
                    flight.redraw()
                    for graph in flight.graphs:
                        self.assertTrue(graph.isVisible())
                        self.assertGreater(graph.widget.height(), 40)
                flight.toggle_fullscreen()
                pump(0.05)
                self.assertTrue(app.isFullScreen())
                flight.exit_fullscreen()
                pump(0.05)
                self.assertFalse(app.isFullScreen())
                # The default summary/state/GPS/power and ALL six graphs show their readouts.
                flight.data.accept_aim('TEST_AIM', 'z_acceleration_payload/v1', {'z_acceleration': 1},
                                       time.monotonic() - 1, False)
                flight.data.accept_aim('TEST_AIM', 'z_acceleration_payload/v1', {'z_acceleration': 20},
                                       time.monotonic(), False)
                from test_aim_orientation import rotation, sample
                now = time.monotonic()
                flight.data.accept_aim('TEST_AIM', 'computer_battery_payload/v1', {'voltage': 4.123456}, now, False)
                flight.data.accept_aim('TEST_AIM', 'ejection_battery_payload/v1', {'voltage': 8.765432}, now, False)
                for index, angle in enumerate((0, 9, 18)):
                    flight.data.accept_aim('TEST_AIM', 'orientation_payload/v1',
                                           sample(rotation((1, 0, 0), angle), index * .1),
                                           time.monotonic() - .3 + index * .1, False)
                flight.redraw()
                texts = flight.texts()
                self.assertIn("PAD IDLE", texts)
                self.assertIn("TEST_flight", texts)
                # Witness GPS: satellites with the status-byte lock bit beside it.
                self.assertRegex(" | ".join(texts), r"GPS sats \d+!?  lock [01]")
                self.assertTrue(any(value.startswith("HITL off") for value in texts))
                self.assertTrue(any("RSSI" in value for value in texts))
                self.assertTrue(any('FW tracker battery' in value for value in texts))
                self.assertTrue(any('FW local / GS battery' in value for value in texts))
                self.assertIn('4.123 raw', texts)
                self.assertIn('8.765 raw', texts)
                self.assertTrue(any('AX~ 90' in value for value in texts))
                self.assertTrue(any('AY~ 0' in value for value in texts))
                self.assertTrue(any('AZ~ 0' in value for value in texts))
                # Readable colours, AIM dashed, Witness solid, curves hold data.
                for graph in flight.graphs:
                    for label in graph.legend_labels:
                        color = label.property("fg")
                        if color:
                            self.assertGreaterEqual(contrast_on_white(color), 7)
                    for curve, trace in zip(graph.curves, graph.plot.traces):
                        style = curve.opts["pen"].style()
                        expected = QtCore.Qt.PenStyle.DashLine if trace.input == "aim" else QtCore.Qt.PenStyle.SolidLine
                        self.assertEqual(style, expected)
                gyro = next(g for g in flight.graphs if g.plot.title.startswith("Gyro"))
                aim_curves = [c for c, t in zip(gyro.curves, gyro.plot.traces) if t.input == "aim"]
                self.assertTrue(any(c.xData is not None and len(c.xData) for c in aim_curves))
                flight.toggle_pause()
                frozen = flight.frozen
                flight.select_window(30)
                self.assertEqual(flight.data.layout.window_seconds, 30)
                self.assertEqual(flight.window_label(), "30 s")
                self.assertIs(frozen, flight.frozen)
                decoder.feed(demo_packets(7000)[0x02])
                flight.redraw()
                self.assertIs(frozen, flight.frozen)
                self.assertIn("PAUSED", flight.banner.text())
                self.assertEqual(flight.data.snapshot()[0][0].latest, 48)
                flight.toggle_pause()
                flight.redraw(end=time.monotonic() + 12)
                self.assertTrue(any("STALE" in value for value in flight.texts()))
                bad = Path(directory) / "bad.json"
                bad.write_text('{"version": 5}')
                with patch("flight_view.show_error") as error:
                    self.assertFalse(flight.load_config(bad))
                    self.assertTrue(error.called)
                    self.assertEqual(len(flight.data.layout.plots), 6)
                good = Path(directory) / "custom.json"
                good.write_text(json.dumps(document(title="Custom flight")))
                self.assertTrue(flight.load_config(good))
                self.assertEqual(len(flight.data.layout.plots), 1)
                self.assertEqual(len(flight.graphs), 1)
                self.assertIsNone(flight.data.snapshot()[0][0].latest)
                saved = Path(directory) / "saved.json"
                flight.select_window(600)
                self.assertTrue(flight.save_layout(str(saved)))
                self.assertEqual(load_layout(saved).title, "Custom flight")
                self.assertEqual(load_layout(saved).window_seconds, 600)
                self.assertTrue(flight.load_config(DEFAULT_LAYOUT))
                self.assertEqual(flight.window_label(), "120 s")
                # AIM tab forwards its callback to the subscriber, no MQTT started.
                with patch("aim_tab.AimMqttReceiver.start"):
                    app.aim.toggle_connection()
                self.assertIsNotNone(app.aim.receiver.on_sample)
                app.aim.receiver.on_sample("localhost:1883/entacore/aim/1234", "rssi_payload/v1",
                                           {"rssi": -99}, time.monotonic(), False)
                flight.redraw()
                self.assertEqual(flight.data.snapshot()[5][1].latest, -99)
                flight.clear()
                self.assertIsNone(flight.data.snapshot()[5][1].latest)
                self.assertIsNotNone(app.featherweight.on_record)
            finally:
                app.close()

    def test_redraws_are_timer_driven_not_per_record(self):
        from horizon_qt_support import make_horizon, pump
        with tempfile.TemporaryDirectory() as directory:
            app = make_horizon(directory)
            try:
                app.show()
                pump(0.1)
                flight = app.flight
                before = flight.redraws
                started = time.monotonic()
                for i in range(3000):
                    flight.data.accept_record("witness", {"source": "COM1", "received_at": time.time(),
                        "packet": {"packet": "state", "status": 2, "uptime": i, "kalman_altitude": float(i)}})
                pump(0.6)
                elapsed = time.monotonic() - started
                draws = flight.redraws - before
                self.assertGreaterEqual(draws, 1)
                self.assertLessEqual(draws, elapsed / (max(33, flight.data.layout.refresh_ms) / 1000) + 3)
                self.assertEqual(flight.data.snapshot()[0][0].latest, 2999)
            finally:
                app.close()

    def test_invalid_startup_layout_keeps_other_tabs_working(self):
        from horizon_qt_support import make_horizon
        with tempfile.TemporaryDirectory() as directory:
            app = make_horizon(directory, flight_layout=Path("missing_layout_file.json"))
            try:
                app.flight.redraw()
                self.assertIn("LAYOUT ERROR", app.flight.banner.text())
                self.assertEqual(app.notebook.count(), 6)
            finally:
                app.close()


if __name__ == "__main__":
    unittest.main()
