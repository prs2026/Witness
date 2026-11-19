"""Imperial display: ft / mi / ft/s conversion happens only where values are shown."""

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

import flight_render as fr  # noqa: E402
import geodesy as geo  # noqa: E402
import witness_protocol as wp  # noqa: E402
from flight_data import DEFAULT_LAYOUT, FlightData, History, load_layout, validate_layout  # noqa: E402
from nmea_capture import summarize_gga  # noqa: E402
from pad_reference import PadReference  # noqa: E402

FT = 0.3048


class DistanceFormatTest(unittest.TestCase):
    def test_constants_are_exact(self):
        self.assertEqual(geo.FT_M, 0.3048)
        self.assertEqual(geo.MI_M, 5280 * 0.3048)

    def test_feet_below_one_mile_miles_at_and_above(self):
        self.assertEqual(geo.format_distance(0), "0 ft")
        self.assertEqual(geo.format_distance(500 * FT), "500 ft")
        self.assertEqual(geo.format_distance(5279 * FT), "5279 ft")
        self.assertEqual(geo.format_distance(5279.4 * FT), "5279 ft")
        self.assertEqual(geo.format_distance(5280 * FT), "1.00 mi")
        self.assertEqual(geo.format_distance(1.25 * geo.MI_M), "1.25 mi")
        self.assertEqual(geo.format_distance(float("nan")), "--")
        self.assertEqual(geo.format_distance(None), "--")

    def test_nice_ring_steps_are_round_in_the_shown_unit(self):
        for span_ft, want in ((300, 100), (800, 200), (1000, 250), (2000, 500), (4000, 1000),
                              (8000, 2000), (10000, 2640), (12000, 5280), (42000, 10560)):
            self.assertAlmostEqual(geo.nice_ring_step(span_ft * FT, 4) / FT, want, places=6, msg=span_ft)
        # beyond the foot steps the steps are 0.5 / 1 / 2 / 5 / 10 miles
        miles = [geo.nice_ring_step(m * geo.MI_M, 1) / geo.MI_M for m in (0.4, 0.9, 1.5, 3, 7, 15)]
        self.assertEqual([round(m, 9) for m in miles], [0.5, 1, 2, 5, 10, 20])
        # and the labels come out as whole feet / two-decimal miles
        self.assertEqual(geo.format_distance(geo.nice_ring_step(1000 * FT)), "250 ft")
        self.assertEqual(geo.format_distance(geo.nice_ring_step(8 * geo.MI_M, 1)), "10.00 mi")


class WitnessDisplayTest(unittest.TestCase):
    def test_labels_use_feet(self):
        labels = {name: label for fields in wp.SECTION_SPECS.values() for name, label in fields}
        self.assertEqual(labels["position"], "Kalman position [ft; E, N, U from pad]")
        self.assertEqual(labels["velocity"], "Kalman velocity [ft/s; E, N, U]")
        self.assertEqual(labels["kalman_altitude"], "Kalman altitude above pad [ft]")
        self.assertIn("[ft MSL]", labels["gps_altitude"])
        for fields in wp.SECTION_SPECS.values():
            for name, label in fields:
                self.assertNotRegex(label, r"\[m(;|/| |\])", name)

    def test_values_convert_by_field_name_only_at_display(self):
        self.assertEqual(wp.format_display_value("kalman_altitude", 100.0), "328.08")
        self.assertEqual(wp.format_display_value("gps_altitude", 304.8), "1000.00")
        self.assertEqual(wp.format_display_value("position", [30.48, -3.048, 0.0]), "[100.00, -10.00, 0.00]")
        self.assertEqual(wp.format_display_value("velocity", (0.3048, 0.0, 3.048)), "[1.00, 0.00, 10.00]")
        self.assertEqual(wp.format_display_value("position", [1.0, None, 2.0]).count("null"), 1)
        self.assertEqual(wp.format_display_value("kalman_altitude", None), "null")
        # not length fields: unchanged
        self.assertEqual(wp.format_display_value("rssi", -90), "-90")
        self.assertEqual(wp.format_display_value("filtered_accel", [1.0, 2.0, 3.0]), "[1.0, 2.0, 3.0]")
        self.assertEqual(wp.format_display_value("witness_battery_voltage", 4.1), "4.1")
        self.assertEqual(wp.display_unit_for_field("velocity"), "ft/s")
        self.assertAlmostEqual(wp.display_scale("position"), 1 / 0.3048)
        self.assertEqual(wp.display_scale("gyro"), 1.0)

    def test_packet_values_stay_metres(self):
        packet = {"packet": "state", "kalman_altitude": 100.0, "position": [1.0, 2.0, 3.0]}
        line = wp.format_packet(packet)
        self.assertIn("kalman_altitude=328.08", line)
        self.assertEqual(packet["kalman_altitude"], 100.0)     # decoded data untouched

    def test_gga_summary_shows_feet(self):
        text = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        self.assertEqual(summarize_gga(text), "GGA 12:35:19 UTC, fix 1, sats 8, alt 1789 ft")


class FlightUnitDisplayTest(unittest.TestCase):
    def setUp(self):
        self.model = FlightData(load_layout(DEFAULT_LAYOUT))

    def row(self, title, group=""):
        return next((i, r) for i, r in enumerate(self.model.layout.rows) if r.title == title and r.group == group)

    def test_layout_keeps_si_units(self):
        self.assertEqual(self.row("Altitude AGL")[1].unit, "m")
        self.assertEqual(self.row("Vertical velocity")[1].unit, "m/s")
        self.assertEqual(self.row("Altitude AGL", "flight")[1].unit, "m")

    def test_metre_value_reads_in_feet(self):
        self.model.accept("witness", "W", "state", {"kalman_altitude": 100.0, "velocity[2]": 30.48}, 1.0)
        end = 1.5
        index, _row = self.row("Altitude AGL", "flight")
        panel = fr.flight_panel(self.model.layout, self.model.snapshot(), end)
        cells = dict(panel["metrics"])["Altitude AGL"]
        self.assertTrue(cells[0][0].startswith("W: 328.08 ft"), cells[0][0])
        vz = dict(panel["metrics"])["Vertical velocity"]
        self.assertTrue(vz[0][0].startswith("W: 100 ft/s"), vz[0][0])
        # stored history is untouched SI
        self.assertEqual(self.model.snapshot()[index][0].latest, 100.0)
        # the graph legend shows ft too
        pidx, plot = self.row("Altitude AGL")
        legend = fr.legend(plot, self.model.snapshot()[pidx], end)
        self.assertEqual(legend[0][0], "W KF 328.08")

    def test_featherweight_agl_from_feet_shows_about_500_ft(self):
        pad = PadReference()
        model = FlightData(load_layout(DEFAULT_LAYOUT), pad=pad)
        base = {"packet": "GPS_STAT", "unit_type": "BASE", "altitude_ft": 1000, "fix_type": 3,
                "latitude_deg": 34.0, "longitude_deg": -117.0, "text_crc_valid": True,
                "radio_crc_status": "CRC_OK", "vertical_velocity_fps": 0}
        record = {"source": "COM22", "received_mono": 1, "packet": base}
        pad.accept_record("featherweight", record)
        model.accept_record("featherweight", record)
        tracker = dict(base, unit_type="TRACKER", altitude_ft=1500, vertical_velocity_fps=100,
                       latitude_deg=34.001)
        model.accept_record("featherweight", {"source": "COM22", "received_mono": 2, "packet": tracker})
        panel = fr.flight_panel(model.layout, model.snapshot(), 2.2)
        alt = dict(panel["metrics"])["Altitude AGL"]
        f_cell = next(cell[0] for cell in alt if cell[0].startswith("F:"))
        self.assertEqual(f_cell.split()[1:3], ["500", "ft"], f_cell)
        vz = dict(panel["metrics"])["Vertical velocity"]
        f_vz = next(cell[0] for cell in vz if cell[0].startswith("F:"))
        self.assertEqual(f_vz.split()[1:3], ["100", "ft/s"], f_vz)       # fps round-trips through m/s
        idx, _ = self.row("Altitude AGL", "flight")
        self.assertAlmostEqual(model.snapshot()[idx][2].latest, 500 * FT)  # stored value is metres

    def test_series_and_fixed_limits_scale_but_raw_axes_do_not(self):
        _, plot = self.row("Altitude AGL")
        series = [(np.array([0.0, 1.0]), np.array([FT * 10, FT * 20])) for _ in plot.traces]
        shown = fr.scale_series(plot, series)
        for trace, (_x, y) in zip(plot.traces, shown):
            if trace.axis == "right":
                np.testing.assert_allclose(y, [FT * 10, FT * 20])      # "GPS raw" untouched
            else:
                np.testing.assert_allclose(y, [10, 20])
        doc = {"version": 1, "auto_scale": False, "plots": [{"title": "A", "unit": "m", "y_range": [0, 304.8],
               "traces": [{"label": "W", "input": "witness", "packet": "state", "field": "kalman_altitude"}]}]}
        layout = validate_layout(doc)
        ranges, _ = fr.axis_ranges(layout.plots[0], [(np.array([0.0]), np.array([500.0]))], layout)
        self.assertAlmostEqual(ranges["left"][1], 1000.0, places=6)
        self.assertEqual(ranges["left"][0], 0)               # 304.8 m limit shown as 1000 ft

    def test_unit_labels_and_non_length_units(self):
        self.assertEqual([fr.unit_label(u) for u in ("m", "m/s", "km", "g", "deg/s", "V", "GPS raw")],
                         ["ft", "ft/s", "mi", "g", "deg/s", "V", "GPS raw"])
        self.assertAlmostEqual(fr.display_unit("km")[1] * 1.609344, 1.0, places=9)
        trace = self.row("Gyro")[1].traces[0]
        self.assertEqual(fr.value_text(trace, History([], latest=12.5), "deg/s"), "12.5")
        self.assertEqual(fr.value_text(trace, History([], latest=100.0), "m"), "328.08")

    def test_landed_line_in_feet(self):
        status = {"landed": {"received": 0.0, "position_valid": True, "latitude": 34.0, "longitude": -117.0,
                             "gps_altitude": 304.8, "fix_age": 1}}
        self.assertIn("1000 ft MSL", fr.landed_line(status, 1.0)[0])

    def test_graph_panel_labels_in_feet(self):
        from PyQt6 import QtWidgets
        import flight_view
        import pyqtgraph as pg
        pg.setConfigOption("foreground", "k")
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        _, plot = self.row("Altitude AGL")
        panel = flight_view.GraphPanel(plot, 0.0)
        self.assertIn("[ft | GPS raw]", panel.title.text())
        panel.close()


class WitnessTableUnitsTest(unittest.TestCase):
    """The Witness tab table shows ft / ft/s, as its note says; the feed stays SI."""

    def test_length_fields_converted_once(self):
        import time
        from telemetry_model import TelemetryTable, WITNESS_FIELDS
        table = TelemetryTable(WITNESS_FIELDS)
        packet = {"packet": "state", "status": 0, "kalman_altitude": 304.8,
                  "position": [3.048, 0.0, 30.48], "velocity": [0.0, 0.0, -3.048]}
        table.update_record({"source": "W", "received_at": time.time(), "packet": packet},
                            witness=True)
        value = {key[2]: row.value for key, row in table.rows.items() if key[1] == "state"}
        self.assertEqual(value.get("kalman_altitude"), "1000.00")
        self.assertEqual(value.get("position[0]"), "10.00")
        self.assertEqual(value.get("position[2]"), "100.00")
        self.assertEqual(value.get("velocity[2]"), "-10.00")
        self.assertEqual(packet["kalman_altitude"], 304.8)   # input untouched


if __name__ == "__main__":
    unittest.main()
