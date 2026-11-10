"""Arbitrary CSVs in the viewer: Blue Raven, GoPro, semicolon, headerless."""

import math
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import any_csv  # noqa: E402
import viewer_data as viewer  # noqa: E402  (data logic)
from viewer_qt_support import ViewerCase, pump  # noqa: E402


# A small Blue Raven LR-style log: Year/Month/Day/Time, Flight_Time_(s) that
# starts negative, hex FER columns (one only goes hex past the head sample,
# one has a broken header), and repeated flag/Reserved names.
RAVEN_HEADER = ("Year,Month,Day,Time,Flight_Time_(s),Baro_Altitude_AGL_(feet),"
                "Rocket_FER_Hex,Apo_FER_H  ex,LT_AGL1,Reserved,LT_AGL1,Reserved,LT_AGL1")


def raven_rows(count=300):
    rows = []
    for i in range(count):
        seconds = 22.562 + i * 0.02
        rocket = "200" if i < count - 5 else "eff"
        rows.append(f"2025,7,19,12:26:{seconds:06.3f},{-1.90 + i * 0.02:.2f},"
                    f"{i * 0.5:.1f},{rocket},8c5,{i % 2},0,1,0,{i % 3}")
    return rows


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def write(self, name, text, encoding="utf-8"):
        path = Path(self.tmp) / name
        path.write_bytes(text.encode(encoding))
        return path


class BlueRavenTests(Fixture):
    def test_quirks(self):
        path = self.write("raven.csv", RAVEN_HEADER + "\n" + "\n".join(raven_rows()) + "\n")
        sniffed = any_csv.sniff(path)
        self.assertFalse(sniffed.is_plain)
        data = viewer.load_dataset(path)
        names = data.numeric_columns
        for name in ("LT_AGL1", "LT_AGL1#2", "LT_AGL1#3", "Reserved", "Reserved#2",
                     "Apo_FER_H ex", "Flight_Time_(s)", "Time", "datetime"):
            self.assertIn(name, names)
        self.assertEqual(data.kinds["Rocket_FER_Hex"], "hex")
        self.assertEqual(data.kinds["Apo_FER_H ex"], "hex")
        self.assertEqual(data.numeric_data["Apo_FER_H ex"][0], 0x8C5)
        self.assertEqual(data.numeric_data["Rocket_FER_Hex"][-1], 0xEFF)
        self.assertEqual(data.kinds["Time"], "tod")
        self.assertAlmostEqual(data.numeric_data["Time"][0], 12 * 3600 + 26 * 60 + 22.562)
        self.assertAlmostEqual(data.numeric_data["datetime"][0], 44782.562)
        # Auto time axis: Flight_Time_(s), seconds, starting negative.
        self.assertEqual((data.time_column, data.time_unit), ("Flight_Time_(s)", "s"))
        self.assertAlmostEqual(data.times()[0], -1.90)
        # Time of day is offered as a time axis too.
        self.assertIn("Time", data.time_choices())
        data.set_time_axis(column="Time")
        self.assertEqual(data.time_unit, "s")
        self.assertIn("decoded", "decoded")

    def test_hex_as_text_option(self):
        path = self.write("raven.csv", RAVEN_HEADER + "\n" + "\n".join(raven_rows(20)) + "\n")
        data = viewer.load_dataset(path, hex_as_numbers=False)
        self.assertNotIn("Rocket_FER_Hex", data.numeric_columns)
        self.assertEqual(data.text_columns["Apo_FER_H ex"][0], "8c5")

    def test_crossing_midnight_stays_continuous(self):
        text = ("Year,Month,Day,Time,v\n2025,7,19,23:59:59.500,1\n"
                "2025,7,20,00:00:00.500,2\n")
        data = viewer.load_dataset(self.write("m.csv", text))
        self.assertEqual(data.numeric_data["datetime"].tolist(), [86399.5, 86400.5])


class GoProTests(Fixture):
    GOPRO = (
        "﻿GoPro telemetry export\n"
        "# Camera: HERO11 Black\n"
        "Source,GH010123.MP4\n"
        "\n"
        "cts,date,Accelerometer (z) [m/s2],Accelerometer (x) [m/s2],temperature [°C]\n"
        "0,2024-05-14T12:00:00.000Z,9.81,0.1,41.2\n"
        "5.005,2024-05-14T12:00:00.005Z,9.79,,41.2\n"
        "10.01,2024-05-14T12:00:00.010Z,NaN,0.3,41.3\n"
        "15.015,2024-05-14T12:00:00.015Z,9.80,0.2,\n")

    def test_metadata_bom_units_and_cts(self):
        path = self.write("gopro.csv", self.GOPRO)
        sniffed = any_csv.sniff(path)
        self.assertEqual(sniffed.skipped, 3)
        data = viewer.load_dataset(path)
        self.assertEqual(data.row_count, 4)
        self.assertEqual((data.time_column, data.time_unit), ("cts", "ms"))
        self.assertAlmostEqual(data.times()[1], 0.005005)
        accel = data.numeric_data["Accelerometer (z) [m/s2]"]
        self.assertTrue(math.isnan(accel[2]))                       # NaN cell
        self.assertTrue(math.isnan(data.numeric_data["Accelerometer (x) [m/s2]"][1]))
        self.assertEqual(data.kinds["date"], "datetime")
        np.testing.assert_allclose(data.numeric_data["date"], [43200.0, 43200.005,
                                                               43200.010, 43200.015])


class FormatTests(Fixture):
    def test_semicolon_decimal_comma_and_units_row(self):
        text = "time;alt;mode\ns;m;\n0,0;1,5;PAD\n0,1;2,5;PAD\n0,2;3,5;BOOST\n"
        path = self.write("semi.csv", text)
        sniffed = any_csv.sniff(path)
        self.assertEqual(sniffed.delimiter, ";")
        self.assertTrue(sniffed.decimal_comma)
        self.assertTrue(sniffed.units_row)
        data = viewer.load_dataset(path)
        self.assertEqual(data.numeric_data["alt [m]"].tolist(), [1.5, 2.5, 3.5])
        self.assertEqual((data.time_column, data.time_unit), ("time [s]", "s"))
        self.assertEqual(data.text_columns["mode"][2], "BOOST")
        self.assertNotIn("mode", data.numeric_columns)

    def test_tab_separated(self):
        data = viewer.load_dataset(self.write("t.tsv", "t\tx\n0\t1\n1\t2\n"))
        self.assertEqual(data.numeric_data["x"].tolist(), [1.0, 2.0])

    def test_headerless_numeric_file_gets_col_names_and_row_index(self):
        data = viewer.load_dataset(self.write("h.csv", "1,2,3\n4,5,6\n7,8,9\n"))
        self.assertEqual(data.numeric_columns, ["col1", "col2", "col3"])
        self.assertEqual(data.row_count, 3)
        self.assertEqual(data.time_column, viewer.ROW_INDEX)
        self.assertEqual(data.times().tolist(), [0.0, 1.0, 2.0])

    def test_ragged_rows_and_missing_tokens(self):
        text = "a,b,c\n1,2,3\n4,5\n7,NA,9,10\n,null,12\n"
        data = viewer.load_dataset(self.write("r.csv", text))
        np.testing.assert_array_equal(data.numeric_data["a"], [1, 4, 7, np.nan])
        np.testing.assert_array_equal(data.numeric_data["b"], [2, 5, np.nan, np.nan])
        np.testing.assert_array_equal(data.numeric_data["c"], [3, np.nan, 9, 12])

    def test_plain_converted_csv_still_uses_fast_loader(self):
        path = self.write("S1.csv", "packet_type,uptime,kalman_altitude\nstate,1000,5\n")
        self.assertTrue(any_csv.sniff(path).is_plain)
        data = viewer.load_dataset(path)
        self.assertEqual(data.time_column, viewer.AUTO_TIME)
        self.assertEqual(data.packet_types, ["state"])
        self.assertTrue(data.summary["max_altitude"].startswith("5.000 m"))

    def test_kalman_altitude_falls_back_to_barometric(self):
        path = self.write("old.csv", "uptime,barometric_altitude\n1000,7\n2000,9\n")
        self.assertTrue(viewer.load_dataset(path).summary["max_altitude"].startswith(
            "9.000 m"))
        rows = [{"kalman_altitude": "", "barometric_altitude": "4"},
                {"kalman_altitude": "3", "barometric_altitude": "8"}]
        self.assertTrue(viewer.calculate_summary(rows)["max_altitude"].startswith("4.000"))

    def test_unique_names_and_units(self):
        self.assertEqual(any_csv.unique_names(["a", "a", "a#2", "", " b  c "]),
                         ["a", "a#2", "a#2#2", "col4", "b c"])
        self.assertEqual(any_csv.split_unit("Flight_Time_(s)"), ("Flight_Time", "s"))
        self.assertEqual(viewer.infer_time_unit("time (ms)"), "ms")
        self.assertEqual(viewer.infer_time_unit("cts"), "ms")
        self.assertEqual(viewer.infer_time_unit("Milliseconds"), "ms")

    def test_time_column_names(self):
        pick = any_csv.pick_time_column
        self.assertEqual(pick(["x", "Milliseconds"]), ("Milliseconds", "ms"))
        self.assertEqual(pick(["x", "t", "Flight_Time_(s)"]), ("Flight_Time_(s)", "s"))
        self.assertEqual(pick(["x", "timestamp [us]"]), ("timestamp [us]", "us"))
        self.assertEqual(pick(["v", "date"], {"date": "datetime"}), ("date", "s"))
        self.assertIsNone(pick(["a", "b"]))


class TimeOfDayTests(unittest.TestCase):
    def test_scalar_and_vector_agree(self):
        values = ["12:26:22.422", "1:02:03", "00:00:59,5", "nan", "", "23:59:59.999",
                  "12:6:22.4", "x", "100:00:00.25"]
        expected = [44782.422, 3723.0, 59.5, math.nan, math.nan, 86399.999, math.nan,
                    math.nan, 360000.25]
        np.testing.assert_allclose(any_csv.times_of_day(values), expected)
        np.testing.assert_allclose([any_csv.parse_time_of_day(v) for v in values], expected)

    def test_format_and_iso(self):
        self.assertEqual(any_csv.format_time_of_day(44782.422), "12:26:22.422")
        self.assertEqual(any_csv.format_time_of_day(86400.5), "d+1 00:00:00.500")
        from datetime import datetime
        ref = datetime(2024, 5, 14)
        np.testing.assert_allclose(
            any_csv.iso_seconds(["2024-05-14T00:00:01.5", "2024-05-14 01:00:00",
                                 "2024-05-14T02:00:00+02:00", "2024/05/15", "junk"], ref),
            [1.5, 3600.0, 0.0, 86400.0, math.nan])


class PanelTests(ViewerCase):
    def test_panel_opens_generic_csv_with_text_readout(self):
        path = self.write_text("semi.csv", "time;alt;mode;flags_hex\n" + "".join(
            f"{i / 10};{i};{'PAD' if i < 50 else 'BOOST'};0x{i:x}\n" for i in range(100)))
        panel = self.make()
        self.assertTrue(panel.load_csv(path))
        dataset = next(iter(panel.datasets.values()))
        self.assertIn("semicolon", dataset.format_note)
        panel.layout_model.clear()
        panel.layout_model.add([(0, "flags_hex")])
        panel.rebuild_graphs()
        self.plots(panel)
        low, high = panel.current_x_range()
        rows = panel.hover_readout(0, low + 0.75 * (high - low))
        self.assertTrue(any("= 0x" in row for row in rows), rows)
        self.assertIn("mode: BOOST", rows)
        # The hover box and crosshair draw on the live plot.
        panel.show_hover(0, low + 0.75 * (high - low))
        slot = panel.slots[0]
        self.assertTrue(slot.hover_text.isVisible())
        self.assertTrue(slot.hover_line.isVisible())
        panel.clear_hover()
        self.assertFalse(slot.hover_text.isVisible())

    def test_hex_checkbox_applies_to_new_loads(self):
        path = self.write_text("h.csv", "t;flags_hex\n" + "".join(
            f"{i};0x{i:x}\n" for i in range(20)))
        panel = self.make()
        panel.hex_check.setChecked(False)
        panel.add_files([path])
        self.assertTrue(panel.wait_for_loads(30))
        dataset = next(iter(panel.datasets.values()))
        self.assertNotIn("flags_hex", dataset.numeric_columns)
        self.assertIn("flags_hex", dataset.text_columns)


if __name__ == "__main__":
    unittest.main()
