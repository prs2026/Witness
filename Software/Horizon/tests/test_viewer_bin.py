"""Opening raw Witness .bin logs in the viewer (decoded by converter.py)."""

import os
import shutil
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import converter  # noqa: E402
import viewer_data as viewer  # noqa: E402  (data logic)
from viewer_qt_support import ViewerCase, pump  # noqa: E402


def imu_record(us, low, high, gyro, status, sequence):
    return struct.pack(">I9hBB", us & 0xFFFFFFFF, *low, *high, *gyro, status, sequence)


def wl_v2_record(status0, status1, sequence, uptime_ms, temp, pressure, ms_temp):
    return (b"WL" + bytes([2, status0, status1, 0])
            + struct.pack(">IIhii", sequence, uptime_ms, temp, pressure, ms_temp))


def packet_record(us, source, frame):
    return struct.pack(">IBB", us, source, len(frame)) + frame


def state_frame(status, uptime_ms, baro_alt):
    # status, uptime, orientation/position/velocity/lat/lon (46 bytes), gps alt,
    # baro alt, battery, satellites: 63 bytes.
    payload = (struct.pack(">HI", status, uptime_ms) + bytes(46)
               + struct.pack(">f", 0.0) + struct.pack(">f", baro_alt)
               + struct.pack(">H", 7400) + bytes([9]))
    return bytes([0x02]) + payload + bytes([0x0A])


class BinFixtures(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, name, data):
        path = self.tmp / name
        path.write_bytes(data)
        return path

    def imu_file(self, n=3000):
        data = b"".join(
            imu_record(0xFFFF0000 + i * 1042, (i % 50, -i % 70, 2048), (i % 9, 0, -3),
                       (i % 11, 5, -5), (0x09 if i > 1000 else 0x08) | 0x70, i % 256)
            for i in range(n))
        return self.write("I0000001.BIN", data + b"\x01\x02")   # 2 stray bytes

    def wl_file(self, n=500):
        data = b"".join(wl_v2_record(0x6E, (1 if 200 <= i < 300 else 0) | 0x08, i,
                                     1000 + i * 10, 256, 98000 - i * 3, 2500)
                        for i in range(n))
        return self.write("S0000001.BIN", data)

    def packet_file(self):
        data = b"".join(packet_record(i * 20000, 1, state_frame(0x6E00 | (i // 10 % 3),
                                                                5000 + i * 20, i * 1.5))
                        for i in range(60))
        data += packet_record(1_300_000, 2, bytes([0x05, 0, 0, 0, 0, 0, 0, 0x73, 0x01, 0x0A]))
        return self.write("P0000001.BIN", data)


class LoadBinTests(BinFixtures):
    def assert_same_as_csv(self, path):
        """The .bin loaded directly == its converter CSV loaded."""
        from_bin = viewer.load_dataset(path)          # dispatches on .bin
        data = path.read_bytes()
        _fmt, decoder = converter.select_decoder(path, data)
        csv_path = path.with_suffix(".csv")
        converter.write_csv(list(decoder(data)), csv_path)
        from_csv = viewer.load_dataset(csv_path)
        self.assertEqual(from_bin.numeric_columns, from_csv.numeric_columns)
        for name in from_csv.numeric_columns:
            np.testing.assert_array_equal(from_bin.numeric_data[name],
                                          from_csv.numeric_data[name], err_msg=name)
        self.assertEqual(from_bin.summary, from_csv.summary)
        self.assertEqual(from_bin.packet_types, from_csv.packet_types)
        self.assertEqual((from_bin.time_column, from_bin.time_unit),
                         (from_csv.time_column, from_csv.time_unit))
        self.assertEqual(from_bin.label, path.name)
        return from_bin

    def test_imu_log(self):
        dataset = self.assert_same_as_csv(self.imu_file())
        self.assertEqual(dataset.source_format, "Witness IMU records (24-byte)")
        self.assertEqual(dataset.row_count, 3000)
        self.assertEqual(dataset.discarded, 2)
        self.assertEqual((dataset.time_column, dataset.time_unit), ("timestamp_us", "us"))
        # The 32-bit microsecond counter wraps inside the file: still monotonic.
        self.assertTrue(np.all(np.diff(dataset.times()) > 0))

    def test_wl_flash_log_with_paired_txt(self):
        path = self.wl_file()
        path.with_suffix(".TXT").write_text("I (2500) flight_state: pad idle -> boost\n")
        dataset = self.assert_same_as_csv(path)
        self.assertEqual(dataset.source_format, "Witness flash records")
        self.assertEqual(dataset.time_column, viewer.AUTO_TIME)
        self.assertIsNone(dataset.paired_state_log)   # WL v2 carries flight state

    def test_packet_log(self):
        dataset = self.assert_same_as_csv(self.packet_file())
        self.assertEqual(dataset.packet_types, ["command", "state"])
        self.assertIn("kalman_altitude", dataset.numeric_columns)

    def test_unknown_and_empty_files_fail_clearly(self):
        junk = self.write("X0000001.BIN", bytes(range(256)) * 40)
        with self.assertRaisesRegex(ValueError, "no records could be decoded"):
            viewer.load_dataset(junk)
        with self.assertRaisesRegex(ValueError, "empty"):
            viewer.load_dataset(self.write("I0000002.BIN", b""))

    def test_streaming_csv_writer_matches_write_csv(self):
        path = self.packet_file()
        data = path.read_bytes()
        rows, _ = converter.decode_packet_log(data)
        a, b = self.tmp / "a.csv", self.tmp / "b.csv"
        converter.write_csv(rows, a)
        count = converter.write_csv_stream(lambda: converter.iter_packet_log(data), b)
        self.assertEqual(count, len(rows))
        self.assertEqual(a.read_bytes(), b.read_bytes())


class PanelBinTests(BinFixtures, ViewerCase):
    def setUp(self):
        ViewerCase.setUp(self)
        BinFixtures.setUp(self)
        self.switch = sys.getswitchinterval()

    def tearDown(self):
        BinFixtures.tearDown(self)

    def test_open_bin_in_background_save_csv_and_errors(self):
        panel = self.make()
        imu = self.imu_file()
        junk = self.write("Z0000001.BIN", b"\x55" * 4000)
        panel.add_files([imu, junk])
        self.assertTrue(panel.loading)
        self.assertTrue(panel.wait_for_loads(60))
        self.assertEqual(sys.getswitchinterval(), self.switch)   # restored
        names = [d.label for d in panel.datasets.values()]
        self.assertEqual(names, ["I0000001.BIN"])
        errors = [m for m in panel.messages if m[0] == "error"]
        self.assertEqual(len(errors), 1)
        self.assertIn("Z0000001.BIN", errors[0][2])
        # Save the decoded log as CSV next to it (worker thread).
        target = imu.with_suffix(".csv")
        panel.save_bin_as_csv(imu, target)
        import qt_common
        self.assertTrue(qt_common.process_events_until(
            lambda: panel._save_worker is None, 30))
        self.assertIn("Wrote 3,000 rows", panel.last_save_message)
        data = imu.read_bytes()
        reference = self.tmp_path("ref.csv")
        converter.write_csv(converter.decode_imu_log(data)[0], reference)
        self.assertEqual(target.read_bytes(), reference.read_bytes())
        self.assertEqual(sys.getswitchinterval(), self.switch)
        self.assertTrue(panel.load_bar.isHidden())

    def tmp_path(self, name):
        return Path(self.tmp) / name


if __name__ == "__main__":
    unittest.main()
