import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hitl_protocol as hp  # noqa: E402
import fake_witness as fw  # noqa: E402


class CommandTests(unittest.TestCase):
    def test_enter_exit_bytes(self):
        self.assertEqual(hp.encode_command(hp.CMD_HITL_ENTER),
                         bytes.fromhex("05 00 00 00 00 00 00 48 49 0A"))
        self.assertEqual(hp.encode_command(hp.CMD_HITL_EXIT),
                         bytes.fromhex("05 00 00 00 00 00 00 48 58 0A"))

    def test_destructive_commands_refused(self):
        for cmd in (0x6868, 0x6869, 0x7301, 0x00E0, 0x0051):
            with self.assertRaises(ValueError):
                hp.encode_command(cmd)


class HitlFrameTests(unittest.TestCase):
    def test_exact_vector(self):
        s = hp.HitlInput(
            sim_time_ms=0x01020304, low_g=(1.0, -0.000488, 0.0),
            high_g=(10.417, 0.0, -0.010417), gyro_dps=(0.07, -7.0, 0.0),
            imu_temp_c=26.0, pressure_pa=85432, baro_temp_c=-1.5,
            lat_deg=32.9903, lon_deg=-106.975, gps_alt_m=1401.0,
            satellites=10, fix_quality=1, battery_mv=7800)
        f = hp.encode_hitl(s)
        expected = (
            "0C 0000 01020304"
            " 0801 FFFF 0000"          # 1 g = 2049.18 -> 2049 counts
            " 03E8 0000 FFFF"          # high-g 1000 counts
            " 0001 FF9C 0000"          # gyro 1, -100
            " 0100"                    # IMU temp +1 C -> 256
            " 014DB8"                  # 85432 Pa
            " FF6A"                    # -150 centi-C
            " 0508AF"                  # 329903
            " EFAD4A"                  # -1069750
            + " " + struct.pack(">f", 1401.0).hex() +
            " 0A 01 1E78 0A")
        self.assertEqual(f.hex(" ").upper().replace(" ", ""),
                         expected.replace(" ", "").upper())
        self.assertEqual(len(f), 47)
        d = hp.decode_hitl(f)
        self.assertEqual(d["sim_time_ms"], 0x01020304)
        self.assertAlmostEqual(d["lat_deg"], 32.9903, places=6)
        self.assertAlmostEqual(d["lon_deg"], -106.975, places=6)
        self.assertEqual(d["pressure_pa"], 85432)
        self.assertEqual(d["satellites"], 10)  # 0x0A inside payload
        self.assertAlmostEqual(d["baro_temp_c"], -1.5)
        self.assertAlmostEqual(d["imu_temp_c"], 26.0)

    def test_saturation(self):
        f = hp.encode_hitl(hp.HitlInput(low_g=(40.0, -40.0, 0.0),
                                        high_g=(500.0, -500.0, 0.0),
                                        gyro_dps=(5000.0, -5000.0, 0.0)))
        d = hp.decode_hitl(f)
        self.assertEqual(struct.unpack_from(">hh", f, 7), (32767, -32768))
        self.assertEqual(struct.unpack_from(">hh", f, 13), (32767, -32768))
        self.assertEqual(struct.unpack_from(">hh", f, 19), (32767, -32768))
        self.assertLess(d["low_g"][0], 16.0)

    def test_independent_parser_agrees(self):
        s = hp.HitlInput(sim_time_ms=12340, low_g=(-2.5, 0.1, 0.2),
                         pressure_pa=101325, lat_deg=-45.5, lon_deg=170.25,
                         gps_alt_m=-12.5, satellites=7, fix_quality=1,
                         battery_mv=8123)
        a = hp.decode_hitl(hp.encode_hitl(s))
        b = fw.parse_hitl(hp.encode_hitl(s))
        self.assertEqual(a["sim_time_ms"], b["sim_time_ms"])
        self.assertAlmostEqual(a["low_g"][0], b["low_g"][0])
        self.assertAlmostEqual(a["lat_deg"], b["lat"])
        self.assertAlmostEqual(a["lon_deg"], b["lon"])
        self.assertEqual(a["battery_mv"], b["batt"])


class DecoderTests(unittest.TestCase):
    def setUp(self):
        self.status = fw.frame_status(0x6E, 2, True, True)

    def test_state_packet_exact(self):
        f = fw.make_state(self.status, 0x0A0A0A0A, 32.9903, -106.975,
                          1500.5, 1499.25, 7800, 10)
        self.assertEqual(len(f), 65)
        self.assertEqual(f[63], 10)  # satellites byte equals EOF value
        ev = hp.StreamDecoder().feed(f)
        self.assertEqual(len(ev), 1)
        kind, p = ev[0]
        self.assertEqual(kind, "packet")
        self.assertEqual(p["name"], "state")
        self.assertEqual(p["uptime_ms"], 0x0A0A0A0A)
        self.assertEqual(p["status"]["flight_state"], 2)
        self.assertTrue(p["status"]["hitl_active"])
        self.assertTrue(p["status"]["gps_locked"])
        self.assertTrue(p["status"]["baro_ready"])
        self.assertFalse(p["status"]["nand_init_failed"])
        self.assertAlmostEqual(p["lat_deg"], 32.9903)
        self.assertAlmostEqual(p["lon_deg"], -106.975)
        self.assertAlmostEqual(p["gps_alt_m"], 1500.5)
        self.assertAlmostEqual(p["baro_alt_m"], 1499.25)
        self.assertEqual(p["battery_mv"], 7800)
        self.assertEqual(p["gps_satellites"], 10)

    def test_sensor_packet_vector(self):
        raw = bytes.fromhex("01 0210 0A0A0A0A 0061A8 FFFFD8 000000"
                            " 000A FFF6 0000 0A")
        (kind, p), = hp.StreamDecoder().feed(raw)
        self.assertAlmostEqual(p["accel_g"][0], 1.0)       # 25000 * 40 ug
        self.assertAlmostEqual(p["accel_g"][1], -0.0016)
        self.assertAlmostEqual(p["gyro_dps"][0], 0.7)
        self.assertAlmostEqual(p["gyro_dps"][1], -0.7)
        self.assertEqual(p["status"]["flight_state"], 0)
        self.assertTrue(p["status"]["hitl_active"])

    def test_heartbeat_vector(self):
        (kind, p), = hp.StreamDecoder().feed(
            bytes.fromhex("FF 6F 01 00 0A 0A 01 0A"))
        self.assertEqual(p["name"], "heartbeat")
        self.assertEqual(p["uptime_ms"], 0x000A0A01)
        self.assertTrue(p["status"]["heartbeat"])
        self.assertEqual(p["status"]["flight_state_name"], "Boost")

    def test_byte_by_byte_and_text_interleave(self):
        hb = fw.make_heartbeat(self.status, 0x0A0A0A0A)
        st = fw.make_state(self.status, 5, 0, 0, 0, 0, 0x0A0A, 0x0A)
        echo = hp.encode_hitl(hp.HitlInput(sim_time_ms=0x0A0A, satellites=10))
        cmd_echo = hp.encode_command(hp.CMD_HITL_ENTER)
        stream = (b"\x1b[0;32mI (12) hitl: entered\x1b[0m\r\n" + hb
                  + b"$GPGGA,1,2*00\r\n" + echo + b"\x00\x99" + st
                  + b"W (55) flight_" + hb + b"state: pad idle -> boost\r\n"
                  + cmd_echo)
        for chunk in (len(stream), 1, 3, 7):
            dec = hp.StreamDecoder()
            events = []
            for i in range(0, len(stream), chunk):
                events += dec.feed(stream[i:i + chunk])
            texts = [v for k, v in events if k == "text"]
            pkts = [v for k, v in events if k == "packet"]
            self.assertEqual(texts, ["I (12) hitl: entered", "$GPGGA,1,2*00",
                                     "W (55) flight_state: pad idle -> boost"],
                             f"chunk {chunk}")
            self.assertEqual([p["name"] for p in pkts],
                             ["heartbeat", "hitl_echo", "state", "heartbeat",
                              "command"])
            self.assertTrue(pkts[1]["echo"])
            self.assertTrue(pkts[4]["probable_echo"])
            self.assertEqual(pkts[2]["battery_mv"], 0x0A0A)
            self.assertEqual(dec.stats.echo_frames, 1)
            self.assertEqual(dec.stats.junk_bytes, 2)

    def test_resync_after_corruption(self):
        hb = fw.make_heartbeat(self.status, 1234)
        bad = bytearray(hb)
        bad[-1] = 0x00  # broken EOF
        events = hp.StreamDecoder().feed(bytes(bad) + hb)
        pkts = [v for k, v in events if k == "packet"]
        self.assertEqual(len(pkts), 1)
        self.assertEqual(pkts[0]["uptime_ms"], 1234)

    def test_reserved_status_bits_rejected(self):
        frame = bytes.fromhex("FF 00 E0 00 00 00 01 0A")
        self.assertEqual(
            [k for k, _ in hp.StreamDecoder().feed(frame)], [])

    def test_partial_frame_waits(self):
        hb = fw.make_heartbeat(self.status, 99)
        dec = hp.StreamDecoder()
        self.assertEqual(dec.feed(hb[:5]), [])
        self.assertEqual(len(dec.feed(hb[5:])), 1)

    def test_state_log_parse(self):
        self.assertEqual(hp.parse_state_log(
            "I (123) flight_state: coast -> descent"), (2, 3))
        self.assertIsNone(hp.parse_state_log("I (1) flight_state: launch"))

    def test_pressure_altitude(self):
        self.assertAlmostEqual(hp.pressure_altitude_m(101325), 0.0, places=3)
        self.assertAlmostEqual(hp.pressure_altitude_m(89874.6), 1000, delta=1)


if __name__ == "__main__":
    unittest.main()
