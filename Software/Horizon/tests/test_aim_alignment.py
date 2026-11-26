"""AIM-to-Witness axis alignment (aim_alignment) and the Witness gyro-carried attitude flag."""

import math
from pathlib import Path
import struct
import tempfile
import time
import unittest

from aim_alignment import AXES, AimAlignment, axis_map_quaternion, witness_z_name
import attitude_math as am
from attitude_data import LiveAimAttitude, LiveAttitude, is_carried, make_sample
from attitude_scene import attitude_condition


def aim_reading(q_witness, mapping, roll_deg, heading_deg):
    """What an AIM mounted per (mapping, roll) with its own heading would report."""
    m = am.multiply(axis_map_quaternion(*mapping), am.from_axis_angle((1, 0, 0), math.radians(roll_deg)))
    h = am.from_axis_angle((0, 0, 1), math.radians(-heading_deg))
    return am.normalize(am.multiply(am.multiply(h, q_witness), am.conjugate(m)))


class AlignmentMath(unittest.TestCase):
    def test_identity_leaves_aim_unchanged(self):
        q = am.from_tilt_azimuth_spin(20.0, 40.0, 10.0)
        a = AimAlignment()
        self.assertTrue(a.is_identity())
        self.assertLess(am.angle_between_deg(a.apply(q), q), 1e-6)

    def test_every_perpendicular_mapping_is_a_proper_rotation(self):
        count = 0
        for x in AXES:
            for y in AXES:
                q = axis_map_quaternion(x, y)
                if q is None:
                    continue
                count += 1
                self.assertAlmostEqual(am.norm(q), 1.0, places=9)
                for axis, name in (((1, 0, 0), x), ((0, 1, 0), y), ((0, 0, 1), witness_z_name(x, y))):
                    v = am.rotate(q, axis)
                    sign = 1.0 if name[0] == "+" else -1.0
                    index = "XYZ".index(name[1])
                    self.assertAlmostEqual(v[index], sign, places=9)
        self.assertEqual(count, 24)
        self.assertIsNone(axis_map_quaternion("+X", "-X"))

    def test_mapping_roll_and_heading_recover_the_witness_attitude(self):
        mapping, roll, heading = ("+Z", "+X"), 5.8, 73.0
        q_pad = am.from_tilt_azimuth_spin(3.0, 120.0, 30.0)
        a = AimAlignment(*mapping, roll, 0.0)
        result = a.match_heading(q_pad, aim_reading(q_pad, mapping, roll, heading))
        self.assertAlmostEqual(am.wrap_deg(result - heading), 0.0, places=6)
        # Then it holds for any later attitude (body mount and world heading separate).
        for tilt, az, spin in ((45.0, 10.0, 200.0), (120.0, 250.0, -40.0), (170.0, 5.0, 90.0)):
            q = am.from_tilt_azimuth_spin(tilt, az, spin)
            self.assertLess(a.residual_deg(q, aim_reading(q, mapping, roll, heading)), 1e-6)
        # A wrong roll leaves a residual of about that roll.
        wrong = AimAlignment(*mapping, 0.0, heading)
        q = am.from_tilt_azimuth_spin(45.0, 10.0, 200.0)
        self.assertAlmostEqual(wrong.residual_deg(q, aim_reading(q, mapping, roll, heading)), roll, delta=0.2)

    def test_match_needs_both_samples(self):
        a = AimAlignment()
        self.assertIsNone(a.match_heading(None, am.IDENTITY))
        self.assertIsNone(a.match_heading(am.IDENTITY, None))
        self.assertEqual(a.heading_deg, 0.0)

    def test_save_load_and_bad_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "aim_alignment.json"
            AimAlignment("-Y", "+Z", 4.5, -30.0).save(path)
            b = AimAlignment.load(path)
            self.assertEqual((b.witness_x, b.witness_y, b.roll_deg, b.heading_deg), ("-Y", "+Z", 4.5, -30.0))
            path.write_text('{"witness_x": "+X", "witness_y": "-X", "roll_deg": "nan"}', encoding="utf-8")
            c = AimAlignment.load(path)
            self.assertEqual((c.witness_x, c.witness_y, c.roll_deg), ("+X", "+Y", 0.0))
            self.assertTrue(AimAlignment.load(Path(tmp) / "missing.json").is_identity())

    def test_presets_are_valid(self):
        for x, y, _roll in AimAlignment.PRESETS.values():
            self.assertIsNotNone(axis_map_quaternion(x, y))


class CarriedFlag(unittest.TestCase):
    def test_sign_of_w_is_the_flag(self):
        q = am.from_tilt_azimuth_spin(30.0, 0.0, 0.0)
        normal = make_sample(1.0, q, "0x02")
        carried = make_sample(1.0, tuple(-c for c in q), "0x02")
        self.assertFalse(normal.carried)
        self.assertTrue(carried.carried)
        self.assertLess(am.angle_between_deg(normal.q, carried.q), 1e-6)   # same rotation
        self.assertTrue(is_carried((1.0, 0.0, 0.0, -0.0)))                 # negative zero
        self.assertFalse(is_carried((1.0, 0.0, 0.0, 0.0)))
        self.assertFalse(make_sample(1.0, (0.0, 0.0, 0.0, -0.0), "0x02").carried)   # not available
        self.assertEqual(attitude_condition(carried, None, 3)[0], "carried")
        self.assertEqual(attitude_condition(normal, None, 3)[0], "ok")

    def test_flag_survives_the_packet_decoder(self):
        from converter import parse_packet
        q = (1.0, 0.0, 0.0, -0.0)
        payload = (struct.pack(">HI", 3 << 8, 1000) + struct.pack(">4f", *q) + bytes(24)
                   + bytes(6) + struct.pack(">ff", 0, 0) + struct.pack(">H", 3900) + bytes([7]))
        packet = parse_packet(0x02, payload)
        self.assertTrue(is_carried(tuple(packet["orientation"])))


class AlignmentGui(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from horizon_qt_support import app  # noqa: F401 - creates the QApplication

    def test_panel_draws_aligned_aim_and_dialog_matches_heading(self):
        from visualizer import OrientationPanel
        from test_visualizer import state_frame_record
        mapping, roll, heading = ("+Z", "+X"), 5.8, 73.0
        q_w = am.from_tilt_azimuth_spin(2.0, 0.0, 0.0)
        live, aim = LiveAttitude(), LiveAimAttitude()
        now = time.monotonic()
        live.accept_record(state_frame_record(q_w, t=now))
        x, y, z, w = aim_reading(q_w, mapping, roll, heading)
        aim.accept_aim("AIM", "orientation_payload/v1", {"timestamp": 1, "x": x, "y": y, "z": z, "w": w}, now)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "aim_alignment.json"
            panel = OrientationPanel(live=live, aim=aim, use_gl=False, alignment=AimAlignment(),
                                     alignment_path=path)
            panel.refresh(now)
            self.assertGreater(am.angle_between_deg(panel.aim_shown, q_w), 30.0)
            panel.show_alignment()
            dialog = panel._align_dialog
            preset = next(i for i in range(dialog.preset.count()) if "2026-11-21" in dialog.preset.itemText(i))
            dialog._preset(preset)
            self.assertEqual((panel.alignment.witness_x, panel.alignment.witness_y), mapping)
            self.assertIsNotNone(panel.match_heading_now(now))
            panel.refresh(now)
            self.assertLess(am.angle_between_deg(panel.aim_shown, q_w), 0.01)
            self.assertIn("AIM in Witness axes", panel.values["aim_axes"].get())
            self.assertTrue(path.exists())
            self.assertAlmostEqual(AimAlignment.load(path).heading_deg, panel.alignment.heading_deg)
            # An impossible mapping is refused and not applied.
            dialog.y_axis.setCurrentText("+Z")
            self.assertEqual(panel.alignment.witness_y, "+X")
            dialog.close()
            panel.close()


if __name__ == "__main__":
    unittest.main()
