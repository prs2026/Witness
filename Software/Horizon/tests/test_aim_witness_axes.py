"""AIM body-frame data (orientation, Z acceleration, derived gyro) shown in Witness axes
through the one shared "AIM axes..." alignment (aim_alignment.SharedAimAlignment)."""

import dataclasses
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from aim_alignment import (ACCEL_WITNESS_FIELDS, ALIGNED_Q_FIELDS, AimAlignment, SharedAimAlignment,
                           align_fields)
from aim_mqtt import AimMqttReceiver, AimSettings
from aim_orientation import GYRO_AIM_FIELDS, GYRO_FIELDS, to_witness_axes
import attitude_math as am
from flight_data import FlightData
from test_aim_orientation import gyro_layout, rotation, sample

PRESET = next(v for k, v in AimAlignment.PRESETS.items() if "2026-11-21" in k)   # ("+Z", "+X", 5.8)
ROLL = math.radians(5.8)
BASE = "entacore/aim/1234"
ACCEL_PLOT, AX, AY, AZ = 1, 3, 4, 5       # default layout: Acceleration graph, A X/Y/Z traces
GYRO_PLOT = 2                              # gyro_layout(): A gyro traces at 3, 4, 5


def preset(heading=0.0):
    return AimAlignment(*PRESET, heading)


def message(topic, fields, retain=False):
    return SimpleNamespace(topic=f"{BASE}/{topic}", payload=json.dumps(fields).encode(), qos=0, retain=retain)


def spin_samples(model, axis, start=1.0):
    """Two AIM orientations 0.1 s apart turning 9 deg about an AIM body axis (90 deg/s)."""
    model.accept_aim("A", "orientation_payload/v1", sample((1, 0, 0, 0), 0), start, False)
    model.accept_aim("A", "orientation_payload/v1", sample(rotation(axis, 9), .1), start + .1, False)


class BodyVectorsInWitnessAxes(unittest.TestCase):
    def test_preset_maps_aim_axes_onto_witness_axes(self):
        frame = preset().frame()
        # Witness x = AIM z (the nose; the roll is about it, so it stays exact) ...
        for got, want in zip(frame.body_to_witness((0, 0, 1)), (1, 0, 0)):
            self.assertAlmostEqual(got, want, places=12)
        # ... and Witness y = AIM x turned by the +5.8 deg roll about the nose.
        for got, want in zip(frame.body_to_witness((1, 0, 0)), (0, math.cos(ROLL), -math.sin(ROLL))):
            self.assertAlmostEqual(got, want, places=12)
        self.assertEqual(frame.aim_axis_in_witness((0, 0, 1)), (0, 1.0))

    def test_z_acceleration_lands_on_witness_x(self):
        fields = {"timestamp": 5, "z_acceleration": 9.81}
        out = align_fields("z_acceleration_payload/v1", fields, preset().frame())
        self.assertEqual(out["z_acceleration"], 9.81)               # raw kept
        self.assertEqual(out[ACCEL_WITNESS_FIELDS[0]], 9.81)
        self.assertNotIn(ACCEL_WITNESS_FIELDS[1], out)              # not measured by AIM
        self.assertNotIn(ACCEL_WITNESS_FIELDS[2], out)
        self.assertEqual(fields, {"timestamp": 5, "z_acceleration": 9.81})   # input untouched
        # A mapping that turns AIM +Z onto Witness -Y flips the sign.
        flipped = align_fields("z_acceleration_payload/v1", fields, AimAlignment("+X", "-Z").frame())
        self.assertEqual(flipped[ACCEL_WITNESS_FIELDS[1]], -9.81)
        # AIM +Z off every Witness axis (roll about another axis) cannot be placed.
        off = align_fields("z_acceleration_payload/v1", fields, AimAlignment("+X", "+Y", 5.8).frame())
        self.assertFalse(any(key in off for key in ACCEL_WITNESS_FIELDS))
        bad = align_fields("z_acceleration_payload/v1", {"z_acceleration": None}, preset().frame())
        self.assertIsNone(bad[ACCEL_WITNESS_FIELDS[0]])

    def test_derived_gyro_rotates_into_witness_axes(self):
        shared = SharedAimAlignment(preset(), path=None)
        model = FlightData(gyro_layout(), aim_alignment=shared)
        spin_samples(model, (0, 0, 1))                              # spin about AIM z = Witness nose
        gyro = model.snapshot()[GYRO_PLOT]
        self.assertAlmostEqual(gyro[3].latest, 90, places=6)
        self.assertAlmostEqual(gyro[4].latest, 0, places=6)
        self.assertAlmostEqual(gyro[5].latest, 0, places=6)
        model.clear()
        spin_samples(model, (1, 0, 0))                              # about AIM x = Witness y + roll
        gyro = model.snapshot()[GYRO_PLOT]
        self.assertAlmostEqual(gyro[3].latest, 0, places=6)
        self.assertAlmostEqual(gyro[4].latest, 90 * math.cos(ROLL), places=6)
        self.assertAlmostEqual(gyro[5].latest, -90 * math.sin(ROLL), places=6)

    def test_rotated_rate_equals_rate_of_the_aligned_orientation(self):
        frame = preset(heading=73.0).frame()
        rates = {key: v for key, v in zip(GYRO_FIELDS, (12.0, -40.0, 250.0))}
        out = to_witness_axes(rates, frame)
        self.assertEqual(tuple(out[key] for key in GYRO_AIM_FIELDS), (12.0, -40.0, 250.0))
        # Same as differencing the AIM rocket drawn in Witness axes (q_aim * M, any heading).
        from aim_orientation import rotation_rate
        q0 = am.from_tilt_azimuth_spin(20.0, 30.0, 10.0)
        omega = am.rotate(frame.mount, (12.0, -40.0, 250.0))        # AIM-axis rate of a Witness rate
        dt = 0.01
        angle = math.radians(math.hypot(*omega)) * dt
        q1 = am.multiply(q0, am.from_axis_angle(omega, angle))       # AIM body turn
        wxyz = lambda q: (q[3], q[0], q[1], q[2])                    # noqa: E731
        aim_rate = rotation_rate(wxyz(q0), wxyz(q1), dt)
        aligned_rate = rotation_rate(wxyz(frame.orientation(q0)), wxyz(frame.orientation(q1)), dt)
        for got, want in zip(frame.body_to_witness(aim_rate), aligned_rate):
            self.assertAlmostEqual(got, want, places=6)

    def test_identity_leaves_data_unchanged(self):
        frame = AimAlignment().frame()
        q = am.normalize((0.1, -0.2, 0.3, 0.9))
        out = align_fields("orientation_payload/v1", dict(zip("xyzw", q), timestamp=1), frame)
        for key, value in zip(ALIGNED_Q_FIELDS, q):
            self.assertAlmostEqual(out[key], value, places=12)
        accel = align_fields("z_acceleration_payload/v1", {"z_acceleration": 3.5}, frame)
        self.assertEqual(accel[ACCEL_WITNESS_FIELDS[2]], 3.5)
        rates = dict(zip(GYRO_FIELDS, (1.0, 2.0, 3.0)))
        self.assertEqual(tuple(to_witness_axes(rates, frame)[k] for k in GYRO_FIELDS), (1.0, 2.0, 3.0))
        # Scalars / world-frame topics pass through untouched.
        for topic, fields in (("gnss_payload/v1", {"gnss_coordinate.latitude": 1.5}),
                              ("pressure_payload/v1", {"pressure": 101325}),
                              ("temperature_payload/v1", {"temperature": 20})):
            self.assertIs(align_fields(topic, fields, preset().frame()), fields)

    def test_heading_does_not_affect_body_vectors(self):
        a, b = preset(0.0).frame(), preset(73.0).frame()
        for v in ((1, 0, 0), (0, 1, 0), (0.3, -2.0, 9.0)):
            self.assertEqual(a.body_to_witness(v), b.body_to_witness(v))
        fields = {"z_acceleration": 4.0}
        self.assertEqual(align_fields("z_acceleration_payload/v1", fields, a),
                         align_fields("z_acceleration_payload/v1", fields, b))
        rates = dict(zip(GYRO_FIELDS, (5.0, 6.0, 7.0)))
        self.assertEqual(to_witness_axes(rates, a), to_witness_axes(rates, b))
        # The orientation does take the heading (as the Visualizer draws it).
        q = dict(zip("xyzw", am.from_tilt_azimuth_spin(10, 20, 30)))
        qa = [align_fields("orientation_payload/v1", q, f) for f in (a, b)]
        self.assertGreater(am.angle_between_deg(tuple(qa[0][k] for k in ALIGNED_Q_FIELDS),
                                                tuple(qa[1][k] for k in ALIGNED_Q_FIELDS)), 1.0)
        self.assertLess(am.angle_between_deg(tuple(qa[1][k] for k in ALIGNED_Q_FIELDS),
                                             preset(73.0).apply(tuple(q[k] for k in "xyzw"))), 1e-9)


class SharedAlignmentState(unittest.TestCase):
    def test_snapshot_is_immutable_and_only_published_edits_count(self):
        shared = SharedAimAlignment(AimAlignment(), path=None)
        frame = shared.frame()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            frame.roll_deg = 3.0
        shared.alignment.witness_x, shared.alignment.witness_y = "+Z", "+X"   # GUI half-way through an edit
        self.assertIs(shared.frame(), frame)                                 # readers still see the old one
        shared.publish(save=False)
        self.assertEqual((shared.frame().witness_x, shared.frame().witness_y), ("+Z", "+X"))
        self.assertTrue(frame.identity)

    def test_publish_saves_and_reload_restores(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "aim_alignment.json"
            shared = SharedAimAlignment(path=path)
            self.assertTrue(shared.frame().identity)
            shared.set(preset(12.0))
            again = SharedAimAlignment(path=path)
            self.assertEqual((again.alignment.witness_x, again.alignment.witness_y, again.alignment.roll_deg,
                              again.alignment.heading_deg), (*PRESET, 12.0))

    def test_runtime_change_changes_the_shown_values(self):
        shared = SharedAimAlignment(AimAlignment(), path=None)
        model = FlightData(gyro_layout(), aim_alignment=shared)
        model.accept_aim("A", "z_acceleration_payload/v1", {"z_acceleration": 2.0}, 1.0, False)
        accel = model.snapshot()[ACCEL_PLOT]
        self.assertEqual(accel[AZ].latest, 2.0)          # identity: AIM Z is Witness Z
        self.assertIsNone(accel[AX].latest)
        shared.set(preset(), save=False)
        model.accept_aim("A", "z_acceleration_payload/v1", {"z_acceleration": 3.0}, 2.0, False)
        accel = model.snapshot()[ACCEL_PLOT]
        self.assertEqual(accel[AX].latest, 3.0)          # now AIM Z is Witness X
        self.assertEqual(accel[AZ].latest, 2.0)          # older Z sample only, not updated
        spin_samples(model, (0, 0, 1), start=3.0)
        self.assertAlmostEqual(model.snapshot()[GYRO_PLOT][3].latest, 90, places=6)
        shared.set(AimAlignment(), save=False)
        model.clear()                                    # fresh estimator baseline (new timestamps)
        spin_samples(model, (0, 0, 1), start=5.0)
        gyro = model.snapshot()[GYRO_PLOT]
        self.assertAlmostEqual(gyro[3].latest, 0, places=6)
        self.assertAlmostEqual(gyro[5].latest, 90, places=6)


class ReceiverKeepsRawLog(unittest.TestCase):
    def test_raw_log_stays_raw_and_shown_fields_are_aligned(self):
        shared = SharedAimAlignment(preset(30.0), path=None)
        seen = []
        with tempfile.TemporaryDirectory() as tmp:
            receiver = AimMqttReceiver(AimSettings("localhost", 1883, "124", BASE), Path(tmp),
                                       on_sample=lambda *args: seen.append(args), align=shared.align_fields)
            log_path = Path(tmp) / "aim_mqtt_test.jsonl"
            receiver._log = log_path.open("x", encoding="utf-8")
            q = am.from_tilt_azimuth_spin(5, 40, 10)
            raw_q = {"timestamp": 1, "x": q[0], "y": q[1], "z": q[2], "w": q[3]}
            receiver._on_message(None, None, message("orientation_payload/v1", raw_q))
            receiver._on_message(None, None, message("z_acceleration_payload/v1", {"timestamp": 2, "z_acceleration": 7.5}))
            receiver._log.close()
            receiver._log = None
            lines = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
        logged = [r["fields"] for r in lines if r.get("event") == "message"]
        self.assertEqual(logged, [raw_q, {"timestamp": 2, "z_acceleration": 7.5}])
        self.assertFalse(any(key.startswith("_derived") for fields in logged for key in fields))
        # What the app is handed (on_sample and the AIM tab's coalesced fields) carries both.
        (_, topic, fields, _, _), (_, _, accel, _, _) = seen
        self.assertEqual(topic, "orientation_payload/v1")
        self.assertEqual(fields["x"], raw_q["x"])
        self.assertLess(am.angle_between_deg(tuple(fields[k] for k in ALIGNED_Q_FIELDS), preset(30.0).apply(q)), 1e-9)
        self.assertEqual((accel["z_acceleration"], accel[ACCEL_WITNESS_FIELDS[0]]), (7.5, 7.5))
        updates, _stats, _diag = receiver.drain()
        self.assertEqual(updates["z_acceleration_payload/v1"][ACCEL_WITNESS_FIELDS[0]][0], 7.5)
        self.assertEqual(updates["z_acceleration_payload/v1"]["z_acceleration"][0], 7.5)


class HorizonSharesOneAlignment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from horizon_qt_support import app  # noqa: F401 - creates the QApplication

    def test_dialog_change_reaches_every_aim_view(self):
        from horizon_qt_support import make_horizon
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "aim_alignment.json"
            shared = SharedAimAlignment(path=path)
            app = make_horizon(tmp, aim_alignment=shared)
            try:
                panel = app.map_tab.orientation
                self.assertIs(panel.shared, shared)
                self.assertIs(app.flight.data.aim_alignment, shared)
                window = app.open_visualizer()
                self.assertIs(window.orientation.alignment, panel.alignment)
                app.flight.redraw()
                self.assertNotIn("Witness axes", app.flight.flight_panel.title.text())
                panel.show_alignment()
                dialog = panel._align_dialog
                dialog._preset(next(i for i in range(dialog.preset.count())
                                    if "2026-11-21" in dialog.preset.itemText(i)))
                self.assertEqual((shared.frame().witness_x, shared.frame().witness_y), PRESET[:2])
                self.assertTrue(path.exists())
                # AIM tab: a receiver made now aligns; its fields table shows both values.
                receiver = app.aim.make_receiver()
                receiver.on_sample = app._aim_sample
                receiver._on_message(None, None, message("z_acceleration_payload/v1",
                                                         {"timestamp": 1, "z_acceleration": 4.25}))
                updates, _stats, diagnostics = receiver.drain()
                app.aim.show_records(updates, diagnostics)
                self.assertEqual(app.aim.value("z_acceleration_payload/v1", "z_acceleration"), "4.25")
                self.assertEqual(app.aim.value("z_acceleration_payload/v1", ACCEL_WITNESS_FIELDS[0]), "4.25")
                app.aim._poll()
                self.assertIn("AIM in Witness axes", app.aim.axes_text.get())
                # Flight View: the Witness-X AIM trace has it, and the title says so.
                self.assertEqual(app.flight.data.snapshot()[ACCEL_PLOT][AX].latest, 4.25)
                app.flight.redraw()
                self.assertIn("A in Witness axes", app.flight.flight_panel.title.text())
                self.assertIn("x = AIM +Z", app.flight.display_details())
                # Reset in the dialog: back to as received everywhere.
                dialog._reset()
                self.assertTrue(shared.frame().identity)
                app.flight.redraw()
                self.assertNotIn("Witness axes", app.flight.flight_panel.title.text())
                dialog.close()
                window.close()
            finally:
                app.close()


if __name__ == "__main__":
    unittest.main()
