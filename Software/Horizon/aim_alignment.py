"""Rotate the AIM Xtra orientation into Witness's axes so the two can be compared.

AIM's quaternion maps AIM's body axes to AIM's world; Witness's maps its LSM6
body axes (+X along the nose) to local ENU with an arbitrary heading. The two
boards are mounted differently and AIM's heading reference is its own, so the
AIM rocket is drawn as

    q_aim_in_witness = Rz(heading) * q_aim * M,      M = Map * Rx(roll)

- ``Map`` is the axis mapping: which AIM body axis Witness +X and +Y point
  along (Witness +Z follows, right-handed). It takes Witness body vectors to
  AIM body vectors.
- ``roll`` is an extra turn about Witness +X (the nose), for a board clocked
  a few degrees off the AIM in the airframe.
- ``heading`` turns AIM's world about Up onto Witness's arbitrary heading.
  :meth:`AimAlignment.match_heading` sets it from one pair of samples.

Body-frame AIM vectors use the mount only (heading is a world turn):

    v_witness = conjugate(M) * v_aim * M     (attitude_math.rotate(conjugate(M), v))

so the same "AIM axes..." setting puts every AIM readout in Witness's axes,
not only the drawn rocket. AIM BASE 4.3.0 publishes these body-frame data:
orientation_payload/v1 (x, y, z, w), z_acceleration_payload/v1 (one component,
along AIM +Z) and, derived in Horizon, the quaternion-difference rate
(aim_orientation.py). GNSS, pressure, temperature, batteries and RSSI are
world-frame or scalars and are never rotated. :func:`align_fields` adds the
Witness-axis values BESIDE the raw ones (``_derived.q_witness[0..3]``,
``_derived.accel_witness[0..2]``); the raw MQTT JSONL log keeps what AIM sent.

:class:`SharedAimAlignment` is the app's single alignment: the GUI edits its
``alignment`` and calls :meth:`SharedAimAlignment.publish`; receiver threads read
:meth:`SharedAimAlignment.frame`, an immutable :class:`AimFrame` snapshot that is
swapped under a lock, so a half-edited alignment is never used.

Quaternions are (x, y, z, w), Hamilton, body -> world, as in attitude_math.
Pure Python (no Qt) so it is unit tested; saved as JSON beside this file.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import threading

import attitude_math as am

AXES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
_VECTORS = {"+X": (1.0, 0.0, 0.0), "-X": (-1.0, 0.0, 0.0), "+Y": (0.0, 1.0, 0.0),
            "-Y": (0.0, -1.0, 0.0), "+Z": (0.0, 0.0, 1.0), "-Z": (0.0, 0.0, -1.0)}
DEFAULT_PATH = Path(__file__).resolve().parent / "aim_alignment.json"


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def from_matrix(m):
    """Unit quaternion (x, y, z, w) of a proper rotation matrix (row-major)."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    return am.normalize(q)


def axis_map_quaternion(witness_x, witness_y):
    """Rotation taking Witness body vectors to AIM body vectors, or None if the
    two axes are not perpendicular (e.g. +X and -X)."""
    a, b = _VECTORS[witness_x], _VECTORS[witness_y]
    if abs(sum(i * j for i, j in zip(a, b))) > 1e-9:
        return None
    c = _cross(a, b)
    # Columns: images of Witness +X, +Y, +Z in AIM coordinates.
    return from_matrix(((a[0], b[0], c[0]), (a[1], b[1], c[1]), (a[2], b[2], c[2])))


def witness_z_name(witness_x, witness_y):
    a, b = _VECTORS[witness_x], _VECTORS[witness_y]
    c = _cross(a, b)
    for name, v in _VECTORS.items():
        if v == c:
            return name
    return "?"


@dataclass
class AimAlignment:
    witness_x: str = "+X"     # AIM body axis that Witness +X (the nose) points along
    witness_y: str = "+Y"     # AIM body axis that Witness +Y points along
    roll_deg: float = 0.0     # extra turn about Witness +X
    heading_deg: float = 0.0  # AIM world turned about Up onto Witness's heading

    PRESETS = {
        "None (AIM axes as received)": ("+X", "+Y", 0.0),
        # FLIGHT_ANALYSIS_2026-11-21 section 7.2: Witness x = AIM z, y = AIM x,
        # z = AIM y, roll +5.8 deg (from the coast roll and the pad gravity).
        "2026-11-21 flight mount (x=AIM z, y=AIM x, roll +5.8°)": ("+Z", "+X", 5.8),
    }

    def valid(self):
        return axis_map_quaternion(self.witness_x, self.witness_y) is not None

    def mount(self):
        mapping = axis_map_quaternion(self.witness_x, self.witness_y) or am.IDENTITY
        return am.multiply(mapping, am.from_axis_angle((1.0, 0.0, 0.0), math.radians(self.roll_deg)))

    def is_identity(self):
        return (self.witness_x, self.witness_y) == ("+X", "+Y") and self.roll_deg == 0.0 \
            and self.heading_deg == 0.0

    def apply(self, q_aim):
        """AIM quaternion expressed in Witness's body axes and heading (or None)."""
        if q_aim is None:
            return None
        q = am.multiply(am.normalize(q_aim), self.mount())
        return am.normalize(am.apply_heading(q, self.heading_deg))

    def match_heading(self, q_witness, q_aim):
        """Set heading_deg so the AIM rocket's heading equals Witness's now.
        Returns the new heading, or None if either sample is missing."""
        if q_witness is None or q_aim is None:
            return None
        without = AimAlignment(self.witness_x, self.witness_y, self.roll_deg, 0.0).apply(q_aim)
        self.heading_deg = am.wrap_deg(am.heading_offset_deg(without, q_witness))
        return self.heading_deg

    def body_to_witness(self, v_aim):
        """A body-frame AIM vector (accel, rate...) in Witness's body axes. Heading is
        a turn of the world, so it does not apply here."""
        return am.rotate(am.conjugate(self.mount()), v_aim)

    def frame(self):
        """Immutable snapshot of this alignment, safe to hand to another thread."""
        return AimFrame.of(self)

    def residual_deg(self, q_witness, q_aim):
        """Full rotation angle left between Witness and the aligned AIM."""
        if q_witness is None or q_aim is None:
            return None
        return am.angle_between_deg(am.normalize(q_witness), self.apply(q_aim))

    def describe(self):
        if self.is_identity():
            return "AIM axes as received"
        return (f"AIM in Witness axes: x = AIM {self.witness_x}, y = AIM {self.witness_y}, "
                f"z = AIM {witness_z_name(self.witness_x, self.witness_y)}, roll {self.roll_deg:+.1f}°, "
                f"heading {self.heading_deg:+.1f}°")

    # ------------------------------------------------------------ storage
    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        a = cls()
        if isinstance(data, dict):
            if data.get("witness_x") in AXES:
                a.witness_x = data["witness_x"]
            if data.get("witness_y") in AXES:
                a.witness_y = data["witness_y"]
            for key in ("roll_deg", "heading_deg"):
                try:
                    value = float(data.get(key, 0.0))
                except (TypeError, ValueError):
                    value = 0.0
                setattr(a, key, value if math.isfinite(value) else 0.0)
        if not a.valid():
            a.witness_x, a.witness_y = "+X", "+Y"
        return a

    @classmethod
    def load(cls, path=DEFAULT_PATH):
        try:
            return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
        except (OSError, ValueError):
            return cls()

    def save(self, path=DEFAULT_PATH):
        try:
            Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
            return True
        except OSError:
            return False


# ------------------------------------------------------------ shared, thread safe

# A component AIM measures along one of its axes (z_acceleration) is shown on a
# Witness axis only if that AIM axis lies within this angle of it (a roll about
# the same axis keeps it exact). Otherwise the component cannot be placed.
AXIS_TOLERANCE_DEG = 1.0
ALIGNED_Q_FIELDS = tuple(f"_derived.q_witness[{i}]" for i in range(4))       # x, y, z, w
ACCEL_WITNESS_FIELDS = tuple(f"_derived.accel_witness[{i}]" for i in range(3))
ORIENTATION_TOPIC = "orientation_payload/v1"
Z_ACCEL_TOPIC = "z_acceleration_payload/v1"


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class AimFrame:
    """Immutable alignment snapshot: precomputed mount and its inverse."""
    witness_x: str
    witness_y: str
    roll_deg: float
    heading_deg: float
    mount: tuple
    unmount: tuple
    identity: bool
    description: str

    @classmethod
    def of(cls, alignment):
        mount = am.normalize(alignment.mount())
        return cls(alignment.witness_x, alignment.witness_y, float(alignment.roll_deg),
                   float(alignment.heading_deg), mount, am.conjugate(mount), alignment.is_identity(),
                   alignment.describe())

    def body_to_witness(self, v_aim):
        return am.rotate(self.unmount, v_aim)

    def orientation(self, q_aim):
        """q_aim_in_witness = Rz(heading) * q_aim * M (as AimAlignment.apply)."""
        if q_aim is None:
            return None
        return am.normalize(am.apply_heading(am.multiply(am.normalize(q_aim), self.mount), self.heading_deg))

    def aim_axis_in_witness(self, aim_axis):
        """(index, sign) of the Witness axis an AIM body axis lies along, or None."""
        v = self.body_to_witness(aim_axis)
        index = max(range(3), key=lambda i: abs(v[i]))
        if abs(v[index]) < math.cos(math.radians(AXIS_TOLERANCE_DEG)):
            return None
        return index, 1.0 if v[index] > 0 else -1.0


IDENTITY_FRAME = AimFrame.of(AimAlignment())


def align_fields(topic, fields, frame):
    """Decoded AIM fields plus their Witness-axis versions (raw keys untouched).

    orientation_payload/v1: _derived.q_witness[0..3] = aligned (x, y, z, w), the
    heading included, or None when the raw quaternion is unusable.
    z_acceleration_payload/v1: AIM measures only along its +Z; the value goes to
    _derived.accel_witness[i] for the Witness axis i that AIM +Z lies along (sign
    applied). The other two Witness components are not measured and stay absent;
    if AIM +Z is not along a Witness axis nothing is added. Other topics are
    returned unchanged. Never mutates ``fields``.
    """
    if not isinstance(fields, dict):
        return fields
    frame = IDENTITY_FRAME if frame is None else frame
    if topic == ORIENTATION_TOPIC and any(key in fields for key in ("x", "y", "z", "w")):
        q = tuple(fields.get(key) for key in ("x", "y", "z", "w"))
        aligned = (None,) * 4
        if all(_number(v) and abs(v) <= 1e100 for v in q) and am.norm(q) > 1e-12:
            aligned = frame.orientation(q)
        return {**fields, **dict(zip(ALIGNED_Q_FIELDS, aligned))}
    if topic == Z_ACCEL_TOPIC and "z_acceleration" in fields:
        axis = frame.aim_axis_in_witness((0.0, 0.0, 1.0))
        if axis is None:
            return fields
        index, sign = axis
        value = fields["z_acceleration"]
        return {**fields, ACCEL_WITNESS_FIELDS[index]: sign * value if _number(value) else None}
    return fields


class SharedAimAlignment:
    """The one AIM alignment the whole app uses (Visualizer panels, AIM tab, Flight View).

    ``alignment`` is the editable AimAlignment, touched on the GUI thread only
    (the "AIM axes..." dialog edits it in place, then calls publish()). Any
    thread reads frame(): an immutable AimFrame swapped under a lock.
    """

    def __init__(self, alignment=None, path=DEFAULT_PATH):
        self.path = path
        if alignment is None:
            alignment = AimAlignment.load(path) if path is not None else AimAlignment()
        self.alignment = alignment
        self._lock = threading.Lock()
        self._frame = AimFrame.of(alignment)
        self.version = 0

    def frame(self):
        with self._lock:
            return self._frame

    def publish(self, save=True):
        """Make the edited alignment current for every reader (and save it)."""
        frame = AimFrame.of(self.alignment)
        with self._lock:
            self._frame = frame
            self.version += 1
        if save and self.path is not None:
            self.alignment.save(self.path)
        return frame

    def set(self, alignment, save=True):
        a = self.alignment
        a.witness_x, a.witness_y = alignment.witness_x, alignment.witness_y
        a.roll_deg, a.heading_deg = alignment.roll_deg, alignment.heading_deg
        return self.publish(save)

    def describe(self):
        return self.frame().description

    def align_fields(self, topic, fields):
        """AimMqttReceiver's ``align`` hook (MQTT thread)."""
        return align_fields(topic, fields, self.frame())
