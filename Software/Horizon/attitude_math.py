"""Pure quaternion/attitude math for the Horizon Attitude tab. No GUI imports.

Conventions (match comms.h "Orientation quaternion"):

* Quaternions are tuples ``(x, y, z, w)`` -- scalar LAST, the wire order.
* Hamilton product. ``q`` rotates BODY vectors into the WORLD frame:
  ``v_world = q * v_body * q^-1`` (see :func:`rotate`).
* WORLD is local ENU: +X east, +Y north, +Z up.
* BODY +X is the rocket nose axis. An upright rocket on the pad has body +X
  pointing to world +Z; :data:`Q_UPRIGHT` is that attitude with zero roll
  (a -90 degree rotation about world +Y, which is also what the firmware's
  shortest-arc tilt alignment produces for a perfectly vertical rocket).
* All four components zero means "attitude not available". Such samples (and
  non-finite ones) are reported as invalid by :func:`checked`; never draw them.

Angle sets reported by :func:`attitude_angles` (all degrees):

* ``tilt``: angle between body +X (nose) and world +Z, 0 = vertical, 180 = nose
  straight down. Independent of heading.
* ``azimuth``: compass direction of the nose's horizontal projection, 0 = north,
  90 = east (clockwise from north). Undefined for a vertical nose.
* ``spin``: roll about the nose, pad-referenced. ``q * Q_UPRIGHT^-1`` is split
  into a horizontal-axis "swing" (the tilt) applied after a twist about world
  +Z; ``spin`` is that twist angle, right-handed about the nose. It is 0 when
  ``q == Q_UPRIGHT`` and is well defined everywhere except nose straight down.
  On Witness, heading is unobservable, so spin (and azimuth) include the
  arbitrary initial heading.
* ``yaw``/``pitch``/``roll``: classic intrinsic Z-Y'-X'' (yaw-pitch-roll) of
  body relative to ENU, ``R = Rz(yaw) Ry(pitch) Rx(roll)``, each right-handed.
  Yaw is measured from east toward north; pitch > 0 is nose DOWN (right-hand
  about +Y); the upright pad attitude is pitch = -90 degrees, which is this
  sequence's gimbal lock, so yaw/roll are degenerate near vertical (``gimbal``
  is True there and roll is folded into yaw).
"""

import math

EPS = 1e-9
IDENTITY = (0.0, 0.0, 0.0, 1.0)
_S = math.sqrt(0.5)
#: Upright on the pad, zero spin: body +X -> world +Z, +Y -> north, +Z -> west.
Q_UPRIGHT = (0.0, -_S, 0.0, _S)
NORM_TOLERANCE = 0.01


def norm(q):
    return math.sqrt(sum(c * c for c in q))


def checked(q):
    """Return ``(unit_quaternion_or_None, raw_norm_or_None, reason)``.

    ``reason`` is "" for a usable sample, otherwise a short explanation:
    "not available" (all zero), "missing" (None/short), "non-finite", or
    "degenerate" (norm too small to normalise). Usable samples with
    ``abs(norm - 1) > NORM_TOLERANCE`` are still returned (normalised); callers
    should flag them with :func:`norm_bad`.
    """
    if q is None:
        return None, None, "missing"
    try:
        values = tuple(float(c) for c in q)
    except (TypeError, ValueError):
        return None, None, "missing"
    if len(values) != 4:
        return None, None, "missing"
    if not all(math.isfinite(c) for c in values):
        return None, None, "non-finite"
    if all(c == 0.0 for c in values):
        return None, 0.0, "not available"
    n = norm(values)
    if n < 1e-6:
        return None, n, "degenerate"
    return tuple(c / n for c in values), n, ""


def norm_bad(raw_norm, tolerance=NORM_TOLERANCE):
    return raw_norm is not None and abs(raw_norm - 1.0) > tolerance


def normalize(q):
    n = norm(q)
    if n < EPS:
        raise ValueError("cannot normalise a zero quaternion")
    return tuple(c / n for c in q)


def conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def multiply(a, b):
    """Hamilton product ``a * b`` (apply b first, then a, when rotating)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def from_axis_angle(axis, angle_rad):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < EPS:
        return IDENTITY
    s = math.sin(angle_rad / 2.0) / n
    return (ax * s, ay * s, az * s, math.cos(angle_rad / 2.0))


def rotate(q, v):
    """Rotate vector ``v`` by unit quaternion ``q``: body -> world."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def to_matrix(q):
    """3x3 rotation matrix (row-major); columns are body axes in world."""
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def angle_between_deg(a, b):
    """Rotation angle (0..180 deg) taking unit quaternion a to b."""
    d = min(1.0, abs(dot(a, b)))
    return math.degrees(2.0 * math.acos(d))


def wrap_deg(angle):
    """Wrap to (-180, 180]."""
    angle = math.fmod(angle, 360.0)
    if angle <= -180.0:
        angle += 360.0
    elif angle > 180.0:
        angle -= 360.0
    return angle


# ---------------------------------------------------------------- Euler sets

def to_euler_zyx(q):
    """Return ``(yaw, pitch, roll, gimbal)`` in degrees; see module docstring."""
    x, y, z, w = q
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0 - 1e-6:
        # Gimbal lock: only yaw +/- roll is defined. Put it all in yaw.
        pitch = math.copysign(90.0, sinp)
        yaw = math.degrees(-2.0 * math.copysign(1.0, sinp) * math.atan2(x, w))
        return wrap_deg(yaw), pitch, 0.0, True
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    pitch = math.degrees(math.asin(sinp))
    roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    return yaw, pitch, roll, abs(pitch) > 89.0


def from_euler_zyx(yaw_deg, pitch_deg, roll_deg):
    qz = from_axis_angle((0, 0, 1), math.radians(yaw_deg))
    qy = from_axis_angle((0, 1, 0), math.radians(pitch_deg))
    qx = from_axis_angle((1, 0, 0), math.radians(roll_deg))
    return multiply(multiply(qz, qy), qx)


def nose(q):
    return rotate(q, (1.0, 0.0, 0.0))


def tilt_deg(q):
    """Angle between body +X (nose) and world +Z."""
    return math.degrees(math.acos(max(-1.0, min(1.0, nose(q)[2]))))


def azimuth_deg(q):
    """Compass azimuth (0 = north, 90 = east) of the nose; None if vertical."""
    e, n, _ = nose(q)
    if math.hypot(e, n) < 1e-9:
        return None
    azimuth = math.degrees(math.atan2(e, n)) % 360.0
    return 0.0 if azimuth > 360.0 - 1e-9 else azimuth


def twist_about_z(q):
    """Twist angle (deg, (-180,180]) of q about world +Z, swing-twist split
    ``q = swing * twist`` with twist about +Z. None if undefined."""
    x, y, z, w = q
    if math.hypot(z, w) < 1e-9:
        return None
    return wrap_deg(math.degrees(2.0 * math.atan2(z, w)))


def spin_deg(q):
    """Roll about the nose relative to :data:`Q_UPRIGHT`; see module docstring."""
    return twist_about_z(multiply(q, conjugate(Q_UPRIGHT)))


def from_tilt_azimuth_spin(tilt, azimuth, spin):
    """Inverse of (tilt, azimuth, spin): useful for tests and synthetic data."""
    # Tilt toward the azimuth: rotate +Z toward the horizontal unit vector h
    # about the horizontal axis Z x h.
    az = math.radians(azimuth)
    h = (math.sin(az), math.cos(az), 0.0)
    axis = (-h[1], h[0], 0.0)  # Z x h
    swing = from_axis_angle(axis, math.radians(tilt))
    twist = from_axis_angle((0, 0, 1), math.radians(spin))
    return multiply(multiply(swing, twist), Q_UPRIGHT)


def attitude_angles(q):
    """All derived angles for a unit quaternion as a dict (degrees)."""
    yaw, pitch, roll, gimbal = to_euler_zyx(q)
    tilt = tilt_deg(q)
    return {"tilt": tilt, "azimuth": azimuth_deg(q), "spin": spin_deg(q),
            "yaw": yaw, "pitch": pitch, "roll": roll, "gimbal": gimbal,
            "elevation": 90.0 - tilt}


# ------------------------------------------------------- heading alignment

def heading_offset_deg(q_estimate, q_truth):
    """World-Z rotation (deg) that best maps the estimate onto truth.

    Witness heading is arbitrary, so ``q_truth ~= Rz(offset) * q_estimate``.
    The optimum is the twist about +Z of ``q_truth * q_estimate^-1``.
    Returns 0.0 if undefined.
    """
    offset = twist_about_z(multiply(q_truth, conjugate(q_estimate)))
    return 0.0 if offset is None else offset


def apply_heading(q, offset_deg):
    return multiply(from_axis_angle((0, 0, 1), math.radians(offset_deg)), q)


def attitude_errors(q_estimate, q_truth, offset_deg=0.0):
    """Return ``(tilt_error_deg, attitude_error_deg)``.

    Tilt error is estimate minus truth (heading independent). Attitude error
    is the full rotation angle between truth and the heading-aligned estimate.
    """
    aligned = apply_heading(q_estimate, offset_deg)
    return tilt_deg(q_estimate) - tilt_deg(q_truth), angle_between_deg(aligned, q_truth)


# ------------------------------------------------------------- interpolation

def slerp(a, b, t):
    """Shortest-path spherical interpolation between unit quaternions."""
    d = dot(a, b)
    if d < 0.0:
        b = tuple(-c for c in b)
        d = -d
    if d > 0.9995:
        return normalize(tuple(x + t * (y - x) for x, y in zip(a, b)))
    theta = math.acos(d)
    s = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / s
    wb = math.sin(t * theta) / s
    return tuple(wa * x + wb * y for x, y in zip(a, b))


# --------------------------------------------------------------- projection

def camera_basis(azimuth_deg_, elevation_deg):
    """Orthographic camera looking at the origin from compass azimuth /
    elevation. Returns ``(right, up, toward_camera)`` world unit vectors."""
    az = math.radians(azimuth_deg_)
    el = math.radians(elevation_deg)
    c = (math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el))
    if math.cos(el) < 1e-6:
        # Looking straight down/up: keep north at the top of the screen.
        right = (math.cos(az), -math.sin(az), 0.0) if el > 0 else (math.cos(az), math.sin(az), 0.0)
    else:
        rx, ry = -c[1], c[0]          # Z x c
        n = math.hypot(rx, ry)
        right = (rx / n, ry / n, 0.0)
    up = (c[1] * right[2] - c[2] * right[1],   # c x right
          c[2] * right[0] - c[0] * right[2],
          c[0] * right[1] - c[1] * right[0])
    return right, up, c


def project(point, basis):
    """Return ``(screen_right, screen_up, depth_toward_camera)``."""
    right, up, c = basis
    return (sum(p * r for p, r in zip(point, right)),
            sum(p * u for p, u in zip(point, up)),
            sum(p * k for p, k in zip(point, c)))
