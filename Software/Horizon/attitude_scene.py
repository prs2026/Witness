"""Rocket geometry, camera presets and attitude readout text (no GUI).

Shared by the Attitude tab and the Visualizer's orientation view
(attitude_view.RocketScene renders it with pyqtgraph.opengl, or as a 2D
projection when OpenGL is unavailable). Conventions: attitude_math.py.
"""

from __future__ import annotations

import math

import numpy as np

import attitude_math as am

BG = "#ffffff"
TEXT = "#000000"
MUTED = "#555555"
GRID = "#d0d0d0"
BAD = "#990000"
WARN = "#704000"
OK = "#0a5a0a"
ROCKET_NEAR = "#1a1a1a"
ROCKET_FAR = "#8a8a8a"
MARK = "#c04000"
GHOST = "#2f6fb0"
AIM_COLOR = "#8e24aa"          # AIM Xtra overlay rocket (drawn translucent)
AIM_ALPHA = 0.35
AIM_STALE_ALPHA = 0.12         # AIM overlay when its sample is stale
STALE_ALPHA = 0.3              # Witness rocket when its sample is stale
AXIS_COLORS = ("#b00020", "#007a33", "#0040c0")   # body x, y, z
Q_COLORS = ("#b00020", "#007a33", "#0040c0", "#6a3d9a")
FIN_FILL = {"fin": "#c8c8c8", "mark": "#f2a36b"}
GROUND = -0.75
IN_FLIGHT = (1, 2, 3)
VIEWS = {  # compass azimuth / elevation of the camera position
    "Iso": (135.0, math.degrees(math.asin(1 / math.sqrt(3)))),
    "From East": (90.0, 0.0),
    "From North": (0.0, 0.0),
    "From Above": (0.0, 90.0),
}

CONVENTIONS = (
    "Quaternion: Hamilton, x,y,z,w (scalar last), rotates BODY into WORLD. "
    "WORLD = ENU (+X east, +Y north, +Z up). BODY +X = nose; upright on the pad "
    "body +X points up. 0,0,0,0 = attitude not available.\n\n"
    "Tilt from vertical: angle between the nose and world up.\n"
    "Nose azimuth: compass direction of the nose, 0 = north, 90 = east "
    "(undefined when vertical).\n"
    "Spin about nose: twist about the nose, right-handed, 0 for the pad's "
    "shortest-arc alignment (body +Y north, +Z west).\n"
    "ZYX yaw/pitch/roll: intrinsic Z-Y'-X'' of body relative to ENU, each "
    "right-handed. Yaw from east toward north; pitch > 0 = nose down; upright = "
    "pitch -90 (gimbal lock, roll folded into yaw).\n\n"
    "Heading is unobservable on Witness (no magnetometer): azimuth, spin and yaw "
    "include an arbitrary heading set at initialisation. Truth comparison removes "
    "a constant heading offset (auto at the first valid pair, or Align heading here).\n\n"
    "Orange fin and stripe = body +Y. Body triad xb/yb/zb. Truth = blue dashed ghost."
)


def rocket_segments():
    """Wireframe in BODY coordinates (+X nose).

    Returns ``(segments, fins)``: segments are ``(p1, p2, role)`` lines and fins
    are ``(corner_points, role)`` polygons.
    """
    segments, fins = [], []
    radius, tail, shoulder, tip = 0.07, -0.6, 0.32, 0.65
    n = 12
    ring = [(math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n)) for i in range(n)]
    for x in (tail, shoulder):
        for (c1, s1), (c2, s2) in zip(ring, ring[1:] + ring[:1]):
            segments.append(((x, radius * c1, radius * s1), (x, radius * c2, radius * s2), "body"))
    for c, s in ring[1::3]:
        segments.append(((tail, radius * c, radius * s), (shoulder, radius * c, radius * s), "body"))
    for c, s in ring[::2]:
        segments.append(((shoulder, radius * c, radius * s), (tip, 0.0, 0.0), "nose"))
    # Four fins on body +Y, +Z, -Y, -Z. The +Y fin and a +Y stripe are marked
    # so roll is visible.
    for index, (dy, dz) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
        role = "mark" if index == 0 else "fin"
        span = 0.2
        outline = [(tail, radius), (tail + 0.26, radius), (tail + 0.1, radius + span), (tail - 0.03, radius + span)]
        fins.append(([(x, dy * r, dz * r) for x, r in outline], role))
    segments.append(((tail, radius, 0.0), (shoulder, radius, 0.0), "mark"))
    return segments, fins


ROCKET, FINS = rocket_segments()
_SEG = np.array([[a, b] for a, b, _role in ROCKET], dtype=float)          # (n, 2, 3)
_ROLES = np.array([role for _a, _b, role in ROCKET])
_FINS = np.array([points for points, _role in FINS], dtype=float)        # (4, 4, 3)
FIN_ROLES = [role for _points, role in FINS]
BODY_AXES = ((1.0, 0.0, 0.0, 0.85), (0.0, 1.0, 0.0, 0.45), (0.0, 0.0, 1.0, 0.45))


def rotation_matrix(q):
    return np.asarray(am.to_matrix(q), dtype=float).reshape(3, 3)


def rocket_world(q):
    """(segments[n,2,3], roles[n], fins[4,4,3]) rotated into world ENU."""
    m = rotation_matrix(q)
    return _SEG @ m.T, _ROLES, _FINS @ m.T


def body_axes_world(q):
    m = rotation_matrix(q)
    return [(m @ np.array(axis[:3]) * axis[3]) for axis in BODY_AXES]


def ground_grid(ground=GROUND):
    """Line segment pairs (n, 2, 3) of the 0.25-spaced ground grid."""
    lines = []
    for v in [i * 0.25 - 1.0 for i in range(9)]:
        lines.append(((v, -1, ground), (v, 1, ground)))
        lines.append(((-1, v, ground), (1, v, ground)))
    return np.array(lines, dtype=float)


def projector(view_az, view_el):
    """Return project(points[..., 3]) -> (x[...], y[...], depth[...]) for a camera."""
    basis = np.asarray(am.camera_basis(view_az, view_el), dtype=float)
    right, up, toward = basis[0], basis[1], basis[2]

    def project(points):
        points = np.asarray(points, dtype=float)
        return points @ right, points @ up, points @ toward
    return project


def fmt(value, digits=1, unit="°"):
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}{unit}"


def attitude_condition(sample, last_valid, flight_state, hitl=None):
    """(code, title, detail) for the current attitude sample.

    code: none (no data), ok, lost (zero quaternion in flight after a valid
    one: gyro saturation), unavailable (zero in flight, never valid here),
    init (zero on the pad: waiting for the stationary window).
    """
    if sample is None:
        return "none", "NO ATTITUDE DATA", "waiting for Witness 0x02 / 0xE0"
    if sample.q is not None:
        return "ok", "", ""
    if flight_state in IN_FLIGHT or flight_state == 4:
        if last_valid is not None:
            return ("lost", "ATTITUDE LOST",
                    "Gyro saturated in flight (>= 1950 dps): Witness stops its attitude for the rest "
                    "of the flight and its Kalman filter ignores the IMU. Last valid attitude dashed.")
        return ("unavailable", "ATTITUDE NOT AVAILABLE",
                "Zero quaternion in flight: no stationary window before launch, or lost "
                "before Horizon received a valid attitude.")
    detail = f"Waiting for a 1 s stationary window on the pad ({sample.reason})."
    if hitl == 1:
        detail += " HITL entry/exit resets the estimator."
    return "init", "ATTITUDE NOT INITIALISED", detail


def chart_series(samples, t0, t1, max_gap, *, max_points=2000, truth=False):
    """Strip-chart arrays: (t, q[n,4], tilt[n], truth_q[n,4] | None, truth_tilt | None) with NaN gaps."""
    visible = [s for s in samples if t0 <= s.time <= t1]
    stride = max(1, len(visible) // max(1, max_points))
    visible = visible[::stride]
    gap = max_gap * stride
    times, qs, tilts, tq, tt = [], [], [], [], []
    nan4 = (math.nan,) * 4
    last = None
    for s in visible:
        if last is not None and s.time - last > gap:
            times.append((s.time + last) / 2)
            qs.append(nan4)
            tilts.append(math.nan)
            tq.append(nan4)
            tt.append(math.nan)
        last = s.time
        times.append(s.time)
        qs.append(s.q if s.q is not None else nan4)
        tilts.append(am.tilt_deg(s.q) if s.q is not None else math.nan)
        tq.append(s.truth if s.truth is not None else nan4)
        tt.append(am.tilt_deg(s.truth) if s.truth is not None else math.nan)
    t = np.asarray(times, dtype=float)
    out = (t, np.asarray(qs, dtype=float).reshape(-1, 4), np.asarray(tilts, dtype=float))
    if truth:
        return out + (np.asarray(tq, dtype=float).reshape(-1, 4), np.asarray(tt, dtype=float))
    return out + (None, None)
