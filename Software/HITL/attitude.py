"""Quaternion helpers and the truth-vs-Witness attitude comparison.

Conventions (match Witness firmware and comms.h "Orientation quaternion"):

* WORLD is local ENU (+X east, +Y north, +Z up).
* BODY is the LSM6 sensor frame, +X along the rocket nose axis.
* Quaternions are Hamilton, rotating BODY vectors into WORLD
  (v_world = q * v_body * q^-1), stored as tuples ``(x, y, z, w)`` (scalar
  last, the wire order).
* Heading (rotation about world +Z) is unobservable on Witness, so the full
  attitude error is evaluated after removing a constant heading offset.
"""

from __future__ import annotations

import math

EX = (1.0, 0.0, 0.0)
EZ = (0.0, 0.0, 1.0)
IDENTITY = (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Vector / quaternion primitives
# ---------------------------------------------------------------------------

def dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm3(a):
    return math.sqrt(dot3(a, a))


def unit3(a):
    n = norm3(a)
    if n <= 0.0:
        raise ValueError("zero vector")
    return (a[0] / n, a[1] / n, a[2] / n)


def qmul(a, b):
    """Hamilton product a*b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qnorm(q):
    return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def qnormalize(q):
    n = qnorm(q)
    if n <= 0.0:
        raise ValueError("zero quaternion")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def qcanon(q):
    """Same rotation with w >= 0 (q and -q are the same attitude)."""
    return q if q[3] >= 0 else (-q[0], -q[1], -q[2], -q[3])


def rotate(q, v):
    """Rotate body vector v into world: q * v * q^-1."""
    x, y, z, w = q
    # t = 2 * cross(q.xyz, v); v' = v + w t + cross(q.xyz, t)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def rotate_inv(q, v):
    """Rotate world vector v into body: q^-1 * v * q (= R(q)^T v)."""
    return rotate(qconj(q), v)


def from_axis_angle(axis, angle_rad):
    ax = unit3(axis)
    s = math.sin(angle_rad / 2.0)
    return (ax[0] * s, ax[1] * s, ax[2] * s, math.cos(angle_rad / 2.0))


def from_rotvec(rv):
    """Quaternion for rotation vector rv (axis * angle, rad)."""
    ang = norm3(rv)
    if ang < 1e-12:
        return qnormalize((rv[0] / 2, rv[1] / 2, rv[2] / 2, 1.0))
    s = math.sin(ang / 2.0) / ang
    return (rv[0] * s, rv[1] * s, rv[2] * s, math.cos(ang / 2.0))


def to_rotvec(q):
    """Rotation vector (rad) of q, using the shorter of q / -q."""
    x, y, z, w = qcanon(q)
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    ang = 2.0 * math.atan2(s, w)
    k = ang / s
    return (x * k, y * k, z * k)


def shortest_arc(u, v):
    """Quaternion rotating unit vector u onto unit vector v (minimal angle)."""
    u = unit3(u)
    v = unit3(v)
    d = dot3(u, v)
    if d < -1.0 + 1e-12:
        # 180 deg: any axis perpendicular to u.
        axis = cross3(u, EX)
        if norm3(axis) < 1e-6:
            axis = cross3(u, (0.0, 1.0, 0.0))
        return from_axis_angle(axis, math.pi)
    c = cross3(u, v)
    return qnormalize((c[0], c[1], c[2], 1.0 + d))


def angle_between(qa, qb):
    """Rotation angle (rad) between two attitudes."""
    d = abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3])
    d = min(1.0, d / (qnorm(qa) * qnorm(qb)))
    return 2.0 * math.acos(d)


def yaw_quat(psi_rad):
    return (0.0, 0.0, math.sin(psi_rad / 2.0), math.cos(psi_rad / 2.0))


def is_valid(q) -> bool:
    """Witness reports all-zero components while attitude is unavailable."""
    if q is None or len(q) != 4:
        return False
    if any(not math.isfinite(c) for c in q):
        return False
    return qnorm(q) > 0.5


# ---------------------------------------------------------------------------
# Comparison metrics
# ---------------------------------------------------------------------------

def nose_world(q):
    return rotate(q, EX)


def tilt_from_vertical_deg(q) -> float:
    """Angle between body +X (nose) and world +Z, degrees."""
    nz = max(-1.0, min(1.0, nose_world(q)[2]))
    return math.degrees(math.acos(nz))


def nose_angle_deg(q_a, q_b) -> float:
    """Angle between the two nose (body +X) directions in WORLD, degrees.

    A heading offset rotates a tilted nose about world Z, so compare with the
    heading offset removed (see ``nose_error_deg``) unless the nose is
    vertical.
    """
    a = nose_world(q_a)
    b = nose_world(q_b)
    d = max(-1.0, min(1.0, dot3(a, b) / (norm3(a) * norm3(b))))
    return math.degrees(math.acos(d))


def tilt_deg_error(q_truth, q_witness) -> float:
    """|tilt-from-vertical(truth) - tilt-from-vertical(Witness)|, degrees.

    Fully heading-invariant (yaw about world Z does not change the angle of
    the nose from vertical).
    """
    return abs(tilt_from_vertical_deg(q_truth)
               - tilt_from_vertical_deg(q_witness))


def heading_offset_rad(q_truth, q_witness) -> float:
    """Yaw psi about world +Z that best maps truth onto Witness.

    q_d = q_w * q_t^-1 is the world-frame error rotation; its twist about
    world Z (swing-twist decomposition) is the heading offset.
    """
    d = qmul(q_witness, qconj(q_truth))
    if abs(d[2]) < 1e-15 and abs(d[3]) < 1e-15:
        return 0.0
    return 2.0 * math.atan2(d[2], d[3])


def attitude_error_deg(q_truth, q_witness, psi_rad: float) -> float:
    """Full attitude error after removing heading offset psi, degrees."""
    corrected = qmul(yaw_quat(-psi_rad), q_witness)
    return math.degrees(angle_between(q_truth, corrected))


def nose_error_deg(q_truth, q_witness, psi_rad: float) -> float:
    """Angle between noses after removing heading offset psi, degrees."""
    corrected = qmul(yaw_quat(-psi_rad), q_witness)
    return nose_angle_deg(q_truth, corrected)


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Run-level summary
# ---------------------------------------------------------------------------

DEFAULT_THRESHOLDS = {
    # Applied only when Witness reports non-zero quaternions.
    "final_tilt_max_deg": 5.0,      # |truth tilt - Witness tilt| at the end
    "final_att_max_deg": None,      # full error (heading removed) at the end
    "flight_tilt_max_deg": None,    # max tilt error Boost..Descent
}

SUMMARY_PHASES = ("Pad idle", "Boost", "Coast", "Descent", "Landed")


def _stats(values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    if not vals:
        return None
    return {"n": len(vals), "max": round(max(vals), 3),
            "rms": round(math.sqrt(sum(v * v for v in vals) / len(vals)), 3)}


def compute_records(records, psi_rad):
    """Fill heading-removed errors into packet records (in place)."""
    for r in records:
        if not r["valid"]:
            r["att_err_deg"] = None
            r["nose_err_deg"] = None
            continue
        if psi_rad is None:
            r["att_err_deg"] = None
            r["nose_err_deg"] = None
        else:
            r["att_err_deg"] = attitude_error_deg(r["q_truth"], r["q_witness"],
                                                  psi_rad)
            r["nose_err_deg"] = nose_error_deg(r["q_truth"], r["q_witness"],
                                               psi_rad)
    return records


def summarize(records, psi_rad, thresholds=None, heading_source=""):
    """Attitude summary and pass/fail. Returns a JSON-able dict.

    ``records`` are dicts with keys sim_s, phase (expected Witness state
    name at that tick), valid, q_truth, q_witness, tilt_err_deg.
    """
    th = dict(DEFAULT_THRESHOLDS)
    th.update({k: v for k, v in (thresholds or {}).items() if k in th})
    valid = [r for r in records if r["valid"]]
    out = {"packets": len(records), "valid_packets": len(valid),
           "thresholds": th, "available": bool(valid), "problems": [],
           "warnings": [], "phases": {}, "final": None,
           "heading_offset_deg": None if psi_rad is None
           else round(math.degrees(psi_rad), 3),
           "heading_source": heading_source}
    if not records:
        out["warnings"].append("attitude not available: no 0x02/0xE0 "
                               "packets were received during the run")
        out["passed"] = True
        return out
    if not valid:
        out["warnings"].append(
            "attitude not available: Witness quaternions stayed all-zero "
            "(attitude estimator not flashed or not initialised)")
        out["passed"] = True
        return out
    out["first_valid_sim_s"] = round(valid[0]["sim_s"], 3)
    for ph in SUMMARY_PHASES:
        rs = [r for r in valid if r["phase"] == ph]
        if not rs:
            continue
        out["phases"][ph] = {
            "tilt_err_deg": _stats(r["tilt_err_deg"] for r in rs),
            "nose_err_deg": _stats(r.get("nose_err_deg") for r in rs),
            "att_err_deg": _stats(r.get("att_err_deg") for r in rs)}
    last = valid[-1]
    out["final"] = {
        "sim_s": round(last["sim_s"], 3), "phase": last["phase"],
        "tilt_err_deg": round(last["tilt_err_deg"], 3),
        "nose_err_deg": None if last.get("nose_err_deg") is None
        else round(last["nose_err_deg"], 3),
        "att_err_deg": None if last.get("att_err_deg") is None
        else round(last["att_err_deg"], 3),
        "truth_tilt_deg": round(tilt_from_vertical_deg(last["q_truth"]), 3),
        "witness_tilt_deg": round(tilt_from_vertical_deg(last["q_witness"]),
                                  3)}
    passed = True
    lim = th.get("final_tilt_max_deg")
    if lim is not None and out["final"]["tilt_err_deg"] > lim:
        passed = False
        out["problems"].append(
            f"final tilt error {out['final']['tilt_err_deg']:.2f} deg > "
            f"{lim:.2f} deg")
    lim = th.get("final_att_max_deg")
    if lim is not None:
        fe = out["final"]["att_err_deg"]
        if fe is None:
            out["warnings"].append("final attitude error not evaluated: no "
                                   "heading reference (Boost not seen with "
                                   "valid quaternions)")
        elif fe > lim:
            passed = False
            out["problems"].append(
                f"final attitude error {fe:.2f} deg > {lim:.2f} deg")
    lim = th.get("flight_tilt_max_deg")
    if lim is not None:
        fl = [r["tilt_err_deg"] for r in valid
              if r["phase"] in ("Boost", "Coast", "Descent")]
        if fl and max(fl) > lim:
            passed = False
            out["problems"].append(
                f"max in-flight tilt error {max(fl):.2f} deg > {lim:.2f} deg")
    out["passed"] = passed
    return out


def format_summary(att: dict) -> str:
    if att is None:
        return ""
    lines = ["Attitude (truth vs Witness):"]
    if not att["available"]:
        for w in att["warnings"]:
            lines.append("  WARNING: " + w)
        return "\n".join(lines)

    def s(st, key):
        v = (st or {}).get(key)
        return "-" if v is None else f"{v['max']:6.2f}/{v['rms']:6.2f}"
    lines.append(f"  {att['valid_packets']}/{att['packets']} packets with "
                 f"valid quaternions (first at sim "
                 f"{att['first_valid_sim_s']:.2f} s); heading offset "
                 + ("-" if att["heading_offset_deg"] is None
                    else f"{att['heading_offset_deg']:.1f} deg")
                 + (f" ({att['heading_source']})"
                    if att.get("heading_source") else ""))
    lines.append(f"  {'Phase':<9}{'tilt err max/rms':>18}"
                 f"{'nose err max/rms':>18}{'att err max/rms':>18}")
    for ph, st in att["phases"].items():
        lines.append(f"  {ph:<9}{s(st, 'tilt_err_deg'):>18}"
                     f"{s(st, 'nose_err_deg'):>18}"
                     f"{s(st, 'att_err_deg'):>18}")
    f = att["final"]

    def g(v):
        return "-" if v is None else f"{v:.2f}"
    lines.append(f"  Final ({f['phase']}, sim {f['sim_s']:.1f} s): tilt err "
                 f"{g(f['tilt_err_deg'])} deg (truth {g(f['truth_tilt_deg'])}"
                 f", Witness {g(f['witness_tilt_deg'])}), nose err "
                 f"{g(f['nose_err_deg'])} deg, att err "
                 f"{g(f['att_err_deg'])} deg")
    for p in att["problems"]:
        lines.append("  PROBLEM: " + p)
    for w in att["warnings"]:
        lines.append("  WARNING: " + w)
    lines.append("  Attitude " + ("PASS" if att["passed"] else "FAIL"))
    return "\n".join(lines)
