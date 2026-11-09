"""Drive a HITL run from a trajectory CSV export.

``load_trajectory(path)`` parses a CSV whose header cells look like
``name [unit]`` (e.g. ``altitude [ft]``, ``vertical velocity [ft/s]``,
``roll rate [rev/s]``). ``build(params)`` resamples it to an exact 100 Hz
timeline, adds a pad hold and a post-landing hold, and converts it into the
same per-tick ``SimSample`` objects the sender streams (each extended with the
truth quaternion), plus a conversion report.

Conventions (match Witness firmware and comms.h):

* WORLD = ENU (+Z up). BODY = LSM6 sensor frame, +X = nose axis.
* Quaternion: Hamilton, BODY -> WORLD, scalar last on the wire.
* Accelerometers read specific force f = a - g_world, g_world = (0, 0, -g0),
  expressed in BODY: f_body = R(q)^T f_world. On the pad f_body = (+1 g, 0, 0)
  for a vertical rail.
* Gyro sample k is the body rate that rotates q_{k-1} into q_k
  (omega_k = log(q_{k-1}^-1 q_k) / dt), so integrating the gyro with
  q_k = q_{k-1} * exp(omega_k dt) reproduces the truth quaternion.

Every modelling assumption is listed in README.md ("CSV trajectories").
"""

from __future__ import annotations

import bisect
import csv
import dataclasses
import math
import random
import re
from dataclasses import dataclass, field

import attitude as att
import hitl_protocol as hp
import sim_model as sm

G0 = sm.G0
DT_S = sm.DT_S
TICK_MS = sm.TICK_MS

LOW_G_LIMIT_G = 16.0       # LSM6 low-g full scale
HIGH_G_LIMIT_G = 320.0     # LSM6 high-g full scale
GYRO_LIMIT_DPS = 2000.0    # LSM6 gyro full scale
MS5607_MIN_PA = 1000.0     # MS5607 range ends at 10 mbar (~31 km ISA)
ATTITUDE_MODES = ("hold", "pendulum")


# ---------------------------------------------------------------------------
# ISA 1976 atmosphere (to 84.8 km geopotential)
# ---------------------------------------------------------------------------

_R_AIR = 287.05287
_R_EARTH = 6356766.0
# (base geopotential altitude m, base temperature K, lapse K/m)
_ISA_LAYERS = ((0.0, 288.15, -0.0065), (11000.0, 216.65, 0.0),
               (20000.0, 216.65, 0.001), (32000.0, 228.65, 0.0028),
               (47000.0, 270.65, 0.0), (51000.0, 270.65, -0.0028),
               (71000.0, 214.65, -0.002), (84852.0, 186.946, 0.0))


def _layer_base_pressures(p0):
    out = [p0]
    for i in range(len(_ISA_LAYERS) - 1):
        h0, t0, lap = _ISA_LAYERS[i]
        h1 = _ISA_LAYERS[i + 1][0]
        p = out[-1]
        if lap == 0.0:
            p = p * math.exp(-G0 * (h1 - h0) / (_R_AIR * t0))
        else:
            t1 = t0 + lap * (h1 - h0)
            p = p * (t1 / t0) ** (-G0 / (_R_AIR * lap))
        out.append(p)
    return out


_BASE_P = {}


def isa(alt_msl_m: float, sea_level_pa: float = 101325.0):
    """(pressure Pa, temperature K) at geometric altitude MSL."""
    base = _BASE_P.get(sea_level_pa)
    if base is None:
        base = _BASE_P[sea_level_pa] = _layer_base_pressures(sea_level_pa)
    h = _R_EARTH * alt_msl_m / (_R_EARTH + alt_msl_m)  # geopotential
    i = 0
    while i + 1 < len(_ISA_LAYERS) and h >= _ISA_LAYERS[i + 1][0]:
        i += 1
    h0, t0, lap = _ISA_LAYERS[i]
    p0 = base[i]
    t = t0 + lap * (h - h0)
    if lap == 0.0:
        p = p0 * math.exp(-G0 * (h - h0) / (_R_AIR * t0))
    else:
        p = p0 * (t / t0) ** (-G0 / (_R_AIR * lap))
    return p, t


# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------

_TAU = 2.0 * math.pi
UNITS = {
    "time": {"s": 1.0, "sec": 1.0, "ms": 1e-3},
    "length": {"ft": 0.3048, "feet": 0.3048, "m": 1.0, "km": 1000.0},
    "speed": {"ft/s": 0.3048, "fps": 0.3048, "m/s": 1.0, "km/h": 1 / 3.6},
    "angrate": {"rev/s": _TAU, "rps": _TAU, "rpm": _TAU / 60.0,
                "deg/s": math.pi / 180.0, "dps": math.pi / 180.0,
                "°/s": math.pi / 180.0, "rad/s": 1.0},
}
SI_UNIT = {"time": "s", "length": "m", "speed": "m/s", "angrate": "rad/s"}

# key -> (accepted names, quantity kind)
COLUMNS = {
    "time": (("time", "t", "flight time"), "time"),
    "east": (("east", "east position", "position east", "x east"),
             "length"),
    "north": (("north", "north position", "position north", "y north"),
              "length"),
    "altitude": (("altitude", "altitude agl", "altitude above ground",
                  "height", "height agl", "up"), "length"),
    "ve": (("east velocity", "velocity east", "v east"), "speed"),
    "vn": (("north velocity", "velocity north", "v north"), "speed"),
    "vu": (("vertical velocity", "up velocity", "velocity up",
            "vertical speed", "v up"), "speed"),
    "roll_rate": (("roll rate", "spin rate"), "angrate"),
}
REQUIRED = ("time", "altitude")
_HEADER_RE = re.compile(r"^\s*(.*?)\s*(?:\[\s*(.*?)\s*\])?\s*$")


def _norm_name(name: str) -> str:
    return re.sub(r"\s+", " ", name.strip().lower().replace("_", " "))


def _norm_unit(unit: str) -> str:
    return unit.strip().lower().replace(" ", "").replace("sec", "s") \
        if unit else ""


def parse_header(cells):
    """Map CSV header cells to {key: (index, scale_to_SI, unit)}."""
    found = {}
    notes = []
    for idx, cell in enumerate(cells):
        m = _HEADER_RE.match(cell.lstrip("﻿"))
        name = _norm_name(m.group(1))
        unit = m.group(2) or ""
        for key, (names, kind) in COLUMNS.items():
            if name in names and key not in found:
                u = _norm_unit(unit)
                if not u:
                    scale = 1.0
                    notes.append(f"column {cell!r} has no unit; assuming "
                                 f"{SI_UNIT[kind]}")
                elif u in UNITS[kind]:
                    scale = UNITS[kind][u]
                else:
                    raise ValueError(
                        f"column {cell!r}: unsupported unit {unit!r} "
                        f"(supported: {', '.join(UNITS[kind])})")
                found[key] = (idx, scale, unit or SI_UNIT[kind])
                break
    missing = [k for k in REQUIRED if k not in found]
    if missing:
        raise ValueError("trajectory CSV is missing required column(s): "
                         + ", ".join(missing)
                         + " (header cells look like 'altitude [ft]')")
    return found, notes


def _to_float(text: str) -> float:
    text = text.strip()
    if not text:
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


@dataclass
class Trajectory:
    """Kinematics in SI on the de-duplicated source time base (CSV time)."""
    t: list
    east: list
    north: list
    up: list
    ve: list
    vn: list
    vu: list
    roll_rate: list            # rad/s about body X
    path: str = ""
    columns: dict = field(default_factory=dict)   # key -> "header [unit]"
    derived: list = field(default_factory=list)   # keys derived, not read
    aero_nan_t: float | None = None
    rows: int = 0
    duplicates_merged: int = 0
    rows_dropped: int = 0
    dt_min: float = 0.0
    dt_max: float = 0.0
    notes: list = field(default_factory=list)
    extra: dict = field(default_factory=dict)     # e.g. mach column

    @property
    def t_end(self) -> float:
        return self.t[-1]


def _derivative(t, x):
    """Non-uniform central differences (one-sided at the ends)."""
    n = len(t)
    if n < 2:
        return [0.0] * n
    out = [0.0] * n
    for i in range(n):
        lo, hi = max(0, i - 1), min(n - 1, i + 1)
        dt = t[hi] - t[lo]
        out[i] = (x[hi] - x[lo]) / dt if dt > 0 else 0.0
    return out


def load_trajectory(path: str) -> Trajectory:
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader)
        cols, notes = parse_header(header)
        raw = [row for row in reader if any(c.strip() for c in row)]
    mapped = {i for i, _s, _u in cols.values()}
    other = [i for i in range(len(header)) if i not in mapped]
    mach_idx = next((i for i, c in enumerate(header)
                     if _norm_name(_HEADER_RE.match(c).group(1)) == "mach"),
                    None)

    def get(row, key):
        if key not in cols:
            return math.nan
        i, scale, _u = cols[key]
        return _to_float(row[i]) * scale if i < len(row) else math.nan

    keys = ("east", "north", "altitude", "ve", "vn", "vu", "roll_rate")
    recs = []
    dropped = 0
    aero_nan_t = None
    first_other = None
    last_t = -math.inf
    for row in raw:
        t = get(row, "time")
        alt = get(row, "altitude")
        if math.isnan(t) or math.isnan(alt) or t < last_t:
            dropped += 1
            continue
        last_t = t
        vals = [get(row, k) for k in keys]
        mach = _to_float(row[mach_idx]) if mach_idx is not None and \
            mach_idx < len(row) else math.nan
        recs.append((t, vals, mach))
        nan_other = [i for i in other
                     if i < len(row) and math.isnan(_to_float(row[i]))]
        if first_other is None:
            first_other = set(nan_other)
        elif aero_nan_t is None and set(nan_other) - first_other:
            aero_nan_t = t
    if len(recs) < 2:
        raise ValueError("trajectory CSV needs at least two valid rows")

    # Merge duplicate timestamps (exports print time to ~6 significant
    # digits, so distinct samples can share a printed time): average them.
    merged = []
    dups = 0
    i = 0
    while i < len(recs):
        j = i
        while j + 1 < len(recs) and recs[j + 1][0] == recs[i][0]:
            j += 1
        group = recs[i:j + 1]
        dups += len(group) - 1
        vals = []
        for k in range(len(keys)):
            fin = [g[1][k] for g in group if not math.isnan(g[1][k])]
            vals.append(sum(fin) / len(fin) if fin else math.nan)
        machs = [g[2] for g in group if not math.isnan(g[2])]
        merged.append((recs[i][0], vals,
                       sum(machs) / len(machs) if machs else math.nan))
        i = j + 1

    t = [m[0] for m in merged]
    col = {k: [m[1][n] for m in merged] for n, k in enumerate(keys)}
    derived = []
    for k in ("east", "north"):
        if k not in cols:
            derived.append(k + " (zero)")
        col[k] = [0.0 if math.isnan(v) else v for v in col[k]]
    # Fill position gaps by holding the previous value.
    for k in ("east", "north", "altitude"):
        prev = 0.0
        for n, v in enumerate(col[k]):
            if math.isnan(v):
                col[k][n] = prev
            else:
                prev = v
    for vk, pk in (("ve", "east"), ("vn", "north"), ("vu", "altitude")):
        d = _derivative(t, col[pk])
        if vk not in cols:
            derived.append(vk + f" (d {pk}/dt)")
            col[vk] = d
        else:
            col[vk] = [d[n] if math.isnan(v) else v
                       for n, v in enumerate(col[vk])]
    if "roll_rate" not in cols:
        derived.append("roll_rate (zero)")
    col["roll_rate"] = [0.0 if math.isnan(v) else v
                        for v in col["roll_rate"]]
    dts = [b - a for a, b in zip(t, t[1:])]
    return Trajectory(
        t=t, east=col["east"], north=col["north"], up=col["altitude"],
        ve=col["ve"], vn=col["vn"], vu=col["vu"],
        roll_rate=col["roll_rate"], path=path,
        columns={k: header[v[0]].strip() for k, v in cols.items()},
        derived=derived, aero_nan_t=aero_nan_t, rows=len(raw),
        duplicates_merged=dups, rows_dropped=dropped,
        dt_min=min(dts), dt_max=max(dts), notes=notes,
        extra={"mach": [m[2] for m in merged]})


class _Interp:
    """Piecewise-linear interpolation of several channels over t."""

    def __init__(self, t, channels):
        self.t = t
        self.ch = channels
        self.n = len(t)

    def __call__(self, x):
        t = self.t
        if x <= t[0]:
            return [c[0] for c in self.ch]
        if x >= t[-1]:
            return [c[-1] for c in self.ch]
        i = bisect.bisect_right(t, x) - 1
        t0, t1 = t[i], t[i + 1]
        a = (x - t0) / (t1 - t0)
        return [c[i] + (c[i + 1] - c[i]) * a for c in self.ch]


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

@dataclass
class CsvParams:
    """Parameters of a CSV-driven run. Times s, altitudes m, rates m/s."""
    csv_path: str = ""
    seed: int = 1
    site_alt_msl_m: float = 1400.0
    lat_deg: float = 32.9903
    lon_deg: float = -106.9750
    sea_level_pressure_pa: float = 101325.0
    temp_offset_c: float = 0.0          # added to ISA temperature
    pad_hold_s: float = 20.0
    post_landing_s: float = 40.0
    touchdown_s: float = 0.2            # velocity -> 0 over this time
    accel_smooth_s: float = 0.05        # centred moving average window
    follow_speed_mps: float = 10.0      # nose follows velocity above 2x this
    rail_dir_speed_mps: float = 0.5     # first motion defines rail direction
    after_apogee_attitude: str = "hold"  # hold | pendulum
    pendulum_amp_deg: float = 10.0
    pendulum_period_s: float = 4.0
    accel_noise_g: float = 0.003
    high_g_noise_g: float = 0.01
    gyro_noise_dps: float = 0.05
    gyro_bias_x_dps: float = 0.0
    gyro_bias_y_dps: float = 0.0
    gyro_bias_z_dps: float = 0.0
    baro_noise_pa: float = 1.0
    gps_sats: int = 10
    gps_fix: bool = True
    battery_mv: float = 7800.0

    @classmethod
    def from_dict(cls, data: dict) -> "CsvParams":
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in names})

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


@dataclass
class TrajSample(sm.SimSample):
    q_truth: tuple = att.IDENTITY     # (x, y, z, w) BODY -> WORLD
    tau_s: float = 0.0                # CSV time (negative on the pad)


@dataclass
class CsvBuild:
    params: CsvParams
    trajectory: Trajectory
    samples: list
    report: dict

    @property
    def truth_quats(self):
        return [s.q_truth for s in self.samples]


def _quantize_gyro(values, residual, rng_noise):
    """Error-feedback quantisation to exact 70 mdps counts, +-2000 dps."""
    out = []
    sat = False
    for a in range(3):
        v = values[a] + rng_noise[a] + residual[a]
        if v > GYRO_LIMIT_DPS or v < -GYRO_LIMIT_DPS:
            v = max(-GYRO_LIMIT_DPS, min(GYRO_LIMIT_DPS, v))
            sat = True
            residual[a] = 0.0
            counts = int(v / hp.GYRO_DPS_PER_COUNT)
        else:
            counts = math.floor(v / hp.GYRO_DPS_PER_COUNT + 0.5)
            residual[a] = v - counts * hp.GYRO_DPS_PER_COUNT
        out.append(counts * hp.GYRO_DPS_PER_COUNT)
    return out, sat


def _windows(flags, times, merge_gap_s=1.0):
    """[(start, end)] spans where flags is True; gaps <= merge_gap_s merge."""
    out = []
    start = prev = last_true = None
    for f, t in zip(flags, times):
        if f:
            if start is None:
                if out and t - out[-1][1] <= merge_gap_s:
                    start = out.pop()[0]
                else:
                    start = t
            last_true = t
        elif start is not None:
            out.append((start, last_true))
            start = None
        prev = t
    if start is not None:
        out.append((start, last_true if last_true is not None else prev))
    return out


def _raw_edges(tr: Trajectory, rail, t_hold):
    """Axial specific-force edges straight from the CSV knots (unsmoothed)."""
    t = tr.t
    pts = []
    i = 0
    n = len(t)
    while i < n - 1 and t[i] < t_hold:
        j = i + 1
        while j < n - 1 and t[j] - t[i] < 0.005:
            j += 1
        dt = t[j] - t[i]
        if dt <= 0:
            break
        a = ((tr.ve[j] - tr.ve[i]) / dt, (tr.vn[j] - tr.vn[i]) / dt,
             (tr.vu[j] - tr.vu[i]) / dt + G0)
        v = ((tr.ve[i] + tr.ve[j]) / 2, (tr.vn[i] + tr.vn[j]) / 2,
             (tr.vu[i] + tr.vu[j]) / 2)
        d = att.unit3(v) if att.norm3(v) > 1.0 else rail
        pts.append(((t[i] + t[j]) / 2, att.dot3(a, d) / G0))
        i = j
    onset = next((tm for tm, f in pts if f > sm.ReferenceStateMachine.
                  LAUNCH_G), None)
    burnouts = []
    ignitions = []
    above = True
    for tm, f in pts:
        if onset is None or tm < onset:
            continue
        if above and f < 1.0:
            burnouts.append(tm)
            above = False
        elif not above and f > 1.5:
            ignitions.append(tm)
            above = True
    return {"thrust_10g_s": onset, "burnout_1g_s": burnouts,
            "reignition_s": ignitions}


def build(params: CsvParams, trajectory: Trajectory | None = None) -> CsvBuild:
    p = params
    if p.after_apogee_attitude not in ATTITUDE_MODES:
        raise ValueError(f"after_apogee_attitude must be one of "
                         f"{', '.join(ATTITUDE_MODES)}")
    tr = trajectory or load_trajectory(p.csv_path)
    rng = random.Random(p.seed)
    t0 = tr.t[0]
    tt = [x - t0 for x in tr.t]
    tau_end = tt[-1]
    kin = _Interp(tt, [tr.east, tr.north, tr.up, tr.ve, tr.vn, tr.vu,
                       tr.roll_rate])

    # Rail direction: velocity direction just after first motion.
    rail = att.EZ
    for k in range(len(tt)):
        v = (tr.ve[k], tr.vn[k], tr.vu[k])
        if att.norm3(v) > p.rail_dir_speed_mps:
            rail = att.unit3(v)
            break
    # Attitude hold start: apogee (max altitude) or the aero-nan region.
    i_apo = max(range(len(tt)), key=lambda k: (tr.up[k], -k))
    t_apo = tt[i_apo]
    t_hold = t_apo
    if tr.aero_nan_t is not None and 0 < tr.aero_nan_t - t0 < t_hold:
        t_hold = tr.aero_nan_t - t0

    td = max(p.touchdown_s, 0.0)
    pad = max(p.pad_hold_s, 0.0)
    end_s = pad + tau_end + td + max(p.post_landing_s, 0.0)
    n_ticks = int(round(end_s / DT_S)) + 1
    v_end = (tr.ve[-1], tr.vn[-1], tr.vu[-1])
    pos0 = (tr.east[0], tr.north[0], tr.up[0])
    pos_end = (tr.east[-1], tr.north[-1], tr.up[-1])

    def vel(tau):
        if tau <= 0.0:
            return (0.0, 0.0, 0.0)
        if tau <= tau_end:
            k = kin(tau)
            return (k[3], k[4], k[5])
        if tau < tau_end + td:
            f = 1.0 - (tau - tau_end) / td
            return (v_end[0] * f, v_end[1] * f, v_end[2] * f)
        return (0.0, 0.0, 0.0)

    # Pass 1: raw acceleration (central difference over one tick of the
    # linear velocity interpolant), then a centred moving average.
    h = DT_S / 2.0
    taus = [k * DT_S - pad for k in range(n_ticks)]
    raw_acc = []
    for tau in taus:
        a = vel(tau + h)
        b = vel(tau - h)
        raw_acc.append(((a[0] - b[0]) / DT_S, (a[1] - b[1]) / DT_S,
                        (a[2] - b[2]) / DT_S))
    win = max(1, int(round(p.accel_smooth_s / DT_S)))
    if win % 2 == 0:
        win += 1
    half = win // 2
    acc = []
    sx = sy = sz = 0.0
    # Running-sum moving average with edge clamping.
    pref = [(0.0, 0.0, 0.0)]
    for a in raw_acc:
        sx += a[0]
        sy += a[1]
        sz += a[2]
        pref.append((sx, sy, sz))
    for k in range(n_ticks):
        lo, hi = max(0, k - half), min(n_ticks, k + half + 1)
        m = hi - lo
        acc.append(((pref[hi][0] - pref[lo][0]) / m,
                    (pref[hi][1] - pref[lo][1]) / m,
                    (pref[hi][2] - pref[lo][2]) / m))

    # Pass 2: attitude, sensors.
    q_tilt = att.shortest_arc(att.EX, rail)
    d_prev = rail
    roll = 0.0
    q_prev = q_tilt
    q_hold = None
    t_land = tau_end + td
    residual = [0.0, 0.0, 0.0]
    bias = (p.gyro_bias_x_dps, p.gyro_bias_y_dps, p.gyro_bias_z_dps)
    samples = []
    max_f = [0.0, 0.0, 0.0]
    max_f_mag = 0.0
    max_w = [0.0, 0.0, 0.0]
    max_w_mag = 0.0
    low_sat = []
    gyro_sat_ticks = 0
    pressures = []
    sim_times = []
    cos_lat = max(math.cos(math.radians(p.lat_deg)), 1e-6)
    for k in range(n_ticks):
        tau = taus[k]
        if tau <= 0.0:
            pos = pos0
        elif tau <= tau_end:
            kk = kin(tau)
            pos = (kk[0], kk[1], kk[2])
        else:
            pos = pos_end
        v = vel(tau)
        # --- truth attitude
        if tau < t_hold and tau <= tau_end:
            if tau > 0.0:
                speed = att.norm3(v)
                vf = max(p.follow_speed_mps, 1e-6)
                w = min(1.0, max(0.0, (speed - vf) / vf))
                if w > 0.0:
                    vd = att.unit3(v)
                    d = att.unit3(tuple(rail[i] * (1 - w) + vd[i] * w
                                        for i in range(3)))
                else:
                    d = rail
                q_tilt = att.qmul(att.shortest_arc(d_prev, d), q_tilt)
                # Remove accumulated drift so the nose is exactly on d.
                q_tilt = att.qnormalize(att.qmul(
                    att.shortest_arc(att.rotate(q_tilt, att.EX), d), q_tilt))
                d_prev = d
                roll += kin(tau)[6] * DT_S
            q = att.qnormalize(att.qmul(
                q_tilt, att.from_axis_angle(att.EX, roll)))
            phase = "pad" if tau <= 0.0 else "flight"
        else:
            if q_hold is None:
                q_hold = q_prev
            q = q_hold
            if p.after_apogee_attitude == "pendulum" and tau < t_land:
                per = max(p.pendulum_period_s, 0.1)
                env = min(1.0, (tau - t_hold) / per, (t_land - tau) / per)
                theta = math.radians(p.pendulum_amp_deg) * max(env, 0.0) * \
                    math.sin(_TAU * (tau - t_hold) / per)
                q = att.qmul(q_hold, att.from_axis_angle((0.0, 1.0, 0.0),
                                                         theta))
            phase = ("recovery" if tau <= tau_end else
                     "touchdown" if tau < t_land else "landed")
        if att.qmul(att.qconj(q_prev), q)[3] < 0:
            q = (-q[0], -q[1], -q[2], -q[3])  # keep the sign continuous
        # --- gyro: body rate from q_{k-1} to q_k
        if k == 0:
            w_dps = (0.0, 0.0, 0.0)
        else:
            rv = att.to_rotvec(att.qmul(att.qconj(q_prev), q))
            w_dps = tuple(math.degrees(c) / DT_S for c in rv)
        q_prev = q
        for i in range(3):
            max_w[i] = max(max_w[i], abs(w_dps[i]))
        max_w_mag = max(max_w_mag, att.norm3(w_dps))
        noise = [rng.gauss(0.0, p.gyro_noise_dps) + bias[i]
                 if p.gyro_noise_dps > 0 else bias[i] for i in range(3)]
        gyro, sat = _quantize_gyro(w_dps, residual, noise)
        gyro_sat_ticks += sat
        # --- specific force
        a = acc[k]
        f_world = (a[0], a[1], a[2] + G0)
        fb = att.rotate_inv(q, f_world)
        fb = (fb[0] / G0, fb[1] / G0, fb[2] / G0)
        for i in range(3):
            max_f[i] = max(max_f[i], abs(fb[i]))
        max_f_mag = max(max_f_mag, att.norm3(fb))

        def ns(sigma):
            return rng.gauss(0.0, sigma) if sigma > 0 else 0.0
        low = tuple(max(-LOW_G_LIMIT_G, min(LOW_G_LIMIT_G,
                                            fb[i] + ns(p.accel_noise_g)))
                    for i in range(3))
        low_sat.append(any(abs(fb[i]) > LOW_G_LIMIT_G for i in range(3)))
        high = tuple(max(-HIGH_G_LIMIT_G, min(HIGH_G_LIMIT_G,
                                              fb[i] + ns(p.high_g_noise_g)))
                     for i in range(3))
        # --- baro / temperature / GPS
        alt_msl = p.site_alt_msl_m + pos[2]
        pres, temp_k = isa(alt_msl, p.sea_level_pressure_pa)
        pres = max(0.0, pres + ns(p.baro_noise_pa))
        temp_c = temp_k - 273.15 + p.temp_offset_c
        pressures.append(pres)
        sim_s = k * DT_S
        sim_times.append(sim_s)
        if p.gps_fix:
            lat = p.lat_deg + pos[1] / 111_320.0
            lon = p.lon_deg + pos[0] / (111_320.0 * cos_lat)
            gps_alt, sats, fix = alt_msl, p.gps_sats, 1
        else:
            lat = lon = gps_alt = 0.0
            sats, fix = p.gps_sats, 0
        hitl = hp.HitlInput(
            sim_time_ms=k * TICK_MS, low_g=low, high_g=high,
            gyro_dps=tuple(gyro), imu_temp_c=temp_c, pressure_pa=pres,
            baro_temp_c=temp_c, lat_deg=lat, lon_deg=lon, gps_alt_m=gps_alt,
            satellites=sats, fix_quality=fix, battery_mv=p.battery_mv)
        samples.append(TrajSample(
            t_ms=k * TICK_MS, phase=phase, alt_msl_m=alt_msl, vel_mps=v[2],
            hitl=hitl, q_truth=att.qcanon(q), tau_s=tau))

    report = _report(p, tr, samples, rail, t_apo, t_hold, pad, tau_end,
                     max_f, max_f_mag, max_w, max_w_mag, low_sat,
                     gyro_sat_ticks, pressures, sim_times)
    return CsvBuild(params=p, trajectory=tr, samples=samples, report=report)


def _report(p, tr, samples, rail, t_apo, t_hold, pad, tau_end, max_f,
            max_f_mag, max_w, max_w_mag, low_sat, gyro_sat_ticks, pressures,
            sim_times):
    i_apo = max(range(len(tr.t)), key=lambda k: (tr.up[k], -k))
    speeds = [math.sqrt(a * a + b * b + c * c)
              for a, b, c in zip(tr.ve, tr.vn, tr.vu)]
    i_vmax = max(range(len(speeds)), key=speeds.__getitem__)
    mach = tr.extra.get("mach") or []
    raw = _raw_edges(tr, rail, t_hold)
    # Achieved edges in the frames actually sent.
    hx = [s.hitl.high_g[0] for s in samples]
    lx = [s.hitl.low_g[0] for s in samples]
    onset = next((s.t_ms / 1000 for s, v in zip(samples, hx)
                  if v > sm.ReferenceStateMachine.LAUNCH_G), None)
    burn, ign = [], []
    above = True
    for s, v in zip(samples, lx):
        tm = s.t_ms / 1000
        if onset is None or tm < onset or s.tau_s > t_hold:
            continue
        if above and v < 1.0:
            burn.append(round(tm, 2))
            above = False
        elif not above and v > 1.5:
            ign.append(round(tm, 2))
            above = True
    low_windows = _windows(low_sat, sim_times)
    ms_windows = _windows([pr < MS5607_MIN_PA for pr in pressures],
                          sim_times)
    i_min = min(range(len(pressures)), key=pressures.__getitem__)
    rnd = (lambda v, n=2: None if v is None else round(v, n))
    return {
        "csv": {
            "path": tr.path, "rows": tr.rows,
            "duplicates_merged": tr.duplicates_merged,
            "rows_dropped": tr.rows_dropped,
            "dt_min_s": rnd(tr.dt_min, 6), "dt_max_s": rnd(tr.dt_max, 4),
            "columns": tr.columns, "derived": tr.derived,
            "aero_nan_t_s": rnd(tr.aero_nan_t, 3), "notes": tr.notes},
        "timeline": {
            "pad_hold_s": pad, "csv_offset_s": pad,
            "touchdown_csv_s": rnd(tau_end, 3),
            "touchdown_sim_s": rnd(pad + tau_end, 3),
            "attitude_hold_csv_s": rnd(t_hold, 3),
            "attitude_hold_sim_s": rnd(pad + t_hold, 3),
            "duration_s": rnd(len(samples) * DT_S, 2),
            "ticks": len(samples)},
        "flight": {
            "apogee_agl_m": rnd(tr.up[i_apo], 1),
            "apogee_agl_ft": rnd(tr.up[i_apo] / 0.3048, 0),
            "apogee_csv_s": rnd(tr.t[i_apo] - tr.t[0], 3),
            "max_speed_mps": rnd(speeds[i_vmax], 1),
            "max_speed_csv_s": rnd(tr.t[i_vmax] - tr.t[0], 2),
            "max_mach": rnd(max((m for m in mach if not math.isnan(m)),
                                default=None), 2),
            "touchdown_vertical_mps": rnd(tr.vu[-1], 2),
            "rail_direction_enu": [round(c, 5) for c in rail],
            "rail_tilt_deg": rnd(math.degrees(math.acos(
                max(-1.0, min(1.0, rail[2])))), 3)},
        "sensors": {
            "max_specific_force_g": [rnd(v) for v in max_f],
            "max_specific_force_mag_g": rnd(max_f_mag),
            "low_g_saturates": bool(low_windows),
            "low_g_saturated_s": rnd(sum(low_sat) * DT_S, 2),
            "low_g_saturation_windows_sim_s": [
                [rnd(a), rnd(b)] for a, b in low_windows],
            "max_gyro_dps": [rnd(v) for v in max_w],
            "max_gyro_mag_dps": rnd(max_w_mag),
            "gyro_saturates": gyro_sat_ticks > 0,
            "high_g_saturates": max_f_mag > HIGH_G_LIMIT_G},
        "baro": {
            "min_pressure_pa": rnd(pressures[i_min], 1),
            "min_pressure_sim_s": rnd(sim_times[i_min]),
            "ms5607_min_pa": MS5607_MIN_PA,
            "below_ms5607_range_windows_sim_s": [
                [rnd(a), rnd(b)] for a, b in ms_windows],
            "below_ms5607_range_s": rnd(sum(
                b - a for a, b in ms_windows), 2),
            "ms5607_note": "MS5607 range ends at 10 mbar (~31 km ISA); "
                           "pressures below it are sent but cannot be "
                           "measured by the real sensor"},
        "edges": {
            "csv_thrust_10g_sim_s": rnd(None if raw["thrust_10g_s"] is None
                                        else raw["thrust_10g_s"] + pad, 3),
            "csv_burnout_1g_sim_s": [rnd(t + pad, 3)
                                     for t in raw["burnout_1g_s"]],
            "csv_reignition_sim_s": [rnd(t + pad, 3)
                                     for t in raw["reignition_s"]],
            "sent_high_g_x_above_10g_sim_s": rnd(onset),
            "sent_low_g_x_below_1g_sim_s": burn,
            "sent_low_g_x_above_1p5g_sim_s": ign},
    }


def format_report(rep: dict) -> str:
    c, tl, fl, se, ba, ed = (rep["csv"], rep["timeline"], rep["flight"],
                             rep["sensors"], rep["baro"], rep["edges"])
    lines = [
        f"CSV {c['path']}: {c['rows']} rows, {c['duplicates_merged']} "
        f"duplicate timestamps merged, {c['rows_dropped']} dropped, dt "
        f"{c['dt_min_s']}..{c['dt_max_s']} s"
        + (f", derived: {', '.join(c['derived'])}" if c["derived"] else ""),
        f"Apogee {fl['apogee_agl_m']:.0f} m ({fl['apogee_agl_ft']:.0f} ft) "
        f"AGL at CSV {fl['apogee_csv_s']:.2f} s; max speed "
        f"{fl['max_speed_mps']:.0f} m/s"
        + (f" (Mach {fl['max_mach']})" if fl["max_mach"] else "")
        + f"; rail tilt {fl['rail_tilt_deg']:.2f} deg",
        f"Timeline: pad hold {tl['pad_hold_s']:.1f} s, attitude held from "
        f"sim {tl['attitude_hold_sim_s']:.2f} s, touchdown sim "
        f"{tl['touchdown_sim_s']:.2f} s, total {tl['duration_s']:.1f} s",
        f"Max specific force {se['max_specific_force_mag_g']:.1f} g "
        f"(|x| {se['max_specific_force_g'][0]:.1f}); low-g "
        + (f"SATURATES at 16 g for {se['low_g_saturated_s']:.2f} s "
           f"(high-g covers it)" if se["low_g_saturates"]
           else "within 16 g")
        + f"; max gyro {se['max_gyro_mag_dps']:.1f} dps"
        + (" (SATURATES)" if se["gyro_saturates"] else ""),
        f"Min pressure {ba['min_pressure_pa']:.0f} Pa at sim "
        f"{ba['min_pressure_sim_s']:.1f} s"
        + ("; below the MS5607 10 mbar range for "
           f"{ba['below_ms5607_range_s']:.1f} s "
           + str(ba["below_ms5607_range_windows_sim_s"])
           + " (values sent anyway; not measurable by the real sensor)"
           if ba["below_ms5607_range_windows_sim_s"] else ""),
        f"Edges (sim s): CSV thrust>10 g {ed['csv_thrust_10g_sim_s']}, "
        f"sent high-g X>10 g {ed['sent_high_g_x_above_10g_sim_s']}; CSV "
        f"burnout<1 g {ed['csv_burnout_1g_sim_s']}, sent low-g X<1 g "
        f"{ed['sent_low_g_x_below_1g_sim_s']}; re-ignition CSV "
        f"{ed['csv_reignition_sim_s']}, sent {ed['sent_low_g_x_above_1p5g_sim_s']}",
    ]
    return "\n".join(lines)
