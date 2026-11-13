"""Offline replay and scoring of Witness's position filter on recorded HITL runs.

  python kalman_replay.py RUN_DIR [key=value ...]        replay + score once
  python kalman_replay.py RUN_DIR --compare-flight        also score Witness's own in-flight output

The firmware filter runs on the host (Witness/softwarev1/tools/kalman_replay.cpp,
built on first use into rf_tools/bin) from the run's recorded inputs:
  IMU    imu960.npz (the 960 Hz stream sent to Witness)
  baro   sent.csv pressure_pa (100 Hz)
  GPS    nmea.txt GGA (+ GSA VDOP), at the sim time Witness received them
  phase  comparison_flash.csv witness_state (Witness's flight states)
  attitude truth.csv quaternion (attitude="truth") - the attitude estimator is
           not replayed; HITL shows it within ~1.2 deg on eayf.

Score: altitude error against truth, relative to max(true height above the
pad, FLOOR_M), from launch to landing; the goal is max relative error < 5 %.
"""
from __future__ import annotations

import bisect
import csv
import functools
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FW = HERE.parent / "Witness" / "softwarev1"
EXE = HERE / "rf_tools" / "bin" / ("kalman_replay.exe" if os.name == "nt" else "kalman_replay")
FLOOR_M = 100.0
PHASES = {"Pad idle": 0, "Boost": 1, "Coast": 2, "Descent": 3, "Landed": 4}
SOURCES = ["tools/kalman_replay.cpp", "main/position_filter.cpp", "main/accel_filter.cpp",
           "main/heading_align.cpp"]
HEADERS = ["main/position_filter.h", "main/accel_filter.h", "main/pressure_altitude.h",
           "main/heading_align.h"]


def build(force: bool = False, compiler: str = "g++") -> Path:
    """(Re)builds the replay when any firmware source is newer than the exe."""
    files = [FW / s for s in SOURCES + HEADERS]
    if not force and EXE.exists() and EXE.stat().st_mtime >= max(f.stat().st_mtime for f in files):
        return EXE
    EXE.parent.mkdir(parents=True, exist_ok=True)
    cmd = [compiler, "-std=c++17", "-O2", "-I", str(FW / "main"), *[str(FW / s) for s in SOURCES],
           "-o", str(EXE)]
    if os.name == "nt":
        cmd.append("-static")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("kalman_replay build failed:\n" + r.stderr[-3000:])
    return EXE


@functools.lru_cache(maxsize=16)
def _truth_cached(path: str):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)


def _truth(run: Path):
    return _truth_cached(str(Path(run) / "truth.csv"))


def make_events(run: Path, attitude: str = "truth") -> Path:
    """Writes run/replay_events.txt (cached) and returns its path."""
    out = run / f"replay_events_{attitude}.txt"
    if out.exists() and out.stat().st_mtime >= (run / "nmea.txt").stat().st_mtime:
        return out
    events = []   # (t, order, text)
    z = np.load(run / "imu960.npz")
    t_imu = z["time_s"]
    low = z["low_g_counts"] * z["low_g_per_count"] * 1000.0
    high = z["high_g_counts"] * z["high_g_per_count"] * 1000.0
    tr = _truth(run)
    tt = tr["sim_s"]
    q = np.stack([tr["q_x"], tr["q_y"], tr["q_z"], tr["q_w"]], 1)
    idx = np.clip(np.searchsorted(tt, t_imu), 0, len(tt) - 1)
    qi = q[idx]
    for k in range(len(t_imu)):
        events.append((t_imu[k], 3, "I %.6f %.2f %.2f %.2f %.1f %.1f %.1f %.6f %.6f %.6f %.6f" % (
            t_imu[k], low[k, 0], low[k, 1], low[k, 2], high[k, 0], high[k, 1], high[k, 2],
            qi[k, 0], qi[k, 1], qi[k, 2], qi[k, 3])))
    with open(run / "sent.csv", newline="") as f:
        for r in csv.DictReader(f):
            t = float(r["sim_ms"]) / 1000.0
            events.append((t, 1, f"B {t:.4f} {float(r['pressure_pa']):.1f}"))
    last_phase = None
    with open(run / "comparison_flash.csv", newline="") as f:
        for r in csv.DictReader(f):
            ph = PHASES.get(r["witness_state"])
            if ph is not None and ph != last_phase:
                t = float(r["sim_s"])
                events.append((t, 0, f"S {t:.4f} {ph}"))
                last_phase = ph
    vdop = float("nan")
    gga = re.compile(r"\s*[\d.]+ sim=\s*([-\d.]+) \$G[NP]GGA,[^,]*,(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d),(\d+),([^,]*),([-\d.]+),M")
    gsa = re.compile(r"\s*[\d.]+ sim=\s*([-\d.]+) \$G[A-Z]GSA,([^*]*)\*")
    for line in open(run / "nmea.txt", encoding="utf-8", errors="replace"):
        m = gsa.match(line)
        if m:
            f = m.group(2).split(",")
            if len(f) >= 17 and f[1] in ("2", "3") and f[16]:
                try:
                    vdop = float(f[16])
                except ValueError:
                    pass
            continue
        m = gga.match(line)
        if not m or int(m.group(6)) == 0:
            continue
        t = float(m.group(1))
        lat = float(m.group(2)[:2]) + float(m.group(2)[2:]) / 60
        lat = -lat if m.group(3) == "S" else lat
        lon = float(m.group(4)[:3]) + float(m.group(4)[3:]) / 60
        lon = -lon if m.group(5) == "W" else lon
        hdop = m.group(8) or "nan"
        events.append((t, 2, f"G {t:.4f} {lat:.9f} {lon:.9f} {float(m.group(9)):.2f} {int(m.group(7))} "
                             f"{hdop} {'nan' if math.isnan(vdop) else vdop}"))
    events.sort(key=lambda e: (e[0], e[1]))
    with open(out, "w") as f:
        for e in events:
            f.write(e[2] + "\n")
    return out


def replay(run: Path, args=(), attitude: str = "truth") -> np.ndarray:
    exe = build()
    ev = make_events(run, attitude)
    out = run / "replay_out.csv"
    r = subprocess.run([str(exe), str(ev), str(out), *args], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return np.genfromtxt(out, delimiter=",", names=True)


def score(run: Path, est_t, est_up, label: str = "", phases=None) -> dict:
    """Altitude (up from the pad) error against truth from launch to landing."""
    tr = _truth(run)
    tt, up = tr["sim_s"], tr["up_m"]
    exp = tr["expected_state"]
    launch = tt[np.argmax(exp != "Pad idle")]
    land_idx = np.where(exp == "Landed")[0]
    land = tt[land_idx[0]] if len(land_idx) else tt[-1]
    m = (est_t >= launch) & (est_t <= land)
    truth_up = np.interp(est_t[m], tt, up)
    err = est_up[m] - truth_up
    rel = np.abs(err) / np.maximum(truth_up, FLOOR_M)
    i = int(np.argmax(rel))
    res = {"label": label, "max_rel_pct": round(100 * float(rel[i]), 2), "at_s": round(float(est_t[m][i]), 2),
           "err_there_m": round(float(err[i]), 1), "max_abs_m": round(float(np.max(np.abs(err))), 1),
           "rms_m": round(float(np.sqrt(np.mean(err ** 2))), 1),
           "pct_time_over_5": round(100 * float(np.mean(rel > 0.05)), 2)}
    # Ascent (launch to apogee) is the objective; descent must stay sane.
    apogee = float(tt[np.argmax(up)])
    asc = est_t[m] <= apogee
    if asc.any():
        res["ascent_max_rel_pct"] = round(100 * float(np.max(rel[asc])), 2)
    if (~asc).any():
        res["descent_max_rel_pct"] = round(100 * float(np.max(rel[~asc])), 2)
    for name, a, b in phases or ():
        mm = (est_t[m] >= a) & (est_t[m] < b)
        if mm.any():
            res[name] = round(100 * float(np.max(rel[mm])), 2)
    return res


EAYF_PHASES = [("boost", 20, 23), ("coast", 23, 33), ("sustain", 33, 36.2), ("gap", 36.2, 50),
               ("ascent", 50, 129), ("descent", 129, 420)]


def flight_output(run: Path):
    rows = list(csv.DictReader(open(run / "comparison_flash.csv", newline="")))
    t = np.array([float(r["sim_s"]) for r in rows])
    u = np.array([float(r["witness_up_m"] or "nan") for r in rows])
    return t, u


def main(argv) -> int:
    run = Path(argv[0])
    rest = [a for a in argv[1:] if "=" in a]
    if "--compare-flight" in argv:
        t, u = flight_output(run)
        print(score(run, t, u, "Witness in flight", EAYF_PHASES))
    out = replay(run, rest)
    print(score(run, out["t"], out["u"], "replay " + " ".join(rest), EAYF_PHASES))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
