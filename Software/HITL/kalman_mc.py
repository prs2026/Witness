"""Monte Carlo scenarios and tuning for Witness's position filter.

Each realization reuses a recorded HITL run's truth trajectory, RF GPS fixes
(nmea.txt, the receiver's real behaviour through the outage) and Witness's
flight states, and redraws what real flight will differ in:
  IMU       new synthesis seed (zero offsets, noise), vibration from the
            flight's predicted profile at scale U(0.2, 1.0), sensitivity
            error N(0, 1 %) low-g / N(0, 3 %) high-g per axis, mounting
            misalignment N(0, 0.5 deg) per axis
  attitude  the replay's attitude = truth x error, error N(0, 1 deg) per tilt
            axis plus a drift N(0, 1 deg) over the flight
  baro      pressure of a non-standard atmosphere: ISA + dT, dT U(-15, +15) K,
            plus N(0, 2 Pa) noise; the filter still assumes ISA

  python kalman_mc.py make RUN [RUN ...] --n 8        write the realizations (cached)
  python kalman_mc.py eval key=value ...              score a configuration on all of them
  python kalman_mc.py tune                            coordinate search over the tuning keys
"""
from __future__ import annotations

import csv
import json
import math
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import imu_synth as ims  # noqa: E402
import kalman_replay as kr  # noqa: E402

MC_DIR = HERE / "runs_mc"
G = 9.80665
R_AIR = 287.053


def _rot(axis_angles):
    """Rotation matrix for small rotation vector (rad)."""
    th = np.linalg.norm(axis_angles)
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(axis_angles) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def _quat_mul(a, b):
    x1, y1, z1, w1 = a.T
    x2, y2, z2, w2 = b.T
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], 1)


def _quat_from_rotvec(r):
    th = np.linalg.norm(r, axis=1, keepdims=True)
    half = th / 2
    s = np.where(th > 1e-12, np.sin(half) / np.maximum(th, 1e-12), 0.5)
    return np.concatenate([r * s, np.cos(half)], 1)


def isa_temp(h):
    h = np.asarray(h, dtype=float)
    return np.select([h < 11000, h < 20000, h < 32000, h < 47000],
                     [288.15 - 0.0065 * h, 216.65 + 0 * h, 216.65 + 0.001 * (h - 20000),
                      228.65 + 0.0028 * (h - 32000)], 270.65)


def pressure_profile(dT, h_max=60000.0, step=10.0):
    """(h, p) of a hydrostatic atmosphere with T = ISA + dT, p(0) = 101325."""
    h = np.arange(-500.0, h_max, step)
    T = isa_temp(h) + dT
    lnp = np.concatenate([[0.0], np.cumsum(-G / (R_AIR * 0.5 * (T[1:] + T[:-1])) * step)])
    p = np.exp(lnp)
    p *= 101325.0 / np.interp(0.0, h, p)
    return h, p


def make_realization(run: Path, k: int, out_dir: Path, high_sens_sigma: float = 0.03) -> Path:
    out = out_dir / f"{run.name}-mc{k:02d}.txt"
    meta = out.with_suffix(".json")
    if out.exists() and meta.exists():
        return out
    rng = np.random.default_rng(1000 + k)
    tr = kr._truth(run)
    t100 = tr["sim_s"]
    f = np.stack([tr["f_body_x_g"], tr["f_body_y_g"], tr["f_body_z_g"]], 1)
    w = np.stack([tr["w_body_x_dps"], tr["w_body_y_dps"], tr["w_body_z_dps"]], 1)
    mis = _rot(rng.normal(0, math.radians(0.5), 3))
    f_s, w_s = f @ mis.T, w @ mis.T
    vib_scale = float(rng.uniform(0.2, 1.0))
    prof = ims.resolve_profile(str(HERE / "trajectories" / "eayftrajectory.csv"), "eayftrajectory")
    launch = float(t100[np.argmax(tr["expected_state"] != "Pad idle")])
    stream = ims.synthesize(t100, f_s, w_s, launch, ims.ImuSynthParams(
        seed=2000 + k, vibration_scale=vib_scale,
        vibration_thrust_path=prof["thrust_path"] if prof else "",
        vibration_lateral_path=prof["lateral_path"] if prof else "",
        vibration_full_scale_g=prof["full_scale_g"] if prof else 150.67))
    low = stream.low_g.astype(float) * ims.LOW_G_PER_COUNT * 1000.0
    high = stream.high_g.astype(float) * ims.HIGH_G_PER_COUNT * 1000.0
    s_low = 1 + rng.normal(0, 0.01, 3)
    s_high = 1 + rng.normal(0, 0.03, 3) * (high_sens_sigma / 0.03)   # same draws, scaled
    low *= s_low
    high *= s_high
    t_imu = stream.time_s
    q = np.stack([tr["q_x"], tr["q_y"], tr["q_z"], tr["q_w"]], 1)
    qi = q[np.clip(np.searchsorted(t100, t_imu), 0, len(t100) - 1)]
    e0 = np.array([*rng.normal(0, math.radians(1.0), 2), 0.0])
    e1 = np.array([*rng.normal(0, math.radians(1.0), 2), 0.0])
    frac = np.clip((t_imu - launch) / max(1.0, t_imu[-1] - launch), 0, 1)[:, None]
    err = np.where(t_imu[:, None] >= launch, e0 + e1 * frac, e0 * 0.0)   # pad: attitude settles on gravity
    # error applied in the world frame: q_est = q_err * q_true
    q_est = _quat_mul(_quat_from_rotvec(err), qi)
    dT = float(rng.uniform(-15, 15))
    hp, pp = pressure_profile(dT)
    pressure = np.interp(tr["alt_msl_m"], hp, pp) + rng.normal(0, 2.0, len(t100))
    pressure = np.maximum(pressure, 1.0)
    lines = []
    for i in range(len(t_imu)):
        lines.append((t_imu[i], 3, "I %.6f %.2f %.2f %.2f %.1f %.1f %.1f %.6f %.6f %.6f %.6f" % (
            t_imu[i], low[i, 0], low[i, 1], low[i, 2], high[i, 0], high[i, 1], high[i, 2],
            q_est[i, 0], q_est[i, 1], q_est[i, 2], q_est[i, 3])))
    for i in range(len(t100)):
        lines.append((t100[i], 1, f"B {t100[i]:.4f} {pressure[i]:.1f}"))
    base = kr.make_events(run, "truth")
    for line in open(base):
        if line[0] in "GS":
            t = float(line.split()[1])
            lines.append((t, 0 if line[0] == "S" else 2, line.rstrip("\n")))
    lines.sort(key=lambda e: (e[0], e[1]))
    with open(out, "w") as fo:
        for e in lines:
            fo.write(e[2] + "\n")
    meta.write_text(json.dumps({"run": str(run), "k": k, "vibration_scale": vib_scale,
                                "sens_low": s_low.tolist(), "sens_high": s_high.tolist(),
                                "misalign_deg": np.degrees(np.array([mis[2, 1], mis[0, 2], mis[1, 0]])).tolist(),
                                "att_err0_deg": np.degrees(e0).tolist(), "att_drift_deg": np.degrees(e1).tolist(),
                                "dT_K": dT}, indent=1))
    return out


def realizations(directory: Path | None = None):
    return sorted((directory or MC_DIR).glob("*-mc*.txt"))


def eval_one(ev: Path, args) -> dict:
    meta = json.loads(ev.with_suffix(".json").read_text())
    run = Path(meta["run"])
    out = ev.with_name(ev.stem + f"-out-{os.getpid()}-{abs(hash(tuple(args))) % 10**8}.csv")
    r = subprocess.run([str(kr.build()), str(ev), str(out), *args], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    o = np.genfromtxt(out, delimiter=",", names=True)
    out.unlink(missing_ok=True)
    s = kr.score(run, o["t"], o["u"], ev.stem, kr.EAYF_PHASES)
    # consistency: NEES of the up position over the flight
    tr = kr._truth(run)
    m = (o["t"] > 20) & (o["t"] < 420)
    e = o["u"][m] - np.interp(o["t"][m], tr["sim_s"], tr["up_m"])
    s["nees_up"] = round(float(np.mean((e / np.maximum(o["sigma_u"][m], 1e-3)) ** 2)), 2)
    return s


def evaluate(args, files=None, workers=6) -> dict:
    files = files or realizations()
    kr.build()                       # once, before the workers start
    with ThreadPoolExecutor(workers) as ex:
        res = list(ex.map(lambda f: eval_one(f, args), files))
    rel = np.array([r["max_rel_pct"] for r in res])
    asc = np.array([r.get("ascent_max_rel_pct", 0.0) for r in res])
    desc = np.array([r.get("descent_max_rel_pct", 0.0) for r in res])
    return {"args": " ".join(args), "worst_pct": float(rel.max()), "p90_pct": float(np.percentile(rel, 90)),
            "mean_pct": float(rel.mean()), "rms_m": float(np.mean([r["rms_m"] for r in res])),
            "worst_ascent_pct": float(asc.max()), "mean_ascent_pct": float(asc.mean()),
            "worst_descent_pct": float(desc.max()),
            "nees_up": float(np.median([r["nees_up"] for r in res])), "runs": res}


DESCENT_LIMIT_PCT = 5.0


def cost(summary) -> float:
    """Ascent (launch to apogee) is what matters: its worst case across the
    set, plus a little of its mean so ties break towards accuracy. Descent
    must stay physically sane: anything over DESCENT_LIMIT_PCT is penalized
    hard, below it descent does not count."""
    return (summary["worst_ascent_pct"] + 0.2 * summary["mean_ascent_pct"]
            + 3.0 * max(0.0, summary["worst_descent_pct"] - DESCENT_LIMIT_PCT))


TUNE_KEYS = {   # name: candidate values (log-ish grids around physical values)
    "accel_noise_mps2": [0.1, 0.2, 0.5, 1.0, 2.0],
    "accel_noise_fraction": [0.005, 0.01, 0.02, 0.035, 0.05, 0.08],
    "gps_lag_accel_s2": [0.5, 1, 2, 5, 10, 20],
    "gps_lag_speed_s": [0.0, 0.05, 0.1, 0.2],
    "gps_stress_tau_s": [1, 2, 5, 10],
    "gps_vertical_uere_m": [2, 3.3, 5, 8],
    "baro_pressure_sigma_pa": [5, 10, 20, 50, 100, 200],
    "baro_scale_sigma": [0.01, 0.02, 0.03, 0.05],
    "baro_scale_rate": [0.0, 0.002, 0.005, 0.01, 0.02],
    "gate_sigma": [3, 5, 8],
}


def tune(start: dict, rounds: int = 3, log=print) -> dict:
    """Coordinate search: each round tries every candidate of every key with
    the others fixed, keeping any improvement (cheap, deterministic, and the
    Monte Carlo set is fixed, so comparisons use common random numbers)."""
    best = dict(start)
    args = lambda d: [f"{k}={v}" for k, v in d.items()]
    best_s = evaluate(args(best))
    log(f"start cost {cost(best_s):.3f} ascent worst {best_s['worst_ascent_pct']:.2f}% descent worst "
        f"{best_s['worst_descent_pct']:.2f}% {best}")
    for rnd in range(rounds):
        improved = False
        for key, values in TUNE_KEYS.items():
            for v in values:
                if best.get(key) == v:
                    continue
                cand = dict(best, **{key: v})
                s = evaluate(args(cand))
                if cost(s) < cost(best_s) - 1e-3:
                    best, best_s, improved = cand, s, True
                    log(f"round {rnd} {key}={v}: cost {cost(s):.3f} ascent worst {s['worst_ascent_pct']:.2f}% "
                        f"mean {s['mean_ascent_pct']:.2f}% descent worst {s['worst_descent_pct']:.2f}% "
                        f"nees {s['nees_up']}")
        if not improved:
            break
    return {"config": best, "summary": best_s}


def main(argv):
    MC_DIR.mkdir(exist_ok=True)
    if argv[0] == "make":
        n = int(argv[argv.index("--n") + 1]) if "--n" in argv else 8
        runs = [Path(a) for a in argv[1:] if not a.startswith("--") and not a.isdigit()]
        for run in runs:
            for k in range(n):
                p = make_realization(run, k, MC_DIR)
                print(p.name, json.loads(p.with_suffix(".json").read_text())["dT_K"])
    elif argv[0] == "eval":
        s = evaluate(argv[1:])
        for r in s["runs"]:
            print(r)
        print({k: v for k, v in s.items() if k != "runs"})
    elif argv[0] == "tune":
        start = dict(a.split("=", 1) for a in argv[1:])
        res = tune({k: float(v) for k, v in start.items()})
        print(json.dumps({"config": res["config"],
                          "summary": {k: v for k, v in res["summary"].items() if k != "runs"}}, indent=1))
        (MC_DIR / "tune_result.json").write_text(json.dumps(res, indent=1, default=str))


if __name__ == "__main__":
    main(sys.argv[1:])
