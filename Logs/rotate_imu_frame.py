r"""Rotate the IMU vectors of a log into another axis frame, so Witness, Raven and AIM axes line up.

    .venv\Scripts\python.exe rotate_imu_frame.py LOG.csv --preset raven --map "x=-x,y=y,z=-z" -o raven_aligned.csv
    .venv\Scripts\python.exe rotate_imu_frame.py LOG.csv --preset aim --map "x=z,y=x,z=y" --roll 30
    .venv\Scripts\python.exe rotate_imu_frame.py LOG.csv --preset raven --stats -1.5:-0.5 --stats 0.2:0.9

Vectors: --preset picks a log's accelerometer/gyro/magnetometer column triples (see PRESETS),
or give your own with --vec "colX,colY,colZ" (repeatable). Every triple gets the same rotation.

Rotation, applied in this order:
  --map "x=-x,y=z,z=y"   new axis = +/- an old axis (axis swaps and sign flips). Must be a proper
                          rotation (determinant +1) unless --allow-reflection (a sensor that is
                          left-handed, or a log that reports acceleration with the opposite sign).
  --roll DEG              then a rotation about the new x axis (the rocket's long axis if the map
                          put the nose on x): use it for a board mounted at an angle around the tube.
  --euler "z:30,y:-2"     then any further rotations about the new axes, in the order given.
  --matrix "r11,r12,...,r33"  a full 3x3 rotation instead of the three above.

Helpers:
  --stats A:B             print each vector's mean over that time window (after the rotation),
                          e.g. a pad window (gravity) and a boost window (thrust axis).
  --estimate-roll REF.csv --ref-preset P [--ref-map ...] --window A:B
                          the roll angle about x that best lines up this log's lateral gyro (y, z)
                          with the reference's over the window (both logs on a common time: see
                          --time-offset / --ref-time-offset); add --apply-estimate to use it.
Times: the preset's time column (or --time-col), plus --time-offset seconds, e.g. to put liftoff
at 0. The output is the input with the vector columns replaced by the rotated values (or written as
new columns with --suffix _aligned); all other columns are copied unchanged.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

PRESETS = {
    "witness": {"time": "time_seconds",
                "vecs": [("low_accel_x_g", "low_accel_y_g", "low_accel_z_g"),
                         ("high_accel_x_g", "high_accel_y_g", "high_accel_z_g"),
                         ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")],
                "gyro": 2},
    "raven": {"time": "Flight_Time_(s)",
              "vecs": [("Accel_X", "Accel_Y", "Accel_Z"), ("Gyro_X", "Gyro_Y", "Gyro_Z")],
              "gyro": 1},
    "aim": {"time": "time_s",                                  # aim_csv_flatten.py output
            "vecs": [("lat_x_accel", "lat_y_accel", "acceleration"), ("gyro_x", "gyro_y", "gyro_z"),
                     ("mag_x", "mag_y", "mag_z")],
            "gyro": 1},
}
AXES = {"x": 0, "y": 1, "z": 2}


def rot_axis(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def map_matrix(text: str) -> np.ndarray:
    """'x=-x,y=z,z=y' -> M with new = M @ old."""
    m = np.zeros((3, 3))
    seen = set()
    for part in text.replace(" ", "").split(","):
        hit = re.fullmatch(r"([xyz])=([+-]?)([xyz])", part.lower())
        if not hit:
            raise SystemExit(f"bad --map entry {part!r}; use e.g. x=-x,y=z,z=y")
        new, sign, old = hit.groups()
        m[AXES[new], AXES[old]] = -1.0 if sign == "-" else 1.0
        seen.add(new)
    if seen != set("xyz") or sorted(np.abs(m).sum(axis=0)) != [1, 1, 1]:
        raise SystemExit("--map must give x, y and z each exactly one old axis")
    return m


def build_rotation(args) -> np.ndarray:
    if args.matrix:
        r = np.array([float(v) for v in args.matrix.split(",")]).reshape(3, 3)
    else:
        r = map_matrix(args.map) if args.map else np.eye(3)
        if args.roll:
            r = rot_axis("x", args.roll) @ r
        for step in filter(None, (args.euler or "").replace(" ", "").split(",")):
            axis, deg = step.split(":")
            r = rot_axis(axis.lower(), float(deg)) @ r
    det = np.linalg.det(r)
    if not np.allclose(r @ r.T, np.eye(3), atol=1e-6):
        raise SystemExit("the rotation is not orthonormal")
    if det < 0 and not args.allow_reflection:
        raise SystemExit("this mapping is a reflection (determinant -1), not a rotation; flip one more "
                         "axis, or pass --allow-reflection if the log really is mirrored")
    return r


def load(path, preset, vecs, time_col):
    p = PRESETS.get(preset or "", {})
    triples = [tuple(v.split(",")) for v in vecs] if vecs else p.get("vecs", [])
    df = pd.read_csv(path, low_memory=False)
    triples = [t for t in triples if all(c in df.columns for c in t)]
    if not triples:
        raise SystemExit(f"{path}: none of the vector columns are in the file (use --vec or --preset)")
    tcol = time_col or p.get("time")
    if tcol and tcol not in df.columns:
        tcol = None
    return df, triples, tcol


def rotate(df, triples, r):
    out = df.copy()
    for t in triples:
        v = df[list(t)].to_numpy(dtype=float)
        out.loc[:, list(t)] = v @ r.T
    return out


def window(text):
    a, b = (float(x) for x in text.split(":"))
    return a, b


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="output CSV (default: <input>_aligned.csv)")
    ap.add_argument("--preset", choices=sorted(PRESETS))
    ap.add_argument("--vec", action="append", help="'colX,colY,colZ' (repeatable; replaces the preset's)")
    ap.add_argument("--time-col")
    ap.add_argument("--time-offset", type=float, default=0.0, help="seconds added to the time column")
    ap.add_argument("--map")
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--euler")
    ap.add_argument("--matrix")
    ap.add_argument("--allow-reflection", action="store_true")
    ap.add_argument("--suffix", default="", help="write rotated vectors as new columns <col><suffix>")
    ap.add_argument("--stats", action="append", type=window, metavar="A:B")
    ap.add_argument("--estimate-roll", type=Path, metavar="REF.csv")
    ap.add_argument("--ref-preset", choices=sorted(PRESETS))
    ap.add_argument("--ref-vec", help="the reference's gyro triple 'colX,colY,colZ'")
    ap.add_argument("--ref-map", help="the reference's own --map into the common frame")
    ap.add_argument("--ref-time-col")
    ap.add_argument("--ref-time-offset", type=float, default=0.0)
    ap.add_argument("--window", type=window, metavar="A:B", help="time window for --estimate-roll")
    ap.add_argument("--apply-estimate", action="store_true")
    args = ap.parse_args()

    df, triples, tcol = load(args.input, args.preset, args.vec, args.time_col)
    r = build_rotation(args)
    print("rotation (new = R @ old):\n" + np.array2string(r, precision=4, suppress_small=True))
    rotated = rotate(df, triples, r)
    t = (df[tcol].to_numpy(dtype=float) + args.time_offset) if tcol else None

    if args.estimate_roll:
        if t is None or not args.window:
            raise SystemExit("--estimate-roll needs a time column and --window")
        ref, ref_triples, ref_tcol = load(args.estimate_roll, args.ref_preset,
                                          [args.ref_vec] if args.ref_vec else None, args.ref_time_col)
        if not ref_tcol:
            raise SystemExit("the reference has no time column (use --ref-time-col)")
        gi = PRESETS.get(args.ref_preset or "", {}).get("gyro", 0) if not args.ref_vec else 0
        ref_r = map_matrix(args.ref_map) if args.ref_map else np.eye(3)
        ref_g = ref[list(ref_triples[gi])].to_numpy(float) @ ref_r.T
        ref_t = ref[ref_tcol].to_numpy(float) + args.ref_time_offset
        mi = PRESETS.get(args.preset or "", {}).get("gyro", 0) if not args.vec else len(triples) - 1
        my_g = rotated[list(triples[mi])].to_numpy(float)
        a, b = args.window
        sel = (t >= a) & (t <= b) & np.all(np.isfinite(my_g), axis=1)
        ref_ok = np.all(np.isfinite(ref_g), axis=1)
        ry = np.interp(t[sel], ref_t[ref_ok], ref_g[ref_ok, 1])
        rz = np.interp(t[sel], ref_t[ref_ok], ref_g[ref_ok, 2])
        mine = my_g[sel, 1] + 1j * my_g[sel, 2]
        theirs = ry + 1j * rz
        theta = math.degrees(np.angle(np.sum(theirs * np.conj(mine))))
        resid = np.sqrt(np.mean(np.abs(theirs - mine * np.exp(1j * math.radians(theta))) ** 2))
        corr_x = np.corrcoef(my_g[sel, 0], np.interp(t[sel], ref_t[ref_ok], ref_g[ref_ok, 0]))[0, 1]
        print(f"estimated roll about x: {theta:+.1f} deg (lateral gyro residual {resid:.1f}, "
              f"x-axis gyro correlation {corr_x:+.3f} over {sel.sum()} samples)")
        # Time shift between the logs: best x-gyro correlation within +/-2 s (reported only).
        best = max(((np.corrcoef(my_g[sel, 0], np.interp(t[sel] + d, ref_t[ref_ok], ref_g[ref_ok, 0]))[0, 1], d)
                    for d in np.arange(-2.0, 2.0001, 0.005)))
        print(f"  best time alignment: reference time = this log's time {best[1]:+.3f} s "
              f"(x-gyro correlation {best[0]:+.3f}); if it is not ~0, adjust --time-offset and re-run")
        if corr_x < 0:
            print("  the x axes run opposite ways: fix the --map first (flip x and one lateral axis)")
        if args.apply_estimate:
            r = rot_axis("x", theta) @ r
            rotated = rotate(df, triples, r)
            print("applied; final rotation:\n" + np.array2string(r, precision=4, suppress_small=True))

    if args.stats:
        for a, b in args.stats:
            if t is None:
                print("--stats needs a time column")
                break
            sel = (t >= a) & (t <= b)
            for tr in triples:
                m = rotated.loc[sel, list(tr)].mean().to_numpy()
                print(f"  {a:g}..{b:g} s  {tr[0].rsplit('_', 1)[0] if '_' in tr[0] else tr[0]:<16} "
                      f"mean x {m[0]:+9.3f}  y {m[1]:+9.3f}  z {m[2]:+9.3f}  ({sel.sum()} rows)")

    if args.suffix:
        out_df = df.copy()
        for tr in triples:
            for c in tr:
                out_df[c + args.suffix] = rotated[c]
    else:
        out_df = rotated
    out = args.output or args.input.with_name(args.input.stem + "_aligned.csv")
    out_df.to_csv(out, index=False)
    print(f"Wrote {out} ({len(triples)} vector(s) rotated: {', '.join(t[0] for t in triples)} ...)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
