"""Small synthetic trajectory CSVs for the tests (no external data)."""

from __future__ import annotations

import math

G0 = 9.80665
FT = 0.3048


def simulate(burn_s=1.5, thrust_g=15.0, tilt_deg=5.0, descent_mps=40.0,
             roll_rev_s=0.1):
    """Point-mass flight: tilted burn, drag-free coast, steady descent.

    Returns rows of (t, e, n, u, ve, vn, vu, roll_rev_s, aero_ok) in SI,
    on a variable time step (10 ms in the burn, 50-200 ms later).
    """
    d = (math.sin(math.radians(tilt_deg)), 0.0,
         math.cos(math.radians(tilt_deg)))
    t = 0.0
    p = [0.0, 0.0, 0.0]
    v = [0.0, 0.0, 0.0]
    rows = []
    dt = 0.001
    next_out = 0.0
    apogee = False
    while True:
        if t < burn_s:
            if t < 0.05:  # vertical rail for the first 50 ms
                a = [0.0, 0.0, thrust_g * G0 - G0]
            else:
                a = [thrust_g * G0 * d[0], 0.0, thrust_g * G0 * d[2] - G0]
        elif v[2] > -descent_mps:
            a = [0.0, 0.0, -G0]
        else:
            a = [0.0, 0.0, 0.0]
            v[2] = -descent_mps
        if v[2] < 0:
            apogee = True
        if apogee:  # drift stops after apogee
            v[0] = 0.0
        out_dt = 0.01 if t < burn_s + 0.5 else 0.2
        if t >= next_out - 1e-9:
            rows.append((t, p[0], p[1], p[2], v[0], v[1], v[2],
                         roll_rev_s if t < burn_s else 0.0, not apogee))
            next_out += out_dt
        for i in range(3):
            v[i] += a[i] * dt
            p[i] += v[i] * dt
        t += dt
        if p[2] <= 0.0 and t > burn_s:
            rows.append((t, p[0], p[1], 0.0, v[0], v[1], v[2], 0.0, False))
            return rows


def write(path, rows, units="ft", velocities=True, duplicates=True,
          blank_line=True):
    """Write rows as an export-like CSV ('name [unit]' headers)."""
    ls = FT if units == "ft" else 1.0
    lu = "ft" if units == "ft" else "m"
    vu = lu + "/s"
    head = ["time [s]", f"east [{lu}]", f"north [{lu}]", f"altitude [{lu}]"]
    if velocities:
        head += [f"east velocity [{vu}]", f"north velocity [{vu}]",
                 f"vertical velocity [{vu}]"]
    head += ["roll rate [rev/s]", "angle of attack [deg]", "Mach"]
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(head) + "\n")
        for k, (t, e, n, u, ve, vn, vz, rr, aero) in enumerate(rows):
            cells = [t, e / ls, n / ls, u / ls]
            if velocities:
                cells += [ve / ls, vn / ls, vz / ls]
            cells += [rr]
            line = ",".join(f"{c:.6g}" for c in cells)
            aoa = "0.5" if aero else "nan"
            mach = f"{math.sqrt(ve * ve + vn * vn + vz * vz) / 340:.4f}"
            line += f",{aoa},{mach}\n"
            f.write(line)
            if duplicates and k in (30, 31, 200):
                f.write(line)  # duplicated timestamp
            if blank_line and k == 100:
                f.write("\n")
