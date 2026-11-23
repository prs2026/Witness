r"""Turn an AIM onboard CSV export (one "time" column per data group) into a CSV with one time
column, for the Witness/Horizon CSV viewer.

    .venv\Scripts\python.exe aim_csv_flatten.py "flight logs\aimflight.csv"               # -> aimflight_flat.csv
    .venv\Scripts\python.exe aim_csv_flatten.py "flight logs\aimflight.csv" --rate 100    # resampled, 100 Hz
    .venv\Scripts\python.exe aim_csv_flatten.py "flight logs\aimflight.csv" --zero-at-liftoff

Output columns: time_s, flight_state (0 Pad, 1 Boost, 2 Coast, 3 Descent, 4 Landed, from the AIM
state; the viewer's "Sync logs on" uses it), then every AIM field (gyro_x, pressure, latitude,
qx, ...; a name used by two groups gets its group as a prefix).
  * Default: one row per distinct sample time of any group, with each group's values on its own
    rows and the other cells empty (no invented data).
  * --rate HZ: a fixed time step; fractional numeric fields are interpolated linearly inside each group's
    own time span (empty outside it); text, integer-valued fields (flags, counts) and
    flight_state hold the last value.
time_s is the AIM's own clock (offset 0), or seconds from the AIM's Pad -> Boost change with
--zero-at-liftoff. The export's shifted columns are handled by logs_to_mcap.read_aim_csv.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logs_to_mcap as logs  # noqa: E402

STATES = {"pad": 0, "boost": 1, "coast": 2, "decent": 3, "descent": 3, "landed": 4}


def numeric(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def load_groups(path: Path):
    """{group: {"t": [...], field: [...]}} from the AIM CSV, in file order."""
    groups: dict[str, dict] = {}
    for _log, _pub, topic, msg in logs.read_aim_csv(path):
        group = topic.rsplit("/", 1)[-1]
        g = groups.setdefault(group, {"t": []})
        n = len(g["t"])
        g["t"].append(msg["t_s"])
        for k, v in msg.items():
            if k == "t_s":
                continue
            col = g.setdefault(k, [None] * n)
            col.append(v)
        for k, col in g.items():
            if len(col) < n + 1:
                col.append(None)
    return groups


def column_names(groups):
    seen: dict[str, int] = {}
    for g in groups.values():
        for k in g:
            if k != "t":
                seen[k] = seen.get(k, 0) + 1
    return {(group, k): (k if seen[k] == 1 else f"{group}_{k}")
            for group, g in groups.items() for k in g if k != "t"}


def state_series(groups):
    """(times, flight_state codes) from the state group."""
    g = groups.get("state")
    if not g or "state" not in g:
        return np.array([]), np.array([])
    t, codes = [], []
    for ti, s in zip(g["t"], g["state"]):
        code = STATES.get(str(s).strip().lower()) if s is not None else None
        if code is not None:
            t.append(ti)
            codes.append(code)
    return np.array(t), np.array(codes)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="AIM onboard CSV export")
    ap.add_argument("-o", "--output", type=Path, help="output CSV (default: <input>_flat.csv)")
    ap.add_argument("--rate", type=float, help="resample to this many rows per second")
    ap.add_argument("--zero-at-liftoff", action="store_true",
                    help="time_s = seconds from the AIM's Pad -> Boost change")
    args = ap.parse_args()

    groups = load_groups(args.input)
    if not groups:
        print("no AIM data groups found", file=sys.stderr)
        return 2
    names = column_names(groups)
    st_t, st_code = state_series(groups)
    offset = 0.0
    if args.zero_at_liftoff:
        lift = next((st_t[i] for i in range(1, len(st_code)) if st_code[i] == 1 and st_code[i - 1] == 0), None)
        if lift is None:
            print("no Pad -> Boost state change; time left on the AIM clock", file=sys.stderr)
        else:
            offset = -lift
            print(f"liftoff at AIM time {lift:.3f} s")

    all_t = np.unique(np.concatenate([np.asarray(g["t"], float) for g in groups.values()]))
    times = (np.arange(all_t[0], all_t[-1] + 1e-9, 1.0 / args.rate) if args.rate else all_t)
    out_cols = ["time_s", "flight_state"] + list(names.values())
    table = {c: [None] * len(times) for c in out_cols}
    table["time_s"] = [round(t + offset, 6) for t in times]
    if len(st_t):
        idx = np.searchsorted(st_t, times, side="right") - 1
        table["flight_state"] = [int(st_code[i]) if i >= 0 else None for i in idx]

    for group, g in groups.items():
        t = np.asarray(g["t"], float)
        for k, values in g.items():
            if k == "t":
                continue
            col = table[names[(group, k)]]
            is_num = all(v is None or numeric(v) for v in values) and any(numeric(v) for v in values)
            if not args.rate:
                pos = np.searchsorted(times, t)
                for p, v in zip(pos, values):
                    if v is not None:
                        col[p] = v
            elif is_num and not all(v is None or float(v).is_integer() for v in values):
                ok = np.array([numeric(v) for v in values])
                tv, vv = t[ok], np.array([v for v in values if numeric(v)], float)
                inside = (times >= tv[0]) & (times <= tv[-1])
                interp = np.interp(times, tv, vv)
                for i in np.nonzero(inside)[0]:
                    col[i] = round(float(interp[i]), 6)
            else:
                idx = np.searchsorted(t, times, side="right") - 1
                for i, j in enumerate(idx):
                    if 0 <= j and times[i] <= t[-1]:
                        col[i] = values[j]

    out = args.output or args.input.with_name(args.input.stem + "_flat.csv")
    with open(out, "w", newline="", encoding="utf-8") as handle:
        w = csv.writer(handle)
        w.writerow(out_cols)
        for i in range(len(times)):
            w.writerow(["" if table[c][i] is None else table[c][i] for c in out_cols])
    print(f"Wrote {len(times)} rows x {len(out_cols)} columns to {out}"
          f" ({'every sample time' if not args.rate else f'{args.rate:g} Hz'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
