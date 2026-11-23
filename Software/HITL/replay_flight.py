r"""Replay a real flight's IMU log (Witness I*.BIN decoded to CSV) into the bench Witness over HITL.

    py -3.14 replay_flight.py ..\..\Logs\flightreal\I00003E1.csv --raven "..\..\Logs\flight logs\BlRv_SN2878_LR_11-21-2026_13_47_32.csv"

The recorded low-g / high-g / gyro samples (960 Hz up to apogee; the 1-in-10 Descent records
are interpolated back to 960 Hz) are sent as 0x0E batches, with 100 Hz 0x0C frames carrying the
same IMU values, pressure 0 (barometer invalid, as in the flight) and no GPS fix. A quiet pad
period is built by repeating the quietest stretch of the recorded pre-launch ring. Truth for
the comparison files is the Raven's baro AGL. The run is scored against the real Witness's
transitions from that flight (--expected), so "PASS" means "behaved like the flight".
The tool's normal log export and hub power cycle run at the end (one cycle per run).
"""

from __future__ import annotations

import argparse
import json
import queue
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import csv

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import hitl_gui as hg            # noqa: E402
import hitl_protocol as hp       # noqa: E402
import hitl_session as hs        # noqa: E402
import sim_model as sm           # noqa: E402
import verify_image as vi        # noqa: E402  (host, recorder helpers, hub-gap guard)

RATE = 960.0
PAD_S = 20.0


@dataclass
class ReplayParams:
    source: str
    pad_s: float
    end_ft_s: float
    site_alt_msl_m: float

    def to_dict(self):
        return asdict(self)


def load_imu(path: Path, end_ft: float):
    cols = ["time_seconds", "flight_state", "low_accel_x_g", "low_accel_y_g", "low_accel_z_g",
            "high_accel_x_g", "high_accel_y_g", "high_accel_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]
    with open(path, newline="") as handle:
        rows = [[float(r[c]) for c in cols] for r in csv.DictReader(handle)]
    arr = np.array(rows)
    t0 = arr[arr[:, 1] == 1, 0][0]
    ft_all = arr[:, 0] - t0
    keep = ft_all <= end_ft
    arr, ft_all = arr[keep], ft_all[keep]
    order = np.argsort(ft_all, kind="stable")
    arr, ft_all = arr[order], ft_all[order]
    uniq = np.concatenate([[True], np.diff(ft_all) > 0])
    arr, ft_all = arr[uniq], ft_all[uniq]
    names = cols[2:]
    vals = arr[:, 2:]
    # Quietest 0.5 s of the pre-launch ring, repeated to make the pad hold.
    pre = ft_all < 0
    best, start = None, ft_all[pre].min()
    for a in np.arange(ft_all[pre].min(), -0.6, 0.05):
        w = (ft_all >= a) & (ft_all < a + 0.5)
        if w.sum() > 300:
            sd = vals[w].std(axis=0).sum()
            if best is None or sd < best:
                best, start = sd, a
    quiet = vals[(ft_all >= start) & (ft_all < start + 0.5)]
    fl = ft_all >= start + 0.5
    grid = np.arange(ft_all[fl][0], end_ft, 1.0 / RATE)
    data = np.column_stack([np.interp(grid, ft_all[fl], vals[fl, i]) for i in range(len(names))])
    pad_n = int(PAD_S * RATE)
    pad = np.tile(quiet, (pad_n // len(quiet) + 1, 1))[:pad_n]
    all_data = np.vstack([pad, data])
    ft = np.concatenate([grid[0] - (pad_n - np.arange(pad_n)) / RATE, grid])
    print(f"pad: quiet window at ft {start:.2f}..{start + 0.5:.2f} s (std sum {best:.3f}), "
          f"repeated for {PAD_S:.0f} s; flight {grid[0]:.2f}..{end_ft:.0f} s, {len(all_data)} samples")
    return ft, all_data


def flight_calibration(txt_path: Path):
    """The flight board's stored IMU calibration, from its S*.TXT log ("IMU calibration loaded
    from NVS" lines): (gyro bias dps, low-g offset g, low-g scale, high-g offset g, high-g scale)."""
    text = Path(txt_path).read_text(errors="replace")
    num = r"(-?[0-9.]+)"
    g = re.search(r"IMU calibration loaded from NVS: gyro bias " + " ".join([num] * 3), text)
    a = re.search(r"IMU calibration loaded from NVS: low-g offset " + " ".join([num] * 3) + r" mg, scale "
                  + " ".join([num] * 3) + r"; high-g offset " + " ".join([num] * 3) + r" mg, scale "
                  + " ".join([num] * 3), text)
    if not g or not a:
        raise SystemExit(f"no 'IMU calibration loaded from NVS' lines in {txt_path}")
    v = [float(x) for x in a.groups()]
    return (np.array([float(x) for x in g.groups()]), np.array(v[0:3]) / 1000.0, np.array(v[3:6]),
            np.array(v[6:9]) / 1000.0, np.array(v[9:12]))


def apply_calibration(data, cal):
    """What the flight board's estimator saw: Witness's imu_calibration::apply on the raw log
    values, so a bench board WITHOUT a stored calibration sees the same corrected inputs."""
    bias, low_off, low_scale, high_off, high_scale = cal
    out = data.copy()
    out[:, 0:3] = (data[:, 0:3] - low_off) / low_scale
    out[:, 3:6] = (data[:, 3:6] - high_off) / high_scale
    out[:, 6:9] = data[:, 6:9] - bias
    return out


def counts(values, scale):
    return np.clip(np.round(values / scale), -32768, 32767).astype(">i2")


def build_run(ft, data, raven_csv, site_alt, end_ft, expected):
    sim_s = ft - ft[0]                         # sim time 0 = start of the pad hold
    lift_sim = -ft[0]                          # sim time of liftoff
    low, high, gyro = data[:, 0:3], data[:, 3:6], data[:, 6:9]
    truth_t, truth_agl = np.array([0.0]), np.array([0.0])
    if raven_csv:
        with open(raven_csv, newline="", encoding="utf-8", errors="replace") as handle:
            rr = list(csv.DictReader(handle))
        truth_t = np.array([float(r["Flight_Time_(s)"]) for r in rr])
        truth_agl = np.array([float(r["Baro_Altitude_AGL_(feet)"]) for r in rr]) * 0.3048
    # 100 Hz 0x0C frames.
    samples = []
    for ms in range(0, int(sim_s[-1] * 1000), 10):
        k = min(int(round(ms / 1000.0 * RATE)), len(data) - 1)
        f = ms / 1000.0 - lift_sim
        agl = float(np.interp(f, truth_t, truth_agl, left=0.0, right=float(truth_agl[-1])))
        h = hp.HitlInput(sim_time_ms=ms, low_g=tuple(low[k]), high_g=tuple(high[k]), gyro_dps=tuple(gyro[k]),
                         imu_temp_c=25.0, pressure_pa=0.0, baro_temp_c=0.0, lat_deg=0.0, lon_deg=0.0,
                         gps_alt_m=0.0, satellites=0, fix_quality=0, battery_mv=3150)
        samples.append(sm.SimSample(t_ms=ms, phase="pad" if f < 0 else "flight",
                                    alt_msl_m=site_alt + agl, vel_mps=0.0, hitl=h))
    frames = [hp.encode_hitl(s.hitl) for s in samples]
    run = hs.RunState(params=ReplayParams("flight IMU replay", PAD_S, end_ft, site_alt),
                      preset="flight IMU replay", samples=samples, frames=frames,
                      expected={k: v + lift_sim for k, v in expected.items()},
                      tolerance_s=5.0, time_scale=1, request_debug=True)
    block = np.concatenate([counts(low, hp.LOW_G_PER_COUNT), counts(high, hp.HIGH_G_PER_COUNT),
                            counts(gyro, hp.GYRO_DPS_PER_COUNT)], axis=1).astype(">i2")
    # (np.concatenate returns native byte order: the frames must be big-endian)
    n = len(block) - len(block) % hp.IMU_BATCH_SAMPLES
    for j in range(0, n, hp.IMU_BATCH_SAMPLES):
        run.imu_frames.append(bytes((hp.ID_HITL_IMU,)) + (j & 0xFFFFFFFF).to_bytes(4, "big")
                              + block[j:j + hp.IMU_BATCH_SAMPLES].tobytes() + bytes((hp.EOF,)))
        run.imu_due_ms.append(float(sim_s[j + hp.IMU_BATCH_SAMPLES - 1]) * 1000.0)
    return run, lift_sim


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("imu_csv", type=Path)
    ap.add_argument("--raven", help="Raven LR CSV for truth altitude (optional)")
    ap.add_argument("--end", type=float, default=220.0, help="flight seconds to replay (default 220)")
    ap.add_argument("--site-alt", type=float, default=609.0)
    ap.add_argument("--calibration-from", type=Path,
                    help="flight S*.TXT: apply the flight board's stored IMU calibration to the replayed "
                         "data (use with a bench board that has no calibration)")
    ap.add_argument("--expected", default="1=0,2=1.41,3=73.8",
                    help="real-flight Witness transitions, state=flight seconds")
    args = ap.parse_args()
    expected = {int(k): float(v) for k, v in (x.split("=") for x in args.expected.split(","))}

    ft, data = load_imu(args.imu_csv, args.end)
    if args.calibration_from:
        cal = flight_calibration(args.calibration_from)
        data = apply_calibration(data, cal)
        print("applied the flight calibration: gyro bias %s dps, low-g offset %s g scale %s, "
              "high-g offset %s g scale %s" % tuple(np.round(c, 4).tolist() for c in cal))
    run, lift_sim = build_run(ft, data, args.raven, args.site_alt, args.end, expected)
    print(f"{len(run.frames)} 0x0C frames, {len(run.imu_frames)} 0x0E batches, liftoff at sim {lift_sim:.2f} s")

    wait = vi.hub_gap_ok()
    if wait > 0:
        print(f"waiting {wait:.0f} s for the 4 min gap since the last hub cycle")
        time.sleep(wait)
    session_no = vi.erase()
    print(f"store erased, new session {session_no}")
    session = hs.HitlSession(runs_dir=hs.DEFAULT_RUNS_DIR)
    session.fc_log_config = hg.fc_log_config(hg.load_settings(), True)
    session.connect(vi.host(), vi.WITNESS_PORT, timeout=5)
    session.start_run(run, abort_on_no_hitl=True)
    while not session.wait_run(0.5):
        while True:
            try:
                kind, payload = session.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("log", "warning") and re.search(r"state|FC logs|ENTER|EXIT|attitude|saturat|reset",
                                                         str(payload)):
                print(" ", payload, flush=True)
        if not session.connected and run.streaming:
            break
    session.wait_run(900)
    vi.note_hub_cycle()
    session.disconnect(send_exit=False)
    result = run.result or hs.evaluate(run)
    print(hs.format_table(result))
    print("RUNDIR", run.run_dir, "liftoff sim", lift_sim)
    Path(run.run_dir, "replay_info.json").write_text(json.dumps(
        {"liftoff_sim_s": lift_sim, "expected_flight_s": expected, "source": str(args.imu_csv)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
