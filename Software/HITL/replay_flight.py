r"""Replay a real flight's IMU log (Witness I*.BIN decoded to CSV) into the bench Witness over HITL.

    py -3.14 replay_flight.py ..\..\Logs\flightreal\I00003E1.csv --raven "..\..\Logs\flight logs\BlRv_SN2878_LR_11-21-2026_13_47_32.csv"

The recorded low-g / high-g / gyro samples (960 Hz up to apogee; the 1-in-10 Descent records
are interpolated back to 960 Hz) are sent as 0x0E batches, with 100 Hz 0x0C frames carrying the
same IMU values, pressure 0 (barometer invalid, as in the flight; or the Raven's recorded static
pressure with --baro raven, which carries the real transonic error and the ejection-charge spike)
and no GPS fix. A quiet pad
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


PAD_HEADING_AXES = {"+Y": 0, "-Y": 1, "+Z": 2, "-Z": 3}
PAD_HEADING_SEND_SIM_S = 6.0     # into the pad hold: after the first stationary window


def pad_heading_frame(axis: str, azimuth_deg: float) -> bytes:
    """comms.h IRIS_COMMAND_PAD_HEADING (0x4844), as Horizon's "Set pad heading" sends it:
    05, axis code, azimuth in 0.01 deg (u16 BE), 3 zero bytes, 48 44, 0A."""
    centideg = int(round((azimuth_deg % 360.0) * 100.0)) % 36000
    return (bytes((0x05, PAD_HEADING_AXES[axis])) + centideg.to_bytes(2, "big") + bytes(3)
            + (0x4844).to_bytes(2, "big") + bytes((0x0A,)))


def raven_time_shift(raven_csv, ft, data, v_mark=200.0):
    """Seconds to ADD to the Raven's Flight_Time to put it on the Witness IMU flight time.
    The two zeros differ: the Witness's is its launch detection (2.5 g held 300 ms), the
    Raven's its own liftoff. Matched where each vehicle velocity first passes v_mark m/s in
    boost: the Raven's Velocity_Up (ft/s) against the Witness axial acceleration (|x| axis,
    high-g above 15 g) integrated from the motion onset."""
    with open(raven_csv, newline="", encoding="utf-8", errors="replace") as handle:
        rr = list(csv.DictReader(handle))
    rt = np.array([float(r["Flight_Time_(s)"]) for r in rr])
    rv = np.array([float(r["Velocity_Up"]) for r in rr]) * 0.3048
    i = np.argmax((rv >= v_mark) & (rt > -2) & (rt < 5))
    t_r = np.interp(v_mark, rv[i - 1:i + 1], rt[i - 1:i + 1])
    ax = np.where(np.abs(data[:, 0]) < 15.0, data[:, 0], data[:, 3])
    m = (ft > -2.0) & (ft < 5.0)
    t, a = ft[m], ax[m]
    onset = t[np.argmax((a > 2.0) & (np.convolve(a > 2.0, np.ones(50), "same") >= 50))] - 0.05
    k = t >= onset
    v = np.concatenate([[0.0], np.cumsum((a[k][1:] - 1.0) * 9.80665 * np.diff(t[k]))])
    j = np.argmax(v >= v_mark)
    t_w = np.interp(v_mark, v[j - 1:j + 1], t[k][j - 1:j + 1])
    return float(t_w - t_r)


def raven_pressure(raven_csv, excursion_gain=1.0, window_s=1.5, spike_cap_pa=None, shift_s=0.0):
    """Raven static pressure (Pa) and temperature (C) against flight time. excursion_gain scales
    the short excursions about a running median of window_s (the Mach dip and the ejection
    spike): 1 = as recorded, 3 = three times as deep. spike_cap_pa removes overpressure above
    that many Pa over the running median (None: keep)."""
    with open(raven_csv, newline="", encoding="utf-8", errors="replace") as handle:
        rr = list(csv.DictReader(handle))
    t = np.array([float(r["Flight_Time_(s)"]) for r in rr]) + shift_s
    p = np.array([float(r["Baro_Press_(atm)"]) for r in rr]) * 101325.0
    temp_c = (np.array([float(r["Temperature_(F)"]) for r in rr]) - 32.0) * 5.0 / 9.0
    if excursion_gain != 1.0 or spike_cap_pa is not None:
        dt = float(np.median(np.diff(t)))
        half = max(1, int(round(window_s / dt / 2)))
        base = np.array([np.median(p[max(0, i - half):i + half + 1]) for i in range(len(p))])
        dev = p - base
        if spike_cap_pa is not None:
            dev = np.minimum(dev, spike_cap_pa)
        p = base + excursion_gain * dev
    return t, np.clip(p, 1000.0, 119000.0), temp_c


def amplify_transonic(t, p, gain, ref_p_csv, t_lo=0.3, t_hi=4.0, t_end=7.0):
    """Scale the Raven's transonic altitude error by `gain`. The reference is an IMU-only Kalman
    track of the same flight (a bench replay's P*.csv), scaled so its apogee equals the Raven's
    (the IMU track runs ~1.7 % high). The error e = Raven AGL - reference is multiplied by `gain`
    between t_lo and t_hi s, tapering to 1 by t_end; the pressure is changed by the ISA ratio
    for that altitude change, so everything else (pad, apogee, spike, descent) stays as recorded."""
    with open(ref_p_csv, newline="") as handle:
        rows = [r for r in csv.DictReader(handle) if r["packet_type"] == "state"]
    kt = np.array([float(r["time_seconds"]) for r in rows])
    ka = np.array([float(r["kalman_altitude"]) for r in rows])
    fs = np.array([int(float(r["flight_state"])) for r in rows])
    kt = kt - kt[fs == 1][0]
    pad_p = float(np.median(p[t < 0])) if (t < 0).any() else float(p[0])
    tk = 284.2                                        # ISA K at ~600 m
    h = tk / 0.0065 * (1.0 - (p / pad_p) ** (1.0 / 5.2559))   # AGL from pressure
    apo = (t > 15) & (t < 20.2)
    i_apo = np.argmax(np.where(apo, h, -1e9))
    ref = np.interp(t, kt, ka)
    ref = ref * (h[i_apo] / max(1.0, float(np.interp(t[i_apo], kt, ka))))
    w = np.clip((t - (t_lo - 0.2)) / 0.2, 0, 1) * np.where(
        t <= t_hi, 1.0, 0.5 * (1 + np.cos(np.pi * np.clip((t - t_hi) / (t_end - t_hi), 0, 1))))
    h_new = h + (gain - 1.0) * w * (h - ref)
    p_new = pad_p * np.clip(1.0 - 0.0065 * h_new / tk, 1e-3, None) ** 5.2559
    e = h - ref
    m = (t > t_lo) & (t < t_hi)
    print(f"transonic error (Raven - scaled IMU Kalman) {t_lo}-{t_hi} s: min {e[m].min():+.0f} m, "
          f"max {e[m].max():+.0f} m; x{gain}: min {(gain * e[m]).min():+.0f} m, max {(gain * e[m]).max():+.0f} m")
    return np.where(w > 0, p_new, p)


def counts(values, scale):
    return np.clip(np.round(values / scale), -32768, 32767).astype(">i2")


def build_run(ft, data, raven_csv, site_alt, end_ft, expected, baro=None, raven_shift_s=0.0):
    sim_s = ft - ft[0]                         # sim time 0 = start of the pad hold
    lift_sim = -ft[0]                          # sim time of liftoff
    low, high, gyro = data[:, 0:3], data[:, 3:6], data[:, 6:9]
    truth_t, truth_agl = np.array([0.0]), np.array([0.0])
    if raven_csv:
        with open(raven_csv, newline="", encoding="utf-8", errors="replace") as handle:
            rr = list(csv.DictReader(handle))
        truth_t = np.array([float(r["Flight_Time_(s)"]) for r in rr]) + raven_shift_s
        truth_agl = np.array([float(r["Baro_Altitude_AGL_(feet)"]) for r in rr]) * 0.3048
    if baro is not None:
        bt, bp, btemp = baro
        pad_p = float(np.median(bp[bt < 0])) if (bt < 0).any() else float(bp[0])
        pad_temp = float(np.median(btemp[bt < 0])) if (bt < 0).any() else float(btemp[0])
    # 100 Hz 0x0C frames.
    samples = []
    for ms in range(0, int(sim_s[-1] * 1000), 10):
        k = min(int(round(ms / 1000.0 * RATE)), len(data) - 1)
        f = ms / 1000.0 - lift_sim
        agl = float(np.interp(f, truth_t, truth_agl, left=0.0, right=float(truth_agl[-1])))
        pressure, baro_temp = 0.0, 0.0
        if baro is not None:
            pressure = pad_p if f < bt[0] else float(np.interp(f, bt, bp))
            baro_temp = pad_temp if f < bt[0] else float(np.interp(f, bt, btemp))
        h = hp.HitlInput(sim_time_ms=ms, low_g=tuple(low[k]), high_g=tuple(high[k]), gyro_dps=tuple(gyro[k]),
                         imu_temp_c=25.0, pressure_pa=pressure, baro_temp_c=baro_temp, lat_deg=0.0, lon_deg=0.0,
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
    ap.add_argument("--baro", choices=("none", "raven"), default="none",
                    help="barometer input: none (pressure 0, as in the flight) or the Raven's "
                         "recorded static pressure (needs --raven)")
    ap.add_argument("--baro-excursion-gain", type=float, default=1.0,
                    help="scale the short pressure excursions (Mach dip, ejection spike) about a "
                         "1.5 s running median, e.g. 3 = three times as deep")
    ap.add_argument("--baro-spike-cap", type=float, default=None,
                    help="cap overpressure above the running median at this many Pa (removes the "
                         "ejection spike)")
    ap.add_argument("--transonic-gain", type=float, default=1.0,
                    help="scale the Raven's transonic altitude error (0.3-4 s, taper to 7 s) by this "
                         "factor; needs --transonic-ref")
    ap.add_argument("--transonic-ref", type=Path,
                    help="IMU-only Kalman reference for --transonic-gain: a bench replay's P*.csv")
    ap.add_argument("--raven-shift", type=float,
                    help="seconds added to the Raven flight time to align it with the Witness IMU "
                         "(default: matched on the boost velocity, see raven_time_shift)")
    ap.add_argument("--label", default="", help="free text stored in replay_info.json")
    ap.add_argument("--pad-heading", metavar="AXIS:DEG",
                    help="send the pad heading command during the pad hold, e.g. +Y:183.6 "
                         "(body axis across the rocket and its azimuth, degrees from true north)")
    args = ap.parse_args()
    heading_frame = None
    if args.pad_heading:
        axis, deg = args.pad_heading.split(":")
        if axis not in PAD_HEADING_AXES:
            raise SystemExit(f"--pad-heading axis must be one of {sorted(PAD_HEADING_AXES)}")
        heading_frame = pad_heading_frame(axis, float(deg))
    expected = {int(k): float(v) for k, v in (x.split("=") for x in args.expected.split(","))}

    ft, data = load_imu(args.imu_csv, args.end)
    if args.calibration_from:
        cal = flight_calibration(args.calibration_from)
        data = apply_calibration(data, cal)
        print("applied the flight calibration: gyro bias %s dps, low-g offset %s g scale %s, "
              "high-g offset %s g scale %s" % tuple(np.round(c, 4).tolist() for c in cal))
    shift = 0.0
    if args.raven:
        shift = raven_time_shift(args.raven, ft, data) if args.raven_shift is None else args.raven_shift
        print(f"Raven time shift {shift:+.3f} s (Raven flight time + shift = Witness flight time)")
    baro = None
    if args.baro == "raven":
        if not args.raven:
            raise SystemExit("--baro raven needs --raven")
        baro = raven_pressure(args.raven, args.baro_excursion_gain, spike_cap_pa=args.baro_spike_cap,
                              shift_s=shift)
        if args.transonic_gain != 1.0:
            if not args.transonic_ref:
                raise SystemExit("--transonic-gain needs --transonic-ref")
            baro = (baro[0], amplify_transonic(baro[0], baro[1], args.transonic_gain, args.transonic_ref),
                    baro[2])
        print(f"barometer: Raven static pressure, excursion gain {args.baro_excursion_gain}, "
              f"spike cap {args.baro_spike_cap}; {baro[1].min():.0f}..{baro[1].max():.0f} Pa")
    run, lift_sim = build_run(ft, data, args.raven, args.site_alt, args.end, expected, baro, shift)
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
        if heading_frame is not None and session._sim_s() >= PAD_HEADING_SEND_SIM_S:
            session._send(heading_frame)
            print(f"  sent pad heading {args.pad_heading} at sim {session._sim_s():.1f} s "
                  f"({heading_frame.hex(' ')})", flush=True)
            heading_frame = None
        while True:
            try:
                kind, payload = session.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("log", "warning") and re.search(r"state|FC logs|ENTER|EXIT|attitude|saturat|reset|heading",
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
        {"liftoff_sim_s": lift_sim, "expected_flight_s": expected, "source": str(args.imu_csv),
         "pad_heading": args.pad_heading, "baro": args.baro,
         "baro_excursion_gain": args.baro_excursion_gain, "baro_spike_cap": args.baro_spike_cap,
         "transonic_gain": args.transonic_gain,
         "label": args.label}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
