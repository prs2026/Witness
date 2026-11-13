"""Run outputs for observed-vs-expected comparison, and FC log fetching.

Files written into a run directory (runs/<timestamp>/):

  truth.csv            expected values per 100 Hz tick: phase and expected
                       Witness state, truth position/velocity (ENU from the pad),
                       altitude, body specific force and rate, attitude
  comparison.csv       each Witness 0xE0 packet (10 Hz) with the truth at the
                       same simulation time: observed vs expected position,
                       velocity, altitude, attitude, flight state, and errors
  imu960.npz           the 960 Hz readings sent as 0x0E frames (counts and
                       scales) plus the synthesis report
  fc_logs/             Witness flash files of the run's session (through the
                       bench Pi), converted to CSV
  comparison_flash.csv the 50 Hz state packets from the flash log (P*.BIN)
                       matched to truth (uptime mapped to sim time from 0xE0)

summary.json gains "comparison": RMS and max errors per expected phase.
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time

import attitude as att
import hitl_protocol as hp

HERE = os.path.dirname(os.path.abspath(__file__))
HORIZON_DIR = os.path.normpath(os.path.join(HERE, "..", "Horizon"))

def state_name(state) -> str:
    if state is None:
        return ""
    try:
        return hp.FLIGHT_STATES[int(state)]
    except (IndexError, ValueError, TypeError):
        return str(state)


def _tick_index(samples, sim_s: float) -> int:
    k = int(round(sim_s * 100.0))
    return max(0, min(len(samples) - 1, k))


def truth_pos_vel(samples, s):
    """Truth ENU position and velocity of a sample. CSV samples carry them;
    preset samples (vertical flight model) have altitude and vertical speed
    only, so position is (0, 0, altitude above the first sample)."""
    if hasattr(s, "pos_enu_m"):
        return s.pos_enu_m, s.vel_enu_mps
    return (0.0, 0.0, s.alt_msl_m - samples[0].alt_msl_m), (0.0, 0.0, s.vel_mps)


def write_truth_csv(path: str, samples, state_track) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sim_s", "phase", "expected_state", "east_m", "north_m",
                    "up_m", "alt_msl_m", "vel_east_mps", "vel_north_mps",
                    "vel_up_mps", "speed_mps", "f_body_x_g", "f_body_y_g",
                    "f_body_z_g", "w_body_x_dps", "w_body_y_dps", "w_body_z_dps",
                    "q_x", "q_y", "q_z", "q_w", "tilt_deg"])
        for k, s in enumerate(samples):
            pos, vel = truth_pos_vel(samples, s)
            fb = getattr(s, "f_body_g", (0.0, 0.0, 0.0))
            wb = getattr(s, "w_body_dps", (0.0, 0.0, 0.0))
            q = getattr(s, "q_truth", att.IDENTITY)
            exp = state_track[k] if state_track else None
            w.writerow([f"{s.t_ms / 1000:.2f}", s.phase,
                        state_name(exp),
                        *(f"{c:.3f}" for c in pos), f"{s.alt_msl_m:.3f}",
                        *(f"{c:.3f}" for c in vel), f"{math.sqrt(sum(c * c for c in vel)):.3f}",
                        *(f"{c:.5f}" for c in fb), *(f"{c:.4f}" for c in wb),
                        *(f"{c:.7f}" for c in q),
                        f"{att.tilt_from_vertical_deg(q):.3f}"])


def _att_error_deg(q_truth, q_witness) -> float | None:
    if q_witness is None or all(c == 0.0 for c in q_witness):
        return None
    d = att.qmul(att.qconj(q_truth), att.qnormalize(q_witness))
    return math.degrees(2.0 * math.acos(min(1.0, abs(d[3]))))


COMPARISON_HEADER = [
    "sim_s", "uptime_ms", "witness_state", "expected_state",
    "witness_east_m", "truth_east_m", "witness_north_m", "truth_north_m",
    "witness_up_m", "truth_up_m", "error_east_m", "error_north_m", "error_up_m",
    "error_horizontal_m", "witness_vel_east_mps", "truth_vel_east_mps",
    "witness_vel_north_mps", "truth_vel_north_mps", "witness_vel_up_mps",
    "truth_vel_up_mps", "error_speed_mps", "witness_kalman_alt_m",
    "truth_alt_msl_m", "witness_tilt_deg", "truth_tilt_deg", "error_attitude_deg",
]


def _compare_row(samples, state_track, sim_s, uptime_ms, state, pos, vel, quat, kalman_alt):
    k = _tick_index(samples, sim_s)
    s = samples[k]
    tp, tv = truth_pos_vel(samples, s)
    has_att = hasattr(s, "q_truth")      # preset runs have no truth attitude
    tq = getattr(s, "q_truth", att.IDENTITY)
    err = [pos[i] - tp[i] for i in range(3)]
    speed_err = math.sqrt(sum(c * c for c in vel)) - math.sqrt(sum(c * c for c in tv))
    exp = state_track[k] if state_track else None
    w_tilt = (att.tilt_from_vertical_deg(att.qnormalize(quat))
              if quat and any(quat) else None)
    return {
        "sim_s": sim_s, "uptime_ms": uptime_ms,
        "witness_state": state_name(state),
        "expected_state": state_name(exp),
        "witness_east_m": pos[0], "truth_east_m": tp[0],
        "witness_north_m": pos[1], "truth_north_m": tp[1],
        "witness_up_m": pos[2], "truth_up_m": tp[2],
        "error_east_m": err[0], "error_north_m": err[1], "error_up_m": err[2],
        "error_horizontal_m": math.hypot(err[0], err[1]),
        "witness_vel_east_mps": vel[0], "truth_vel_east_mps": tv[0],
        "witness_vel_north_mps": vel[1], "truth_vel_north_mps": tv[1],
        "witness_vel_up_mps": vel[2], "truth_vel_up_mps": tv[2],
        "error_speed_mps": speed_err,
        "witness_kalman_alt_m": kalman_alt, "truth_alt_msl_m": s.alt_msl_m,
        "witness_tilt_deg": w_tilt,
        "truth_tilt_deg": att.tilt_from_vertical_deg(tq) if has_att else None,
        "error_attitude_deg": _att_error_deg(tq, quat) if quat and has_att else None,
    }


def _write_rows(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COMPARISON_HEADER)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if v is None else (f"{v:.4f}" if isinstance(v, float) else v))
                        for k, v in r.items()})


def summarize(rows) -> dict:
    """RMS and max of the errors per expected phase."""
    out = {}
    for r in rows:
        d = out.setdefault(r["expected_state"] or "unknown",
                           {"n": 0, "up2": 0.0, "h2": 0.0, "v2": 0.0, "a2": 0.0, "na": 0,
                            "up_max": 0.0, "h_max": 0.0, "v_max": 0.0, "a_max": 0.0})
        d["n"] += 1
        for key, src in (("up", "error_up_m"), ("h", "error_horizontal_m"),
                         ("v", "error_speed_mps")):
            e = abs(r[src])
            d[key + "2"] += e * e
            d[key + "_max"] = max(d[key + "_max"], e)
        if r["error_attitude_deg"] is not None:
            d["na"] += 1
            d["a2"] += r["error_attitude_deg"] ** 2
            d["a_max"] = max(d["a_max"], r["error_attitude_deg"])
    result = {}
    for phase, d in out.items():
        n = d["n"]
        result[phase] = {
            "packets": n,
            "up_rms_m": math.sqrt(d["up2"] / n), "up_max_m": d["up_max"],
            "horizontal_rms_m": math.sqrt(d["h2"] / n), "horizontal_max_m": d["h_max"],
            "speed_rms_mps": math.sqrt(d["v2"] / n), "speed_max_mps": d["v_max"],
            "attitude_rms_deg": math.sqrt(d["a2"] / d["na"]) if d["na"] else None,
            "attitude_max_deg": d["a_max"] if d["na"] else None,
        }
    return result


def write_comparison(run, run_dir: str) -> dict | None:
    """truth.csv, comparison.csv and imu960.npz; returns the phase summary."""
    write_truth_csv(os.path.join(run_dir, "truth.csv"), run.samples, run.state_track)
    if run.imu_stream is not None:
        import numpy as np
        s = run.imu_stream
        np.savez_compressed(
            os.path.join(run_dir, "imu960.npz"), time_s=s.time_s, low_g_counts=s.low_g,
            high_g_counts=s.high_g, gyro_counts=s.gyro,
            low_g_per_count=0.488e-3, high_g_per_count=10.417e-3, gyro_dps_per_count=0.070)
    rows = []
    for sim_s, pkt in run.debug_records:
        if "position_m" not in pkt:
            continue
        rows.append(_compare_row(run.samples, run.state_track, sim_s, pkt.get("uptime_ms"),
                                 pkt["status"]["flight_state"], pkt["position_m"],
                                 pkt["velocity_mps"], pkt.get("quat_xyzw"),
                                 pkt.get("kalman_alt_m")))
    if not rows:
        return None
    _write_rows(os.path.join(run_dir, "comparison.csv"), rows)
    return summarize(rows)


# ---------------------------------------------------------------------------
# FC logs through the bench Pi
# ---------------------------------------------------------------------------

def fetch_fc_logs(session, run, config: dict, log, activity=None) -> dict:
    """Exports Witness's flash logs and downloads this run's session.

    Sends the export command over the HITL connection, closes it, then reads
    the USB disk on the bench Pi with Horizon's witness_download. Optionally
    power-cycles the Pi's USB hub 2 afterwards so Witness logs again (this
    resets every board on that hub). Never raises; returns a status dict.
    """
    status = {"ok": False, "files": [], "dir": None, "error": None}
    out_dir = os.path.join(run.run_dir, "fc_logs")
    step = activity or (lambda _text: None)
    try:
        if HORIZON_DIR not in sys.path:
            sys.path.insert(0, HORIZON_DIR)
        import witness_download as wd
        pi = wd.Pi(config.get("pi_host") or session._host, config.get("pi_user") or None,
                   config.get("pi_key") or None)
        step("FC logs: exporting Witness flash")
        # Witness only exports in Pad Idle or Landed (USB; refused in flight
        # states), and after a HITL flight it is usually in Descent: set
        # Landed first (USB may always move a flight state to Landed).
        log("FC logs: setting Witness to Landed (05 73 04), then exporting flash (05 68 68)")
        session._send(bytes((hp.ID_COMMAND, 0x73, hp.LANDED)))
        time.sleep(0.3)
        # Compact 3-byte form; the export command is not in the GUI allowlist.
        session._send(bytes((hp.ID_COMMAND,)) + hp.CMD_MASS_STORAGE.to_bytes(2, "big"))
        time.sleep(0.5)
        session.disconnect(send_exit=False)
        deadline = time.monotonic() + float(config.get("wait_s", 40))
        step("FC logs: waiting for the log disk on the Pi")
        while True:
            code, out, err = pi.agent_text("find")
            if code == 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("the Witness log disk did not appear on the Pi")
            time.sleep(2)
        files = []
        for _ in range(5):  # udev may still be applying the disk permissions
            try:
                files = wd.list_files(pi)
                break
            except wd.DownloadError:
                time.sleep(1)
        if not files:
            raise RuntimeError("no files listed on the Witness log disk")
        newest = max(wd.generation(f["name"]) for f in files)
        wanted = [f for f in files if wd.generation(f["name"]) == newest]
        os.makedirs(out_dir, exist_ok=True)
        for i, f in enumerate(wanted, 1):
            step(f"FC logs: downloading {f['name']} ({i}/{len(wanted)}, {f['size'] / 1e6:.1f} MB)")
            path = os.path.join(out_dir, f["name"])
            wd.download(pi, f["name"], f["size"], __import__("pathlib").Path(path))
            status["files"].append(f["name"])
            converted = wd.convert(__import__("pathlib").Path(path))
            if converted is not None:
                status["files"].append(converted.name)
        status["ok"] = True
        status["dir"] = out_dir
        log(f"FC logs: {len(wanted)} file(s) from session {newest:X} in {out_dir}")
    except Exception as exc:  # never lose the run over the logs
        status["error"] = f"{type(exc).__name__}: {exc}"
        log(f"FC log fetch failed: {status['error']}")
    finally:
        if config.get("restart", True):
            step("FC logs: power-cycling Pi USB hub 2")
            try:
                import witness_download as wd
                pi = wd.Pi(config.get("pi_host") or session._host,
                           config.get("pi_user") or None, config.get("pi_key") or None)
                import subprocess
                r = subprocess.run(pi.base + [pi.target, wd.HUB_RESTART],
                                   capture_output=True, text=True)
                log("FC logs: Pi USB hub 2 power-cycled; Witness reboots to normal"
                    if r.returncode == 0 else
                    f"FC logs: hub restart failed ({r.stderr.strip()}); replug Witness")
            except Exception as exc:
                log(f"FC logs: hub restart failed: {exc}")
    if status["ok"]:
        step("Comparing flash log with truth")
        try:
            flash = write_flash_comparison(run, out_dir)
            if flash:
                status["flash_comparison"] = flash
        except Exception as exc:
            log(f"flash comparison failed: {exc!r}")
    return status


def write_flash_comparison(run, fc_dir: str) -> dict | None:
    """comparison_flash.csv from the 50 Hz state packets in P*.csv."""
    p_csv = next((os.path.join(fc_dir, n) for n in sorted(os.listdir(fc_dir))
                  if n.upper().startswith("P") and n.lower().endswith(".csv")), None)
    if p_csv is None or not run.debug_records:
        return None
    # Uptime -> sim time: median offset over the live 0xE0 packets.
    offsets = sorted(sim_s - pkt["uptime_ms"] / 1000.0
                     for sim_s, pkt in run.debug_records if "uptime_ms" in pkt)
    if not offsets:
        return None
    offset = offsets[len(offsets) // 2]
    rows = []
    with open(p_csv, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("log_source") != "witness_state" or not r.get("uptime"):
                continue
            if r.get("hitl_active") not in ("1", "1.0", "True"):
                continue  # real-sensor packets before/after the HITL run
            sim_s = int(r["uptime"]) / 1000.0 + offset
            if sim_s < 0 or sim_s > run.samples[-1].t_ms / 1000.0:
                continue

            def num(key):
                v = r.get(key, "")
                return float(v) if v not in ("", None) else 0.0
            pos = (num("position_x"), num("position_y"), num("position_z"))
            vel = (num("velocity_x"), num("velocity_y"), num("velocity_z"))
            quat = (num("orientation_x"), num("orientation_y"),
                    num("orientation_z"), num("orientation_w"))
            state = int(float(r["flight_state"])) if r.get("flight_state") else 0
            rows.append(_compare_row(run.samples, run.state_track, sim_s, int(r["uptime"]),
                                     state, pos, vel, quat,
                                     num("kalman_altitude") if r.get("kalman_altitude")
                                     not in ("", None) else num("barometric_altitude")))
    if not rows:
        return None
    _write_rows(os.path.join(run.run_dir, "comparison_flash.csv"), rows)
    return summarize(rows)


def regenerate(run_dir: str) -> dict:
    """Rebuilds truth/comparison outputs for a saved CSV run.

    Uses params.json (to rebuild the same trajectory and IMU stream) and the
    0xE0 packets recorded in received.jsonl. Usage:
        python hitl_outputs.py runs/<timestamp>
    """
    import json
    import types
    import sim_model as sm
    import trajectory_csv as tc
    with open(os.path.join(run_dir, "params.json")) as f:
        saved = json.load(f)
    params = tc.CsvParams.from_dict(saved["params"])
    build = tc.build(params)
    run = types.SimpleNamespace(
        samples=build.samples, state_track=sm.predict_state_track(build.samples),
        run_dir=run_dir, debug_records=[], imu_stream=None)
    if getattr(params, "imu_synthesis", False):
        try:
            import imu_synth as ims
            prof = ims.find_profile(params.csv_path, getattr(params, "vibration_profile", ""))
            run.imu_stream = ims.from_samples(build.samples, ims.ImuSynthParams(
                seed=params.seed, vibration_scale=params.vibration_scale,
                noise_scale=params.imu_noise_scale, offset_scale=params.imu_offset_scale,
                vibration_thrust_path=prof["thrust_path"] if prof else "",
                vibration_lateral_path=prof["lateral_path"] if prof else "",
                vibration_full_scale_g=prof["full_scale_g"] if prof else 150.67))
        except ImportError:
            pass
    with open(os.path.join(run_dir, "received.jsonl")) as f:
        for line in f:
            pkt = json.loads(line)
            if pkt.get("id") == hp.ID_WITNESS_DEBUG and                     pkt.get("status", {}).get("hitl_active"):
                run.debug_records.append((pkt["sim_ms"] / 1000.0, pkt))
    out = {"comparison": write_comparison(run, run_dir)}
    fc_dir = os.path.join(run_dir, "fc_logs")
    if os.path.isdir(fc_dir):
        out["comparison_flash"] = write_flash_comparison(run, fc_dir)
    summary_path = os.path.join(run_dir, "summary.json")
    if os.path.exists(summary_path):
        with open(summary_path) as f:
            summary = json.load(f)
        summary.update({k: v for k, v in out.items() if v})
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)
    return out


if __name__ == "__main__":
    result = regenerate(sys.argv[1])
    for key, phases in result.items():
        if not phases:
            continue
        print(key + ":")
        for phase, d in phases.items():
            att_txt = (f", attitude {d['attitude_rms_deg']:.2f} deg rms"
                       if d.get("attitude_rms_deg") is not None else "")
            print(f"  {phase:<9} up {d['up_rms_m']:.1f} m rms (max {d['up_max_m']:.1f}), "
                  f"horizontal {d['horizontal_rms_m']:.1f} m rms (max {d['horizontal_max_m']:.1f}), "
                  f"speed {d['speed_rms_mps']:.1f} m/s rms{att_txt}")
