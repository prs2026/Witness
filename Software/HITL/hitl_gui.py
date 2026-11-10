"""Witness hardware-in-the-loop simulator: shared helpers and headless runner.

The HITL bench GUI is the "HITL bench" tab of witnessOS
(Software/witnessOS/witnessos.py, widget in wos_hitl_tab.py). This module is
Qt-free: settings, vibration choice, prediction and the headless runner.

Headless:  python ../witnessOS/witnessos.py --hitl-headless                --preset "Nominal 10 g" --host prs2026hitl --port 7100
CSV:       python ../witnessOS/witnessos.py --hitl-headless --csv trajectories/x.csv
Alias:     python hitl_gui.py --headless ...   (deprecated; same runner)

The tool only ever transmits HITL ENTER (0x4849), HITL EXIT (0x4858), Witness
debug start (0x00E0, CSV runs only) and 0x0C HITL sensor frames. See README.md
for the safety rules.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import queue
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import attitude as att  # noqa: E402
import hitl_protocol as hp  # noqa: E402
import hitl_session as hs  # noqa: E402
import sim_model as sm  # noqa: E402
import trajectory_csv as tc  # noqa: E402

SETTINGS_PATH = os.path.join(HERE, "hitl_settings.json")
DEFAULT_HOST = "prs2026hitl"
DEFAULT_PORT = 7100
DEFAULT_TOLERANCE_S = 1.0
TRAJECTORY_DIR = os.path.join(HERE, "trajectories")
CSV_SOURCE = "CSV trajectory"
VIBRATION_DIR = os.path.join(HERE, "vibration")
VIB_AUTO = "Auto (profile for this trajectory)"
VIB_NONE = "None"


def vibration_profiles(directory: str = VIBRATION_DIR) -> list[str]:
    """Profile folder names (vibration/<name>/profile.json), sorted."""
    try:
        names = os.listdir(directory)
    except OSError:
        return []
    return sorted(name for name in names
                  if os.path.isfile(os.path.join(directory, name, "profile.json")))


def vibration_display(value) -> str:
    """CsvParams.vibration_profile -> picker text."""
    value = (value or "").strip()
    if not value:
        return VIB_AUTO
    if value.lower() == "none":
        return VIB_NONE
    return value


def vibration_value(display: str) -> str:
    """Picker text -> CsvParams.vibration_profile ("" auto, "none", folder)."""
    display = (display or "").strip()
    if display in ("", VIB_AUTO):
        return ""
    if display == VIB_NONE:
        return "none"
    return display


def describe_vibration(csv_path: str, choice: str, scale: float) -> str:
    """What a vibration choice resolves to for a trajectory (for the UI)."""
    try:
        import imu_synth as ims
        profile = ims.find_profile(csv_path, choice)
    except ImportError as exc:
        return f"Vibration: unknown ({exc.name} is not installed)"
    except (OSError, ValueError) as exc:
        return f"Vibration: {exc}"
    head = "Auto -> " if not (choice or "").strip() else ""
    if profile is None or not scale:
        why = ("scale 0" if profile is not None else
               "no profile for this trajectory" if head else "none selected")
        return f"Vibration: {head}none ({why})"
    return f"Vibration: {head}profile {profile['name']!r} x{scale:g}"


def vibration_used(run) -> str:
    """Which vibration a prepared run actually used (from its IMU report)."""
    report = getattr(run, "imu_report", None) or {}
    if not report:
        return "Last run: 960 Hz IMU synthesis off (no vibration)"
    profile = report.get("vibration_profile")
    scale = report.get("params", {}).get("vibration_scale", 1.0)
    if not profile:
        return "Last run used: no vibration profile"
    return f"Last run used: profile {profile['name']!r} x{scale:g}"


def compute_prediction(params) -> dict:
    """Expected transitions and the summary text for a preset or CSV.

    Pure (no Qt): the GUI runs it on a worker thread. Raises OSError /
    ValueError for a bad CSV.
    """
    report = ""
    vibration = ""
    if isinstance(params, tc.CsvParams):
        build = tc.build(params)
        samples = build.samples
        expected = hs.expected_from_track(samples, sm.predict_state_track(samples))
        report = "\n" + tc.format_report(build.report)
        if getattr(params, "imu_synthesis", False):
            vibration = describe_vibration(params.csv_path, params.vibration_profile,
                                           params.vibration_scale)
        else:
            vibration = "Vibration: off (imu_synthesis=false)"
    else:
        samples = sm.generate(params)
        expected = sm.predict_transitions(samples)
    s = sm.summarize(samples)
    text = (f"Duration {s['duration_s']:.1f} s, apogee "
            f"{s['apogee_agl_m']:.0f} m AGL at {s['apogee_t_s']:.1f} s\n"
            + "\n".join(f"{hp.FLIGHT_STATES[k]}: {v:.2f} s"
                        for k, v in sorted(expected.items(), key=lambda kv: kv[1]))
            + ("" if expected else "No transitions expected") + report)
    return {"text": text, "expected": expected, "vibration": vibration}


def load_settings(path: str = SETTINGS_PATH) -> dict:
    settings = default_settings()
    try:
        with open(path) as f:
            data = json.load(f)
        if isinstance(data, dict):
            settings.update({k: data[k] for k in settings if k in data})
    except (OSError, ValueError):
        pass
    return settings


def default_settings() -> dict:
    return {"host": DEFAULT_HOST, "port": DEFAULT_PORT,
                "preset": "Nominal 10 g", "tolerance_s": DEFAULT_TOLERANCE_S,
                "csv_path": "", "att_final_tilt_deg": att.DEFAULT_THRESHOLDS[
                    "final_tilt_max_deg"],
                # FC flash logs fetched through the bench Pi after each run.
                "fetch_fc_logs": True, "pi_host": "", "pi_user": "",
                "pi_key": "", "fetch_restart_hub": True,
                # CSV runs: vibration profile ("" auto, "none", or a folder
                # under vibration/) and its scale.
                "vibration_profile": "", "vibration_scale": 1.0}


def save_settings(settings: dict, path: str = SETTINGS_PATH):
    try:
        with open(path, "w") as f:
            json.dump(settings, f, indent=2)
    except OSError:
        pass


def parse_overrides(items, cls=sm.SimParams) -> dict:
    """--set key=value pairs -> typed overrides for a params dataclass."""
    types = {f.name: f.type for f in dataclasses.fields(cls)}
    out = {}
    for item in items or []:
        key, _, raw = item.partition("=")
        key = key.strip()
        if key not in types:
            raise SystemExit(f"unknown parameter {key!r}")
        out[key] = coerce(types[key], raw.strip())
    return out


def coerce(type_name, raw: str):
    t = str(type_name)
    if "None" in t and raw.lower() in ("", "none"):
        return None
    if t == "str":
        return raw
    if "bool" in t:
        return raw.lower() in ("1", "true", "yes", "on")
    if "int" in t and "float" not in t:
        return int(float(raw))
    return float(raw)


# ---------------------------------------------------------------------------
# Headless
# ---------------------------------------------------------------------------

def attitude_thresholds(args) -> dict:
    return {"final_tilt_max_deg": args.att_final_tilt,
            "final_att_max_deg": args.att_final_error,
            "flight_tilt_max_deg": args.att_flight_tilt}


def fc_log_config(settings: dict, enabled: bool = True) -> dict:
    """FC log fetch settings; env WITNESS_PI_USER / _KEY / _HOST fill gaps."""
    return {
        "enabled": enabled and bool(settings.get("fetch_fc_logs", True)),
        "pi_host": settings.get("pi_host") or os.environ.get("WITNESS_PI_HOST", ""),
        "pi_user": settings.get("pi_user") or os.environ.get("WITNESS_PI_USER", ""),
        "pi_key": settings.get("pi_key") or os.environ.get("WITNESS_PI_KEY", ""),
        "restart": bool(settings.get("fetch_restart_hub", True)),
    }


def run_headless(args) -> int:
    scale = max(int(args.time_scale), int(args.speed or 1))
    if scale != 1 and not hs.is_loopback(args.host):
        print("--speed is only allowed against the local fake Witness "
              "(127.0.0.1); hardware runs are real time", file=sys.stderr)
        return 2
    if args.csv:
        try:
            params = tc.CsvParams(csv_path=args.csv,
                                  **parse_overrides(args.set, tc.CsvParams))
            label = "CSV " + os.path.basename(args.csv)
        except (OSError, ValueError, TypeError) as exc:
            print(f"bad CSV run arguments: {exc}", file=sys.stderr)
            return 2
    else:
        try:
            params = sm.preset_params(args.preset)
        except KeyError as exc:
            print(exc.args[0], file=sys.stderr)
            return 2
        params = dataclasses.replace(params, **parse_overrides(args.set))
        label = args.preset
    runs_dir = None if args.no_save else (args.runs_dir or hs.DEFAULT_RUNS_DIR)
    session = hs.HitlSession(runs_dir=runs_dir)
    session.fc_log_config = fc_log_config(load_settings(), not args.no_fetch_logs)
    try:
        run = session.prepare_run(params, preset=label,
                                  tolerance_s=args.tolerance,
                                  time_scale=scale,
                                  att_thresholds=attitude_thresholds(args),
                                  att_latency_ms=args.att_latency_ms)
    except (OSError, ValueError) as exc:
        print(f"cannot build run: {exc}", file=sys.stderr)
        return 2
    summary = sm.summarize(run.samples)
    print(f"{'Source' if args.csv else 'Preset'} {label!r}: "
          f"{summary['duration_s']:.1f} s sim, "
          f"apogee {summary['apogee_agl_m']:.0f} m AGL at "
          f"{summary['apogee_t_s']:.1f} s")
    if run.csv_report:
        print(tc.format_report(run.csv_report))
        for line in hs.HitlSession.imu_summary(run):
            print(line)
    print("Expected: " + (", ".join(
        f"{hp.FLIGHT_STATES[k]} {v:.2f} s"
        for k, v in sorted(run.expected.items(), key=lambda kv: kv[1]))
        or "no transitions"))
    if args.dry_run:
        return 0
    try:
        session.connect(args.host, args.port, timeout=args.connect_timeout)
    except OSError as exc:
        print(f"connect to {args.host}:{args.port} failed: {exc}",
              file=sys.stderr)
        return 2

    def drain():
        while True:
            try:
                kind, payload = session.events.get_nowait()
            except queue.Empty:
                return
            if args.verbose and kind in ("log", "text"):
                print(payload)
            elif kind == "warning":
                print("WARNING:", payload)

    # Give Witness a moment to report its current state before starting.
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline and session.view.flight_state is None:
        time.sleep(0.05)
    drain()
    try:
        try:
            session.start_run(run, abort_on_no_hitl=True)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            session.disconnect()
            return 2
        while not session.wait_run(0.2):
            drain()
            # The FC log fetch closes the connection on purpose after the run.
            if not session.connected and run.streaming:
                break
    except KeyboardInterrupt:
        session.stop_run("interrupted")
    finally:
        session.wait_run(600 if session.fc_log_config.get("enabled") else 5)
        drain()
        session.disconnect(send_exit=False)
    result = run.result or hs.evaluate(run)
    print(hs.format_table(result))
    for line in result_lines(run, result):
        print(line)
    return 0 if result["passed"] else 1


def result_lines(run, result: dict) -> list[str]:
    """Run summary after the transition table: IMU frames, the observed vs
    truth comparison, FC logs and the run folder (headless and GUI)."""
    lines = []
    if run.imu_report:
        lines.append(f"960 Hz IMU: {run.imu_frames_sent} 0x0E frames sent "
                     f"({run.imu_frames_sent * 8} samples)")
    for key, title in (("comparison", "Witness 0xE0 (10 Hz) vs truth"),
                       ("comparison_flash", "Flash state packets (50 Hz) vs truth")):
        if result.get(key):
            lines.append(title + ":")
            for phase, d in result[key].items():
                att_txt = (f", attitude {d['attitude_rms_deg']:.2f} deg rms"
                           if d.get("attitude_rms_deg") is not None else "")
                lines.append(
                    f"  {phase:<9} up {d['up_rms_m']:.1f} m rms (max {d['up_max_m']:.1f}), "
                    f"horizontal {d['horizontal_rms_m']:.1f} m rms, "
                    f"speed {d['speed_rms_mps']:.1f} m/s rms{att_txt}")
    fc = result.get("fc_logs")
    if fc:
        lines.append(f"FC logs: {', '.join(fc['files']) if fc.get('ok') else 'failed: ' + str(fc.get('error'))}")
    if run.run_dir:
        lines.append(f"Run saved to {run.run_dir}")
    return lines


# Tick helpers kept from the Tk canvas plot (pure; pyqtgraph does its own).

def _nice_ticks(lo: float, hi: float, target: int = 5) -> list:
    """Round tick values covering [lo, hi] with about ``target`` intervals."""
    import math
    span = hi - lo
    if not math.isfinite(span) or span <= 0:
        return [lo, lo + 1.0]
    rough = span / max(1, target)
    mag = 10.0 ** math.floor(math.log10(rough))
    step = next(m for m in (1, 2, 5, 10) if rough / mag <= m) * mag
    i0, i1 = math.floor(lo / step + 1e-9), math.ceil(hi / step - 1e-9)
    if i1 - i0 > 200:
        i1 = i0 + 200
    return [i * step for i in range(i0, i1 + 1)]


def _tick_label(v: float, step: float) -> str:
    import math
    if abs(v) < step * 1e-6:
        return "0"
    if step >= 1:
        return f"{v:.0f}"
    decimals = min(6, max(0, -math.floor(math.log10(step))))
    return f"{v:.{decimals}f}"


def build_parser():
    s = load_settings()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--headless", action="store_true",
                    help="run one simulation (the GUI is witnessOS's HITL tab)")
    ap.add_argument("--host", default=s["host"])
    ap.add_argument("--port", type=int, default=s["port"])
    ap.add_argument("--preset", default="Nominal 10 g")
    ap.add_argument("--csv", metavar="PATH",
                    help="drive the run from a trajectory CSV instead of a "
                    "preset (--set then applies to CSV parameters)")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="override a model parameter (repeatable)")
    ap.add_argument("--tolerance", type=float, default=s["tolerance_s"],
                    help="max |Witness - expected| transition delta, s")
    ap.add_argument("--time-scale", type=int, default=1,
                    help=argparse.SUPPRESS)  # fake-Witness tests only
    ap.add_argument("--speed", type=int, default=1, metavar="N",
                    help="send N sim ticks per 10 ms; ONLY for the local fake "
                    "Witness (refused for non-loopback hosts)")
    ap.add_argument("--dry-run", action="store_true",
                    help="build the frames, print the prediction and exit "
                    "without connecting")
    ap.add_argument("--att-final-tilt", type=float,
                    default=att.DEFAULT_THRESHOLDS["final_tilt_max_deg"],
                    metavar="DEG", help="max final tilt error (CSV runs)")
    ap.add_argument("--att-final-error", type=float, default=None,
                    metavar="DEG", help="max final attitude error with the "
                    "heading offset removed (CSV runs; default off)")
    ap.add_argument("--att-flight-tilt", type=float, default=None,
                    metavar="DEG", help="max tilt error during Boost..Descent "
                    "(CSV runs; default off)")
    ap.add_argument("--att-latency-ms", type=int, default=0,
                    help="compare Witness attitude with the truth this many "
                    "ms before the last frame sent")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to wait for telemetry before starting")
    ap.add_argument("--connect-timeout", type=float, default=5.0)
    ap.add_argument("--runs-dir", default=None)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--no-fetch-logs", action="store_true",
                    help="do not download the FC flash logs through the bench Pi "
                         "after the run (default: fetch, then power-cycle Pi USB hub 2)")
    ap.add_argument("--list-presets", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_presets:
        for name in sm.PRESETS:
            exp = sm.predict_transitions(sm.generate(sm.preset_params(name)))
            print(f"{name:<24}" + ", ".join(
                f"{hp.FLIGHT_STATES[k]} {v:.2f}s" for k, v in exp.items()))
        return 0
    if args.headless:
        return run_headless(args)
    print("The HITL bench GUI is now the \"HITL bench\" tab of witnessOS: "
          "python Software/witnessOS/witnessos.py --tab hitl", file=sys.stderr)
    return 2


DEPRECATION_NOTE = ("hitl_gui.py is deprecated; use "
                    "python Software/witnessOS/witnessos.py --hitl-headless ...")

if __name__ == "__main__":
    print(DEPRECATION_NOTE, file=sys.stderr)
    sys.exit(main())
