"""Connection, 100 Hz sender, telemetry tracking and run recording.

Shared by the Tkinter GUI and the headless CLI. All network I/O happens on
background threads; consumers read ``snapshot()`` and drain ``events``.
"""

from __future__ import annotations

import csv
import json
import math
import os
import queue
import socket
import threading
import time
from dataclasses import dataclass, field

import attitude as att
import hitl_protocol as hp
import sim_model as sm
import trajectory_csv as tc

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RUNS_DIR = os.path.join(HERE, "runs")
HITL_ACTIVE_TIMEOUT_S = 2.0
STATE_CONFIRM_PACKETS = 2
BACKWARD_CONFIRM_PACKETS = 5
TRANSITION_STATES = (hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED)
LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "::1")


def is_loopback(host: str | None) -> bool:
    host = (host or "").strip().lower()
    return host in LOOPBACK_HOSTS or host.startswith("127.")


def expected_from_track(samples, track) -> dict:
    """{state: sim s of entry} from a per-tick expected-state track."""
    out = {}
    prev = hp.PAD_IDLE
    for s, st in zip(samples, track):
        if st != prev:
            out.setdefault(st, s.t_ms / 1000.0)
            prev = st
    return out


@dataclass
class WitnessView:
    connected: bool = False
    rx_bytes: int = 0
    tx_bytes: int = 0
    last_status: dict | None = None
    last_status_time: float = 0.0
    flight_state: int | None = None
    hitl_active: bool = False
    gps_locked: bool = False
    gps_sats: int | None = None
    baro_alt_m: float | None = None
    gps_alt_m: float | None = None
    battery_mv: int | None = None
    accel_x_g: float | None = None
    uptime_ms: int | None = None
    echo_frames: int = 0
    nmea_lines: int = 0


@dataclass
class RunState:
    params: object               # sm.SimParams or tc.CsvParams
    preset: str
    samples: list
    frames: list
    expected: dict
    tolerance_s: float
    time_scale: int = 1
    started_wall: float = 0.0
    sim_ms: int = 0
    frames_sent: int = 0
    late_ticks: int = 0
    running: bool = False
    streaming: bool = False  # 0x0C frames still being sent
    finished: bool = False
    aborted: bool = False
    abort_reason: str = ""
    hitl_seen: bool = False
    hitl_dropped: bool = False
    initial_state: int | None = None
    observed: dict = field(default_factory=dict)     # state -> sim s
    sequence: list = field(default_factory=list)     # confirmed states
    warnings: list = field(default_factory=list)
    plot_true: list = field(default_factory=list)    # (sim s, alt msl)
    plot_accel: list = field(default_factory=list)   # (sim s, sent low-g x)
    plot_baro: list = field(default_factory=list)    # (sim s, Witness baro)
    plot_wacc: list = field(default_factory=list)    # (sim s, Witness acc x)
    run_dir: str | None = None
    result: dict | None = None
    # CSV trajectory runs: truth attitude and Witness attitude comparison.
    source: str = "preset"
    csv_report: dict | None = None
    truth_quats: list | None = None
    state_track: list | None = None   # expected Witness state per tick
    request_debug: bool = False       # send 0x00E0 once HITL is active
    debug_sent: bool = False
    att_thresholds: dict = field(default_factory=dict)
    att_latency_ms: int = 0
    att_records: list = field(default_factory=list)
    att_psi: float | None = None      # heading offset, rad
    att_psi_source: str = ""
    plot_tilt_truth: list = field(default_factory=list)  # (sim s, deg)
    plot_tilt_w: list = field(default_factory=list)
    plot_tilt_err: list = field(default_factory=list)
    plot_att_err: list = field(default_factory=list)
    attitude: dict | None = None      # attitude summary at the end
    # 960 Hz IMU synthesis (CSV runs): 0x0E frames sent by simulation time.
    imu_frames: list = field(default_factory=list)
    imu_due_ms: list = field(default_factory=list)   # sim ms of each frame's last sample
    imu_stream: object = None
    imu_report: dict | None = None
    imu_frames_sent: int = 0
    # Witness 0xE0 estimates for the observed-vs-expected comparison.
    debug_records: list = field(default_factory=list)  # (sim s, packet)
    comparison: dict | None = None


class RunRecorder:
    """Writes runs/<timestamp>/ files. Thread-safe."""

    def __init__(self, base_dir: str, run: RunState):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(base_dir, stamp)
        k = 1
        while os.path.exists(path):
            k += 1
            path = os.path.join(base_dir, f"{stamp}-{k}")
        os.makedirs(path)
        self.dir = path
        self._lock = threading.Lock()
        with open(os.path.join(path, "params.json"), "w") as f:
            json.dump({"preset": run.preset, "params": run.params.to_dict(),
                       "tolerance_s": run.tolerance_s,
                       "time_scale": run.time_scale,
                       "expected_transitions_s": {
                           hp.FLIGHT_STATES[k]: v
                           for k, v in run.expected.items()},
                       "model_summary": sm.summarize(run.samples),
                       "source": run.source,
                       "csv_report": run.csv_report,
                       "attitude_thresholds": run.att_thresholds or None},
                      f, indent=2)
        self._sent = open(os.path.join(path, "sent.csv"), "w", newline="")
        self._sent_w = csv.writer(self._sent)
        self._sent_w.writerow(["wall_s", "sim_ms", "phase", "alt_msl_m",
                               "vel_mps", "low_g_x", "high_g_x", "gyro_x",
                               "pressure_pa", "gps_sats", "gps_fix",
                               "battery_mv", "low_g_y", "low_g_z",
                               "high_g_y", "high_g_z", "gyro_y", "gyro_z",
                               "q_truth_x", "q_truth_y", "q_truth_z",
                               "q_truth_w"])
        self._rx = open(os.path.join(path, "received.jsonl"), "w")
        self._log = open(os.path.join(path, "log.txt"), "w")
        self._nmea = open(os.path.join(path, "nmea.txt"), "w")

    def sent(self, wall_s, s: sm.SimSample):
        h = s.hitl
        qt = getattr(s, "q_truth", None)
        q = ["", "", "", ""] if qt is None else [f"{c:.7f}" for c in qt]
        with self._lock:
            if self._sent.closed:
                return
            self._sent_w.writerow([
                f"{wall_s:.4f}", s.t_ms, s.phase, f"{s.alt_msl_m:.2f}",
                f"{s.vel_mps:.2f}", f"{h.low_g[0]:.4f}", f"{h.high_g[0]:.4f}",
                f"{h.gyro_dps[0]:.3f}", f"{h.pressure_pa:.1f}", h.satellites,
                h.fix_quality, f"{h.battery_mv:.0f}", f"{h.low_g[1]:.4f}",
                f"{h.low_g[2]:.4f}", f"{h.high_g[1]:.4f}",
                f"{h.high_g[2]:.4f}", f"{h.gyro_dps[1]:.3f}",
                f"{h.gyro_dps[2]:.3f}", *q])

    def packet(self, wall_s, sim_ms, pkt):
        with self._lock:
            if not self._rx.closed:
                self._rx.write(json.dumps(
                    {"wall_s": round(wall_s, 4), "sim_ms": sim_ms, **pkt})
                    + "\n")

    def text(self, wall_s, sim_ms, line, nmea=False):
        with self._lock:
            f = self._nmea if nmea else self._log
            if not f.closed:
                f.write(f"{wall_s:10.3f} sim={sim_ms / 1000:8.2f} {line}\n")

    def finish(self, result: dict, att_records=None):
        with self._lock:
            for f in (self._sent, self._rx, self._log, self._nmea):
                f.close()
        if att_records is not None:
            write_attitude_csv(os.path.join(self.dir, "attitude.csv"),
                               att_records)
        with open(os.path.join(self.dir, "transitions.csv"), "w",
                  newline="") as f:
            w = csv.writer(f)
            w.writerow(["state", "expected_s", "witness_s", "delta_s",
                        "result"])
            for r in result["rows"]:
                w.writerow([r["state"], r["expected_s"], r["witness_s"],
                            r["delta_s"], r["result"]])
        with open(os.path.join(self.dir, "summary.json"), "w") as f:
            json.dump(result, f, indent=2)


# Column layout loaded directly by Horizon's Attitude tab (replay). The
# first 12 columns are fixed; the rest are extra diagnostics.
ATT_CSV_HEADER = ("time_s", "packet_id", "witness_qx", "witness_qy",
                  "witness_qz", "witness_qw", "truth_qx", "truth_qy",
                  "truth_qz", "truth_qw", "flight_state", "hitl_active")
ATT_CSV_EXTRA = ("wall_s", "packet", "flight_state_name", "expected_state",
                 "valid", "truth_tilt_deg", "witness_tilt_deg",
                 "tilt_err_deg", "nose_err_deg", "att_err_deg")


def _cell(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return f"{v:.4f}"
    return v


def write_attitude_csv(path, records):
    """One row per 0x02/0xE0 packet: Witness vs truth quaternion.

    time_s is sim time (s) of the truth sample paired with the packet.
    packet_id is decimal (2 = 0x02, 224 = 0xE0). witness_q* is the raw
    reported quaternion (all zero while unavailable). truth_q* uses the same
    convention: Hamilton, BODY -> ENU WORLD, body +X = nose, x,y,z,w.
    flight_state is the Witness state code 0-4 (empty if not yet known);
    hitl_active is the status bit of that packet (0/1).
    """
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(ATT_CSV_HEADER) + list(ATT_CSV_EXTRA))
        for r in records:
            qw = r["q_witness_raw"] if r["valid"] else (0.0, 0.0, 0.0, 0.0)
            st = r.get("witness_state_code")
            row = [f"{r['sim_s']:.3f}", r["packet_id"]]
            row += [f"{c:.7f}" for c in qw]
            row += [f"{c:.7f}" for c in r["q_truth"]]
            row += ["" if st is None else st, int(bool(r.get("hitl_active")))]
            row += [_cell(r.get(k)) for k in (
                "wall_s", "packet", "witness_state", "phase", "valid",
                "truth_tilt_deg", "witness_tilt_deg", "tilt_err_deg",
                "nose_err_deg", "att_err_deg")]
            w.writerow(row)


def finish_attitude(run: RunState):
    """Lock a heading reference if none was found, then summarise."""
    if run.truth_quats is None:
        return None
    recs = run.att_records
    psi, source = run.att_psi, run.att_psi_source
    if psi is None:
        first = next((r for r in recs if r["valid"]), None)
        if first is not None:
            psi = att.heading_offset_rad(first["q_truth"], first["q_witness"])
            source = (f"first valid packet, sim {first['sim_s']:.2f} s "
                      "(Boost not seen with valid quaternions)")
    att.compute_records(recs, psi)
    run.attitude = att.summarize(recs, psi, run.att_thresholds, source)
    return run.attitude


def evaluate(run: RunState) -> dict:
    rows = []
    passed = True
    for st in TRANSITION_STATES:
        exp = run.expected.get(st)
        obs = run.observed.get(st)
        delta = None if exp is None or obs is None else obs - exp
        if exp is None and obs is None:
            continue
        if exp is None:
            res = "UNEXPECTED"
        elif obs is None:
            res = "MISSED"
        elif delta > run.tolerance_s:
            res = "LATE"
        elif delta < -run.tolerance_s:
            res = "EARLY"
        else:
            res = "OK"
        if res != "OK":
            passed = False
        rows.append({"state": hp.FLIGHT_STATES[st],
                     "expected_s": None if exp is None else round(exp, 3),
                     "witness_s": None if obs is None else round(obs, 3),
                     "delta_s": None if delta is None else round(delta, 3),
                     "result": res})
    expected_order = sorted(run.expected, key=run.expected.get)
    observed_order = [s for s in run.sequence if s != hp.PAD_IDLE]
    order_ok = observed_order == expected_order
    problems = []
    if not order_ok:
        passed = False
        problems.append("state order mismatch: Witness reported "
                        + " -> ".join(hp.FLIGHT_STATES[s]
                                      for s in run.sequence or [])
                        + "; expected Pad idle -> "
                        + " -> ".join(hp.FLIGHT_STATES[s]
                                      for s in expected_order))
    if not run.hitl_seen:
        passed = False
        problems.append("HITL-active status bit never seen")
    if run.hitl_dropped:
        passed = False
        problems.append("HITL-active status bit dropped during the run")
    if run.initial_state not in (None, hp.PAD_IDLE):
        passed = False
        problems.append("Witness was not in Pad idle at start "
                        f"({hp.FLIGHT_STATES[run.initial_state]})")
    if run.aborted:
        passed = False
        problems.append("run aborted: " + (run.abort_reason or "stopped"))
    warnings = list(run.warnings)
    if run.attitude is not None:
        if not run.attitude["passed"]:
            passed = False
        problems += ["attitude: " + p for p in run.attitude["problems"]]
        warnings += ["attitude: " + w for w in run.attitude["warnings"]]
    return {"passed": passed, "rows": rows, "order_ok": order_ok,
            "problems": problems, "warnings": warnings,
            "attitude": run.attitude, "source": run.source,
            "tolerance_s": run.tolerance_s, "frames_sent": run.frames_sent,
            "late_ticks": run.late_ticks, "preset": run.preset,
            "observed_sequence": [hp.FLIGHT_STATES[s] for s in run.sequence]}


def format_table(result: dict) -> str:
    def f(v):
        return "-" if v is None else f"{v:.2f}"
    lines = [f"{'State':<10}{'Expected s':>12}{'Witness s':>12}"
             f"{'Delta s':>10}  Result"]
    for r in result["rows"]:
        lines.append(f"{r['state']:<10}{f(r['expected_s']):>12}"
                     f"{f(r['witness_s']):>12}{f(r['delta_s']):>10}  "
                     f"{r['result']}")
    if not result["rows"]:
        lines.append("(no transitions expected or reported)")
    for p in result["problems"]:
        lines.append("PROBLEM: " + p)
    for w in result["warnings"]:
        lines.append("WARNING: " + w)
    if result.get("attitude"):
        lines.append(att.format_summary(result["attitude"]))
    lines.append(("PASS" if result["passed"] else "FAIL")
                 + f" (tolerance {result['tolerance_s']:.2f} s, "
                 f"{result['frames_sent']} frames sent)")
    return "\n".join(lines)


class HitlSession:
    """One TCP connection to Witness (via ser2net) plus at most one run."""

    def __init__(self, runs_dir: str | None = DEFAULT_RUNS_DIR):
        self.runs_dir = runs_dir
        self.view = WitnessView()
        self.events: queue.Queue = queue.Queue()  # (kind, payload)
        self.run: RunState | None = None
        self._sock: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._lock = threading.RLock()
        self._reader: threading.Thread | None = None
        self._sender: threading.Thread | None = None
        self._stop_run = threading.Event()
        self._closing = threading.Event()
        self._decoder = hp.StreamDecoder()
        self._recorder: RunRecorder | None = None
        self._pending_state = None
        self._pending_count = 0
        self._pending_sim = 0.0
        self._hitl_off_count = 0
        self._entered = False
        self._host = None
        # End-of-run FC log fetch through the bench Pi (hitl_outputs.py):
        # {"enabled", "pi_host", "pi_user", "pi_key", "restart", "wait_s"}.
        self.fc_log_config: dict = {}
        # Optional callable(bytes) given every chunk received from Witness,
        # e.g. witnessOS mirroring this connection into the Horizon feed.
        # Called on the reader thread; exceptions are swallowed.
        self.rx_tap = None
        self.t0 = time.monotonic()

    # ---------------------------------------------------------------- util
    def now(self) -> float:
        return time.monotonic() - self.t0

    def _event(self, kind, payload):
        self.events.put((kind, payload))

    def log(self, msg: str):
        line = f"[tool] {msg}"
        self._event("log", line)
        rec = self._recorder
        if rec:
            rec.text(self.now(), self.run.sim_ms if self.run else 0, line)

    def _warn(self, msg: str):
        run = self.run
        if run is not None and msg not in run.warnings:
            run.warnings.append(msg)
        self._event("warning", msg)
        self.log("WARNING: " + msg)

    # ---------------------------------------------------------- connection
    def connect(self, host: str, port: int, timeout: float = 5.0):
        if self._sock is not None:
            raise RuntimeError("already connected")
        sock = socket.create_connection((host, int(port)), timeout=timeout)
        self._host = host
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.2)
        self._sock = sock
        self._closing.clear()
        self._decoder = hp.StreamDecoder()
        self._pending_state = None
        self._pending_count = 0
        with self._lock:
            self.view = WitnessView(connected=True)
        self._reader = threading.Thread(target=self._read_loop,
                                        name="hitl-reader", daemon=True)
        self._reader.start()
        self.log(f"connected to {host}:{port}")
        self._event("connection", True)

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def disconnect(self, send_exit: bool = True):
        if self.run is not None and self.run.running:
            self.stop_run("disconnect")
        elif send_exit and self._sock is not None and (
                self._entered or self.view.hitl_active):
            self._try_send_command(hp.CMD_HITL_EXIT)
        self._closing.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        if self._reader is not None and \
                self._reader is not threading.current_thread():
            self._reader.join(timeout=2)
        with self._lock:
            self.view.connected = False
        self._event("connection", False)
        self.log("disconnected")

    def _send(self, data: bytes):
        sock = self._sock
        if sock is None:
            raise ConnectionError("not connected")
        with self._send_lock:
            sock.sendall(data)
        with self._lock:
            self.view.tx_bytes += len(data)

    def send_command(self, command: int):
        frame = hp.encode_command(command)  # raises for anything not allowed
        self._send(frame)
        if command == hp.CMD_HITL_ENTER:
            self._entered = True
        elif command == hp.CMD_HITL_EXIT:
            self._entered = False
        self.log(f"sent {hp.ALLOWED_COMMANDS[command]} ({frame.hex(' ')})")

    def _try_send_command(self, command: int):
        try:
            self.send_command(command)
        except OSError as exc:
            self.log(f"could not send {hp.ALLOWED_COMMANDS[command]}: {exc}")

    # -------------------------------------------------------------- reader
    def _read_loop(self):
        sock = self._sock
        while not self._closing.is_set():
            try:
                data = sock.recv(8192)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._closing.is_set():
                    self.log(f"connection error: {exc}")
                break
            if not data:
                if not self._closing.is_set():
                    self.log("connection closed by remote")
                break
            with self._lock:
                self.view.rx_bytes += len(data)
            tap = self.rx_tap
            if tap is not None:
                try:
                    tap(data)
                except Exception:  # never let a mirror break the run
                    pass
            for kind, value in self._decoder.feed(data):
                if kind == "packet":
                    self._on_packet(value)
                else:
                    self._on_text(value)
        if not self._closing.is_set():
            # Remote side went away; tear down from a helper thread.
            threading.Thread(target=self.disconnect, kwargs={
                "send_exit": False}, daemon=True).start()

    def _sim_s(self) -> float:
        run = self.run
        return run.sim_ms / 1000.0 if run is not None else 0.0

    def _on_text(self, line: str):
        nmea = hp.is_nmea(line)
        run = self.run
        if self._recorder:
            self._recorder.text(self.now(), run.sim_ms if run else 0, line,
                                nmea)
        if nmea:
            with self._lock:
                self.view.nmea_lines += 1
            return
        self._event("text", line)
        tr = hp.parse_state_log(line)
        if tr is not None and run is not None and run.running:
            self._accept_state(tr[1], self._sim_s(), source="log")

    def _on_packet(self, pkt: dict):
        pid = pkt["id"]
        run = self.run
        if pid in (hp.ID_HITL, hp.ID_HITL_IMU):
            with self._lock:
                self.view.echo_frames += 1
            return  # our own echo; never interpreted
        if self._recorder:
            self._recorder.packet(self.now(), run.sim_ms if run else 0, pkt)
        if pid == hp.ID_COMMAND:
            if pkt.get("probable_echo"):
                return
        with self._lock:
            v = self.view
            if pid == hp.ID_STATE:
                v.baro_alt_m = pkt["baro_alt_m"]
                v.gps_alt_m = pkt["gps_alt_m"]
                v.battery_mv = pkt["battery_mv"]
                v.gps_sats = pkt["gps_satellites"]
                if run is not None and run.running:
                    run.plot_baro.append((self._sim_s(), pkt["baro_alt_m"]))
            elif pid == hp.ID_WITNESS_DEBUG:
                v.baro_alt_m = pkt["baro_alt_m"]
                v.gps_alt_m = pkt["gps_alt_m"]
                v.battery_mv = pkt["battery_mv"]
                if run is not None and run.running:
                    run.plot_baro.append((self._sim_s(), pkt["baro_alt_m"]))
            if pid in (hp.ID_SENSORS, hp.ID_WITNESS_DEBUG):
                v.accel_x_g = pkt["accel_g"][0]
                if run is not None and run.running:
                    run.plot_wacc.append((self._sim_s(), pkt["accel_g"][0]))
        if pid in (hp.ID_STATE, hp.ID_WITNESS_DEBUG) and run is not None \
                and run.streaming and run.truth_quats is not None:
            self._attitude_packet(run, pkt)
        # Only packets computed from simulated data (after EXIT Witness resets
        # its estimators, so those would not be comparable).
        if pid == hp.ID_WITNESS_DEBUG and run is not None and run.streaming                 and pkt["status"].get("hitl_active"):
            run.debug_records.append((self._sim_s(), pkt))
        if pid in hp.TELEMETRY_STATUS_PACKETS:
            self._on_status(pkt["status"], pkt["uptime_ms"])

    def _attitude_packet(self, run: RunState, pkt: dict):
        """Pair a Witness quaternion with the truth at the current sim time.

        The truth index is the last frame sent (minus att_latency_ms), so
        USB/ser2net/network latency shows up as a small apparent lag.
        """
        n = len(run.truth_quats)
        idx = run.sim_ms // sm.TICK_MS - run.att_latency_ms // sm.TICK_MS
        idx = max(0, min(n - 1, idx))
        qt = run.truth_quats[idx]
        raw = tuple(float(c) for c in pkt["quat_xyzw"])
        valid = att.is_valid(raw)
        sim_s = idx * sm.DT_S
        wstate = self.view.flight_state
        rec = {"wall_s": round(self.now(), 4), "sim_s": sim_s,
               "packet": pkt["name"], "packet_id": pkt["id"],
               "witness_state_code": pkt.get("status", {}).get("flight_state"),
               "hitl_active": bool(pkt.get("status", {}).get("hitl_active")),
               "witness_state": None if wstate is None
               else hp.FLIGHT_STATES[wstate],
               "phase": hp.FLIGHT_STATES[run.state_track[idx]]
               if run.state_track else "",
               "valid": valid, "q_truth": qt, "q_witness_raw": raw,
               "q_witness": att.qnormalize(raw) if valid else raw,
               "truth_tilt_deg": att.tilt_from_vertical_deg(qt),
               "witness_tilt_deg": None, "tilt_err_deg": None,
               "nose_err_deg": None, "att_err_deg": None}
        if valid:
            qw = rec["q_witness"]
            rec["witness_tilt_deg"] = att.tilt_from_vertical_deg(qw)
            rec["tilt_err_deg"] = abs(rec["truth_tilt_deg"]
                                      - rec["witness_tilt_deg"])
            if run.att_psi is None and wstate is not None and \
                    wstate >= hp.BOOST:
                # Use the last valid packet from before Boost was reported:
                # the vehicle is still on the pad, so packet latency cannot
                # alias the boost roll (about world Z while the nose is
                # vertical) into the heading offset.
                ref = next((r for r in reversed(run.att_records)
                            if r["valid"] and r.get("witness_state_code")
                            == hp.PAD_IDLE), None)
                if ref is not None:
                    run.att_psi = att.heading_offset_rad(ref["q_truth"],
                                                         ref["q_witness"])
                    run.att_psi_source = (
                        f"locked when Boost was detected (sim {sim_s:.2f} s) "
                        f"from the last pad packet, sim {ref['sim_s']:.2f} s")
                else:
                    run.att_psi = att.heading_offset_rad(qt, qw)
                    run.att_psi_source = (f"locked when Boost was detected, "
                                          f"sim {sim_s:.2f} s")
                self.log(f"attitude heading offset "
                         f"{math.degrees(run.att_psi):.1f} deg "
                         f"({run.att_psi_source})")
            if run.att_psi is not None:
                rec["att_err_deg"] = att.attitude_error_deg(qt, qw,
                                                            run.att_psi)
                rec["nose_err_deg"] = att.nose_error_deg(qt, qw, run.att_psi)
                run.plot_att_err.append((sim_s, rec["att_err_deg"]))
            run.plot_tilt_w.append((sim_s, rec["witness_tilt_deg"]))
            run.plot_tilt_err.append((sim_s, rec["tilt_err_deg"]))
        run.att_records.append(rec)

    def _on_status(self, status: dict, uptime_ms: int):
        run = self.run
        with self._lock:
            v = self.view
            v.last_status = status
            v.last_status_time = self.now()
            v.uptime_ms = uptime_ms
            v.gps_locked = status["gps_locked"]
            was_active = v.hitl_active
            v.hitl_active = status["hitl_active"]
        if run is not None and run.running:
            if status["hitl_active"]:
                self._hitl_off_count = 0
                if not run.hitl_seen:
                    run.hitl_seen = True
                    self.log(f"HITL active bit seen at sim "
                             f"{self._sim_s():.2f} s")
                    self._event("hitl", True)
            elif run.hitl_seen and not run.hitl_dropped and run.streaming:
                self._hitl_off_count += 1
                if self._hitl_off_count >= STATE_CONFIRM_PACKETS:
                    run.hitl_dropped = True
                    self._warn("HITL-active bit dropped during the run "
                               f"(sim {self._sim_s():.2f} s)")
        elif was_active != status["hitl_active"]:
            self._event("hitl", status["hitl_active"])
        state = status["flight_state"]
        if state == self._pending_state:
            self._pending_count += 1
        else:
            self._pending_state = state
            self._pending_count = 1
            self._pending_sim = self._sim_s()
        current = self.view.flight_state
        # A single packet never changes the state (guards against false frame
        # matches); going backwards needs a longer run of packets because log
        # lines and queued telemetry can arrive slightly out of order.
        needed = (STATE_CONFIRM_PACKETS if current is None or state > current
                  else BACKWARD_CONFIRM_PACKETS)
        if current is None or (state != current
                               and self._pending_count >= needed):
            self._accept_state(state, self._pending_sim, source="status")

    def _accept_state(self, state: int, sim_s: float, source: str):
        run = self.run
        prev = self.view.flight_state
        if state != prev:
            with self._lock:
                self.view.flight_state = state
            self._event("state", state)
            if prev is not None:
                self.log(f"Witness state {hp.FLIGHT_STATES[prev]} -> "
                         f"{hp.FLIGHT_STATES[state]} ({source}, sim "
                         f"{sim_s:.2f} s)")
        if run is None or not run.running:
            return
        if run.initial_state is None:
            run.initial_state = state
            run.sequence.append(state)
            return
        if not run.sequence or run.sequence[-1] != state:
            run.sequence.append(state)
        if state != hp.PAD_IDLE and (state not in run.observed
                                     or sim_s < run.observed[state]):
            run.observed[state] = sim_s

    # ----------------------------------------------------------------- run
    def prepare_run(self, params, preset: str = "custom",
                    tolerance_s: float = 1.0, time_scale: int = 1,
                    att_thresholds: dict | None = None,
                    att_latency_ms: int = 0) -> RunState:
        """Build the frames for a preset (SimParams) or CSV (CsvParams)."""
        if isinstance(params, tc.CsvParams):
            return self._prepare_csv(params, preset, tolerance_s, time_scale,
                                     att_thresholds, att_latency_ms)
        samples = sm.generate(params)
        frames = [hp.encode_hitl(s.hitl) for s in samples]
        expected = sm.predict_transitions(samples)
        return RunState(params=params, preset=preset, samples=samples,
                        frames=frames, expected=expected,
                        tolerance_s=tolerance_s,
                        time_scale=max(1, int(time_scale)))

    def _prepare_csv(self, params, preset, tolerance_s, time_scale,
                     att_thresholds, att_latency_ms) -> RunState:
        build = tc.build(params)
        samples = build.samples
        frames = [hp.encode_hitl(s.hitl) for s in samples]
        track = sm.predict_state_track(samples)
        run = RunState(params=params, preset=preset, samples=samples,
                       frames=frames,
                       expected=expected_from_track(samples, track),
                       tolerance_s=tolerance_s,
                       time_scale=max(1, int(time_scale)), source="csv",
                       csv_report=build.report,
                       truth_quats=[s.q_truth for s in samples],
                       state_track=track, request_debug=True,
                       att_thresholds=dict(att_thresholds or {}),
                       att_latency_ms=int(att_latency_ms))
        run.plot_tilt_truth = [(s.t_ms / 1000,
                                att.tilt_from_vertical_deg(s.q_truth))
                               for s in samples[::10]]
        if getattr(params, "imu_synthesis", False):
            self._prepare_imu(run, params)
        baro = build.report["baro"]
        if baro["below_ms5607_range_windows_sim_s"]:
            run.warnings.append(
                "simulated pressure is below the MS5607 10 mbar range for "
                f"{baro['below_ms5607_range_s']:.1f} s (sim "
                + ", ".join(f"{a:.1f}-{b:.1f} s" for a, b in
                            baro["below_ms5607_range_windows_sim_s"])
                + "); sent anyway, but the real sensor could not measure it")
        return run

    def _prepare_imu(self, run: RunState, params):
        """Synthesizes the 960 Hz IMU stream and encodes its 0x0E frames.

        Raises ValueError (shown before the run starts) when the synthesis
        packages are missing from this interpreter, instead of silently
        falling back to 100 Hz IMU values.
        """
        import sys
        try:
            import imu_synth as ims
            reason = ims.available()
        except ImportError as exc:
            reason = f"{exc.name} is not installed"
        if reason:
            raise ValueError(
                f"960 Hz IMU synthesis needs numpy, scipy and soundfile: {reason} "
                f"in {sys.executable}. Install them with: \"{sys.executable}\" -m pip "
                "install numpy scipy soundfile (or set imu_synthesis=false for "
                "100 Hz IMU values)")
        import numpy as np
        profile = ims.find_profile(params.csv_path, getattr(params, "vibration_profile", ""))
        stream = ims.from_samples(run.samples, ims.ImuSynthParams(
            seed=params.seed, vibration_scale=params.vibration_scale,
            noise_scale=params.imu_noise_scale,
            offset_scale=params.imu_offset_scale,
            vibration_thrust_path=profile["thrust_path"] if profile else "",
            vibration_lateral_path=profile["lateral_path"] if profile else "",
            vibration_full_scale_g=profile["full_scale_g"] if profile else 150.67))
        stream.report["vibration_profile"] = (
            {k: profile[k] for k in ("name", "dir", "thrust", "lateral", "full_scale_g",
                                     "description") if k in profile}
            if profile else None)
        run.imu_stream = stream
        run.imu_report = stream.report
        n = len(stream) - len(stream) % hp.IMU_BATCH_SAMPLES
        block = np.concatenate([stream.low_g, stream.high_g, stream.gyro],
                               axis=1).astype(">i2")   # per sample: low, high, gyro
        for j in range(0, n, hp.IMU_BATCH_SAMPLES):
            run.imu_frames.append(
                bytes((hp.ID_HITL_IMU,)) + (j & 0xFFFFFFFF).to_bytes(4, "big")
                + block[j:j + hp.IMU_BATCH_SAMPLES].tobytes() + bytes((hp.EOF,)))
            run.imu_due_ms.append(float(stream.time_s[j + hp.IMU_BATCH_SAMPLES - 1]) * 1000.0)

    @staticmethod
    def imu_summary(run: RunState) -> list[str]:
        """One-line descriptions of the 960 Hz synthesis for logs."""
        rep = run.imu_report
        if not rep:
            return ["960 Hz IMU synthesis off (imu_synthesis=false): 100 Hz 0x0C IMU values"]
        prof = rep.get("vibration_profile")
        p = rep.get("params", {})
        vib = ("vibration: none (no profile for this trajectory)" if not prof else
               f"vibration: profile {prof['name']!r} x{p.get('vibration_scale', 1.0):g} "
               f"({prof['thrust']} / {prof['lateral']}, {prof['full_scale_g']:g} g full "
               f"scale, from {rep.get('vibration_start_s', 0):.2f} s for "
               f"{rep.get('vibration_duration_s', 0):.1f} s)")
        return [f"960 Hz IMU synthesis: {rep['samples']} samples, noise x{p.get('noise_scale', 1):g}, "
                f"offsets x{p.get('offset_scale', 1):g}; {vib}"]

    def start_run(self, run: RunState, abort_on_no_hitl: bool = False):
        if not self.connected:
            raise ConnectionError("not connected")
        if self.run is not None and self.run.running:
            raise RuntimeError("a run is already in progress")
        if run.time_scale != 1 and not is_loopback(self._host):
            # Witness integrates with its own clock: hardware is real time.
            raise ValueError("speed/time-scale factors are only allowed "
                             "against a local fake Witness (127.0.0.1)")
        if self.view.flight_state not in (None, hp.PAD_IDLE):
            self._warn("Witness is not in Pad idle; it will reject HITL "
                       "ENTER (reboot Witness to return to Pad idle)")
        self.run = run
        self._stop_run.clear()
        self._hitl_off_count = 0
        if self.view.flight_state is not None:
            run.initial_state = self.view.flight_state
            run.sequence.append(self.view.flight_state)
        if self.runs_dir:
            os.makedirs(self.runs_dir, exist_ok=True)
            self._recorder = RunRecorder(self.runs_dir, run)
            run.run_dir = self._recorder.dir
        run.running = True
        run.streaming = True
        run.started_wall = self.now()
        self._sender = threading.Thread(
            target=self._send_loop, args=(run, abort_on_no_hitl),
            name="hitl-sender", daemon=True)
        self._sender.start()
        self._event("run", "started")

    def stop_run(self, reason: str = "stopped by user"):
        run = self.run
        if run is None or not run.running:
            return
        run.aborted = True
        run.abort_reason = reason
        self._stop_run.set()
        if self._sender is not None and \
                self._sender is not threading.current_thread():
            self._sender.join(timeout=5)

    def wait_run(self, timeout: float | None = None) -> bool:
        if self._sender is None:
            return True
        self._sender.join(timeout)
        return not self._sender.is_alive()

    def _send_loop(self, run: RunState, abort_on_no_hitl: bool):
        try:
            if not self.view.hitl_active:
                self.send_command(hp.CMD_HITL_ENTER)
            else:
                run.hitl_seen = True
            period = sm.DT_S
            per_tick = run.time_scale
            start = time.perf_counter()
            wall0 = self.now()
            tick = 0
            i = 0
            n = len(run.frames)
            warned_hitl = False
            jj = 0                       # next 0x0E frame
            n_imu = len(run.imu_frames)
            while i < n and not self._stop_run.is_set():
                target = start + tick * period
                delay = target - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.05:
                    run.late_ticks += 1
                for _ in range(per_tick):
                    if i >= n:
                        break
                    s = run.samples[i]
                    # 960 Hz samples up to this tick go out first.
                    while jj < n_imu and run.imu_due_ms[jj] <= s.t_ms:
                        self._send(run.imu_frames[jj])
                        jj += 1
                        run.imu_frames_sent += 1
                    self._send(run.frames[i])
                    run.sim_ms = s.t_ms
                    run.frames_sent += 1
                    if i % 5 == 0:
                        run.plot_true.append((s.t_ms / 1000, s.alt_msl_m))
                        run.plot_accel.append((s.t_ms / 1000, s.hitl.low_g[0]))
                    if self._recorder:
                        self._recorder.sent(self.now(), s)
                    i += 1
                tick += 1
                if run.request_debug and not run.debug_sent and \
                        run.hitl_seen:
                    run.debug_sent = True
                    self.send_command(hp.CMD_WITNESS_DEBUG_START)
                if not run.hitl_seen and not warned_hitl and \
                        self.now() - wall0 > HITL_ACTIVE_TIMEOUT_S:
                    warned_hitl = True
                    self._warn("HITL-active bit not seen within "
                               f"{HITL_ACTIVE_TIMEOUT_S:.0f} s of starting")
                    if abort_on_no_hitl:
                        run.aborted = True
                        run.abort_reason = "HITL never became active"
                        break
        except OSError as exc:
            run.aborted = True
            run.abort_reason = f"send failed: {exc}"
            self.log(run.abort_reason)
        finally:
            run.streaming = False
            if self.connected:
                self._try_send_command(hp.CMD_HITL_EXIT)
            # Allow trailing telemetry / log lines to arrive.
            time.sleep(0.6 if not run.aborted else 0.3)
            run.running = False
            run.finished = True
            try:
                finish_attitude(run)
            except Exception as exc:  # never lose the run over a metric
                self.log(f"attitude summary failed: {exc!r}")
            run.result = evaluate(run)
            if self._recorder:
                try:
                    import hitl_outputs
                    run.comparison = hitl_outputs.write_comparison(
                        run, self._recorder.dir)
                    if run.comparison:
                        run.result["comparison"] = run.comparison
                except Exception as exc:  # never lose the run over a metric
                    self.log(f"comparison outputs failed: {exc!r}")
                self._recorder.finish(
                    run.result, run.att_records
                    if run.truth_quats is not None else None)
                self.log(f"run saved to {self._recorder.dir}")
                self._recorder = None
                cfg = self.fc_log_config
                if cfg.get("enabled") and run.aborted:
                    self.log("FC log fetch skipped: the run was stopped or aborted")
                elif cfg.get("enabled") and not cfg.get("pi_user") and \
                        not is_loopback(self._host):
                    self.log("FC log fetch skipped: no Pi user set (pi_user / pi_key in "
                             "hitl_settings.json, or WITNESS_PI_USER / WITNESS_PI_KEY)")
                elif cfg.get("enabled") and run.run_dir and \
                        not is_loopback(self._host) and self.connected:
                    import hitl_outputs
                    fc = hitl_outputs.fetch_fc_logs(self, run, cfg, self.log)
                    run.result["fc_logs"] = fc
                    if fc.get("flash_comparison"):
                        run.result["comparison_flash"] = fc["flash_comparison"]
                    with open(os.path.join(run.run_dir, "summary.json"), "w") as f:
                        json.dump(run.result, f, indent=2)
            self._event("run", "finished")
