"""Connection, 100 Hz sender, telemetry tracking and run recording.

Shared by the Tkinter GUI and the headless CLI. All network I/O happens on
background threads; consumers read ``snapshot()`` and drain ``events``.
"""

from __future__ import annotations

import csv
import json
import os
import queue
import socket
import threading
import time
from dataclasses import dataclass, field

import hitl_protocol as hp
import sim_model as sm

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RUNS_DIR = os.path.join(HERE, "runs")
HITL_ACTIVE_TIMEOUT_S = 2.0
STATE_CONFIRM_PACKETS = 2
BACKWARD_CONFIRM_PACKETS = 5
TRANSITION_STATES = (hp.BOOST, hp.COAST, hp.DESCENT, hp.LANDED)


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
    params: sm.SimParams
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
                       "model_summary": sm.summarize(run.samples)},
                      f, indent=2)
        self._sent = open(os.path.join(path, "sent.csv"), "w", newline="")
        self._sent_w = csv.writer(self._sent)
        self._sent_w.writerow(["wall_s", "sim_ms", "phase", "alt_msl_m",
                               "vel_mps", "low_g_x", "high_g_x", "gyro_x",
                               "pressure_pa", "gps_sats", "gps_fix",
                               "battery_mv"])
        self._rx = open(os.path.join(path, "received.jsonl"), "w")
        self._log = open(os.path.join(path, "log.txt"), "w")
        self._nmea = open(os.path.join(path, "nmea.txt"), "w")

    def sent(self, wall_s, s: sm.SimSample):
        h = s.hitl
        with self._lock:
            if self._sent.closed:
                return
            self._sent_w.writerow([
                f"{wall_s:.4f}", s.t_ms, s.phase, f"{s.alt_msl_m:.2f}",
                f"{s.vel_mps:.2f}", f"{h.low_g[0]:.4f}", f"{h.high_g[0]:.4f}",
                f"{h.gyro_dps[0]:.3f}", f"{h.pressure_pa:.1f}", h.satellites,
                h.fix_quality, f"{h.battery_mv:.0f}"])

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

    def finish(self, result: dict):
        with self._lock:
            for f in (self._sent, self._rx, self._log, self._nmea):
                f.close()
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
    return {"passed": passed, "rows": rows, "order_ok": order_ok,
            "problems": problems, "warnings": list(run.warnings),
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
        if pid == hp.ID_HITL:
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
        if pid in hp.TELEMETRY_STATUS_PACKETS:
            self._on_status(pkt["status"], pkt["uptime_ms"])

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
    def prepare_run(self, params: sm.SimParams, preset: str = "custom",
                    tolerance_s: float = 1.0, time_scale: int = 1) -> RunState:
        samples = sm.generate(params)
        frames = [hp.encode_hitl(s.hitl) for s in samples]
        expected = sm.predict_transitions(samples)
        return RunState(params=params, preset=preset, samples=samples,
                        frames=frames, expected=expected,
                        tolerance_s=tolerance_s,
                        time_scale=max(1, int(time_scale)))

    def start_run(self, run: RunState, abort_on_no_hitl: bool = False):
        if not self.connected:
            raise ConnectionError("not connected")
        if self.run is not None and self.run.running:
            raise RuntimeError("a run is already in progress")
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
                    self._send(run.frames[i])
                    s = run.samples[i]
                    run.sim_ms = s.t_ms
                    run.frames_sent += 1
                    if i % 5 == 0:
                        run.plot_true.append((s.t_ms / 1000, s.alt_msl_m))
                        run.plot_accel.append((s.t_ms / 1000, s.hitl.low_g[0]))
                    if self._recorder:
                        self._recorder.sent(self.now(), s)
                    i += 1
                tick += 1
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
            run.result = evaluate(run)
            if self._recorder:
                self._recorder.finish(run.result)
                self.log(f"run saved to {self._recorder.dir}")
                self._recorder = None
            self._event("run", "finished")
