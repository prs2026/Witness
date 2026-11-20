"""One-command bench verification of a Witness image.

    py -3.14 verify_image.py --flash        # flash images/, then check boot + one HITL flight
    py -3.14 verify_image.py                # board already flashed: boot check + flight
    py -3.14 verify_image.py --no-flight    # flash/boot checks only
    py -3.14 verify_image.py --make-scenario  # one-time: build the 30 s lead-in GPS scenario

Steps: [flash_verified.ps1 -CheckOnly, flash, retry once] -> Wi-Fi round trip
(0x5746 on/off) and boot-log check of both images (version, flight slot, ELF
hashes vs images/, GPS identification) -> erase (0x6869) -> one HITL flight on the eayf profile with
RF GPS (30 s lead-in, cold start, commanded BIT before and after HITL ENTER) ->
the tool exports the log and power-cycles the Pi hub (the run's only hub cycle) ->
power-up check (GPS baud, recoveries, reset reason), commanded BIT, erase.
Writes verify_summary.md into the run folder and prints it.

Bench rules built in: only the 710x ports outside flashing, one hub cycle per run
and never within 4 min of the previous one, HackRF TX VGA 0 dB and amp off, no
GPS baud or factory-reset commands, radio power untouched. Host names come from
hitl_settings.json and are never printed.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import queue
import random
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HITL = Path(__file__).resolve().parent
SOFTWAREV1 = HITL.parent / "Witness" / "softwarev1"
IMAGES = SOFTWAREV1 / "images"
PROFILE_CSV = HITL / "trajectories" / "eayftrajectory.csv"
SCENARIO_NAME = "flight-eayftrajectory-lead30-verify"
# Launch at the same GPS time as the lead150-geom480 scenario (start 21:46:29 +
# 150 s), so the constellation in flight matches the earlier runs.
SCENARIO_START = dt.datetime(2026, 11, 19, 21, 48, 29)
LEAD_IN_S = 30.0
LAST_HUB_FILE = HITL / "runs" / ".last_hub_cycle"
HUB_MIN_GAP_S = 240.0

sys.path.insert(0, str(HITL))
os.chdir(HITL)

# Profile: the eayf flight used since 11-18 (pad hold 20 s, vibration 0.2).
PROFILE = {
    "seed": 1, "site_alt_msl_m": 609.0, "lat_deg": 35.34705005064222, "lon_deg": -117.80906435293228,
    "sea_level_pressure_pa": 101325.0, "temp_offset_c": 0.0, "pad_hold_s": 20.0, "post_landing_s": 40.0,
    "touchdown_s": 0.2, "accel_smooth_s": 0.05, "follow_speed_mps": 10.0, "rail_dir_speed_mps": 0.5,
    "after_apogee_attitude": "hold", "pendulum_amp_deg": 10.0, "pendulum_period_s": 4.0,
    "accel_noise_g": 0.003, "high_g_noise_g": 0.01, "gyro_noise_dps": 0.05, "gyro_bias_x_dps": 0.0,
    "gyro_bias_y_dps": 0.0, "gyro_bias_z_dps": 0.0, "baro_noise_pa": 1.0, "gps_sats": 10, "gps_fix": True,
    "battery_mv": 7800.0, "imu_synthesis": True, "vibration_profile": "", "vibration_scale": 0.2,
    "vibration_thrust_file": str(HITL / "vibration" / "eayftrajectory" / "vibration_flight_thrust.ogg"),
    "vibration_lateral_file": str(HITL / "vibration" / "eayftrajectory" / "vibration_flight_lateral.ogg"),
    "vibration_full_scale_g": 0.0, "imu_noise_scale": 1.0, "imu_offset_scale": 1.0,
    "deployment_spin_dps": 0.0, "deployment_shock_g": 0.0, "deployment_delay_s": 1.0,
    "deployment_duration_s": 0.3,
}

WITNESS_PORT = 7100
APERTURE_PORT = 7102


def log(msg: str) -> None:
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def settings() -> dict:
    return json.load(open(HITL / "hitl_settings.json"))


def host() -> str:
    return settings()["host"]


def cmd(code: int, data: bytes = b"\0\0\0\0\0\0") -> bytes:
    return bytes([5]) + data + bytes([code >> 8, code & 255, 10])


def nonce() -> bytes:
    return bytes([0, 0]) + bytes(random.getrandbits(8) for _ in range(4))


class Rec(threading.Thread):
    """Raw recorder for a ser2net port; reconnects on drop."""

    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.port, self.chunks, self.stop, self.sock = port, [], False, None

    def run(self):
        while not self.stop:
            try:
                s = socket.create_connection((host(), self.port), 5)
                s.settimeout(0.5)
                self.sock = s
                while not self.stop:
                    try:
                        b = s.recv(65536)
                    except socket.timeout:
                        continue
                    if not b:
                        raise OSError("closed")
                    self.chunks.append((time.time(), b))
            except OSError:
                self.sock = None
                time.sleep(1.0)
        if self.sock:
            self.sock.close()

    def send(self, data: bytes) -> None:
        for _ in range(50):
            if self.sock:
                self.sock.sendall(data)
                return
            time.sleep(0.1)
        raise RuntimeError("recorder not connected")

    def text(self) -> str:
        t = b"".join(b for _, b in self.chunks)
        t = re.sub(rb"\x1b\[[0-9;]*m", b"", t).decode("latin1")
        return re.sub(r"[^\x09\x0a\x20-\x7e]", ".", t)


def record(port: int, seconds: float, sends=()) -> str:
    """Record `seconds` from a port; sends = [(delay_s, bytes)] after connecting."""
    r = Rec(port)
    r.start()
    t0 = time.time()
    while r.sock is None and time.time() - t0 < 10:
        time.sleep(0.1)
    for delay, data in sends:
        time.sleep(delay)
        r.send(data)
    time.sleep(max(0.0, seconds - (time.time() - t0)))
    r.stop = True
    time.sleep(0.7)
    return r.text()


def lines(text: str, pattern: str) -> list:
    out = []
    for m in re.finditer(r"[IWE] \(\d+\) [\w.]+: [^\n]*", text):
        s = m.group(0).strip()
        if re.search(pattern, s):
            out.append(s[:220])
    return out


# ------------------------------------------------------------------ image
def image_info(path: Path) -> dict:
    b = path.read_bytes()
    s = lambda o, n: b[o:o + n].split(b"\0")[0].decode(errors="replace")
    return {"size": len(b), "version": s(0x30, 32), "elf": b[0xB0:0xD0].hex()}


def flash() -> list:
    notes = []
    ps = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(SOFTWAREV1 / "flash_verified.ps1")]
    r = subprocess.run(ps + ["-CheckOnly"], capture_output=True, text=True, cwd=SOFTWAREV1, timeout=180)
    ok = "OK to flash" in (r.stdout + r.stderr)
    notes.append(f"target check: {'OK to flash' if ok else 'FAILED'}")
    if not ok:
        raise SystemExit("flash_verified -CheckOnly did not say OK to flash; nothing written\n" + r.stdout[-800:])
    for attempt in (1, 2):
        r = subprocess.run(ps, capture_output=True, text=True, cwd=SOFTWAREV1, timeout=600)
        out = r.stdout + r.stderr
        verified = out.count("Hash of data verified")
        if verified >= 5:
            notes.append(f"flash attempt {attempt}: {verified} writes, all hash verified")
            return notes
        notes.append(f"flash attempt {attempt}: {verified} hash-verified writes (rc {r.returncode})")
        time.sleep(5)
    raise SystemExit("flash failed twice:\n" + "\n".join(notes))


def boot_check(want: dict, want_maint: dict) -> tuple[list, bool]:
    """Wi-Fi round trip (0x5746 on, then off) with the recorder attached: boots the
    maintenance image, then the flight image, so both boot logs are captured. The
    GPS keeps power. (0x4252 restart is refused unless HITL is active.)"""
    r = Rec(WITNESS_PORT)
    r.start()
    t0 = time.time()
    while r.sock is None and time.time() - t0 < 10:
        time.sleep(0.1)
    time.sleep(1.0)

    def wait_for(pattern: str, since: int, limit: float) -> bool:
        end = time.time() + limit
        while time.time() < end:
            if re.search(pattern, r.text()[since:]):
                return True
            time.sleep(0.5)
        return False

    r.send(cmd(0x5746, bytes([1, 0, 0, 0, 0, 0])))
    maint_up = wait_for(r"Wi-Fi AP .*up|AP up", 0, 40)
    mark = len(r.text())
    r.send(cmd(0x5746, bytes([0, 0, 0, 0, 0, 0])))
    back = wait_for(r"selftest: running flight slot", mark, 45)
    wait_for(r"already at 25 Hz|GPS RX: baud", mark, 12)
    r.stop = True
    time.sleep(0.7)
    t = r.text()
    tm, tf = t[:mark], t[mark:]
    vm = re.findall(r"App version:\s+(\S+?)\.?\s", tm)
    em = re.findall(r"ELF file SHA256:\s+([0-9a-f]{6,})", tm)
    ver = re.findall(r"App version:\s+(\S+?)\.?\s", tf)
    elf = re.findall(r"ELF file SHA256:\s+([0-9a-f]{6,})", tf)
    slot = re.search(r"selftest: running (flight slot \d) \(([^)]+)\)", tf)
    pending = "pending verification" in tf
    flight_ok = bool(ver and ver[-1] == want["version"] and elf and want["elf"].startswith(elf[-1]))
    maint_ok = bool(maint_up and em and want_maint["elf"].startswith(em[-1]))
    ok = flight_ok and maint_ok and bool(slot) and back and not pending
    notes = [
        f"maintenance image: {vm[-1] if vm else '?'}, ELF {em[-1] if em else '?'} vs images/maint.bin "
        f"{want_maint['elf'][:9]}: {'match' if maint_ok else 'MISMATCH or no AP'}",
        f"flight image: {ver[-1] if ver else '?'}, ELF {elf[-1] if elf else '?'} vs images/flight.bin "
        f"{want['elf'][:9]}: {'match' if flight_ok else 'MISMATCH'}",
        f"{slot.group(1) if slot else 'flight slot NOT seen'}; pending verification: {'YES' if pending else 'no'}",
    ] + lines(tf, r"GPS identified|already at|slow state|confirmed at")
    return notes, ok


def erase() -> str:
    t = record(WITNESS_PORT, 9, sends=[(1.5, cmd(0x6869))])
    m = re.search(r"new session (\d+)", t)
    return m.group(1) if m else "?"


def end_checks() -> dict:
    """After the tool's hub cycle: GPS state, reset reason, commanded BIT, erase."""
    t = record(WITNESS_PORT, 30, sends=[(14.0, cmd(0x5442, nonce())), (12.0, cmd(0x6869))])
    gps = lines(t, r"GPS RX: baud")
    cpu = lines(t, r"last reset")
    m = re.search(r"new session (\d+)", t)
    return {
        "gps": gps[-1] if gps else "no GPS status line",
        "rate_skip": lines(t, r"already at 25 Hz"),
        "slow": lines(t, r"slow state"),
        "reset": re.search(r"last reset (\w+)", cpu[-1]).group(1) if cpu else "?",
        "bit": lines(t, r"bit: BIT"),
        "session": m.group(1) if m else "?",
    }


# ------------------------------------------------------------------ hub guard
def hub_gap_ok() -> float:
    try:
        last = float(LAST_HUB_FILE.read_text())
    except (OSError, ValueError):
        return 0.0
    return max(0.0, HUB_MIN_GAP_S - (time.time() - last))


def note_hub_cycle() -> None:
    LAST_HUB_FILE.parent.mkdir(exist_ok=True)
    LAST_HUB_FILE.write_text(str(time.time()))


# ------------------------------------------------------------------ scenario
def find_scenario():
    import gps_rf
    found = [p for p in sorted(gps_rf.SCENARIO_DIR.glob(f"*-{SCENARIO_NAME}")) if (p / "scenario.json").exists()]
    return gps_rf.Scenario.load(found[-1]) if found else None


def make_scenario() -> None:
    import gps_rf
    import trajectory_csv as tc
    params = tc.CsvParams(csv_path=str(PROFILE_CSV), **PROFILE)
    samples = tc.build(params).samples
    rinex = HITL / "rf_tools" / "BRDC00WRD_R_20263140000_from20h_MN.rnx"
    sc = gps_rf.create_flight(samples, site_lat=params.lat_deg, site_lon=params.lon_deg,
                              site_alt_msl=params.site_alt_msl_m, lead_in_s=LEAD_IN_S,
                              rinex=rinex, start_gps=SCENARIO_START, name=SCENARIO_NAME,
                              source="CSV eayftrajectory.csv, 30 s lead-in, launch at the lead150-geom480 GPS time")
    log(f"scenario ready: {sc.folder.name}, {sc.duration_s:.0f} s, {len(sc.visible_prns)} satellites visible")


# ------------------------------------------------------------------ flight
def fly(scenario) -> tuple:
    import hitl_gui as hg
    import hitl_session as hs
    import trajectory_csv as tc

    params = tc.CsvParams(csv_path=str(PROFILE_CSV), **PROFILE)
    gps = {"scenario": scenario, "tx_vga_db": 0, "amp": False, "exe": None, "remote": None,
           "upload_ephemeris": True, "cold_start": True, "power_cycle_retry": False}
    session = hs.HitlSession(runs_dir=hs.DEFAULT_RUNS_DIR)
    session.fc_log_config = hg.fc_log_config(hg.load_settings(), True)
    run = session.prepare_run(params, preset="CSV eayftrajectory.csv (verify)", tolerance_s=10.0, time_scale=1,
                              att_thresholds={"final_tilt_max_deg": 5.0, "final_att_max_deg": None,
                                              "flight_tilt_max_deg": None}, gps=gps)
    run.gps_tx["power_cycle_retry"] = False
    aperture = Rec(APERTURE_PORT)
    aperture.start()
    session.connect(host(), WITNESS_PORT, timeout=5)

    def bit(tag):
        session.log(f"[verify] commanded BIT 0x5442 ({tag})")
        session._send(cmd(0x5442, nonce()))

    def helper():
        while not run.streaming:
            time.sleep(0.2)
        fixed_at, done = None, False
        while run.streaming and (run.sim_ms or 0) == 0:
            if session.view.gps_locked and fixed_at is None:
                fixed_at = time.time()
            if fixed_at and not done and time.time() - fixed_at >= 8 and not session.view.hitl_active:
                done = True
                bit("lead-in, HITL off")
            time.sleep(0.3)
        while (run.sim_ms or 0) < 5000 and run.streaming:
            time.sleep(0.1)
        bit("sim 5 s, HITL on")

    threading.Thread(target=helper, daemon=True).start()
    session.start_run(run, abort_on_no_hitl=True)
    while not session.wait_run(0.5):
        while True:
            try:
                kind, payload = session.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("log", "warning") and re.search(r"state|fix|FC logs|ENTER|EXIT|lead-in", str(payload)):
                log(f"  {payload}")
        if not session.connected and run.streaming:
            break
    session.wait_run(900)
    note_hub_cycle()            # the tool's export power-cycles the hub at the end of the run
    session.disconnect(send_exit=False)
    aperture.stop = True
    time.sleep(1)
    result = run.result or hs.evaluate(run)
    return run, result, aperture


def flight_numbers(run_dir: Path, aperture: Rec) -> dict:
    s = json.load(open(run_dir / "summary.json"))
    text = (run_dir / "log.txt").read_text(errors="replace")
    heap = re.findall(r"heap free \d+ \(min (\d+)\)", text)
    comp = s.get("comparison_flash") or s.get("comparison") or {}
    up = {ph: (v.get("up_max_m"), v.get("up_rms_m")) for ph, v in comp.items()}
    # Radio: Witness state packets (0x01, 23 B) relayed by Aperture, during HITL.
    stream = b"".join(b for _, b in aperture.chunks)
    radio = len(re.findall(rb"\x01.{21}\n", stream, re.S))
    return {
        "passed": s.get("passed"), "rows": s.get("rows", []), "problems": s.get("problems", []),
        "first_fix_s": (s.get("gps") or {}).get("first_fix_s"),
        "up": up, "tilt": (s.get("attitude") or {}).get("final", {}).get("tilt_err_deg"),
        "rejects": text.count("HITL frame rejected"), "unknown": text.count("unknown command"),
        "resets": len(re.findall(r"rst:0x", text)), "panic": len(re.findall(r"Guru Meditation|abort\(\)", text)),
        "heap_min": min(map(int, heap)) if heap else None,
        "session": (s.get("fc_logs") or {}).get("files", ["?"])[0][1:8] if (s.get("fc_logs") or {}).get("files") else "?",
        "fc_ok": (s.get("fc_logs") or {}).get("ok"), "radio_frames": radio,
    }


# ------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flash", action="store_true", help="flash images/ with flash_verified.ps1 first")
    ap.add_argument("--no-flight", action="store_true", help="flash and boot checks only")
    ap.add_argument("--make-scenario", action="store_true", help="build the 30 s lead-in GPS scenario and exit")
    a = ap.parse_args()

    if a.make_scenario:
        make_scenario()
        return 0

    t_start = time.time()
    want = image_info(IMAGES / "flight.bin")
    out = [f"# Image verification {time.strftime('%Y-%m-%d %H:%M')}", "",
           f"Image: images/flight.bin version **{want['version']}**, {want['size']} B, ELF {want['elf'][:9]}", ""]
    verdict = []

    if a.flash:
        log("flashing images/ with flash_verified.ps1")
        out += ["## Flash"] + [f"- {n}" for n in flash()] + [""]
        time.sleep(8)
    log("boot check (Wi-Fi round trip: maintenance image, then flight image)")
    notes, boot_ok = boot_check(want, image_info(IMAGES / "maint.bin"))
    verdict.append(("boot", boot_ok))
    out += ["## Boot"] + [f"- {n}" for n in notes] + [""]

    run_dir = None
    if not a.no_flight:
        scenario = find_scenario()
        if scenario is None:
            raise SystemExit("no 30 s lead-in scenario: run  verify_image.py --make-scenario  once")
        wait = hub_gap_ok()
        if wait > 0:
            log(f"waiting {wait:.0f} s for the 4 min gap since the last hub cycle")
            time.sleep(wait)
        session_no = erase()
        log(f"store erased, new session {session_no}; flying (30 s lead-in, ~9 min)")
        run, result, aperture = fly(scenario)
        run_dir = Path(run.run_dir)
        f = flight_numbers(run_dir, aperture)
        verdict.append(("HITL flight", bool(f["passed"])))
        verdict.append(("log is this run's", f["fc_ok"] and session_no != "?" and
                        f"{int(session_no):07X}" == f["session"]))
        out += ["## HITL flight", f"Run `{run_dir.name}`, scenario `{scenario.folder.name}`", "",
                "| State | Truth (s) | Witness (s) | Delta (s) |", "|---|---|---|---|"]
        out += [f"| {r['state']} | {r['expected_s']} | {r['witness_s']} | {r.get('delta_s')} |" for r in f["rows"]]
        out += ["", f"- Tool verdict: **{'PASS' if f['passed'] else 'FAIL'}** {f['problems'] or ''}",
                f"- First GPS fix after RF start: {f['first_fix_s']:.1f} s" if f["first_fix_s"] else "- No GPS fix",
                "- Filter up error max / RMS (m): " + ", ".join(
                    f"{ph} {mx:.0f}/{rms:.0f}" for ph, (mx, rms) in f["up"].items() if mx is not None),
                f"- Final tilt error: {f['tilt']} deg",
                f"- Heap minimum: {f['heap_min']} B; resets in log: {f['resets']}; panics: {f['panic']}",
                f"- Log export: {'OK' if f['fc_ok'] else 'FAILED'}, session {f['session']} "
                f"(erase created {session_no})",
                f"- Radio state packets relayed by Aperture: {f['radio_frames']}",
                f"- Bench link: {f['rejects']} HITL frames rejected, {f['unknown']} unknown commands "
                "(known network-link issue, not firmware)", ""]
        log("power-up checks after the tool's hub cycle")
        time.sleep(20)
        e = end_checks()
        gps_ok = bool(re.search(r"baud=11[45]\d{3}\b", e["gps"])) and "recoveries 0" in e["gps"] and not e["slow"]
        verdict.append(("GPS power-up", gps_ok))
        out += ["## After the hub power cycle", f"- {e['gps']}", f"- Last reset: {e['reset']}",
                f"- Rate command skipped: {e['rate_skip'][-1] if e['rate_skip'] else 'line not captured'}",
                f"- Slow-state warning: {'YES' if e['slow'] else 'no'}",
                f"- Store erased, new session {e['session']}", "", "Commanded BIT:", "```"] + e["bit"] + ["```", ""]

    out.insert(4, "**Result: " + ", ".join(f"{k} {'OK' if v else 'FAIL'}" for k, v in verdict) +
               f"** ({(time.time() - t_start) / 60:.1f} min)\n")
    summary = "\n".join(out)
    target = (run_dir or (HITL / "runs")) / ("verify_summary.md" if run_dir else
                                             f"verify_summary_{time.strftime('%Y%m%d-%H%M%S')}.md")
    target.write_text(summary, encoding="utf-8")
    print(summary)
    log(f"summary: {target}")
    return 0 if all(v for _, v in verdict) else 1


if __name__ == "__main__":
    sys.exit(main())
