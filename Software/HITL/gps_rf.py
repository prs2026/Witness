"""RF GPS simulation for HITL: gps-sdr-sim scenarios played into Witness's
real GPS receiver with a HackRF.

Pipeline
--------
1. ``prepare_nav``: a RINEX broadcast-ephemeris file is moved to the scenario
   start time (``Horizon/gps_assist.shift_to``) and written as RINEX 2 for
   gps-sdr-sim. The same file is what ``ephemeris upload`` sends to the
   receiver, so uploaded and transmitted ephemerides match.
2. ``create_static`` / ``create_flight``: a gps-sdr-sim IQ file (8-bit I/Q,
   2.6 MS/s, L1 C/A) for a fixed position or for a HITL trajectory. A flight
   file starts with ``lead_in_s`` of static signal at the pad (time for the
   receiver to acquire), then follows the trajectory from sim time 0 at the
   10 Hz motion rate gps-sdr-sim uses. Each scenario is a folder with
   ``scenario.json`` (start time, lead-in, site, geoid separation, PRNs),
   ``nav.n``, ``motion.csv`` and ``iq.bin``.
3. ``verify_iq``: offline acquisition of the C/A codes in the IQ file
   (FFT code-phase search over Doppler) to confirm the expected satellites
   are present before anything is transmitted.
4. ``HackRFPlayer``: ``hackrf_transfer -t`` at 1575.42 MHz.
   **Only into a closed RF path** (coax and attenuators into the receiver's
   antenna input, or a shielded enclosure): radiating L1 is illegal and
   spoofs every receiver nearby. TX VGA defaults to 0 dB and the TX amplifier
   is off.
5. ``static_test``: plays a static scenario and watches Witness's NMEA until a
   fix, then checks the reported position against the scenario (and records
   the receiver's geoid separation, needed to convert MSL altitude to the
   ellipsoid height gps-sdr-sim takes).

Altitude: gps-sdr-sim positions are WGS-84 ellipsoid heights; GGA reports
mean-sea-level altitude = ellipsoid height - geoid separation N (GGA field
11). The motion file uses h = MSL + N, with N from the static test at the
site (``scenario.json`` records which N was used).
"""

from __future__ import annotations

import dataclasses
import datetime as dt
import json
import math
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "Horizon"))
import gps_assist as ga  # noqa: E402
from sim_model import LAUNCH_ALT_MSL_M, LAUNCH_LAT_DEG, LAUNCH_LON_DEG  # noqa: E402

RF_DIR = HERE / "rf_tools"
BIN_DIR = RF_DIR / "bin"
SCENARIO_DIR = HERE / "gps_sim"
SITE_FILE = RF_DIR / "sites.json"          # geoid separation measured per site
GPS_SDR_SIM_SOURCE = HERE.parents[2] / "gps-sdr-sim"   # sibling of the Witness repo
L1_HZ = 1575420000
SAMPLE_RATE = 2600000
MOTION_RATE_HZ = 10
DEFAULT_LEAD_IN_S = 90.0
EXE = ".exe" if os.name == "nt" else ""
CA_CHIP_RATE = 1.023e6


# ------------------------------------------------------------------ tools
def gps_sdr_sim_path() -> Path | None:
    for candidate in (BIN_DIR / f"gps-sdr-sim{EXE}",):
        if candidate.exists():
            return candidate
    found = shutil.which("gps-sdr-sim")
    return Path(found) if found else None


def build_gps_sdr_sim(source: Path = GPS_SDR_SIM_SOURCE, compiler: str = "gcc") -> Path:
    """Compile gps-sdr-sim from the sibling checkout into rf_tools/bin, with a
    10,000 s dynamic-motion limit (the stock build stops at 300 s)."""
    source = Path(source)
    if not (source / "gpssim.c").exists():
        raise FileNotFoundError(f"gps-sdr-sim source not found at {source}")
    BIN_DIR.mkdir(parents=True, exist_ok=True)
    out = BIN_DIR / f"gps-sdr-sim{EXE}"
    sources = [str(source / "gpssim.c")]
    if os.name == "nt" and (source / "getopt.c").exists():
        sources.append(str(source / "getopt.c"))
    cmd = [compiler, "-O3", "-DUSER_MOTION_SIZE=100000", "-o", str(out), *sources, "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"gps-sdr-sim build failed:\n{result.stderr[-2000:]}")
    return out


def hackrf_transfer_path(configured: str | None = None) -> Path | None:
    candidates = []
    if configured:
        candidates.append(Path(configured))
    candidates += [BIN_DIR / f"hackrf_transfer{EXE}"]
    for root in (os.environ.get("ProgramFiles", r"C:\Program Files"),
                 os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")):
        candidates += [Path(root) / "PothosSDR" / "bin" / f"hackrf_transfer{EXE}",
                       Path(root) / "HackRF" / "bin" / f"hackrf_transfer{EXE}"]
    home = Path.home()
    candidates += [home / "radioconda" / "Library" / "bin" / f"hackrf_transfer{EXE}",
                   home / "miniforge3" / "Library" / "bin" / f"hackrf_transfer{EXE}"]
    for c in candidates:
        if c.exists():
            return c
    found = shutil.which("hackrf_transfer")
    return Path(found) if found else None


# ------------------------------------------------------------- geodesy
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def llh_to_ecef(lat_deg, lon_deg, h_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + h_m) * math.cos(lat) * math.cos(lon), (n + h_m) * math.cos(lat) * math.sin(lon),
            (n * (1 - WGS84_E2) + h_m) * math.sin(lat))


def horizontal_error_m(lat1, lon1, lat2, lon2) -> float:
    m_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat1))
    m_lon = 111412.84 * math.cos(math.radians(lat1))
    return math.hypot((lat2 - lat1) * m_lat, (lon2 - lon1) * m_lon)


def load_sites() -> dict:
    try:
        return json.loads(SITE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def site_key(lat, lon) -> str:
    return f"{lat:.2f},{lon:.2f}"


def geoid_separation(lat, lon) -> float | None:
    """N (m) measured by a static test within ~1 km of this site, or None."""
    entry = load_sites().get(site_key(lat, lon))
    return None if entry is None else float(entry["geoid_sep_m"])


def save_geoid_separation(lat, lon, n_m, source="static test") -> None:
    sites = load_sites()
    sites[site_key(lat, lon)] = {"geoid_sep_m": round(float(n_m), 2), "source": source,
                                 "measured_utc": dt.datetime.now(dt.timezone.utc).replace(tzinfo=None).isoformat(timespec="seconds")}
    RF_DIR.mkdir(parents=True, exist_ok=True)
    SITE_FILE.write_text(json.dumps(sites, indent=2), encoding="utf-8")


# ------------------------------------------------------------- scenarios
@dataclasses.dataclass
class Scenario:
    folder: Path
    kind: str                       # "static" | "flight"
    start_gps: str                  # ISO GPS time of IQ sample 0
    duration_s: float
    lead_in_s: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    geoid_sep_m: float
    sample_rate: int
    prns: list
    source: str = ""                # trajectory / description
    rinex_source: str = ""
    created_utc: str = ""
    iq_bytes: int = 0
    verify: dict | None = None
    visible_prns: list = dataclasses.field(default_factory=list)   # at the start

    @property
    def iq(self) -> Path:
        return self.folder / "iq.bin"

    @property
    def nav(self) -> Path:
        return self.folder / "nav.n"

    def save(self) -> None:
        data = dataclasses.asdict(self)
        data["folder"] = str(self.folder)
        (self.folder / "scenario.json").write_text(json.dumps(data, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, folder) -> "Scenario":
        folder = Path(folder)
        data = json.loads((folder / "scenario.json").read_text(encoding="utf-8"))
        data["folder"] = folder
        known = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})


def list_scenarios(base: Path = SCENARIO_DIR) -> list:
    out = []
    if base.exists():
        for p in sorted(base.iterdir()):
            if (p / "scenario.json").exists():
                try:
                    out.append(Scenario.load(p))
                except (OSError, ValueError, TypeError):
                    pass
    return out


def prepare_nav(rinex, start_gps: dt.datetime, out_path: Path) -> tuple[Path, list]:
    """Shift `rinex` to the scenario start and write it as RINEX 2."""
    nav = ga.parse_rinex_nav(rinex)
    shifted, _ = ga.shift_to(nav, start_gps)
    ga.write_rinex2(shifted, out_path)
    prns = sorted(ga.select_for(shifted, start_gps))
    return out_path, prns


def default_rinex() -> Path:
    """The newest RINEX nav file in rf_tools/ or gps-sdr-sim's bundled one."""
    found = sorted([*RF_DIR.glob("*.??n"), *RF_DIR.glob("*.rnx")], key=lambda p: p.stat().st_mtime)
    if found:
        return found[-1]
    bundled = GPS_SDR_SIM_SOURCE / "brdc0010.22n"
    if bundled.exists():
        return bundled
    raise FileNotFoundError("no RINEX navigation file: put one in HITL/rf_tools/")


def _run_gps_sdr_sim(args, progress=None, cancel: threading.Event | None = None) -> str:
    exe = gps_sdr_sim_path()
    if exe is None:
        exe = build_gps_sdr_sim()
    proc = subprocess.Popen([str(exe), *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    text, buf = [], b""
    while True:
        chunk = proc.stdout.read(256)
        if not chunk:
            break
        buf += chunk
        parts = re.split(rb"[\r\n]", buf)
        buf = parts.pop()
        for part in parts:
            line = part.decode(errors="replace").strip()
            if not line:
                continue
            m = re.match(r"Time into run =\s*([\d.]+)", line)
            if m:
                if progress:
                    progress(float(m.group(1)))
            else:
                text.append(line)
        if cancel is not None and cancel.is_set():
            proc.kill()
            raise RuntimeError("cancelled")
    proc.stdout.close()
    code = proc.wait()
    if code != 0:
        raise RuntimeError("gps-sdr-sim failed:\n" + "\n".join(text[-20:]))
    return "\n".join(text)


def visible_prns(output: str) -> list:
    """PRNs gps-sdr-sim listed as channels (PRN az el range iono)."""
    return sorted({int(m.group(1)) for m in re.finditer(
        r"^(\d\d)\s+[\d.]+\s+-?[\d.]+\s+[\d.]+\s+[\d.]+\s*$", output, re.M)})


def _new_folder(name: str, base: Path) -> Path:
    base.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    folder = base / f"{stamp}-{re.sub(r'[^A-Za-z0-9_.-]+', '_', name)}"
    folder.mkdir()
    return folder


def start_time(start_gps: dt.datetime | None) -> dt.datetime:
    """Scenario start (GPS time), whole seconds; default now."""
    t = start_gps or ga.gps_now()
    return t.replace(microsecond=0)


def create_static(lat_deg, lon_deg, alt_msl_m, duration_s=300.0, *, rinex=None,
                  start_gps=None, geoid_sep_m=None, name="static", base=SCENARIO_DIR,
                  progress=None, cancel=None, sample_rate=SAMPLE_RATE) -> Scenario:
    """A fixed-position scenario (static tests)."""
    t0 = start_time(start_gps)
    n = geoid_separation(lat_deg, lon_deg) if geoid_sep_m is None else geoid_sep_m
    folder = _new_folder(name, base)
    rinex = Path(rinex) if rinex else default_rinex()
    nav, prns = prepare_nav(rinex, t0, folder / "nav.n")
    h = alt_msl_m + (n or 0.0)
    out = _run_gps_sdr_sim(["-e", str(nav), "-l", f"{lat_deg:.9f},{lon_deg:.9f},{h:.3f}",
                      "-t", t0.strftime("%Y/%m/%d,%H:%M:%S"), "-d", f"{duration_s:.0f}",
                      "-b", "8", "-s", str(sample_rate), "-o", str(folder / "iq.bin")],
                     progress, cancel)
    sc = Scenario(folder=folder, kind="static", start_gps=t0.isoformat(), duration_s=float(duration_s),
                  lead_in_s=0.0, lat_deg=lat_deg, lon_deg=lon_deg, alt_msl_m=alt_msl_m,
                  geoid_sep_m=float(n or 0.0), sample_rate=sample_rate, prns=prns,
                  source=f"static {lat_deg:.6f}, {lon_deg:.6f}, {alt_msl_m:.1f} m MSL",
                  rinex_source=str(rinex), created_utc=dt.datetime.now(dt.timezone.utc).replace(tzinfo=None).isoformat(timespec="seconds"),
                  iq_bytes=(folder / "iq.bin").stat().st_size, visible_prns=visible_prns(out))
    sc.save()
    return sc


def motion_rows(samples, lead_in_s, geoid_sep_m):
    """10 Hz (t, lat, lon, h) rows: lead-in at the first sample's position,
    then every 10th 100 Hz HITL sample (truth position, MSL -> ellipsoid)."""
    first = samples[0].hitl
    rows = []
    n_lead = int(round(lead_in_s * MOTION_RATE_HZ))
    for k in range(n_lead):
        rows.append((k / MOTION_RATE_HZ, first.lat_deg, first.lon_deg, samples[0].alt_msl_m + geoid_sep_m))
    step = max(1, int(round(100 / MOTION_RATE_HZ)))
    for s in samples[::step]:
        rows.append((lead_in_s + s.t_ms / 1000.0, s.hitl.lat_deg, s.hitl.lon_deg, s.alt_msl_m + geoid_sep_m))
    return rows


def create_flight(samples, *, site_lat, site_lon, site_alt_msl, lead_in_s=DEFAULT_LEAD_IN_S,
                  rinex=None, start_gps=None, geoid_sep_m=None, name="flight", source="",
                  base=SCENARIO_DIR, progress=None, cancel=None, sample_rate=SAMPLE_RATE) -> Scenario:
    """A scenario that follows HITL samples (sm.SimSample list, 100 Hz)."""
    if not samples:
        raise ValueError("no samples")
    t0 = start_time(start_gps)
    n = geoid_separation(site_lat, site_lon) if geoid_sep_m is None else geoid_sep_m
    folder = _new_folder(name, base)
    rinex = Path(rinex) if rinex else default_rinex()
    nav, prns = prepare_nav(rinex, t0, folder / "nav.n")
    rows = motion_rows(samples, lead_in_s, n or 0.0)
    with open(folder / "motion.csv", "w", encoding="ascii") as f:
        for t, lat, lon, h in rows:
            f.write(f"{t:.1f},{lat:.9f},{lon:.9f},{h:.3f}\n")
    duration = len(rows) / MOTION_RATE_HZ
    out = _run_gps_sdr_sim(["-e", str(nav), "-x", str(folder / "motion.csv"),
                      "-t", t0.strftime("%Y/%m/%d,%H:%M:%S"), "-d", f"{duration:.1f}",
                      "-b", "8", "-s", str(sample_rate), "-o", str(folder / "iq.bin")],
                     progress, cancel)
    sc = Scenario(folder=folder, kind="flight", start_gps=t0.isoformat(), duration_s=duration,
                  lead_in_s=float(lead_in_s), lat_deg=site_lat, lon_deg=site_lon,
                  alt_msl_m=site_alt_msl, geoid_sep_m=float(n or 0.0), sample_rate=sample_rate,
                  prns=prns, source=source, rinex_source=str(rinex),
                  created_utc=dt.datetime.now(dt.timezone.utc).replace(tzinfo=None).isoformat(timespec="seconds"),
                  iq_bytes=(folder / "iq.bin").stat().st_size, visible_prns=visible_prns(out))
    sc.save()
    return sc


def ephemeris_frames(scenario: Scenario, at_s: float = 0.0) -> tuple[list, list]:
    """(0x05 frames, PRNs) uploading the scenario's ephemerides to Witness."""
    nav = ga.parse_rinex_nav(scenario.nav)
    when = dt.datetime.fromisoformat(scenario.start_gps) + dt.timedelta(seconds=at_s)
    blob, prns = ga.ephemeris_upload(nav, when)
    return ga.assist_frames(blob), prns


# ------------------------------------------------------- offline check
def ca_code(prn: int):
    """1023-chip C/A code (+1/-1) for PRN 1-32 (IS-GPS-200 G2 delays)."""
    import numpy as np
    delays = [5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470, 471,
              472, 473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862]
    g1, g2 = [1] * 10, [1] * 10
    g1_out, g2_out = [], []
    for _ in range(1023):
        g1_out.append(g1[9])
        g2_out.append(g2[9])
        f1 = g1[2] ^ g1[9]
        f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [f1] + g1[:9]
        g2 = [f2] + g2[:9]
    d = delays[prn - 1]
    chips = [g1_out[i] ^ g2_out[(i - d) % 1023] for i in range(1023)]
    return 1 - 2 * np.array(chips, dtype=np.float32)


def verify_iq(iq_path, sample_rate=SAMPLE_RATE, offset_s=1.0, ms=10, prns=range(1, 33),
              doppler_hz=6000, step_hz=250) -> dict:
    """Acquire C/A codes in an 8-bit I/Q file: per PRN the best correlation
    peak over Doppler, relative to the noise floor. Returns {prn: ratio} and
    the list of PRNs above the detection threshold."""
    import numpy as np
    n = int(sample_rate * 1e-3)
    count = n * ms
    with open(iq_path, "rb") as f:
        f.seek(int(offset_s * sample_rate) * 2)
        raw = np.frombuffer(f.read(count * 2), dtype=np.int8).astype(np.float32)
    if raw.size < count * 2:
        raise ValueError("IQ file shorter than the check window")
    x = (raw[0::2] + 1j * raw[1::2]).reshape(ms, n)
    t = np.arange(n) / sample_rate
    results = {}
    for prn in prns:
        code = ca_code(prn)
        idx = (np.floor(np.arange(n) * CA_CHIP_RATE / sample_rate) % 1023).astype(int)
        cf = np.conj(np.fft.fft(code[idx]))
        best, floor = 0.0, 1e-9
        for fd in np.arange(-doppler_hz, doppler_hz + 1, step_hz):
            wipe = np.exp(-2j * np.pi * fd * t)
            power = np.zeros(n)
            for k in range(ms):        # non-coherent sum of 1 ms correlations
                power += np.abs(np.fft.ifft(np.fft.fft(x[k] * wipe) * cf)) ** 2
            peak = power.max()
            if peak > best:
                best, floor = peak, np.median(power)
        results[prn] = float(best / floor)
    threshold = 14.0   # peak/median; noise-only PRNs reach about 7-11 with 10 ms
    return {"ratio": results, "detected": sorted(p for p, r in results.items() if r > threshold),
            "threshold": threshold}


# ------------------------------------------------------------- playback
# hackrf_transfer's once-per-second report, e.g.
# "5.2 MB / 1.014 sec =  5.2 MB/second, average power -11.5 dBfs" (older
# releases print MiB).
RATE_LINE = re.compile(r"Mi?B\s*/\s*[\d.]+\s*sec")


class HackRFPlayer:
    """``hackrf_transfer -t`` in a background process.

    started_at is the wall time (time.monotonic) when the HackRF began
    streaming, from its first per-second rate report (minus one second), as
    the alignment reference for HITL timing.
    """

    def __init__(self, iq_path, *, sample_rate=SAMPLE_RATE, tx_vga_db=0, amp=False,
                 exe=None, log=print):
        self.iq_path = Path(iq_path)
        self.sample_rate = int(sample_rate)
        self.tx_vga_db = int(max(0, min(47, tx_vga_db)))
        self.amp = bool(amp)
        self.exe = hackrf_transfer_path(exe)
        self.log = log
        self.proc = None
        self.launched_at = None
        self.started_at = None
        self.lines: queue.Queue = queue.Queue()
        self._reader = None

    def command(self) -> list:
        if self.exe is None:
            raise FileNotFoundError(
                "hackrf_transfer not found: put hackrf_transfer.exe (HackRF host tools) in "
                f"{BIN_DIR} or on PATH")
        # "-t -": samples come from our stdin pipe (see start/stop).
        return [str(self.exe), "-t", "-", "-f", str(L1_HZ), "-s", str(self.sample_rate),
                "-x", str(self.tx_vga_db), "-a", "1" if self.amp else "0"]

    def start(self):
        cmd = self.command()
        self.log("HackRF: " + " ".join(cmd))
        self.launched_at = time.monotonic()
        # The IQ file is fed through stdin. Stopping closes the pipe, so
        # hackrf_transfer sees end of file and stops TX and releases the
        # device itself. (A console-less process gets no Ctrl-C/Break, and a
        # forced kill on Windows can leave it stuck in the USB driver with the
        # HackRF claimed until it is replugged.)
        self._stop_feed = threading.Event()
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self._reader = threading.Thread(target=self._read, name="hackrf-transfer", daemon=True)
        self._reader.start()
        self._feeder = threading.Thread(target=self._feed, name="hackrf-feed", daemon=True)
        self._feeder.start()

    def _feed(self):
        chunk = 1 << 18           # 256 KiB = 50 ms at 2.6 MS/s
        try:
            with open(self.iq_path, "rb") as f:
                while not self._stop_feed.is_set():
                    data = f.read(chunk)
                    if not data:
                        break
                    self.proc.stdin.write(data)
        except (OSError, ValueError):
            pass                  # process gone / pipe closed
        finally:
            try:
                self.proc.stdin.close()
            except OSError:
                pass

    def _read(self):
        for raw in iter(self.proc.stdout.readline, b""):
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if self.started_at is None and RATE_LINE.search(line):
                self.started_at = time.monotonic() - 1.0
            self.lines.put(line)

    def wait_streaming(self, timeout=10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.started_at is not None:
                return True
            if self.proc is not None and self.proc.poll() is not None:
                return False
            time.sleep(0.05)
        return False

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def elapsed_s(self) -> float:
        ref = self.started_at if self.started_at is not None else self.launched_at
        return 0.0 if ref is None else time.monotonic() - ref

    def output(self) -> list:
        out = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                return out

    def stop(self) -> bool:
        """Stop transmitting; True once the process has exited (the HackRF
        is free). Closing stdin first (clean), then terminate, then kill."""
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return True
        self._stop_feed.set()         # feeder closes stdin -> EOF -> clean exit
        try:
            proc.wait(timeout=6)
            return True
        except subprocess.TimeoutExpired:
            pass
        for step in (proc.terminate, proc.kill):   # last resort
            try:
                step()
                proc.wait(timeout=4)
                return True
            except (OSError, subprocess.TimeoutExpired):
                continue
        self.log(f"HackRF: hackrf_transfer (pid {proc.pid}) did not exit; the HackRF stays busy")
        return False


# ------------------------------------------------------------ NMEA tools
def parse_gga(line: str) -> dict | None:
    m = re.search(r"\$(G[PNL])GGA,([^*]*)\*", line)
    if not m:
        return None
    f = m.group(2).split(",")
    if len(f) < 11:
        return None

    def coord(v, hemi, lat):
        if not v:
            return None
        deg = int(float(v) / 100)
        value = deg + (float(v) - deg * 100) / 60
        return -value if hemi in ("S", "W") else value
    try:
        quality = int(f[5] or 0)
        out = {"utc": f[0], "quality": quality, "sats": int(f[6] or 0),
               "hdop": float(f[7]) if f[7] else None}
        if quality > 0:
            out.update(lat=coord(f[1], f[2], True), lon=coord(f[3], f[4], False),
                       alt_msl=float(f[8]) if f[8] else None,
                       geoid_sep=float(f[10]) if len(f) > 10 and f[10] else None)
        return out
    except ValueError:
        return None


def parse_gsa(line: str) -> dict | None:
    m = re.search(r"\$(G[PNL])GSA,([^*]*)\*", line)
    if not m:
        return None
    f = m.group(2).split(",")
    if len(f) < 17:
        return None
    try:
        return {"fix_type": int(f[1] or 1), "sv": [int(v) for v in f[2:14] if v],
                "pdop": float(f[14]) if f[14] else None, "hdop": float(f[15]) if f[15] else None,
                "vdop": float(f[16].split(",")[0]) if f[16] else None}
    except ValueError:
        return None


@dataclasses.dataclass
class StaticResult:
    passed: bool
    ttff_s: float | None
    fixes: int
    horizontal_error_m: float | None
    vertical_error_m: float | None
    sats: int
    hdop: float | None
    vdop: float | None
    geoid_sep_m: float | None
    message: str
    nmea_lines: int = 0


def static_test(scenario: Scenario, nmea_source, *, player: HackRFPlayer | None = None,
                timeout_s=180.0, settle_fixes=25, max_horizontal_m=30.0, max_vertical_m=50.0,
                log=print, cancel: threading.Event | None = None, upload=None) -> StaticResult:
    """Play `scenario` and judge Witness's fix.

    nmea_source(): returns new NMEA lines (strings) since the last call.
    upload(frames): optional, sends the ephemeris upload once streaming.
    """
    own = player is None
    player = player or HackRFPlayer(scenario.iq, sample_rate=scenario.sample_rate, log=log)
    if own:
        player.start()
    if not player.wait_streaming(15):
        out = player.output()
        player.stop()
        return StaticResult(False, None, 0, None, None, 0, None, None, None,
                            "HackRF did not start streaming: " + (" | ".join(out[-5:]) or "no output"))
    log(f"HackRF streaming {scenario.iq.name}; waiting up to {timeout_s:.0f} s for a fix")
    if upload is not None:
        frames, prns = ephemeris_frames(scenario, player.elapsed_s())
        upload(frames)
        log(f"ephemeris upload sent: {len(prns)} satellites, {len(frames)} frames")
    first_fix = None
    fixes, errs_h, errs_v, last_gga, last_gsa, lines = 0, [], [], None, None, 0
    settle_started = None
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                raise RuntimeError("cancelled")
            if not player.running:
                break
            for line in nmea_source():
                lines += 1
                gsa = parse_gsa(line)
                if gsa and gsa.get("fix_type", 1) >= 2:
                    last_gsa = gsa
                gga = parse_gga(line)
                if not gga:
                    continue
                last_gga = gga
                if gga["quality"] > 0 and gga.get("lat") is not None:
                    if first_fix is None:
                        first_fix = player.elapsed_s()
                        log(f"first fix after {first_fix:.1f} s: {gga['sats']} satellites, "
                            f"HDOP {gga['hdop']}")
                    fixes += 1
                    errs_h.append(horizontal_error_m(scenario.lat_deg, scenario.lon_deg,
                                                     gga["lat"], gga["lon"]))
                    if gga.get("alt_msl") is not None:
                        # The scenario was generated with h = MSL + N_used;
                        # the receiver reports MSL = h - N_receiver.
                        n_rx = gga.get("geoid_sep") or 0.0
                        expected_msl = scenario.alt_msl_m + scenario.geoid_sep_m - n_rx
                        errs_v.append(gga["alt_msl"] - expected_msl)
            # Enough fixes; wait up to 8 s more for a GSA (DOPs come every 5 s).
            if fixes >= settle_fixes and (last_gsa is not None or settle_started is not None
                                          and time.monotonic() - settle_started > 8.0):
                break
            if fixes >= settle_fixes and settle_started is None:
                settle_started = time.monotonic()
            time.sleep(0.05)
    finally:
        if own:
            player.stop()
    tail_h = errs_h[-settle_fixes:]
    tail_v = errs_v[-settle_fixes:]
    h = sorted(tail_h)[len(tail_h) // 2] if tail_h else None
    v = sorted(tail_v)[len(tail_v) // 2] if tail_v else None
    n_rx = (last_gga or {}).get("geoid_sep")
    passed = fixes >= settle_fixes and h is not None and h <= max_horizontal_m and \
        (v is None or abs(v) <= max_vertical_m)
    msg = ("PASS" if passed else "FAIL") + (
        f": first fix {first_fix:.1f} s after TX start, median horizontal error {h:.1f} m"
        + (f", vertical {v:+.1f} m" if v is not None else "") if first_fix is not None else
        f": no fix within {timeout_s:.0f} s ({lines} NMEA lines"
        + (f", last GGA {last_gga['sats']} sats quality {last_gga['quality']}" if last_gga else "") + ")")
    if n_rx is not None and first_fix is not None:
        save_geoid_separation(scenario.lat_deg, scenario.lon_deg, n_rx)
        msg += f"; receiver geoid separation {n_rx:+.1f} m saved for this site"
    return StaticResult(passed, first_fix, fixes, h, v, (last_gga or {}).get("sats", 0),
                        (last_gga or {}).get("hdop"), (last_gsa or {}).get("vdop"), n_rx, msg, lines)


# -------------------------------------------------------------------- CLI
def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="gps-sdr-sim + HackRF tools for Witness HITL")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("build", help="compile gps-sdr-sim into rf_tools/bin")
    st = sub.add_parser("static", help="create a static scenario")
    st.add_argument("--lat", type=float, default=LAUNCH_LAT_DEG)
    st.add_argument("--lon", type=float, default=LAUNCH_LON_DEG)
    st.add_argument("--alt", type=float, default=LAUNCH_ALT_MSL_M, help="m MSL (default: FAR)")
    st.add_argument("--duration", type=float, default=300.0)
    st.add_argument("--rinex")
    vf = sub.add_parser("verify", help="acquire the C/A codes in a scenario's IQ file")
    vf.add_argument("scenario")
    sub.add_parser("list", help="list scenarios")
    tt = sub.add_parser("test", help="static RF test: transmit a fixed position and check "
                                     "Witness's GPS fix (Witness must not be in HITL)")
    tt.add_argument("--lat", type=float, default=LAUNCH_LAT_DEG)
    tt.add_argument("--lon", type=float, default=LAUNCH_LON_DEG)
    tt.add_argument("--alt", type=float, default=LAUNCH_ALT_MSL_M, help="m MSL (default: FAR)")
    tt.add_argument("--timeout", type=float, default=180.0, help="seconds to wait for a fix")
    tt.add_argument("--tx-gain", type=int, default=0, help="HackRF TX VGA dB (0-47)")
    tt.add_argument("--host", default="prs2026hitl")
    tt.add_argument("--port", type=int, help="Witness raw port (default: found by listening)")
    tt.add_argument("--no-upload", action="store_true", help="skip the ephemeris upload")
    tt.add_argument("--cold-start", action="store_true",
                    help="cold-start the receiver first (clears stored ephemerides, almanac, "
                         "position and time; Pad Idle only)")
    tx = sub.add_parser("tx", help="transmit only (watch the NMEA yourself); Ctrl-C stops")
    tx.add_argument("--lat", type=float, default=LAUNCH_LAT_DEG)
    tx.add_argument("--lon", type=float, default=LAUNCH_LON_DEG)
    tx.add_argument("--alt", type=float, default=LAUNCH_ALT_MSL_M, help="m MSL (default: FAR)")
    tx.add_argument("--duration", type=float, default=600.0, help="seconds of signal to generate")
    tx.add_argument("--tx-gain", type=int, default=0, help="HackRF TX VGA dB (0-47)")
    tx.add_argument("--amp", action="store_true", help="HackRF TX amplifier on (+14 dB)")
    tx.add_argument("--scenario", help="play an existing scenario folder instead")
    tx.add_argument("--keep", action="store_true", help="keep a generated scenario afterwards")
    args = ap.parse_args(argv)
    if args.cmd == "tx":
        if args.scenario:
            sc, made = Scenario.load(args.scenario), False
        else:
            print(f"generating {args.duration:.0f} s of signal at {args.lat}, {args.lon}, "
                  f"{args.alt} m MSL ...")
            sc, made = create_static(args.lat, args.lon, args.alt, args.duration, name="tx"), True
        player = HackRFPlayer(sc.iq, sample_rate=sc.sample_rate, tx_vga_db=args.tx_gain,
                              amp=args.amp, log=print)
        try:
            player.start()
            if not player.wait_streaming(15):
                print("HackRF did not start: " + " | ".join(player.output()[-5:]))
                return 1
            print(f"transmitting {sc.folder.name} ({sc.duration_s:.0f} s, {len(sc.visible_prns)} "
                  f"satellites in view, GPS time {sc.start_gps}); Ctrl-C to stop")
            while player.running:
                time.sleep(1)
                print(f"\r  {player.elapsed_s():6.0f} s", end="", flush=True)
            print("\nend of file")
        except KeyboardInterrupt:
            print("\nstopping")
        finally:
            print("HackRF released" if player.stop() else "HackRF did not stop: run rf_tools\\restart_hackrf.cmd")
            if made and not args.keep:
                shutil.rmtree(sc.folder, ignore_errors=True)
        return 0
    if args.cmd == "test":
        import bench_ports
        import hitl_session as hs
        port = args.port or bench_ports.witness_port(args.host)
        print(f"Witness on {args.host}:{port}")
        session = hs.HitlSession(runs_dir=None)
        session.connect(args.host, port, timeout=5)
        try:
            time.sleep(2)
            if args.cold_start:
                print("cold-starting the GPS receiver ...")
                if not session.cold_start_gps():
                    print("cold start not confirmed (is Witness in Pad Idle?)")
                    return 1
            print("creating the static scenario ...")
            sc = create_static(args.lat, args.lon, args.alt, args.timeout + 30, name="static-test")
            player = HackRFPlayer(sc.iq, sample_rate=sc.sample_rate, tx_vga_db=args.tx_gain, log=print)
            try:
                player.start()
                result = static_test(sc, session.nmea_since, player=player, timeout_s=args.timeout,
                                     log=print, upload=None if args.no_upload else session.upload_assist)
            finally:
                player.stop()
            print(result.message)
            print(f"satellites {result.sats}, HDOP {result.hdop}, VDOP {result.vdop}, "
                  f"NMEA lines {result.nmea_lines}")
            return 0 if result.passed else 1
        finally:
            session.disconnect(send_exit=False)
            shutil.rmtree(sc.folder, ignore_errors=True) if "sc" in locals() else None
    if args.cmd == "build":
        print(build_gps_sdr_sim())
    elif args.cmd == "static":
        sc = create_static(args.lat, args.lon, args.alt, args.duration, rinex=args.rinex,
                           progress=None)
        print(sc.folder, sc.prns)
    elif args.cmd == "verify":
        sc = Scenario.load(args.scenario)
        r = verify_iq(sc.iq, sc.sample_rate)
        print("detected", r["detected"], "visible in the scenario", sc.visible_prns)
    elif args.cmd == "list":
        for sc in list_scenarios():
            print(sc.folder.name, sc.kind, f"{sc.duration_s:.0f} s", sc.source)
    return 0


if __name__ == "__main__":
    sys.exit(main())
