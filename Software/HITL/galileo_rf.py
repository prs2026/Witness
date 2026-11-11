"""Galileo E1 OS signals for RF HITL, mixed into the gps-sdr-sim IQ.

galileo-sdr-sim (https://github.com/harshadms/galileo-sdr-sim, GPL-3) makes a
real E1-B/C signal with a full I/NAV message from a RINEX 3 navigation file.
It is a separate checkout next to the Witness repo (like gps-sdr-sim); its
``witness-windows`` branch adds a MinGW file-sink build, a 10 Hz motion file
(-x, the gps-sdr-sim format) and the sample rate as a build option. The
binary goes to rf_tools/bin/galileo-sdr-sim.exe, built for SAMPLE_RATE.

Galileo E1 shares the 1575.42 MHz carrier with GPS L1, so its IQ is added to
the GPS IQ sample by sample and the HackRF plays one stream. (GLONASS L1 at
~1602 MHz and BeiDou B1I at 1561.098 MHz would need a much wider stream.)

Time: Galileo System Time runs with GPS time (same seconds, weeks 1024 apart),
so both simulators take the same start string. Each constellation's
ephemerides are moved to the scenario hour separately (``shift_nav``): the
receiver only needs every satellite to be consistent with its own message.
"""

from __future__ import annotations

import datetime as dt
import re
import subprocess
import threading
from pathlib import Path

import gps_rf
from gps_rf import BIN_DIR, EXE, RF_DIR, SAMPLE_RATE, ga

GALILEO_SOURCE = gps_rf.HERE.parents[2] / "galileo-sdr-sim"   # sibling of the Witness repo
GALILEO_BRANCH = "witness-windows"
BUNDLED_NAV = GALILEO_SOURCE / "rinex_files" / "20feb2022.rnx"
INAV_SOURCES = (517, 257)       # data source flags of E1-B I/NAV records (what is simulated)
IODNAV_SETS_PER_HOUR = 64       # >= I/NAV sets per satellite in a daily file
# Per-satellite power relative to GPS. E1 OS minimum received power is
# -157 dBW against -158.5 dBW for L1 C/A, half of it in the pilot.
DEFAULT_GAL_DB = 0.0


# ------------------------------------------------------------------ tools
def galileo_sim_path() -> Path | None:
    exe = BIN_DIR / f"galileo-sdr-sim{EXE}"
    return exe if exe.exists() else None


def build_galileo_sim(source: Path = GALILEO_SOURCE, sample_rate: int = SAMPLE_RATE,
                      compiler: str = "g++") -> Path:
    """Compile the file-sink galileo-sdr-sim into rf_tools/bin (MinGW g++)."""
    if not (source / "compat").exists():
        raise FileNotFoundError(
            f"galileo-sdr-sim with the Windows build not found at {source} "
            f"(clone it next to the Witness repo and check out {GALILEO_BRANCH})")
    BIN_DIR.mkdir(parents=True, exist_ok=True)
    out = BIN_DIR / f"galileo-sdr-sim{EXE}"
    srcs = [str(p) for p in sorted((source / "src").glob("*.cpp"))
            if p.name not in ("usrp.cpp", "downloader.cpp")]
    cmd = [compiler, "-O3", "-std=gnu++14", f"-I{source / 'compat'}",
           f"-DTX_SAMPLERATE={int(sample_rate)}.0", "-D_USE_MATH_DEFINES", "-o", str(out),
           *srcs, str(source / "compat" / "no_device.cpp"), "-lws2_32", "-static", "-lpthread"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"galileo-sdr-sim build failed:\n{result.stderr[-2000:]}")
    return out


def default_nav() -> Path:
    """Newest RINEX 3 file with Galileo records in rf_tools/, else the one
    bundled with galileo-sdr-sim."""
    for p in sorted(RF_DIR.glob("*.rnx"), key=lambda p: p.stat().st_mtime, reverse=True):
        with open(p, encoding="ascii", errors="replace") as f:
            if any(line.startswith("E") and line[1:3].isdigit() for line in f):
                return p
    if BUNDLED_NAV.exists():
        return BUNDLED_NAV
    raise FileNotFoundError("no Galileo RINEX 3 navigation file: put one in HITL/rf_tools/")


# ------------------------------------------------------------------ RINEX
def _d(value: float) -> str:
    return f"{value:19.12E}"


def _field(line: str, k: int) -> float:
    """k-th 19-character field of a RINEX 3 orbit line (4 leading spaces)."""
    text = line[4 + 19 * k: 4 + 19 * (k + 1)].replace("D", "E").strip()
    return float(text) if text else 0.0


def _set_field(line: str, k: int, value: float) -> str:
    line = line.rstrip("\n").ljust(4 + 19 * 4)
    a = 4 + 19 * k
    return line[:a] + _d(value) + line[a + 19:]


def parse_galileo_nav(path) -> tuple[list, list]:
    """(header lines, records); a record is 8 raw lines of one E1-B I/NAV set."""
    with open(path, encoding="ascii", errors="replace") as f:
        lines = f.read().splitlines()
    end = next(i for i, l in enumerate(lines) if "END OF HEADER" in l)
    header, body = lines[:end + 1], lines[end + 1:]
    records, i = [], 0
    while i < len(body):
        line = body[i]
        if line.startswith("E") and line[1:3].isdigit() and i + 7 < len(body):
            rec = body[i:i + 8]
            i += 8
            if int(_field(rec[5], 1)) in INAV_SOURCES:
                records.append(rec)
        else:
            i += 1
    return header, records


def _toc(rec) -> dt.datetime:
    y, mo, d, h, mi, s = (int(x) for x in rec[0][4:23].split())
    return dt.datetime(y, mo, d, h, mi, s)


def shift_nav(src, start_gps: dt.datetime, out_path: Path) -> tuple[Path, list]:
    """Move every I/NAV record so the earliest clock time is the hour of
    `start_gps`, with a new IODnav per scenario; returns (file, PRNs that have
    a set at the start)."""
    header, records = parse_galileo_nav(src)
    if not records:
        raise ValueError(f"{src}: no Galileo E1-B I/NAV records")
    # The file's hour with the most satellites (a daily file starts with a
    # few sets from the day before) becomes the scenario hour.
    hours_: dict = {}
    for r in records:
        hours_.setdefault(_toc(r).replace(minute=0, second=0), set()).add(r[0][:3])
    first = max(sorted(hours_), key=lambda h: len(hours_[h]))
    target = start_gps.replace(minute=0, second=0, microsecond=0)
    delta = target - first
    hours = int((target - ga.GPS_EPOCH).total_seconds() // 3600)
    base = (hours * IODNAV_SETS_PER_HOUR) % 1024
    by_prn: dict = {}
    for r in records:
        by_prn.setdefault(r[0][:3], set()).add(_toc(r))
    out_lines = []
    week_t, sec_t = ga.gps_from_datetime(target)
    for line in header:
        if line.startswith("GAUT") and "TIME SYSTEM CORR" in line:
            # reference time at the scenario hour (A0/A1 kept)
            line = line[:38] + f"{int(sec_t // 3600) * 3600:7d}{week_t:5d}" + line[50:]
        out_lines.append(line)
    prns = set()
    for r in sorted(records, key=lambda r: (_toc(r), r[0][:3])):
        toc = _toc(r) + delta
        rank = sorted(by_prn[r[0][:3]]).index(_toc(r))
        rec = list(r)
        rec[0] = rec[0][:4] + toc.strftime("%Y %m %d %H %M %S") + rec[0][23:]
        week_e = int(_field(rec[5], 2))
        toe = ga.datetime_from_gps(week_e, _field(rec[3], 0)) + delta
        week, toe_sec = ga.gps_from_datetime(toe)
        rec[3] = _set_field(rec[3], 0, round(toe_sec))
        rec[5] = _set_field(rec[5], 2, week)
        ttx = ga.datetime_from_gps(week_e, _field(rec[7], 0)) + delta
        rec[7] = _set_field(rec[7], 0, round(ga.gps_from_datetime(ttx)[1]))
        rec[1] = _set_field(rec[1], 0, (base + rank) % 1024)
        out_lines.extend(rec)
        if abs((toc - start_gps).total_seconds()) < 4 * 3600:   # rinex.cpp epoch_matcher
            prns.add(int(r[0][1:3]))
    out_path.write_text("\n".join(out_lines) + "\n", encoding="ascii")
    return out_path, sorted(prns)


# ------------------------------------------------------------- generate
def generate(nav: Path, start_gps: dt.datetime, out_iq: Path, duration_s: float, *,
             motion: Path | None = None, llh=None, progress=None,
             cancel: threading.Event | None = None) -> str:
    """Run galileo-sdr-sim into a 16-bit IQ file; returns its text output."""
    exe = galileo_sim_path() or build_galileo_sim()
    args = [str(exe), "-e", str(nav), "-o", str(out_iq), "-U", "1", "-b", "1",
            "-t", start_gps.strftime("%Y/%m/%d,%H:%M:%S"), "-d", f"{duration_s:.1f}"]
    if motion is not None:
        args += ["-x", str(motion)]
    else:
        args += ["-l", f"{llh[0]:.9f},{llh[1]:.9f},{llh[2]:.3f}"]
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            cwd=str(out_iq.parent),
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
    out = "\n".join(text)
    if code != 0 or "ERROR" in out:
        raise RuntimeError("galileo-sdr-sim failed:\n" + "\n".join(text[-20:]))
    return out


# ------------------------------------------------------------------ mix
def _rms(path: Path, dtype, offset_samples: int, count: int) -> float:
    import numpy as np
    item = np.dtype(dtype).itemsize
    with open(path, "rb") as f:
        f.seek(offset_samples * 2 * item)
        x = np.frombuffer(f.read(count * 2 * item), dtype=dtype).astype(np.float64)
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def mix_into(gps_iq: Path, gal_iq: Path, out_iq: Path, *, n_gps: int, n_gal: int,
             gal_db: float = DEFAULT_GAL_DB, sample_rate: int = SAMPLE_RATE,
             progress=None) -> dict:
    """out = GPS (int8 IQ) + scaled Galileo (int16 IQ), as int8 IQ.

    Galileo is scaled so one Galileo satellite has the power of one GPS
    satellite (+gal_db): the RMS of each file over a second, divided by the
    root of its satellite count. The sum is clipped to int8."""
    import numpy as np
    probe = int(sample_rate)
    g_rms = _rms(gps_iq, np.int8, probe, probe)
    e_rms = _rms(gal_iq, np.int16, probe, probe)
    if g_rms <= 0 or e_rms <= 0:
        raise ValueError("empty IQ file")
    per_gps = g_rms / max(1, n_gps) ** 0.5
    per_gal = e_rms / max(1, n_gal) ** 0.5
    scale = per_gps / per_gal * 10 ** (gal_db / 20)
    chunk = int(sample_rate) // 10 * 2           # 0.1 s of interleaved I/Q
    total = gps_iq.stat().st_size
    clipped = done = 0
    with open(gps_iq, "rb") as fg, open(gal_iq, "rb") as fe, open(out_iq, "wb") as fo:
        while True:
            g = fg.read(chunk)
            if not g:
                break
            a = np.frombuffer(g, dtype=np.int8).astype(np.float32)
            e = np.frombuffer(fe.read(len(a) * 2), dtype=np.int16).astype(np.float32)
            if e.size < a.size:
                e = np.pad(e, (0, a.size - e.size))
            s = np.rint(a + e * scale)
            clipped += int(np.count_nonzero((s > 127) | (s < -128)))
            fo.write(np.clip(s, -128, 127).astype(np.int8).tobytes())
            done += len(g)
            if progress:
                progress(done / total)
    return {"scale": scale, "gps_rms": g_rms, "gal_rms": e_rms * scale,
            "clipped_fraction": clipped / max(1, total), "gal_db": gal_db}


def add_galileo(sc, *, motion: Path | None = None, nav_src=None, gal_db: float = DEFAULT_GAL_DB,
                keep_raw: bool = False, progress=None, cancel=None, log=print) -> dict:
    """Generate Galileo for scenario `sc` (same start, place / motion and
    length) and mix it into sc.iq in place. Returns the report stored in
    scenario.json under "galileo"."""
    start = dt.datetime.fromisoformat(sc.start_gps)
    nav_src = Path(nav_src) if nav_src else default_nav()
    nav, prns = shift_nav(nav_src, start, sc.folder / "nav_gal.rnx")
    raw = sc.folder / "iq_gal16.bin"
    log(f"Galileo: generating {sc.duration_s:.0f} s from {nav_src.name} ...")
    out = generate(nav, start, raw, sc.duration_s, motion=motion,
                   llh=None if motion else (sc.lat_deg, sc.lon_deg, sc.alt_msl_m + sc.geoid_sep_m),
                   progress=(lambda t: progress(0.7 * t / max(1.0, sc.duration_s))) if progress else None,
                   cancel=cancel)
    in_view = channel_prns(out)
    log(f"Galileo: in view at start {in_view}; mixing into the GPS IQ ...")
    gps_only = sc.folder / "iq_gps.bin"
    sc.iq.replace(gps_only)
    report = mix_into(gps_only, raw, sc.iq, n_gps=max(1, len(sc.visible_prns)),
                      n_gal=max(1, len(in_view)), gal_db=gal_db, sample_rate=sc.sample_rate,
                      progress=(lambda f: progress(0.7 + 0.3 * f)) if progress else None)
    if not keep_raw:
        raw.unlink(missing_ok=True)
        gps_only.unlink(missing_ok=True)
    report.update({"nav_source": str(nav_src), "prns_with_ephemeris": prns, "visible_prns": in_view})
    log(f"Galileo: mixed at {gal_db:+.1f} dB per satellite, "
        f"{100 * report['clipped_fraction']:.3f} % samples clipped")
    return report


def channel_prns(output: str) -> list:
    """PRNs galileo-sdr-sim allocated: its "PRN az el range sec" lines."""
    return sorted({int(m.group(1)) for m in re.finditer(
        r"^(\d\d)\s+-?[\d.]+\s+-?[\d.]+\s+[\d.]+\s+[\d.]+\s*$", output, re.M)})
