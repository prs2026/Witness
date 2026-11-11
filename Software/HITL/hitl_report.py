"""Post-run PDF report for a HITL run folder (runs/<timestamp>/report.pdf).

    py -3.14 hitl_report.py runs/<timestamp>        # (re)build one report

Pages:
  1. Summary: source, result, transitions, warnings, GPS mode and IMU setup.
  2. Altitude and speed: truth vs Witness Kalman (live 0xE0 and, if fetched,
     the 50 Hz flash state log) and GPS, with errors.
  3. GNSS performance: satellites, fix quality, HDOP/VDOP/PDOP, time to first
     fix, availability, GPS position/altitude error against truth (from the
     receiver's NMEA in RF GPS runs; from the simulated 0x0C fields otherwise).
  4. Kalman filter performance: error statistics per flight phase, error
     traces, and, when the flash Kalman log (0xE2) was fetched, innovations
     against their 3-sigma bounds, normalized innovation squared, position
     sigmas and the measurement counters.
  5. Attitude (CSV runs): truth vs Witness tilt and attitude error.

Uses only the files a run writes (summary.json, params.json, truth.csv,
comparison.csv, comparison_flash.csv, sent.csv, nmea.txt, attitude.csv,
fc_logs/P*.BIN), so old runs can be reported too. matplotlib (Agg) only; no
Qt, so the headless runner can call it.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "Horizon"))

PHASE_ORDER = ("Pad idle", "Boost", "Coast", "Descent", "Landed")
COLORS = {"truth": "#222222", "witness": "#1f6fd1", "flash": "#d1601f", "gps": "#2b9a3e",
          "err": "#c0392b", "band": "#9bbbe8"}


# ------------------------------------------------------------------ input
def _read_csv(path: Path) -> list:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def _f(row, key):
    try:
        v = float(row.get(key, ""))
    except (TypeError, ValueError):
        return math.nan
    return v


def _cols(rows, *keys):
    import numpy as np
    return [np.array([_f(r, k) for r in rows], dtype=float) for k in keys]


NMEA_LINE = re.compile(r"^\s*([\d.]+)\s+sim=\s*(-?[\d.]+)\s+(\$.*)$")


def parse_nmea_file(path: Path) -> dict:
    """GGA/GSA series from nmea.txt ('<wall> sim= <s> $...')."""
    import numpy as np
    sys.path.insert(0, str(HERE))
    import gps_rf
    gga, gsa = [], []
    if path.exists():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            m = NMEA_LINE.match(line)
            if not m:
                continue
            sim = float(m.group(2))
            wall = float(m.group(1))
            g = gps_rf.parse_gga(m.group(3))
            if g:
                gga.append((wall, sim, g))
                continue
            s = gps_rf.parse_gsa(m.group(3))
            if s:
                gsa.append((wall, sim, s))
    return {"gga": gga, "gsa": gsa}


def enu_from_llh(lat, lon, lat0, lon0):
    import numpy as np
    phi = math.radians(lat0)
    m_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return (np.asarray(lon) - lon0) * m_lon, (np.asarray(lat) - lat0) * m_lat


def phase_stats(rows, keys) -> dict:
    """{phase: {key: (rms, max abs, n)}} from comparison rows by expected state."""
    out = {}
    for phase in PHASE_ORDER:
        sel = [r for r in rows if r.get("expected_state") == phase]
        if not sel:
            continue
        out[phase] = {}
        for key in keys:
            vals = [abs(_f(r, key)) for r in sel]
            vals = [v for v in vals if v == v]
            if vals:
                out[phase][key] = (math.sqrt(sum(v * v for v in vals) / len(vals)), max(vals), len(vals))
    return out


def kalman_rows(run_dir: Path) -> list:
    """0xE2 Kalman log rows from fc_logs/P*.BIN (empty if not fetched)."""
    files = sorted((run_dir / "fc_logs").glob("P*.BIN"))
    if not files:
        return []
    try:
        import converter
    except ImportError:
        return []
    rows = []
    for row in converter.iter_packet_log(files[-1].read_bytes()):
        if row.get("packet_type") == "kalman_log":
            rows.append(row)
    return rows


def rows_from_received(run_dir: Path, truth) -> list:
    """comparison.csv-like rows from the live 0x02 state packets in
    received.jsonl (runs without 0xE0 debug packets): Kalman position,
    velocity and altitude against truth at the packet's sim time."""
    import bisect
    path = run_dir / "received.jsonl"
    if not path.exists() or not truth:
        return []
    tt = [_f(r, "sim_s") for r in truth]
    alt0 = _f(truth[0], "alt_msl_m")
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                pkt = json.loads(line)
            except ValueError:
                continue
            if pkt.get("name") != "state" or not pkt.get("sim_ms") or "position_m" not in pkt:
                continue
            if not (pkt.get("status") or {}).get("hitl_active"):
                continue
            sim_s = pkt["sim_ms"] / 1000.0
            k = min(max(bisect.bisect_left(tt, sim_s), 0), len(truth) - 1)
            t = truth[k]
            up_t = _f(t, "alt_msl_m") - alt0
            vel = pkt.get("velocity_mps") or [0, 0, 0]
            pos = pkt["position_m"]
            tv = [_f(t, "vel_east_mps"), _f(t, "vel_north_mps"), _f(t, "vel_up_mps")]
            rows.append({
                "sim_s": sim_s, "uptime_ms": pkt.get("uptime_ms"),
                "expected_state": t.get("expected_state", ""),
                "witness_up_m": pos[2], "truth_up_m": up_t, "error_up_m": pos[2] - up_t,
                "error_horizontal_m": math.hypot(pos[0] - _f(t, "east_m"), pos[1] - _f(t, "north_m")),
                "witness_vel_east_mps": vel[0], "witness_vel_north_mps": vel[1],
                "witness_vel_up_mps": vel[2], "truth_vel_up_mps": tv[2],
                "error_speed_mps": math.sqrt(sum(v * v for v in vel)) - math.sqrt(sum(v * v for v in tv)),
                "witness_kalman_alt_m": alt0 + pos[2],
            })
    return rows


def uptime_to_sim(comparison) -> tuple[float, float] | None:
    """Linear fit sim_s = a * uptime_s + b from the live 0xE0 rows."""
    pts = [(_f(r, "uptime_ms") / 1000.0, _f(r, "sim_s")) for r in comparison]
    pts = [(u, s) for u, s in pts if u == u and s == s and u > 0]
    if len(pts) < 10:
        return None
    n = len(pts)
    mu = sum(p[0] for p in pts) / n
    ms = sum(p[1] for p in pts) / n
    var = sum((p[0] - mu) ** 2 for p in pts)
    if var <= 0:
        return None
    a = sum((p[0] - mu) * (p[1] - ms) for p in pts) / var
    return a, ms - a * mu


# ----------------------------------------------------------------- pages
def _fig(title, nrows=1, ncols=1, height=10.0, sharex=True):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(nrows, ncols, figsize=(8.27, height), sharex=sharex, squeeze=False)
    fig.suptitle(title, fontsize=13, fontweight="bold", x=0.02, ha="left")
    for ax in axes.flat:
        ax.grid(True, color="#dddddd", linewidth=0.6)
        ax.tick_params(labelsize=8)
    return fig, axes


def _text_page(pdf, title, blocks):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(8.27, 11.69))
    fig.text(0.05, 0.965, title, fontsize=15, fontweight="bold", va="top")
    y = 0.93
    import textwrap
    for heading, lines, color in blocks:
        lines = [part for line in lines
                 for part in (textwrap.wrap(line, 104, subsequent_indent="    ") or [""])]
        if y < 0.06:
            break
        if heading:
            fig.text(0.05, y, heading, fontsize=11, fontweight="bold", va="top", color=color or "#222")
            y -= 0.025
        for line in lines:
            if y < 0.04:
                fig.text(0.07, y, "...", fontsize=8.5, va="top")
                break
            fig.text(0.07, y, line, fontsize=8.5, va="top", family="monospace", color="#222")
            y -= 0.0175
        y -= 0.012
    pdf.savefig(fig)
    plt.close(fig)


def _summary_page(pdf, run_dir, summary, params, gnss_stats):
    p = params.get("params", {})
    passed = summary.get("passed")
    head = [f"Run folder: {run_dir.name}",
            f"Source:     {params.get('preset', summary.get('preset', '?'))}",
            f"Result:     {'PASS' if passed else 'FAIL'}"
            + (f" (tolerance {summary.get('tolerance_s'):.2f} s)" if summary.get("tolerance_s") else ""),
            f"Frames:     {summary.get('frames_sent', '?')} sent, {summary.get('late_ticks', 0)} late ticks"]
    rows = [f"{'State':<10}{'Expected s':>12}{'Witness s':>12}{'Delta s':>10}  Result"]
    for r in summary.get("rows", []):
        fmt = (lambda v: "-" if v is None else f"{v:.2f}")
        rows.append(f"{r.get('state', ''):<10}{fmt(r.get('expected_s')):>12}{fmt(r.get('witness_s')):>12}"
                    f"{fmt(r.get('delta_s')):>10}  {r.get('result', '')}")
    gps = summary.get("gps") or {}
    gps_lines = []
    if gps.get("mode") == "rf":
        gps_lines += [f"GPS:        RF simulation (Witness receiver), scenario {Path(gps.get('scenario', '')).name}",
                      f"            lead-in {gps.get('lead_in_s', 0):.0f} s, TX VGA {gps.get('tx_vga_db')} dB, "
                      f"amp {'on' if gps.get('amp') else 'off'}, ephemeris uploaded for "
                      f"{len(gps.get('ephemeris_prns') or [])} SVs",
                      "            first fix during lead-in: "
                      + ("none" if gps.get("first_fix_s") is None else f"{gps['first_fix_s']:.1f} s")]
    else:
        gps_lines.append(f"GPS:        simulated in the 0x0C frames ({p.get('gps_sats', '?')} sats, "
                         f"fix {'on' if p.get('gps_fix', True) else 'off'})")
    gps_lines += gnss_stats.get("summary", [])
    imu = params.get("csv_report", {}) or {}
    site = [f"Site:       {p.get('lat_deg', '?')}, {p.get('lon_deg', '?')}, {p.get('site_alt_msl_m', '?')} m MSL",
            f"IMU:        {'960 Hz synthesis' if p.get('imu_synthesis') else '100 Hz 0x0C'}; vibration "
            f"{p.get('vibration_thrust_file') or p.get('vibration_profile') or 'auto/none'} x{p.get('vibration_scale', 1)}"]
    comp = summary.get("comparison_flash") or summary.get("comparison") or {}
    kf = [f"{'Phase':<10}{'up RMS':>9}{'up max':>9}{'horiz RMS':>11}{'speed RMS':>11}{'att RMS':>9}"]
    for phase in PHASE_ORDER:
        c = comp.get(phase)
        if not c:
            continue
        g = (lambda k: c.get(k) if c.get(k) is not None else math.nan)
        kf.append(f"{phase:<10}{g('up_rms_m'):9.2f}{g('up_max_m'):9.2f}{g('horizontal_rms_m'):11.2f}"
                  f"{g('speed_rms_mps'):11.2f}{g('attitude_rms_deg'):9.2f}")
    att = summary.get("attitude") or {}
    att_lines = [f"{k}: {v}" for k, v in att.items() if not isinstance(v, (dict, list))][:8]
    blocks = [("Run", head + site, None), ("Transitions", rows, None), ("GPS", gps_lines, None),
              ("Kalman error vs truth (m, m/s, deg; "
               + ("flash 50 Hz" if summary.get("comparison_flash") else "live 0xE0") + ")", kf, None)]
    if att_lines:
        blocks.append(("Attitude", att_lines, None))
    problems = list(summary.get("problems") or []) + list(summary.get("warnings") or [])
    if problems:
        blocks.append(("Problems and warnings", [f"- {w}" for w in problems], "#b00020"))
    _text_page(pdf, "Witness HITL run report", blocks)


def _altitude_speed_page(pdf, truth, comp, flash, gnss):
    import numpy as np
    fig, ax = _fig("Altitude and speed", 4, 1, 11.69)
    t, alt, spd = _cols(truth, "sim_s", "alt_msl_m", "speed_mps")
    ax[0][0].plot(t, alt, color=COLORS["truth"], lw=1.2, label="truth (MSL)")
    if comp:
        ct, kalt = _cols(comp, "sim_s", "witness_kalman_alt_m")
        ax[0][0].plot(ct, kalt, color=COLORS["witness"], lw=0.9, label="Witness Kalman (live)")
    if flash:
        ft, falt = _cols(flash, "sim_s", "witness_baro_alt_msl_m")
        if np.isfinite(falt).any():
            ax[0][0].plot(ft, falt, color=COLORS["flash"], lw=0.7, label="Witness (flash 50 Hz)")
    if gnss.get("t") is not None and len(gnss["t"]):
        ax[0][0].plot(gnss["t"], gnss["alt"], ".", ms=1.5, color=COLORS["gps"], label="GPS (GGA)")
    ax[0][0].set_ylabel("altitude MSL (m)", fontsize=8)
    ax[0][0].legend(fontsize=7, loc="upper right")
    src = flash or comp
    if src:
        st, eu = _cols(src, "sim_s", "error_up_m")
        ax[1][0].plot(st, eu, color=COLORS["err"], lw=0.8, label="Kalman up error")
    if gnss.get("t") is not None and len(gnss["t"]):
        ax[1][0].plot(gnss["t"], gnss["alt_err"], ".", ms=1.2, color=COLORS["gps"], label="GPS alt error")
    ax[1][0].set_ylabel("altitude error (m)", fontsize=8)
    if ax[1][0].get_legend_handles_labels()[0]:
        ax[1][0].legend(fontsize=7, loc="upper right")
    ax[2][0].plot(t, spd, color=COLORS["truth"], lw=1.2, label="truth")
    if src:
        ve, vn, vu = _cols(src, "witness_vel_east_mps", "witness_vel_north_mps", "witness_vel_up_mps")
        ax[2][0].plot(st, np.sqrt(ve ** 2 + vn ** 2 + vu ** 2), color=COLORS["witness"], lw=0.9,
                      label="Witness Kalman")
    ax[2][0].set_ylabel("speed (m/s)", fontsize=8)
    ax[2][0].legend(fontsize=7, loc="upper right")
    if src:
        _, es = _cols(src, "sim_s", "error_speed_mps")
        ax[3][0].plot(st, es, color=COLORS["err"], lw=0.8, label="speed error")
        _, vut = _cols(src, "sim_s", "truth_vel_up_mps")
        ax[3][0].plot(st, vu - vut, color="#8e44ad", lw=0.6, label="vertical velocity error")
    ax[3][0].set_ylabel("velocity error (m/s)", fontsize=8)
    ax[3][0].set_xlabel("sim time (s)", fontsize=8)
    if src:
        ax[3][0].legend(fontsize=7, loc="upper right")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    pdf.savefig(fig)
    import matplotlib.pyplot as plt
    plt.close(fig)


def gnss_series(nmea, truth, params, sent, rf: bool) -> dict:
    """GPS series and statistics against truth."""
    import numpy as np
    p = params.get("params", {})
    lat0, lon0 = float(p.get("lat_deg", 0) or 0), float(p.get("lon_deg", 0) or 0)
    tt, te, tn, talt = _cols(truth, "sim_s", "east_m", "north_m", "alt_msl_m")
    out = {"summary": []}
    if not rf:
        # Simulated: the values in the 0x0C frames are the truth (quantized).
        st, sats, fix = _cols(sent, "sim_ms", "gps_sats", "gps_fix")
        out.update(t=None, sats_t=st / 1000.0, sats=sats, quality=fix)
        out["summary"].append("            GPS values were simulated (no receiver errors to assess)")
        return out
    gga = nmea["gga"]
    if not gga:
        out["summary"].append("            no GGA sentences recorded")
        out.update(t=np.array([]))
        return out
    sim = np.array([g[1] for g in gga])
    qual = np.array([g[2]["quality"] for g in gga])
    sats = np.array([g[2]["sats"] for g in gga])
    hdop = np.array([g[2]["hdop"] if g[2]["hdop"] is not None else np.nan for g in gga])
    fixmask = qual > 0
    lat = np.array([g[2].get("lat") if g[2].get("lat") is not None else np.nan for g in gga])
    lon = np.array([g[2].get("lon") if g[2].get("lon") is not None else np.nan for g in gga])
    alt = np.array([g[2].get("alt_msl") if g[2].get("alt_msl") is not None else np.nan for g in gga])
    e, n = enu_from_llh(lat, lon, lat0, lon0)
    run_mask = sim > 0          # after sim time 0 (the lead-in reads sim = 0)
    he = np.full(len(sim), np.nan)
    ae = np.full(len(sim), np.nan)
    if len(tt):
        he = np.hypot(e - np.interp(sim, tt, te), n - np.interp(sim, tt, tn))
        ae = alt - np.interp(sim, tt, talt)
    gsa = nmea["gsa"]
    out.update(t=sim[fixmask], alt=alt[fixmask], alt_err=ae[fixmask], h_err=he[fixmask],
               sats_t=sim, sats=sats, quality=qual, hdop=hdop,
               gsa_t=np.array([g[1] for g in gsa]),
               vdop=np.array([g[2]["vdop"] if g[2]["vdop"] is not None else np.nan for g in gsa]),
               pdop=np.array([g[2]["pdop"] if g[2]["pdop"] is not None else np.nan for g in gsa]))
    walls = np.array([g[0] for g in gga])
    first = np.argmax(fixmask) if fixmask.any() else None
    s = out["summary"]
    if first is not None:
        s.append(f"            first fix in the record: {walls[first] - walls[0]:.1f} s after the first GGA")
    in_run = run_mask
    if in_run.any():
        avail = 100.0 * fixmask[in_run].mean()
        s.append(f"            fix availability during the run: {avail:.1f}% of {in_run.sum()} GGA epochs, "
                 f"median {np.nanmedian(sats[in_run]):.0f} satellites, median HDOP "
                 f"{np.nanmedian(hdop[in_run & fixmask]) if (in_run & fixmask).any() else float('nan'):.2f}")
        good = in_run & fixmask & np.isfinite(he)
        if good.any():
            s.append(f"            GPS error vs truth: horizontal RMS {np.sqrt(np.nanmean(he[good] ** 2)):.1f} m "
                     f"(max {np.nanmax(he[good]):.1f}), altitude RMS {np.sqrt(np.nanmean(ae[good] ** 2)):.1f} m "
                     f"(max {np.nanmax(np.abs(ae[good])):.1f})")
        # Outages (no fix for > 1 s during the run).
        gaps, start = [], None
        for t_i, ok in zip(sim[in_run], fixmask[in_run]):
            if not ok and start is None:
                start = t_i
            elif ok and start is not None:
                if t_i - start > 1.0:
                    gaps.append((start, t_i))
                start = None
        if start is not None:
            gaps.append((start, sim[in_run][-1]))
        if gaps:
            s.append("            fix lost: " + ", ".join(f"{a:.1f}-{b:.1f} s" for a, b in gaps[:6])
                     + (" ..." if len(gaps) > 6 else ""))
            out["gaps"] = gaps
    return out


def _gnss_page(pdf, gnss, truth, rf):
    import numpy as np
    import matplotlib.pyplot as plt
    fig, ax = _fig("GNSS performance" + ("" if rf else " (simulated GPS fields)"), 4, 1, 11.69)
    if gnss.get("sats_t") is not None and len(gnss["sats_t"]):
        ax[0][0].step(gnss["sats_t"], gnss["sats"], where="post", color=COLORS["gps"], lw=0.9, label="satellites used")
        ax2 = ax[0][0].twinx()
        ax2.step(gnss["sats_t"], gnss["quality"], where="post", color="#888", lw=0.7, label="fix quality")
        ax2.set_ylim(-0.2, 3)
        ax2.set_ylabel("GGA quality", fontsize=8)
        ax2.tick_params(labelsize=8)
    ax[0][0].set_ylabel("satellites", fontsize=8)
    for a, b in gnss.get("gaps", []):
        for row in ax:
            row[0].axvspan(a, b, color="#f5b7b1", alpha=0.4, lw=0)
    if rf and gnss.get("hdop") is not None:
        ax[1][0].plot(gnss["sats_t"], gnss["hdop"], color=COLORS["witness"], lw=0.8, label="HDOP")
        if len(gnss.get("gsa_t", [])):
            ax[1][0].plot(gnss["gsa_t"], gnss["vdop"], color=COLORS["flash"], lw=0.8, label="VDOP")
            ax[1][0].plot(gnss["gsa_t"], gnss["pdop"], color="#7f8c8d", lw=0.6, label="PDOP")
        ax[1][0].set_ylim(0, max(4, np.nanpercentile(gnss["hdop"], 95) * 1.5 if np.isfinite(gnss["hdop"]).any() else 4))
        ax[1][0].legend(fontsize=7, loc="upper right")
    else:
        ax[1][0].text(0.5, 0.5, "no dilution of precision (simulated GPS)", ha="center", va="center",
                      transform=ax[1][0].transAxes, fontsize=9, color="#777")
    ax[1][0].set_ylabel("DOP", fontsize=8)
    if rf and gnss.get("t") is not None and len(gnss["t"]):
        ax[2][0].plot(gnss["t"], gnss["h_err"], ".", ms=1.5, color=COLORS["gps"])
        ax[3][0].plot(gnss["t"], gnss["alt_err"], ".", ms=1.5, color=COLORS["gps"])
    ax[2][0].set_ylabel("GPS horizontal error (m)", fontsize=8)
    ax[3][0].set_ylabel("GPS altitude error (m)", fontsize=8)
    ax[3][0].set_xlabel("sim time (s)", fontsize=8)
    if truth:
        t, spd, alt = _cols(truth, "sim_s", "speed_mps", "alt_msl_m")
        for row, y, label in ((ax[2][0], spd, "truth speed (m/s, right)"), (ax[3][0], alt, "truth altitude (m, right)")):
            tw = row.twinx()
            tw.plot(t, y, color="#bbbbbb", lw=0.8)
            tw.set_ylabel(label, fontsize=7, color="#888")
            tw.tick_params(labelsize=7, colors="#888")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    pdf.savefig(fig)
    plt.close(fig)


def _kalman_page(pdf, comp, flash, kal, comparison):
    import numpy as np
    import matplotlib.pyplot as plt
    src = flash or comp
    n_rows = 4 if kal else 2
    fig, ax = _fig("Kalman filter performance", n_rows, 1, 11.69)
    if src:
        st, eu, eh, es = _cols(src, "sim_s", "error_up_m", "error_horizontal_m", "error_speed_mps")
        ax[0][0].plot(st, eu, color=COLORS["err"], lw=0.8, label="up error")
        ax[0][0].plot(st, eh, color=COLORS["witness"], lw=0.8, label="horizontal error")
        ax[0][0].set_ylabel("position error (m)", fontsize=8)
        ax[0][0].legend(fontsize=7, loc="upper right")
        ax[1][0].plot(st, es, color="#8e44ad", lw=0.8, label="speed error")
        ax[1][0].set_ylabel("speed error (m/s)", fontsize=8)
        ax[1][0].legend(fontsize=7, loc="upper right")
    if kal:
        fit = uptime_to_sim(comparison)
        tus = np.array([_f(r, "timestamp_us") for r in kal]) / 1e6
        t = tus * fit[0] + fit[1] if fit else tus - tus[0]
        if fit:   # the flash log also covers the time before HITL: show the run
            keep = t >= -5.0
            kal = [r for r, k in zip(kal, keep) if k]
            t = t[keep]
        bi, bv = _cols(kal, "baro_innovation", "baro_innovation_variance")
        bsig = 3 * np.sqrt(np.clip(bv, 0, None))
        ax[2][0].plot(t, bi, color=COLORS["witness"], lw=0.6, label="baro innovation")
        ax[2][0].fill_between(t, -bsig, bsig, color=COLORS["band"], alpha=0.4, lw=0, label="±3σ (S)")
        gx, gy, gz = _cols(kal, "gps_innovation_x", "gps_innovation_y", "gps_innovation_z")
        ax[2][0].plot(t, gz, ".", ms=1.2, color=COLORS["gps"], label="GPS up innovation")
        ax[2][0].plot(t, np.hypot(gx, gy), ".", ms=1.0, color=COLORS["flash"], label="GPS horiz. innovation")
        ax[2][0].set_ylabel("innovation (m)", fontsize=8)
        ax[2][0].legend(fontsize=7, loc="upper right", ncol=2)
        lim = np.nanpercentile(np.abs(np.concatenate([bi, gz])), 99) if np.isfinite(bi).any() else 10
        ax[2][0].set_ylim(-max(lim, 1) * 1.5, max(lim, 1) * 1.5)
        px, py_, pz = _cols(kal, "position_sigma_x", "position_sigma_y", "position_sigma_z")
        ax[3][0].plot(t, pz, color=COLORS["err"], lw=0.8, label="σ up")
        ax[3][0].plot(t, np.hypot(px, py_), color=COLORS["witness"], lw=0.8, label="σ horizontal")
        nis = bi ** 2 / bv
        ok = np.isfinite(nis)
        if ok.any():
            ax3 = ax[3][0].twinx()
            ax3.plot(t[ok], nis[ok], ".", ms=1, color="#7f8c8d")
            ax3.set_ylabel("baro NIS (χ²₁)", fontsize=7, color="#777")
            ax3.set_ylim(0, 12)
            ax3.tick_params(labelsize=7)
        ax[3][0].set_ylabel("position σ (m)", fontsize=8)
        ax[3][0].legend(fontsize=7, loc="upper left")
    ax[-1][0].set_xlabel("sim time (s)", fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    pdf.savefig(fig)
    plt.close(fig)
    # Counters and NIS statistics as text.
    lines = []
    if kal:
        last = kal[-1]
        lines.append(f"baro updates {last.get('baro_updates')}, gated (Mach) {last.get('baro_gated_count')}, "
                     f"rejected {last.get('baro_rejects')}; GPS updates {last.get('gps_updates')}, "
                     f"rejected {last.get('gps_rejects')} (16-bit counters)")
        bi, bv = _cols(kal, "baro_innovation", "baro_innovation_variance")
        nis = (bi ** 2 / bv)
        nis = nis[np.isfinite(nis)]
        if nis.size:
            lines.append(f"baro NIS mean {nis.mean():.2f} (consistent filter ≈ 1.0), "
                         f"{100 * (nis > 9).mean():.1f}% beyond 3σ (expected 0.3%)")
    else:
        lines.append("flash Kalman log (0xE2) not available: fetch FC logs after the run for innovations and σ")
    return lines


def _attitude_page(pdf, att_rows):
    import numpy as np
    import matplotlib.pyplot as plt
    fig, ax = _fig("Attitude", 2, 1, 8.0)
    t, tt, tw, te, ae = _cols(att_rows, "time_s", "truth_tilt_deg", "witness_tilt_deg",
                              "tilt_err_deg", "att_err_deg")
    ax[0][0].plot(t, tt, color=COLORS["truth"], lw=1.0, label="truth tilt")
    ax[0][0].plot(t, tw, color=COLORS["witness"], lw=0.8, label="Witness tilt")
    ax[0][0].set_ylabel("tilt from vertical (deg)", fontsize=8)
    ax[0][0].legend(fontsize=7)
    ax[1][0].plot(t, te, color=COLORS["err"], lw=0.8, label="tilt error")
    ax[1][0].plot(t, ae, color="#8e44ad", lw=0.6, label="attitude error")
    ax[1][0].set_ylabel("error (deg)", fontsize=8)
    ax[1][0].set_xlabel("sim time (s)", fontsize=8)
    ax[1][0].legend(fontsize=7)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    pdf.savefig(fig)
    plt.close(fig)


# ------------------------------------------------------------------- API
def build_report(run_dir, out_path=None) -> Path:
    """Write runs/<timestamp>/report.pdf; returns its path."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.backends.backend_pdf import PdfPages
    run_dir = Path(run_dir)
    out_path = Path(out_path) if out_path else run_dir / "report.pdf"
    summary = json.loads((run_dir / "summary.json").read_text()) if (run_dir / "summary.json").exists() else {}
    params = json.loads((run_dir / "params.json").read_text()) if (run_dir / "params.json").exists() else {}
    truth = _read_csv(run_dir / "truth.csv")
    comp = _read_csv(run_dir / "comparison.csv")
    flash = _read_csv(run_dir / "comparison_flash.csv")
    sent = _read_csv(run_dir / "sent.csv")
    att_rows = _read_csv(run_dir / "attitude.csv")
    rf = (summary.get("gps") or {}).get("mode") == "rf"
    nmea = parse_nmea_file(run_dir / "nmea.txt")
    gnss = gnss_series(nmea, truth, params, sent, rf)
    kal = kalman_rows(run_dir)
    if not comp and not flash:
        comp = rows_from_received(run_dir, truth)   # older runs without 0xE0
    with PdfPages(out_path) as pdf:
        _summary_page(pdf, run_dir, summary, params, gnss)
        _altitude_speed_page(pdf, truth, comp, flash, gnss)
        _gnss_page(pdf, gnss, truth, rf)
        kf_lines = _kalman_page(pdf, comp, flash, kal, comp)
        if att_rows:
            _attitude_page(pdf, att_rows)
        _text_page(pdf, "Kalman filter notes", [("Filter statistics", kf_lines, None),
                                                ("Files", [f.name for f in sorted(run_dir.iterdir())][:40], None)])
        info = pdf.infodict()
        info["Title"] = f"Witness HITL run {run_dir.name}"
    return out_path


if __name__ == "__main__":
    for arg in sys.argv[1:] or []:
        print(build_report(arg))
