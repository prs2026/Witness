"""Desktop synthesis of the 960 Hz LSM6DSV320X readings Witness gets in HITL.

Pipeline (per channel; runs on the desktop, never on the flight computer):

  100 Hz trajectory truth (body specific force in g, body rate in dps)
    -> linear interpolation to the sensor's 7.68 kHz internal rate
    + vibration (48 kHz flight recordings, anti-alias resampled to 7.68 kHz),
      accelerometers only, from launch. Vibration profiles belong to one
      trajectory: vibration/<trajectory file stem>/profile.json names the
      thrust and lateral recordings and their full scale (find_profile).
    + datasheet errors: zero offsets and gyro sensitivity error at their
      worst-case bounds (random sign per axis, from the seed)
    + white noise at the datasheet noise density (sigma = ND * sqrt(fs / 2))
    -> clip at the full scale (the ADC saturates before the digital filter)
    -> 2nd-order low-pass at the bandwidth Witness configures on the chip
       (gyro LPF1+LPF2 407 Hz, low-g LPF2 ODR/20 = 384 Hz, high-g ~400 Hz)
    -> keep every 8th sample (the FIFO's 960 Hz batch rate)
    -> quantize to the sensor's counts (int16, saturating)

The result is sent to Witness as 0x0E frames (hitl_protocol.encode_imu_batch).
The on-chip filters are modelled as Butterworth sections at the datasheet
-3 dB bandwidth; ST does not publish the exact filter structure.

Datasheet values (LSM6DSV320X DS14623 Rev 3, Table 3, high-performance mode):
  gyro:  3.8 mdps/sqrt(Hz), zero-rate level +/-1 dps, sensitivity +/-0.3 %
  low-g: 60 ug/sqrt(Hz), zero-g level +/-12 mg
  high-g: 1000 ug/sqrt(Hz), zero-g level +/-1.5 g
Accelerometer sensitivity tolerance is not given, so none is applied.

Needs numpy, scipy and soundfile (pip install numpy scipy soundfile).
"""

from __future__ import annotations

import dataclasses
import functools
import math
import os
from dataclasses import dataclass, field

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
VIBRATION_DIR = os.path.join(HERE, "vibration")


def find_profile(csv_path: str, choice: str = "") -> dict | None:
    """The vibration profile for a trajectory, or None.

    choice "" (default): vibration/<csv stem>/profile.json if it exists, so a
    profile is only used with the trajectory it was made for. "none": no
    vibration. Anything else: a profile folder (absolute, or under vibration/).
    Returns {"name", "dir", "thrust_path", "lateral_path", "full_scale_g", ...}.
    """
    import json
    choice = (choice or "").strip()
    if choice.lower() == "none":
        return None
    if choice:
        folder = choice if os.path.isabs(choice) else os.path.join(VIBRATION_DIR, choice)
        if not os.path.isfile(os.path.join(folder, "profile.json")):
            raise ValueError(f"vibration profile {choice!r}: no profile.json in {folder}")
    else:
        stem = os.path.splitext(os.path.basename(csv_path or ""))[0]
        folder = os.path.join(VIBRATION_DIR, stem)
        if not stem or not os.path.isfile(os.path.join(folder, "profile.json")):
            return None
    with open(os.path.join(folder, "profile.json")) as f:
        data = json.load(f)
    profile = dict(data)
    profile["name"] = os.path.basename(os.path.normpath(folder))
    profile["dir"] = folder
    profile["thrust_path"] = os.path.join(folder, data["thrust"])
    profile["lateral_path"] = os.path.join(folder, data["lateral"])
    profile["full_scale_g"] = float(data["full_scale_g"])
    return profile

RAW_RATE_HZ = 7680
OUT_RATE_HZ = 960
DECIMATION = RAW_RATE_HZ // OUT_RATE_HZ  # 8

# Witness's configuration (sensors.h): counts per unit and full scales.
LOW_G_PER_COUNT = 0.488e-3      # g
HIGH_G_PER_COUNT = 10.417e-3    # g
GYRO_DPS_PER_COUNT = 0.070      # dps
LOW_G_FULL_SCALE = 16.0
HIGH_G_FULL_SCALE = 320.0
GYRO_FULL_SCALE = 2000.0

# Datasheet (see module docstring).
GYRO_NOISE_DENSITY = 3.8e-3     # dps / sqrt(Hz)
LOW_G_NOISE_DENSITY = 60e-6     # g / sqrt(Hz)
HIGH_G_NOISE_DENSITY = 1000e-6  # g / sqrt(Hz)
GYRO_ZERO_RATE = 1.0            # dps
LOW_G_ZERO_G = 12e-3            # g
HIGH_G_ZERO_G = 1.5             # g
GYRO_SENSITIVITY_TOL = 0.003


@dataclass
class ImuSynthParams:
    enabled: bool = True
    seed: int = 1
    # Vibration recordings (empty = no vibration); see find_profile().
    vibration_thrust_path: str = ""
    vibration_lateral_path: str = ""
    vibration_full_scale_g: float = 150.67   # sample value 1.0
    vibration_scale: float = 1.0             # 0 disables vibration
    # The z axis uses the lateral recording delayed by this much, so y and z
    # carry the same spectrum and envelope but are not identical.
    lateral_z_delay_s: float = 0.137
    noise_scale: float = 1.0                 # 1 = datasheet noise density
    offset_scale: float = 1.0                # 1 = datasheet zero-level bounds
    sensitivity_scale: float = 1.0           # 1 = datasheet gyro tolerance
    gyro_cutoff_hz: float = 407.0
    low_g_cutoff_hz: float = 384.0
    high_g_cutoff_hz: float = 400.0

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


@dataclass
class ImuStream:
    """960 Hz readings in native counts; sample k is at time_s[k]."""
    low_g: np.ndarray     # (N, 3) int16
    high_g: np.ndarray    # (N, 3) int16
    gyro: np.ndarray      # (N, 3) int16
    time_s: np.ndarray    # (N,) float64, simulation seconds
    report: dict = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.time_s)


@functools.lru_cache(maxsize=4)
def load_vibration(path: str, full_scale_g: float) -> np.ndarray:
    """A vibration recording in g at 7.68 kHz (48 kHz anti-alias resampled)."""
    import soundfile as sf
    data, rate = sf.read(path, dtype="float64", always_2d=False)
    if data.ndim > 1:
        data = data[:, 0]
    g = math.gcd(int(rate), RAW_RATE_HZ)
    up, down = RAW_RATE_HZ // g, int(rate) // g
    resampled = signal.resample_poly(data, up, down)  # 48 kHz: up 4, down 25
    return (resampled * full_scale_g).astype(np.float64)


def _lowpass(x: np.ndarray, cutoff_hz: float) -> np.ndarray:
    sos = signal.butter(2, cutoff_hz, btype="low", fs=RAW_RATE_HZ, output="sos")
    zi = signal.sosfilt_zi(sos) * x[0]   # start settled at the first value
    y, _ = signal.sosfilt(sos, x, zi=zi)
    return y


def _place(length: int, vibration: np.ndarray, start: int, scale: float) -> np.ndarray:
    out = np.zeros(length)
    if scale == 0.0 or start >= length:
        return out
    n = min(len(vibration), length - start)
    out[start:start + n] = vibration[:n] * scale
    return out


def synthesize(time_s, f_body_g, w_body_dps, launch_time_s: float | None,
               params: ImuSynthParams | None = None) -> ImuStream:
    """Builds the 960 Hz readings from 100 Hz truth.

    time_s: (M,) tick times in seconds; f_body_g, w_body_dps: (M, 3) truth.
    launch_time_s: when vibration starts (None for no vibration).
    """
    p = params or ImuSynthParams()
    t100 = np.asarray(time_s, dtype=np.float64)
    f100 = np.asarray(f_body_g, dtype=np.float64)
    w100 = np.asarray(w_body_dps, dtype=np.float64)
    n_raw = int(math.floor(t100[-1] * RAW_RATE_HZ)) + 1
    t_raw = np.arange(n_raw) / RAW_RATE_HZ
    keep = np.arange(DECIMATION - 1, n_raw, DECIMATION)   # FIFO sample instants
    rng = np.random.default_rng(p.seed)
    nyquist = RAW_RATE_HZ / 2.0

    def signs():
        return rng.choice((-1.0, 1.0), size=3)

    gyro_offset = signs() * GYRO_ZERO_RATE * p.offset_scale
    gyro_gain = 1.0 + signs() * GYRO_SENSITIVITY_TOL * p.sensitivity_scale
    low_offset = signs() * LOW_G_ZERO_G * p.offset_scale
    high_offset = signs() * HIGH_G_ZERO_G * p.offset_scale

    vib = [np.zeros(n_raw)] * 3
    report: dict = {"raw_rate_hz": RAW_RATE_HZ, "rate_hz": OUT_RATE_HZ,
                    "gyro_offset_dps": gyro_offset.tolist(),
                    "gyro_gain": gyro_gain.tolist(),
                    "low_g_offset_g": low_offset.tolist(),
                    "high_g_offset_g": high_offset.tolist(),
                    "params": p.to_dict()}
    if launch_time_s is not None and p.vibration_scale != 0.0 and \
            p.vibration_thrust_path and p.vibration_lateral_path:
        thrust = load_vibration(p.vibration_thrust_path, p.vibration_full_scale_g)
        lateral = load_vibration(p.vibration_lateral_path, p.vibration_full_scale_g)
        start = int(round(launch_time_s * RAW_RATE_HZ))
        delay = int(round(p.lateral_z_delay_s * RAW_RATE_HZ))
        vib = [_place(n_raw, thrust, start, p.vibration_scale),
               _place(n_raw, lateral, start, p.vibration_scale),
               _place(n_raw, lateral[delay:], start, p.vibration_scale)]
        report["vibration_start_s"] = launch_time_s
        report["vibration_duration_s"] = len(thrust) / RAW_RATE_HZ
        report["vibration_peak_g"] = [float(np.max(np.abs(v))) for v in vib]
        report["vibration_rms_g"] = [
            float(np.sqrt(np.mean(v[start:start + len(thrust)] ** 2)))
            if start < n_raw else 0.0 for v in vib]

    def channel(truth, extra, gain, offset, density, full_scale, cutoff, per_count):
        raw = truth * gain + extra + offset
        sigma = density * math.sqrt(nyquist) * p.noise_scale
        if sigma > 0:
            raw = raw + rng.standard_normal(n_raw) * sigma
        clipped = np.abs(raw) > full_scale
        raw = np.clip(raw, -full_scale, full_scale)
        out = _lowpass(raw, cutoff)[keep]
        counts = np.clip(np.rint(out / per_count), -32768, 32767).astype(np.int16)
        return counts, int(np.count_nonzero(clipped)), sigma

    low = np.empty((len(keep), 3), dtype=np.int16)
    high = np.empty_like(low)
    gyro = np.empty_like(low)
    clip_low = [0, 0, 0]
    clip_high = [0, 0, 0]
    clip_gyro = [0, 0, 0]
    for axis in range(3):
        f_raw = np.interp(t_raw, t100, f100[:, axis])
        low[:, axis], clip_low[axis], low_sigma = channel(
            f_raw, vib[axis], 1.0, low_offset[axis], LOW_G_NOISE_DENSITY,
            LOW_G_FULL_SCALE, p.low_g_cutoff_hz, LOW_G_PER_COUNT)
        high[:, axis], clip_high[axis], high_sigma = channel(
            f_raw, vib[axis], 1.0, high_offset[axis], HIGH_G_NOISE_DENSITY,
            HIGH_G_FULL_SCALE, p.high_g_cutoff_hz, HIGH_G_PER_COUNT)
        w_raw = np.interp(t_raw, t100, w100[:, axis])
        gyro[:, axis], clip_gyro[axis], gyro_sigma = channel(
            w_raw, 0.0, gyro_gain[axis], gyro_offset[axis], GYRO_NOISE_DENSITY,
            GYRO_FULL_SCALE, p.gyro_cutoff_hz, GYRO_DPS_PER_COUNT)
        del f_raw, w_raw

    report.update({
        "samples": int(len(keep)),
        "raw_noise_sigma": {"low_g_mg": low_sigma * 1e3, "high_g_mg": high_sigma * 1e3,
                            "gyro_dps": gyro_sigma},
        "raw_clipped_fraction": {
            "low_g": [c / n_raw for c in clip_low],
            "high_g": [c / n_raw for c in clip_high],
            "gyro": [c / n_raw for c in clip_gyro]},
        "output_saturated_fraction": {
            "low_g": float(np.mean(np.abs(low.astype(np.int32)) >= 32767)),
            "gyro": float(np.mean(np.abs(gyro.astype(np.int32)) >= 32767))},
    })
    return ImuStream(low_g=low, high_g=high, gyro=gyro, time_s=t_raw[keep], report=report)


def from_samples(samples, params: ImuSynthParams | None = None) -> ImuStream:
    """Synthesizes from hitl trajectory samples (TrajSample with truth fields).

    Vibration starts at the first sample after the pad hold (tau_s > 0).
    """
    t = np.array([s.t_ms / 1000.0 for s in samples])
    f = np.array([s.f_body_g for s in samples])
    w = np.array([s.w_body_dps for s in samples])
    launch = next((s.t_ms / 1000.0 for s in samples
                   if getattr(s, "tau_s", 0.0) > 0.0), None)
    return synthesize(t, f, w, launch, params)


def available() -> str | None:
    """None if the synthesis can run here, else the reason it cannot."""
    try:
        import numpy  # noqa: F401
        import scipy  # noqa: F401
        import soundfile  # noqa: F401
    except ImportError as exc:
        return f"{exc.name} is not installed (pip install numpy scipy soundfile)"
    return None
