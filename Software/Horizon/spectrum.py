"""Spectrogram (short-time FFT) and Welch average spectrum, numpy only.

Pure computation for the viewer's spectrogram window, so it can be tested
without Tk:

* ``uniform_signal`` cuts a time window out of a trace, derives the sample
  rate from the median time step and, when the sampling is uneven (jitter,
  dropped samples, duplicate stamps), resamples linearly onto a uniform grid.
* ``spectrogram`` gives the one-sided power spectral density of overlapping,
  windowed segments, and their mean, which is Welch's estimate. Segments are
  processed in batches, and long signals are averaged down to at most
  ``max_columns`` time columns, so memory stays bounded for any window.
* ``analyze`` runs both steps and converts the results to dB.
* ``image_grid`` and ``colorize`` turn a dB matrix into the RGB pixels of an
  image of a given size. A PPM of them loads straight into a Tk PhotoImage.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

WINDOWS = ("Hann", "Hamming", "Blackman", "Rectangular")
DETRENDS = ("Mean", "Linear", "None")
FFT_LENGTHS = (256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536)
MAX_FFT_LEN = 1 << 21            # zero-padded FFT length limit
MAX_BAND_BINS = 16384            # frequency bins kept per column (memory bound)
MAX_FFT_WORK = 600_000_000       # segments x FFT length (time bound, a few seconds)
UNEVEN_TOLERANCE = 0.01          # relative dt deviation that counts as uneven
BATCH_SAMPLES = 4_000_000        # samples per FFT batch (memory bound)


def window_array(name: str, n: int) -> np.ndarray:
    """Periodic (DFT-even) window of length ``n``."""
    x = 2.0 * np.pi * np.arange(n) / n
    key = name.lower()
    if key.startswith("hann"):
        return 0.5 - 0.5 * np.cos(x)
    if key.startswith("hamming"):
        return 0.54 - 0.46 * np.cos(x)
    if key.startswith("blackman"):
        return 0.42 - 0.5 * np.cos(x) + 0.08 * np.cos(2 * x)
    if key.startswith("rect") or key in ("none", "boxcar"):
        return np.ones(n)
    raise ValueError(f"unknown window {name!r}")


@dataclass
class Sampling:
    """A trace cut to a time window, on a uniform time grid."""
    t0: float                    # time of the first sample (s)
    values: np.ndarray
    fs: float                    # Hz
    median_dt: float
    n_original: int
    uneven: bool = False
    resampled: bool = False
    jitter: float = 0.0          # largest |dt - median| / median
    gaps: int = 0                # steps longer than 1.5 x the median
    duplicates: int = 0          # samples sharing a timestamp (dropped)
    warnings: list = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.values.size / self.fs


def uniform_signal(t, y, start: float | None = None, end: float | None = None,
                   resample: bool = True, tolerance: float = UNEVEN_TOLERANCE) -> Sampling:
    """Samples of (t, y) with start <= t <= end, on a uniform grid.

    NaNs are dropped, samples are sorted by time and duplicate timestamps
    keep their first value. The sample rate is 1 / median(dt). When some dt
    differs from the median by more than ``tolerance`` (relative), the
    sampling is flagged uneven and, with ``resample``, the values are
    interpolated linearly onto t0 + k / fs.
    """
    t = np.asarray(t, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    keep = np.isfinite(t) & np.isfinite(y)
    if start is not None:
        keep &= t >= start
    if end is not None:
        keep &= t <= end
    t, y = t[keep], y[keep]
    if t.size > 1 and not bool(np.all(t[1:] >= t[:-1])):
        order = np.argsort(t, kind="stable")
        t, y = t[order], y[order]
    n_original = int(t.size)
    duplicates = 0
    if t.size > 1:
        first = np.r_[True, t[1:] != t[:-1]]
        duplicates = int(t.size - np.count_nonzero(first))
        if duplicates:
            t, y = t[first], y[first]
    if t.size < 2:
        raise ValueError("fewer than 2 samples in the time window")
    dt = np.diff(t)
    median_dt = float(np.median(dt))
    if not median_dt > 0:
        raise ValueError("the time column does not increase")
    deviation = np.abs(dt - median_dt) / median_dt
    jitter = float(deviation.max())
    gaps = int(np.count_nonzero(dt > 1.5 * median_dt))
    uneven = jitter > tolerance or duplicates > 0
    fs = 1.0 / median_dt
    warnings = []
    values = y
    resampled = False
    if uneven:
        text = (f"uneven sampling: dt varies up to {jitter * 100:.3g}% from the median "
                f"{median_dt * 1e3:.4g} ms")
        if gaps:
            text += f", {gaps} gap(s) (largest {dt.max():.4g} s)"
        if duplicates:
            text += f", {duplicates} duplicate timestamp(s) dropped"
        if resample:
            count = int(math.floor((t[-1] - t[0]) / median_dt + 1e-9)) + 1
            grid = t[0] + np.arange(count) * median_dt
            values = np.interp(grid, t, y)
            resampled = True
            text += f"; resampled linearly onto a uniform {fs:.6g} Hz grid"
        warnings.append(text)
    return Sampling(t0=float(t[0]), values=values, fs=fs, median_dt=median_dt,
                    n_original=n_original, uneven=uneven, resampled=resampled,
                    jitter=jitter, gaps=gaps, duplicates=duplicates, warnings=warnings)


def _detrend(segments: np.ndarray, mode: str) -> np.ndarray:
    key = (mode or "none").lower()
    if key == "none":
        return segments
    if key == "mean":
        return segments - segments.mean(axis=1, keepdims=True)
    if key == "linear":
        n = segments.shape[1]
        x = np.arange(n, dtype=np.float64) - (n - 1) / 2.0
        slope = segments @ x / float(x @ x)
        return segments - segments.mean(axis=1, keepdims=True) - slope[:, None] * x
    raise ValueError(f"unknown detrend {mode!r}")


@dataclass
class Spectrogram:
    freqs: np.ndarray            # Hz, nfft // 2 + 1 bins
    times: np.ndarray            # centre of each column, s after the first sample
    psd: np.ndarray              # (columns, bins) one-sided PSD, units^2 / Hz
    welch: np.ndarray            # (bins,) mean PSD over every segment
    segments: int
    hop: int
    nfft: int                    # segment length (samples): the time resolution
    fs: float
    fft_len: int = 0             # FFT length incl. zero padding: the bin width

    @property
    def columns(self) -> int:
        return int(self.psd.shape[0])

    @property
    def bin_hz(self) -> float:
        return self.fs / (self.fft_len or self.nfft)


def spectrogram(values, fs: float, nfft: int = 1024, overlap: float = 0.5,
                window: str = "Hann", detrend: str = "Mean",
                max_columns: int | None = 2000, fft_len: int | None = None,
                fmin: float | None = None, fmax: float | None = None) -> Spectrogram:
    """Short-time FFT power spectral density and the Welch average.

    ``nfft`` is the segment length (time resolution). ``fft_len`` (>= nfft)
    zero-pads each segment so the bins are fs / fft_len wide: finer bins that
    interpolate the spectrum (the true resolution stays about fs / nfft).
    Only bins with fmin <= f <= fmax are kept (at most MAX_BAND_BINS).
    ``overlap`` is a fraction (0 to 0.95). When there are more segments than
    ``max_columns``, consecutive segments are averaged into that many
    columns. The Welch average always uses every segment.
    """
    values = np.asarray(values, dtype=np.float64)
    nfft = int(nfft)
    if nfft < 2:
        raise ValueError("the FFT length must be at least 2")
    if values.size < nfft:
        raise ValueError(f"the window has {values.size} samples, fewer than the "
                         f"FFT length {nfft}")
    fft_len = max(nfft, int(fft_len or nfft))
    if fft_len > MAX_FFT_LEN:
        raise ValueError(f"FFT length {fft_len} is over the {MAX_FFT_LEN} limit; "
                         "use wider frequency bins")
    overlap = min(max(float(overlap), 0.0), 0.95)
    hop = max(1, int(round(nfft * (1.0 - overlap))))
    nseg = 1 + (values.size - nfft) // hop
    columns = nseg if not max_columns else min(nseg, int(max_columns))
    if fft_len > nfft and nseg * fft_len > MAX_FFT_WORK:
        raise ValueError(f"{fs / fft_len:.4g} Hz bins over {nseg:,} segments is too much work: "
                         "use wider bins, a shorter time window or less overlap")
    w = window_array(window, nfft)
    scale = 1.0 / (fs * float(np.sum(w * w)))
    all_freqs = np.fft.rfftfreq(fft_len, 1.0 / fs)
    lo = 0 if fmin is None else int(np.searchsorted(all_freqs, max(0.0, float(fmin)) - 1e-9))
    hi = all_freqs.size if fmax is None else int(np.searchsorted(all_freqs, float(fmax) + 1e-9,
                                                                 side="right"))
    lo = max(0, min(lo, all_freqs.size - 1))
    hi = max(lo + 1, min(hi, all_freqs.size))
    nbins = hi - lo
    if nbins > MAX_BAND_BINS:
        raise ValueError(f"{nbins:,} frequency bins in the band (at most {MAX_BAND_BINS:,}): "
                         "narrow the frequency range or use wider bins")
    one_sided = np.full(all_freqs.size, 2.0)
    one_sided[0] = 1.0
    if fft_len % 2 == 0:
        one_sided[-1] = 1.0
    one_sided = one_sided[lo:hi]
    frames = np.lib.stride_tricks.sliding_window_view(values, nfft)[::hop][:nseg]
    # Column of each segment: equal runs of consecutive segments.
    column_of = (np.arange(nseg, dtype=np.int64) * columns) // nseg
    psd = np.zeros((columns, nbins))
    counts = np.bincount(column_of, minlength=columns).astype(np.float64)
    welch = np.zeros(nbins)
    batch = max(1, BATCH_SAMPLES // fft_len)
    for first in range(0, nseg, batch):
        last = min(nseg, first + batch)
        segment = _detrend(np.array(frames[first:last], dtype=np.float64), detrend) * w
        spectrum = np.fft.rfft(segment, n=fft_len, axis=1)[:, lo:hi]
        power = (spectrum.real ** 2 + spectrum.imag ** 2) * (scale * one_sided)
        welch += power.sum(axis=0)
        cols = column_of[first:last]
        starts = np.flatnonzero(np.r_[True, cols[1:] != cols[:-1]])
        psd[cols[starts]] += np.add.reduceat(power, starts, axis=0)
    psd /= counts[:, None]
    welch /= nseg
    centres = (np.arange(nseg) * hop + nfft / 2.0) / fs
    times = np.bincount(column_of, weights=centres, minlength=columns) / counts
    return Spectrogram(freqs=all_freqs[lo:hi], times=times,
                       psd=psd.astype(np.float32), welch=welch, segments=nseg, hop=hop,
                       nfft=nfft, fs=fs, fft_len=fft_len)


def welch(values, fs: float, nfft: int = 1024, overlap: float = 0.5,
          window: str = "Hann", detrend: str = "Mean"):
    """(freqs, PSD): Welch's averaged periodogram."""
    result = spectrogram(values, fs, nfft, overlap, window, detrend, max_columns=1)
    return result.freqs, result.welch


def to_db(power, floor_db: float = -300.0) -> np.ndarray:
    """10 log10(power), clamped at ``floor_db`` (no -inf for zeros)."""
    power = np.asarray(power, dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        db = 10.0 * np.log10(power)
    return np.nan_to_num(np.maximum(db, floor_db), nan=floor_db, neginf=floor_db)


@dataclass
class Analysis:
    """Everything the spectrogram window shows."""
    sampling: Sampling
    result: Spectrogram
    db: np.ndarray               # (columns, bins) float32
    welch_db: np.ndarray
    t_start: float               # time extent of the image (absolute s)
    t_end: float
    warnings: list

    @property
    def freqs(self) -> np.ndarray:
        return self.result.freqs

    @property
    def times(self) -> np.ndarray:
        """Absolute centre time of each column."""
        return self.sampling.t0 + self.result.times

    def auto_range(self) -> tuple[float, float]:
        """A colour range: top 0.1% clipped, about 80 dB of dynamic range."""
        finite = self.db[self.db > -299]
        if finite.size == 0:
            return -100.0, 0.0
        high = float(np.percentile(finite, 99.9))
        low = max(float(np.percentile(finite, 2)), high - 80.0)
        if high <= low:
            low = high - 1.0
        return math.floor(low), math.ceil(high)

    def value_at(self, t: float, f: float):
        """(column time, bin freq, dB) nearest to time t and frequency f."""
        index = self.bin_index(f)
        if not self.t_start <= t <= self.t_end or index is None:
            return None
        span = self.t_end - self.t_start
        column = min(self.result.columns - 1,
                     int((t - self.t_start) / span * self.result.columns)) if span > 0 else 0
        return float(self.times[column]), float(self.freqs[index]), float(self.db[column, index])

    @property
    def bin_hz(self) -> float:
        return self.result.bin_hz

    def band_edges(self) -> tuple[float, float]:
        """(low, high) Hz covered by the kept bins (half a bin past the end centres)."""
        half = self.bin_hz / 2.0
        return (max(0.0, float(self.freqs[0]) - half),
                min(self.sampling.fs / 2.0, float(self.freqs[-1]) + half))

    def bin_index(self, f: float):
        """Index of the kept bin nearest to f, or None outside the band."""
        low, high = self.band_edges()
        if self.freqs.size == 0 or not low <= f <= high:
            return None
        return int(min(self.freqs.size - 1,
                       max(0, round((f - float(self.freqs[0])) / self.bin_hz))))


def analyze(t, y, start: float | None = None, end: float | None = None,
            nfft: int = 1024, overlap: float = 0.5, window: str = "Hann",
            detrend: str = "Mean", max_columns: int = 2000,
            bin_hz: float | None = None, fmin: float | None = None,
            fmax: float | None = None) -> Analysis:
    """uniform_signal + spectrogram + dB, as the spectrogram window shows it.

    A window shorter than ``nfft`` uses the largest power of two that fits
    (at least 8 samples), with a warning. ``bin_hz`` asks for bins at most
    that wide, by zero-padding the FFT; ``fmin`` / ``fmax`` keep only that band.
    """
    sampling = uniform_signal(t, y, start, end)
    warnings = list(sampling.warnings)
    n = sampling.values.size
    nfft = int(nfft)
    if n < nfft:
        fitted = 1 << int(math.floor(math.log2(n)))
        if fitted < 8:
            raise ValueError(f"only {n} samples in the time window; need at least 8")
        warnings.append(f"only {n} samples in the window: FFT length reduced "
                        f"from {nfft} to {fitted}")
        nfft = fitted
    fft_len = nfft
    if bin_hz:
        wanted = int(math.ceil(sampling.fs / float(bin_hz) - 1e-9))
        if wanted > MAX_FFT_LEN:
            warnings.append(f"{bin_hz:g} Hz bins need a {wanted:,}-point FFT; limited to "
                            f"{MAX_FFT_LEN:,} ({sampling.fs / MAX_FFT_LEN:.4g} Hz bins)")
            wanted = MAX_FFT_LEN
        fft_len = max(nfft, wanted)
        if fft_len > nfft:
            warnings.append(f"{sampling.fs / fft_len:.4g} Hz bins are interpolated by zero padding; "
                            f"a {nfft}-sample segment resolves about {sampling.fs / nfft:.4g} Hz "
                            "(a longer FFT length resolves finer, at the cost of time resolution)")
    result = spectrogram(sampling.values, sampling.fs, nfft, overlap, window, detrend,
                         max_columns, fft_len=fft_len, fmin=fmin, fmax=fmax)
    t_start = sampling.t0
    t_end = sampling.t0 + ((result.segments - 1) * result.hop + result.nfft) / sampling.fs
    return Analysis(sampling=sampling, result=result,
                    db=to_db(result.psd).astype(np.float32),
                    welch_db=to_db(result.welch), t_start=t_start, t_end=t_end,
                    warnings=warnings)


def _fit_axis(a: np.ndarray, axis: int, size: int) -> np.ndarray:
    """Resize one axis to ``size``: max of each bin when shrinking (peaks
    survive), nearest neighbour when growing."""
    n = a.shape[axis]
    if n == size:
        return a
    if n > size:
        edges = (np.arange(size, dtype=np.int64) * n) // size
        return np.maximum.reduceat(a, edges, axis=axis)
    index = (np.arange(size, dtype=np.int64) * n) // size
    return np.take(a, index, axis=axis)


def image_grid(db: np.ndarray, width: int, height: int) -> np.ndarray:
    """(columns, bins) dB -> (height, width) pixels, low frequencies at the bottom."""
    width, height = max(1, int(width)), max(1, int(height))
    grid = _fit_axis(_fit_axis(np.asarray(db), 0, width), 1, height)
    return grid.T[::-1]


_VIRIDIS = np.array([
    (68, 1, 84), (72, 35, 116), (64, 67, 135), (52, 94, 141), (41, 120, 142),
    (32, 144, 140), (34, 167, 132), (68, 190, 112), (121, 209, 81), (189, 223, 38),
    (253, 231, 37)], dtype=np.float64)
COLORMAP = np.stack([np.interp(np.linspace(0, 1, 256), np.linspace(0, 1, len(_VIRIDIS)),
                               _VIRIDIS[:, c]) for c in range(3)], axis=1).astype(np.uint8)


def colorize(values: np.ndarray, low: float, high: float) -> np.ndarray:
    """Values -> uint8 RGB (..., 3) through COLORMAP, clipped to [low, high]."""
    span = (high - low) or 1.0
    index = np.clip((np.asarray(values, dtype=np.float64) - low) / span * 255.0, 0, 255)
    return COLORMAP[index.astype(np.uint8)]


def ppm(rgb: np.ndarray) -> bytes:
    """A binary PPM (P6) of an (h, w, 3) uint8 array, for tk.PhotoImage(data=...)."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = rgb.shape[:2]
    return b"P6 %d %d 255 " % (width, height) + rgb.tobytes()
