"""Spectrogram / Welch maths (spectrum.py) and the viewer's spectrogram window."""

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import spectrum  # noqa: E402
import viewer_data as viewer  # noqa: E402  (data logic)
from viewer_qt_support import ViewerCase, pump  # noqa: E402


def sine(freq, fs, seconds, amplitude=1.0, offset=0.0):
    t = np.arange(int(fs * seconds)) / fs
    return t, offset + amplitude * np.sin(2 * np.pi * freq * t)


class SpectrumTests(unittest.TestCase):
    def test_sine_peaks_in_the_right_bin(self):
        fs, f0 = 500.0, 37.5
        t, y = sine(f0, fs, 20, offset=5.0)
        for window in spectrum.WINDOWS:
            result = spectrum.spectrogram(y, fs, nfft=1000, overlap=0.5, window=window)
            peak = result.freqs[np.argmax(result.welch)]
            self.assertAlmostEqual(peak, f0, places=6, msg=window)
            # Every column peaks there too (the mean is removed: no DC peak).
            self.assertTrue(np.all(result.freqs[np.argmax(result.psd, axis=1)] == f0))
        self.assertEqual(result.psd.shape, (result.columns, 501))

    def test_psd_integrates_to_the_variance(self):
        fs = 1000.0
        _t, y = sine(50.0, fs, 10, amplitude=2.0)
        freqs, psd = spectrum.welch(y, fs, nfft=1024, overlap=0.5, window="Hann")
        power = np.sum(psd) * (freqs[1] - freqs[0])
        self.assertAlmostEqual(power, 2.0, delta=0.05)          # A^2 / 2

    def test_segments_hop_and_column_cap(self):
        y = np.zeros(10_000)
        result = spectrum.spectrogram(y, 100.0, nfft=256, overlap=0.75, max_columns=20)
        self.assertEqual(result.hop, 64)
        self.assertEqual(result.segments, 1 + (10_000 - 256) // 64)
        self.assertEqual(result.columns, 20)
        self.assertTrue(np.all(np.diff(result.times) > 0))
        with self.assertRaises(ValueError):
            spectrum.spectrogram(np.zeros(100), 100.0, nfft=256)

    def test_detrend_modes(self):
        n = 512
        ramp = np.arange(n, dtype=float)
        linear = spectrum._detrend(ramp[None, :] * 3 + 7, "Linear")
        self.assertLess(np.abs(linear).max(), 1e-9)
        mean = spectrum._detrend(ramp[None, :], "Mean")
        self.assertAlmostEqual(mean.mean(), 0.0)
        np.testing.assert_array_equal(spectrum._detrend(ramp[None, :], "None"), ramp[None, :])

    def test_uneven_sampling_is_flagged_and_resampled(self):
        rng = np.random.default_rng(1)
        fs, f0 = 200.0, 20.0
        t = np.cumsum(rng.uniform(0.004, 0.006, 6000))           # jittered ~200 Hz
        t = np.delete(t, np.arange(1000, 1040))                  # and a gap
        y = np.sin(2 * np.pi * f0 * t)
        sampling = spectrum.uniform_signal(t, y)
        self.assertTrue(sampling.uneven)
        self.assertTrue(sampling.resampled)
        self.assertGreater(sampling.gaps, 0)
        self.assertAlmostEqual(sampling.fs, fs, delta=2.0)
        self.assertIn("uneven sampling", sampling.warnings[0])
        analysis = spectrum.analyze(t, y, nfft=512)
        peak = analysis.freqs[np.argmax(analysis.welch_db)]
        self.assertAlmostEqual(peak, f0, delta=analysis.freqs[1])
        self.assertTrue(analysis.warnings)

    def test_even_sampling_window_and_duplicates(self):
        t, y = sine(10.0, 100.0, 10)
        sampling = spectrum.uniform_signal(t, y, start=2.0, end=4.0)
        self.assertFalse(sampling.uneven)
        self.assertEqual(sampling.values.size, 201)
        self.assertAlmostEqual(sampling.t0, 2.0)
        dup = spectrum.uniform_signal(np.r_[t[:50], t[49:]], np.r_[y[:50], y[49:]])
        self.assertEqual(dup.duplicates, 1)
        with self.assertRaises(ValueError):
            spectrum.uniform_signal([1.0], [2.0])

    def test_analyze_reduces_nfft_and_reads_values(self):
        t, y = sine(5.0, 100.0, 3)
        analysis = spectrum.analyze(t, y, nfft=4096)
        self.assertEqual(analysis.result.nfft, 256)
        self.assertIn("FFT length reduced", analysis.warnings[-1])
        low, high = analysis.auto_range()
        self.assertLess(low, high)
        hit = analysis.value_at((analysis.t_start + analysis.t_end) / 2, 5.0)
        self.assertAlmostEqual(hit[1], 5.0, delta=analysis.freqs[1])
        self.assertIsNone(analysis.value_at(analysis.t_end + 1, 5.0))

    def test_image_grid_and_colours(self):
        db = np.zeros((3000, 513))
        db[1234, 100] = 50.0                                   # one hot cell
        grid = spectrum.image_grid(db, 400, 200)
        self.assertEqual(grid.shape, (200, 400))
        self.assertEqual(grid.max(), 50.0)                     # max-pooled: survives
        row = np.argwhere(grid == 50.0)[0][0]
        self.assertGreater(row, 100)                           # low freq near bottom
        small = spectrum.image_grid(np.arange(6.0).reshape(2, 3), 4, 6)
        self.assertEqual(small.shape, (6, 4))
        rgb = spectrum.colorize(np.array([[-10.0, 0.0, 10.0]]), -5, 5)
        self.assertEqual(rgb.dtype, np.uint8)
        np.testing.assert_array_equal(rgb[0, 0], spectrum.COLORMAP[0])
        np.testing.assert_array_equal(rgb[0, 2], spectrum.COLORMAP[255])
        self.assertTrue(spectrum.ppm(rgb).startswith(b"P6 3 1 255 "))


import pyqtgraph as pg  # noqa: E402


class SpectrogramWindowTests(ViewerCase):
    def test_window_computes_renders_and_hovers(self):
        path = Path(self.tmp) / "imu.csv"
        t, y = sine(40.0, 500.0, 8)
        with path.open("w") as f:
            f.write("Flight_Time_(s),Accel_X\n")
            for ti, yi in zip(t - 2.0, y):
                f.write(f"{ti:.3f},{yi:.5f}\n")
        panel = self.make(900, 600)
        self.assertTrue(panel.load_csv(path))
        self.plots(panel)
        ref = (next(iter(panel.datasets)), "Accel_X")
        window = panel.open_spectrogram(ref)
        pump()
        self.assertEqual(float(window.start_edit.text()), -2.0)
        self.assertTrue(window.wait(30))
        self.assertIsNotNone(window.analysis, window.status_label.text())
        self.assertIn("fs = 500 Hz", window.status_label.text())
        # Rendered headless: image with a colour bar, time x frequency axes.
        a = window.analysis
        self.assertIsNotNone(window.image.image)
        self.assertEqual(window.image.image.shape[1], a.result.columns)
        rect = window.image.boundingRect()
        self.assertGreater(rect.width(), 0)
        x_range, y_range = window.p1.vb.viewRange()
        self.assertAlmostEqual(x_range[0], a.t_start, places=6)
        self.assertAlmostEqual(y_range[1], float(a.freqs[-1]), places=6)
        self.assertEqual(tuple(window.colorbar.levels()), tuple(map(float, window.db_range())))
        self.assertTrue(window.p2.isVisible())
        text = window.readout_at((a.t_start + a.t_end) / 2, 40.0)
        self.assertRegex(text, r"t = .* s +f = 4\d(\.\d+)? Hz +-?\d+\.\d dB")
        self.assertIn("average spectrum", window.welch_readout_at(40.0))
        # Crosshair from a mouse move over the image.
        scene_pos = window.p1.vb.mapViewToScene(
            pg.Point((a.t_start + a.t_end) / 2, 40.0))
        window._mouse_moved(scene_pos)
        self.assertTrue(window.v_line.isVisible())
        self.assertIn("Hz", window.hover_label.text())
        # A zoomed window, other settings, manual colours; bad input is reported.
        panel.set_x_view((0.0, 2.0))
        window.use_zoom()
        window.nfft_combo.setCurrentText("256")
        window.window_combo.setCurrentText("Blackman")
        window.detrend_combo.setCurrentText("Linear")
        window.compute()
        self.assertTrue(window.wait(30))
        self.assertEqual(window.analysis.result.nfft, 256)
        self.assertAlmostEqual(window.analysis.sampling.t0, 0.0)
        window.db_min_edit.setText("-120")
        window.db_max_edit.setText("0")
        window.apply_colours()
        self.assertEqual(window.db_range(), (-120.0, 0.0))
        self.assertEqual(tuple(window.colorbar.levels()), (-120.0, 0.0))
        window.welch_check.setChecked(False)
        self.assertFalse(window.p2.isVisible())
        window.end_edit.setText("-5")
        window.compute()
        self.assertIn("Check the settings", window.status_label.text())
        window.close()
        pump()


if __name__ == "__main__":
    unittest.main()
