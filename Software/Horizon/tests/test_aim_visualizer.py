"""AIM Xtra in the Visualizer (map track + translucent attitude overlay) and light mode."""

import time
import unittest

import attitude_math as am
from attitude_data import LiveAimAttitude, LiveAttitude
from attitude_scene import AIM_ALPHA, AIM_STALE_ALPHA, STALE_ALPHA
import geodesy as geo
from map_data import MapData
from test_visualizer import PAD_LAT, PAD_LON, TILT_EAST_10, fw, state_frame_record

TILT_NORTH_20 = am.from_tilt_azimuth_spin(20.0, 0.0, 0.0)


def aim_gnss(lat, lon, alt=750.0, sats=9):
    return {"timestamp": 1, "gnss_coordinate.latitude": lat, "gnss_coordinate.longitude": lon,
            "gnss_coordinate.altitude": alt, "satellites_used": sats}


def aim_orientation(q):
    x, y, z, w = q
    return {"timestamp": 1, "x": x, "y": y, "z": z, "w": w}


class AimMapTrack(unittest.TestCase):
    def test_aim_gnss_is_plotted_relative_to_pad(self):
        data = MapData()
        data.pad.accept_record("featherweight", fw("BASE", PAD_LAT, PAD_LON))
        lat, lon, _ = geo.enu_to_geodetic(0, 300, 0, PAD_LAT, PAD_LON)
        self.assertFalse(data.accept_aim("AIM", "pressure_payload/v1", {"pressure": 1}, 1.0, False))
        self.assertFalse(data.accept_aim("AIM", "gnss_payload/v1", aim_gnss(lat, lon, sats=3), 1.0, False))
        self.assertFalse(data.accept_aim("AIM", "gnss_payload/v1", aim_gnss(0, 0), 1.0, False))
        self.assertFalse(data.accept_aim("AIM", "gnss_payload/v1", aim_gnss(lat, lon), 1.0, True))  # retained
        self.assertTrue(data.accept_aim("AIM", "gnss_payload/v1", aim_gnss(lat, lon, alt=812.5), 2.0, False))
        latest = data.snapshot()["latest"]["aim"]
        self.assertAlmostEqual(latest["range"], 300, delta=0.5)
        self.assertAlmostEqual(latest["bearing"], 0, delta=0.1)
        self.assertIsNone(latest["point"].agl)              # AIM altitude unit unverified
        self.assertEqual(latest["point"].alt_raw, 812.5)

    def test_aim_attitude_keeps_newest_sample(self):
        live = LiveAimAttitude()
        self.assertFalse(live.accept_aim("AIM", "gnss_payload/v1", {}, 1.0))
        self.assertTrue(live.accept_aim("AIM", "orientation_payload/v1", aim_orientation(TILT_EAST_10), 2.0))
        self.assertFalse(live.accept_aim("AIM", "orientation_payload/v1", aim_orientation(TILT_NORTH_20), 1.0))
        self.assertAlmostEqual(am.attitude_angles(live.current().q)["tilt"], 10.0, places=4)


class AimVisualizerGui(unittest.TestCase):
    def setUp(self):
        from horizon_qt_support import app  # noqa: F401 - creates the QApplication

    def test_overlay_rocket_drawn_with_witness(self):
        from visualizer import OrientationPanel
        live, aim = LiveAttitude(), LiveAimAttitude()
        tab = OrientationPanel(live=live, aim=aim, use_gl=False)
        try:
            live.accept_record(state_frame_record(TILT_EAST_10, state=1))
            tab.refresh()
            plain = tab.scene.item_count()
            self.assertIsNone(tab.scene.state[2])
            self.assertEqual(tab.values["aim_age"].get(), "no AIM orientation")
            aim.accept_aim("AIM", "orientation_payload/v1", aim_orientation(TILT_NORTH_20), time.monotonic())
            tab.refresh()
            self.assertIsNotNone(tab.scene.state[0])        # Witness rocket still drawn
            self.assertIsNotNone(tab.scene.state[2])        # plus the AIM overlay
            self.assertGreater(tab.scene.item_count(), plain)
            self.assertEqual(tab.values["aim_tilt"].get(), "20.0°")
            self.assertIn("AIM Xtra", tab.scene.texts()[0])
            self.assertEqual(tab.scene.state[3:], (1.0, AIM_ALPHA))
            # Stale samples stay on screen, only fainter (both rockets).
            tab.refresh(now=time.monotonic() + 10)
            self.assertIsNotNone(tab.scene.state[0])
            self.assertIsNotNone(tab.scene.state[2])
            self.assertEqual(tab.scene.state[3:], (STALE_ALPHA, AIM_STALE_ALPHA))
            self.assertIn("STALE", tab.values["aim_age"].get())
        finally:
            tab.close()

    def test_map_auto_range_follows_farthest_rocket(self):
        import tempfile
        from map_tiles import TileCache
        from visualizer import MapPanel, auto_range_radius
        self.assertEqual(auto_range_radius({"latest": {}}), 500)
        self.assertEqual(auto_range_radius({"latest": {"a": {"range": 1300}}}), 2000)
        self.assertEqual(auto_range_radius({"latest": {"a": {"range": 90000}}}), 20000)
        with tempfile.TemporaryDirectory() as tmp:
            data = MapData()
            data.pad.accept_record("featherweight", fw("BASE", PAD_LAT, PAD_LON))
            tab = MapPanel(data=data, tiles=TileCache(tmp, fetch=None))
            tab.resize(800, 600)
            try:
                tab.refresh()

                def half_extent():
                    (x0, x1), (y0, y1) = tab.view.viewRange()
                    return min(x1 - x0, y1 - y0) / 2

                self.assertEqual(tab.auto_radius, 500)
                self.assertAlmostEqual(half_extent(), 500, delta=5)
                lat, lon, _ = geo.enu_to_geodetic(0, 3500, 0, PAD_LAT, PAD_LON)
                data.accept_aim("AIM", "gnss_payload/v1", aim_gnss(lat, lon), time.monotonic(), False)
                tab.refresh()
                self.assertEqual(tab.auto_radius, 5000)
                self.assertAlmostEqual(half_extent(), 5000, delta=50)
                self.assertAlmostEqual(tab.pan[0], 0, delta=1)
                lat, lon, _ = geo.enu_to_geodetic(40000, 0, 0, PAD_LAT, PAD_LON)
                data.accept_record("featherweight", fw("TRACKER", lat, lon, t=time.monotonic()))
                tab.refresh()
                self.assertEqual(tab.auto_radius, 20000)          # capped at 20 km
                tab._manual_zoom(tab.zoom + 1)                    # +/- (or wheel/drag) takes over
                self.assertFalse(tab.auto_range.get())
            finally:
                tab.close()

    def test_light_mode(self):
        from PyQt6 import QtGui, QtWidgets
        import pyqtgraph as pg
        palette = QtWidgets.QApplication.instance().palette()
        roles = QtGui.QPalette.ColorRole
        self.assertEqual(palette.color(roles.Base).name(), "#ffffff")
        self.assertEqual(palette.color(roles.Text).name(), "#000000")
        self.assertEqual(palette.color(roles.WindowText).name(), "#000000")
        self.assertEqual(pg.getConfigOption("background"), "w")
        self.assertEqual(pg.getConfigOption("foreground"), "k")

    def test_horizon_feeds_aim_to_map_and_attitude(self):
        import tempfile
        from horizon_qt_support import make_horizon
        with tempfile.TemporaryDirectory() as tmp:
            app = make_horizon(tmp)
            try:
                app._featherweight_record(fw("BASE", PAD_LAT, PAD_LON))
                lat, lon, _ = geo.enu_to_geodetic(100, 0, 0, PAD_LAT, PAD_LON)
                now = time.monotonic()
                app._aim_sample("AIM", "gnss_payload/v1", aim_gnss(lat, lon), now, False)
                app._aim_sample("AIM", "orientation_payload/v1", aim_orientation(TILT_EAST_10), now, False)
                window = app.open_visualizer()
                window.map.refresh()
                self.assertIn("aim", window.map.last_snapshot["latest"])
                self.assertIn("AIM", " ".join(window.map.texts()))
                window.orientation.refresh()
                self.assertIsNotNone(window.orientation.scene.state[2])
                window.close()
            finally:
                app.close()


if __name__ == "__main__":
    unittest.main()
