"""Visualizer window: geodesy, pad selection, tile maths/cache, map tracks and
the orientation tab's quaternion handling, plus a headless GUI smoke test."""

import math
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import attitude_math as am  # noqa: E402
from attitude_data import LiveAttitude, make_sample
import geodesy as geo
from map_data import MapData
import map_tiles
from map_tiles import PNG_SIGNATURE, TileCache, precache_plan, tile_path, tile_url
from pad_reference import PadReference

PAD_LAT, PAD_LON = 35.3466, -117.8085      # a desert launch site
TILT_EAST_10 = am.from_tilt_azimuth_spin(10.0, 90.0, 0.0)


def fw(unit, lat, lon, alt_ft=2000, fix=3, t=1.0, crc=True):
    return {"source": "COM5", "received_mono": t, "packet": {
        "packet": "GPS_STAT", "unit_type": unit, "latitude_deg": lat, "longitude_deg": lon,
        "altitude_ft": alt_ft, "fix_type": fix, "text_crc_valid": crc, "radio_crc_status": "CRC_OK"}}


def aperture(lat, lon, alt=700.0, t=1.0, age=500):
    return {"source": "COM7", "received_mono": t, "packet": {
        "packet": "ground_station_status", "rssi": -60, "uptime": 1, "gps_fix_quality": 1,
        "gps_satellites": 9, "latitude": lat, "longitude": lon, "gps_altitude": alt,
        "gps_time": None, "gps_hdop": 0.9, "gps_fix_age_ms": age}}


def witness(position=(0, 0, 0), *, lat=0.0, lon=0.0, alt=0.0, state=0, locked=0, t=1.0,
            q=(0, -0.70710678, 0, 0.70710678), kind="state", replay=False):
    status = state | (locked << 3)
    return {"source": "COM7", "received_mono": t, "replay": replay, "packet": {
        "packet": kind, "status": status, "uptime": int(t * 1000), "orientation": list(q),
        "position": list(position), "velocity": [0, 0, 0], "latitude": lat, "longitude": lon,
        "gps_altitude": alt, "kalman_altitude": position[2]}}


class Geodesy(unittest.TestCase):
    def test_enu_round_trip_and_axes(self):
        for e, n, u in ((0, 0, 0), (1234.5, -987.6, 321.0), (-5000, 4000, 12000)):
            lat, lon, alt = geo.enu_to_geodetic(e, n, u, PAD_LAT, PAD_LON, 700)
            back = geo.geodetic_to_enu(lat, lon, alt, PAD_LAT, PAD_LON, 700)
            for a, b in zip(back, (e, n, u)):
                self.assertAlmostEqual(a, b, delta=1e-3)
        e, n, u = geo.geodetic_to_enu(PAD_LAT + 0.001, PAD_LON, 0, PAD_LAT, PAD_LON, 0)
        self.assertAlmostEqual(e, 0, delta=1e-6)
        self.assertAlmostEqual(n, 110.94, delta=0.05)    # WGS-84 metres per mdeg latitude at 35 N
        e, n, _ = geo.geodetic_to_enu(PAD_LAT, PAD_LON + 0.001, 0, PAD_LAT, PAD_LON, 0)
        self.assertAlmostEqual(e, 91.0, delta=0.1)
        self.assertAlmostEqual(n, 0, delta=0.01)

    def test_distance_bearing(self):
        self.assertAlmostEqual(geo.haversine_m(0, 0, 0, 1), 111195.1, delta=1)
        self.assertAlmostEqual(geo.initial_bearing_deg(0, 0, 0, 1), 90.0)
        self.assertAlmostEqual(geo.initial_bearing_deg(0, 0, 1, 0), 0.0)
        self.assertAlmostEqual(geo.initial_bearing_deg(0, 0, -1, 0), 180.0)
        self.assertAlmostEqual(geo.initial_bearing_deg(0, 0, 0, -1), 270.0)
        self.assertEqual(geo.initial_bearing_deg(10, 10, 10, 10), 0.0)
        rng, bearing = geo.enu_range_bearing(300, 400)
        self.assertAlmostEqual(rng, 500)
        self.assertAlmostEqual(bearing, math.degrees(math.atan2(300, 400)))
        self.assertAlmostEqual(geo.enu_range_bearing(-1, 0)[1], 270)
        self.assertEqual([geo.compass_point(b) for b in (0, 44, 90, 181, 350)], ["N", "NE", "E", "S", "N"])
        # Short ranges: ENU and spherical great-circle agree to < 0.5 %.
        lat, lon, _ = geo.enu_to_geodetic(800, 600, 0, PAD_LAT, PAD_LON)
        self.assertAlmostEqual(geo.haversine_m(PAD_LAT, PAD_LON, lat, lon), 1000, delta=5)
        self.assertAlmostEqual(geo.initial_bearing_deg(PAD_LAT, PAD_LON, lat, lon), 53.13, delta=0.2)
        self.assertEqual(geo.format_distance(950), "950 m")
        self.assertEqual(geo.format_distance(1500), "1.50 km")

    def test_validity(self):
        self.assertTrue(geo.valid_lat_lon(PAD_LAT, PAD_LON))
        self.assertFalse(geo.valid_lat_lon(0, 0))
        self.assertTrue(geo.valid_lat_lon(0, 0, reject_null_island=False))
        for lat, lon in ((91, 0), (0, 181), (math.nan, 1), (None, 1), ("1", 2)):
            self.assertFalse(geo.valid_lat_lon(lat, lon))

    def test_tile_maths(self):
        self.assertEqual(geo.lat_lon_to_tile(0.0001, 0.0001, 1), (1, 0))
        self.assertEqual(geo.lat_lon_to_tile(51.5074, -0.1278, 10), (511, 340))   # London
        self.assertEqual(geo.lat_lon_to_tile(85.1, -180, 3), (0, 0))
        self.assertEqual(geo.lat_lon_to_tile(-85.1, 179.999, 3), (7, 7))
        x, y = geo.lat_lon_to_world_px(PAD_LAT, PAD_LON, 16)
        lat, lon = geo.world_px_to_lat_lon(x, y, 16)
        self.assertAlmostEqual(lat, PAD_LAT, places=9)
        self.assertAlmostEqual(lon, PAD_LON, places=9)
        self.assertAlmostEqual(geo.metres_per_pixel(0, 0), 156543.03, delta=0.01)
        self.assertAlmostEqual(geo.metres_per_pixel(60, 1), 156543.03 / 4, delta=0.01)
        tiles = geo.tiles_covering(PAD_LAT, PAD_LON, 1000, 16)
        self.assertIn(geo.lat_lon_to_tile(PAD_LAT, PAD_LON, 16), tiles)
        span = 2000 / (geo.metres_per_pixel(PAD_LAT, 16) * 256)    # ~10 tiles wide
        self.assertLessEqual(len(tiles), (math.ceil(span) + 1) ** 2)
        self.assertGreaterEqual(len(tiles), math.floor(span) ** 2)
        wrapped = geo.tiles_covering(0, 179.9999, 50000, 6)
        self.assertTrue(all(0 <= x < 64 for x, _ in wrapped))
        self.assertEqual([geo.nice_ring_step(s, 4) for s in (90, 1000, 3000)], [25, 250, 1000])


class Tiles(unittest.TestCase):
    def test_cache_paths_urls_and_plan(self):
        root = Path("cache")
        self.assertEqual(tile_path(root, 16, 11300, 26200), root / "16" / "11300" / "26200.png")
        self.assertEqual(tile_url(3, 1, 2), "https://tile.openstreetmap.org/3/1/2.png")
        for bad in ((3, 8, 0), (3, 0, -1), (20, 0, 0)):
            with self.assertRaises(ValueError):
                tile_path(root, *bad)
        plan = precache_plan(PAD_LAT, PAD_LON, 2000, range(12, 20))
        self.assertEqual({z for z, _, _ in plan}, set(range(12, 18)))   # never above MAX_PRECACHE_ZOOM
        self.assertLessEqual(len(plan), map_tiles.MAX_PRECACHE_TILES)

    def test_user_agent_identifies_app_without_personal_data(self):
        self.assertIn("Horizon", map_tiles.USER_AGENT)
        self.assertNotIn("@", map_tiles.USER_AGENT)
        self.assertLessEqual(map_tiles.MAX_REQUESTS_PER_S, 2)

    def test_worker_fetches_caches_and_backs_off(self):
        calls, lock = [], threading.Lock()

        def fetch(url):
            with lock:
                calls.append(url)
            if url.endswith("/9.png"):
                return b"<html>error</html>"
            return PNG_SIGNATURE + b"tile"
        with tempfile.TemporaryDirectory() as directory:
            cache = TileCache(directory, fetch=fetch, rate=100)
            try:
                self.assertIsNone(cache.request(10, 5, 6))
                self.assertIsNone(cache.request(10, 5, 6))      # deduplicated while queued
                deadline = time.monotonic() + 5
                while not cache.cached(10, 5, 6) and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(cache.request(10, 5, 6), Path(directory) / "10" / "5" / "6.png")
                self.assertEqual(cache.drain_completed(), {(10, 5, 6)})
                self.assertEqual(calls, ["https://tile.openstreetmap.org/10/5/6.png"])
                # A non-PNG answer is an error and puts the cache into offline backoff.
                self.assertIsNone(cache.fetch_now(10, 5, 9))
                self.assertEqual(cache.errors, 1)
                self.assertIn("OFFLINE", cache.status())
                self.assertFalse((Path(directory) / "10" / "5" / "9.png").exists())
            finally:
                cache.close()

    def test_offline_never_fetches_and_precache_skips_cached(self):
        with tempfile.TemporaryDirectory() as directory:
            offline = TileCache(directory, fetch=None)
            self.assertFalse(offline.online)
            self.assertIsNone(offline.request(1, 0, 0))
            self.assertIsNone(offline._thread)
            path = tile_path(directory, 1, 0, 0)
            path.parent.mkdir(parents=True)
            path.write_bytes(PNG_SIGNATURE)
            self.assertEqual(offline.request(1, 0, 0), path)
            gate = threading.Event()
            online = TileCache(directory, fetch=lambda url: gate.wait(5) and PNG_SIGNATURE, rate=100)
            try:
                self.assertEqual(online.precache([(1, 0, 0), (1, 1, 0), (1, 0, 1)]), 2)
                self.assertEqual(online.precache_total, 2)
                gate.set()
                deadline = time.monotonic() + 5
                while online.precache_done < 2 and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(online.precache_done, 2)
                online.set_online(False)
                self.assertEqual(online.status(), "tiles off (offline grid only)")
            finally:
                online.close()


class PadSelection(unittest.TestCase):
    def test_priority_and_manual_override(self):
        pad = PadReference()
        self.assertIsNone(pad.position())
        self.assertTrue(pad.accept_record("featherweight", fw("TRACKER", 35.0, -117.0, 2100)))
        self.assertEqual(pad.position().source, "tracker")
        self.assertFalse(pad.accept_record("featherweight", fw("TRACKER", 36.0, -117.0)))  # first fix only
        pad.accept_record("witness", witness(lat=35.1, lon=-117.1, alt=690, locked=1))
        self.assertEqual(pad.position().source, "witness")
        pad.accept_record("witness", aperture(35.2, -117.2, 701.0))
        self.assertEqual(pad.position().source, "aperture")
        self.assertEqual(pad.altitude_msl(), (701.0, "aperture"))
        pad.accept_record("featherweight", fw("BASE", 35.3, -117.3, alt_ft=1000, fix=2))
        self.assertEqual(pad.position().source, "featherweight")
        self.assertEqual(pad.altitude_msl(), (701.0, "aperture"))     # 2D fix: no altitude
        pad.accept_record("featherweight", fw("BASE", 35.3, -117.3, alt_ft=1000, fix=3))
        self.assertAlmostEqual(pad.altitude_msl()[0], 304.8)
        pad.set_manual(35.4, -117.4)
        self.assertEqual((pad.position().source, pad.position().lat), ("manual", 35.4))
        self.assertEqual(pad.altitude_msl()[1], "featherweight")      # manual has no altitude
        pad.clear_manual()
        self.assertEqual(pad.position().source, "featherweight")
        with self.assertRaises(ValueError):
            pad.set_manual(95, 0)

    def test_rejects_unusable_fixes(self):
        pad = PadReference()
        for record in (fw("BASE", 0, 0), fw("BASE", 35, -117, fix=1), fw("BASE", 35, -117, crc=False),
                       fw("FOUND", 35, -117), aperture(None, None), aperture(0.0, 0.0),
                       witness(lat=35, lon=-117, locked=0), witness(lat=35, lon=-117, locked=1, state=1),
                       dict(aperture(35, -117), replay=True), {"packet": "junk"}, None):
            for kind in ("featherweight", "witness"):
                self.assertFalse(pad.accept_record(kind, record), (kind, record))
        self.assertIsNone(pad.position())

    def test_witness_pad_fix_is_a_mean_and_launch_site(self):
        pad = PadReference()
        for lat in (35.0001, 35.0003):
            pad.accept_record("witness", witness(lat=lat, lon=-117.0, alt=700, locked=1))
        site = pad.witness_launch_site()
        self.assertAlmostEqual(site.lat, 35.0002)
        self.assertEqual(site.detail, "2 fixes")
        self.assertEqual(pad.altitude_msl(), (700, "witness"))


class MapTracks(unittest.TestCase):
    def test_kalman_track_relative_to_pad_and_launch_site(self):
        data = MapData()
        data.pad.accept_record("witness", aperture(PAD_LAT, PAD_LON, 700.0))
        # Not initialised (zero quaternion and zero position): no track point.
        self.assertFalse(data.accept_record("witness", witness(q=(0, 0, 0, 0), t=0.5)))
        data.accept_record("witness", witness((300, 400, 1000), t=1.0, state=2))
        snap = data.snapshot()
        point = snap["latest"]["kf"]["point"]
        self.assertEqual((point.east, point.north, point.agl), (300, 400, 1000))   # origin assumed at pad
        self.assertAlmostEqual(snap["latest"]["kf"]["range"], 500)
        self.assertFalse(snap["kf_placed"])
        # Witness GPS on the pad 100 m north of Aperture places the Kalman origin there.
        north_lat, north_lon, _ = geo.enu_to_geodetic(0, 100, 0, PAD_LAT, PAD_LON)
        data.pad.accept_record("witness", witness(lat=north_lat, lon=north_lon, alt=702, locked=1))
        snap = data.snapshot()
        point = snap["latest"]["kf"]["point"]
        self.assertTrue(snap["kf_placed"])
        self.assertAlmostEqual(point.east, 300, delta=0.5)
        self.assertAlmostEqual(point.north, 500, delta=0.5)
        self.assertEqual(snap["reference"][2], "aperture")

    def test_gps_tracks_need_lock_or_fix_and_use_pad_msl(self):
        data = MapData()
        data.pad.accept_record("featherweight", fw("BASE", PAD_LAT, PAD_LON, alt_ft=2000))
        lat, lon, _ = geo.enu_to_geodetic(-200, 0, 0, PAD_LAT, PAD_LON)
        self.assertFalse(data.accept_record("witness", witness(q=(0, 0, 0, 0), lat=lat, lon=lon, locked=0)))
        data.accept_record("witness", witness(q=(0, 0, 0, 0), lat=lat, lon=lon, alt=2000 * 0.3048 + 50,
                                              locked=1, t=2.0))
        data.accept_record("featherweight", fw("TRACKER", lat, lon, alt_ft=2100, t=3.0))
        self.assertFalse(data.accept_record("featherweight", fw("TRACKER", lat, lon, fix=1, t=4.0)))
        self.assertFalse(data.accept_record("featherweight", fw("TRACKER", lat, lon, t=2.5)))  # out of order
        snap = data.snapshot()
        wgps, fwp = snap["latest"]["wgps"], snap["latest"]["fw"]
        self.assertAlmostEqual(wgps["range"], 200, delta=0.5)
        self.assertAlmostEqual(wgps["bearing"], 270, delta=0.1)
        self.assertAlmostEqual(wgps["point"].agl, 50, places=6)
        self.assertAlmostEqual(fwp["point"].agl, 100 * 0.3048, places=6)
        self.assertEqual(len(snap["tracks"]["fw"]), 1)
        # Cached conversions are recomputed when the pad changes.
        data.pad.set_manual(lat, lon)
        self.assertAlmostEqual(data.snapshot()["latest"]["fw"]["range"], 0, delta=0.01)

    def test_bounded_track_and_no_reference(self):
        data = MapData(points=10)
        for i in range(25):
            data.accept_record("witness", witness((i, 0, 0), t=float(i)))
        snap = data.snapshot()
        self.assertIsNone(snap["reference"])
        self.assertEqual([p.east for p in snap["tracks"]["kf"]], list(range(15, 25)))
        data.accept_record("witness", witness((99, 0, 0), t=30.0))
        self.assertEqual(data.snapshot()["tracks"]["kf"][-1].east, 99)
        self.assertEqual(len(data.snapshot()["tracks"]["kf"]), 10)


class OrientationHandling(unittest.TestCase):
    def test_attitude_condition(self):
        from visualizer import attitude_condition
        valid = make_sample(1.0, TILT_EAST_10, "0x02")
        zero = make_sample(2.0, (0, 0, 0, 0), "0x02")
        self.assertEqual(attitude_condition(None, None, None)[0], "none")
        self.assertEqual(attitude_condition(valid, None, 0)[0], "ok")
        self.assertEqual(attitude_condition(zero, None, 0)[0], "init")
        self.assertIn("HITL", attitude_condition(zero, None, 0, hitl=1)[2])
        self.assertEqual(attitude_condition(zero, valid, 1)[0], "lost")
        self.assertEqual(attitude_condition(zero, valid, 3)[1], "ATTITUDE LOST")
        self.assertEqual(attitude_condition(zero, None, 2)[0], "unavailable")
        nan = make_sample(2.0, (math.nan, 0, 0, 1), "0x02")
        self.assertEqual(attitude_condition(nan, None, 0)[0], "init")


def state_frame_record(q, *, state=0, t=None):
    from converter import parse_packet
    payload = (struct.pack(">HI", state, 1000) + struct.pack(">4f", *q) + bytes(24)
               + bytes(6) + struct.pack(">ff", 0, 0) + struct.pack(">H", 3900) + bytes([7]))
    return {"source": "COM7", "received_at": time.time(), "received_mono": time.monotonic() if t is None else t,
            "packet": parse_packet(0x02, payload)}


PNG_1PX = ("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg==")


class VisualizerGui(unittest.TestCase):
    def setUp(self):
        from horizon_qt_support import app  # noqa: F401 - creates the QApplication
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_orientation_panel_valid_then_lost(self):
        from visualizer import OrientationPanel
        live = LiveAttitude()
        tab = OrientationPanel(live=live, use_gl=False)
        try:
            tab.refresh()
            self.assertEqual(tab.condition[0], "none")
            self.assertIn("NO ATTITUDE DATA", tab.scene.texts())
            live.accept_record(state_frame_record(TILT_EAST_10, state=1))
            tab.refresh()
            self.assertEqual(tab.condition[0], "ok")
            self.assertEqual(tab.values["tilt"].get(), "10.0°")
            self.assertEqual(tab.values["heading"].get(), "90.0°")
            self.assertEqual(tab.labels["tilt"].text(), "10.0°")
            live.accept_record(state_frame_record((0, 0, 0, 0), state=2))
            tab.refresh()
            self.assertEqual(tab.condition[0], "lost")
            self.assertEqual(tab.values["tilt"].get(), "--")
            self.assertIn("ATTITUDE LOST", tab.values["status"].get())
            self.assertIn("ATTITUDE LOST", tab.scene.texts())
            self.assertIsNotNone(tab.scene.state[1])      # last valid attitude drawn as a ghost
            # Back on the pad, a zero quaternion is a fresh initialisation, not "lost".
            live.accept_record(state_frame_record((0, 0, 0, 0), state=0))
            tab.refresh()
            self.assertEqual(tab.condition[0], "init")
        finally:
            tab.close()

    def test_map_panel_draws_offline_tiles_and_sets_pad(self):
        from horizon_qt_support import pump_until
        from visualizer import MapPanel
        data = MapData()
        tiles = TileCache(self.tmp.name, fetch=None)
        tab = MapPanel(data=data, tiles=tiles)
        tab.auto_range.set(False)      # manual zoom/pan behaviour (auto range: test_aim_visualizer)
        tab.resize(800, 600)
        try:
            tab.refresh()
            self.assertIn("no data", tab.values["kf"].get())
            self.assertIn("PAD POSITION UNKNOWN", tab.warning.text())
            data.pad.accept_record("witness", aperture(PAD_LAT, PAD_LON))
            data.accept_record("witness", witness((300, 400, 120), t=time.monotonic(), state=2))
            tab.set_zoom(16)
            tab._readout_at = 0
            tab.refresh()
            self.assertIn("500 m @ 37", tab.values["kf"].get())
            self.assertIn("120.0 m AGL", tab.values["kf"].get())
            self.assertIn("Aperture", tab.values["pad"].get())
            texts = tab.texts()
            self.assertIn("PAD", texts)
            self.assertIn("N", texts)
            self.assertIn("KF 120 m", texts)
            self.assertNotIn(map_tiles.ATTRIBUTION, texts)        # offline: no tiles drawn
            self.assertEqual(tab.view.state["aspectLocked"], 1.0)
            # Track drawn in ENU metres relative to the pad.
            line = tab._tracks["kf"][0]
            self.assertAlmostEqual(line.xData[-1], 300, delta=1)
            self.assertAlmostEqual(line.yData[-1], 400, delta=1)
            # A cached tile is drawn offline (decoded on a worker thread), with attribution.
            z = tab.zoom
            x, y = geo.lat_lon_to_tile(PAD_LAT, PAD_LON, z)
            path = tile_path(self.tmp.name, z, x, y)
            path.parent.mkdir(parents=True)
            import base64
            path.write_bytes(base64.b64decode(PNG_1PX))
            tab._tiles_dirty = True
            tab.refresh()
            self.assertTrue(pump_until(lambda: tab.tiles_drawn >= 1, 5))
            self.assertIn(map_tiles.ATTRIBUTION, tab.texts())
            item = tab._tile_items[(z, x, y)]
            rect = item.mapRectToParent(item.boundingRect())
            east, north, size = __import__("visualizer").tile_rect_enu(z, x, y, PAD_LAT, PAD_LON)
            self.assertAlmostEqual(rect.left(), east, delta=0.01)
            self.assertAlmostEqual(rect.width(), size, delta=0.01)
            self.assertTrue(rect.left() <= 0 <= rect.right() and rect.top() <= 0 <= rect.bottom())
            # Set pad here: a click 100 m east of the centre.
            tab.toggle_set_pad()
            self.assertTrue(tab.set_pad_mode)
            tab.set_pad_at_enu(100.0, 0.0)
            pad = data.pad.position()
            self.assertEqual(pad.source, "manual")
            e, n, _ = geo.geodetic_to_enu(pad.lat, pad.lon, 0, PAD_LAT, PAD_LON, 0)
            self.assertAlmostEqual(e, 100, delta=0.5)
            self.assertAlmostEqual(n, 0, delta=0.5)
            self.assertFalse(tab.set_pad_mode)
            tab.set_zoom(99)
            self.assertEqual(tab.zoom, 19)
            self.assertAlmostEqual(tab.mpp(), geo.metres_per_pixel(tab._ref_lat(), 19), delta=1e-6)
            tab.follow.set(True)
            tab._invalidate()
            tab.refresh()
            self.assertNotEqual(tab.pan, (0.0, 0.0))
            with patch("visualizer.ask_yes_no", return_value=False) as ask, patch("visualizer.inform") as inform:
                self.assertFalse(tab.precache_dialog())          # offline: told to enable tiles
                inform.assert_called_once()
                ask.assert_not_called()
        finally:
            tab.close()

    def test_visualizer_shows_map_and_attitude_side_by_side(self):
        from PyQt6 import QtCore
        from horizon_qt_support import pump
        from visualizer import VisualizerWindow
        window = VisualizerWindow(map_data=MapData(), tiles=TileCache(self.tmp.name, fetch=None), use_gl=False)
        try:
            window.resize(1200, 700)
            window.show()
            pump(0.1)
            self.assertEqual(window.splitter.orientation(), QtCore.Qt.Orientation.Horizontal)
            self.assertEqual(window.splitter.count(), 2)
            self.assertIs(window.splitter.widget(0), window.orientation)
            self.assertIs(window.splitter.widget(1), window.map)
            # Both visible at the same time (not tabs), next to each other, both usable size.
            self.assertTrue(window.orientation.isVisible())
            self.assertTrue(window.map.isVisible())
            left, right = window.orientation.geometry(), window.map.geometry()
            self.assertLessEqual(left.right(), right.left())
            self.assertGreater(left.width(), 200)
            self.assertGreater(right.width(), 300)
            self.assertTrue(left.intersects(QtCore.QRect(left.left(), right.top(), left.width(), right.height())))
            # The splitter is resizable and never collapses either view.
            window.splitter.setSizes([300, 900])
            pump(0.05)
            self.assertAlmostEqual(window.splitter.sizes()[0], 300, delta=10)
            self.assertFalse(window.splitter.childrenCollapsible())
            self.assertTrue(window.orientation.scene.isVisible())
            self.assertTrue(window.map.plot.isVisible())
        finally:
            window.close()
        self.assertFalse(window.is_open())

    def test_horizon_opens_visualizer_fed_by_records(self):
        from horizon_qt_support import make_horizon
        app = make_horizon(self.tmp.name)
        try:
            app._featherweight_record(fw("BASE", PAD_LAT, PAD_LON, alt_ft=2000))
            app._witness_record(state_frame_record(TILT_EAST_10, state=1))
            window = app.open_visualizer()
            self.assertIs(app.open_visualizer(), window)
            self.assertEqual(window.splitter.count(), 2)
            window.orientation.refresh()
            self.assertEqual(window.orientation.values["tilt"].get(), "10.0°")
            self.assertEqual(app.pad.position().source, "featherweight")
            self.assertIs(app.flight.data.pad, app.pad)
            window.map.refresh()
            self.assertIn("kf", window.map.last_snapshot["latest"])
            window.close()
            self.assertIsNot(app.open_visualizer(), window)
        finally:
            app.close()


class HorizonAuditHelpers(unittest.TestCase):
    def test_imu_calibration_commands_are_compact_usb_frames(self):
        from witness_protocol import IMU_CALIBRATION_COMMANDS, build_imu_calibration_packet
        self.assertEqual([build_imu_calibration_packet(a) for _, a in IMU_CALIBRATION_COMMANDS],
                         [b"\x05\x43\x47", b"\x05\x43\x41", b"\x05\x43\x58", b"\x05\x43\x53"])
        with self.assertRaises(ValueError):
            build_imu_calibration_packet(0x00)

    def test_download_reports_kalman_log_rows(self):
        from witness_download import count_kalman_rows, describe
        self.assertIn("Kalman", describe("P0000001.BIN"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "P0000001.csv"
            path.write_text("packet_index,packet_type,log_source\n0,state,witness_state\n"
                            "1,kalman_log,kalman_log\n2,kalman_log,kalman_log\n", encoding="utf-8")
            self.assertEqual(count_kalman_rows(path), 2)
            self.assertEqual(count_kalman_rows(Path(directory) / "missing.csv"), 0)


if __name__ == "__main__":
    unittest.main()
