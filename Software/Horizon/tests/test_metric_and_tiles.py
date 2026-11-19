"""Imperial distance text, tile cache location and the 10 km pre-cache plan."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import map_tiles  # noqa: E402
from geodesy import format_distance  # noqa: E402


class FormatDistanceTest(unittest.TestCase):
    def test_feet_below_one_mile_miles_above(self):
        self.assertEqual(format_distance(500), "1640 ft")
        self.assertEqual(format_distance(0.3048), "1 ft")
        self.assertEqual(format_distance(1609.0), "5279 ft")
        self.assertEqual(format_distance(1609.344), "1.00 mi")
        self.assertEqual(format_distance(2012.0), "1.25 mi")
        self.assertEqual(format_distance(12345.4), "7.67 mi")
        self.assertEqual(format_distance(None), "--")


class TileCacheLocationTest(unittest.TestCase):
    def test_default_dir_outside_repo(self):
        repo = Path(__file__).resolve().parents[3]
        self.assertNotIn(repo, map_tiles.default_tile_cache_dir().resolve().parents)

    def test_env_override(self):
        old = os.environ.get("HORIZON_TILE_CACHE")
        os.environ["HORIZON_TILE_CACHE"] = "/x/tiles"
        try:
            self.assertEqual(map_tiles.default_tile_cache_dir(), Path("/x/tiles"))
        finally:
            if old is None:
                del os.environ["HORIZON_TILE_CACHE"]
            else:
                os.environ["HORIZON_TILE_CACHE"] = old

    def test_migrate_moves_and_keeps_existing(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, dst = Path(tmp, "old"), Path(tmp, "new")
            for root, name, data in ((src, "1.png", b"a"), (src, "2.png", b"b"), (dst, "2.png", b"keep")):
                (root / "5" / "3").mkdir(parents=True, exist_ok=True)
                (root / "5" / "3" / name).write_bytes(data)
            self.assertEqual(map_tiles.migrate_legacy_cache(src, dst), 1)
            self.assertEqual((dst / "5" / "3" / "1.png").read_bytes(), b"a")
            self.assertEqual((dst / "5" / "3" / "2.png").read_bytes(), b"keep")


class PrecachePlanTest(unittest.TestCase):
    def test_covers_10_km_within_limit(self):
        for lat in (0.0, 30.0, 45.0):   # launch-site latitudes; nearer the poles zoom 17 is trimmed
            plan = map_tiles.precache_area_plan(lat, -100.0)
            self.assertLessEqual(len(plan), map_tiles.MAX_PRECACHE_TILES)
            self.assertEqual(len(plan), len(set(plan)))
            self.assertEqual(max(z for z, _, _ in plan), 17)
        # Every zoom-15 tile covering 10 km is included (no truncation at mid latitudes).
        want = set(map_tiles.precache_plan(45.0, -100.0, 10_000, (15,)))
        self.assertTrue(want <= set(map_tiles.precache_area_plan(45.0, -100.0)))


if __name__ == "__main__":
    unittest.main()
