"""OpenStreetMap raster tile disk cache with a single, rate-limited worker thread.

The map always works without this (pad-centred grid). When online, missing
tiles are fetched in the background and written to TILE_CACHE_DIR so they are
available at the launch site without internet. The GUI never runs on the worker:
the UI calls request() (non-blocking, returns a cached path or None) and
drain_completed() from its timer, then decodes the PNG on a Qt worker thread (visualizer.load_tile_rgba).

OSM tile usage policy (https://operations.osmfoundation.org/policies/tiles/):
 - a User-Agent that identifies the application (no personal data is sent);
 - one connection, at most MAX_REQUESTS_PER_S requests per second;
 - tiles are cached locally and re-used; they are only re-fetched after
   REFRESH_AFTER_S, and an old tile is still shown meanwhile;
 - "Pre-cache area" is limited to MAX_PRECACHE_TILES tiles and MAX_PRECACHE_ZOOM;
 - the map shows "(c) OpenStreetMap contributors".
"""

from collections import deque
import os
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request

from geodesy import MAX_ZOOM, tiles_covering

TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "HorizonGroundStation/1.0 (amateur rocketry ground station map; Python urllib)"
TILE_CACHE_DIR = Path(__file__).resolve().parent / "tile_cache" / "osm"
ATTRIBUTION = "© OpenStreetMap contributors"
MAX_REQUESTS_PER_S = 2.0
REFRESH_AFTER_S = 30 * 24 * 3600
OFFLINE_BACKOFF_S = 30.0
MAX_QUEUE = 256
MAX_PRECACHE_TILES = 600
MAX_PRECACHE_ZOOM = 17
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def tile_path(root, z, x, y):
    """Cache path <root>/<z>/<x>/<y>.png (the same layout as the tile URL)."""
    if not (0 <= z <= MAX_ZOOM and 0 <= x < (1 << z) and 0 <= y < (1 << z)):
        raise ValueError(f"tile {z}/{x}/{y} out of range")
    return Path(root) / str(int(z)) / str(int(x)) / f"{int(y)}.png"


def tile_url(z, x, y):
    return TILE_URL.format(z=int(z), x=int(x), y=int(y))


def precache_plan(lat, lon, radius_m, zooms):
    """Tiles (z, x, y) covering +/-radius_m around a point at each zoom."""
    plan = []
    for z in sorted(set(int(z) for z in zooms)):
        if 0 <= z <= MAX_PRECACHE_ZOOM:
            plan += [(z, x, y) for x, y in tiles_covering(lat, lon, radius_m, z)]
    return plan


def _default_fetch(url, timeout=10.0):
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT, "Accept": "image/png"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read(2_000_000)


class TileCache:
    """Disk cache plus background fetcher. fetch=None disables the network."""

    def __init__(self, root=TILE_CACHE_DIR, *, online=True, fetch=_default_fetch,
                 rate=MAX_REQUESTS_PER_S, clock=time.monotonic):
        self.root = Path(root)
        self.fetch = fetch
        self.online = bool(online) and fetch is not None
        self.min_interval = 1.0 / max(0.1, rate)
        self.clock = clock
        self._lock = threading.Condition()
        self._queue = deque()          # viewport requests, newest first
        self._bulk = deque()           # pre-cache requests, after viewport ones
        self._queued = set()
        self._bulk_keys = set()
        self._failed = {}              # key -> retry-after (missing tiles, HTTP errors)
        self._completed = set()
        self._offline_until = 0.0
        self._stop = False
        self._thread = None
        self.fetched = self.errors = 0
        self.last_error = ""
        self.precache_total = self.precache_done = 0

    # ------------------------------------------------------------ paths
    def path(self, z, x, y):
        return tile_path(self.root, z, x, y)

    def cached(self, z, x, y):
        path = self.path(z, x, y)
        return path if path.is_file() else None

    # ------------------------------------------------------------- UI API
    def request(self, z, x, y):
        """Cached file path now (maybe stale), else None; queues a fetch if online."""
        path = self.path(z, x, y)
        try:
            stat = path.stat()
        except OSError:
            stat = None
        if stat is None or time.time() - stat.st_mtime > REFRESH_AFTER_S:
            self._enqueue((z, x, y), bulk=False)
        return path if stat is not None else None

    def precache(self, keys):
        """Queue uncached tiles from precache_plan(). Returns how many were queued."""
        keys = list(keys)[:MAX_PRECACHE_TILES]
        todo = [k for k in keys if self.cached(*k) is None]
        with self._lock:
            self.precache_total, self.precache_done = len(todo), 0
        for key in todo:
            self._enqueue(key, bulk=True)
        return len(todo)

    def set_online(self, online):
        with self._lock:
            self.online = bool(online) and self.fetch is not None
            self._offline_until = 0.0
            if not self.online:
                self._queue.clear()
                self._bulk.clear()
                self._queued.clear()
                self._bulk_keys.clear()
            self._lock.notify_all()

    def drain_completed(self):
        with self._lock:
            done, self._completed = self._completed, set()
        return done

    def status(self):
        with self._lock:
            pending = len(self._queue) + len(self._bulk)
            offline = self.clock() < self._offline_until
        if not self.online:
            return "tiles off (offline grid only)"
        text = f"tiles: {self.fetched} fetched, {pending} queued"
        if self.precache_total:
            text += f", pre-cache {self.precache_done}/{self.precache_total}"
        if offline:
            text += f", OFFLINE ({self.last_error})"
        elif self.errors:
            text += f", {self.errors} errors"
        return text

    def close(self):
        with self._lock:
            self._stop = True
            self._lock.notify_all()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2)

    # ------------------------------------------------------------ worker
    def _enqueue(self, key, *, bulk):
        if not self.online:
            return
        with self._lock:
            if key in self._queued or self._failed.get(key, 0) > self.clock():
                return
            queue = self._bulk if bulk else self._queue
            if bulk:
                queue.append(key)
                self._bulk_keys.add(key)
            else:
                queue.appendleft(key)
                while len(queue) > MAX_QUEUE:
                    self._queued.discard(queue.pop())
            self._queued.add(key)
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="osm-tiles", daemon=True)
                self._thread.start()
            self._lock.notify_all()

    def _next(self):
        with self._lock:
            while not self._stop:
                wait = self._offline_until - self.clock()
                if (self._queue or self._bulk) and wait <= 0:
                    return (self._queue or self._bulk).popleft()
                self._lock.wait(timeout=max(0.05, wait) if wait > 0 else 0.5)
            return None

    def _run(self):
        last = -1e9
        while True:
            key = self._next()
            if key is None:
                return
            delay = self.min_interval - (self.clock() - last)
            if delay > 0:
                time.sleep(delay)
            last = self.clock()
            self.fetch_now(*key)
            with self._lock:
                bulk = key in self._bulk_keys
                self._bulk_keys.discard(key)
                self._queued.discard(key)
                if bulk:
                    self.precache_done += 1

    def fetch_now(self, z, x, y):
        """Fetch one tile synchronously (worker thread). Returns its path or None."""
        key = (z, x, y)
        try:
            data = self.fetch(tile_url(z, x, y))
            if not data.startswith(PNG_SIGNATURE):
                raise ValueError("response is not a PNG")
            path = self.path(z, x, y)
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".tmp")
            tmp.write_bytes(data)
            os.replace(tmp, path)
        except urllib.error.HTTPError as exc:
            with self._lock:
                self.errors += 1
                self.last_error = f"HTTP {exc.code}"
                self._failed[key] = self.clock() + (3600 if exc.code == 404 else 300)
                if exc.code in (403, 429):     # blocked / throttled: stop asking for a while
                    self._offline_until = self.clock() + 10 * OFFLINE_BACKOFF_S
            return None
        except (OSError, ValueError) as exc:
            with self._lock:
                self.errors += 1
                self.last_error = type(exc).__name__
                self._offline_until = self.clock() + OFFLINE_BACKOFF_S
                self._failed[key] = self.clock() + OFFLINE_BACKOFF_S
            return None
        with self._lock:
            self.fetched += 1
            self._completed.add(key)
        return path
