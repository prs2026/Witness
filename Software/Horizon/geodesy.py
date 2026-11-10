"""Small, dependency-free geodesy for Horizon's map: WGS-84 ENU, range/bearing
and Web-Mercator (OpenStreetMap "slippy map") tile maths.

Conventions match Witness: ENU = +E east, +N north, +U up, metres. Bearings are
compass degrees clockwise from true north, 0..360. Latitude/longitude are
decimal degrees, north/east positive. Altitudes are metres; the caller decides
whether they are MSL or relative (ENU only needs a consistent reference).
"""

import math

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
MEAN_EARTH_RADIUS_M = 6371008.8
TILE_SIZE = 256
MAX_MERCATOR_LAT = 85.05112878
MIN_ZOOM, MAX_ZOOM = 0, 19


def valid_lat_lon(lat, lon, *, reject_null_island=True):
    """True for finite in-range coordinates. 0,0 is the decoders' "no fix"."""
    if type(lat) not in (int, float) or type(lon) not in (int, float):
        return False
    if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
        return False
    return not (reject_null_island and lat == 0 and lon == 0)


def geodetic_to_ecef(lat, lon, alt=0.0):
    phi, lam = math.radians(lat), math.radians(lon)
    sin_phi = math.sin(phi)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_phi * sin_phi)
    return ((n + alt) * math.cos(phi) * math.cos(lam),
            (n + alt) * math.cos(phi) * math.sin(lam),
            (n * (1 - WGS84_E2) + alt) * sin_phi)


def ecef_to_geodetic(x, y, z):
    """Bowring-start iteration; sub-millimetre for terrestrial points."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    alt = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / math.cos(lat) - n if abs(lat) < math.radians(89.9) else abs(z) / abs(sin_lat) - n * (1 - WGS84_E2)
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0=0.0):
    """ENU metres of (lat, lon, alt) relative to the origin (lat0, lon0, alt0)."""
    x, y, z = geodetic_to_ecef(lat, lon, alt)
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(lat0), math.radians(lon0)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    east = -sl * dx + cl * dy
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    up = cp * cl * dx + cp * sl * dy + sp * dz
    return east, north, up


def enu_to_geodetic(east, north, up, lat0, lon0, alt0=0.0):
    phi, lam = math.radians(lat0), math.radians(lon0)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    dx = -sl * east - sp * cl * north + cp * cl * up
    dy = cl * east - sp * sl * north + cp * sl * up
    dz = cp * north + sp * up
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance on the mean-radius sphere, metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi, dlam = p2 - p1, math.radians(lon2 - lon1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * MEAN_EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, h))))


def initial_bearing_deg(lat1, lon1, lat2, lon2):
    """Compass bearing from point 1 towards point 2, 0..360 (0 for equal points)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlam)
    if abs(x) < 1e-15 and abs(y) < 1e-15:
        return 0.0
    return math.degrees(math.atan2(y, x)) % 360.0


def enu_range_bearing(east, north):
    """(horizontal range m, compass bearing deg) of an ENU offset."""
    rng = math.hypot(east, north)
    return rng, (math.degrees(math.atan2(east, north)) % 360.0 if rng > 1e-9 else 0.0)


def compass_point(bearing):
    names = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
             "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")
    return names[int((bearing % 360.0) / 22.5 + 0.5) % 16]


# ------------------------------------------------------------ Web Mercator

def clamp_zoom(zoom):
    return max(MIN_ZOOM, min(MAX_ZOOM, int(zoom)))


def lat_lon_to_world_px(lat, lon, zoom):
    """Global pixel coordinates (x east, y south) at an integer zoom level."""
    lat = max(-MAX_MERCATOR_LAT, min(MAX_MERCATOR_LAT, lat))
    scale = TILE_SIZE * (1 << zoom)
    x = (lon + 180.0) / 360.0 * scale
    s = math.sin(math.radians(lat))
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * scale
    return x, y


def world_px_to_lat_lon(x, y, zoom):
    scale = TILE_SIZE * (1 << zoom)
    lon = x / scale * 360.0 - 180.0
    n = math.pi - 2 * math.pi * y / scale
    return math.degrees(math.atan(math.sinh(n))), lon


def lat_lon_to_tile(lat, lon, zoom):
    x, y = lat_lon_to_world_px(lat, lon, zoom)
    n = 1 << zoom
    return min(n - 1, max(0, int(x // TILE_SIZE))), min(n - 1, max(0, int(y // TILE_SIZE)))


def metres_per_pixel(lat, zoom):
    return 2 * math.pi * WGS84_A * math.cos(math.radians(lat)) / (TILE_SIZE * (1 << zoom))


def tiles_covering(lat, lon, radius_m, zoom):
    """(x, y) tiles of one zoom level covering a square of +/-radius_m around a point."""
    if not valid_lat_lon(lat, lon, reject_null_island=False) or radius_m < 0:
        return []
    cx, cy = lat_lon_to_world_px(lat, lon, zoom)
    half = radius_m / metres_per_pixel(lat, zoom)
    n = 1 << zoom
    x0, x1 = int((cx - half) // TILE_SIZE), int((cx + half) // TILE_SIZE)
    y0, y1 = max(0, int((cy - half) // TILE_SIZE)), min(n - 1, int((cy + half) // TILE_SIZE))
    return [(x % n, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]


def nice_ring_step(span_m, rings=4):
    """Range-ring spacing (1/2/2.5/5 x 10^k metres) giving about `rings` rings."""
    raw = max(span_m / max(1, rings), 1.0)
    magnitude = 10 ** math.floor(math.log10(raw))
    for step in (1, 2, 2.5, 5, 10):
        if step * magnitude >= raw:
            return step * magnitude
    return 10 * magnitude


def format_distance(metres):
    if metres is None or not math.isfinite(metres):
        return "--"
    return f"{metres / 1000:.2f} km" if abs(metres) >= 1000 else f"{metres:.0f} m"
