# scenarios/base_generator.py
import json
import math
import time

# HSS yapı tipi:
# HSS = {
#   "HSS_1": {
#       "center": (lat, lon),
#       "radius_m": 40.0
#   },
#   ...
# }

def load_hss_polygons(path: str):
    """
    HSS bölgelerini JSON'dan okur.
    Artık polygon yerine dairesel HSS kullanıyoruz:
    - center: [lat, lon]
    - radius_m: yarıçap (metre)
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    zones = {}
    for z in data.get("zones", []):
        cid = z.get("id", "HSS_1")
        center = z.get("center", [38.76410, 30.52360])
        radius_m = float(z.get("radius_m", 40.0))
        zones[cid] = {
            "center": (float(center[0]), float(center[1])),
            "radius_m": radius_m,
        }
    return zones


def _haversine(lat1, lon1, lat2, lon2):
    """
    İki koordinasyon arasındaki mesafeyi metre cinsinden hesaplar.
    """
    R = 6371000.0  # Dünya yarıçapı (metre)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def is_in_hss(lat: float, lon: float, hss_zones) -> tuple[bool, str | None]:
    """
    Verilen (lat, lon) koordinatının dairesel HSS içinde olup olmadığını kontrol eder.
    Birden fazla HSS varsa, ilk bulduğunu döner.

    Dönen değer:
    - (True, "HSS_1")  -> içerde
    - (False, None)    -> dışarda
    """
    for zone_id, zinfo in hss_zones.items():
        (clat, clon) = zinfo["center"]
        radius_m = zinfo["radius_m"]
        d = _haversine(lat, lon, clat, clon)
        if d <= radius_m:
            return True, zone_id
    return False, None


def now() -> float:
    return time.time()
