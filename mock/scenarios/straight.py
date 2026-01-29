import math
import time
import random

def straight_generator(
    id="rakip_1",
    start=(38.7640, 30.5235, 120),
    heading_deg=None,   # Rastgele yön için None
    speed=2,
    rate_hz=1
):
    lat, lon, alt = start
    dt = 1.0 / rate_hz

    # Eğer heading None ise rastgele yön seç
    if heading_deg is None:
        heading_deg = random.randint(0, 359)

    curr_yaw = heading_deg

    while True:
        heading = math.radians(curr_yaw)

        # Dünyada metre -> derece dönüşümü
        d_lat = (speed * math.cos(heading)) * dt / 111_320
        d_lon = (speed * math.sin(heading)) * dt / (111_320 * math.cos(math.radians(lat)))

        lat += d_lat
        lon += d_lon

        msg = {
            "timestamp": time.time(),
            "id": id,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "vx": speed * math.cos(heading),
            "vy": speed * math.sin(heading),
            "vz": 0,
            "yaw": curr_yaw
        }

        # SERVER'dan HSS kaçınma sırasında gelen yaw override edilebilir.
        # Bu nedenle generator sonraki döngüde yeni yaw'ı kullanmalı.
        if "yaw" in msg:
            curr_yaw = msg["yaw"]

        yield msg
        time.sleep(dt)
