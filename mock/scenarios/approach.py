import time, math
import random
from .base_generator import now

def approach_generator(
    id="rakip_1",
    start=(38.7655, 30.5230, 120),
    target=(38.7640, 30.5235, 120),   # Bizim İHA konumu
    speed=2,
    rate_hz=1
):
    lat, lon, alt = start
    dt = 1.0 / rate_hz
    meters_lat = 111320
    # Hedefi değiştirilebilir yap
    current_target = list(target)

    while True:
        meters_lon = meters_lat * math.cos(math.radians(lat))

        dlat = (current_target[0] - lat) * meters_lat
        dlon = (current_target[1] - lon) * meters_lon
        dist = math.hypot(dlat, dlon)

        if dist < 1.0:
            # Hedefe ulaştı, rastgele yönde devam et (sürekli hareket)
            random_heading = math.radians(random.randint(0, 359))
            vx = speed * math.sin(random_heading)
            vy = speed * math.cos(random_heading)
            # Hedefi de güncelle (yeni rastgele nokta, 100-200m uzaklıkta)
            offset_dist = random.uniform(100, 200)  # metre
            current_target[0] = lat + (offset_dist / meters_lat) * math.cos(random_heading)
            current_target[1] = lon + (offset_dist / meters_lon) * math.sin(random_heading)
        else:
            vx = speed * (dlon / dist)
            vy = speed * (dlat / dist)
        
        lat += (vy / meters_lat) * dt
        lon += (vx / meters_lon) * dt

        yield {
            "timestamp": now(),
            "id": id,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "vx": vx,
            "vy": vy,
            "vz": 0,
            "yaw": 0,
        }

        time.sleep(dt)
