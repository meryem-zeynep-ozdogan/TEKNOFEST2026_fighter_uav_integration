# scenarios/hss_approach.py

import time, math
from .base_generator import now

def hss_approach_generator(
    id="uav",
    start=(38.7648, 30.5239, 120),
    hss_center=(38.76410, 30.52360),
    hss_radius_m=40,
    stop_margin_m=15,   # HSS'e 15 metre kala geri dön
    speed=6,
    rate_hz=5
):

    lat, lon, alt = start
    dt = 1.0 / rate_hz

    meters_lat = 111320
    avoid_timer = 0   # HSS kenarinda kalmamasi icin kisa sure tangente kac

    while True:

        meters_lon = meters_lat * math.cos(math.radians(lat))

        dlat = (hss_center[0] - lat)
        dlon = (hss_center[1] - lon)

        dist_m = math.sqrt(
            (dlat * meters_lat)**2 +
            (dlon * meters_lon)**2
        )

        if avoid_timer > 0:
            # Kisa sureli tangensiyel kacis
            avoid_timer -= 1
            heading_rad = math.atan2(dlon, dlat) + math.pi / 2  # merkeze dik
            lat += (speed * math.cos(heading_rad)) * dt / meters_lat
            lon += (speed * math.sin(heading_rad)) * dt / meters_lon

            yield {
                "timestamp": now(),
                "id": id,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "status": "BOUNDARY_AVOID"
            }
            time.sleep(dt)
            continue

        # --- Sınırdan geri dönme ---
        if dist_m <= hss_radius_m + stop_margin_m:

            yield {
                "timestamp": now(),
                "id": id,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "status": "BOUNDARY_AVOID"
            }

            # 2 m geri (HSS merkezinden uzaklaş)
            backoff_m = 2.0
            lat -= (dlat / dist_m) * backoff_m
            lon -= (dlon / dist_m) * backoff_m
            avoid_timer = int(1.5 / dt)  # ~1.5 saniye tangente kac

        else:
            # --- HSS'e doğru yaklaş ---
            step_lat = (dlat / dist_m) * speed * dt * (1 / meters_lat) * meters_lat
            step_lon = (dlon / dist_m) * speed * dt * (1 / meters_lon) * meters_lon

            lat += step_lat
            lon += step_lon

            yield {
                "timestamp": now(),
                "id": id,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "status": "NORMAL"
            }

        time.sleep(dt)
