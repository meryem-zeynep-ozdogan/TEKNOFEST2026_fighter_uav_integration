import math
import time

def circular_generator(
    id="rakip_2",
    hss_center=(38.76410, 30.52360),
    offset_m=120,           # başlangıç uzaklığı (merkezin kuzeyi)
    radius_m=40,            # HSS yarıçapı (sınır)
    angular_speed_deg_s=12, # hız m/s (geriye uyumlu isim)
    rate_hz=5
):
    """
    Düz doğrultuda HSS merkezine gelir; sınırda kırmızı olup geri döner,
    dış çapa ulaşınca tekrar merkeze yönelir.
    """
    meters_lat = 111320
    dt = 1.0 / rate_hz

    # Başlangıç konumu: HSS merkezinin offset_m kadar kuzeyi
    lat = hss_center[0] + (offset_m / meters_lat)
    lon = hss_center[1]
    altitude = 120

    speed_m_s = angular_speed_deg_s  # isim uyumluluğu için
    backoff_m = 5
    heading_to_center = True  # True: merkeze git, False: dışarı çık

    while True:
        meters_lon = meters_lat * math.cos(math.radians(lat))

        dlat_m = (hss_center[0] - lat) * meters_lat
        dlon_m = (hss_center[1] - lon) * meters_lon
        dist_m = math.hypot(dlat_m, dlon_m)

        # Nadir 0 mesafe durumunda küçük bir itme ile kuzeye doğru hareket et
        if dist_m == 0:
            lat += (speed_m_s * dt) / meters_lat
            dist_m = abs((hss_center[0] - lat) * meters_lat)

        step_m = speed_m_s * dt
        status = "NORMAL"

        if heading_to_center:
            # merkeze doğru ilerle
            if dist_m > radius_m + 0.5:
                # merkeze doğru ilerle
                step_m = min(step_m, dist_m - radius_m)
                if dist_m > 0:
                    lat += (dlat_m / dist_m) * (step_m / meters_lat)
                    lon += (dlon_m / dist_m) * (step_m / meters_lon)
            else:
                # sınırda: kırmızı, aynı iterasyonda geri sekiş başlasın
                status = "AUTO_AVOID:HSS_1"
                heading_to_center = False
                bounce_m = backoff_m + step_m
                if dist_m > 0:
                    lat -= (dlat_m / dist_m) * (bounce_m / meters_lat)
                    lon -= (dlon_m / dist_m) * (bounce_m / meters_lon)
        else:
            # merkezden uzaklaş
            status = "AUTO_AVOID:HSS_1"
            if dist_m < offset_m:
                if dist_m > 0:
                    lat -= (dlat_m / dist_m) * (step_m / meters_lat)
                    lon -= (dlon_m / dist_m) * (step_m / meters_lon)
            else:
                heading_to_center = True

        yield {
            "timestamp": time.time(),
            "id": id,
            "lat": lat,
            "lon": lon,
            "alt": altitude,
            "status": status
        }

        time.sleep(dt)
