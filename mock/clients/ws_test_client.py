# ws_test_client.py
import socketio
import time

sio = socketio.Client()

# ----------------------------------------------------------
# Bağlantı kurulduğunda çalışır
# ----------------------------------------------------------
@sio.event
def connect():
    print("\n🔗 Bağlantı kuruldu! Drone'lar başlatılıyor...\n")
    
    # Server ile uyumlu son senaryolar:
    # bizim_iha  → hss_approach
    # rakip_1   → hss_approach
    # rakip_2   → circular

    sio.emit("start_multiple", {
        "drones": [
            {
                "id": "bizim_iha",
                "scenario": "hss_approach",
                "lat": 38.76480,
                "lon": 30.52300
            },
            {
                "id": "rakip_1",
                "scenario": "hss_approach",
                "lat": 38.76450,
                "lon": 30.52300
            },
            {
                "id": "rakip_2",
                "scenario": "circular",
                "lat": 38.76430,
                "lon": 30.52370
            }
        ]
    })

    print("🚀 Drone'lar başlatıldı. 2 saniye bekleniyor...\n")
    time.sleep(2)

    # Test lock denemesi
    test_lock_attempt()


# ----------------------------------------------------------
# Telemetri
# ----------------------------------------------------------
@sio.event
def telemetry(data):
    uav = data["id"]
    lat = round(data["lat"], 6)
    lon = round(data["lon"], 6)
    status = data.get("status", "")
    col = data.get("collision", False)

    print(f"📡 {uav} | {lat}, {lon} | {status} | collision={col}")


# ----------------------------------------------------------
# LOCK RESPONSE
# ----------------------------------------------------------
@sio.event
def lock_response(data):
    print("\n🔐 LOCK RESPONSE:")
    print(data)
    print("----------------------------------------------------\n")


# ----------------------------------------------------------
# Lock test
# ----------------------------------------------------------
def test_lock_attempt():
    print("🔐 Kilit denemesi gönderiliyor...")

    # %100 HSS dışı bir hedef (SUCCESS garanti)
    sio.emit("lock_attempt", {
        "id": "rakip_1",
        "target_lat": 38.76520,
        "target_lon": 30.52430
    })


# ----------------------------------------------------------
@sio.event
def connect_error(err):
    print("❌ Bağlantı hatası:", err)


@sio.event
def disconnect():
    print("🔌 Bağlantı kesildi")


print("🌍 Sunucuya bağlanılıyor...")
sio.connect("http://localhost:8000")
sio.wait()
