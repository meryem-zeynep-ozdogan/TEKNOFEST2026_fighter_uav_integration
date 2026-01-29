# server/ws_server.py

from gevent import sleep
from math import radians, sin, cos, sqrt, atan2

from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit

# -------------------------------------------------------------------
# FLASK + WEBSOCKET UYGULAMASI
# -------------------------------------------------------------------
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="gevent")

# -------------------------------------------------------------------
# HSS + Senaryo yüklemeleri
# -------------------------------------------------------------------
from scenarios.hss_approach import hss_approach_generator
from scenarios.straight import straight_generator
from scenarios.circular import circular_generator
from scenarios.base_generator import load_hss_polygons, is_in_hss


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


HSS = load_hss_polygons("config/hss_zones.json")
first_zone = list(HSS.values())[0]
HSS_CENTER = (first_zone["center"][0], first_zone["center"][1])
HSS_RADIUS = first_zone["radius_m"]

active_positions = {}
active_generators = {}


# -------------------------------------------------------------------
# 1) WEBSOCKET EVENTLERİ
# -------------------------------------------------------------------

@socketio.on("lock_success")
def handle_lock_success(data):
    print("[LOCK_SUCCESS] Paket alindi:", data)


def broadcast(gen, uav_id, stop_flag):
    try:
        for msg in gen:
            if stop_flag.get("stop"):
                break

            lat = msg.get("lat")
            lon = msg.get("lon")

            if lat is None or lon is None:
                continue

            inside, zone_id = is_in_hss(lat, lon, HSS)
            active_positions[uav_id] = (lat, lon)

            # HSS sadece yasak bölge kontrolü için kullanılır; görev durumu üretilmez.
            if inside:
                msg["lat"] += 0.00001
                msg["lon"] += 0.00001

            telemetry = {
                "id": msg.get("id", uav_id),
                "lat": msg.get("lat"),
                "lon": msg.get("lon"),
                "alt": msg.get("alt"),
            }

            if "timestamp" in msg:
                telemetry["timestamp"] = msg["timestamp"]
            if "vx" in msg:
                telemetry["vx"] = msg["vx"]
            if "vy" in msg:
                telemetry["vy"] = msg["vy"]
            if "vz" in msg:
                telemetry["vz"] = msg["vz"]
            if "yaw" in msg:
                telemetry["yaw"] = msg["yaw"]
            if "speed" in msg:
                telemetry["speed"] = msg["speed"]
            elif all(k in msg for k in ("vx", "vy", "vz")):
                telemetry["speed"] = sqrt(msg["vx"] ** 2 + msg["vy"] ** 2 + msg["vz"] ** 2)

            socketio.emit("telemetry", telemetry)
            sleep(0.02)

    except Exception as exc:
        print(f"[SERVER] Generator hata verdi ({uav_id}): {exc}")
    finally:
        active_positions.pop(uav_id, None)
        active_generators.pop(uav_id, None)


@socketio.on("lock_attempt")
def handle_lock_attempt(data):
    uav_id = data.get("id")
    tlat = data.get("target_lat")
    tlon = data.get("target_lon")

    resp = {"id": uav_id, "distance": 0, "status": "FAIL", "reason": ""}

    if uav_id not in active_positions:
        resp["reason"] = "Drone aktif degil"
        emit("lock_response", resp)
        return

    if tlat is None or tlon is None:
        resp["reason"] = "Hedef koordinat eksik"
        emit("lock_response", resp)
        return

    inside, zid = is_in_hss(tlat, tlon, HSS)
    curr_lat, curr_lon = active_positions[uav_id]

    d = haversine(curr_lat, curr_lon, tlat, tlon)
    resp["distance"] = d

    if inside:
        resp["reason"] = f"Hedef HSS icinde (zone={zid})"
        emit("lock_response", resp)
        return

    resp["status"] = "OK"
    resp["reason"] = "Lock basarili"
    emit("lock_response", resp)

    socketio.emit("lock_success", {"id": uav_id, "distance": d})

    print(f"[LOCK] {uav_id}: OK - Mesafe={d:.2f}m")


@socketio.on("start_multiple")
def start_multiple(data):
    drones = data.get("drones", [])
    print(f"[SERVER] STARTING {len(drones)} DRONES")

    for d in drones:
        uid = d["id"]
        scenario = d["scenario"]

        if uid in active_generators:
            active_generators[uid]["stop"] = True
            sleep(0.05)

        lat = float(d.get("lat"))
        lon = float(d.get("lon"))

        if scenario == "hss_approach":
            gen = hss_approach_generator(
                id=uid,
                start=(lat, lon, 120),
                hss_center=HSS_CENTER,
                hss_radius_m=HSS_RADIUS,
                stop_margin_m=20,
                speed=4,
                rate_hz=5,
            )
        elif scenario == "circular":
            gen = circular_generator(
                id=uid,
                hss_center=HSS_CENTER,
                offset_m=120,
                radius_m=HSS_RADIUS,
                angular_speed_deg_s=6,
                rate_hz=5,
            )
        else:
            gen = straight_generator(id=uid, start=(lat, lon, 120))

        print(f"[SERVER] {uid} baslatiliyor - scenario={scenario}")

        stop_flag = {"stop": False}
        active_generators[uid] = stop_flag

        socketio.start_background_task(broadcast, gen, uid, stop_flag)

    emit("info", {"status": "OK"})


# -------------------------------------------------------------------
# 2) HTTP ENDPOINTLER - Yarışma ihtiyaçları
# -------------------------------------------------------------------

@app.route("/api/giris", methods=["POST"])
def api_giris():
    data = request.get_json()
    print("[HTTP] Giris istegi:", data)
    return jsonify({"team": 39}), 200


@app.route("/api/sunucusaati", methods=["GET"])
def api_sunucusaati():
    from datetime import datetime

    now = datetime.utcnow()
    print("[HTTP] Sunucu saati gonderildi")
    return jsonify(
        {
            "gun": now.day,
            "saat": now.hour,
            "dakika": now.minute,
            "saniye": now.second,
            "milisaniye": int(now.microsecond / 1000),
        }
    )


@app.route("/api/kamikaze_bilgisi", methods=["POST"])
def kamikaze_api():
    data = request.get_json()
    print("[HTTP] Kamikaze bilgisi alindi:", data)
    return jsonify({"result": "OK"}), 200


@app.route("/api/kilitlenme_bilgisi", methods=["POST"])
def kilitlenme_api():
    data = request.get_json()
    print("[HTTP] Kilitlenme bilgisi alindi:", data)
    return jsonify({"result": "OK"}), 200


@app.route("/api/qr", methods=["POST"])
def qr_api():
    data = request.get_json()
    print("[HTTP] QR alindi:", data)
    return jsonify({"result": "OK"}), 200


@app.route("/api/takeoff", methods=["POST"])
def api_takeoff():
    data = request.get_json(silent=True)
    print("[HTTP] Takeoff bilgisi alindi:", data)
    return jsonify({"result": "OK"}), 200


# -------------------------------------------------------------------
# RUN
# -------------------------------------------------------------------
if __name__ == "__main__":
    print("[SERVER] Mock Sunucu (WS + HTTP) basliyor ws://127.0.0.1:8000  |  http://127.0.0.1:8000")
    socketio.run(app, host="0.0.0.0", port=8000, debug=False)
