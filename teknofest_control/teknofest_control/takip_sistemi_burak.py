# -*- coding: utf-8 -*-
"""
Takip Sistemi (Dogfight, ArduPlane SITL)
DroneKit/MAVLink akışı korunur; UKF (constant-turn), trail/lead, hız-irtifa eşleme,
çarpışma kaçışı ve vision histerezisi içerir.
"""

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import time
import math
import socket
import json
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
from dronekit import connect, LocationGlobalRelative, VehicleMode
from ukf import NonlinearKalmanUKF
from guidance import GuidanceLaw
from pymavlink import mavutil

# Parametreler
try:
    from parametreler import *  # noqa
except ImportError:
    print("parametreler.py bulunamadı, varsayılan parametreler kullanılıyor...")
    D_TRAIL_FAR = 80.0
    D_TRAIL_NEAR = 30.0
    D_SAFE = 22.0
    D_VISION = 35.0
    K_SPEED = 0.35
    V_MIN = 10.0
    V_MAX = 28.0
    K_ALTITUDE = 0.5
    VZ_MAX = 3.0
    T_LEAD_MIN = 0.4
    T_LEAD_MAX = 1.6
    ALPHA_LEAD = 0.8
    UPDATE_RATE_HZ = 5
    SWITCH_MARGIN = 0.15
    LOCK_MIN_TIME = 2.5
    LOST_TIMEOUT = 1.2
    SCORE_WEIGHTS = {
        "distance": 0.6,
        "angle": 1.2,
        "rel_speed": 0.8,
        "altitude": 0.5,
        "risk": 2.0,
        "stability": 1.0,
        "latency": 0.4,
    }

# Bağlantılar
TAKIPCI_CONNECTION = '127.0.0.1:14551'  # Takipçiye direkt bağlan (komut göndermek için)
GPS_SUNUCU_IP = '127.0.0.1'             # GPS sunucu IP
GPS_SUNUCU_PORT = 5001                  # Takipçi sorgu portu
GUNCELLEME_SURESI = 1.0 / UPDATE_RATE_HZ


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def hedef_gps_al(sock):
    """GPS sunucusundan hedef GPS verisini alır."""
    try:
        # GPS isteği gönder
        istek = {"type": "GPS_REQUEST"}
        sock.sendto(json.dumps(istek).encode('utf-8'), (GPS_SUNUCU_IP, GPS_SUNUCU_PORT))

        # Yanıt bekle (kısa timeout)
        sock.settimeout(0.5)
        data, addr = sock.recvfrom(1024)
        yanit = json.loads(data.decode('utf-8'))

        if yanit.get("type") == "GPS_RESPONSE":
            return yanit.get("data")

    except socket.timeout:
        print("[UYARI] GPS sunucudan yanıt alınamadı (timeout)!")
    except Exception as e:
        print(f"[HATA] GPS alınırken: {e}")

    return None


def heading_to_unit(heading_deg: float) -> np.ndarray:
    rad = math.radians(heading_deg)
    return np.array([math.cos(rad), math.sin(rad)])


def send_yaw_command(vehicle, yaw_deg, yaw_rate=15, relative=False):
    """ArduPlane CONDITION_YAW desteklemediği için no-op."""
    return


def send_altitude_command(vehicle, target_alt):
    """
    MAV_CMD_DO_CHANGE_ALT ile irtifa hedefi gonder.
    ArduPlane'de irtifa kontrolü için en etkili yöntem.
    """
    try:
        msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            178,  # MAV_CMD_DO_CHANGE_ALT (sayısal değer - pymavlink sabitinde yok)
            0,  # confirmation
            target_alt,  # param1: Altitude (meters)
            3,           # param2: Frame (3 = GLOBAL_RELATIVE_ALT)
            0,  # param3: unused
            0,  # param4: unused
            0, 0, 0  # param5-7: unused
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print(f"[DEBUG] Alt komut gönderildi: {target_alt:.1f}m")
    except Exception as e:
        print(f"[ERROR] Alt komutu hatası: {e}")
        pass


def send_speed_command(vehicle, speed_mps, airspeed=True, also_ground=True):
    """
    MAV_CMD_DO_CHANGE_SPEED ile hiz hedefi gonder; air + ground.
    HER İKİSİNİ DE gönderiyoruz çünkü ArduPlane konfigürasyonuna göre
    biri diğerini ignore edebilir.
    """
    try:
        if airspeed:
            # Airspeed komutu (speed_type = 0)
            msg = vehicle.message_factory.command_long_encode(
                0, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,  # confirmation
                0,  # speed_type: 0 = Airspeed
                speed_mps,  # speed (m/s)
                -1,  # throttle (-1 = no change)
                0,  # absolute/relative (0 = absolute)
                0, 0, 0  # unused params
            )
            vehicle.send_mavlink(msg)

        if also_ground:
            # Groundspeed komutu (speed_type = 1)
            msg_gs = vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1,  # speed_type: 1 = Groundspeed
                speed_mps,
                -1,
                0,
                0, 0, 0
            )
            vehicle.send_mavlink(msg_gs)

        vehicle.flush()
        print(f"[DEBUG] Hız komutu gönderildi: {speed_mps:.1f} m/s (airspeed={airspeed}, ground={also_ground})")
    except Exception as e:
        print(f"[ERROR] Hız komutu hatası: {e}")
        pass


def send_position_target(vehicle, lat, lon, alt, vx=0, vy=0, vz=0):
    """
    GUIDED pozisyon + hız setpoint'ini SET_POSITION_TARGET_GLOBAL_INT ile gonder.
    vx, vy, vz: NED frame'de m/s cinsinden hız bileşenleri
    """
    try:
        # Type mask: pozisyon + velocity kullan, diğerleri yoksay
        # Bit 0-2: pos (0=kullan), Bit 3-5: vel (0=kullan), Bit 6-11: ignore (1=yoksay)
        type_mask = 0b0000111111000000  # pozisyon + velocity, accel/yaw ignore

        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            vx, vy, vz,  # Hız bileşenleri (NED frame, m/s)
            0, 0, 0,  # afx, afy, afz (ignore)
            0, 0  # yaw, yaw_rate (ignore)
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
    except Exception as e:
        print(f"[WARN] send_position_target hatası: {e}")
        pass


def set_speed_limits(vehicle):
    """ArduPlane hiz parametrelerini 30 m/s tavan ile ayarla (Teknofest uyumlu)."""
    cap = 30.0
    desired_max = min(V_MAX, cap)
    desired_min = max(8.0, V_MIN)
    cruise = max(V_MIN + 2.0, min(desired_max, desired_max - 2.0))

    param_updates = {
        # Hız limitleri (EKSTRA PARAMETRELER EKLENDI!)
        "ARSPD_FBW_MAX": desired_max,
        "ARSPD_FBW_MIN": desired_min,
        "AIRSPEED_MAX": desired_max,  # EKLENDI: Mutlak max airspeed
        "AIRSPEED_CRUISE": cruise,  # EKLENDI: Cruise speed
        "TRIM_ARSPD_CM": int(cruise * 100),  # Trim airspeed cm/s cinsinden
        "WP_SPEED": int(desired_max * 100),  # Waypoint hızını max'a ayarla
        "WP_SPEED_MAX": int(desired_max * 100),

        # Throttle ayarları
        "THR_MAX": 100,  # Maksimum throttle %100
        "THR_MIN": 0,    # Minimum throttle %0
        "TRIM_THROTTLE": 65,  # Cruise throttle %65 (daha agresif)

        # GUIDED mod ayarları (ÇOOOOK ÖNEMLİ!)
        "GUIDED_OPTIONS": 0,  # Standart davranış, velocity ignore

        # Waypoint davranışı (hızlı tepki için)
        "WP_RADIUS": 30,  # Waypoint kabul yarıçapı 30m (daha yakın takip)
        "WP_LOITER_RAD": 50,  # Loiter yarıçapı 50m

        # L1 Controller (turning performance - HIZ KAYBI AZALTMA!)
        "NAVL1_PERIOD": 18,  # L1 period 18s (daha geniş dönüşler = daha az hız kaybı)
        "NAVL1_DAMPING": 0.75,  # Damping 0.75 (stabil ama responsive)

        # TECS (Total Energy Control System) - İRTİFA VE HIZ KONTROLÜ!
        "TECS_CLMB_MAX": 10.0,  # Maksimum tırmanma hızı 10 m/s (hızlı yükselme)
        "TECS_SINK_MAX": 8.0,   # Maksimum iniş hızı 8 m/s (hızlı alçalma)
        "TECS_SINK_MIN": 2.0,   # Minimum iniş hızı 2 m/s
        "TECS_SPDWEIGHT": 2.0,  # Hız kontrolüne daha fazla öncelik (dönüşte hız kaybını azaltır)

        # Roll limit (dönüş kaybını azaltmak için)
        "ROLL_LIMIT_DEG": 45,  # 45 derece max roll (agresif ama güvenli dönüş)
    }

    for name, value in param_updates.items():
        try:
            if name in vehicle.parameters:
                vehicle.parameters[name] = value
                print(f"      {name} -> {value}")
        except Exception as e:
            print(f"[WARN] {name} ayarlanamadi: {e}")


def ensure_guided(vehicle):
    """GUIDED mod ve arm durumunu koru."""
    try:
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
        if not vehicle.armed:
            vehicle.armed = True
    except Exception:
        pass


class TelemetryBuffer:
    def __init__(self, maxlen=20):
        self.samples = deque(maxlen=maxlen)

    def add(self, pos_ned: np.ndarray, timestamp: float):
        self.samples.append((pos_ned.copy(), timestamp))

    def age(self, now: float) -> float:
        if not self.samples:
            return float('inf')
        return now - self.samples[-1][1]

    def stability_penalty(self, now: float, window: float = 2.0) -> float:
        if len(self.samples) < 3:
            return 3.0
        times = [t for _, t in self.samples if now - t <= window]
        if len(times) < 2:
            return 3.0
        intervals = np.diff(times)
        max_gap = float(np.max(intervals)) if len(intervals) else window
        jitter = float(np.std(intervals)) if len(intervals) else 0.0
        penalty = 0.0
        if max_gap > 0.6:
            penalty += (max_gap - 0.6) * 4.0
        penalty += jitter * 5.0
        return penalty


class TargetEstimator:
    """UKF/constant-turn + lead tahmini."""

    def __init__(self, buffer_size: int = 20):
        self.buffer = deque(maxlen=buffer_size)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.omega = 0.0
        self.origin_lat = None
        self.origin_lon = None
        self.last_timestamp: Optional[float] = None
        self.filter = NonlinearKalmanUKF()

    def _gps_to_ned(self, lat, lon, alt) -> np.ndarray:
        x = (lat - self.origin_lat) * 111320
        y = (lon - self.origin_lon) * 111320 * math.cos(math.radians(lat))
        z = alt
        return np.array([x, y, z])

    def gps_to_ned(self, lat, lon, alt) -> np.ndarray:
        if self.origin_lat is None:
            raise ValueError("Origin is not set")
        return self._gps_to_ned(lat, lon, alt)

    def ned_to_gps(self, pos_ned: np.ndarray) -> Tuple[float, float, float]:
        lat = self.origin_lat + pos_ned[0] / 111320
        lon = self.origin_lon + pos_ned[1] / (111320 * math.cos(math.radians(self.origin_lat)))
        alt = pos_ned[2]
        return lat, lon, alt

    def update(self, lat, lon, alt, timestamp, speed=0.0, heading=0.0):
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon

        meas_pos = self._gps_to_ned(lat, lon, alt)
        meas_vel = np.zeros(3)
        if speed > 0.1:
            heading_vec = heading_to_unit(heading)
            meas_vel[:2] = heading_vec * speed

        # Basit gecikme kompanzasyonu (ölçüm yaşı kadar ileri sar)
        latency = max(0.0, time.time() - timestamp)
        if latency > 0.0:
            meas_pos = meas_pos + meas_vel * latency

        if self.last_timestamp is None:
            self.filter.x[:3] = meas_pos
            self.filter.x[3:6] = meas_vel
            self.last_timestamp = timestamp
            self.pos = meas_pos
            self.vel = meas_vel
            self.acc = np.zeros(3)
            self.omega = 0.0
            self.buffer.append((meas_pos[0], meas_pos[1], meas_pos[2], timestamp, speed, heading))
            return

        dt = max(1e-3, timestamp - self.last_timestamp)
        self.last_timestamp = timestamp

        self.filter.predict(dt)
        meas_var_vel = 8.0 if speed < 1.0 else 3.0
        z = np.array([meas_pos[0], meas_pos[1], meas_pos[2], meas_vel[0], meas_vel[1], meas_vel[2]])
        self.filter.update(z, meas_var_vel=meas_var_vel, meas_var_vz=4.0)

        self.pos = self.filter.x[:3].copy()
        self.vel = self.filter.x[3:6].copy()
        self.omega = float(self.filter.x[6])
        self.acc = np.array([-self.omega * self.vel[1], self.omega * self.vel[0], 0.0])

        self.buffer.append((meas_pos[0], meas_pos[1], meas_pos[2], timestamp, speed, heading))

    def predict_lead(self, t_lead: float) -> np.ndarray:
        return self.filter._process_model(self.filter.x.copy(), t_lead)[:3]

    def get_velocity_magnitude(self):
        return float(np.linalg.norm(self.vel[:2]))

    def get_heading_vector(self):
        v_mag = self.get_velocity_magnitude()
        if v_mag < 0.5:
            return np.array([1.0, 0.0])
        return self.vel[:2] / v_mag


class MissionManager:
    """SEARCH -> APPROACH -> LOCK -> VISION_READY histerezisli."""

    def __init__(self):
        self.state = "SEARCH"
        self.vision_ready_counter = 0
        self.vision_exit_counter = 0
        self.vision_entry_frames = 5
        self.vision_exit_frames = 25
        self.frozen_target_id: Optional[str] = None

    def update(self, distance, angle_error, relative_vel, stale=False):
        vision_conditions = (
            distance <= D_VISION
            and abs(angle_error) <= 15.0
            and relative_vel <= 3.0
            and not stale
        )

        if self.state == "SEARCH":
            if distance < 200:
                self.state = "APPROACH"

        elif self.state == "APPROACH":
            if distance < 70:
                self.state = "LOCK"

        elif self.state == "LOCK":
            if vision_conditions:
                self.vision_ready_counter += 1
                if self.vision_ready_counter >= self.vision_entry_frames:
                    self.state = "VISION_READY"
                    self.vision_ready_counter = 0
            else:
                self.vision_ready_counter = 0

        elif self.state == "VISION_READY":
            if not vision_conditions:
                self.vision_exit_counter += 1
                if self.vision_exit_counter >= self.vision_exit_frames:
                    self.state = "LOCK"
                    self.vision_exit_counter = 0
                    self.frozen_target_id = None
            else:
                self.vision_exit_counter = 0

        return self.state

    def freeze_target(self, target_id: Optional[str]):
        self.frozen_target_id = target_id


@dataclass
class TargetTrack:
    estimator: TargetEstimator
    buffer: TelemetryBuffer
    last_latency: float = 0.0


class TargetSelector:
    """Skor + histerezis + kilit süresi."""

    def __init__(self, weights=None, switch_margin=SWITCH_MARGIN, lock_min_time=LOCK_MIN_TIME, lost_timeout=LOST_TIMEOUT):
        self.weights = weights or SCORE_WEIGHTS
        self.switch_margin = switch_margin
        self.lock_min_time = lock_min_time
        self.lost_timeout = lost_timeout
        self.current_target_id: Optional[str] = None
        self.lock_time: float = 0.0
        self.frozen_target_id: Optional[str] = None
        self.loyalty_bonus = 0.08

    def set_frozen(self, target_id: Optional[str]):
        self.frozen_target_id = target_id

    def _metrics(self, track_id: str, track: TargetTrack, own_state: dict, now: float):
        est = track.estimator
        if est.origin_lat is None:
            return None
        own_pos = est.gps_to_ned(own_state["lat"], own_state["lon"], own_state["alt"])
        target_pos = est.pos
        delta = target_pos - own_pos
        distance = float(np.linalg.norm(delta[:2]))
        alt_err = float(delta[2])

        heading_vec = est.get_heading_vector()
        if np.linalg.norm(delta[:2]) > 0.1:
            to_follower = own_pos[:2] - target_pos[:2]
            to_follower_norm = to_follower / np.linalg.norm(to_follower)
            angle_error = math.degrees(math.acos(np.clip(np.dot(-heading_vec, to_follower_norm), -1, 1)))
        else:
            angle_error = 0.0

        target_vel = est.vel
        rel_vel_vec = target_vel - own_state["vel"]
        rel_speed = float(np.linalg.norm(rel_vel_vec[:2]))

        risk = 0.0
        if distance < D_SAFE:
            risk = (D_SAFE - distance) / max(D_SAFE, 1.0)

        age = track.buffer.age(now)
        if age > self.lost_timeout:
            return None

        stability = track.buffer.stability_penalty(now)
        latency_penalty = max(0.0, track.last_latency - 0.2) * 3.0

        return {
            "distance": distance,
            "angle_error": angle_error,
            "rel_speed": rel_speed,
            "altitude_err": abs(alt_err),
            "risk": risk,
            "stability_penalty": stability,
            "latency_penalty": latency_penalty,
            "rel_vel_vec": rel_vel_vec,
            "own_pos_ned": own_pos,
            "target_pos_ned": target_pos,
        }

    def _score(self, m):
        w = self.weights
        return (
            w["distance"] * m["distance"]
            + w["angle"] * abs(m["angle_error"])
            + w["rel_speed"] * m["rel_speed"]
            + w["altitude"] * m["altitude_err"]
            + w["risk"] * m["risk"]
            + w["stability"] * m["stability_penalty"]
            + w["latency"] * m["latency_penalty"]
        )

    def select(self, tracks: Dict[str, TargetTrack], own_state: dict, now: float):
        metrics_map = {}
        scores = {}

        for tid, track in tracks.items():
            m = self._metrics(tid, track, own_state, now)
            if m is None:
                continue
            score = self._score(m)
            if tid == self.current_target_id:
                score *= (1 - self.loyalty_bonus)
            metrics_map[tid] = m
            scores[tid] = score

        if not scores:
            return None, None

        if self.frozen_target_id and self.frozen_target_id in scores:
            tid = self.frozen_target_id
            return tid, metrics_map[tid]

        best_id = min(scores, key=scores.get)

        if (
            self.current_target_id
            and self.current_target_id in scores
            and (now - self.lock_time) < self.lock_min_time
        ):
            return self.current_target_id, metrics_map[self.current_target_id]

        if (
            self.current_target_id
            and self.current_target_id in scores
            and best_id != self.current_target_id
        ):
            current_score = scores[self.current_target_id]
            if scores[best_id] < current_score * (1 - self.switch_margin):
                self.current_target_id = best_id
                self.lock_time = now
            else:
                best_id = self.current_target_id
        else:
            self.current_target_id = best_id
            self.lock_time = now

        return best_id, metrics_map.get(best_id)


def _vehicle_velocity(vehicle):
    if getattr(vehicle, "velocity", None):
        vx, vy, vz = vehicle.velocity
        return np.array([vx, vy, vz])
    speed = vehicle.groundspeed or 0.0
    heading = getattr(vehicle, "heading", 0.0) or 0.0
    vxy = heading_to_unit(heading) * speed
    return np.array([vxy[0], vxy[1], 0.0])


def main():
    print("=" * 70)
    print("DOGFIGHT TAKIP SISTEMI (GPS Sunucu + Direkt Takipçi)")
    print("=" * 70)

    print(f"\n[1/2] Takipci baglan: {TAKIPCI_CONNECTION}")
    takipci = connect(TAKIPCI_CONNECTION, wait_ready=True)
    print("      -> Takipci baglantisi OK")

    # COMMAND_ACK listener: ArduPlane komutları kabul ediyor mu?
    @takipci.on_message('COMMAND_ACK')
    def _listener(_self, _name, message):
        cmd_id = message.command
        result = message.result
        # MAV_RESULT: 0=Accepted, 1=Temp rejected, 2=Denied, 3=Unsupported, 4=Failed
        if result == 0:
            cmd_name = "UNKNOWN"
            if cmd_id == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
                cmd_name = "DO_CHANGE_SPEED"
            elif cmd_id == mavutil.mavlink.MAV_CMD_DO_CHANGE_ALT:
                cmd_name = "DO_CHANGE_ALT"
            else:
                return  # Diğer komutları ignore et
            print(f"[ACK] ✓ {cmd_name} kabul edildi")
        elif result != 0:
            print(f"[ACK] ✗ Komut {cmd_id} REDDEDILDI (result={result})")

    print("      -> COMMAND_ACK dinleyicisi aktif")
    print("      -> Hiz limitleri guncelleniyor (airspeed/groundspeed)")
    set_speed_limits(takipci)

    print(f"\n[2/2] GPS Sunucu:     {GPS_SUNUCU_IP}:{GPS_SUNUCU_PORT}")
    gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("      -> GPS sunucu socket olusturuldu")

    guidance = GuidanceLaw()
    mission = MissionManager()
    selector = TargetSelector()
    tracks: Dict[str, TargetTrack] = {}

    print("\n--- TAKIP BASLADI ---")
    print("Lead + trail + koridor + hiz/irtifa esleme + UKF kestirim aktif.\n")

    try:
        sayac = 0
        while True:
            sayac += 1
            now = time.time()
            ensure_guided(takipci)

            takipci_loc = takipci.location.global_relative_frame
            own_state = {
                "lat": takipci_loc.lat,
                "lon": takipci_loc.lon,
                "alt": takipci_loc.alt,
                "vel": _vehicle_velocity(takipci),
                "speed": takipci.groundspeed or 0.0,
                "airspeed": getattr(takipci, "airspeed", 0.0) or 0.0,
                "timestamp": now,
            }

            # GPS sunucudan hedef GPS verisi al
            hedef_gps = hedef_gps_al(gps_sock)

            if hedef_gps is None or hedef_gps.get("lat", 0) == 0:
                print("[WARN] Hedef GPS alinamadi, bekleniyor...")
                time.sleep(GUNCELLEME_SURESI)
                continue

            hedef_lat = hedef_gps.get("lat", 0.0)
            hedef_lon = hedef_gps.get("lon", 0.0)
            hedef_alt = hedef_gps.get("alt", 0.0)
            hedef_speed = hedef_gps.get("speed", 0.0)
            hedef_heading = hedef_gps.get("heading", 0)

            target_id = "hedef"
            if target_id not in tracks:
                tracks[target_id] = TargetTrack(estimator=TargetEstimator(), buffer=TelemetryBuffer())

            track = tracks[target_id]
            track.estimator.update(
                hedef_lat,
                hedef_lon,
                hedef_alt,
                now,
                hedef_speed,
                hedef_heading,
            )
            track.buffer.add(track.estimator.pos, now)
            track.last_latency = 0.0

            selected_id, metrics = selector.select(tracks, own_state, now)
            if selected_id is None or metrics is None:
                print("[WARN] Hedef yok/telemetri yok, bekleniyor...")
                time.sleep(GUNCELLEME_SURESI)
                continue

            est = tracks[selected_id].estimator

            # HİBRİT TAKİP STRATEJİSİ:
            # - UZAKTA: Lead prediction kullan (gelecekteki konuma git, yakalamak için)
            # - YAKINDA: Current position kullan (arkadan kilitle)
            rel_speed_mag = np.linalg.norm(metrics["rel_vel_vec"][:2])

            # Mod takibi için (sadece mod değişiminde print)
            if not hasattr(mission, 'last_mode'):
                mission.last_mode = None

            if metrics["distance"] > guidance.d_catch:
                # UZAKTA: Lead prediction ile INTERCEPT yap
                # 1 Hz update rate için ÇOK AGRESIF lead time (daire takibinde gerekli)
                if metrics["distance"] > 150.0:
                    t_lead = 8.0  # 8 saniye ileri bak (uzak mesafe)
                elif metrics["distance"] > 100.0:
                    t_lead = 6.0  # 6 saniye ileri bak
                elif metrics["distance"] > 50.0:
                    t_lead = 4.0  # 4 saniye ileri bak
                else:
                    t_lead = 3.0  # 3 saniye ileri bak (LOCK'a yakın)

                trail_point_ned = est.predict_lead(t_lead)
                behind = False  # Intercept modunda, önden yaklaşabilir

                if mission.last_mode != 'INTERCEPT':
                    print(f"[MOD DEĞİŞİMİ] INTERCEPT moduna geçildi (mesafe: {metrics['distance']:.1f}m)")
                    mission.last_mode = 'INTERCEPT'
            else:
                # YAKINDA: Arkadan kilitle (current position)
                trail_point_ned = est.pos.copy()
                behind = True  # Arkadan kilitlenme

                if mission.last_mode != 'LOCK':
                    print(f"[MOD DEĞİŞİMİ] LOCK moduna geçildi! (mesafe: {metrics['distance']:.1f}m)")
                    mission.last_mode = 'LOCK'

            # Çarpışmayı önle (çok yakınsa itme kuvveti ekle)
            if metrics["distance"] < 80.0:
                rep = guidance.repulsive(metrics["own_pos_ned"], est.pos)
                trail_point_ned[:2] += rep

            # CLOSING RATE: Hedef uzaklaşıyor mu, yaklaşıyor mu?
            # Negatif = uzaklaşıyor (kötü!), Pozitif = yaklaşıyor (iyi!)
            delta_vec = metrics["target_pos_ned"] - metrics["own_pos_ned"]
            range_unit = delta_vec[:2] / max(np.linalg.norm(delta_vec[:2]), 0.1)
            closing_rate = -np.dot(metrics["rel_vel_vec"][:2], range_unit)

            trail_lat, trail_lon, trail_alt = est.ned_to_gps(trail_point_ned)
            target_alt_cmd = guidance.compute_altitude_command(trail_alt, own_state["alt"])

            # AKILLI HIZ KOMUTU: mesafeye ve hedef hızına göre agresif yakalama
            target_speed = est.get_velocity_magnitude()

            # INTERCEPT modunda dönüş kaybı telafisi (+3 m/s)
            # Gerçek hız ~30 m/s olacak, bu sadece dönüş kaybını telafi eder
            turn_compensation = 3.0 if metrics["distance"] > guidance.d_catch else 0.0

            # Mesafe bazlı hız stratejisi
            if metrics["distance"] > 100.0:
                # Çok uzak: maksimum hız + telafi
                speed_cmd = guidance.v_max + turn_compensation
            elif metrics["distance"] > 50.0:
                # Orta mesafe: hedef hız + agresif boost + telafi
                speed_cmd = clamp(target_speed + 12.0 + turn_compensation, guidance.v_min + 10.0, guidance.v_max + turn_compensation)
            elif metrics["distance"] > guidance.d_catch:
                # Yakalama mesafesine yakın: hedef hız + boost + telafi
                speed_cmd = clamp(target_speed + 5.0 + turn_compensation, guidance.v_min + 5.0, guidance.v_max + turn_compensation)
            else:
                # LOCK modu: hedef hızı eşle (telafi YOK)
                speed_cmd = clamp(target_speed + 0.5, guidance.v_min, guidance.v_max)

            # HYBRID YAKLAŞIM: simple_goto (pozisyon) + DO_CHANGE_SPEED (hız) + DO_CHANGE_ALT (irtifa)
            # Bu ArduPlane'de EN GÜVENİLİR yöntemdir!

            # 1. Waypoint gönder (simple_goto - ArduPlane'in native komutu)
            waypoint = LocationGlobalRelative(trail_lat, trail_lon, target_alt_cmd)
            takipci.simple_goto(waypoint)

            # 2. İrtifa komutunu AYRI gönder (ArduPlane irtifa için bunu daha iyi algılıyor!)
            send_altitude_command(takipci, target_alt_cmd)

            # 3. Hız komutunu agresif şekilde gönder (HER DÖNGÜDE!)
            # ArduPlane bazen tek seferlik komutu kaçırıyor, her döngüde gönderiyoruz
            send_speed_command(takipci, speed_cmd, airspeed=True, also_ground=True)

            # 4. Ekstra: airspeed property'sini de set et (ek güvenlik)
            try:
                takipci.airspeed = speed_cmd
            except Exception:
                pass

            # Burnu trail noktas?na ?evir; hedef sert d?nd???nde LOS'a bak?p h?zl? tepki ver
            vec_to_trail = trail_point_ned[:2] - metrics["own_pos_ned"][:2]
            vec_to_target = est.pos[:2] - metrics["own_pos_ned"][:2]
            yaw_deg = None
            yaw_rate = 20
            if np.linalg.norm(vec_to_target) > 1.0:
                los_yaw = math.degrees(math.atan2(vec_to_target[1], vec_to_target[0])) % 360
            else:
                los_yaw = None
            if np.linalg.norm(vec_to_trail) > 1.0:
                trail_yaw = math.degrees(math.atan2(vec_to_trail[1], vec_to_trail[0])) % 360
            else:
                trail_yaw = None

            if los_yaw is not None and (abs(est.omega) > 0.05 or metrics["distance"] < (guidance.d_safe + 5.0)):
                yaw_deg = los_yaw
                yaw_rate = 25
            elif trail_yaw is not None:
                yaw_deg = trail_yaw

            if yaw_deg is not None:
                send_yaw_command(takipci, yaw_deg, yaw_rate=yaw_rate)

            stale = tracks[selected_id].buffer.age(now) > LOST_TIMEOUT
            state = mission.update(metrics["distance"], metrics["angle_error"], rel_speed_mag, stale=stale)
            if state == "VISION_READY":
                mission.freeze_target(selected_id)
                selector.set_frozen(selected_id)
            else:
                mission.freeze_target(None)
                selector.set_frozen(None)

            # Mod belirleme
            mode = "YAKALAMA" if metrics["distance"] > guidance.d_catch else "SABİT_KAL"
            closing_status = "UZAKLAŞ" if closing_rate < -0.5 else "YAKLAŞ" if closing_rate > 0.5 else "SABIT"

            print(
                f"[{sayac:03d}] {state:12s} | {mode:9s} | "
                f"Mesafe {metrics['distance']:5.1f}m | "
                f"CloseRate {closing_rate:+5.1f}m/s {closing_status:7s} | "
                f"HizCmd {speed_cmd:5.1f}m/s | "
                f"HedefHz {est.get_velocity_magnitude():5.1f}m/s | "
                f"TakipHz {own_state['airspeed']:4.1f}/{own_state['speed']:4.1f}m/s | "
                f"Aci {metrics['angle_error']:4.1f}° | "
                f"Behind {'Y' if behind else 'N'}"
            )

            if sayac % 10 == 0:
                print(
                    f"      Target GPS: ({hedef_lat:.6f}, {hedef_lon:.6f}, {hedef_alt:.1f}m) | "
                    f"Trail GPS: ({trail_lat:.6f}, {trail_lon:.6f}, {target_alt_cmd:.1f}m) | "
                    f"Own GPS: ({takipci_loc.lat:.6f}, {takipci_loc.lon:.6f}, {takipci_loc.alt:.1f}m) | "
                    f"Vision {'READY' if state == 'VISION_READY' else 'GPS'}"
                )

            time.sleep(GUNCELLEME_SURESI)

    except KeyboardInterrupt:
        print("\nKullanici durdurdu.")
    except Exception as e:
        print(f"\nHata: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nBaglantilar kapatiliyor...")
        try:
            takipci.close()
        except:
            pass
        try:
            gps_sock.close()
        except:
            pass
        print("Bitti.")


if __name__ == "__main__":
    main()
