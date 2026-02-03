#!/usr/bin/env python3
"""
================================================================================
L1 ADAPTIVE GUIDANCE LIBRARY
================================================================================
TEKNOFEST Savaşan İHA - Saf Python L1 Guidance Kütüphanesi

Bu modül ROS2'den tamamen bağımsızdır ve sadece saf Python + NumPy kullanır.
"Separation of Concerns" prensibi ile ayrılmıştır.

İçerikler:
1. Veri Yapıları (AircraftState, TargetScore, GuidanceCommand)
2. Koordinat Dönüşümleri (geodetic_to_ned, bearing, distance)
3. Filtreler (LowPassFilter, RateLimiter)
4. Hedef Seçimi (WeightedTargetSelector)
5. L1 Guidance (L1GuidanceController)
6. Durum Makinesi (TrackingStateMachine)

Kullanım:
    from teknofest_control.l1_guidance import (
        L1GuidanceController,
        WeightedTargetSelector,
        TrackingStateMachine,
        AircraftState,
        GuidanceCommand
    )

Yazar: HAVK Takımı
Tarih: 2026
Lisans: Apache-2.0
================================================================================
"""

import numpy as np
import math
import time
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict


# ============================================================================
# SİSTEM DURUM MAKİNESİ (SimpleStateMachine)
# ============================================================================
# Bu bölüm ROS2'den tamamen bağımsızdır.
# L1 Guidance testleri için geçici bir state machine yapısıdır.
# ============================================================================

class SystemState(Enum):
    """
    Basit Sistem Durumları
    L1 Guidance testleri için kullanılır
    """
    GROUND_IDLE = 0         # Yerde bekleme
    TAKEOFF_CLIMB = 1       # Kalkış/Tırmanma
    SAFE_LOITER = 2         # Güvenli Daire Çizme
    ACTIVE_PURSUIT = 3      # Aktif L1 Takibi


class SimpleStateMachine:
    """
    Basit Durum Makinesi - L1 Guidance Testleri İçin
    
    Bu sınıf sadece temel irtifa ve hedef kontrolü yapar.
    ROS2 kullanmaz, sadece saf Python.
    
    Kullanım:
        sm = SimpleStateMachine(min_safe_alt=30.0, data_timeout=1.0)
        state = sm.update(current_alt=50.0, target_list=[target1, target2])
        print(f"Mevcut durum: {state.name}")
    """
    
    def __init__(self, min_safe_alt: float = 30.0, data_timeout: float = 1.0):
        """
        Args:
            min_safe_alt: Minimum güvenli irtifa (metre). Bu irtifanın altında
                         takip yapılmaz.
            data_timeout: Hedef verisi zaman aşımı (saniye). Bu süre boyunca
                         hedef güncellemesi gelmezse hedef kayıp sayılır.
        """
        self.min_safe_alt = min_safe_alt
        self.data_timeout = data_timeout
        self.current_state = SystemState.GROUND_IDLE
        self.last_target_time = 0.0
        self.ground_threshold = 5.0  # 5 metre altı = yerde
        
        # Durum değişikliği geçmişi (debug için)
        self.state_history: List[Tuple[float, SystemState]] = []
        self.max_history = 100
    
    def update(self, current_alt: float, target_list: List) -> SystemState:
        """
        Durumu güncelle
        
        Args:
            current_alt: Mevcut irtifa (metre, AGL veya MSL)
            target_list: Hedef listesi (boş liste = hedef yok)
        
        Returns:
            SystemState: Yeni sistem durumu
        """
        previous_state = self.current_state
        current_time = time.time()
        
        # Hedef listesi kontrolü
        has_targets = len(target_list) > 0 if target_list else False
        
        # Son hedef zamanını güncelle
        if has_targets:
            self.last_target_time = current_time
        
        # Hedef zaman aşımı kontrolü
        target_timed_out = (current_time - self.last_target_time) > self.data_timeout
        
        # Durum geçiş mantığı
        if current_alt < self.ground_threshold:
            # Yerde
            self.current_state = SystemState.GROUND_IDLE
            
        elif current_alt < self.min_safe_alt:
            # Tırmanma aşamasında (güvenli irtifaya ulaşmadı)
            self.current_state = SystemState.TAKEOFF_CLIMB
            
        elif has_targets and not target_timed_out:
            # Güvenli irtifada ve hedef var
            self.current_state = SystemState.ACTIVE_PURSUIT
            
        else:
            # Güvenli irtifada ama hedef yok veya zaman aşımı
            self.current_state = SystemState.SAFE_LOITER
        
        # Durum değişikliği kaydı
        if self.current_state != previous_state:
            self._log_state_change(current_time, previous_state, self.current_state)
        
        return self.current_state
    
    def _log_state_change(self, timestamp: float, old_state: SystemState, 
                          new_state: SystemState) -> None:
        """Durum değişikliğini kaydet"""
        self.state_history.append((timestamp, new_state))
        
        # Geçmiş boyutunu sınırla
        if len(self.state_history) > self.max_history:
            self.state_history = self.state_history[-self.max_history:]
    
    def get_state_name(self) -> str:
        """Mevcut durumun adını döndür"""
        return self.current_state.name
    
    def is_safe_to_pursue(self) -> bool:
        """Takip için güvenli mi?"""
        return self.current_state == SystemState.ACTIVE_PURSUIT
    
    def is_on_ground(self) -> bool:
        """Yerde mi?"""
        return self.current_state == SystemState.GROUND_IDLE
    
    def reset(self) -> None:
        """Durum makinesini sıfırla"""
        self.current_state = SystemState.GROUND_IDLE
        self.last_target_time = 0.0
        self.state_history.clear()


# ============================================================================
# ENUMlar
# ============================================================================

class TrackingState(Enum):
    """
    Takip Durum Makinesi Durumları
    Her durum farklı bir kontrol stratejisi uygular
    """
    IDLE = 0                # Bekleme - hedef yok
    SEARCHING = 1           # Hedef arıyor
    APPROACHING = 2         # Hedefe yaklaşıyor (uzak mesafe)
    PURSUING = 3            # Aktif takip (orta mesafe)
    LOCKED = 4              # Kilitlenmiş (yakın mesafe, stabil takip)
    LOITERING = 5           # Çok yakın - etrafında dönüyor
    EVADING = 6             # Kaçınma manevrası
    LOST_TARGET = 7         # Hedef kayboldu


# ============================================================================
# VERİ YAPILARI
# ============================================================================

@dataclass
class AircraftState:
    """
    Uçak durumu veri yapısı
    Hem kendi uçağımız hem de rakipler için kullanılır
    """
    id: int = 0
    latitude: float = 0.0       # Enlem (derece)
    longitude: float = 0.0      # Boylam (derece)
    altitude: float = 0.0       # İrtifa (metre, MSL)
    ground_speed: float = 0.0   # Yer hızı (m/s)
    airspeed: float = 0.0       # Hava hızı (m/s)
    heading: float = 0.0        # Başlık açısı (derece, 0-360)
    climb_rate: float = 0.0     # Tırmanma hızı (m/s)
    roll: float = 0.0           # Roll açısı (derece)
    pitch: float = 0.0          # Pitch açısı (derece)
    timestamp: float = 0.0      # Zaman damgası
    
    # NED koordinatları (Local frame)
    x: float = 0.0              # Kuzey (metre)
    y: float = 0.0              # Doğu (metre)
    z: float = 0.0              # Aşağı (metre, negatif = yukarı)
    
    # Hız vektörü (NED)
    vx: float = 0.0             # Kuzey hızı (m/s)
    vy: float = 0.0             # Doğu hızı (m/s)
    vz: float = 0.0             # Aşağı hızı (m/s)
    
    def get_velocity_vector(self) -> np.ndarray:
        """Hız vektörünü numpy array olarak döndür"""
        return np.array([self.vx, self.vy, self.vz])
    
    def get_position_ned(self) -> np.ndarray:
        """NED pozisyonunu numpy array olarak döndür"""
        return np.array([self.x, self.y, self.z])
    
    def get_speed_2d(self) -> float:
        """2D yatay hızı hesapla"""
        return math.sqrt(self.vx**2 + self.vy**2)


@dataclass
class TargetScore:
    """
    Hedef puanlama sonucu
    """
    target_id: int
    total_score: float
    distance_score: float
    angle_score: float
    speed_score: float
    is_tail_position: bool      # Kuyruk pozisyonunda mıyız?
    is_head_on: bool            # Kafa kafaya mı geliyor?
    distance: float             # Metre cinsinden mesafe
    bearing: float              # Hedefe olan açı (derece)
    aspect_angle: float         # Görüş açısı (hedefin bize bakış açısı)


@dataclass 
class GuidanceCommand:
    """
    L1 Guidance çıktısı - Kontrol komutları
    """
    velocity_north: float = 0.0     # Kuzey hızı komutu (m/s)
    velocity_east: float = 0.0      # Doğu hızı komutu (m/s)
    velocity_down: float = 0.0      # Aşağı hızı komutu (m/s)
    yaw_setpoint: float = 0.0       # Yaw açısı (radyan)
    yaw_rate: float = 0.0           # Yaw hızı (rad/s)
    airspeed_setpoint: float = 0.0  # Hava hızı komutu (m/s)
    is_valid: bool = False


@dataclass
class L1GuidanceParams:
    """L1 Guidance parametreleri"""
    l1_distance: float = 50.0       # L1 mesafesi (metre)
    l1_damping: float = 0.85        # Sönümleme faktörü
    l1_period: float = 25.0         # L1 periyodu
    adaptive_l1: bool = True        # Adaptif L1 mesafesi
    min_airspeed: float = 15.0      # Minimum hava hızı (m/s)
    max_airspeed: float = 35.0      # Maksimum hava hızı (m/s)
    cruise_airspeed: float = 22.0   # Seyir hava hızı (m/s)
    max_bank_angle: float = 45.0    # Maksimum bank açısı (derece)
    loiter_radius: float = 40.0     # Loiter yarıçapı (metre)


@dataclass
class TargetSelectorParams:
    """Hedef seçim parametreleri"""
    w_distance: float = 0.35        # Mesafe ağırlığı
    w_angle: float = 0.45           # Açı ağırlığı
    w_speed: float = 0.20           # Hız ağırlığı
    tail_bonus: float = 2.5         # Kuyruk pozisyonu bonusu
    head_on_penalty: float = 0.2    # Kafa kafaya cezası
    tail_cone: float = 45.0         # Kuyruk konisi yarı açısı (derece)
    head_on_cone: float = 30.0      # Kafa kafaya konisi yarı açısı (derece)


@dataclass
class StateMachineParams:
    """Durum makinesi parametreleri"""
    lock_distance: float = 80.0         # Kilit mesafesi (metre)
    approach_distance: float = 150.0    # Yaklaşma mesafesi (metre)
    loiter_trigger: float = 25.0        # Loiter tetikleme mesafesi (metre)
    lock_confirm_time: float = 4.0      # Kilit onay süresi (saniye)
    target_timeout: float = 5.0         # Hedef zaman aşımı (saniye)


# ============================================================================
# MATEMATİKSEL YARDIMCI FONKSİYONLAR
# ============================================================================

def geodetic_to_ned(lat: float, lon: float, alt: float,
                    ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    GPS koordinatlarını NED (North-East-Down) koordinatlarına çevir
    
    Args:
        lat, lon, alt: Hedef koordinatları (derece, derece, metre)
        ref_lat, ref_lon, ref_alt: Referans noktası (kendi konumumuz)
    
    Returns:
        (north, east, down): Metre cinsinden NED koordinatları
    """
    EARTH_RADIUS = 6371000.0  # metre
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    
    d_lat = lat_rad - ref_lat_rad
    d_lon = lon_rad - ref_lon_rad
    
    north = d_lat * EARTH_RADIUS
    east = d_lon * EARTH_RADIUS * math.cos(ref_lat_rad)
    down = ref_alt - alt
    
    return north, east, down


def ned_to_geodetic(north: float, east: float, down: float,
                    ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    NED koordinatlarını GPS koordinatlarına çevir
    
    Args:
        north, east, down: NED koordinatları (metre)
        ref_lat, ref_lon, ref_alt: Referans noktası
    
    Returns:
        (lat, lon, alt): Derece ve metre cinsinden GPS koordinatları
    """
    EARTH_RADIUS = 6371000.0
    
    ref_lat_rad = math.radians(ref_lat)
    
    d_lat = north / EARTH_RADIUS
    d_lon = east / (EARTH_RADIUS * math.cos(ref_lat_rad))
    
    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    alt = ref_alt - down
    
    return lat, lon, alt


def normalize_angle(angle: float) -> float:
    """
    Açıyı -180 ile +180 arasına normalize et (derece)
    """
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def normalize_angle_rad(angle: float) -> float:
    """
    Açıyı -π ile +π arasına normalize et (radyan)
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def wrap_to_2pi(angle: float) -> float:
    """
    Açıyı 0-2π arasına normalize et (radyan)
    """
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle


def wrap_to_360(angle: float) -> float:
    """
    Açıyı 0-360 arasına normalize et (derece)
    """
    while angle < 0:
        angle += 360.0
    while angle >= 360.0:
        angle -= 360.0
    return angle


def calculate_bearing(from_north: float, from_east: float,
                     to_north: float, to_east: float) -> float:
    """
    İki NED noktası arasındaki bearing (yön açısı) hesapla
    
    Returns:
        Derece cinsinden bearing (0-360, kuzey = 0, doğu = 90)
    """
    d_north = to_north - from_north
    d_east = to_east - from_east
    
    bearing = math.degrees(math.atan2(d_east, d_north))
    if bearing < 0:
        bearing += 360.0
    
    return bearing


def calculate_distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2D Öklid mesafesi hesapla"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def calculate_distance_3d(p1: np.ndarray, p2: np.ndarray) -> float:
    """3D Öklid mesafesi hesapla"""
    return float(np.linalg.norm(p2 - p1))


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    İki GPS koordinatı arasındaki mesafeyi Haversine formülü ile hesapla
    
    Returns:
        Mesafe (metre)
    """
    EARTH_RADIUS = 6371000.0
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    
    a = math.sin(d_lat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return EARTH_RADIUS * c


# ============================================================================
# FİLTRE SINIFLARI
# ============================================================================

class LowPassFilter:
    """
    Birinci dereceden Low-Pass Filter
    Ani değişimleri yumuşatmak için kullanılır
    """
    
    def __init__(self, cutoff_freq: float, sample_rate: float):
        """
        Args:
            cutoff_freq: Kesim frekansı (Hz)
            sample_rate: Örnekleme frekansı (Hz)
        """
        self.alpha = self._calculate_alpha(cutoff_freq, sample_rate)
        self.prev_output: Optional[float] = None
    
    def _calculate_alpha(self, cutoff_freq: float, sample_rate: float) -> float:
        """Alpha katsayısını hesapla"""
        dt = 1.0 / sample_rate
        rc = 1.0 / (2 * math.pi * cutoff_freq)
        return dt / (rc + dt)
    
    def filter(self, value: float) -> float:
        """
        Değeri filtrele
        
        Args:
            value: Ham girdi değeri
            
        Returns:
            Filtrelenmiş değer
        """
        if self.prev_output is None:
            self.prev_output = value
            return value
        
        output = self.alpha * value + (1 - self.alpha) * self.prev_output
        self.prev_output = output
        return output
    
    def reset(self, value: Optional[float] = None):
        """Filtreyi sıfırla"""
        self.prev_output = value


class RateLimiter:
    """
    Rate Limiter - değişim hızını sınırlar
    Ani manevraları önlemek için kritik
    """
    
    def __init__(self, max_rate: float, sample_rate: float):
        """
        Args:
            max_rate: Maksimum değişim hızı (birim/saniye)
            sample_rate: Örnekleme frekansı (Hz)
        """
        self.max_rate = max_rate
        self.dt = 1.0 / sample_rate
        self.max_change = max_rate * self.dt
        self.prev_output: Optional[float] = None
    
    def limit(self, value: float) -> float:
        """
        Değişim hızını sınırla
        
        Args:
            value: İstenen değer
            
        Returns:
            Hız sınırlı değer
        """
        if self.prev_output is None:
            self.prev_output = value
            return value
        
        change = value - self.prev_output
        
        if abs(change) > self.max_change:
            change = math.copysign(self.max_change, change)
        
        output = self.prev_output + change
        self.prev_output = output
        return output
    
    def reset(self, value: Optional[float] = None):
        """Limiter'ı sıfırla"""
        self.prev_output = value


class CommandSmoother:
    """
    Komut yumuşatıcı - birden fazla filtre kombinasyonu
    """
    
    def __init__(self, sample_rate: float, 
                 lpf_cutoff: float = 0.5,
                 heading_rate_limit: float = 15.0,
                 altitude_rate_limit: float = 3.0,
                 smoothing_alpha: float = 0.15):
        
        self.sample_rate = sample_rate
        self.smoothing_alpha = smoothing_alpha
        self.heading_rate_limit = heading_rate_limit
        
        # Filtreler
        self.heading_lpf = LowPassFilter(lpf_cutoff, sample_rate)
        self.heading_rate_limiter = RateLimiter(heading_rate_limit, sample_rate)
        self.altitude_lpf = LowPassFilter(0.3, sample_rate)
        self.altitude_rate_limiter = RateLimiter(altitude_rate_limit, sample_rate)
        self.vn_lpf = LowPassFilter(0.5, sample_rate)
        self.ve_lpf = LowPassFilter(0.5, sample_rate)
        self.vd_lpf = LowPassFilter(0.3, sample_rate)
        
        self.prev_command = GuidanceCommand()
    
    def smooth(self, command: GuidanceCommand) -> GuidanceCommand:
        """
        Komutu smooth et - ani manevraları önle
        """
        smoothed = GuidanceCommand(is_valid=True)
        
        # Velocity smoothing
        smoothed.velocity_north = self.vn_lpf.filter(command.velocity_north)
        smoothed.velocity_east = self.ve_lpf.filter(command.velocity_east)
        smoothed.velocity_down = self.vd_lpf.filter(command.velocity_down)
        
        # Heading smoothing
        current_yaw = command.yaw_setpoint
        prev_yaw = self.prev_command.yaw_setpoint
        
        yaw_diff = current_yaw - prev_yaw
        yaw_diff = normalize_angle_rad(yaw_diff)
        
        max_yaw_rate = math.radians(self.heading_rate_limit) / self.sample_rate
        yaw_diff = np.clip(yaw_diff, -max_yaw_rate, max_yaw_rate)
        
        smoothed.yaw_setpoint = prev_yaw + yaw_diff
        
        # Exponential smoothing
        alpha = self.smoothing_alpha
        smoothed.velocity_north = alpha * smoothed.velocity_north + \
                                  (1 - alpha) * self.prev_command.velocity_north
        smoothed.velocity_east = alpha * smoothed.velocity_east + \
                                 (1 - alpha) * self.prev_command.velocity_east
        
        smoothed.airspeed_setpoint = command.airspeed_setpoint
        
        self.prev_command = smoothed
        return smoothed
    
    def reset(self):
        """Tüm filtreleri sıfırla"""
        self.heading_lpf.reset()
        self.heading_rate_limiter.reset()
        self.altitude_lpf.reset()
        self.altitude_rate_limiter.reset()
        self.vn_lpf.reset()
        self.ve_lpf.reset()
        self.vd_lpf.reset()
        self.prev_command = GuidanceCommand()


# ============================================================================
# HEDEF SEÇİM ALGORİTMASI
# ============================================================================

class WeightedTargetSelector:
    """
    Ağırlıklı puanlama ile hedef seçim algoritması
    
    Formül:
    Score = (W1 * 1/Distance) + (W2 * Angle_Factor) + (W3 * Speed_Factor)
    """
    
    def __init__(self, params: Optional[TargetSelectorParams] = None):
        self.params = params or TargetSelectorParams()
    
    def select_best_target(self, own_state: AircraftState, 
                           targets: Dict[int, AircraftState]) -> Optional[TargetScore]:
        """
        Tüm hedefleri puanla ve en iyisini seç
        
        Args:
            own_state: Kendi uçağımızın durumu
            targets: Hedef uçak listesi (id -> AircraftState)
        
        Returns:
            En yüksek puanlı hedefin TargetScore objesi veya None
        """
        if not targets:
            return None
        
        own_pos = own_state.get_position_ned()
        own_heading = own_state.heading
        own_speed = own_state.get_speed_2d()
        
        scores: List[TargetScore] = []
        
        for _, target in targets.items():
            score = self.calculate_score(target, own_pos, own_heading, own_speed)
            scores.append(score)
        
        scores.sort(key=lambda x: x.total_score, reverse=True)
        
        return scores[0] if scores else None
    
    def calculate_score(self, target: AircraftState,
                        own_pos: np.ndarray,
                        _own_heading: float,
                        own_speed: float) -> TargetScore:
        """
        Tek bir hedef için ağırlıklı skor hesapla
        
        Args:
            target: Hedef uçak durumu
            own_pos: Kendi NED pozisyonumuz
            _own_heading: Kendi başlık açımız (şu an kullanılmıyor, ileride eklenebilir)
            own_speed: Kendi 2D hızımız
        """
        p = self.params
        target_pos = target.get_position_ned()
        
        # 1. MESAFE SKORU
        distance = calculate_distance_3d(own_pos, target_pos)
        distance = max(distance, 10.0)  # Sıfıra bölmeyi önle
        distance_score = 1.0 / (1.0 + (distance / 100.0))
        
        # 2. AÇI UYGUNLUĞU SKORU
        bearing = calculate_bearing(own_pos[0], own_pos[1], 
                                   target_pos[0], target_pos[1])
        
        reverse_bearing = calculate_bearing(target_pos[0], target_pos[1],
                                           own_pos[0], own_pos[1])
        
        aspect_angle = abs(normalize_angle(target.heading - reverse_bearing))
        
        is_tail = aspect_angle < p.tail_cone
        is_head_on = aspect_angle > (180 - p.head_on_cone)
        
        if is_tail:
            angle_score = p.tail_bonus
        elif is_head_on:
            angle_score = p.head_on_penalty
        else:
            angle_score = 1.0 - (abs(aspect_angle - 90) / 90.0)
        
        # 3. HIZ FAKTÖRÜ SKORU
        target_speed = target.ground_speed
        speed_diff = own_speed - target_speed
        
        if speed_diff > 0:
            speed_score = min(1.0 + speed_diff / 10.0, 2.0)
        else:
            speed_score = max(0.3, 1.0 + speed_diff / 20.0)
        
        # TOPLAM SKOR
        total_score = (
            p.w_distance * distance_score +
            p.w_angle * angle_score +
            p.w_speed * speed_score
        )
        
        return TargetScore(
            target_id=target.id,
            total_score=total_score,
            distance_score=distance_score,
            angle_score=angle_score,
            speed_score=speed_score,
            is_tail_position=is_tail,
            is_head_on=is_head_on,
            distance=distance,
            bearing=bearing,
            aspect_angle=aspect_angle
        )


# ============================================================================
# DURUM MAKİNESİ
# ============================================================================

class TrackingStateMachine:
    """
    Takip durum makinesi
    Mesafeye göre durumlar arası geçiş yapar
    """
    
    def __init__(self, params: Optional[StateMachineParams] = None):
        self.params = params or StateMachineParams()
        self.current_state = TrackingState.IDLE
        self.previous_state = TrackingState.IDLE
        self.lock_start_time: Optional[float] = None
        self.lock_confirmed: bool = False
        self.state_transitions: int = 0
    
    def update(self, distance: float, current_time: float, 
               has_target: bool) -> TrackingState:
        """
        Durumu güncelle
        
        Args:
            distance: Hedefe mesafe (metre)
            current_time: Mevcut zaman (saniye)
            has_target: Hedef var mı?
        
        Returns:
            Yeni durum
        """
        p = self.params
        
        if not has_target:
            if self.current_state not in [TrackingState.IDLE, TrackingState.SEARCHING]:
                self._change_state(TrackingState.SEARCHING)
            return self.current_state
        
        # Duruma göre geçişler
        if self.current_state == TrackingState.IDLE:
            self._change_state(TrackingState.SEARCHING)
        
        elif self.current_state == TrackingState.SEARCHING:
            if distance < p.approach_distance:
                self._change_state(TrackingState.APPROACHING)
        
        elif self.current_state == TrackingState.APPROACHING:
            if distance < p.lock_distance:
                self._change_state(TrackingState.PURSUING)
            elif distance > p.approach_distance * 1.2:
                self._change_state(TrackingState.SEARCHING)
        
        elif self.current_state == TrackingState.PURSUING:
            if distance < p.loiter_trigger:
                self._change_state(TrackingState.LOITERING)
            elif distance < p.lock_distance * 0.8:
                self._change_state(TrackingState.LOCKED)
                self.lock_start_time = current_time
            elif distance > p.lock_distance * 1.3:
                self._change_state(TrackingState.APPROACHING)
        
        elif self.current_state == TrackingState.LOCKED:
            if distance > p.lock_distance * 1.5:
                self._change_state(TrackingState.PURSUING)
                self.lock_start_time = None
                self.lock_confirmed = False
            elif distance < p.loiter_trigger:
                self._change_state(TrackingState.LOITERING)
            else:
                # Kilit onay kontrolü
                if self.lock_start_time is not None:
                    lock_duration = current_time - self.lock_start_time
                    if lock_duration >= p.lock_confirm_time and not self.lock_confirmed:
                        self.lock_confirmed = True
        
        elif self.current_state == TrackingState.LOITERING:
            if distance > p.loiter_trigger * 2:
                self._change_state(TrackingState.PURSUING)
        
        return self.current_state
    
    def _change_state(self, new_state: TrackingState):
        """Durum değiştir"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_transitions += 1
    
    def reset(self):
        """Durum makinesini sıfırla"""
        self.current_state = TrackingState.IDLE
        self.previous_state = TrackingState.IDLE
        self.lock_start_time = None
        self.lock_confirmed = False
    
    def get_lock_duration(self, current_time: float) -> float:
        """Mevcut kilit süresini döndür"""
        if self.lock_start_time is None:
            return 0.0
        return current_time - self.lock_start_time


# ============================================================================
# L1 GUIDANCE CONTROLLER
# ============================================================================

class L1GuidanceController:
    """
    L1 Adaptive Guidance Kontrolcüsü
    
    Bu algoritma:
    1. Hedefin arkasında sanal bir nokta hesaplar (L1 mesafesinde)
    2. Bu noktaya gitmek için gerekli hız vektörünü hesaplar
    3. Cross-track error'u minimize eder
    4. Rüzgar düzeltmesi (crab angle) uygular
    """
    
    def __init__(self, params: Optional[L1GuidanceParams] = None):
        self.params = params or L1GuidanceParams()
        
        # Internal state
        self.virtual_target = np.zeros(3)
        self.l1_reference_point = np.zeros(3)
        self.cross_track_error: float = 0.0
        self.along_track_error: float = 0.0
        self.crab_angle: float = 0.0
    
    def compute(self, own_state: AircraftState, 
                target: AircraftState,
                tracking_state: TrackingState) -> GuidanceCommand:
        """
        L1 Guidance komutunu hesapla
        
        Args:
            own_state: Kendi uçağımızın durumu
            target: Hedef uçak durumu
            tracking_state: Mevcut takip durumu
        
        Returns:
            GuidanceCommand objesi
        """
        p = self.params
        
        # Pozisyon ve hız vektörleri
        own_pos = own_state.get_position_ned()
        own_vel = own_state.get_velocity_vector()
        target_pos = target.get_position_ned()
        target_vel = target.get_velocity_vector()
        
        # Loiter modunda farklı komut
        if tracking_state == TrackingState.LOITERING:
            return self._generate_loiter_command(own_pos, target_pos)
        
        # 1. SANAL HEDEF NOKTASI HESAPLA
        target_speed = float(np.linalg.norm(target_vel[:2]))
        
        if target_speed > 0.5:
            target_heading_vec = target_vel[:2] / target_speed
            
            if p.adaptive_l1:
                l1_dist = p.l1_distance * (1 + target_speed / 30.0)
                l1_dist = float(np.clip(l1_dist, 30.0, 100.0))
            else:
                l1_dist = p.l1_distance
            
            virtual_target_2d = target_pos[:2] - l1_dist * target_heading_vec
            virtual_target_z = target_pos[2]
        else:
            virtual_target_2d = target_pos[:2]
            virtual_target_z = target_pos[2]
            l1_dist = p.l1_distance
        
        self.virtual_target = np.array([virtual_target_2d[0], 
                                        virtual_target_2d[1], 
                                        virtual_target_z])
        
        # 2. L1 REFERANS NOKTASI VE HATA HESABI
        los_vec = self.virtual_target - own_pos
        los_distance = float(np.linalg.norm(los_vec[:2]))
        los_distance = max(los_distance, 1.0)
        
        own_vel_norm = float(np.linalg.norm(own_vel[:2]))
        if own_vel_norm > 0.5:
            vel_unit = own_vel / np.linalg.norm(own_vel)
        else:
            vel_unit = np.array([1.0, 0.0, 0.0])
        
        self.l1_reference_point = own_pos + l1_dist * vel_unit
        
        # Cross-track error
        if own_vel_norm > 0.01:
            self.cross_track_error = float(np.cross(
                los_vec[:2], 
                own_vel[:2] / own_vel_norm
            ))
        else:
            self.cross_track_error = 0.0
        
        # 3. L1 LATERAL ACCELERATION VE HEADING HESABI
        own_speed = max(own_vel_norm, p.min_airspeed)
        
        los_angle = math.atan2(los_vec[1], los_vec[0])
        current_heading = math.radians(own_state.heading)
        
        eta = wrap_to_2pi(los_angle) - wrap_to_2pi(current_heading)
        eta = normalize_angle(math.degrees(eta))
        eta = math.radians(eta)
        
        l1_accel = 2 * (own_speed ** 2) / l1_dist * math.sin(eta)
        
        g = 9.81
        commanded_bank = math.atan(l1_accel / g)
        commanded_bank = float(np.clip(commanded_bank, 
                                 -math.radians(p.max_bank_angle), 
                                 math.radians(p.max_bank_angle)))
        
        # 4. RÜZGAR DÜZELTMESİ
        if own_state.airspeed > 0:
            wind_component = self.cross_track_error * 0.1
            self.crab_angle = math.atan2(wind_component, own_speed)
            self.crab_angle = float(np.clip(self.crab_angle, 
                                      -math.radians(20), 
                                      math.radians(20)))
        
        # 5. KOMUT OLUŞTUR
        desired_heading = los_angle + self.crab_angle
        
        # Hız büyüklüğü
        if los_distance > 100:
            speed_cmd = p.max_airspeed
        elif los_distance > 50:
            speed_cmd = p.cruise_airspeed + (p.max_airspeed - p.cruise_airspeed) * \
                       (los_distance - 50) / 50
        else:
            speed_cmd = p.cruise_airspeed
        
        if target_speed > 0:
            speed_cmd = max(speed_cmd, target_speed + 3.0)
        
        speed_cmd = float(np.clip(speed_cmd, p.min_airspeed, p.max_airspeed))
        
        command = GuidanceCommand(is_valid=True)
        command.velocity_north = speed_cmd * math.cos(desired_heading)
        command.velocity_east = speed_cmd * math.sin(desired_heading)
        
        altitude_error = target_pos[2] - own_pos[2]
        command.velocity_down = float(np.clip(-altitude_error * 0.5, -3.0, 3.0))
        
        command.yaw_setpoint = desired_heading
        command.airspeed_setpoint = speed_cmd
        
        return command
    
    def _generate_loiter_command(self, own_pos: np.ndarray, 
                                  center: np.ndarray) -> GuidanceCommand:
        """
        Loiter (dönme) komutu oluştur
        """
        p = self.params
        command = GuidanceCommand(is_valid=True)
        
        to_center = center[:2] - own_pos[:2]
        dist_to_center = float(np.linalg.norm(to_center))
        
        if dist_to_center > 0.1:
            radial_unit = to_center / dist_to_center
            tangent_unit = np.array([-radial_unit[1], radial_unit[0]])
        else:
            tangent_unit = np.array([1.0, 0.0])
            radial_unit = np.array([0.0, 1.0])
        
        radius_error = dist_to_center - p.loiter_radius
        
        tangent_speed = p.cruise_airspeed
        radial_correction = radius_error * 0.3
        
        velocity_2d = tangent_speed * tangent_unit + \
                      radial_correction * radial_unit
        
        command.velocity_north = float(velocity_2d[0])
        command.velocity_east = float(velocity_2d[1])
        command.velocity_down = 0.0
        command.yaw_setpoint = math.atan2(velocity_2d[1], velocity_2d[0])
        command.airspeed_setpoint = tangent_speed
        
        return command
    
    def get_virtual_target(self) -> np.ndarray:
        """Sanal hedef noktasını döndür"""
        return self.virtual_target.copy()
    
    def get_cross_track_error(self) -> float:
        """Cross-track error değerini döndür"""
        return self.cross_track_error


# ============================================================================
# ANA L1 GUIDANCE FACADE
# ============================================================================

class L1Guidance:
    """
    L1 Guidance Facade - Tüm bileşenleri birleştirir
    
    Kullanım:
        guidance = L1Guidance()
        
        # Her kontrol döngüsünde:
        command = guidance.update(own_state, targets, current_time)
        if command.is_valid:
            # Komutu uygula
    """
    
    def __init__(self,
                 l1_params: Optional[L1GuidanceParams] = None,
                 selector_params: Optional[TargetSelectorParams] = None,
                 state_params: Optional[StateMachineParams] = None,
                 sample_rate: float = 50.0):
        
        self.l1_controller = L1GuidanceController(l1_params)
        self.target_selector = WeightedTargetSelector(selector_params)
        self.state_machine = TrackingStateMachine(state_params)
        self.command_smoother = CommandSmoother(sample_rate)
        
        self.locked_target_id: int = -1
        self.locked_target: Optional[AircraftState] = None
        self.target_switch_time: float = 0.0
        self.target_switch_cooldown: float = 3.0
    
    def update(self, own_state: AircraftState,
               targets: Dict[int, AircraftState],
               current_time: float) -> GuidanceCommand:
        """
        Ana güncelleme fonksiyonu
        
        Args:
            own_state: Kendi uçağımızın durumu
            targets: Hedef listesi
            current_time: Mevcut zaman (saniye)
        
        Returns:
            GuidanceCommand objesi
        """
        # 1. Hedef seçimi
        if targets:
            best_target = self.target_selector.select_best_target(own_state, targets)
            
            if best_target is not None:
                if self.locked_target_id != best_target.target_id:
                    if current_time - self.target_switch_time > self.target_switch_cooldown:
                        self._switch_target(best_target.target_id, current_time)
                
                self.locked_target = targets.get(self.locked_target_id)
        else:
            self.locked_target = None
        
        # 2. Durum makinesi güncelle
        if self.locked_target is not None:
            own_pos = own_state.get_position_ned()
            target_pos = self.locked_target.get_position_ned()
            distance = calculate_distance_3d(own_pos, target_pos)
        else:
            distance = float('inf')
        
        self.state_machine.update(distance, current_time, self.locked_target is not None)
        
        # 3. L1 Guidance hesapla
        if self.locked_target is not None and \
           self.state_machine.current_state != TrackingState.IDLE:
            
            command = self.l1_controller.compute(
                own_state, 
                self.locked_target,
                self.state_machine.current_state
            )
            
            # 4. Komutu smooth et
            smoothed_command = self.command_smoother.smooth(command)
            return smoothed_command
        
        return GuidanceCommand(is_valid=False)
    
    def _switch_target(self, new_target_id: int, current_time: float):
        """Hedef değiştir"""
        self.locked_target_id = new_target_id
        self.target_switch_time = current_time
        self.state_machine.lock_start_time = None
        self.state_machine.lock_confirmed = False
        self.command_smoother.reset()
    
    def get_state(self) -> TrackingState:
        """Mevcut durumu döndür"""
        return self.state_machine.current_state
    
    def get_locked_target_id(self) -> int:
        """Kilitli hedef ID'sini döndür"""
        return self.locked_target_id
    
    def is_lock_confirmed(self) -> bool:
        """Kilit onaylı mı?"""
        return self.state_machine.lock_confirmed
    
    def get_lock_duration(self, current_time: float) -> float:
        """Kilit süresini döndür"""
        return self.state_machine.get_lock_duration(current_time)
    
    def get_virtual_target(self) -> np.ndarray:
        """Sanal hedef noktasını döndür"""
        return self.l1_controller.get_virtual_target()
    
    def get_cross_track_error(self) -> float:
        """Cross-track error döndür"""
        return self.l1_controller.get_cross_track_error()
