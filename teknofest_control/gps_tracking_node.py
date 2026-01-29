#!/usr/bin/env python3
"""
================================================================================
TEKNOFEST SAVAŞAN İHA - GPS TAKİP NODE'U
================================================================================
L1 Adaptive Guidance ile Gelişmiş Hedef Takip Sistemi

Yazar: HAVK Takımı
Tarih: 2026
Lisans: Apache-2.0

Bu node şu görevleri yerine getirir:
1. Weighted Scoring ile en uygun hedefi seçer
2. L1 Guidance algoritması ile smooth takip sağlar
3. PX4 Autopilot'a TrajectorySetpoint komutları gönderir
4. Rüzgar düzeltmesi ve smooth kontrol uygular

Kullanım:
  ros2 run teknofest_control gps_tracking_node --ros-args --params-file config/tracking_params.yaml
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
from collections import deque
import time
import math

# ============================================================================
# PX4 MESAJ TİPLERİ
# ============================================================================
# Not: px4_msgs paketi kurulu olmalı
# Kurulum: sudo apt install ros-humble-px4-msgs
try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleGlobalPosition,
        VehicleAttitude,
        VehicleStatus,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleCommand,
        VehicleOdometry
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False
    print("[UYARI] px4_msgs paketi bulunamadı. Simülasyon modunda çalışılacak.")

# Standart ROS2 mesajları (fallback ve debug için)
from std_msgs.msg import Header, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry


# ============================================================================
# SABİT DEĞERLER VE ENUMlar
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
    L1 Guidance çıktısı - PX4'e gönderilecek komut
    """
    velocity_north: float = 0.0     # Kuzey hızı komutu (m/s)
    velocity_east: float = 0.0      # Doğu hızı komutu (m/s)
    velocity_down: float = 0.0      # Aşağı hızı komutu (m/s)
    yaw_setpoint: float = 0.0       # Yaw açısı (radyan)
    yaw_rate: float = 0.0           # Yaw hızı (rad/s)
    airspeed_setpoint: float = 0.0  # Hava hızı komutu (m/s)
    is_valid: bool = False


# ============================================================================
# YARDIMCI FONKSİYONLAR
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
    # WGS84 sabitleri
    EARTH_RADIUS = 6371000.0  # metre
    
    # Radyana çevir
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    
    # Fark hesapla
    d_lat = lat_rad - ref_lat_rad
    d_lon = lon_rad - ref_lon_rad
    
    # NED koordinatları
    north = d_lat * EARTH_RADIUS
    east = d_lon * EARTH_RADIUS * math.cos(ref_lat_rad)
    down = ref_alt - alt  # Aşağı pozitif
    
    return north, east, down


def normalize_angle(angle: float) -> float:
    """
    Açıyı -180 ile +180 arasına normalize et
    """
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
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
    return np.linalg.norm(p2 - p1)


# ============================================================================
# LOW-PASS FİLTRE SINIFI
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
        self.prev_output = None
    
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
    
    def reset(self, value: float = None):
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
        self.prev_output = None
    
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
        
        # Değişimi sınırla
        if abs(change) > self.max_change:
            change = math.copysign(self.max_change, change)
        
        output = self.prev_output + change
        self.prev_output = output
        return output
    
    def reset(self, value: float = None):
        """Limiter'ı sıfırla"""
        self.prev_output = value


# ============================================================================
# ANA GPS TAKİP NODE'U
# ============================================================================

class GPSTrackingNode(Node):
    """
    TEKNOFEST Savaşan İHA - GPS Takip Node'u
    
    Bu node:
    1. Kendi uçağımızın durumunu (Own_State) takip eder
    2. Rakip listesinden (Target_List) en uygun hedefi seçer
    3. L1 Guidance ile smooth takip komutu üretir
    4. PX4'e TrajectorySetpoint gönderir
    """
    
    def __init__(self):
        super().__init__('gps_tracking_node')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEKNOFEST GPS TAKİP NODE'U BAŞLATILIYOR")
        self.get_logger().info("=" * 60)
        
        # ====================================================================
        # PARAMETRELERİ TANIMLA VE YÜKLE
        # ====================================================================
        self._declare_parameters()
        self._load_parameters()
        
        # ====================================================================
        # QoS PROFİLİ - PX4 UYUMLU
        # ====================================================================
        # PX4-ROS2 bridge için Best Effort QoS gerekli
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Standart QoS (diğer node'larla iletişim için)
        self.standard_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ====================================================================
        # DURUM DEĞİŞKENLERİ
        # ====================================================================
        self.own_state = AircraftState(id=0)  # Kendi uçağımız
        self.target_list: Dict[int, AircraftState] = {}  # Rakip listesi
        self.locked_target: Optional[AircraftState] = None  # Kilitli hedef
        self.locked_target_id: int = -1
        
        self.current_state = TrackingState.IDLE
        self.previous_state = TrackingState.IDLE
        
        # Kilitlenme zamanlaması
        self.lock_start_time: Optional[float] = None
        self.lock_confirmed: bool = False
        self.target_switch_time: float = 0.0
        
        # Referans noktası (home position)
        self.home_lat: float = 0.0
        self.home_lon: float = 0.0
        self.home_alt: float = 0.0
        self.home_set: bool = False
        
        # ====================================================================
        # SMOOTH KONTROL FİLTRELERİ
        # ====================================================================
        self._init_filters()
        
        # ====================================================================
        # L1 GUIDANCE DEĞİŞKENLERİ
        # ====================================================================
        self.virtual_target = np.zeros(3)  # Sanal hedef noktası (NED)
        self.l1_reference_point = np.zeros(3)
        self.cross_track_error: float = 0.0
        self.along_track_error: float = 0.0
        
        # Rüzgar tahmini
        self.estimated_wind = np.zeros(3)  # NED rüzgar vektörü
        self.crab_angle: float = 0.0
        
        # Önceki komutlar (smoothing için)
        self.prev_command = GuidanceCommand()
        
        # ====================================================================
        # İSTATİSTİKLER VE DEBUG
        # ====================================================================
        self.stats = {
            'targets_evaluated': 0,
            'successful_locks': 0,
            'lock_duration_total': 0.0,
            'current_lock_duration': 0.0,
            'state_transitions': 0
        }
        
        # Komut geçmişi (debug için)
        self.command_history = deque(maxlen=100)
        
        # ====================================================================
        # SUBSCRIBER'LAR
        # ====================================================================
        self._create_subscribers()
        
        # ====================================================================
        # PUBLISHER'LAR
        # ====================================================================
        self._create_publishers()
        
        # ====================================================================
        # TIMER'LAR
        # ====================================================================
        # Ana kontrol döngüsü
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            control_period,
            self.control_loop_callback
        )
        
        # Offboard heartbeat (PX4 gereksinimi)
        self.offboard_timer = self.create_timer(
            0.1,  # 10 Hz
            self.offboard_heartbeat_callback
        )
        
        # Durum raporu
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.status_report_callback
        )
        
        self.get_logger().info("GPS Takip Node'u başarıyla başlatıldı!")
        self.get_logger().info(f"Kontrol frekansı: {self.control_frequency} Hz")
        self.get_logger().info(f"L1 Mesafesi: {self.l1_distance} m")
    
    # ========================================================================
    # PARAMETRE YÖNETİMİ
    # ========================================================================
    
    def _declare_parameters(self):
        """ROS2 parametrelerini tanımla"""
        
        # Ağırlıklar
        self.declare_parameter('weights.distance', 0.35)
        self.declare_parameter('weights.angle', 0.45)
        self.declare_parameter('weights.speed', 0.20)
        
        # Açı puanlama
        self.declare_parameter('angle_scoring.tail_angle_bonus', 2.5)
        self.declare_parameter('angle_scoring.head_on_penalty', 0.2)
        self.declare_parameter('angle_scoring.tail_cone_half_angle', 45.0)
        self.declare_parameter('angle_scoring.head_on_cone_half_angle', 30.0)
        
        # L1 Guidance
        self.declare_parameter('l1_guidance.l1_distance', 50.0)
        self.declare_parameter('l1_guidance.l1_damping', 0.85)
        self.declare_parameter('l1_guidance.l1_period', 25.0)
        self.declare_parameter('l1_guidance.adaptive_l1', True)
        
        # Uçak limitleri
        self.declare_parameter('aircraft_limits.min_airspeed', 15.0)
        self.declare_parameter('aircraft_limits.max_airspeed', 35.0)
        self.declare_parameter('aircraft_limits.cruise_airspeed', 22.0)
        self.declare_parameter('aircraft_limits.max_bank_angle', 45.0)
        self.declare_parameter('aircraft_limits.min_turn_radius', 30.0)
        self.declare_parameter('aircraft_limits.loiter_radius', 40.0)
        
        # Smoothing
        self.declare_parameter('smoothing.heading_rate_limit', 15.0)
        self.declare_parameter('smoothing.altitude_rate_limit', 3.0)
        self.declare_parameter('smoothing.command_smoothing_alpha', 0.15)
        self.declare_parameter('smoothing.lpf_cutoff_heading', 0.5)
        
        # Durum makinesi
        self.declare_parameter('state_machine.lock_distance', 80.0)
        self.declare_parameter('state_machine.approach_distance', 150.0)
        self.declare_parameter('state_machine.loiter_trigger_distance', 25.0)
        self.declare_parameter('state_machine.lock_confirmation_time', 4.0)
        
        # PX4
        self.declare_parameter('px4.control_frequency', 50.0)
        
        # Güvenlik
        self.declare_parameter('safety.min_altitude_agl', 50.0)
        self.declare_parameter('safety.collision_avoidance_distance', 15.0)
    
    def _load_parameters(self):
        """Parametreleri yükle ve değişkenlere ata"""
        
        # Ağırlıklar
        self.w_distance = self.get_parameter('weights.distance').value
        self.w_angle = self.get_parameter('weights.angle').value
        self.w_speed = self.get_parameter('weights.speed').value
        
        # Açı puanlama
        self.tail_bonus = self.get_parameter('angle_scoring.tail_angle_bonus').value
        self.head_on_penalty = self.get_parameter('angle_scoring.head_on_penalty').value
        self.tail_cone = self.get_parameter('angle_scoring.tail_cone_half_angle').value
        self.head_on_cone = self.get_parameter('angle_scoring.head_on_cone_half_angle').value
        
        # L1 Guidance
        self.l1_distance = self.get_parameter('l1_guidance.l1_distance').value
        self.l1_damping = self.get_parameter('l1_guidance.l1_damping').value
        self.l1_period = self.get_parameter('l1_guidance.l1_period').value
        self.adaptive_l1 = self.get_parameter('l1_guidance.adaptive_l1').value
        
        # Uçak limitleri
        self.min_airspeed = self.get_parameter('aircraft_limits.min_airspeed').value
        self.max_airspeed = self.get_parameter('aircraft_limits.max_airspeed').value
        self.cruise_airspeed = self.get_parameter('aircraft_limits.cruise_airspeed').value
        self.max_bank = self.get_parameter('aircraft_limits.max_bank_angle').value
        self.min_turn_radius = self.get_parameter('aircraft_limits.min_turn_radius').value
        self.loiter_radius = self.get_parameter('aircraft_limits.loiter_radius').value
        
        # Smoothing
        self.heading_rate_limit = self.get_parameter('smoothing.heading_rate_limit').value
        self.altitude_rate_limit = self.get_parameter('smoothing.altitude_rate_limit').value
        self.smoothing_alpha = self.get_parameter('smoothing.command_smoothing_alpha').value
        self.lpf_cutoff = self.get_parameter('smoothing.lpf_cutoff_heading').value
        
        # Durum makinesi
        self.lock_distance = self.get_parameter('state_machine.lock_distance').value
        self.approach_distance = self.get_parameter('state_machine.approach_distance').value
        self.loiter_trigger = self.get_parameter('state_machine.loiter_trigger_distance').value
        self.lock_confirm_time = self.get_parameter('state_machine.lock_confirmation_time').value
        
        # PX4
        self.control_frequency = self.get_parameter('px4.control_frequency').value
        
        # Güvenlik
        self.min_alt = self.get_parameter('safety.min_altitude_agl').value
        self.collision_dist = self.get_parameter('safety.collision_avoidance_distance').value
    
    def _init_filters(self):
        """Smooth kontrol filtreleri başlat"""
        
        # Heading için low-pass filter ve rate limiter
        self.heading_lpf = LowPassFilter(self.lpf_cutoff, self.control_frequency)
        self.heading_rate_limiter = RateLimiter(self.heading_rate_limit, self.control_frequency)
        
        # Altitude için
        self.altitude_lpf = LowPassFilter(0.3, self.control_frequency)
        self.altitude_rate_limiter = RateLimiter(self.altitude_rate_limit, self.control_frequency)
        
        # Velocity için
        self.vn_lpf = LowPassFilter(0.5, self.control_frequency)
        self.ve_lpf = LowPassFilter(0.5, self.control_frequency)
        self.vd_lpf = LowPassFilter(0.3, self.control_frequency)
        
        self.get_logger().info("Smooth kontrol filtreleri başlatıldı")
    
    # ========================================================================
    # SUBSCRIBER'LAR
    # ========================================================================
    
    def _create_subscribers(self):
        """Tüm subscriber'ları oluştur"""
        
        if PX4_MSGS_AVAILABLE:
            # PX4 Lokal Pozisyon
            self.local_pos_sub = self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self.vehicle_local_position_callback,
                self.px4_qos
            )
            
            # PX4 Global Pozisyon (GPS)
            self.global_pos_sub = self.create_subscription(
                VehicleGlobalPosition,
                '/fmu/out/vehicle_global_position',
                self.vehicle_global_position_callback,
                self.px4_qos
            )
            
            # PX4 Attitude
            self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
                self.vehicle_attitude_callback,
                self.px4_qos
            )
            
            # PX4 Status
            self.status_sub = self.create_subscription(
                VehicleStatus,
                '/fmu/out/vehicle_status',
                self.vehicle_status_callback,
                self.px4_qos
            )
        
        # Hedef listesi (yarışma sunucusu veya simülasyondan)
        self.target_list_sub = self.create_subscription(
            Float64MultiArray,
            '/competition/target_list',
            self.target_list_callback,
            self.standard_qos
        )
        
        # Alternatif hedef topic'i (özel mesaj formatı)
        self.targets_sub = self.create_subscription(
            String,
            '/simulation/enemy_aircraft',
            self.enemy_aircraft_callback,
            self.standard_qos
        )
        
        self.get_logger().info("Subscriber'lar oluşturuldu")
    
    # ========================================================================
    # PUBLISHER'LAR
    # ========================================================================
    
    def _create_publishers(self):
        """Tüm publisher'ları oluştur"""
        
        if PX4_MSGS_AVAILABLE:
            # PX4 TrajectorySetpoint
            self.trajectory_pub = self.create_publisher(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint',
                self.px4_qos
            )
            
            # PX4 OffboardControlMode
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode,
                '/fmu/in/offboard_control_mode',
                self.px4_qos
            )
            
            # PX4 VehicleCommand
            self.command_pub = self.create_publisher(
                VehicleCommand,
                '/fmu/in/vehicle_command',
                self.px4_qos
            )
        
        # Debug ve görselleştirme
        self.debug_pub = self.create_publisher(
            String,
            '/tracking/debug_info',
            self.standard_qos
        )
        
        self.virtual_target_pub = self.create_publisher(
            PoseStamped,
            '/tracking/virtual_target',
            self.standard_qos
        )
        
        self.state_pub = self.create_publisher(
            String,
            '/tracking/state',
            self.standard_qos
        )
        
        self.get_logger().info("Publisher'lar oluşturuldu")
    
    # ========================================================================
    # CALLBACK FONKSİYONLARI - VERİ ALMA
    # ========================================================================
    
    def vehicle_local_position_callback(self, msg):
        """
        PX4'ten lokal pozisyon verisi al
        NED koordinatlarında konum ve hız bilgisi içerir
        """
        self.own_state.x = msg.x
        self.own_state.y = msg.y
        self.own_state.z = msg.z
        self.own_state.vx = msg.vx
        self.own_state.vy = msg.vy
        self.own_state.vz = msg.vz
        self.own_state.heading = math.degrees(msg.heading)
        
        # Home pozisyonunu ayarla
        if not self.home_set and msg.xy_global:
            self.home_lat = msg.ref_lat
            self.home_lon = msg.ref_lon
            self.home_alt = msg.ref_alt
            self.home_set = True
            self.get_logger().info(f"Home pozisyonu ayarlandı: {self.home_lat:.6f}, {self.home_lon:.6f}")
    
    def vehicle_global_position_callback(self, msg):
        """
        PX4'ten global GPS pozisyon verisi al
        """
        self.own_state.latitude = msg.lat
        self.own_state.longitude = msg.lon
        self.own_state.altitude = msg.alt
        self.own_state.timestamp = self.get_clock().now().nanoseconds / 1e9
    
    def vehicle_attitude_callback(self, msg):
        """
        PX4'ten attitude (duruş) verisi al
        Quaternion'dan Euler açılarına çevir
        """
        # Quaternion: [w, x, y, z]
        q = msg.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        self.own_state.roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            self.own_state.pitch = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            self.own_state.pitch = math.degrees(math.asin(sinp))
    
    def vehicle_status_callback(self, msg):
        """PX4 durum bilgisi"""
        pass  # Gerektiğinde kullanılacak
    
    def target_list_callback(self, msg):
        """
        Yarışma sunucusundan hedef listesi al
        
        Mesaj formatı (Float64MultiArray):
        Her hedef için: [id, lat, lon, alt, speed, heading]
        Toplam: N_targets * 6 eleman
        """
        data = msg.data
        num_targets = len(data) // 6
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for i in range(num_targets):
            idx = i * 6
            target_id = int(data[idx])
            
            # Yeni hedef oluştur veya güncelle
            if target_id not in self.target_list:
                self.target_list[target_id] = AircraftState(id=target_id)
            
            target = self.target_list[target_id]
            target.latitude = data[idx + 1]
            target.longitude = data[idx + 2]
            target.altitude = data[idx + 3]
            target.ground_speed = data[idx + 4]
            target.heading = data[idx + 5]
            target.timestamp = current_time
            
            # NED koordinatlarına çevir
            if self.home_set:
                n, e, d = geodetic_to_ned(
                    target.latitude, target.longitude, target.altitude,
                    self.home_lat, self.home_lon, self.home_alt
                )
                target.x = n
                target.y = e
                target.z = d
                
                # Hız vektörünü hesapla
                heading_rad = math.radians(target.heading)
                target.vx = target.ground_speed * math.cos(heading_rad)
                target.vy = target.ground_speed * math.sin(heading_rad)
                target.vz = 0.0  # Varsayılan
        
        self.stats['targets_evaluated'] = num_targets
    
    def enemy_aircraft_callback(self, msg):
        """
        Simülasyondan düşman uçak verisi al (String formatı)
        JSON parse edilebilir
        """
        import json
        try:
            data = json.loads(msg.data)
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            for aircraft in data.get('aircraft', []):
                target_id = aircraft.get('id', 0)
                
                if target_id not in self.target_list:
                    self.target_list[target_id] = AircraftState(id=target_id)
                
                target = self.target_list[target_id]
                target.latitude = aircraft.get('lat', 0.0)
                target.longitude = aircraft.get('lon', 0.0)
                target.altitude = aircraft.get('alt', 0.0)
                target.ground_speed = aircraft.get('speed', 0.0)
                target.heading = aircraft.get('heading', 0.0)
                target.timestamp = current_time
                
                # NED koordinatlarına çevir
                if self.home_set:
                    n, e, d = geodetic_to_ned(
                        target.latitude, target.longitude, target.altitude,
                        self.home_lat, self.home_lon, self.home_alt
                    )
                    target.x = n
                    target.y = e
                    target.z = d
                    
                    heading_rad = math.radians(target.heading)
                    target.vx = target.ground_speed * math.cos(heading_rad)
                    target.vy = target.ground_speed * math.sin(heading_rad)
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse hatası: {e}")
    
    # ========================================================================
    # ANA KONTROL DÖNGÜSÜ
    # ========================================================================
    
    def control_loop_callback(self):
        """
        Ana kontrol döngüsü - Her döngüde:
        1. Eski hedefleri temizle
        2. En iyi hedefi seç
        3. Durum makinesini güncelle
        4. L1 Guidance hesapla
        5. Komutu smooth et ve yayınla
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ----------------------------------------------------------------
        # 1. ESKİ HEDEFLERİ TEMİZLE (timeout)
        # ----------------------------------------------------------------
        self._cleanup_stale_targets(current_time)
        
        # ----------------------------------------------------------------
        # 2. HEDEF SEÇİMİ
        # ----------------------------------------------------------------
        if len(self.target_list) > 0:
            best_target = self.select_best_target()
            
            if best_target is not None:
                # Hedef değişikliği kontrolü
                if self.locked_target_id != best_target.target_id:
                    # Cooldown kontrolü
                    if current_time - self.target_switch_time > 3.0:
                        self._switch_target(best_target.target_id)
                
                self.locked_target = self.target_list.get(self.locked_target_id)
        else:
            self.locked_target = None
            if self.current_state != TrackingState.SEARCHING:
                self._change_state(TrackingState.SEARCHING)
        
        # ----------------------------------------------------------------
        # 3. DURUM MAKİNESİ
        # ----------------------------------------------------------------
        self._update_state_machine(current_time)
        
        # ----------------------------------------------------------------
        # 4. L1 GUIDANCE VE KOMUT ÜRETİMİ
        # ----------------------------------------------------------------
        if self.locked_target is not None and self.current_state != TrackingState.IDLE:
            command = self.l1_guidance_logic()
            
            if command.is_valid:
                # Komutu smooth et
                smoothed_command = self._smooth_command(command)
                
                # PX4'e yayınla
                self.publish_command(smoothed_command)
                
                # Sanal hedefi yayınla (görselleştirme için)
                self._publish_virtual_target()
        
        # ----------------------------------------------------------------
        # 5. KİLİTLENME SÜRESİ TAKİBİ
        # ----------------------------------------------------------------
        if self.current_state == TrackingState.LOCKED:
            if self.lock_start_time is not None:
                lock_duration = current_time - self.lock_start_time
                self.stats['current_lock_duration'] = lock_duration
                
                if lock_duration >= self.lock_confirm_time and not self.lock_confirmed:
                    self.lock_confirmed = True
                    self.stats['successful_locks'] += 1
                    self.get_logger().info(
                        f"✓ KİLİTLENME ONAYLANDI! Hedef: {self.locked_target_id}, "
                        f"Süre: {lock_duration:.2f}s"
                    )
    
    def _cleanup_stale_targets(self, current_time: float):
        """Timeout olan hedefleri temizle"""
        stale_ids = []
        for target_id, target in self.target_list.items():
            if current_time - target.timestamp > 5.0:  # 5 saniye timeout
                stale_ids.append(target_id)
        
        for target_id in stale_ids:
            del self.target_list[target_id]
            if target_id == self.locked_target_id:
                self.get_logger().warn(f"Kilitli hedef kayboldu: {target_id}")
                self.locked_target = None
                self.locked_target_id = -1
    
    def _switch_target(self, new_target_id: int):
        """Hedef değiştir"""
        self.get_logger().info(
            f"Hedef değiştirildi: {self.locked_target_id} -> {new_target_id}"
        )
        self.locked_target_id = new_target_id
        self.target_switch_time = self.get_clock().now().nanoseconds / 1e9
        self.lock_start_time = None
        self.lock_confirmed = False
        
        # Filtreleri sıfırla (yeni hedefe smooth geçiş için)
        self._reset_filters_for_new_target()
    
    def _reset_filters_for_new_target(self):
        """Hedef değiştiğinde filtreleri sıfırla"""
        self.heading_lpf.reset()
        self.heading_rate_limiter.reset()
        self.vn_lpf.reset()
        self.ve_lpf.reset()
        self.vd_lpf.reset()
    
    # ========================================================================
    # HEDEF SEÇİM ALGORİTMASI (Weighted Scoring)
    # ========================================================================
    
    def select_best_target(self) -> Optional[TargetScore]:
        """
        Tüm hedefleri puanla ve en iyisini seç
        
        Formül:
        Score = (W1 * 1/Distance) + (W2 * Angle_Factor) + (W3 * Speed_Factor)
        
        Returns:
            En yüksek puanlı hedefin TargetScore objesi
        """
        if len(self.target_list) == 0:
            return None
        
        scores: List[TargetScore] = []
        
        own_pos = self.own_state.get_position_ned()
        own_heading = self.own_state.heading
        
        for target_id, target in self.target_list.items():
            score = self.calculate_weighted_score(target, own_pos, own_heading)
            scores.append(score)
        
        # En yüksek skoru bul
        scores.sort(key=lambda x: x.total_score, reverse=True)
        
        if len(scores) > 0:
            return scores[0]
        return None
    
    def calculate_weighted_score(self, target: AircraftState, 
                                  own_pos: np.ndarray,
                                  own_heading: float) -> TargetScore:
        """
        Tek bir hedef için ağırlıklı skor hesapla
        
        Args:
            target: Hedef uçak durumu
            own_pos: Kendi NED pozisyonumuz
            own_heading: Kendi başlık açımız (derece)
        
        Returns:
            TargetScore objesi
        """
        target_pos = target.get_position_ned()
        
        # ----------------------------------------------------------------
        # 1. MESAFE SKORU
        # ----------------------------------------------------------------
        distance = calculate_distance_3d(own_pos, target_pos)
        
        # Normalize et (0-1000m aralığında)
        # Yakın = yüksek skor
        if distance < 10:
            distance = 10  # Sıfıra bölmeyi önle
        
        # Sigmoid benzeri normalize (50-200m arası optimum)
        distance_score = 1.0 / (1.0 + (distance / 100.0))
        
        # ----------------------------------------------------------------
        # 2. AÇI UYGUNLUĞU SKORU
        # ----------------------------------------------------------------
        # Hedefe olan bearing (biz hedefe bakış açısı)
        bearing = calculate_bearing(own_pos[0], own_pos[1], 
                                   target_pos[0], target_pos[1])
        
        # Hedefin bize bakış açısı (Aspect Angle)
        # 0° = kuyruk (arkadan bakıyor), 180° = kafa kafaya
        reverse_bearing = calculate_bearing(target_pos[0], target_pos[1],
                                           own_pos[0], own_pos[1])
        
        aspect_angle = abs(normalize_angle(target.heading - reverse_bearing))
        
        # Kuyruk pozisyonu kontrolü (0-45° = kuyruk)
        is_tail = aspect_angle < self.tail_cone
        
        # Kafa kafaya kontrolü (150-180° = tehlikeli)
        is_head_on = aspect_angle > (180 - self.head_on_cone)
        
        # Açı skoru hesapla
        if is_tail:
            # BONUS: Kuyruk pozisyonundayız! En iyi durum.
            angle_score = self.tail_bonus
        elif is_head_on:
            # CEZA: Kafa kafaya geliyor - çarpışma riski!
            angle_score = self.head_on_penalty
        else:
            # Normal: Açıya göre lineer skor
            # 90° = 1.0, yanlara doğru
            angle_score = 1.0 - (abs(aspect_angle - 90) / 90.0)
        
        # ----------------------------------------------------------------
        # 3. HIZ FAKTÖRÜ SKORU
        # ----------------------------------------------------------------
        # Yavaş hedef = yakalamak kolay
        # Hızlı hedef = takip zor
        
        our_speed = math.sqrt(self.own_state.vx**2 + self.own_state.vy**2)
        target_speed = target.ground_speed
        
        # Hız farkı (pozitif = biz daha hızlıyız)
        speed_diff = our_speed - target_speed
        
        # Normalize et
        if speed_diff > 0:
            # Biz daha hızlıyız - iyi!
            speed_score = min(1.0 + speed_diff / 10.0, 2.0)
        else:
            # Hedef daha hızlı - kötü
            speed_score = max(0.3, 1.0 + speed_diff / 20.0)
        
        # ----------------------------------------------------------------
        # TOPLAM SKOR
        # ----------------------------------------------------------------
        total_score = (
            self.w_distance * distance_score +
            self.w_angle * angle_score +
            self.w_speed * speed_score
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
    
    # ========================================================================
    # DURUM MAKİNESİ
    # ========================================================================
    
    def _update_state_machine(self, current_time: float):
        """
        Takip durum makinesini güncelle
        Mevcut duruma ve koşullara göre geçişler yap
        """
        if self.locked_target is None:
            if self.current_state not in [TrackingState.IDLE, TrackingState.SEARCHING]:
                self._change_state(TrackingState.SEARCHING)
            return
        
        # Hedefe mesafe hesapla
        own_pos = self.own_state.get_position_ned()
        target_pos = self.locked_target.get_position_ned()
        distance = calculate_distance_3d(own_pos, target_pos)
        
        # Mevcut duruma göre geçişler
        if self.current_state == TrackingState.IDLE:
            self._change_state(TrackingState.SEARCHING)
        
        elif self.current_state == TrackingState.SEARCHING:
            if distance < self.approach_distance:
                self._change_state(TrackingState.APPROACHING)
        
        elif self.current_state == TrackingState.APPROACHING:
            if distance < self.lock_distance:
                self._change_state(TrackingState.PURSUING)
            elif distance > self.approach_distance * 1.2:  # Hysteresis
                self._change_state(TrackingState.SEARCHING)
        
        elif self.current_state == TrackingState.PURSUING:
            if distance < self.loiter_trigger:
                self._change_state(TrackingState.LOITERING)
            elif distance < self.lock_distance * 0.8:
                # Stabil takip - kilitleme başlat
                self._change_state(TrackingState.LOCKED)
                self.lock_start_time = current_time
            elif distance > self.lock_distance * 1.3:
                self._change_state(TrackingState.APPROACHING)
        
        elif self.current_state == TrackingState.LOCKED:
            if distance > self.lock_distance * 1.5:
                # Kilitlenme kayboldu
                self._change_state(TrackingState.PURSUING)
                self.lock_start_time = None
                self.lock_confirmed = False
            elif distance < self.loiter_trigger:
                self._change_state(TrackingState.LOITERING)
        
        elif self.current_state == TrackingState.LOITERING:
            if distance > self.loiter_trigger * 2:
                self._change_state(TrackingState.PURSUING)
    
    def _change_state(self, new_state: TrackingState):
        """Durum değiştir ve logla"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.stats['state_transitions'] += 1
            
            self.get_logger().info(
                f"Durum değişti: {self.previous_state.name} -> {new_state.name}"
            )
            
            # Durum yayınla
            state_msg = String()
            state_msg.data = new_state.name
            self.state_pub.publish(state_msg)
    
    # ========================================================================
    # L1 GUIDANCE ALGORİTMASI
    # ========================================================================
    
    def l1_guidance_logic(self) -> GuidanceCommand:
        """
        L1 Adaptive Guidance algoritması
        
        Bu algoritma:
        1. Hedefin arkasında sanal bir nokta hesaplar (L1 mesafesinde)
        2. Bu noktaya gitmek için gerekli hız vektörünü hesaplar
        3. Cross-track error'u minimize eder
        4. Rüzgar düzeltmesi (crab angle) uygular
        
        Returns:
            GuidanceCommand objesi
        """
        if self.locked_target is None:
            return GuidanceCommand(is_valid=False)
        
        command = GuidanceCommand(is_valid=True)
        
        # Pozisyon ve hız vektörleri
        own_pos = self.own_state.get_position_ned()
        own_vel = self.own_state.get_velocity_vector()
        target_pos = self.locked_target.get_position_ned()
        target_vel = self.locked_target.get_velocity_vector()
        
        # ----------------------------------------------------------------
        # 1. SANAL HEDEF NOKTASI HESAPLA (Virtual Target)
        # ----------------------------------------------------------------
        # Hedefin hız vektörünün tersi yönünde L1 mesafesi kadar geri git
        # Amaç: Hedefin tam arkasına konumlanmak
        
        target_speed = np.linalg.norm(target_vel[:2])  # 2D hız
        
        if target_speed > 0.5:
            # Hedef hareket ediyor - arkasına git
            target_heading_vec = target_vel[:2] / target_speed
            
            # Adaptif L1 mesafesi (hıza bağlı)
            if self.adaptive_l1:
                l1_dist = self.l1_distance * (1 + target_speed / 30.0)
                l1_dist = np.clip(l1_dist, 30.0, 100.0)
            else:
                l1_dist = self.l1_distance
            
            # Sanal hedef = Hedef - (L1 * heading_vector)
            virtual_target_2d = target_pos[:2] - l1_dist * target_heading_vec
            virtual_target_z = target_pos[2]  # Aynı irtifa
        else:
            # Hedef durağan - direkt hedefe git
            virtual_target_2d = target_pos[:2]
            virtual_target_z = target_pos[2]
            l1_dist = self.l1_distance
        
        self.virtual_target = np.array([virtual_target_2d[0], 
                                        virtual_target_2d[1], 
                                        virtual_target_z])
        
        # ----------------------------------------------------------------
        # 2. L1 REFERANS NOKTASI VE HATA HESABI
        # ----------------------------------------------------------------
        # Sanal hedefe vektör
        los_vec = self.virtual_target - own_pos  # Line of Sight
        los_distance = np.linalg.norm(los_vec[:2])
        
        if los_distance < 1.0:
            los_distance = 1.0
        
        los_unit = los_vec / np.linalg.norm(los_vec)
        
        # L1 referans noktası (mevcut pozisyon + L1 mesafesi ileride)
        if np.linalg.norm(own_vel[:2]) > 0.5:
            vel_unit = own_vel / np.linalg.norm(own_vel)
        else:
            vel_unit = np.array([1.0, 0.0, 0.0])
        
        self.l1_reference_point = own_pos + l1_dist * vel_unit
        
        # Cross-track error (yandan sapma)
        # Hedef çizgisine dik mesafe
        track_vec = self.virtual_target - self.l1_reference_point
        self.cross_track_error = np.cross(los_vec[:2], own_vel[:2] / 
                                          (np.linalg.norm(own_vel[:2]) + 0.01))
        
        # Along-track error (boyuna hata)
        self.along_track_error = np.dot(los_vec[:2], 
                                        own_vel[:2] / (np.linalg.norm(own_vel[:2]) + 0.01))
        
        # ----------------------------------------------------------------
        # 3. L1 LATERAL ACCELERATION VE HEADING HESABI
        # ----------------------------------------------------------------
        # L1 lateral ivme formülü:
        # a_cmd = 2 * V^2 / L1 * sin(eta)
        # eta = LOS açısı - heading açısı
        
        own_speed = np.linalg.norm(own_vel[:2])
        if own_speed < self.min_airspeed:
            own_speed = self.min_airspeed
        
        # LOS açısı
        los_angle = math.atan2(los_vec[1], los_vec[0])
        
        # Mevcut heading
        current_heading = math.radians(self.own_state.heading)
        
        # Eta (açı farkı)
        eta = wrap_to_2pi(los_angle) - wrap_to_2pi(current_heading)
        eta = normalize_angle(math.degrees(eta))
        eta = math.radians(eta)
        
        # L1 lateral ivme
        l1_accel = 2 * (own_speed ** 2) / l1_dist * math.sin(eta)
        
        # İvmeyi bank açısına çevir
        g = 9.81
        commanded_bank = math.atan(l1_accel / g)
        commanded_bank = np.clip(commanded_bank, 
                                 -math.radians(self.max_bank), 
                                 math.radians(self.max_bank))
        
        # ----------------------------------------------------------------
        # 4. RÜZGAR DÜZELTMESİ (Crab Angle)
        # ----------------------------------------------------------------
        # Basit rüzgar tahmini: Ground speed - Airspeed farkından
        # Daha gelişmiş: EKF ile rüzgar vektörü tahmini
        
        if self.own_state.airspeed > 0:
            # Basit crab angle hesabı
            wind_component = self.cross_track_error * 0.1  # Basitleştirilmiş
            self.crab_angle = math.atan2(wind_component, own_speed)
            self.crab_angle = np.clip(self.crab_angle, 
                                      -math.radians(20), 
                                      math.radians(20))
        
        # ----------------------------------------------------------------
        # 5. KOMUT OLUŞTUR
        # ----------------------------------------------------------------
        
        # Duruma göre hız komutu
        if self.current_state == TrackingState.LOITERING:
            # Loiter modu - etrafında dön
            command = self._generate_loiter_command(target_pos)
        else:
            # Normal takip
            desired_heading = los_angle + self.crab_angle
            
            # Hız büyüklüğü (mesafeye göre ayarla)
            if los_distance > 100:
                speed_cmd = self.max_airspeed
            elif los_distance > 50:
                speed_cmd = self.cruise_airspeed + (self.max_airspeed - self.cruise_airspeed) * \
                           (los_distance - 50) / 50
            else:
                speed_cmd = self.cruise_airspeed
            
            # Hedef de hareket ediyorsa, hızını hesaba kat
            if target_speed > 0:
                speed_cmd = max(speed_cmd, target_speed + 3.0)  # Biraz daha hızlı git
            
            speed_cmd = np.clip(speed_cmd, self.min_airspeed, self.max_airspeed)
            
            # Velocity komutları (NED)
            command.velocity_north = speed_cmd * math.cos(desired_heading)
            command.velocity_east = speed_cmd * math.sin(desired_heading)
            
            # Altitude komutu (hedefle aynı irtifada kal)
            altitude_error = target_pos[2] - own_pos[2]  # Down = pozitif
            command.velocity_down = np.clip(-altitude_error * 0.5, 
                                           -self.altitude_rate_limit, 
                                           self.altitude_rate_limit)
            
            # Yaw komutu
            command.yaw_setpoint = desired_heading
            command.airspeed_setpoint = speed_cmd
        
        return command
    
    def _generate_loiter_command(self, center: np.ndarray) -> GuidanceCommand:
        """
        Loiter (dönme) komutu oluştur
        Hedefin etrafında sabit yarıçapla dön
        """
        command = GuidanceCommand(is_valid=True)
        
        own_pos = self.own_state.get_position_ned()
        
        # Merkeze vektör
        to_center = center[:2] - own_pos[:2]
        dist_to_center = np.linalg.norm(to_center)
        
        # Teğet yön (saat yönünde)
        if dist_to_center > 0.1:
            radial_unit = to_center / dist_to_center
            tangent_unit = np.array([-radial_unit[1], radial_unit[0]])  # 90° döndür
        else:
            tangent_unit = np.array([1.0, 0.0])
        
        # Yarıçap düzeltmesi
        radius_error = dist_to_center - self.loiter_radius
        
        # Hız: teğet + radyal düzeltme
        tangent_speed = self.cruise_airspeed
        radial_correction = radius_error * 0.3  # P kontrolcü
        
        velocity_2d = tangent_speed * tangent_unit + radial_correction * (to_center / (dist_to_center + 0.01))
        
        command.velocity_north = velocity_2d[0]
        command.velocity_east = velocity_2d[1]
        command.velocity_down = 0.0  # İrtifa koru
        command.yaw_setpoint = math.atan2(velocity_2d[1], velocity_2d[0])
        command.airspeed_setpoint = tangent_speed
        
        return command
    
    # ========================================================================
    # KOMUT SMOOTH ET VE YAYINLA
    # ========================================================================
    
    def _smooth_command(self, command: GuidanceCommand) -> GuidanceCommand:
        """
        Komutu smooth et - ani manevraları önle
        
        Bu fonksiyon:
        1. Low-pass filter uygular
        2. Rate limiter uygular
        3. Exponential smoothing uygular
        """
        smoothed = GuidanceCommand(is_valid=True)
        
        # Velocity smoothing
        smoothed.velocity_north = self.vn_lpf.filter(command.velocity_north)
        smoothed.velocity_east = self.ve_lpf.filter(command.velocity_east)
        smoothed.velocity_down = self.vd_lpf.filter(command.velocity_down)
        
        # Heading smoothing (açı wrap dikkat!)
        current_yaw = command.yaw_setpoint
        prev_yaw = self.prev_command.yaw_setpoint
        
        # Açı farkını normalize et
        yaw_diff = current_yaw - prev_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Rate limit uygula
        max_yaw_rate = math.radians(self.heading_rate_limit) / self.control_frequency
        yaw_diff = np.clip(yaw_diff, -max_yaw_rate, max_yaw_rate)
        
        smoothed.yaw_setpoint = prev_yaw + yaw_diff
        
        # Exponential smoothing
        alpha = self.smoothing_alpha
        smoothed.velocity_north = alpha * smoothed.velocity_north + \
                                  (1 - alpha) * self.prev_command.velocity_north
        smoothed.velocity_east = alpha * smoothed.velocity_east + \
                                 (1 - alpha) * self.prev_command.velocity_east
        
        smoothed.airspeed_setpoint = command.airspeed_setpoint
        
        # Önceki komutu güncelle
        self.prev_command = smoothed
        
        return smoothed
    
    def publish_command(self, command: GuidanceCommand):
        """
        PX4'e TrajectorySetpoint yayınla
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        # TrajectorySetpoint mesajı oluştur
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # Mikrosaniye
        
        # Velocity setpoint (NED frame)
        msg.velocity[0] = command.velocity_north
        msg.velocity[1] = command.velocity_east
        msg.velocity[2] = command.velocity_down
        
        # Position NaN (velocity modunda)
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        
        # Yaw setpoint
        msg.yaw = command.yaw_setpoint
        msg.yawspeed = float('nan')  # Yaw rate kullanmıyoruz
        
        # Acceleration NaN
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        
        self.trajectory_pub.publish(msg)
        
        # Komut geçmişine ekle
        self.command_history.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'vn': command.velocity_north,
            've': command.velocity_east,
            'vd': command.velocity_down,
            'yaw': command.yaw_setpoint
        })
    
    def offboard_heartbeat_callback(self):
        """
        PX4 Offboard heartbeat - sürekli gönderilmeli
        Aksi halde PX4 otomatik olarak manuel moda geçer
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Velocity kontrol modu
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_mode_pub.publish(msg)
    
    def _publish_virtual_target(self):
        """
        Sanal hedef noktasını görselleştirme için yayınla
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = self.virtual_target[0]
        msg.pose.position.y = self.virtual_target[1]
        msg.pose.position.z = -self.virtual_target[2]  # NED -> ENU
        
        self.virtual_target_pub.publish(msg)
    
    # ========================================================================
    # DURUM RAPORU
    # ========================================================================
    
    def status_report_callback(self):
        """Periyodik durum raporu"""
        
        debug_info = {
            'state': self.current_state.name,
            'locked_target': self.locked_target_id,
            'targets_count': len(self.target_list),
            'lock_confirmed': self.lock_confirmed,
            'lock_duration': self.stats['current_lock_duration'],
            'successful_locks': self.stats['successful_locks']
        }
        
        if self.locked_target is not None:
            own_pos = self.own_state.get_position_ned()
            target_pos = self.locked_target.get_position_ned()
            distance = calculate_distance_3d(own_pos, target_pos)
            debug_info['distance_to_target'] = round(distance, 1)
            debug_info['cross_track_error'] = round(self.cross_track_error, 2)
        
        msg = String()
        msg.data = str(debug_info)
        self.debug_pub.publish(msg)


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main(args=None):
    """
    Node başlatma fonksiyonu
    """
    rclpy.init(args=args)
    
    try:
        node = GPSTrackingNode()
        
        # Multi-threaded executor kullan (paralel callback'ler için)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Hata: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
