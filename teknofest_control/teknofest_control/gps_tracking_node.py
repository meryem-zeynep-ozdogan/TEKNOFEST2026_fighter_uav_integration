#!/usr/bin/env python3
"""
================================================================================
TEKNOFEST SAVAŞAN İHA - GPS TAKİP NODE'U
================================================================================
L1 Adaptive Guidance ile Gelişmiş Hedef Takip Sistemi


Bu node şu görevleri yerine getirir:
1. PX4 ile ROS2 arasında köprü görevi görür
2. Kendi uçak durumunu hedef verilerini alır
3. L1 Guidance modülünü çağırarak takip komutu üretir
4. PX4 Autopilot'a TrajectorySetpoint komutları gönderir

Separation of Concerns:
- Tüm matematik ve guidance mantığı: l1_guidance.py (ROS2 bağımsız)
- Bu dosya: Sadece ROS2 entegrasyonu (subscriber, publisher, callback)

Kullanım:
  ros2 run teknofest_control gps_tracking_node --ros-args --params-file config/tracking_params.yaml
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from typing import Dict
from collections import deque
import math
import json

# ============================================================================
# L1 GUIDANCE MODÜLÜ (ROS2 Bağımsız)
# ============================================================================
from teknofest_control.l1_guidance import (
    # Ana sınıflar
    L1Guidance,
    
    # Veri yapıları
    AircraftState,
    GuidanceCommand,
    TrackingState,
    
    # Parametre sınıfları
    L1GuidanceParams,
    TargetSelectorParams,
    StateMachineParams,
    
    # Yardımcı fonksiyonlar
    geodetic_to_ned,
    calculate_distance_3d,
)

# ============================================================================
# PX4 MESAJ TİPLERİ
# ============================================================================
try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleGlobalPosition,
        VehicleAttitude,
        VehicleStatus,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleCommand
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False
    print("[UYARI] px4_msgs paketi bulunamadı. Simülasyon modunda çalışılacak.")

# Standart ROS2 mesajları
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped


# ============================================================================
# ANA GPS TAKİP NODE'U
# ============================================================================

class GPSTrackingNode(Node):
    """
    TEKNOFEST Savaşan İHA - GPS Takip Node'u
    
    Bu node ROS2 ile L1 Guidance modülü arasında köprü görevi görür:
    1. PX4'ten kendi uçak durumunu alır (VehicleLocalPosition, VehicleGlobalPosition)
    2. Mock server/yarışma sunucusundan hedef listesini alır
    3. L1Guidance modülünü çağırarak takip komutu üretir
    4. PX4'e TrajectorySetpoint gönderir
    
    Separation of Concerns:
    - Tüm matematik ve guidance mantığı: l1_guidance.py modülünde
    - Bu sınıf: Sadece ROS2 subscriber/publisher yönetimi
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
            durability=DurabilityPolicy.VOLATILE,  # PX4 için VOLATILE olmalı
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
        
        # Referans noktası (home position)
        self.home_lat: float = 0.0
        self.home_lon: float = 0.0
        self.home_alt: float = 0.0
        self.home_set: bool = False
        
        # Offboard mod kontrolü
        self.offboard_setpoint_counter: int = 0  # PX4 en az 100 setpoint bekler
        self.offboard_mode_active: bool = False
        self.armed: bool = False
        self.vehicle_status_received: bool = False
        
        # ====================================================================
        # L1 GUIDANCE MODÜLÜ
        # ====================================================================
        self._init_l1_guidance()
        
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
        
        # Debug: publish_command iterasyon sayacı (rate-limited loglama için)
        self._publish_counter: int = 0
        
        # Debug: Hedef callback sayacı
        self._target_local_callback_count: int = 0
        self._target_global_callback_count: int = 0
        
        # Veri alım durumu bayrakları
        self._local_pos_received: bool = False
        self._global_pos_received: bool = False
        
        # Durum takibi (control_loop_callback için)
        self._last_state = TrackingState.IDLE
        self._last_lock_reported: int = -1
        
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
        
        # Bağlantı durumu kontrolü (başlangıçta)
        self.connection_check_timer = self.create_timer(
            5.0,  # 5 saniyede bir
            self._check_px4_connection
        )
        
        self.get_logger().info("GPS Takip Node'u başarıyla başlatıldı!")
        self.get_logger().info(f"Kontrol frekansı: {self.control_frequency} Hz")
        self.get_logger().info(f"L1 Mesafesi: {self.l1_distance} m")
        self.get_logger().info("📡 PX4 bağlantısı bekleniyor...")
    
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
        self.declare_parameter('l1_guidance.l1_distance', 0.0)
        self.declare_parameter('l1_guidance.l1_damping', 0.85)
        self.declare_parameter('l1_guidance.l1_period', 10.0)
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
        self.declare_parameter('state_machine.loiter_trigger_distance', 5.0)
        self.declare_parameter('state_machine.lock_confirmation_time', 4.0)
        
        # PX4
        self.declare_parameter('px4.control_frequency', 50.0)
        
        # Güvenlik
        self.declare_parameter('safety.min_altitude_agl', 50.0)
        self.declare_parameter('safety.collision_avoidance_distance', 15.0)
        
        # Namespace ayarları (Multi-vehicle PX4 SITL için)
        self.declare_parameter('namespaces.own', '')           # Kendi uçağımız (boş = varsayılan)
        self.declare_parameter('namespaces.targets', '/px4_1,/px4_2')  # Hedef uçaklar (virgülle ayrılmış)
    
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
        
        # Namespace ayarları
        self.own_namespace = self.get_parameter('namespaces.own').value
        targets_str = self.get_parameter('namespaces.targets').value
        
        # Virgülle ayrılmış namespace'leri parse et
        self.target_namespaces = [ns.strip() for ns in targets_str.split(',') if ns.strip()]
        
        self.get_logger().info(f"Namespace ayarları: own='{self.own_namespace}'")
        self.get_logger().info(f"Hedef namespace'ler: {self.target_namespaces}")
    
    def _init_l1_guidance(self):
        """L1 Guidance modülünü başlat"""
        
        # L1 Guidance parametreleri oluştur
        l1_params = L1GuidanceParams(
            l1_distance=self.l1_distance,
            l1_damping=self.l1_damping,
            l1_period=self.l1_period,
            adaptive_l1=self.adaptive_l1,
            min_airspeed=self.min_airspeed,
            max_airspeed=self.max_airspeed,
            cruise_airspeed=self.cruise_airspeed,
            max_bank_angle=self.max_bank,
            loiter_radius=self.loiter_radius
        )
        
        # Hedef seçim parametreleri
        selector_params = TargetSelectorParams(
            w_distance=self.w_distance,
            w_angle=self.w_angle,
            w_speed=self.w_speed,
            tail_bonus=self.tail_bonus,
            head_on_penalty=self.head_on_penalty,
            tail_cone=self.tail_cone,
            head_on_cone=self.head_on_cone
        )
        
        # Durum makinesi parametreleri
        state_params = StateMachineParams(
            lock_distance=self.lock_distance,
            approach_distance=self.approach_distance,
            loiter_trigger=self.loiter_trigger,
            lock_confirm_time=self.lock_confirm_time
        )
        
        # Ana L1 Guidance objesi - Tüm mantık burada
        self.l1_guidance = L1Guidance(
            l1_params=l1_params,
            selector_params=selector_params,
            state_params=state_params,
            sample_rate=self.control_frequency
        )
        
        self.get_logger().info("L1 Guidance modülü başlatıldı")
        self.get_logger().info(f"  L1 Distance: {self.l1_distance}m")
        self.get_logger().info(f"  Adaptive L1: {self.adaptive_l1}")
        self.get_logger().info(f"  Cruise Airspeed: {self.cruise_airspeed}m/s")
    
    # ========================================================================
    # YARDİMCI FONKSİYONLAR
    # ========================================================================
    
    def _get_topic(self, base_topic: str) -> str:
        """
        Kendi uçağımız için namespace-aware topic ismi oluştur
        
        Args:
            base_topic: 'fmu/out/vehicle_local_position' gibi (başında / olmadan)
        
        Returns:
            '/fmu/out/...' veya namespace varsa '/px4_X/fmu/out/...'
        """
        if self.own_namespace:
            # Namespace varsa ekle (başında / varsa temizle)
            clean_ns = self.own_namespace.lstrip('/')
            return f"/{clean_ns}/{base_topic}"
        else:
            # Namespace yoksa direkt kullan
            return f"/{base_topic}"
    
    # ========================================================================
    # SUBSCRIBER'LAR
    # ========================================================================
    
    def _create_subscribers(self):
        """Tüm subscriber'ları oluştur"""
        
        if PX4_MSGS_AVAILABLE:
            # PX4 Lokal Pozisyon (v1 format - yeni PX4)
            self.local_pos_sub = self.create_subscription(
                VehicleLocalPosition,
                self._get_topic('fmu/out/vehicle_local_position'),
                self.vehicle_local_position_callback,
                self.px4_qos
            )
            
            # Yeni PX4 versiyonları için _v1 suffix'li topic'e de abone ol
            self.local_pos_v1_sub = self.create_subscription(
                VehicleLocalPosition,
                self._get_topic('fmu/out/vehicle_local_position_v1'),
                self.vehicle_local_position_callback,
                self.px4_qos
            )
            
            # PX4 Global Pozisyon (GPS)
            self.global_pos_sub = self.create_subscription(
                VehicleGlobalPosition,
                self._get_topic('fmu/out/vehicle_global_position'),
                self.vehicle_global_position_callback,
                self.px4_qos
            )
            
            # PX4 Attitude
            self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                self._get_topic('fmu/out/vehicle_attitude'),
                self.vehicle_attitude_callback,
                self.px4_qos
            )
            
            # PX4 Status (v1 format)
            self.status_sub = self.create_subscription(
                VehicleStatus,
                self._get_topic('fmu/out/vehicle_status'),
                self.vehicle_status_callback,
                self.px4_qos
            )
            
            # Yeni PX4 versiyonları için _v1 suffix'li topic
            self.status_v1_sub = self.create_subscription(
                VehicleStatus,
                self._get_topic('fmu/out/vehicle_status_v1'),
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
        
        # ================================================================
        # HEDEF UÇAK SUBSCRIBER'LARI (PX4 SITL Multi-Vehicle)
        # ================================================================
        # Her hedef namespace için subscriber oluştur
        self.target_subscribers = []  # Subscriber referanslarını sakla
        
        if PX4_MSGS_AVAILABLE and self.target_namespaces:
            for idx, target_ns in enumerate(self.target_namespaces):
                target_id = idx + 1  # ID: 1, 2, 3, ...
                
                # Hedef namespace için topic oluştur
                def make_topic(base_topic, ns):
                    if ns:
                        ns = ns.rstrip('/')
                        return f"{ns}/{base_topic}"
                    return f"/{base_topic}"
                
                local_topic = make_topic('fmu/out/vehicle_local_position', target_ns)
                local_topic_v1 = make_topic('fmu/out/vehicle_local_position_v1', target_ns)
                global_topic = make_topic('fmu/out/vehicle_global_position', target_ns)
                global_topic_v1 = make_topic('fmu/out/vehicle_global_position_v1', target_ns)
                
                # Closure için lambda kullan - target_id'yi yakala
                def make_local_callback(tid, tns):
                    def callback(msg):
                        self._target_local_callback(msg, tid, tns)
                    return callback
                
                def make_global_callback(tid, tns):
                    def callback(msg):
                        self._target_global_callback(msg, tid, tns)
                    return callback
                
                # Subscriber'ları oluştur
                sub1 = self.create_subscription(
                    VehicleLocalPosition, local_topic,
                    make_local_callback(target_id, target_ns), self.px4_qos)
                
                sub2 = self.create_subscription(
                    VehicleLocalPosition, local_topic_v1,
                    make_local_callback(target_id, target_ns), self.px4_qos)
                
                sub3 = self.create_subscription(
                    VehicleGlobalPosition, global_topic,
                    make_global_callback(target_id, target_ns), self.px4_qos)
                
                sub4 = self.create_subscription(
                    VehicleGlobalPosition, global_topic_v1,
                    make_global_callback(target_id, target_ns), self.px4_qos)
                
                self.target_subscribers.extend([sub1, sub2, sub3, sub4])
                
                self.get_logger().info(f"🎯 Hedef {target_id} subscriber'ları oluşturuldu:")
                self.get_logger().info(f"   Namespace: '{target_ns}'")
                self.get_logger().info(f"   Local: {local_topic_v1}")
                self.get_logger().info(f"   Global: {global_topic_v1}")
        
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
                self._get_topic('fmu/in/trajectory_setpoint'),
                self.px4_qos
            )
            
            # PX4 OffboardControlMode
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode,
                self._get_topic('fmu/in/offboard_control_mode'),
                self.px4_qos
            )
            
            # PX4 VehicleCommand
            self.command_pub = self.create_publisher(
                VehicleCommand,
                self._get_topic('fmu/in/vehicle_command'),
                self.px4_qos
            )
            
            self.get_logger().info("📤 Publisher'lar oluşturuldu:")
            self.get_logger().info(f"   Trajectory: {self._get_topic('fmu/in/trajectory_setpoint')}")
            self.get_logger().info(f"   Offboard: {self._get_topic('fmu/in/offboard_control_mode')}")
        
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
        
        # YENİ: Hedef skor bilgileri
        self.target_scores_pub = self.create_publisher(
            String,
            '/tracking/target_scores',
            self.standard_qos
        )
        
        # YENİ: Kamera FOV durumu
        self.camera_fov_pub = self.create_publisher(
            String,
            '/tracking/camera_fov_status',
            self.standard_qos
        )
        
        # YENİ: En iyi hedef bilgisi
        self.best_target_pub = self.create_publisher(
            String,
            '/tracking/best_target',
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
        
        # Ground speed hesapla
        self.own_state.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)
        
        # Home pozisyonunu ayarla - birden fazla yöntem dene
        if not self.home_set:
            # Yöntem 1: xy_global ve ref koordinatları varsa
            if hasattr(msg, 'xy_global') and msg.xy_global:
                if hasattr(msg, 'ref_lat') and msg.ref_lat != 0.0:
                    self.home_lat = msg.ref_lat
                    self.home_lon = msg.ref_lon
                    self.home_alt = msg.ref_alt if hasattr(msg, 'ref_alt') else 0.0
                    self.home_set = True
                    self.get_logger().info(f"✅ Home pozisyonu ayarlandı (ref): {self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m")
            
            # Yöntem 2: Global position callback'ten gelirse (fallback)
            # vehicle_global_position_callback içinde ayarlanacak
        
        # Debug: İlk veri geldiğinde logla
        if not self._local_pos_received:
            self._local_pos_received = True
            self.get_logger().info(f"📍 Local position verisi alındı: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}")
            self.get_logger().info(f"   xy_global={getattr(msg, 'xy_global', 'N/A')}, z_global={getattr(msg, 'z_global', 'N/A')}")
    
    def vehicle_global_position_callback(self, msg):
        """
        PX4'ten global GPS pozisyon verisi al
        """
        self.own_state.latitude = msg.lat
        self.own_state.longitude = msg.lon
        self.own_state.altitude = msg.alt
        self.own_state.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Fallback: Home pozisyonu hala set edilmemişse ve geçerli GPS varsa
        if not self.home_set and msg.lat != 0.0 and msg.lon != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.home_set = True
            self.get_logger().info(f"✅ Home pozisyonu ayarlandı (GPS): {self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m")
        
        # Debug: İlk veri geldiğinde logla
        if not self._global_pos_received:
            self._global_pos_received = True
            self.get_logger().info(f"🌍 Global position verisi alındı: lat={msg.lat:.6f}, lon={msg.lon:.6f}, alt={msg.alt:.1f}m")
    
    def _target_local_callback(self, msg, target_id: int, target_ns: str):
        """
        Hedef uçaktan lokal pozisyon verisi al (PX4 SITL multi-vehicle)
        
        KRİTİK NOT: Sadece HIZ verisi kullanılır!
        x,y,z pozisyon değerleri hedefin KENDİ NED frame'indedir,
        bizim NED frame'imizde DEĞİLDİR. Multi-vehicle SITL'de her uçağın
        kendi home noktası vardır. Pozisyon verisi SADECE
        _target_global_callback üzerinden geodetic_to_ned
        dönüşümüyle alınmalıdır.
        
        Hız vektörü (vx,vy,vz) ise NED yönleri her yerde aynı olduğu için
        frame'den bağımsızdır ve burada güvenle kullanılabilir.
        
        Args:
            msg: VehicleLocalPosition mesajı
            target_id: Hedef ID (1, 2, 3, ...)
            target_ns: Hedef namespace ('/px4_1', '/px4_2', ...)
        """
        self._target_local_callback_count += 1
        
        # Target listesinde yoksa oluştur
        if target_id not in self.target_list:
            self.target_list[target_id] = AircraftState(id=target_id)
        
        target = self.target_list[target_id]
        
        # ❌ YAPILMAMALI: target.x, target.y, target.z = msg.x, msg.y, msg.z
        #    Bunlar hedefin KENDİ NED frame'inde, bizim frame'imizde değil!
        
        # ✅ Sadece hız verisi (NED yönleri evrensel, frame bağımsız)
        target.vx = msg.vx
        target.vy = msg.vy
        target.vz = msg.vz
        target.heading = math.degrees(msg.heading)
        target.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)
        target.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Debug: Her 100 mesajda bir logla (50-100Hz @ ~2 saniye)
        if self._target_local_callback_count % 200 == 1:
            self.get_logger().info(
                f"📡 Hedef {target_id} ({target_ns}) velocity: "
                f"vx={msg.vx:.1f}, vy={msg.vy:.1f}, hdg={math.degrees(msg.heading):.1f}°"
            )
    
    def _target_global_callback(self, msg, target_id: int, target_ns: str):
        """
        Hedef uçaktan global GPS pozisyon verisi al (PX4 SITL multi-vehicle)
        GPS koordinatlarını bizim NED frame'imize çevirir.
        
        Args:
            msg: VehicleGlobalPosition mesajı
            target_id: Hedef ID (1, 2, 3, ...)
            target_ns: Hedef namespace ('/px4_1', '/px4_2', ...)
        """
        self._target_global_callback_count += 1
        
        # Target listesinde yoksa oluştur
        if target_id not in self.target_list:
            self.target_list[target_id] = AircraftState(id=target_id)
        
        target = self.target_list[target_id]
        target.latitude = msg.lat
        target.longitude = msg.lon
        target.altitude = msg.alt
        target.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # NED koordinatlarına çevir (home set edilmişse)
        if self.home_set:
            n, e, d = geodetic_to_ned(
                target.latitude, target.longitude, target.altitude,
                self.home_lat, self.home_lon, self.home_alt
            )
            target.x = n
            target.y = e
            target.z = d
        
        # Debug: Her 20 mesajda bir logla
        if self._target_global_callback_count % 40 == 1:
            self.get_logger().info(
                f"🌍 Hedef {target_id} ({target_ns}): "
                f"lat={msg.lat:.6f}, lon={msg.lon:.6f} → NED=[{target.x:.1f}, {target.y:.1f}]"
            )
    
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
        """PX4 durum bilgisi - arming ve mod kontrolü için"""
        self.vehicle_status_received = True
        self.armed = msg.arming_state == 2  # ARMING_STATE_ARMED = 2
        
        # Nav state kontrolü - NAVIGATION_STATE_OFFBOARD = 14
        if msg.nav_state == 14:
            self.offboard_mode_active = True
        else:
            self.offboard_mode_active = False
    
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
        Ana kontrol döngüsü - L1Guidance modülünü çağırır
        
        Bu callback sadece:
        1. Home pozisyonu ve offboard mod kontrolü
        2. Eski hedefleri temizler
        3. L1Guidance.update() çağırır
        4. Komutu PX4'e yayınlar
        
        Tüm hedef seçimi, durum makinesi ve guidance mantığı
        l1_guidance.py modülündedir.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ----------------------------------------------------------------
        # 0. ÖN KOŞUL KONTROLLERİ
        # ----------------------------------------------------------------
        # Home pozisyonu ayarlanmadan NED dönüşümleri yapılamaz
        # AMA offboard için setpoint göndermeye devam etmeliyiz!
        if not self.home_set:
            if self.offboard_setpoint_counter % 50 == 0:
                self.get_logger().warn("⏳ Home pozisyonu bekleniyor... (setpoint gönderiliyor)")
            # Home olmadan da dummy setpoint gönder (offboard geçişi için gerekli)
            self._publish_safe_loiter_command()
            return
        
        # Offboard mod aktif değilse sadece loiter/safe modda bekle
        if not self.offboard_mode_active and self.offboard_setpoint_counter > 200:
            if self.offboard_setpoint_counter % 100 == 0:
                self.get_logger().warn("⚠️ Offboard mod aktif değil, yeniden deneniyor...")
                self._engage_offboard_mode()
        
        # ----------------------------------------------------------------
        # 1. ESKİ HEDEFLERİ TEMİZLE (timeout)
        # ----------------------------------------------------------------
        self._cleanup_stale_targets(current_time)
        
        # ----------------------------------------------------------------
        # 2. L1 GUIDANCE MODÜLÜNÜ ÇAĞIR
        # ----------------------------------------------------------------
        # L1Guidance.update() tüm işi yapar:
        # - Hedef seçimi (weighted scoring)
        # - Durum makinesi güncelleme
        # - L1 guidance hesaplama
        # - Komut smoothing
        command = self.l1_guidance.update(
            own_state=self.own_state,
            targets=self.target_list,
            current_time=current_time
        )
        
        # ----------------------------------------------------------------
        # 3. KOMUT YAYINLA
        # ----------------------------------------------------------------
        if command.is_valid:
            self.publish_command(command)
            self._publish_virtual_target()
        else:
            # Hedef yoksa güvenli loiter komutu gönder
            self._publish_safe_loiter_command()
        
        # ----------------------------------------------------------------
        # 4. İSTATİSTİK GÜNCELLE
        # ----------------------------------------------------------------
        self.stats['targets_evaluated'] = len(self.target_list)
        
        current_state = self.l1_guidance.get_state()
        if current_state == TrackingState.LOCKED:
            lock_duration = self.l1_guidance.get_lock_duration(current_time)
            self.stats['current_lock_duration'] = lock_duration
            
            if self.l1_guidance.is_lock_confirmed():
                if not hasattr(self, '_last_lock_reported') or \
                   self._last_lock_reported != self.l1_guidance.get_locked_target_id():
                    self._last_lock_reported = self.l1_guidance.get_locked_target_id()
                    self.stats['successful_locks'] += 1
                    self.get_logger().info(
                        f"✓ KİLİTLENME ONAYLANDI! Hedef: {self._last_lock_reported}, "
                        f"Süre: {lock_duration:.2f}s"
                    )
        
        # Durum değişikliği logla
        if not hasattr(self, '_last_state'):
            self._last_state = TrackingState.IDLE
        
        if current_state != self._last_state:
            self.get_logger().info(
                f"Durum değişti: {self._last_state.name} -> {current_state.name}"
            )
            self._last_state = current_state
            self.stats['state_transitions'] += 1
            
            # Durumu ROS2 topic'ine yayınla
            state_msg = String()
            state_msg.data = current_state.name
            self.state_pub.publish(state_msg)
        
        # ----------------------------------------------------------------
        # 5. HEDEF SKOR BİLGİLERİNİ YAYINLA (YENİ)
        # ----------------------------------------------------------------
        self._publish_target_scores()
    
    def _publish_target_scores(self):
        """
        Tüm hedeflerin skor bilgilerini ROS2 topic'lerine yayınla
        Bu bilgiler debug ve görselleştirme için kullanılır
        """
        # 1. Tüm hedef skorlarını JSON olarak yayınla
        all_scores = self.l1_guidance.get_all_target_scores()
        if all_scores:
            scores_data = []
            for score in all_scores:
                scores_data.append({
                    'id': score.target_id,
                    'total_score': round(score.total_score, 3),
                    'distance': round(score.distance, 1),
                    'distance_score': round(score.distance_score, 3),
                    'angle_score': round(score.angle_score, 3),
                    'speed_score': round(score.speed_score, 3),
                    'camera_fov_score': round(score.camera_fov_score, 3),
                    'reachability_score': round(score.reachability_score, 3),
                    'is_in_fov': score.is_in_camera_fov,
                    'is_tail': score.is_tail_position,
                    'intercept_time': round(score.estimated_intercept_time, 1) if score.estimated_intercept_time < 1000 else 'inf'
                })
            
            scores_msg = String()
            scores_msg.data = json.dumps(scores_data, indent=2)
            self.target_scores_pub.publish(scores_msg)
        
        # 2. En iyi hedef bilgisini yayınla
        target_score = self.l1_guidance.get_target_score()
        if target_score:
            best_msg = String()
            best_msg.data = self.l1_guidance.get_score_breakdown_str()
            self.best_target_pub.publish(best_msg)
            
            # 3. Kamera FOV durumunu yayınla
            fov_msg = String()
            in_fov = self.l1_guidance.is_target_in_camera_fov()
            intercept_time = self.l1_guidance.get_estimated_intercept_time()
            fov_msg.data = json.dumps({
                'target_id': target_score.target_id,
                'in_camera_fov': in_fov,
                'heading_diff': round(target_score.heading_diff, 1),
                'distance': round(target_score.distance, 1),
                'intercept_time': round(intercept_time, 1) if intercept_time < 1000 else 'inf',
                'is_tail_position': target_score.is_tail_position
            })
            self.camera_fov_pub.publish(fov_msg)
    
    def _cleanup_stale_targets(self, current_time: float):
        """Timeout olan hedefleri temizle"""
        stale_ids = []
        for target_id, target in self.target_list.items():
            if current_time - target.timestamp > 5.0:  # 5 saniye timeout
                stale_ids.append(target_id)
        
        for target_id in stale_ids:
            del self.target_list[target_id]
            self.get_logger().warn(f"Hedef zaman aşımı: {target_id}")
    
    # ========================================================================
    # KOMUT YAYINLAMA
    # ========================================================================
    
    def publish_command(self, command: GuidanceCommand):
        """
        PX4'e TrajectorySetpoint yayınla (Fixed-Wing Position Modu)
        
        GÜNCELLEME: Velocity modu yerine Position modu kullanılıyor.
        Velocity modunda yönelim (heading) sorunları yaşandığı için
        L1 algoritmasının ürettiği sanal hedef noktasına (Virtual Target)
        doğrudan gitmesi söyleniyor. PX4 kendi navigasyonunu yapacak.
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        self._publish_counter += 1
        
        # TrajectorySetpoint mesajı oluştur
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Position Setpoint (L1 Virtual Target)
        # NaN kontrolü yaparak gönder
        if not math.isnan(command.position_north) and \
           not math.isnan(command.position_east) and \
           not math.isnan(command.position_down):
               
            msg.position[0] = command.position_north
            msg.position[1] = command.position_east
            msg.position[2] = command.position_down
        else:
            # Fallback (Valid pozisyon yoksa)
            msg.position[0] = float('nan')
            msg.position[1] = float('nan')
            msg.position[2] = float('nan')
            
        # Velocity ve Acceleration NaN (PX4 hesaplasın)
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]

        # Yaw setpoint
        # Position modunda Yaw genellikle NaN bırakılır (Auto-heading)
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        
        self.trajectory_pub.publish(msg)
        
        # ============================================================
        # DEBUG LOG - Saniyede 1 kez (her 50 iterasyonda, @50Hz)
        # ============================================================
        if self._publish_counter % 50 == 0:
            dist = -1.0
            tgt_info = "yok"
            if self.l1_guidance.locked_target is not None:
                own_pos = self.own_state.get_position_ned()
                target_pos = self.l1_guidance.locked_target.get_position_ned()
                dist = calculate_distance_3d(own_pos, target_pos)
                tgt_info = f"N={target_pos[0]:.1f} E={target_pos[1]:.1f} D={target_pos[2]:.1f}"
            
            own_info = f"N={self.own_state.x:.1f} E={self.own_state.y:.1f} D={self.own_state.z:.1f}"
            
            # Position command logla
            self.get_logger().info(
                f"[CMD] PosSet=[N={msg.position[0]:.1f} E={msg.position[1]:.1f} D={msg.position[2]:.1f}] | "
                f"Dist={dist:.1f}m"
            )
            self.get_logger().info(
                f"[POS] Own=[{own_info}] Tgt=[{tgt_info}] | "
                f"State={self.l1_guidance.get_state().name}"
            )
        
        # Komut geçmişine ekle (Debug için)
        self.command_history.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'pos_n': msg.position[0],
            'pos_e': msg.position[1],
            'pos_d': msg.position[2]
        })
    
    def offboard_heartbeat_callback(self):
        """
        PX4 Offboard heartbeat - sürekli gönderilmeli
        Aksi halde PX4 otomatik olarak manuel moda geçer
        
        Fixed-wing için Position Control Modu kullanılır.
        Velocity modu yerine Position modu seçildi (daha stabil yönelim için).
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Position Control Modu (L1 Virtual Target için)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_mode_pub.publish(msg)
        
        # Setpoint sayacını artır
        self.offboard_setpoint_counter += 1
        
        # İlk 150 setpoint'ten sonra offboard moda geç ve arm et
        if self.offboard_setpoint_counter == 150:
            self.get_logger().info("🚀 Offboard mod geçişi başlatılıyor...")
            self._engage_offboard_mode()
    
    def _engage_offboard_mode(self):
        """
        PX4'ü offboard moduna geçir ve arm et.
        Bu fonksiyon yeterli sayıda setpoint gönderildikten sonra çağrılmalı.
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        # 1. Offboard mod komutu gönder
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode
            param2=6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )
        self.get_logger().info("📡 Offboard mod komutu gönderildi")
        
        # 2. ARM komutu gönder (zaten armed değilse)
        if not self.armed:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0  # 1.0 = ARM, 0.0 = DISARM
            )
            self.get_logger().info("🔓 ARM komutu gönderildi")
    
    def _send_vehicle_command(self, command: int, param1: float = 0.0, 
                               param2: float = 0.0, param3: float = 0.0,
                               param4: float = 0.0, param5: float = 0.0,
                               param6: float = 0.0, param7: float = 0.0):
        """
        PX4'e VehicleCommand gönder
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.command_pub.publish(msg)
    
    def _publish_safe_loiter_command(self):
        """
        Hedef yokken güvenli loiter komutu gönder.
        Fixed-wing için belirli bir hızda düz uçuş yapar.
        Home pozisyonu olmadan da çalışır (offboard geçişi için).
        """
        if not PX4_MSGS_AVAILABLE:
            return
        
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
         # Mevcut heading'de düz uçuş (veya varsayılan kuzey yönü)
        if self.own_state.heading != 0.0 or self.home_set:
            current_heading_rad = math.radians(self.own_state.heading)
        else:
            current_heading_rad = 0.0  # Varsayılan: Kuzey yönü
        
        # Position NaN (velocity modunda kullanılmaz)
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        
       
        # Sabit hızda ileri uçuş
        msg.velocity[0] = self.cruise_airspeed * math.cos(current_heading_rad)  
        msg.velocity[1] = self.cruise_airspeed * math.sin(current_heading_rad)  
        msg.velocity[2] = 0.0  # İrtifa koru
        
        msg.yaw = current_heading_rad
        msg.yawspeed = float('nan')
        
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        
        self.trajectory_pub.publish(msg)
    
    def _publish_virtual_target(self):
        """
        Sanal hedef noktasını görselleştirme için yayınla
        L1 Guidance modülünden sanal hedefi al
        """
        virtual_target = self.l1_guidance.get_virtual_target()
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = virtual_target[0]
        msg.pose.position.y = virtual_target[1]
        msg.pose.position.z = -virtual_target[2]  # NED -> ENU
        
        self.virtual_target_pub.publish(msg)
    
    # ========================================================================
    # DURUM RAPORU
    # ========================================================================
    
    def status_report_callback(self):
        """Periyodik durum raporu - L1 Guidance modülünden bilgi al"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        current_state = self.l1_guidance.get_state()
        locked_target_id = self.l1_guidance.get_locked_target_id()
        
        debug_info = {
            'state': current_state.name,
            'locked_target': locked_target_id,
            'targets_count': len(self.target_list),
            'lock_confirmed': self.l1_guidance.is_lock_confirmed(),
            'lock_duration': self.l1_guidance.get_lock_duration(current_time),
            'successful_locks': self.stats['successful_locks'],
            'cross_track_error': round(self.l1_guidance.get_cross_track_error(), 2)
        }
        
        # Hedefe mesafe bilgisi
        if locked_target_id >= 0 and locked_target_id in self.target_list:
            own_pos = self.own_state.get_position_ned()
            target = self.target_list[locked_target_id]
            target_pos = target.get_position_ned()
            distance = calculate_distance_3d(own_pos, target_pos)
            debug_info['distance_to_target'] = round(distance, 1)
        
        msg = String()
        msg.data = str(debug_info)
        self.debug_pub.publish(msg)
    
    def _check_px4_connection(self):
        """
        PX4 bağlantı durumunu kontrol et ve kullanıcıya bilgi ver
        """
        status_parts = []
        
        # Local position verisi alındı mı?
        local_pos_ok = hasattr(self, '_local_pos_received') and self._local_pos_received
        status_parts.append(f"LocalPos: {'✓' if local_pos_ok else '✗'}")
        
        # Global position verisi alındı mı?
        global_pos_ok = hasattr(self, '_global_pos_received') and self._global_pos_received
        status_parts.append(f"GlobalPos: {'✓' if global_pos_ok else '✗'}")
        
        # Home set mi?
        status_parts.append(f"Home: {'✓' if self.home_set else '✗'}")
        
        # Vehicle status alındı mı?
        status_parts.append(f"Status: {'✓' if self.vehicle_status_received else '✗'}")
        
        # Offboard mod aktif mi?
        status_parts.append(f"Offboard: {'✓' if self.offboard_mode_active else '✗'}")
        
        # Armed mı?
        status_parts.append(f"Armed: {'✓' if self.armed else '✗'}")
        
        # Setpoint sayısı
        status_parts.append(f"Setpoints: {self.offboard_setpoint_counter}")
        
        status_line = " | ".join(status_parts)
        
        if not local_pos_ok or not global_pos_ok:
            self.get_logger().warn(f"📡 PX4 Bağlantı: {status_line}")
            self.get_logger().warn("   ⚠️  PX4 SITL çalışıyor mu? Topic'ler:")
            self.get_logger().warn("      /fmu/out/vehicle_local_position veya /fmu/out/vehicle_local_position_v1")
            self.get_logger().warn("      /fmu/out/vehicle_global_position")
        elif not self.home_set:
            self.get_logger().info(f"📡 PX4 Bağlantı: {status_line}")
            self.get_logger().warn("   ⚠️  GPS fix bekleniyor (home pozisyonu için)")
        elif not self.offboard_mode_active:
            self.get_logger().info(f"📡 PX4 Bağlantı: {status_line}")
            self.get_logger().info("   ℹ️  Offboard mod için QGroundControl'den manuel geçiş yapın veya bekleyin")
        else:
            self.get_logger().info(f"📡 PX4 Bağlantı: {status_line}")
            self.get_logger().info("   ✅ Sistem hazır - hedef takibi aktif")


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
    except (RuntimeError, ValueError) as e:
        print(f"Hata: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
