#!/usr/bin/env python3
"""
PX4 → Mock Sunucu Köprüsü
=========================
PX4 SITL'deki rakip İHA'ların GPS verilerini alır ve Mock sunucuya iletir.

Veri Akışı:
  PX4 (rc_cessna_1, rc_cessna_2, ...) 
    → ROS2 Topics (/px4_X/fmu/out/vehicle_global_position)
    → Bu Node
    → WebSocket (external_telemetry event)
    → Mock Sunucu (ws_server.py)
    → telemetry event olarak yayınlar

Kullanım:
  ros2 run teknofest_control px4_to_mock_bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import socketio
import time
import math
from typing import Dict, List
from dataclasses import dataclass, field

# PX4 mesaj tipleri
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, VehicleAttitude


@dataclass
class UAVState:
    """Tek bir İHA'nın durumu"""
    id: str
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    last_update: float = 0.0


class PX4ToMockBridge(Node):
    """PX4 GPS verilerini Mock sunucuya ileten ROS2 node'u"""
    
    def __init__(self):
        super().__init__('px4_to_mock_bridge')
        
        # Parametreler
        self.declare_parameter('mock_server_url', 'http://localhost:8000')
        self.declare_parameter('enemy_instance_ids', [1, 2])  # Rakip İHA instance ID'leri
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('auto_reconnect', True)
        
        self.mock_server_url = self.get_parameter('mock_server_url').value
        self.enemy_ids: List[int] = list(self.get_parameter('enemy_instance_ids').value)
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        
        # İHA durumları
        self.uav_states: Dict[int, UAVState] = {}
        for eid in self.enemy_ids:
            self.uav_states[eid] = UAVState(id=f"rakip_{eid}")
        
        # QoS profili (PX4 uyumlu)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Her rakip İHA için subscriber oluştur
        self.subscriptions_list = []
        for eid in self.enemy_ids:
            # Global position (GPS)
            topic_global = f'/px4_{eid}/fmu/out/vehicle_global_position'
            sub_global = self.create_subscription(
                VehicleGlobalPosition,
                topic_global,
                lambda msg, id=eid: self.global_position_callback(msg, id),
                qos
            )
            self.subscriptions_list.append(sub_global)
            self.get_logger().info(f'📡 Abone olundu: {topic_global}')
            
            # Local position (hız bilgisi için)
            topic_local = f'/px4_{eid}/fmu/out/vehicle_local_position'
            sub_local = self.create_subscription(
                VehicleLocalPosition,
                topic_local,
                lambda msg, id=eid: self.local_position_callback(msg, id),
                qos
            )
            self.subscriptions_list.append(sub_local)
            
            # Attitude (yaw için)
            topic_att = f'/px4_{eid}/fmu/out/vehicle_attitude'
            sub_att = self.create_subscription(
                VehicleAttitude,
                topic_att,
                lambda msg, id=eid: self.attitude_callback(msg, id),
                qos
            )
            self.subscriptions_list.append(sub_att)
        
        # Socket.IO client
        self.sio = socketio.Client(reconnection=self.auto_reconnect)
        self.connected = False
        self._setup_socketio_events()
        
        # Bağlantı dene
        self._connect_to_server()
        
        # Periyodik gönderim timer'ı
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.send_telemetry)
        
        self.get_logger().info(f'🚀 PX4 → Mock Bridge başlatıldı')
        self.get_logger().info(f'   Sunucu: {self.mock_server_url}')
        self.get_logger().info(f'   Rakip İHA\'lar: {self.enemy_ids}')
    
    def _setup_socketio_events(self):
        """Socket.IO event handler'larını ayarla"""
        
        @self.sio.event
        def connect():
            self.connected = True
            self.get_logger().info('✅ Mock sunucuya bağlandı!')
        
        @self.sio.event
        def disconnect():
            self.connected = False
            self.get_logger().warn('⚠️ Mock sunucu bağlantısı kesildi')
        
        @self.sio.event
        def connect_error(data):
            self.get_logger().error(f'❌ Bağlantı hatası: {data}')
    
    def _connect_to_server(self):
        """Mock sunucuya bağlan"""
        try:
            if not self.connected:
                self.get_logger().info(f'🔌 Bağlanılıyor: {self.mock_server_url}')
                self.sio.connect(self.mock_server_url, wait_timeout=5)
        except Exception as e:
            self.get_logger().warn(f'⚠️ Bağlantı başarısız: {e}')
    
    def global_position_callback(self, msg: VehicleGlobalPosition, instance_id: int):
        """GPS verisi geldiğinde"""
        if instance_id not in self.uav_states:
            return
        
        state = self.uav_states[instance_id]
        state.lat = msg.lat
        state.lon = msg.lon
        state.alt = msg.alt
        state.last_update = time.time()
    
    def local_position_callback(self, msg: VehicleLocalPosition, instance_id: int):
        """Local position (hız) verisi geldiğinde"""
        if instance_id not in self.uav_states:
            return
        
        state = self.uav_states[instance_id]
        state.vx = msg.vx
        state.vy = msg.vy
        state.vz = msg.vz
        state.speed = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)
    
    def attitude_callback(self, msg: VehicleAttitude, instance_id: int):
        """Attitude (yaw) verisi geldiğinde"""
        if instance_id not in self.uav_states:
            return
        
        # Quaternion'dan yaw hesapla
        q = msg.q
        # yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.uav_states[instance_id].yaw = math.degrees(yaw)
    
    def send_telemetry(self):
        """Telemetri verilerini mock sunucuya gönder"""
        if not self.connected:
            # Yeniden bağlanmayı dene
            if self.auto_reconnect:
                self._connect_to_server()
            return
        
        now = time.time()
        
        for instance_id, state in self.uav_states.items():
            # Son 2 saniye içinde güncelleme yoksa gönderme
            if now - state.last_update > 2.0:
                continue
            
            # Telemetri paketi oluştur
            telemetry = {
                'id': state.id,
                'lat': state.lat,
                'lon': state.lon,
                'alt': state.alt,
                'vx': state.vx,
                'vy': state.vy,
                'vz': state.vz,
                'yaw': state.yaw,
                'speed': state.speed,
                'timestamp': now,
                'status': 'ACTIVE'
            }
            
            try:
                self.sio.emit('external_telemetry', telemetry)
            except Exception as e:
                self.get_logger().error(f'Gönderim hatası: {e}')
                self.connected = False
    
    def destroy_node(self):
        """Node kapanırken bağlantıyı kapat"""
        if self.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = PX4ToMockBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
