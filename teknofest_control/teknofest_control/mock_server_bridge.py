#!/usr/bin/env python3
"""
================================================================================
TEKNOFEST - MOCK SERVER BRIDGE
================================================================================
Mock Server (WebSocket) → ROS2 Bridge

Mock server'dan gelen telemetri verilerini ROS2 topic'lerine çevirir.

Kullanım:
  ros2 run teknofest_control mock_server_bridge

Gereksinimler:
  pip install python-socketio[client]
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped

import socketio
import json
import threading
from typing import Dict
from dataclasses import dataclass
import time


@dataclass
class DroneState:
    """Drone durumu"""
    id: str
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    status: str = "NORMAL"
    timestamp: float = 0.0


class MockServerBridge(Node):
    """
    Mock Server WebSocket → ROS2 Bridge
    
    Mock server'dan gelen rakip uçak telemetrilerini
    GPS Tracking Node'un anlayacağı formata çevirir.
    """
    
    def __init__(self):
        super().__init__('mock_server_bridge')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("MOCK SERVER BRIDGE BAŞLATILIYOR")
        self.get_logger().info("=" * 60)
        
        # Parametreler
        self.declare_parameter('server_url', 'http://localhost:8080')
        self.declare_parameter('hero_id', 'bizim_iha')
        self.declare_parameter('publish_rate', 10.0)
        
        self.server_url = self.get_parameter('server_url').value
        self.hero_id = self.get_parameter('hero_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        # GPS Tracking Node'un dinlediği topic
        self.target_list_pub = self.create_publisher(
            Float64MultiArray,
            '/competition/target_list',
            qos
        )
        
        # JSON formatında da yayınla (debug için)
        self.json_pub = self.create_publisher(
            String,
            '/simulation/enemy_aircraft',
            qos
        )
        
        # Hero uçağımızın konumu (opsiyonel)
        self.hero_pose_pub = self.create_publisher(
            PoseStamped,
            '/hero/pose',
            qos
        )
        
        # Drone durumları
        self.drones: Dict[str, DroneState] = {}
        self.connected = False
        
        # Socket.IO client
        self.sio = socketio.Client(reconnection=True, reconnection_delay=1)
        self._setup_socket_events()
        
        # Bağlantı thread'i
        self.connect_thread = threading.Thread(target=self._connect_to_server, daemon=True)
        self.connect_thread.start()
        
        # Yayınlama timer'ı
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_targets
        )
        
        self.get_logger().info(f"Server URL: {self.server_url}")
        self.get_logger().info(f"Hero ID: {self.hero_id}")
    
    def _setup_socket_events(self):
        """Socket.IO event handler'larını ayarla"""
        
        @self.sio.event
        def connect():
            self.connected = True
            self.get_logger().info("✓ Mock Server'a bağlandı!")
            
            # Drone'ları başlat (opsiyonel - manuel başlatılabilir)
            # self._start_drones()
        
        @self.sio.event
        def disconnect():
            self.connected = False
            self.get_logger().warn("✗ Mock Server bağlantısı kesildi!")
        
        @self.sio.event
        def connect_error(err):
            self.get_logger().error(f"Bağlantı hatası: {err}")
        
        @self.sio.event
        def telemetry(data):
            """Mock server'dan telemetri al"""
            self._handle_telemetry(data)
        
        @self.sio.event
        def lock_response(data):
            """Lock yanıtı"""
            self.get_logger().info(f"Lock yanıtı: {data}")
    
    def _connect_to_server(self):
        """Server'a bağlan (ayrı thread'de)"""
        while rclpy.ok():
            if not self.connected:
                try:
                    self.get_logger().info(f"Bağlanılıyor: {self.server_url}")
                    self.sio.connect(self.server_url)
                except Exception as e:
                    self.get_logger().warn(f"Bağlantı başarısız: {e}")
                    time.sleep(2)
            else:
                time.sleep(1)
    
    def _handle_telemetry(self, data: dict):
        """
        Telemetri verisini işle
        
        Args:
            data: Mock server'dan gelen telemetri
                {
                    "id": "rakip_1",
                    "lat": 38.76450,
                    "lon": 30.52300,
                    "alt": 120,
                    ...
                }
        """
        drone_id = data.get('id', 'unknown')
        
        # Drone durumunu güncelle veya oluştur
        if drone_id not in self.drones:
            self.drones[drone_id] = DroneState(id=drone_id)
            self.get_logger().info(f"Yeni drone tespit edildi: {drone_id}")
        
        drone = self.drones[drone_id]
        drone.lat = data.get('lat', drone.lat)
        drone.lon = data.get('lon', drone.lon)
        drone.alt = data.get('alt', drone.alt)
        drone.vx = data.get('vx', 0.0)
        drone.vy = data.get('vy', 0.0)
        drone.vz = data.get('vz', 0.0)
        drone.yaw = data.get('yaw', 0.0)
        drone.speed = data.get('speed', 0.0)
        drone.status = data.get('status', 'NORMAL')
        drone.timestamp = data.get('timestamp', time.time())
        
        # Speed yoksa hesapla
        if drone.speed == 0 and (drone.vx != 0 or drone.vy != 0):
            import math
            drone.speed = math.sqrt(drone.vx**2 + drone.vy**2)
    
    def publish_targets(self):
        """
        Rakip drone'ları ROS2 topic'lerine yayınla
        """
        if not self.drones:
            return
        
        # Rakip listesi (hero hariç)
        targets = []
        for drone_id, drone in self.drones.items():
            if drone_id == self.hero_id:
                continue  # Hero'yu atla, o rakip değil
            
            targets.append({
                'id': drone_id,
                'lat': drone.lat,
                'lon': drone.lon,
                'alt': drone.alt,
                'speed': drone.speed,
                'heading': drone.yaw
            })
        
        if not targets:
            return
        
        # Float64MultiArray formatında yayınla
        # Format: [id, lat, lon, alt, speed, heading] * N
        msg = Float64MultiArray()
        data = []
        for i, t in enumerate(targets):
            # ID'yi sayıya çevir (rakip_1 -> 1, rakip_2 -> 2)
            try:
                numeric_id = int(''.join(filter(str.isdigit, t['id'])))
            except:
                numeric_id = i + 1
            
            data.extend([
                float(numeric_id),
                t['lat'],
                t['lon'],
                t['alt'],
                t['speed'],
                t['heading']
            ])
        
        msg.data = data
        self.target_list_pub.publish(msg)
        
        # JSON formatında da yayınla
        json_msg = String()
        json_msg.data = json.dumps({'aircraft': targets})
        self.json_pub.publish(json_msg)
    
    def start_drones(self):
        """
        Mock server'da drone'ları başlat
        Bu fonksiyon dışarıdan çağrılabilir
        """
        if not self.connected:
            self.get_logger().warn("Server'a bağlı değil!")
            return
        
        # Mock server'ın beklediği format
        start_data = {
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
        }
        
        self.sio.emit("start_multiple", start_data)
        self.get_logger().info("Drone'lar başlatıldı!")
    
    def send_lock_attempt(self, target_id: str, target_lat: float, target_lon: float):
        """
        Lock denemesi gönder
        """
        if not self.connected:
            self.get_logger().warn("Server'a bağlı değil!")
            return
        
        lock_data = {
            "id": target_id,
            "target_lat": target_lat,
            "target_lon": target_lon
        }
        
        self.sio.emit("lock_attempt", lock_data)
        self.get_logger().info(f"Lock denemesi: {target_id}")
    
    def destroy_node(self):
        """Node kapatılırken"""
        if self.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MockServerBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
