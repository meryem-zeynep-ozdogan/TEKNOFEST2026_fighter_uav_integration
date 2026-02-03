#!/usr/bin/env python3
"""
================================================================================
MOCK TARGET TRACKING LAUNCH FILE
================================================================================
Mock sunucudan hedef verisi alma ve L1 Adaptive GPS takip sistemini başlatır.

İçerir:
  1. mock_target_receiver - WebSocket'ten hedef verilerini alır, gps_tracking_node'a iletir
  2. gps_tracking_node - L1 Guidance ile hedef takibi yapar
  3. (Opsiyonel) px4_to_mock_bridge - PX4'ten mock sunucuya veri iletir

Kullanım:
  ros2 launch teknofest_control mock_tracking.launch.py

Parametreler:
  mock_server_url - Mock sunucu adresi (default: http://localhost:8080)
  enable_px4_bridge - PX4 bridge'i aktif et (default: false)
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    mock_server_url_arg = DeclareLaunchArgument(
        'mock_server_url',
        default_value='http://localhost:8080',
        description='Mock sunucu WebSocket URL\'si'
    )

    enable_px4_bridge_arg = DeclareLaunchArgument(
        'enable_px4_bridge',
        default_value='false',
        description='PX4 to Mock bridge aktif et'
    )

    own_uav_id_arg = DeclareLaunchArgument(
        'own_uav_id',
        default_value='bizim_iha',
        description='Kendi uçağımızın ID\'si (filtreleme için)'
    )

    # =========================================================================
    # NODE'LAR
    # =========================================================================

    # Mock Target Receiver Node - hedef verilerini mock sunucudan alır
    mock_target_receiver_node = Node(
        package='teknofest_control',
        executable='mock_target_receiver',
        name='mock_target_receiver',
        output='screen',
        parameters=[{
            'mock_server_url': LaunchConfiguration('mock_server_url'),
            'target_timeout_sec': 5.0,
            'publish_rate_hz': 50.0,
            'own_uav_id': LaunchConfiguration('own_uav_id'),
            'auto_reconnect': True,
            'reconnect_delay_sec': 3.0,
        }]
    )

    # GPS Tracking Node - L1 Guidance ile hedef takibi
    gps_tracking_node = Node(
        package='teknofest_control',
        executable='gps_tracking_node',
        name='gps_tracking_node',
        output='screen',
        parameters=[{
            # Ağırlıklar
            'weights.distance': 0.35,
            'weights.angle': 0.45,
            'weights.speed': 0.20,
            # L1 Guidance
            'l1_guidance.l1_distance': 50.0,
            'l1_guidance.l1_damping': 0.85,
            'l1_guidance.adaptive_l1': True,
            # Durum makinesi
            'state_machine.lock_distance': 80.0,
            'state_machine.approach_distance': 150.0,
            'state_machine.lock_confirmation_time': 4.0,
            # PX4
            'px4.control_frequency': 50.0,
        }]
    )

    # PX4 to Mock Bridge (Opsiyonel) - PX4'ten mock sunucuya veri gönderir
    px4_to_mock_bridge_node = Node(
        package='teknofest_control',
        executable='px4_to_mock_bridge',
        name='px4_to_mock_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_px4_bridge')),
        parameters=[{
            'mock_server_url': LaunchConfiguration('mock_server_url'),
            'enemy_instance_ids': [1, 2],
            'publish_rate_hz': 10.0,
            'auto_reconnect': True,
        }]
    )

    # =========================================================================
    # LOG BİLGİLERİ
    # =========================================================================
    log_start = LogInfo(
        msg='🚀 Mock Target Tracking System (L1 Guidance) başlatılıyor...'
    )

    log_config = LogInfo(
        msg=['   Mock Server: ', LaunchConfiguration('mock_server_url')]
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription([
        # Arguments
        mock_server_url_arg,
        enable_px4_bridge_arg,
        own_uav_id_arg,

        # Logs
        log_start,
        log_config,

        # Nodes
        mock_target_receiver_node,
        gps_tracking_node,
        px4_to_mock_bridge_node,
    ])
