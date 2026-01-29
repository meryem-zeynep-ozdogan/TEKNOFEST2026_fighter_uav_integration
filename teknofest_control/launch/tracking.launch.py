#!/usr/bin/env python3
"""
================================================================================
TEKNOFEST - GPS TAKİP LAUNCH DOSYASI
================================================================================
GPS Tracking Node'u başlatır

Kullanım:
  ros2 launch teknofest_control tracking.launch.py
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description oluştur"""
    
    # Package directory
    pkg_dir = get_package_share_directory('teknofest_control')
    
    # ================================================================
    # LAUNCH ARGÜMANLARI
    # ================================================================
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'tracking_params.yaml'),
        description='Parametre dosyası yolu'
    )
    
    # ================================================================
    # NODE TANIMLARI
    # ================================================================
    
    # Ana GPS Tracking Node
    gps_tracking_node = Node(
        package='teknofest_control',
        executable='gps_tracking_node',
        name='gps_tracking_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # ================================================================
    # LAUNCH DESCRIPTION
    # ================================================================
    
    return LaunchDescription([
        config_file_arg,
        gps_tracking_node,
    ])
