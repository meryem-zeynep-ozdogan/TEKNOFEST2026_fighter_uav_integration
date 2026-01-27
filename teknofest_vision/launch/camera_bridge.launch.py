#!/usr/bin/env python3
"""
TEKNOFEST Vision - Gazebo to ROS2 Camera Bridge Launch File
Hero uçağının kamera görüntüsünü ROS2 topic'ine aktarır

Kullanım:
  ros2 launch teknofest_vision camera_bridge.launch.py

Gazebo Topic: /world/competition/model/rc_cessna_0/link/base_link/sensor/front_camera/image
ROS2 Topic:   /hero/camera/image_raw
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Gazebo -> ROS2 image bridge using ros_gz_image
    gz_ros_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='hero_camera_bridge',
        arguments=[
            '/world/competition/model/rc_cessna_0/link/base_link/sensor/front_camera/image'
        ],
        remappings=[
            ('/world/competition/model/rc_cessna_0/link/base_link/sensor/front_camera/image', 
             '/hero/camera/image_raw')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_ros_bridge
    ])
