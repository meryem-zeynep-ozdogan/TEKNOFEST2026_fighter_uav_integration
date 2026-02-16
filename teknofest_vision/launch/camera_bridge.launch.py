#!/usr/bin/env python3
"""
TEKNOFEST Vision - Gazebo to ROS2 Camera Bridge Launch File
Hero uçağının kamera görüntüsünü ROS2 topic'ine aktarır

Kullanım:
  ros2 launch teknofest_vision camera_bridge.launch.py

Gazebo Topic: /world/competition/model/hero_cessna/link/base_link/sensor/front_camera/image
ROS2 Topic:   /hero/camera/image_raw

NOT: Model ismi PX4 SITL spawn sırasına göre değişebilir:
  - hero_cessna (ilk spawn)
  - hero_cessna_0, hero_cessna_1, ... (sonraki spawn'lar)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Model ismi argümanı - spawn sırasına göre değişebilir
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='hero_cessna',
        description='Gazebo model ismi (hero_cessna veya hero_cessna_0)'
    )
    
    model_name = LaunchConfiguration('model_name')
    
    # Gazebo -> ROS2 image bridge using ros_gz_image
    gz_ros_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='hero_camera_bridge',
        arguments=[
            ['/world/competition/model/', model_name, '/link/base_link/sensor/front_camera/image']
        ],
        remappings=[
            (['/world/competition/model/', model_name, '/link/base_link/sensor/front_camera/image'], 
             '/hero/camera/image_raw')
        ],
        output='screen'
    )
    
    # Kamera info publisher (opsiyonel)
    camera_info_pub = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hero_camera_info_bridge',
        arguments=[
            ['/world/competition/model/', model_name, '/link/base_link/sensor/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        ],
        remappings=[
            (['/world/competition/model/', model_name, '/link/base_link/sensor/front_camera/camera_info'],
             '/hero/camera/camera_info')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        model_name_arg,
        gz_ros_bridge,
        camera_info_pub
    ])
