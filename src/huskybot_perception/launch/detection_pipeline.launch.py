#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Velodyne FIRST (most important)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('velodyne'),
                    'launch', 'velodyne-all-nodes-VLP32C-launch.py'
                )
            ])
        ),
        
        # 2. Wait then start cameras
        TimerAction(
            period=3.0,  # Wait 3 seconds for Velodyne to initialize
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('huskybot_camera'),
                            'launch', 'camera.launch.py'
                        )
                    ])
                )
            ]
        ),
        
        # 3. Wait then start detection
        TimerAction(
            period=6.0,  # Wait 6 seconds total
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('huskybot_detection'),
                            'launch', 'detection.launch.py'
                        )
                    ])
                )
            ]
        ),
        
        # 4. Wait then start fusion
        TimerAction(
            period=9.0,  # Wait 9 seconds total
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion_node',
                    output='screen',
                    parameters=[{
                        'max_laser_distance': 50.0,
                        'min_laser_distance': 0.5,
                        'confidence_threshold': 0.25,
                        'publish_rate': 2.0,  # Slower for debugging
                        'image_width': 1920,
                        'image_height': 1080
                    }]
                )
            ]
        ),
    ])