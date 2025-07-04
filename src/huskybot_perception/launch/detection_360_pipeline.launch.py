#!/usr/bin/env python3
"""
Master launch file untuk 360° Object Detection Pipeline
Pipeline: 6 Cameras -> YOLOv12 Detection -> LiDAR Fusion -> Visualization

Usage:
ros2 launch huskybot_perception detection_360_pipeline.launch.py

Target Output: Class=..., Confidence=..., Distance: ...m, Coordinate: (..., ..., ...)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Velodyne LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('velodyne'),
                    'launch', 'velodyne-all-nodes-VLP32C-launch.py'
                )
            ])
        ),
        
        # 2. Start 6 cameras (wait for LiDAR to stabilize)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('huskybot_camera'),
                            'launch', 'multicamera.launch.py'
                        )
                    ])
                )
            ]
        ),
        
        # 3. Start YOLOv12 Detection (wait for cameras)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='huskybot_detection',
                    executable='multicam_detection_node',
                    name='multicam_detection',
                    output='screen',
                    parameters=[{
                        'cam_count': 6,
                        'model_path': 'yolo12x.engine',
                        'device': 'cuda:0',  # FIXED device string
                        'conf_thres': 0.25,
                        'visualization_enabled': True,
                    }],
                    respawn=True,
                    respawn_delay=2.0
                )
            ]
        ),
        
        # 4. Start Fusion Node (wait for detection)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='fusion_360',
                    output='screen',
                    parameters=[{
                        'max_laser_distance': 50.0,
                        'min_laser_distance': 0.5,
                        'confidence_threshold': 0.25,
                        'publish_rate': 10.0,  # Faster rate untuk real-time
                        'enable_3d_output': True,  # Enable target output format
                    }],
                    respawn=True
                )
            ]
        ),
        
        # 5. Start Visualization
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='fusion_visualizer_node',
                    name='visualizer_360',
                    output='screen',
                    parameters=[{
                        'display_fusion_results': True,
                        'show_3d_coordinates': True,
                        'camera_count': 6
                    }]
                )
            ]
        ),
    ])