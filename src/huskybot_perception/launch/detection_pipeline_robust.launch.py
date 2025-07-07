#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Velodyne FIRST
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('velodyne'),
                    'launch', 'velodyne-all-nodes-VLP32C-launch.py'
                )
            ])
        ),
        
        # 2. Wait then start cameras with optimized parameters
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
        
        # 3. Wait then start detection with fixed device parameter
        TimerAction(
            period=15.0,
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
                    respawn=False,
                )
            ]
        ),
        
        # 4. Wait then start fusion
        TimerAction(
            period=20.0,
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
                        'publish_rate': 10.0,
                        'enable_3d_output': True,
                    }],
                    respawn=False,
                )
            ]
        ),
    ])