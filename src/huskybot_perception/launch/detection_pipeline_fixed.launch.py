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
        
        # 2. Wait then start cameras with optimized launch
        TimerAction(
            period=5.0,  # Wait 5 seconds for Velodyne
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('huskybot_camera'),
                            'launch', 'camera_jetson_optimized.launch.py'
                        )
                    ])
                )
            ]
        ),
        
        # 3. Wait longer for cameras to stabilize
        TimerAction(
            period=20.0,  # Wait 20 seconds total for all cameras
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
            period=25.0,  # Wait 25 seconds total
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
                        'publish_rate': 2.0,
                        'image_width': 1920,
                        'image_height': 1080
                    }],
                    respawn=True,
                    respawn_delay=5.0,
                )
            ]
        ),
        
        # 5. Build huskybot_detection package
        TimerAction(
            period=30.0,  # Wait 30 seconds before building again
            actions=[
                Node(
                    package='huskybot_detection',
                    executable='multicam_detection_node',
                    name='multicam_detection_node',
                    output='screen',
                    parameters=[{
                        'device': 'cuda:0'
                    }],
                    respawn=True,
                    respawn_delay=5.0,
                )
            ]
        ),
        
        # 6. Verify CUDA availability
        TimerAction(
            period=35.0,  # Wait 35 seconds before verifying CUDA
            actions=[
                Node(
                    package='python3',
                    executable='-c',
                    name='cuda_verification',
                    output='screen',
                    parameters=[{
                        'script': "import torch; print(f'CUDA: {torch.cuda.is_available()}, Device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"None\"}')"
                    }],
                    respawn=True,
                    respawn_delay=5.0,
                )
            ]
        ),
    ])