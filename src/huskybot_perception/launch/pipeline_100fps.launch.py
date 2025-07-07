#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Start Velodyne
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # 2. Start Cameras
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # 3. Start ULTRA-FAST DeepStream
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='ultra_fast_deepstream',
                    name='ultra_fast_deepstream',
                    output='screen'
                )
            ]
        ),
        
        # 4. Start Enhanced Fusion
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen'
                )
            ]
        ),
        
    ])