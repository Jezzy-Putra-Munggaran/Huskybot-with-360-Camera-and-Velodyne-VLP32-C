#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('pipeline_type', default_value='segmentation',
                             choices=['segmentation', 'detection']),
        DeclareLaunchArgument('use_deepstream', default_value='false'),
        
        # 1. Start Cameras (highest priority)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('huskybot_camera'),
                           'launch', 'camera.launch.py')
            ])
        ),
        
        # 2. Start Velodyne LiDAR
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # 3. Start YOLO Processing (conditional)
        TimerAction(
            period=8.0,
            actions=[
                # Segmentation pipeline
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_segmentation'),
                                   'launch', 'segmentation_working.launch.py')
                    ]),
                    condition=LaunchConfigurationEquals('pipeline_type', 'segmentation')
                ),
                
                # Detection pipeline  
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_detection'),
                                   'launch', 'detection.launch.py')
                    ]),
                    condition=LaunchConfigurationEquals('pipeline_type', 'detection')
                ),
            ]
        ),
        
        # 4. Start Fusion
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen'
                )
            ]
        ),
        
        # 5. Start DeepStream (optional)
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_deepstream'),
                                   'launch', 'deepstream_yolo.launch.py')
                    ]),
                    condition=LaunchConfigurationEquals('use_deepstream', 'true')
                )
            ]
        ),
        
    ])