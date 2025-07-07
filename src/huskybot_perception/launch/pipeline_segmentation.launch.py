#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        
        # 1. Start Gazebo (optional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('huskybot_gazebo'),
                           'launch', 'huskybot_launch.py')
            ]),
            condition=LaunchConfigurationEquals('use_gazebo', 'true')
        ),
        
        # 2. Start Velodyne LiDAR (highest priority)
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
        
        # 3. Start Cameras
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # 4. Start YOLOv11 Segmentation
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_segmentation'),
                                   'launch', 'segmentation_working.launch.py')
                    ])
                )
            ]
        ),
        
        # 5. Start Fusion Node
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen',
                    parameters=[{
                        'enable_3d_visualization': True,
                        'max_detection_distance': 50.0,
                        'min_confidence': 0.25
                    }]
                )
            ]
        ),
        
        # 6. Start RViz2 (optional)
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    condition=LaunchConfigurationEquals('use_rviz', 'true')
                )
            ]
        ),
        
    ])