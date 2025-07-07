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
        DeclareLaunchArgument('model_path', default_value='yolo11n-seg.engine'),
        DeclareLaunchArgument('fps_target', default_value='60'),  # ✅ REALISTIC target
        
        # 1. Start Velodyne LiDAR
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
        
        # 3. Start OPTIMIZED DeepStream
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='deepstream_yolo_node',
                    name='deepstream_yolo',
                    output='screen',
                    parameters=[{
                        'model_engine': LaunchConfiguration('model_path'),
                        'fps_target': LaunchConfiguration('fps_target'),
                        'input_width': 640,   # ✅ MATCH model exactly
                        'input_height': 640,  # ✅ MATCH model exactly
                        'skip_frames': 2,     # ✅ REDUCED for better performance
                        'batch_size': 3,      # ✅ REDUCED for stability
                        'device_id': 0
                    }]
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