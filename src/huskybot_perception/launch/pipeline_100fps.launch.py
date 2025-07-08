#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('model_path', default_value='yolo11n-seg.engine'),
        DeclareLaunchArgument('fps_target', default_value='30'),  # ✅ REALISTIC target
        DeclareLaunchArgument('debug_mode', default_value='true'),
        
        # ✅ 1. Start Velodyne LiDAR FIRST
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 2. Wait for LiDAR, then start cameras
        TimerAction(
            period=8.0,  # ✅ INCREASED wait time for LiDAR
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Wait for cameras, then start DeepStream
        TimerAction(
            period=15.0,  # ✅ INCREASED wait time for cameras
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
                        'skip_frames': 2,     # ✅ OPTIMIZED for better performance
                        'batch_size': 3,      # ✅ REDUCED for stability
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 4. Wait for detections, then start fusion
        TimerAction(
            period=25.0,  # ✅ INCREASED wait time for detections
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 5. Debug monitoring (optional)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '🚀 Pipeline fully launched! Check topics with: ros2 topic list'],
                    output='screen'
                )
            ]
        ),
        
    ])