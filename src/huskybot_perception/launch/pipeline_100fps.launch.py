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
        DeclareLaunchArgument('fps_target', default_value='120'),
        DeclareLaunchArgument('debug_mode', default_value='true'),
        
        # ✅ 1. Start Velodyne LiDAR FIRST
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 2. Start cameras immediately (parallel)
        TimerAction(
            period=5.0,  # ✅ Reduced wait time
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start DeepStream ASAP
        TimerAction(
            period=10.0,  # ✅ Reduced wait time
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='deepstream_yolo_node',
                    name='deepstream_yolo',
                    output='screen',
                    parameters=[{
                        'model_engine': LaunchConfiguration('model_path'),
                        'fps_target': LaunchConfiguration('fps_target'),
                        'input_width': 640,
                        'input_height': 640,
                        'skip_frames': 0,     # ✅ NO frame skipping
                        'batch_size': 6,      # ✅ FULL batch processing
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=2.0
                )
            ]
        ),
        
        # ✅ 4. Start fusion quickly
        TimerAction(
            period=15.0,  # ✅ Reduced wait time
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0
                )
            ]
        ),
        
        # ✅ 5. Launch confirmation
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '🚀 ULTRA Pipeline launched! Check: ros2 topic list | grep camera'],
                    output='screen'
                )
            ]
        ),
        
    ])