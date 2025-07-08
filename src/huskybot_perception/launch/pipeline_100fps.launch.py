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
        
        # ✅ 2. Start cameras with proper timing
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
        
        # ✅ 3. Start DeepStream with enough time for camera initialization
        TimerAction(
            period=25.0,  # ✅ FIXED: Wait longer for cameras to be stable
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
                        'skip_frames': 0,
                        'batch_size': 6,
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 4. Start fusion after DeepStream is stable
        TimerAction(
            period=30.0,
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
        
        # ✅ 5. Final status check
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 FIXED Pipeline Status:" && '
                         'echo "📡 Camera topics:" && ros2 topic list | grep image_raw && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep detections && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "✅ Pipeline launched successfully!"'],
                    output='screen'
                )
            ]
        ),
        
    ])