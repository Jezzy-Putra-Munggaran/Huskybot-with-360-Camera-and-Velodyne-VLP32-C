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
        DeclareLaunchArgument('auto_display', default_value='true'),
        
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
        
        # ✅ 3. Start DeepStream with FIXED model loading
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='deepstream_yolo_node',
                    name='deepstream_yolo',
                    output='screen',
                    parameters=[{
                        'model_engine': LaunchConfiguration('model_path'),
                        'fps_target': LaunchConfiguration('fps_target'),
                        'input_width': 640,  # ✅ Balanced for performance
                        'input_height': 640,
                        'skip_frames': 1,    # ✅ Reduced skipping
                        'batch_size': 6,
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 4. Start fusion after DeepStream
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
        
        # ✅ 5. AUTO-POPUP RViz2 with 3D PointCloud visualization
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'sleep 2 && rviz2 -d /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz/huskybot_3d.rviz &'],
                    output='screen',
                    name='auto_rviz2'
                )
            ]
        ),
        
        # ✅ 6. AUTO-POPUP RQT Image View for grid visualization
        TimerAction(
            period=37.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'sleep 2 && rqt_image_view /deepstream_grid &'],
                    output='screen',
                    name='auto_grid_viewer'
                )
            ]
        ),
        
        # ✅ 7. Create RViz2 config file automatically
        TimerAction(
            period=32.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'mkdir -p /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz'],
                    output='screen',
                    name='create_rviz_dir'
                )
            ]
        ),
        
        # ✅ 8. Create RViz2 configuration
        TimerAction(
            period=33.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='create_rviz_config.py',
                    name='rviz_config_creator',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 9. Final status check with auto-display
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 ULTRA-FAST Pipeline Status:" && '
                         'echo "📡 Camera topics:" && ros2 topic list | grep image_raw && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep detections && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "🎯 3D Objects topic:" && ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "✅ Pipeline launched with AUTO-DISPLAY!" && '
                         'echo "🖥️  RViz2: Auto-opened for 3D visualization" && '
                         'echo "📺 RQT: Auto-opened for grid visualization"'],
                    output='screen'
                )
            ]
        ),
        
    ])