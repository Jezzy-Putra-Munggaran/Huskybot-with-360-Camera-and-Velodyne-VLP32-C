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
        
        # ✅ 1. Start Velodyne LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 2. Start cameras
        TimerAction(
            period=3.0,  # Reduced delay
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start ULTRA-OPTIMIZED DeepStream
        TimerAction(
            period=10.0,  # Reduced delay
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='deepstream_yolo_node',
                    name='deepstream_yolo',
                    output='screen',
                    parameters=[{
                        'model_engine': LaunchConfiguration('model_path'),
                        'fps_target': LaunchConfiguration('fps_target'),
                        'input_width': 416,   # ✅ Optimized for speed
                        'input_height': 416,
                        'skip_frames': 2,     # ✅ More aggressive skipping
                        'batch_size': 6,
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=2.0
                )
            ]
        ),
        
        # ✅ 4. Start fusion
        TimerAction(
            period=15.0,
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
        
        # ✅ 5. Create RViz directory and config
        TimerAction(
            period=18.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'mkdir -p /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz'],
                    output='screen',
                    name='create_rviz_dir'
                )
            ]
        ),
        
        # ✅ 6. Generate RViz config
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='create_rviz_config',
                    name='rviz_config_creator',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. AUTO-POPUP RViz2 for 3D visualization
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && rviz2 -d /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz/huskybot_3d.rviz > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_popup_rviz2'
                )
            ]
        ),
        
        # ✅ 8. AUTO-POPUP RQT Image View for grid visualization
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && rqt_image_view /deepstream_grid > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_popup_grid_viewer'
                )
            ]
        ),
        
        # ✅ 9. Performance optimization commands
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 ULTRA-FAST Performance Optimizations:" && '
                         'sudo jetson_clocks && '
                         'sudo nvpmodel -m 0 && '
                         'echo "✅ Jetson performance mode activated" || echo "⚠️  Performance mode failed"'],
                    output='screen',
                    name='performance_optimization'
                )
            ]
        ),
        
        # ✅ 10. Final status and auto-display confirmation
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-FAST Pipeline Status:" && '
                         'echo "📡 Camera topics:" && ros2 topic list | grep image_raw | head -6 && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep detections | head -6 && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "🎯 3D Objects topic:" && ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "✅ AUTO-DISPLAY ACTIVATED!" && '
                         'echo "🖥️  RViz2: Auto-opened for 3D visualization" && '
                         'echo "📺 RQT: Auto-opened for grid visualization" && '
                         'echo "🚀 TARGET: 100+ FPS Achievement!"'],
                    output='screen'
                )
            ]
        ),
        
    ])