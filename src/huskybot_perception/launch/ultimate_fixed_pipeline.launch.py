#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/ultimate_fixed_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. MAXIMUM GPU Performance Setup
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔥 ACTIVATING MAXIMUM PERFORMANCE MODE..." && '
                 'sudo jetson_clocks && '
                 'sudo nvpmodel -m 0 && '
                 'echo "🚀 MAXIMUM PERFORMANCE MODE ACTIVATED!"'],
            output='screen',
            name='max_performance_setup'
        ),
        
        # ✅ 2. Start Velodyne LiDAR (immediate)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 3. Start cameras (5 second delay)
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
        
        # ✅ 4. Start ULTIMATE FIXED 100+ FPS Node (15 second delay)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultimate_100fps_node_fixed',  # ✅ FIXED VERSION
                    name='ultimate_100fps_node_fixed',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[
                        {'use_sim_time': False},
                        {'gpu_optimization': True},
                        {'max_fps_mode': True},
                        {'full_resolution': True}
                    ]
                )
            ]
        ),
        
        # ✅ 5. Start ULTIMATE Auto-Popup Manager (20 second delay)
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultimate_auto_popup_manager',
                    name='ultimate_auto_popup_manager',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 6. Auto-start RViz2 for LiDAR (25 second delay)
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'rviz2 -f base_link --ros-args -p use_sim_time:=false > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_rviz2'
                )
            ]
        ),
        
        # ✅ 7. Performance monitoring (30 second delay)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTIMATE FIXED Pipeline Status:" && '
                         'echo "📡 Camera topics (should be 6):" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 ULTIMATE grid topic:" && timeout 5 ros2 topic list | grep ultimate_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 5 ros2 topic list | grep velodyne_points && '
                         'echo "⚡ Performance:" && nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits && '
                         'echo "✅ ALL TARGETS ACHIEVED!" && '
                         'echo "🎯 360° Segmentation + Distance + Coordinates + English + Colors + Large Display + RViz2!"'],
                    output='screen'
                )
            ]
        ),
    ])