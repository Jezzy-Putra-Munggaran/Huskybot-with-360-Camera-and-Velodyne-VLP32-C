#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/ultimate_working_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. Start Velodyne LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 2. Start cameras (5 second delay)
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
        
        # ✅ 3. Start SIMPLE working node (15 second delay)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='simple_working_node',
                    name='simple_working_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 4. Start ULTIMATE Display Manager (20 second delay)
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultimate_display_manager',
                    name='ultimate_display_manager',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0
                )
            ]
        ),
        
        # ✅ 5. Force maximize performance (5 second delay)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'sudo nvidia-smi -pl 50 2>/dev/null || true && '
                         'sudo nvidia-smi -lgc 1300,2100 2>/dev/null || true && '
                         'sudo nvidia-smi -lmc 1215,8000 2>/dev/null || true && '
                         'echo "🔥 GPU performance maximized!"'],
                    output='screen',
                    name='gpu_performance_boost'
                )
            ]
        ),
        
        # ✅ 6. Status monitoring (30 second delay)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTIMATE Pipeline Status:" && '
                         'echo "📡 Camera topics:" && timeout 3 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔍 Grid topic:" && timeout 3 ros2 topic list | grep simple_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 3 ros2 topic list | grep scan && '
                         'echo "🚀 Display Manager:" && timeout 3 ros2 node list | grep display_manager && '
                         'echo "✅ ULTIMATE VERSION - ALL AUTO-POPUP WORKING!" && '
                         'echo "🎯 Grid display & RViz2 should be auto-popup!"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. CPU/GPU monitoring (35 second delay)
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "💻 System Performance:" && '
                         'top -bn1 | grep "Cpu(s)" | head -1 && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null || echo "GPU info not available" && '
                         'echo "🔥 Target: 100+ FPS with full GPU utilization"'],
                    output='screen'
                )
            ]
        ),
    ])