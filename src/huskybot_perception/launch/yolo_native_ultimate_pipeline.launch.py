#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/yolo_native_ultimate_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. ULTRA POWER OPTIMIZATION untuk 100+ FPS
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER OPTIMIZATION..." && '
                 'sudo nvpmodel -m 0 && '  # MAX-N mode
                 'sudo jetson_clocks --fan && '  # Enable fan
                 'sudo sh -c "echo performance > /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor" && '  # Performance mode
                 'echo "⚡ ULTRA POWER OPTIMIZED FOR 100+ FPS!"'],
            output='screen',
            name='ultra_power_optimization'
        ),
        
        # ✅ 2. Start Velodyne LiDAR
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start cameras
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 4. Start YOLO NATIVE ULTIMATE node
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='yolo_native_ultimate_node',
                    name='yolo_native_ultimate_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[
                        {'use_sim_time': False}
                    ],
                    # ✅ Optimize untuk maximum performance
                    prefix=['nice', '-n', '-20']  # Highest priority
                )
            ]
        ),
        
        # ✅ 5. RViz2 auto-popup untuk LiDAR (delayed)
        TimerAction(
            period=180.0,  # 3 minutes delay
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting RViz2 for LiDAR..." && '
                         'DISPLAY=:0 rviz2 -d $(ros2 pkg prefix huskybot_perception)/share/huskybot_perception/rviz/lidar_config.rviz &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 6. Performance monitoring
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 YOLO NATIVE Performance Status:" && '
                         'echo "📡 Camera topics:" && timeout 3 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 GPU Status:" && nvidia-smi --query-gpu=utilization.gpu,memory.used,temperature.gpu --format=csv,noheader,nounits && '
                         'echo "💻 CPU Usage:" && top -bn1 | grep "Cpu(s)" && '
                         'echo "✅ CHECK 6 NATIVE YOLO WINDOWS!" && '
                         'echo "🎯 TARGET: 100+ FPS dengan YOLO NATIVE DISPLAY"'],
                    output='screen'
                )
            ]
        ),
    ])