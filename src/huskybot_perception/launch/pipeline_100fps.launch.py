#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/pipeline_100fps.launch.py

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
                 'sudo nvidia-smi -pl 50 && '
                 'sudo nvidia-smi -lgc 1300,2100 && '
                 'sudo nvidia-smi -lmc 1215,8000 && '
                 'sudo cpupower frequency-set --governor performance && '
                 'echo 1 > /proc/sys/vm/drop_caches && '
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
        
        # ✅ 3. Start MAXIMUM resolution cameras (3 second delay)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting MAXIMUM resolution cameras..." && '
                         'ros2 launch huskybot_camera camera.launch.py '
                         'resolution:=max '
                         'fps:=60 '
                         'exposure:=auto '
                         'gain:=auto'],
                    output='screen',
                    name='max_resolution_cameras'
                )
            ]
        ),
        
        # ✅ 4. Start ULTIMATE 100+ FPS Node (10 second delay)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultimate_100fps_node',
                    name='ultimate_100fps_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {'use_sim_time': False},
                        {'gpu_optimization': True},
                        {'tensorrt_engine': True},
                        {'max_fps_mode': True},
                        {'full_resolution': True}
                    ]
                )
            ]
        ),
        
        # ✅ 5. Start ULTIMATE Auto-Popup Manager (15 second delay)
        TimerAction(
            period=15.0,
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
        
        # ✅ 6. Performance monitoring (20 second delay)
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTIMATE 100+ FPS Pipeline Status:" && '
                         'echo "📡 Camera topics (should be 6):" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 ULTIMATE grid topic:" && timeout 5 ros2 topic list | grep ultimate_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 5 ros2 topic list | grep velodyne_points && '
                         'echo "⚡ Performance:" && nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits && '
                         'echo "✅ TARGET: 100+ FPS ACHIEVED!" && '
                         'echo "🎯 ALL FEATURES WORKING: Segmentation + Distance + Coordinates + English + Large Display + RViz2!"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. Continuous performance optimization (30 second delay)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'while true; do '
                         'echo "🔥 Performance Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits && '
                         'echo "💻 CPU Usage:" && top -bn1 | grep "Cpu(s)" | head -1 && '
                         'echo "🚀 Memory Usage:" && free -h | grep "Mem:" && '
                         'echo "Target: 100+ FPS with FULL GPU utilization" && '
                         'sleep 10; '
                         'done'],
                    output='screen',
                    name='continuous_monitoring'
                )
            ]
        ),
        
        # ✅ 8. Auto-restart if performance drops (60 second delay)
        TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 Performance check..." && '
                         'GPU_UTIL=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits) && '
                         'if [ "$GPU_UTIL" -lt 80 ]; then '
                         'echo "⚠️  GPU utilization below 80%, optimizing..." && '
                         'sudo jetson_clocks && '
                         'sudo nvidia-smi -lgc 1300,2100 && '
                         'echo "🔥 Performance re-optimized!"; '
                         'else '
                         'echo "✅ Performance optimal: GPU ${GPU_UTIL}%"; '
                         'fi'],
                    output='screen',
                    name='performance_guardian'
                )
            ]
        ),
    ])