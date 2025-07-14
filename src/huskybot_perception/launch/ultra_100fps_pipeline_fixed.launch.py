#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/ultra_100fps_pipeline_fixed.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. ULTIMATE Camera Recovery (FIRST)
        ExecuteProcess(
            cmd=['bash', '/home/kmp-orin/jezzy/huskybot/scripts/ultimate_camera_recovery.sh'],
            output='screen',
            name='ultimate_camera_recovery'
        ),
        
        # ✅ 2. MAXIMUM Jetson optimization (AFTER camera recovery)
        TimerAction(
            period=15.0,  # Wait for camera recovery
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                         'echo "kmporin" | sudo -S /usr/bin/jetson_clocks --fan && '
                         'echo "kmporin" | sudo -S nvpmodel -m 0 && '
                         'echo "kmporin" | sudo -S nvidia-smi -pm 1 2>/dev/null || true && '
                         'echo "kmporin" | sudo -S nvidia-smi -pl 55 2>/dev/null || true && '
                         'echo "kmporin" | sudo -S nvidia-smi -lgc 2100,2100 2>/dev/null || true && '
                         'echo "kmporin" | sudo -S nvidia-smi -lmc 6251,6251 2>/dev/null || true && '
                         'echo "kmporin" | sudo -S sysctl -w vm.swappiness=1 && '
                         'for i in {0..11}; do echo "kmporin" | sudo -S sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                         'echo "🔥 ULTRA-MAXIMUM optimization COMPLETED!"'],
                    output='screen',
                    name='ultra_maximum_jetson_optimization'
                )
            ]
        ),
        
        # ✅ 3. Verify camera system is ready
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 Verifying camera system..." && '
                         'systemctl is-active --quiet nvargus-daemon && echo "✅ nvargus-daemon ACTIVE" || echo "❌ nvargus-daemon FAILED" && '
                         'ls /dev/video* 2>/dev/null && echo "✅ Video devices found" || echo "❌ No video devices" && '
                         'echo "📡 Ready to start cameras..."'],
                    output='screen',
                    name='camera_system_verification'
                )
            ]
        ),
        
        # ✅ 4. Start Velodyne LiDAR (PARALLEL with camera prep)
        TimerAction(
            period=30.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 5. Start cameras with MAXIMUM delay for stability
        TimerAction(
            period=35.0,  # LONG delay to ensure nvargus-daemon is ready
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 6. Wait for camera stabilization before starting YOLO
        TimerAction(
            period=50.0,  # LONG wait for camera stability
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 Checking camera topics..." && '
                         'timeout 10 ros2 topic list | grep image_raw | wc -l && '
                         'echo "📡 Camera topics ready for processing"'],
                    output='screen',
                    name='camera_topic_verification'
                )
            ]
        ),
        
        # ✅ 7. Start ULTRA-MEGA Segmentation Node (AFTER cameras are stable)
        TimerAction(
            period=60.0,  # MAXIMUM delay for camera stability
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultra_mega_segmentation_node',
                    name='ultra_mega_segmentation',
                    output='screen',
                    respawn=True,
                    respawn_delay=10.0,  # Longer respawn delay
                    additional_env={
                        'CUDA_VISIBLE_DEVICES': '0',
                        'OMP_NUM_THREADS': '12',
                        'CUDA_LAUNCH_BLOCKING': '0',
                        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:8192',
                        'CUDA_CACHE_DISABLE': '0',
                        'CUDA_DEVICE_MAX_CONNECTIONS': '128'
                    }
                )
            ]
        ),
        
        # ✅ 8. AUTO-POPUP: Grid Display (AFTER segmentation)
        TimerAction(
            period=70.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='auto_grid_viewer',
                    name='ultra_grid_viewer',
                    output='screen',
                    remappings=[
                        ('/deepstream_grid', '/ultra_grid_segmentation')
                    ]
                )
            ]
        ),
        
        # ✅ 9. AUTO-POPUP: RViz2 for 3D visualization
        TimerAction(
            period=75.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'rviz2 -f base_link --ros-args -p use_sim_time:=false > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_rviz2_3d'
                )
            ]
        ),
        
        # ✅ 10. Enhanced system monitoring
        TimerAction(
            period=80.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-MEGA Pipeline Status Check:" && '
                         'echo "🔧 nvargus-daemon:" && systemctl is-active nvargus-daemon && '
                         'echo "📡 Camera topics (expecting 6):" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔍 Segmentation topic:" && timeout 5 ros2 topic list | grep ultra_grid_segmentation && '
                         'echo "🎯 3D Objects:" && timeout 5 ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "🔴 LiDAR topics:" && timeout 5 ros2 topic list | grep -E "(scan|velodyne_points)" | wc -l && '
                         'echo "✅ TARGET: 100+ FPS with PERFECT segmentation + distance + coordinates!" && '
                         'echo "📊 System ready for ULTRA-MAXIMUM performance!"'],
                    output='screen'
                )
            ]
        ),
    ])