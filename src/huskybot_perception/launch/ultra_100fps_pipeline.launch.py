#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/ultra_100fps_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. MAXIMUM Jetson + Camera optimization
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                 'echo "kmporin" | sudo -S systemctl stop nvargus-daemon && sleep 2 && '
                 'echo "kmporin" | sudo -S systemctl start nvargus-daemon && sleep 3 && '
                 'echo "kmporin" | sudo -S /usr/bin/jetson_clocks --fan && '
                 'echo "kmporin" | sudo -S nvpmodel -m 0 && '
                 'echo "kmporin" | sudo -S nvidia-smi -pm 1 && '
                 'echo "kmporin" | sudo -S nvidia-smi -pl 55 && '
                 'echo "kmporin" | sudo -S nvidia-smi -lgc 2100,2100 && '
                 'echo "kmporin" | sudo -S nvidia-smi -lmc 6251,6251 && '
                 'echo "kmporin" | sudo -S sysctl -w vm.swappiness=1 && '
                 'for i in {0..11}; do echo "kmporin" | sudo -S sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor"; done && '
                 'echo "🔥 ULTRA-MAXIMUM optimization COMPLETED!"'],
            output='screen',
            name='ultra_maximum_optimization'
        ),
        
        # ✅ 2. Start cameras with enhanced recovery
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔄 Camera subsystem reset..." && '
                         'echo "kmporin" | sudo -S pkill -f nvargus-daemon || true && '
                         'sleep 3 && '
                         'echo "kmporin" | sudo -S rm -f /tmp/.argus* || true && '
                         'echo "kmporin" | sudo -S systemctl restart nvargus-daemon && '
                         'sleep 5 && '
                         'echo "kmporin" | sudo -S chmod 666 /dev/video* || true && '
                         'echo "✅ Camera subsystem ready!"'],
                    output='screen'
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start Velodyne LiDAR
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 4. Start ULTRA-MEGA Segmentation Node
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='ultra_mega_segmentation_node',
                    name='ultra_mega_segmentation',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
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
        
        # ✅ 5. AUTO-POPUP: Grid Display
        TimerAction(
            period=30.0,
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
        
        # ✅ 6. AUTO-POPUP: RViz2 for 3D objects + LiDAR
        TimerAction(
            period=35.0,
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
        
        # ✅ 7. Performance monitoring
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-MEGA Pipeline Status:" && '
                         'echo "📡 Camera topics:" && ros2 topic list | grep image_raw && '
                         'echo "🔍 Grid topic:" && ros2 topic list | grep ultra_grid && '
                         'echo "🎯 3D Objects:" && ros2 topic list | grep objects_3d && '
                         'echo "🔴 LiDAR topics:" && ros2 topic list | grep -E "(scan|velodyne_points)" && '
                         'echo "✅ TARGET: 100+ FPS with PERFECT segmentation + distance + coordinates!"'],
                    output='screen'
                )
            ]
        ),
    ])