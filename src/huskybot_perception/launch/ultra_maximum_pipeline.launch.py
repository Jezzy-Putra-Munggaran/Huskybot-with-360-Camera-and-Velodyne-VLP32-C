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
        
        # ✅ 1. CRITICAL: ULTRA-MAXIMUM Jetson optimization first
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                 'echo kmporin | sudo -S /usr/bin/jetson_clocks --fan && '
                 'echo kmporin | sudo -S nvpmodel -m 0 && '
                 'echo kmporin | sudo -S nvidia-smi -pm 1 && '
                 'echo kmporin | sudo -S nvidia-smi -pl 55 && '
                 'echo kmporin | sudo -S nvidia-smi -lgc 2100,2100 && '
                 'echo kmporin | sudo -S nvidia-smi -lmc 6251,6251 && '
                 'echo kmporin | sudo -S sysctl -w vm.swappiness=1 && '
                 'echo kmporin | sudo -S sysctl -w vm.vfs_cache_pressure=10 && '
                 'for i in {0..11}; do echo kmporin | sudo -S sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor"; done && '
                 'echo "🔥 ULTRA-MAXIMUM optimization COMPLETED!"'],
            output='screen',
            name='ultra_maximum_optimization'
        ),
        
        # ✅ 2. Start cameras with correct CSI mapping
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start ULTRA-MAXIMUM DeepStream node
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='ultra_maximum_deepstream.py',
                    name='ultra_maximum_performance',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                    additional_env={
                        'CUDA_VISIBLE_DEVICES': '0',
                        'OMP_NUM_THREADS': '12',
                        'CUDA_LAUNCH_BLOCKING': '0',
                        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:2048',
                        'CUDA_CACHE_DISABLE': '0',
                        'CUDA_DEVICE_MAX_CONNECTIONS': '64'
                    }
                )
            ]
        ),
        
        # ✅ 4. AUTO-POPUP: Enhanced Grid Display
        TimerAction(
            period=25.0,
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
        
        # ✅ 5. Enhanced fusion for 3D visualization
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='ultra_fusion',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 6. AUTO-POPUP: RViz2 for 3D objects
        TimerAction(
            period=30.0,
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
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-MAXIMUM Pipeline Status:" && '
                         'echo "📡 Camera topics:" && ros2 topic list | grep image_raw && '
                         'echo "🔍 Segmentation topics:" && ros2 topic list | grep segmentation && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep ultra_grid && '
                         'echo "🎯 3D Objects:" && ros2 topic list | grep objects_3d && '
                         'echo "✅ YOLO11X-seg.engine + ULTRA-MAXIMUM performance!" && '
                         'echo "🚀 TARGET: 100+ FPS with segmentation masks + distance + coordinates!"'],
                    output='screen'
                )
            ]
        ),
    ])