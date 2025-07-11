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
        
        # ✅ 1. FIXED: Auto password with expect
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                 'echo kmporin | sudo -S /usr/bin/jetson_clocks --fan && '
                 'echo kmporin | sudo -S nvpmodel -m 0 && '
                 'echo kmporin | sudo -S nvidia-smi -pm 1 2>/dev/null || true && '
                 'echo kmporin | sudo -S nvidia-smi -pl 55 2>/dev/null || true && '
                 'echo kmporin | sudo -S nvidia-smi -lgc 2100,2100 2>/dev/null || true && '
                 'echo kmporin | sudo -S nvidia-smi -lmc 6251,6251 2>/dev/null || true && '
                 'echo kmporin | sudo -S sysctl -w vm.swappiness=1 && '
                 'echo kmporin | sudo -S sysctl -w vm.vfs_cache_pressure=10 && '
                 'for i in {0..11}; do echo kmporin | sudo -S sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'echo "🔥 ULTRA-MAXIMUM optimization COMPLETED!"'],
            output='screen',
            name='ultra_maximum_optimization'
        ),
        
        # ✅ 2. Start cameras with stability improvements
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔄 Resetting camera subsystem..." && '
                         'echo kmporin | sudo -S systemctl restart nvargus-daemon 2>/dev/null || true && '
                         'sleep 2'],
                    output='screen',
                    name='camera_reset'
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start ULTRA-MAXIMUM DeepStream node with FIXED imports
        TimerAction(
            period=20.0,  # Give more time for cameras to stabilize
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='ultra_maximum_deepstream.py',
                    name='ultra_maximum_performance',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                    additional_env={
                        'CUDA_VISIBLE_DEVICES': '0',
                        'OMP_NUM_THREADS': '12',
                        'CUDA_LAUNCH_BLOCKING': '0',
                        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:2048',
                        'CUDA_CACHE_DISABLE': '0',
                        'CUDA_DEVICE_MAX_CONNECTIONS': '64',
                        'PYTHONPATH': '/home/kmp-orin/jezzy/huskybot/install/huskybot_deepstream/lib/python3.10/site-packages:/home/kmp-orin/jezzy/huskybot/install/yolov12_msgs/local/lib/python3.10/dist-packages'
                    }
                )
            ]
        ),
        
        # ✅ 4. AUTO-POPUP: Enhanced Grid Display
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
        
        # ✅ 5. Enhanced fusion for 3D visualization
        TimerAction(
            period=25.0,
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
        
        # ✅ 7. Performance monitoring with detailed camera check
        TimerAction(
            period=45.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-MAXIMUM Pipeline Status:" && '
                         'echo "📡 Camera topics active:" && '
                         'timeout 3 ros2 topic list | grep image_raw || echo "⚠️ No camera topics found" && '
                         'echo "🔍 Segmentation topics:" && '
                         'timeout 3 ros2 topic list | grep segmentation || echo "⚠️ No segmentation topics" && '
                         'echo "📊 Grid topic:" && '
                         'timeout 3 ros2 topic list | grep ultra_grid || echo "⚠️ No grid topic" && '
                         'echo "🎯 3D Objects:" && '
                         'timeout 3 ros2 topic list | grep objects_3d || echo "⚠️ No 3D objects topic" && '
                         'echo "✅ TARGET: 100+ FPS with PERFECT segmentation + distance + coordinates!"'],
                    output='screen'
                )
            ]
        ),
    ])