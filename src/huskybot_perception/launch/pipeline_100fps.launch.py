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
        
        # Arguments for ULTRA-MAXIMUM performance
        DeclareLaunchArgument('model_path', default_value='yolo11x-seg.engine'),  # ✅ Use .engine
        DeclareLaunchArgument('fps_target', default_value='120'),
        DeclareLaunchArgument('debug_mode', default_value='true'),
        DeclareLaunchArgument('auto_display', default_value='true'),
        
        # ✅ 1. MAXIMUM Jetson optimization
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 Starting ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                 'echo kmporin | sudo -S /usr/bin/jetson_clocks --fan && '
                 'echo kmporin | sudo -S nvpmodel -m 0 && '
                 'echo kmporin | sudo -S nvidia-smi -pm 1 && '
                 'echo kmporin | sudo -S nvidia-smi -pl 55 && '
                 'echo kmporin | sudo -S sysctl -w vm.swappiness=1 && '
                 'echo kmporin | sudo -S sysctl -w vm.vfs_cache_pressure=10 && '
                 'echo kmporin | sudo -S sh -c "echo never > /sys/kernel/mm/transparent_hugepage/enabled" && '
                 'for i in {0..11}; do echo kmporin | sudo -S sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor"; done && '
                 'echo kmporin | sudo -S nvidia-smi -lgc 2100,2100 && '
                 'echo kmporin | sudo -S nvidia-smi -lmc 6251,6251 && '
                 'echo "🔥 ULTRA-MAXIMUM Jetson optimization COMPLETED!"'],
            output='screen',
            name='ultra_maximum_jetson_optimization'
        ),
        
        # ✅ 2. Start Velodyne LiDAR
        TimerAction(
            period=5.0,
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
        
        # ✅ 4. Start MAXIMUM DeepStream with YOLO11X
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_deepstream',
                    executable='deepstream_yolo_node',
                    name='ultra_maximum_deepstream',
                    output='screen',
                    parameters=[{
                        'model_engine': LaunchConfiguration('model_path'),
                        'fps_target': LaunchConfiguration('fps_target'),
                        'input_width': 640,
                        'input_height': 640,
                        'batch_size': 6,
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=2.0,
                    additional_env={
                        'CUDA_VISIBLE_DEVICES': '0', 
                        'OMP_NUM_THREADS': '12',
                        'CUDA_LAUNCH_BLOCKING': '0',
                        'PYTORCH_CUDA_ALLOC_CONF': 'max_split_size_mb:1024',
                        'CUDA_CACHE_DISABLE': '0',
                        'CUDA_DEVICE_MAX_CONNECTIONS': '64'
                    }
                )
            ]
        ),
        
        # ✅ 5. Start ULTRA-enhanced fusion
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='ultra_enhanced_fusion',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0
                )
            ]
        ),
        
        # ✅ 6. AUTO-POPUP: MAXIMUM Grid Display  
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='auto_grid_viewer',
                    name='ultra_maximum_auto_grid_viewer',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. AUTO-POPUP: Enhanced RViz2 with LiDAR
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'rviz2 -f velodyne --ros-args -p use_sim_time:=false > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_popup_rviz2_lidar'
                )
            ]
        ),
        
        # ✅ 8. Performance monitoring with detailed info
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 MAXIMUM Pipeline Status:" && '
                         'echo "📡 Camera topics (6 cameras):" && ros2 topic list | grep image_raw | head -6 && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep -E "(detections|segmentation)" && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "🎯 3D Objects topic:" && ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "🔴 LiDAR topics:" && ros2 topic list | grep -E "(scan|velodyne_points)" && '
                         'echo "✅ YOLO11X-seg.engine + MAXIMUM Jetson optimization!" && '
                         'echo "🚀 TARGET: 100+ FPS with PERFECT segmentation + distance + coordinates!" && '
                         'echo "🎯 Features: Segmentation ✅ | Distance ✅ | 3D Coordinates ✅ | RViz2 ✅" && '
                         'echo "💪 GPU/RAM: MAXIMUM UTILIZATION | Power: FULL MODE"'],
                    output='screen'
                )
            ]
        ),
    ])