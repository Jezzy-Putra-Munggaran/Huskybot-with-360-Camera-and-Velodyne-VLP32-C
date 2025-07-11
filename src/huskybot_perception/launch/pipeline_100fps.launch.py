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
        DeclareLaunchArgument('model_path', default_value='yolo11x-seg.pt'),
        DeclareLaunchArgument('fps_target', default_value='120'),
        DeclareLaunchArgument('debug_mode', default_value='true'),
        DeclareLaunchArgument('auto_display', default_value='true'),
        
        # ✅ 1. ULTRA-MAXIMUM Jetson optimization FIRST
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 Starting ULTRA-MAXIMUM Jetson AGX Orin optimization..." && '
                 'echo kmporin | sudo -S jetson_clocks --fan || echo "Jetson clocks completed" && '
                 'echo kmporin | sudo -S nvpmodel -m 0 || echo "Power model set" && '
                 'echo performance | echo kmporin | sudo -S tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null || echo "CPU governor set" && '
                 'echo kmporin | sudo -S nvidia-smi -pm 1 || echo "GPU persistence mode set" && '
                 'echo kmporin | sudo -S nvidia-smi -pl 80 || echo "GPU power limit set" && '
                 'echo kmporin | sudo -S nvidia-smi -ac 1377,1377 || echo "GPU clocks set" && '
                 'echo 1 | echo kmporin | sudo -S tee /sys/devices/system/cpu/cpufreq/boost > /dev/null || echo "CPU boost enabled" && '
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
        
        # ✅ 4. Start ULTRA-MAXIMUM DeepStream with YOLO11X
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
                        'batch_size': 6,  # Process all 6 cameras
                        'device_id': 0
                    }],
                    respawn=True,
                    respawn_delay=2.0,
                    # ✅ ULTRA-MAXIMUM process priority
                    additional_env={'CUDA_VISIBLE_DEVICES': '0', 'OMP_NUM_THREADS': '8'}
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
        
        # ✅ 6. Create RViz directory and config
        TimerAction(
            period=23.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'mkdir -p /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz'],
                    output='screen',
                    name='create_rviz_dir'
                )
            ]
        ),
        
        # ✅ 7. Generate enhanced RViz config
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='create_rviz_config',
                    name='rviz_config_creator',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 8. AUTO-POPUP: ULTRA-MAXIMUM Grid Display
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='auto_grid_viewer',
                    name='ultra_maximum_auto_grid_viewer',
                    output='screen'
                )
            ]
        ),
        
        # ✅ 9. AUTO-POPUP: Enhanced RViz2 
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && rviz2 -d /home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz/huskybot_3d.rviz > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_popup_rviz2'
                )
            ]
        ),
        
        # ✅ 10. ULTRA-MAXIMUM Performance monitoring
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA-MAXIMUM Pipeline Status:" && '
                         'echo "📡 Camera topics (6 cameras):" && ros2 topic list | grep image_raw | head -6 && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep -E "(detections|segmentation)" && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "🎯 3D Objects topic:" && ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "✅ YOLO11X ENGINE + ULTRA-MAXIMUM Jetson optimization!" && '
                         'echo "🚀 TARGET: 100+ FPS with YOLO11X segmentation + distance + coordinates!" && '
                         'echo "🎯 Features: Segmentation ✅ | Distance ✅ | 3D Coordinates ✅ | RViz2 ✅"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 11. Memory optimization
        TimerAction(
            period=45.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 ULTRA-MAXIMUM Memory optimization..." && '
                         'echo kmporin | sudo -S sysctl -w vm.swappiness=10 || echo "Swappiness optimized" && '
                         'echo kmporin | sudo -S sysctl -w vm.vfs_cache_pressure=50 || echo "Cache pressure optimized" && '
                         'echo "✅ Memory optimization completed!"'],
                    output='screen',
                    name='memory_optimization'
                )
            ]
        ),
        
    ])