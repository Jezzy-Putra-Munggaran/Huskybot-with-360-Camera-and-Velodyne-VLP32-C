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
        
        # Arguments with YOLO11X for MAXIMUM speed
        DeclareLaunchArgument('model_path', default_value='yolo11x-seg.engine'),
        DeclareLaunchArgument('fps_target', default_value='120'),
        DeclareLaunchArgument('debug_mode', default_value='true'),
        DeclareLaunchArgument('auto_display', default_value='true'),
        
        # ✅ 1. EXTREME Jetson optimization FIRST - before everything else
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🚀 Starting EXTREME Jetson AGX Orin optimization..." && '
                 'echo kmporin | sudo -S jetson_clocks --fan && '  # Maximum fan speed
                 'echo kmporin | sudo -S nvpmodel -m 0 && '  # Maximum performance mode
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu5/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu6/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S bash -c "echo performance > /sys/devices/system/cpu/cpu7/cpufreq/scaling_governor" && '
                 'echo kmporin | sudo -S nvidia-smi -pm 1 && '  # Enable persistent mode
                 'echo kmporin | sudo -S nvidia-smi -pl 80 && '  # Max power limit
                 'echo kmporin | sudo -S nvidia-smi -ac 1377,1377 && '  # Max memory/GPU clock
                 'echo kmporin | sudo -S bash -c "echo 1 > /sys/devices/system/cpu/cpufreq/boost" && '
                 # GPU max frequency if paths exist
                 'if [ -f /sys/devices/gpu.0/devfreq/17000000.gv11b/min_freq ]; then '
                 '  echo kmporin | sudo -S bash -c "echo 2000000000 > /sys/devices/gpu.0/devfreq/17000000.gv11b/min_freq"; '
                 'fi && '
                 # CPU max frequency
                 'echo kmporin | sudo -S bash -c "echo 2265600 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2265600 > /sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2265600 > /sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2265600 > /sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2420000 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2420000 > /sys/devices/system/cpu/cpu5/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2420000 > /sys/devices/system/cpu/cpu6/cpufreq/scaling_min_freq" && '
                 'echo kmporin | sudo -S bash -c "echo 2420000 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_min_freq" && '
                 # Memory governor
                 'echo kmporin | sudo -S bash -c "echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked" && '
                 'echo "🔥 EXTREME Jetson optimization COMPLETED - Maximum performance unlocked!"'],
            output='screen',
            name='extreme_jetson_optimization'
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
                    name='maximum_optimized_deepstream',
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
                    respawn_delay=2.0
                )
            ]
        ),
        
        # ✅ 5. Start enhanced fusion
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='enhanced_fusion',
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
        
        # ✅ 8. AUTO-POPUP: MAXIMUM Grid Display
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='auto_grid_viewer',
                    name='maximum_auto_grid_viewer',
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
        
        # ✅ 10. Performance monitoring
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 EXTREME-OPTIMIZED Pipeline Status:" && '
                         'echo "📡 Camera topics (6 cameras):" && ros2 topic list | grep image_raw && '
                         'echo "🔍 Detection topics:" && ros2 topic list | grep detections && '
                         'echo "📊 Grid topic:" && ros2 topic list | grep deepstream_grid && '
                         'echo "🎯 3D Objects topic:" && ros2 topic list | grep objects_3d_pointcloud && '
                         'echo "✅ YOLO11X ENGINE + EXTREME Jetson optimization!" && '
                         'echo "🚀 TARGET: 100+ FPS with YOLO11X segmentation!"'],
                    output='screen'
                )
            ]
        ),
        
    ])