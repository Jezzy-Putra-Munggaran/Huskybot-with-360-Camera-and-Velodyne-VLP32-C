#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera_rear/launch/camera_rear_ultimate_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. ULTRA POWER OPTIMIZATION
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER OPTIMIZATION FOR CAMERA REAR 100+ FPS..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'sudo sysctl -w vm.swappiness=1 && '
                 'echo "⚡ ULTRA POWER OPTIMIZED FOR CAMERA REAR!"'],
            output='screen',
            name='ultra_power_optimization'
        ),
        
        # ✅ 2. Start ROS_DEEP_LEARNING (CSI Camera)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting ROS_DEEP_LEARNING CSI Camera..." && '
                         'ros2 run ros_deep_learning video_source '
                         '--input=csi://2 '
                         '--output=image_raw '
                         '--width=1920 '
                         '--height=1080 '
                         '--framerate=30 '
                         '--remap image_raw:=/camera_rear/image_raw &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 3. Start HUSKYBOT_CAMERA (if needed)
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ]),
                    launch_arguments={
                        'camera_count': '1',
                        'camera_topics': '["/camera_rear/image_raw"]'
                    }.items()
                )
            ]
        ),
        
        # ✅ 4. Start LiDAR
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 5. Start CAMERA REAR ULTIMATE NODE
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='huskybot_camera_rear',
                    executable='camera_rear_ultimate_node',
                    name='camera_rear_ultimate_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        ),
        
        # ✅ 6. TOPIC MONITORING
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 CAMERA REAR TOPIC MONITORING:" && '
                         'echo "📡 Available topics:" && '
                         'ros2 topic list | grep -E "(camera|image)" && '
                         'echo "📡 Camera topic rate:" && '
                         'timeout 10 ros2 topic hz /camera_rear/image_raw 2>/dev/null || echo "❌ No rate data" && '
                         'echo "📡 Display topic rate:" && '
                         'timeout 10 ros2 topic hz /camera_rear_display 2>/dev/null || echo "❌ No display data"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. PERFORMANCE MONITORING
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 CAMERA REAR PERFORMANCE STATUS:" && '
                         'echo "📡 Camera FPS:" && '
                         'timeout 8 ros2 topic hz /camera_rear/image_raw 2>/dev/null || echo "❌ No camera data" && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 CPU Usage:" && '
                         'top -bn1 | grep "Cpu(s)" && '
                         'echo "🧠 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "🎯 TARGET: 100+ FPS CAMERA REAR YOLO SEGMENTATION"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 8. RViz2 untuk LiDAR
        TimerAction(
            period=120.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting RViz2 for LiDAR..." && '
                         'export DISPLAY=:0 && '
                         'rviz2 -d /opt/ros/humble/share/rviz_common/default_plugins/robot_model.rviz &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 9. GPU optimization
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 GPU optimization..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export NVIDIA_TF32_OVERRIDE=0 && '
                 'export CUDA_CACHE_MAXSIZE=2147483648'],
            output='screen'
        ),
    ])