#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_multicam_parallel/launch/multicam_parallel_pipeline.launch.py
 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    
    # ✅ Camera configuration - FIXED REAL MAPPING
    camera_configs = [
        {
            'name': 'camera_front',
            'topic': '/camera_front/image_raw',
            'real_name': 'REAR CAMERA',  # FIXED: Real life mapping
            'idx': 0
        },
        {
            'name': 'camera_front_left',
            'topic': '/camera_front_left/image_raw', 
            'real_name': 'LEFT REAR CAMERA',  # FIXED: Real life mapping
            'idx': 1
        },
        {
            'name': 'camera_left',
            'topic': '/camera_left/image_raw',
            'real_name': 'LEFT FRONT CAMERA',  # FIXED: Real life mapping
            'idx': 2
        },
        {
            'name': 'camera_rear',
            'topic': '/camera_rear/image_raw',
            'real_name': 'FRONT CAMERA',  # FIXED: Real life mapping
            'idx': 3
        },
        {
            'name': 'camera_rear_right',
            'topic': '/camera_rear_right/image_raw',
            'real_name': 'RIGHT FRONT CAMERA',  # FIXED: Real life mapping
            'idx': 4
        },
        {
            'name': 'camera_right',
            'topic': '/camera_right/image_raw',
            'real_name': 'RIGHT REAR CAMERA',  # FIXED: Real life mapping
            'idx': 5
        }
    ]
    
    launch_actions = []
    
    # ✅ 1. ULTRA POWER OPTIMIZATION
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER OPTIMIZATION FOR MULTICAM PARALLEL 100+ FPS..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'sudo sysctl -w vm.swappiness=1 && '
                 'echo "⚡ ULTRA POWER OPTIMIZED FOR MULTICAM PARALLEL!"'],
            output='screen',
            name='ultra_power_optimization'
        )
    )
    
    # ✅ 2. Start Camera Drivers
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting Camera Drivers..." && '
                         'ros2 launch huskybot_camera camera.launch.py &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 3. Start LiDAR
    launch_actions.append(
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting LiDAR..." && '
                         'ros2 run velodyne_driver velodyne_driver_node --ros-args '
                         '-p model:=VLP32C -p rpm:=600.0 -p port:=2368 -p device_ip:=192.168.1.201 &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 4. Start Individual Camera Processors (PARALLEL) - FIXED
    base_delay = 15.0
    for i, config in enumerate(camera_configs):
        launch_actions.append(
            TimerAction(
                period=base_delay + i * 2.0,  # Stagger starts
                actions=[
                    Node(
                        package='huskybot_multicam_parallel',
                        executable='single_camera_processor',
                        name=f'{config["name"]}_processor',
                        output='screen',
                        respawn=True,
                        respawn_delay=2.0,
                        parameters=[
                            {'camera_name': config['name']},
                            {'camera_topic': config['topic']},
                            {'camera_real_name': config['real_name']},
                            {'camera_idx': config['idx']},
                            {'use_sim_time': False}
                        ]
                    )
                ]
            )
        )
    
    # ✅ 5. Start Display Node
    launch_actions.append(
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='huskybot_multicam_parallel',
                    executable='multicam_parallel_node',
                    name='multicam_parallel_display',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        )
    )
    
    # ✅ 6. Performance Monitoring
    launch_actions.append(
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 MULTICAM PARALLEL PERFORMANCE STATUS:" && '
                         'echo "📡 Camera Topics:" && '
                         'ros2 topic list | grep -E "(camera.*processed|camera.*image_raw)" && '
                         'echo "📡 Topic Rates:" && '
                         'for topic in /camera_front_processed /camera_rear_processed; do '
                         '  echo "Testing $topic..." && '
                         '  timeout 5 ros2 topic hz $topic 2>/dev/null || echo "❌ No data on $topic"; '
                         'done && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 CPU Usage:" && '
                         'top -bn1 | grep "Cpu(s)" && '
                         'echo "🧠 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "🎯 TARGET: 100+ FPS MULTICAM PARALLEL YOLO SEGMENTATION"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 7. RViz2 for LiDAR
    launch_actions.append(
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
        )
    )
    
    # ✅ 8. GPU Optimization
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 GPU optimization for parallel processing..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export NVIDIA_TF32_OVERRIDE=0 && '
                 'export CUDA_CACHE_MAXSIZE=2147483648 && '
                 'export CUDA_CACHE_DISABLE=0'],
            output='screen'
        )
    )
    
    return LaunchDescription(launch_actions)