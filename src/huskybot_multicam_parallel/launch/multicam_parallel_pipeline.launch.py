#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_multicam_parallel/launch/multicam_parallel_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    
    # ✅ Camera configuration - REAL MAPPING CORRECTED
    camera_configs = [
        {
            'name': 'camera_front',
            'topic': '/camera_front/image_raw',
            'real_name': 'REAR CAMERA',
            'idx': 0
        },
        {
            'name': 'camera_front_left',
            'topic': '/camera_front_left/image_raw', 
            'real_name': 'LEFT REAR CAMERA',
            'idx': 1
        },
        {
            'name': 'camera_left',
            'topic': '/camera_left/image_raw',
            'real_name': 'LEFT FRONT CAMERA',
            'idx': 2
        },
        {
            'name': 'camera_rear',
            'topic': '/camera_rear/image_raw',
            'real_name': 'FRONT CAMERA',
            'idx': 3
        },
        {
            'name': 'camera_rear_right',
            'topic': '/camera_rear_right/image_raw',
            'real_name': 'RIGHT FRONT CAMERA',
            'idx': 4
        },
        {
            'name': 'camera_right',
            'topic': '/camera_right/image_raw',
            'real_name': 'RIGHT REAR CAMERA',
            'idx': 5
        }
    ]
    
    launch_actions = []
    
    # ✅ 1. ULTRA JETSON POWER OPTIMIZATION
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA JETSON POWER OPTIMIZATION..." && '
                 'sudo nvpmodel -m 0 && '  # Max performance mode
                 'sudo jetson_clocks --fan && '  # Max clocks
                 # CPU optimization
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 # Memory optimization for MAXIMUM parallel processing
                 'sudo sysctl -w vm.swappiness=1 && '
                 'sudo sysctl -w vm.dirty_background_ratio=5 && '
                 'sudo sysctl -w vm.dirty_ratio=10 && '
                 'sudo sysctl -w kernel.sched_migration_cost_ns=5000000 && '  # Better CPU scheduling
                 'sudo sysctl -w kernel.sched_min_granularity_ns=10000000 && '  # Better CPU scheduling
                 # GPU optimization
                 'sudo sh -c "echo 1 > /sys/devices/gpu.0/power_control_enable" 2>/dev/null || true && '
                 'echo "⚡ JETSON ULTRA OPTIMIZED FOR 100+ FPS!"'],
            output='screen'
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
            period=5.0,
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
    
    # ✅ 4. Start ULTRA OPTIMIZED Camera Processors (TRUE PARALLEL)
    base_delay = 8.0
    for i, config in enumerate(camera_configs):
        launch_actions.append(
            TimerAction(
                period=base_delay + i * 0.5,  # Faster startup
                actions=[
                    Node(
                        package='huskybot_multicam_parallel',
                        executable='single_camera_processor',
                        name=f'{config["name"]}_ultra_processor',
                        output='screen',
                        respawn=True,
                        respawn_delay=0.5,
                        parameters=[
                            {'camera_name': config['name']},
                            {'camera_topic': config['topic']},
                            {'camera_real_name': config['real_name']},
                            {'camera_idx': config['idx']},
                            {'use_sim_time': False}
                        ],
                        # ✅ ULTRA OPTIMIZATION Environment variables
                        additional_env={
                            'CUDA_VISIBLE_DEVICES': '0',
                            'OMP_NUM_THREADS': '4',        # Optimized CPU threads
                            'OPENCV_LOG_LEVEL': 'ERROR',   # Reduce logging overhead
                            'PYTHONUNBUFFERED': '1'        # Faster Python output
                        }
                    )
                ]
            )
        )
    
    # ✅ 5. Start Display Node
    launch_actions.append(
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='huskybot_multicam_parallel',
                    executable='multicam_parallel_node',
                    name='multicam_ultra_display',
                    output='screen',
                    respawn=True,
                    respawn_delay=0.5,
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
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ULTRA PERFORMANCE STATUS:" && '
                         'echo "📡 Camera Topics:" && '
                         'ros2 topic list | grep camera && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "🎯 TARGET: 100+ FPS ULTRA PARALLEL MULTICAM"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 7. RViz2 for LiDAR
    launch_actions.append(
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting RViz2 for LiDAR 3D Mapping..." && '
                         'export DISPLAY=:0 && '
                         'rviz2 &'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription(launch_actions)