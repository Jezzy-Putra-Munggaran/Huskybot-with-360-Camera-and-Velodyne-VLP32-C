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
    
    # ✅ 1. ULTRA POWER OPTIMIZATION + WIDER FOV CAMERA SETTINGS
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER + WIDER FOV OPTIMIZATION..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'sudo sysctl -w vm.swappiness=1 && '
                 # ✅ WIDER FOV: Set camera driver parameters for maximum FOV
                 'export ARDUCAM_FOV_MODE=WIDE && '  # Set wide FOV mode
                 'export ARDUCAM_RESOLUTION=HIGH && '  # High resolution for better quality
                 'export V4L2_WIDE_FOV=1 && '  # Enable wide FOV in V4L2
                 'echo "⚡ ULTRA POWER + WIDER FOV OPTIMIZED!"'],
            output='screen',
            name='ultra_power_wider_fov_optimization'
        )
    )
    
    # ✅ 2. Start Camera Drivers with WIDER FOV settings
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting Camera Drivers with WIDER FOV..." && '
                         # ✅ WIDER FOV: Configure camera drivers for maximum FOV
                         'for i in {0..5}; do '
                         '  if [ -e /dev/video$i ]; then '
                         '    echo "Configuring /dev/video$i for WIDER FOV..." && '
                         '    v4l2-ctl -d /dev/video$i --set-ctrl=zoom_absolute=100 2>/dev/null || true && '  # Minimum zoom
                         '    v4l2-ctl -d /dev/video$i --set-ctrl=white_balance_auto_preset=1 2>/dev/null || true && '
                         '    v4l2-ctl -d /dev/video$i --set-fmt-video=width=1920,height=1080,pixelformat=YUYV 2>/dev/null || true; '
                         '  fi; '
                         'done && '
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
    
    # ✅ 4. Start Individual Camera Processors with WIDER FOV support
    base_delay = 15.0
    for i, config in enumerate(camera_configs):
        launch_actions.append(
            TimerAction(
                period=base_delay + i * 2.0,
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
                        ],
                        # ✅ WIDER FOV: Environment variables for optimal processing
                        additional_env={
                            'CUDA_VISIBLE_DEVICES': '0',
                            'OPENCV_FOV_CORRECTION': '1',      # Enable FOV correction
                            'ARDUCAM_WIDE_MODE': '1',          # Wide mode processing
                            'OMP_NUM_THREADS': '4'
                        }
                    )
                ]
            )
        )
    
    # ✅ 5. Start Display Node (unchanged)
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
    
    # ✅ 6. Enhanced Performance Monitoring
    launch_actions.append(
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 WIDER FOV MULTICAM PARALLEL STATUS:" && '
                         'echo "📡 Camera Topics:" && '
                         'ros2 topic list | grep -E "(camera.*processed|camera.*image_raw)" && '
                         'echo "📡 FOV Status:" && '
                         'for i in {0..5}; do '
                         '  if [ -e /dev/video$i ]; then '
                         '    echo "Camera /dev/video$i FOV settings:" && '
                         '    v4l2-ctl -d /dev/video$i --get-ctrl=zoom_absolute 2>/dev/null || echo "  Zoom: Not available"; '
                         '  fi; '
                         'done && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "🎯 TARGET: WIDER FOV + 100+ FPS MULTICAM PARALLEL"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 7. RViz2 for LiDAR (unchanged)
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
    
    return LaunchDescription(launch_actions)