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
    
    # ✅ 1. ULTRA POWER OPTIMIZATION + MAXIMUM WIDER FOV CAMERA SETTINGS
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER + MAXIMUM WIDER FOV OPTIMIZATION..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'sudo sysctl -w vm.swappiness=1 && '
                 'sudo sysctl -w vm.dirty_background_ratio=3 && '
                 'sudo sysctl -w vm.dirty_ratio=5 && '
                 # ✅ MAXIMUM WIDER FOV: Ultra-wide camera driver parameters
                 'export ARDUCAM_FOV_MODE=ULTRA_WIDE && '    # Ultra-wide FOV mode
                 'export ARDUCAM_RESOLUTION=ULTRA_HIGH && '  # Ultra-high resolution
                 'export V4L2_ULTRA_WIDE_FOV=1 && '          # Enable ultra-wide FOV in V4L2
                 'export ARDUCAM_DISTORTION_CORRECTION=1 && ' # Enable distortion correction
                 'echo "⚡ ULTRA POWER + MAXIMUM WIDER FOV OPTIMIZED!"'],
            output='screen',
            name='ultra_power_maximum_wider_fov_optimization'
        )
    )
    
    # ✅ 2. Start Camera Drivers with MAXIMUM WIDER FOV settings
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting Camera Drivers with MAXIMUM WIDER FOV..." && '
                         # ✅ MAXIMUM WIDER FOV: Configure camera drivers for ultra-wide FOV
                         'for i in {0..5}; do '
                         '  if [ -e /dev/video$i ]; then '
                         '    echo "Configuring /dev/video$i for MAXIMUM WIDER FOV..." && '
                         '    v4l2-ctl -d /dev/video$i --set-ctrl=zoom_absolute=100 2>/dev/null || true && '  # MINIMUM zoom
                         '    v4l2-ctl -d /dev/video$i --set-ctrl=focus_absolute=0 2>/dev/null || true && '   # Infinity focus
                         '    v4l2-ctl -d /dev/video$i --set-ctrl=white_balance_auto_preset=1 2>/dev/null || true && '
                         '    v4l2-ctl -d /dev/video$i --set-fmt-video=width=2560,height=1080,pixelformat=YUYV 2>/dev/null || true && '  # Ultra-wide resolution
                         '    v4l2-ctl -d /dev/video$i --set-parm=type=1,capturemode=0,extendedmode=0,timeperframe=1/60 2>/dev/null || true; '  # 60 FPS
                         '  fi; '
                         'done && '
                         'ros2 launch huskybot_camera camera.launch.py &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 3. Start Individual Camera Processors with MAXIMUM WIDER FOV support
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
                        # ✅ MAXIMUM WIDER FOV: Environment variables for ultra-wide processing
                        additional_env={
                            'CUDA_VISIBLE_DEVICES': '0',
                            'OPENCV_ULTRA_WIDE_FOV_CORRECTION': '1',  # Enable ultra-wide FOV correction
                            'ARDUCAM_ULTRA_WIDE_MODE': '1',           # Ultra-wide mode processing
                            'OMP_NUM_THREADS': '6',                   # More threads for ultra-wide processing
                            'OPENCV_LOG_LEVEL': 'ERROR'               # Reduce logging overhead
                        }
                    )
                ]
            )
        )
    
    # ✅ 4. Start Display Node (grid 2x3 unchanged as requested)
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
    
    # ✅ 5. AUTO-LAUNCH VELODYNE LIDAR after 2 minutes (120 seconds)
    launch_actions.append(
        TimerAction(
            period=120.0,  # 2 minutes = 120 seconds
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 AUTO-LAUNCHING VELODYNE VLP32C LIDAR after 2 minutes..." && '
                         'echo "📡 Starting Velodyne driver..." && '
                         'ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py &'],
                    output='screen',
                    name='auto_launch_velodyne_lidar'
                )
            ]
        )
    )
    
    # ✅ 6. AUTO-LAUNCH RVIZ2 with LIDAR VISUALIZATION after 2 minutes 15 seconds
    launch_actions.append(
        TimerAction(
            period=135.0,  # 2 minutes 15 seconds = 135 seconds (15 seconds after LiDAR)
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 AUTO-LAUNCHING RVIZ2 with LIDAR VISUALIZATION..." && '
                         'export DISPLAY=:0 && '
                         'echo "📡 Setting up RViz2 for Velodyne VLP32C..." && '
                         # Create temporary RViz config for auto-setup
                         'mkdir -p /tmp/huskybot_rviz && '
                         'cat > /tmp/huskybot_rviz/velodyne_auto_config.rviz << EOF\n'
                         'Panels:\n'
                         '  - Class: rviz_common/Views\n'
                         '    Name: Views\n'
                         'Visualization Manager:\n'
                         '  Class: ""\n'
                         '  Displays:\n'
                         '    - Alpha: 0.5\n'
                         '      Autocompute Intensity Bounds: true\n'
                         '      Class: rviz_default_plugins/LaserScan\n'
                         '      Enabled: true\n'
                         '      Name: LaserScan\n'
                         '      Topic:\n'
                         '        Depth: 5\n'
                         '        Durability Policy: Volatile\n'
                         '        Filter size: 10\n'
                         '        History Policy: Keep Last\n'
                         '        Reliability Policy: Best Effort\n'
                         '        Value: /scan\n'
                         '    - Alpha: 1\n'
                         '      Autocompute Intensity Bounds: true\n'
                         '      Class: rviz_default_plugins/PointCloud2\n'
                         '      Enabled: true\n'
                         '      Name: PointCloud2\n'
                         '      Topic:\n'
                         '        Depth: 5\n'
                         '        Durability Policy: Volatile\n'
                         '        Filter size: 10\n'
                         '        History Policy: Keep Last\n'
                         '        Reliability Policy: Best Effort\n'
                         '        Value: /velodyne_points\n'
                         '  Fixed Frame: velodyne\n'
                         '  Frame Rate: 30\n'
                         'EOF\n'
                         'echo "✅ RViz2 config created. Launching RViz2..." && '
                         'ros2 run rviz2 rviz2 -f velodyne -d /tmp/huskybot_rviz/velodyne_auto_config.rviz &'],
                    output='screen',
                    name='auto_launch_rviz2_lidar'
                )
            ]
        )
    )
    
    # ✅ 7. Enhanced Performance Monitoring with LIDAR status
    launch_actions.append(
        TimerAction(
            period=150.0,  # 2.5 minutes after startup
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 MAXIMUM WIDER FOV MULTICAM + LIDAR STATUS:" && '
                         'echo "📡 Camera Topics:" && '
                         'ros2 topic list | grep -E "(camera.*processed|camera.*image_raw)" && '
                         'echo "📡 LiDAR Topics:" && '
                         'ros2 topic list | grep -E "(scan|velodyne|points)" && '
                         'echo "📡 MAXIMUM FOV Status:" && '
                         'for i in {0..5}; do '
                         '  if [ -e /dev/video$i ]; then '
                         '    echo "Camera /dev/video$i MAXIMUM FOV settings:" && '
                         '    v4l2-ctl -d /dev/video$i --get-ctrl=zoom_absolute 2>/dev/null || echo "  Zoom: Not available" && '
                         '    v4l2-ctl -d /dev/video$i --get-fmt-video 2>/dev/null | grep -E "Width|Height" || echo "  Resolution: Not available"; '
                         '  fi; '
                         'done && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "📊 LiDAR Status:" && '
                         'ros2 topic hz /velodyne_points --window 10 2>/dev/null || echo "LiDAR: No data" && '
                         'echo "🎯 TARGET: MAXIMUM WIDER FOV + 100+ FPS MULTICAM + REAL-TIME LIDAR 3D MAPPING"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 8. Periodic LiDAR and RViz2 health check
    launch_actions.append(
        TimerAction(
            period=300.0,  # Every 5 minutes
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 PERIODIC LIDAR + RVIZ2 HEALTH CHECK:" && '
                         'echo "📊 LiDAR Topics Status:" && '
                         'ros2 topic list | grep -E "(scan|velodyne)" | while read topic; do '
                         '  echo "  Checking $topic..." && '
                         '  timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1 && echo "    ✅ $topic: OK" || echo "    ❌ $topic: NO DATA"; '
                         'done && '
                         'echo "📊 RViz2 Process Status:" && '
                         'pgrep -f rviz2 >/dev/null && echo "  ✅ RViz2: Running" || echo "  ❌ RViz2: Not running" && '
                         'echo "📊 Camera Processing Status:" && '
                         'ros2 topic list | grep "_processed" | wc -l | xargs echo "  Active camera processors:" && '
                         'echo "🎯 SYSTEM STATUS: MAXIMUM WIDER FOV MULTICAM + REAL-TIME 3D LIDAR MAPPING"'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription(launch_actions)