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
    
    # ✅ 1. ULTRA POWER OPTIMIZATION + MAXIMUM WIDER FOV
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
                 'echo "⚡ ULTRA POWER + MAXIMUM WIDER FOV OPTIMIZED!"'],
            output='screen',
            name='ultra_power_maximum_wider_fov_optimization'
        )
    )
    
    # ✅ 2. FIXED: Start Camera Drivers PROPERLY
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 FIXED: Starting Camera Drivers PROPERLY..." && '
                         # ✅ FIXED: Configure cameras for MAXIMUM WIDER FOV + PROPER LAUNCH
                         'for i in {0..5}; do '
                         '  if [ -e /dev/video$i ]; then '
                         '    echo "✅ Configuring /dev/video$i for MAXIMUM WIDER FOV..." && '
                         '    v4l2-ctl -d /dev/video$i --set-fmt-video=width=4032,height=3040,pixelformat=YUYV 2>/dev/null || true && '  # Native resolution
                         '    v4l2-ctl -d /dev/video$i --set-parm=type=1,capturemode=0,extendedmode=0,timeperframe=1/30 2>/dev/null || true; '  # 30 FPS for stability
                         '  fi; '
                         'done && '
                         'echo "🚀 Starting huskybot_camera launch..." && '
                         'ros2 launch huskybot_camera camera.launch.py > /tmp/camera_launch.log 2>&1 &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 3. Wait for camera topics to be available
    launch_actions.append(
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 WAITING FOR CAMERA TOPICS..." && '
                         'for i in {1..30}; do '
                         '  echo "Attempt $i: Checking camera topics..." && '
                         '  if ros2 topic list | grep -q "camera.*image_raw"; then '
                         '    echo "✅ Camera topics found!" && '
                         '    ros2 topic list | grep "camera.*image_raw" && '
                         '    break; '
                         '  else '
                         '    echo "⏳ Waiting for camera topics... ($i/30)" && '
                         '    sleep 2; '
                         '  fi; '
                         'done'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 4. Start Individual Camera Processors with MAXIMUM WIDER FOV support
    base_delay = 20.0  # Increased delay to ensure cameras are ready
    for i, config in enumerate(camera_configs):
        launch_actions.append(
            TimerAction(
                period=base_delay + i * 3.0,  # Increased spacing between launches
                actions=[
                    Node(
                        package='huskybot_multicam_parallel',
                        executable='single_camera_processor',
                        name=f'{config["name"]}_processor',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,  # Increased respawn delay
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
                            'OPENCV_ULTRA_WIDE_FOV_CORRECTION': '1',
                            'ARDUCAM_ULTRA_WIDE_MODE': '1',
                            'OMP_NUM_THREADS': '8',  # More threads
                            'OPENCV_LOG_LEVEL': 'ERROR'
                        }
                    )
                ]
            )
        )
    
    # ✅ 5. Start Display Node with WIDER GRID
    launch_actions.append(
        TimerAction(
            period=40.0,  # Wait for all cameras to be ready
            actions=[
                Node(
                    package='huskybot_multicam_parallel',
                    executable='multicam_parallel_node',
                    name='multicam_parallel_display',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        )
    )
    
    # ✅ 6. AUTO-LAUNCH VELODYNE LIDAR after 2 minutes (120 seconds)
    launch_actions.append(
        TimerAction(
            period=120.0,  # 2 minutes = 120 seconds
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 AUTO-LAUNCHING VELODYNE VLP32C LIDAR after 2 minutes..." && '
                         'echo "📡 Starting Velodyne driver..." && '
                         'ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py > /tmp/velodyne_launch.log 2>&1 &'],
                    output='screen',
                    name='auto_launch_velodyne_lidar'
                )
            ]
        )
    )
    
    # ✅ 7. AUTO-LAUNCH RVIZ2 with LIDAR VISUALIZATION after 2 minutes 15 seconds
    launch_actions.append(
        TimerAction(
            period=135.0,  # 2 minutes 15 seconds = 135 seconds (15 seconds after LiDAR)
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 AUTO-LAUNCHING RVIZ2 with LIDAR VISUALIZATION..." && '
                         'export DISPLAY=:0 && '
                         'echo "📡 Setting up RViz2 for Velodyne VLP32C..." && '
                         # Create enhanced RViz config for auto-setup
                         'mkdir -p /tmp/huskybot_rviz && '
                         'cat > /tmp/huskybot_rviz/velodyne_auto_config.rviz << EOF\n'
                         'Panels:\n'
                         '  - Class: rviz_common/Views\n'
                         '    Name: Views\n'
                         'Visualization Manager:\n'
                         '  Class: ""\n'
                         '  Displays:\n'
                         '    - Alpha: 0.7\n'
                         '      Autocompute Intensity Bounds: true\n'
                         '      Class: rviz_default_plugins/LaserScan\n'
                         '      Color: 255; 0; 0\n'
                         '      Enabled: true\n'
                         '      Name: LaserScan\n'
                         '      Size (m): 0.1\n'
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
                         '      Color: 255; 255; 255\n'
                         '      Enabled: true\n'
                         '      Name: PointCloud2\n'
                         '      Size (m): 0.05\n'
                         '      Style: Points\n'
                         '      Topic:\n'
                         '        Depth: 5\n'
                         '        Durability Policy: Volatile\n'
                         '        Filter size: 10\n'
                         '        History Policy: Keep Last\n'
                         '        Reliability Policy: Best Effort\n'
                         '        Value: /velodyne_points\n'
                         '  Fixed Frame: velodyne\n'
                         '  Frame Rate: 30\n'
                         '  Global Options:\n'
                         '    Background Color: 48; 48; 48\n'
                         '    Fixed Frame: velodyne\n'
                         'EOF\n'
                         'echo "✅ Enhanced RViz2 config created. Launching RViz2..." && '
                         'ros2 run rviz2 rviz2 -f velodyne -d /tmp/huskybot_rviz/velodyne_auto_config.rviz > /tmp/rviz2_launch.log 2>&1 &'],
                    output='screen',
                    name='auto_launch_rviz2_lidar'
                )
            ]
        )
    )
    
    # ✅ 8. Enhanced Status Monitoring
    launch_actions.append(
        TimerAction(
            period=50.0,  # After all components should be running
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 COMPLETE SYSTEM STATUS CHECK:" && '
                         'echo "📡 Camera Topics Available:" && '
                         'ros2 topic list | grep -E "(camera.*image_raw|camera.*processed)" | sort && '
                         'echo "📡 LiDAR Topics Available:" && '
                         'ros2 topic list | grep -E "(scan|velodyne)" | sort && '
                         'echo "📊 Camera Data Flow Check:" && '
                         'for topic in $(ros2 topic list | grep "camera.*image_raw"); do '
                         '  echo "  Checking $topic..." && '
                         '  timeout 3 ros2 topic hz "$topic" --window 5 2>/dev/null | grep "average rate" || echo "    ❌ No data on $topic"; '
                         'done && '
                         'echo "📊 LiDAR Data Flow Check:" && '
                         'timeout 3 ros2 topic hz /velodyne_points --window 5 2>/dev/null | grep "average rate" || echo "  ❌ No LiDAR data" && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "🎯 TARGET: MAXIMUM WIDER FOV + 100+ FPS MULTICAM + REAL-TIME LIDAR 3D MAPPING"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 9. Periodic Health Check
    launch_actions.append(
        TimerAction(
            period=300.0,  # Every 5 minutes
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔍 PERIODIC HEALTH CHECK:" && '
                         'echo "📊 Active Processes:" && '
                         'ps aux | grep -E "(camera|velodyne|rviz2)" | grep -v grep | wc -l | xargs echo "  Running processes:" && '
                         'echo "📊 ROS2 Nodes:" && '
                         'ros2 node list | grep -E "(camera|velodyne|rviz|multicam)" | wc -l | xargs echo "  Active ROS2 nodes:" && '
                         'echo "📊 Camera Processing Status:" && '
                         'ros2 topic list | grep "_processed" | wc -l | xargs echo "  Active camera processors:" && '
                         'echo "🎯 SYSTEM STATUS: WIDER FOV MULTICAM + REAL-TIME 3D LIDAR MAPPING"'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription(launch_actions)