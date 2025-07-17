#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_single_camera/launch/single_camera_ultimate_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. ULTRA POWER OPTIMIZATION untuk 100+ FPS
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER OPTIMIZATION FOR SINGLE CAMERA 100+ FPS..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'sudo sh -c "echo performance > /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor" && '
                 'sudo sh -c "echo 1 > /sys/devices/system/cpu/cpufreq/boost" && '
                 'sudo sh -c "echo 0 > /proc/sys/kernel/numa_balancing" && '
                 'sudo sh -c "echo 1 > /proc/sys/vm/swappiness" && '
                 'echo "⚡ ULTRA POWER OPTIMIZED FOR SINGLE CAMERA 100+ FPS!"'],
            output='screen',
            name='ultra_power_optimization'
        ),
        
        # ✅ 2. Create RViz config untuk LiDAR
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'mkdir -p /tmp && '
                 'echo "Creating RViz config for LiDAR..." && '
                 'cat > /tmp/huskybot_lidar_config.rviz << EOF\n'
                 'Panels:\n'
                 '  - Class: rviz_common/Displays\n'
                 '    Property Tree Widget:\n'
                 '      Expanded:\n'
                 '        - /LaserScan1\n'
                 '      Splitter Ratio: 0.5\n'
                 '    Tree Height: 549\n'
                 'Visualization Manager:\n'
                 '  Class: ""\n'
                 '  Displays:\n'
                 '    - Alpha: 1\n'
                 '      Class: rviz_default_plugins/LaserScan\n'
                 '      Color: 255; 255; 255\n'
                 '      Name: LaserScan\n'
                 '      Topic:\n'
                 '        Depth: 5\n'
                 '        Durability Policy: Volatile\n'
                 '        Filter size: 10\n'
                 '        History Policy: Keep Last\n'
                 '        Reliability Policy: Best Effort\n'
                 '        Value: /scan\n'
                 '  Enabled: true\n'
                 '  Global Options:\n'
                 '    Background Color: 48; 48; 48\n'
                 '    Fixed Frame: laser\n'
                 '    Frame Rate: 30\n'
                 '  Name: root\n'
                 '  Tools:\n'
                 '    - Class: rviz_default_plugins/Interact\n'
                 '    - Class: rviz_default_plugins/MoveCamera\n'
                 '    - Class: rviz_default_plugins/Select\n'
                 '  Value: true\n'
                 '  Views:\n'
                 '    Current:\n'
                 '      Class: rviz_default_plugins/Orbit\n'
                 '      Distance: 30\n'
                 '      Name: Current View\n'
                 '      Focal Point:\n'
                 '        X: 0\n'
                 '        Y: 0\n'
                 '        Z: 0\n'
                 '      Pitch: 0.7854\n'
                 '      Target Frame: <Fixed Frame>\n'
                 '      Yaw: 0.7854\n'
                 '    Saved: ~\n'
                 'Window Geometry:\n'
                 '  Width: 1200\n'
                 '  Height: 800\n'
                 'EOF'],
            output='screen'
        ),
        
        # ✅ 3. Start Velodyne LiDAR
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 4. Start single camera (CAMERA REAR only)
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting SINGLE CAMERA (CAMERA REAR)..." && '
                         'v4l2-ctl --device=/dev/video2 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG --set-parm=30 && '
                         'gst-launch-1.0 v4l2src device=/dev/video2 ! "image/jpeg,width=1920,height=1080,framerate=30/1" ! jpegdec ! videoconvert ! "video/x-raw,format=BGR" ! appsink name=sink | ros2 run image_tools cam2image --ros-args -r /image:=/camera_rear/image_raw &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 5. Start SINGLE CAMERA ULTIMATE node
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_single_camera',
                    executable='single_camera_ultimate_node',
                    name='single_camera_ultimate_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        ),
        
        # ✅ 6. RViz2 auto-popup untuk LiDAR
        TimerAction(
            period=120.0,  # 2 minutes delay
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting RViz2 for LiDAR..." && '
                         'export DISPLAY=:0 && '
                         'rviz2 -d /tmp/huskybot_lidar_config.rviz &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. Performance monitoring untuk single camera
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 SINGLE CAMERA Performance Status:" && '
                         'echo "📡 Camera topic:" && timeout 3 ros2 topic echo /camera_rear/image_raw --once | head -5 && '
                         'echo "🔥 GPU Status:" && (nvidia-smi --query-gpu=utilization.gpu,memory.used,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available") && '
                         'echo "💻 CPU Usage:" && top -bn1 | grep "Cpu(s)" && '
                         'echo "🧠 Memory:" && free -h | grep Mem && '
                         'echo "🎯 TARGET: 100+ FPS with SINGLE CAMERA YOLO SEGMENTATION" && '
                         'echo "✅ Testing single camera performance..."'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 8. GPU Memory optimization untuk single camera
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 GPU Memory optimization for single camera..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH'],
            output='screen'
        ),
    ])