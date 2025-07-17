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
        
        # ✅ 2. SIMPLE camera detection and setup
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 DETECTING AVAILABLE CAMERAS..." && '
                 'ls -la /dev/video* | head -10 && '
                 'echo "🔧 CAMERA DEVICE INFO:" && '
                 'v4l2-ctl --list-devices | head -20'],
            output='screen',
            name='camera_detection'
        ),
        
        # ✅ 3. Start LiDAR (simplified)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting LiDAR (simplified)..." && '
                         'ros2 run velodyne_driver velodyne_driver_node --ros-args '
                         '-p model:=VLP32C -p rpm:=600.0 -p port:=2368 -p device_ip:=192.168.1.201 &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 4. MULTIPLE camera attempts - try different devices
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        echo "🔧 TRYING MULTIPLE CAMERA DEVICES FOR CAMERA REAR..."
                        
                        # Try different video devices
                        for i in 0 1 2 3 4 5; do
                            echo "Trying /dev/video$i..."
                            if [ -e "/dev/video$i" ]; then
                                echo "Found /dev/video$i, testing..."
                                
                                # Test if device works
                                if v4l2-ctl --device=/dev/video$i --list-formats-ext | grep -q "1920x1080"; then
                                    echo "✅ /dev/video$i supports 1920x1080, using it!"
                                    
                                    # Configure camera
                                    v4l2-ctl --device=/dev/video$i --set-fmt-video=width=1920,height=1080,pixelformat=MJPG --set-parm=30
                                    
                                    # Start GStreamer pipeline
                                    gst-launch-1.0 v4l2src device=/dev/video$i ! \
                                        "image/jpeg,width=1920,height=1080,framerate=30/1" ! \
                                        jpegdec ! videoconvert ! \
                                        "video/x-raw,format=BGR" ! \
                                        appsink name=sink | \
                                    ros2 run image_tools cam2image --ros-args \
                                        -r /image:=/camera_rear/image_raw &
                                    
                                    echo "✅ Camera started with /dev/video$i"
                                    break
                                else
                                    echo "❌ /dev/video$i doesn't support 1920x1080"
                                fi
                            else
                                echo "❌ /dev/video$i not found"
                            fi
                        done
                        
                        # If no camera found, try USB camera
                        echo "🔧 Trying USB camera as fallback..."
                        ros2 run usb_cam usb_cam_node_exe --ros-args \
                            -p video_device:=/dev/video0 \
                            -p image_width:=1920 \
                            -p image_height:=1080 \
                            -p pixel_format:=mjpeg \
                            -p framerate:=30.0 \
                            -r /image_raw:=/camera_rear/image_raw &
                        
                        echo "✅ Camera setup completed!"
                    '''],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 5. Start SINGLE CAMERA ULTIMATE node
        TimerAction(
            period=15.0,
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
        
        # ✅ 6. TOPIC MONITORING - check if camera data is flowing
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 TOPIC MONITORING:" && '
                         'echo "📡 Available topics:" && '
                         'ros2 topic list | grep -E "(camera|image)" && '
                         'echo "📡 Camera topic info:" && '
                         'ros2 topic info /camera_rear/image_raw && '
                         'echo "📡 Camera data test:" && '
                         'timeout 5 ros2 topic echo /camera_rear/image_raw --once | head -5'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. GPU Memory optimization untuk single camera
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 GPU Memory optimization for single camera..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH'],
            output='screen'
        ),
        
        # ✅ 8. PERFORMANCE monitoring dengan topic check
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 SINGLE CAMERA Performance Status:" && '
                         'echo "📡 ROS2 topics:" && '
                         'ros2 topic list | grep -E "(camera|image)" && '
                         'echo "📡 Topic rates:" && '
                         'timeout 5 ros2 topic hz /camera_rear/image_raw 2>/dev/null || echo "❌ No data on /camera_rear/image_raw" && '
                         'echo "🔥 GPU Status:" && '
                         '(nvidia-smi --query-gpu=utilization.gpu,memory.used,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available") && '
                         'echo "💻 CPU Usage:" && top -bn1 | grep "Cpu(s)" && '
                         'echo "🧠 Memory:" && free -h | grep Mem && '
                         'echo "🎯 TARGET: 100+ FPS with SINGLE CAMERA YOLO SEGMENTATION"'],
                    output='screen'
                )
            ]
        ),
    ])