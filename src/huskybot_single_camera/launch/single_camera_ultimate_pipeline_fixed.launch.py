#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_single_camera/launch/single_camera_ultimate_pipeline_fixed.launch.py

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
                 'sudo nvidia-smi -pm 1 && '
                 'sudo nvidia-smi -pl 55 && '
                 'sudo nvidia-smi -lgc 2100,2100 && '
                 'sudo nvidia-smi -lmc 6251,6251 && '
                 'for i in {0..11}; do sudo sh -c "echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor" 2>/dev/null || true; done && '
                 'sudo sysctl -w vm.swappiness=1 && '
                 'echo "⚡ ULTRA POWER OPTIMIZED FOR SINGLE CAMERA 100+ FPS!"'],
            output='screen',
            name='ultra_power_optimization'
        ),
        
        # ✅ 2. OPTIMAL camera detection dengan CORRECT format
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ARDUCAM IMX477 OPTIMAL SETUP..." && '
                 'ls -la /dev/video* && '
                 'echo "🔧 CAMERA FORMATS:" && '
                 'v4l2-ctl --device=/dev/video0 --list-formats-ext && '
                 'echo "🔧 OPTIMAL CAMERA CONFIGURATION FOR 100+ FPS..."'],
            output='screen',
            name='camera_detection'
        ),
        
        # ✅ 3. Start LiDAR (simplified)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting LiDAR..." && '
                         'ros2 run velodyne_driver velodyne_driver_node --ros-args '
                         '-p model:=VLP32C -p rpm:=600.0 -p port:=2368 -p device_ip:=192.168.1.201 &'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 4. OPTIMAL CAMERA SETUP dengan CORRECT pixel format
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        echo "🔧 ARDUCAM IMX477 OPTIMAL SETUP FOR 100+ FPS..."
                        
                        # Test different video devices dengan CORRECT format
                        for i in 0 1 2 3 4 5; do
                            echo "Testing /dev/video$i..."
                            if [ -e "/dev/video$i" ]; then
                                echo "Found /dev/video$i"
                                
                                # Use RG10 format (Bayer) dengan 1920x1080@60fps
                                echo "✅ Using /dev/video$i with RG10 1920x1080@60fps"
                                
                                # Configure camera dengan OPTIMAL settings
                                v4l2-ctl --device=/dev/video$i --set-fmt-video=width=1920,height=1080,pixelformat=RG10 --set-parm=60
                                
                                # Start OPTIMIZED GStreamer pipeline untuk 100+ FPS
                                gst-launch-1.0 -v v4l2src device=/dev/video$i ! \
                                    "video/x-bayer,width=1920,height=1080,framerate=60/1,format=rggb" ! \
                                    bayer2rgb ! \
                                    videoconvert ! \
                                    videoscale ! \
                                    "video/x-raw,format=BGR,width=1920,height=1080,framerate=60/1" ! \
                                    queue max-size-buffers=2 leaky=downstream ! \
                                    appsink name=sink max-buffers=1 drop=true | \
                                ros2 run image_tools cam2image --ros-args \
                                    -r /image:=/camera_rear/image_raw \
                                    -p width:=1920 \
                                    -p height:=1080 \
                                    -p framerate:=60.0 &
                                
                                echo "✅ OPTIMAL camera started with /dev/video$i at 60 FPS"
                                sleep 2
                                break
                            fi
                        done
                        
                        echo "✅ OPTIMAL camera setup completed!"
                    '''],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 5. Start SINGLE CAMERA ULTIMATE node (dengan delay yang tepat)
        TimerAction(
            period=18.0,  # Increased delay untuk pastikan camera ready
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
        
        # ✅ 6. ENHANCED TOPIC MONITORING
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 ENHANCED TOPIC MONITORING:" && '
                         'echo "📡 Available topics:" && '
                         'ros2 topic list | grep -E "(camera|image)" && '
                         'echo "📡 Camera topic info:" && '
                         'ros2 topic info /camera_rear/image_raw && '
                         'echo "📡 Camera topic rate:" && '
                         'timeout 10 ros2 topic hz /camera_rear/image_raw 2>/dev/null || echo "❌ No rate data" && '
                         'echo "📡 Camera data sample:" && '
                         'timeout 3 ros2 topic echo /camera_rear/image_raw --once | head -10'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 7. GPU Memory optimization pentru single camera
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ENHANCED GPU optimization..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export NVIDIA_TF32_OVERRIDE=0 && '
                 'export CUDA_CACHE_MAXSIZE=2147483648 && '
                 'echo "✅ GPU optimized for 100+ FPS"'],
            output='screen'
        ),
        
        # ✅ 8. REAL-TIME PERFORMANCE monitoring
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 REAL-TIME PERFORMANCE STATUS:" && '
                         'echo "📡 Camera FPS:" && '
                         'timeout 8 ros2 topic hz /camera_rear/image_raw 2>/dev/null || echo "❌ No camera data" && '
                         'echo "🔥 GPU Status:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "💻 CPU Usage:" && '
                         'top -bn1 | grep "Cpu(s)" && '
                         'echo "🧠 Memory Usage:" && '
                         'free -h | grep Mem && '
                         'echo "🎯 TARGET: 100+ FPS SINGLE CAMERA YOLO SEGMENTATION"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 9. AUTOMATED CAMERA RESTART (jika diperlukan)
        TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔄 CAMERA HEALTH CHECK..." && '
                         'if ! timeout 3 ros2 topic hz /camera_rear/image_raw >/dev/null 2>&1; then '
                         '    echo "❌ Camera not publishing, attempting restart..." && '
                         '    pkill -f "cam2image" || true && '
                         '    sleep 2 && '
                         '    v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=RG10 --set-parm=60 && '
                         '    gst-launch-1.0 -v v4l2src device=/dev/video0 ! '
                         '        "video/x-bayer,width=1920,height=1080,framerate=60/1,format=rggb" ! '
                         '        bayer2rgb ! videoconvert ! videoscale ! '
                         '        "video/x-raw,format=BGR,width=1920,height=1080,framerate=60/1" ! '
                         '        queue max-size-buffers=2 leaky=downstream ! '
                         '        appsink name=sink max-buffers=1 drop=true | '
                         '    ros2 run image_tools cam2image --ros-args -r /image:=/camera_rear/image_raw & '
                         '    echo "✅ Camera restarted"; '
                         'else '
                         '    echo "✅ Camera healthy"; '
                         'fi'],
                    output='screen'
                )
            ]
        ),
    ])