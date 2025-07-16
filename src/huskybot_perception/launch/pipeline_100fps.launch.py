#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/pipeline_100fps.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. MAXIMUM GPU Performance Setup
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔥 ACTIVATING MAXIMUM PERFORMANCE MODE..." && '
                 'sudo jetson_clocks && '
                 'sudo nvpmodel -m 0 && '
                 'sudo nvidia-smi -pl 50 && '
                 'sudo nvidia-smi -lgc 1300,2100 && '
                 'sudo nvidia-smi -lmc 1215,8000 && '
                 'sudo cpupower frequency-set --governor performance && '
                 'echo 1 > /proc/sys/vm/drop_caches && '
                 'echo "🚀 MAXIMUM PERFORMANCE MODE ACTIVATED!"'],
            output='screen',
            name='max_performance_setup'
        ),
        
        # ✅ 2. Start Velodyne LiDAR (immediate)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 3. Start MAXIMUM resolution cameras (5 second delay)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 4. Start SIMPLE ULTIMATE WORKING node (15 second delay)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='simple_ultimate_working_node',
                    name='simple_ultimate_working_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        ),
        
        # ✅ 5. AUTO-POPUP LARGE Display (25 second delay)
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '-c', '''
import rclpy, cv2, numpy as np, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class AutoDisplay(Node):
    def __init__(self):
        super().__init__("auto_display")
        self.bridge = CvBridge()
        self.latest_grid = None
        self.sub = self.create_subscription(Image, "/ultimate_grid_display", self.callback, 10)
        print("🎯 AUTO-DISPLAY STARTED - WAITING FOR GRID...")
        
    def callback(self, msg):
        try:
            self.latest_grid = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: 
            pass
        
    def display_loop(self):
        window_name = "HUSKYBOT 360° ULTIMATE SEGMENTATION - 100+ FPS TARGET - Press ESC/Q to exit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # ✅ FULLSCREEN display
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        frame_count = 0
        fps_start = time.time()
        
        while True:
            if self.latest_grid is not None:
                display_img = self.latest_grid.copy()
                
                # ✅ FPS counter
                frame_count += 1
                if frame_count % 30 == 0:
                    fps = 30.0 / (time.time() - fps_start)
                    fps_start = time.time()
                    
                    cv2.putText(display_img, f"Display FPS: {fps:.1f}", 
                               (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)
                
                cv2.imshow(window_name, display_img)
            else:
                blank = np.zeros((1080, 1920, 3), dtype=np.uint8)
                cv2.putText(blank, "WAITING FOR GRID...", (600, 500), 
                           cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 5)
                cv2.imshow(window_name, blank)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
        cv2.destroyAllWindows()

rclpy.init()
node = AutoDisplay()
import threading
display_thread = threading.Thread(target=node.display_loop, daemon=True)
display_thread.start()
rclpy.spin(node)
                    '''],
                    output='screen',
                    name='auto_fullscreen_display'
                )
            ]
        ),
        
        # ✅ 6. RViz2 auto-popup for LiDAR 3D mapping (180 second delay)
        TimerAction(
            period=180.0,  # 3 minutes
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'echo "🚀 Starting RViz2 for LiDAR 3D mapping..." && '
                         'rviz2 -f base_link --ros-args -p use_sim_time:=false > /dev/null 2>&1 &'],
                    output='screen',
                    name='delayed_rviz2'
                )
            ]
        ),
        
        # ✅ 7. Status monitoring (30 second delay)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 100+ FPS Pipeline Status:" && '
                         'echo "📡 Camera topics (should be 6):" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 ULTIMATE grid topic:" && timeout 5 ros2 topic list | grep ultimate_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 5 ros2 topic list | grep scan && '
                         'echo "✅ CHECK TERMINAL FOR DETECTION RESULTS!" && '
                         'echo "📺 FULLSCREEN DISPLAY AUTO-POPUP IN ~10 SECONDS"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 8. Performance monitoring (60 second delay)
        TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "💻 System Performance:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits && '
                         'echo "📊 Memory Usage:" && free -h | grep "Mem:" && '
                         'echo "🎯 TARGET: 100+ FPS with MAXIMUM GPU utilization"'],
                    output='screen'
                )
            ]
        ),
    ])