#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/ultimate_working_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. GPU Performance boost
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔥 BOOSTING GPU PERFORMANCE..." && '
                 'sudo jetson_clocks 2>/dev/null || true && '
                 'sudo nvpmodel -m 0 2>/dev/null || true && '
                 'echo "🚀 GPU PERFORMANCE BOOSTED!"'],
            output='screen',
            name='gpu_boost'
        ),
        
        # ✅ 2. Start Velodyne LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 3. Start cameras (5 second delay)
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
        
        # ✅ 5. AUTO-POPUP Display Manager (25 second delay)
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
        except: pass
        
    def display_loop(self):
        window_name = "HUSKYBOT 360° ULTIMATE SEGMENTATION - Press ESC/Q to exit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1800, 1200)  # Large display
        
        while True:
            if self.latest_grid is not None:
                cv2.imshow(window_name, self.latest_grid)
            else:
                blank = np.zeros((600, 800, 3), dtype=np.uint8)
                cv2.putText(blank, "WAITING FOR GRID...", (200, 300), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
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
                    name='auto_display'
                )
            ]
        ),
        
        # ✅ 6. RViz2 auto-popup (3 minutes delay)
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
                         'echo "🎯 ULTIMATE WORKING Pipeline Status:" && '
                         'echo "📡 Camera topics (should be 6):" && timeout 3 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 Grid topic:" && timeout 3 ros2 topic list | grep ultimate_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 3 ros2 topic list | grep scan && '
                         'echo "✅ CHECK TERMINAL FOR DETECTION RESULTS!" && '
                         'echo "📺 LARGE DISPLAY SHOULD AUTO-POPUP IN ~10 SECONDS"'],
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
                         'echo "🔥 GPU Status:" && nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null || echo "GPU info not available" && '
                         'echo "📊 Memory Usage:" && free -h | grep "Mem:" && '
                         'echo "🎯 TARGET: 100+ FPS with full GPU utilization"'],
                    output='screen'
                )
            ]
        ),
    ])