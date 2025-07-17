#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/simple_ultimate_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. POWER OPTIMIZATION (prevent over-current)
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 OPTIMIZING POWER SETTINGS..." && '
                 'sudo nvpmodel -m 0 && '  # MAX-N mode
                 'sudo jetson_clocks --fan && '  # Enable fan
                 'echo "⚡ POWER OPTIMIZED!"'],
            output='screen',
            name='power_optimization'
        ),
        
        # ✅ 2. Start Velodyne LiDAR (with longer timeout)
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('velodyne'),
                                   'launch', 'velodyne-all-nodes-VLP32C-launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start cameras (longer delay for stability)
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 4. Start SIMPLE ULTIMATE WORKING node
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='simple_ultimate_working_node',
                    name='simple_ultimate_working_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        ),
        
        # ✅ 5. AUTO-POPUP LARGE Display (NOT fullscreen)
        TimerAction(
            period=30.0,
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
        self.sub = self.create_subscription(Image, "/simple_grid_display", self.callback, 10)
        print("🎯 AUTO-DISPLAY STARTED - WAITING FOR GRID...")
        
    def callback(self, msg):
        try:
            self.latest_grid = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: 
            pass
        
    def display_loop(self):
        window_name = "HUSKYBOT 360 ULTIMATE SEGMENTATION - Press ESC/Q to exit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # ✅ LARGE window (NOT fullscreen) - 90% of screen
        cv2.resizeWindow(window_name, 1720, 970)  # Large but not fullscreen
        cv2.moveWindow(window_name, 100, 50)  # Center position
        
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
                               (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                
                cv2.imshow(window_name, display_img)
            else:
                blank = np.zeros((970, 1720, 3), dtype=np.uint8)
                cv2.putText(blank, "WAITING FOR GRID...", (600, 485), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 4)
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
                    name='auto_large_display'
                )
            ]
        ),
        
        # ✅ 6. RViz2 auto-popup for LiDAR 3D mapping (4 minutes delay)
        TimerAction(
            period=240.0,  # 4 minutes
            actions=[
                ExecuteProcess(
                    cmd=['python3', '-c', '''
import subprocess, time, os

# Create proper RViz config
rviz_config = \"""
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /PointCloud21
        - /LaserScan1
        - /Grid1
      Splitter Ratio: 0.5
    Tree Height: 800
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Rainbow
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: PointCloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /velodyne_points
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 25; 0
      Color Transformer: Rainbow
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 5
      Size (m): 0.1
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785
    Saved: ~
\"""

# Save config
with open("/tmp/huskybot_lidar_config.rviz", "w") as f:
    f.write(rviz_config)

print("🚀 Starting RViz2 for LiDAR 3D mapping...")
env = os.environ.copy()
env["DISPLAY"] = ":0"

subprocess.run([
    "rviz2", 
    "-d", "/tmp/huskybot_lidar_config.rviz",
    "--ros-args", "-p", "use_sim_time:=false"
], env=env)
                    '''],
                    output='screen',
                    name='delayed_rviz2_lidar'
                )
            ]
        ),
        
        # ✅ 7. Status monitoring
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 SIMPLE ULTIMATE Pipeline Status:" && '
                         'echo "📡 Camera topics (should be 6):" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔥 Grid topic:" && timeout 5 ros2 topic list | grep simple_grid_display && '
                         'echo "🔴 LiDAR topics:" && timeout 5 ros2 topic list | grep -E "(scan|velodyne)" && '
                         'echo "✅ CHECK TERMINAL FOR DETECTION RESULTS!" && '
                         'echo "📺 LARGE DISPLAY AUTO-POPUP READY!" && '
                         'echo "🗺️ RViz2 3D MAPPING WILL START IN 3 MINUTES"'],
                    output='screen'
                )
            ]
        ),
        
        # ✅ 8. Performance monitoring
        TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "💻 System Performance:" && '
                         'echo "🔥 GPU Status:" && nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits && '
                         'echo "📊 Memory Usage:" && free -h | grep "Mem:" && '
                         'echo "🎯 TARGET: Optimized for stability and performance"'],
                    output='screen'
                )
            ]
        ),
    ])