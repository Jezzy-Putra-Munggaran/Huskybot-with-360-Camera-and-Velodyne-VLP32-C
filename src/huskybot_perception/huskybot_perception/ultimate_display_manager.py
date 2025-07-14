#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/ultimate_display_manager.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import threading
import time
import os

class UltimateDisplayManager(Node):
    def __init__(self):
        super().__init__('ultimate_display_manager')
        
        self.bridge = CvBridge()
        self.latest_grid = None
        self.grid_lock = threading.Lock()
        
        # ✅ Create subscription to grid display
        self.grid_sub = self.create_subscription(
            Image, '/simple_grid_display', self.grid_callback, 10
        )
        
        self.get_logger().info("🚀 Ultimate Display Manager - AUTO-POPUP Grid & RViz2!")
        
        # ✅ Start auto-popup threads
        self.start_auto_popup_threads()
    
    def grid_callback(self, msg):
        """Receive grid display"""
        try:
            with self.grid_lock:
                self.latest_grid = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info("✅ Grid received - ready for display", once=True)
        except Exception as e:
            self.get_logger().error(f"❌ Grid callback error: {e}")
    
    def start_auto_popup_threads(self):
        """Start auto-popup threads"""
        # ✅ Grid display thread
        grid_thread = threading.Thread(target=self.auto_popup_grid, daemon=True)
        grid_thread.start()
        
        # ✅ RViz2 auto-popup thread
        rviz_thread = threading.Thread(target=self.auto_popup_rviz, daemon=True)
        rviz_thread.start()
        
        self.get_logger().info("🎯 Auto-popup threads started!")
    
    def auto_popup_grid(self):
        """Auto-popup grid display dengan retry mechanism"""
        wait_time = 5.0
        max_wait = 30.0
        
        while wait_time <= max_wait:
            time.sleep(wait_time)
            
            with self.grid_lock:
                if self.latest_grid is not None:
                    break
            
            self.get_logger().info(f"⏳ Waiting for grid... ({wait_time}s)")
            wait_time += 5.0
        
        if self.latest_grid is None:
            self.get_logger().error("❌ No grid received after 30s")
            return
        
        self.get_logger().info("🎯 Starting ULTIMATE Grid Display!")
        
        # ✅ Create fullscreen window
        window_name = "HUSKYBOT 360° ULTIMATE SEGMENTATION - Press ESC/Q to exit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        # ✅ Display loop
        frame_count = 0
        fps_start = time.time()
        
        while True:
            try:
                with self.grid_lock:
                    if self.latest_grid is not None:
                        display_img = self.latest_grid.copy()
                    else:
                        continue
                
                # ✅ Add FPS counter
                frame_count += 1
                if frame_count % 30 == 0:
                    fps = 30.0 / (time.time() - fps_start)
                    fps_start = time.time()
                    
                    cv2.putText(display_img, f"Display FPS: {fps:.1f}", 
                               (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                
                # ✅ Show fullscreen
                cv2.imshow(window_name, display_img)
                
                # ✅ Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):  # ESC or Q
                    break
                    
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.get_logger().info("🛑 Grid display closed")
    
    def auto_popup_rviz(self):
        """Auto-popup RViz2 dengan config yang proper"""
        time.sleep(10.0)  # Wait untuk LiDAR ready
        
        try:
            self.get_logger().info("🚀 Starting RViz2 auto-popup...")
            
            # ✅ Create custom RViz config
            rviz_config = self.create_rviz_config()
            
            # ✅ Launch RViz2 dengan custom config
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            
            cmd = [
                'rviz2',
                '-d', rviz_config,
                '--ros-args', '-p', 'use_sim_time:=false'
            ]
            
            process = subprocess.Popen(
                cmd, env=env, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            
            self.get_logger().info("✅ RViz2 launched successfully!")
            
            # ✅ Monitor process
            while True:
                if process.poll() is not None:
                    self.get_logger().info("🛑 RViz2 closed")
                    break
                time.sleep(5.0)
                
        except Exception as e:
            self.get_logger().error(f"❌ RViz2 launch error: {e}")
    
    def create_rviz_config(self):
        """Create custom RViz config for LiDAR"""
        config_content = """
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
      Splitter Ratio: 0.5
    Tree Height: 787
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
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
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
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
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: PointCloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.1
      Style: Flat Squares
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
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.1
      Style: Flat Squares
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
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.4
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1016
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039efc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039e000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730100000041000000e60000000000000000fb0000000c004b006900740065007400630068006500620165000001a100000228000000a900ffffff000000010000010f0000039efc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d0000039e000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007800000003efc0100000002fb0000000800540069006d00650100000000000007800000000000000000fb0000000800540069006d00650100000000000004500000000000000000000004f40000039e00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1920
  X: 0
  Y: 27
"""
        
        # ✅ Save config file
        config_path = "/tmp/huskybot_rviz_config.rviz"
        with open(config_path, 'w') as f:
            f.write(config_content)
        
        return config_path

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltimateDisplayManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down display manager...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()