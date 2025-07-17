#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/ultimate_auto_popup_manager.py

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

class UltimateAutoPopupManager(Node):
    def __init__(self):
        super().__init__('ultimate_auto_popup_manager')
        
        self.bridge = CvBridge()
        self.latest_grid = None
        self.grid_lock = threading.Lock()
        
        # ✅ Subscribe to ULTIMATE grid
        self.grid_sub = self.create_subscription(
            Image, '/ultimate_grid_display', self.grid_callback, 10
        )
        
        self.get_logger().info("🚀 ULTIMATE Auto-Popup Manager - LARGE DISPLAY + RViz2!")
        
        # ✅ Start auto-popup threads
        self.start_ultimate_popup_threads()
    
    def grid_callback(self, msg):
        """Receive ULTIMATE grid display"""
        try:
            with self.grid_lock:
                self.latest_grid = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Grid callback error: {e}")
    
    def start_ultimate_popup_threads(self):
        """Start ULTIMATE auto-popup threads"""
        # ✅ LARGE grid display thread
        grid_thread = threading.Thread(target=self.ultimate_grid_display, daemon=True)
        grid_thread.start()
        
        # ✅ RViz2 auto-popup thread
        rviz_thread = threading.Thread(target=self.ultimate_rviz_popup, daemon=True)
        rviz_thread.start()
        
        self.get_logger().info("🎯 ULTIMATE Auto-popup threads started!")
    
    def ultimate_grid_display(self):
        """ULTIMATE grid display - LARGE size untuk visibility"""
        wait_time = 5.0
        max_wait = 30.0
        
        while wait_time <= max_wait:
            time.sleep(wait_time)
            
            with self.grid_lock:
                if self.latest_grid is not None:
                    break
            
            self.get_logger().info(f"⏳ Waiting for ULTIMATE grid... ({wait_time}s)")
            wait_time += 5.0
        
        if self.latest_grid is None:
            self.get_logger().error("❌ No ULTIMATE grid received after 30s")
            return
        
        self.get_logger().info("🔥 Starting ULTIMATE LARGE Display!")
        
        # ✅ Create LARGE window (not fullscreen for better usability)
        window_name = "HUSKYBOT 360° ULTIMATE SEGMENTATION - 100+ FPS - Press ESC/Q to exit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # ✅ Set LARGE window size (almost fullscreen)
        screen_width = 1920
        screen_height = 1080
        window_width = int(screen_width * 0.95)  # 95% of screen width
        window_height = int(screen_height * 0.90)  # 90% of screen height
        
        cv2.resizeWindow(window_name, window_width, window_height)
        cv2.moveWindow(window_name, 25, 25)  # Position near top-left
        
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
                
                # ✅ Resize to LARGE display size
                display_img = cv2.resize(display_img, (window_width, window_height), 
                                       interpolation=cv2.INTER_CUBIC)
                
                # ✅ Add ULTIMATE FPS counter
                frame_count += 1
                if frame_count % 30 == 0:
                    fps = 30.0 / (time.time() - fps_start)
                    fps_start = time.time()
                    
                    # ✅ LARGE FPS display
                    cv2.putText(display_img, f"Display FPS: {fps:.1f}", 
                               (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)
                    
                    cv2.putText(display_img, f"TARGET: 100+ FPS", 
                               (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 3)
                
                # ✅ Show LARGE display
                cv2.imshow(window_name, display_img)
                
                # ✅ Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):  # ESC or Q
                    break
                    
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.get_logger().info("🛑 ULTIMATE display closed")
    
    def ultimate_rviz_popup(self):
        """ULTIMATE RViz2 auto-popup dengan LiDAR 3D mapping"""
        time.sleep(15.0)  # Wait untuk LiDAR ready
        
        try:
            self.get_logger().info("🚀 Starting ULTIMATE RViz2 auto-popup...")
            
            # ✅ Create ULTIMATE RViz config
            rviz_config = self.create_ultimate_rviz_config()
            
            # ✅ Launch RViz2 dengan ULTIMATE config
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            
            cmd = [
                'rviz2',
                '-d', rviz_config,
                '--ros-args', 
                '-p', 'use_sim_time:=false',
                '-p', 'fixed_frame:=base_link'
            ]
            
            process = subprocess.Popen(
                cmd, env=env, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE
            )
            
            self.get_logger().info("✅ ULTIMATE RViz2 launched successfully!")
            
            # ✅ Monitor process
            while True:
                if process.poll() is not None:
                    self.get_logger().info("🛑 RViz2 closed")
                    break
                time.sleep(5.0)
                
        except Exception as e:
            self.get_logger().error(f"❌ RViz2 launch error: {e}")
    
    def create_ultimate_rviz_config(self):
        """Create ULTIMATE RViz config for 3D LiDAR mapping"""
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
Window Geometry:
  Displays:
    collapsed: false
  Height: 1016
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1920
  X: 0
  Y: 27
"""
        
        # ✅ Save ULTIMATE config
        config_path = "/tmp/huskybot_ultimate_rviz_config.rviz"
        with open(config_path, 'w') as f:
            f.write(config_content)
        
        return config_path

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltimateAutoPopupManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down ULTIMATE popup manager...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()