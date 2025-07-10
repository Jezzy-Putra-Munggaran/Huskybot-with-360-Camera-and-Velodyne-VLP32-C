#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
import numpy as np
import queue

class UltraFastAutoGridViewer(Node):
    def __init__(self):
        super().__init__('ultra_fast_auto_grid_viewer')
        
        self.bridge = CvBridge()
        self.running = True
        
        # ✅ MAXIMUM Performance: Pre-allocated arrays
        self.latest_images = [None] * 6
        self.latest_timestamps = [0.0] * 6
        self.image_locks = [threading.Lock() for _ in range(6)]
        
        # ✅ CORRECT camera mapping with ENGLISH labels
        self.camera_topics = [
            '/camera_front/image_raw',      # REAR CAMERA (0)
            '/camera_right/image_raw',      # REAR RIGHT CAMERA (1)
            '/camera_rear_right/image_raw', # FRONT RIGHT CAMERA (2)
            '/camera_rear/image_raw',       # FRONT CAMERA (3)
            '/camera_left/image_raw',       # FRONT LEFT CAMERA (4)
            '/camera_front_left/image_raw'  # REAR LEFT CAMERA (5)
        ]
        
        self.camera_names = [
            'REAR CAMERA', 'REAR RIGHT CAMERA', 'FRONT RIGHT CAMERA', 
            'FRONT CAMERA', 'FRONT LEFT CAMERA', 'REAR LEFT CAMERA'
        ]
        
        # ✅ High-performance subscriptions
        self.subs = []
        for i, topic in enumerate(self.camera_topics):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.ultra_fast_callback(msg, idx), 
                1  # Minimal queue for speed
            )
            self.subs.append(sub)
        
        # ✅ MAXIMUM performance display thread
        self.display_thread = threading.Thread(target=self.ultra_fast_display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("🚀 ULTRA-FAST Auto Grid Viewer started - ALL 6 cameras")

    def ultra_fast_callback(self, msg, camera_idx):
        """MAXIMUM speed image callback"""
        try:
            # ✅ Ultra-fast conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ Thread-safe update with minimal lock time
            with self.image_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
                self.latest_timestamps[camera_idx] = time.time()
                
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def ultra_fast_display_loop(self):
        """MAXIMUM performance display loop"""
        # ✅ Create OPTIMAL window size
        window_name = "HUSKYBOT 360° ULTRA-FAST VISION - 6 CAMERAS"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1920, 1080)  # Optimal size
        cv2.moveWindow(window_name, 0, 0)
        
        # ✅ Pre-allocate target size for speed
        target_width, target_height = 640, 360  # Optimal per camera
        
        while self.running:
            try:
                grid_images = []
                
                # ✅ Process ALL 6 cameras with MAXIMUM speed
                for i in range(6):
                    if self.latest_images[i] is not None:
                        with self.image_locks[i]:
                            img = self.latest_images[i].copy()
                        
                        # ✅ Ultra-fast resize
                        img_resized = cv2.resize(img, (target_width, target_height), 
                                               interpolation=cv2.INTER_AREA)
                        
                        # ✅ Enhanced camera info overlay
                        cv2.rectangle(img_resized, (0, 0), (target_width, 40), (0, 0, 0), -1)
                        cv2.putText(img_resized, f"CAM {i+1}: {self.camera_names[i]}", 
                                  (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        # ✅ Age indicator
                        age = time.time() - self.latest_timestamps[i]
                        age_color = (0, 255, 0) if age < 0.1 else (0, 255, 255) if age < 0.5 else (0, 0, 255)
                        cv2.putText(img_resized, f"Age: {age:.2f}s", 
                                  (target_width-120, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, age_color, 1)
                        
                        grid_images.append(img_resized)
                    else:
                        # ✅ Waiting placeholder
                        black_img = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                        cv2.putText(black_img, f"CAM {i+1}: {self.camera_names[i]}", 
                                  (target_width//4, target_height//2-20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        cv2.putText(black_img, "WAITING...", 
                                  (target_width//3, target_height//2+20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        grid_images.append(black_img)
                
                # ✅ Create OPTIMAL 2x3 grid
                if len(grid_images) == 6:
                    top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                    bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                    grid = np.vstack([top_row, bottom_row])
                    
                    # ✅ ENHANCED performance info
                    info_height = 60
                    cv2.rectangle(grid, (0, grid.shape[0]-info_height), (grid.shape[1], grid.shape[0]), (0, 0, 0), -1)
                    
                    current_time = time.time()
                    active_cameras = sum(1 for i in range(6) if self.latest_images[i] is not None)
                    
                    info_text = f"HUSKYBOT 360° VISION | Active: {active_cameras}/6 cameras | FPS: REALTIME | Press 'q' to quit"
                    cv2.putText(grid, info_text, (20, grid.shape[0]-30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    
                    cv2.putText(grid, f"Resolution: {grid.shape[1]}x{grid.shape[0]} | Target: 100+ FPS", 
                               (20, grid.shape[0]-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    
                    # ✅ Display with MAXIMUM performance
                    cv2.imshow(window_name, grid)
                
                # ✅ OPTIMAL frame rate
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.running = False
                    break
                
                time.sleep(0.0167)  # ~60 FPS for smooth display
                
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.get_logger().info("🛑 ULTRA-FAST Grid viewer closed")

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    viewer = UltraFastAutoGridViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()