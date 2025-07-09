#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class LargeGridViewer(Node):
    def __init__(self):
        super().__init__('large_grid_viewer')
        
        self.bridge = CvBridge()
        self.latest_grid = None
        
        # Subscribe to grid topic
        self.grid_sub = self.create_subscription(
            Image, '/deepstream_grid', self.grid_callback, 1)
        
        # Start display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("🖥️ Large Grid Viewer started - Press 'q' to quit")

    def grid_callback(self, msg):
        """Receive grid image"""
        try:
            self.latest_grid = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Grid conversion error: {e}")

    def display_loop(self):
        """Display loop for large grid"""
        # Create large window
        cv2.namedWindow("HUSKYBOT 360° ULTRA-FAST - Press 'q' to quit", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HUSKYBOT 360° ULTRA-FAST - Press 'q' to quit", 1920, 1080)
        
        while True:
            if self.latest_grid is not None:
                cv2.imshow("HUSKYBOT 360° ULTRA-FAST - Press 'q' to quit", self.latest_grid)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            time.sleep(0.033)  # ~30 FPS display
        
        cv2.destroyAllWindows()
        self.get_logger().info("🛑 Grid viewer closed")

def main(args=None):
    rclpy.init(args=args)
    viewer = LargeGridViewer()
    
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