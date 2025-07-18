#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_multicam_parallel/huskybot_multicam_parallel/multicam_parallel_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

class MultiCamParallelNode(Node):
    def __init__(self):
        super().__init__('multicam_parallel_node')
        
        self.bridge = CvBridge()
        
        # ✅ Camera configuration - REAL MAPPING
        self.camera_configs = [
            {
                'name': 'camera_front',
                'topic': '/camera_front_processed',
                'real_name': 'REAR CAMERA',
                'position': (0, 0)
            },
            {
                'name': 'camera_front_left', 
                'topic': '/camera_front_left_processed',
                'real_name': 'LEFT REAR CAMERA',
                'position': (0, 1)
            },
            {
                'name': 'camera_left',
                'topic': '/camera_left_processed', 
                'real_name': 'LEFT FRONT CAMERA',
                'position': (0, 2)
            },
            {
                'name': 'camera_rear',
                'topic': '/camera_rear_processed',
                'real_name': 'FRONT CAMERA',
                'position': (1, 0)
            },
            {
                'name': 'camera_rear_right',
                'topic': '/camera_rear_right_processed',
                'real_name': 'RIGHT FRONT CAMERA',
                'position': (1, 1)
            },
            {
                'name': 'camera_right',
                'topic': '/camera_right_processed',
                'real_name': 'RIGHT REAR CAMERA',
                'position': (1, 2)
            }
        ]
        
        # ✅ Data storage
        self.latest_images = {}
        self.image_locks = {}
        self.subscribers = {}
        
        # ✅ FIXED: Constant canvas size to prevent resizing
        self.CELL_WIDTH = 800  # FIXED SIZE
        self.CELL_HEIGHT = 600  # FIXED SIZE
        self.HEADER_HEIGHT = 80  # FIXED SIZE
        self.TITLE_HEIGHT = 100  # FIXED SIZE
        self.GRID_ROWS = 2
        self.GRID_COLS = 3
        
        # Calculate total canvas size once
        self.CANVAS_WIDTH = self.GRID_COLS * self.CELL_WIDTH
        self.CANVAS_HEIGHT = self.GRID_ROWS * (self.CELL_HEIGHT + self.HEADER_HEIGHT) + self.TITLE_HEIGHT
        
        # ✅ Setup subscribers
        self.setup_subscribers()
        
        # ✅ Setup display
        self.setup_display()
        
        self.get_logger().info("🚀 MULTICAM PARALLEL DISPLAY NODE STARTED!")

    def setup_subscribers(self):
        """Setup subscribers for all processed camera topics"""
        for config in self.camera_configs:
            name = config['name']
            topic = config['topic']
            
            # Initialize storage
            self.latest_images[name] = None
            self.image_locks[name] = threading.Lock()
            
            # Create subscriber
            self.subscribers[name] = self.create_subscription(
                Image, topic, 
                lambda msg, n=name: self.camera_callback(msg, n), 
                10
            )
            
            self.get_logger().info(f"📡 Subscribed to {topic}")

    def camera_callback(self, msg, camera_name):
        """Camera callback for processed images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_locks[camera_name]:
                self.latest_images[camera_name] = cv_image.copy()
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error for {camera_name}: {e}")

    def setup_display(self):
        """Setup display loop"""
        self.display_active = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        self.get_logger().info("✅ Display thread started!")

    def display_loop(self):
        """Main display loop - Grid 2x3"""
        while self.display_active:
            try:
                self.create_grid_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_grid_display(self):
        """Create 2x3 grid display - FIXED ALL ISSUES"""
        try:
            # ✅ FIXED: Create canvas with FIXED SIZE
            canvas = np.zeros((self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3), dtype=np.uint8)
            
            # ✅ FIXED: Add title - CENTERED
            title_canvas = canvas[:self.TITLE_HEIGHT, :, :]
            title_text = "HUSKYBOT MULTICAM SEGMENTATION"
            
            # Calculate center position for title
            (text_width, text_height), _ = cv2.getTextSize(title_text, cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4)
            title_x = (self.CANVAS_WIDTH - text_width) // 2  # PERFECTLY CENTERED
            title_y = (self.TITLE_HEIGHT + text_height) // 2
            
            cv2.putText(title_canvas, title_text, 
                       (title_x, title_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 255), 4)
            
            # ✅ Fill grid with camera images and headers
            for config in self.camera_configs:
                name = config['name']
                real_name = config['real_name']
                row, col = config['position']
                
                # Get latest image
                with self.image_locks[name]:
                    if self.latest_images[name] is not None:
                        img = self.latest_images[name].copy()
                    else:
                        img = None
                
                # ✅ FIXED: Calculate positions with FIXED sizes
                x_start = col * self.CELL_WIDTH
                y_start = self.TITLE_HEIGHT + row * (self.CELL_HEIGHT + self.HEADER_HEIGHT)
                x_end = x_start + self.CELL_WIDTH
                y_end = y_start + self.HEADER_HEIGHT + self.CELL_HEIGHT
                
                # ✅ FIXED: Draw camera label header
                header_y_start = y_start
                header_y_end = y_start + self.HEADER_HEIGHT
                
                # Header background - DARK GRAY
                cv2.rectangle(canvas, (x_start, header_y_start), (x_end, header_y_end), (50, 50, 50), -1)
                
                # ✅ FIXED: Header text - CENTERED and BRIGHT
                (header_text_width, header_text_height), _ = cv2.getTextSize(real_name, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
                header_text_x = x_start + (self.CELL_WIDTH - header_text_width) // 2  # CENTERED
                header_text_y = header_y_start + (self.HEADER_HEIGHT + header_text_height) // 2
                
                cv2.putText(canvas, real_name, 
                           (header_text_x, header_text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
                
                # Image area
                img_y_start = y_start + self.HEADER_HEIGHT
                img_y_end = y_end
                
                if img is not None:
                    # ✅ FIXED: Resize image to EXACT cell size
                    img_resized = cv2.resize(img, (self.CELL_WIDTH, self.CELL_HEIGHT))
                    canvas[img_y_start:img_y_end, x_start:x_end] = img_resized
                else:
                    # Create waiting image with FIXED size
                    waiting_img = np.zeros((self.CELL_HEIGHT, self.CELL_WIDTH, 3), dtype=np.uint8)
                    waiting_text = "WAITING FOR SIGNAL..."
                    
                    # Center waiting text
                    (wait_text_width, wait_text_height), _ = cv2.getTextSize(waiting_text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
                    wait_x = (self.CELL_WIDTH - wait_text_width) // 2
                    wait_y = (self.CELL_HEIGHT + wait_text_height) // 2
                    
                    cv2.putText(waiting_img, waiting_text, 
                               (wait_x, wait_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                    canvas[img_y_start:img_y_end, x_start:x_end] = waiting_img
                
                # ✅ Draw grid lines - THICKER
                cv2.rectangle(canvas, (x_start, y_start), (x_end-1, y_end-1), (150, 150, 150), 4)
            
            # ✅ FIXED: Display with FIXED window size
            cv2.namedWindow('HUSKYBOT MULTICAM PARALLEL', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('HUSKYBOT MULTICAM PARALLEL', self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
            cv2.imshow('HUSKYBOT MULTICAM PARALLEL', canvas)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"❌ Grid display creation error: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.display_active = False
        time.sleep(0.5)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = MultiCamParallelNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down MULTICAM PARALLEL...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()