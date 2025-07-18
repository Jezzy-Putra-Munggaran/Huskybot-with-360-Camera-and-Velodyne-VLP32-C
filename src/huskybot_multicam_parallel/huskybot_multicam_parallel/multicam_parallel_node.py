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
        
        # ✅ Camera configuration - REAL MAPPING dengan FIXED labels
        self.camera_configs = [
            {
                'name': 'camera_front',
                'topic': '/camera_front_processed',
                'real_name': 'REAR CAMERA',  # FIXED: Real life mapping
                'position': (0, 0)  # Top-left in 2x3 grid
            },
            {
                'name': 'camera_front_left', 
                'topic': '/camera_front_left_processed',
                'real_name': 'LEFT REAR CAMERA',  # FIXED: Real life mapping
                'position': (0, 1)  # Top-middle in 2x3 grid
            },
            {
                'name': 'camera_left',
                'topic': '/camera_left_processed', 
                'real_name': 'LEFT FRONT CAMERA',  # FIXED: Real life mapping
                'position': (0, 2)  # Top-right in 2x3 grid
            },
            {
                'name': 'camera_rear',
                'topic': '/camera_rear_processed',
                'real_name': 'FRONT CAMERA',  # FIXED: Real life mapping
                'position': (1, 0)  # Bottom-left in 2x3 grid
            },
            {
                'name': 'camera_rear_right',
                'topic': '/camera_rear_right_processed',
                'real_name': 'RIGHT FRONT CAMERA',  # FIXED: Real life mapping
                'position': (1, 1)  # Bottom-middle in 2x3 grid
            },
            {
                'name': 'camera_right',
                'topic': '/camera_right_processed',
                'real_name': 'RIGHT REAR CAMERA',  # FIXED: Real life mapping
                'position': (1, 2)  # Bottom-right in 2x3 grid
            }
        ]
        
        # ✅ Data storage
        self.latest_images = {}
        self.image_locks = {}
        self.subscribers = {}
        
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
            # ✅ Grid configuration - LARGER SIZE for better visibility
            grid_rows = 2
            grid_cols = 3
            cell_width = 720  # INCREASED from 640
            cell_height = 540  # INCREASED from 480
            header_height = 60  # Height for camera labels
            
            # ✅ Create grid canvas with headers
            canvas_width = grid_cols * cell_width
            canvas_height = grid_rows * (cell_height + header_height)
            canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
            
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
                
                # Calculate cell position with header
                x_start = col * cell_width
                y_start = row * (cell_height + header_height)
                x_end = x_start + cell_width
                y_end = y_start + cell_height + header_height
                
                # ✅ FIXED: Draw camera label header
                header_y_start = y_start
                header_y_end = y_start + header_height
                
                # Header background
                cv2.rectangle(canvas, (x_start, header_y_start), (x_end, header_y_end), (40, 40, 40), -1)
                
                # Header text - BRIGHT and LARGE
                text_x = x_start + 20
                text_y = header_y_start + 40
                cv2.putText(canvas, real_name, 
                           (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                
                # Image area
                img_y_start = y_start + header_height
                img_y_end = y_end
                
                if img is not None:
                    # Resize image to fit cell
                    img_resized = cv2.resize(img, (cell_width, cell_height))
                    canvas[img_y_start:img_y_end, x_start:x_end] = img_resized
                else:
                    # Create waiting image
                    waiting_img = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
                    cv2.putText(waiting_img, "WAITING FOR SIGNAL...", 
                               (cell_width//6, cell_height//2), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                    canvas[img_y_start:img_y_end, x_start:x_end] = waiting_img
                
                # Draw grid lines
                cv2.rectangle(canvas, (x_start, y_start), (x_end-1, y_end-1), (100, 100, 100), 3)
            
            # ✅ FIXED: Add title - REMOVED "100+ FPS"
            title_height = 80
            title_canvas = np.zeros((title_height, canvas_width, 3), dtype=np.uint8)
            cv2.putText(title_canvas, "HUSKYBOT MULTICAM SEGMENTATION", 
                       (canvas_width//6, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 255), 4)
            
            # ✅ Combine title and grid
            final_canvas = np.vstack([title_canvas, canvas])
            
            # ✅ Display full screen
            cv2.namedWindow('HUSKYBOT MULTICAM PARALLEL', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('HUSKYBOT MULTICAM PARALLEL', 1920, 1080)
            cv2.imshow('HUSKYBOT MULTICAM PARALLEL', final_canvas)
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