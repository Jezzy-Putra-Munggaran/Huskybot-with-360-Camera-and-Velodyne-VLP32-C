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
        
        # ✅ Camera configuration - REAL MAPPING WITH FIXED DEGREE SYMBOLS
        self.camera_configs = [
            {
                'name': 'camera_front',
                'topic': '/camera_front_processed',
                'real_name': 'REAR CAMERA (150-210 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (0, 0)
            },
            {
                'name': 'camera_front_left', 
                'topic': '/camera_front_left_processed',
                'real_name': 'LEFT REAR CAMERA (210-270 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (0, 1)
            },
            {
                'name': 'camera_left',
                'topic': '/camera_left_processed', 
                'real_name': 'LEFT FRONT CAMERA (270-330 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (0, 2)
            },
            {
                'name': 'camera_rear',
                'topic': '/camera_rear_processed',
                'real_name': 'FRONT CAMERA (330-30 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (1, 0)
            },
            {
                'name': 'camera_rear_right',
                'topic': '/camera_rear_right_processed',
                'real_name': 'RIGHT FRONT CAMERA (30-90 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (1, 1)
            },
            {
                'name': 'camera_right',
                'topic': '/camera_right_processed',
                'real_name': 'RIGHT REAR CAMERA (90-150 deg)',  # FIXED: Using "deg" instead of "°"
                'position': (1, 2)
            }
        ]
        
        # ✅ Data storage
        self.latest_images = {}
        self.image_locks = {}
        self.subscribers = {}
        
        # ✅ MEGA FIXED: ABSOLUTE CONSTANT canvas size with MAXIMUM FOV
        self.CELL_WIDTH = 1000   # INCREASED for MAXIMUM FOV display
        self.CELL_HEIGHT = 750   # INCREASED for MAXIMUM FOV display
        self.HEADER_HEIGHT = 120  # INCREASED for better spacing
        self.TITLE_HEIGHT = 140   # INCREASED for title
        self.GRID_ROWS = 2
        self.GRID_COLS = 3
        
        # Calculate total canvas size once - THESE VALUES NEVER CHANGE
        self.CANVAS_WIDTH = self.GRID_COLS * self.CELL_WIDTH
        self.CANVAS_HEIGHT = self.GRID_ROWS * (self.CELL_HEIGHT + self.HEADER_HEIGHT) + self.TITLE_HEIGHT
        
        # ✅ MEGA FIXED: Create window ONCE with EXACT properties
        self.window_name = 'HUSKYBOT MULTICAM PARALLEL'
        self.window_created = False
        
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
        """Create 2x3 grid display - MEGA FIXED ALL ISSUES"""
        try:
            # ✅ MEGA FIXED: Create canvas with ABSOLUTE FIXED SIZE for MAXIMUM FOV
            canvas = np.zeros((self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3), dtype=np.uint8)
            
            # ✅ MEGA FIXED: Add title - CENTERED with larger font
            title_canvas = canvas[:self.TITLE_HEIGHT, :, :]
            title_text = "HUSKYBOT MULTICAM SEGMENTATION"
            
            # Calculate center position for title
            (text_width, text_height), _ = cv2.getTextSize(title_text, cv2.FONT_HERSHEY_SIMPLEX, 3.0, 8)
            title_x = (self.CANVAS_WIDTH - text_width) // 2  # PERFECTLY CENTERED
            title_y = (self.TITLE_HEIGHT + text_height) // 2
            
            # Draw title with ultra shadow effect
            cv2.putText(title_canvas, title_text, 
                       (title_x + 5, title_y + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 0), 12)  # Ultra Shadow
            cv2.putText(title_canvas, title_text, 
                       (title_x, title_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 255, 255), 8)  # Main text
            
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
                
                # ✅ MEGA FIXED: Calculate positions with ABSOLUTE FIXED sizes
                x_start = col * self.CELL_WIDTH
                y_start = self.TITLE_HEIGHT + row * (self.CELL_HEIGHT + self.HEADER_HEIGHT)
                x_end = x_start + self.CELL_WIDTH
                y_end = y_start + self.HEADER_HEIGHT + self.CELL_HEIGHT
                
                # ✅ MEGA FIXED: Draw camera label header - SOLID BLACK BACKGROUND + PROPER SPACING
                header_y_start = y_start + 15  # FIXED: Added spacing from border
                header_y_end = y_start + self.HEADER_HEIGHT - 15  # FIXED: Added spacing to border
                
                # ✅ FIXED: Header background - SOLID BLACK (no gradient)
                cv2.rectangle(canvas, (x_start + 10, header_y_start), (x_end - 10, header_y_end), 
                             (0, 0, 0), -1)  # SOLID BLACK background
                
                # ✅ MEGA FIXED: Header text - CENTERED and ULTRA BRIGHT with degrees
                # Split camera name and degrees for better formatting
                if '(' in real_name:
                    camera_part = real_name.split('(')[0].strip()
                    degree_part = '(' + real_name.split('(')[1]
                    
                    # Draw camera name
                    (camera_text_width, camera_text_height), _ = cv2.getTextSize(camera_part, cv2.FONT_HERSHEY_SIMPLEX, 1.4, 5)
                    camera_text_x = x_start + (self.CELL_WIDTH - camera_text_width) // 2
                    camera_text_y = header_y_start + 35
                    
                    # Draw degree info
                    (degree_text_width, degree_text_height), _ = cv2.getTextSize(degree_part, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 4)
                    degree_text_x = x_start + (self.CELL_WIDTH - degree_text_width) // 2
                    degree_text_y = header_y_start + 75
                    
                    # Draw with shadow effects
                    cv2.putText(canvas, camera_part, 
                               (camera_text_x + 3, camera_text_y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.4, (50, 50, 50), 7)  # Shadow
                    cv2.putText(canvas, camera_part, 
                               (camera_text_x, camera_text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 5)  # Main text
                    
                    cv2.putText(canvas, degree_part, 
                               (degree_text_x + 3, degree_text_y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.1, (50, 50, 50), 6)  # Shadow
                    cv2.putText(canvas, degree_part, 
                               (degree_text_x, degree_text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 0), 4)  # Degree text
                else:
                    # Fallback for names without degrees
                    (header_text_width, header_text_height), _ = cv2.getTextSize(real_name, cv2.FONT_HERSHEY_SIMPLEX, 1.4, 5)
                    header_text_x = x_start + (self.CELL_WIDTH - header_text_width) // 2
                    header_text_y = header_y_start + ((header_y_end - header_y_start) + header_text_height) // 2
                    
                    cv2.putText(canvas, real_name, 
                               (header_text_x + 3, header_text_y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.4, (50, 50, 50), 7)  # Shadow
                    cv2.putText(canvas, real_name, 
                               (header_text_x, header_text_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 5)  # Main text
                
                # Image area
                img_y_start = y_start + self.HEADER_HEIGHT
                img_y_end = y_end
                
                if img is not None:
                    # ✅ MEGA FIXED: Resize image to MAXIMUM CELL SIZE for FULL FOV
                    img_resized = cv2.resize(img, (self.CELL_WIDTH, self.CELL_HEIGHT), 
                                           interpolation=cv2.INTER_LINEAR)
                    canvas[img_y_start:img_y_end, x_start:x_end] = img_resized
                else:
                    # Create waiting image with EXACT FIXED size
                    waiting_img = np.zeros((self.CELL_HEIGHT, self.CELL_WIDTH, 3), dtype=np.uint8)
                    waiting_text = "WAITING FOR SIGNAL..."
                    
                    # Center waiting text
                    (wait_text_width, wait_text_height), _ = cv2.getTextSize(waiting_text, cv2.FONT_HERSHEY_SIMPLEX, 1.8, 5)
                    wait_x = (self.CELL_WIDTH - wait_text_width) // 2
                    wait_y = (self.CELL_HEIGHT + wait_text_height) // 2
                    
                    cv2.putText(waiting_img, waiting_text, 
                               (wait_x + 3, wait_y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 0, 0), 8)  # Shadow
                    cv2.putText(waiting_img, waiting_text, 
                               (wait_x, wait_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.8, (255, 255, 255), 5)  # Main text
                    canvas[img_y_start:img_y_end, x_start:x_end] = waiting_img
                
                # ✅ Draw grid lines - MEGA THICK with 3D effect
                # Outer border
                cv2.rectangle(canvas, (x_start, y_start), (x_end-1, y_end-1), (220, 220, 220), 8)
                # Inner border
                cv2.rectangle(canvas, (x_start+4, y_start+4), (x_end-5, y_end-5), (120, 120, 120), 3)
            
            # ✅ MEGA FIXED: Create and manage window with ABSOLUTE FIXED properties
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
                
                # ✅ MEGA FIXED: Set window properties to PREVENT resizing
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_KEEPRATIO)
                
                self.window_created = True
                self.get_logger().info(f"✅ Window created: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}")
            
            # ✅ MEGA FIXED: Display with EXACT same size every time
            cv2.imshow(self.window_name, canvas)
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