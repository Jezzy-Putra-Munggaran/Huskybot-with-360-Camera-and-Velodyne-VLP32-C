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
        self.detection_counts = {}
        self.fps_counters = {}
        self.fps_timers = {}
        
        # ✅ WIDER GRID: Increased dimensions for wider FOV display
        self.CELL_WIDTH = 1200   # WIDER for better FOV display
        self.CELL_HEIGHT = 800   # TALLER for better FOV display
        self.HEADER_HEIGHT = 100  
        self.TITLE_HEIGHT = 120   
        self.STATUS_HEIGHT = 100  # Status bar at bottom
        self.GRID_ROWS = 2
        self.GRID_COLS = 3
        
        # Calculate total canvas size - WIDER DISPLAY
        self.CANVAS_WIDTH = self.GRID_COLS * self.CELL_WIDTH
        self.CANVAS_HEIGHT = self.GRID_ROWS * (self.CELL_HEIGHT + self.HEADER_HEIGHT) + self.TITLE_HEIGHT + self.STATUS_HEIGHT
        
        # ✅ Create window once
        self.window_name = 'HUSKYBOT MULTICAM PARALLEL - WIDER FOV'
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
            self.detection_counts[name] = 0
            self.fps_counters[name] = 0
            self.fps_timers[name] = time.time()
            
            # Create subscriber with larger queue for stability
            self.subscribers[name] = self.create_subscription(
                Image, topic, 
                lambda msg, n=name: self.camera_callback(msg, n), 
                20  # Larger queue for stability
            )
            
            self.get_logger().info(f"📡 Subscribed to {topic}")

    def camera_callback(self, msg, camera_name):
        """Camera callback for processed images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_locks[camera_name]:
                self.latest_images[camera_name] = cv_image.copy()
            
            # Update FPS counter
            self.fps_counters[camera_name] += 1
            if self.fps_counters[camera_name] % 30 == 0:
                current_time = time.time()
                fps = 30.0 / (current_time - self.fps_timers[camera_name])
                self.fps_timers[camera_name] = current_time
                self.get_logger().info(f"🔥 {camera_name} Display FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error for {camera_name}: {e}")

    def setup_display(self):
        """Setup display loop"""
        self.display_active = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        self.get_logger().info("✅ Display thread started!")

    def display_loop(self):
        """Main display loop - WIDER Grid 2x3"""
        while self.display_active:
            try:
                self.create_wider_grid_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_wider_grid_display(self):
        """Create WIDER 2x3 grid display with status bar"""
        try:
            # ✅ Create WIDER canvas
            canvas = np.zeros((self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3), dtype=np.uint8)
            
            # ✅ Title section
            title_canvas = canvas[:self.TITLE_HEIGHT, :, :]
            title_text = "HUSKYBOT MULTICAM PARALLEL - WIDER FOV SEGMENTATION"
            
            # Center title
            (text_width, text_height), _ = cv2.getTextSize(title_text, cv2.FONT_HERSHEY_SIMPLEX, 2.5, 6)
            title_x = (self.CANVAS_WIDTH - text_width) // 2
            title_y = (self.TITLE_HEIGHT + text_height) // 2
            
            # Draw title with shadow
            cv2.putText(title_canvas, title_text, 
                       (title_x + 3, title_y + 3), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 0, 0), 8)  # Shadow
            cv2.putText(title_canvas, title_text, 
                       (title_x, title_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 255, 255), 6)  # Main text
            
            # ✅ Camera grid section
            total_detections = 0
            active_cameras = 0
            
            for config in self.camera_configs:
                name = config['name']
                real_name = config['real_name']
                row, col = config['position']
                
                # Get latest image
                with self.image_locks[name]:
                    if self.latest_images[name] is not None:
                        img = self.latest_images[name].copy()
                        active_cameras += 1
                    else:
                        img = None
                
                # Calculate grid positions
                x_start = col * self.CELL_WIDTH
                y_start = self.TITLE_HEIGHT + row * (self.CELL_HEIGHT + self.HEADER_HEIGHT)
                x_end = x_start + self.CELL_WIDTH
                y_end = y_start + self.HEADER_HEIGHT + self.CELL_HEIGHT
                
                # Draw camera header
                header_y_start = y_start + 10
                header_y_end = y_start + self.HEADER_HEIGHT - 10
                
                # Header background
                cv2.rectangle(canvas, (x_start + 5, header_y_start), (x_end - 5, header_y_end), 
                             (20, 20, 20), -1)
                
                # Header text
                (header_text_width, header_text_height), _ = cv2.getTextSize(real_name, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
                header_text_x = x_start + (self.CELL_WIDTH - header_text_width) // 2
                header_text_y = header_y_start + ((header_y_end - header_y_start) + header_text_height) // 2
                
                # Estimate detections from image
                detection_count = self.estimate_detections(img) if img is not None else 0
                self.detection_counts[name] = detection_count
                total_detections += detection_count
                
                # Draw header with detection count
                header_with_count = f"{real_name} (Objects: {detection_count})"
                (header_full_width, _), _ = cv2.getTextSize(header_with_count, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
                header_full_x = x_start + (self.CELL_WIDTH - header_full_width) // 2
                
                cv2.putText(canvas, header_with_count, 
                           (header_full_x + 2, header_text_y + 2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)  # Shadow
                cv2.putText(canvas, header_with_count, 
                           (header_full_x, header_text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)  # Main text
                
                # Image area
                img_y_start = y_start + self.HEADER_HEIGHT
                img_y_end = y_end
                
                if img is not None:
                    # ✅ WIDER: Resize to FULL cell size for maximum FOV display
                    img_resized = cv2.resize(img, (self.CELL_WIDTH, self.CELL_HEIGHT), 
                                           interpolation=cv2.INTER_LINEAR)
                    canvas[img_y_start:img_y_end, x_start:x_end] = img_resized
                else:
                    # Create placeholder
                    placeholder = np.zeros((self.CELL_HEIGHT, self.CELL_WIDTH, 3), dtype=np.uint8)
                    placeholder[:] = (30, 30, 30)
                    
                    # Placeholder text
                    wait_text = "LOADING CAMERA..."
                    (wait_width, wait_height), _ = cv2.getTextSize(wait_text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 4)
                    wait_x = (self.CELL_WIDTH - wait_width) // 2
                    wait_y = (self.CELL_HEIGHT + wait_height) // 2
                    
                    cv2.putText(placeholder, wait_text, 
                               (wait_x + 3, wait_y + 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 6)  # Shadow
                    cv2.putText(placeholder, wait_text, 
                               (wait_x, wait_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)  # Main text
                    
                    canvas[img_y_start:img_y_end, x_start:x_end] = placeholder
                
                # Grid borders
                cv2.rectangle(canvas, (x_start, y_start), (x_end-1, y_end-1), (100, 100, 100), 4)
                cv2.rectangle(canvas, (x_start+2, y_start+2), (x_end-3, y_end-3), (200, 200, 200), 2)
            
            # ✅ Status bar at bottom
            status_y = self.CANVAS_HEIGHT - self.STATUS_HEIGHT
            status_area = canvas[status_y:, :]
            status_area[:] = (15, 15, 15)
            
            # Status information
            current_time = time.strftime("%H:%M:%S")
            main_status = f"HUSKYBOT PARALLEL | Active: {active_cameras}/6 | Total Objects: {total_detections} | Time: {current_time}"
            
            # Center status text
            (status_width, status_height), _ = cv2.getTextSize(main_status, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            status_x = (self.CANVAS_WIDTH - status_width) // 2
            status_y_pos = 40
            
            cv2.putText(status_area, main_status,
                       (status_x + 2, status_y_pos + 2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)  # Shadow
            cv2.putText(status_area, main_status,
                       (status_x, status_y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)  # Main text
            
            # Performance info
            perf_info = f"Target: 100+ FPS | Processing: PARALLEL | FOV: ARDUCAM IMX477 NATIVE | Resolution: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}"
            (perf_width, _), _ = cv2.getTextSize(perf_info, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            perf_x = (self.CANVAS_WIDTH - perf_width) // 2
            
            cv2.putText(status_area, perf_info,
                       (perf_x + 1, 75 + 1), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)  # Shadow
            cv2.putText(status_area, perf_info,
                       (perf_x, 75), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)  # Main text
            
            # ✅ Create and display WIDER window
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
                cv2.moveWindow(self.window_name, 0, 0)
                
                self.window_created = True
                self.get_logger().info(f"✅ WIDER Window created: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}")
            
            cv2.imshow(self.window_name, canvas)
            
            # Handle window close
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or Escape
                self.get_logger().info("🛑 User requested shutdown")
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"❌ WIDER Grid display error: {e}")

    def estimate_detections(self, img):
        """Estimate number of detections from processed image"""
        if img is None:
            return 0
        
        try:
            # Simple heuristic: count colored rectangles (bounding boxes)
            # Look for rectangular contours which indicate bounding boxes
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detection_count = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 2000:  # Minimum area for valid detection
                    # Check if it's roughly rectangular
                    x, y, w, h = cv2.boundingRect(contour)
                    if w > 60 and h > 60:  # Minimum size for detection
                        detection_count += 1
            
            return min(detection_count, 25)  # Cap at reasonable number
            
        except:
            return 0

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
        print("🛑 Shutting down MULTICAM PARALLEL WIDER...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()