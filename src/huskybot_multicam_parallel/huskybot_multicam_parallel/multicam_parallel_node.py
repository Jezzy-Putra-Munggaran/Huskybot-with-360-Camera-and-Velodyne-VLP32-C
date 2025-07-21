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
        
        # ✅ Camera configuration - REAL MAPPING CORRECTED
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
        self.detection_counts = {}  # Track detections per camera
        self.fps_counters = {}      # Track FPS per camera
        self.fps_timers = {}        # FPS timers
        
        # ✅ FULLSCREEN OPTIMIZED: Match simple_ultimate_working_node.py dimensions
        self.setup_fullscreen_dimensions()
        
        # ✅ Performance tracking
        self.total_detections = 0
        self.total_fps = 0.0
        self.active_cameras = 0
        
        # ✅ Setup subscribers
        self.setup_subscribers()
        
        # ✅ Setup display
        self.setup_display()
        
        self.get_logger().info("🚀 MULTICAM PARALLEL DISPLAY NODE - FULLSCREEN MODE STARTED!")

    def setup_fullscreen_dimensions(self):
        """Setup FULLSCREEN dimensions like simple_ultimate_working_node.py"""
        # ✅ MAXIMUM resolution per camera for clarity (same as simple_ultimate)
        self.CAM_WIDTH = 960   # High resolution per camera
        self.CAM_HEIGHT = 720  # High resolution per camera
        
        # ✅ Layout configuration
        self.GRID_ROWS = 2
        self.GRID_COLS = 3
        self.STATUS_HEIGHT = 120  # Status bar at bottom
        self.HEADER_HEIGHT = 80   # Header per camera
        
        # ✅ Calculate FULLSCREEN canvas
        self.CANVAS_WIDTH = self.GRID_COLS * self.CAM_WIDTH
        self.CANVAS_HEIGHT = self.GRID_ROWS * (self.CAM_HEIGHT + self.HEADER_HEIGHT) + self.STATUS_HEIGHT
        
        # ✅ Window configuration
        self.window_name = 'HUSKYBOT MULTICAM PARALLEL - FULLSCREEN MODE'
        self.window_created = False
        
        self.get_logger().info(f"🖥️ FULLSCREEN Setup: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}")

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
            
            # Create subscriber
            self.subscribers[name] = self.create_subscription(
                Image, topic, 
                lambda msg, n=name: self.camera_callback(msg, n), 
                10
            )
            
            self.get_logger().info(f"📡 Subscribed to {topic}")

    def camera_callback(self, msg, camera_name):
        """Camera callback for processed images with detection counting"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_locks[camera_name]:
                self.latest_images[camera_name] = cv_image.copy()
            
            # ✅ Update FPS counter
            self.fps_counters[camera_name] += 1
            if self.fps_counters[camera_name] % 30 == 0:
                current_time = time.time()
                fps = 30.0 / (current_time - self.fps_timers[camera_name])
                self.fps_timers[camera_name] = current_time
                self.get_logger().info(f"🔥 {camera_name} FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error for {camera_name}: {e}")

    def setup_display(self):
        """Setup display loop"""
        self.display_active = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        self.get_logger().info("✅ FULLSCREEN Display thread started!")

    def display_loop(self):
        """Main display loop - FULLSCREEN Grid 2x3"""
        while self.display_active:
            try:
                self.create_fullscreen_grid_display()
                time.sleep(0.0167)  # ~60 FPS display for smooth experience
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_fullscreen_grid_display(self):
        """Create FULLSCREEN 2x3 grid display - 100% like simple_ultimate_working_node.py"""
        try:
            # ✅ Create FULLSCREEN canvas
            canvas = np.zeros((self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3), dtype=np.uint8)
            
            # ✅ Camera processing and layout
            grid_images = []
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
                
                # ✅ Process each camera position
                x_start = col * self.CAM_WIDTH
                y_start = row * (self.CAM_HEIGHT + self.HEADER_HEIGHT)
                x_end = x_start + self.CAM_WIDTH
                y_end = y_start + self.HEADER_HEIGHT + self.CAM_HEIGHT
                
                if img is not None:
                    # ✅ FULLSCREEN resize with MAXIMUM FOV
                    img_resized = cv2.resize(img, (self.CAM_WIDTH, self.CAM_HEIGHT), 
                                           interpolation=cv2.INTER_CUBIC)
                    
                    # ✅ Count detections (estimate from processed image)
                    detection_count = self.estimate_detections_from_image(img_resized)
                    self.detection_counts[name] = detection_count
                    total_detections += detection_count
                    
                    # ✅ Draw camera header with detection count
                    self.draw_camera_header(canvas, x_start, y_start, real_name, detection_count)
                    
                    # ✅ Place image in grid
                    img_y_start = y_start + self.HEADER_HEIGHT
                    img_y_end = y_end
                    canvas[img_y_start:img_y_end, x_start:x_end] = img_resized
                    
                else:
                    # ✅ Draw waiting placeholder
                    self.draw_waiting_placeholder(canvas, x_start, y_start, real_name)
                
                # ✅ Draw grid borders
                cv2.rectangle(canvas, (x_start, y_start), (x_end-1, y_end-1), (100, 100, 100), 3)
            
            # ✅ Draw COMPREHENSIVE status bar (like simple_ultimate_working_node.py)
            self.draw_comprehensive_status_bar(canvas, total_detections, active_cameras)
            
            # ✅ Create and display FULLSCREEN window
            self.create_fullscreen_window()
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"❌ FULLSCREEN Grid display error: {e}")

    def draw_camera_header(self, canvas, x_start, y_start, real_name, detection_count):
        """Draw camera header with detection count"""
        header_area = canvas[y_start:y_start + self.HEADER_HEIGHT, x_start:x_start + self.CAM_WIDTH]
        
        # ✅ Clean black background
        header_area[:] = (0, 0, 0)
        
        # ✅ Camera name
        cv2.putText(header_area, real_name,
                   (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        
        # ✅ Detection count
        count_text = f"Objects: {detection_count}"
        cv2.putText(header_area, count_text,
                   (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    def draw_waiting_placeholder(self, canvas, x_start, y_start, real_name):
        """Draw waiting placeholder"""
        # ✅ Header
        self.draw_camera_header(canvas, x_start, y_start, real_name, 0)
        
        # ✅ Waiting image
        img_y_start = y_start + self.HEADER_HEIGHT
        img_y_end = y_start + self.HEADER_HEIGHT + self.CAM_HEIGHT
        waiting_area = canvas[img_y_start:img_y_end, x_start:x_start + self.CAM_WIDTH]
        waiting_area[:] = (30, 30, 30)
        
        # ✅ Waiting text
        waiting_text = "WAITING FOR CAMERA..."
        (text_w, text_h), _ = cv2.getTextSize(waiting_text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
        text_x = (self.CAM_WIDTH - text_w) // 2
        text_y = (self.CAM_HEIGHT + text_h) // 2
        
        cv2.putText(waiting_area, waiting_text,
                   (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    def draw_comprehensive_status_bar(self, canvas, total_detections, active_cameras):
        """Draw comprehensive status bar like simple_ultimate_working_node.py"""
        status_y = self.CANVAS_HEIGHT - self.STATUS_HEIGHT
        status_area = canvas[status_y:, :]
        status_area[:] = (20, 20, 20)  # Dark background
        
        # ✅ Status information
        current_time = time.time()
        
        # ✅ Main status line
        main_status = f"HUSKYBOT MULTICAM PARALLEL | Active: {active_cameras}/6 cameras | Total Objects: {total_detections}"
        cv2.putText(status_area, main_status,
                   (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        
        # ✅ Performance info
        perf_info = f"Target: 100+ FPS | Processing: PARALLEL | FOV: MAXIMUM | Resolution: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}"
        cv2.putText(status_area, perf_info,
                   (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # ✅ Instructions
        instructions = "Real-time 360° Object Segmentation + Distance + 3D Coordinates | Press 'q' to quit"
        cv2.putText(status_area, instructions,
                   (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    def estimate_detections_from_image(self, img):
        """Estimate number of detections from processed image (simple heuristic)"""
        try:
            # ✅ Simple heuristic: count colored rectangles (bounding boxes)
            # This is a rough estimation based on processed image analysis
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Count significant rectangles (potential bounding boxes)
            detection_count = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Minimum area for valid detection
                    x, y, w, h = cv2.boundingRect(contour)
                    if w > 50 and h > 50:  # Minimum size
                        detection_count += 1
            
            return min(detection_count, 20)  # Cap at reasonable number
        except:
            return 0

    def create_fullscreen_window(self):
        """Create FULLSCREEN window"""
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
            cv2.moveWindow(self.window_name, 0, 0)
            
            # ✅ Set fullscreen properties
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            
            self.window_created = True
            self.get_logger().info(f"🖥️ FULLSCREEN Window created: {self.CANVAS_WIDTH}x{self.CANVAS_HEIGHT}")

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
        print("🛑 Shutting down MULTICAM PARALLEL FULLSCREEN...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()