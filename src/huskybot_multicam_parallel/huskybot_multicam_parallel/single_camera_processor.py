#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_multicam_parallel/huskybot_multicam_parallel/single_camera_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os

class SingleCameraProcessor(Node):
    def __init__(self):
        super().__init__('single_camera_processor')
        
        self.bridge = CvBridge()
        
        # ✅ Get parameters from ROS2 parameter server
        self.declare_parameter('camera_name', 'camera_rear')
        self.declare_parameter('camera_topic', '/camera_rear/image_raw')
        self.declare_parameter('camera_real_name', 'FRONT CAMERA')
        self.declare_parameter('camera_idx', 3)
        
        # ✅ Camera configuration
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_real_name = self.get_parameter('camera_real_name').get_parameter_value().string_value
        self.camera_idx = self.get_parameter('camera_idx').get_parameter_value().integer_value
        
        # ✅ Data storage
        self.latest_image = None
        self.detection_result = []
        self.frame_lock = threading.Lock()
        self.fps_counter = 0
        self.fps_timer = time.time()
        
        # ✅ Setup YOLO
        self.setup_yolo()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup processing
        self.setup_processing()
        
        self.get_logger().info(f"🚀 {self.camera_real_name} PROCESSOR STARTED!")

    def setup_yolo(self):
        """Setup YOLO"""
        try:
            from ultralytics import YOLO
            
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.pt",
                "yolo11m-seg.pt",
                "yolo11n-seg.pt"
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading model: {model_path}")
                        self.yolo_model = YOLO(model_path)
                        
                        # Test model
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        results = self.yolo_model.predict(test_img, verbose=False, task='segment')
                        
                        self.get_logger().info(f"✅ SUCCESS! Model loaded: {model_path}")
                        break
                        
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed: {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO YOLO MODEL LOADED!")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOLO setup failed: {e}")
            self.yolo_model = None

    def setup_connections(self):
        """Setup connections"""
        try:
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 10)
            self.get_logger().info(f"📡 Subscribed: {self.camera_topic}")
            
            # Publisher for processed results
            self.result_pub = self.create_publisher(
                Image, f'/{self.camera_name}_processed', 10)
            self.get_logger().info(f"📡 Publisher created: /{self.camera_name}_processed")
            
        except Exception as e:
            self.get_logger().error(f"❌ Connection setup failed: {e}")

    def setup_processing(self):
        """Setup processing"""
        self.processing_active = True
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()
        self.get_logger().info(f"✅ {self.camera_real_name} processing thread started!")

    def camera_callback(self, msg):
        """Camera callback with MAXIMUM WIDER FOV for Arducam IMX477"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ MAXIMUM WIDER FOV: Apply ultra-wide FOV optimization for Arducam IMX477
            cv_image = self.apply_maximum_wider_fov_arducam(cv_image)
            
            with self.frame_lock:
                self.latest_image = cv_image.copy()
            
            # FPS tracking
            self.fps_counter += 1
            if self.fps_counter % 100 == 0:
                current_time = time.time()
                fps = 100.0 / (current_time - self.fps_timer)
                self.fps_timer = current_time
                self.get_logger().info(f"🔥 {self.camera_real_name} FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error: {e}")

    def apply_maximum_wider_fov_arducam(self, image):
        """Apply MAXIMUM WIDER FOV optimization for Arducam IMX477"""
        try:
            height, width = image.shape[:2]
            
            # ✅ ARDUCAM IMX477 MAXIMUM SPECS:
            # - Native resolution: 4032x3040 (4:3 ratio)
            # - Sensor diagonal: 7.9mm
            # - Maximum usable FOV: ~83° diagonal, ~75° horizontal, ~62° vertical
            # - Target: Extract MAXIMUM usable area for ultra-wide display
            
            # ✅ STEP 1: Use MAXIMUM center crop (95% of sensor area)
            crop_ratio = 0.95  # Use 95% of image for MAXIMUM usable FOV
            
            center_x = width // 2
            center_y = height // 2
            crop_width = int(width * crop_ratio)
            crop_height = int(height * crop_ratio)
            
            x1 = center_x - crop_width // 2
            y1 = center_y - crop_height // 2
            x2 = center_x + crop_width // 2
            y2 = center_y + crop_height // 2
            
            # Ensure bounds
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(width, x2)
            y2 = min(height, y2)
            
            cropped = image[y1:y2, x1:x2]
            
            # ✅ STEP 2: Resize to ULTRA-WIDE display ratio
            # Target: 21:9 ultra-wide aspect ratio for MAXIMUM FOV display
            target_width = 2100   # Ultra-wide for maximum horizontal coverage
            target_height = 900   # 21:9 ultra-wide ratio
            
            # Ultra-high quality resize
            resized = cv2.resize(cropped, (target_width, target_height), 
                               interpolation=cv2.INTER_LANCZOS4)
            
            # ✅ STEP 3: Enhanced distortion correction for MAXIMUM FOV recovery
            corrected = self.apply_enhanced_distortion_correction(resized)
            
            return corrected
            
        except Exception as e:
            self.get_logger().error(f"❌ MAXIMUM FOV optimization error: {e}")
            return image

    def apply_enhanced_distortion_correction(self, image):
        """Apply enhanced distortion correction for MAXIMUM FOV recovery"""
        try:
            height, width = image.shape[:2]
            
            # ✅ ARDUCAM IMX477: Enhanced camera matrix for MAXIMUM FOV
            camera_matrix = np.array([
                [width * 0.75, 0, width / 2],     # fx - optimized for ultra-wide
                [0, height * 0.75, height / 2],   # fy - optimized for ultra-wide
                [0, 0, 1]
            ], dtype=np.float32)
            
            # ✅ Enhanced distortion coefficients for MAXIMUM FOV recovery
            dist_coeffs = np.array([
                -0.25,  # k1 - moderate barrel correction for wide FOV
                0.1,    # k2 - slight positive for edge balance
                0.0,    # p1 - no tangential distortion
                0.0,    # p2 - no tangential distortion
                -0.05   # k3 - slight negative for edge enhancement
            ], dtype=np.float32)
            
            # Apply enhanced undistortion for MAXIMUM FOV
            optimized_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (width, height), 
                alpha=1.0,  # Keep ALL pixels for MAXIMUM FOV
                newImgSize=(width, height)
            )
            
            undistorted = cv2.undistort(
                image, camera_matrix, dist_coeffs, 
                None, optimized_camera_matrix
            )
            
            return undistorted
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced distortion correction error: {e}")
            return image

    def processing_loop(self):
        """Ultra-high performance processing loop"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                try:
                    with self.frame_lock:
                        if self.latest_image is not None:
                            frame = self.latest_image.copy()
                        else:
                            continue
                    
                    # Store original frame
                    original_frame = frame.copy()
                    
                    # ✅ ULTRA-FAST: Process at optimized resolution for 100+ FPS
                    frame_resized = cv2.resize(frame, (640, 640))
                    scale_x = frame.shape[1] / 640
                    scale_y = frame.shape[0] / 640
                    
                    # YOLO inference with ULTRA-HIGH performance settings
                    results = self.yolo_model.predict(
                        frame_resized,
                        conf=0.3,     # Balanced confidence for good detection
                        iou=0.45,     # Standard IoU
                        verbose=False,
                        task='segment',
                        device=0,
                        imgsz=640,    # Optimized size for speed
                        half=True,    # FP16 for ultra-high speed
                        stream=True   # Stream mode for better performance
                    )
                    
                    # Process results with MAXIMUM FOV coordinates
                    detections = self.process_results_maximum_fov(results, self.camera_idx, original_frame, scale_x, scale_y)
                    
                    with self.frame_lock:
                        self.detection_result = detections
                    
                    # Terminal output with enhanced info
                    for detection in detections:
                        terminal_output = (
                            f"Camera: {self.camera_real_name} | "
                            f"Class: {detection['class']} | "
                            f"Confidence: {detection['confidence']:.2f} | "
                            f"Distance: {detection['distance']:.1f}m | "
                            f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                        )
                        self.get_logger().info(terminal_output)
                    
                    # Create and publish enhanced processed image
                    processed_img = self.create_enhanced_processed_image(original_frame, detections)
                    if processed_img is not None:
                        try:
                            processed_msg = self.bridge.cv2_to_imgmsg(processed_img, 'bgr8')
                            processed_msg.header.stamp = self.get_clock().now().to_msg()
                            processed_msg.header.frame_id = f"{self.camera_name}_frame"
                            self.result_pub.publish(processed_msg)
                        except Exception as e:
                            self.get_logger().error(f"❌ Publish error: {e}")
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Processing error: {e}")
                
                time.sleep(0.008)  # 125 FPS target (ultra-high performance)
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_results_maximum_fov(self, results, camera_idx, original_frame, scale_x, scale_y):
        """Process results with MAXIMUM FOV parameters"""
        detections = []
        
        try:
            if not results:
                return detections
            
            # Handle both single result and list of results
            if hasattr(results, '__iter__'):
                result = next(iter(results))
            else:
                result = results
            
            original_height, original_width = original_frame.shape[:2]
            
            # ✅ ARDUCAM IMX477: Enhanced camera angles for 360° coverage
            camera_angles = [180, 240, 300, 0, 60, 120]  # Real physical angles
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # Process masks with enhanced quality
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # Scale coordinates back to original frame
                    x1 = int(box[0] * scale_x)
                    y1 = int(box[1] * scale_y)
                    x2 = int(box[2] * scale_x)
                    y2 = int(box[3] * scale_y)
                    
                    # Ensure coordinates are within frame
                    x1 = max(0, min(original_width, x1))
                    y1 = max(0, min(original_height, y1))
                    x2 = max(0, min(original_width, x2))
                    y2 = max(0, min(original_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # Calculate distance with ultra-wide compensation
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance_ultra_wide(class_name, bbox_area, original_width, original_height)
                    
                    # ✅ MAXIMUM FOV: Calculate 3D coordinates with ULTRA-EXPANDED FOV
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ARDUCAM IMX477 MAXIMUM horizontal FOV: 75° (enhanced from native 62°)
                    angle_offset = ((center_x / original_width) - 0.5) * 75  # MAXIMUM 75° horizontal FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # ARDUCAM IMX477 MAXIMUM vertical FOV: 62° (enhanced from native 48°)
                    vertical_angle = ((center_y / original_height) - 0.5) * 62  # MAXIMUM 62° vertical FOV
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # Ultra-distinct colors for COCO classes
                    color = self.get_ultra_distinct_coco_color(int(cls_id))
                    text_color = self.get_ultra_contrasting_text_color(color)
                    
                    # Process mask with ultra-high quality
                    processed_mask = None
                    if masks is not None and i < len(masks):
                        try:
                            mask = masks[i]
                            mask_resized = cv2.resize(
                                mask.astype(np.float32), 
                                (original_width, original_height), 
                                interpolation=cv2.INTER_LANCZOS4
                            )
                            processed_mask = (mask_resized > 0.5).astype(np.uint8)
                        except:
                            processed_mask = None
                    
                    detection = {
                        'class': class_name,
                        'confidence': float(score),
                        'bbox': (x1, y1, x2, y2),
                        'distance': distance,
                        'x': coord_x,
                        'y': coord_y,
                        'z': coord_z,
                        'angle': object_angle,
                        'color': color,
                        'text_color': text_color,
                        'mask': processed_mask
                    }
                    
                    detections.append(detection)
                    
        except Exception as e:
            self.get_logger().error(f"❌ MAXIMUM FOV result processing error: {e}")
        
        return detections

    def calculate_distance_ultra_wide(self, class_name, bbox_area, frame_width, frame_height):
        """Calculate distance optimized for ultra-wide FOV"""
        object_sizes = {
            'person': 1.7, 'bicycle': 1.8, 'car': 4.5, 'motorcycle': 2.0, 'airplane': 30.0,
            'bus': 12.0, 'train': 50.0, 'truck': 8.0, 'boat': 6.0, 'traffic light': 1.0,
            'fire hydrant': 1.0, 'stop sign': 0.6, 'parking meter': 1.5, 'bench': 1.5,
            'bird': 0.3, 'cat': 0.5, 'dog': 0.6, 'horse': 2.0, 'sheep': 1.0, 'cow': 2.5,
            'elephant': 3.0, 'bear': 1.5, 'zebra': 2.0, 'giraffe': 4.0, 'backpack': 0.5,
            'umbrella': 1.0, 'handbag': 0.3, 'tie': 0.15, 'suitcase': 0.6, 'frisbee': 0.25,
            'bottle': 0.3, 'wine glass': 0.2, 'cup': 0.12, 'chair': 1.0, 'couch': 2.0,
            'laptop': 0.35, 'tv': 1.2, 'book': 0.25, 'cell phone': 0.15
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        frame_area = frame_width * frame_height
        relative_size = bbox_area / frame_area
        
        if relative_size > 0:
            # ARDUCAM IMX477 ultra-wide: Enhanced focal length for ultra-wide FOV
            focal_length = 1200  # Optimized for ultra-wide FOV
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_ultra_distinct_coco_color(self, class_id):
        """Get ultra-distinct colors for 80 COCO classes"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 0, 255), (255, 192, 203), (0, 128, 128),
            (128, 128, 0), (255, 165, 0), (75, 0, 130), (255, 20, 147), (0, 191, 255),
            (50, 205, 50), (255, 69, 0), (138, 43, 226), (255, 215, 0), (220, 20, 60),
            (0, 250, 154), (255, 105, 180), (30, 144, 255), (255, 140, 0), (148, 0, 211),
            (255, 99, 71), (0, 206, 209), (255, 228, 196), (127, 255, 0), (255, 0, 127),
            (70, 130, 180), (255, 160, 122), (32, 178, 170), (255, 182, 193), (135, 206, 235),
            (255, 20, 20), (20, 255, 20), (20, 20, 255), (255, 255, 20), (255, 20, 255),
            (20, 255, 255), (255, 128, 128), (128, 255, 128), (128, 128, 255), (255, 255, 128),
            (128, 255, 255), (255, 128, 255), (128, 128, 255), (192, 192, 192), (128, 128, 128),
            (255, 127, 80), (255, 160, 122), (255, 218, 185), (255, 239, 213), (255, 228, 181),
            (255, 222, 173), (245, 222, 179), (222, 184, 135), (210, 180, 140), (205, 133, 63),
            (160, 82, 45), (139, 69, 19), (255, 245, 238), (250, 240, 230), (253, 245, 230),
            (255, 228, 196), (255, 218, 185), (255, 192, 203), (255, 182, 193), (255, 174, 185),
            (240, 128, 128), (233, 150, 122), (250, 128, 114), (255, 160, 122), (255, 127, 80),
            (255, 99, 71), (255, 69, 0), (255, 140, 0), (255, 165, 0), (255, 215, 0)
        ]
        return colors[class_id % len(colors)]

    def get_ultra_contrasting_text_color(self, bg_color):
        """Get ultra-contrasting text color"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        if brightness > 160:
            return (0, 0, 0)      # Black for bright colors
        elif brightness < 80:
            return (255, 255, 255)  # White for dark colors
        else:
            return (255, 255, 0)    # Yellow for medium colors

    def create_enhanced_processed_image(self, original_frame, detections):
        """Create enhanced processed image with full annotations"""
        try:
            canvas = original_frame.copy()
            
            # Draw detections with enhanced visibility
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # Draw enhanced mask with smooth overlay
                if detection['mask'] is not None:
                    try:
                        mask = detection['mask']
                        mask_colored = np.zeros_like(canvas, dtype=np.uint8)
                        mask_colored[mask == 1] = bbox_color
                        
                        # Apply mask with enhanced alpha blending
                        alpha = 0.4
                        mask_indices = mask == 1
                        if np.any(mask_indices):
                            canvas[mask_indices] = cv2.addWeighted(
                                canvas[mask_indices], 1-alpha, 
                                mask_colored[mask_indices], alpha, 0
                            )
                        
                        # Draw enhanced contours
                        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(canvas, contours, -1, (255, 255, 255), 4)  # White outline
                        cv2.drawContours(canvas, contours, -1, bbox_color, 2)       # Color fill
                        
                    except Exception as e:
                        self.get_logger().error(f"❌ Enhanced mask drawing error: {e}")
                
                # Draw enhanced bounding box
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 255, 255), 5)  # White outer
                cv2.rectangle(canvas, (x1, y1), (x2, y2), bbox_color, 3)       # Color main
                
                # Draw enhanced text information
                info_lines = [
                    f"Camera: {self.camera_real_name}",
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                font_scale = 0.8      # Larger for ultra-wide visibility
                font_thickness = 3    # Thicker for better visibility
                line_height = 35      # More spacing
                
                # Calculate text background size
                max_line_width = 0
                for line in info_lines:
                    (line_width, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
                    max_line_width = max(max_line_width, line_width)
                
                text_bg_height = len(info_lines) * line_height + 20
                text_bg_width = max_line_width + 30
                
                # Smart text positioning
                if y1 - text_bg_height > 20:
                    text_x = x1
                    text_y = y1 - text_bg_height
                else:
                    text_x = x1
                    text_y = y2 + 20
                
                # Keep text within frame
                text_x = max(10, min(canvas.shape[1] - text_bg_width - 10, text_x))
                text_y = max(10, min(canvas.shape[0] - text_bg_height - 10, text_y))
                
                # Draw enhanced text background
                cv2.rectangle(canvas, (text_x-5, text_y-5), 
                             (text_x + text_bg_width + 5, text_y + text_bg_height + 5), 
                             (0, 0, 0), -1)  # Black background
                
                cv2.rectangle(canvas, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             bbox_color, -1)  # Color background
                
                cv2.rectangle(canvas, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             (255, 255, 255), 2)  # White border
                
                # Draw enhanced text
                for i, line in enumerate(info_lines):
                    text_pos_x = text_x + 15
                    text_pos_y = text_y + 25 + i * line_height
                    
                    # Enhanced shadow
                    cv2.putText(canvas, line, 
                               (text_pos_x + 2, text_pos_y + 2),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
                    
                    # Main text
                    cv2.putText(canvas, line, 
                               (text_pos_x, text_pos_y),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)
            
            return canvas
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced processed image creation error: {e}")
            return None

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(0.5)
        super().destroy_node()

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = SingleCameraProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down single camera processor...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()