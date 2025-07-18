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
        
        # ✅ Get parameters from ROS2 parameter server - FIXED
        self.declare_parameter('camera_name', 'camera_rear')
        self.declare_parameter('camera_topic', '/camera_rear/image_raw')
        self.declare_parameter('camera_real_name', 'CAMERA FRONT')
        self.declare_parameter('camera_idx', 3)
        
        # ✅ Camera configuration - FIXED
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
        """Setup YOLO - FIXED"""
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
        """Setup connections - FIXED"""
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
        """Camera callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
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

    def processing_loop(self):
        """Processing loop"""
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
                    
                    # Resize for processing
                    processing_height = 640
                    height, width = frame.shape[:2]
                    if height > processing_height:
                        scale = processing_height / height
                        new_width = int(width * scale)
                        new_height = int(height * scale)
                        frame_resized = cv2.resize(frame, (new_width, new_height))
                    else:
                        frame_resized = frame
                        scale = 1.0
                    
                    # YOLO inference
                    results = self.yolo_model.predict(
                        frame_resized,
                        conf=0.25,
                        iou=0.45,
                        verbose=False,
                        task='segment',
                        device=0
                    )
                    
                    # Process results
                    detections = self.process_results(results, self.camera_idx, original_frame, scale)
                    
                    with self.frame_lock:
                        self.detection_result = detections
                    
                    # Terminal output
                    for detection in detections:
                        terminal_output = (
                            f"📍 {self.camera_real_name} | "
                            f"Class: {detection['class']} | "
                            f"Confidence: {detection['confidence']:.2f} | "
                            f"Distance: {detection['distance']:.1f}m | "
                            f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                        )
                        self.get_logger().info(terminal_output)
                    
                    # Create and publish processed image
                    processed_img = self.create_processed_image(original_frame, detections)
                    if processed_img is not None:
                        try:
                            processed_msg = self.bridge.cv2_to_imgmsg(processed_img, 'bgr8')
                            processed_msg.header.stamp = self.get_clock().now().to_msg()
                            self.result_pub.publish(processed_msg)
                        except Exception as e:
                            self.get_logger().error(f"❌ Publish error: {e}")
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Processing error: {e}")
                
                time.sleep(0.01)  # High-speed processing
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_results(self, results, camera_idx, original_frame, processing_scale):
        """Process results"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            original_height, original_width = original_frame.shape[:2]
            
            # Camera angles - REAL MAPPING
            camera_angles = [180, 240, 300, 0, 60, 120]  # Real angles
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # Process masks
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # Scale coordinates back to original frame
                    x1 = int(box[0] / processing_scale)
                    y1 = int(box[1] / processing_scale)
                    x2 = int(box[2] / processing_scale)
                    y2 = int(box[3] / processing_scale)
                    
                    # Ensure coordinates are within frame
                    x1 = max(0, min(original_width, x1))
                    y1 = max(0, min(original_height, y1))
                    x2 = max(0, min(original_width, x2))
                    y2 = max(0, min(original_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # Calculate distance
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance(class_name, bbox_area, original_width, original_height)
                    
                    # Calculate 3D coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # FOV calculation
                    angle_offset = ((center_x / original_width) - 0.5) * 120
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # Vertical angle
                    vertical_angle = ((center_y / original_height) - 0.5) * 90
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # Colors
                    color = self.get_distinct_coco_color(int(cls_id))
                    text_color = self.get_contrasting_text_color(color)
                    
                    # Process mask
                    processed_mask = None
                    if masks is not None and i < len(masks):
                        processed_mask = self.process_mask_for_original_frame(
                            masks[i], processing_scale, original_width, original_height
                        )
                    
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
            self.get_logger().error(f"❌ Result processing error: {e}")
        
        return detections

    def process_mask_for_original_frame(self, mask, processing_scale, original_width, original_height):
        """Process mask for original frame"""
        try:
            mask_resized = cv2.resize(
                mask.astype(np.uint8), 
                (original_width, original_height), 
                interpolation=cv2.INTER_NEAREST
            )
            return mask_resized
        except Exception as e:
            self.get_logger().error(f"❌ Mask processing error: {e}")
            return None

    def calculate_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Calculate distance"""
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
            focal_length = 900
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_distinct_coco_color(self, class_id):
        """Get distinct colors"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 0, 255), (255, 192, 203), (0, 128, 128),
            (128, 128, 0), (255, 165, 0), (75, 0, 130), (255, 20, 147), (0, 191, 255),
            (50, 205, 50), (255, 69, 0), (138, 43, 226), (255, 215, 0), (220, 20, 60),
            (0, 250, 154), (255, 105, 180), (30, 144, 255), (255, 140, 0), (148, 0, 211),
            (255, 99, 71), (0, 206, 209), (255, 228, 196), (127, 255, 0), (255, 0, 127),
            (70, 130, 180), (255, 160, 122), (32, 178, 170), (255, 182, 193), (135, 206, 235)
        ]
        return colors[class_id % len(colors)]

    def get_contrasting_text_color(self, bg_color):
        """Get contrasting text color"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def create_processed_image(self, original_frame, detections):
        """Create processed image with annotations"""
        try:
            canvas = original_frame.copy()
            
            # Draw detections
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # Draw mask
                if detection['mask'] is not None:
                    try:
                        mask = detection['mask']
                        mask_overlay = np.zeros_like(canvas)
                        mask_overlay[mask > 0] = bbox_color
                        canvas = cv2.addWeighted(canvas, 0.7, mask_overlay, 0.3, 0)
                    except Exception as e:
                        self.get_logger().error(f"❌ Mask drawing error: {e}")
                
                # Draw bounding box
                cv2.rectangle(canvas, (x1, y1), (x2, y2), bbox_color, 3)
                
                # Draw text
                info_lines = [
                    f"Camera: {self.camera_real_name}",
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                font_scale = 0.5
                font_thickness = 2
                line_height = 20
                
                text_bg_height = len(info_lines) * line_height + 10
                text_bg_width = max(len(line) * 10 for line in info_lines) + 20
                
                # Text positioning
                if y1 - text_bg_height > 5:
                    text_x = x1
                    text_y = y1 - text_bg_height
                else:
                    text_x = x1
                    text_y = y2
                
                text_x = max(0, min(canvas.shape[1] - text_bg_width, text_x))
                text_y = max(0, min(canvas.shape[0] - text_bg_height, text_y))
                
                # Draw text background
                cv2.rectangle(canvas, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             bbox_color, -1)
                
                # Draw text lines
                for i, line in enumerate(info_lines):
                    cv2.putText(canvas, line, 
                               (text_x + 10, text_y + 15 + i * line_height),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)
            
            return canvas
            
        except Exception as e:
            self.get_logger().error(f"❌ Processed image creation error: {e}")
            return None

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(0.5)
        super().destroy_node()

def main(args=None):
    """Main function - FIXED"""
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