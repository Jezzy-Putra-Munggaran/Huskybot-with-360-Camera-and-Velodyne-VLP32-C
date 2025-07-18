#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_single_camera/huskybot_single_camera/single_camera_ultimate_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os

class SingleCameraUltimateNode(Node):
    def __init__(self):
        super().__init__('single_camera_ultimate_node')
        
        self.bridge = CvBridge()
        
        # ✅ HANYA SATU KAMERA - CAMERA REAR (yang sebenarnya adalah KAMERA DEPAN di real life)
        self.camera_topic = '/camera_rear/image_raw'  # KAMERA DEPAN (Real)
        self.camera_name = 'CAMERA FRONT'  # Sesuai real life
        
        # ✅ SIMPLE data storage - SAMA DENGAN SIMPLE tapi hanya 1 kamera
        self.latest_image = None
        self.detection_result = []
        self.frame_lock = threading.Lock()
        self.fps_counter = 0
        self.fps_timer = time.time()
        self.latest_laser = None
        
        # ✅ Setup YOLO SIMPLE - 100% SAMA DENGAN SIMPLE
        self.setup_simple_yolo()
        
        # ✅ Setup connections - SAMA DENGAN SIMPLE
        self.setup_connections()
        
        # ✅ Setup processing - SAMA DENGAN SIMPLE
        self.setup_processing()
        
        self.get_logger().info("🚀 SINGLE CAMERA ULTIMATE NODE - TESTING PERFORMANCE!")

    def setup_simple_yolo(self):
        """Setup YOLO SIMPLE yang PASTI WORKING - 100% SAMA DENGAN SIMPLE"""
        try:
            from ultralytics import YOLO
            
            # ✅ Try models berdasarkan priority - 100% SAMA DENGAN SIMPLE
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",  # TensorRT FASTEST
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.pt",     # PyTorch
                "yolo11x-seg.pt",                                    # Auto-download
                "yolo11n-seg.pt"                                     # Lightweight
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading model: {model_path}")
                        
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Test model
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
        """Setup connections SIMPLE - SAMA DENGAN SIMPLE"""
        # ✅ Single Camera subscription - SAMA DENGAN SIMPLE
        try:
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic,
                self.camera_callback,
                30  # Good queue size
            )
            self.get_logger().info(f"📡 Subscribed: {self.camera_topic}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed subscribe {self.camera_topic}: {e}")
        
        # ✅ LiDAR subscription - SAMA DENGAN SIMPLE
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            self.get_logger().info("📡 Subscribed to LiDAR")
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR subscription failed: {e}")
        
        # ✅ Publisher - SAMA DENGAN SIMPLE
        try:
            self.single_pub = self.create_publisher(Image, '/single_camera_display', 30)
            self.get_logger().info("📡 Single camera publisher created")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher creation failed: {e}")

    def setup_processing(self):
        """Setup processing SIMPLE - SAMA DENGAN SIMPLE"""
        self.processing_active = True
        
        # ✅ Single processing thread - SAMA DENGAN SIMPLE
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Display thread - SAMA DENGAN SIMPLE
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("✅ Single camera processing threads started!")

    def camera_callback(self, msg):
        """SIMPLE camera callback - SAMA DENGAN SIMPLE"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_lock:
                self.latest_image = cv_image.copy()
            
            # ✅ FPS tracking - SAMA DENGAN SIMPLE
            self.fps_counter += 1
            if self.fps_counter % 100 == 0:
                current_time = time.time()
                fps = 100.0 / (current_time - self.fps_timer)
                self.fps_timer = current_time
                self.get_logger().info(f"🔥 SINGLE CAMERA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error: {e}")

    def laser_callback(self, msg):
        """SIMPLE laser callback - SAMA DENGAN SIMPLE"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def processing_loop(self):
        """SIMPLE processing loop - SAMA DENGAN SIMPLE"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # ✅ Process single camera - SAMA DENGAN SIMPLE
                try:
                    with self.frame_lock:
                        if self.latest_image is not None:
                            frame = self.latest_image.copy()
                        else:
                            continue
                    
                    # ✅ YOLO segmentation inference - SAMA DENGAN SIMPLE
                    results = self.yolo_model.predict(
                        frame,
                        conf=0.25,
                        iou=0.45,
                        verbose=False,
                        task='segment',
                        device=0  # Force GPU
                    )
                    
                    # ✅ Process results - SAMA DENGAN SIMPLE
                    detections = self.process_results(results, frame)
                    self.detection_result = detections
                    
                    # ✅ TERMINAL OUTPUT in FULL ENGLISH - SAMA DENGAN SIMPLE
                    for detection in detections:
                        terminal_output = (
                            f"📍 {self.camera_name} | "
                            f"Class: {detection['class']} | "
                            f"Confidence: {detection['confidence']:.2f} | "
                            f"Distance: {detection['distance']:.1f}m | "
                            f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                        )
                        self.get_logger().info(terminal_output)
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Processing error: {e}")
                
                time.sleep(0.01)  # High-speed processing
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_results(self, results, frame):
        """Process YOLO results dengan FULL FOV - SAMA DENGAN SIMPLE"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # ✅ Camera angle untuk CAMERA REAR (0° - depan robot) - SAMA DENGAN SIMPLE
            base_angle = 0  # CAMERA REAR = KAMERA DEPAN (0°)
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # ✅ Process masks - SAMA DENGAN SIMPLE
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    x1, y1, x2, y2 = map(int, box)
                    
                    # ✅ Ensure coordinates are within frame - SAMA DENGAN SIMPLE
                    x1 = max(0, min(frame_width, x1))
                    y1 = max(0, min(frame_height, y1))
                    x2 = max(0, min(frame_width, x2))
                    y2 = max(0, min(frame_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ Calculate distance - SAMA DENGAN SIMPLE
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ Calculate 3D coordinates - SAMA DENGAN SIMPLE
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ✅ MAXIMUM FOV calculation (120° horizontal FOV untuk Arducam IMX477) - SAMA DENGAN SIMPLE
                    angle_offset = ((center_x / frame_width) - 0.5) * 120  # 120° horizontal FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # ✅ Vertical angle calculation (90° vertical FOV) - SAMA DENGAN SIMPLE
                    vertical_angle = ((center_y / frame_height) - 0.5) * 90
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))  # Camera height 1.5m
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # ✅ DISTINCT COCO colors - SAMA DENGAN SIMPLE
                    color = self.get_distinct_coco_color(int(cls_id))
                    text_color = self.get_contrasting_text_color(color)
                    
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
                        'mask': masks[i] if masks is not None and i < len(masks) else None
                    }
                    
                    detections.append(detection)
                    
        except Exception as e:
            self.get_logger().error(f"❌ Result processing error: {e}")
        
        return detections

    def calculate_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Calculate distance based on COCO object real-world sizes - SAMA DENGAN SIMPLE"""
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
            # ✅ Distance calculation optimized for Arducam IMX477 - SAMA DENGAN SIMPLE
            focal_length = 900  # Calibrated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_distinct_coco_color(self, class_id):
        """Generate 80 DISTINCT colors for COCO classes - SAMA DENGAN SIMPLE"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 0, 255), (255, 192, 203), (0, 128, 128),
            (128, 128, 0), (255, 165, 0), (75, 0, 130), (255, 20, 147), (0, 191, 255),
            (50, 205, 50), (255, 69, 0), (138, 43, 226), (255, 215, 0), (220, 20, 60),
            (0, 250, 154), (255, 105, 180), (30, 144, 255), (255, 140, 0), (148, 0, 211),
            (255, 99, 71), (0, 206, 209), (255, 228, 196), (127, 255, 0), (255, 0, 127),
            (70, 130, 180), (255, 160, 122), (32, 178, 170), (255, 182, 193), (135, 206, 235),
            (255, 218, 185), (152, 251, 152), (255, 240, 245), (175, 238, 238), (255, 228, 181),
            (221, 160, 221), (255, 239, 213), (173, 216, 230), (255, 218, 185), (144, 238, 144),
            (255, 192, 203), (176, 196, 222), (255, 255, 224), (255, 239, 213), (230, 230, 250),
            (255, 228, 225), (255, 248, 220), (255, 245, 238), (240, 255, 240), (255, 250, 240),
            (255, 255, 240), (240, 248, 255), (248, 248, 255), (245, 245, 245), (255, 250, 250),
            (255, 255, 255), (0, 0, 0), (105, 105, 105), (128, 128, 128), (169, 169, 169),
            (192, 192, 192), (211, 211, 211), (220, 220, 220), (245, 245, 245), (255, 250, 250),
            (240, 255, 255), (255, 255, 240), (255, 255, 224), (255, 250, 205), (250, 250, 210),
            (255, 239, 213), (255, 228, 181), (255, 218, 185), (255, 192, 203), (255, 182, 193)
        ]
        
        return colors[class_id % len(colors)]

    def get_contrasting_text_color(self, bg_color):
        """Get contrasting text color - SAMA DENGAN SIMPLE"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def display_loop(self):
        """SIMPLE display loop - SAMA DENGAN SIMPLE"""
        while self.processing_active:
            try:
                self.create_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_display(self):
        """Create single camera display dengan MAXIMUM clarity - SAMA DENGAN SIMPLE"""
        try:
            if self.latest_image is not None:
                with self.frame_lock:
                    img = self.latest_image.copy()
                
                # ✅ LARGE size untuk clarity - SAMA DENGAN SIMPLE
                display_width, display_height = 1920, 1080  # Full HD display
                
                # ✅ HIGH-QUALITY resize maintaining aspect ratio - SAMA DENGAN SIMPLE
                height, width = img.shape[:2]
                scale = min(display_width/width, display_height/height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                
                img_resized = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
                
                # ✅ Create canvas and center image - SAMA DENGAN SIMPLE
                canvas = np.zeros((display_height, display_width, 3), dtype=np.uint8)
                y_offset = (display_height - new_height) // 2
                x_offset = (display_width - new_width) // 2
                canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = img_resized
                
                # ✅ Draw detections - SAMA DENGAN SIMPLE
                if self.detection_result:
                    canvas = self.draw_detections(canvas, self.detection_result, scale, x_offset, y_offset)
                
                # ✅ Camera label - SAMA DENGAN SIMPLE
                label_height = 80
                cv2.rectangle(canvas, (0, 0), (display_width, label_height), (0, 0, 0), -1)
                cv2.putText(canvas, f"{self.camera_name}", 
                           (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 255), 4)
                
                # ✅ Detection count - SAMA DENGAN SIMPLE
                det_count = len(self.detection_result)
                cv2.putText(canvas, f"Objects: {det_count}", 
                           (display_width-400, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                
                # ✅ STATUS BAR - SAMA DENGAN SIMPLE
                status_height = 80
                canvas_with_status = np.zeros((display_height + status_height, display_width, 3), dtype=np.uint8)
                canvas_with_status[:display_height, :] = canvas
                
                # ✅ Status information - SAMA DENGAN SIMPLE
                main_status = f"HUSKYBOT SINGLE CAMERA YOLO SEGMENTATION | {self.camera_name} | Objects: {det_count}"
                cv2.putText(canvas_with_status, main_status, 
                           (50, display_height + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                
                # ✅ Show using OpenCV - SAMA DENGAN SIMPLE
                cv2.namedWindow('HUSKYBOT SINGLE CAMERA SEGMENTATION', cv2.WINDOW_NORMAL)
                cv2.imshow('HUSKYBOT SINGLE CAMERA SEGMENTATION', canvas_with_status)
                cv2.waitKey(1)
                
                # ✅ Publish - SAMA DENGAN SIMPLE
                try:
                    display_msg = self.bridge.cv2_to_imgmsg(canvas_with_status, 'bgr8')
                    display_msg.header.stamp = self.get_clock().now().to_msg()
                    self.single_pub.publish(display_msg)
                except Exception as e:
                    self.get_logger().error(f"❌ Display publish error: {e}")
            
            else:
                # ✅ Waiting screen - SAMA DENGAN SIMPLE
                waiting_img = np.zeros((1080, 1920, 3), dtype=np.uint8)
                cv2.putText(waiting_img, f"{self.camera_name}", 
                           (1920//3, 1080//2-50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
                cv2.putText(waiting_img, "WAITING FOR SIGNAL...", 
                           (1920//4, 1080//2+50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                
                cv2.namedWindow('HUSKYBOT SINGLE CAMERA SEGMENTATION', cv2.WINDOW_NORMAL)
                cv2.imshow('HUSKYBOT SINGLE CAMERA SEGMENTATION', waiting_img)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"❌ Display creation error: {e}")

    def draw_detections(self, img, detections, scale, x_offset, y_offset):
        """Draw detections dengan PERFECT text positioning dan LARGER FONT - SAMA DENGAN SIMPLE"""
        try:
            for detection in detections:
                # ✅ Scale bbox to display coordinates - SAMA DENGAN SIMPLE
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale) + x_offset
                y1 = int(y1 * scale) + y_offset
                x2 = int(x2 * scale) + x_offset
                y2 = int(y2 * scale) + y_offset
                
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ Draw segmentation mask - SAMA DENGAN SIMPLE
                if detection['mask'] is not None:
                    mask = detection['mask']
                    # Scale mask to match display
                    mask_height, mask_width = mask.shape
                    mask_scale = min(img.shape[1]/mask_width, img.shape[0]/mask_height)
                    new_mask_width = int(mask_width * mask_scale)
                    new_mask_height = int(mask_height * mask_scale)
                    
                    if new_mask_width > 0 and new_mask_height > 0:
                        mask_resized = cv2.resize(mask.astype(np.uint8), (new_mask_width, new_mask_height))
                        
                        # Apply mask at correct position
                        mask_y_offset = (img.shape[0] - new_mask_height) // 2
                        mask_x_offset = (img.shape[1] - new_mask_width) // 2
                        
                        # Create mask overlay
                        mask_overlay = np.zeros_like(img)
                        if (mask_y_offset + new_mask_height <= img.shape[0] and 
                            mask_x_offset + new_mask_width <= img.shape[1]):
                            mask_overlay[mask_y_offset:mask_y_offset+new_mask_height, 
                                        mask_x_offset:mask_x_offset+new_mask_width][mask_resized > 0] = bbox_color
                            
                            # Blend with original image
                            img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                
                # ✅ Draw bounding box - SAMA DENGAN SIMPLE
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 6)  # Thicker bounding box
                
                # ✅ PERFECT text positioning dengan LARGER FONT - SAMA DENGAN SIMPLE
                info_lines = [
                    f"Camera: {self.camera_name}",
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ LARGER FONT SETTINGS - SAMA DENGAN SIMPLE
                font_scale = 1.2  # Larger font for single camera
                font_thickness = 4  # Thicker text
                line_height = 45  # More line spacing
                
                text_bg_height = len(info_lines) * line_height + 30
                text_bg_width = max(len(line) * 25 for line in info_lines) + 40
                
                # ✅ PERFECT positioning logic - SAMA DENGAN SIMPLE
                if y1 - text_bg_height > 15:  # Space above
                    text_x = x1
                    text_y = y1 - text_bg_height
                    text_draw_y = y1 - 15
                else:  # Space below
                    text_x = x1
                    text_y = y2
                    text_draw_y = y2 + text_bg_height - 15
                
                # ✅ Ensure text stays within image bounds - SAMA DENGAN SIMPLE
                text_x = max(0, min(img.shape[1] - text_bg_width, text_x))
                text_y = max(0, min(img.shape[0] - text_bg_height, text_y))
                
                # ✅ Draw text background - SAMA DENGAN SIMPLE
                cv2.rectangle(img, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             bbox_color, -1)
                
                # ✅ Draw text lines dengan LARGER FONT - SAMA DENGAN SIMPLE
                for i, line in enumerate(info_lines):
                    cv2.putText(img, line, 
                               (text_x + 20, text_y + 35 + i * line_height),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)
            
            return img
            
        except Exception as e:
            self.get_logger().error(f"❌ Drawing error: {e}")
            return img

    def destroy_node(self):
        """Clean shutdown - SAMA DENGAN SIMPLE"""
        self.processing_active = False
        time.sleep(1.0)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = SingleCameraUltimateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down SINGLE CAMERA...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()