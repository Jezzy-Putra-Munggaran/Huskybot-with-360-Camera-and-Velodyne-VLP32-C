#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera_rear/huskybot_camera_rear/camera_rear_ultimate_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os

class CameraRearUltimateNode(Node):
    def __init__(self):
        super().__init__('camera_rear_ultimate_node')
        
        self.bridge = CvBridge()
        
        # ✅ CAMERA REAR ONLY - 100% MIRIP SIMPLE
        self.camera_topic = '/camera_rear/image_raw'  # KAMERA DEPAN (Real Life)
        self.camera_name = 'CAMERA FRONT'  # Real life name
        self.camera_idx = 3  # Index sama dengan simple_ultimate_working_node.py
        
        # ✅ Data storage - 100% MIRIP SIMPLE
        self.latest_image = None
        self.detection_result = []
        self.frame_lock = threading.Lock()
        self.fps_counter = 0
        self.fps_timer = time.time()
        self.latest_laser = None
        
        # ✅ Setup YOLO - 100% MIRIP SIMPLE
        self.setup_yolo()
        
        # ✅ Setup connections - 100% MIRIP SIMPLE
        self.setup_connections()
        
        # ✅ Setup processing - 100% MIRIP SIMPLE
        self.setup_processing()
        
        self.get_logger().info("🚀 CAMERA REAR ULTIMATE NODE - 100% MIRIP SIMPLE!")

    def setup_yolo(self):
        """Setup YOLO - 100% MIRIP SIMPLE"""
        try:
            from ultralytics import YOLO
            
            # ✅ Model paths priority - 100% MIRIP SIMPLE
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",  # TensorRT
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.pt",     # PyTorch
                "yolo11m-seg.pt",                                    # Auto-download
                "yolo11n-seg.pt"                                     # Lightweight
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading model: {model_path}")
                        
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Test model - 100% MIRIP SIMPLE
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
        """Setup connections - 100% MIRIP SIMPLE"""
        # ✅ Camera subscription - 100% MIRIP SIMPLE
        try:
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic,
                self.camera_callback,
                10  # Same as simple
            )
            self.get_logger().info(f"📡 Subscribed: {self.camera_topic}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed subscribe {self.camera_topic}: {e}")
        
        # ✅ LiDAR subscription - 100% MIRIP SIMPLE
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            self.get_logger().info("📡 Subscribed to LiDAR")
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR subscription failed: {e}")
        
        # ✅ Publisher - 100% MIRIP SIMPLE
        try:
            self.display_pub = self.create_publisher(Image, '/camera_rear_display', 10)
            self.get_logger().info("📡 Camera rear display publisher created")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher creation failed: {e}")

    def setup_processing(self):
        """Setup processing - 100% MIRIP SIMPLE"""
        self.processing_active = True
        
        # ✅ Processing thread - 100% MIRIP SIMPLE
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Display thread - 100% MIRIP SIMPLE
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("✅ Camera rear processing threads started!")

    def camera_callback(self, msg):
        """Camera callback - 100% MIRIP SIMPLE"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_lock:
                self.latest_image = cv_image.copy()
            
            # ✅ FPS tracking - 100% MIRIP SIMPLE
            self.fps_counter += 1
            if self.fps_counter % 100 == 0:
                current_time = time.time()
                fps = 100.0 / (current_time - self.fps_timer)
                self.fps_timer = current_time
                self.get_logger().info(f"🔥 CAMERA REAR FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error: {e}")

    def laser_callback(self, msg):
        """Laser callback - 100% MIRIP SIMPLE"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def processing_loop(self):
        """Processing loop - 100% MIRIP SIMPLE"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # ✅ Process camera - 100% MIRIP SIMPLE
                try:
                    with self.frame_lock:
                        if self.latest_image is not None:
                            frame = self.latest_image.copy()
                        else:
                            continue
                    
                    # ✅ Resize frame untuk processing - 100% MIRIP SIMPLE
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
                    
                    # ✅ YOLO inference - 100% MIRIP SIMPLE
                    results = self.yolo_model.predict(
                        frame_resized,
                        conf=0.25,
                        iou=0.45,
                        verbose=False,
                        task='segment',
                        device=0  # Force GPU
                    )
                    
                    # ✅ Process results - 100% MIRIP SIMPLE
                    detections = self.process_results(results, self.camera_idx, frame, scale, 0, 0)
                    self.detection_result = detections
                    
                    # ✅ TERMINAL OUTPUT - 100% MIRIP SIMPLE
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

    def process_results(self, results, camera_idx, frame, scale, x_offset, y_offset):
        """Process results - 100% MIRIP SIMPLE"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # ✅ Camera angles - 100% MIRIP SIMPLE
            camera_angles = [180, 240, 300, 0, 60, 120]  # Sesuai simple
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # ✅ Process masks - 100% MIRIP SIMPLE
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # ✅ Convert coordinates - 100% MIRIP SIMPLE
                    x1 = int((box[0] - x_offset) / scale)
                    y1 = int((box[1] - y_offset) / scale)
                    x2 = int((box[2] - x_offset) / scale)
                    y2 = int((box[3] - y_offset) / scale)
                    
                    # ✅ Ensure coordinates are within frame - 100% MIRIP SIMPLE
                    x1 = max(0, min(frame_width, x1))
                    y1 = max(0, min(frame_height, y1))
                    x2 = max(0, min(frame_width, x2))
                    y2 = max(0, min(frame_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ Calculate distance - 100% MIRIP SIMPLE
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ Calculate 3D coordinates - 100% MIRIP SIMPLE
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ✅ FOV calculation - 100% MIRIP SIMPLE
                    angle_offset = ((center_x / frame_width) - 0.5) * 120  # 120° horizontal FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # ✅ Vertical angle - 100% MIRIP SIMPLE
                    vertical_angle = ((center_y / frame_height) - 0.5) * 90
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # ✅ Colors - 100% MIRIP SIMPLE
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
                        'mask': masks[i] if masks is not None and i < len(masks) else None,
                        'mask_scale': scale,
                        'mask_offset': (x_offset, y_offset)
                    }
                    
                    detections.append(detection)
                    
        except Exception as e:
            self.get_logger().error(f"❌ Result processing error: {e}")
        
        return detections

    def calculate_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Calculate distance - 100% MIRIP SIMPLE"""
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
            focal_length = 900  # Calibrated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_distinct_coco_color(self, class_id):
        """Get distinct colors - 100% MIRIP SIMPLE"""
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
        """Get contrasting text color - 100% MIRIP SIMPLE"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def display_loop(self):
        """Display loop - 100% MIRIP SIMPLE"""
        while self.processing_active:
            try:
                self.create_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_display(self):
        """Create display - 100% MIRIP SIMPLE"""
        try:
            if self.latest_image is not None:
                with self.frame_lock:
                    img = self.latest_image.copy()
                
                # ✅ Full screen display - 100% MIRIP SIMPLE
                display_width, display_height = 1920, 1080
                
                # ✅ Resize maintaining aspect ratio - 100% MIRIP SIMPLE
                height, width = img.shape[:2]
                scale = min(display_width/width, display_height/height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                
                img_resized = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
                
                # ✅ Create canvas - 100% MIRIP SIMPLE
                canvas = np.zeros((display_height, display_width, 3), dtype=np.uint8)
                y_offset = (display_height - new_height) // 2
                x_offset = (display_width - new_width) // 2
                canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = img_resized
                
                # ✅ Draw detections - 100% MIRIP SIMPLE
                if self.detection_result:
                    canvas = self.draw_detections(canvas, self.detection_result, scale, x_offset, y_offset)
                
                # ✅ Camera label - 100% MIRIP SIMPLE
                label_height = 80
                cv2.rectangle(canvas, (0, 0), (display_width, label_height), (0, 0, 0), -1)
                cv2.putText(canvas, f"{self.camera_name}", 
                           (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 255), 4)
                
                # ✅ Detection count - 100% MIRIP SIMPLE
                det_count = len(self.detection_result)
                cv2.putText(canvas, f"Objects: {det_count}", 
                           (display_width-400, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                
                # ✅ Show display - 100% MIRIP SIMPLE
                cv2.namedWindow('HUSKYBOT CAMERA REAR SEGMENTATION', cv2.WINDOW_NORMAL)
                cv2.imshow('HUSKYBOT CAMERA REAR SEGMENTATION', canvas)
                cv2.waitKey(1)
                
                # ✅ Publish - 100% MIRIP SIMPLE
                try:
                    display_msg = self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
                    display_msg.header.stamp = self.get_clock().now().to_msg()
                    self.display_pub.publish(display_msg)
                except Exception as e:
                    self.get_logger().error(f"❌ Display publish error: {e}")
            
            else:
                # ✅ Waiting screen - 100% MIRIP SIMPLE
                waiting_img = np.zeros((1080, 1920, 3), dtype=np.uint8)
                cv2.putText(waiting_img, f"{self.camera_name}", 
                           (1920//3, 1080//2-50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
                cv2.putText(waiting_img, "WAITING FOR SIGNAL...", 
                           (1920//4, 1080//2+50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                
                cv2.namedWindow('HUSKYBOT CAMERA REAR SEGMENTATION', cv2.WINDOW_NORMAL)
                cv2.imshow('HUSKYBOT CAMERA REAR SEGMENTATION', waiting_img)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"❌ Display creation error: {e}")

    def draw_detections(self, img, detections, scale, x_offset, y_offset):
        """Draw detections - 100% MIRIP SIMPLE"""
        try:
            for detection in detections:
                # ✅ Scale bbox - 100% MIRIP SIMPLE
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale) + x_offset
                y1 = int(y1 * scale) + y_offset
                x2 = int(x2 * scale) + x_offset
                y2 = int(y2 * scale) + y_offset
                
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ Draw mask - 100% MIRIP SIMPLE
                if detection['mask'] is not None:
                    try:
                        mask = detection['mask']
                        mask_scale = detection['mask_scale']
                        mask_offset_x, mask_offset_y = detection['mask_offset']
                        
                        # Scale mask
                        mask_height, mask_width = mask.shape
                        scaled_mask_width = int(mask_width * mask_scale * scale)
                        scaled_mask_height = int(mask_height * mask_scale * scale)
                        
                        if scaled_mask_width > 0 and scaled_mask_height > 0:
                            mask_resized = cv2.resize(mask.astype(np.uint8), 
                                                    (scaled_mask_width, scaled_mask_height))
                            
                            # Position mask
                            mask_x = x_offset + int(mask_offset_x * scale)
                            mask_y = y_offset + int(mask_offset_y * scale)
                            
                            # Apply mask
                            if (mask_x >= 0 and mask_y >= 0 and 
                                mask_x + scaled_mask_width <= img.shape[1] and 
                                mask_y + scaled_mask_height <= img.shape[0]):
                                
                                mask_overlay = np.zeros_like(img)
                                mask_overlay[mask_y:mask_y+scaled_mask_height, 
                                           mask_x:mask_x+scaled_mask_width][mask_resized > 0] = bbox_color
                                
                                img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                    except Exception as e:
                        self.get_logger().error(f"❌ Mask drawing error: {e}")
                
                # ✅ Draw bounding box - 100% MIRIP SIMPLE
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 6)
                
                # ✅ Draw text - 100% MIRIP SIMPLE
                info_lines = [
                    f"Camera: {self.camera_name}",
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ Text settings - 100% MIRIP SIMPLE
                font_scale = 1.2
                font_thickness = 4
                line_height = 45
                
                text_bg_height = len(info_lines) * line_height + 30
                text_bg_width = max(len(line) * 25 for line in info_lines) + 40
                
                # ✅ Text positioning - 100% MIRIP SIMPLE
                if y1 - text_bg_height > 15:
                    text_x = x1
                    text_y = y1 - text_bg_height
                else:
                    text_x = x1
                    text_y = y2
                
                text_x = max(0, min(img.shape[1] - text_bg_width, text_x))
                text_y = max(0, min(img.shape[0] - text_bg_height, text_y))
                
                # ✅ Draw text background - 100% MIRIP SIMPLE
                cv2.rectangle(img, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             bbox_color, -1)
                
                # ✅ Draw text lines - 100% MIRIP SIMPLE
                for i, line in enumerate(info_lines):
                    cv2.putText(img, line, 
                               (text_x + 20, text_y + 35 + i * line_height),
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)
            
            return img
            
        except Exception as e:
            self.get_logger().error(f"❌ Drawing error: {e}")
            return img

    def destroy_node(self):
        """Clean shutdown - 100% MIRIP SIMPLE"""
        self.processing_active = False
        time.sleep(1.0)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = CameraRearUltimateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down CAMERA REAR...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()