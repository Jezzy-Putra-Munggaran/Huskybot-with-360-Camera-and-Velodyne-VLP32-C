#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/simple_ultimate_working_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os

class SimpleUltimateWorkingNode(Node):
    def __init__(self):
        super().__init__('simple_ultimate_working_node')
        
        self.bridge = CvBridge()
        
        # ✅ CORRECTED camera mapping berdasarkan kondisi REAL
        self.camera_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG (Real)
            '/camera_right/image_raw',      # KAMERA KANAN BELAKANG (Real)
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN (Real)
            '/camera_rear/image_raw',       # KAMERA DEPAN (Real)
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN (Real)
            '/camera_front_left/image_raw'  # KAMERA KIRI BELAKANG (Real)
        ]
        
        self.camera_names = [
            'CAMERA REAR', 'CAMERA RIGHT REAR', 'CAMERA RIGHT FRONT',
            'CAMERA FRONT', 'CAMERA LEFT FRONT', 'CAMERA LEFT REAR'
        ]
        
        # ✅ SIMPLE data storage
        self.latest_images = [None] * 6
        self.detection_results = [[] for _ in range(6)]
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ Setup YOLO SIMPLE
        self.setup_simple_yolo()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup processing
        self.setup_processing()
        
        self.get_logger().info("🚀 SIMPLE ULTIMATE WORKING NODE - FOCUSED ON RESULTS!")

    def setup_simple_yolo(self):
        """Setup YOLO SIMPLE yang PASTI WORKING"""
        try:
            from ultralytics import YOLO
            
            # ✅ Try models berdasarkan priority
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",  # TensorRT FASTEST
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
        """Setup connections SIMPLE"""
        # ✅ Camera subscriptions
        self.camera_subs = []
        for i, topic in enumerate(self.camera_topics):
            try:
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.camera_callback(msg, idx),
                    30  # Good queue size
                )
                self.camera_subs.append(sub)
                self.get_logger().info(f"📡 Subscribed: {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed subscribe {topic}: {e}")
        
        # ✅ LiDAR subscription
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            self.get_logger().info("📡 Subscribed to LiDAR")
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR subscription failed: {e}")
        
        # ✅ Publisher
        try:
            self.grid_pub = self.create_publisher(Image, '/simple_grid_display', 30)
            self.get_logger().info("📡 Grid publisher created")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher creation failed: {e}")

    def setup_processing(self):
        """Setup processing SIMPLE"""
        self.processing_active = True
        
        # ✅ Single processing thread
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("✅ Processing threads started!")

    def camera_callback(self, msg, camera_idx):
        """SIMPLE camera callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image.copy()
            
            # ✅ FPS tracking
            self.fps_counters[camera_idx] += 1
            if self.fps_counters[camera_idx] % 50 == 0:
                current_time = time.time()
                fps = 50.0 / (current_time - self.fps_timers[camera_idx])
                self.fps_timers[camera_idx] = current_time
                
                if camera_idx == 0:
                    self.get_logger().info(f"🔥 CAMERA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def laser_callback(self, msg):
        """SIMPLE laser callback"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def processing_loop(self):
        """SIMPLE processing loop"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # ✅ Process each camera
                for i in range(6):
                    try:
                        with self.frame_locks[i]:
                            if self.latest_images[i] is not None:
                                frame = self.latest_images[i].copy()
                            else:
                                continue
                        
                        # ✅ YOLO segmentation inference
                        # Use FULL resolution for better FOV
                        height, width = frame.shape[:2]
                        
                        # ✅ Maintain aspect ratio but resize for inference
                        scale = min(640/width, 640/height)
                        new_width = int(width * scale)
                        new_height = int(height * scale)
                        
                        resized = cv2.resize(frame, (new_width, new_height))
                        
                        # ✅ Pad to 640x640 to maintain full FOV
                        padded = np.zeros((640, 640, 3), dtype=np.uint8)
                        y_offset = (640 - new_height) // 2
                        x_offset = (640 - new_width) // 2
                        padded[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized
                        
                        results = self.yolo_model.predict(
                            padded,
                            conf=0.25,
                            iou=0.45,
                            verbose=False,
                            task='segment'
                        )
                        
                        # ✅ Process results
                        detections = self.process_results(results, i, frame, scale, x_offset, y_offset)
                        self.detection_results[i] = detections
                        
                        # ✅ TERMINAL OUTPUT in FULL ENGLISH
                        for detection in detections:
                            terminal_output = (
                                f"📍 {self.camera_names[i]} | "
                                f"Class: {detection['class']} | "
                                f"Confidence: {detection['confidence']:.2f} | "
                                f"Distance: {detection['distance']:.1f}m | "
                                f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                            )
                            self.get_logger().info(terminal_output)
                        
                    except Exception as e:
                        self.get_logger().error(f"❌ Processing error camera {i}: {e}")
                
                time.sleep(0.03)  # ~30Hz processing
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_results(self, results, camera_idx, frame, scale, x_offset, y_offset):
        """Process YOLO results dengan FULL FOV"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # ✅ Camera angles untuk coordinate calculation (REAL mapping)
            camera_angles = [180, 240, 300, 0, 60, 120]  # Sesuai kondisi real
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # ✅ Process masks
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # ✅ Convert coordinates back to original frame
                    x1 = int((box[0] - x_offset) / scale)
                    y1 = int((box[1] - y_offset) / scale)
                    x2 = int((box[2] - x_offset) / scale)
                    y2 = int((box[3] - y_offset) / scale)
                    
                    # ✅ Ensure coordinates are within frame
                    x1 = max(0, min(frame_width, x1))
                    y1 = max(0, min(frame_height, y1))
                    x2 = max(0, min(frame_width, x2))
                    y2 = max(0, min(frame_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ Calculate distance
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ Calculate 3D coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ✅ MAXIMUM FOV calculation (120° horizontal FOV untuk Arducam IMX477)
                    angle_offset = ((center_x / frame_width) - 0.5) * 120  # 120° horizontal FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # ✅ Vertical angle calculation (90° vertical FOV)
                    vertical_angle = ((center_y / frame_height) - 0.5) * 90
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))  # Camera height 1.5m
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # ✅ DISTINCT COCO colors
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
        """Calculate distance based on COCO object real-world sizes"""
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
            # ✅ Distance calculation optimized for Arducam IMX477
            focal_length = 900  # Calibrated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_distinct_coco_color(self, class_id):
        """Generate 80 DISTINCT colors for COCO classes"""
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
        """Get contrasting text color"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def display_loop(self):
        """SIMPLE display loop"""
        while self.processing_active:
            try:
                self.create_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_display(self):
        """Create 2x3 grid display dengan MAXIMUM clarity"""
        try:
            # ✅ LARGE size per camera untuk clarity
            cam_width, cam_height = 1200, 900  # Large for clear visibility
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.frame_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # ✅ HIGH-QUALITY resize maintaining aspect ratio
                    height, width = img.shape[:2]
                    scale = min(cam_width/width, cam_height/height)
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    
                    img_resized = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
                    
                    # ✅ Create canvas and center image
                    canvas = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    y_offset = (cam_height - new_height) // 2
                    x_offset = (cam_width - new_width) // 2
                    canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = img_resized
                    
                    # ✅ Draw detections
                    if self.detection_results[i]:
                        canvas = self.draw_detections(canvas, self.detection_results[i], img.shape, scale, x_offset, y_offset)
                    
                    # ✅ Camera label
                    label_height = 60
                    cv2.rectangle(canvas, (0, 0), (cam_width, label_height), (0, 0, 0), -1)
                    cv2.putText(canvas, f"{self.camera_names[i]}", 
                               (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                    
                    # ✅ Detection count
                    det_count = len(self.detection_results[i])
                    cv2.putText(canvas, f"Objects: {det_count}", 
                               (cam_width-250, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    
                    grid_images.append(canvas)
                else:
                    # ✅ Waiting screen
                    black_img = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i]}", 
                               (cam_width//3, cam_height//2-30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    cv2.putText(black_img, "WAITING FOR SIGNAL...", 
                               (cam_width//4, cam_height//2+30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                # ✅ Create 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ SIMPLE status bar (removed unnecessary text)
                status_height = 80
                total_width = grid.shape[1]
                grid_with_status = np.zeros((grid.shape[0] + status_height, total_width, 3), dtype=np.uint8)
                grid_with_status[:grid.shape[0], :] = grid
                
                # ✅ SIMPLE status information
                total_detections = sum(len(detections) for detections in self.detection_results)
                
                main_status = f"HUSKYBOT 360 ULTIMATE SEGMENTATION | Total Objects: {total_detections}"
                cv2.putText(grid_with_status, main_status, 
                           (50, grid.shape[0] + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                
                # ✅ Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid_with_status, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def draw_detections(self, img, detections, original_shape, scale, x_offset, y_offset):
        """Draw detections dengan PERFECT text positioning dan LARGER FONT"""
        try:
            for detection in detections:
                # ✅ Scale bbox to display coordinates
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale) + x_offset
                y1 = int(y1 * scale) + y_offset
                x2 = int(x2 * scale) + x_offset
                y2 = int(y2 * scale) + y_offset
                
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ Draw segmentation mask
                if detection['mask'] is not None:
                    mask = detection['mask']
                    # Scale mask to match display
                    mask_height, mask_width = mask.shape
                    mask_scale = min(img.shape[1]/mask_width, img.shape[0]/mask_height)
                    new_mask_width = int(mask_width * mask_scale)
                    new_mask_height = int(mask_height * mask_scale)
                    
                    mask_resized = cv2.resize(mask.astype(np.uint8), (new_mask_width, new_mask_height))
                    
                    # Apply mask at correct position
                    mask_y_offset = (img.shape[0] - new_mask_height) // 2
                    mask_x_offset = (img.shape[1] - new_mask_width) // 2
                    
                    # Create mask overlay
                    mask_overlay = np.zeros_like(img)
                    mask_overlay[mask_y_offset:mask_y_offset+new_mask_height, 
                                mask_x_offset:mask_x_offset+new_mask_width][mask_resized > 0] = bbox_color
                    
                    # Blend with original image
                    img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                
                # ✅ Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 4)  # Thicker bounding box
                
                # ✅ PERFECT text positioning dengan LARGER FONT
                info_lines = [
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ 🔥 LARGER FONT SETTINGS
                font_scale = 0.9  # ✅ INCREASED from 0.6 to 0.9
                font_thickness = 3  # ✅ INCREASED from 2 to 3
                line_height = 35  # ✅ INCREASED from 25 to 35
                
                text_bg_height = len(info_lines) * line_height + 20  # ✅ INCREASED padding
                text_bg_width = max(len(line) * 18 for line in info_lines) + 30  # ✅ INCREASED width
                
                # ✅ PERFECT positioning logic
                if y1 - text_bg_height > 15:  # Space above
                    text_x = x1
                    text_y = y1 - text_bg_height
                    text_draw_y = y1 - 15
                else:  # Space below
                    text_x = x1
                    text_y = y2
                    text_draw_y = y2 + text_bg_height - 15
                
                # ✅ Ensure text stays within image bounds
                text_x = max(0, min(img.shape[1] - text_bg_width, text_x))
                text_y = max(0, min(img.shape[0] - text_bg_height, text_y))
                
                # ✅ Draw text background
                cv2.rectangle(img, (text_x, text_y), 
                             (text_x + text_bg_width, text_y + text_bg_height), 
                             bbox_color, -1)
                
                # ✅ Draw text lines dengan LARGER FONT
                for i, line in enumerate(info_lines):
                    cv2.putText(img, line, 
                               (text_x + 15, text_y + 25 + i * line_height),  # ✅ INCREASED padding
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)
            
            return img
            
        except Exception as e:
            self.get_logger().error(f"❌ Drawing error: {e}")
            return img

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = SimpleUltimateWorkingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()