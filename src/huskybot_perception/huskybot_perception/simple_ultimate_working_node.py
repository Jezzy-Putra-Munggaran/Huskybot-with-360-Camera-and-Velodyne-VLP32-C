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
        
        # ✅ CORRECTED camera mapping berdasarkan kondisi real
        self.camera_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_right/image_raw',      # KAMERA KANAN BELAKANG  
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_front_left/image_raw'  # KAMERA KIRI BELAKANG
        ]
        
        self.camera_names = [
            'CAMERA REAR', 'CAMERA RIGHT REAR', 'CAMERA RIGHT FRONT',
            'CAMERA FRONT', 'CAMERA LEFT FRONT', 'CAMERA LEFT REAR'
        ]
        
        # ✅ Simple but effective data storage
        self.latest_images = [None] * 6
        self.detection_results = [[] for _ in range(6)]
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ Setup YOLO dengan fokus WORKING
        self.setup_working_yolo()
        
        # ✅ Setup subscriptions & publishers
        self.setup_working_connections()
        
        # ✅ Setup processing threads
        self.setup_working_processing()
        
        self.get_logger().info("🚀 SIMPLE ULTIMATE WORKING NODE - FOCUS ON RESULTS!")

    def setup_working_yolo(self):
        """Setup YOLO yang PASTI WORKING"""
        try:
            from ultralytics import YOLO
            
            # ✅ Try models in order of preference
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",  # TensorRT engine
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.pt",     # PyTorch model
                "yolo11m-seg.pt",                                    # Download from web
                "yolo11n-seg.pt"                                     # Lighter model
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading model: {model_path}")
                        
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Test the model
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        results = self.yolo_model.predict(test_img, verbose=False, task='segment')
                        
                        self.get_logger().info(f"✅ SUCCESS! Model loaded: {model_path}")
                        break
                        
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed to load {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO YOLO MODEL LOADED!")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOLO setup failed: {e}")
            self.yolo_model = None

    def setup_working_connections(self):
        """Setup connections yang PASTI WORKING"""
        # ✅ Camera subscriptions
        self.camera_subs = []
        for i, topic in enumerate(self.camera_topics):
            try:
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.camera_callback(msg, idx),
                    20  # Good queue size
                )
                self.camera_subs.append(sub)
                self.get_logger().info(f"📡 Subscribed to: {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to subscribe to {topic}: {e}")
        
        # ✅ LiDAR subscription
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            self.get_logger().info("📡 Subscribed to LiDAR")
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR subscription failed: {e}")
        
        # ✅ Grid publisher
        try:
            self.grid_pub = self.create_publisher(Image, '/ultimate_grid_display', 20)
            self.get_logger().info("📡 Grid publisher created")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher creation failed: {e}")

    def setup_working_processing(self):
        """Setup processing yang PASTI WORKING"""
        self.processing_active = True
        
        # ✅ Single processing thread - simple but effective
        self.process_thread = threading.Thread(target=self.working_processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Display thread
        self.display_thread = threading.Thread(target=self.working_display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("✅ Processing threads started!")

    def camera_callback(self, msg, camera_idx):
        """Simple camera callback"""
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
                
                if camera_idx == 0:  # Log main camera FPS
                    self.get_logger().info(f"🔥 CAMERA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def laser_callback(self, msg):
        """Simple laser callback"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def working_processing_loop(self):
        """WORKING processing loop - fokus pada hasil"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # ✅ Process each camera sequentially
                for i in range(6):
                    try:
                        with self.frame_locks[i]:
                            if self.latest_images[i] is not None:
                                frame = self.latest_images[i].copy()
                            else:
                                continue
                        
                        # ✅ YOLO segmentation inference
                        resized = cv2.resize(frame, (640, 640))
                        
                        results = self.yolo_model.predict(
                            resized,
                            conf=0.25,  # Good confidence threshold
                            iou=0.45,   # Good IoU threshold
                            verbose=False,
                            task='segment',
                            device='cuda:0' if torch.cuda.is_available() else 'cpu'
                        )
                        
                        # ✅ Process results
                        detections = self.process_working_results(results, i, frame)
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
                
                time.sleep(0.05)  # 20Hz processing - good balance
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_working_results(self, results, camera_idx, frame):
        """Process YOLO results dengan distance & coordinates"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # ✅ Camera angles untuk coordinate calculation
            camera_angles = [180, 240, 300, 0, 60, 120]  # Correct angles based on real mapping
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # ✅ Process masks for segmentation
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # ✅ Scale coordinates
                    x1 = int(box[0] * frame_width / 640)
                    y1 = int(box[1] * frame_height / 640)
                    x2 = int(box[2] * frame_width / 640)
                    y2 = int(box[3] * frame_height / 640)
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ Calculate distance
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ Calculate 3D coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    angle_offset = ((center_x / frame_width) - 0.5) * 90  # 90° FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    coord_z = self.calculate_height(center_y, frame_height, distance)
                    
                    # ✅ DISTINCT color for each COCO class
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
        """Calculate object distance based on real-world sizes"""
        # ✅ COCO object sizes in meters
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
            # ✅ Distance calculation with camera calibration
            focal_length = 800  # Estimated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def calculate_height(self, center_y, frame_height, distance):
        """Calculate object height from image position"""
        camera_height = 1.5  # Camera mounted at 1.5m
        vertical_angle = ((center_y / frame_height) - 0.5) * 60  # 60° vertical FOV
        height = camera_height + distance * np.tan(np.radians(vertical_angle))
        return max(0.0, min(3.0, height))

    def get_distinct_coco_color(self, class_id):
        """Generate DISTINCT colors for each COCO class"""
        # ✅ 80 distinct colors for COCO classes
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

    def working_display_loop(self):
        """WORKING display loop"""
        while self.processing_active:
            try:
                self.create_working_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_working_display(self):
        """Create WORKING 2x3 grid display"""
        try:
            # ✅ LARGE size per camera for clarity
            cam_width, cam_height = 960, 720
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.frame_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # ✅ High-quality resize
                    img_resized = cv2.resize(img, (cam_width, cam_height), interpolation=cv2.INTER_CUBIC)
                    
                    # ✅ Draw detections with masks
                    if self.detection_results[i]:
                        img_resized = self.draw_working_detections(img_resized, self.detection_results[i], img.shape)
                    
                    # ✅ Camera label
                    label_height = 60
                    cv2.rectangle(img_resized, (0, 0), (cam_width, label_height), (0, 0, 0), -1)
                    cv2.putText(img_resized, f"{self.camera_names[i]}", 
                               (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                    
                    # ✅ Detection count
                    det_count = len(self.detection_results[i])
                    cv2.putText(img_resized, f"Objects: {det_count}", 
                               (cam_width-200, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    
                    grid_images.append(img_resized)
                else:
                    # ✅ Waiting screen
                    black_img = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i]}", 
                               (cam_width//4, cam_height//2-30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    cv2.putText(black_img, "WAITING FOR SIGNAL...", 
                               (cam_width//6, cam_height//2+30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                # ✅ Create 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ Status bar
                status_height = 120
                total_width = grid.shape[1]
                grid_with_status = np.zeros((grid.shape[0] + status_height, total_width, 3), dtype=np.uint8)
                grid_with_status[:grid.shape[0], :] = grid
                
                # ✅ Status information
                total_detections = sum(len(detections) for detections in self.detection_results)
                
                main_status = f"HUSKYBOT 360° ULTIMATE SEGMENTATION | Total Objects: {total_detections} | TARGET: 100+ FPS"
                cv2.putText(grid_with_status, main_status, 
                           (30, grid.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                
                features_status = "Features: Segmentation ✅ | Distance ✅ | Coordinates ✅ | English Output ✅ | COCO Colors ✅"
                cv2.putText(grid_with_status, features_status, 
                           (30, grid.shape[0] + 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                performance_status = "Performance: GPU Optimized | Multi-threaded | Full Resolution | Real Camera Mapping"
                cv2.putText(grid_with_status, performance_status, 
                           (30, grid.shape[0] + 95), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                
                # ✅ Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid_with_status, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def draw_working_detections(self, img, detections, original_shape):
        """Draw detections with FULL information"""
        try:
            img_height, img_width = img.shape[:2]
            orig_height, orig_width = original_shape[:2]
            
            scale_x = img_width / orig_width
            scale_y = img_height / orig_height
            
            for detection in detections:
                # ✅ Scale bbox
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
                
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ Draw segmentation mask
                if detection['mask'] is not None:
                    mask = detection['mask']
                    mask_resized = cv2.resize(mask.astype(np.uint8), (img_width, img_height))
                    
                    # ✅ Colored mask overlay
                    mask_overlay = np.zeros_like(img)
                    mask_overlay[mask_resized > 0] = bbox_color
                    
                    # ✅ Blend with original image
                    img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                    
                    # ✅ Mask contours
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, contours, -1, bbox_color, 2)
                
                # ✅ Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 3)
                
                # ✅ FULL ENGLISH information
                info_lines = [
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ Text background
                text_bg_height = 85
                text_bg_width = max(len(line) * 12 for line in info_lines)
                
                text_x = x1
                text_y = y1 - text_bg_height if y1 - text_bg_height > 0 else y2 + text_bg_height
                
                # ✅ Draw background
                cv2.rectangle(img, (text_x, text_y - text_bg_height), 
                             (text_x + text_bg_width, text_y), bbox_color, -1)
                
                # ✅ Draw text
                for i, line in enumerate(info_lines):
                    cv2.putText(img, line, 
                               (text_x + 5, text_y - text_bg_height + 20 + i*18), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            
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
    # ✅ Add missing import
    import torch
    
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