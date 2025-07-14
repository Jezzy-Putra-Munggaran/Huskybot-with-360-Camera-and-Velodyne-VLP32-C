#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/simple_working_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os

class SimpleWorkingNode(Node):
    def __init__(self):
        super().__init__('simple_working_node')
        
        self.bridge = CvBridge()
        
        # ✅ SIMPLE camera setup - NO complex optimization
        self.camera_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_right/image_raw',      # KAMERA KANAN BELAKANG  
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_front_left/image_raw'  # KAMERA KIRI BELAKANG
        ]
        
        self.camera_names = [
            'KAMERA BELAKANG', 'KAMERA KANAN BELAKANG', 'KAMERA KANAN DEPAN',
            'KAMERA DEPAN', 'KAMERA KIRI DEPAN', 'KAMERA KIRI BELAKANG'
        ]
        
        # ✅ Simple data storage
        self.latest_images = [None] * 6
        self.latest_laser = None
        self.detection_results = [[] for _ in range(6)]
        self.frame_locks = [threading.Lock() for _ in range(6)]
        
        # ✅ Setup YOLO - SIMPLE approach
        self.setup_simple_yolo()
        
        # ✅ Simple subscriptions
        self.setup_simple_subscriptions()
        
        # ✅ Simple publishers
        self.setup_simple_publishers()
        
        # ✅ Simple processing
        self.setup_simple_processing()
        
        self.get_logger().info("🚀 SIMPLE Working Node - TARGET: SEMUA FITUR BEKERJA!")

    def setup_simple_yolo(self):
        """Setup YOLO dengan cara SIMPLE yang pasti work"""
        try:
            from ultralytics import YOLO
            
            # ✅ Coba beberapa path model
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.pt", 
                "yolo11x-seg.pt",
                "yolo11n-seg.pt"  # Fallback ke model kecil
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Trying model: {model_path}")
                        self.yolo_model = YOLO(model_path)
                        
                        # Simple test
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        results = self.yolo_model.predict(test_img, verbose=False)
                        
                        self.get_logger().info(f"✅ Model loaded successfully: {model_path}")
                        break
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed to load {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO YOLO MODEL LOADED! Please check model files")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOLO setup failed: {e}")
            self.yolo_model = None

    def setup_simple_subscriptions(self):
        """Simple subscriptions yang pasti work"""
        self.camera_subs = []
        
        for i, topic in enumerate(self.camera_topics):
            try:
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.simple_image_callback(msg, idx),
                    10  # Larger queue for stability
                )
                self.camera_subs.append(sub)
                self.get_logger().info(f"📡 Subscribed to: {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to subscribe to {topic}: {e}")
        
        # ✅ LiDAR subscription
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.simple_laser_callback, 10)
            self.get_logger().info("📡 Subscribed to LiDAR /scan")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to subscribe to LiDAR: {e}")

    def setup_simple_publishers(self):
        """Simple publishers"""
        try:
            self.grid_pub = self.create_publisher(Image, '/simple_grid_display', 10)
            self.get_logger().info("📡 Grid publisher created")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to create publishers: {e}")

    def setup_simple_processing(self):
        """Simple processing thread"""
        self.processing_active = True
        
        # ✅ Simple processing thread
        self.process_thread = threading.Thread(target=self.simple_processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Simple display thread
        self.display_thread = threading.Thread(target=self.simple_display_loop, daemon=True)
        self.display_thread.start()

    def simple_image_callback(self, msg, camera_idx):
        """SIMPLE image callback yang pasti work"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image.copy()
            
            # Log sukses sekali saja
            if camera_idx == 0:  # Only log for first camera to avoid spam
                self.get_logger().info(f"✅ Received frame from camera {camera_idx}", once=True)
                
        except Exception as e:
            self.get_logger().error(f"❌ Image callback error {camera_idx}: {e}")

    def simple_laser_callback(self, msg):
        """Simple laser callback"""
        try:
            self.latest_laser = msg
            self.get_logger().info("✅ Received LiDAR data", once=True)
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def simple_processing_loop(self):
        """SIMPLE processing loop untuk segmentation"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # Process each camera
                for i in range(6):
                    with self.frame_locks[i]:
                        if self.latest_images[i] is not None:
                            frame = self.latest_images[i].copy()
                        else:
                            continue
                    
                    # ✅ SIMPLE YOLO inference
                    try:
                        # Resize untuk inference
                        resized = cv2.resize(frame, (640, 640))
                        
                        # YOLO segmentation
                        results = self.yolo_model.predict(
                            resized, 
                            conf=0.3,  # Higher confidence for better results
                            verbose=False,
                            task='segment'
                        )
                        
                        # Process results
                        detections = self.process_simple_results(results, i, frame)
                        self.detection_results[i] = detections
                        
                        # Print detection results to terminal
                        for detection in detections:
                            self.get_logger().info(
                                f"📍 {self.camera_names[i]} | "
                                f"Class: {detection['class']} | "
                                f"Confidence: {detection['conf']:.2f} | "
                                f"Distance: {detection['distance']:.1f}m | "
                                f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                            )
                        
                    except Exception as e:
                        self.get_logger().error(f"❌ YOLO inference error camera {i}: {e}")
                
                time.sleep(0.1)  # 10Hz processing
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(1.0)

    def process_simple_results(self, results, camera_idx, frame):
        """Process YOLO results dengan distance dan coordinates"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # Camera angles untuk coordinate calculation
            camera_angles = [180, 225, 315, 0, 45, 135]  # sesuai urutan camera_topics
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # Process masks if available
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # Scale coordinates back
                    x1 = int(box[0] * frame_width / 640)
                    y1 = int(box[1] * frame_height / 640)
                    x2 = int(box[2] * frame_width / 640)
                    y2 = int(box[3] * frame_height / 640)
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ Calculate distance (simple method)
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_simple_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ Calculate 3D coordinates
                    center_x = (x1 + x2) / 2
                    angle_offset = ((center_x / frame_width) - 0.5) * 60  # 60° FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    coord_z = max(0.2, min(2.0, (y2-y1) / frame_height * distance * 0.5))
                    
                    # ✅ Generate color for this class
                    color = self.get_class_color(int(cls_id))
                    
                    detection = {
                        'class': class_name,
                        'conf': float(score),
                        'bbox': (x1, y1, x2, y2),
                        'distance': distance,
                        'x': coord_x,
                        'y': coord_y, 
                        'z': coord_z,
                        'angle': object_angle,
                        'color': color,
                        'mask': masks[i] if masks is not None and i < len(masks) else None
                    }
                    
                    detections.append(detection)
                    
        except Exception as e:
            self.get_logger().error(f"❌ Result processing error: {e}")
        
        return detections

    def calculate_simple_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Simple distance calculation"""
        # Object sizes in meters
        object_sizes = {
            'person': 1.7, 'car': 4.5, 'bicycle': 1.8, 'motorcycle': 2.0,
            'bus': 12.0, 'truck': 8.0, 'bottle': 0.3, 'chair': 1.0,
            'laptop': 0.35, 'tv': 1.2, 'couch': 2.0, 'dining table': 1.5
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        frame_area = frame_width * frame_height
        relative_size = bbox_area / frame_area
        
        if relative_size > 0:
            distance = real_size / (relative_size ** 0.5) * 2.5
            return max(0.5, min(30.0, distance))
        else:
            return 5.0

    def get_class_color(self, class_id):
        """Generate distinct color for each class"""
        # Simple color generation
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 255, 0), (0, 128, 255), (255, 0, 128),
            (128, 0, 255), (0, 255, 128), (192, 192, 192), (128, 128, 128), (255, 192, 203)
        ]
        return colors[class_id % len(colors)]

    def simple_display_loop(self):
        """SIMPLE display loop untuk grid 2x3"""
        while self.processing_active:
            try:
                self.create_simple_grid_display()
                time.sleep(0.033)  # ~30 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.1)

    def create_simple_grid_display(self):
        """Create SIMPLE 2x3 grid dengan segmentation results"""
        try:
            # Target size per camera
            cam_width, cam_height = 640, 480
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.frame_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # Resize
                    img_resized = cv2.resize(img, (cam_width, cam_height))
                    
                    # Draw detections
                    if self.detection_results[i]:
                        img_resized = self.draw_simple_detections(img_resized, self.detection_results[i], img.shape)
                    
                    # Camera label
                    cv2.rectangle(img_resized, (0, 0), (cam_width, 50), (0, 0, 0), -1)
                    cv2.putText(img_resized, f"{self.camera_names[i]}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    grid_images.append(img_resized)
                else:
                    # No image placeholder
                    black_img = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i]}", 
                               (cam_width//4, cam_height//2-20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(black_img, "WAITING...", 
                               (cam_width//3, cam_height//2+20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                # Create 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # Add status info
                status_height = 100
                grid_with_status = np.zeros((grid.shape[0] + status_height, grid.shape[1], 3), dtype=np.uint8)
                grid_with_status[:grid.shape[0], :] = grid
                
                # Status text
                total_detections = sum(len(detections) for detections in self.detection_results)
                status_text = f"HUSKYBOT 360° SEGMENTATION | Total Objects: {total_detections} | SIMPLE WORKING VERSION"
                cv2.putText(grid_with_status, status_text, 
                           (20, grid.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                feature_text = "Features: Segmentation ✅ | Distance ✅ | Coordinates ✅ | Terminal Output ✅"
                cv2.putText(grid_with_status, feature_text, 
                           (20, grid.shape[0] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                # Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid_with_status, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def draw_simple_detections(self, img, detections, original_shape):
        """Draw detections dengan segmentation masks"""
        try:
            img_height, img_width = img.shape[:2]
            orig_height, orig_width = original_shape[:2]
            
            scale_x = img_width / orig_width
            scale_y = img_height / orig_height
            
            for detection in detections:
                # Scale bbox
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y) 
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
                
                color = detection['color']
                
                # Draw mask if available
                if detection['mask'] is not None:
                    mask = detection['mask']
                    mask_resized = cv2.resize(mask.astype(np.uint8), (img_width, img_height))
                    
                    # Apply colored mask
                    mask_overlay = np.zeros_like(img)
                    mask_overlay[mask_resized > 0] = color
                    img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                
                # Draw bbox
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                
                # Draw info text
                info_text = f"{detection['class']}: {detection['conf']:.2f}"
                distance_text = f"{detection['distance']:.1f}m"
                coord_text = f"({detection['x']:.1f},{detection['y']:.1f},{detection['z']:.1f})"
                
                # Background for text
                text_bg_height = 60
                cv2.rectangle(img, (x1, y1-text_bg_height), (x2, y1), color, -1)
                
                # Text color (contrasting)
                text_color = (0, 0, 0) if sum(color) > 400 else (255, 255, 255)
                
                cv2.putText(img, info_text, (x1+5, y1-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
                cv2.putText(img, distance_text, (x1+5, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
                cv2.putText(img, coord_text, (x1+5, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
            
            return img
            
        except Exception as e:
            self.get_logger().error(f"❌ Drawing error: {e}")
            return img

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(0.5)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = SimpleWorkingNode()
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