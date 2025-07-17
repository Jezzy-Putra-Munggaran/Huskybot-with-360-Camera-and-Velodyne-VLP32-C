#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/yolo_native_ultimate_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os
import multiprocessing

class YoloNativeUltimateNode(Node):
    def __init__(self):
        super().__init__('yolo_native_ultimate_node')
        
        self.bridge = CvBridge()
        
        # ✅ CORRECTED camera mapping
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
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ Setup YOLO SIMPLE
        self.setup_yolo_simple()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup SIMPLE processing
        self.setup_simple_processing()
        
        self.get_logger().info("🚀 YOLO NATIVE ULTIMATE NODE - SIMPLE + WORKING!")

    def setup_yolo_simple(self):
        """Setup YOLO SIMPLE yang GUARANTEED WORKING"""
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
                        self.get_logger().info(f"🔄 Loading SIMPLE model: {model_path}")
                        
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

    def setup_simple_processing(self):
        """Setup SIMPLE processing"""
        self.processing_active = True
        
        # ✅ Single thread untuk semua kamera (SIMPLE approach)
        self.process_thread = threading.Thread(target=self.simple_processing_loop, daemon=True)
        self.process_thread.start()
        
        # ✅ Single display thread
        self.display_thread = threading.Thread(target=self.simple_display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("✅ SIMPLE processing threads started!")

    def camera_callback(self, msg, camera_idx):
        """SIMPLE camera callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image.copy()
            
            # ✅ FPS tracking
            self.fps_counters[camera_idx] += 1
            if self.fps_counters[camera_idx] % 100 == 0:
                current_time = time.time()
                fps = 100.0 / (current_time - self.fps_timers[camera_idx])
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

    def simple_processing_loop(self):
        """SIMPLE processing loop - process all cameras sequentially"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(1.0)
                    continue
                
                # ✅ Process each camera sequentially (SIMPLE)
                for i in range(6):
                    try:
                        with self.frame_locks[i]:
                            if self.latest_images[i] is not None:
                                frame = self.latest_images[i].copy()
                            else:
                                continue
                        
                        # ✅ SIMPLE YOLO inference
                        results = self.yolo_model.predict(
                            frame,
                            conf=0.25,
                            iou=0.45,
                            verbose=False,
                            task='segment',
                            device=0  # Use GPU
                        )
                        
                        # ✅ Process results dengan SIMPLE approach
                        if results and len(results) > 0:
                            detections = self.process_simple_results(results[0], i, frame)
                            
                            # ✅ TERMINAL output in FULL ENGLISH
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
                
                time.sleep(0.01)  # Small delay untuk stability
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop error: {e}")
                time.sleep(0.5)

    def process_simple_results(self, result, camera_idx, frame):
        """Process results dengan SIMPLE approach"""
        detections = []
        
        try:
            if not hasattr(result, 'boxes') or result.boxes is None:
                return detections
            
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            names = result.names if hasattr(result, 'names') else {}
            
            # ✅ Camera angles untuk coordinate calculation
            camera_angles = [180, 240, 300, 0, 60, 120]
            base_angle = camera_angles[camera_idx]
            
            frame_height, frame_width = frame.shape[:2]
            
            for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                x1, y1, x2, y2 = map(int, box)
                class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                
                # ✅ Calculate distance
                bbox_area = (x2 - x1) * (y2 - y1)
                distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                
                # ✅ Calculate 3D coordinates
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # ✅ MAXIMUM FOV calculation
                angle_offset = ((center_x / frame_width) - 0.5) * 120  # 120° horizontal FOV
                object_angle = (base_angle + angle_offset) % 360
                
                coord_x = distance * np.cos(np.radians(object_angle))
                coord_y = distance * np.sin(np.radians(object_angle))
                
                # ✅ Vertical calculation
                vertical_angle = ((center_y / frame_height) - 0.5) * 90
                coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                coord_z = max(0.0, min(3.0, coord_z))
                
                # ✅ DISTINCT colors untuk COCO classes
                color = self.get_coco_color(int(cls_id))
                text_color = self.get_text_color(color)
                
                detection = {
                    'class': class_name,
                    'confidence': float(score),
                    'bbox': (x1, y1, x2, y2),
                    'distance': distance,
                    'x': coord_x,
                    'y': coord_y,
                    'z': coord_z,
                    'color': color,
                    'text_color': text_color,
                    'camera_idx': camera_idx
                }
                
                detections.append(detection)
                
        except Exception as e:
            self.get_logger().error(f"❌ Result processing error: {e}")
        
        return detections

    def calculate_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Calculate distance based on COCO object sizes"""
        object_sizes = {
            'person': 1.7, 'bicycle': 1.8, 'car': 4.5, 'motorcycle': 2.0,
            'bus': 12.0, 'truck': 8.0, 'boat': 6.0, 'traffic light': 1.0,
            'chair': 1.0, 'bottle': 0.3, 'cup': 0.12, 'laptop': 0.35
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

    def get_coco_color(self, class_id):
        """Get distinct color for COCO class"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 0, 255), (255, 192, 203), (0, 128, 128),
            (128, 128, 0), (255, 165, 0), (75, 0, 130), (255, 20, 147), (0, 191, 255),
            (50, 205, 50), (255, 69, 0), (138, 43, 226), (255, 215, 0), (220, 20, 60)
        ]
        return colors[class_id % len(colors)]

    def get_text_color(self, bg_color):
        """Get contrasting text color"""
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def simple_display_loop(self):
        """SIMPLE display loop - create 6 windows"""
        # ✅ Create 6 windows
        for i in range(6):
            window_name = f"HUSKYBOT {self.camera_names[i]}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 640, 480)
            
            # ✅ Position windows dalam grid
            col = i % 3
            row = i // 3
            cv2.moveWindow(window_name, col * 650, row * 500)
        
        self.get_logger().info("✅ 6 NATIVE WINDOWS CREATED!")
        
        frame_count = 0
        fps_start = time.time()
        
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(0.1)
                    continue
                
                # ✅ Display all 6 cameras
                for i in range(6):
                    window_name = f"HUSKYBOT {self.camera_names[i]}"
                    
                    with self.frame_locks[i]:
                        if self.latest_images[i] is not None:
                            frame = self.latest_images[i].copy()
                        else:
                            # ✅ Show waiting screen
                            frame = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(frame, "WAITING...", (250, 240), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                    
                    # ✅ Quick YOLO inference untuk display
                    if self.latest_images[i] is not None:
                        try:
                            results = self.yolo_model.predict(frame, verbose=False, task='segment')
                            if results and len(results) > 0:
                                # ✅ Use YOLO native plot
                                annotated = results[0].plot(
                                    conf=True, labels=True, boxes=True, masks=True
                                )
                                
                                # ✅ Add custom info
                                annotated = self.add_custom_info(annotated, results[0], i)
                                frame = annotated
                                
                        except Exception as e:
                            self.get_logger().error(f"❌ Display inference error {i}: {e}")
                    
                    # ✅ Add camera info
                    cv2.rectangle(frame, (0, 0), (640, 30), (0, 0, 0), -1)
                    cv2.putText(frame, f"{self.camera_names[i]}", 
                               (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # ✅ Show frame
                    cv2.imshow(window_name, frame)
                
                # ✅ FPS calculation
                frame_count += 1
                if frame_count % 30 == 0:
                    fps = 30.0 / (time.time() - fps_start)
                    fps_start = time.time()
                    self.get_logger().info(f"🔥 DISPLAY FPS: {fps:.1f}")
                
                # ✅ Check for exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
                time.sleep(0.033)  # ~30 FPS display
                
            except Exception as e:
                self.get_logger().error(f"❌ Display loop error: {e}")
                time.sleep(0.1)

    def add_custom_info(self, img, result, camera_idx):
        """Add custom distance dan coordinate info"""
        try:
            if not hasattr(result, 'boxes') or result.boxes is None:
                return img
            
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            names = result.names if hasattr(result, 'names') else {}
            
            camera_angles = [180, 240, 300, 0, 60, 120]
            base_angle = camera_angles[camera_idx]
            
            frame_height, frame_width = img.shape[:2]
            
            for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                x1, y1, x2, y2 = map(int, box)
                class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                
                # ✅ Calculate distance dan coordinates
                bbox_area = (x2 - x1) * (y2 - y1)
                distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                angle_offset = ((center_x / frame_width) - 0.5) * 120
                object_angle = (base_angle + angle_offset) % 360
                
                coord_x = distance * np.cos(np.radians(object_angle))
                coord_y = distance * np.sin(np.radians(object_angle))
                coord_z = 1.5 + distance * np.tan(np.radians(((center_y / frame_height) - 0.5) * 90))
                coord_z = max(0.0, min(3.0, coord_z))
                
                # ✅ Add info text
                info_text = f"Dist:{distance:.1f}m Pos:({coord_x:.1f},{coord_y:.1f},{coord_z:.1f})"
                
                # ✅ Position text
                text_y = max(y1 - 10, 15)
                
                # ✅ Text background
                (text_w, text_h), _ = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(img, (x1, text_y - text_h - 5), (x1 + text_w + 5, text_y + 5), (0, 0, 0), -1)
                
                # ✅ Draw text
                cv2.putText(img, info_text, (x1 + 2, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            return img
            
        except Exception as e:
            self.get_logger().error(f"❌ Custom info error: {e}")
            return img

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(1.0)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = YoloNativeUltimateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down YOLO NATIVE...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()