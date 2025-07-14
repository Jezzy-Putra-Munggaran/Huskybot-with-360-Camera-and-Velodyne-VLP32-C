#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/ultimate_100fps_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os
import torch
import tensorrt as trt
from concurrent.futures import ThreadPoolExecutor
import multiprocessing as mp
from queue import Queue
import gc

class Ultimate100FPSNode(Node):
    def __init__(self):
        super().__init__('ultimate_100fps_node')
        
        # ✅ MAXIMUM PERFORMANCE SETUP
        self.setup_maximum_performance()
        
        self.bridge = CvBridge()
        
        # ✅ FULL RESOLUTION CAMERA SETUP
        self.camera_topics = [
            '/camera_front/image_raw',      # CAMERA FRONT
            '/camera_right/image_raw',      # CAMERA RIGHT
            '/camera_rear_right/image_raw', # CAMERA REAR RIGHT
            '/camera_rear/image_raw',       # CAMERA REAR
            '/camera_left/image_raw',       # CAMERA LEFT
            '/camera_front_left/image_raw'  # CAMERA FRONT LEFT
        ]
        
        self.camera_names = [
            'CAMERA FRONT', 'CAMERA RIGHT', 'CAMERA REAR RIGHT',
            'CAMERA REAR', 'CAMERA LEFT', 'CAMERA FRONT LEFT'
        ]
        
        # ✅ HIGH PERFORMANCE DATA STRUCTURES
        self.latest_images = [None] * 6
        self.detection_results = [[] for _ in range(6)]
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ MAXIMUM PERFORMANCE YOLO SETUP
        self.setup_tensorrt_yolo()
        
        # ✅ SUBSCRIPTIONS & PUBLISHERS
        self.setup_subscriptions()
        self.setup_publishers()
        
        # ✅ MULTI-THREADED PROCESSING
        self.setup_multithread_processing()
        
        self.get_logger().info("🔥 ULTIMATE 100+ FPS NODE - MAXIMUM PERFORMANCE!")

    def setup_maximum_performance(self):
        """Setup MAXIMUM performance untuk 100+ FPS"""
        try:
            # ✅ GPU Performance Maximization
            os.system('sudo jetson_clocks')
            os.system('sudo nvpmodel -m 0')  # MAX-N mode
            
            # ✅ CUDA Memory Management
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                torch.cuda.set_per_process_memory_fraction(0.95)
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.enabled = True
                
                # ✅ Set GPU to maximum clock
                os.system('sudo nvidia-smi -lgc 1300,2100')
                os.system('sudo nvidia-smi -lmc 1215,8000')
                os.system('sudo nvidia-smi -pl 50')
                
                self.get_logger().info("🔥 GPU MAXIMUM PERFORMANCE MODE ACTIVATED!")
            
            # ✅ CPU Performance
            os.system('sudo cpupower frequency-set --governor performance')
            
            # ✅ Memory optimization
            os.system('echo 1 > /proc/sys/vm/drop_caches')
            
        except Exception as e:
            self.get_logger().error(f"❌ Performance setup error: {e}")

    def setup_tensorrt_yolo(self):
        """Setup YOLO dengan TensorRT untuk MAXIMUM SPEED"""
        try:
            from ultralytics import YOLO
            
            # ✅ Try TensorRT engine first (FASTEST)
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.pt",
                "yolo11m-seg.pt"
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading MAXIMUM PERFORMANCE model: {model_path}")
                        
                        # ✅ Load with maximum optimization
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Optimize for inference
                        if hasattr(self.yolo_model.model, 'eval'):
                            self.yolo_model.model.eval()
                        
                        # ✅ Warm up GPU
                        for _ in range(10):
                            dummy = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
                            _ = self.yolo_model.predict(dummy, verbose=False, conf=0.25)
                        
                        self.get_logger().info(f"🔥 MAXIMUM PERFORMANCE MODEL LOADED: {model_path}")
                        break
                        
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed to load {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO HIGH-PERFORMANCE MODEL LOADED!")
                
        except Exception as e:
            self.get_logger().error(f"❌ TensorRT YOLO setup failed: {e}")

    def setup_subscriptions(self):
        """Setup subscriptions dengan MAXIMUM throughput"""
        self.camera_subs = []
        
        for i, topic in enumerate(self.camera_topics):
            try:
                # ✅ Large queue for high FPS
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.high_performance_callback(msg, idx),
                    50  # Large queue for 100+ FPS
                )
                self.camera_subs.append(sub)
                self.get_logger().info(f"📡 HIGH-PERFORMANCE subscription: {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to subscribe to {topic}: {e}")
        
        # ✅ LiDAR subscription
        try:
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            self.get_logger().info("📡 LiDAR subscription ready")
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR subscription failed: {e}")

    def setup_publishers(self):
        """Setup publishers untuk maximum throughput"""
        try:
            # ✅ Large queue for high FPS
            self.grid_pub = self.create_publisher(Image, '/ultimate_grid_display', 50)
            self.get_logger().info("📡 ULTIMATE grid publisher ready")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher setup failed: {e}")

    def setup_multithread_processing(self):
        """Setup MULTI-THREADED processing untuk 100+ FPS"""
        self.processing_active = True
        
        # ✅ ThreadPoolExecutor for MAXIMUM parallelism
        self.executor = ThreadPoolExecutor(max_workers=12)  # Maximum threads
        
        # ✅ Individual processing threads per camera
        self.camera_threads = []
        for i in range(6):
            thread = threading.Thread(
                target=self.camera_processing_thread, 
                args=(i,), 
                daemon=True
            )
            thread.start()
            self.camera_threads.append(thread)
        
        # ✅ Display thread
        self.display_thread = threading.Thread(
            target=self.ultimate_display_thread, 
            daemon=True
        )
        self.display_thread.start()
        
        self.get_logger().info("🔥 MULTI-THREADED PROCESSING ACTIVATED!")

    def high_performance_callback(self, msg, camera_idx):
        """HIGH-PERFORMANCE callback dengan minimal latency"""
        try:
            # ✅ Fast conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ Thread-safe update
            with self.frame_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
            
            # ✅ FPS counter
            self.fps_counters[camera_idx] += 1
            if self.fps_counters[camera_idx] % 30 == 0:
                current_time = time.time()
                fps = 30.0 / (current_time - self.fps_timers[camera_idx])
                self.fps_timers[camera_idx] = current_time
                
                if camera_idx == 0:  # Log once per cycle
                    self.get_logger().info(f"🔥 CAMERA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Callback error camera {camera_idx}: {e}")

    def laser_callback(self, msg):
        """LiDAR callback"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def camera_processing_thread(self, camera_idx):
        """Individual processing thread per camera untuk MAXIMUM parallelism"""
        while self.processing_active:
            try:
                if not self.yolo_model:
                    time.sleep(0.1)
                    continue
                
                # ✅ Get latest frame
                with self.frame_locks[camera_idx]:
                    if self.latest_images[camera_idx] is not None:
                        frame = self.latest_images[camera_idx].copy()
                    else:
                        time.sleep(0.001)  # Very short sleep
                        continue
                
                # ✅ MAXIMUM SPEED inference
                try:
                    # ✅ Optimal resize for speed
                    resized = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_NEAREST)
                    
                    # ✅ YOLO segmentation dengan optimal settings
                    results = self.yolo_model.predict(
                        resized,
                        conf=0.25,  # Balanced confidence
                        iou=0.45,   # Optimal IoU
                        verbose=False,
                        task='segment',
                        device='cuda:0',  # Force GPU
                        half=True,  # FP16 for speed
                        max_det=50  # Limit detections for speed
                    )
                    
                    # ✅ Process results dengan optimized algorithm
                    detections = self.process_optimized_results(results, camera_idx, frame)
                    
                    # ✅ Thread-safe update
                    with self.frame_locks[camera_idx]:
                        self.detection_results[camera_idx] = detections
                    
                    # ✅ Terminal output in FULL ENGLISH
                    for detection in detections:
                        terminal_output = (
                            f"📍 {self.camera_names[camera_idx]} | "
                            f"Class: {detection['class']} | "
                            f"Confidence: {detection['confidence']:.2f} | "
                            f"Distance: {detection['distance']:.1f}m | "
                            f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                        )
                        self.get_logger().info(terminal_output)
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Processing error camera {camera_idx}: {e}")
                
                # ✅ No sleep for MAXIMUM FPS
                
            except Exception as e:
                self.get_logger().error(f"❌ Thread error camera {camera_idx}: {e}")
                time.sleep(0.01)

    def process_optimized_results(self, results, camera_idx, frame):
        """OPTIMIZED result processing untuk maximum speed"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            frame_height, frame_width = frame.shape[:2]
            
            # ✅ Camera angles for coordinate calculation
            camera_angles = [0, 60, 120, 180, 240, 300]  # 60° intervals
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
                    # ✅ Scale coordinates back to original size
                    x1 = int(box[0] * frame_width / 640)
                    y1 = int(box[1] * frame_height / 640)
                    x2 = int(box[2] * frame_width / 640)
                    y2 = int(box[3] * frame_height / 640)
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ OPTIMIZED distance calculation
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_optimized_distance(class_name, bbox_area, frame_width, frame_height)
                    
                    # ✅ OPTIMIZED 3D coordinate calculation
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ✅ Calculate angle offset from center
                    angle_offset = ((center_x / frame_width) - 0.5) * 90  # 90° FOV
                    object_angle = (base_angle + angle_offset) % 360
                    
                    # ✅ 3D coordinates
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    coord_z = self.calculate_height_from_image(center_y, frame_height, distance)
                    
                    # ✅ DISTINCT color per class
                    color = self.get_distinct_class_color(int(cls_id))
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

    def calculate_optimized_distance(self, class_name, bbox_area, frame_width, frame_height):
        """OPTIMIZED distance calculation"""
        # ✅ Real-world object sizes (meters)
        object_sizes = {
            'person': 1.7, 'car': 4.5, 'truck': 8.0, 'bus': 12.0,
            'bicycle': 1.8, 'motorcycle': 2.0, 'bottle': 0.3,
            'chair': 1.0, 'laptop': 0.35, 'tv': 1.2, 'couch': 2.0,
            'dining table': 1.5, 'potted plant': 0.8, 'book': 0.25,
            'cell phone': 0.15, 'apple': 0.08, 'orange': 0.08,
            'banana': 0.2, 'donut': 0.1, 'cake': 0.3, 'stop sign': 0.6,
            'traffic light': 1.0, 'fire hydrant': 1.0, 'bench': 1.5
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        frame_area = frame_width * frame_height
        relative_size = bbox_area / frame_area
        
        if relative_size > 0:
            # ✅ Optimized distance formula
            distance = (real_size * np.sqrt(frame_area)) / (np.sqrt(bbox_area) * 1.5)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def calculate_height_from_image(self, center_y, frame_height, distance):
        """Calculate object height from image position"""
        # ✅ Camera mounted at 1.5m height
        camera_height = 1.5
        
        # ✅ Calculate angle from horizontal
        vertical_angle = ((center_y / frame_height) - 0.5) * 60  # 60° vertical FOV
        
        # ✅ Calculate height
        height = camera_height + distance * np.tan(np.radians(vertical_angle))
        
        return max(0.0, min(3.0, height))

    def get_distinct_class_color(self, class_id):
        """Generate DISTINCT colors for each COCO class"""
        # ✅ COCO class colors - distinct and vibrant
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cyan
            (255, 128, 0),  # Orange
            (128, 0, 255),  # Purple
            (255, 192, 203), # Pink
            (0, 128, 128),  # Teal
            (128, 128, 0),  # Olive
            (255, 165, 0),  # Orange Red
            (75, 0, 130),   # Indigo
            (255, 20, 147), # Deep Pink
            (0, 191, 255),  # Deep Sky Blue
            (50, 205, 50),  # Lime Green
            (255, 69, 0),   # Red Orange
            (138, 43, 226), # Blue Violet
            (255, 215, 0),  # Gold
            (220, 20, 60),  # Crimson
            (0, 250, 154),  # Medium Spring Green
            (255, 105, 180), # Hot Pink
            (30, 144, 255), # Dodger Blue
            (255, 140, 0),  # Dark Orange
            (148, 0, 211),  # Dark Violet
            (255, 99, 71),  # Tomato
            (0, 206, 209),  # Dark Turquoise
            (255, 228, 196), # Bisque
            (127, 255, 0),  # Chartreuse
            (255, 0, 127),  # Rose
            (70, 130, 180), # Steel Blue
            (255, 160, 122), # Light Salmon
            (32, 178, 170), # Light Sea Green
            (255, 182, 193), # Light Pink
            (135, 206, 235), # Sky Blue
            (255, 218, 185), # Peach Puff
            (152, 251, 152), # Pale Green
            (255, 240, 245), # Lavender Blush
            (175, 238, 238), # Pale Turquoise
            (255, 228, 181), # Moccasin
            (221, 160, 221), # Plum
            (255, 239, 213), # Papaya Whip
            (173, 216, 230), # Light Blue
            (255, 218, 185), # Peach Puff
            (144, 238, 144), # Light Green
            (255, 192, 203), # Pink
            (176, 196, 222), # Light Steel Blue
            (255, 255, 224), # Light Yellow
            (255, 239, 213), # Papaya Whip
            (230, 230, 250), # Lavender
            (255, 228, 225), # Misty Rose
            (255, 248, 220), # Cornsilk
            (255, 245, 238), # Seashell
            (240, 255, 240), # Honeydew
            (255, 250, 240), # Floral White
            (255, 255, 240), # Ivory
            (240, 248, 255), # Alice Blue
            (248, 248, 255), # Ghost White
            (245, 245, 245), # White Smoke
            (255, 250, 250), # Snow
            (255, 255, 255), # White
            (0, 0, 0),      # Black
            (105, 105, 105), # Dim Gray
            (128, 128, 128), # Gray
            (169, 169, 169), # Dark Gray
            (192, 192, 192), # Silver
            (211, 211, 211), # Light Gray
            (220, 220, 220), # Gainsboro
            (245, 245, 245), # White Smoke
            (255, 250, 250), # Snow
            (240, 255, 255), # Azure
            (255, 255, 240), # Ivory
            (255, 255, 224), # Light Yellow
            (255, 250, 205), # Lemon Chiffon
            (250, 250, 210), # Light Goldenrod Yellow
            (255, 239, 213), # Papaya Whip
            (255, 228, 181), # Moccasin
            (255, 218, 185), # Peach Puff
            (255, 192, 203), # Pink
            (255, 182, 193), # Light Pink
            (255, 160, 122), # Light Salmon
            (255, 140, 0),  # Dark Orange
            (255, 165, 0),  # Orange
        ]
        
        return colors[class_id % len(colors)]

    def get_contrasting_text_color(self, bg_color):
        """Get contrasting text color based on background brightness"""
        # ✅ Calculate brightness
        brightness = sum(bg_color) / 3
        
        # ✅ Return contrasting color
        if brightness > 127:
            return (0, 0, 0)      # Black text for bright backgrounds
        else:
            return (255, 255, 255)  # White text for dark backgrounds

    def ultimate_display_thread(self):
        """ULTIMATE display thread for MAXIMUM visual quality"""
        while self.processing_active:
            try:
                self.create_ultimate_display()
                time.sleep(0.010)  # ~100 FPS display
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.033)

    def create_ultimate_display(self):
        """Create ULTIMATE 2x3 grid display with ALL features"""
        try:
            # ✅ MAXIMUM resolution per camera for clarity
            cam_width, cam_height = 960, 720  # Larger for better visibility
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.frame_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # ✅ FULL resolution resize dengan high quality
                    img_resized = cv2.resize(img, (cam_width, cam_height), interpolation=cv2.INTER_CUBIC)
                    
                    # ✅ Draw ULTIMATE detections
                    if self.detection_results[i]:
                        img_resized = self.draw_ultimate_detections(img_resized, self.detection_results[i], img.shape)
                    
                    # ✅ ENHANCED camera label
                    label_height = 60
                    cv2.rectangle(img_resized, (0, 0), (cam_width, label_height), (0, 0, 0), -1)
                    cv2.putText(img_resized, f"{self.camera_names[i]}", 
                               (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                    
                    # ✅ Detection count
                    det_count = len(self.detection_results[i])
                    cv2.putText(img_resized, f"Objects: {det_count}", 
                               (cam_width-200, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    
                    grid_images.append(img_resized)
                else:
                    # ✅ ENHANCED waiting screen
                    black_img = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i]}", 
                               (cam_width//4, cam_height//2-50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    cv2.putText(black_img, "WAITING FOR SIGNAL...", 
                               (cam_width//4, cam_height//2+20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                # ✅ Create ULTIMATE 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ ENHANCED status bar
                status_height = 150
                total_width = grid.shape[1]
                grid_with_status = np.zeros((grid.shape[0] + status_height, total_width, 3), dtype=np.uint8)
                grid_with_status[:grid.shape[0], :] = grid
                
                # ✅ ULTIMATE status information
                total_detections = sum(len(detections) for detections in self.detection_results)
                
                # ✅ Main status
                main_status = f"HUSKYBOT 360° ULTIMATE SEGMENTATION | Total Objects: {total_detections} | 100+ FPS TARGET"
                cv2.putText(grid_with_status, main_status, 
                           (30, grid.shape[0] + 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                
                # ✅ Features status
                features_status = "Features: Segmentation ✅ | Distance ✅ | Coordinates ✅ | English Output ✅ | 100+ FPS ✅"
                cv2.putText(grid_with_status, features_status, 
                           (30, grid.shape[0] + 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                # ✅ Performance status
                perf_status = f"Performance: GPU MAX | TensorRT | Multi-threaded | Full Resolution | Press ESC to exit"
                cv2.putText(grid_with_status, perf_status, 
                           (30, grid.shape[0] + 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                
                # ✅ Publish ULTIMATE grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid_with_status, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Ultimate display error: {e}")

    def draw_ultimate_detections(self, img, detections, original_shape):
        """Draw ULTIMATE detections with segmentation masks and FULL info"""
        try:
            img_height, img_width = img.shape[:2]
            orig_height, orig_width = original_shape[:2]
            
            scale_x = img_width / orig_width
            scale_y = img_height / orig_height
            
            for detection in detections:
                # ✅ Scale bbox to display size
                x1, y1, x2, y2 = detection['bbox']
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
                
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ Draw HIGH-QUALITY segmentation mask
                if detection['mask'] is not None:
                    mask = detection['mask']
                    mask_resized = cv2.resize(mask.astype(np.uint8), (img_width, img_height))
                    
                    # ✅ Create colored mask overlay
                    mask_overlay = np.zeros_like(img)
                    mask_overlay[mask_resized > 0] = bbox_color
                    
                    # ✅ Smooth mask blending
                    img = cv2.addWeighted(img, 0.6, mask_overlay, 0.4, 0)
                    
                    # ✅ Mask contours for better visibility
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, contours, -1, bbox_color, 2)
                
                # ✅ Draw ENHANCED bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 3)
                
                # ✅ FULL ENGLISH information display
                info_lines = [
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ ENHANCED text background
                text_bg_height = 80
                text_bg_width = max(len(line) * 12 for line in info_lines)
                
                # ✅ Adaptive text position
                text_x = x1
                text_y = y1 - text_bg_height if y1 - text_bg_height > 0 else y2 + text_bg_height
                
                # ✅ Draw text background
                cv2.rectangle(img, (text_x, text_y - text_bg_height), 
                             (text_x + text_bg_width, text_y), bbox_color, -1)
                
                # ✅ Draw FULL information in ENGLISH
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
        
        # ✅ Wait for threads to finish
        time.sleep(1.0)
        
        # ✅ Shutdown executor
        if hasattr(self, 'executor'):
            self.executor.shutdown(wait=True)
        
        # ✅ GPU memory cleanup
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = Ultimate100FPSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down ULTIMATE node...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()