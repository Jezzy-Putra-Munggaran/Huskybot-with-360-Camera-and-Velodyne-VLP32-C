#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/huskybot_perception/ultimate_100fps_node_fixed.py

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
# Remove tensorrt import since it's causing issues
from concurrent.futures import ThreadPoolExecutor
import multiprocessing as mp
from queue import Queue
import gc

class Ultimate100FPSNodeFixed(Node):
    def __init__(self):
        super().__init__('ultimate_100fps_node_fixed')
        
        # ✅ MAXIMUM PERFORMANCE SETUP
        self.setup_maximum_performance()
        
        self.bridge = CvBridge()
        
        # ✅ CORRECT CAMERA MAPPING berdasarkan kondisi real
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
        
        # ✅ HIGH PERFORMANCE DATA STRUCTURES
        self.latest_images = [None] * 6
        self.detection_results = [[] for _ in range(6)]
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ MAXIMUM PERFORMANCE YOLO SETUP
        self.setup_optimized_yolo()
        
        # ✅ SUBSCRIPTIONS & PUBLISHERS
        self.setup_subscriptions()
        self.setup_publishers()
        
        # ✅ MULTI-THREADED PROCESSING
        self.setup_multithread_processing()
        
        self.get_logger().info("🔥 ULTIMATE 100+ FPS NODE FIXED - MAXIMUM PERFORMANCE!")

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

    def setup_optimized_yolo(self):
        """Setup YOLO dengan optimizations untuk MAXIMUM SPEED"""
        try:
            from ultralytics import YOLO
            
            # ✅ Try different model paths dengan priority pada speed
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",  # TensorRT engine FASTEST
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.pt",     # PyTorch model
                "yolo11m-seg.pt",                                    # Download if needed
                "yolo11n-seg.pt"                                     # Nano for speed
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading OPTIMIZED model: {model_path}")
                        
                        # ✅ Load with maximum optimization
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Optimize for inference
                        if hasattr(self.yolo_model.model, 'eval'):
                            self.yolo_model.model.eval()
                        
                        # ✅ Warm up GPU dengan multiple iterations
                        self.get_logger().info("🔥 Warming up GPU...")
                        for i in range(20):
                            dummy = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
                            _ = self.yolo_model.predict(dummy, verbose=False, conf=0.25, task='segment')
                            if i % 5 == 0:
                                self.get_logger().info(f"Warmup: {i+1}/20")
                        
                        self.get_logger().info(f"🔥 OPTIMIZED MODEL LOADED: {model_path}")
                        break
                        
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed to load {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO OPTIMIZED MODEL LOADED!")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOLO setup failed: {e}")

    def setup_subscriptions(self):
        """Setup subscriptions dengan MAXIMUM throughput"""
        self.camera_subs = []
        
        for i, topic in enumerate(self.camera_topics):
            try:
                # ✅ Large queue for high FPS
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.high_performance_callback(msg, idx),
                    100  # Extra large queue for 100+ FPS
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
            self.grid_pub = self.create_publisher(Image, '/ultimate_grid_display', 100)
            self.get_logger().info("📡 ULTIMATE grid publisher ready")
        except Exception as e:
            self.get_logger().error(f"❌ Publisher setup failed: {e}")

    def setup_multithread_processing(self):
        """Setup MULTI-THREADED processing untuk 100+ FPS"""
        self.processing_active = True
        
        # ✅ ThreadPoolExecutor for MAXIMUM parallelism
        self.executor = ThreadPoolExecutor(max_workers=16)  # Maximum threads
        
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
            
            # ✅ Camera angles untuk CORRECT mapping
            camera_angles = [180, 300, 60, 0, 120, 240]  # Correct angles based on real mapping
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
                    
                    # ✅ DISTINCT color per COCO class
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

    def calculate_optimized_distance(self, class_name, bbox_area, frame_width, frame_height):
        """OPTIMIZED distance calculation with real-world measurements"""
        # ✅ Real-world object sizes (meters) - COCO dataset based
        object_sizes = {
            'person': 1.7, 'bicycle': 1.8, 'car': 4.5, 'motorcycle': 2.0, 'airplane': 30.0,
            'bus': 12.0, 'train': 50.0, 'truck': 8.0, 'boat': 6.0, 'traffic light': 1.0,
            'fire hydrant': 1.0, 'stop sign': 0.6, 'parking meter': 1.5, 'bench': 1.5,
            'bird': 0.3, 'cat': 0.5, 'dog': 0.6, 'horse': 2.0, 'sheep': 1.0, 'cow': 2.5,
            'elephant': 3.0, 'bear': 1.5, 'zebra': 2.0, 'giraffe': 4.0, 'backpack': 0.5,
            'umbrella': 1.0, 'handbag': 0.3, 'tie': 0.15, 'suitcase': 0.6, 'frisbee': 0.25,
            'skis': 1.7, 'snowboard': 1.5, 'sports ball': 0.22, 'kite': 1.0, 'baseball bat': 1.0,
            'baseball glove': 0.3, 'skateboard': 0.8, 'surfboard': 2.0, 'tennis racket': 0.7,
            'bottle': 0.3, 'wine glass': 0.2, 'cup': 0.12, 'fork': 0.2, 'knife': 0.25,
            'spoon': 0.18, 'bowl': 0.25, 'banana': 0.2, 'apple': 0.08, 'sandwich': 0.15,
            'orange': 0.08, 'broccoli': 0.15, 'carrot': 0.2, 'hot dog': 0.15, 'pizza': 0.3,
            'donut': 0.1, 'cake': 0.3, 'chair': 1.0, 'couch': 2.0, 'potted plant': 0.8,
            'bed': 2.0, 'dining table': 1.5, 'toilet': 0.7, 'tv': 1.2, 'laptop': 0.35,
            'mouse': 0.1, 'remote': 0.15, 'keyboard': 0.45, 'cell phone': 0.15, 'microwave': 0.5,
            'oven': 0.6, 'toaster': 0.3, 'sink': 0.6, 'refrigerator': 1.8, 'book': 0.25,
            'clock': 0.3, 'vase': 0.3, 'scissors': 0.2, 'teddy bear': 0.4, 'hair drier': 0.25,
            'toothbrush': 0.2
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        frame_area = frame_width * frame_height
        relative_size = bbox_area / frame_area
        
        if relative_size > 0:
            # ✅ Optimized distance formula with camera calibration
            focal_length = 800  # Estimated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
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

    def get_distinct_coco_color(self, class_id):
        """Generate DISTINCT colors for each COCO class - 80 unique colors"""
        # ✅ 80 DISTINCT colors for COCO classes
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
        """Get contrasting text color based on background brightness"""
        # ✅ Calculate brightness using luminance formula
        brightness = (0.299 * bg_color[0] + 0.587 * bg_color[1] + 0.114 * bg_color[2])
        
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
                time.sleep(0.005)  # ~200 FPS display rate
            except Exception as e:
                self.get_logger().error(f"❌ Display error: {e}")
                time.sleep(0.033)

    def create_ultimate_display(self):
        """Create ULTIMATE 2x3 grid display with ALL features"""
        try:
            # ✅ LARGE resolution per camera for maximum clarity
            cam_width, cam_height = 1280, 960  # Large for clear visibility
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.frame_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # ✅ HIGH-QUALITY resize
                    img_resized = cv2.resize(img, (cam_width, cam_height), interpolation=cv2.INTER_CUBIC)
                    
                    # ✅ Draw ULTIMATE detections
                    if self.detection_results[i]:
                        img_resized = self.draw_ultimate_detections(img_resized, self.detection_results[i], img.shape)
                    
                    # ✅ ENHANCED camera label dengan correct names
                    label_height = 80
                    cv2.rectangle(img_resized, (0, 0), (cam_width, label_height), (0, 0, 0), -1)
                    cv2.putText(img_resized, f"{self.camera_names[i]}", 
                               (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 4)
                    
                    # ✅ Detection count
                    det_count = len(self.detection_results[i])
                    cv2.putText(img_resized, f"Objects: {det_count}", 
                               (cam_width-300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                    
                    grid_images.append(img_resized)
                else:
                    # ✅ ENHANCED waiting screen
                    black_img = np.zeros((cam_height, cam_width, 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i]}", 
                               (cam_width//4, cam_height//2-50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
                    cv2.putText(black_img, "WAITING FOR SIGNAL...", 
                               (cam_width//4, cam_height//2+50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                # ✅ Create ULTIMATE 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ ENHANCED status bar
                status_height = 200
                total_width = grid.shape[1]
                grid_with_status = np.zeros((grid.shape[0] + status_height, total_width, 3), dtype=np.uint8)
                grid_with_status[:grid.shape[0], :] = grid
                
                # ✅ ULTIMATE status information
                total_detections = sum(len(detections) for detections in self.detection_results)
                
                # ✅ Main status
                main_status = f"HUSKYBOT 360° ULTIMATE SEGMENTATION | Total Objects: {total_detections} | TARGET: 100+ FPS"
                cv2.putText(grid_with_status, main_status, 
                           (50, grid.shape[0] + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4)
                
                # ✅ Features status
                features_status = "Features: Segmentation ✅ | Distance ✅ | Coordinates ✅ | English Output ✅ | Distinct Colors ✅"
                cv2.putText(grid_with_status, features_status, 
                           (50, grid.shape[0] + 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                
                # ✅ Performance status
                perf_status = f"Performance: GPU MAX | Multi-threaded | Full Resolution | Real Camera Mapping"
                cv2.putText(grid_with_status, perf_status, 
                           (50, grid.shape[0] + 150), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 3)
                
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
                    img = cv2.addWeighted(img, 0.7, mask_overlay, 0.3, 0)
                    
                    # ✅ Mask contours for better visibility
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, contours, -1, bbox_color, 3)
                
                # ✅ Draw ENHANCED bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 4)
                
                # ✅ FULL ENGLISH information display
                info_lines = [
                    f"Class: {detection['class']}",
                    f"Confidence: {detection['confidence']:.2f}",
                    f"Distance: {detection['distance']:.1f}m",
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f})"
                ]
                
                # ✅ ENHANCED text background
                text_bg_height = 100
                text_bg_width = max(len(line) * 15 for line in info_lines)
                
                # ✅ Adaptive text position
                text_x = x1
                text_y = y1 - text_bg_height if y1 - text_bg_height > 0 else y2 + text_bg_height
                
                # ✅ Draw text background
                cv2.rectangle(img, (text_x, text_y - text_bg_height), 
                             (text_x + text_bg_width, text_y), bbox_color, -1)
                
                # ✅ Draw FULL information in ENGLISH
                for i, line in enumerate(info_lines):
                    cv2.putText(img, line, 
                               (text_x + 10, text_y - text_bg_height + 25 + i*22), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            
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
        node = Ultimate100FPSNodeFixed()
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