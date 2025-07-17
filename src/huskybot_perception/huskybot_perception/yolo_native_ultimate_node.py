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
from concurrent.futures import ThreadPoolExecutor
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
        
        # ✅ HIGH-PERFORMANCE data storage
        self.latest_images = [None] * 6
        self.frame_locks = [threading.Lock() for _ in range(6)]
        self.fps_counters = [0] * 6
        self.fps_timers = [time.time()] * 6
        
        # ✅ Performance optimization
        self.max_workers = min(8, multiprocessing.cpu_count())
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers)
        
        # ✅ Setup YOLO NATIVE
        self.setup_yolo_native()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup NATIVE processing
        self.setup_native_processing()
        
        self.get_logger().info("🚀 YOLO NATIVE ULTIMATE NODE - 100+ FPS TARGET!")

    def setup_yolo_native(self):
        """Setup YOLO dengan NATIVE display features"""
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
                        self.get_logger().info(f"🔄 Loading NATIVE model: {model_path}")
                        
                        self.yolo_model = YOLO(model_path)
                        
                        # ✅ Configure NATIVE display settings
                        self.yolo_model.overrides['show'] = True  # Enable native display
                        self.yolo_model.overrides['save'] = False  # Disable saving
                        self.yolo_model.overrides['verbose'] = False  # Reduce verbosity
                        
                        # ✅ Test model
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        results = self.yolo_model.predict(test_img, verbose=False, task='segment', show=False)
                        
                        self.get_logger().info(f"✅ SUCCESS! NATIVE Model loaded: {model_path}")
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
        """Setup connections dengan HIGH PERFORMANCE"""
        # ✅ Camera subscriptions dengan optimal QoS
        self.camera_subs = []
        for i, topic in enumerate(self.camera_topics):
            try:
                sub = self.create_subscription(
                    Image, topic,
                    lambda msg, idx=i: self.camera_callback(msg, idx),
                    50  # Higher queue size untuk performance
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

    def setup_native_processing(self):
        """Setup NATIVE processing dengan MAXIMUM performance"""
        self.processing_active = True
        
        # ✅ NATIVE display threads - one per camera untuk maximum parallelism
        self.display_threads = []
        for i in range(6):
            thread = threading.Thread(
                target=self.native_camera_processor,
                args=(i,),
                daemon=True
            )
            thread.start()
            self.display_threads.append(thread)
        
        self.get_logger().info(f"✅ Started {len(self.display_threads)} NATIVE display threads!")

    def camera_callback(self, msg, camera_idx):
        """HIGH-PERFORMANCE camera callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.frame_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image.copy()
            
            # ✅ FPS tracking
            self.fps_counters[camera_idx] += 1
            if self.fps_counters[camera_idx] % 100 == 0:  # Check every 100 frames
                current_time = time.time()
                fps = 100.0 / (current_time - self.fps_timers[camera_idx])
                self.fps_timers[camera_idx] = current_time
                
                if camera_idx == 0:
                    self.get_logger().info(f"🔥 NATIVE CAMERA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def laser_callback(self, msg):
        """SIMPLE laser callback"""
        try:
            self.latest_laser = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def native_camera_processor(self, camera_idx):
        """NATIVE processing per camera dengan YOLO built-in display"""
        window_name = f"HUSKYBOT {self.camera_names[camera_idx]}"
        
        # ✅ Create NATIVE window dengan optimal settings
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)  # Optimal size untuk performance
        
        # ✅ Position windows dalam grid layout
        col = camera_idx % 3
        row = camera_idx // 3
        cv2.moveWindow(window_name, col * 650, row * 500)
        
        frame_count = 0
        fps_start = time.time()
        
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
                
                # ✅ YOLO NATIVE inference dengan built-in features
                results = self.yolo_model.predict(
                    frame,
                    conf=0.25,
                    iou=0.45,
                    verbose=False,
                    task='segment',
                    show=False,  # We'll handle display manually for custom info
                    save=False,
                    device=0  # Use GPU
                )
                
                # ✅ Use YOLO's NATIVE .plot() method
                if results and len(results) > 0:
                    # ✅ Get annotated image dengan NATIVE YOLO plotting
                    annotated_frame = results[0].plot(
                        conf=True,          # Show confidence
                        labels=True,        # Show labels
                        boxes=True,         # Show bounding boxes
                        masks=True,         # Show segmentation masks
                        probs=False         # Hide probabilities
                    )
                    
                    # ✅ Add custom information untuk distance dan coordinates
                    annotated_frame = self.add_custom_info(
                        annotated_frame, results[0], camera_idx
                    )
                    
                    # ✅ Add camera label
                    cv2.rectangle(annotated_frame, (0, 0), (annotated_frame.shape[1], 40), (0, 0, 0), -1)
                    cv2.putText(annotated_frame, f"{self.camera_names[camera_idx]}", 
                               (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    # ✅ Add FPS info
                    frame_count += 1
                    if frame_count % 30 == 0:
                        fps = 30.0 / (time.time() - fps_start)
                        fps_start = time.time()
                    
                    if frame_count > 30:
                        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", 
                                   (annotated_frame.shape[1]-120, 25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # ✅ Display dengan NATIVE OpenCV
                    cv2.imshow(window_name, annotated_frame)
                    
                    # ✅ TERMINAL output in FULL ENGLISH
                    self.print_detections(results[0], camera_idx)
                    
                else:
                    # ✅ Show original frame jika no detections
                    cv2.imshow(window_name, frame)
                
                # ✅ MINIMAL delay untuk maximum FPS
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
                time.sleep(0.001)  # Very minimal sleep untuk maximum performance
                
            except Exception as e:
                self.get_logger().error(f"❌ Native processing error camera {camera_idx}: {e}")
                time.sleep(0.01)

    def add_custom_info(self, img, result, camera_idx):
        """Add custom distance dan coordinate information"""
        try:
            if not hasattr(result, 'boxes') or result.boxes is None:
                return img
            
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            names = result.names if hasattr(result, 'names') else {}
            
            # ✅ Camera angles untuk coordinate calculation (REAL mapping)
            camera_angles = [180, 240, 300, 0, 60, 120]  # Sesuai kondisi real
            base_angle = camera_angles[camera_idx]
            
            frame_height, frame_width = img.shape[:2]
            
            for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                x1, y1, x2, y2 = map(int, box)
                class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                
                # ✅ Calculate distance
                bbox_area = (x2 - x1) * (y2 - y1)
                distance = self.calculate_distance(class_name, bbox_area, frame_width, frame_height)
                
                # ✅ Calculate 3D coordinates
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # ✅ MAXIMUM FOV calculation (120° horizontal FOV untuk Arducam IMX477)
                angle_offset = ((center_x / frame_width) - 0.5) * 120
                object_angle = (base_angle + angle_offset) % 360
                
                coord_x = distance * np.cos(np.radians(object_angle))
                coord_y = distance * np.sin(np.radians(object_angle))
                
                # ✅ Vertical angle calculation (90° vertical FOV)
                vertical_angle = ((center_y / frame_height) - 0.5) * 90
                coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                coord_z = max(0.0, min(3.0, coord_z))
                
                # ✅ Add distance dan coordinates info
                info_text = f"Dist:{distance:.1f}m Pos:({coord_x:.1f},{coord_y:.1f},{coord_z:.1f})"
                
                # ✅ Position text di bawah bounding box
                text_y = min(y2 + 20, frame_height - 10)
                
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
            focal_length = 900  # Calibrated for Arducam IMX477
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def print_detections(self, result, camera_idx):
        """Print detections to terminal in FULL ENGLISH"""
        try:
            if not hasattr(result, 'boxes') or result.boxes is None:
                return
            
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            names = result.names if hasattr(result, 'names') else {}
            
            # ✅ Camera angles untuk coordinate calculation
            camera_angles = [180, 240, 300, 0, 60, 120]
            base_angle = camera_angles[camera_idx]
            
            for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                x1, y1, x2, y2 = map(int, box)
                
                # Calculate distance dan coordinates
                bbox_area = (x2 - x1) * (y2 - y1)
                distance = self.calculate_distance(class_name, bbox_area, 640, 480)
                
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                angle_offset = ((center_x / 640) - 0.5) * 120
                object_angle = (base_angle + angle_offset) % 360
                
                coord_x = distance * np.cos(np.radians(object_angle))
                coord_y = distance * np.sin(np.radians(object_angle))
                coord_z = 1.5 + distance * np.tan(np.radians(((center_y / 480) - 0.5) * 90))
                coord_z = max(0.0, min(3.0, coord_z))
                
                # ✅ TERMINAL OUTPUT in FULL ENGLISH
                terminal_output = (
                    f"📍 {self.camera_names[camera_idx]} | "
                    f"Class: {class_name} | "
                    f"Confidence: {score:.2f} | "
                    f"Distance: {distance:.1f}m | "
                    f"Coordinate: ({coord_x:.1f}, {coord_y:.1f}, {coord_z:.1f})"
                )
                self.get_logger().info(terminal_output)
                
        except Exception as e:
            self.get_logger().error(f"❌ Print detection error: {e}")

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
        print("🛑 Shutting down NATIVE YOLO...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()