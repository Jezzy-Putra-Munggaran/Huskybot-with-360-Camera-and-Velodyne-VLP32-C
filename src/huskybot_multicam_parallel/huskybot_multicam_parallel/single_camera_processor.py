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
        
        # ✅ Get parameters from ROS2 parameter server
        self.declare_parameter('camera_name', 'camera_rear')
        self.declare_parameter('camera_topic', '/camera_rear/image_raw')
        self.declare_parameter('camera_real_name', 'FRONT CAMERA')
        self.declare_parameter('camera_idx', 3)
        
        # ✅ Camera configuration
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
        self.yolo_model = None
        
        # ✅ ULTRA SIMPLE: Skip DeepStream, use OPTIMIZED TensorRT directly
        self.setup_ultra_optimized_yolo()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup MAXIMUM parallel processing
        self.setup_maximum_parallel_processing()
        
        self.get_logger().info(f"🚀 {self.camera_real_name} ULTRA OPTIMIZED PROCESSOR STARTED!")

    def setup_ultra_optimized_yolo(self):
        """Setup ULTRA OPTIMIZED TensorRT YOLO - NO DEEPSTREAM OVERHEAD"""
        try:
            from ultralytics import YOLO
            
            self.get_logger().info("🔥 Setting up ULTRA OPTIMIZED TensorRT YOLO...")
            
            # ✅ ULTRA OPTIMIZED model paths - TensorRT ENGINE FIRST
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot/yolo11m-seg.engine",
                "/home/kmp-orin/jezzy/huskybot/yolo11n-seg.engine",  # Smaller, faster model
                "yolo11n-seg.pt"  # Last resort
            ]
            
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading ULTRA OPTIMIZED model: {model_path}")
                        
                        # ✅ ULTRA OPTIMIZED YOLO settings
                        self.yolo_model = YOLO(model_path, task='segment')
                        
                        # ✅ MAXIMUM optimization test
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        
                        # ✅ ULTRA FAST inference settings
                        results = self.yolo_model.predict(
                            test_img, 
                            verbose=False, 
                            task='segment',
                            device=0,           # GPU
                            half=True,          # FP16
                            imgsz=640,          # Fixed size
                            conf=0.15,          # Lower confidence = faster
                            iou=0.5,            # Higher IoU = faster NMS
                            max_det=30,         # Fewer detections = faster
                            agnostic_nms=True,  # Faster NMS
                            retina_masks=False, # Faster masks
                            stream=True         # Stream processing
                        )
                        
                        self.get_logger().info(f"✅ ULTRA OPTIMIZED model loaded: {model_path}")
                        break
                        
                except Exception as e:
                    self.get_logger().warn(f"❌ Failed: {model_path}: {e}")
                    continue
            
            if not self.yolo_model:
                self.get_logger().error("❌ NO MODEL LOADED!")
                
        except Exception as e:
            self.get_logger().error(f"❌ ULTRA OPTIMIZED YOLO setup failed: {e}")
            self.yolo_model = None

    def setup_connections(self):
        """Setup connections"""
        try:
            # ✅ OPTIMIZED subscriber with larger queue
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 5)  # Smaller queue = less latency
            self.get_logger().info(f"📡 Subscribed: {self.camera_topic}")
            
            # ✅ OPTIMIZED publisher
            self.result_pub = self.create_publisher(
                Image, f'/{self.camera_name}_processed', 5)  # Smaller queue = less latency
            self.get_logger().info(f"📡 Publisher created: /{self.camera_name}_processed")
            
        except Exception as e:
            self.get_logger().error(f"❌ Connection setup failed: {e}")

    def setup_maximum_parallel_processing(self):
        """Setup MAXIMUM parallel processing threads"""
        self.processing_active = True
        
        # ✅ MAXIMUM parallel threads for ULTRA SPEED
        self.process_threads = []
        num_threads = 6  # MAXIMUM threads for parallel processing
        
        for i in range(num_threads):
            thread = threading.Thread(target=self.ultra_processing_loop, args=(i,), daemon=True)
            thread.start()
            self.process_threads.append(thread)
            
        self.get_logger().info(f"✅ {self.camera_real_name} MAXIMUM processing threads started! ({num_threads} threads)")

    def camera_callback(self, msg):
        """ULTRA FAST camera callback"""
        try:
            # ✅ ULTRA FAST conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ NON-BLOCKING frame update
            if self.frame_lock.acquire(blocking=False):
                try:
                    self.latest_image = cv_image
                finally:
                    self.frame_lock.release()
            
            # ✅ FPS tracking every 50 frames for speed
            self.fps_counter += 1
            if self.fps_counter % 50 == 0:
                current_time = time.time()
                fps = 50.0 / (current_time - self.fps_timer)
                self.fps_timer = current_time
                self.get_logger().info(f"🔥 {self.camera_real_name} ULTRA FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error: {e}")

    def ultra_processing_loop(self, thread_id):
        """ULTRA FAST processing loop"""
        while self.processing_active:
            try:
                if hasattr(self, 'yolo_model') and self.yolo_model:
                    self.process_with_ultra_yolo()
                else:
                    time.sleep(0.1)
                    continue
                
                # ✅ MINIMAL sleep for MAXIMUM throughput
                time.sleep(0.001)
                
            except Exception as e:
                self.get_logger().error(f"❌ Processing loop {thread_id} error: {e}")
                time.sleep(0.01)

    def process_with_ultra_yolo(self):
        """ULTRA FAST YOLO processing"""
        try:
            # ✅ NON-BLOCKING frame get
            if self.frame_lock.acquire(blocking=False):
                try:
                    if self.latest_image is not None:
                        frame = self.latest_image.copy()
                    else:
                        return
                finally:
                    self.frame_lock.release()
            else:
                return
            
            # ✅ Store original frame
            original_frame = frame.copy()
            
            # ✅ ULTRA FAST resize with INTER_NEAREST (fastest)
            frame_resized = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_NEAREST)
            scale_x = original_frame.shape[1] / 640
            scale_y = original_frame.shape[0] / 640
            
            # ✅ ULTRA FAST YOLO inference
            start_inference = time.time()
            results = self.yolo_model.predict(
                frame_resized,
                conf=0.15,          # Lower confidence = faster
                iou=0.5,            # Higher IoU = faster NMS
                verbose=False,
                task='segment',
                device=0,           # GPU
                half=True,          # FP16
                imgsz=640,
                max_det=30,         # Fewer detections = faster
                agnostic_nms=True,  # Faster NMS
                retina_masks=False  # Faster masks
            )
            inference_time = (time.time() - start_inference) * 1000
            
            # ✅ ULTRA FAST result processing
            detections = self.ultra_process_results(results, self.camera_idx, original_frame, scale_x, scale_y)
            
            # ✅ Terminal output in FULL ENGLISH
            for detection in detections:
                terminal_output = (
                    f"Camera: {self.camera_real_name} | "
                    f"Class: {detection['class']} | "
                    f"Confidence: {detection['confidence']:.2f} | "
                    f"Distance: {detection['distance']:.1f}m | "
                    f"Coordinate: ({detection['x']:.1f}, {detection['y']:.1f}, {detection['z']:.1f}) | "
                    f"Inference: {inference_time:.1f}ms"
                )
                self.get_logger().info(terminal_output)
            
            # ✅ Create and publish processed image
            processed_img = self.create_ultra_processed_image(original_frame, detections)
            if processed_img is not None:
                self.publish_processed_frame(processed_img)
            
        except Exception as e:
            self.get_logger().error(f"❌ ULTRA processing error: {e}")

    def ultra_process_results(self, results, camera_idx, original_frame, scale_x, scale_y):
        """ULTRA FAST result processing"""
        detections = []
        
        try:
            if not results or len(results) == 0:
                return detections
            
            result = results[0]
            original_height, original_width = original_frame.shape[:2]
            
            # ✅ Camera angles - REAL MAPPING CORRECTED
            camera_angles = [180, 240, 300, 0, 60, 120]  # Rear, LeftRear, LeftFront, Front, RightFront, RightRear
            base_angle = camera_angles[camera_idx]
            
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                names = result.names if hasattr(result, 'names') else {}
                
                # ✅ Process masks (if available)
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    # ✅ ULTRA FAST coordinate scaling
                    x1 = int(box[0] * scale_x)
                    y1 = int(box[1] * scale_y)
                    x2 = int(box[2] * scale_x)
                    y2 = int(box[3] * scale_y)
                    
                    # ✅ Ensure coordinates are within frame
                    x1 = max(0, min(original_width, x1))
                    y1 = max(0, min(original_height, y1))
                    x2 = max(0, min(original_width, x2))
                    y2 = max(0, min(original_height, y2))
                    
                    class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                    
                    # ✅ ULTRA FAST distance calculation
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.ultra_calculate_distance(class_name, bbox_area)
                    
                    # ✅ ULTRA FAST 3D coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # ✅ MAXIMUM FOV calculation (120° horizontal FOV)
                    angle_offset = ((center_x / original_width) - 0.5) * 120
                    object_angle = (base_angle + angle_offset) % 360
                    
                    coord_x = distance * np.cos(np.radians(object_angle))
                    coord_y = distance * np.sin(np.radians(object_angle))
                    
                    # ✅ MAXIMUM Vertical FOV (90° vertical FOV)
                    vertical_angle = ((center_y / original_height) - 0.5) * 90
                    coord_z = 1.5 + distance * np.tan(np.radians(vertical_angle))
                    coord_z = max(0.0, min(3.0, coord_z))
                    
                    # ✅ ULTRA FAST colors
                    color = self.get_ultra_coco_color(int(cls_id))
                    text_color = self.get_ultra_contrasting_text_color(color)
                    
                    # ✅ ULTRA FAST mask processing
                    processed_mask = None
                    if masks is not None and i < len(masks):
                        try:
                            mask = masks[i]
                            # ✅ ULTRA FAST mask resize with INTER_NEAREST
                            mask_resized = cv2.resize(
                                mask.astype(np.float32), 
                                (original_width, original_height), 
                                interpolation=cv2.INTER_NEAREST
                            )
                            processed_mask = (mask_resized > 0.5).astype(np.uint8)
                        except:
                            processed_mask = None
                    
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

    def ultra_calculate_distance(self, class_name, bbox_area):
        """ULTRA FAST distance calculation"""
        # ✅ Simplified object sizes for speed
        object_sizes = {
            'person': 1.7, 'car': 4.5, 'bicycle': 1.8, 'motorcycle': 2.0,
            'chair': 1.0, 'laptop': 0.35, 'bottle': 0.3, 'cup': 0.12,
            'keyboard': 0.4, 'mouse': 0.1, 'book': 0.25, 'cell phone': 0.15
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        
        # ✅ ULTRA FAST calculation
        if bbox_area > 0:
            distance = (real_size * 500) / np.sqrt(bbox_area)  # Simplified focal length
            return max(0.3, min(50.0, distance))
        else:
            return 5.0

    def get_ultra_coco_color(self, class_id):
        """ULTRA FAST distinct colors"""
        # ✅ Pre-computed colors for speed
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (255, 128, 0), (128, 0, 255), (255, 192, 203), (0, 128, 128),
            (128, 128, 0), (255, 165, 0), (75, 0, 130), (255, 20, 147), (0, 191, 255)
        ]
        return colors[class_id % len(colors)]

    def get_ultra_contrasting_text_color(self, bg_color):
        """ULTRA FAST contrasting text color"""
        brightness = (bg_color[0] + bg_color[1] + bg_color[2]) / 3  # Simplified brightness
        return (0, 0, 0) if brightness > 127 else (255, 255, 255)

    def create_ultra_processed_image(self, original_frame, detections):
        """ULTRA FAST processed image creation"""
        try:
            canvas = original_frame.copy()
            
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                bbox_color = detection['color']
                text_color = detection['text_color']
                
                # ✅ ULTRA FAST mask drawing
                if detection['mask'] is not None:
                    try:
                        mask = detection['mask']
                        # ✅ SIMPLIFIED mask overlay for speed
                        mask_colored = np.zeros_like(canvas, dtype=np.uint8)
                        mask_colored[mask == 1] = bbox_color
                        
                        # ✅ FAST alpha blending
                        mask_indices = mask == 1
                        if np.any(mask_indices):
                            canvas[mask_indices] = (canvas[mask_indices] * 0.4 + mask_colored[mask_indices] * 0.6).astype(np.uint8)
                    except:
                        pass  # Skip mask on error for speed
                
                # ✅ ULTRA FAST bounding box
                cv2.rectangle(canvas, (x1, y1), (x2, y2), bbox_color, 4)
                cv2.rectangle(canvas, (x1-1, y1-1), (x2+1, y2+1), (255, 255, 255), 1)
                
                # ✅ ULTRA FAST text - SIMPLIFIED
                info_text = f"{detection['class']}: {detection['confidence']:.2f} | {detection['distance']:.1f}m"
                
                # ✅ FAST text positioning
                text_x = x1
                text_y = y1 - 10 if y1 > 30 else y2 + 25
                
                # ✅ SIMPLIFIED text background
                (text_width, text_height), _ = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(canvas, (text_x-2, text_y-text_height-2), (text_x+text_width+2, text_y+2), (0, 0, 0), -1)
                
                # ✅ FAST text drawing
                cv2.putText(canvas, info_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            
            return canvas
            
        except Exception as e:
            self.get_logger().error(f"❌ Image creation error: {e}")
            return None

    def publish_processed_frame(self, processed_img):
        """ULTRA FAST publish"""
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_img, 'bgr8')
            processed_msg.header.stamp = self.get_clock().now().to_msg()
            self.result_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"❌ Publish error: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        time.sleep(0.2)
        super().destroy_node()

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = SingleCameraProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down ULTRA OPTIMIZED camera processor...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()