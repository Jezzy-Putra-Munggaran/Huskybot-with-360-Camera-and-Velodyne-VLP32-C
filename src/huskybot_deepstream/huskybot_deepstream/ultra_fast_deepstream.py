#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os

class UltraFastDeepStreamNode(Node):
    def __init__(self):
        super().__init__('ultra_fast_deepstream')
        
        self.bridge = CvBridge()
        
        # ULTRA-AGGRESSIVE parameters for 100+ FPS
        self.input_size = 320  # Very small for max speed
        self.skip_frames = 3   # Process every 3rd frame
        self.max_det = 10      # Reasonable detections
        self.conf_thres = 0.4  # Balanced confidence
        self.camera_count = 6
        
        # Setup components
        self.setup_model()
        self.setup_topics()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.latest_images = [None] * 6
        self.frame_skip_counters = [0] * 6
        
        # Timers
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        
        self.get_logger().info("🚀 ULTRA-FAST DeepStream for 100+ FPS initialized!")

    def setup_model(self):
        """Setup ultra-optimized model"""
        try:
            from ultralytics import YOLO
            
            # Try to find model
            model_candidates = [
                'yolo11n-seg.engine',
                'yolo11n.pt',
                '/home/kmp-orin/yolo11n.pt'
            ]
            
            model_path = None
            for candidate in model_candidates:
                if os.path.exists(candidate):
                    model_path = candidate
                    break
            
            if not model_path:
                self.get_logger().warn("❌ No model found, using default")
                model_path = 'yolo11n.pt'
            
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Model loaded: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model error: {e}")
            self.model = None

    def setup_topics(self):
        """Setup topics"""
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        # Subscribers
        self.camera_subs = []
        for i, name in enumerate(camera_names):
            topic = f'/camera_{name}/image_raw'
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.ultra_fast_callback(msg, idx), 1)
            self.camera_subs.append(sub)
        
        # Publishers
        self.result_pubs = []
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/deepstream_detections', 1)
            self.result_pubs.append(pub)
        
        # Grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)

    def ultra_fast_callback(self, msg, camera_idx):
        """ULTRA-FAST callback"""
        try:
            # Frame skipping
            self.frame_skip_counters[camera_idx] += 1
            if self.frame_skip_counters[camera_idx] % self.skip_frames != 0:
                return
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_images[camera_idx] = cv_image
            
            # Fast inference
            if self.model:
                self.minimal_inference(cv_image, msg.header, camera_idx)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def minimal_inference(self, image, header, camera_idx):
        """MINIMAL inference for maximum speed"""
        try:
            # Resize to very small size
            small = cv2.resize(image, (self.input_size, self.input_size))
            
            # ULTRA-FAST inference
            results = self.model(small,
                               conf=self.conf_thres,
                               verbose=False,
                               max_det=self.max_det)
            
            # Create message
            msg = Yolov12Inference()
            msg.header = header
            msg.camera_name = f"camera_{camera_idx}"
            msg.task = "detect"
            
            # Process results
            if results[0].boxes is not None:
                scale = image.shape[1] / self.input_size
                
                for box in results[0].boxes[:self.max_det]:
                    result = InferenceResult()
                    result.class_name = results[0].names[int(box.cls)]
                    result.confidence = float(box.conf)
                    
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale)
                    result.top = int(y1 * scale)
                    result.right = int(x2 * scale)
                    result.bottom = int(y2 * scale)
                    
                    msg.yolov12_inference.append(result)
            
            # Publish
            if camera_idx < len(self.result_pubs):
                self.result_pubs[camera_idx].publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Inference error: {e}")

    def log_fps(self):
        """Log FPS"""
        fps = self.frame_count / 2.0
        self.get_logger().info(f"🚀 Ultra-Fast FPS: {fps:.1f}")
        
        if fps >= 50:
            self.get_logger().info("🎯 Excellent performance!")
        
        self.frame_count = 0

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltraFastDeepStreamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()