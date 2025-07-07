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
        self.setup_fusion_integration()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.latest_images = [None] * 6
        self.frame_skip_counters = [0] * 6
        
        # Timers
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.grid_timer = self.create_timer(0.033, self.publish_grid)  # 30 FPS grid
        
        self.get_logger().info("🚀 ULTRA-FAST DeepStream for 100+ FPS initialized!")

    def setup_model(self):
        """Setup ultra-optimized model"""
        try:
            from ultralytics import YOLO
            
            # Try to find model
            model_candidates = [
                'yolo11n-seg.engine',
                'yolo12x.engine', 
                'yolo11n.engine',
                '/home/jezzy/huskybot/models/yolo11n-seg.engine'
            ]
            
            model_path = None
            for candidate in model_candidates:
                if os.path.exists(candidate):
                    model_path = candidate
                    break
            
            if not model_path:
                raise FileNotFoundError("No YOLO model found")
            
            self.model = YOLO(model_path)
            
            # MINIMAL warmup
            dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            for i in range(2):
                start = time.time()
                self.model(dummy, conf=self.conf_thres, device='cuda:0', 
                          half=True, verbose=False, max_det=self.max_det,
                          imgsz=self.input_size, agnostic_nms=True)
                self.get_logger().info(f"🔥 Warmup {i+1}: {(time.time()-start)*1000:.1f}ms")
            
            self.get_logger().info(f"✅ Model ready: {os.path.basename(model_path)}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model error: {e}")
            self.model = None

    def setup_topics(self):
        """Setup topics for 100+ FPS"""
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        # Subscribers
        self.camera_subs = []
        for i, name in enumerate(camera_names):
            topic = f'/camera_{name}/image_raw'
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.ultra_fast_callback(msg, idx), 1)
            self.camera_subs.append(sub)
        
        # Publishers for fusion integration
        self.result_pubs = []
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/detections', 1)
            self.result_pubs.append(pub)
        
        # Grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 Topics ready for fusion integration")

    def setup_fusion_integration(self):
        """Setup integration with fusion node"""
        # Camera angle mapping for fusion
        self.camera_angles = {
            0: 180,   # front -> real back
            1: 225,   # front_left -> real left_back  
            2: 270,   # left -> real left_front
            3: 0,     # rear -> real front
            4: 315,   # rear_right -> real right_front
            5: 45     # right -> real right_back
        }

    def ultra_fast_callback(self, msg, camera_idx):
        """ULTRA-FAST callback for 100+ FPS"""
        try:
            # AGGRESSIVE frame skipping
            self.frame_skip_counters[camera_idx] += 1
            if self.frame_skip_counters[camera_idx] % self.skip_frames != 0:
                return
            
            # Convert and store
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_images[camera_idx] = cv_image
            
            # OPTIMIZED inference
            if self.model:
                self.minimal_inference(cv_image, msg.header, camera_idx)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def minimal_inference(self, image, header, camera_idx):
        """ULTRA-FAST inference optimized for fusion"""
        try:
            # Resize to very small size
            small = cv2.resize(image, (self.input_size, self.input_size), 
                             interpolation=cv2.INTER_NEAREST)
            
            # ULTRA-FAST inference
            results = self.model(small,
                               conf=self.conf_thres,
                               device='cuda:0',
                               half=True,
                               verbose=False,
                               max_det=self.max_det,
                               imgsz=self.input_size,
                               agnostic_nms=True)
            
            # Create message optimized for fusion
            msg = Yolov12Inference()
            msg.header = header
            msg.camera_name = f"camera_{camera_idx}"
            msg.task = "detect"
            msg.frame_type = "deepstream_optimized"
            
            # Process detections for fusion
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
                    
                    # Add camera angle for fusion
                    result.angle = self.camera_angles.get(camera_idx, 0)
                    
                    msg.yolov12_inference.append(result)
            
            # Publish for fusion
            self.result_pubs[camera_idx].publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Inference error: {e}")

    def publish_grid(self):
        """Create grid for monitoring"""
        try:
            valid_images = [img for img in self.latest_images if img is not None]
            
            if len(valid_images) >= 4:
                grid_images = []
                target_size = (160, 120)
                
                for i in range(6):
                    if self.latest_images[i] is not None:
                        img = cv2.resize(self.latest_images[i], target_size, 
                                       interpolation=cv2.INTER_NEAREST)
                        cv2.putText(img, f"C{i}", (5, 15), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        grid_images.append(img)
                    else:
                        black = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                        cv2.putText(black, f"C{i}", (5, 15), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        grid_images.append(black)
                
                # Create 2x3 grid
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # Add FPS info
                fps = self.frame_count / 2.0 if hasattr(self, 'frame_count') else 0
                cv2.putText(grid, f"FPS: {fps:.1f}", (5, grid.shape[0] - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                # Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "deepstream_grid"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid error: {e}")

    def log_fps(self):
        """Log performance"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"🚀 DeepStream FPS: {fps:.1f}")
            
            if fps >= 100:
                self.get_logger().info("🎯 TARGET ACHIEVED: 100+ FPS!")
            elif fps >= 60:
                self.get_logger().info("✅ Excellent performance!")
            elif fps >= 30:
                self.get_logger().info("✅ Good performance!")
            else:
                self.get_logger().warn(f"⚡ Performance: {fps:.1f} FPS")
        
        self.frame_count = 0
        self.last_fps_time = current_time

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