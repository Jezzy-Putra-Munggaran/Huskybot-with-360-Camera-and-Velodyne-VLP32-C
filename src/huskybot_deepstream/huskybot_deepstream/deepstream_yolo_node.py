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
import sys
import gi

# Import GStreamer
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

class DeepStreamYOLONode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        # Initialize GStreamer
        Gst.init(None)
        GObject.threads_init()
        
        # Bridge
        self.bridge = CvBridge()
        
        # Setup parameters
        self.setup_parameters()
        
        # Setup ROS topics
        self.setup_ros_topics()
        
        # Initialize OPTIMIZED frame processing
        self.setup_optimized_processing()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        # Camera data tracking
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.last_process_time = [0.0] * 6
        
        self.get_logger().info("🚀 OPTIMIZED DeepStream YOLO Node initialized!")

    def setup_parameters(self):
        """Setup parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)   # ✅ Match model
        self.declare_parameter('input_height', 640)  # ✅ Match model
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 60)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 5)     # Process every 5th frame
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        # Frame skip counters
        self.frame_skip_counters = [0] * 6

    def setup_ros_topics(self):
        """Setup ROS2 topics"""
        # Subscribers for 6 cameras
        self.camera_subs = []
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        for i, name in enumerate(camera_names):
            topic = f'/camera_{name}/image_raw'
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 1)  # Small queue
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic}")
        
        # Publishers for results
        self.result_pubs = []
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/deepstream_detections', 1)
            self.result_pubs.append(pub)
        
        # Grid visualization publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 ROS2 topics configured")

    def setup_optimized_processing(self):
        """Setup ULTRA-OPTIMIZED processing"""
        # Load model with optimizations
        try:
            from ultralytics import YOLO
            model_path = os.path.join(os.path.dirname(__file__), 'config', self.model_engine)
            if not os.path.exists(model_path):
                model_path = self.model_engine
            
            self.yolo_model = YOLO(model_path)
            
            # AGGRESSIVE warm-up
            self.get_logger().info(f"🔥 Warming up model with {self.input_width}x{self.input_height}...")
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            # Multiple warmup iterations
            for i in range(3):
                start_time = time.time()
                results = self.yolo_model(dummy_image, 
                                       conf=0.5, 
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=10,
                                       imgsz=self.input_width)
                warmup_time = time.time() - start_time
                self.get_logger().info(f"🔥 Warmup {i+1}: {warmup_time*1000:.1f}ms")
            
            self.get_logger().info(f"✅ Model loaded and optimized: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def camera_callback(self, msg, camera_idx):
        """OPTIMIZED camera callback with frame skipping"""
        try:
            if camera_idx == 0:  # Log hanya camera pertama
                self.get_logger().info(f"📸 Received frame from camera {camera_idx}")
            
            # Frame skipping for performance
            self.frame_skip_counters[camera_idx] += 1
            if self.frame_skip_counters[camera_idx] % self.skip_frames != 0:
                return
            
            # Check if too frequent
            current_time = time.time()
            if current_time - self.last_process_time[camera_idx] < 0.033:  # Max 30 FPS per camera
                return
            
            self.last_process_time[camera_idx] = current_time
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Store for batch processing
            self.latest_images[camera_idx] = cv_image
            self.latest_headers[camera_idx] = msg.header
            
            # Process immediately for real-time feedback
            self.process_single_frame(cv_image, msg.header, camera_idx)
            
            # Try to create grid if we have enough images
            self.try_create_grid()
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def process_single_frame(self, frame, header, camera_idx):  # ✅ FIXED indentation
        """FAST single frame processing with DETAILED logging"""
        try:
            if not self.yolo_model:
                self.get_logger().warn("❌ No YOLO model available")
                return
            
            # Resize for faster inference - MATCH model size exactly
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_LINEAR)
            
            # Fast inference with detailed logging
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.25,  # ✅ LOWER confidence for more detections
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=10,  # ✅ More detections
                                   imgsz=self.input_width)
            
            inference_time = time.time() - start_time
            
            # DETAILED result logging
            if results[0].boxes is not None:
                self.get_logger().info(f"🎯 Camera {camera_idx}: Found {len(results[0].boxes)} raw detections")
            else:
                self.get_logger().warn(f"⚠️ Camera {camera_idx}: No boxes detected, conf_thres might be too high")
            
            # Create detection message
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{camera_idx}"
            detection_msg.task = "detect"
            
            # Process results with detailed logging
            detection_count = 0
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                scale_x = frame.shape[1] / self.input_width
                scale_y = frame.shape[0] / self.input_height
                
                for i, box in enumerate(results[0].boxes):
                    result = InferenceResult()
                    result.class_name = results[0].names[int(box.cls)]
                    result.confidence = float(box.conf)
                    
                    # Scale back to original size
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale_x)
                    result.top = int(y1 * scale_y)
                    result.right = int(x2 * scale_x)
                    result.bottom = int(y2 * scale_y)
                    
                    detection_msg.yolov12_inference.append(result)
                    detection_count += 1
                    
                    # Log first few detections for debugging
                    if i < 3:  # Log first 3 detections
                        self.get_logger().info(
                            f"🎯 Detection {i+1}: {result.class_name} "
                            f"conf={result.confidence:.2f} "
                            f"bbox=({result.left},{result.top},{result.right},{result.bottom})"
                        )
            
            # Publish detection with confirmation
            if camera_idx < len(self.result_pubs):
                self.result_pubs[camera_idx].publish(detection_msg)
                if detection_count > 0:
                    self.get_logger().info(f"✅ Published {detection_count} detections for camera {camera_idx}")
            
            self.detection_count += detection_count
            
            # Performance logging
            if camera_idx == 0:  # Log only camera 0 to reduce spam
                self.get_logger().info(
                    f"📊 Camera {camera_idx}: {detection_count} detections, "
                    f"{inference_time*1000:.1f}ms inference, "
                    f"input_size: {self.input_width}x{self.input_height}"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error {camera_idx}: {e}")
            import traceback
            traceback.print_exc()

    def try_create_grid(self):
        """Create grid visualization if enough images available"""
        try:
            # Check if we have recent images from all cameras
            current_time = time.time()
            valid_images = []
            
            for i in range(6):
                if (self.latest_images[i] is not None and 
                    current_time - self.last_process_time[i] < 2.0):  # Within 2 seconds
                    valid_images.append(i)
            
            # Need at least 4 cameras for grid
            if len(valid_images) >= 4:
                self.create_grid_visualization(valid_images)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def create_grid_visualization(self, valid_cameras):
        """Create and publish grid visualization"""
        try:
            # Get images and resize for grid
            grid_images = []
            target_size = (320, 240)  # Small for performance
            
            for i in range(6):
                if i in valid_cameras and self.latest_images[i] is not None:
                    img = cv2.resize(self.latest_images[i], target_size)
                    
                    # Add camera label
                    cv2.putText(img, f"Cam {i}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    grid_images.append(img)
                else:
                    # Black placeholder
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_img, f"Cam {i} OFF", (50, 120), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    grid_images.append(black_img)
            
            # Create 2x3 grid
            top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
            bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
            grid = np.vstack([top_row, bottom_row])
            
            # Add info overlay
            info_text = f"FPS: {self.frame_count/2.0:.1f} | Detections: {self.detection_count}"
            cv2.putText(grid, info_text, (10, grid.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Publish grid
            grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = "deepstream_grid"
            self.grid_pub.publish(grid_msg)
            
            self.get_logger().info(f"📸 Grid published with {len(valid_cameras)} active cameras")
            
        except Exception as e:
            self.get_logger().error(f"❌ Grid visualization error: {e}")

    def log_fps(self):
        """Enhanced FPS logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            
            self.get_logger().info(f"🚀 DeepStream FPS: {fps:.1f} | Detections/s: {detection_rate:.1f}")
            
            if fps >= 30:
                self.get_logger().info("✅ Good performance!")
            elif fps >= 15:
                self.get_logger().warn(f"⚡ Moderate performance: {fps:.1f} FPS")
            else:
                self.get_logger().warn(f"🐌 Low performance: {fps:.1f} FPS")
        
        # Reset counters
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        try:
            self.get_logger().info("🛑 DeepStream node shutdown")
            super().destroy_node()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DeepStreamYOLONode()
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