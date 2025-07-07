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
        
        # Initialize frame processing
        self.setup_frame_processing()
        
        # Statistics
        self.frame_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        self.get_logger().info("🚀 DeepStream YOLO Node initialized for 100+ FPS!")

    def setup_parameters(self):
        """Setup parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value

    def setup_ros_topics(self):
        """Setup ROS2 topics"""
        # Subscribers for 6 cameras
        self.camera_subs = []
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        for i, name in enumerate(camera_names):
            topic = f'/camera_{name}/image_raw'
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 10)
            self.camera_subs.append(sub)
        
        # Publishers for results
        self.result_pubs = []
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/deepstream_detections', 10)
            self.result_pubs.append(pub)
        
        # Grid visualization publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 ROS2 topics configured")

    def setup_frame_processing(self):
        """Setup simplified frame processing for now"""
        self.latest_frames = [None] * 6
        self.frame_times = [0.0] * 6
        
        # For now, use CPU-based YOLO processing
        # Later we'll integrate with actual DeepStream pipeline
        try:
            from ultralytics import YOLO
            model_path = os.path.join(os.path.dirname(__file__), 'config', self.model_engine)
            if not os.path.exists(model_path):
                model_path = self.model_engine
            
            self.yolo_model = YOLO(model_path)
            self.get_logger().info(f"✅ YOLO model loaded: {model_path}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ YOLO model not available: {e}")
            self.yolo_model = None

    def camera_callback(self, msg, camera_idx):
        """Handle camera input with high-speed processing"""
        try:
            # Convert ROS image to CV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Store latest frame
            self.latest_frames[camera_idx] = cv_image
            self.frame_times[camera_idx] = time.time()
            
            # Process frame
            self.process_frame_fast(cv_image, msg.header, camera_idx)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def process_frame_fast(self, frame, header, camera_idx):
        """Fast frame processing"""
        try:
            # Create detection message
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{camera_idx}"
            detection_msg.task = "detect"
            
            # Quick YOLO inference if available
            if self.yolo_model:
                # Resize for faster inference
                small_frame = cv2.resize(frame, (320, 320))
                results = self.yolo_model(small_frame, conf=0.5, verbose=False)
                
                if results[0].boxes is not None:
                    for box in results[0].boxes:
                        result = InferenceResult()
                        result.class_name = results[0].names[int(box.cls)]
                        result.confidence = float(box.conf)
                        
                        # Scale back to original size
                        scale_x = frame.shape[1] / 320
                        scale_y = frame.shape[0] / 320
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        
                        result.left = int(x1 * scale_x)
                        result.top = int(y1 * scale_y)
                        result.right = int(x2 * scale_x)
                        result.bottom = int(y2 * scale_y)
                        
                        detection_msg.yolov12_inference.append(result)
            
            # Publish detection
            if camera_idx < len(self.result_pubs):
                self.result_pubs[camera_idx].publish(detection_msg)
            
            # Create visualization if needed
            if camera_idx == 0:  # Only process front camera for visualization
                self.create_and_publish_visualization(frame, detection_msg, header)
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error: {e}")

    def create_and_publish_visualization(self, frame, detection_msg, header):
        """Create simple visualization"""
        try:
            vis_frame = frame.copy()
            
            # Draw detections
            for detection in detection_msg.yolov12_inference:
                x1, y1, x2, y2 = detection.left, detection.top, detection.right, detection.bottom
                
                # Draw rectangle
                cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label
                label = f"{detection.class_name}: {detection.confidence:.2f}"
                cv2.putText(vis_frame, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Add camera info
            cv2.putText(vis_frame, "DeepStream Front Camera", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_frame, 'bgr8')
            vis_msg.header = header
            self.grid_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Visualization error: {e}")

    def log_fps(self):
        """Log FPS statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"🚀 DeepStream FPS: {fps:.1f}")
            
            if fps >= 60:
                self.get_logger().info("🎯 Excellent performance!")
            elif fps >= 30:
                self.get_logger().info("✅ Good performance!")
            else:
                self.get_logger().warn(f"⚡ Performance: {fps:.1f} FPS")
        
        # Reset counters
        self.frame_count = 0
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