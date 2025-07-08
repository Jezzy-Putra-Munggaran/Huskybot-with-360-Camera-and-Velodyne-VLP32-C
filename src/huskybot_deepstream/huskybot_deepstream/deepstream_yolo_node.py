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

class DeepStreamYOLONode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        # Bridge
        self.bridge = CvBridge()
        
        # Setup parameters
        self.setup_parameters()
        
        # Setup ROS topics - FIXED MAPPING
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
        
        # ✅ NEW: Detection results for visualization
        self.latest_detections = [[] for _ in range(6)]
        
        self.get_logger().info("🚀 ENHANCED DeepStream YOLO Node initialized!")

    def setup_parameters(self):
        """Setup parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 60)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 3)  # ✅ REDUCED for better response
        
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
        """Setup ROS2 topics - FIXED MAPPING"""
        # ✅ FIXED: Use correct topic names from ros_deep_learning
        self.camera_subs = []
        camera_topics = [
            '/video_source/raw',  # This is the actual topic from ros_deep_learning
            '/video_source/raw',  # Will need to differentiate by camera index
            '/video_source/raw',
            '/video_source/raw', 
            '/video_source/raw',
            '/video_source/raw'
        ]
        
        # ✅ BETTER: Subscribe to individual camera topics if available
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        actual_topics = [
            '/camera_front/image_raw',
            '/camera_front_left/image_raw', 
            '/camera_left/image_raw',
            '/camera_rear/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_right/image_raw'
        ]
        
        for i, (name, topic) in enumerate(zip(camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 1)
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic}")
        
        # Publishers for results - ENHANCED
        self.result_pubs = []
        for name in camera_names:
            # ✅ FIXED: Use both detections and segmentation topics
            det_pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # Grid visualization publisher - ENHANCED
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 Enhanced ROS2 topics configured")

    def setup_optimized_processing(self):
        """Setup ULTRA-OPTIMIZED processing"""
        try:
            from ultralytics import YOLO
            model_path = os.path.join(os.path.dirname(__file__), 'config', self.model_engine)
            if not os.path.exists(model_path):
                # ✅ FIXED: Use absolute path
                model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.yolo_model = YOLO(model_path)
            
            # AGGRESSIVE warm-up
            self.get_logger().info(f"🔥 Warming up model with {self.input_width}x{self.input_height}...")
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            # Multiple warmup iterations
            for i in range(3):
                start_time = time.time()
                results = self.yolo_model(dummy_image, 
                                       conf=0.3,  # ✅ OPTIMIZED confidence
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=20,  # ✅ More detections
                                       imgsz=self.input_width)
                warmup_time = time.time() - start_time
                self.get_logger().info(f"🔥 Warmup {i+1}: {warmup_time*1000:.1f}ms")
            
            self.get_logger().info(f"✅ Model loaded and optimized: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            import traceback
            traceback.print_exc()
            self.yolo_model = None

    def camera_callback(self, msg, camera_idx):
        """ENHANCED camera callback with better logging"""
        try:
            # Frame skipping for performance
            self.frame_skip_counters[camera_idx] += 1
            if self.frame_skip_counters[camera_idx] % self.skip_frames != 0:
                return
            
            # Check if too frequent
            current_time = time.time()
            if current_time - self.last_process_time[camera_idx] < 0.05:  # Max 20 FPS per camera
                return
            
            self.last_process_time[camera_idx] = current_time
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Store for batch processing
            self.latest_images[camera_idx] = cv_image
            self.latest_headers[camera_idx] = msg.header
            
            # Process immediately for real-time feedback
            detections = self.process_single_frame(cv_image, msg.header, camera_idx)
            self.latest_detections[camera_idx] = detections
            
            # Try to create enhanced grid
            self.try_create_enhanced_grid()
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def process_single_frame(self, frame, header, camera_idx):
        """ENHANCED single frame processing with segmentation"""
        detections = []
        try:
            if not self.yolo_model:
                self.get_logger().warn("❌ No YOLO model available")
                return detections
            
            # Resize for faster inference
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_LINEAR)
            
            # ✅ ENHANCED inference for segmentation
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.25,  # ✅ OPTIMIZED confidence
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=15,  # ✅ More detections
                                   imgsz=self.input_width,
                                   task='segment')  # ✅ SEGMENTATION MODE
            
            inference_time = time.time() - start_time
            
            # Create detection messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{camera_idx}"
            detection_msg.task = "detect"
            detection_msg.frame_type = "segmentation"  # ✅ SPECIFY SEGMENTATION
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{camera_idx}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = "segmentation"
            
            # Process results with enhanced logging
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
                    segmentation_msg.yolov12_inference.append(result)
                    detections.append(result)
                    detection_count += 1
                    
                    # ✅ ENHANCED logging with distance estimation
                    if i < 3:  # Log first 3 detections
                        # Simple distance estimation based on bbox size
                        bbox_area = (result.right - result.left) * (result.bottom - result.top)
                        estimated_distance = max(1.0, 50000.0 / (bbox_area + 1))  # Simple heuristic
                        
                        self.get_logger().info(
                            f"🎯 Camera {camera_idx}: {result.class_name} "
                            f"conf={result.confidence:.2f} "
                            f"Distance≈{estimated_distance:.1f}m "
                            f"bbox=({result.left},{result.top},{result.right},{result.bottom})"
                        )
            
            # ✅ PUBLISH to both detection and segmentation topics
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
                
                if detection_count > 0:
                    self.get_logger().info(f"✅ Published {detection_count} detections/segments for camera {camera_idx}")
            
            self.detection_count += detection_count
            
            # Performance logging for camera 0 only
            if camera_idx == 0:
                self.get_logger().info(
                    f"📊 Camera {camera_idx}: {detection_count} detections, "
                    f"{inference_time*1000:.1f}ms inference, "
                    f"segmentation mode"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error {camera_idx}: {e}")
            import traceback
            traceback.print_exc()
        
        return detections

    def try_create_enhanced_grid(self):
        """Create ENHANCED grid visualization with bounding boxes"""
        try:
            # Check if we have recent images from all cameras
            current_time = time.time()
            valid_images = []
            
            for i in range(6):
                if (self.latest_images[i] is not None and 
                    current_time - self.last_process_time[i] < 3.0):  # Within 3 seconds
                    valid_images.append(i)
            
            # Need at least 4 cameras for grid
            if len(valid_images) >= 4:
                self.create_enhanced_grid_visualization(valid_images)
                
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced grid creation error: {e}")

    def create_enhanced_grid_visualization(self, valid_cameras):
        """Create ENHANCED grid visualization with bounding boxes and info"""
        try:
            # Get images and resize for grid
            grid_images = []
            target_size = (320, 240)  # Grid cell size
            
            camera_names = ['Front', 'Front-L', 'Left', 'Rear', 'Rear-R', 'Right']
            
            for i in range(6):
                if i in valid_cameras and self.latest_images[i] is not None:
                    img = cv2.resize(self.latest_images[i], target_size)
                    
                    # ✅ DRAW BOUNDING BOXES from detections
                    if i < len(self.latest_detections) and self.latest_detections[i]:
                        # Scale detection coordinates to grid size
                        scale_x = target_size[0] / self.latest_images[i].shape[1]
                        scale_y = target_size[1] / self.latest_images[i].shape[0]
                        
                        for det in self.latest_detections[i]:
                            # Scale bounding box to grid size
                            x1 = int(det.left * scale_x)
                            y1 = int(det.top * scale_y)
                            x2 = int(det.right * scale_x)
                            y2 = int(det.bottom * scale_y)
                            
                            # Draw bounding box
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # Draw label with confidence
                            label = f"{det.class_name} {det.confidence:.2f}"
                            cv2.putText(img, label, (x1, y1-5), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    
                    # Add camera label
                    cv2.putText(img, f"{camera_names[i]}", (5, 20), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Add detection count
                    det_count = len(self.latest_detections[i]) if i < len(self.latest_detections) else 0
                    cv2.putText(img, f"Det: {det_count}", (5, target_size[1]-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    
                    grid_images.append(img)
                else:
                    # Black placeholder
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{camera_names[i]} OFF", (30, 120), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    grid_images.append(black_img)
            
            # Create 2x3 grid
            top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
            bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
            grid = np.vstack([top_row, bottom_row])
            
            # ✅ ADD ENHANCED INFO OVERLAY
            total_detections = sum(len(dets) for dets in self.latest_detections)
            info_text = f"FPS: {self.frame_count/2.0:.1f} | Total Detections: {total_detections} | Active Cams: {len(valid_cameras)}"
            cv2.putText(grid, info_text, (10, grid.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add timestamp
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(grid, f"Time: {timestamp}", (10, grid.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Publish enhanced grid
            grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = "deepstream_enhanced_grid"
            self.grid_pub.publish(grid_msg)
            
            self.get_logger().info(f"📸 Enhanced Grid: {len(valid_cameras)} cams, {total_detections} total detections")
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced grid visualization error: {e}")

    def log_fps(self):
        """Enhanced FPS logging with detection stats"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            total_detections = sum(len(dets) for dets in self.latest_detections)
            
            self.get_logger().info(
                f"🚀 DeepStream FPS: {fps:.1f} | "
                f"Detections/s: {detection_rate:.1f} | "
                f"Current Total: {total_detections}"
            )
            
            if fps >= 15:
                self.get_logger().info("✅ Good performance!")
            elif fps >= 8:
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
            self.get_logger().info("🛑 Enhanced DeepStream node shutdown")
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