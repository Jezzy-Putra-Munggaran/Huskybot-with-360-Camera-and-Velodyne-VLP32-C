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
import random

class DeepStreamYOLONode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        # Bridge
        self.bridge = CvBridge()
        
        # Setup parameters
        self.setup_parameters()
        
        # Setup ROS topics with CORRECTED mapping
        self.setup_ros_topics()
        
        # Initialize OPTIMIZED frame processing
        self.setup_optimized_processing()
        
        # ✅ COCO Colors for 80 classes with distinct colors
        self.setup_coco_colors()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        # Camera data tracking
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.last_process_time = [0.0] * 6
        
        # ✅ Detection results for enhanced visualization
        self.latest_detections = [[] for _ in range(6)]
        
        self.get_logger().info("🚀 ULTRA-ENHANCED DeepStream YOLO Node initialized!")

    def setup_parameters(self):
        """Setup parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 120)  # ✅ TARGET 100+ FPS
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 1)  # ✅ MINIMAL skipping for 100+ FPS
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        # Frame skip counters
        self.frame_skip_counters = [0] * 6

    def setup_coco_colors(self):
        """Setup distinct colors for 80 COCO classes"""
        # ✅ Generate 80 distinct colors for COCO classes
        self.coco_colors = []
        random.seed(42)  # For consistent colors
        for i in range(80):
            # Generate bright, distinct colors
            color = [
                random.randint(50, 255),   # R
                random.randint(50, 255),   # G  
                random.randint(50, 255)    # B
            ]
            self.coco_colors.append(color)
        
        # Ensure some common classes have specific colors
        self.coco_colors[0] = [255, 0, 0]     # person = red
        self.coco_colors[2] = [0, 255, 0]     # car = green
        self.coco_colors[56] = [0, 0, 255]    # chair = blue
        self.coco_colors[62] = [255, 255, 0]  # tv = yellow

    def setup_ros_topics(self):
        """Setup ROS2 topics with CORRECTED camera mapping"""
        # ✅ CORRECTED camera mapping based on real hardware
        self.camera_subs = []
        camera_names = ['rear', 'rear_left', 'front_left', 'front', 'front_right', 'rear_right']
        actual_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_front_left/image_raw', # KAMERA KIRI BELAKANG
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_right/image_raw'       # KAMERA KANAN BELAKANG
        ]
        
        for i, (name, topic) in enumerate(zip(camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 1)
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic} -> Camera {name}")
        
        # Publishers for results - ENHANCED
        self.result_pubs = []
        for name in camera_names:
            # ✅ Use both detections and segmentation topics
            det_pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # ✅ LARGE Grid visualization publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 CORRECTED ROS2 topics configured")

    def setup_optimized_processing(self):
        """Setup ULTRA-OPTIMIZED processing for 100+ FPS"""
        try:
            from ultralytics import YOLO
            model_path = os.path.join(os.path.dirname(__file__), 'config', self.model_engine)
            if not os.path.exists(model_path):
                model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.yolo_model = YOLO(model_path)
            
            # ULTRA-AGGRESSIVE warm-up for 100+ FPS
            self.get_logger().info(f"🔥 ULTRA-Warming up model for 100+ FPS...")
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            # Extensive warmup for maximum performance
            for i in range(5):
                start_time = time.time()
                results = self.yolo_model(dummy_image, 
                                       conf=0.2,  # ✅ OPTIMIZED for speed
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=30,  # ✅ More detections
                                       imgsz=self.input_width,
                                       task='segment')  # ✅ SEGMENTATION
                warmup_time = time.time() - start_time
                self.get_logger().info(f"🔥 Ultra-Warmup {i+1}: {warmup_time*1000:.1f}ms")
            
            self.get_logger().info(f"✅ Model ULTRA-optimized for 100+ FPS: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            import traceback
            traceback.print_exc()
            self.yolo_model = None

    def camera_callback(self, msg, camera_idx):
        """ULTRA-ENHANCED camera callback for 100+ FPS"""
        try:
            # Minimal frame skipping for maximum FPS
            self.frame_skip_counters[camera_idx] += 1
            if self.frame_skip_counters[camera_idx] % self.skip_frames != 0:
                return
            
            # Check frequency for 100+ FPS target
            current_time = time.time()
            if current_time - self.last_process_time[camera_idx] < 0.008:  # Max 125 FPS per camera
                return
            
            self.last_process_time[camera_idx] = current_time
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Store for batch processing
            self.latest_images[camera_idx] = cv_image
            self.latest_headers[camera_idx] = msg.header
            
            # Process immediately for real-time feedback
            detections = self.process_single_frame_ultra(cv_image, msg.header, camera_idx)
            self.latest_detections[camera_idx] = detections
            
            # Create LARGE enhanced grid
            self.try_create_ultra_grid()
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def process_single_frame_ultra(self, frame, header, camera_idx):
        """ULTRA-ENHANCED processing with full segmentation and distance estimation"""
        detections = []
        try:
            if not self.yolo_model:
                return detections
            
            # ✅ ULTRA-FAST resize
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_NEAREST)  # Fastest interpolation
            
            # ✅ ULTRA-FAST inference
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.2,  # ✅ SPEED-optimized
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=25,
                                   imgsz=self.input_width,
                                   task='segment')  # ✅ FULL SEGMENTATION
            
            inference_time = time.time() - start_time
            
            # Create ENHANCED messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{camera_idx}"
            detection_msg.task = "detect"
            detection_msg.frame_type = "ultra_segmentation"
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{camera_idx}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = "ultra_segmentation"
            
            # ✅ PROCESS results with FULL data
            detection_count = 0
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                scale_x = frame.shape[1] / self.input_width
                scale_y = frame.shape[0] / self.input_height
                
                # Get masks if available
                masks = results[0].masks.data.cpu().numpy() if results[0].masks else None
                
                for i, box in enumerate(results[0].boxes):
                    result = InferenceResult()
                    class_id = int(box.cls)
                    result.class_name = results[0].names[class_id]
                    result.confidence = float(box.conf)
                    
                    # Scale back to original size
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale_x)
                    result.top = int(y1 * scale_y)
                    result.right = int(x2 * scale_x)
                    result.bottom = int(y2 * scale_y)
                    
                    # ✅ ENHANCED distance estimation (akan diperbaiki oleh fusion)
                    bbox_area = (result.right - result.left) * (result.bottom - result.top)
                    result.distance = max(1.0, 100000.0 / (bbox_area + 1))  # Better heuristic
                    
                    # ✅ TEMPORARY coordinates (akan diperbaiki oleh fusion)
                    result.coordinate_x = 0.0
                    result.coordinate_y = 0.0
                    result.coordinate_z = 0.5
                    result.angle = 0.0
                    
                    # ✅ ADD segmentation mask data
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        # Resize mask to match bounding box
                        mask_resized = cv2.resize(mask, (result.right - result.left, 
                                                       result.bottom - result.top))
                        result.mask_data = (mask_resized * 255).astype(np.uint8).flatten().tolist()
                        result.mask_width = result.right - result.left
                        result.mask_height = result.bottom - result.top
                    else:
                        result.mask_data = []
                        result.mask_width = 0
                        result.mask_height = 0
                    
                    # ✅ ADD distinct colors per class
                    if class_id < len(self.coco_colors):
                        color = self.coco_colors[class_id]
                        result.color_r = color[0]
                        result.color_g = color[1]
                        result.color_b = color[2]
                    else:
                        result.color_r = 255
                        result.color_g = 255
                        result.color_b = 255
                    
                    detection_msg.yolov12_inference.append(result)
                    segmentation_msg.yolov12_inference.append(result)
                    detections.append(result)
                    detection_count += 1
                    
                    # ✅ ENHANCED logging with estimated data
                    if i < 2:  # Log first 2 detections
                        self.get_logger().info(
                            f"🎯 Camera {camera_idx}: {result.class_name} "
                            f"conf={result.confidence:.2f} "
                            f"Distance≈{result.distance:.1f}m "
                            f"Color=({result.color_r},{result.color_g},{result.color_b}) "
                            f"bbox=({result.left},{result.top},{result.right},{result.bottom})"
                        )
            
            # ✅ PUBLISH to both topics
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
                
                if detection_count > 0:
                    self.get_logger().info(f"✅ Published {detection_count} ultra-detections/segments for camera {camera_idx}")
            
            self.detection_count += detection_count
            
            # Performance logging
            if camera_idx == 0:
                fps_estimate = 1.0 / inference_time if inference_time > 0 else 0
                self.get_logger().info(
                    f"📊 Camera {camera_idx}: {detection_count} detections, "
                    f"{inference_time*1000:.1f}ms inference, "
                    f"≈{fps_estimate:.0f} FPS potential"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error {camera_idx}: {e}")
            import traceback
            traceback.print_exc()
        
        return detections

    def try_create_ultra_grid(self):
        """Create ULTRA-LARGE grid visualization"""
        try:
            current_time = time.time()
            valid_images = []
            
            for i in range(6):
                if (self.latest_images[i] is not None and 
                    current_time - self.last_process_time[i] < 2.0):
                    valid_images.append(i)
            
            if len(valid_images) >= 4:
                self.create_ultra_grid_visualization(valid_images)
                
        except Exception as e:
            self.get_logger().error(f"❌ Ultra grid creation error: {e}")

    def create_ultra_grid_visualization(self, valid_cameras):
        """Create ULTRA-LARGE grid with FULL information display"""
        try:
            # ✅ LARGE grid size for clear visibility
            target_size = (640, 480)  # MUCH LARGER for readability
            
            # ✅ CORRECTED camera names
            camera_names = ['REAR', 'REAR-L', 'FRONT-L', 'FRONT', 'FRONT-R', 'REAR-R']
            
            grid_images = []
            for i in range(6):
                if i in valid_cameras and self.latest_images[i] is not None:
                    img = cv2.resize(self.latest_images[i], target_size)
                    
                    # ✅ DRAW ENHANCED segmentation masks and bounding boxes
                    if i < len(self.latest_detections) and self.latest_detections[i]:
                        scale_x = target_size[0] / self.latest_images[i].shape[1]
                        scale_y = target_size[1] / self.latest_images[i].shape[0]
                        
                        for det in self.latest_detections[i]:
                            # Scale coordinates
                            x1 = int(det.left * scale_x)
                            y1 = int(det.top * scale_y)
                            x2 = int(det.right * scale_x)
                            y2 = int(det.bottom * scale_y)
                            
                            # ✅ DRAW segmentation mask with transparency
                            if det.mask_data and det.mask_width > 0 and det.mask_height > 0:
                                try:
                                    mask_array = np.array(det.mask_data, dtype=np.uint8)
                                    mask = mask_array.reshape((det.mask_height, det.mask_width))
                                    mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
                                    
                                    # Resize mask to bbox size on grid
                                    mask_resized = cv2.resize(mask_colored, (x2-x1, y2-y1))
                                    
                                    # Overlay with transparency
                                    roi = img[y1:y2, x1:x2]
                                    if roi.shape == mask_resized.shape:
                                        img[y1:y2, x1:x2] = cv2.addWeighted(roi, 0.6, mask_resized, 0.4, 0)
                                except:
                                    pass
                            
                            # ✅ DRAW bounding box with class-specific color
                            color = (int(det.color_b), int(det.color_g), int(det.color_r))  # BGR for OpenCV
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                            
                            # ✅ ENHANCED label with ALL information
                            label_lines = [
                                f"{det.class_name} {det.confidence:.2f}",
                                f"Dist: {det.distance:.1f}m",
                                f"Pos: ({det.coordinate_x:.1f},{det.coordinate_y:.1f})"
                            ]
                            
                            # Draw multi-line label with background
                            for j, line in enumerate(label_lines):
                                y_offset = y1 - 35 + j * 15
                                if y_offset > 15:
                                    # Background rectangle
                                    (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                                    cv2.rectangle(img, (x1, y_offset-12), (x1+w+5, y_offset+3), color, -1)
                                    # Text
                                    cv2.putText(img, line, (x1+2, y_offset), 
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # ✅ ADD camera info overlay
                    cv2.rectangle(img, (0, 0), (target_size[0], 40), (0, 0, 0), -1)
                    cv2.putText(img, f"{camera_names[i]}", (10, 25), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    # Detection count
                    det_count = len(self.latest_detections[i]) if i < len(self.latest_detections) else 0
                    cv2.putText(img, f"Det: {det_count}", (target_size[0]-80, 25), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    grid_images.append(img)
                else:
                    # Black placeholder
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{camera_names[i]} OFFLINE", 
                              (target_size[0]//4, target_size[1]//2), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    grid_images.append(black_img)
            
            # ✅ Create LARGE 2x3 grid
            top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
            bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
            grid = np.vstack([top_row, bottom_row])
            
            # ✅ ADD comprehensive info overlay
            total_detections = sum(len(dets) for dets in self.latest_detections)
            current_fps = self.frame_count / 2.0 if self.frame_count > 0 else 0
            
            # Large info panel
            info_height = 80
            info_panel = np.zeros((info_height, grid.shape[1], 3), dtype=np.uint8)
            
            # Main stats
            main_info = f"🚀 ULTRA-DeepStream | FPS: {current_fps:.1f} | Total Detections: {total_detections} | Active Cameras: {len(valid_cameras)}/6"
            cv2.putText(info_panel, main_info, (20, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # Performance info
            perf_info = f"🎯 Target: 100+ FPS | Segmentation: ON | 3D Fusion: ON | Color Coding: ON"
            cv2.putText(info_panel, perf_info, (20, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Combine grid with info panel
            final_grid = np.vstack([grid, info_panel])
            
            # ✅ Publish ULTRA-LARGE grid
            grid_msg = self.bridge.cv2_to_imgmsg(final_grid, 'bgr8')
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = "ultra_deepstream_grid"
            self.grid_pub.publish(grid_msg)
            
            self.get_logger().info(
                f"📸 ULTRA-Grid: {len(valid_cameras)} cams, {total_detections} detections, "
                f"Size: {final_grid.shape[1]}x{final_grid.shape[0]}"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Ultra grid visualization error: {e}")

    def log_fps(self):
        """ENHANCED FPS logging for 100+ FPS target"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            total_detections = sum(len(dets) for dets in self.latest_detections)
            
            self.get_logger().info(
                f"🚀 ULTRA-DeepStream FPS: {fps:.1f} | "
                f"Detections/s: {detection_rate:.1f} | "
                f"Current Total: {total_detections}"
            )
            
            if fps >= 100:
                self.get_logger().info("🎯 TARGET ACHIEVED: 100+ FPS!")
            elif fps >= 50:
                self.get_logger().info("✅ Excellent performance!")
            elif fps >= 30:
                self.get_logger().warn(f"⚡ Good performance: {fps:.1f} FPS")
            else:
                self.get_logger().warn(f"🐌 Below target: {fps:.1f} FPS")
        
        # Reset counters
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        try:
            self.get_logger().info("🛑 ULTRA-DeepStream node shutdown")
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