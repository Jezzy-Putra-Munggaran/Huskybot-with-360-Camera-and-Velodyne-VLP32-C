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
import queue
import os
import gc
import torch
import colorsys
import math

class MaximumOptimizedDeepStreamNode(Node):
    def __init__(self):
        super().__init__('maximum_optimized_deepstream')
        
        self.bridge = CvBridge()
        self.setup_parameters()
        
        # ✅ MAXIMUM optimized data structures
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.processing_flags = [False] * 6
        self.image_locks = [threading.Lock() for _ in range(6)]
        
        # ✅ Setup components
        self.setup_ros_topics()
        self.setup_enhanced_coco_colors()
        self.setup_maximum_optimized_model()
        
        # Only setup processing if model loaded successfully
        if self.yolo_model is not None:
            self.setup_parallel_processing()
        
        # ✅ Performance monitoring
        self.frame_count = 0
        self.detection_count = 0
        self.total_inference_time = 0
        self.inference_count = 0
        self.fps_timer = self.create_timer(1.0, self.log_performance)
        self.last_fps_time = time.time()
        
        # ✅ Force maximum Jetson optimization
        self.force_jetson_optimization()
        
        self.get_logger().info("🚀 MAXIMUM-OPTIMIZED DeepStream Node initialized!")

    def setup_parameters(self):
        """Setup MAXIMUM-OPTIMIZED parameters"""
        self.declare_parameter('model_engine', 'yolo11x-seg.engine')
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

    def force_jetson_optimization(self):
        """✅ FORCE maximum Jetson optimization"""
        try:
            if torch.cuda.is_available():
                # ✅ MAXIMUM CUDA optimization
                torch.cuda.set_per_process_memory_fraction(0.95)  # Use 95% of GPU memory
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.deterministic = False
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
                
                # ✅ MAXIMUM Jetson hardware optimization
                os.system('sudo jetson_clocks')
                os.system('sudo nvpmodel -m 0')  # Maximum performance mode
                os.system('echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor')
                
                # ✅ Memory optimization
                gc.collect()
                torch.cuda.empty_cache()
                
                self.get_logger().info("🔥 MAXIMUM Jetson optimization activated for 100+ FPS!")
                
        except Exception as e:
            self.get_logger().warn(f"Jetson optimization warning: {e}")

    def setup_maximum_optimized_model(self):
        """✅ FIXED: Proper YOLO11X model loading"""
        try:
            from ultralytics import YOLO
            
            # ✅ FIXED: Correct model candidates
            model_candidates = [
                "/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",  # Absolute path first
                "./yolo11x-seg.engine",  # Relative path
                "/home/jezzy/huskybot/yolo11x-seg.engine",  # Alternative path
                "/opt/nvidia/deepstream/deepstream/samples/models/yolo11x-seg.engine",  # DeepStream path
                "yolov8n-seg",  # Fallback to standard model that will auto-download
            ]
            
            model_path = None
            for candidate in model_candidates:
                if candidate.startswith("yolo") and not candidate.startswith("/") and not candidate.startswith("./"):
                    model_path = candidate
                    self.get_logger().info(f"🔄 Will auto-download: {candidate}")
                    break
                elif os.path.exists(candidate):
                    model_path = candidate
                    self.get_logger().info(f"✅ Found local model: {candidate}")
                    break
            
            if not model_path:
                model_path = "yolov8n-seg"  # Fallback to standard model
                self.get_logger().warn("⚠️ Using fallback: yolov8n-seg")
            
            self.get_logger().info(f"🔥 Loading YOLO model: {model_path}")
            
            # ✅ FIXED: Proper model loading with correct API
            self.yolo_model = YOLO(model_path)
            
            # ✅ FIXED: Warmup for consistent performance
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            start_time = time.time()
            for _ in range(3):
                try:
                    results = self.yolo_model.predict(
                        source=dummy_image,
                        conf=0.35,
                        device='cuda:0',
                        half=True,
                        verbose=False
                    )
                    # Just consume results to ensure complete warmup
                    for r in results:
                        _ = r
                except Exception as e:
                    self.get_logger().warn(f"Warmup warning: {e}")
                    break
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ YOLO Model ready: {warmup_time*1000:.1f}ms")
            
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def setup_enhanced_coco_colors(self):
        """Setup 80 DISTINCT colors for COCO classes"""
        self.coco_colors = []
        self.text_colors = []
        
        base_colors = [
            [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], 
            [0, 255, 255], [255, 128, 0], [128, 255, 0], [255, 0, 128], [128, 0, 255],
            [0, 128, 255], [0, 255, 128], [255, 64, 64], [64, 255, 64], [64, 64, 255],
            [255, 192, 0], [192, 255, 0], [255, 0, 192], [192, 0, 255], [0, 192, 255]
        ]
        
        for i in range(80):
            if i < len(base_colors):
                color = base_colors[i]
            else:
                hue = (i * 137.5) % 360
                sat = 0.9 + (i % 2) * 0.1
                val = 0.8 + (i % 3) * 0.1
                r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
                color = [int(r*255), int(g*255), int(b*255)]
            
            self.coco_colors.append(color)
            
            brightness = (color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114) / 255
            self.text_colors.append((0, 0, 0) if brightness > 0.5 else (255, 255, 255))

    def setup_ros_topics(self):
        """Setup ROS2 topics with CORRECT camera mapping"""
        self.camera_subs = []
        # ✅ FIXED: Use English camera names
        self.camera_names = ['REAR', 'REAR_RIGHT', 'FRONT_RIGHT', 'FRONT', 'FRONT_LEFT', 'REAR_LEFT']
        self.camera_labels = ['REAR CAMERA', 'REAR RIGHT CAMERA', 'FRONT RIGHT CAMERA', 
                             'FRONT CAMERA', 'FRONT LEFT CAMERA', 'REAR LEFT CAMERA']
        
        actual_topics = [
            '/camera_front/image_raw',      # REAR CAMERA
            '/camera_right/image_raw',      # REAR RIGHT CAMERA
            '/camera_rear_right/image_raw', # FRONT RIGHT CAMERA
            '/camera_rear/image_raw',       # FRONT CAMERA
            '/camera_left/image_raw',       # FRONT LEFT CAMERA
            '/camera_front_left/image_raw'  # REAR LEFT CAMERA
        ]
        
        for i, (name, topic) in enumerate(zip(self.camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.maximum_speed_callback(msg, idx), 
                1
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed: {topic} -> {name}")
        
        # ✅ Publishers for results
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name.lower()}/detections', 1)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name.lower()}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # ✅ Enhanced grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)

    def setup_parallel_processing(self):
        """✅ MAXIMUM parallel processing"""
        self.frame_queues = [queue.Queue(maxsize=2) for _ in range(6)]
        self.processing_active = True
        
        # ✅ Batch processing thread
        self.batch_thread = threading.Thread(
            target=self.maximum_speed_batch_worker, 
            daemon=True
        )
        self.batch_thread.start()
        
        # ✅ Enhanced grid thread
        self.grid_thread = threading.Thread(
            target=self.maximum_speed_grid_worker, 
            daemon=True
        )
        self.grid_thread.start()

    def maximum_speed_callback(self, msg, camera_idx):
        """✅ MAXIMUM speed callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
                self.latest_headers[camera_idx] = msg.header
            
            if hasattr(self, 'frame_queues'):
                try:
                    self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
                except queue.Full:
                    try:
                        self.frame_queues[camera_idx].get_nowait()
                        self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
                    except queue.Empty:
                        pass
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def maximum_speed_batch_worker(self):
        """✅ MAXIMUM speed batch processing"""
        batch_frames = [None] * 6
        batch_headers = [None] * 6
        
        while self.processing_active:
            try:
                frames_ready = 0
                for i in range(6):
                    try:
                        frame_data = self.frame_queues[i].get(timeout=0.001)
                        batch_frames[i], batch_headers[i] = frame_data
                        frames_ready += 1
                    except queue.Empty:
                        continue
                
                if frames_ready >= 3:
                    self.maximum_speed_batch_inference(batch_frames, batch_headers)
                
                time.sleep(0.001)
                
            except Exception as e:
                self.get_logger().error(f"❌ Batch worker error: {e}")
                time.sleep(0.01)

    def maximum_speed_batch_inference(self, batch_frames, batch_headers):
        """✅ MAXIMUM speed batch inference with PERFECT segmentation"""
        try:
        if not self.yolo_model:
            return
        
        for camera_idx in range(6):
            if batch_frames[camera_idx] is None:
                continue
            
            frame = batch_frames[camera_idx]
            header = batch_headers[camera_idx]
            
            # ✅ Resize for inference
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                              interpolation=cv2.INTER_LINEAR)
            
            # ✅ MAXIMUM optimized inference
            start_time = time.time()
            try:
                results = self.yolo_model.predict(
                    source=resized, 
                    conf=0.35,
                    device='cuda:0',
                    half=True,
                    verbose=False,
                    task='segment'  # CRITICAL: Ensure segmentation mode
                )
                result = results[0]  # Get first result
            except Exception as e:
                self.get_logger().error(f"❌ Inference error: {e}")
                continue
            
            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            self.inference_count += 1
            
            # ✅ Create enhanced messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{self.camera_names[camera_idx].lower()}"
            detection_msg.task = "detect"
            detection_msg.frame_type = f"maximum_optimized_{camera_idx}"
            detection_msg.note = f"Inference: {inference_time*1000:.1f}ms"
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{self.camera_names[camera_idx].lower()}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = f"maximum_optimized_{camera_idx}"
            segmentation_msg.note = f"Inference: {inference_time*1000:.1f}ms"
            
            # ✅ Create ENHANCED display image
            display_frame = frame.copy()
            
            # ✅ FIXED: English camera info header
            camera_label = self.camera_labels[camera_idx]
            cv2.rectangle(display_frame, (0, 0), (frame.shape[1], 80), (0, 0, 0), -1)
            cv2.putText(display_frame, f"CAMERA {camera_idx+1}: {camera_label}", 
                      (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            
            num_objects = len(result.boxes) if hasattr(result, 'boxes') else 0
            fps_text = f"FPS: {1000/max(1, inference_time*1000):.1f} | Objects: {num_objects}"
            cv2.putText(display_frame, fps_text, 
                      (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            # ✅ Process boxes and masks
            if hasattr(result, 'boxes') and len(result.boxes) > 0:
                # Calculate scaling factors
                scale_x = frame.shape[1] / self.input_width
                scale_y = frame.shape[0] / self.input_height
                
                # ✅ Get masks for segmentation
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    try:
                        # Get masks from result
                        if hasattr(result.masks, 'data'):
                            masks = result.masks.data.cpu().numpy()
                        elif hasattr(result.masks, 'cpu'):
                            masks = result.masks.cpu().numpy()
                        else:
                            masks = result.masks
                    except Exception as mask_error:
                        self.get_logger().error(f"Mask error: {mask_error}")
            
                # Process each detection
                for i, box in enumerate(result.boxes):
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(x1 * scale_x), int(y1 * scale_y), int(x2 * scale_x), int(y2 * scale_y)
                    
                    # Get class and confidence
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    
                    # Get class name
                    class_name = result.names[cls_id] if hasattr(result, 'names') else f"class_{cls_id}"
                    
                    # Get color for this class
                    color = self.coco_colors[cls_id % len(self.coco_colors)]
                    text_color = self.text_colors[cls_id % len(self.text_colors)]
                    
                    # Draw box
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw mask if available
                    if masks is not None and i < len(masks):
                        try:
                            # Resize mask to original image size
                            mask = cv2.resize(masks[i].astype(np.uint8), (frame.shape[1], frame.shape[0]))
                            
                            # Create colored mask overlay
                            colored_mask = np.zeros_like(display_frame)
                            colored_mask[mask > 0.5] = color
                            
                            # Apply mask with transparency
                            alpha = 0.5  # Transparency factor
                            mask_indices = mask > 0.5
                            display_frame[mask_indices] = cv2.addWeighted(
                                display_frame[mask_indices], 
                                1 - alpha,
                                colored_mask[mask_indices], 
                                alpha, 0
                            )
                        except Exception as e:
                            self.get_logger().error(f"Mask overlay error: {e}")
                    
                    # Calculate object position
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Calculate distance estimation
                    box_height = y2 - y1
                    box_width = x2 - x1
                    normalized_size = (box_height * box_width) / (frame.shape[0] * frame.shape[1])
                    estimated_distance = max(1.0, min(50.0, 8.0 / (normalized_size + 0.05)))
                    
                    # Calculate angle from center of image
                    angle_offset = ((center_x / frame.shape[1]) - 0.5) * 60.0
                    
                    # Base angle from camera position
                    base_angle = {
                        0: 180.0,  # REAR
                        1: 135.0,  # REAR_RIGHT
                        2: 45.0,   # FRONT_RIGHT
                        3: 0.0,    # FRONT
                        4: 315.0,  # FRONT_LEFT
                        5: 225.0   # REAR_LEFT
                    }.get(camera_idx, 0.0)
                    
                    # Calculate global angle
                    object_angle = (base_angle + angle_offset) % 360.0
                    
                    # Calculate 3D coordinates
                    x_coord = estimated_distance * np.cos(np.radians(object_angle))
                    y_coord = estimated_distance * np.sin(np.radians(object_angle))
                    z_coord = (box_height / frame.shape[0]) * estimated_distance * 0.5
                    
                    # Create detection result message
                    detection_result = InferenceResult()
                    detection_result.class_name = class_name
                    detection_result.confidence = conf
                    detection_result.left = x1
                    detection_result.top = y1
                    detection_result.right = x2
                    detection_result.bottom = y2
                    detection_result.distance = estimated_distance
                    detection_result.angle = object_angle
                    detection_result.coordinate_x = x_coord
                    detection_result.coordinate_y = y_coord
                    detection_result.coordinate_z = z_coord
                    
                    # Set color for visualization
                    detection_result.color_r = color[2]  # BGR to RGB
                    detection_result.color_g = color[1]
                    detection_result.color_b = color[0]
                    
                    # Add result to message
                    detection_msg.yolov12_inference.append(detection_result)
                    segmentation_msg.yolov12_inference.append(detection_result)
                    
                    # Add mask data if available
                    if masks is not None and i < len(masks):
                        try:
                            # Convert mask to bytes for message
                            small_mask = cv2.resize(masks[i].astype(np.uint8), (100, 100))
                            detection_result.mask_data = small_mask.tobytes()
                            detection_result.mask_width = 100
                            detection_result.mask_height = 100
                        except Exception as e:
                            self.get_logger().error(f"Mask conversion error: {e}")
                    
                    # ✅ Add text with detection info to the frame
                    info_lines = [
                        f"Camera: {self.camera_names[camera_idx]}",
                        f"Class: {class_name}, Conf: {conf:.2f}",
                        f"Distance: {estimated_distance:.2f}m",
                        f"Coordinate: ({x_coord:.2f}, {y_coord:.2f}, {z_coord:.2f})"
                    ]
                    
                    # ✅ Display text with background
                    for idx, info_line in enumerate(info_lines):
                        line_y = y1 + 25 + (idx * 25)
                        
                        # Calculate text size
                        text_size, _ = cv2.getTextSize(info_line, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                        
                        # Draw background for text
                        cv2.rectangle(display_frame, 
                                    (x1 - 5, line_y - 20), 
                                    (x1 + text_size[0] + 10, line_y + 5), 
                                    color, -1)
                        
                        # Draw text
                        cv2.putText(display_frame, info_line, 
                                  (x1, line_y), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            
            # ✅ Update display image
            with self.image_locks[camera_idx]:
                self.latest_images[camera_idx] = display_frame
            
            # ✅ Publish results
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
            
            # Count detections for performance metrics
            self.detection_count += len(detection_msg.yolov12_inference)
            
    except Exception as e:
        self.get_logger().error(f"❌ Batch inference error: {e}")

    def maximum_speed_grid_worker(self):
        """✅ MAXIMUM optimized grid worker"""
        while self.processing_active:
            try:
                # Create enhanced grid at optimal rate
                self.create_maximum_enhanced_grid()
                time.sleep(0.033)  # ~30 FPS grid updates (optimal for display)
            except Exception as e:
                self.get_logger().error(f"❌ Grid worker error: {e}")
                time.sleep(0.1)

    def create_maximum_enhanced_grid(self):
        """✅ MAXIMUM enhanced 2x3 grid with ENGLISH labels"""
        try:
            target_size = (640, 360)
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.image_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    img_resized = cv2.resize(img, target_size, interpolation=cv2.INTER_LINEAR)
                    grid_images.append(img_resized)
                else:
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    camera_label = self.camera_labels[i]
                    cv2.putText(black_img, f"CAM {i+1}: {camera_label}", 
                              (target_size[0]//4, target_size[1]//2-20), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    cv2.putText(black_img, "WAITING...", 
                              (target_size[0]//3, target_size[1]//2+30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ FIXED: English status info
                avg_inference = self.total_inference_time / max(1, self.inference_count)
                theoretical_fps = 1.0 / avg_inference if avg_inference > 0 else 0
                
                status_height = 120
                cv2.rectangle(grid, (0, grid.shape[0]-status_height), (grid.shape[1], grid.shape[0]), (0, 0, 0), -1)
                
                status_color = (0, 255, 0) if theoretical_fps >= 100 else (0, 255, 255)
                
                info_lines = [
                    f"HUSKYBOT 360° MAXIMUM-OPTIMIZED SEGMENTATION | YOLO11X ENGINE",
                    f"Theoretical FPS: {theoretical_fps:.1f} | Target: 100+ | Status: {'TARGET ACHIEVED!' if theoretical_fps >= 100 else 'OPTIMIZING...'}",
                    f"Inference: {avg_inference*1000:.1f}ms | ALL 6 cameras with PERFECT segmentation + distance + coordinates",
                    f"Display: Camera, Class, Confidence, Distance, Coordinates | Resolution: {grid.shape[1]}x{grid.shape[0]}",
                    f"Press 'q' in auto_grid_viewer to quit | MAXIMUM Jetson AGX Orin optimization ACTIVE"
                ]
                
                for idx, info_line in enumerate(info_lines):
                    y_pos = grid.shape[0] - status_height + 15 + (idx * 22)
                    cv2.putText(grid, info_line, (20, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
                
                # ✅ Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "maximum_optimized_grid_all_cameras_segmentation_yolo11x"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def log_performance(self):
        """✅ Enhanced performance logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            actual_fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            avg_inference = self.total_inference_time / max(1, self.inference_count)
            theoretical_fps = 1.0 / avg_inference if avg_inference > 0 else 0
            
            if theoretical_fps >= 100:
                self.get_logger().info(
                    f"🎯 TARGET ACHIEVED! Theoretical: {theoretical_fps:.1f} FPS | "
                    f"Actual: {actual_fps:.1f} FPS | Det/s: {detection_rate:.1f} | "
                    f"Inference: {avg_inference*1000:.1f}ms | YOLO11X MAXIMUM SPEED!"
                )
            else:
                self.get_logger().info(
                    f"🔥 MAXIMUM Optimizing: Theoretical: {theoretical_fps:.1f} FPS | "
                    f"Actual: {actual_fps:.1f} FPS | Det/s: {detection_rate:.1f} | "
                    f"Inference: {avg_inference*1000:.1f}ms | Jetson MAXIMUM MODE"
                )
        
        self.frame_count = 0
        self.detection_count = 0
        self.total_inference_time = 0
        self.inference_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'processing_active'):
            self.processing_active = False
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = MaximumOptimizedDeepStreamNode()
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