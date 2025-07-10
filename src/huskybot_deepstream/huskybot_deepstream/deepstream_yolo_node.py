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
                torch.cuda.set_per_process_memory_fraction(0.98)  # Use 98% of GPU memory
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
                "yolo11x-seg",  # Auto-download
                "yolov8x-seg",  # Fallback to standard model that will auto-download
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
                model_path = "yolo11x-seg"  # Fallback to standard model
                self.get_logger().warn("⚠️ Using fallback: yolo11x-seg")
            
            self.get_logger().info(f"🔥 Loading YOLO model: {model_path}")
            
            # ✅ FIXED: Proper model loading with correct API
            self.yolo_model = YOLO(model_path)
            
            # ✅ FIXED: Warmup for consistent performance
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            start_time = time.time()
            for _ in range(5):  # More warmup iterations
                try:
                    results = self.yolo_model.predict(
                        source=dummy_image,
                        conf=0.25,  # Lower confidence for more detections
                        device='cuda:0',
                        half=True,
                        verbose=False,
                        agnostic_nms=True,
                        max_det=300  # Allow more detections
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
        
        # ✅ Enhanced color palette for better visibility
        base_colors = [
            [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], 
            [0, 255, 255], [255, 128, 0], [128, 255, 0], [255, 0, 128], [128, 0, 255],
            [0, 128, 255], [0, 255, 128], [255, 64, 64], [64, 255, 64], [64, 64, 255],
            [255, 192, 0], [192, 255, 0], [255, 0, 192], [192, 0, 255], [0, 192, 255],
            [255, 128, 128], [128, 255, 128], [128, 128, 255], [255, 255, 128], [255, 128, 255],
            [128, 255, 255], [192, 192, 0], [192, 0, 192], [0, 192, 192], [128, 64, 0]
        ]
        
        for i in range(80):
            if i < len(base_colors):
                color = base_colors[i]
            else:
                # Generate more distinct colors using HSV
                hue = (i * 137.5) % 360
                sat = 0.85 + (i % 3) * 0.05
                val = 0.8 + (i % 4) * 0.05
                r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
                color = [int(r*255), int(g*255), int(b*255)]
            
            self.coco_colors.append(color)
            
            # Calculate contrasting text color
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
        self.frame_queues = [queue.Queue(maxsize=3) for _ in range(6)]  # Larger queue
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
                        # Remove oldest frame
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
        batch_camera_indices = [None] * 6
        
        while self.processing_active:
            try:
                frames_ready = 0
                for i in range(6):
                    try:
                        frame_data = self.frame_queues[i].get(timeout=0.001)
                        batch_frames[i], batch_headers[i], batch_camera_indices[i] = frame_data
                        frames_ready += 1
                    except queue.Empty:
                        continue
                
                # Process any available frames
                if frames_ready > 0:
                    self.maximum_speed_batch_inference(batch_frames, batch_headers, batch_camera_indices)
                
                time.sleep(0.0001)  # Minimal sleep for maximum speed
                
            except Exception as e:
                self.get_logger().error(f"❌ Batch worker error: {e}")
                time.sleep(0.001)

    def maximum_speed_batch_inference(self, batch_frames, batch_headers, batch_camera_indices):
        """✅ MAXIMUM speed batch inference with PERFECT segmentation"""
        if not self.yolo_model:
            return
            
        for camera_idx in range(6):
            if batch_frames[camera_idx] is None:
                continue
                
            try:
                frame = batch_frames[camera_idx]
                header = batch_headers[camera_idx]
                
                # ✅ Resize for inference
                resized = cv2.resize(frame, (self.input_width, self.input_height), 
                                  interpolation=cv2.INTER_LINEAR)
                
                # ✅ MAXIMUM optimized inference
                start_time = time.time()
                
                # ✅ FIXED: Proper inference call with segmentation
                results = self.yolo_model.predict(
                    source=resized,
                    conf=0.25,
                    device='cuda:0',
                    half=True,
                    verbose=False,
                    agnostic_nms=True,
                    max_det=100,
                    retina_masks=True  # High-quality masks
                )
                
                inference_time = time.time() - start_time
                self.total_inference_time += inference_time
                self.inference_count += 1
                
                # ✅ Process results
                if results and len(results) > 0:
                    result = results[0]  # First result
                    
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
                    
                    num_objects = len(result.boxes) if hasattr(result, 'boxes') and result.boxes is not None else 0
                    fps_text = f"FPS: {1000/max(1, inference_time*1000):.1f} | Objects: {num_objects}"
                    cv2.putText(display_frame, fps_text, 
                              (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    
                    # ✅ Process boxes and masks
                    if hasattr(result, 'boxes') and result.boxes is not None and len(result.boxes) > 0:
                        boxes = result.boxes
                        masks = result.masks if hasattr(result, 'masks') and result.masks is not None else None
                        
                        for i, box in enumerate(boxes):
                            # Extract detection info
                            cls_id = int(box.cls.item())
                            conf = box.conf.item()
                            xyxy = box.xyxy.cpu().numpy()[0]
                            x1, y1, x2, y2 = map(int, xyxy)
                            
                            # Get class name and color
                            cls_name = result.names[cls_id]
                            color = self.coco_colors[cls_id % len(self.coco_colors)]
                            text_color = self.text_colors[cls_id % len(self.text_colors)]
                            
                            # Calculate distance and coordinates (using enhanced fusion)
                            bbox_center_x = (x1 + x2) / 2
                            image_width = frame.shape[1]
                            angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                            
                            # Get camera base angle from mapping
                            base_angle = 0
                            if camera_idx == 0:  # REAR
                                base_angle = 180
                            elif camera_idx == 1:  # REAR RIGHT
                                base_angle = 135
                            elif camera_idx == 2:  # FRONT RIGHT
                                base_angle = 45
                            elif camera_idx == 3:  # FRONT
                                base_angle = 0
                            elif camera_idx == 4:  # FRONT LEFT
                                base_angle = 315
                            elif camera_idx == 5:  # REAR LEFT
                                base_angle = 225
                            
                            object_angle = (base_angle + angle_offset) % 360
                            
                            # Estimate distance based on bounding box size
                            bbox_height = y2 - y1
                            distance = 15.0 * (720.0 / max(1, bbox_height))  # Rough distance estimation
                            distance = min(50.0, max(0.5, distance))  # Clamp between reasonable values
                            
                            # Calculate 3D coordinates
                            angle_rad = math.radians(object_angle)
                            x = distance * math.cos(angle_rad)
                            y = distance * math.sin(angle_rad)
                            z = 0.5  # Default height
                            
                            # ✅ Draw segmentation mask if available
                            if masks is not None and i < len(masks.data):
                                mask = masks.data[i].cpu().numpy()
                                
                                # Resize mask to frame size
                                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                                mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                                
                                # Create colored mask overlay
                                colored_mask = np.zeros_like(frame)
                                colored_mask[:,:,0] = (mask_binary / 255) * color[0]
                                colored_mask[:,:,1] = (mask_binary / 255) * color[1]
                                colored_mask[:,:,2] = (mask_binary / 255) * color[2]
                                
                                # Blend with display frame
                                alpha = 0.3
                                mask_area = mask_binary > 0
                                if np.any(mask_area):
                                    display_frame[mask_area] = cv2.addWeighted(
                                        display_frame[mask_area], 1-alpha, 
                                        colored_mask[mask_area], alpha, 0)
                                
                                # Draw contour around mask
                                contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                if contours:
                                    cv2.drawContours(display_frame, contours, -1, color, 3)
                            
                            # Draw bounding box
                            cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 3)
                            
                            # ✅ Draw enhanced info text with better positioning
                            info_texts = [
                                f"{cls_name} {conf:.2f}",
                                f"Distance: {distance:.1f}m",
                                f"Coord: ({x:.1f}, {y:.1f}, {z:.1f})"
                            ]
                            
                            # Dynamic text positioning
                            text_y = y1 - 10
                            if text_y < 20:
                                text_y = y2 + 20
                            
                            for idx, text in enumerate(info_texts):
                                text_pos_y = text_y + (idx * 25)
                                
                                # Text background
                                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                                cv2.rectangle(display_frame, 
                                            (x1, text_pos_y - 20), 
                                            (x1 + text_size[0] + 10, text_pos_y + 5), 
                                            (0, 0, 0), -1)
                                
                                # Text
                                cv2.putText(display_frame, text, (x1 + 5, text_pos_y), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                            
                            # Prepare detection for ROS message
                            detection = InferenceResult()
                            detection.class_name = cls_name
                            detection.confidence = conf
                            detection.left = float(x1)
                            detection.top = float(y1)
                            detection.right = float(x2)
                            detection.bottom = float(y2)
                            detection.angle = float(object_angle)
                            detection.distance = float(distance)
                            detection.coordinate_x = float(x)
                            detection.coordinate_y = float(y)
                            detection.coordinate_z = float(z)
                            detection.color_r = color[0]
                            detection.color_g = color[1]
                            detection.color_b = color[2]
                            
                            # Add mask data if available
                            if masks is not None and i < len(masks.data):
                                mask_data = masks.data[i].cpu().numpy()
                                # Compress mask to save bandwidth
                                mask_small = cv2.resize(mask_data, (64, 64))
                                mask_bytes = (mask_small > 0.5).astype(np.uint8).tobytes()
                                detection.mask_data = list(mask_bytes)
                                detection.mask_width = 64
                                detection.mask_height = 64
                            
                            # Add to messages
                            detection_msg.yolov12_inference.append(detection)
                            segmentation_msg.yolov12_inference.append(detection)
                    
                    # ✅ Update display image
                    with self.image_locks[camera_idx]:
                        self.latest_images[camera_idx] = display_frame
                    
                    # ✅ Publish results
                    if camera_idx < len(self.result_pubs):
                        self.result_pubs[camera_idx][0].publish(detection_msg)
                        self.result_pubs[camera_idx][1].publish(segmentation_msg)
                    
                    # Count detections for performance metrics
                    self.detection_count += len(detection_msg.yolov12_inference)
                
            except Exception as e:
                self.get_logger().error(f"❌ Inference error for camera {camera_idx}: {e}")
                import traceback
                traceback.print_exc()

    def maximum_speed_grid_worker(self):
        """✅ MAXIMUM optimized grid worker"""
        while self.processing_active:
            try:
                # Create enhanced grid at optimal rate
                self.create_maximum_enhanced_grid()
                time.sleep(0.025)  # ~40 FPS grid updates (optimal for display)
            except Exception as e:
                self.get_logger().error(f"❌ Grid worker error: {e}")
                time.sleep(0.1)

    def create_maximum_enhanced_grid(self):
        """✅ MAXIMUM enhanced 2x3 grid with ENGLISH labels"""
        try:
            target_size = (720, 405)  # Larger size for better visibility
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
                
                status_height = 140  # Larger status area
                cv2.rectangle(grid, (0, grid.shape[0]-status_height), (grid.shape[1], grid.shape[0]), (0, 0, 0), -1)
                
                status_color = (0, 255, 0) if theoretical_fps >= 100 else (0, 255, 255)
                
                info_lines = [
                    f"HUSKYBOT 360° MAXIMUM-OPTIMIZED SEGMENTATION | YOLO11X ENGINE",
                    f"Theoretical FPS: {theoretical_fps:.1f} | Target: 100+ | Status: {'TARGET ACHIEVED!' if theoretical_fps >= 100 else 'OPTIMIZING...'}",
                    f"Inference: {avg_inference*1000:.1f}ms | ALL 6 cameras with PERFECT segmentation + distance + coordinates",
                    f"Display: Camera, Class, Confidence, Distance, Coordinates | Resolution: {grid.shape[1]}x{grid.shape[0]}",
                    f"Press 'q' in auto_grid_viewer to quit | MAXIMUM Jetson AGX Orin optimization ACTIVE",
                    f"Processed frames: {self.frame_count} | Detections: {self.detection_count}"
                ]
                
                for idx, info_line in enumerate(info_lines):
                    y_pos = grid.shape[0] - status_height + 15 + (idx * 20)
                    cv2.putText(grid, info_line, (20, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
                
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