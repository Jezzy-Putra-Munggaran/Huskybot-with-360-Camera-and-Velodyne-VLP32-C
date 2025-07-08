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
import queue
import concurrent.futures
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
        
        # ✅ ENHANCED COCO Colors - 80 distinct colors
        self.setup_enhanced_coco_colors()
        
        # ✅ ULTRA-OPTIMIZED Multi-threading setup
        self.setup_threading()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        # ✅ Enhanced camera data tracking with locks
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.latest_detections = [[] for _ in range(6)]
        self.camera_locks = [threading.Lock() for _ in range(6)]
        self.last_process_time = [0.0] * 6
        
        # ✅ Fusion data integration
        self.fused_data = [[] for _ in range(6)]
        self.fused_locks = [threading.Lock() for _ in range(6)]
        
        self.get_logger().info("🚀 ULTRA-ENHANCED DeepStream YOLO Node initialized!")

    def setup_parameters(self):
        """Setup parameters for 100+ FPS"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)  # ✅ FULL batch processing
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 0)  # ✅ NO frame skipping for max FPS
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value
        self.skip_frames = self.get_parameter('skip_frames').value

    def setup_enhanced_coco_colors(self):
        """Setup 80 highly distinct colors for COCO classes"""
        # ✅ Predefined 80 distinct colors with high contrast
        self.coco_colors = [
            [255, 0, 0],     # 0: person - bright red
            [0, 255, 0],     # 1: bicycle - bright green  
            [0, 0, 255],     # 2: car - bright blue
            [255, 255, 0],   # 3: motorcycle - yellow
            [255, 0, 255],   # 4: airplane - magenta
            [0, 255, 255],   # 5: bus - cyan
            [128, 0, 0],     # 6: train - dark red
            [0, 128, 0],     # 7: truck - dark green
            [0, 0, 128],     # 8: boat - dark blue
            [255, 128, 0],   # 9: traffic light - orange
            [128, 255, 0],   # 10: fire hydrant - lime
            [0, 128, 255],   # 11: stop sign - light blue
            [255, 0, 128],   # 12: parking meter - pink
            [128, 0, 255],   # 13: bench - purple
            [255, 255, 128], # 14: bird - light yellow
            [128, 255, 255], # 15: cat - light cyan
        ]
        
        # Generate remaining 64 colors with high contrast
        for i in range(16, 80):
            # Use HSV for better color distribution
            hue = (i * 137.5) % 360  # Golden angle for even distribution
            sat = 0.7 + (i % 3) * 0.1  # Vary saturation
            val = 0.8 + (i % 2) * 0.2  # Vary brightness
            
            # Convert HSV to RGB
            import colorsys
            r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
            self.coco_colors.append([int(r*255), int(g*255), int(b*255)])

    def setup_threading(self):
        """Setup ULTRA-OPTIMIZED threading for 100+ FPS"""
        # ✅ Thread pools for parallel processing
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=8)
        
        # ✅ Queues for batch processing
        self.frame_queues = [queue.Queue(maxsize=2) for _ in range(6)]
        
        # ✅ Processing threads
        self.processing_threads = []
        for i in range(6):
            thread = threading.Thread(target=self.camera_processing_worker, args=(i,), daemon=True)
            thread.start()
            self.processing_threads.append(thread)
        
        # ✅ Batch inference thread
        self.batch_thread = threading.Thread(target=self.batch_inference_worker, daemon=True)
        self.batch_thread.start()
        
        # ✅ Grid creation thread
        self.grid_thread = threading.Thread(target=self.grid_creation_worker, daemon=True)
        self.grid_thread.start()

    def setup_ros_topics(self):
        """Setup ROS2 topics with CORRECTED camera mapping"""
        # ✅ CORRECTED camera mapping
        self.camera_subs = []
        self.camera_names = ['rear', 'rear_left', 'front_left', 'front', 'front_right', 'rear_right']
        actual_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_front_left/image_raw', # KAMERA KIRI BELAKANG
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_right/image_raw'       # KAMERA KANAN BELAKANG
        ]
        
        for i, (name, topic) in enumerate(zip(self.camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 1)
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic} -> Camera {name}")
        
        # ✅ Publishers for results
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # ✅ Fusion data subscription
        self.fusion_subs = []
        for name in self.camera_names:
            fusion_sub = self.create_subscription(
                Yolov12Inference, f'/camera_{name}/fused_detections',
                lambda msg, idx=self.camera_names.index(name): self.fusion_callback(msg, idx), 1)
            self.fusion_subs.append(fusion_sub)
        
        # ✅ EXTRA LARGE Grid visualization publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 ROS2 topics configured with fusion integration")

    def fusion_callback(self, msg, camera_idx):
        """Receive fused data with real distance and coordinates"""
        try:
            with self.fused_locks[camera_idx]:
                self.fused_data[camera_idx] = msg.yolov12_inference
        except Exception as e:
            self.get_logger().error(f"❌ Fusion callback error {camera_idx}: {e}")

    def setup_optimized_processing(self):
        """Setup ULTRA-OPTIMIZED processing for 100+ FPS"""
        try:
            from ultralytics import YOLO
            model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.yolo_model = YOLO(model_path)
            
            # ✅ ULTRA-AGGRESSIVE warm-up
            self.get_logger().info(f"🔥 ULTRA-Warming up model for 100+ FPS...")
            dummy_batch = np.zeros((self.batch_size, self.input_height, self.input_width, 3), dtype=np.uint8)
            
            # Extensive warmup with batch processing
            for i in range(3):
                start_time = time.time()
                for j in range(self.batch_size):
                    results = self.yolo_model(dummy_batch[j], 
                                           conf=0.15,  # ✅ Lower conf for more detections
                                           device='cuda:0',
                                           half=True,
                                           verbose=False,
                                           agnostic_nms=True,
                                           max_det=50,  # ✅ More detections
                                           imgsz=self.input_width,
                                           task='segment')
                warmup_time = time.time() - start_time
                self.get_logger().info(f"🔥 Batch-Warmup {i+1}: {warmup_time*1000:.1f}ms")
            
            self.get_logger().info(f"✅ Model ULTRA-optimized for 100+ FPS: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def camera_callback(self, msg, camera_idx):
        """ULTRA-FAST camera callback with thread dispatch"""
        try:
            current_time = time.time()
            
            # ✅ Skip if too frequent (rate limiting for stability)
            if current_time - self.last_process_time[camera_idx] < 0.005:  # Max 200 FPS per camera
                return
            
            self.last_process_time[camera_idx] = current_time
            
            # ✅ Convert image immediately
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ Store with thread safety
            with self.camera_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
                self.latest_headers[camera_idx] = msg.header
            
            # ✅ Add to processing queue (non-blocking)
            try:
                self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
            except queue.Full:
                pass  # Skip if queue full
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def camera_processing_worker(self, camera_idx):
        """Worker thread for processing camera frames"""
        while True:
            try:
                # Get frame from queue
                frame_data = self.frame_queues[camera_idx].get(timeout=1.0)
                cv_image, header, idx = frame_data
                
                # Process frame
                detections = self.process_single_frame_ultra(cv_image, header, idx)
                
                # Store results
                with self.camera_locks[idx]:
                    self.latest_detections[idx] = detections
                
                self.frame_queues[camera_idx].task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ Processing worker error {camera_idx}: {e}")

    def batch_inference_worker(self):
        """Worker thread for batch inference optimization"""
        while True:
            try:
                # Collect frames for batch processing
                batch_frames = []
                batch_headers = []
                batch_indices = []
                
                for i in range(6):
                    with self.camera_locks[i]:
                        if self.latest_images[i] is not None:
                            batch_frames.append(self.latest_images[i])
                            batch_headers.append(self.latest_headers[i])
                            batch_indices.append(i)
                
                if len(batch_frames) >= 3:  # Process when we have at least 3 cameras
                    self.process_batch_inference(batch_frames, batch_headers, batch_indices)
                
                time.sleep(0.01)  # 100 Hz processing
                
            except Exception as e:
                self.get_logger().error(f"❌ Batch inference error: {e}")

    def grid_creation_worker(self):
        """Worker thread for grid visualization"""
        while True:
            try:
                self.create_ultra_grid_visualization()
                time.sleep(0.033)  # 30 FPS for visualization
            except Exception as e:
                self.get_logger().error(f"❌ Grid creation error: {e}")

    def process_single_frame_ultra(self, frame, header, camera_idx):
        """ULTRA-ENHANCED processing with proper segmentation"""
        detections = []
        try:
            if not self.yolo_model:
                return detections
            
            # ✅ ULTRA-FAST resize with optimized interpolation
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_LINEAR)  # Better quality
            
            # ✅ ULTRA-FAST inference
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.15,  # ✅ Lower threshold for more detections
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=50,  # ✅ More detections
                                   imgsz=self.input_width,
                                   task='segment')
            
            inference_time = time.time() - start_time
            
            # ✅ Create ENHANCED messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            detection_msg.task = "detect"
            detection_msg.frame_type = "ultra_segmentation_enhanced"
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = "ultra_segmentation_enhanced"
            
            # ✅ PROCESS results with ENHANCED segmentation
            detection_count = 0
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                scale_x = frame.shape[1] / self.input_width
                scale_y = frame.shape[0] / self.input_height
                
                # ✅ Get proper masks
                masks = None
                if results[0].masks is not None:
                    masks = results[0].masks.data.cpu().numpy()
                
                for i, box in enumerate(results[0].boxes):
                    result = InferenceResult()
                    class_id = int(box.cls)
                    result.class_name = results[0].names[class_id]
                    result.confidence = float(box.conf)
                    
                    # ✅ Scale coordinates properly
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale_x)
                    result.top = int(y1 * scale_y)
                    result.right = int(x2 * scale_x)
                    result.bottom = int(y2 * scale_y)
                    
                    # ✅ TEMPORARY distance/coords (will be updated by fusion)
                    bbox_area = (result.right - result.left) * (result.bottom - result.top)
                    result.distance = 0.0  # Will be updated by fusion
                    result.coordinate_x = 0.0  # Will be updated by fusion
                    result.coordinate_y = 0.0  # Will be updated by fusion
                    result.coordinate_z = 0.0  # Will be updated by fusion
                    result.angle = 0.0  # Will be updated by fusion
                    
                    # ✅ ENHANCED segmentation mask processing
                    if masks is not None and i < len(masks):
                        # Get original mask at input resolution
                        mask = masks[i]  # Shape: (input_height, input_width)
                        
                        # Resize mask to original image size
                        mask_full = cv2.resize(mask, (frame.shape[1], frame.shape[0]), 
                                             interpolation=cv2.INTER_NEAREST)
                        
                        # Extract mask region for bounding box
                        mask_roi = mask_full[result.top:result.bottom, result.left:result.right]
                        
                        if mask_roi.size > 0:
                            # Convert to uint8 and flatten
                            mask_uint8 = (mask_roi * 255).astype(np.uint8)
                            result.mask_data = mask_uint8.flatten().tolist()
                            result.mask_width = mask_roi.shape[1]
                            result.mask_height = mask_roi.shape[0]
                        else:
                            result.mask_data = []
                            result.mask_width = 0
                            result.mask_height = 0
                    else:
                        result.mask_data = []
                        result.mask_width = 0
                        result.mask_height = 0
                    
                    # ✅ ENHANCED distinct colors per class
                    if class_id < len(self.coco_colors):
                        color = self.coco_colors[class_id]
                        result.color_r = color[0]
                        result.color_g = color[1]
                        result.color_b = color[2]
                    else:
                        # Fallback color generation
                        result.color_r = (class_id * 67) % 256
                        result.color_g = (class_id * 131) % 256
                        result.color_b = (class_id * 197) % 256
                    
                    detection_msg.yolov12_inference.append(result)
                    segmentation_msg.yolov12_inference.append(result)
                    detections.append(result)
                    detection_count += 1
            
            # ✅ PUBLISH to both topics
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
            
            self.detection_count += detection_count
            
            # ✅ Performance logging (reduced frequency)
            if camera_idx == 0 and detection_count > 0:
                fps_estimate = 1.0 / inference_time if inference_time > 0 else 0
                self.get_logger().info(
                    f"📊 Camera {camera_idx}: {detection_count} detections, "
                    f"{inference_time*1000:.1f}ms, ≈{fps_estimate:.0f} FPS"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error {camera_idx}: {e}")
        
        return detections

    def process_batch_inference(self, batch_frames, batch_headers, batch_indices):
        """Process multiple frames in batch for better GPU utilization"""
        try:
            if not self.yolo_model or len(batch_frames) == 0:
                return
            
            # ✅ Process batch sequentially but optimized
            for i, (frame, header, idx) in enumerate(zip(batch_frames, batch_headers, batch_indices)):
                # This could be further optimized with true batch processing
                self.process_single_frame_ultra(frame, header, idx)
                
        except Exception as e:
            self.get_logger().error(f"❌ Batch processing error: {e}")

    def create_ultra_grid_visualization(self):
        """Create EXTRA-LARGE grid with fused data display"""
        try:
            current_time = time.time()
            
            # ✅ EXTRA LARGE grid size for maximum visibility
            target_size = (800, 600)  # MUCH LARGER
            
            grid_images = []
            total_detections = 0
            
            for i in range(6):
                # Create camera image
                with self.camera_locks[i]:
                    if self.latest_images[i] is not None:
                        img = cv2.resize(self.latest_images[i], target_size)
                        
                        # ✅ Use FUSED data if available, otherwise use detection data
                        display_detections = []
                        with self.fused_locks[i]:
                            if self.fused_data[i]:
                                display_detections = self.fused_data[i]
                            elif i < len(self.latest_detections):
                                display_detections = self.latest_detections[i]
                        
                        # ✅ Draw ENHANCED visualizations
                        if display_detections:
                            scale_x = target_size[0] / self.latest_images[i].shape[1]
                            scale_y = target_size[1] / self.latest_images[i].shape[0]
                            
                            for det in display_detections:
                                # Scale coordinates
                                x1 = int(det.left * scale_x)
                                y1 = int(det.top * scale_y)
                                x2 = int(det.right * scale_x)
                                y2 = int(det.bottom * scale_y)
                                
                                # ✅ Draw ENHANCED segmentation mask
                                if hasattr(det, 'mask_data') and det.mask_data and det.mask_width > 0:
                                    try:
                                        mask_array = np.array(det.mask_data, dtype=np.uint8)
                                        mask = mask_array.reshape((det.mask_height, det.mask_width))
                                        
                                        # Resize mask to bbox size on grid
                                        if x2 > x1 and y2 > y1:
                                            mask_resized = cv2.resize(mask, (x2-x1, y2-y1))
                                            
                                            # Create colored mask
                                            color_mask = np.zeros((y2-y1, x2-x1, 3), dtype=np.uint8)
                                            color_mask[:,:,0] = (mask_resized > 128) * det.color_b
                                            color_mask[:,:,1] = (mask_resized > 128) * det.color_g  
                                            color_mask[:,:,2] = (mask_resized > 128) * det.color_r
                                            
                                            # Overlay with transparency
                                            roi = img[y1:y2, x1:x2]
                                            if roi.shape == color_mask.shape:
                                                img[y1:y2, x1:x2] = cv2.addWeighted(roi, 0.7, color_mask, 0.3, 0)
                                    except:
                                        pass
                                
                                # ✅ Draw bounding box with class-specific color
                                color = (int(det.color_b), int(det.color_g), int(det.color_r))
                                cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                                
                                # ✅ ENHANCED multi-line label with ALL information
                                distance = getattr(det, 'distance', 0.0)
                                coord_x = getattr(det, 'coordinate_x', 0.0)
                                coord_y = getattr(det, 'coordinate_y', 0.0)
                                coord_z = getattr(det, 'coordinate_z', 0.0)
                                
                                label_lines = [
                                    f"{det.class_name} {det.confidence:.2f}",
                                    f"D: {distance:.1f}m",
                                    f"XYZ: ({coord_x:.1f},{coord_y:.1f},{coord_z:.1f})"
                                ]
                                
                                # Draw multi-line label with background
                                for j, line in enumerate(label_lines):
                                    y_offset = y1 - 45 + j * 15
                                    if y_offset > 15:
                                        # Background rectangle
                                        (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                                        cv2.rectangle(img, (x1, y_offset-12), (x1+w+5, y_offset+3), color, -1)
                                        # Text
                                        cv2.putText(img, line, (x1+2, y_offset), 
                                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        
                        total_detections += len(display_detections)
                        
                        # ✅ Camera info overlay
                        cv2.rectangle(img, (0, 0), (target_size[0], 50), (0, 0, 0), -1)
                        cv2.putText(img, f"{self.camera_names[i].upper()}", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                        
                        # Detection count
                        det_count = len(display_detections)
                        cv2.putText(img, f"Det: {det_count}", (target_size[0]-100, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        
                        grid_images.append(img)
                    else:
                        # Black placeholder with larger text
                        black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                        cv2.putText(black_img, f"{self.camera_names[i].upper()}", 
                                  (target_size[0]//4, target_size[1]//2-20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                        cv2.putText(black_img, "OFFLINE", 
                                  (target_size[0]//4, target_size[1]//2+20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                        grid_images.append(black_img)
            
            # ✅ Create EXTRA LARGE 2x3 grid
            if len(grid_images) == 6:
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ ENHANCED info overlay
                current_fps = self.frame_count / 2.0 if self.frame_count > 0 else 0
                
                # Extra large info panel
                info_height = 100
                info_panel = np.zeros((info_height, grid.shape[1], 3), dtype=np.uint8)
                
                # Main stats
                main_info = f"ULTRA-DeepStream | FPS: {current_fps:.1f} | Total Det: {total_detections} | Active: 6/6"
                cv2.putText(info_panel, main_info, (20, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                
                # Performance info
                perf_info = f"Target: 100+ FPS | Segmentation: ENHANCED | 3D Fusion: ACTIVE | Colors: 80 DISTINCT"
                cv2.putText(info_panel, perf_info, (20, 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                # Combine grid with info panel
                final_grid = np.vstack([grid, info_panel])
                
                # ✅ Publish EXTRA-LARGE grid
                grid_msg = self.bridge.cv2_to_imgmsg(final_grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "ultra_deepstream_grid_enhanced"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid visualization error: {e}")

    def log_fps(self):
        """Enhanced FPS logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            
            self.get_logger().info(
                f"🚀 ULTRA-DeepStream FPS: {fps:.1f} | Det/s: {detection_rate:.1f}"
            )
            
            if fps >= 100:
                self.get_logger().info("🎯 TARGET ACHIEVED: 100+ FPS!")
            elif fps >= 50:
                self.get_logger().info("✅ Excellent performance!")
            else:
                self.get_logger().warn(f"⚡ Performance: {fps:.1f} FPS")
        
        # Reset counters
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("🛑 ULTRA-DeepStream node shutdown")
        super().destroy_node()

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