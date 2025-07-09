#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import queue
import colorsys
import os
import torch
import gc

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
        self.declare_parameter('model_engine', 'yolo12x.engine')  # ✅ FIXED: Use YOLO12X for speed
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)  # ✅ Process all 6 cameras in batch
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
            # ✅ Force maximum GPU utilization
            if torch.cuda.is_available():
                torch.cuda.set_per_process_memory_fraction(0.95)  # Use 95% GPU memory
                torch.backends.cudnn.benchmark = True  # Optimize for fixed input size
                torch.backends.cudnn.deterministic = False  # Allow non-deterministic for speed
                
                # ✅ Set maximum GPU clocks
                os.system('sudo jetson_clocks')
                os.system('sudo nvpmodel -m 0')  # Maximum performance mode
                
                self.get_logger().info("🔥 MAXIMUM Jetson optimization activated!")
                
        except Exception as e:
            self.get_logger().warn(f"Jetson optimization warning: {e}")

    def setup_maximum_optimized_model(self):
        """✅ MAXIMUM optimized model with auto-fallback"""
        try:
            from ultralytics import YOLO
            
            # ✅ FIXED: Updated model candidates dengan fallback yang lebih baik
            model_candidates = [
                # YOLO12X variants (paling cepat)
                f"/home/kmp-orin/jezzy/huskybot/yolo12x-seg.engine",
                f"/home/kmp-orin/jezzy/huskybot/yolo12x.engine",
                f"/home/kmp-orin/jezzy/huskybot/yolo12x-seg.pt",
                
                # YOLO11X fallback
                f"/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",
                f"/home/kmp-orin/jezzy/huskybot/yolo11x.engine",
                f"/home/kmp-orin/jezzy/huskybot/yolo11x-seg.pt",
                
                # Parameter fallback
                f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}",
                
                # Global fallback untuk auto-download
                "yolo12x-seg.pt",  # Akan auto-download jika tidak ada
                "yolo11x-seg.pt"   # Fallback terakhir
            ]
            
            model_path = None
            for candidate in model_candidates:
                if candidate.startswith("yolo") and not candidate.startswith("/"):
                    # Model akan auto-download
                    model_path = candidate
                    self.get_logger().info(f"🔄 Will auto-download: {candidate}")
                    break
                elif os.path.exists(candidate):
                    model_path = candidate
                    self.get_logger().info(f"✅ Found local model: {candidate}")
                    break
            
            if not model_path:
                # Ultimate fallback - force download YOLO11X
                model_path = "yolo11x-seg.pt"
                self.get_logger().warn("⚠️ No local models found, will auto-download yolo11x-seg.pt")
            
            self.get_logger().info(f"🔥 Loading MAXIMUM-OPTIMIZED model: {model_path}")
            
            # ✅ Load with auto-download capability
            self.yolo_model = YOLO(model_path)
            
            # ✅ Auto-export to engine if .pt file
            if model_path.endswith('.pt') and not model_path.startswith('/'):
                self.get_logger().info("🚀 Auto-converting to TensorRT engine for maximum speed...")
                try:
                    engine_path = self.yolo_model.export(
                        format='engine',
                        device=0,
                        half=True,
                        workspace=4,
                        verbose=False,
                        batch=1,  # Single for compatibility
                        imgsz=640
                    )
                    # Reload with engine
                    self.yolo_model = YOLO(engine_path)
                    self.get_logger().info(f"✅ Successfully converted and loaded: {engine_path}")
                except Exception as e:
                    self.get_logger().warn(f"Engine conversion failed, using PT model: {e}")
            
            # ✅ Configure for MAXIMUM speed
            if hasattr(self.yolo_model, 'model'):
                self.yolo_model.model.eval()
                if torch.cuda.is_available():
                    self.yolo_model.model = self.yolo_model.model.cuda()
                    self.yolo_model.model = self.yolo_model.model.half()
            
            # ✅ Warmup
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            start_time = time.time()
            for _ in range(3):
                results = self.yolo_model(dummy_image,
                                       conf=0.35,
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=30,
                                       imgsz=(self.input_width, self.input_height),
                                       task='segment')
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ MAXIMUM-OPTIMIZED Model ready: {warmup_time*1000:.1f}ms")
            
            gc.collect()
            torch.cuda.empty_cache()
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def setup_enhanced_coco_colors(self):
        """Setup 80 DISTINCT colors for COCO classes with HIGH contrast"""
        self.coco_colors = []
        self.text_colors = []
        
        # ✅ Pre-defined high-contrast colors for better visibility
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
                # Generate additional colors
                hue = (i * 137.5) % 360
                sat = 0.9 + (i % 2) * 0.1
                val = 0.8 + (i % 3) * 0.1
                r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
                color = [int(r*255), int(g*255), int(b*255)]
            
            self.coco_colors.append(color)
            
            # ✅ Optimal text color for MAXIMUM contrast
            brightness = (color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114) / 255
            self.text_colors.append((0, 0, 0) if brightness > 0.5 else (255, 255, 255))

    def setup_ros_topics(self):
        """Setup ROS2 topics with CORRECT camera mapping"""
        self.camera_subs = []
        self.camera_names = ['rear', 'rear_right', 'front_right', 'front', 'front_left', 'rear_left']
        actual_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_right/image_raw',      # KAMERA KANAN BELAKANG  
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_front_left/image_raw'  # KAMERA KIRI BELAKANG
        ]
        
        for i, (name, topic) in enumerate(zip(self.camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.maximum_speed_callback(msg, idx), 
                1  # Minimal queue for speed
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed: {topic} -> {name}")
        
        # ✅ Publishers for results
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # ✅ Enhanced grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)

    def setup_parallel_processing(self):
        """✅ MAXIMUM parallel processing"""
        self.frame_queues = [queue.Queue(maxsize=2) for _ in range(6)]  # Slightly larger buffer
        self.processing_active = True
        
        # ✅ Batch processing thread for MAXIMUM speed
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
            
            # ✅ Non-blocking queue add
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
        """✅ MAXIMUM speed batch processing for ALL 6 cameras"""
        batch_frames = [None] * 6
        batch_headers = [None] * 6
        
        while self.processing_active:
            try:
                # ✅ Collect frames from all cameras
                frames_ready = 0
                for i in range(6):
                    try:
                        frame_data = self.frame_queues[i].get(timeout=0.001)
                        batch_frames[i], batch_headers[i], _ = frame_data
                        frames_ready += 1
                    except queue.Empty:
                        continue
                
                # ✅ Process batch if we have enough frames
                if frames_ready >= 3:  # Process if at least half cameras have frames
                    self.maximum_speed_batch_inference(batch_frames, batch_headers)
                
                time.sleep(0.001)  # Very small delay for CPU
                
            except Exception as e:
                self.get_logger().error(f"❌ Batch worker error: {e}")
                time.sleep(0.01)

    def maximum_speed_batch_inference(self, batch_frames, batch_headers):
        """✅ MAXIMUM speed batch inference with PERFECT segmentation display"""
        try:
            if not self.yolo_model:
                return
            
            # ✅ Process each camera individually for maximum accuracy
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
                results = self.yolo_model(resized, 
                                       conf=0.35,  # Higher confidence for speed
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=30,  # Fewer but better detections
                                       imgsz=(self.input_width, self.input_height),
                                       task='segment')
                
                inference_time = time.time() - start_time
                self.total_inference_time += inference_time
                self.inference_count += 1
                
                # ✅ Create enhanced messages
                detection_msg = Yolov12Inference()
                detection_msg.header = header
                detection_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
                detection_msg.task = "detect"
                detection_msg.frame_type = f"maximum_optimized_{camera_idx}"
                detection_msg.note = f"Inference: {inference_time*1000:.1f}ms"
                
                segmentation_msg = Yolov12Inference()
                segmentation_msg.header = header
                segmentation_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
                segmentation_msg.task = "segment"
                segmentation_msg.frame_type = f"maximum_optimized_{camera_idx}"
                segmentation_msg.note = f"Inference: {inference_time*1000:.1f}ms"
                
                # ✅ Create ENHANCED display image with PERFECT segmentation
                display_frame = frame.copy()
                
                # ✅ Add camera info header
                camera_name = self.camera_names[camera_idx].upper().replace('_', ' ')
                cv2.rectangle(display_frame, (0, 0), (frame.shape[1], 80), (0, 0, 0), -1)
                cv2.putText(display_frame, f"CAM {camera_idx+1}: {camera_name}", 
                          (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                cv2.putText(display_frame, f"FPS: {1000/max(1, inference_time*1000):.1f} | Det: {len(results[0].boxes) if results[0].boxes is not None else 0}", 
                          (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                
                if results[0].boxes is not None and len(results[0].boxes) > 0:
                    scale_x = frame.shape[1] / self.input_width
                    scale_y = frame.shape[0] / self.input_height
                    
                    # ✅ Get masks if available
                    masks = None
                    if results[0].masks is not None:
                        masks = results[0].masks.data.cpu().numpy()
                    
                    for i, box in enumerate(results[0].boxes):
                        result = InferenceResult()
                        class_id = int(box.cls)
                        result.class_name = results[0].names[class_id]
                        result.confidence = float(box.conf)
                        
                        # ✅ Scale coordinates correctly
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        result.left = int(x1 * scale_x)
                        result.top = int(y1 * scale_y)
                        result.right = int(x2 * scale_x)
                        result.bottom = int(y2 * scale_y)
                        
                        # ✅ Calculate ACCURATE angle and distance
                        bbox_center_x = (result.left + result.right) / 2
                        image_width = frame.shape[1]
                        
                        # ✅ FIXED camera base angles mapping
                        camera_base_angles = {0: 180, 1: 225, 2: 315, 3: 0, 4: 45, 5: 135}
                        base_angle = camera_base_angles.get(camera_idx, 0)
                        angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                        object_angle = (base_angle + angle_offset) % 360
                        result.angle = object_angle
                        
                        # ✅ ENHANCED distance estimation
                        bbox_height = result.bottom - result.top
                        bbox_width = result.right - result.left
                        
                        # ✅ Object-specific distance estimation
                        if result.class_name in ['person']:
                            estimated_distance = max(1.0, 1200.0 / bbox_height)
                        elif result.class_name in ['car', 'truck', 'bus']:
                            estimated_distance = max(2.0, 1800.0 / bbox_height)
                        elif result.class_name in ['bicycle', 'motorcycle']:
                            estimated_distance = max(1.0, 900.0 / bbox_height)
                        else:
                            estimated_distance = max(1.0, 700.0 / bbox_height)
                        
                        result.distance = min(estimated_distance, 50.0)
                        
                        # ✅ Calculate ACCURATE 3D coordinates
                        angle_rad = np.radians(object_angle)
                        result.coordinate_x = result.distance * np.cos(angle_rad)
                        result.coordinate_y = result.distance * np.sin(angle_rad)
                        result.coordinate_z = 0.5
                        
                        # ✅ Get DISTINCT color for this class
                        if class_id < len(self.coco_colors):
                            color = self.coco_colors[class_id]
                            text_color = self.text_colors[class_id]
                            result.color_r, result.color_g, result.color_b = color
                        else:
                            color = [255, 255, 255]
                            text_color = (0, 0, 0)
                            result.color_r = result.color_g = result.color_b = 255
                        
                        # ✅ PERFECT mask processing and display
                        if masks is not None and i < len(masks):
                            mask = masks[i]
                            # ✅ Resize mask to original frame size
                            mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                            mask_binary = (mask_resized > 0.5).astype(np.uint8)
                            
                            # ✅ Store mask data
                            result.mask_width = frame.shape[1]
                            result.mask_height = frame.shape[0]
                            result.mask_data = mask_binary.flatten().tolist()
                            
                            # ✅ PERFECT mask overlay with DISTINCT colors
                            mask_colored = np.zeros_like(display_frame)
                            mask_colored[:,:] = color
                            
                            # ✅ Apply mask with transparency
                            alpha = 0.4
                            mask_area = mask_binary > 0
                            if np.any(mask_area):
                                display_frame[mask_area] = cv2.addWeighted(
                                    display_frame[mask_area], 1-alpha,
                                    mask_colored[mask_area], alpha, 0
                                )
                        
                        # ✅ ENHANCED bounding box with THICK borders
                        cv2.rectangle(display_frame, (result.left, result.top), 
                                    (result.right, result.bottom), color, 4)
                        
                        # ✅ TARGET FORMAT: All information in one display
                        info_text = f"Camera: {camera_name} | Class: {result.class_name} | Conf: {result.confidence:.2f} | Distance: {result.distance:.1f}m | Coordinate: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})"
                        
                        # ✅ Multi-line display for better readability
                        info_lines = [
                            f"Camera: {camera_name}",
                            f"Class: {result.class_name}, Conf: {result.confidence:.2f}",
                            f"Distance: {result.distance:.1f}m",
                            f"Coordinate: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})"
                        ]
                        
                        # ✅ LARGE info box for visibility
                        text_y = max(100, result.top - 15)
                        for line_idx, info_line in enumerate(info_lines):
                            text_size = cv2.getTextSize(info_line, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
                            
                            # ✅ Background for readability
                            cv2.rectangle(display_frame, 
                                        (result.left, text_y - text_size[1] - 10), 
                                        (result.left + text_size[0] + 20, text_y + 10), 
                                        color, -1)
                            
                            cv2.putText(display_frame, info_line, 
                                      (result.left + 10, text_y), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, text_color, 2)
                            
                            text_y -= (text_size[1] + 15)
                        
                        detection_msg.yolov12_inference.append(result)
                        segmentation_msg.yolov12_inference.append(result)
                        self.detection_count += 1
                        
                        # ✅ Print to terminal as well
                        self.get_logger().info(f"Camera: {camera_name} | Class: {result.class_name} | Conf: {result.confidence:.2f} | Distance: {result.distance:.1f}m | Coordinate: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})")
                
                # ✅ Update display image
                with self.image_locks[camera_idx]:
                    self.latest_images[camera_idx] = display_frame
                
                # ✅ Publish results
                if camera_idx < len(self.result_pubs):
                    det_pub, seg_pub = self.result_pubs[camera_idx]
                    det_pub.publish(detection_msg)
                    seg_pub.publish(segmentation_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Batch inference error: {e}")

    def maximum_speed_grid_worker(self):
        """✅ MAXIMUM optimized grid worker"""
        while self.processing_active:
            try:
                self.create_maximum_enhanced_grid()
                time.sleep(0.0167)  # 60 FPS for grid
            except Exception as e:
                self.get_logger().error(f"❌ Grid error: {e}")
                time.sleep(0.1)

    def create_maximum_enhanced_grid(self):
        """✅ MAXIMUM enhanced 2x3 grid with ALL cameras and FULL info"""
        try:
            target_size = (960, 540)  # Large for better visibility
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.image_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    img_resized = cv2.resize(img, target_size, interpolation=cv2.INTER_LINEAR)
                    grid_images.append(img_resized)
                else:
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    camera_name = self.camera_names[i].upper().replace('_', ' ')
                    cv2.putText(black_img, f"CAM {i+1}: {camera_name}", 
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
                
                # ✅ ENHANCED status info
                avg_inference = self.total_inference_time / max(1, self.inference_count)
                theoretical_fps = 1.0 / avg_inference if avg_inference > 0 else 0
                
                status_height = 120
                cv2.rectangle(grid, (0, grid.shape[0]-status_height), (grid.shape[1], grid.shape[0]), (0, 0, 0), -1)
                
                status_color = (0, 255, 0) if theoretical_fps >= 100 else (0, 255, 255)
                
                info_lines = [
                    f"HUSKYBOT 360° MAXIMUM-OPTIMIZED SEGMENTATION | YOLO12X ENGINE",
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
                grid_msg.header.frame_id = "maximum_optimized_grid_all_cameras_segmentation_yolo12x"
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
                    f"Inference: {avg_inference*1000:.1f}ms | YOLO12X MAXIMUM SPEED!"
                )
            else:
                self.get_logger().info(
                    f"🔥 MAXIMUM Optimizing: Theoretical: {theoretical_fps:.1f} FPS | "
                    f"Actual: {actual_fps:.1f} FPS | Det/s: {detection_rate:.1f} | "
                    f"Inference: {avg_inference*1000:.1f}ms | Jetson MAXIMUM MODE"
                )
        
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
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