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
                torch.cuda.set_per_process_memory_fraction(0.95)
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.deterministic = False
                
                os.system('sudo jetson_clocks')
                os.system('sudo nvpmodel -m 0')
                
                self.get_logger().info("🔥 MAXIMUM Jetson optimization activated!")
                
        except Exception as e:
            self.get_logger().warn(f"Jetson optimization warning: {e}")

    def setup_maximum_optimized_model(self):
        """✅ FIXED: Proper YOLO11X model loading"""
        try:
            from ultralytics import YOLO
            
            # ✅ FIXED: Correct model candidates
            model_candidates = [
                f"/home/kmp-orin/jezzy/huskybot/yolo11x-seg.engine",
                f"/home/kmp-orin/jezzy/huskybot/yolo11x.engine", 
                f"/home/kmp-orin/jezzy/huskybot/yolo11x-seg.pt",
                f"/home/kmp-orin/jezzy/huskybot/yolo11x.pt",
                "./yolo11x-seg.engine",
                "./yolo11x-seg.pt",
                "./yolo11x.pt",
                f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}",
                "yolo11x-seg.pt",
                "yolo11x.pt",
                "yolo11n-seg.pt"
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
                model_path = "yolo11x-seg.pt"
                self.get_logger().warn("⚠️ Using fallback: yolo11x-seg.pt")
            
            self.get_logger().info(f"🔥 Loading YOLO11X model: {model_path}")
            
            # ✅ FIXED: Proper model loading
            self.yolo_model = YOLO(model_path)
            
            # ✅ FIXED: Proper model configuration
            if hasattr(self.yolo_model, 'model') and self.yolo_model.model is not None:
                self.yolo_model.model.eval()
                if torch.cuda.is_available():
                    self.yolo_model.model = self.yolo_model.model.cuda()
                    if hasattr(self.yolo_model.model, 'half'):
                        self.yolo_model.model = self.yolo_model.model.half()
            
            # ✅ FIXED: Proper warmup
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            start_time = time.time()
            for _ in range(3):
                try:
                    results = self.yolo_model(dummy_image,
                                           conf=0.35,
                                           device='cuda:0',
                                           half=True,
                                           verbose=False,
                                           task='segment')
                except Exception as e:
                    self.get_logger().warn(f"Warmup warning: {e}")
                    break
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ YOLO11X Model ready: {warmup_time*1000:.1f}ms")
            
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
                        batch_frames[i], batch_headers[i], _ = frame_data
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
                    results = self.yolo_model(resized, 
                                           conf=0.35,
                                           device='cuda:0',
                                           half=True,
                                           verbose=False,
                                           task='segment')
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
                cv2.putText(display_frame, f"CAM {camera_idx+1}: {camera_label}", 
                          (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                cv2.putText(display_frame, f"FPS: {1000/max(1, inference_time*1000):.1f} | Objects: {len(results[0].boxes) if results[0].boxes is not None else 0}", 
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
                        
                        # ✅ FIXED: Camera angle mapping (ENGLISH)
                        camera_base_angles = {0: 180, 1: 225, 2: 315, 3: 0, 4: 45, 5: 135}
                        base_angle = camera_base_angles.get(camera_idx, 0)
                        
                        bbox_center_x = (result.left + result.right) / 2
                        image_width = frame.shape[1]
                        angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                        object_angle = (base_angle + angle_offset) % 360
                        result.angle = object_angle
                        
                        # ✅ ENHANCED distance estimation
                        bbox_height = result.bottom - result.top
                        if result.class_name in ['person']:
                            estimated_distance = max(1.0, 1200.0 / bbox_height)
                        elif result.class_name in ['car', 'truck', 'bus']:
                            estimated_distance = max(2.0, 1800.0 / bbox_height)
                        elif result.class_name in ['bicycle', 'motorcycle']:
                            estimated_distance = max(1.0, 900.0 / bbox_height)
                        else:
                            estimated_distance = max(1.0, 700.0 / bbox_height)
                        
                        result.distance = min(estimated_distance, 50.0)
                        
                        # ✅ Calculate 3D coordinates
                        angle_rad = np.radians(object_angle)
                        result.coordinate_x = result.distance * np.cos(angle_rad)
                        result.coordinate_y = result.distance * np.sin(angle_rad)
                        result.coordinate_z = 0.5
                        
                        # ✅ Get DISTINCT color
                        color = self.coco_colors[class_id] if class_id < len(self.coco_colors) else [255, 255, 255]
                        text_color = self.text_colors[class_id] if class_id < len(self.text_colors) else (255, 255, 255)
                        
                        # ✅ PERFECT mask processing
                        if masks is not None and i < len(masks):
                            mask = masks[i]
                            mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                            mask_colored = np.zeros_like(frame)
                            mask_colored[:, :] = color
                            mask_overlay = cv2.bitwise_and(mask_colored, mask_colored, mask=(mask_resized > 0.5).astype(np.uint8))
                            display_frame = cv2.addWeighted(display_frame, 1.0, mask_overlay, 0.3, 0)
                        
                        # ✅ ENHANCED bounding box
                        cv2.rectangle(display_frame, (result.left, result.top), 
                                    (result.right, result.bottom), color, 4)
                        
                        # ✅ FIXED: English information display
                        info_lines = [
                            f"Camera: {camera_label}",
                            f"Class: {result.class_name}, Conf: {result.confidence:.2f}",
                            f"Distance: {result.distance:.1f}m",
                            f"Coordinate: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})"
                        ]
                        
                        text_y = max(100, result.top - 15)
                        for line_idx, info_line in enumerate(info_lines):
                            line_y = text_y + (line_idx * 25)
                            
                            # Background for text
                            text_size = cv2.getTextSize(info_line, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                            cv2.rectangle(display_frame, 
                                        (result.left - 5, line_y - 20), 
                                        (result.left + text_size[0] + 10, line_y + 5), 
                                        color, -1)
                            
                            cv2.putText(display_frame, info_line, 
                                      (result.left, line_y), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
                        
                        detection_msg.yolov12_inference.append(result)
                        segmentation_msg.yolov12_inference.append(result)
                        self.detection_count += 1
                        
                        # ✅ FIXED: English terminal output
                        self.get_logger().info(
                            f"Camera: {camera_label} | Class: {result.class_name} | "
                            f"Conf: {result.confidence:.2f} | Distance: {result.distance:.1f}m | "
                            f"Coordinate: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})"
                        )
                
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
        """✅ MAXIMUM enhanced 2x3 grid with ENGLISH labels"""
        try:
            target_size = (960, 540)
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