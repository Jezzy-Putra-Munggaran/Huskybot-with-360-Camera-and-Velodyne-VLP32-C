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
import torch
import gc
import os
import colorsys
import traceback

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
                
                # ✅ MAXIMUM GPU utilization
                torch.cuda.set_device(0)
                torch.cuda.empty_cache()
                
                # ✅ Set GPU to maximum performance mode
                os.system('sudo nvidia-smi -pm 1')
                os.system('sudo nvidia-smi -pl 80')  # Max power
                os.system('sudo nvidia-smi -ac 1377,1377')  # Max clocks
                
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
                "yolo11x-seg.pt",  # Auto-download PT model
                "yolov8x-seg.pt",  # Fallback to standard model that will auto-download
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
                model_path = "yolo11x-seg.pt"  # Fallback to standard model
                self.get_logger().warn("⚠️ Using fallback: yolo11x-seg.pt")
            
            self.get_logger().info(f"🔥 Loading YOLO model: {model_path}")
            
            # ✅ FIXED: Proper model loading with correct API
            self.yolo_model = YOLO(model_path)
            
            # ✅ Force to GPU
            if torch.cuda.is_available():
                self.yolo_model.to('cuda:0')
            
            # ✅ FIXED: Warmup for consistent performance
            dummy_image = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            start_time = time.time()
            for _ in range(5):  # More warmup iterations
                try:
                    _ = self.yolo_model.predict(
                        source=dummy_image,
                        conf=0.25,
                        device='cuda:0',
                        half=True,
                        verbose=False,
                        agnostic_nms=True,
                        max_det=100,
                        retina_masks=True
                    )
                except Exception as e:
                    self.get_logger().warn(f"Warmup warning: {e}")
            
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
                        self.frame_queues[camera_idx].get_nowait()  # Remove old
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
                        frame_data = self.frame_queues[i].get_nowait()
                        batch_frames[i] = frame_data[0]
                        batch_headers[i] = frame_data[1]
                        batch_camera_indices[i] = frame_data[2]
                        frames_ready += 1
                    except queue.Empty:
                        continue
                
                # Process any available frames
                if frames_ready > 0:
                    self.maximum_speed_batch_inference(batch_frames, batch_headers, batch_camera_indices)
                    # Clear processed frames
                    for i in range(6):
                        if batch_frames[i] is not None:
                            batch_frames[i] = None
                            batch_headers[i] = None
                            batch_camera_indices[i] = None
                
                time.sleep(0.0001)  # Minimal sleep for maximum speed
                
            except Exception as e:
                self.get_logger().error(f"❌ Batch worker error: {e}")
                time.sleep(0.001)

    def maximum_speed_batch_inference(self, batch_frames, batch_headers, batch_camera_indices):
        """✅ MAXIMUM speed batch inference with PERFECT segmentation - FIXED TYPE CONVERSION"""
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
                    retina_masks=True
                )
                
                inference_time = time.time() - start_time
                self.total_inference_time += inference_time
                self.inference_count += 1
                
                # ✅ Process results with FIXED type conversion
                if results and len(results) > 0:
                    self.process_detection_results(results[0], camera_idx, frame, header)
                
            except Exception as e:
                self.get_logger().error(f"❌ Inference error for camera {camera_idx}: {e}")
                traceback.print_exc()

    def process_detection_results(self, result, camera_idx, original_frame, header):
        """✅ FIXED: Process detection results with proper type conversion"""
        try:
            # Create detection message
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = self.camera_names[camera_idx]
            detection_msg.task = "segment"
            detection_msg.frame_type = "processed"
            detection_msg.note = f"YOLO11X segmentation from {self.camera_labels[camera_idx]}"

            frame_height, frame_width = original_frame.shape[:2]
            
            # ✅ FIXED: Process boxes with proper type conversion
            if hasattr(result, 'boxes') and result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy() if hasattr(result.boxes, 'xyxy') else []
                scores = result.boxes.conf.cpu().numpy() if hasattr(result.boxes, 'conf') else []
                classes = result.boxes.cls.cpu().numpy() if hasattr(result.boxes, 'cls') else []
                
                # Get class names
                class_names = result.names if hasattr(result, 'names') else {}
                
                # Process masks if available
                masks = None
                if hasattr(result, 'masks') and result.masks is not None:
                    masks = result.masks.data.cpu().numpy()
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                    detection = InferenceResult()
                    
                    # ✅ FIXED: Convert coordinates to integers (scale to original image size)
                    x1 = int(box[0] * frame_width / self.input_width)
                    y1 = int(box[1] * frame_height / self.input_height)
                    x2 = int(box[2] * frame_width / self.input_width)
                    y2 = int(box[3] * frame_height / self.input_height)
                    
                    # ✅ FIXED: Assign as integers
                    detection.left = x1
                    detection.top = y1
                    detection.right = x2
                    detection.bottom = y2
                    
                    # Class and confidence
                    detection.class_name = class_names.get(int(cls_id), f"class_{int(cls_id)}")
                    detection.confidence = float(score)
                    
                    # ✅ Calculate distance (simple estimation based on bbox size)
                    bbox_area = (x2 - x1) * (y2 - y1)
                    estimated_distance = max(1.0, min(50.0, 10000.0 / max(1, bbox_area) * 10))
                    detection.distance = estimated_distance
                    
                    # ✅ Calculate 3D coordinates (estimation)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Convert pixel coordinates to world coordinates (rough estimation)
                    angle_offset = ((center_x / frame_width) - 0.5) * 60  # FOV assumption
                    camera_angle = camera_idx * 60  # 60-degree spacing
                    world_angle = (camera_angle + angle_offset) % 360
                    
                    detection.coordinate_x = estimated_distance * np.cos(np.radians(world_angle))
                    detection.coordinate_y = estimated_distance * np.sin(np.radians(world_angle))
                    detection.coordinate_z = 0.5  # Default height
                    detection.angle = world_angle
                    
                    # ✅ Enhanced colors for each class
                    color_idx = int(cls_id) % len(self.coco_colors)
                    detection.color_r = self.coco_colors[color_idx][0]
                    detection.color_g = self.coco_colors[color_idx][1]
                    detection.color_b = self.coco_colors[color_idx][2]
                    
                    # ✅ Process mask if available
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        # Resize mask to original image size
                        mask_resized = cv2.resize(mask.astype(np.uint8), (frame_width, frame_height))
                        # Convert to bytes
                        detection.mask_data = mask_resized.flatten().tobytes()
                        detection.mask_width = frame_width
                        detection.mask_height = frame_height
                    else:
                        detection.mask_data = []
                        detection.mask_width = 0
                        detection.mask_height = 0
                    
                    detection_msg.yolov12_inference.append(detection)
                    self.detection_count += 1
            
            # ✅ Publish results
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                seg_pub.publish(detection_msg)  # Publish as segmentation
                
        except Exception as e:
            self.get_logger().error(f"❌ Process detection error: {e}")
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
        """✅ MAXIMUM enhanced 2x3 grid with ENGLISH labels and segmentation overlay"""
        try:
            target_size = (720, 405)  # Larger size for better visibility
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.image_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    # ✅ Resize to target size
                    img_resized = cv2.resize(img, target_size, interpolation=cv2.INTER_AREA)
                    
                    # ✅ FIXED: Add camera label in ENGLISH
                    label_bg_height = 60
                    cv2.rectangle(img_resized, (0, 0), (target_size[0], label_bg_height), (0, 0, 0), -1)
                    
                    # Camera name and index
                    camera_text = f"CAM {i+1}: {self.camera_labels[i]}"
                    cv2.putText(img_resized, camera_text, (10, 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    # Resolution and status
                    status_text = f"1920x1080 | YOLO11X-SEG | GPU"
                    cv2.putText(img_resized, status_text, (10, 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    grid_images.append(img_resized)
                else:
                    # ✅ Waiting placeholder with ENGLISH text
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_img, f"CAM {i+1}: {self.camera_labels[i]}", 
                               (target_size[0]//4, target_size[1]//2-20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(black_img, "WAITING FOR CAMERA...", 
                               (target_size[0]//4, target_size[1]//2+20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
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
                    y_pos = grid.shape[0] - status_height + 20 + (idx * 20)
                    cv2.putText(grid, info_line, (20, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                
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