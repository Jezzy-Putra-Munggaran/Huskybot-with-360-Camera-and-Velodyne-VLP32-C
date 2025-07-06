#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os
import sys
import traceback
import queue
from concurrent.futures import ThreadPoolExecutor

# Import YOLO
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    print("[ERROR] Ultralytics YOLO tidak tersedia. Install dengan: pip install ultralytics")
    ULTRALYTICS_AVAILABLE = False
    YOLO = None

class MulticamSegmentationNode(Node):
    def __init__(self):
        super().__init__('multicam_segmentation_node')
        
        # Initialize basic components
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Setup parameters
        self._setup_parameters()
        
        # Initialize model
        self._initialize_model()
        
        # Setup threading for ultra high performance
        self._setup_threading()
        
        # Setup subscriptions and publishers
        self._setup_topics()
        
        # Initialize visualization
        self._setup_visualization()
        
        # Initialize timers
        self._setup_timers()
        
        # Statistics
        self.stats = {
            'total_frames': 0,
            'successful_segmentations': 0,
            'failed_segmentations': 0,
            'average_inference_time': 0.0,
            'fps': 0.0,
            'viz_fps': 0.0
        }
        
        # COCO class colors for segmentation - More vibrant colors
        self._setup_class_colors()
        
        self.get_logger().info("🚀 Optimized YOLOv11 Segmentation Node initialized")

    def _setup_class_colors(self):
        """Setup vibrant colors for COCO classes"""
        # Predefined vibrant colors for better visibility
        vibrant_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128), (128, 128, 0),
            (128, 0, 128), (0, 128, 128), (255, 128, 0), (255, 0, 128), (128, 255, 0),
            (0, 255, 128), (128, 0, 255), (0, 128, 255), (255, 64, 64), (64, 255, 64)
        ]
        
        # Extend with more colors
        self.class_colors = vibrant_colors * 4  # Repeat for 80 classes
        
        # Ensure we have exactly 80 colors
        while len(self.class_colors) < 80:
            np.random.seed(len(self.class_colors))
            color = [int(x) for x in np.random.randint(50, 255, 3)]
            self.class_colors.append(tuple(color))

    def _setup_parameters(self):
        """Setup optimized parameters"""
        try:
            # Core parameters
            self.declare_parameter('cam_count', 6)
            self.declare_parameter('model_path', 'yolo11x-seg.engine')
            self.declare_parameter('device', 'cuda:0')
            self.declare_parameter('conf_thres', 0.3)  # Slightly higher for quality
            self.declare_parameter('visualization_enabled', True)
            self.declare_parameter('publish_rate', 10.0)  # More realistic target
            self.declare_parameter('image_width', 1920)
            self.declare_parameter('image_height', 1080)
            
            # OPTIMIZED Performance parameters
            self.declare_parameter('inference_threads', 3)  # Reduce threads for stability
            self.declare_parameter('input_size', 480)  # Smaller input for speed
            self.declare_parameter('half_precision', True)
            self.declare_parameter('batch_size', 1)
            self.declare_parameter('max_det', 50)  # Limit detections for speed
            
            # LARGER Visualization parameters
            self.declare_parameter('viz_scale', 0.8)  # Much larger visualization
            self.declare_parameter('viz_fps_limit', 15.0)  # Lower viz FPS for performance
            self.declare_parameter('show_fps', True)
            self.declare_parameter('grid_layout', True)
            self.declare_parameter('skip_masks', False)
            self.declare_parameter('simple_viz', False)
            self.declare_parameter('show_confidence', True)
            self.declare_parameter('show_labels', True)
            self.declare_parameter('mask_alpha', 0.4)  # Slightly more opaque
            
            # Performance tuning for speed
            self.declare_parameter('queue_size', 1)  # Minimal queue for low latency
            self.declare_parameter('async_publish', True)
            self.declare_parameter('memory_pool', True)
            self.declare_parameter('process_every_nth_frame', 3)  # Process every 3rd frame
            
            # Camera topics
            camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
            for i in range(6):
                topic_param = f'camera_topic_{i}'
                default_topic = f'/camera_{camera_names[i]}/image_raw'
                self.declare_parameter(topic_param, default_topic)
            
            # Get parameters
            self.cam_count = self.get_parameter('cam_count').get_parameter_value().integer_value
            self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
            
            # Handle device parameter
            device_param = self.get_parameter('device')
            if device_param.type_ == Parameter.Type.STRING:
                self.device = device_param.get_parameter_value().string_value
            elif device_param.type_ == Parameter.Type.INTEGER:
                device_int = device_param.get_parameter_value().integer_value
                self.device = f'cuda:{device_int}' if device_int >= 0 else 'cpu'
            else:
                self.device = 'cuda:0'
            
            self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
            self.visualization_enabled = self.get_parameter('visualization_enabled').get_parameter_value().bool_value
            self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            
            # Performance parameters
            self.inference_threads = self.get_parameter('inference_threads').get_parameter_value().integer_value
            self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
            self.half_precision = self.get_parameter('half_precision').get_parameter_value().bool_value
            self.batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value
            self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
            
            # Visualization parameters
            self.viz_scale = self.get_parameter('viz_scale').get_parameter_value().double_value
            self.viz_fps_limit = self.get_parameter('viz_fps_limit').get_parameter_value().double_value
            self.show_fps = self.get_parameter('show_fps').get_parameter_value().bool_value
            self.grid_layout = self.get_parameter('grid_layout').get_parameter_value().bool_value
            self.skip_masks = self.get_parameter('skip_masks').get_parameter_value().bool_value
            self.simple_viz = self.get_parameter('simple_viz').get_parameter_value().bool_value
            self.show_confidence = self.get_parameter('show_confidence').get_parameter_value().bool_value
            self.show_labels = self.get_parameter('show_labels').get_parameter_value().bool_value
            self.mask_alpha = self.get_parameter('mask_alpha').get_parameter_value().double_value
            
            # Performance tuning
            self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
            self.async_publish = self.get_parameter('async_publish').get_parameter_value().bool_value
            self.memory_pool = self.get_parameter('memory_pool').get_parameter_value().bool_value
            self.process_every_nth_frame = self.get_parameter('process_every_nth_frame').get_parameter_value().integer_value
            
            # Frame counters for skipping
            self.frame_counters = [0] * self.cam_count
            
            # Build camera topics
            self.camera_topics = []
            for i in range(self.cam_count):
                topic_param = f'camera_topic_{i}'
                topic = self.get_parameter(topic_param).get_parameter_value().string_value
                self.camera_topics.append(topic)
            
            self.get_logger().info(f"📹 Camera topics: {self.camera_topics}")
            self.get_logger().info(f"🤖 Model: {self.model_path}")
            self.get_logger().info(f"🔧 Device: {self.device}")
            self.get_logger().info(f"⚡ Optimized: {self.inference_threads} threads, input_size={self.input_size}")
            self.get_logger().info(f"🎯 Target FPS: {self.publish_rate}")
            self.get_logger().info(f"🎨 Large Display: viz_scale={self.viz_scale}")
            self.get_logger().info(f"⏩ Frame Skip: process every {self.process_every_nth_frame} frames")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up parameters: {e}")
            raise e

    def _initialize_model(self):
        """Initialize YOLO model with maximum optimizations"""
        try:
            if not ULTRALYTICS_AVAILABLE:
                raise ImportError("Ultralytics tidak tersedia")
            
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            self.get_logger().info(f"🔄 Loading optimized model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            # Warm up model for optimal performance
            self.get_logger().info(f"🔥 Warming up model...")
            dummy_image = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            
            # Multiple warm-up runs
            for i in range(5):
                results = self.model(dummy_image, 
                                   conf=self.conf_thres, 
                                   task='segment',
                                   device=self.device,
                                   half=self.half_precision,
                                   max_det=self.max_det,
                                   verbose=False,
                                   stream=False)
            
            self.get_logger().info(f"✅ Model warmed up and ready")
            self.get_logger().info(f"📊 Model classes: {len(self.model.names)}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize model: {e}")
            raise e

    def _setup_threading(self):
        """Setup optimized threading"""
        self.thread_pool = ThreadPoolExecutor(max_workers=self.inference_threads + 1)
        
        # Minimal queues for lowest latency
        self.image_queues = [queue.Queue(maxsize=self.queue_size) for _ in range(self.cam_count)]
        self.result_queues = [queue.Queue(maxsize=1) for _ in range(self.cam_count)]
        
        # Pre-allocate memory pools
        if self.memory_pool:
            self._setup_memory_pools()
        
        # Start inference threads (fewer threads)
        self.inference_threads_list = []
        for i in range(min(self.inference_threads, self.cam_count)):
            thread = threading.Thread(target=self._inference_worker, args=(i,), daemon=True)
            thread.start()
            self.inference_threads_list.append(thread)
        
        # Async publishing
        if self.async_publish:
            self.publish_thread = threading.Thread(target=self._async_publisher_worker, daemon=True)
            self.publish_thread.start()

    def _setup_memory_pools(self):
        """Pre-allocate optimized memory pools"""
        self.memory_pools = {
            'resized': [np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8) 
                       for _ in range(self.cam_count)],
            'viz': [np.zeros((int(1080 * self.viz_scale), int(1920 * self.viz_scale), 3), dtype=np.uint8) 
                   for _ in range(self.cam_count)]
        }
        self.pool_indices = {'resized': 0, 'viz': 0}

    def _get_from_pool(self, pool_name):
        """Get pre-allocated memory from pool"""
        if not self.memory_pool or pool_name not in self.memory_pools:
            return None
        
        pool = self.memory_pools[pool_name]
        idx = self.pool_indices[pool_name]
        self.pool_indices[pool_name] = (idx + 1) % len(pool)
        return pool[idx]

    def _setup_topics(self):
        """Setup subscriptions and publishers"""
        try:
            # Create subscriptions
            self.image_subs = []
            for i, topic in enumerate(self.camera_topics):
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_idx=i: self.image_callback(msg, cam_idx),
                    1  # Minimal queue for lowest latency
                )
                self.image_subs.append(sub)
                self.get_logger().info(f"📡 Subscribed to: {topic}")
            
            # Create publishers
            self.segmentation_pubs = []
            self.visualization_pubs = []
            
            camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
            for i in range(self.cam_count):
                # Segmentation result publisher
                seg_topic = f'/camera_{camera_names[i]}/segmentation'
                seg_pub = self.create_publisher(Yolov12Inference, seg_topic, 1)
                self.segmentation_pubs.append(seg_pub)
                
                # Visualization publisher
                if self.visualization_enabled:
                    vis_topic = f'/camera_{camera_names[i]}/segmentation_vis'
                    vis_pub = self.create_publisher(Image, vis_topic, 1)
                    self.visualization_pubs.append(vis_pub)
            
            self.get_logger().info(f"📤 Created {len(self.segmentation_pubs)} segmentation publishers")
            if self.visualization_enabled:
                self.get_logger().info(f"🎨 Created {len(self.visualization_pubs)} visualization publishers")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up topics: {e}")
            raise e

    def _setup_visualization(self):
        """Setup large OpenCV visualization"""
        if self.visualization_enabled and self.grid_layout:
            self.latest_vis_images = [None] * self.cam_count
            self.viz_lock = threading.Lock()
            self.last_viz_time = time.time()
            
            # Start visualization thread
            self.viz_thread = threading.Thread(target=self._visualization_worker, daemon=True)
            self.viz_thread.start()

    def _setup_timers(self):
        """Setup timers"""
        self.stats_timer = self.create_timer(3.0, self.log_statistics)  # Less frequent

    def image_callback(self, msg, camera_index):
        """Optimized image callback with error handling"""
        try:
            # Check if shutting down
            if hasattr(self, '_shutdown_flag') and self._shutdown_flag:
                return
            
            # Frame skipping for performance
            self.frame_counters[camera_index] += 1
            if self.frame_counters[camera_index] % self.process_every_nth_frame != 0:
                return  # Skip this frame
            
            # Convert ROS image to OpenCV with error handling
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn(f"Image conversion failed for camera {camera_index}: {e}")
                return
            
            # Non-blocking queue insertion - drop if full
            if not self.image_queues[camera_index].full():
                self.image_queues[camera_index].put((cv_image, msg.header, camera_index))
            
        except Exception as e:
            if not hasattr(self, '_shutdown_flag') or not self._shutdown_flag:
                self.get_logger().error(f"❌ Image callback error camera {camera_index}: {e}")

    def _inference_worker(self, worker_id):
        """ULTRA-OPTIMIZED inference worker for HIGH FPS"""
        camera_assignments = list(range(worker_id, self.cam_count, self.inference_threads))
        
        while True:
            try:
                processed_any = False
                
                for camera_index in camera_assignments:
                    try:
                        # Non-blocking get
                        cv_image, header, cam_idx = self.image_queues[camera_index].get_nowait()
                        processed_any = True
                        
                        # ULTRA-FAST preprocessing with aggressive optimization
                        h, w = cv_image.shape[:2]
                        
                        # Use smaller input size for speed
                        target_size = self.input_size
                        
                        # Simple resize without padding for maximum speed
                        resized = cv2.resize(cv_image, (target_size, target_size), 
                                           interpolation=cv2.INTER_LINEAR)
                        
                        # Calculate scale for coordinate conversion
                        scale_x = target_size / w
                        scale_y = target_size / h
                        scale = min(scale_x, scale_y)  # Use minimum for aspect ratio
                        
                        # ULTRA-OPTIMIZED inference with aggressive settings
                        start_time = time.time()
                        results = self.model(resized,
                                           conf=self.conf_thres,
                                           task='segment',
                                           device=self.device,
                                           half=self.half_precision,
                                           max_det=self.max_det,
                                           verbose=False,
                                           stream=False,
                                           # Additional speed optimizations
                                           agnostic_nms=True,  # Faster NMS
                                           save=False,
                                           save_txt=False,
                                           save_conf=False)
                        inference_time = time.time() - start_time
                        
                        # Update statistics
                        with self.lock:
                            self.stats['total_frames'] += 1
                            self.stats['successful_segmentations'] += 1
                            total_frames = self.stats['total_frames']
                            self.stats['average_inference_time'] = (
                                (self.stats['average_inference_time'] * (total_frames - 1) + inference_time) 
                                / total_frames
                            )
                            self.stats['fps'] = 1.0 / inference_time if inference_time > 0 else 0
                    
                        # Process results with corrected scaling
                        result_data = {
                            'results': results[0],
                            'original_image': cv_image,
                            'header': header,
                            'camera_index': camera_index,
                            'scale': scale,  # Use calculated scale
                            'scale_x': scale_x,  # Individual scales for correction
                            'scale_y': scale_y,
                            'inference_time': inference_time
                        }
                        
                        # Process results immediately if not async
                        if self.async_publish:
                            if not self.result_queues[camera_index].full():
                                self.result_queues[camera_index].put(result_data)
                        else:
                            self._process_results(result_data)
                        
                    except queue.Empty:
                        continue
            
                if not processed_any:
                    time.sleep(0.0001)  # Extremely short sleep for maximum responsiveness
                
            except Exception as e:
                with self.lock:
                    self.stats['failed_segmentations'] += 1
                self.get_logger().error(f"❌ Inference worker {worker_id} error: {e}")

    def _async_publisher_worker(self):
        """Optimized async publisher worker"""
        while True:
            try:
                processed_any = False
                for camera_index in range(self.cam_count):
                    try:
                        result_data = self.result_queues[camera_index].get_nowait()
                        self._process_results(result_data)
                        processed_any = True
                    except queue.Empty:
                        continue
                
                if not processed_any:
                    time.sleep(0.001)
                    
            except Exception as e:
                self.get_logger().error(f"❌ Async publisher error: {e}")

    def _process_results(self, result_data):
        """Process results with optimized segmentation"""
        try:
            results = result_data['results']
            original_image = result_data['original_image']
            header = result_data['header']
            camera_index = result_data['camera_index']
            scale = result_data['scale']
            
            # Create and publish segmentation message (minimal)
            if not self.skip_masks:
                seg_msg = self._create_minimal_segmentation_message(results, header, camera_index, scale)
                if camera_index < len(self.segmentation_pubs):
                    self.segmentation_pubs[camera_index].publish(seg_msg)
            
            # Create optimized visualization
            if self.visualization_enabled:
                vis_image = self._create_optimized_segmentation_visualization(
                    original_image, results, scale, result_data['inference_time'])
                
                # Publish individual visualization
                if camera_index < len(self.visualization_pubs):
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                    vis_msg.header = header
                    self.visualization_pubs[camera_index].publish(vis_msg)
                
                # Store for grid visualization
                if self.grid_layout:
                    with self.viz_lock:
                        self.latest_vis_images[camera_index] = vis_image
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing results: {e}")

    def _create_minimal_segmentation_message(self, results, header, camera_index, scale):
        """Create minimal segmentation message for performance"""
        seg_msg = Yolov12Inference()
        seg_msg.header = header
        seg_msg.camera_name = f'camera_{camera_index}'
        seg_msg.frame_type = 'segmentation'
        seg_msg.task = 'segment'
        
        if results.boxes is not None and len(results.boxes) > 0:
            for i, box in enumerate(results.boxes):
                inference_result = InferenceResult()
                inference_result.class_name = results.names[int(box.cls)]
                inference_result.confidence = float(box.conf)
                
                # Scale bounding box back to original size
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                inference_result.left = int(x1 / scale)
                inference_result.top = int(y1 / scale)
                inference_result.right = int(x2 / scale)
                inference_result.bottom = int(y2 / scale)
                
                seg_msg.yolov12_inference.append(inference_result)
        
        return seg_msg

    def _create_optimized_segmentation_visualization(self, image, results, scale, inference_time):
        """Create high-quality segmentation visualization with correct coordinate mapping"""
        # Scale down for visualization (larger than before)
        viz_h = int(image.shape[0] * self.viz_scale)
        viz_w = int(image.shape[1] * self.viz_scale)
        
        # Resize original image to visualization size
        vis_image = cv2.resize(image, (viz_w, viz_h), interpolation=cv2.INTER_AREA)
        
        if results.boxes is not None and len(results.boxes) > 0:
            # Draw masks first with CORRECTED scaling
            if results.masks is not None and not self.skip_masks:
                # Create mask overlay
                mask_overlay = np.zeros_like(vis_image, dtype=np.float32)
                
                for i, mask in enumerate(results.masks):
                    # Get mask data
                    mask_data = mask.data[0].cpu().numpy()
                    
                    # Calculate correct scaling for mask
                    # mask_data is at model input size, need to scale to original then to viz
                    mask_h, mask_w = mask_data.shape
                    orig_h, orig_w = image.shape[:2]
                    
                    # First scale mask to original image size
                    mask_original_size = cv2.resize(
                        mask_data, (orig_w, orig_h), 
                        interpolation=cv2.INTER_CUBIC
                    )
                    
                    # Then scale to visualization size
                    mask_resized = cv2.resize(
                        mask_original_size, (viz_w, viz_h), 
                        interpolation=cv2.INTER_CUBIC
                    )
                    
                    # Apply Gaussian blur for smoother edges
                    mask_resized = cv2.GaussianBlur(mask_resized, (3, 3), 0)
                    
                    # Get class color
                    class_id = int(results.boxes[i].cls)
                    color = self.class_colors[class_id % len(self.class_colors)]
                    
                    # Create colored mask with smooth alpha
                    mask_colored = np.zeros_like(vis_image, dtype=np.float32)
                    for c in range(3):
                        mask_colored[:, :, c] = mask_resized * color[c]
                    
                    # Blend with cumulative mask
                    alpha = mask_resized * self.mask_alpha
                    for c in range(3):
                        mask_overlay[:, :, c] = (1 - alpha) * mask_overlay[:, :, c] + alpha * mask_colored[:, :, c]
            
            # Apply mask overlay to image
            vis_image = vis_image.astype(np.float32)
            vis_image = (1 - self.mask_alpha) * vis_image + mask_overlay
            vis_image = np.clip(vis_image, 0, 255).astype(np.uint8)
        
        # Draw CORRECTED bounding boxes and labels
        for i, box in enumerate(results.boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # CORRECT scaling: from model input coordinates to visualization coordinates
            # Original image dimensions
            orig_h, orig_w = image.shape[:2]
            
            # Scale from model input size to original image size
            x1_orig = x1 / scale
            y1_orig = y1 / scale
            x2_orig = x2 / scale
            y2_orig = y2 / scale
            
            # Then scale to visualization size
            x1 = int(x1_orig * self.viz_scale)
            y1 = int(y1_orig * self.viz_scale)
            x2 = int(x2_orig * self.viz_scale)
            y2 = int(y2_orig * self.viz_scale)
            
            # Ensure coordinates are within bounds
            x1 = max(0, min(x1, viz_w))
            y1 = max(0, min(y1, viz_h))
            x2 = max(0, min(x2, viz_w))
            y2 = max(0, min(y2, viz_h))
            
            # Get class info
            class_id = int(box.cls)
            confidence = float(box.conf)
            class_name = results.names[class_id]
            
            # Get class color
            color = self.class_colors[class_id % len(self.class_colors)]
            
            # Draw thick, clean bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 3)
            
            # Draw corner markers for modern look
            corner_length = 15
            cv2.line(vis_image, (x1, y1), (x1 + corner_length, y1), color, 4)
            cv2.line(vis_image, (x1, y1), (x1, y1 + corner_length), color, 4)
            cv2.line(vis_image, (x2, y1), (x2 - corner_length, y1), color, 4)
            cv2.line(vis_image, (x2, y1), (x2, y1 + corner_length), color, 4)
            cv2.line(vis_image, (x1, y2), (x1 + corner_length, y2), color, 4)
            cv2.line(vis_image, (x1, y2), (x1, y2 - corner_length), color, 4)
            cv2.line(vis_image, (x2, y2), (x2 - corner_length, y2), color, 4)
            cv2.line(vis_image, (x2, y2), (x2, y2 - corner_length), color, 4)
            
            # Draw professional label
            if self.show_labels:
                if self.show_confidence:
                    label = f'{class_name} {confidence:.2f}'
                else:
                    label = class_name
                
                # Calculate text size
                font_scale = 0.7
                thickness = 2
                (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
                
                # Draw rounded label background
                label_bg_y1 = y1 - text_h - 20
                label_bg_y2 = y1 - 5
                label_bg_x1 = x1
                label_bg_x2 = x1 + text_w + 20
                
                # Ensure label stays in bounds
                if label_bg_y1 < 0:
                    label_bg_y1 = y2 + 5
                    label_bg_y2 = y2 + text_h + 20
                
                # Draw label background with slight transparency
                overlay = vis_image.copy()
                cv2.rectangle(overlay, (label_bg_x1, label_bg_y1), (label_bg_x2, label_bg_y2), color, -1)
                cv2.addWeighted(overlay, 0.8, vis_image, 0.2, 0, vis_image)
                
                # Draw label text
                text_y = label_bg_y1 + text_h + 10
                cv2.putText(vis_image, label, (x1 + 10, text_y), 
                          cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
    
        # Add professional FPS overlay
        if self.show_fps:
            fps_text = f'FPS: {1.0/inference_time:.1f}' if inference_time > 0 else 'FPS: --'
            # Draw FPS background
            cv2.rectangle(vis_image, (5, 5), (120, 40), (0, 0, 0), -1)
            cv2.putText(vis_image, fps_text, (10, 28), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return vis_image

    def _visualization_worker(self):
        """Optimized grid visualization worker"""
        while True:
            try:
                current_time = time.time()
                if current_time - self.last_viz_time < (1.0 / self.viz_fps_limit):
                    time.sleep(0.01)
                    continue
                
                with self.viz_lock:
                    images = self.latest_vis_images.copy()
                
                # Check if we have enough images
                valid_images = [img for img in images if img is not None]
                if len(valid_images) < 2:
                    time.sleep(0.05)
                    continue
                
                # Create large grid layout
                grid_image = self._create_large_grid_layout(images)
                if grid_image is not None:
                    cv2.imshow('HuskyBot YOLOv11 Segmentation - Large Display', grid_image)
                    cv2.waitKey(1)
                    
                    # Update viz FPS
                    viz_time = time.time() - current_time
                    with self.lock:
                        self.stats['viz_fps'] = 1.0 / viz_time if viz_time > 0 else 0
                
                self.last_viz_time = current_time
                
            except Exception as e:
                self.get_logger().error(f"❌ Visualization worker error: {e}")
                time.sleep(0.05)

    def _create_large_grid_layout(self, images):
        """Create large 2x3 grid layout for better visibility"""
        try:
            # MUCH LARGER target size for better visibility
            target_width, target_height = 512, 384  # Much larger than before
            
            display_images = []
            camera_names = ['Front-Left', 'Front', 'Right', 'Left', 'Rear', 'Rear-Right']
            
            for i, name in enumerate(camera_names):
                if i < len(images) and images[i] is not None:
                    img = cv2.resize(images[i], (target_width, target_height), 
                                   interpolation=cv2.INTER_AREA)
                else:
                    # Professional placeholder
                    img = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                    cv2.rectangle(img, (10, 10), (target_width-10, target_height-10), (50, 50, 50), 2)
                    cv2.putText(img, name, (target_width//4, target_height//2), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (150, 150, 150), 2)
                
                # Add professional camera label with background
                label_bg_h = 35
                overlay = img.copy()
                cv2.rectangle(overlay, (0, 0), (target_width, label_bg_h), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)
                
                cv2.putText(img, name, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                display_images.append(img)
            
            # Arrange in 2x3 grid with spacing
            if len(display_images) >= 6:
                spacing = 5
                
                # Top row: front_left(1), front(0), right(2)
                top_row = np.hstack([
                    display_images[1], 
                    np.zeros((target_height, spacing, 3), dtype=np.uint8),
                    display_images[0], 
                    np.zeros((target_height, spacing, 3), dtype=np.uint8),
                    display_images[2]
                ])
                
                # Bottom row: left(3), rear(4), rear_right(5)
                bottom_row = np.hstack([
                    display_images[3], 
                    np.zeros((target_height, spacing, 3), dtype=np.uint8),
                    display_images[4], 
                    np.zeros((target_height, spacing, 3), dtype=np.uint8),
                    display_images[5]
                ])
                
                # Vertical spacing
                h_spacing = np.zeros((spacing, top_row.shape[1], 3), dtype=np.uint8)
                
                # Combine rows
                combined = np.vstack([top_row, h_spacing, bottom_row])
                
                # Professional title with stats
                title_height = 70
                title_img = np.zeros((title_height, combined.shape[1], 3), dtype=np.uint8)
                
                # Gradient background for title
                for i in range(title_height):
                    alpha = i / title_height
                    title_img[i, :] = [int(30 * alpha), int(30 * alpha), int(60 * alpha)]
                
                with self.lock:
                    stats_text = f'HuskyBot YOLOv11 Segmentation | Real-time Performance'
                    perf_text = f'FPS: {self.stats["fps"]:.1f} | Inference: {self.stats["average_inference_time"]*1000:.0f}ms | Frames: {self.stats["total_frames"]}'
                
                cv2.putText(title_img, stats_text, (20, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(title_img, perf_text, (20, 55), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                final_img = np.vstack([title_img, combined])
                return final_img
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"❌ Error creating large grid layout: {e}")
            return None

    def log_statistics(self):
        """Log performance statistics"""
        try:
            with self.lock:
                stats = self.stats.copy()
            
            if stats['total_frames'] > 0:
                success_rate = (stats['successful_segmentations'] / stats['total_frames']) * 100
                
                self.get_logger().info(
                    f"🚀 Optimized Performance: "
                    f"Frames={stats['total_frames']}, "
                    f"Success Rate={success_rate:.1f}%, "
                    f"Inference={stats['average_inference_time']*1000:.0f}ms, "
                    f"FPS={stats['fps']:.1f}, "
                    f"VizFPS={stats['viz_fps']:.1f}"
                )
        except Exception as e:
            self.get_logger().error(f"❌ Error logging statistics: {e}")

    def destroy_node(self):
        """Clean shutdown with proper error handling"""
        try:
            self.get_logger().info("🛑 Shutting down segmentation node...")
            
            # Stop all processing
            self._shutdown_flag = True
            
            # Wait for threads to finish
            if hasattr(self, 'inference_threads_list'):
                for thread in self.inference_threads_list:
                    if thread.is_alive():
                        thread.join(timeout=1.0)
            
            # Shutdown thread pool
            if hasattr(self, 'thread_pool'):
                self.thread_pool.shutdown(wait=False)
            
            # Clear queues
            if hasattr(self, 'image_queues'):
                for q in self.image_queues:
                    try:
                        while not q.empty():
                            q.get_nowait()
                    except:
                        pass
            
            # Close OpenCV windows
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            # Call parent destroy
            super().destroy_node()
            
        except Exception as e:
            print(f"Warning during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Set highest process priority
    try:
        os.nice(-20)
    except:
        pass
    
    node = None
    try:
        node = MulticamSegmentationNode()
        
        # Add shutdown flag
        node._shutdown_flag = False
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            try:
                node._shutdown_flag = True
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()