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
        super().__init__('multicam_segmentation')
        
        # Initialize basic components
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # ENHANCED debug counters with detailed tracking
        self.debug_counters = {
            'image_callbacks': [0] * 6,
            'last_callback_time': [0.0] * 6,
            'total_callbacks': 0,
            'queue_insertions': [0] * 6,
            'inference_starts': [0] * 6,
            'inference_completions': [0] * 6,
            'publish_attempts': [0] * 6
        }
        
        # Setup parameters
        self._setup_parameters()
        
        # Initialize model
        self._initialize_model()
        
        # Setup ULTRA-OPTIMIZED threading
        self._setup_threading()
        
        # Setup topics
        self._setup_topics()
        
        # MINIMAL visualization setup
        self._setup_visualization()
        
        # Setup timers
        self._setup_timers()
        
        # Performance statistics
        self.stats = {
            'total_frames': 0,
            'successful_segmentations': 0,
            'failed_segmentations': 0,
            'average_inference_time': 0.0,
            'fps': 0.0,
            'viz_fps': 0.0,
            'last_update_time': time.time()
        }
        
        # MINIMAL class colors for speed
        self._setup_class_colors()
        
        self.get_logger().info("🚀 ULTRA-OPTIMIZED YOLOv11 Segmentation Node for 60+ FPS initialized")
        
        # Start debug timer (more frequent for debugging)
        self.debug_timer = self.create_timer(5.0, self.debug_callback_status)

    def _setup_parameters(self):
        """Setup ULTRA-OPTIMIZED parameters for 60+ FPS"""
        try:
            # Core parameters
            self.declare_parameter('cam_count', 6)
            self.declare_parameter('model_path', 'yolo11n-seg.engine')
            self.declare_parameter('device', 'cuda:0')
            self.declare_parameter('conf_thres', 0.5)  # Lower for more detections to test
            self.declare_parameter('visualization_enabled', True)
            self.declare_parameter('publish_rate', 60.0)  # TARGET 60 FPS
            
            # ULTRA-AGGRESSIVE Performance parameters
            self.declare_parameter('inference_threads', 6)  # One per camera
            self.declare_parameter('input_size', 320)  # Slightly larger for stability
            self.declare_parameter('half_precision', True)
            self.declare_parameter('batch_size', 1)
            self.declare_parameter('max_det', 25)  # More detections for testing
            
            # MINIMAL visualization
            self.declare_parameter('viz_scale', 0.5)  # Larger for visibility
            self.declare_parameter('viz_fps_limit', 30.0)
            self.declare_parameter('show_fps', True)
            self.declare_parameter('grid_layout', True)  # ENABLE for debugging
            self.declare_parameter('skip_masks', False)  # ENABLE masks for testing
            self.declare_parameter('simple_viz', False)  # Full viz for debugging
            self.declare_parameter('show_confidence', True)  # ENABLE for debugging
            self.declare_parameter('show_labels', True)  # ENABLE for debugging
            self.declare_parameter('mask_alpha', 0.3)
            
            # MAXIMUM speed optimizations
            self.declare_parameter('queue_size', 2)  # Larger queue for debugging
            self.declare_parameter('async_publish', False)  # Sync for debugging
            self.declare_parameter('memory_pool', True)
            self.declare_parameter('process_every_nth_frame', 1)  # Process all frames
            
            # Camera topics
            for i in range(6):
                topic_param = f'camera_topic_{i}'
                camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
                default_topic = f'/camera_{camera_names[i]}/image_raw'
                self.declare_parameter(topic_param, default_topic)
            
            # Get parameters
            self.cam_count = self.get_parameter('cam_count').get_parameter_value().integer_value
            self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
            self.device = self.get_parameter('device').get_parameter_value().string_value
            self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
            self.visualization_enabled = self.get_parameter('visualization_enabled').get_parameter_value().bool_value
            self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            
            # Performance parameters
            self.inference_threads = self.get_parameter('inference_threads').get_parameter_value().integer_value
            self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
            self.half_precision = self.get_parameter('half_precision').get_parameter_value().bool_value
            self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
            
            # Visualization parameters
            self.viz_scale = self.get_parameter('viz_scale').get_parameter_value().double_value
            self.skip_masks = self.get_parameter('skip_masks').get_parameter_value().bool_value
            self.simple_viz = self.get_parameter('simple_viz').get_parameter_value().bool_value
            self.show_confidence = self.get_parameter('show_confidence').get_parameter_value().bool_value
            self.show_labels = self.get_parameter('show_labels').get_parameter_value().bool_value
            self.grid_layout = self.get_parameter('grid_layout').get_parameter_value().bool_value
            
            # Speed optimizations
            self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
            self.async_publish = self.get_parameter('async_publish').get_parameter_value().bool_value
            self.memory_pool = self.get_parameter('memory_pool').get_parameter_value().bool_value
            self.process_every_nth_frame = self.get_parameter('process_every_nth_frame').get_parameter_value().integer_value
            
            # Frame counters
            self.frame_counters = [0] * self.cam_count
            
            # Build camera topics
            self.camera_topics = []
            for i in range(self.cam_count):
                topic = self.get_parameter(f'camera_topic_{i}').get_parameter_value().string_value
                self.camera_topics.append(topic)
            
            self.get_logger().info(f"📹 Camera topics: {self.camera_topics}")
            self.get_logger().info(f"🤖 Model: {self.model_path}")
            self.get_logger().info(f"🔧 Device: {self.device}")
            self.get_logger().info(f"⚡ Optimized: {self.inference_threads} threads, input_size={self.input_size}")
            self.get_logger().info(f"🎯 Target FPS: {self.publish_rate}")
            self.get_logger().info(f"🎨 Display: viz_scale={self.viz_scale}")
            self.get_logger().info(f"⏩ Frame Skip: process every {self.process_every_nth_frame} frames")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up parameters: {e}")
            raise e

    def _initialize_model(self):
        """Initialize YOLO model with DEBUG information"""
        try:
            if not ULTRALYTICS_AVAILABLE:
                raise ImportError("Ultralytics tidak tersedia")
            
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            self.get_logger().info(f"🔄 Loading optimized model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            # Model warm-up with detailed logging
            self.get_logger().info(f"🔥 Warming up model...")
            dummy_image = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            
            # Test inference
            start_time = time.time()
            results = self.model(dummy_image, 
                               conf=self.conf_thres, 
                               task='segment',
                               device=self.device,
                               half=self.half_precision,
                               max_det=self.max_det,
                               verbose=False,
                               stream=False,
                               # Optimizations
                               agnostic_nms=True,
                               classes=None,
                               retina_masks=True,
                               imgsz=self.input_size)
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ Model warmed up and ready")
            self.get_logger().info(f"📊 Model classes: {len(self.model.names)}")
            self.get_logger().info(f"⏱️ Warmup inference time: {warmup_time*1000:.1f}ms")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize model: {e}")
            traceback.print_exc()
            raise e

    def _setup_threading(self):
        """Setup threading with optimal configuration"""
        # Reduced thread pool for better resource management
        self.thread_pool = ThreadPoolExecutor(max_workers=self.inference_threads + 1)
        
        # Smaller queues for speed
        self.image_queues = [queue.Queue(maxsize=1) for _ in range(self.cam_count)]
        self.result_queues = [queue.Queue(maxsize=1) for _ in range(self.cam_count)]
        
        # Pre-allocate memory pools for speed
        if self.memory_pool:
            self._setup_memory_pools()
        
        # Start OPTIMIZED inference threads (fewer threads, better performance)
        self.inference_threads_list = []
        for i in range(self.cam_count):
            thread = threading.Thread(target=self._dedicated_inference_worker, args=(i,), daemon=True)
            thread.start()
            self.inference_threads_list.append(thread)
            self.get_logger().info(f"🔄 Started inference thread for camera {i}")
        
        # Enable async publishing
        if self.async_publish:
            self.publish_thread = threading.Thread(target=self._ultra_fast_publisher_worker, daemon=True)
            self.publish_thread.start()
            self.get_logger().info("🔄 Started async publisher thread")

    def _setup_memory_pools(self):
        """Pre-allocate memory pools"""
        self.memory_pools = {
            'resized': [np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8) 
                       for _ in range(self.cam_count)],
            'viz': [np.zeros((int(1080 * self.viz_scale), int(1920 * self.viz_scale), 3), dtype=np.uint8) 
                   for _ in range(self.cam_count)]
        }
        self.get_logger().info(f"💾 Memory pools allocated for {self.cam_count} cameras")

    def _setup_topics(self):
        """Setup topics with detailed logging"""
        try:
            # Create subscriptions
            self.image_subs = []
            for i, topic in enumerate(self.camera_topics):
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_idx=i: self.ultra_fast_image_callback(msg, cam_idx),
                    self.queue_size
                )
                self.image_subs.append(sub)
                self.get_logger().info(f"📡 Subscribed to: {topic}")
            
            # Create publishers
            self.segmentation_pubs = []
            self.visualization_pubs = []
            
            camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
            for i in range(self.cam_count):
                # Segmentation publisher
                seg_topic = f'/camera_{camera_names[i]}/segmentation'
                seg_pub = self.create_publisher(Yolov12Inference, seg_topic, self.queue_size)
                self.segmentation_pubs.append(seg_pub)
                
                # Visualization publisher
                if self.visualization_enabled:
                    vis_topic = f'/camera_{camera_names[i]}/segmentation_vis'
                    vis_pub = self.create_publisher(Image, vis_topic, self.queue_size)
                    self.visualization_pubs.append(vis_pub)
            
            self.get_logger().info(f"📤 Created {len(self.segmentation_pubs)} segmentation publishers")
            self.get_logger().info(f"🎨 Created {len(self.visualization_pubs)} visualization publishers")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up topics: {e}")
            raise e

    def _setup_visualization(self):
        """Setup visualization"""
        if self.visualization_enabled:
            self.latest_vis_images = [None] * self.cam_count
            self.viz_lock = threading.Lock()
            if self.grid_layout:
                # Grid visualization publisher
                self.grid_pub = self.create_publisher(Image, '/segmentation_grid', 1)
                self.get_logger().info("🎨 Grid visualization enabled")

    def _setup_timers(self):
        """Setup timers"""
        self.stats_timer = self.create_timer(3.0, self.log_statistics)

    def _setup_class_colors(self):
        """Setup colors"""
        self.class_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 255, 0), (255, 128, 0), (128, 0, 255), (255, 0, 128)
        ] * 8

    def ultra_fast_image_callback(self, msg, camera_index):
        """Enhanced image callback with detailed logging"""
        try:
            # Enhanced debug tracking
            self.debug_counters['image_callbacks'][camera_index] += 1
            self.debug_counters['last_callback_time'][camera_index] = time.time()
            
            # Frame skipping check
            self.frame_counters[camera_index] += 1
            if self.frame_counters[camera_index] % self.process_every_nth_frame != 0:
                return
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Queue insertion with better error handling
            try:
                self.image_queues[camera_index].put_nowait((cv_image, msg.header, camera_index))
                self.debug_counters['queue_insertions'][camera_index] += 1
                
                # Log first successful insertion
                if self.debug_counters['queue_insertions'][camera_index] == 1:
                    self.get_logger().info(f"✅ Camera {camera_index}: First frame received! Size: {cv_image.shape}")
                    
            except queue.Full:
                # Try to clear old frame and insert new one
                try:
                    self.image_queues[camera_index].get_nowait()
                    self.image_queues[camera_index].put_nowait((cv_image, msg.header, camera_index))
                    self.debug_counters['queue_insertions'][camera_index] += 1
                except:
                    pass
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in image callback cam {camera_index}: {e}")

    def _dedicated_inference_worker(self, camera_index):
        """Enhanced inference worker with detailed logging"""
        self.get_logger().info(f"🔄 Inference worker {camera_index} started")
        
        while True:
            try:
                # Get image (blocking)
                cv_image, header, cam_idx = self.image_queues[camera_index].get()
                self.debug_counters['inference_starts'][camera_index] += 1
                
                # Log first inference start
                if self.debug_counters['inference_starts'][camera_index] == 1:
                    self.get_logger().info(f"🚀 Camera {camera_index}: Starting first inference")
                
                # Preprocessing
                h, w = cv_image.shape[:2]
                resized = cv2.resize(cv_image, (self.input_size, self.input_size), 
                                   interpolation=cv2.INTER_LINEAR)
                scale_x = w / self.input_size
                scale_y = h / self.input_size
                
                # Inference with error handling
                start_time = time.time()
                try:
                    results = self.model(resized,
                                       conf=self.conf_thres,
                                       task='segment' if not self.skip_masks else 'detect',
                                       device=self.device,
                                       half=self.half_precision,
                                       max_det=self.max_det,
                                       verbose=False,
                                       stream=False,
                                       agnostic_nms=True,
                                       retina_masks=True,
                                       imgsz=self.input_size)
                    
                    inference_time = time.time() - start_time
                    self.debug_counters['inference_completions'][camera_index] += 1
                    
                    # Log first successful inference
                    if self.debug_counters['inference_completions'][camera_index] == 1:
                        self.get_logger().info(f"✅ Camera {camera_index}: First inference completed! Time: {inference_time*1000:.1f}ms")
                    
                    # Update statistics
                    with self.lock:
                        self.stats['total_frames'] += 1
                        self.stats['successful_segmentations'] += 1
                        self.stats['average_inference_time'] = inference_time
                        self.stats['fps'] = 1.0 / inference_time if inference_time > 0 else 0
                        self.stats['last_update_time'] = time.time()
                    
                    # Create result data
                    result_data = {
                        'results': results[0],
                        'original_image': cv_image,
                        'header': header,
                        'camera_index': camera_index,
                        'scale_x': scale_x,
                        'scale_y': scale_y,
                        'inference_time': inference_time
                    }
                    
                    # Process results
                    if self.async_publish:
                        try:
                            self.result_queues[camera_index].put_nowait(result_data)
                        except queue.Full:
                            # Clear old result and add new one
                            try:
                                self.result_queues[camera_index].get_nowait()
                                self.result_queues[camera_index].put_nowait(result_data)
                            except:
                                pass
                    else:
                        self._ultra_fast_process_results(result_data)
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Inference error cam {camera_index}: {e}")
                    with self.lock:
                        self.stats['failed_segmentations'] += 1
                
            except Exception as e:
                self.get_logger().error(f"❌ Worker error cam {camera_index}: {e}")

    def _ultra_fast_publisher_worker(self):
        """Async publisher worker"""
        while True:
            try:
                for camera_index in range(self.cam_count):
                    try:
                        result_data = self.result_queues[camera_index].get_nowait()
                        self._ultra_fast_process_results(result_data)
                    except queue.Empty:
                        continue
                time.sleep(0.001)  # Small sleep to prevent CPU spinning
            except Exception as e:
                self.get_logger().error(f"❌ Publisher worker error: {e}")

    def _ultra_fast_process_results(self, result_data):
        """Enhanced result processing with detailed logging"""
        try:
            results = result_data['results']
            camera_index = result_data['camera_index']
            scale_x = result_data['scale_x']
            scale_y = result_data['scale_y']
            header = result_data['header']
            
            self.debug_counters['publish_attempts'][camera_index] += 1
            
            # Create segmentation message
            seg_msg = Yolov12Inference()
            seg_msg.header = header
            seg_msg.camera_name = f'camera_{camera_index}'
            seg_msg.frame_type = 'segmentation' if not self.skip_masks else 'detection'
            seg_msg.task = 'segment' if not self.skip_masks else 'detect'
            
            # Process detections
            detection_count = 0
            if results.boxes is not None and len(results.boxes) > 0:
                for box in results.boxes:
                    inference_result = InferenceResult()
                    inference_result.class_name = results.names[int(box.cls)]
                    inference_result.confidence = float(box.conf)
                    
                    # Scale bounding box back to original size
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    inference_result.left = int(x1 * scale_x)
                    inference_result.top = int(y1 * scale_y)
                    inference_result.right = int(x2 * scale_x)
                    inference_result.bottom = int(y2 * scale_y)
                    
                    seg_msg.yolov12_inference.append(inference_result)
                    detection_count += 1
            
            # Log first successful detection
            if detection_count > 0 and self.debug_counters['publish_attempts'][camera_index] == 1:
                self.get_logger().info(f"🎯 Camera {camera_index}: First detection! Found {detection_count} objects")
            
            # Publish segmentation
            if camera_index < len(self.segmentation_pubs):
                self.segmentation_pubs[camera_index].publish(seg_msg)
            
            # Create and publish visualization
            if self.visualization_enabled:
                vis_image = self._create_visualization(
                    result_data['original_image'], results, scale_x, scale_y, camera_index)
                
                if camera_index < len(self.visualization_pubs):
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                    vis_msg.header = header
                    self.visualization_pubs[camera_index].publish(vis_msg)
                
                # Store for grid if enabled
                if self.grid_layout:
                    with self.viz_lock:
                        self.latest_vis_images[camera_index] = vis_image
                        # Create grid if we have enough images
                        if all(img is not None for img in self.latest_vis_images):
                            self._publish_grid()
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing results cam {camera_index}: {e}")

    def _create_visualization(self, image, results, scale_x, scale_y, camera_index):
        """Create enhanced visualization"""
        # Resize for visualization
        viz_h = int(image.shape[0] * self.viz_scale)
        viz_w = int(image.shape[1] * self.viz_scale)
        vis_image = cv2.resize(image, (viz_w, viz_h), interpolation=cv2.INTER_LINEAR)
        
        # Scale factors for visualization
        viz_scale_x = self.viz_scale
        viz_scale_y = self.viz_scale
        
        # Draw detections
        if results.boxes is not None and len(results.boxes) > 0:
            for i, box in enumerate(results.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf)
                class_id = int(box.cls)
                class_name = results.names[class_id]
                
                # Scale to visualization coordinates
                x1_viz = int(x1 * viz_scale_x)
                y1_viz = int(y1 * viz_scale_y)
                x2_viz = int(x2 * viz_scale_x)
                y2_viz = int(y2 * viz_scale_y)
                
                # Color
                color = self.class_colors[class_id % len(self.class_colors)]
                
                # Draw rectangle
                cv2.rectangle(vis_image, (x1_viz, y1_viz), (x2_viz, y2_viz), color, 2)
                
                # Draw label if enabled
                if self.show_labels or self.show_confidence:
                    label = ""
                    if self.show_labels:
                        label += class_name
                    if self.show_confidence:
                        if label:
                            label += f" {confidence:.2f}"
                        else:
                            label = f"{confidence:.2f}"
                    
                    if label:
                        # Get text size
                        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        # Draw background
                        cv2.rectangle(vis_image, (x1_viz, y1_viz - text_height - 5), 
                                    (x1_viz + text_width, y1_viz), color, -1)
                        # Draw text
                        cv2.putText(vis_image, label, (x1_viz, y1_viz - 5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add camera info
        info_text = f"Cam {camera_index}"
        cv2.putText(vis_image, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return vis_image

    def _publish_grid(self):
        """Create and publish grid visualization with detailed logging"""
        try:
            # Check if we have all images
            valid_images = [img for img in self.latest_vis_images if img is not None]
            self.get_logger().info(f"🖼️ Grid check: {len(valid_images)}/6 images available")
            
            if len(valid_images) >= 6:
                # Get dimensions from first image
                h, w = self.latest_vis_images[0].shape[:2]
                self.get_logger().info(f"🎨 Creating grid with image size: {w}x{h}")
                
                # Create 2x3 grid
                top_row = np.hstack([self.latest_vis_images[0], self.latest_vis_images[1], self.latest_vis_images[2]])
                bottom_row = np.hstack([self.latest_vis_images[3], self.latest_vis_images[4], self.latest_vis_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "segmentation_grid"
                self.grid_pub.publish(grid_msg)
                
                self.get_logger().info(f"✅ Grid published! Size: {grid.shape[1]}x{grid.shape[0]}")
                
            else:
                self.get_logger().warn(f"⚠️ Not enough images for grid: {len(valid_images)}/6")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error creating grid: {e}")
            import traceback
            traceback.print_exc()

    def debug_callback_status(self):
        """Enhanced debug callback"""
        try:
            total_callbacks = sum(self.debug_counters['image_callbacks'])
            total_queue_insertions = sum(self.debug_counters['queue_insertions'])
            total_inference_starts = sum(self.debug_counters['inference_starts'])
            total_inference_completions = sum(self.debug_counters['inference_completions'])
            total_publish_attempts = sum(self.debug_counters['publish_attempts'])
            
            if total_callbacks == 0:
                self.get_logger().warn("⚠️ NO CAMERA DATA - Check camera topics!")
            else:
                self.get_logger().info(f"📊 Callbacks={total_callbacks}, Queued={total_queue_insertions}, "
                                     f"Started={total_inference_starts}, Completed={total_inference_completions}, "
                                     f"Published={total_publish_attempts}")
                
                # Individual camera status
                for i in range(self.cam_count):
                    self.get_logger().info(f"📸 Camera {i}: {self.debug_counters['image_callbacks'][i]} frames received, "
                                         f"last {time.time() - self.debug_counters['last_callback_time'][i]:.1f}s ago")
                
                self.get_logger().info(f"✅ Total callbacks: {total_callbacks}, Processing frames...")
            
            # Reset counters
            self.debug_counters['image_callbacks'] = [0] * 6
            self.debug_counters['queue_insertions'] = [0] * 6
            self.debug_counters['inference_starts'] = [0] * 6
            self.debug_counters['inference_completions'] = [0] * 6
            self.debug_counters['publish_attempts'] = [0] * 6
            
        except Exception as e:
            self.get_logger().error(f"❌ Debug callback error: {e}")

    def log_statistics(self):
        """Enhanced statistics logging"""
        try:
            with self.lock:
                stats = self.stats.copy()
            
            current_time = time.time()
            time_since_last_update = current_time - stats['last_update_time']
            
            if stats['total_frames'] > 0:
                success_rate = (stats['successful_segmentations'] / stats['total_frames']) * 100
                self.get_logger().info(
                    f"🚀 Performance: "
                    f"Callbacks={stats['total_frames']}, "
                    f"Frames={stats['successful_segmentations']}, "
                    f"Success={success_rate:.1f}%, "
                    f"Inference={stats['average_inference_time']*1000:.0f}ms, "
                    f"FPS={stats['fps']:.1f}"
                )
                
                # Check performance
                if stats['fps'] >= 30:
                    self.get_logger().info("🎯 Good performance!")
                elif stats['fps'] >= 10:
                    self.get_logger().warn(f"⚡ Moderate performance: {stats['fps']:.1f} FPS")
                else:
                    self.get_logger().warn(f"🐌 Low performance: {stats['fps']:.1f} FPS")
            else:
                if time_since_last_update > 10:
                    self.get_logger().warn("⚠️ No frames processed in last 10 seconds - Check inference pipeline!")
                
        except Exception as e:
            self.get_logger().error(f"❌ Stats error: {e}")

    def destroy_node(self):
        """Enhanced shutdown"""
        try:
            self.get_logger().info("🛑 Shutting down segmentation node...")
            super().destroy_node()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    # Set process priority
    try:
        os.nice(-10)
    except:
        pass
    
    node = None
    try:
        node = MulticamSegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()