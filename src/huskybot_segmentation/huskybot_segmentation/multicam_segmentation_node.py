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
import multiprocessing
import ctypes

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
        
        self.get_logger().info("🚀 Ultra High-Performance YOLOv11 Segmentation + Visualization Node initialized")

    def _setup_parameters(self):
        """Setup parameters untuk 60+ FPS"""
        try:
            # Core parameters
            self.declare_parameter('cam_count', 6)
            self.declare_parameter('model_path', 'yolo11x-seg.engine')
            self.declare_parameter('device', 'cuda:0')
            self.declare_parameter('conf_thres', 0.5)  # Higher threshold for speed
            self.declare_parameter('visualization_enabled', True)
            self.declare_parameter('publish_rate', 60.0)  # Target 60 FPS
            self.declare_parameter('image_width', 1920)
            self.declare_parameter('image_height', 1080)
            
            # Ultra performance optimization parameters
            self.declare_parameter('inference_threads', 6)  # One per camera
            self.declare_parameter('input_size', 320)  # Much smaller input for speed
            self.declare_parameter('half_precision', True)  # FP16 for speed
            self.declare_parameter('batch_size', 1)
            self.declare_parameter('max_det', 50)  # Limit detections aggressively
            
            # Ultra visualization parameters
            self.declare_parameter('viz_scale', 0.25)  # Scale down visualization significantly
            self.declare_parameter('viz_fps_limit', 30.0)  # Limit viz FPS
            self.declare_parameter('show_fps', True)
            self.declare_parameter('grid_layout', True)
            self.declare_parameter('skip_masks', True)  # Skip mask processing for speed
            self.declare_parameter('simple_viz', True)  # Ultra simple visualization
            
            # Performance tuning
            self.declare_parameter('queue_size', 1)  # Minimal queue for lowest latency
            self.declare_parameter('async_publish', True)  # Async publishing
            self.declare_parameter('memory_pool', True)  # Pre-allocated memory
            
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
            
            # Performance tuning
            self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
            self.async_publish = self.get_parameter('async_publish').get_parameter_value().bool_value
            self.memory_pool = self.get_parameter('memory_pool').get_parameter_value().bool_value
            
            # Build camera topics
            self.camera_topics = []
            for i in range(self.cam_count):
                topic_param = f'camera_topic_{i}'
                topic = self.get_parameter(topic_param).get_parameter_value().string_value
                self.camera_topics.append(topic)
            
            self.get_logger().info(f"📹 Camera topics: {self.camera_topics}")
            self.get_logger().info(f"🤖 Model: {self.model_path}")
            self.get_logger().info(f"🔧 Device: {self.device}")
            self.get_logger().info(f"⚡ Ultra Performance: {self.inference_threads} threads, input_size={self.input_size}, half_precision={self.half_precision}")
            self.get_logger().info(f"🎯 Target FPS: {self.publish_rate}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up parameters: {e}")
            raise e

    def _initialize_model(self):
        """Initialize YOLO model with ultra optimizations"""
        try:
            if not ULTRALYTICS_AVAILABLE:
                raise ImportError("Ultralytics tidak tersedia")
            
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            self.get_logger().info(f"🔄 Loading model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            # For TensorRT models, device is handled in inference
            self.get_logger().info(f"✅ Model loaded successfully")
            
            # Test inference with ultra optimizations
            dummy_image = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            results = self.model(dummy_image, 
                               conf=self.conf_thres, 
                               task='segment',
                               device=self.device,
                               half=self.half_precision,
                               max_det=self.max_det,
                               verbose=False,
                               agnostic_nms=True,  # Faster NMS
                               retina_masks=False)  # Disable retina masks for speed
            
            self.get_logger().info(f"📊 Model classes: {len(self.model.names)}")
            self.get_logger().info(f"🚀 Model ready for ultra-high performance inference")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize model: {e}")
            raise e

    def _setup_threading(self):
        """Setup thread pools for ultra high performance"""
        # Dedicated thread per camera for maximum parallelization
        self.executor = ThreadPoolExecutor(max_workers=self.inference_threads + 2)  # +2 for publishing
        
        # Minimal queues for lowest latency
        self.image_queues = [queue.Queue(maxsize=self.queue_size) for _ in range(self.cam_count)]
        self.result_queues = [queue.Queue(maxsize=2) for _ in range(self.cam_count)]
        
        # Pre-allocate memory pools if enabled
        if self.memory_pool:
            self._setup_memory_pools()
        
        # Start dedicated inference threads per camera
        self.inference_threads_list = []
        for i in range(self.cam_count):
            thread = threading.Thread(target=self._inference_worker, args=(i,), daemon=True)
            thread.start()
            self.inference_threads_list.append(thread)
        
        # Async publishing threads
        if self.async_publish:
            self.publish_thread = threading.Thread(target=self._async_publisher_worker, daemon=True)
            self.publish_thread.start()

    def _setup_memory_pools(self):
        """Pre-allocate memory pools for zero-copy operations"""
        self.memory_pools = {
            'resized': [np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8) 
                       for _ in range(self.cam_count * 2)],
            'viz': [np.zeros((int(1080 * self.viz_scale), int(1920 * self.viz_scale), 3), dtype=np.uint8) 
                   for _ in range(self.cam_count * 2)]
        }
        self.pool_indices = {'resized': 0, 'viz': 0}

    def _get_from_pool(self, pool_name):
        """Get pre-allocated memory from pool"""
        if not self.memory_pool:
            return None
        
        pool = self.memory_pools[pool_name]
        idx = self.pool_indices[pool_name]
        self.pool_indices[pool_name] = (idx + 1) % len(pool)
        return pool[idx]

    def _setup_topics(self):
        """Setup subscriptions and publishers with minimal queues"""
        try:
            # Create subscriptions with minimal queue
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
            
            # Create publishers with minimal queue
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
        """Setup ultra-fast OpenCV visualization"""
        if self.visualization_enabled and self.grid_layout:
            self.latest_vis_images = [None] * self.cam_count
            self.viz_lock = threading.Lock()
            self.last_viz_time = time.time()
            
            # Start visualization thread with higher priority
            self.viz_thread = threading.Thread(target=self._visualization_worker, daemon=True)
            self.viz_thread.start()

    def _setup_timers(self):
        """Setup timers"""
        self.stats_timer = self.create_timer(2.0, self.log_statistics)  # More frequent stats

    def image_callback(self, msg, camera_index):
        """Ultra-fast image callback - minimal processing"""
        try:
            # Convert ROS image to OpenCV (fastest method)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Non-blocking queue insertion - drop if full
            if not self.image_queues[camera_index].full():
                self.image_queues[camera_index].put((cv_image, msg.header, camera_index))
            # If queue full, just drop the frame for lowest latency
            
        except Exception as e:
            self.get_logger().error(f"❌ Image callback error camera {camera_index}: {e}")

    def _inference_worker(self, camera_index):
        """Ultra-fast inference worker per camera"""
        while True:
            try:
                # Get image from queue with minimal timeout
                cv_image, header, cam_idx = self.image_queues[camera_index].get(timeout=0.5)
                
                # Ultra-fast resize using pre-allocated memory
                h, w = cv_image.shape[:2]
                scale = self.input_size / max(h, w)
                new_h, new_w = int(h * scale), int(w * scale)
                
                # Use memory pool if available
                padded_image = self._get_from_pool('resized')
                if padded_image is None:
                    padded_image = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
                else:
                    padded_image.fill(0)  # Clear previous data
                
                # Fastest resize method
                resized = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                padded_image[:new_h, :new_w] = resized
                
                # Ultra-fast inference
                start_time = time.time()
                results = self.model(padded_image,
                                   conf=self.conf_thres,
                                   task='segment',
                                   device=self.device,
                                   half=self.half_precision,
                                   max_det=self.max_det,
                                   verbose=False,
                                   agnostic_nms=True,
                                   retina_masks=False)
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
                
                # Process results immediately
                result_data = {
                    'results': results[0],
                    'original_image': cv_image,
                    'header': header,
                    'camera_index': camera_index,
                    'scale': scale,
                    'inference_time': inference_time
                }
                
                # Process and publish with minimal delay
                if self.async_publish:
                    if not self.result_queues[camera_index].full():
                        self.result_queues[camera_index].put(result_data)
                else:
                    self._process_results(result_data)
                
            except queue.Empty:
                continue
            except Exception as e:
                with self.lock:
                    self.stats['failed_segmentations'] += 1
                self.get_logger().error(f"❌ Inference worker error camera {camera_index}: {e}")

    def _async_publisher_worker(self):
        """Async publisher worker for maximum throughput"""
        while True:
            try:
                for camera_index in range(self.cam_count):
                    try:
                        result_data = self.result_queues[camera_index].get_nowait()
                        self._process_results(result_data)
                    except queue.Empty:
                        continue
                time.sleep(0.001)  # Minimal sleep
            except Exception as e:
                self.get_logger().error(f"❌ Async publisher error: {e}")

    def _process_results(self, result_data):
        """Ultra-fast result processing"""
        try:
            results = result_data['results']
            original_image = result_data['original_image']
            header = result_data['header']
            camera_index = result_data['camera_index']
            scale = result_data['scale']
            
            # Create and publish segmentation message (simplified)
            if not self.skip_masks:  # Only if masks are needed
                seg_msg = self._create_fast_segmentation_message(results, header, camera_index, scale)
                if camera_index < len(self.segmentation_pubs):
                    self.segmentation_pubs[camera_index].publish(seg_msg)
            
            # Create ultra-fast visualization
            if self.visualization_enabled:
                vis_image = self._create_ultra_fast_visualization(original_image, results, scale, result_data['inference_time'])
                
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

    def _create_fast_segmentation_message(self, results, header, camera_index, scale):
        """Ultra-fast segmentation message creation"""
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
                
                # Skip mask processing for ultra speed
                seg_msg.yolov12_inference.append(inference_result)
        
        return seg_msg

    def _create_ultra_fast_visualization(self, image, results, scale, inference_time):
        """Ultra-fast visualization - minimal drawing"""
        # Scale down for visualization using memory pool
        viz_h = int(image.shape[0] * self.viz_scale)
        viz_w = int(image.shape[1] * self.viz_scale)
        
        vis_image = self._get_from_pool('viz')
        if vis_image is None or vis_image.shape[:2] != (viz_h, viz_w):
            vis_image = cv2.resize(image, (viz_w, viz_h), interpolation=cv2.INTER_LINEAR)
        else:
            cv2.resize(image, (viz_w, viz_h), dst=vis_image, interpolation=cv2.INTER_LINEAR)
        
        if self.simple_viz and results.boxes is not None and len(results.boxes) > 0:
            # Ultra-simple visualization - only boxes
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Scale coordinates
                x1 = int((x1 / scale) * self.viz_scale)
                y1 = int((y1 / scale) * self.viz_scale)
                x2 = int((x2 / scale) * self.viz_scale)
                y2 = int((y2 / scale) * self.viz_scale)
                
                # Simple green box only
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
        
        # Add FPS info if enabled
        if self.show_fps:
            fps_text = f'FPS: {1.0/inference_time:.0f}' if inference_time > 0 else 'FPS: --'
            cv2.putText(vis_image, fps_text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return vis_image

    def _visualization_worker(self):
        """Ultra-fast grid visualization worker"""
        while True:
            try:
                current_time = time.time()
                if current_time - self.last_viz_time < (1.0 / self.viz_fps_limit):
                    time.sleep(0.005)  # Minimal sleep
                    continue
                
                with self.viz_lock:
                    images = self.latest_vis_images.copy()
                
                # Check if we have enough images
                valid_images = [img for img in images if img is not None]
                if len(valid_images) < 2:  # Lower threshold
                    time.sleep(0.01)
                    continue
                
                # Create ultra-fast grid layout
                grid_image = self._create_ultra_fast_grid_layout(images)
                if grid_image is not None:
                    cv2.imshow('HuskyBot YOLOv11 Segmentation - Ultra High Performance', grid_image)
                    cv2.waitKey(1)
                    
                    # Update viz FPS
                    viz_time = time.time() - current_time
                    with self.lock:
                        self.stats['viz_fps'] = 1.0 / viz_time if viz_time > 0 else 0
                
                self.last_viz_time = current_time
                
            except Exception as e:
                self.get_logger().error(f"❌ Visualization worker error: {e}")
                time.sleep(0.01)

    def _create_ultra_fast_grid_layout(self, images):
        """Ultra-fast 2x3 grid layout"""
        try:
            # Ultra-small target size for maximum performance
            target_width, target_height = 240, 180
            
            display_images = []
            camera_names = ['FL', 'F', 'R', 'L', 'Rear', 'RR']  # Shorter names
            
            for i, name in enumerate(camera_names):
                if i < len(images) and images[i] is not None:
                    img = cv2.resize(images[i], (target_width, target_height), interpolation=cv2.INTER_LINEAR)
                else:
                    # Simple placeholder
                    img = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                    cv2.putText(img, name, (5, target_height//2), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                
                # Add minimal camera label
                cv2.putText(img, name, (2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                display_images.append(img)
            
            # Arrange in 2x3 grid
            if len(display_images) >= 6:
                # Top row: [1], [0], [2] -> front_left, front, right
                top_row = np.hstack([display_images[1], display_images[0], display_images[2]])
                # Bottom row: [3], [4], [5] -> left, rear, rear_right  
                bottom_row = np.hstack([display_images[3], display_images[4], display_images[5]])
                
                combined = np.vstack([top_row, bottom_row])
                
                # Minimal title
                title_height = 40
                title_img = np.zeros((title_height, combined.shape[1], 3), dtype=np.uint8)
                
                with self.lock:
                    stats_text = f'HuskyBot Ultra | FPS: {self.stats["fps"]:.0f} | Viz: {self.stats["viz_fps"]:.0f} | Frames: {self.stats["total_frames"]}'
                
                cv2.putText(title_img, stats_text, (5, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(title_img, f'Inf: {self.stats["average_inference_time"]*1000:.0f}ms', 
                           (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                final_img = np.vstack([title_img, combined])
                return final_img
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"❌ Error creating grid layout: {e}")
            return None

    def log_statistics(self):
        """Log ultra performance statistics"""
        try:
            with self.lock:
                stats = self.stats.copy()
            
            if stats['total_frames'] > 0:
                success_rate = (stats['successful_segmentations'] / stats['total_frames']) * 100
                
                self.get_logger().info(
                    f"🚀 Ultra Performance: "
                    f"Frames={stats['total_frames']}, "
                    f"Success={stats['successful_segmentations']}, "
                    f"Failed={stats['failed_segmentations']}, "
                    f"Success Rate={success_rate:.1f}%, "
                    f"Inf={stats['average_inference_time']*1000:.1f}ms, "
                    f"FPS={stats['fps']:.1f}, "
                    f"VizFPS={stats['viz_fps']:.1f}"
                )
        except Exception as e:
            self.get_logger().error(f"❌ Error logging statistics: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Set process priority for better performance
    try:
        os.nice(-10)  # Higher priority
    except:
        pass
    
    try:
        node = MulticamSegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()