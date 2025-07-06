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
        
        # ULTRA-FAST debug counters
        self.debug_counters = {
            'image_callbacks': [0] * 6,
            'last_callback_time': [0.0] * 6,
            'total_callbacks': 0
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
            'viz_fps': 0.0
        }
        
        # MINIMAL class colors for speed
        self._setup_class_colors()
        
        self.get_logger().info("🚀 ULTRA-OPTIMIZED YOLOv11 Segmentation Node for 60+ FPS initialized")
        
        # Start debug timer (less frequent)
        self.debug_timer = self.create_timer(10.0, self.debug_callback_status)

    def _setup_parameters(self):
        """Setup ULTRA-OPTIMIZED parameters for 60+ FPS"""
        try:
            # Core parameters
            self.declare_parameter('cam_count', 6)
            self.declare_parameter('model_path', 'yolo11n-seg.engine')
            self.declare_parameter('device', 'cuda:0')
            self.declare_parameter('conf_thres', 0.7)  # Higher for speed
            self.declare_parameter('visualization_enabled', True)
            self.declare_parameter('publish_rate', 60.0)  # TARGET 60 FPS
            
            # ULTRA-AGGRESSIVE Performance parameters
            self.declare_parameter('inference_threads', 6)  # One per camera
            self.declare_parameter('input_size', 256)  # SMALLER for speed
            self.declare_parameter('half_precision', True)
            self.declare_parameter('batch_size', 1)
            self.declare_parameter('max_det', 10)  # FEWER detections
            
            # MINIMAL visualization
            self.declare_parameter('viz_scale', 0.25)
            self.declare_parameter('viz_fps_limit', 30.0)
            self.declare_parameter('show_fps', True)
            self.declare_parameter('grid_layout', False)  # DISABLE
            self.declare_parameter('skip_masks', True)  # SKIP masks
            self.declare_parameter('simple_viz', True)
            self.declare_parameter('show_confidence', False)  # DISABLE
            self.declare_parameter('show_labels', False)  # DISABLE
            self.declare_parameter('mask_alpha', 0.1)
            
            # MAXIMUM speed optimizations
            self.declare_parameter('queue_size', 1)
            self.declare_parameter('async_publish', True)
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
            
            self.get_logger().info(f"🎯 TARGET: {self.publish_rate} FPS with input_size={self.input_size}")
            self.get_logger().info(f"⚡ ULTRA-OPTIMIZED: {self.inference_threads} threads, max_det={self.max_det}")
            self.get_logger().info(f"🎨 MINIMAL VIZ: skip_masks={self.skip_masks}, simple={self.simple_viz}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up parameters: {e}")
            raise e

    def _initialize_model(self):
        """Initialize YOLO model with MAXIMUM optimizations for 60+ FPS"""
        try:
            if not ULTRALYTICS_AVAILABLE:
                raise ImportError("Ultralytics tidak tersedia")
            
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            self.get_logger().info(f"🔄 Loading ULTRA-FAST model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            # MINIMAL warm-up for speed
            self.get_logger().info(f"🔥 Quick model warm-up...")
            dummy_image = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            
            # Single warm-up for speed
            results = self.model(dummy_image, 
                               conf=self.conf_thres, 
                               task='segment',
                               device=self.device,
                               half=self.half_precision,
                               max_det=self.max_det,
                               verbose=False,
                               stream=False,
                               # AGGRESSIVE optimizations
                               agnostic_nms=True,
                               classes=None,  # All classes
                               retina_masks=False,  # Faster masks
                               imgsz=self.input_size)
            
            self.get_logger().info(f"✅ ULTRA-FAST model ready for 60+ FPS")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize model: {e}")
            raise e

    def _setup_threading(self):
        """Setup ULTRA-OPTIMIZED threading for 60+ FPS"""
        # Dedicated thread pool for max performance
        self.thread_pool = ThreadPoolExecutor(max_workers=self.inference_threads + 2)
        
        # Minimal queues for ZERO latency
        self.image_queues = [queue.Queue(maxsize=1) for _ in range(self.cam_count)]
        self.result_queues = [queue.Queue(maxsize=1) for _ in range(self.cam_count)]
        
        # Pre-allocate memory pools for speed
        if self.memory_pool:
            self._setup_memory_pools()
        
        # Start DEDICATED inference threads (one per camera for max speed)
        self.inference_threads_list = []
        for i in range(self.cam_count):
            thread = threading.Thread(target=self._dedicated_inference_worker, args=(i,), daemon=True)
            thread.start()
            self.inference_threads_list.append(thread)
        
        # Async publishing for speed
        if self.async_publish:
            self.publish_thread = threading.Thread(target=self._ultra_fast_publisher_worker, daemon=True)
            self.publish_thread.start()

    def _setup_memory_pools(self):
        """Pre-allocate OPTIMIZED memory pools"""
        self.memory_pools = {
            'resized': [np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8) 
                       for _ in range(self.cam_count)],
            'viz': [np.zeros((int(1080 * self.viz_scale), int(1920 * self.viz_scale), 3), dtype=np.uint8) 
                   for _ in range(self.cam_count)]
        }

    def _setup_topics(self):
        """Setup topics with minimal overhead"""
        try:
            # Create subscriptions with minimal queue
            self.image_subs = []
            for i, topic in enumerate(self.camera_topics):
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_idx=i: self.ultra_fast_image_callback(msg, cam_idx),
                    1  # Minimal queue
                )
                self.image_subs.append(sub)
            
            # Create publishers
            self.segmentation_pubs = []
            self.visualization_pubs = []
            
            camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
            for i in range(self.cam_count):
                # Segmentation publisher
                seg_topic = f'/camera_{camera_names[i]}/segmentation'
                seg_pub = self.create_publisher(Yolov12Inference, seg_topic, 1)
                self.segmentation_pubs.append(seg_pub)
                
                # Visualization publisher
                if self.visualization_enabled:
                    vis_topic = f'/camera_{camera_names[i]}/segmentation_vis'
                    vis_pub = self.create_publisher(Image, vis_topic, 1)
                    self.visualization_pubs.append(vis_pub)
            
            self.get_logger().info(f"📤 Created publishers for 60+ FPS processing")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up topics: {e}")
            raise e

    def _setup_visualization(self):
        """MINIMAL visualization setup"""
        if self.visualization_enabled and not self.grid_layout:
            self.latest_vis_images = [None] * self.cam_count
            self.viz_lock = threading.Lock()

    def _setup_timers(self):
        """Setup minimal timers"""
        self.stats_timer = self.create_timer(5.0, self.log_statistics)

    def _setup_class_colors(self):
        """Minimal color setup for speed"""
        # Simple colors for speed
        self.class_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 255, 0), (255, 128, 0), (128, 0, 255), (255, 0, 128)
        ] * 8  # 80 colors total

    def ultra_fast_image_callback(self, msg, camera_index):
        """ULTRA-FAST image callback with minimal overhead"""
        try:
            # Minimal debug tracking
            self.debug_counters['image_callbacks'][camera_index] += 1
            self.debug_counters['last_callback_time'][camera_index] = time.time()
            
            # Frame skipping check
            self.frame_counters[camera_index] += 1
            if self.frame_counters[camera_index] % self.process_every_nth_frame != 0:
                return
            
            # ULTRA-FAST image conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # NON-BLOCKING queue insertion with immediate drop if full
            try:
                self.image_queues[camera_index].put_nowait((cv_image, msg.header, camera_index))
            except queue.Full:
                pass  # Drop frame if queue full for zero latency
            
        except Exception as e:
            pass  # Ignore errors for maximum speed

    def _dedicated_inference_worker(self, camera_index):
        """DEDICATED inference worker per camera for MAXIMUM speed"""
        while True:
            try:
                # Get image (blocking)
                cv_image, header, cam_idx = self.image_queues[camera_index].get()
                
                # ULTRA-FAST preprocessing
                h, w = cv_image.shape[:2]
                
                # Simple resize for MAXIMUM speed (no aspect ratio preservation)
                resized = cv2.resize(cv_image, (self.input_size, self.input_size), 
                                   interpolation=cv2.INTER_LINEAR)
                
                # Calculate simple scale
                scale = min(self.input_size / w, self.input_size / h)
                
                # ULTRA-FAST inference with AGGRESSIVE optimizations
                start_time = time.time()
                results = self.model(resized,
                                   conf=self.conf_thres,
                                   task='detect' if self.skip_masks else 'segment',  # Use detection if skipping masks
                                   device=self.device,
                                   half=self.half_precision,
                                   max_det=self.max_det,
                                   verbose=False,
                                   stream=False,
                                   # MAXIMUM speed optimizations
                                   agnostic_nms=True,
                                   retina_masks=False,
                                   save=False,
                                   save_txt=False,
                                   save_conf=False,
                                   imgsz=self.input_size)
                inference_time = time.time() - start_time
                
                # Update statistics (minimal)
                with self.lock:
                    self.stats['total_frames'] += 1
                    self.stats['successful_segmentations'] += 1
                    self.stats['average_inference_time'] = inference_time
                    self.stats['fps'] = 1.0 / inference_time if inference_time > 0 else 0
                
                # Create result data
                result_data = {
                    'results': results[0],
                    'original_image': cv_image,
                    'header': header,
                    'camera_index': camera_index,
                    'scale': scale,
                    'inference_time': inference_time
                }
                
                # Process results
                if self.async_publish:
                    try:
                        self.result_queues[camera_index].put_nowait(result_data)
                    except queue.Full:
                        pass  # Drop if full
                else:
                    self._ultra_fast_process_results(result_data)
                
            except Exception as e:
                with self.lock:
                    self.stats['failed_segmentations'] += 1

    def _ultra_fast_publisher_worker(self):
        """ULTRA-FAST async publisher worker"""
        while True:
            try:
                for camera_index in range(self.cam_count):
                    try:
                        result_data = self.result_queues[camera_index].get_nowait()
                        self._ultra_fast_process_results(result_data)
                    except queue.Empty:
                        continue
            except Exception as e:
                pass

    def _ultra_fast_process_results(self, result_data):
        """ULTRA-FAST result processing"""
        try:
            results = result_data['results']
            camera_index = result_data['camera_index']
            scale = result_data['scale']
            header = result_data['header']
            
            # Create MINIMAL segmentation message
            seg_msg = Yolov12Inference()
            seg_msg.header = header
            seg_msg.camera_name = f'camera_{camera_index}'
            seg_msg.frame_type = 'detection' if self.skip_masks else 'segmentation'
            seg_msg.task = 'detect' if self.skip_masks else 'segment'
            
            # Process detections (minimal)
            if results.boxes is not None and len(results.boxes) > 0:
                for box in results.boxes:
                    inference_result = InferenceResult()
                    inference_result.class_name = results.names[int(box.cls)]
                    inference_result.confidence = float(box.conf)
                    
                    # Scale bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    inference_result.left = int(x1 / scale)
                    inference_result.top = int(y1 / scale)
                    inference_result.right = int(x2 / scale)
                    inference_result.bottom = int(y2 / scale)
                    
                    seg_msg.yolov12_inference.append(inference_result)
            
            # Publish segmentation
            if camera_index < len(self.segmentation_pubs):
                self.segmentation_pubs[camera_index].publish(seg_msg)
            
            # MINIMAL visualization if enabled
            if self.visualization_enabled and self.simple_viz:
                vis_image = self._create_ultra_fast_visualization(
                    result_data['original_image'], results, scale)
                
                if camera_index < len(self.visualization_pubs):
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                    vis_msg.header = header
                    self.visualization_pubs[camera_index].publish(vis_msg)
                
        except Exception as e:
            pass  # Ignore errors for speed

    def _create_ultra_fast_visualization(self, image, results, scale):
        """ULTRA-FAST minimal visualization"""
        # Small visualization for speed
        viz_h = int(image.shape[0] * self.viz_scale)
        viz_w = int(image.shape[1] * self.viz_scale)
        vis_image = cv2.resize(image, (viz_w, viz_h), interpolation=cv2.INTER_LINEAR)
        
        # Draw ONLY bounding boxes (no masks, labels, or confidence)
        if results.boxes is not None and len(results.boxes) > 0:
            for i, box in enumerate(results.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Scale to visualization
                x1 = int((x1 / scale) * self.viz_scale)
                y1 = int((y1 / scale) * self.viz_scale)
                x2 = int((x2 / scale) * self.viz_scale)
                y2 = int((y2 / scale) * self.viz_scale)
                
                # Simple rectangle only
                color = self.class_colors[int(box.cls) % len(self.class_colors)]
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
        
        return vis_image

    def debug_callback_status(self):
        """Minimal debug callback"""
        total_callbacks = sum(self.debug_counters['image_callbacks'])
        if total_callbacks == 0:
            self.get_logger().warn("⚠️ NO CAMERA DATA - Check camera topics!")
        else:
            self.get_logger().info(f"✅ Processing {total_callbacks} callbacks")
        self.debug_counters['image_callbacks'] = [0] * 6

    def log_statistics(self):
        """Log performance statistics"""
        try:
            with self.lock:
                stats = self.stats.copy()
            
            if stats['total_frames'] > 0:
                success_rate = (stats['successful_segmentations'] / stats['total_frames']) * 100
                self.get_logger().info(
                    f"🚀 ULTRA-PERFORMANCE: "
                    f"Frames={stats['total_frames']}, "
                    f"Success={success_rate:.1f}%, "
                    f"Inference={stats['average_inference_time']*1000:.1f}ms, "
                    f"FPS={stats['fps']:.1f}"
                )
                
                # Check if we're meeting target
                if stats['fps'] >= 60:
                    self.get_logger().info("🎯 TARGET 60+ FPS ACHIEVED!")
                elif stats['fps'] >= 30:
                    self.get_logger().warn(f"⚡ Approaching target: {stats['fps']:.1f} FPS")
                else:
                    self.get_logger().warn(f"🐌 Below target: {stats['fps']:.1f} FPS")
                
        except Exception as e:
            pass

    def destroy_node(self):
        """Fast shutdown"""
        try:
            self.get_logger().info("🛑 Shutting down ULTRA-FAST node...")
            super().destroy_node()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    # Set MAXIMUM process priority
    try:
        os.nice(-20)
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