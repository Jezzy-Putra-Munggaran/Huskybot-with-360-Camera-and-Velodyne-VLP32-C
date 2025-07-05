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
import platform
import math

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
        
        # Setup parameters dengan handling yang lebih robust
        self._setup_parameters()
        
        # Initialize model
        self._initialize_model()
        
        # Setup subscriptions and publishers
        self._setup_topics()
        
        # Initialize timers
        self._setup_timers()
        
        # Statistics
        self.stats = {
            'total_frames': 0,
            'successful_segmentations': 0,
            'failed_segmentations': 0,
            'average_inference_time': 0.0
        }
        
        self.get_logger().info("🚀 YOLOv11 Segmentation Node initialized successfully")

    def _setup_parameters(self):
        """Setup parameters dengan handling tipe data yang robust"""
        try:
            # FIXED: Declare parameters dengan default values yang tepat
            self.declare_parameter('cam_count', 6)
            self.declare_parameter('model_path', 'yolo11x-seg.engine') 
            self.declare_parameter('device', 'cuda:0')  # Pastikan default string
            self.declare_parameter('conf_thres', 0.25)
            self.declare_parameter('visualization_enabled', True)
            self.declare_parameter('publish_rate', 10.0)
            self.declare_parameter('image_width', 1920)
            self.declare_parameter('image_height', 1080)
            self.declare_parameter('max_detection_distance', 50.0)
            self.declare_parameter('min_detection_size', 0.01)
            self.declare_parameter('enable_diagnostics', True)
            self.declare_parameter('log_level', 'INFO')
            
            # Declare camera topics sebagai individual parameters
            camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
            for i in range(6):
                topic_param = f'camera_topic_{i}'
                default_topic = f'/camera_{camera_names[i]}/image_raw'
                self.declare_parameter(topic_param, default_topic)
            
            # FIXED: Get parameters dengan type checking
            self.cam_count = self.get_parameter('cam_count').get_parameter_value().integer_value
            self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
            
            # Handle device parameter dengan type checking
            device_param = self.get_parameter('device')
            if device_param.type_ == Parameter.Type.STRING:
                self.device = device_param.get_parameter_value().string_value
            elif device_param.type_ == Parameter.Type.INTEGER:
                # Convert integer to proper device string
                device_int = device_param.get_parameter_value().integer_value
                self.device = f'cuda:{device_int}' if device_int >= 0 else 'cpu'
                self.get_logger().warn(f"Device parameter was integer ({device_int}), converted to: {self.device}")
            else:
                self.device = 'cuda:0'  # Safe fallback
                self.get_logger().warn(f"Device parameter type unexpected, using fallback: {self.device}")
            
            self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
            self.visualization_enabled = self.get_parameter('visualization_enabled').get_parameter_value().bool_value
            self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
            self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
            
            # Build camera topics list dari individual parameters
            self.camera_topics = []
            for i in range(self.cam_count):
                topic_param = f'camera_topic_{i}'
                topic = self.get_parameter(topic_param).get_parameter_value().string_value
                self.camera_topics.append(topic)
            
            # Log parameter values
            self.get_logger().info(f"📹 Camera topics: {self.camera_topics}")
            self.get_logger().info(f"🤖 Model: {self.model_path}")
            self.get_logger().info(f"🔧 Device: {self.device} (type: {type(self.device)})")
            self.get_logger().info(f"⚙️ Confidence threshold: {self.conf_thres}")
            self.get_logger().info(f"🎨 Visualization enabled: {self.visualization_enabled}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up parameters: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            raise e

    def _initialize_model(self):
        """Initialize YOLO model"""
        try:
            if not ULTRALYTICS_AVAILABLE:
                raise ImportError("Ultralytics tidak tersedia")
            
            # Validate model file exists
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            # Load model
            self.get_logger().info(f"🔄 Loading model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            # Move to device dengan handling yang lebih robust
            if 'cuda' in self.device and self.device != 'cpu':
                try:
                    self.model.to('cuda')
                    self.get_logger().info(f"🚀 Model moved to CUDA device: {self.device}")
                except Exception as cuda_err:
                    self.get_logger().warn(f"⚠️ Failed to move model to CUDA: {cuda_err}")
                    self.device = 'cpu'
                    self.get_logger().info("💻 Falling back to CPU")
            else:
                self.get_logger().info("💻 Model using CPU")
            
            # Test inference untuk memastikan model bekerja
            dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
            results = self.model(dummy_image, conf=self.conf_thres, task='segment', verbose=False)
            
            self.get_logger().info(f"✅ YOLOv11 Segmentation model loaded successfully")
            self.get_logger().info(f"📊 Model classes: {len(self.model.names)}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize model: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            raise e

    def _setup_topics(self):
        """Setup subscriptions and publishers"""
        try:
            # Create subscriptions for all cameras
            self.image_subs = []
            for i, topic in enumerate(self.camera_topics):
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_idx=i: self.image_callback(msg, cam_idx),
                    10
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
                seg_pub = self.create_publisher(Yolov12Inference, seg_topic, 10)
                self.segmentation_pubs.append(seg_pub)
                
                # Visualization publisher
                if self.visualization_enabled:
                    vis_topic = f'/camera_{camera_names[i]}/segmentation_vis'
                    vis_pub = self.create_publisher(Image, vis_topic, 10)
                    self.visualization_pubs.append(vis_pub)
            
            self.get_logger().info(f"📤 Created {len(self.segmentation_pubs)} segmentation publishers")
            if self.visualization_enabled:
                self.get_logger().info(f"🎨 Created {len(self.visualization_pubs)} visualization publishers")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up topics: {e}")
            raise e

    def _setup_timers(self):
        """Setup processing timers"""
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.log_statistics)

    def image_callback(self, msg, camera_index):
        """Process incoming images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run segmentation
            start_time = time.time()
            results = self.model(cv_image, conf=self.conf_thres, task='segment', verbose=False)
            inference_time = time.time() - start_time
            
            # Update statistics
            with self.lock:
                self.stats['total_frames'] += 1
                self.stats['successful_segmentations'] += 1
                self.stats['average_inference_time'] = (
                    (self.stats['average_inference_time'] * (self.stats['total_frames'] - 1) + inference_time) 
                    / self.stats['total_frames']
                )
            
            # Process results
            self._process_segmentation_results(results[0], msg, camera_index, cv_image)
            
        except Exception as e:
            with self.lock:
                self.stats['failed_segmentations'] += 1
            self.get_logger().error(f"❌ Segmentation error camera {camera_index}: {e}")

    def _process_segmentation_results(self, result, original_msg, camera_index, original_image):
        """Process and publish segmentation results"""
        try:
            # Create result message
            seg_msg = Yolov12Inference()
            seg_msg.header = original_msg.header
            seg_msg.camera_name = f'camera_{camera_index}'
            seg_msg.frame_type = 'segmentation'
            seg_msg.task = 'segment'
            
            # Process detections
            if result.boxes is not None and len(result.boxes) > 0:
                for i, box in enumerate(result.boxes):
                    inference_result = InferenceResult()
                    inference_result.class_name = result.names[int(box.cls)]
                    inference_result.confidence = float(box.conf)
                    
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    inference_result.left = int(x1)
                    inference_result.top = int(y1) 
                    inference_result.right = int(x2)
                    inference_result.bottom = int(y2)
                    
                    # Mask indices (if available)
                    if result.masks is not None and i < len(result.masks.data):
                        mask = result.masks.data[i].cpu().numpy()
                        # Convert mask to indices
                        mask_indices = np.where(mask > 0.5)
                        if len(mask_indices[0]) > 0:
                            # Flatten and convert to list
                            mask_flat = []
                            for row, col in zip(mask_indices[0], mask_indices[1]):
                                mask_flat.append(int(row * mask.shape[1] + col))
                            # Limit size to avoid message overflow
                            inference_result.mask_indices = mask_flat[:2000]
                    
                    seg_msg.yolov12_inference.append(inference_result)
            
            # Publish segmentation results
            if camera_index < len(self.segmentation_pubs):
                self.segmentation_pubs[camera_index].publish(seg_msg)
            
            # Create and publish visualization
            if self.visualization_enabled and camera_index < len(self.visualization_pubs):
                vis_image = self._create_visualization(original_image, result)
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                vis_msg.header = original_msg.header
                self.visualization_pubs[camera_index].publish(vis_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing segmentation results: {e}")

    def _create_visualization(self, image, result):
        """Create visualization with segmentation masks and bounding boxes"""
        try:
            vis_image = image.copy()
            
            if result.boxes is not None and len(result.boxes) > 0:
                # Draw masks first
                if result.masks is not None:
                    for i, mask in enumerate(result.masks.data):
                        mask_np = mask.cpu().numpy()
                        mask_resized = cv2.resize(mask_np, (image.shape[1], image.shape[0]))
                        
                        # Create colored mask
                        colors = [
                            (255, 0, 0), (0, 255, 0), (0, 0, 255),
                            (255, 255, 0), (255, 0, 255), (0, 255, 255),
                            (128, 0, 128), (255, 165, 0), (255, 192, 203)
                        ]
                        color = colors[i % len(colors)]
                        
                        colored_mask = np.zeros_like(vis_image)
                        colored_mask[mask_resized > 0.5] = color
                        
                        # Blend with original
                        vis_image = cv2.addWeighted(vis_image, 0.7, colored_mask, 0.3, 0)
                
                # Draw bounding boxes and labels
                for i, box in enumerate(result.boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf)
                    cls_name = result.names[int(box.cls)]
                    
                    # Draw box
                    cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label
                    label = f'{cls_name}: {conf:.2f}'
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(vis_image, (x1, y1 - h - 10), (x1 + w, y1), (0, 255, 0), -1)
                    cv2.putText(vis_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            
            return vis_image
            
        except Exception as e:
            self.get_logger().error(f"❌ Error creating visualization: {e}")
            return image

    def log_statistics(self):
        """Log performance statistics"""
        try:
            with self.lock:
                stats = self.stats.copy()
            
            if stats['total_frames'] > 0:
                success_rate = (stats['successful_segmentations'] / stats['total_frames']) * 100
                fps = 1.0 / stats['average_inference_time'] if stats['average_inference_time'] > 0 else 0
                
                self.get_logger().info(
                    f"📊 Segmentation Stats: "
                    f"Frames={stats['total_frames']}, "
                    f"Success={stats['successful_segmentations']}, "
                    f"Failed={stats['failed_segmentations']}, "
                    f"Success Rate={success_rate:.1f}%, "
                    f"Avg Inference={stats['average_inference_time']*1000:.1f}ms, "
                    f"FPS={fps:.1f}"
                )
        except Exception as e:
            self.get_logger().error(f"❌ Error logging statistics: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MulticamSegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()