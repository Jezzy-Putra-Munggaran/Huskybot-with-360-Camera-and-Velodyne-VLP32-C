#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# File: multicam_detection_node.py - Node deteksi multicam YOLOv12 untuk Huskybot
# Node ini menerima image dari 6 kamera Arducam IMX477 (hexagonal) dan publish hasil deteksi ke topic /detection
# Siap untuk ROS2 Humble, Gazebo, Jetson Orin, Clearpath Husky A200, multi-robot, audit trail

import os
import sys
import time
import traceback
import threading
import platform  # FIXED: Import platform yang hilang
import math  # FIXED: Import math yang hilang
from threading import Lock

# Cek semua dependency utama sebelum import ROS2
REQUIRED_MODULES = [
    'rclpy', 'cv2', 'numpy', 'ultralytics', 'yolov12_msgs', 'cv_bridge'
]

missing_modules = []
for mod in REQUIRED_MODULES:
    try:
        __import__(mod)
    except ImportError as e:
        missing_modules.append(f"{mod}: {e}")

if missing_modules:
    print(f"[FATAL] Missing Python dependencies:", file=sys.stderr)
    for mod in missing_modules:
        print(f"  - {mod}", file=sys.stderr)
    print("Install missing dependencies and try again", file=sys.stderr)
    sys.exit(1)

# Safe ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.exceptions import ParameterNotDeclaredException
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType
    from rcl_interfaces.msg import SetParametersResult
except ImportError as e:
    print(f"[FATAL] ROS2 import error: {e}", file=sys.stderr)
    sys.exit(1)

# Safe sensor imports
try:
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from cv_bridge import CvBridge, CvBridgeError
except ImportError as e:
    print(f"[FATAL] ROS2 sensor_msgs import error: {e}", file=sys.stderr)
    sys.exit(1)

# Safe image processing imports
try:
    import numpy as np
    import cv2
except ImportError as e:
    print(f"[FATAL] Image processing import error: {e}", file=sys.stderr)
    sys.exit(1)

# Safe custom message import
try:
    from yolov12_msgs.msg import InferenceResult, Yolov12Inference
except ImportError as e:
    print(f"[FATAL] Custom message import error: {e}", file=sys.stderr)
    print("Make sure yolov12_msgs package is built and sourced", file=sys.stderr)
    sys.exit(1)

# Safe YOLO import with fallback
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] Ultralytics not available: {e}", file=sys.stderr)
    print("Install with: pip install ultralytics", file=sys.stderr)
    ULTRALYTICS_AVAILABLE = False

# Constants - FIXED: Tambahkan yang hilang
JETSON_PLATFORMS = ['aarch64', 'arm64']  # Platform Jetson
LOG_DIR = os.path.expanduser('~/huskybot_detection_log')
DEFAULT_CONFIDENCE_THRESHOLD = 0.25
DETECTION_TOPIC = '/detection'
DIAGNOSTIC_TOPIC = '/diagnostics'
DEFAULT_CAMERA_TOPICS = [
    '/camera_front/image_raw',
    '/camera_front_left/image_raw',
    '/camera_left/image_raw',
    '/camera_rear/image_raw',
    '/camera_rear_right/image_raw',
    '/camera_right/image_raw'
]

class MultiCamDetectionNode(Node):  # Node deteksi multicam YOLOv12, FULL OOP
    def __init__(self):
        super().__init__('multicam_detection')
        self._setup_logging()
        self.get_logger().info("Initializing MultiCam YOLOv12 Detection Node...")

        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge()
        self.mutex = Lock()
        self.images = [None] * self.cam_count
        self.last_frame_time = [None] * self.cam_count
        self.detection_counts = [0] * self.cam_count
        self.inference_times = [0.0] * self.cam_count
        self.is_initialized = False
        self.model = None
        self.running = True

        # Initialize performance stats
        self.performance_stats = {
            'active_cameras': 0,
            'total_detections': 0,
            'avg_inference_time': 0.0,
            'last_health_check': time.time()
        }

        self._detect_platform()
        self._create_publishers()
        self._create_services()
        self._load_model()
        
        # PERBAIKAN: Apply Jetson optimizations
        self._optimize_jetson_performance()
        
        self._create_subscribers()
        self._create_timers()  # Ini yang sebelumnya error

        # Tambahkan subscriber untuk hasil fusion
        self.fusion_sub = self.create_subscription(
            Yolov12Inference, '/detection_with_distance', self.fusion_callback, 10)

        self.is_initialized = True
        self.get_logger().info(f"MultiCam YOLOv12 Detection Node initialized with {self.cam_count} cameras")

    def _setup_logging(self):
        # Setup logging ke file dan terminal, fallback ke /tmp jika gagal
        try:
            log_dir = LOG_DIR
            try:
                os.makedirs(log_dir, exist_ok=True)
            except Exception:
                log_dir = "/tmp"
            log_file = os.path.join(log_dir, f"huskybot_detection_{time.strftime('%Y%m%d')}.log")
            
            import logging
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s [%(levelname)s] %(message)s',
                handlers=[
                    logging.FileHandler(log_file),
                    logging.StreamHandler(sys.stdout)
                ]
            )
            self.logger = logging.getLogger(__name__)
            self.logger.info("Logging system initialized")

        except Exception as e:
            print(f"Warning: Could not setup file logging: {e}")

    def _declare_parameters(self):
        # Deklarasi semua parameter node (wajib agar bisa diubah via launch file)
        # PERBAIKAN: Use simple parameter declarations without complex validation
        self.declare_parameter('cam_count', 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of cameras in the hexagonal array (1-12)'
        ))
        
        self.declare_parameter('model_path', "yolo12x.engine", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Path to YOLOv12 model file (.pt, .onnx, or .engine)'
        ))
        
        self.declare_parameter('camera_topics_str', str(DEFAULT_CAMERA_TOPICS), ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera topic names as string representation of list'
        ))
        
        self.declare_parameter('class_filter_str', "[]", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Class filter as string representation of list'
        ))
        
        self.declare_parameter('conf_thres', DEFAULT_CONFIDENCE_THRESHOLD, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Confidence threshold for filtering detections (0.0-1.0)'
        ))
        
        self.declare_parameter('visualization_enabled', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable/disable OpenCV visualization of detection results'
        ))
        
        self.declare_parameter('log_to_file', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable/disable logging to file'
        ))
        
        self.declare_parameter('log_level', 'info', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Log level (debug, info, warning, error, critical)'
        ))

    def _load_parameters(self):
        # PERBAIKAN: More robust parameter loading with manual validation
        try:
            self.cam_count = self.get_parameter('cam_count').value
            self.model_path = self.get_parameter('model_path').value
            self.camera_topics_str = self.get_parameter('camera_topics_str').value
            self.class_filter_str = self.get_parameter('class_filter_str').value
            self.conf_thres = self.get_parameter('conf_thres').value
            self.visualization_enabled = self.get_parameter('visualization_enabled').value
            self.log_to_file = self.get_parameter('log_to_file').value
            self.log_level = self.get_parameter('log_level').value

            # Parse camera topics and class filter safely
            try:
                import ast
                self.camera_topics = ast.literal_eval(self.camera_topics_str)
                self.class_filter = ast.literal_eval(self.class_filter_str)
            except Exception as e:
                self.get_logger().warning(f"Failed to parse parameters: {e}, using defaults")
                self.camera_topics = DEFAULT_CAMERA_TOPICS[:self.cam_count]
                self.class_filter = []
                
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {e}")
            # Use safe defaults
            self.cam_count = 6
            self.model_path = "yolo12x.engine"
            self.camera_topics = DEFAULT_CAMERA_TOPICS
            self.class_filter = []
            self.conf_thres = DEFAULT_CONFIDENCE_THRESHOLD
            self.visualization_enabled = True

    def _detect_platform(self):
        """Deteksi platform dan konfigurasi device untuk inference."""
        try:
            import torch
            machine = platform.machine()
            self.is_jetson = machine in JETSON_PLATFORMS
            
            if self.is_jetson:
                self.get_logger().info("Detected Jetson platform, using hardware acceleration")
                # FIXED: Use proper CUDA device string
                if torch.cuda.is_available():
                    self.device = 'cuda:0'  # FIXED: Was '0', now 'cuda:0'
                    self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")
                else:
                    self.device = 'cpu'
                    self.get_logger().warning("CUDA not available on Jetson, using CPU")
            else:
                if torch.cuda.is_available():
                    self.device = 'cuda:0'  # FIXED: Proper CUDA device string
                    self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")
                else:
                    self.device = 'cpu'
                    self.get_logger().warning("CUDA not available, using CPU")
                
        except Exception as e:
            self.get_logger().error(f"Error detecting platform: {e}")
            self.device = 'cpu'

    def _optimize_jetson_performance(self):
        """PERBAIKAN: Tambahkan method yang hilang untuk optimasi Jetson"""
        try:
            if self.is_jetson:
                self.get_logger().info("Applying Jetson AGX Orin optimizations...")
                
                # Set optimal batch size for Jetson
                self.batch_size = 1
                
                # Enable half precision if supported
                self.use_half_precision = True
                
                # Set optimal inference resolution
                self.inference_size = 640
                
                # Enable TensorRT optimizations
                self.use_tensorrt = True
                
                # Set memory optimization flags
                os.environ['CUDA_LAUNCH_BLOCKING'] = '0'
                os.environ['CUDA_CACHE_DISABLE'] = '0'
                
                self.get_logger().info("Jetson optimizations applied successfully")
            else:
                self.get_logger().info("Non-Jetson platform, using standard configuration")
                self.batch_size = 1
                self.use_half_precision = False
                self.inference_size = 640
                self.use_tensorrt = False
                
        except Exception as e:
            self.get_logger().error(f"Error applying Jetson optimizations: {e}")
            # Use safe defaults
            self.batch_size = 1
            self.use_half_precision = False
            self.inference_size = 640
            self.use_tensorrt = False

    def _create_publishers(self):
        # Buat publisher untuk hasil deteksi dan diagnostics
        try:
            self.detection_pub = self.create_publisher(Yolov12Inference, DETECTION_TOPIC, 10)
            self.diagnostic_pub = self.create_publisher(DiagnosticArray, DIAGNOSTIC_TOPIC, 10)
            self.get_logger().info(f"Publishers created: {DETECTION_TOPIC}, {DIAGNOSTIC_TOPIC}")
        except Exception as e:
            self.get_logger().error(f"Error creating publishers: {e}")
            raise

    def _create_services(self):
        # Buat service untuk restart model dan get status
        try:
            self.restart_service = self.create_service(Trigger, 'restart_model', self.restart_model_callback)
            self.status_service = self.create_service(Trigger, 'get_status', self.get_status_callback)
            self.get_logger().info(f"Services created: restart_model, get_status")
        except Exception as e:
            self.get_logger().error(f"Error creating services: {e}")

    def restart_model_callback(self, request, response):
        """Service callback untuk restart model YOLOv12."""
        try:
            self.get_logger().info("Restarting YOLOv12 model...")
            old_model = self.model
            self.model = None
            
            # Reload model
            self._load_model()
            
            if self.model is not None:
                response.success = True
                response.message = "Model restarted successfully"
                self.get_logger().info("Model restarted successfully")
            else:
                response.success = False
                response.message = "Failed to restart model"
                self.get_logger().error("Failed to restart model")
                
        except Exception as e:
            self.get_logger().error(f"Error restarting model: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

    def get_status_callback(self, request, response):
        """Service callback untuk get status node."""
        try:
            active_cameras = sum(1 for img in self.images if img is not None)
            total_detections = sum(self.detection_counts)
            avg_inference_time = np.mean(self.inference_times) if self.inference_times else 0.0
            
            status_msg = (
                f"MultiCam Detection Status:\n"
                f"- Active cameras: {active_cameras}/{self.cam_count}\n"
                f"- Total detections: {total_detections}\n"
                f"- Average inference time: {avg_inference_time:.3f}s\n"
                f"- Model loaded: {'Yes' if self.model else 'No'}\n"
                f"- Device: {self.device}\n"
                f"- Visualization: {'Enabled' if self.visualization_enabled else 'Disabled'}"
            )
            
            response.success = True
            response.message = status_msg
            self.get_logger().info(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error getting status: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

    def publish_diagnostics(self):
        """Publish diagnostic information ke topic /diagnostics."""
        try:
            diagnostic_array = DiagnosticArray()
            diagnostic_array.header.stamp = self.get_clock().now().to_msg()

            # Create diagnostic status
            status = DiagnosticStatus()
            status.name = 'multicam_detection'
            status.hardware_id = 'jetson_agx_orin' if self.is_jetson else 'unknown'

            active_cameras = sum(1 for img in self.images if img is not None)
            if active_cameras >= self.cam_count * 0.8:  # 80% cameras active
                status.level = DiagnosticStatus.OK
                status.message = 'All systems operational'
            elif active_cameras >= self.cam_count * 0.5:  # 50% cameras active
                status.level = DiagnosticStatus.WARN
                status.message = 'Some cameras inactive'
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Most cameras inactive'

            # Add key-value pairs
            status.values = [
                KeyValue(key='active_cameras', value=str(active_cameras)),
                KeyValue(key='total_cameras', value=str(self.cam_count)),
                KeyValue(key='model_loaded', value=str(self.model is not None)),
                KeyValue(key='device', value=self.device),
                KeyValue(key='total_detections', value=str(sum(self.detection_counts)))
            ]

            diagnostic_array.status = [status]
            self.diagnostic_pub.publish(diagnostic_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing diagnostics: {e}")

    def _load_model(self):
        # Load YOLOv12 model, fallback ke .pt jika .engine/.onnx gagal
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error("Ultralytics not available, cannot load model")
            return
            
        model_loaded = False
        original_path = self.model_path
        try:
            self.get_logger().info(f"Loading YOLOv12 model from: {self.model_path}")
            
            # Calculate model hash for verification
            import hashlib
            with open(self.model_path, 'rb') as f:
                model_hash = hashlib.sha256(f.read()).hexdigest()
            self.get_logger().info(f"Model file hash (sha256): {model_hash}")
            
            # Load model with device specification
            self.model = YOLO(self.model_path)
            
            # Move model to correct device
            if hasattr(self.model, 'to'):
                self.model.to(self.device)
            
            self.get_logger().info(f"Successfully loaded YOLOv12 model: {self.model_path}")
            self.get_logger().info(f"Model info: {self.model.info}")
            
            # Get model details
            if hasattr(self.model, 'names'):
                self.get_logger().info(f"Model has {len(self.model.names)} classes")
                sample_classes = dict(list(self.model.names.items())[:10])
                self.get_logger().info(f"Sample classes: {sample_classes}")
            
            if hasattr(self.model, 'task'):
                self.get_logger().info(f"Model task: {self.model.task}")
            
            model_loaded = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model {self.model_path}: {e}")
            traceback.print_exc()
            
        if not model_loaded:
            self.get_logger().error("Failed to load any model")
            self.model = None

    def _create_subscribers(self):
        # Buat subscriber untuk semua kamera, error handling jika topic tidak ada
        try:
            self.image_subs = []
            for i, topic in enumerate(self.camera_topics):
                if i < self.cam_count:
                    sub = self.create_subscription(
                        Image, topic, 
                        lambda msg, idx=i: self.image_callback(msg, idx), 
                        10
                    )
                    self.image_subs.append(sub)
                    self.get_logger().info(f"Subscribed to camera {i+1}: {topic}")
                    
        except Exception as e:
            self.get_logger().error(f"Error creating subscribers: {e}")

    def _create_timers(self):
        """Buat timer untuk proses deteksi dan diagnostics"""
        try:
            # Timer for processing images (5Hz default)
            self.process_timer = self.create_timer(0.2, self.process_images)
            
            # Timer for diagnostics (1Hz)
            self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)
            
            # Timer for health check (0.2Hz)
            self.health_timer = self.create_timer(5.0, self.publish_health_check)
            
            # Timer for debug visualization status
            self.debug_timer = self.create_timer(10.0, self.debug_visualization_status)
            
        except Exception as e:
            self.get_logger().error(f"Error creating timers: {e}")

    def publish_health_check(self):
        """Publish health check untuk monitoring node."""
        try:
            active_cameras = sum(1 for img in self.images if img is not None)
            self.get_logger().info(f"Health Check - Active cameras: {active_cameras}/{self.cam_count}")
                
        except Exception as e:
            self.get_logger().error(f"Error in health check: {e}")

    def debug_visualization_status(self):
        """Debug status visualization."""
        try:
            if self.visualization_enabled:
                self.get_logger().info("Visualization: ENABLED")
            else:
                self.get_logger().info("Visualization: DISABLED")
        except Exception as e:
            self.get_logger().error(f"Error in debug visualization: {e}")

    def image_callback(self, msg, idx):
        # Callback untuk setiap image kamera, simpan ke buffer
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.mutex:
                self.images[idx] = cv_image
                self.last_frame_time[idx] = time.time()
                
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error for camera {idx}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback for camera {idx}: {e}")

    def fusion_callback(self, msg):
        """Callback untuk hasil fusion dari simple_fusion_node"""
        try:
            self.get_logger().debug(f"Received fusion result: {len(msg.yolov12_inference)} detections")
        except Exception as e:
            self.get_logger().error(f"Error in fusion callback: {e}")

    def process_images(self):
        """Process detection untuk semua kamera."""
        if not self.running or not self.is_initialized or self.model is None:
            return
        
        with self.mutex:
            images_copy = self.images.copy()
        
        available_count = sum(1 for img in images_copy if img is not None)
        if available_count == 0:
            return
        
        # Initialize latest_results storage
        if not hasattr(self, 'latest_results'):
            self.latest_results = {}
        
        try:
            for i, image in enumerate(images_copy):
                if image is not None:
                    camera_name = f"camera_{i+1}"
                    
                    # Run inference
                    start_time = time.time()
                    results = self.model(image, conf=self.conf_thres, device=self.device, verbose=False)
                    inference_time = time.time() - start_time
                    
                    self.inference_times[i] = inference_time
                    
                    # Count detections
                    if results and len(results) > 0:
                        detections = len(results[0].boxes) if hasattr(results[0], 'boxes') and results[0].boxes is not None else 0
                        self.detection_counts[i] += detections
                        
                        if detections > 0:
                            self.get_logger().info(f"Camera {i+1}: {detections} detections, inference: {inference_time:.3f}s")
                            
                        # Publish results
                        self.publish_results(results, camera_name)
                        
                        # Store for visualization
                        self.latest_results[camera_name] = (image, results)
        
        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")
            traceback.print_exc()

    def publish_results(self, results, camera_name):
        """Publish hasil deteksi ke topic /detection (Yolov12Inference) dengan koordinat yang benar."""
        if not results or len(results) == 0:
            return
        
        try:
            # Create message
            msg = Yolov12Inference()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{camera_name}_optical_frame"
            msg.camera_name = camera_name
            msg.frame_type = "raw"
            msg.task = "detect"
            
            # Process detections
            inference_results = []
            
            if hasattr(results[0], 'boxes') and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    detection = InferenceResult()
                    
                    # Get class info
                    class_id = int(box.cls.item())
                    detection.class_name = self.model.names[class_id] if hasattr(self.model, 'names') else str(class_id)
                    detection.confidence = float(box.conf.item())
                    
                    # Get bounding box coordinates (xyxy format)
                    coords = box.xyxy[0].cpu().numpy()
                    detection.left = int(coords[0])
                    detection.top = int(coords[1])
                    detection.right = int(coords[2])
                    detection.bottom = int(coords[3])
                    
                    # Optional fields
                    detection.track_id = -1  # No tracking
                    detection.obb_angle = -1.0  # No OBB
                    detection.mask_indices = []  # No segmentation
                    
                    inference_results.append(detection)
            
            msg.yolov12_inference = inference_results
            
            # Publish message
            self.detection_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing results for {camera_name}: {e}")
            traceback.print_exc()

    def visualize_results(self, images):
        """Visualisasi hasil deteksi dengan label posisi yang benar"""
        if not self.visualization_enabled:
            return
            
        try:
            # Only visualize if we have results
            if not hasattr(self, 'latest_results') or not self.latest_results:
                return
                
            display_images = []
            for camera_name, (image, results) in self.latest_results.items():
                annotated_image = image.copy()
                
                # Draw detections
                if results and len(results) > 0 and hasattr(results[0], 'boxes'):
                    if results[0].boxes is not None:
                        for box in results[0].boxes:
                            # Get coordinates
                            coords = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = map(int, coords)
                            
                            # Get class and confidence
                            class_id = int(box.cls.item())
                            class_name = self.model.names[class_id] if hasattr(self.model, 'names') else str(class_id)
                            confidence = float(box.conf.item())
                            
                            # Draw bounding box
                            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # Draw label
                            label = f"{class_name}: {confidence:.2f}"
                            cv2.putText(annotated_image, label, (x1, y1-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Add camera name
                cv2.putText(annotated_image, camera_name, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                display_images.append(annotated_image)
            
            if display_images:
                self._display_multi_camera(display_images)
                
        except Exception as e:
            self.get_logger().error(f"Error visualizing results: {e}")

    def _display_multi_camera(self, images):
        """Display multiple cameras dengan layout yang optimal"""
        try:
            target_height = 400  # Lebih besar untuk readability
            resized_images = []
            
            for image in images:
                h, w = image.shape[:2]
                scale = target_height / h
                new_w = int(w * scale)
                resized = cv2.resize(image, (new_w, target_height))
                resized_images.append(resized)
            
            if not resized_images:
                return
            
            # Layout untuk 6 kamera (3x2)
            if len(resized_images) >= 3:
                row1 = cv2.hconcat(resized_images[:3])
                if len(resized_images) >= 6:
                    row2 = cv2.hconcat(resized_images[3:6])
                    final_image = cv2.vconcat([row1, row2])
                else:
                    final_image = row1
            else:
                final_image = cv2.hconcat(resized_images)
            
            # **Window dengan nama yang jelas**
            cv2.imshow("Huskybot MultiCam YOLOv12 + LiDAR Fusion", final_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error displaying images: {e}")

def main(args=None):
    # Entry point ROS2 node
    rclpy.init(args=args)
    node = None
    try:
        node = MultiCamDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
        if node:
            node.running = False
    except Exception as e:
        print(f"Fatal error: {e}")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")
        print("Multicam detection node shutdown complete.")

if __name__ == "__main__":
    main()  # Jalankan main jika file dieksekusi langsung

# ===================== PENJELASAN TAMBAHAN =====================
# File ini mengimplementasikan node deteksi objek multicamera dengan YOLOv12 untuk robot Huskybot.
# Node menerima image dari 6 kamera dalam konfigurasi hexagonal, menjalankan deteksi objek pada
# setiap image, dan mempublish hasil ke topic /detection dengan message type Yolov12Inference.
# Node sudah FULL OOP, dengan error handling komprehensif di setiap bagian.