#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# File: multicam_detection_node.py - Node deteksi multicam YOLOv12 untuk Huskybot
# Node ini menerima image dari 6 kamera Arducam IMX477 (hexagonal) dan publish hasil deteksi ke topic /detection
# Siap untuk ROS2 Humble, Gazebo, Jetson Orin, Clearpath Husky A200, multi-robot, audit trail

import os  # Operasi file dan path
import sys  # Akses sys.stderr, sys.exit
import time  # Timestamp, delay, log
import traceback  # Print stack trace saat exception
import logging  # Logging ke file dan terminal
import platform  # Deteksi hardware/OS
from threading import Lock  # Thread safety di callback paralel
import hashlib  # Untuk hash file model (audit trail)
import psutil  # Untuk cek resource usage (health check, opsional)

# ===================== ERROR HANDLING: CEK DEPENDENCY PYTHON =====================
# Cek semua dependency utama sebelum import ROS2
REQUIRED_MODULES = [
    'rclpy', 'cv2', 'numpy', 'ultralytics', 'yolov12_msgs', 'cv_bridge'
]
for mod in REQUIRED_MODULES:
    try:
        __import__(mod)
    except ImportError as e:
        print(f"[FATAL] Python dependency not found: {mod} ({e})", file=sys.stderr)
        sys.exit(1)  # Exit jika ada dependency yang kurang

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from rclpy.exceptions import ParameterNotDeclaredException  # Exception parameter tidak ditemukan
from rclpy.parameter import Parameter  # Kelas Parameter ROS2
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Deskriptor parameter
from rcl_interfaces.msg import SetParametersResult  # Return value untuk parameter callback

from sensor_msgs.msg import Image  # Message ROS2 untuk image kamera
from std_msgs.msg import Header  # Header standar ROS2
from std_srvs.srv import Trigger  # Service type untuk restart_model dan health check
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # Diagnostics ROS2

from cv_bridge import CvBridge, CvBridgeError  # Konversi ROS Image <-> OpenCV
import numpy as np  # Array/matrix untuk image processing
import cv2  # OpenCV untuk image processing/visualisasi
from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil deteksi YOLOv12

# ===================== DETECTOR LIBRARY (YOLOv12) DENGAN FALLBACK =====================
try:
    from ultralytics import YOLO  # Library YOLOv12 (pastikan sudah install ultralytics>=v12)
    ULTRALYTICS_AVAILABLE = True  # Flag ketersediaan ultralytics
except ImportError:
    ULTRALYTICS_AVAILABLE = False  # Set flag False jika import error

LOG_DIR = os.path.expanduser('~/huskybot_detection_log')  # Directory log (default di home)
DEFAULT_CONFIDENCE_THRESHOLD = 0.25  # Threshold confidence default
JETSON_PLATFORMS = ['aarch64', 'arm64']  # Platform Jetson
DETECTION_TOPIC = '/detection'  # Topic hasil deteksi
DIAGNOSTIC_TOPIC = '/diagnostics'  # Topic diagnostics
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
        super().__init__('multicam_detection')  # Inisialisasi node ROS2
        self._setup_logging()  # Setup logging ke file dan terminal
        self.get_logger().info("Initializing MultiCam YOLOv12 Detection Node...")  # Log startup

        self._declare_parameters()  # Declare semua parameter
        self._load_parameters()  # Load parameter dari server

        self.bridge = CvBridge()  # Bridge konversi image
        self.mutex = Lock()  # Mutex untuk thread safety
        self.images = [None] * self.cam_count  # Buffer image
        self.last_frame_time = [None] * self.cam_count  # Timestamp terakhir
        self.detection_counts = [0] * self.cam_count  # Counter deteksi
        self.inference_times = [0.0] * self.cam_count  # Waktu inference
        self.is_initialized = False  # Flag init
        self.model = None  # Model YOLOv12
        self.running = True  # Flag running

        self._detect_platform()  # Deteksi Jetson/CUDA
        self._create_publishers()  # Publisher deteksi/diagnostics
        self._create_services()  # Service restart_model/get_status
        self._load_model()  # Load YOLOv12 model
        self._create_subscribers()  # Subscriber kamera
        self._create_timers()  # Timer deteksi/diagnostics

        # Tambahkan subscriber untuk hasil fusion
        self.fusion_sub = self.create_subscription(
            Yolov12Inference, '/detection_with_distance', self.fusion_callback, 10)

        # PERBAIKAN: Performance optimization for Jetson
        if self.is_jetson:
            try:
                import torch
                if torch.cuda.is_available():
                    # Set CUDA optimization
                    torch.backends.cudnn.benchmark = True
                    torch.backends.cudnn.deterministic = False
                    
                    # Set memory management
                    if hasattr(torch.cuda, 'empty_cache'):
                        torch.cuda.empty_cache()
                        
                    self.get_logger().info("CUDA optimizations enabled for Jetson")
            except Exception as e:
                self.get_logger().warning(f"Could not apply CUDA optimizations: {e}")
        
        self.is_initialized = True  # Set flag init selesai
        self.get_logger().info(f"MultiCam YOLOv12 Detection Node initialized with {self.cam_count} cameras")  # Log selesai init

    def _setup_logging(self):
        # Setup logging ke file dan terminal, fallback ke /tmp jika gagal
        try:
            log_dir = LOG_DIR
            try:
                if not os.path.exists(log_dir):
                    os.makedirs(log_dir, exist_ok=True)
                if not os.access(log_dir, os.W_OK):
                    raise PermissionError(f"Log dir {log_dir} not writeable")
            except Exception:
                log_dir = '/tmp'
                if not os.path.exists(log_dir):
                    os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f"huskybot_detection_{time.strftime('%Y%m%d')}.log")
            # Rotasi log jika > 50MB (audit trail, opsional)
            if os.path.exists(log_file) and os.path.getsize(log_file) > 50 * 1024 * 1024:
                os.rename(log_file, log_file + f".{int(time.time())}.bak")
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s [%(levelname)s] %(message)s',
                handlers=[
                    logging.FileHandler(log_file),
                    logging.StreamHandler(sys.stdout)
                ]
            )
            logging.info("Logging system initialized")
        except Exception as e:
            print(f"Error setting up logging: {e}", file=sys.stderr)

    def _declare_parameters(self):
        # Deklarasi semua parameter node (wajib agar bisa diubah via launch file)
        # PERBAIKAN: Use simple parameter declarations without complex validation
        self.declare_parameter('cam_count', 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of cameras in the hexagonal array (1-12)'
            # Remove integer_range untuk menghindari validation error
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
            # Remove floating_point_range untuk menghindari validation error
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
            # Manual validation untuk cam_count
            self.cam_count = self.get_parameter('cam_count').value
            if not isinstance(self.cam_count, int) or not 1 <= self.cam_count <= 12:
                self.get_logger().warning(f"Invalid cam_count {self.cam_count}, using default (6)")
                self.cam_count = 6
                
            self.model_path = self.get_parameter('model_path').value
            
            # PERBAIKAN: Better camera topics parsing
            try:
                camera_topics_str = self.get_parameter('camera_topics_str').value
                if camera_topics_str and camera_topics_str != 'None':
                    # Try multiple parsing methods
                    try:
                        import ast
                        self.camera_topics = ast.literal_eval(camera_topics_str)
                    except:
                        try:
                            import json
                            self.camera_topics = json.loads(camera_topics_str)
                        except:
                            # Fallback: split by comma
                            if ',' in camera_topics_str:
                                self.camera_topics = [t.strip().strip("'\"") for t in camera_topics_str.split(',')]
                            else:
                                self.camera_topics = DEFAULT_CAMERA_TOPICS
                else:
                    self.camera_topics = DEFAULT_CAMERA_TOPICS
                    
                if not isinstance(self.camera_topics, list) or len(self.camera_topics) == 0:
                    self.camera_topics = DEFAULT_CAMERA_TOPICS
                    
            except Exception as e:
                self.get_logger().warning(f"Error parsing camera_topics_str: {e}, using default")
                self.camera_topics = DEFAULT_CAMERA_TOPICS
            
            # PERBAIKAN: Better class filter parsing
            try:
                class_filter_str = self.get_parameter('class_filter_str').value
                if class_filter_str and class_filter_str not in ['[]', 'None', '']:
                    try:
                        import ast
                        self.class_filter = ast.literal_eval(class_filter_str)
                    except:
                        try:
                            import json
                            self.class_filter = json.loads(class_filter_str)
                        except:
                            self.class_filter = []
                else:
                    self.class_filter = []
                    
                if not isinstance(self.class_filter, list):
                    self.class_filter = []
                
            except Exception as e:
                self.get_logger().warning(f"Error parsing class_filter_str: {e}, using empty list")
                self.class_filter = []
            
            # Manual validation untuk conf_thres
            self.conf_thres = self.get_parameter('conf_thres').value
            if not isinstance(self.conf_thres, (int, float)) or not 0.0 <= self.conf_thres <= 1.0:
                self.get_logger().warning(f"Invalid conf_thres {self.conf_thres}, using default ({DEFAULT_CONFIDENCE_THRESHOLD})")
                self.conf_thres = DEFAULT_CONFIDENCE_THRESHOLD
            
            # Adjust camera count based on available topics
            if len(self.camera_topics) < self.cam_count:
                self.get_logger().warning(
                    f"Not enough camera topics ({len(self.camera_topics)}) for cam_count ({self.cam_count})"
                )
                self.cam_count = len(self.camera_topics)
                
            self.visualization_enabled = self.get_parameter('visualization_enabled').value
            self.log_to_file = self.get_parameter('log_to_file').value
            
            log_level = self.get_parameter('log_level').value.lower()
            if log_level == 'debug':
                logging.getLogger().setLevel(logging.DEBUG)
            elif log_level == 'info':
                logging.getLogger().setLevel(logging.INFO)
            elif log_level == 'warning':
                logging.getLogger().setLevel(logging.WARNING)
            elif log_level == 'error':
                logging.getLogger().setLevel(logging.ERROR)
            elif log_level == 'critical':
                logging.getLogger().setLevel(logging.CRITICAL)
            else:
                self.get_logger().warning(f"Invalid log_level '{log_level}', using INFO")
                logging.getLogger().setLevel(logging.INFO)
                
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {e}")
            self.get_logger().error(traceback.format_exc())
            # Use defaults instead of raising
            self.cam_count = 6
            self.camera_topics = DEFAULT_CAMERA_TOPICS
            self.class_filter = []
            self.conf_thres = DEFAULT_CONFIDENCE_THRESHOLD
            self.visualization_enabled = True
            self.log_to_file = True

    def _detect_platform(self):
        # Deteksi platform Jetson/CUDA untuk optimasi model
        try:
            self.is_jetson = platform.machine() in JETSON_PLATFORMS
            if self.is_jetson:
                self.get_logger().info("Detected Jetson platform, using hardware acceleration")
                try:
                    import torch
                    self.cuda_available = torch.cuda.is_available()
                    if self.cuda_available:
                        self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")
                    else:
                        self.get_logger().warning("CUDA not available on Jetson device")
                except ImportError:
                    self.get_logger().warning("PyTorch not installed, can't check CUDA availability")
                    self.cuda_available = False
            else:
                self.get_logger().info("Running on non-Jetson platform")
                self.cuda_available = False
        except Exception as e:
            self.get_logger().error(f"Error detecting platform: {e}")
            self.is_jetson = False
            self.cuda_available = False

    def _create_publishers(self):
        # Buat publisher untuk hasil deteksi dan diagnostics
        try:
            self.publisher = self.create_publisher(Yolov12Inference, DETECTION_TOPIC, 10)
            self.diagnostic_pub = self.create_publisher(DiagnosticArray, DIAGNOSTIC_TOPIC, 10)
            self.get_logger().info(f"Publishers created: {DETECTION_TOPIC}, {DIAGNOSTIC_TOPIC}")
        except Exception as e:
            self.get_logger().error(f"Error creating publishers: {e}")
            raise

    def _create_services(self):
        # Buat service untuk restart model dan get status
        try:
            self.restart_srv = self.create_service(Trigger, 'restart_model', self.restart_model_callback)
            self.status_srv = self.create_service(Trigger, 'get_status', self.get_status_callback)
            self.get_logger().info("Services created: restart_model, get_status")
        except Exception as e:
            self.get_logger().error(f"Error creating services: {e}")
            self.get_logger().error(traceback.format_exc())

    def _load_model(self):
        # Load YOLOv12 model, fallback ke .pt jika .engine/.onnx gagal
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error("Ultralytics not available. Cannot load YOLOv12 model.")
            raise ImportError("Required package 'ultralytics' not found")
        model_loaded = False
        original_path = self.model_path
        try:
            # ===================== ERROR HANDLING: VALIDASI FILE MODEL =====================
            self.get_logger().info(f"Loading YOLOv12 model from: {self.model_path}")
            if not os.path.exists(self.model_path):
                self.get_logger().warning(f"Model file not found: {self.model_path}")
                possible_paths = [
                    self.model_path,
                    os.path.join(os.path.expanduser('~'), self.model_path),
                    os.path.join('/opt/models', self.model_path),
                    os.path.join(os.getcwd(), self.model_path),
                    os.path.join(os.getcwd(), 'models', self.model_path),
                ]
                base_name = os.path.splitext(self.model_path)[0]
                possible_paths.extend([
                    f"{base_name}.pt",
                    f"{base_name}.onnx",
                    f"{base_name}.engine"
                ])
                for path in possible_paths:
                    if os.path.exists(path):
                        self.get_logger().info(f"Found model at: {path}")
                        self.model_path = path
                        break
            # Cek permission file model
            if not os.path.isfile(self.model_path) or not os.access(self.model_path, os.R_OK):
                self.get_logger().error(f"Model file not readable: {self.model_path}")
                raise PermissionError(f"Model file not readable: {self.model_path}")
            # Hash model file untuk audit trail
            try:
                with open(self.model_path, "rb") as f:
                    model_hash = hashlib.sha256(f.read()).hexdigest()
                self.get_logger().info(f"Model file hash (sha256): {model_hash}")
            except Exception as e:
                self.get_logger().warning(f"Could not hash model file: {e}")
            self.model = YOLO(self.model_path, task="detect")
            model_loaded = True
            self.get_logger().info(f"Successfully loaded YOLOv12 model: {self.model_path}")
            
            # PERBAIKAN: Debug model information
            if hasattr(self.model, 'info'):
                self.get_logger().info(f"Model info: {self.model.info}")
            
            # PERBAIKAN: Log class names untuk debugging
            if hasattr(self.model, 'names'):
                self.get_logger().info(f"Model has {len(self.model.names)} classes")
                self.get_logger().info(f"Sample classes: {dict(list(self.model.names.items())[:10])}")
            else:
                self.get_logger().warning("Model does not have class names, using COCO fallback")
            
            self.get_logger().info(f"Model task: detect")
        except Exception as e:
            self.get_logger().error(f"Error loading YOLOv12 model: {e}")
            self.get_logger().error(traceback.format_exc())
            if not model_loaded and original_path.endswith(('.engine', '.onnx')):
                try:
                    fallback_path = original_path.rsplit('.', 1)[0] + '.pt'
                    self.get_logger().info(f"Attempting fallback to PyTorch model: {fallback_path}")
                    if os.path.exists(fallback_path):
                        self.model = YOLO(fallback_path, task="detect")
                        self.model_path = fallback_path
                        model_loaded = True
                        self.get_logger().info(f"Successfully loaded fallback model: {fallback_path}")
                    else:
                        self.get_logger().error(f"Fallback model not found: {fallback_path}")
                except Exception as e2:
                    self.get_logger().error(f"Error loading fallback model: {e2}")
            if not model_loaded:
                self.get_logger().error("Could not load any YOLOv12 model. Exiting.")
                raise

    def _create_subscribers(self):
        # Buat subscriber untuk semua kamera, error handling jika topic tidak ada
        try:
            if not self.camera_topics or len(self.camera_topics) == 0:
                self.get_logger().error("No camera topics specified")
                raise ValueError("No camera topics to subscribe to")
            active_topics = self.camera_topics[:self.cam_count]
            self.get_logger().info(f"Creating subscribers for {len(active_topics)} camera topics")
            self.subs = []
            for i, topic in enumerate(active_topics):
                try:
                    # ===================== ERROR HANDLING: CEK TOPIC KAMERA (OPSIONAL) =====================
                    # (Bisa tambahkan pengecekan ros2 topic list di sini jika ingin lebih advance)
                    sub = self.create_subscription(
                        Image,
                        topic,
                        lambda msg, idx=i: self.image_callback(msg, idx),
                        10
                    )
                    self.subs.append(sub)
                    self.get_logger().info(f"Subscribed to camera topic: {topic} (idx={i})")
                except Exception as e:
                    self.get_logger().error(f"Error subscribing to {topic}: {e}")
            if len(self.subs) == 0:
                self.get_logger().error("Failed to create any camera subscribers")
                raise RuntimeError("No camera subscriptions created")
            if len(self.subs) < self.cam_count:
                self.get_logger().warning(f"Only {len(self.subs)} camera topics subscribed out of {self.cam_count} requested")
                self.cam_count = len(self.subs)
                self.images = [None] * self.cam_count
                self.last_frame_time = [None] * self.cam_count
                self.detection_counts = [0] * self.cam_count
                self.inference_times = [0.0] * self.cam_count
        except Exception as e:
            self.get_logger().error(f"Error setting up camera subscribers: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _create_timers(self):
        # Buat timer untuk proses deteksi dan diagnostics
        try:
            # PERBAIKAN: Slower processing untuk debugging
            self.timer = self.create_timer(0.5, self.process_images)  # Slower untuk debugging
            self.diag_timer = self.create_timer(2.0, self.publish_diagnostics)  # Slower diagnostics
            self.health_timer = self.create_timer(5.0, self.publish_health_check)
            
            # PERBAIKAN: Add visualization debug timer
            if self.visualization_enabled:
                self.viz_timer = self.create_timer(1.0, self.debug_visualization_status)
                
            self.get_logger().info("Timers created for processing, diagnostics, and health check")
        except Exception as e:
            self.get_logger().error(f"Error creating timers: {e}")
            raise

    def debug_visualization_status(self):
        """Debug status visualization."""
        try:
            total_detections = sum(self.detection_counts)
            avg_inference = sum(self.inference_times) / len(self.inference_times) if self.inference_times else 0
            
            self.get_logger().info(
                f"Detection Status: Total={total_detections}, Avg_Inference={avg_inference:.3f}s, "
                f"Cameras_Active={sum(1 for img in self.images if img is not None)}"
            )
        except Exception as e:
            self.get_logger().error(f"Error in debug visualization: {e}")

    def image_callback(self, msg, idx):
        # Callback untuk setiap image kamera, simpan ke buffer
        try:
            with self.mutex:
                self.last_frame_time[idx] = self.get_clock().now()
                # ===================== ERROR HANDLING: VALIDASI IMAGE =====================
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                if img is None or not isinstance(img, np.ndarray):
                    self.get_logger().warning(f"Received invalid image from camera {idx}")
                    return
                if img.ndim != 3 or img.shape[2] != 3:
                    self.get_logger().warning(f"Image shape not valid (expected 3 channels): {img.shape}")
                    return
                self.images[idx] = img
        except CvBridgeError as e:
            self.get_logger().warning(f"Error converting image from topic {self.camera_topics[idx]}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback for camera {idx}: {e}")
            self.get_logger().error(traceback.format_exc())

    def process_images(self):
        """Proses deteksi untuk semua kamera, publish hasil ke /detection."""
        if not self.running or not self.is_initialized:
            return
        
        with self.mutex:
            images_copy = self.images.copy()
        
        available_count = sum(1 for img in images_copy if img is not None)
        if available_count == 0:
            return
        
        # Initialize latest_results storage
        if not hasattr(self, 'latest_results'):
            self.latest_results = [None] * self.cam_count
        
        try:
            for idx, img in enumerate(images_copy):
                if img is not None:
                    start_time = time.time()
                    try:
                        # Validasi image
                        if img.ndim != 3 or img.shape[2] != 3:
                            self.get_logger().warning(f"Image shape not valid for inference: {img.shape}")
                            continue
                        
                        # Run inference
                        results = self.model(img, verbose=False, conf=self.conf_thres)
                        infer_time = time.time() - start_time
                        self.inference_times[idx] = infer_time
                        
                        # Store results untuk visualization
                        self.latest_results[idx] = results
                        
                        # PERBAIKAN: Better detection counting
                        detection_count = 0
                        if results and len(results) > 0 and hasattr(results[0], 'boxes') and results[0].boxes is not None:
                            if self.class_filter:
                                # Filter by class if specified
                                for box in results[0].boxes:
                                    try:
                                        cls = int(box.cls[0].cpu())
                                        if cls in self.class_filter:
                                            detection_count += 1
                                    except Exception as e:
                                        self.get_logger().debug(f"Error filtering detection: {e}")
                            else:
                                # Count all detections
                                detection_count = len(results[0].boxes)
                        
                        # Update detection count
                        self.detection_counts[idx] = detection_count
                        
                        # Warning untuk inference time
                        if infer_time > 1.0:
                            self.get_logger().warning(f"Inference time too long for camera {idx}: {infer_time:.3f}s")
                        
                        # Debug log hanya jika ada deteksi
                        if detection_count > 0:
                            self.get_logger().info(
                                f"Camera {idx}: {detection_count} detections in {infer_time:.3f}s"
                            )
                        
                        # Publish results
                        self.publish_results(results, f"camera_{idx}")
                        
                        # PERBAIKAN: Clear processed image to save memory
                        with self.mutex:
                            self.images[idx] = None
                        
                    except Exception as e:
                        self.get_logger().error(f"Error processing image from camera {idx}: {e}")
                        self.get_logger().error(traceback.format_exc())
            
            # Call visualization after processing all cameras
            if self.visualization_enabled:
                self.visualize_results(images_copy)
                
        except Exception as e:
            self.get_logger().error(f"Error in process_images: {e}")
            self.get_logger().error(traceback.format_exc())

    def publish_results(self, results, camera_name):
        """Publish hasil deteksi ke topic /detection (Yolov12Inference) dengan koordinat yang benar."""
        if not results or len(results) == 0:
            return
        
        try:
            msg = Yolov12Inference()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = camera_name
            msg.camera_name = camera_name
            msg.frame_type = "raw"
            msg.task = "detect"
            msg.note = f"Inference completed at {time.time()}"
            msg.yolov12_inference = []
            
            # PERBAIKAN: Better handling of YOLOv12 results with correct coordinates
            try:
                for result in results:
                    if hasattr(result, 'boxes') and result.boxes is not None:
                        boxes = result.boxes
                        for box in boxes:
                            try:
                                # PERBAIKAN: Extract coordinates in correct format (x1, y1, x2, y2)
                                if hasattr(box, 'xyxy'):
                                    coords = box.xyxy[0].cpu().numpy()
                                    x1, y1, x2, y2 = coords
                                else:
                                    # Fallback untuk format lain
                                    continue
                                
                                # Extract confidence and class
                                confidence = float(box.conf.cpu().numpy()[0]) if hasattr(box, 'conf') else 0.0
                                class_id = int(box.cls.cpu().numpy()[0]) if hasattr(box, 'cls') else 0
                                
                                # Pastikan koordinat dalam range yang benar
                                x1 = max(0, min(x1, 1920))  # Sesuai resolusi kamera
                                y1 = max(0, min(y1, 1080))
                                x2 = max(0, min(x2, 1920))
                                y2 = max(0, min(y2, 1080))
                                
                                # Skip deteksi dengan confidence rendah
                                if confidence < self.conf_thres:
                                    continue
                                
                                # Buat InferenceResult
                                inference_result = InferenceResult()
                                inference_result.class_name = self.model.names[class_id] if hasattr(self.model, 'names') else f"class_{class_id}"
                                inference_result.confidence = confidence
                                
                                # PERBAIKAN: Set koordinat dengan benar (left=x1, top=y1, right=x2, bottom=y2)
                                inference_result.left = int(x1)
                                inference_result.top = int(y1) 
                                inference_result.right = int(x2)
                                inference_result.bottom = int(y2)
                                
                                # Set default values for other fields
                                inference_result.track_id = -1
                                inference_result.obb_angle = -1
                                inference_result.mask_indices = []
                                
                                # Add note if field exists
                                if hasattr(inference_result, 'note'):
                                    inference_result.note = f"Detected by {camera_name}"
                                
                                msg.yolov12_inference.append(inference_result)
                                
                                # Debug log untuk koordinat
                                self.get_logger().debug(
                                    f"{camera_name}: {inference_result.class_name} "
                                    f"conf={confidence:.2f} "
                                    f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})"
                                )
                                
                            except Exception as e:
                                self.get_logger().warning(f"Error processing detection box: {e}")
                                continue
                                
            except Exception as e:
                self.get_logger().error(f"Error processing YOLOv12 results: {e}")
                return
        
            # Publish message
            if hasattr(self, 'publisher') and self.publisher is not None:
                self.publisher.publish(msg)
                self.get_logger().debug(f"Published {len(msg.yolov12_inference)} detections from {camera_name}")
            else:
                self.get_logger().error("Publisher not available")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing detection results: {e}")
            self.get_logger().error(traceback.format_exc())

    def visualize_results(self, images):
        """Visualisasi hasil deteksi semua kamera (side-by-side) dengan bounding box."""
        if not self.visualization_enabled:
            return
        try:
            valid_images = []
            
            for idx, img in enumerate(images):
                if img is not None:
                    # PERBAIKAN: Pastikan annotated_img selalu terdefinisi
                    annotated_img = img.copy()
                    
                    # Better bounding box visualization with class names
                    if hasattr(self, 'latest_results') and idx < len(self.latest_results):
                        results = self.latest_results[idx]
                        if results and len(results) > 0 and hasattr(results[0], 'boxes') and results[0].boxes is not None:
                            boxes = results[0].boxes
                            for box in boxes:
                                try:
                                    # Extract coordinates
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    conf = float(box.conf[0].cpu())
                                    cls = int(box.cls[0].cpu())
                                    
                                    # Get class name dengan fallback ke COCO
                                    class_name = "unknown"
                                    if hasattr(self.model, 'names') and isinstance(self.model.names, dict) and cls in self.model.names:
                                        class_name = str(self.model.names[cls])
                                    else:
                                        # COCO fallback
                                        coco_names = {
                                            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
                                            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
                                            39: 'bottle', 56: 'chair', 57: 'couch', 62: 'tv', 63: 'laptop',
                                            67: 'cell phone', 73: 'book', 74: 'clock'
                                        }
                                        class_name = coco_names.get(cls, f"class_{cls}")
                                    
                                    # Better visualization colors based on class
                                    color_map = {
                                        'person': (0, 255, 0),      # Green
                                        'car': (255, 0, 0),        # Blue  
                                        'bicycle': (0, 255, 255),  # Yellow
                                        'motorcycle': (255, 0, 255),  # Magenta
                                        'truck': (0, 0, 255),      # Red
                                        'bus': (255, 255, 0),      # Cyan
                                    }
                                    box_color = color_map.get(class_name, (0, 255, 0))  # Default green
                                    
                                    # Draw bounding box dengan thickness berdasarkan confidence
                                    thickness = max(1, int(conf * 4))
                                    cv2.rectangle(annotated_img, (int(x1), int(y1)), (int(x2), int(y2)), box_color, thickness)
                                    
                                    # Better label with confidence percentage
                                    label = f"{class_name}: {conf:.1%}"  # Format sebagai percentage
                                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                                    
                                    # Background rectangle untuk text
                                    cv2.rectangle(annotated_img, 
                                                (int(x1), int(y1-25)), 
                                                (int(x1 + label_size[0] + 10), int(y1)), 
                                                box_color, -1)
                                    
                                    # Text label
                                    cv2.putText(annotated_img, label, (int(x1+5), int(y1-8)), 
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                                              
                                except Exception as e:
                                    self.get_logger().debug(f"Error drawing box: {e}")
                    
                    # PERBAIKAN: Add camera label SETELAH annotated_img didefinisikan
                    try:
                        cv2.rectangle(annotated_img, (5, 5), (200, 35), (0, 0, 0), -1)  # Background
                        cv2.putText(annotated_img, f"Camera {idx}", (10, 25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        
                        # Add detection count dengan styling
                        detection_count = self.detection_counts[idx] if idx < len(self.detection_counts) else 0
                        cv2.rectangle(annotated_img, (5, 40), (150, 70), (50, 50, 50), -1)  # Background
                        cv2.putText(annotated_img, f"Detections: {detection_count}", (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        
                        valid_images.append(annotated_img)
                    except Exception as e:
                        self.get_logger().debug(f"Error adding camera overlay for camera {idx}: {e}")
                        # Fallback: tambahkan gambar asli tanpa overlay
                        valid_images.append(img)
        
            if not valid_images:
                return
            
            # PERBAIKAN: Better error handling untuk display
            try:
                # Resize untuk display dengan aspect ratio yang tetap
                target_height = 300
                resized_images = []
                for image in valid_images:
                    if image is not None and image.size > 0:
                        h, w = image.shape[:2]
                        if h > 0 and w > 0:
                            scale = target_height / h
                            new_width = int(w * scale)
                            resized = cv2.resize(image, (new_width, target_height))
                            resized_images.append(resized)
                
                if not resized_images:
                    return
                
                # Display dengan layout yang lebih baik
                if len(resized_images) > 3:
                    # Split into two rows untuk 6 cameras
                    top_row = np.hstack(resized_images[:3])
                    bottom_row_imgs = resized_images[3:]
                    
                    if bottom_row_imgs:
                        bottom_row = np.hstack(bottom_row_imgs)
                        
                        # Pad bottom row jika perlu
                        if bottom_row.shape[1] < top_row.shape[1]:
                            pad_width = top_row.shape[1] - bottom_row.shape[1]
                            bottom_row = np.pad(bottom_row, ((0, 0), (0, pad_width), (0, 0)), 'constant')
                        
                        vis = np.vstack([top_row, bottom_row])
                    else:
                        vis = top_row
                else:
                    vis = np.hstack(resized_images)
                
                cv2.imshow("MultiCam YOLOv12 Detection Results", vis)
                cv2.waitKey(1)
                
            except cv2.error as e:
                self.get_logger().warning(f"OpenCV visualization error: {e}")
            except Exception as e:
                self.get_logger().warning(f"Error creating visualization layout: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
            self.get_logger().error(traceback.format_exc())

    def publish_diagnostics(self):
        # Publish diagnostics ke /diagnostics untuk monitoring health node
        try:
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "huskybot_detection"
            status.hardware_id = platform.node()
            if not self.is_initialized or self.model is None:
                status.level = DiagnosticStatus.ERROR
                status.message = "Node not initialized or model not loaded"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Node running"
            status.values.append(KeyValue(key="model_path", value=self.model_path))
            status.values.append(KeyValue(key="camera_count", value=str(self.cam_count)))
            status.values.append(KeyValue(key="platform", value="Jetson" if self.is_jetson else "Generic"))
            for i, count in enumerate(self.detection_counts):
                status.values.append(KeyValue(key=f"detection_count_camera_{i}", value=str(count)))
            diag_msg.status.append(status)
            
            # Publish diagnostics message
            if hasattr(self, 'diagnostic_pub') and self.diagnostic_pub is not None:
                self.diagnostic_pub.publish(diag_msg)
                self.get_logger().debug(f"Published diagnostics: {status.message}")
            else:
                self.get_logger().error("Diagnostic publisher not initialized")
                
        except Exception as e:
            self.get_logger().warning(f"Error in health check: {e}")

    def restart_model_callback(self, request, response):
        # Service callback untuk restart model YOLOv12
        try:
            self.get_logger().info("Restarting YOLOv12 model")
            self._load_model()
            response.success = True
            response.message = f"Model restarted successfully: {self.model_path}"
            return response
        except Exception as e:
            self.get_logger().error(f"Error restarting model: {e}")
            response.success = False
            response.message = f"Failed to restart model: {str(e)}"
            return response

    def get_status_callback(self, request, response):
        # Service callback untuk get status node (health check)
        try:
            status_msg = f"MultiCamDetectionNode Status:\n"
            status_msg += f"- Camera count: {self.cam_count}\n"
            status_msg += f"- Model path: {self.model_path}\n"
            status_msg += f"- Platform: {platform.platform()}\n"
            status_msg += f"- Is Jetson: {self.is_jetson}\n"
            status_msg += "- Detection counts:\n"
            for i, count in enumerate(self.detection_counts):
                status_msg += f"  Camera {i}: {count}\n"
            status_msg += "- Inference times:\n"
            for i, infer_time in enumerate(self.inference_times):
                status_msg += f"  Camera {i}: {infer_time:.3f}s\n"
            response.success = True
            response.message = status_msg
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting status: {e}")
            response.success = False
            response.message = f"Failed to get status: {str(e)}"
            return response

    def publish_health_check(self):
        """Publish health check information."""
        try:
            # Get system resource usage
            try:
                cpu_percent = psutil.cpu_percent()
                memory_percent = psutil.virtual_memory().percent
            except:
                cpu_percent = 0.0
                memory_percent = 0.0
            
            # Log health check
            logging.info(f"HealthCheck: CPU={cpu_percent:.1f}%, RAM={memory_percent:.1f}%")
            
        except Exception as e:
            self.get_logger().warning(f"Error in health check: {e}")

    def on_shutdown(self):
        # Shutdown node dengan aman, tutup window OpenCV dan logging
        self.running = False
        self.get_logger().info("Shutting down MultiCamDetectionNode...")
        if self.visualization_enabled:
            try:
                cv2.destroyAllWindows()
            except Exception as e:
                self.get_logger().warning(f"Error closing OpenCV windows: {e}")
        try:
            logging.shutdown()
        except Exception as e:
            print(f"Error shutting down logging: {e}")

    # Tambahkan callback untuk subscriber hasil fusion
    def fusion_callback(self, msg):
        # Update visualization dengan distance dan coordinate
        for detection in msg.yolov12_inference:
            # Parse note field untuk distance dan coordinate
            if "Distance:" in detection.note:
                # Update visualization dengan info lengkap
                pass

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
            node.get_logger().info('KeyboardInterrupt, shutting down node.')
    except Exception as e:
        print(f"Fatal error: {e}")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
            node.get_logger().error(traceback.format_exc())
    finally:
        if node:
            node.on_shutdown()
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")
        print("Multicam detection node shutdown complete.")

if __name__ == "__main__":
    main()  # Jalankan main jika file dieksekusi langsung

# ===================== PENJELASAN TAMBAHAN =====================
# File ini mengimplementasikan node deteksi objek multicamera dengan YOLOv12 untuk robot Huskybot.
# Node menerima image dari 6 kamera dalam konfigurasi hexagonal, menjalankan deteksi objek pada
# setiap image, dan mempublish hasil ke topic /detection dengan message type Yolov12Inference.
# Node sudah FULL OOP, dengan error handling komprehensif di setiap bagian.