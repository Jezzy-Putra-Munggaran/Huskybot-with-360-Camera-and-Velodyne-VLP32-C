#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: multicam_detection_node.py - Node untuk deteksi objek multicam YOLOv12 pada robot Huskybot
# Integrasi: Node ini menerima image dari 6 kamera Arducam IMX477 (hexagonal) dan publish hasil deteksi
# ke topic /detection untuk digunakan oleh node fusion, tracking, dan visualizer

import os  # Library untuk operasi file dan environment
import sys  # Library untuk interaksi dengan interpreter Python dan exit codes
import time  # Library untuk fungsi waktu dan timing
import traceback  # Library untuk print stack trace detail saat exception
import logging  # Library untuk advanced logging (file + console)
import platform  # Library untuk deteksi sistem operasi dan hardware
from threading import Lock  # Library untuk thread safety di callback paralel

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from rclpy.exceptions import ParameterNotDeclaredException  # Exception untuk parameter tidak ditemukan
from rclpy.parameter import Parameter  # Kelas Parameter ROS2 untuk validasi dan type safety
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Deskriptor parameter untuk dokumentasi dan type hint
from rcl_interfaces.msg import SetParametersResult  # Return value untuk parameter callback

from sensor_msgs.msg import Image  # Message ROS2 untuk image kamera
from std_msgs.msg import Header  # Header ROS2 standar untuk timestamp dan frame_id
from std_srvs.srv import Trigger  # Service type untuk restart_model dan health check
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # Message untuk diagnostics ROS2

from cv_bridge import CvBridge, CvBridgeError  # Konversi ROS Image <-> OpenCV
import numpy as np  # Library array/matrix untuk image processing
import cv2  # OpenCV untuk image processing dan visualisasi
from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil deteksi YOLOv12

# Detector libraries dengan fallback mechanism
try:
    from ultralytics import YOLO  # Library YOLOv12 (pastikan sudah install ultralytics>=v12)
    ULTRALYTICS_AVAILABLE = True  # Flag ketersediaan ultralytics
except ImportError:
    ULTRALYTICS_AVAILABLE = False  # Set flag False jika import error

# Constants untuk konfigurasi node
LOG_DIR = os.path.expanduser('~/huskybot_detection_log')  # Directory untuk log files
MAX_LOG_SIZE = 10 * 1024 * 1024  # 10MB max log size sebelum rotasi
MAX_LOG_BACKUPS = 5  # Jumlah file backup log
DEFAULT_CONFIDENCE_THRESHOLD = 0.25  # Threshold confidence default untuk filtering hasil
JETSON_PLATFORMS = ['aarch64', 'arm64']  # Platform identifiers untuk Jetson devices
DISPLAY_TIMEOUT_MS = 1  # Timeout display OpenCV (1ms, non-blocking)
DETECTION_TOPIC = '/detection'  # Topic untuk publish hasil deteksi YOLOv12
DIAGNOSTIC_TOPIC = '/diagnostics'  # Topic untuk publish diagnostic status
CAMERA_TIMEOUT_SEC = 5.0  # Timeout sebelum warning kamera disconnected
INFERENCE_TIMEOUT_SEC = 10.0  # Timeout warning untuk inference yang lambat

class MultiCamDetectionNode(Node):  # Node deteksi multicam YOLOv12, FULL OOP
    """
    Node untuk deteksi objek pada multiple camera feeds menggunakan YOLOv12.
    
    Node ini menerima images dari 6 kamera (default) dalam konfigurasi hexagonal,
    melakukan deteksi objek dengan YOLOv12, dan publish hasil ke topic /detection.
    Node juga menyediakan visualisasi real-time dari semua kamera dengan hasil deteksi.
    
    Parameters:
        - cam_count: Jumlah kamera (default 6)
        - model_path: Path ke file model YOLOv12 (.pt, .onnx, atau .engine)
        - camera_topics: List topic kamera ROS2 untuk disubscribe
        - conf_thres: Threshold confidence untuk filtering hasil (0.0-1.0)
        - class_filter: List class IDs untuk difilter (kosong = semua)
        - visualization_enabled: Boolean untuk enable/disable visualisasi OpenCV
        - log_to_file: Boolean untuk enable/disable logging ke file
        - log_level: Level detail logging (debug, info, warning, error)
        
    Publishers:
        - /detection: Hasil deteksi YOLOv12 untuk semua kamera
        - /diagnostics: Status diagnostik node dan model
        
    Services:
        - ~/restart_model: Restart dan reload model YOLOv12
        - ~/get_status: Dapatkan status node dan statistik deteksi
    """
    
    def __init__(self):
        """
        Initialize MultiCamDetectionNode dengan parameter, subscribers, publishers, dan services.
        Setup model YOLOv12 dan buffer untuk image processing multicamera.
        """
        super().__init__('multicam_detection')  # Inisialisasi node ROS2 dengan nama unik
        
        # Setup logging system
        self._setup_logging()  # Setup file dan console logging dengan rotasi
        self.get_logger().info("Initializing MultiCam YOLOv12 Detection Node...")  # Log startup message
        
        # Declare all parameters with descriptors for auto-documentation
        self._declare_parameters()  # Declare semua parameter dengan deskriptor dan validasi
        
        # Get parameters from ROS2 parameter server
        self._load_parameters()  # Load parameter dari parameter server dengan validasi
        
        # Initialize variables
        self.bridge = CvBridge()  # Inisialisasi bridge konversi image ROS <-> OpenCV
        self.mutex = Lock()  # Mutex untuk thread safety di callback dan timer
        self.images = [None] * self.cam_count  # Buffer image untuk setiap kamera
        self.last_frame_time = [None] * self.cam_count  # Timestamp terakhir untuk setiap kamera
        self.detection_counts = [0] * self.cam_count  # Counter deteksi untuk stats
        self.inference_times = [0.0] * self.cam_count  # Tracking inference time untuk diagnostics
        self.is_initialized = False  # Flag initialization status untuk safety
        self.model = None  # Instance model YOLOv12, diinisialisasi di _load_model()
        self.running = True  # Flag untuk kontrol execution loop, for clean shutdown
        
        # Detect running platform (optimize for Jetson)
        self._detect_platform()  # Detect Jetson, CUDA, TensorRT availability
        
        # Create publishers
        self._create_publishers()  # Setup publishers untuk detection dan diagnostics
        
        # Create services
        self._create_services()  # Setup services untuk restart_model dan get_status
        
        # Load YOLOv12 model
        self._load_model()  # Load dan initialize YOLOv12 model sesuai tersedia
        
        # Create subscribers after everything is set up
        self._create_subscribers()  # Setup subscribers untuk semua kamera
        
        # Create timers for periodic tasks
        self._create_timers()  # Setup timer untuk detection, visualization, dan diagnostics
        
        # Set initialization complete
        self.is_initialized = True  # Set flag initialization berhasil
        self.get_logger().info(f"MultiCam YOLOv12 Detection Node initialized with {self.cam_count} cameras")  # Log success message
    
    def _setup_logging(self):
        """
        Setup file dan console logging dengan rotasi dan format timestamp.
        """
        try:
            # Create log directory if it doesn't exist
            if not os.path.exists(LOG_DIR):  # Cek directory log ada
                os.makedirs(LOG_DIR, exist_ok=True)  # Buat directory jika tidak ada dengan exist_ok untuk race condition
            
            # Setup Python logging to file with rotation
            log_file = os.path.join(LOG_DIR, f"huskybot_detection_{time.strftime('%Y%m%d')}.log")  # Path log dengan tanggal
            logging.basicConfig(
                level=logging.INFO,  # Default level logging adalah INFO
                format='%(asctime)s [%(levelname)s] %(message)s',  # Format dengan timestamp dan level
                handlers=[
                    logging.FileHandler(log_file),  # Log ke file
                    logging.StreamHandler(sys.stdout)  # Juga log ke stdout untuk debugging
                ]
            )
            logging.info("Logging system initialized")  # Log bahwa system logging sudah setup
        except Exception as e:
            # If logging setup fails, at least try to print to stderr
            print(f"Error setting up logging: {e}", file=sys.stderr)  # Print error ke stderr jika setup gagal
    
    def _declare_parameters(self):
        """
        Declare semua parameter dengan type checking, constraints, dan descriptions.
        """
        # Parameter untuk jumlah kamera
        self.declare_parameter(
            'cam_count', 
            6,  # Default value
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of cameras in the hexagonal array (1-6)',
                integer_range=[{'from_value': 1, 'to_value': 12, 'step': 1}]  # Range validasi
            )
        )
        
        # Parameter untuk path model YOLOv12
        self.declare_parameter(
            'model_path', 
            "yolo12x.engine",  # Default TensorRT engine untuk Jetson
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to YOLOv12 model file (.pt, .onnx, or .engine)'
            )
        )
        
        # Parameter untuk daftar topic kamera
        self.declare_parameter(
            'camera_topics', 
            [  # Default topic sesuai konfigurasi hexagonal
                '/camera_front/image_raw',
                '/camera_right/image_raw',
                '/camera_rear_right/image_raw',
                '/camera_rear/image_raw',
                '/camera_left/image_raw',
                '/camera_front_left/image_raw'
            ],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='List of camera topic names to subscribe to'
            )
        )
        
        # Parameter untuk threshold confidence
        self.declare_parameter(
            'conf_thres', 
            DEFAULT_CONFIDENCE_THRESHOLD,  # Default threshold confidence
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Confidence threshold for filtering detections (0.0-1.0)',
                floating_point_range=[{'from_value': 0.0, 'to_value': 1.0, 'step': 0.01}]  # Range validasi
            )
        )
        
        # Parameter untuk filter class
        self.declare_parameter(
            'class_filter', 
            [],  # Default: no filtering, detect all classes
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='List of class IDs to keep (empty for all classes)'
            )
        )
        
        # Parameter untuk enable/disable visualisasi
        self.declare_parameter(
            'visualization_enabled', 
            True,  # Default visualisasi aktif
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable/disable OpenCV visualization of detection results'
            )
        )
        
        # Parameter untuk enable/disable file logging
        self.declare_parameter(
            'log_to_file', 
            True,  # Default log ke file aktif
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable/disable logging to file'
            )
        )
        
        # Parameter untuk log level
        self.declare_parameter(
            'log_level', 
            'info',  # Default log level
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Log level (debug, info, warning, error, critical)'
            )
        )
    
    def _load_parameters(self):
        """
        Load dan validasi semua parameter dari ROS2 parameter server.
        """
        try:
            # Get camera count parameter
            self.cam_count = self.get_parameter('cam_count').value  # Jumlah kamera
            if not 1 <= self.cam_count <= 12:  # Validasi range jumlah kamera
                self.get_logger().warning(f"Invalid cam_count {self.cam_count}, using default (6)")  # Warning jika di luar range
                self.cam_count = 6  # Reset ke default
            
            # Get model path parameter
            self.model_path = self.get_parameter('model_path').value  # Path model YOLOv12
            
            # Get camera topics parameter and validate length
            self.camera_topics = self.get_parameter('camera_topics').value  # List topic kamera
            if len(self.camera_topics) < self.cam_count:  # Validasi jumlah topic sesuai jumlah kamera
                self.get_logger().warning(
                    f"Not enough camera topics ({len(self.camera_topics)}) for cam_count ({self.cam_count})"
                )
                self.cam_count = len(self.camera_topics)  # Adjust cam_count to available topics
            
            # Get confidence threshold
            self.conf_thres = self.get_parameter('conf_thres').value  # Threshold confidence
            
            # Get class filter
            self.class_filter = self.get_parameter('class_filter').value  # Filter class ID
            
            # Get visualization parameter
            self.visualization_enabled = self.get_parameter('visualization_enabled').value  # Enable/disable visualisasi
            
            # Get log parameters
            self.log_to_file = self.get_parameter('log_to_file').value  # Enable/disable log ke file
            log_level = self.get_parameter('log_level').value.lower()  # Level log
            
            # Configure log level
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
                self.get_logger().warning(f"Invalid log_level '{log_level}', using INFO")  # Warning jika invalid log level
                logging.getLogger().setLevel(logging.INFO)  # Default ke INFO
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f"Parameter error: {e}")  # Error jika parameter tidak ada
            raise  # Re-raise untuk shutdown node
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {e}")  # Error jika gagal load parameter
            self.get_logger().error(traceback.format_exc())  # Error stack trace
            raise  # Re-raise untuk shutdown node
    
    def _detect_platform(self):
        """
        Deteksi platform dan kemampuan hardware untuk optimasi model.
        """
        try:
            self.is_jetson = platform.machine() in JETSON_PLATFORMS  # Cek jika running di Jetson
            
            if self.is_jetson:
                self.get_logger().info("Detected Jetson platform, using hardware acceleration")  # Log jika Jetson terdeteksi
                # Check for CUDA availability
                try:
                    import torch  # Import torch untuk cek CUDA
                    self.cuda_available = torch.cuda.is_available()  # Cek CUDA available
                    if self.cuda_available:
                        self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")  # Log CUDA device
                    else:
                        self.get_logger().warning("CUDA not available on Jetson device")  # Warning jika CUDA tidak ada
                except ImportError:
                    self.get_logger().warning("PyTorch not installed, can't check CUDA availability")  # Warning jika torch tidak terinstall
                    self.cuda_available = False
            else:
                self.get_logger().info("Running on non-Jetson platform")  # Log jika bukan Jetson
                self.cuda_available = False
        except Exception as e:
            self.get_logger().error(f"Error detecting platform: {e}")  # Error jika gagal deteksi platform
            self.is_jetson = False
            self.cuda_available = False
    
    def _create_publishers(self):
        """
        Buat publishers untuk detection results dan diagnostics.
        """
        try:
            # Publisher untuk hasil deteksi YOLOv12
            self.publisher = self.create_publisher(
                Yolov12Inference,  # Message type
                DETECTION_TOPIC,  # Topic name
                10  # QoS history depth
            )
            # Publisher untuk diagnostics
            self.diagnostic_pub = self.create_publisher(
                DiagnosticArray,  # Message type
                DIAGNOSTIC_TOPIC,  # Topic name
                10  # QoS history depth
            )
            self.get_logger().info(f"Publishers created: {DETECTION_TOPIC}, {DIAGNOSTIC_TOPIC}")  # Log publishers created
        except Exception as e:
            self.get_logger().error(f"Error creating publishers: {e}")  # Error jika gagal buat publisher
            raise  # Re-raise untuk shutdown node
    
    def _create_services(self):
        """
        Buat services untuk restart model dan status check.
        """
        try:
            # Service untuk restart model (reload dari disk)
            self.restart_srv = self.create_service(
                Trigger,  # Service type
                'restart_model',  # Service name
                self.restart_model_callback  # Callback function
            )
            
            # Service untuk get status node
            self.status_srv = self.create_service(
                Trigger,  # Service type
                'get_status',  # Service name
                self.get_status_callback  # Callback function
            )
            self.get_logger().info("Services created: restart_model, get_status")  # Log services created
        except Exception as e:
            self.get_logger().error(f"Error creating services: {e}")  # Error jika gagal buat service
            self.get_logger().error(traceback.format_exc())
    
    def _load_model(self):
        """
        Load YOLOv12 model dengan auto-fallback options jika error.
        Support untuk TensorRT, ONNX, dan PyTorch formats.
        """
        if not ULTRALYTICS_AVAILABLE:  # Cek jika ultralytics tersedia
            self.get_logger().error("Ultralytics not available. Cannot load YOLOv12 model.")  # Error jika tidak tersedia
            raise ImportError("Required package 'ultralytics' not found")  # Raise error untuk exit
        
        # Try to load model with multiple fallback options
        model_loaded = False
        original_path = self.model_path  # Save original path untuk reporting
        
        try:
            # First attempt with original path
            self.get_logger().info(f"Loading YOLOv12 model from: {self.model_path}")  # Log path model
            
            # Check if model file exists
            if not os.path.exists(self.model_path):  # Cek file model ada
                self.get_logger().warning(f"Model file not found: {self.model_path}")  # Warning jika file tidak ada
                
                # Try to find model in common locations
                possible_paths = [
                    self.model_path,  # Original path
                    os.path.join(os.path.expanduser('~'), self.model_path),  # Home directory
                    os.path.join('/opt/models', self.model_path),  # Common model directory
                    os.path.join(os.getcwd(), self.model_path),  # Current working directory
                    os.path.join(os.getcwd(), 'models', self.model_path),  # models subdirectory
                ]
                
                # Try alternate file extensions if not found
                base_name = os.path.splitext(self.model_path)[0]  # Get base name without extension
                possible_paths.extend([
                    f"{base_name}.pt",  # Try PyTorch format
                    f"{base_name}.onnx",  # Try ONNX format
                    f"{base_name}.engine"  # Try TensorRT format
                ])
                
                # Try each potential path
                for path in possible_paths:
                    if os.path.exists(path):
                        self.get_logger().info(f"Found model at: {path}")  # Log model found
                        self.model_path = path  # Update path
                        break
            
            # Load model with task="detect"
            self.model = YOLO(self.model_path, task="detect")  # Load model
            model_loaded = True  # Set flag sukses
            
            # Log successful loading
            self.get_logger().info(f"Successfully loaded YOLOv12 model: {self.model_path}")  # Log sukses
            
            # Log model info
            model_type = "Unknown"  # Default value
            if hasattr(self.model, 'info'):  # Cek attribute info ada
                self.get_logger().info(f"Model info: {self.model.info}")  # Log info model jika tersedia
            
            # Show model task
            self.get_logger().info(f"Model task: detect")  # Log task model
            
        except Exception as e:
            self.get_logger().error(f"Error loading YOLOv12 model: {e}")  # Error jika gagal load
            self.get_logger().error(traceback.format_exc())  # Error stack trace
            
            # Try fallback to PyTorch model if specified model failed
            if not model_loaded and original_path.endswith(('.engine', '.onnx')):  # Jika model engine/onnx gagal
                try:
                    fallback_path = original_path.rsplit('.', 1)[0] + '.pt'  # Ganti extension ke .pt
                    self.get_logger().info(f"Attempting fallback to PyTorch model: {fallback_path}")  # Log fallback
                    if os.path.exists(fallback_path):  # Cek file fallback ada
                        self.model = YOLO(fallback_path, task="detect")  # Load model fallback
                        self.model_path = fallback_path  # Update path
                        model_loaded = True  # Set flag sukses
                        self.get_logger().info(f"Successfully loaded fallback model: {fallback_path}")  # Log sukses fallback
                    else:
                        self.get_logger().error(f"Fallback model not found: {fallback_path}")  # Error jika fallback tidak ada
                except Exception as e2:
                    self.get_logger().error(f"Error loading fallback model: {e2}")  # Error jika fallback gagal
            
            if not model_loaded:  # Jika semua model gagal
                self.get_logger().error("Could not load any YOLOv12 model. Exiting.")  # Error dan exit
                raise  # Re-raise exception untuk exit node
    
    def _create_subscribers(self):
        """
        Buat subscribers untuk semua kamera dengan error handling.
        """
        try:
            # Check if topics list is valid
            if not self.camera_topics or len(self.camera_topics) == 0:  # Validasi list topic tidak kosong
                self.get_logger().error("No camera topics specified")  # Error jika tidak ada topic
                raise ValueError("No camera topics to subscribe to")  # Raise error untuk exit
            
            # Trim to specified camera count
            active_topics = self.camera_topics[:self.cam_count]  # Ambil topic sesuai jumlah kamera
            
            # Create subscription for each camera
            self.get_logger().info(f"Creating subscribers for {len(active_topics)} camera topics")  # Log jumlah subscribers
            
            self.subs = []  # List untuk track semua subscribers
            for i, topic in enumerate(active_topics):  # Loop untuk semua topic
                try:
                    # Create subscription with QoS profile
                    sub = self.create_subscription(
                        Image,  # Message type
                        topic,  # Topic name
                        lambda msg, idx=i: self.image_callback(msg, idx),  # Callback dengan capture index
                        10  # QoS history depth
                    )
                    self.subs.append(sub)  # Tambahkan ke list subs
                    self.get_logger().info(f"Subscribed to camera topic: {topic} (idx={i})")  # Log subscription
                except Exception as e:
                    self.get_logger().error(f"Error subscribing to {topic}: {e}")  # Error jika gagal subscribe
                    # Continue trying other topics, don't fail completely
            
            if len(self.subs) == 0:  # Cek jika semua subscription gagal
                self.get_logger().error("Failed to create any camera subscribers")  # Error jika semua gagal
                raise RuntimeError("No camera subscriptions created")  # Raise error untuk exit
            
            # Update cam_count to actual number of subscribed topics
            if len(self.subs) < self.cam_count:  # Jika jumlah subscription kurang dari cam_count
                self.get_logger().warning(f"Only {len(self.subs)} camera topics subscribed out of {self.cam_count} requested")  # Warning
                self.cam_count = len(self.subs)  # Update cam_count
                self.images = [None] * self.cam_count  # Reset buffer image
                self.last_frame_time = [None] * self.cam_count  # Reset timestamp
                self.detection_counts = [0] * self.cam_count  # Reset counter
                self.inference_times = [0.0] * self.cam_count  # Reset timing
                
        except Exception as e:
            self.get_logger().error(f"Error setting up camera subscribers: {e}")  # Error jika gagal setup subscribers
            self.get_logger().error(traceback.format_exc())
            raise  # Re-raise untuk shutdown node
    
    def _create_timers(self):
        """
        Buat timers untuk processing, visualisasi, dan diagnostics.
        """
        try:
            # Main timer for detection loop - 5Hz (200ms)
            self.timer = self.create_timer(0.2, self.process_images)  # Timer untuk proses image dan deteksi
            
            # Diagnostic timer - 1Hz (1000ms)
            self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)  # Timer untuk publish diagnostik
            
            self.get_logger().info("Timers created for processing and diagnostics")  # Log timers created
        except Exception as e:
            self.get_logger().error(f"Error creating timers: {e}")  # Error jika gagal buat timer
            raise  # Re-raise untuk shutdown node
    
    def image_callback(self, msg, idx):
        """
        Callback untuk menerima image dari kamera dan simpan ke buffer.
        
        Args:
            msg: Image message dari kamera
            idx: Index kamera (0-5 untuk hexagonal array)
        """
        try:
            with self.mutex:  # Thread safety untuk akses buffer image
                # Update timestamp untuk deteksi timeout
                self.last_frame_time[idx] = self.get_clock().now()  # Set timestamp terakhir
                
                # Convert ROS Image message to OpenCV format
                self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV
                
        except CvBridgeError as e:
            self.get_logger().warning(f"Error converting image from topic {self.camera_topics[idx]}: {e}")  # Warning jika konversi gagal
        except Exception as e:
            self.get_logger().error(f"Error in image callback for camera {idx}: {e}")  # Error jika callback error
    
    def process_images(self):
        """
        Process semua image di buffer, jalankan detection, dan publish results.
        Semua image di buffer diproses secara batch jika tersedia atau diproses individual.
        """
        if not self.running or not self.is_initialized:  # Cek jika node masih running dan initialized
            return  # Skip jika node shutting down atau belum initialized
        
        # Check if we have any images
        with self.mutex:  # Thread safety untuk akses buffer
            # Make a copy to avoid race condition during processing
            images_copy = self.images.copy()  # Copy buffer untuk processing
        
        # Count available images
        available_count = sum(1 for img in images_copy if img is not None)  # Hitung jumlah image tersedia
        
        if available_count == 0:  # Skip jika tidak ada image
            return
        
        # Process available images
        try:
            for idx, img in enumerate(images_copy):
                if img is not None:  # Skip None images
                    # Process this image
                    start_time = time.time()  # Start timer untuk benchmark
                    
                    try:
                        # Run YOLOv12 inference
                        results = self.model(img, verbose=False, conf=self.conf_thres)  # Inference dengan model YOLOv12
                        
                        # Measure inference time
                        infer_time = time.time() - start_time  # Hitung waktu inference
                        self.inference_times[idx] = infer_time  # Simpan untuk diagnostics
                        
                        # Apply class filter if specified
                        if results and self.class_filter:  # Jika ada filter class
                            # Filter by class
                            results = [result for result in results if any(cls in self.class_filter for cls in result.boxes.cls)]
                        
                        # Track detection count
                        self.detection_counts[idx] = len(results[0].boxes) if results else 0  # Update counter deteksi
                        
                        # Create annotated image if visualization is enabled
                        if self.visualization_enabled:  # Jika visualisasi enabled
                            annotated = results[0].plot()  # Annotate image dengan hasil deteksi
                            images_copy[idx] = annotated  # Simpan annotated image
                        
                        # Log detection summary
                        self.get_logger().debug(
                            f"Camera {idx}: {self.detection_counts[idx]} detections in {infer_time:.3f}s"
                        )  # Log detail deteksi
                        
                        # Publish detection results
                        self.publish_results(results, f"Camera_{idx}")  # Publish hasil ke topic
                        
                    except Exception as e:
                        self.get_logger().error(f"Error processing image from camera {idx}: {e}")  # Error jika proses gagal
                        self.get_logger().error(traceback.format_exc())
                        # Skip this camera but continue with others
            
            # Visualize results if enabled
            if self.visualization_enabled:  # Jika visualisasi enabled
                self.visualize_results(images_copy)  # Visualisasi hasil
                
        except Exception as e:
            self.get_logger().error(f"Error in process_images: {e}")  # Error jika proses gagal
            self.get_logger().error(traceback.format_exc())
    
    def publish_results(self, results, camera_name):
        """
        Publish hasil deteksi YOLOv12 ke topic /detection.
        
        Args:
            results: Hasil inference YOLOv12
            camera_name: Nama kamera untuk frame_id
        """
        if not results:  # Cek results ada
            return  # Skip jika results None
        
        try:
            # Create detection message
            msg = Yolov12Inference()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp sekarang
            msg.header.frame_id = camera_name  # Set frame_id ke nama kamera
            msg.camera_name = camera_name  # Set camera_name
            msg.frame_type = "raw"  # Type frame
            msg.task = "detect"  # Task type
            msg.note = ""  # Notes (empty)
            msg.yolov12_inference = []  # Initialize list inference hasil
            
            # Add detection results to message
            try:
                for box in results[0].boxes:
                    det = InferenceResult()  # Buat object InferenceResult baru
                    
                    # Get class info (handling different result formats)
                    try:
                        det.class_name = str(int(box.cls.item()))  # Class ID atau name
                    except (AttributeError, TypeError):
                        det.class_name = str(box.cls) if hasattr(box, 'cls') else "0"  # Fallback jika format tidak sesuai
                    
                    # Get confidence
                    try:
                        det.confidence = float(box.conf.item())  # Confidence value
                    except (AttributeError, TypeError):
                        det.confidence = float(box.conf) if hasattr(box, 'conf') else 0.0  # Fallback jika format tidak sesuai
                    
                    # Get bounding box (XYXY format: [left, top, right, bottom])
                    try:
                        coords = box.xyxy[0].tolist() if hasattr(box.xyxy[0], 'tolist') else box.xyxy[0]
                        det.left = int(coords[0])  # Left (X min)
                        det.top = int(coords[1])  # Top (Y min)
                        det.right = int(coords[2])  # Right (X max)
                        det.bottom = int(coords[3])  # Bottom (Y max)
                    except (AttributeError, IndexError, TypeError) as e:
                        self.get_logger().warning(f"Error extracting box coordinates: {e}, using defaults")  # Warning jika ekstrak gagal
                        det.left = 0  # Default left
                        det.top = 0  # Default top
                        det.right = 10  # Default right
                        det.bottom = 10  # Default bottom
                    
                    # Get tracking ID if available
                    try:
                        det.track_id = int(box.id.item()) if hasattr(box, "id") and box.id is not None else -1  # Track ID (jika tracking aktif)
                    except (AttributeError, TypeError):
                        det.track_id = -1  # Default jika tidak ada tracking
                    
                    # Fill fields for other tasks (always included for compatibility)
                    det.obb_angle = -1  # Default untuk task detect (bukan OBB)
                    det.mask_indices = []  # Default untuk task detect (bukan segmentation)
                    
                    msg.yolov12_inference.append(det)  # Tambahkan ke list hasil
                
                # Publish message
                self.publisher.publish(msg)  # Publish message ke topic
                
            except Exception as e:
                self.get_logger().error(f"Error processing detection results: {e}")  # Error jika proses hasil gagal
                self.get_logger().error(traceback.format_exc())
                
        except Exception as e:
            self.get_logger().error(f"Error publishing detection results: {e}")  # Error jika publish gagal
            self.get_logger().error(traceback.format_exc())
    
    def visualize_results(self, images):
        """
        Visualisasikan hasil deteksi dari semua kamera dalam satu window.
        
        Args:
            images: List image hasil deteksi yang sudah diannotate
        """
        if not self.visualization_enabled:  # Skip jika visualisasi disabled
            return
        
        try:
            # Filter out None images
            valid_images = [img for img in images if img is not None]  # Ambil hanya image yang valid
            
            if not valid_images:  # Skip jika tidak ada image valid
                return
            
            # Resize images to consistent height
            target_height = 240  # Tinggi target visualisasi
            resized_images = []
            
            for image in valid_images:
                h, w = image.shape[:2]
                scale = target_height / h if h > target_height else 1  # Scale ke target height
                new_w = int(w * scale)
                resized_image = cv2.resize(image, (new_w, target_height))
                resized_images.append(resized_image)
            
            # Add borders between images
            border_thickness = 5  # Ketebalan border antar kamera
            bordered_images = []
            
            for idx, img in enumerate(resized_images):
                bordered_images.append(img)
                if idx < len(resized_images) - 1:  # Jika bukan image terakhir
                    border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)  # Buat border hitam
                    bordered_images.append(border)
            
            # Combine all images horizontally
            try:
                combined_image = cv2.hconcat(bordered_images)  # Gabungkan semua image horizontal
                
                # Add timestamp to image
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(
                    combined_image,
                    f"Huskybot Detection - {timestamp}",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )
                
                # Show combined image
                cv2.imshow("MultiCam YOLOv12 Detection", combined_image)  # Show visualisasi
                cv2.waitKey(DISPLAY_TIMEOUT_MS)  # Non-blocking display
                
            except cv2.error as e:
                self.get_logger().warning(f"OpenCV error in visualization: {e}")  # Warning jika error CV
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")  # Error jika visualisasi gagal
            if 'Cannot connect to X server' in str(e):  # Error umum di headless server
                self.get_logger().warning("Running in headless mode, disabling visualization")  # Warning dan disable
                self.visualization_enabled = False
    
    def publish_diagnostics(self):
        """
        Publish diagnostic status dari node dan detection ke topic /diagnostics.
        """
        try:
            # Create diagnostic message
            diag_msg = DiagnosticArray()  # Create diagnostic message
            diag_msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
            
            # Add node status
            status = DiagnosticStatus()  # Create status
            status.name = "huskybot_detection"  # Node name
            status.hardware_id = platform.node()  # Use hostname as hardware ID
            
            # Determine overall status
            if not self.is_initialized or self.model is None:
                status.level = DiagnosticStatus.ERROR
                status.message = "Node not fully initialized or model failed to load"
            else:
                # Check if any cameras have timed out
                now = self.get_clock().now()
                timeout_cameras = 0
                
                for i, last_time in enumerate(self.last_frame_time):
                    if last_time is not None:
                        elapsed = (now - last_time).nanoseconds / 1e9
                        if elapsed > CAMERA_TIMEOUT_SEC:
                            timeout_cameras += 1
                
                if timeout_cameras == self.cam_count:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "All cameras timed out"
                elif timeout_cameras > 0:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"{timeout_cameras}/{self.cam_count} cameras timed out"
                else:
                    # Check inference times
                    slow_cameras = sum(1 for t in self.inference_times if t > INFERENCE_TIMEOUT_SEC)
                    if slow_cameras > 0:
                        status.level = DiagnosticStatus.WARN
                        status.message = f"{slow_cameras}/{self.cam_count} cameras with slow inference"
                    else:
                        status.level = DiagnosticStatus.OK
                        status.message = "Normal operation"
            
            # Add key-values to status
            status.values.append(KeyValue(key="model_path", value=self.model_path))
            status.values.append(KeyValue(key="camera_count", value=str(self.cam_count)))
            status.values.append(KeyValue(key="platform", value="Jetson" if self.is_jetson else "Generic"))
            
            # Add detection counts
            for i, count in enumerate(self.detection_counts):
                status.values.append(KeyValue(key=f"detections_cam{i}", value=str(count)))
            
            # Add inference times
            for i, infer_time in enumerate(self.inference_times):
                status.values.append(KeyValue(key=f"inference_time_cam{i}", value=f"{infer_time:.3f}s"))
            
            # Add to array
            diag_msg.status.append(status)
            
            # Publish diagnostic message
            self.diagnostic_pub.publish(diag_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing diagnostics: {e}")  # Error jika publish diagnostic gagal
    
    def restart_model_callback(self, request, response):
        """
        Service callback untuk restart_model service.
        Re-loads model dari disk.
        """
        try:
            self.get_logger().info("Restarting YOLOv12 model")  # Log restart
            
            # Re-load the model
            self._load_model()  # Load ulang model
            
            response.success = True
            response.message = f"Model restarted successfully: {self.model_path}"
            return response
        except Exception as e:
            self.get_logger().error(f"Error restarting model: {e}")  # Error jika restart gagal
            response.success = False
            response.message = f"Failed to restart model: {str(e)}"
            return response
    
    def get_status_callback(self, request, response):
        """
        Service callback untuk get_status service.
        Returns node status information.
        """
        try:
            # Create status message
            status_msg = f"MultiCamDetectionNode Status:\n"
            status_msg += f"- Camera count: {self.cam_count}\n"
            status_msg += f"- Model path: {self.model_path}\n"
            status_msg += f"- Platform: {platform.platform()}\n"
            status_msg += f"- Is Jetson: {self.is_jetson}\n"
            
            # Add detection counts
            status_msg += "- Detection counts:\n"
            for i, count in enumerate(self.detection_counts):
                status_msg += f"  - Camera {i}: {count} detections\n"
            
            # Add inference times
            status_msg += "- Inference times:\n"
            for i, infer_time in enumerate(self.inference_times):
                status_msg += f"  - Camera {i}: {infer_time:.3f}s\n"
            
            response.success = True
            response.message = status_msg
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting status: {e}")  # Error jika get status gagal
            response.success = False
            response.message = f"Failed to get status: {str(e)}"
            return response
    
    def on_shutdown(self):
        """
        Clean shutdown handler when node is being terminated.
        """
        self.running = False  # Set flag untuk stop processing
        self.get_logger().info("Shutting down MultiCamDetectionNode...")  # Log shutdown
        
        # Close OpenCV windows if any
        if self.visualization_enabled:  # Jika visualisasi enabled
            try:
                cv2.destroyAllWindows()  # Close semua window OpenCV
            except Exception as e:
                self.get_logger().warning(f"Error closing OpenCV windows: {e}")  # Warning jika close gagal
        
        # Close logger files
        try:
            logging.shutdown()  # Shutdown logging
        except Exception as e:
            print(f"Error shutting down logging: {e}")  # Print error jika shutdown logging gagal

def main(args=None):
    """
    Main entry point untuk node.
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    """
    # Initialize ROS2
    rclpy.init(args=args)  # Inisialisasi ROS2
    
    node = None  # Initialize node reference
    
    try:
        # Create node
        node = MultiCamDetectionNode()  # Buat instance node
        
        # Register shutdown handler
        rclpy.get_global_executor().add_node(node)  # Add node ke executor
        
        # Spin node
        rclpy.spin(node)  # Spin node sampai shutdown
        
    except KeyboardInterrupt:
        # Handle Ctrl+C
        print("KeyboardInterrupt, shutting down...")
        if node:
            node.get_logger().info('KeyboardInterrupt, shutting down node.')
    except Exception as e:
        # Handle other exceptions
        print(f"Fatal error: {e}")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
            node.get_logger().error(traceback.format_exc())
    finally:
        # Clean up
        if node:
            # Call shutdown handler
            node.on_shutdown()
            
            try:
                node.destroy_node()  # Destroy ROS2 node
            except Exception as e:
                print(f"Error destroying node: {e}")
        
        # Shutdown ROS2
        try:
            rclpy.shutdown()  # Shutdown ROS2
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")
        
        print("Multicam detection node shutdown complete.")

if __name__ == "__main__":
    main()  # Execute main function

# ===================== PENJELASAN TAMBAHAN =====================
# File ini mengimplementasikan node deteksi objek multicamera dengan YOLOv12 untuk robot Huskybot.
# Node menerima image dari 6 kamera dalam konfigurasi hexagonal, menjalankan deteksi objek pada
# setiap image, dan mempublish hasil ke topic /detection dengan message type Yolov12Inference.
# Node sudah FULL OOP, dengan error handling komprehensif di setiap bagian.
#
# Error handling yang diterapkan meliputi:
# 1. Validasi parameter saat startup
# 2. Multiple fallback untuk loading model (coba beberapa path dan format)
# 3. Thread safety dengan mutex di callback dan processing
# 4. Handling exceptions di setiap callback dan timer function
# 5. Diagnostics yang terperinci untuk monitoring health node
# 6. Service API untuk restart model dan status check
# 7. Handler untuk berbagai format output YOLOv12
# 8. Deteksi platform dan optimasi untuk Jetson
# 9. Graceful shutdown dengan cleanup resources
# 10. Support untuk mode headless jika tidak ada display