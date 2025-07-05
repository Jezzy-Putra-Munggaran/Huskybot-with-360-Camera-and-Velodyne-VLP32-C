#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os  # Untuk operasi path dan environment
import sys  # Untuk akses ke sys.exit dan error
import time  # Untuk timing dan delay
import traceback  # Untuk log error stack trace
import threading  # Untuk thread lock dan concurrency
from typing import List, Dict, Any, Optional, Union  # Type hinting
import numpy as np  # Operasi array/matrix
import cv2  # OpenCV untuk image processing
from datetime import datetime  # Timestamp log

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # QoS untuk topic
from rclpy.parameter import Parameter  # Parameter dinamis ROS2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup  # Threading ROS2
from rclpy.executors import MultiThreadedExecutor  # Executor multi-thread
from rclpy.exceptions import ParameterNotDeclaredException  # Exception parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Deskripsi parameter

from sensor_msgs.msg import Image  # Message image ROS2
from cv_bridge import CvBridge, CvBridgeError  # Konversi ROS <-> OpenCV
from std_msgs.msg import Header  # Header standar ROS2
from std_srvs.srv import Trigger  # Service Trigger (restart/status)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # Diagnostik ROS2

try:
    from ultralytics import YOLO  # YOLOv12 segmentasi
except ImportError as e:
    print(f"[FATAL] Ultralytics tidak terinstall: {e}")  # Error jika ultralytics tidak ada
    print("Instal dengan: pip install ultralytics")
    sys.exit(1)  # Exit jika tidak ada ultralytics

from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil YOLOv12

# ===================== KONSTANTA GLOBAL =====================
DEFAULT_CAMERA_COUNT = 6  # Default 6 kamera (hexagonal)
DEFAULT_CAMERA_TOPICS = [
    '/camera_front/image_raw',        # Kamera depan
    '/camera_right/image_raw',        # Kamera kanan
    '/camera_rear_right/image_raw',   # Kamera belakang-kanan
    '/camera_rear/image_raw',         # Kamera belakang
    '/camera_left/image_raw',         # Kamera kiri
    '/camera_front_left/image_raw'    # Kamera depan-kiri
]
DEFAULT_MODEL_PATH = "yolo11x-seg.engine"  # Default model segmentasi
DEFAULT_CONF_THRESHOLD = 0.5  # Default threshold confidence
DEFAULT_MASK_ENABLE = True  # Default mask segmentasi aktif
LOG_DIR = os.path.expanduser("~/huskybot_segmentation_log")  # Direktori log

# ===================== UTILITY FUNCTIONS =====================
def log_to_file(msg: str, level: str = 'INFO', log_file: Optional[str] = None) -> None:
    """Log pesan ke file dengan timestamp dan level tertentu."""
    try:
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR, exist_ok=True)  # Buat direktori log jika belum ada
        if log_file is None:
            current_time = datetime.now()
            log_file = f"{LOG_DIR}/segmentation_{current_time.strftime('%Y%m%d')}.log"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_message = f"[{timestamp}] [{level}] {msg}\n"
        with open(log_file, 'a') as f:
            f.write(log_message)
    except Exception as e:
        print(f"[{level}] {msg}\n[ERROR] Gagal menulis ke log file: {e}", file=sys.stderr)

def is_jetson_platform() -> bool:
    """Deteksi apakah kode berjalan di platform Jetson NVIDIA."""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip().lower()
            return 'jetson' in model or 'tegra' in model
    except:
        try:
            return 'jetson' in os.environ.get('JETSON_TYPE', '').lower()
        except:
            return False

# ===================== NODE MULTICAM SEGMENTATION =====================
class MultiCamSegmentationNode(Node):
    """Node ROS2 untuk segmentasi multicamera menggunakan YOLOv12."""

    def __init__(self):
        super().__init__('multicam_segmentation_node')
        
        # Declare parameters dengan tipe yang tepat
        self.declare_parameter('cam_count', 6)
        self.declare_parameter('model_path', 'yolo11x-seg.engine')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('visualization_enabled', True)
        self.declare_parameter('publish_rate', 10.0)
        
        # FIXED: Declare camera topics sebagai parameter individual
        for i in range(6):
            default_topic = f'/camera_{self._get_camera_name(i)}/image_raw'
            self.declare_parameter(f'camera_topic_{i}', default_topic)
        
        # Get parameters
        self.cam_count = self.get_parameter('cam_count').get_parameter_value().integer_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.visualization_enabled = self.get_parameter('visualization_enabled').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # FIXED: Build camera topics list dari individual parameters
        self.camera_topics = []
        for i in range(self.cam_count):
            topic = self.get_parameter(f'camera_topic_{i}').get_parameter_value().string_value
            self.camera_topics.append(topic)
        
        self.get_logger().info(f"Camera topics: {self.camera_topics}")
        
        # Initialize rest of the node...
        self._initialize_node()

    def _get_camera_name(self, index):
        """Get camera name based on hexagonal arrangement"""
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        return camera_names[index] if index < len(camera_names) else f'camera_{index}'

    def _initialize_node(self):
        """Inisialisasi node setelah parameter dibaca."""
        self.log_file_path = None  # Path file log
        self._setup_logging()  # Setup logging awal
        self.get_logger().info("Menginisialisasi multicam_segmentation node...")
        log_to_file("Menginisialisasi multicam_segmentation node...", log_file=self.log_file_path)

        # Callback groups untuk concurrency
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self._declare_parameters()  # Deklarasi parameter ROS2
        self.image_lock = threading.RLock()  # Lock buffer gambar
        self.process_lock = threading.RLock()  # Lock proses inferensi
        self._load_parameters()  # Load parameter dari server

        self.images = [None] * self.cam_count  # Buffer gambar
        self.last_received = [0.0] * self.cam_count  # Timestamp gambar
        self.frame_count = [0] * self.cam_count  # Counter frame
        self.model = None  # Model YOLOv12
        self.bridge = CvBridge()  # Konversi ROS <-> OpenCV

        self.is_running = True  # Status node
        self.last_inference_time = 0.0  # Waktu inferensi terakhir
        self.fps = 0.0  # FPS
        self.inference_times = []  # List waktu inferensi
        self.max_inference_times = 30  # History max

        self.is_jetson = is_jetson_platform()  # Deteksi Jetson
        if self.is_jetson:
            self.get_logger().info("Terdeteksi platform Jetson - optimasi NVIDIA...")
            log_to_file("Terdeteksi platform Jetson - optimasi NVIDIA...", log_file=self.log_file_path)

        self._setup_services()  # Setup service restart/status
        self._load_model()  # Load model YOLOv12
        self._setup_publishers()  # Setup publisher hasil segmentasi
        self.image_subscribers = []  # List subscription kamera
        self._setup_image_subscribers()  # Setup semua kamera

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.process_and_publish_images, callback_group=self.timer_callback_group
        )  # Timer proses segmentasi

        self.diag_timer = self.create_timer(
            1.0, self.publish_diagnostics, callback_group=self.timer_callback_group
        )  # Timer diagnostik

        self.get_logger().info("Node multicam_segmentation siap!")
        log_to_file("Node multicam_segmentation siap!", log_file=self.log_file_path)

    def _setup_logging(self):
        """Setup file logging path dan direktori."""
        try:
            self.declare_parameter(
                'log_file',
                f"{LOG_DIR}/segmentation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
                ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Path ke file log untuk node ini')
            )
            self.log_file_path = self.get_parameter('log_file').get_parameter_value().string_value
            log_dir = os.path.dirname(self.log_file_path)
            os.makedirs(log_dir, exist_ok=True)
            self.get_logger().info(f"Log file: {self.log_file_path}")
        except Exception as e:
            self.get_logger().error(f"Gagal setup logging: {e}")
            self.log_file_path = f"{LOG_DIR}/segmentation_fallback.log"
            traceback.print_exc()

    def _declare_parameters(self):
        """Deklarasi semua parameter ROS2 dengan deskripsi dan nilai default."""
        try:
            self.declare_parameter('cam_count', DEFAULT_CAMERA_COUNT, ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER, description='Jumlah kamera (default: 6 hexagonal)'))
            self.declare_parameter('model_path', DEFAULT_MODEL_PATH, ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description='Path ke model YOLOv12 (*.pt, *.engine, *.onnx)'))
            self.declare_parameter('camera_topics', DEFAULT_CAMERA_TOPICS, ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY, description='Daftar topic kamera (hexagonal)'))
            self.declare_parameter('conf_threshold', DEFAULT_CONF_THRESHOLD, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description='Threshold confidence (0.0-1.0)'))
            self.declare_parameter('publish_rate', 5.0, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description='Frequency publish hasil segmentasi (Hz)'))
            self.declare_parameter('enable_mask', DEFAULT_MASK_ENABLE, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL, description='Aktifkan mask segmentasi pada output'))
            self.declare_parameter('enable_visualization', True, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL, description='Aktifkan visualisasi OpenCV'))
            self.declare_parameter('class_filter', [], ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY, description='Filter class yang akan dideteksi'))
            self.declare_parameter('device', '0' if self.is_jetson else 'cpu', ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description='Device untuk inferensi (0=GPU, cpu=CPU)'))
        except Exception as e:
            self.get_logger().error(f"Error deklarasi parameter: {e}")
            log_to_file(f"Error deklarasi parameter: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _load_parameters(self):
        """Load semua parameter dari ROS2 parameter server."""
        try:
            self.cam_count = self.get_parameter('cam_count').get_parameter_value().integer_value
            self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
            self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
            self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
            self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
            self.enable_mask = self.get_parameter('enable_mask').get_parameter_value().bool_value
            self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
            self.class_filter = self.get_parameter('class_filter').get_parameter_value().string_array_value
            self.device = self.get_parameter('device').get_parameter_value().string_value

            # Validasi parameter
            if self.cam_count <= 0:
                self.get_logger().warn(f"cam_count ({self.cam_count}) tidak valid, pakai default: {DEFAULT_CAMERA_COUNT}")
                self.cam_count = DEFAULT_CAMERA_COUNT
            if len(self.camera_topics) < self.cam_count:
                self.get_logger().warn(f"Jumlah topic kamera kurang, auto lengkapi dengan default.")
                while len(self.camera_topics) < self.cam_count:
                    idx = len(self.camera_topics)
                    if idx < len(DEFAULT_CAMERA_TOPICS):
                        self.camera_topics.append(DEFAULT_CAMERA_TOPICS[idx])
                    else:
                        self.camera_topics.append(f"/camera_{idx}/image_raw")
            if len(self.camera_topics) > self.cam_count:
                self.camera_topics = self.camera_topics[:self.cam_count]
            if self.conf_threshold < 0.0 or self.conf_threshold > 1.0:
                self.get_logger().warn(f"conf_threshold ({self.conf_threshold}) tidak valid, pakai default: {DEFAULT_CONF_THRESHOLD}")
                self.conf_threshold = DEFAULT_CONF_THRESHOLD
            if self.publish_rate <= 0.0:
                self.get_logger().warn(f"publish_rate ({self.publish_rate}) tidak valid, pakai default: 5.0")
                self.publish_rate = 5.0
            self.get_logger().info(f"Parameter loaded: cam_count={self.cam_count}, model_path={self.model_path}, conf_threshold={self.conf_threshold}")
            self.get_logger().info(f"Camera topics: {self.camera_topics}")
            log_to_file(f"Parameter loaded: cam_count={self.cam_count}, model_path={self.model_path}, conf_threshold={self.conf_threshold}", log_file=self.log_file_path)
            log_to_file(f"Camera topics: {self.camera_topics}", log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error load parameter: {e}")
            log_to_file(f"Error load parameter: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _setup_services(self):
        """Setup services untuk node ini."""
        try:
            self.restart_service = self.create_service(
                Trigger, 'restart_model', self.handle_restart_service, callback_group=self.service_callback_group)
            self.status_service = self.create_service(
                Trigger, 'get_status', self.handle_status_service, callback_group=self.service_callback_group)
            self.get_logger().info(f"Service registered: restart_model, get_status")
            log_to_file(f"Service registered: restart_model, get_status", log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error setup services: {e}")
            log_to_file(f"Error setup services: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def handle_restart_service(self, request, response):
        """Handler untuk service restart_model."""
        try:
            self.get_logger().info("Menerima request restart model...")
            log_to_file("Menerima request restart model...", log_file=self.log_file_path)
            self._load_model()
            response.success = True
            response.message = f"Model segmentasi berhasil di-restart pada {datetime.now().isoformat()}"
        except Exception as e:
            self.get_logger().error(f"Error restart model: {e}")
            log_to_file(f"Error restart model: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            response.success = False
            response.message = f"Gagal restart model: {str(e)}"
        return response

    def handle_status_service(self, request, response):
        """Handler untuk service get_status."""
        try:
            active_cameras = sum(1 for img in self.images if img is not None)
            status_msg = (
                f"Status: {active_cameras}/{self.cam_count} kamera aktif, "
                f"FPS: {self.fps:.2f}, "
                f"Model: {os.path.basename(self.model_path)}, "
                f"Device: {self.device}"
            )
            response.success = True
            response.message = status_msg
            self.get_logger().info(f"Status: {status_msg}")
            log_to_file(f"Status: {status_msg}", log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error get status: {e}")
            log_to_file(f"Error get status: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            response.success = False
            response.message = f"Gagal get status: {str(e)}"
        return response

    def _load_model(self):
        """Load model YOLOv12 untuk segmentasi."""
        try:
            if not os.path.exists(self.model_path):
                error_msg = f"Model file tidak ditemukan: {self.model_path}"
                self.get_logger().error(error_msg)
                log_to_file(error_msg, level='ERROR', log_file=self.log_file_path)
                alt_locations = [
                    os.path.join(os.path.dirname(os.path.realpath(__file__)), self.model_path),
                    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', self.model_path),
                    os.path.join(os.path.expanduser('~'), 'models', self.model_path),
                    os.path.join('/opt/models', self.model_path)
                ]
                found = False
                for loc in alt_locations:
                    if os.path.exists(loc):
                        self.model_path = loc
                        found = True
                        self.get_logger().info(f"Model ditemukan di lokasi alternatif: {loc}")
                        log_to_file(f"Model ditemukan di lokasi alternatif: {loc}", log_file=self.log_file_path)
                        break
                if not found:
                    raise FileNotFoundError(f"Model tidak ditemukan di semua lokasi: {self.model_path}")
            self.get_logger().info(f"Loading model dari {self.model_path}...")
            log_to_file(f"Loading model dari {self.model_path}...", log_file=self.log_file_path)
            model_ext = os.path.splitext(self.model_path)[1].lower()
            start_time = time.time()
            if model_ext == '.engine':
                if not self.is_jetson:
                    self.get_logger().warn("File TensorRT .engine digunakan di non-Jetson. Performa mungkin tidak optimal.")
                    log_to_file("File TensorRT .engine digunakan di non-Jetson. Performa mungkin tidak optimal.", level='WARN', log_file=self.log_file_path)
                self.model = YOLO(self.model_path, task="segment")
            elif model_ext == '.onnx':
                self.model = YOLO(self.model_path, task="segment")
            elif model_ext == '.pt':
                if self.is_jetson:
                    self.get_logger().warn("PyTorch model (.pt) di Jetson. Sarankan export ke ONNX/TensorRT.")
                    log_to_file("PyTorch model (.pt) di Jetson. Sarankan export ke ONNX/TensorRT.", level='WARN', log_file=self.log_file_path)
                self.model = YOLO(self.model_path, task="segment")
            else:
                self.get_logger().warn(f"Format model tidak dikenali: {model_ext}, pakai default loader")
                log_to_file(f"Format model tidak dikenali: {model_ext}, pakai default loader", level='WARN', log_file=self.log_file_path)
                self.model = YOLO(self.model_path, task="segment")
            load_time = time.time() - start_time
            self.get_logger().info(f"Model berhasil dimuat dalam {load_time:.2f} detik")
            log_to_file(f"Model berhasil dimuat dalam {load_time:.2f} detik", log_file=self.log_file_path)
            if hasattr(self.model, 'names'):
                self.class_names = self.model.names
                self.get_logger().info(f"Kelas model: {self.class_names}")
                log_to_file(f"Kelas model: {self.class_names}", log_file=self.log_file_path)
            else:
                self.class_names = {}
                self.get_logger().warn("Tidak dapat membaca kelas dari model")
                log_to_file("Tidak dapat membaca kelas dari model", level='WARN', log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error fatal loading model: {e}")
            log_to_file(f"Error fatal loading model: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _setup_publishers(self):
        """Setup publisher untuk hasil segmentasi dan diagnostik."""
        try:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
            self.publisher = self.create_publisher(
                Yolov12Inference, '/detection', qos_profile)
            self.diag_publisher = self.create_publisher(
                DiagnosticArray, '/diagnostics', 10)
            self.get_logger().info("Publisher registered: /detection, /diagnostics")
            log_to_file("Publisher registered: /detection, /diagnostics", log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error setup publishers: {e}")
            log_to_file(f"Error setup publishers: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _setup_image_subscribers(self):
        """Setup subscribers untuk semua kamera."""
        try:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            for i, topic in enumerate(self.camera_topics):
                try:
                    subscription = self.create_subscription(
                        Image, topic, lambda msg, idx=i: self.image_callback(msg, idx),
                        qos, callback_group=self.subscription_callback_group
                    )
                    self.image_subscribers.append((i, topic, subscription))
                    self.get_logger().info(f"Subscribed ke kamera #{i}: {topic}")
                    log_to_file(f"Subscribed ke kamera #{i}: {topic}", log_file=self.log_file_path)
                except Exception as e:
                    self.get_logger().error(f"Error subscribe ke topic kamera {topic}: {e}")
                    log_to_file(f"Error subscribe ke topic kamera {topic}: {e}", level='ERROR', log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error setup image subscribers: {e}")
            log_to_file(f"Error setup image subscribers: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def image_callback(self, msg, idx):
        """Callback untuk image subscriber - menerima gambar dari kamera."""
        try:
            if idx < 0 or idx >= self.cam_count:
                self.get_logger().warn(f"Index kamera tidak valid: {idx}, max={self.cam_count-1}")
                return
            with self.image_lock:
                try:
                    self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    self.last_received[idx] = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
                    self.frame_count[idx] += 1
                    if self.frame_count[idx] % 30 == 0:
                        self.get_logger().debug(f"Received frame #{self.frame_count[idx]} from camera #{idx}")
                except CvBridgeError as e:
                    self.get_logger().warn(f"Error konversi image dari kamera #{idx}: {e}")
                    log_to_file(f"Error konversi image dari kamera #{idx}: {e}", level='WARN', log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error di image_callback untuk kamera #{idx}: {e}")
            log_to_file(f"Error di image_callback untuk kamera #{idx}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def process_and_publish_images(self):
        """Callback untuk timer - proses semua gambar dan publish hasil segmentasi."""
        if not self.process_lock.acquire(blocking=False):
            return
        try:
            with self.image_lock:
                images_copy = [img.copy() if img is not None else None for img in self.images]
            if all(img is None for img in images_copy):
                available = sum(1 for img in images_copy if img is not None)
                if available == 0:
                    if hasattr(self, '_log_counter'):
                        self._log_counter += 1
                        if self._log_counter >= 20:
                            self.get_logger().info(f"Menunggu feed kamera ({available}/{self.cam_count} tersedia)")
                            self._log_counter = 0
                    else:
                        self._log_counter = 0
                return
            start_time = time.time()
            annotated_images = []
            for idx, img in enumerate(images_copy):
                if img is None:
                    annotated_images.append(None)
                    continue
                try:
                    results = self.model(
                        img,
                        verbose=False,
                        conf=self.conf_threshold,
                        device=self.device,
                        classes=None if not self.class_filter else [int(c) if c.isdigit() else c for c in self.class_filter]
                    )
                    annotated = results[0].plot()
                    annotated_images.append(annotated)
                    self.publish_results(results, f"Camera_{idx+1}")
                    if self.enable_mask and hasattr(results[0], "masks") and results[0].masks is not None:
                        mask_count = len(results[0].masks)
                        if mask_count > 0:
                            self.get_logger().debug(f"[Segmentasi] Kamera #{idx+1}: {mask_count} masks")
                except Exception as e:
                    self.get_logger().error(f"Error segmentasi YOLOv12 kamera #{idx+1}: {e}")
                    log_to_file(f"Error segmentasi YOLOv12 kamera #{idx+1}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
                    if img is not None:
                        annotated_images.append(np.zeros_like(img))
                    else:
                        annotated_images.append(None)
            inference_time = time.time() - start_time
            self.inference_times.append(inference_time)
            if len(self.inference_times) > self.max_inference_times:
                self.inference_times.pop(0)
            if self.inference_times:
                avg_time = sum(self.inference_times) / len(self.inference_times)
                self.fps = 1.0 / avg_time if avg_time > 0 else 0.0
            if self.enable_visualization:
                self.visualize_results(annotated_images)
        except Exception as e:
            self.get_logger().error(f"Error di process_and_publish_images: {e}")
            log_to_file(f"Error di process_and_publish_images: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
        finally:
            self.process_lock.release()

    def visualize_results(self, annotated_images):
        """Visualisasikan hasil segmentasi multi-kamera."""
        try:
            valid_images = [img for img in annotated_images if img is not None]
            if not valid_images:
                return
            target_height = 240
            resized_images = []
            for image in valid_images:
                h, w = image.shape[:2]
                scale = target_height / h if h > target_height else 1
                new_w = int(w * scale)
                resized_image = cv2.resize(image, (new_w, target_height))
                resized_images.append(resized_image)
            border_thickness = 5
            bordered_images = []
            for idx, img in enumerate(resized_images):
                bordered_images.append(img)
                if idx < len(resized_images) - 1:
                    border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                    bordered_images.append(border)
            try:
                combined_image = cv2.hconcat(bordered_images)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(combined_image, f"FPS: {self.fps:.1f}", (10, 30), font, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_image, f"Model: {os.path.basename(self.model_path)}", (10, 60), font, 0.7, (0, 255, 0), 2)
                cv2.imshow("MultiCam YOLOv12 Segmentation", combined_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error menggabungkan gambar: {e}")
                log_to_file(f"Error menggabungkan gambar: {e}", level='ERROR', log_file=self.log_file_path)
        except cv2.error as e:
            self.get_logger().warning(f"Error visualisasi OpenCV: {e}")
            log_to_file(f"Error visualisasi OpenCV: {e}", level='WARN', log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error visualisasi: {e}")
            log_to_file(f"Error visualisasi: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def publish_results(self, results, camera_name):
        """Publish hasil segmentasi ke topic ROS2."""
        try:
            msg = Yolov12Inference()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = camera_name
            msg.camera_name = camera_name
            msg.frame_type = "raw"
            msg.task = "segment"
            msg.note = ""
            msg.yolov12_inference = []
            if not results or not hasattr(results, "__len__") or len(results) == 0:
                self.publisher.publish(msg)
                return
            for box in results[0].boxes:
                try:
                    det = InferenceResult()
                    cls_idx = box.cls.item() if hasattr(box.cls, "item") else int(box.cls)
                    if self.class_names and cls_idx in self.class_names:
                        det.class_name = str(self.class_names[cls_idx])
                    else:
                        det.class_name = str(cls_idx)
                    det.confidence = float(box.conf.item()) if hasattr(box.conf, "item") else float(box.conf)
                    if hasattr(box, "xyxy") and box.xyxy is not None:
                        det.top = int(box.xyxy[0][1])
                        det.left = int(box.xyxy[0][0])
                        det.bottom = int(box.xyxy[0][3])
                        det.right = int(box.xyxy[0][2])
                    elif hasattr(box, "xywh") and box.xywh is not None:
                        x, y, w, h = box.xywh[0]
                        det.left = int(x - w/2)
                        det.top = int(y - h/2)
                        det.right = int(x + w/2)
                        det.bottom = int(y + h/2)
                    else:
                        det.top = 0
                        det.left = 0
                        det.bottom = 0
                        det.right = 0
                        self.get_logger().warn(f"Box format tidak dikenali untuk {camera_name}")
                    if hasattr(box, "id") and box.id is not None:
                        det.track_id = int(box.id.item()) if hasattr(box.id, "item") else int(box.id)
                    else:
                        det.track_id = -1
                    det.obb_angle = 0
                    if self.enable_mask and hasattr(results[0], "masks") and results[0].masks is not None and len(results[0].masks) > 0:
                        try:
                            mask_idx = results[0].boxes.tolist().index(box.tolist()) if hasattr(results[0].boxes, "tolist") and hasattr(box, "tolist") else -1
                            if mask_idx >= 0 and mask_idx < len(results[0].masks):
                                mask = results[0].masks[mask_idx]
                                if hasattr(mask, "data"):
                                    det.mask_indices = mask.data.flatten().tolist()
                                else:
                                    det.mask_indices = []
                            else:
                                det.mask_indices = []
                        except Exception as e:
                            self.get_logger().warn(f"Error extract mask untuk {camera_name}: {e}")
                            det.mask_indices = []
                    else:
                        det.mask_indices = []
                    msg.yolov12_inference.append(det)
                except Exception as e:
                    self.get_logger().error(f"Error proses box untuk {camera_name}: {e}")
                    log_to_file(f"Error proses box untuk {camera_name}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publish_results untuk {camera_name}: {e}")
            log_to_file(f"Error publish_results untuk {camera_name}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def publish_diagnostics(self):
        """Publish diagnostics ke topic /diagnostics untuk monitoring."""
        try:
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "multicam_segmentation"
            status.hardware_id = f"huskybot_segmentation_{os.getpid()}"
            active_cameras = sum(1 for img in self.images if img is not None)
            if active_cameras == self.cam_count:
                status.level = DiagnosticStatus.OK
                status.message = "All cameras active"
            elif active_cameras > 0:
                status.level = DiagnosticStatus.WARN
                status.message = f"Partial cameras active: {active_cameras}/{self.cam_count}"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "No cameras active"
            for i, topic in enumerate(self.camera_topics):
                key = KeyValue()
                key.key = f"camera_{i}"
                key.value = f"Active: {self.frame_count[i]} frames" if self.images[i] is not None else "Inactive"
                status.values.append(key)
            key = KeyValue()
            key.key = "fps"
            key.value = f"{self.fps:.2f}"
            status.values.append(key)
            key = KeyValue()
            key.key = "model"
            key.value = os.path.basename(self.model_path)
            status.values.append(key)
            key = KeyValue()
            key.key = "device"
            key.value = self.device
            status.values.append(key)
            key = KeyValue()
            key.key = "uptime"
            uptime = time.time() - self._start_time if hasattr(self, '_start_time') else 0
            key.value = f"{uptime:.1f} sec"
            status.values.append(key)
            diag_msg.status.append(status)
            self.diag_publisher.publish(diag_msg)
        except Exception as e:
            self.get_logger().error(f"Error publish_diagnostics: {e}")
            log_to_file(f"Error publish_diagnostics: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def cleanup(self):
        """Pembersihan resource saat node shutdown."""
        try:
            self.get_logger().info("Melakukan cleanup node...")
            log_to_file("Melakukan cleanup node...", log_file=self.log_file_path)
            self.is_running = False
            with self.image_lock:
                self.images = [None] * self.cam_count
            try:
                cv2.destroyAllWindows()
            except:
                pass
            self.model = None
            log_to_file(f"Node shutdown complete pada {datetime.now().isoformat()}", log_file=self.log_file_path)
        except Exception as e:
            self.get_logger().error(f"Error saat cleanup: {e}")
            log_to_file(f"Error saat cleanup: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

def main(args=None):
    """Fungsi utama untuk menjalankan node multicam_segmentation."""
    shutdown_called = False
    start_time = time.time()
    try:
        rclpy.init(args=args)
        try:
            node = MultiCamSegmentationNode()
            node._start_time = start_time
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            try:
                node.get_logger().info("MultiCam Segmentation Node running...")
                log_to_file("MultiCam Segmentation Node running...", log_file=node.log_file_path)
                executor.spin()
            except KeyboardInterrupt:
                node.get_logger().info("Keyboard interrupt diterima, shutting down...")
                log_to_file("Keyboard interrupt diterima, shutting down...", log_file=node.log_file_path)
            except Exception as e:
                node.get_logger().error(f"Error di main executor: {str(e)}")
                log_to_file(f"Error di main executor: {str(e)}\n{traceback.format_exc()}", level='ERROR', log_file=node.log_file_path)
            finally:
                node.cleanup()
                if not shutdown_called:
                    shutdown_called = True
                    node.destroy_node()
                    rclpy.shutdown()
        except Exception as e:
            if rclpy.ok():
                print(f"Error saat memulai node: {str(e)}")
                print(traceback.format_exc())
                if not shutdown_called:
                    shutdown_called = True
                    rclpy.shutdown()
            sys.exit(1)
    except Exception as e:
        print(f"Error fatal: {str(e)}")
        print(traceback.format_exc())
        sys.exit(1)

if __name__ == '__main__':
    main()