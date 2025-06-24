#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_segmentation/huskybot_segmentation/multicam_segmentation_node.py

import os                                           # Import modul os untuk operasi sistem seperti path dan environment variables
import sys                                          # Import modul sys untuk mengakses interpreter dan fungsi terkait
import time                                         # Import modul time untuk fungsi timing dan delay
import traceback                                    # Import modul traceback untuk log detail error stack
import threading                                    # Import modul threading untuk implementasi thread lock dan konkurensi
from typing import List, Dict, Any, Optional, Union # Import tipe data untuk type hinting (memudahkan debugging)
import numpy as np                                  # Import numpy untuk operasi array/matrix dan manipulasi citra
import cv2                                          # Import OpenCV untuk pengolahan gambar
from datetime import datetime                       # Import datetime untuk timestamp di log

import rclpy                                        # Import library utama ROS2 Python
from rclpy.node import Node                         # Import class Node, base class untuk semua node ROS2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Import QoS untuk konfigurasi topic
from rclpy.parameter import Parameter               # Import Parameter untuk akses ke parameter ROS2 dinamis
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup # Import callback groups untuk threading
from rclpy.executors import MultiThreadedExecutor   # Import MultiThreadedExecutor untuk eksekusi concurrent
from rclpy.exceptions import ParameterNotDeclaredException # Import exception untuk parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType # Import interface untuk deskripsi parameter

from sensor_msgs.msg import Image                   # Import tipe pesan Image dari sensor_msgs
from cv_bridge import CvBridge, CvBridgeError       # Import CvBridge untuk konversi ROS Image <-> OpenCV
from std_msgs.msg import Header                     # Import tipe pesan Header standar
from std_srvs.srv import Trigger                    # Import tipe service Trigger untuk restart/status
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue # Import tipe pesan diagnostik

try:
    from ultralytics import YOLO                    # Import YOLO dari ultralytics untuk segmentasi objek
except ImportError as e:
    print(f"[FATAL] Ultralytics tidak terinstall: {e}")  # Error handling jika package ultralytics tidak ada
    print("Instal dengan: pip install ultralytics")
    sys.exit(1)                                     # Exit dengan error code 1 jika ultralytics tidak ada

from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Import custom message untuk hasil YOLOv12 inference

# --- KONSTANTA GLOBAL ---
DEFAULT_CAMERA_COUNT = 6                            # Jumlah default kamera yang digunakan (hexagonal)
DEFAULT_CAMERA_TOPICS = [                           # Daftar default topic kamera dalam array hexagonal (urutan berdasarkan posisi fisik)
    '/camera_front/image_raw',                      # Kamera depan (0°)
    '/camera_right/image_raw',                      # Kamera kanan (60°)
    '/camera_rear_right/image_raw',                 # Kamera belakang-kanan (120°)
    '/camera_rear/image_raw',                       # Kamera belakang (180°)
    '/camera_left/image_raw',                       # Kamera kiri (240°)
    '/camera_front_left/image_raw'                  # Kamera depan-kiri (300°)
]
DEFAULT_MODEL_PATH = "yolo11x-seg.engine"           # Path default ke model YOLOv12 segmentation (TensorRT engine file)
DEFAULT_CONF_THRESHOLD = 0.5                        # Threshold confidence default untuk hasil segmentasi (0.0-1.0)
DEFAULT_MASK_ENABLE = True                          # Default status mask segmentasi diaktifkan
LOG_DIR = os.path.expanduser("~/huskybot_segmentation_log")  # Direktori untuk menyimpan file log (di home user)

# --- UTILITY FUNCTIONS ---
def log_to_file(msg: str, 
                level: str = 'INFO',                # Level log (INFO, WARN, ERROR, DEBUG)
                log_file: Optional[str] = None) -> None:    # Path file log custom (opsional)
    """Log pesan ke file dengan timestamp dan level tertentu."""
    try:
        # Buat direktori log jika belum ada
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR, exist_ok=True)
            
        # Default log file dengan timestamp untuk memudahkan debug
        if log_file is None:
            current_time = datetime.now()
            log_file = f"{LOG_DIR}/segmentation_{current_time.strftime('%Y%m%d')}.log"
            
        # Format pesan dengan timestamp dan level
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_message = f"[{timestamp}] [{level}] {msg}\n"
        
        # Tulis ke file log (append mode)
        with open(log_file, 'a') as f:
            f.write(log_message)
    except Exception as e:
        # Fallback ke stderr jika gagal menulis ke file
        print(f"[{level}] {msg}\n[ERROR] Gagal menulis ke log file: {e}", file=sys.stderr)

def is_jetson_platform() -> bool:
    """Deteksi apakah kode berjalan di platform Jetson NVIDIA."""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip().lower()
            return 'jetson' in model or 'tegra' in model
    except:
        try:
            # Fallback ke deteksi CUDA Jetson via env vars
            return 'jetson' in os.environ.get('JETSON_TYPE', '').lower()
        except:
            return False


class MultiCamSegmentationNode(Node):
    """Node ROS2 untuk segmentasi multicamera menggunakan YOLOv12."""
    
    def __init__(self):
        """Inisialisasi node MultiCamSegmentationNode."""
        super().__init__('multicam_segmentation')   # Inisialisasi node ROS2 dengan nama 'multicam_segmentation'
        
        # --- Inisialisasi file log ---
        self.log_file_path = None                   # Path file log, akan diset dari parameter
        self._setup_logging()                       # Setup file logging
        
        self.get_logger().info("Menginisialisasi multicam_segmentation node...")  # Log info inisialisasi node
        log_to_file("Menginisialisasi multicam_segmentation node...", log_file=self.log_file_path)
        
        # --- Callback Groups untuk konkurensi ---
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()  # Callback group untuk timer
        self.subscription_callback_group = ReentrantCallbackGroup()  # Callback group untuk subscriptions
        self.service_callback_group = MutuallyExclusiveCallbackGroup()  # Callback group untuk services
        
        # --- Deklarasi Parameter ROS2 ---
        self._declare_parameters()                  # Deklarasi semua parameter node
        
        # --- Thread Lock untuk Konkurensi ---
        self.image_lock = threading.RLock()         # Lock untuk akses ke buffer gambar
        self.process_lock = threading.RLock()       # Lock untuk akses ke proses inferensi
        
        # --- Evaluasi Parameter ---
        self._load_parameters()                     # Load parameter dari ROS2 parameter server

        # --- Setup Buffer Gambar ---
        self.images = [None] * self.cam_count       # Buffer gambar untuk semua kamera (None = belum ada gambar)
        self.last_received = [0.0] * self.cam_count # Timestamp terakhir diterima untuk semua kamera
        self.frame_count = [0] * self.cam_count     # Counter frame untuk semua kamera
        self.model = None                           # Instance model YOLOv12, akan diinisialisasi di _load_model()
        self.bridge = CvBridge()                    # Inisialisasi converter antara ROS2 Image dan OpenCV
        
        # --- Status Diagnostik ---
        self.is_running = True                      # Flag status node berjalan
        self.last_inference_time = 0.0              # Waktu terakhir inferensi (untuk perhitungan FPS)
        self.fps = 0.0                              # FPS saat ini
        self.inference_times = []                   # List waktu inferensi untuk perhitungan rata-rata
        self.max_inference_times = 30               # Jumlah maksimum sampel waktu inferensi
        
        # --- Platform Specific Config ---
        self.is_jetson = is_jetson_platform()       # Cek apakah berjalan di Jetson platform
        if self.is_jetson:
            self.get_logger().info("Terdeteksi platform Jetson - mengoptimalkan untuk hardware NVIDIA...")
            log_to_file("Terdeteksi platform Jetson - mengoptimalkan untuk hardware NVIDIA...", log_file=self.log_file_path)
            # Optimasi untuk Jetson platform akan diterapkan di _load_model
        
        # --- Setup Services ---
        self._setup_services()                      # Setup service node
        
        # --- Load Model ---
        self._load_model()                          # Load model YOLOv12
        
        # --- Setup Publisher ---
        self._setup_publishers()                    # Setup publisher untuk hasil segmentasi
        
        # --- Setup Image Subscribers ---
        self.image_subscribers = []                 # Daftar subscription kamera (untuk clean-up)
        self._setup_image_subscribers()             # Setup subscribers untuk semua kamera
        
        # --- Setup Display Timer ---
        self.timer = self.create_timer(
            1.0 / self.publish_rate,                # Interval timer = 1/rate (dalam detik)
            self.process_and_publish_images,        # Callback function untuk timer
            callback_group=self.timer_callback_group # Callback group untuk konkurensi
        )
        
        # --- Setup Diagnosis Timer ---
        self.diag_timer = self.create_timer(
            1.0,                                    # 1 Hz timer untuk diagnostik
            self.publish_diagnostics,               # Callback untuk diagnostik
            callback_group=self.timer_callback_group # Pakai callback group yang sama
        )
        
        # Set status ready
        self.get_logger().info("Node multicam_segmentation siap!")
        log_to_file("Node multicam_segmentation siap!", log_file=self.log_file_path)

    def _setup_logging(self):
        """Setup file logging path dan direktori."""
        try:
            # Buat parameter untuk path file log
            self.declare_parameter(
                'log_file', 
                f"{LOG_DIR}/segmentation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    description='Path ke file log untuk node ini'
                )
            )
            
            # Get log file path dari parameter
            self.log_file_path = self.get_parameter('log_file').get_parameter_value().string_value
            
            # Buat direktori log jika belum ada
            log_dir = os.path.dirname(self.log_file_path)
            os.makedirs(log_dir, exist_ok=True)
            
            # Informasi log file
            self.get_logger().info(f"Log file: {self.log_file_path}")
        except Exception as e:
            self.get_logger().error(f"Gagal setup logging: {e}")
            self.log_file_path = f"{LOG_DIR}/segmentation_fallback.log"
            traceback.print_exc()
    
    def _declare_parameters(self):
        """Deklarasi semua parameter ROS2 dengan deskripsi dan nilai default."""
        try:
            # Parameter jumlah kamera
            self.declare_parameter(
                'cam_count', 
                DEFAULT_CAMERA_COUNT,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='Jumlah kamera yang digunakan (default: 6 untuk hexagonal)'
                )
            )
            
            # Parameter path model YOLOv12
            self.declare_parameter(
                'model_path', 
                DEFAULT_MODEL_PATH,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    description='Path ke model YOLOv12 (*.pt, *.engine, atau *.onnx)'
                )
            )
            
            # Parameter daftar topic kamera
            self.declare_parameter(
                'camera_topics', 
                DEFAULT_CAMERA_TOPICS,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    description='Daftar topic kamera yang akan di-subscribe (sesuai urutan hexagonal)'
                )
            )
            
            # Parameter confidence threshold
            self.declare_parameter(
                'conf_threshold', 
                DEFAULT_CONF_THRESHOLD,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Threshold confidence untuk segmentasi (0.0-1.0)'
                )
            )
            
            # Parameter publish rate
            self.declare_parameter(
                'publish_rate', 
                5.0,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Frequency publish hasil segmentasi (Hz)'
                )
            )
            
            # Parameter enable mask
            self.declare_parameter(
                'enable_mask', 
                DEFAULT_MASK_ENABLE,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    description='Aktifkan mask segmentasi pada output'
                )
            )
            
            # Parameter enable visualization
            self.declare_parameter(
                'enable_visualization', 
                True,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    description='Aktifkan visualisasi OpenCV'
                )
            )

            # Parameter filter class
            self.declare_parameter(
                'class_filter', 
                [],
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    description='Filter class yang akan dideteksi (kosong = semua class)'
                )
            )
            
            # Parameter set device
            self.declare_parameter(
                'device', 
                '0' if self.is_jetson else 'cpu',
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    description='Device untuk inferensi (0 = GPU pertama, cpu = CPU)'
                )
            )

        except Exception as e:
            self.get_logger().error(f"Error saat deklarasi parameter: {e}")
            log_to_file(f"Error saat deklarasi parameter: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _load_parameters(self):
        """Load semua parameter dari ROS2 parameter server."""
        try:
            # Get semua parameter
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
                self.get_logger().warn(f"Parameter cam_count ({self.cam_count}) tidak valid, menggunakan default: {DEFAULT_CAMERA_COUNT}")
                log_to_file(f"Parameter cam_count ({self.cam_count}) tidak valid, menggunakan default: {DEFAULT_CAMERA_COUNT}", level='WARN', log_file=self.log_file_path)
                self.cam_count = DEFAULT_CAMERA_COUNT
                
            # Validasi daftar topic
            if len(self.camera_topics) < self.cam_count:
                self.get_logger().warn(f"Jumlah topic kamera ({len(self.camera_topics)}) kurang dari cam_count ({self.cam_count}), menggunakan default untuk yang kurang")
                log_to_file(f"Jumlah topic kamera ({len(self.camera_topics)}) kurang dari cam_count ({self.cam_count}), menggunakan default untuk yang kurang", level='WARN', log_file=self.log_file_path)
                # Tambahkan default topic jika kurang
                while len(self.camera_topics) < self.cam_count:
                    idx = len(self.camera_topics)
                    if idx < len(DEFAULT_CAMERA_TOPICS):
                        self.camera_topics.append(DEFAULT_CAMERA_TOPICS[idx])
                    else:
                        self.camera_topics.append(f"/camera_{idx}/image_raw")
                        
            # Trim topic jika lebih dari cam_count
            if len(self.camera_topics) > self.cam_count:
                self.camera_topics = self.camera_topics[:self.cam_count]
                
            # Validasi confidence threshold
            if self.conf_threshold < 0.0 or self.conf_threshold > 1.0:
                self.get_logger().warn(f"Parameter conf_threshold ({self.conf_threshold}) tidak valid, menggunakan default: {DEFAULT_CONF_THRESHOLD}")
                log_to_file(f"Parameter conf_threshold ({self.conf_threshold}) tidak valid, menggunakan default: {DEFAULT_CONF_THRESHOLD}", level='WARN', log_file=self.log_file_path)
                self.conf_threshold = DEFAULT_CONF_THRESHOLD
                
            # Validasi publish rate
            if self.publish_rate <= 0.0:
                self.get_logger().warn(f"Parameter publish_rate ({self.publish_rate}) tidak valid, menggunakan default: 5.0")
                log_to_file(f"Parameter publish_rate ({self.publish_rate}) tidak valid, menggunakan default: 5.0", level='WARN', log_file=self.log_file_path)
                self.publish_rate = 5.0
                
            # Info parameter
            self.get_logger().info(f"Parameter loaded: cam_count={self.cam_count}, model_path={self.model_path}, conf_threshold={self.conf_threshold}")
            self.get_logger().info(f"Camera topics: {self.camera_topics}")
            log_to_file(f"Parameter loaded: cam_count={self.cam_count}, model_path={self.model_path}, conf_threshold={self.conf_threshold}", log_file=self.log_file_path)
            log_to_file(f"Camera topics: {self.camera_topics}", log_file=self.log_file_path)

        except Exception as e:
            self.get_logger().error(f"Error saat load parameter: {e}")
            log_to_file(f"Error saat load parameter: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise
    
    def _setup_services(self):
        """Setup services untuk node ini."""
        try:
            # Service untuk restart model
            self.restart_service = self.create_service(
                Trigger,                            # Tipe service
                'restart_model',                    # Nama service
                self.handle_restart_service,        # Callback handler
                callback_group=self.service_callback_group  # Callback group untuk konkurensi
            )
            
            # Service untuk get status
            self.status_service = self.create_service(
                Trigger,                            # Tipe service
                'get_status',                       # Nama service
                self.handle_status_service,         # Callback handler
                callback_group=self.service_callback_group  # Callback group untuk konkurensi
            )
            
            self.get_logger().info(f"Service registered: restart_model, get_status")
            log_to_file(f"Service registered: restart_model, get_status", log_file=self.log_file_path)
            
        except Exception as e:
            self.get_logger().error(f"Error saat setup services: {e}")
            log_to_file(f"Error saat setup services: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
    
    def handle_restart_service(self, request, response):
        """Handler untuk service restart_model."""
        try:
            self.get_logger().info("Menerima request restart model...")
            log_to_file("Menerima request restart model...", log_file=self.log_file_path)
            
            # Reload model
            self._load_model()
            
            response.success = True
            response.message = f"Model segmentasi berhasil di-restart pada {datetime.now().isoformat()}"
            
        except Exception as e:
            self.get_logger().error(f"Error saat restart model: {e}")
            log_to_file(f"Error saat restart model: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            response.success = False
            response.message = f"Gagal restart model: {str(e)}"
            
        return response
    
    def handle_status_service(self, request, response):
        """Handler untuk service get_status."""
        try:
            # Hitung berapa kamera yang aktif
            active_cameras = sum(1 for img in self.images if img is not None)
            
            # Status pesan
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
            self.get_logger().error(f"Error saat get status: {e}")
            log_to_file(f"Error saat get status: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            response.success = False
            response.message = f"Gagal get status: {str(e)}"
            
        return response

    def _load_model(self):
        """Load model YOLOv12 untuk segmentasi."""
        try:
            # Validasi path model
            if not os.path.exists(self.model_path):
                error_msg = f"Model file tidak ditemukan: {self.model_path}"
                self.get_logger().error(error_msg)
                log_to_file(error_msg, level='ERROR', log_file=self.log_file_path)
                
                # Cari model di beberapa lokasi alternatif
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
                    raise FileNotFoundError(f"Model tidak ditemukan di semua lokasi yang dicek: {self.model_path}")
            
            self.get_logger().info(f"Loading model dari {self.model_path}...")
            log_to_file(f"Loading model dari {self.model_path}...", log_file=self.log_file_path)
            
            # Cek ekstensi file untuk menentukan tipe model
            model_ext = os.path.splitext(self.model_path)[1].lower()
            
            # Optimasi spesifik berdasarkan tipe model dan platform
            start_time = time.time()
            if model_ext == '.engine':
                # TensorRT engine, khusus untuk Jetson/CUDA
                if not self.is_jetson:
                    self.get_logger().warn("File TensorRT .engine digunakan pada platform non-Jetson. Performa mungkin tidak optimal.")
                    log_to_file("File TensorRT .engine digunakan pada platform non-Jetson. Performa mungkin tidak optimal.", level='WARN', log_file=self.log_file_path)
                
                # Load model dengan TensorRT backend
                self.model = YOLO(self.model_path, task="segment")
                
            elif model_ext == '.onnx':
                # ONNX model, bisa digunakan di Jetson atau platform lain
                if self.is_jetson:
                    # Untuk Jetson, gunakan TensorRT sebagai provider untuk ONNX
                    self.model = YOLO(self.model_path, task="segment")
                else:
                    # Untuk non-Jetson, gunakan default provider (CPU/CUDA)
                    self.model = YOLO(self.model_path, task="segment")
                    
            elif model_ext == '.pt':
                # PyTorch model, perlu optimasi berbeda
                if self.is_jetson:
                    # Di Jetson, lebih baik export ke ONNX/TensorRT
                    self.get_logger().warn("PyTorch model (.pt) digunakan pada Jetson. Sarankan export ke ONNX/TensorRT untuk performa optimal.")
                    log_to_file("PyTorch model (.pt) digunakan pada Jetson. Sarankan export ke ONNX/TensorRT untuk performa optimal.", level='WARN', log_file=self.log_file_path)
                
                # Load model PyTorch
                self.model = YOLO(self.model_path, task="segment")
            else:
                # Format tidak dikenali
                self.get_logger().warn(f"Format model tidak dikenali: {model_ext}, menggunakan default loader")
                log_to_file(f"Format model tidak dikenali: {model_ext}, menggunakan default loader", level='WARN', log_file=self.log_file_path)
                self.model = YOLO(self.model_path, task="segment")
            
            # Ukur waktu load
            load_time = time.time() - start_time
            self.get_logger().info(f"Model berhasil dimuat dalam {load_time:.2f} detik")
            log_to_file(f"Model berhasil dimuat dalam {load_time:.2f} detik", log_file=self.log_file_path)
            
            # Simpan metadata model untuk diagnostik
            if hasattr(self.model, 'names'):
                self.class_names = self.model.names
                self.get_logger().info(f"Kelas model: {self.class_names}")
                log_to_file(f"Kelas model: {self.class_names}", log_file=self.log_file_path)
            else:
                self.class_names = {}
                self.get_logger().warn("Tidak dapat membaca kelas dari model")
                log_to_file("Tidak dapat membaca kelas dari model", level='WARN', log_file=self.log_file_path)
                
        except Exception as e:
            self.get_logger().error(f"Error fatal saat loading model: {e}")
            log_to_file(f"Error fatal saat loading model: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _setup_publishers(self):
        """Setup publisher untuk hasil segmentasi dan diagnostik."""
        try:
            # Set QoS profile untuk image streaming yang reliable
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
            
            # Publisher untuk hasil segmentasi
            self.publisher = self.create_publisher(
                Yolov12Inference,                   # Tipe pesan custom
                '/detection',                       # Topic name
                qos_profile                         # QoS profile
            )
            
            # Publisher diagnostik
            self.diag_publisher = self.create_publisher(
                DiagnosticArray,                    # Tipe pesan diagnostik
                '/diagnostics',                     # Topic diagnostik standar
                10                                  # QoS depth
            )
            
            self.get_logger().info("Publisher registered: /detection (Yolov12Inference), /diagnostics")
            log_to_file("Publisher registered: /detection (Yolov12Inference), /diagnostics", log_file=self.log_file_path)
            
        except Exception as e:
            self.get_logger().error(f"Error saat setup publishers: {e}")
            log_to_file(f"Error saat setup publishers: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def _setup_image_subscribers(self):
        """Setup subscribers untuk semua kamera."""
        try:
            # Best practice QoS untuk streaming video dalam ROS2
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, # Reliable delivery
                history=QoSHistoryPolicy.KEEP_LAST,       # Keep only latest messages
                depth=1                                   # Buffer depth (small for real-time)
            )
            
            # Buat subscriber untuk setiap kamera
            for i, topic in enumerate(self.camera_topics):
                try:
                    # Create subscription dengan timeout dan metadata
                    subscription = self.create_subscription(
                        Image,                           # Tipe pesan ROS2
                        topic,                           # Topic kamera
                        lambda msg, idx=i: self.image_callback(msg, idx),  # Callback dengan idx yang dipertahankan
                        qos,                             # QoS profile
                        callback_group=self.subscription_callback_group  # Callback group untuk konkurensi
                    )
                    
                    # Tambahkan ke list untuk clean-up
                    self.image_subscribers.append((i, topic, subscription))
                    
                    self.get_logger().info(f"Subscribed ke kamera #{i}: {topic}")
                    log_to_file(f"Subscribed ke kamera #{i}: {topic}", log_file=self.log_file_path)
                    
                except Exception as e:
                    self.get_logger().error(f"Error subscribe ke topic kamera {topic}: {e}")
                    log_to_file(f"Error subscribe ke topic kamera {topic}: {e}", level='ERROR', log_file=self.log_file_path)
            
        except Exception as e:
            self.get_logger().error(f"Error saat setup image subscribers: {e}")
            log_to_file(f"Error saat setup image subscribers: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            raise

    def image_callback(self, msg, idx):
        """Callback untuk image subscriber - menerima gambar dari kamera."""
        try:
            # Cek index valid
            if idx < 0 or idx >= self.cam_count:
                self.get_logger().warn(f"Index kamera tidak valid: {idx}, max={self.cam_count-1}")
                return
            
            # Konversi Image ROS2 ke OpenCV dengan thread safety
            with self.image_lock:
                # Coba konversi image
                try:
                    self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    
                    # Update timestamp dan frame count untuk monitoring
                    self.last_received[idx] = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
                    self.frame_count[idx] += 1
                    
                    if self.frame_count[idx] % 30 == 0:  # Log setiap 30 frame
                        self.get_logger().debug(f"Received frame #{self.frame_count[idx]} from camera #{idx}")
                    
                except CvBridgeError as e:
                    self.get_logger().warn(f"Error konversi image dari kamera #{idx}: {e}")
                    log_to_file(f"Error konversi image dari kamera #{idx}: {e}", level='WARN', log_file=self.log_file_path)
                    
        except Exception as e:
            self.get_logger().error(f"Error di image_callback untuk kamera #{idx}: {e}")
            log_to_file(f"Error di image_callback untuk kamera #{idx}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def process_and_publish_images(self):
        """Callback untuk timer - proses semua gambar dan publish hasil segmentasi."""
        # Skip jika masih dalam proses (lock tidak tersedia)
        if not self.process_lock.acquire(blocking=False):
            return
        
        try:
            # Thread-safe copy gambar
            images_copy = None
            with self.image_lock:
                # Copy all images atomically
                images_copy = [img.copy() if img is not None else None for img in self.images]
            
            # Skip jika tidak ada image (semua None)
            if all(img is None for img in images_copy):
                available = sum(1 for img in images_copy if img is not None)
                if available == 0:
                    # Hanya log setiap 20 panggilan untuk mengurangi spam log
                    if hasattr(self, '_log_counter'):
                        self._log_counter += 1
                        if self._log_counter >= 20:
                            self.get_logger().info(f"Menunggu feed kamera ({available}/{self.cam_count} tersedia)")
                            self._log_counter = 0
                    else:
                        self._log_counter = 0
                return
            
            # Mulai penghitungan waktu inferensi
            start_time = time.time()
            
            # Proses setiap gambar yang tersedia
            annotated_images = []
            for idx, img in enumerate(images_copy):
                if img is None:  # Skip image yang belum tersedia
                    annotated_images.append(None)
                    continue
                    
                try:
                    # Run inference YOLOv12 segmentasi
                    results = self.model(
                        img, 
                        verbose=False,
                        conf=self.conf_threshold,  # Confidence threshold
                        device=self.device,       # Device inference
                        classes=None if not self.class_filter else [int(c) if c.isdigit() else c for c in self.class_filter]  # Filter class
                    )
                    
                    # Buat annotated image
                    annotated = results[0].plot()  # Plot hasil segmentasi
                    annotated_images.append(annotated)
                    
                    # Publish hasil segmentasi untuk kamera ini
                    self.publish_results(results, f"Camera_{idx+1}")
                    
                    # Log info mask jika ada dan dalam debug level
                    if self.enable_mask and hasattr(results[0], "masks") and results[0].masks is not None:
                        # Count masks untuk log
                        mask_count = len(results[0].masks)
                        if mask_count > 0:
                            self.get_logger().debug(f"[Segmentasi] Kamera #{idx+1}: {mask_count} masks")
                    
                except Exception as e:
                    self.get_logger().error(f"Error segmentasi YOLOv12 kamera #{idx+1}: {e}")
                    log_to_file(f"Error segmentasi YOLOv12 kamera #{idx+1}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
                    # Fallback: image kosong seukuran frame asli
                    if img is not None:
                        annotated_images.append(np.zeros_like(img))
                    else:
                        annotated_images.append(None)
            
            # Hitung FPS untuk monitoring
            inference_time = time.time() - start_time
            self.inference_times.append(inference_time)
            
            # Keep history terbatas
            if len(self.inference_times) > self.max_inference_times:
                self.inference_times.pop(0)
                
            # Update FPS moving average
            if self.inference_times:
                avg_time = sum(self.inference_times) / len(self.inference_times)
                self.fps = 1.0 / avg_time if avg_time > 0 else 0.0
            
            # Visualisasikan hasil jika diaktifkan
            if self.enable_visualization:
                self.visualize_results(annotated_images)
                
        except Exception as e:
            self.get_logger().error(f"Error di process_and_publish_images: {e}")
            log_to_file(f"Error di process_and_publish_images: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
        finally:
            # Selalu release lock di finally
            self.process_lock.release()

    def visualize_results(self, annotated_images):
        """Visualisasikan hasil segmentasi multi-kamera."""
        try:
            # Filter out None images
            valid_images = [img for img in annotated_images if img is not None]
            
            if not valid_images:
                return  # No valid images to visualize
                
            # Set target height untuk visualisasi
            target_height = 240  # Optimal height untuk display multi-kamera
            
            # Resize semua gambar ke target height
            resized_images = []
            for image in valid_images:
                h, w = image.shape[:2]
                scale = target_height / h if h > target_height else 1
                new_w = int(w * scale)
                resized_image = cv2.resize(image, (new_w, target_height))
                resized_images.append(resized_image)
                
            # Tambahkan border antar gambar
            border_thickness = 5  # Ketebalan border
            bordered_images = []
            
            for idx, img in enumerate(resized_images):
                bordered_images.append(img)
                if idx < len(resized_images) - 1:
                    # Tambahkan border hitam antar gambar
                    border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                    bordered_images.append(border)
                    
            # Gabungkan semua gambar secara horizontal
            try:
                combined_image = cv2.hconcat(bordered_images)
                
                # Tambahkan info overlay
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(combined_image, f"FPS: {self.fps:.1f}", (10, 30), font, 0.7, (0, 255, 0), 2)
                cv2.putText(combined_image, f"Model: {os.path.basename(self.model_path)}", (10, 60), font, 0.7, (0, 255, 0), 2)
                
                # Tampilkan visualisasi
                cv2.imshow("MultiCam YOLOv12 Segmentation", combined_image)
                cv2.waitKey(1)  # Update display, tidak block
                
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
            # Buat pesan Yolov12Inference
            msg = Yolov12Inference()
            
            # Set header dan metadata
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  # ROS2 timestamp saat ini
            msg.header.frame_id = camera_name  # Gunakan nama kamera sebagai frame_id
            
            # Set fields metadata tambahan
            msg.camera_name = camera_name  # Nama kamera
            msg.frame_type = "raw"  # Tipe frame (raw)
            msg.task = "segment"  # Task YOLOv12 (segmentation)
            msg.note = ""  # Note (kosong)
            
            # Init list deteksi
            msg.yolov12_inference = []
            
            # Jika tidak ada results, return pesan kosong
            if not results or not hasattr(results, "__len__") or len(results) == 0:
                self.publisher.publish(msg)
                return
            
            # Process results[0].boxes (deteksi)
            for box in results[0].boxes:
                try:
                    # Buat instance hasil deteksi
                    det = InferenceResult()
                    
                    # Extract data dari box YOLOv12 dan convert ke format pesan ROS2
                    # Class label
                    cls_idx = box.cls.item() if hasattr(box.cls, "item") else int(box.cls)
                    if self.class_names and cls_idx in self.class_names:
                        det.class_name = str(self.class_names[cls_idx])
                    else:
                        det.class_name = str(cls_idx)
                    
                    # Confidence score
                    det.confidence = float(box.conf.item()) if hasattr(box.conf, "item") else float(box.conf)
                    
                    # Bounding box coordinates (top, left, bottom, right)
                    if hasattr(box, "xyxy") and box.xyxy is not None:
                        det.top = int(box.xyxy[0][1])
                        det.left = int(box.xyxy[0][0])
                        det.bottom = int(box.xyxy[0][3])
                        det.right = int(box.xyxy[0][2])
                    elif hasattr(box, "xywh") and box.xywh is not None:
                        # Convert xywh to xyxy
                        x, y, w, h = box.xywh[0]
                        det.left = int(x - w/2)
                        det.top = int(y - h/2)
                        det.right = int(x + w/2)
                        det.bottom = int(y + h/2)
                    else:
                        # Default/fallback
                        det.top = 0
                        det.left = 0
                        det.bottom = 0
                        det.right = 0
                        self.get_logger().warn(f"Box format tidak dikenali untuk {camera_name}")
                    
                    # Track ID jika ada tracking
                    if hasattr(box, "id") and box.id is not None:
                        det.track_id = int(box.id.item()) if hasattr(box.id, "item") else int(box.id)
                    else:
                        det.track_id = -1  # Default track ID (no tracking)
                        
                    # OBB angle (untuk oriented bounding box)
                    det.obb_angle = 0  # Default OBB angle (dalam radian)
                    
                    # Mask indices untuk segmentasi
                    if self.enable_mask and hasattr(results[0], "masks") and results[0].masks is not None and len(results[0].masks) > 0:
                        try:
                            # Cari mask yang sesuai dengan box ini
                            # Ini berdasarkan asumsi urutan masks sama dengan urutan boxes
                            mask_idx = results[0].boxes.tolist().index(box.tolist()) if hasattr(results[0].boxes, "tolist") and hasattr(box, "tolist") else -1
                            
                            if mask_idx >= 0 and mask_idx < len(results[0].masks):
                                mask = results[0].masks[mask_idx]
                                # Convert mask ke format array 1D untuk ROS2 message
                                if hasattr(mask, "data"):
                                    # Flatten dan convert ke list
                                    det.mask_indices = mask.data.flatten().tolist()
                                else:
                                    det.mask_indices = []
                            else:
                                det.mask_indices = []
                        except Exception as e:
                            self.get_logger().warn(f"Error saat extract mask untuk {camera_name}: {e}")
                            det.mask_indices = []
                    else:
                        det.mask_indices = []
                    
                    # Tambahkan ke list hasil
                    msg.yolov12_inference.append(det)
                    
                except Exception as e:
                    self.get_logger().error(f"Error proses box untuk {camera_name}: {e}")
                    log_to_file(f"Error proses box untuk {camera_name}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)
            
            # Publish pesan
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publish_results untuk {camera_name}: {e}")
            log_to_file(f"Error publish_results untuk {camera_name}: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def publish_diagnostics(self):
        """Publish diagnostics ke topic /diagnostics untuk monitoring."""
        try:
            # Buat pesan diagnostik
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Status utama node
            status = DiagnosticStatus()
            status.name = "multicam_segmentation"
            status.hardware_id = f"huskybot_segmentation_{os.getpid()}"
            
            # Determine status level
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
                
            # Tambah detail kamera
            for i, topic in enumerate(self.camera_topics):
                key = KeyValue()
                key.key = f"camera_{i}"
                if self.images[i] is not None:
                    key.value = f"Active: {self.frame_count[i]} frames"
                else:
                    key.value = "Inactive"
                status.values.append(key)
                
            # Tambahkan info performa
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
            
            # Tambahkan status ke array
            diag_msg.status.append(status)
            
            # Publish
            self.diag_publisher.publish(diag_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publish_diagnostics: {e}")
            log_to_file(f"Error publish_diagnostics: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)

    def cleanup(self):
        """Pembersihan resource saat node shutdown."""
        try:
            self.get_logger().info("Melakukan cleanup node...")
            log_to_file("Melakukan cleanup node...", log_file=self.log_file_path)
            
            # Set flag running = False
            self.is_running = False
            
            # Hapus semua image buffer
            with self.image_lock:
                self.images = [None] * self.cam_count
            
            # Tutup semua window OpenCV
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            # Unload model jika perlu
            self.model = None
            
            # Final log
            log_to_file(f"Node shutdown complete pada {datetime.now().isoformat()}", log_file=self.log_file_path)
            
        except Exception as e:
            self.get_logger().error(f"Error saat cleanup: {e}")
            log_to_file(f"Error saat cleanup: {e}\n{traceback.format_exc()}", level='ERROR', log_file=self.log_file_path)


def main(args=None):
    """Fungsi utama untuk menjalankan node multicam_segmentation."""
    # Flag untuk hindarkan double-shutdown
    shutdown_called = False
    
    # Set start time global
    start_time = time.time()
    
    try:
        # Inisialisasi ROS2
        rclpy.init(args=args)
        
        try:
            # Buat node
            node = MultiCamSegmentationNode()
            node._start_time = start_time  # Simpan waktu start untuk uptime
            
            # Buat executor multi-threaded untuk konkurensi
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            
            try:
                # Info siap
                node.get_logger().info("MultiCam Segmentation Node running...")
                log_to_file("MultiCam Segmentation Node running...", log_file=node.log_file_path)
                
                # Spin sampai interupsi
                executor.spin()
                
            except KeyboardInterrupt:
                # Handler Ctrl+C
                node.get_logger().info("Keyboard interrupt diterima, shutting down...")
                log_to_file("Keyboard interrupt diterima, shutting down...", log_file=node.log_file_path)
                
            except Exception as e:
                # Error umum di executor
                node.get_logger().error(f"Error di main executor: {str(e)}")
                log_to_file(f"Error di main executor: {str(e)}\n{traceback.format_exc()}", level='ERROR', log_file=node.log_file_path)
                
            finally:
                # Cleanup semua resource
                node.cleanup()
                
                # Destroy node
                if not shutdown_called:
                    shutdown_called = True
                    node.destroy_node()
                    rclpy.shutdown()
                    
        except Exception as e:
            # Error saat inisialisasi node
            if rclpy.ok():
                print(f"Error saat memulai node: {str(e)}")
                print(traceback.format_exc())
                if not shutdown_called:
                    shutdown_called = True
                    rclpy.shutdown()
                    
            sys.exit(1)
            
    except Exception as e:
        # Error saat inisialisasi ROS
        print(f"Error fatal: {str(e)}")
        print(traceback.format_exc())
        sys.exit(1)


if __name__ == '__main__':
    main()