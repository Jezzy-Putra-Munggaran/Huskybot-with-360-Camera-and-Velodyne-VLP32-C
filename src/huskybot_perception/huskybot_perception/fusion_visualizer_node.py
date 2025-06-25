#!/usr/bin/env python3  # Shebang untuk spesifikasi interpreter Python3 (wajib untuk ROS2 executables)
# -*- coding: utf-8 -*-  # Encoding declaration untuk Unicode support

"""
Fusion Visualizer Node untuk Huskybot

Node ini memvisualisasikan hasil fusion antara deteksi YOLOv12 dan data 3D LiDAR 
dengan menampilkan bounding box, kelas, confidence score, jarak, dan koordinat 3D.
Node ini mendukung visualisasi untuk simulasi Gazebo dan robot real Huskybot
(Clearpath Husky A200 + Jetson AGX ORIN + 6x Arducam IMX477 + Velodyne VLP32-C).

Author: Jezzy Putra Munggaran
License: Apache-2.0
"""

import os  # Import OS untuk akses file system dan environment variables
import time  # Import time untuk timestamp dan delay
import threading  # Import threading untuk thread-safety di callback multi-topic
import logging  # Import logging untuk pencatatan error yang lebih terstruktur
import traceback  # Import traceback untuk stack trace detail saat exception
from typing import Dict, List, Optional, Tuple, Union  # Import type hints untuk dokumentasi tipe data
from datetime import datetime  # Import datetime untuk timestamp yang human-readable

import numpy as np  # Import NumPy untuk operasi array/matriks untuk visualisasi
import cv2  # Import OpenCV untuk visualisasi dan image processing
import yaml  # Import YAML parser untuk file konfigurasi

import rclpy  # Import library utama ROS2 Python
from rclpy.node import Node  # Import base class Node dari ROS2
from rclpy.exceptions import ParameterNotDeclaredException  # Import exception untuk parameter tidak dideklarasikan
from rclpy.parameter import Parameter  # Import class Parameter untuk validasi parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # Import QoS untuk konfigurasi subscription/publisher

from cv_bridge import CvBridge, CvBridgeError  # Import converter ROS Image <-> OpenCV
from sensor_msgs.msg import Image  # Import message type Image untuk data kamera
from yolov12_msgs.msg import Yolov12Inference, Yolov12Instance  # Import custom message untuk hasil fusion/deteksi

# Konstanta untuk visualisasi (class colors, line thickness, dll)
# Menggunakan warna berbeda untuk setiap class (person=red, car=blue, dll)
CLASS_COLORS = {
    "person": (0, 0, 255),     # Merah untuk person (BGR format)
    "bicycle": (255, 0, 0),    # Biru untuk bicycle
    "car": (0, 255, 0),        # Hijau untuk car
    "motorcycle": (255, 255, 0), # Cyan untuk motorcycle
    "bus": (255, 0, 255),      # Magenta untuk bus
    "truck": (0, 255, 255),    # Kuning untuk truck
    "traffic light": (128, 0, 128),  # Ungu untuk traffic light
    "stop sign": (255, 128, 0),  # Orange untuk stop sign
    "default": (0, 255, 0)     # Default hijau jika class tidak ada di dict
}

# Path file log untuk error tracking
LOG_DIR = os.path.expanduser("~/huskybot_logs")  # Default log dir di home user
if not os.path.exists(LOG_DIR):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)  # Buat directory log jika belum ada
    except Exception as e:
        LOG_DIR = "/tmp"  # Fallback ke /tmp jika tidak bisa buat di home
        print(f"Warning: Could not create log directory in home, using /tmp: {e}")  # Print warning

# Setup logger untuk pencatatan error
logging.basicConfig(
    filename=os.path.join(LOG_DIR, "fusion_visualizer.log"),  # Path file log
    level=logging.INFO,  # Level logging (INFO captures warnings and errors)
    format='%(asctime)s - %(levelname)s - %(message)s',  # Format timestamp - level - pesan
    filemode='a'  # Mode append untuk menambahkan log ke file yang sudah ada
)

class FusionVisualizerNode(Node):
    """
    Node visualisasi untuk menampilkan hasil 3D fusion dari deteksi YOLOv12 dan data LiDAR.
    
    Node ini men-subscribe ke topic image dari 6 kamera (hexagonal) dan hasil fusion,
    kemudian memvisualisasikan informasi 3D (jarak, koordinat) pada gambar kamera.
    """
    
    def __init__(self) -> None:
        """
        Inisialisasi FusionVisualizerNode.
        
        Setup parameter, subscriber, publisher, dan configuration untuk visualisasi.
        Implementasi FULL OOP dengan error handling komprehensif.
        """
        super().__init__('fusion_visualizer_node')  # Inisialisasi node dengan nama fusion_visualizer_node
        
        # Setup logger dengan tambahan node info
        self.get_logger().info("Initializing Fusion Visualizer Node")  # Log startup node
        logging.info("Initializing Fusion Visualizer Node")  # Log ke file juga
        
        self.cv_bridge = CvBridge()  # Inisialisasi CV Bridge untuk konversi ROS Image <-> OpenCV
        self.lock = threading.RLock()  # Mutex lock untuk thread safety pada akses data antar callback
        
        # Declare parameter dengan default value dan descriptive comments
        self.declare_parameters(
            namespace='',
            parameters=[
                ('show_bounding_box', Parameter.Type.BOOL, True),  # Parameter untuk toggle bounding box
                ('show_class', Parameter.Type.BOOL, True),         # Parameter untuk toggle class name
                ('show_confidence', Parameter.Type.BOOL, True),    # Parameter untuk toggle confidence score
                ('show_distance', Parameter.Type.BOOL, True),      # Parameter untuk toggle distance
                ('show_coordinates', Parameter.Type.BOOL, True),   # Parameter untuk toggle 3D coordinates
                ('text_scale', Parameter.Type.DOUBLE, 0.7),        # Parameter untuk ukuran teks
                ('text_thickness', Parameter.Type.INTEGER, 2),     # Parameter untuk ketebalan teks
                ('box_thickness', Parameter.Type.INTEGER, 2),      # Parameter untuk ketebalan box
                ('fusion_topic', Parameter.Type.STRING, '/fusion/objects3d'),  # Parameter untuk topic fusion
                ('display_window', Parameter.Type.BOOL, True),     # Parameter untuk toggle display window (headless support)
                ('save_annotated_images', Parameter.Type.BOOL, False),  # Parameter untuk simpan hasil ke file
                ('save_path', Parameter.Type.STRING, os.path.expanduser('~/huskybot_annotated')),  # Path untuk simpan image
                ('custom_camera_topics', Parameter.Type.STRING_ARRAY, []),  # Optional custom list kamera topics
                ('display_resolution_width', Parameter.Type.INTEGER, 640),  # Resolusi display lebar
                ('display_resolution_height', Parameter.Type.INTEGER, 480), # Resolusi display tinggi
                ('display_fps', Parameter.Type.INTEGER, 10),       # FPS untuk display window
                ('auto_resize_images', Parameter.Type.BOOL, True), # Auto-resize images untuk display
                ('verbose_logging', Parameter.Type.BOOL, False),   # Extended logging untuk debug
            ]
        )
        
        # Get dan validasi parameter
        try:
            self.show_bbox = self.get_parameter('show_bounding_box').value  # Ambil parameter show_bounding_box
            self.show_class = self.get_parameter('show_class').value  # Ambil parameter show_class
            self.show_confidence = self.get_parameter('show_confidence').value  # Ambil parameter show_confidence
            self.show_distance = self.get_parameter('show_distance').value  # Ambil parameter show_distance
            self.show_coordinates = self.get_parameter('show_coordinates').value  # Ambil parameter show_coordinates
            self.text_scale = self.get_parameter('text_scale').value  # Ambil parameter text_scale
            self.text_thickness = self.get_parameter('text_thickness').value  # Ambil parameter text_thickness
            self.box_thickness = self.get_parameter('box_thickness').value  # Ambil parameter box_thickness
            self.fusion_topic = self.get_parameter('fusion_topic').value  # Ambil parameter fusion_topic
            self.display_window = self.get_parameter('display_window').value  # Ambil parameter display_window
            self.save_annotated_images = self.get_parameter('save_annotated_images').value  # Ambil parameter save_annotated_images
            self.save_path = self.get_parameter('save_path').value  # Ambil parameter save_path
            self.custom_camera_topics = self.get_parameter('custom_camera_topics').value  # Ambil parameter custom_camera_topics
            self.display_width = self.get_parameter('display_resolution_width').value  # Ambil parameter display_resolution_width
            self.display_height = self.get_parameter('display_resolution_height').value  # Ambil parameter display_resolution_height
            self.display_fps = self.get_parameter('display_fps').value  # Ambil parameter display_fps
            self.auto_resize = self.get_parameter('auto_resize_images').value  # Ambil parameter auto_resize_images
            self.verbose_logging = self.get_parameter('verbose_logging').value  # Ambil parameter verbose_logging
            
            # Validasi parameter yang sensitif
            if self.text_scale <= 0.0:  # Validasi text_scale > 0
                self.get_logger().warning("Invalid text_scale (<=0), resetting to default (0.7)")  # Log warning
                logging.warning("Invalid text_scale (<=0), resetting to default (0.7)")  # Log ke file
                self.text_scale = 0.7  # Reset ke default
                
            if self.text_thickness <= 0:  # Validasi text_thickness > 0
                self.get_logger().warning("Invalid text_thickness (<=0), resetting to default (2)")  # Log warning
                logging.warning("Invalid text_thickness (<=0), resetting to default (2)")  # Log ke file
                self.text_thickness = 2  # Reset ke default
                
            if self.box_thickness <= 0:  # Validasi box_thickness > 0
                self.get_logger().warning("Invalid box_thickness (<=0), resetting to default (2)")  # Log warning
                logging.warning("Invalid box_thickness (<=0), resetting to default (2)")  # Log ke file
                self.box_thickness = 2  # Reset ke default
            
            if self.display_width <= 0 or self.display_height <= 0:  # Validasi resolusi display > 0
                self.get_logger().warning(f"Invalid display resolution ({self.display_width}x{self.display_height}), resetting to default (640x480)")  # Log warning
                logging.warning(f"Invalid display resolution ({self.display_width}x{self.display_height}), resetting to default (640x480)")  # Log ke file
                self.display_width, self.display_height = 640, 480  # Reset ke default
                
            if self.display_fps <= 0:  # Validasi FPS > 0
                self.get_logger().warning(f"Invalid display_fps ({self.display_fps}), resetting to default (10)")  # Log warning
                logging.warning(f"Invalid display_fps ({self.display_fps}), resetting to default (10)")  # Log ke file
                self.display_fps = 10  # Reset ke default
                
        except ParameterNotDeclaredException as e:  # Tangkap error parameter tidak dideklarasikan
            self.get_logger().error(f"Parameter error: {e}")  # Log error
            logging.error(f"Parameter error: {e}")  # Log ke file
            # Tetap gunakan nilai default jika error
            self.show_bbox = True  # Default show_bounding_box
            self.show_class = True  # Default show_class
            self.show_confidence = True  # Default show_confidence
            self.show_distance = True  # Default show_distance
            self.show_coordinates = True  # Default show_coordinates
            self.text_scale = 0.7  # Default text_scale
            self.text_thickness = 2  # Default text_thickness
            self.box_thickness = 2  # Default box_thickness
            self.fusion_topic = '/fusion/objects3d'  # Default fusion_topic
            self.display_window = True  # Default display_window
            self.save_annotated_images = False  # Default save_annotated_images
            self.save_path = os.path.expanduser('~/huskybot_annotated')  # Default save_path
            self.custom_camera_topics = []  # Default custom_camera_topics
            self.display_width = 640  # Default display_resolution_width
            self.display_height = 480  # Default display_resolution_height
            self.display_fps = 10  # Default display_fps
            self.auto_resize = True  # Default auto_resize_images
            self.verbose_logging = False  # Default verbose_logging
        
        # Create save directory jika save_annotated_images=True
        if self.save_annotated_images:  # Jika fitur save image diaktifkan
            try:
                os.makedirs(self.save_path, exist_ok=True)  # Buat directory untuk save dengan exist_ok=True agar tidak error jika sudah ada
                self.get_logger().info(f"Created directory for saving annotated images: {self.save_path}")  # Log info
                logging.info(f"Created directory for saving annotated images: {self.save_path}")  # Log ke file
            except (PermissionError, OSError) as e:  # Tangkap error permission atau OS error
                self.get_logger().error(f"Failed to create directory for saving images: {e}")  # Log error
                logging.error(f"Failed to create directory for saving images: {e}")  # Log ke file
                self.save_annotated_images = False  # Disable fitur save jika folder error
        
        # Setup camera topics berdasarkan parameter atau default
        if self.custom_camera_topics and len(self.custom_camera_topics) > 0:  # Jika ada custom topics
            self.camera_topics = self.custom_camera_topics  # Gunakan custom topics
            self.get_logger().info(f"Using custom camera topics: {self.camera_topics}")  # Log info
        else:  # Jika tidak ada custom topics
            # Default 6 camera topics untuk array hexagonal
            self.camera_topics = [
                '/camera_front/image_raw',  # Camera depan
                '/camera_front_left/image_raw',  # Camera depan kiri
                '/camera_left/image_raw',  # Camera kiri
                '/camera_rear/image_raw',  # Camera belakang
                '/camera_rear_right/image_raw',  # Camera belakang kanan
                '/camera_right/image_raw'  # Camera kanan
            ]
            self.get_logger().info(f"Using default camera topics: {self.camera_topics}")  # Log info
        
        # Dictionary untuk menyimpan subscriber dan image terbaru dari tiap kamera
        self.camera_subs = {}   # Dictionary untuk menyimpan semua subscriber kamera
        self.latest_images = {} # Dictionary untuk menyimpan frame terbaru dari tiap kamera
        self.latest_annotated = {} # Dictionary untuk menyimpan hasil annotasi terbaru
        self.camera_status = {} # Dictionary untuk status sehat tiap kamera
        
        # Setup QoS untuk reliability pada topic realtime
        # Gunakan RELIABLE untuk fusion (tidak boleh packet loss) tapi keep_last(10) saja
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable QoS untuk data penting
            history=QoSHistoryPolicy.KEEP_LAST,  # Simpan message terakhir saja
            depth=10  # Jumlah message yang disimpan
        )
        
        # Gunakan BEST_EFFORT untuk kamera (boleh packet loss, prioritas latency rendah)
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort untuk sensor realtime
            history=QoSHistoryPolicy.KEEP_LAST,  # Simpan message terakhir saja
            depth=1  # Hanya 1 message (frame terbaru)
        )
        
        # Subscribe ke semua topic kamera dengan error handling
        for topic in self.camera_topics:  # Iterasi semua topic kamera
            try:
                # Extract camera name dari topic (misal: /camera_front/image_raw -> camera_front)
                camera_name = topic.split('/')[1]  # Ambil bagian kedua setelah split by '/'
                if not camera_name:  # Error handling untuk topic format tidak valid
                    raise ValueError(f"Invalid camera topic format: {topic}")  # Raise error dengan pesan
                
                # Subscribe ke topic image kamera dengan callback spesifik untuk kamera ini
                # Lambda digunakan agar callback mendapatkan parameter camera_name yang benar
                self.camera_subs[camera_name] = self.create_subscription(
                    Image,                                               # Message type (ROS2 Image)
                    topic,                                               # Nama topic untuk subscribe
                    lambda msg, cam=camera_name: self.image_callback(msg, cam), # Callback dengan closure untuk camera_name
                    qos_profile=self.sensor_qos                          # QoS untuk sensor (best effort)
                )
                
                # Init image buffer dan status kamera
                self.latest_images[camera_name] = None  # Init buffer image dengan None
                self.latest_annotated[camera_name] = None  # Init buffer annotated image dengan None
                self.camera_status[camera_name] = {
                    'last_msg_time': 0.0,  # Timestamp message terakhir
                    'frame_count': 0,  # Counter jumlah frame yang diterima
                    'error_count': 0,  # Counter error yang terjadi
                    'status': 'INITIALIZING'  # Status kamera (INITIALIZING/OK/ERROR)
                }
                
                self.get_logger().info(f"Subscribed to camera topic: {topic}")  # Log info subscribe
            
            except Exception as e:  # Tangkap semua exception saat setup subscription
                self.get_logger().error(f"Failed to subscribe to camera topic {topic}: {e}")  # Log error
                logging.error(f"Failed to subscribe to camera topic {topic}: {e}")  # Log ke file
        
        # Subscribe ke hasil fusion dengan error handling
        try:
            self.fusion_sub = self.create_subscription(
                Yolov12Inference,             # Message type (custom message untuk hasil fusion)
                self.fusion_topic,            # Topic fusion (dari parameter)
                self.fusion_callback,         # Callback untuk data fusion
                qos_profile=self.reliable_qos # QoS reliability untuk fusion data
            )
            self.get_logger().info(f"Subscribed to fusion topic: {self.fusion_topic}")  # Log info subscribe
        except Exception as e:  # Tangkap semua exception saat setup subscription
            self.get_logger().error(f"Failed to subscribe to fusion topic {self.fusion_topic}: {e}")  # Log error
            logging.error(f"Failed to subscribe to fusion topic {self.fusion_topic}: {e}")  # Log ke file
            # Buat dummy subscription untuk fallback (node tetap jalan meski error) 
            self.fusion_sub = self.create_subscription(
                Yolov12Inference,  # Message type
                '/dummy_fusion_topic',  # Dummy topic
                lambda msg: self.get_logger().warning("Using dummy fusion topic, real topic unavailable"),  # Dummy callback
                1  # Minimal QoS depth
            )
            
        # Publishers untuk annotated images dengan error handling
        self.annotated_pubs = {}  # Dictionary untuk menyimpan semua publisher
        for camera_name in self.camera_subs.keys():  # Iterasi semua camera yang sudah disubscribe
            try:
                self.annotated_pubs[camera_name] = self.create_publisher(
                    Image,                    # Message type (ROS2 Image)
                    f'/{camera_name}/annotated',  # Topic untuk publish annotated image
                    10                        # QoS depth (10 message)
                )
                self.get_logger().info(f"Created publisher for annotated images: /{camera_name}/annotated")  # Log info
            except Exception as e:  # Tangkap semua exception saat setup publisher
                self.get_logger().error(f"Failed to create publisher for {camera_name}: {e}")  # Log error
                logging.error(f"Failed to create publisher for {camera_name}: {e}")  # Log ke file
        
        # Timer untuk health check kamera
        self.health_check_timer = self.create_timer(
            5.0,  # Check setiap 5 detik
            self.check_camera_health  # Callback function
        )
        
        # Timer untuk display semua image jika parameter display_window=True
        if self.display_window:  # Jika display window diaktifkan
            try:
                self.display_timer = self.create_timer(
                    1.0 / self.display_fps,  # Update periode berdasarkan FPS
                    self.display_all_cameras  # Callback function
                )
                self.get_logger().info(f"Display timer created, window will be shown at {self.display_fps} FPS")  # Log info
            except Exception as e:  # Tangkap semua exception saat setup timer
                self.get_logger().error(f"Failed to create display timer: {e}")  # Log error
                logging.error(f"Failed to create display timer: {e}")  # Log ke file
                self.display_window = False  # Disable display window jika gagal
        
        # Init variable untuk tracking status
        self.total_fusion_msgs = 0  # Counter total message fusion yang diterima
        self.last_fusion_time = self.get_clock().now()  # Timestamp message terakhir
        self.startup_time = time.time()  # Timestamp node startup
        
        # Performance metrics
        self.fps_stats = {cam: {'frames': 0, 'last_time': time.time(), 'fps': 0.0} for cam in self.camera_subs.keys()}
        self.processing_times = []  # List untuk tracking waktu proses fusion callback
        
        # Init handling untuk OpenCV windows
        if self.display_window:
            try:
                # Named window untuk tiled view
                cv2.namedWindow('Huskybot Fusion Visualizer', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Huskybot Fusion Visualizer', self.display_width, self.display_height)
                self.get_logger().info("OpenCV display window initialized")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize OpenCV window: {e}")
                logging.error(f"Failed to initialize OpenCV window: {e}")
                self.display_window = False  # Disable display window jika gagal
        
        self.get_logger().info("Fusion Visualizer Node initialized successfully")  # Log sukses inisialisasi
        logging.info("Fusion Visualizer Node initialized successfully")  # Log ke file
        
    def check_camera_health(self) -> None:
        """
        Timer callback untuk cek health status semua kamera.
        
        Deteksi kamera yang tidak mengirim data atau mengalami error,
        dan update status untuk diagnostik.
        """
        current_time = time.time()  # Ambil timestamp saat ini
        
        with self.lock:  # Thread-safe access ke shared data
            for camera_name, status in self.camera_status.items():  # Iterasi tiap kamera
                # Cek apakah kamera telah mengirim data dalam 5 detik terakhir
                if current_time - status['last_msg_time'] > 5.0 and status['last_msg_time'] > 0:  # Jika gap > 5 detik dan pernah terima data
                    if status['status'] != 'ERROR':  # Jika status belum error
                        status['status'] = 'ERROR'  # Set status error
                        self.get_logger().warning(f"Camera {camera_name} not sending data for > 5s")  # Log warning
                        logging.warning(f"Camera {camera_name} not sending data for > 5s")  # Log ke file
                
                # Reset status ke OK jika sebelumnya error tapi sekarang sudah dapat data lagi
                elif status['last_msg_time'] > 0 and status['status'] == 'ERROR' and current_time - status['last_msg_time'] < 5.0:  # Jika sudah terima data lagi
                    status['status'] = 'OK'  # Reset status ke OK
                    self.get_logger().info(f"Camera {camera_name} is back online")  # Log info
                    logging.info(f"Camera {camera_name} is back online")  # Log ke file
                    
                # Update status ke OK jika sudah menerima frame dan belum pernah error
                elif status['frame_count'] > 0 and status['status'] == 'INITIALIZING':  # Jika sudah terima frame tapi masih initializing
                    status['status'] = 'OK'  # Set status OK
            
            # Hitung dan update FPS untuk tiap kamera
            for cam, stat in self.fps_stats.items():  # Iterasi tiap kamera
                elapsed = current_time - stat['last_time']  # Hitung elapsed time
                if elapsed > 0:  # Hindari division by zero
                    stat['fps'] = stat['frames'] / elapsed  # Hitung FPS
                    stat['frames'] = 0  # Reset frame counter
                    stat['last_time'] = current_time  # Update timestamp
            
            # Log status camera summary
            active_cameras = sum(1 for status in self.camera_status.values() if status['status'] == 'OK')  # Hitung kamera aktif
            error_cameras = sum(1 for status in self.camera_status.values() if status['status'] == 'ERROR')  # Hitung kamera error
            
            # Log fusion status jika ada pesan dari fusion node
            if self.total_fusion_msgs > 0:  # Jika sudah menerima pesan fusion
                fusion_delay = (self.get_clock().now() - self.last_fusion_time).nanoseconds / 1e9  # Hitung delay dalam detik
                if fusion_delay > 5.0:  # Jika delay > 5 detik
                    self.get_logger().warning(f"No fusion messages received for {fusion_delay:.1f}s")  # Log warning
            
            # Log processing performance metrics
            if self.processing_times:  # Jika sudah ada data processing time
                avg_time = sum(self.processing_times) / len(self.processing_times)  # Hitung rata-rata
                self.get_logger().info(f"Health: {active_cameras}/{len(self.camera_status)} cameras OK, {self.total_fusion_msgs} fusion msgs, Avg process: {avg_time*1000:.1f}ms")  # Log status
                self.processing_times = []  # Reset untuk interval berikutnya
    
    def image_callback(self, msg: Image, camera_name: str) -> None:
        """
        Callback untuk setiap frame dari kamera.
        
        Args:
            msg: Image message dari ROS
            camera_name: Nama kamera yang mengirim image
        """
        # Thread-safety untuk akses data yang dibagi antar callback
        with self.lock:  
            try:
                # Konversi image ROS menjadi format OpenCV
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS -> OpenCV
                
                # Update FPS stats
                self.fps_stats[camera_name]['frames'] += 1  # Increment frame counter
                
                # Resize image ke resolusi display jika ukurannya terlalu besar dan auto_resize=True
                h, w = cv_image.shape[:2]  # Ambil dimensi image
                if self.auto_resize and (w > self.display_width or h > self.display_height):  # Jika perlu resize
                    # Maintain aspect ratio
                    ratio = min(self.display_width / w, self.display_height / h)  # Hitung rasio scaling
                    new_size = (int(w * ratio), int(h * ratio))  # Hitung dimensi baru
                    cv_image = cv2.resize(cv_image, new_size, interpolation=cv2.INTER_AREA)  # Resize image (high quality)
                
                # Update image buffer dan status kamera
                self.latest_images[camera_name] = cv_image  # Simpan frame ke buffer
                self.camera_status[camera_name]['last_msg_time'] = time.time()  # Update timestamp
                self.camera_status[camera_name]['frame_count'] += 1  # Update frame count
                
                if self.verbose_logging and self.camera_status[camera_name]['frame_count'] % 100 == 0:  # Log setiap 100 frame
                    self.get_logger().debug(f"Received {self.camera_status[camera_name]['frame_count']} frames from {camera_name}")  # Log debug
                
            except CvBridgeError as e:  # Error konversi ROS <-> OpenCV
                self.get_logger().error(f'CV Bridge error for camera {camera_name}: {e}')  # Log error
                logging.error(f'CV Bridge error for camera {camera_name}: {e}')  # Log ke file
                self.camera_status[camera_name]['error_count'] += 1  # Update error count
                
            except Exception as e:  # Error umum lainnya
                self.get_logger().error(f'Unexpected error in image callback for camera {camera_name}: {e}\n{traceback.format_exc()}')  # Log error dengan stack trace
                logging.error(f'Unexpected error in image callback for camera {camera_name}: {e}\n{traceback.format_exc()}')  # Log ke file
                self.camera_status[camera_name]['error_count'] += 1  # Update error count
    
    def fusion_callback(self, msg: Yolov12Inference) -> None:
        """
        Callback untuk hasil fusion dari YOLOv12 deteksi dan LiDAR data.
        
        Memproses message Yolov12Inference yang berisi bounding box 2D dan data 3D (jarak, koordinat).
        Menggambar visualisasi pada image kamera yang sesuai dan mempublish hasilnya.
        
        Args:
            msg: Message hasil fusion
        """
        start_time = time.time()  # Track processing time start
        
        self.total_fusion_msgs += 1  # Increment counter pesan fusion
        self.last_fusion_time = self.get_clock().now()  # Update timestamp pesan terakhir
        
        # Thread-safety untuk akses data yang dibagi antar callback
        with self.lock:
            try:
                # Extract camera name dan validasi format
                camera_name = msg.camera_name  # Ambil camera_name dari message
                if not camera_name or camera_name not in self.latest_images:  # Validasi camera_name
                    self.get_logger().warning(f"Camera name '{camera_name}' in fusion message not found in subscribed cameras")  # Log warning
                    logging.warning(f"Camera name '{camera_name}' in fusion message not found in subscribed cameras")  # Log ke file
                    return  # Exit callback
                
                # Skip jika belum ada image dari kamera ini
                if self.latest_images[camera_name] is None:  # Cek apakah sudah ada image
                    return  # Exit callback jika belum ada image
                
                # Get a copy of the image to annotate (jangan modifikasi image asli)
                image = self.latest_images[camera_name].copy()  # Copy image untuk dimodifikasi
                h, w = image.shape[:2]  # Ambil dimensi image
                
                # Draw detections with fusion information
                for det in msg.yolov12_inference:  # Iterasi setiap deteksi
                    try:
                        # Extract dan validasi bounding box coordinates
                        x1, y1, x2, y2 = self._validate_bbox_coordinates(det, w, h)  # Validasi koordinat bbox
                        
                        # Tentukan warna berdasarkan object class
                        color = self._get_class_color(det.class_name)  # Ambil warna berdasarkan class
                        
                        # Draw bounding box jika parameter aktif
                        if self.show_bbox:  # Jika param show_bbox=True
                            cv2.rectangle(image, (x1, y1), (x2, y2), color, self.box_thickness)  # Draw rectangle
                        
                        # Prepare text components berdasarkan parameter yang aktif
                        text_parts = self._prepare_text_components(det)  # Siapkan komponen teks
                        
                        # Draw text di atas bounding box jika ada komponen
                        if text_parts:  # Jika ada teks yang perlu ditampilkan
                            text = " | ".join(text_parts)  # Gabung semua komponen teks
                            # Handling agar teks tidak terpotong di atas frame
                            text_y = max(y1 - 10, 20)  # Minimal 20px dari atas
                            cv2.putText(image, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                                       self.text_scale, color, self.text_thickness)  # Draw teks
                    
                    except Exception as e:  # Error pada satu deteksi
                        self.get_logger().warning(f"Error visualizing detection: {e}")  # Log warning
                        logging.warning(f"Error visualizing detection: {e}")  # Log ke file
                        continue  # Skip deteksi ini, lanjut ke detection berikutnya
                
                # Simpan hasil annotasi ke buffer untuk display dan save
                self.latest_annotated[camera_name] = image  # Update buffer annotated image
                
                # Publish annotated image
                try:
                    annotated_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")  # Convert OpenCV -> ROS
                    annotated_msg.header = msg.header  # Gunakan header asli untuk timestamp dan frame_id
                    if camera_name in self.annotated_pubs:  # Cek apakah publisher ada
                        self.annotated_pubs[camera_name].publish(annotated_msg)  # Publish annotated image
                except CvBridgeError as e:  # Error konversi OpenCV -> ROS
                    self.get_logger().error(f'CV Bridge error when publishing annotated image: {e}')  # Log error
                    logging.error(f'CV Bridge error when publishing annotated image: {e}')  # Log ke file
                except Exception as e:  # Error umum lainnya
                    self.get_logger().error(f'Failed to publish annotated image: {e}')  # Log error
                    logging.error(f'Failed to publish annotated image: {e}')  # Log ke file
                
                # Save annotated image jika parameter aktif
                if self.save_annotated_images:  # Jika param save_annotated_images=True
                    try:
                        # Format timestamp untuk filename (dari header message)
                        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Convert ke detik
                        # Buat filename yang unique dengan timestamp
                        filename = f"{camera_name}_{timestamp:.3f}.jpg"  # Format: camera_timestamp.jpg
                        filepath = os.path.join(self.save_path, filename)  # Path lengkap file
                        
                        # Simpan image ke file
                        cv2.imwrite(filepath, image)  # Simpan ke file
                        
                        if self.verbose_logging:  # Log jika verbose
                            self.get_logger().debug(f"Saved annotated image to {filepath}")  # Log debug
                            
                    except Exception as e:  # Error saat save file
                        self.get_logger().warning(f"Failed to save annotated image: {e}")  # Log warning
                        logging.warning(f"Failed to save annotated image: {e}")  # Log ke file
                
            except Exception as e:  # Error umum pada callback
                self.get_logger().error(f"Error in fusion callback: {e}\n{traceback.format_exc()}")  # Log error dengan stack trace
                logging.error(f"Error in fusion callback: {e}\n{traceback.format_exc()}")  # Log ke file
        
        # Track performance
        processing_time = time.time() - start_time  # Hitung waktu proses
        self.processing_times.append(processing_time)  # Tambahkan ke list untuk logging
        
        # Log verbose performance stats
        if self.verbose_logging:  # Jika verbose logging aktif
            num_detections = len(msg.yolov12_inference) if hasattr(msg, 'yolov12_inference') else 0  # Hitung jumlah deteksi
            self.get_logger().debug(f"Processed {num_detections} detections in {processing_time*1000:.1f}ms")  # Log debug
    
    def _validate_bbox_coordinates(self, det: Yolov12Instance, img_width: int, img_height: int) -> Tuple[int, int, int, int]:
        """
        Validasi dan perbaiki koordinat bounding box agar tidak keluar frame.
        
        Args:
            det: Yolov12Instance detection
            img_width: Lebar gambar
            img_height: Tinggi gambar
            
        Returns:
            Tuple koordinat (x1, y1, x2, y2) yang sudah divalidasi
        
        Raises:
            ValueError: Jika koordinat tidak valid dan tidak bisa diperbaiki
        """
        try:
            # Extract original coordinates
            x1, y1 = int(det.left), int(det.top)  # Koordinat top-left
            x2, y2 = int(det.right), int(det.bottom)  # Koordinat bottom-right
            
            # Validasi nilai integer yang masuk akal
            if x1 < 0 or y1 < 0 or x2 <= 0 or y2 <= 0 or x1 >= img_width or y1 >= img_height:  # Cek invalid coordinates
                self.get_logger().warning(f"Invalid bbox coordinates: ({x1},{y1},{x2},{y2}) in image {img_width}x{img_height}")  # Log warning
                # Coba perbaiki jika memungkinkan
                x1 = max(0, min(x1, img_width - 1))  # Clamp x1
                y1 = max(0, min(y1, img_height - 1))  # Clamp y1
                x2 = max(x1 + 1, min(x2, img_width))  # Clamp x2
                y2 = max(y1 + 1, min(y2, img_height))  # Clamp y2
                
            # Pastikan x2 > x1 dan y2 > y1
            if x2 <= x1 or y2 <= y1:  # Validasi lebar dan tinggi > 0
                self.get_logger().warning(f"Invalid bbox dimensions: width={x2-x1}, height={y2-y1}")  # Log warning
                # Force minimal size 1x1 pixel
                x2 = max(x1 + 1, x2)  # Ensure x2 > x1
                y2 = max(y1 + 1, y2)  # Ensure y2 > y1
            
            # Clipping koordinat agar tidak keluar frame
            x1 = max(0, min(x1, img_width - 1))  # Ensure x1 within frame
            y1 = max(0, min(y1, img_height - 1))  # Ensure y1 within frame
            x2 = max(x1 + 1, min(x2, img_width))  # Ensure x2 within frame and > x1
            y2 = max(y1 + 1, min(y2, img_height))  # Ensure y2 within frame and > y1
            
            return (x1, y1, x2, y2)  # Return validated coordinates
            
        except (ValueError, TypeError) as e:  # Tangkap error nilai atau tipe
            # Jika bounding box tidak valid, return fallback minimal box
            self.get_logger().warning(f"Invalid bbox: {e}, using fallback")  # Log warning
            logging.warning(f"Invalid bbox: {e}, using fallback")  # Log ke file
            return (0, 0, min(10, img_width-1), min(10, img_height-1))  # Return fallback minimal box
    
    def _get_class_color(self, class_name: str) -> Tuple[int, int, int]:
        """
        Dapatkan warna untuk class tertentu.
        
        Args:
            class_name: Nama class object
            
        Returns:
            Tuple RGB color (OpenCV format: BGR)
        """
        # Jika class ada di dictionary, gunakan warna yang sudah ditentukan
        # Jika tidak ada, gunakan warna default
        return CLASS_COLORS.get(class_name.lower(), CLASS_COLORS["default"])  # Get color or default
    
    def _prepare_text_components(self, det: Yolov12Instance) -> List[str]:
        """
        Siapkan komponen text untuk display berdasarkan parameter.
        
        Args:
            det: Yolov12Instance detection
            
        Returns:
            List komponen teks yang akan digabung
        """
        text_parts = []  # Init list kosong
        
        # Tambahkan nama class jika parameter aktif
        if self.show_class and det.class_name:  # Jika show_class aktif dan class_name ada
            text_parts.append(f"{det.class_name}")  # Tambahkan class name ke teks
        
        # Tambahkan confidence score jika parameter aktif    
        if self.show_confidence:  # Jika show_confidence aktif
            text_parts.append(f"{det.confidence:.2f}")  # Tambahkan confidence (2 decimal places)
        
        # Parse distance dan koordinat dari field note jika tersedia
        if det.note:  # Jika field note tidak kosong
            # Tampilkan distance jika parameter aktif dan info tersedia
            if self.show_distance and "Distance:" in det.note:  # Jika show_distance aktif dan info distance ada
                try:
                    # Extract distance parts
                    distance_parts = det.note.split(",")  # Split note by comma
                    distance_text = next((part for part in distance_parts if "Distance:" in part), "")  # Find distance part
                    if distance_text:  # If distance text found
                        text_parts.append(distance_text.strip())  # Add to text parts
                except Exception:
                    pass  # Skip jika format tidak sesuai
            
            # Tampilkan 3D coords jika parameter aktif dan info tersedia    
            if self.show_coordinates and "Coord:" in det.note:  # Jika show_coordinates aktif dan info koordinat ada
                try:
                    # Extract coordinate text dari note
                    coord_text = det.note.split("Coord:")[1].strip()  # Extract text after "Coord:"
                    if coord_text:  # If coordinate text found
                        text_parts.append(f"Coord: {coord_text}")  # Add to text parts
                except Exception:
                    pass  # Skip jika format tidak sesuai
        
        return text_parts  # Return list komponen teks
    
    def display_all_cameras(self) -> None:
        """
        Menampilkan semua kamera dalam satu window (tiled display).
        Hanya dijalankan jika parameter display_window=True.
        """
        if not self.display_window:  # Skip jika display window disabled
            return  # Exit function
            
        with self.lock:  # Thread-safe access ke shared data
            try:
                # Hitung grid layout berdasarkan jumlah kamera
                cameras_with_frames = [cam for cam, img in self.latest_annotated.items() if img is not None]  # Filter kamera dengan frame
                n_cameras = len(cameras_with_frames)  # Hitung jumlah kamera dengan frame
                
                if n_cameras == 0:  # Jika belum ada kamera dengan frame
                    return  # Tunggu sampai ada frame
                
                # Hitung grid size
                grid_size = int(np.ceil(np.sqrt(n_cameras)))  # Grid size = ceil(sqrt(n_cameras))
                grid_w, grid_h = grid_size, grid_size  # Default grid square
                
                # Buat canvas untuk tiled view
                # Gunakan resolusi dari parameter atau ukuran frame terbesar
                max_h, max_w = 0, 0  # Init max height dan width
                for cam in cameras_with_frames:  # Iterasi kamera dengan frame
                    if cam in self.latest_annotated and self.latest_annotated[cam] is not None:  # Jika ada annotated image
                        h, w = self.latest_annotated[cam].shape[:2]  # Ambil dimensi image
                        max_h = max(max_h, h)  # Update max height
                        max_w = max(max_w, w)  # Update max width
                
                # Compute the canvas size for the tiled view
                display_w = max_w * grid_w  # Grid width total
                display_h = max_h * grid_h  # Grid height total
                
                # Limit canvas size berdasarkan parameter
                scale_factor = min(1.0, min(self.display_width / display_w, self.display_height / display_h))  # Scale factor untuk grid
                scaled_w = int(max_w * scale_factor)  # Scaled cell width
                scaled_h = int(max_h * scale_factor)  # Scaled cell height
                
                # Create tiled canvas
                canvas = np.zeros((scaled_h * grid_h, scaled_w * grid_w, 3), dtype=np.uint8)  # Buat canvas kosong
                
                # Add camera status text pada canvas
                status_color = {
                    'OK': (0, 255, 0),  # Green for OK
                    'ERROR': (0, 0, 255),  # Red for ERROR
                    'INITIALIZING': (0, 255, 255)  # Yellow for INITIALIZING
                }
                
                # Populate tiled view
                for i, cam in enumerate(cameras_with_frames):  # Iterasi kamera dengan frame
                    if cam in self.latest_annotated and self.latest_annotated[cam] is not None:  # Jika ada annotated image
                        # Compute grid position
                        grid_x = i % grid_w  # Grid column
                        grid_y = i // grid_w  # Grid row
                        
                        # Resize frame jika perlu
                        frame = self.latest_annotated[cam]  # Get annotated image
                        if frame.shape[0] != scaled_h or frame.shape[1] != scaled_w:  # Jika ukuran tidak sama
                            frame = cv2.resize(frame, (scaled_w, scaled_h))  # Resize frame
                        
                        # Compute position in canvas
                        y_start = grid_y * scaled_h  # Y start position
                        y_end = (grid_y + 1) * scaled_h  # Y end position
                        x_start = grid_x * scaled_w  # X start position
                        x_end = (grid_x + 1) * scaled_w  # X end position
                        
                        # Insert frame into canvas
                        canvas[y_start:y_end, x_start:x_end] = frame  # Copy frame ke canvas
                        
                        # Add camera name dan status
                        status = self.camera_status[cam]['status']  # Get camera status
                        color = status_color.get(status, (255, 255, 255))  # Get status color
                        fps = self.fps_stats[cam]['fps']  # Get FPS
                        text = f"{cam}: {status} ({fps:.1f} FPS)"  # Format text
                        
                        # Draw text pada frame
                        cv2.putText(canvas, text, (x_start + 5, y_start + 25),  # Position
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)  # Style
                
                # Show the tiled view
                cv2.imshow('Huskybot Fusion Visualizer', canvas)  # Show canvas
                cv2.waitKey(1)  # Process events dengan non-blocking wait
                
            except Exception as e:  # Error saat display
                self.get_logger().error(f"Error displaying cameras: {e}\n{traceback.format_exc()}")  # Log error dengan stack trace
                logging.error(f"Error displaying cameras: {e}\n{traceback.format_exc()}")  # Log ke file
    
    def on_shutdown(self) -> None:
        """
        Cleanup ketika node di-shutdown.
        Pastikan semua resources dilepas dengan benar.
        """
        try:
            self.get_logger().info("Shutting down Fusion Visualizer Node...")  # Log shutdown
            logging.info("Shutting down Fusion Visualizer Node...")  # Log ke file
            
            # Hitung uptime
            uptime = time.time() - self.startup_time  # Hitung uptime dalam detik
            hours, remainder = divmod(uptime, 3600)  # Convert ke jam dan sisa
            minutes, seconds = divmod(remainder, 60)  # Convert sisa ke menit dan detik
            
            self.get_logger().info(f"Node uptime: {int(hours)}h {int(minutes)}m {int(seconds)}s")  # Log uptime
            logging.info(f"Node uptime: {int(hours)}h {int(minutes)}m {int(seconds)}s")  # Log ke file
            
            # Log camera stats
            for cam, status in self.camera_status.items():  # Iterasi tiap kamera
                self.get_logger().info(f"Camera {cam}: {status['frame_count']} frames received, "
                                      f"{status['error_count']} errors, status: {status['status']}")  # Log stats
            
            # Close OpenCV windows
            if self.display_window:  # Jika display window aktif
                try:
                    cv2.destroyAllWindows()  # Tutup semua window
                    self.get_logger().info("Closed OpenCV display windows")  # Log info
                except Exception as e:  # Error saat close window
                    self.get_logger().error(f"Error closing OpenCV windows: {e}")  # Log error
                    logging.error(f"Error closing OpenCV windows: {e}")  # Log ke file
            
            # Log summary
            self.get_logger().info(f"Processed {self.total_fusion_msgs} fusion messages")  # Log message count
            logging.info(f"Processed {self.total_fusion_msgs} fusion messages")  # Log ke file
            
        except Exception as e:  # Error saat shutdown
            self.get_logger().error(f"Error during shutdown: {e}\n{traceback.format_exc()}")  # Log error dengan stack trace
            logging.error(f"Error during shutdown: {e}\n{traceback.format_exc()}")  # Log ke file


def main(args=None) -> None:
    """
    Entry point untuk node fusion visualizer.
    
    Inisialisasi ROS2, buat node, dan jalankan sampai shutdown.
    Implementasi error handling untuk semua kondisi runtime.
    
    Args:
        args: Command line arguments
    """
    try:
        # Inisialisasi ROS2
        rclpy.init(args=args)  # Init ROS2 dengan command line args
        
        # Buat node
        node = FusionVisualizerNode()  # Buat instance node
        
        # Log startup info
        node.get_logger().info("Fusion Visualizer Node is running")  # Log info
        
        try:
            # Spin node (blocking call sampai shutdown)
            rclpy.spin(node)  # Spin node (blocking)
        except KeyboardInterrupt:  # Tangkap Ctrl+C
            # Normal shutdown via Ctrl+C
            node.get_logger().info('Keyboard interrupt, shutting down...')  # Log info
        except Exception as e:  # Error saat running
            # Exception saat runtime
            node.get_logger().error(f'Runtime error: {e}\n{traceback.format_exc()}')  # Log error dengan stack trace
            logging.error(f'Runtime error: {e}\n{traceback.format_exc()}')  # Log ke file
        finally:
            # Cleanup saat shutdown (baik normal maupun error)
            try:
                node.on_shutdown()  # Call method cleanup khusus
                node.destroy_node()  # Destroy node (cleanup oleh ROS2)
            except Exception as e:  # Error saat cleanup
                print(f"Error during node cleanup: {e}")  # Print ke stderr
                logging.error(f"Error during node cleanup: {e}")  # Log ke file
            
            try:
                # Shutdown ROS2
                rclpy.shutdown()  # Shutdown ROS2
            except Exception as e:  # Error saat shutdown
                print(f"Error during ROS2 shutdown: {e}")  # Print ke stderr
                logging.error(f"Error during ROS2 shutdown: {e}")  # Log ke file
                
    except Exception as e:  # Error saat inisialisasi
        # Exception saat inisialisasi
        print(f"Fatal error initializing node: {e}\n{traceback.format_exc()}")  # Print ke stderr dengan stack trace
        logging.critical(f"Fatal error initializing node: {e}\n{traceback.format_exc()}")  # Log ke file
        
        # Fallback shutdown
        try:
            rclpy.shutdown()  # Coba shutdown ROS2 meskipun init gagal
        except Exception:
            pass  # Ignore error jika gagal shutdown


if __name__ == '__main__':
    """
    Execute node jika file dijalankan langsung.
    """
    main()  # Call main function