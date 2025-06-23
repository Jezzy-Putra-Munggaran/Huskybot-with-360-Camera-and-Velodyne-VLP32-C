#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/huskybot_camera/multicamera_publisher.py

import os                      # Import untuk operasi sistem seperti akses file dan environment variables
import sys                     # Import untuk akses parameter sistem dan exit codes
import time                    # Import untuk fungsi sleep dan timestamp
import threading               # Import untuk thread lock (thread safety)
import traceback               # Import untuk stack trace detail saat exception
from datetime import datetime  # Import untuk timestamp log/diagnostik
from typing import Dict, List, Tuple, Optional, Any  # Import type hints untuk dokumentasi kode yang lebih jelas

import rclpy                                # Import library utama ROS2 Python
from rclpy.node import Node                 # Import class Node dari ROS2 untuk membuat node
from rclpy.parameter import Parameter       # Import Parameter untuk deklarasi dan validasi parameter
from rclpy.exceptions import ParameterNotDeclaredException  # Import exception untuk parameter tidak ditemukan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy  # Import QoS untuk konfigurasi komunikasi
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Import untuk deskripsi parameter (dokumentasi parameter)
from rclpy.executors import MultiThreadedExecutor  # Import MultiThreadedExecutor untuk concurrent processing

from sensor_msgs.msg import Image           # Import message type Image untuk publish gambar
from sensor_msgs.msg import CameraInfo      # Import message type CameraInfo untuk publish info kamera
from std_msgs.msg import Header            # Import Header untuk timestamp message
from std_srvs.srv import Trigger            # Import service type Trigger untuk service restart/status
from cv_bridge import CvBridge             # Import CvBridge untuk konversi OpenCV <-> ROS images

import cv2                     # Import OpenCV untuk akses kamera dan image processing
import numpy as np             # Import numpy untuk array operations

# Konfigurasi default untuk 6 kamera dalam array hexagonal, digunakan jika parameter tidak dispesifikasi
DEFAULT_CAMERA_CONFIG = [
    # (topic, device, frame_id, width, height, fps)
    ('/camera_front/image_raw', 'csi://0', 'camera_front_optical_frame', 1920, 1080, 30.0),
    ('/camera_front_left/image_raw', 'csi://1', 'camera_front_left_optical_frame', 1920, 1080, 30.0),
    ('/camera_left/image_raw', 'csi://2', 'camera_left_optical_frame', 1920, 1080, 30.0),
    ('/camera_rear/image_raw', 'csi://3', 'camera_rear_optical_frame', 1920, 1080, 30.0),
    ('/camera_rear_right/image_raw', 'csi://4', 'camera_rear_right_optical_frame', 1920, 1080, 30.0),
    ('/camera_right/image_raw', 'csi://5', 'camera_right_optical_frame', 1920, 1080, 30.0),
]


class CameraPublisher(Node):
    """
    Node ROS2 untuk publikasi gambar dari multiple kamera secara simultan.
    
    Publishes:
        image_raw (sensor_msgs/Image): Raw image dari tiap kamera
        camera_info (sensor_msgs/CameraInfo): Informasi kalibrasi kamera (jika parameter publish_camera_info=True)
        
    Services:
        ~/restart_cameras (std_srvs/Trigger): Service untuk restart semua kamera setelah error
        ~/get_status (std_srvs/Trigger): Service untuk mendapatkan status semua kamera
        
    Parameters:
        ~camera_config (string): Path ke file YAML dengan konfigurasi kamera
        ~use_sim_time (bool): Flag untuk menggunakan waktu simulasi
        ~retry_count (int): Jumlah retry untuk koneksi kamera jika gagal
        ~retry_delay (float): Delay antar retry dalam detik
        ~publish_rate (float): Rate publikasi gambar dalam Hz
        ~publish_camera_info (bool): Flag apakah publish camera_info messages
        ~log_file (string): Path untuk log file (kosong = tidak log ke file)
        ~fallback_to_video_files (bool): Jika true, coba file video jika kamera hardware gagal
        ~video_file_dir (string): Path ke direktori berisi fallback video files
    """
    
    def __init__(self):
        """Inisialisasi node kamera, setup subscribers, publishers, services, dan parameters."""
        
        # Inisialisasi node ROS2 dengan nama descriptive
        super().__init__('multicamera_publisher')
        
        # Setup logging
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Default log directory
        
        # Buat folder log jika belum ada
        self.ensure_log_directory()
        
        self.get_logger().info("Initializing MultiCamera Publisher Node...")  # Log info inisialisasi
        
        # Setup thread lock untuk thread safety
        self.lock = threading.RLock()  # Thread lock untuk operasi thread-safe
        
        # Declare dan load parameter-parameter
        self.declare_parameters()
        self.load_parameters()
        
        # Setup CvBridge untuk konversi OpenCV <-> ROS images
        self.bridge = CvBridge()  # Converter antara format gambar OpenCV dan ROS
        
        # Containers untuk camera dan publisher
        self.caps = {}         # Dictionary untuk camera capture objects (key: device)
        self.publishers = {}   # Dictionary untuk image publishers (key: topic)
        self.info_publishers = {}  # Dictionary untuk camera info publishers (key: topic)
        self.camera_active = {}  # Status kamera aktif (key: device)
        self.frame_counts = {}  # Counter frames per kamera (untuk diagnostik)
        self.last_frame_time = {}  # Timestamp frame terakhir (untuk timeout detection)
        self.failed_reads = {}  # Counter kegagalan bacaan per kamera (untuk diagnostik)
        self.retry_counts = {}  # Counter retries per kamera (untuk auto-recovery)
        
        # Setup services
        self.restart_srv = self.create_service(Trigger, 'restart_cameras', self.restart_cameras_callback)  # Service untuk restart kamera
        self.status_srv = self.create_service(Trigger, 'get_status', self.get_status_callback)  # Service untuk cek status kamera
        
        # Membaca konfigurasi
        self.read_camera_config()
        
        # Setup kamera dan publisher
        self.setup_cameras_and_publishers()
        
        # Buat timer untuk publish images
        self.create_timers()
        
        # Timer untuk health check
        self.health_check_timer = self.create_timer(5.0, self.camera_health_check)  # Timer untuk cek kesehatan kamera setiap 5 detik
        
        # Log info startup yang sukses
        self.get_logger().info("MultiCamera Publisher Node initialized successfully!")
        self.log_to_file("MultiCamera Publisher Node initialized successfully!", level='info')
    
    def declare_parameters(self):
        """Deklarasi semua parameter dengan deskripsi dan tipe yang jelas."""
        try:
            # Parameter deskriptor untuk file paths
            path_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to a file or directory'
            )
            
            # Parameter deskriptor untuk booleans
            bool_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Boolean flag'
            )
            
            # Parameter deskriptor untuk integers
            int_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Integer value'
            )
            
            # Parameter deskriptor untuk floats
            float_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, 
                description='Floating point value'
            )
            
            # Parameter deskriptor untuk array/list
            array_descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Array of strings'
            )
            
            # Parameter: config_file (path ke YAML dengan config)
            self.declare_parameter('config_file', '', path_descriptor)
            
            # Parameter: use_sim_time (untuk gazebo)
            self.declare_parameter(
                'use_sim_time', 
                False,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    description='Use simulation time for ROS timestamps (true for Gazebo)'
                )
            )
            
            # Parameter: retry_count (jumlah retry untuk koneksi kamera)
            self.declare_parameter(
                'retry_count', 
                5, 
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='Number of times to retry camera connection if failed'
                )
            )
            
            # Parameter: retry_delay (delay antara retries dalam detik)
            self.declare_parameter(
                'retry_delay', 
                2.0, 
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Delay between retry attempts in seconds'
                )
            )
            
            # Parameter: publish_rate (Hz untuk timer publish)
            self.declare_parameter(
                'publish_rate', 
                20.0, 
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Rate at which to publish camera images in Hz'
                )
            )
            
            # Parameter: publish_camera_info (apakah publish camera_info messages)
            self.declare_parameter(
                'publish_camera_info', 
                False, 
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    description='Whether to publish camera_info messages alongside images'
                )
            )
            
            # Parameter: log_file (path untuk file log)
            self.declare_parameter(
                'log_file', 
                os.path.join(self.log_dir, 'multicamera.log'),
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    description='Path to log file (empty for no file logging)'
                )
            )
            
            # Parameter: fallback_to_video_files (flag untuk use video files jika kamera hardware gagal)
            self.declare_parameter(
                'fallback_to_video_files', 
                False,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_BOOL,
                    description='Fall back to video files if hardware cameras fail'
                )
            )
            
            # Parameter: video_file_dir (direktori berisi fallback video files)
            self.declare_parameter(
                'video_file_dir', 
                '',
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    description='Directory containing fallback video files'
                )
            )
            
            # Parameter untuk konfigurasi kamera (jika tidak menggunakan file config)
            for i, (topic, device, frame_id, width, height, fps) in enumerate(DEFAULT_CAMERA_CONFIG):
                # Camera enable flag
                self.declare_parameter(
                    f'camera{i+1}.enable', 
                    True, 
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_BOOL,
                        description=f'Enable camera {i+1}'
                    )
                )
                
                # Camera topic
                self.declare_parameter(
                    f'camera{i+1}.topic', 
                    topic,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description=f'ROS topic for camera {i+1}'
                    )
                )
                
                # Camera device
                self.declare_parameter(
                    f'camera{i+1}.device', 
                    device,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description=f'Device path for camera {i+1}'
                    )
                )
                
                # Camera frame ID
                self.declare_parameter(
                    f'camera{i+1}.frame_id', 
                    frame_id,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description=f'TF frame ID for camera {i+1}'
                    )
                )
                
                # Camera width
                self.declare_parameter(
                    f'camera{i+1}.width', 
                    width,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description=f'Image width for camera {i+1}'
                    )
                )
                
                # Camera height
                self.declare_parameter(
                    f'camera{i+1}.height', 
                    height,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description=f'Image height for camera {i+1}'
                    )
                )
                
                # Camera framerate
                self.declare_parameter(
                    f'camera{i+1}.fps', 
                    fps,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description=f'Framerate for camera {i+1}'
                    )
                )
                
                # Camera flip flag
                self.declare_parameter(
                    f'camera{i+1}.flip', 
                    False,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_BOOL,
                        description=f'Flip image horizontally for camera {i+1}'
                    )
                )
        except Exception as e:
            self.get_logger().error(f"Error declaring parameters: {str(e)}")
            self.log_to_file(f"Error declaring parameters: {str(e)}", level='error')
            raise  # Re-raise exception setelah log untuk penanganan di level lebih tinggi
    
    def load_parameters(self):
        """Load semua parameter dari ROS parameter server."""
        try:
            # Load parameter primitif
            self.use_sim_time = self.get_parameter('use_sim_time').value
            self.retry_count = self.get_parameter('retry_count').value
            self.retry_delay = self.get_parameter('retry_delay').value
            self.publish_rate = self.get_parameter('publish_rate').value
            self.publish_camera_info = self.get_parameter('publish_camera_info').value
            self.log_file_path = self.get_parameter('log_file').value
            self.fallback_to_video_files = self.get_parameter('fallback_to_video_files').value
            self.video_file_dir = self.get_parameter('video_file_dir').value
            self.config_file = self.get_parameter('config_file').value
            
            # Log parameter yang dimuat
            params_log = (
                f"Loaded parameters:\n"
                f"- use_sim_time: {self.use_sim_time}\n"
                f"- retry_count: {self.retry_count}\n"
                f"- retry_delay: {self.retry_delay}\n"
                f"- publish_rate: {self.publish_rate} Hz\n"
                f"- publish_camera_info: {self.publish_camera_info}\n"
                f"- log_file: {self.log_file_path}\n"
                f"- fallback_to_video_files: {self.fallback_to_video_files}\n"
                f"- video_file_dir: {self.video_file_dir}\n"
                f"- config_file: {self.config_file}"
            )
            self.get_logger().info(params_log)
            self.log_to_file(params_log, level='info')
            
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f"Required parameter not declared: {str(e)}")
            self.log_to_file(f"Required parameter not declared: {str(e)}", level='error')
            raise
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {str(e)}")
            self.log_to_file(f"Error loading parameters: {str(e)}", level='error')
            raise
    
    def read_camera_config(self):
        """Baca konfigurasi kamera dari file atau parameter."""
        try:
            self.camera_configs = []  # List untuk menyimpan konfigurasi semua kamera
            
            # Check if we should read from config file
            if self.config_file and os.path.isfile(self.config_file):
                self.get_logger().info(f"Reading camera configuration from file: {self.config_file}")
                self.log_to_file(f"Reading camera configuration from file: {self.config_file}", level='info')
                
                # Try to read YAML file
                try:
                    import yaml
                    with open(self.config_file, 'r') as f:
                        yaml_config = yaml.safe_load(f)
                    
                    # Parse camera configs from YAML
                    if 'cameras' in yaml_config and isinstance(yaml_config['cameras'], list):
                        for cam_config in yaml_config['cameras']:
                            if all(k in cam_config for k in ['topic', 'device', 'frame_id']):
                                topic = cam_config['topic']
                                device = cam_config['device']
                                frame_id = cam_config['frame_id']
                                width = cam_config.get('width', 1920)
                                height = cam_config.get('height', 1080)
                                fps = float(cam_config.get('fps', 30.0))
                                flip = bool(cam_config.get('flip', False))
                                
                                self.camera_configs.append({
                                    'topic': topic,
                                    'device': device,
                                    'frame_id': frame_id,
                                    'width': width,
                                    'height': height,
                                    'fps': fps,
                                    'flip': flip,
                                    'enable': True
                                })
                    else:
                        self.get_logger().warn("Invalid YAML format: 'cameras' list not found. Using default configuration.")
                        self.log_to_file("Invalid YAML format: 'cameras' list not found. Using default configuration.", level='warn')
                        # Fallback to default
                        self.read_param_configs()
                except Exception as e:
                    self.get_logger().error(f"Failed to parse camera config file: {str(e)}")
                    self.log_to_file(f"Failed to parse camera config file: {str(e)}", level='error')
                    # Fallback to default
                    self.read_param_configs()
            else:
                # If no config file, read from ROS parameters
                self.get_logger().info("Reading camera configuration from ROS parameters")
                self.log_to_file("Reading camera configuration from ROS parameters", level='info')
                self.read_param_configs()
                
            # Validate we have at least one camera
            if not self.camera_configs:
                self.get_logger().error("No cameras configured! Please check your configuration.")
                self.log_to_file("No cameras configured! Please check your configuration.", level='error')
                # Create at least one default camera as fallback
                self.camera_configs = [{
                    'topic': DEFAULT_CAMERA_CONFIG[0][0],
                    'device': DEFAULT_CAMERA_CONFIG[0][1],
                    'frame_id': DEFAULT_CAMERA_CONFIG[0][2],
                    'width': DEFAULT_CAMERA_CONFIG[0][3],
                    'height': DEFAULT_CAMERA_CONFIG[0][4],
                    'fps': DEFAULT_CAMERA_CONFIG[0][5],
                    'flip': False,
                    'enable': True
                }]
                
            # Log the final camera configuration
            self.get_logger().info(f"Final camera configuration: {len(self.camera_configs)} cameras")
            for i, cam in enumerate(self.camera_configs):
                self.get_logger().info(f"Camera {i+1}: {cam['device']} -> {cam['topic']} ({cam['width']}x{cam['height']} @ {cam['fps']} fps)")
                
        except Exception as e:
            self.get_logger().error(f"Error reading camera configuration: {str(e)}")
            self.log_to_file(f"Error reading camera configuration: {str(e)}\n{traceback.format_exc()}", level='error')
            # Set a safe default configuration with just one camera
            self.camera_configs = [{
                'topic': DEFAULT_CAMERA_CONFIG[0][0],
                'device': DEFAULT_CAMERA_CONFIG[0][1],
                'frame_id': DEFAULT_CAMERA_CONFIG[0][2],
                'width': DEFAULT_CAMERA_CONFIG[0][3],
                'height': DEFAULT_CAMERA_CONFIG[0][4],
                'fps': DEFAULT_CAMERA_CONFIG[0][5],
                'flip': False,
                'enable': True
            }]
    
    def read_param_configs(self):
        """Read camera configuration from ROS parameters."""
        try:
            self.camera_configs = []
            
            # Check each potential camera parameter
            for i, (default_topic, default_device, default_frame_id, default_width, default_height, default_fps) in enumerate(DEFAULT_CAMERA_CONFIG):
                try:
                    # Check if camera is enabled
                    if not self.get_parameter(f'camera{i+1}.enable').value:
                        self.get_logger().info(f"Camera {i+1} is disabled in parameters")
                        continue
                        
                    topic = self.get_parameter(f'camera{i+1}.topic').value
                    device = self.get_parameter(f'camera{i+1}.device').value
                    frame_id = self.get_parameter(f'camera{i+1}.frame_id').value
                    width = self.get_parameter(f'camera{i+1}.width').value
                    height = self.get_parameter(f'camera{i+1}.height').value
                    fps = self.get_parameter(f'camera{i+1}.fps').value
                    flip = self.get_parameter(f'camera{i+1}.flip').value
                    
                    self.camera_configs.append({
                        'topic': topic,
                        'device': device,
                        'frame_id': frame_id,
                        'width': width,
                        'height': height,
                        'fps': fps,
                        'flip': flip,
                        'enable': True
                    })
                    
                except ParameterNotDeclaredException:
                    # This camera parameter set doesn't exist - that's okay, might be less than 6 cameras
                    pass
                except Exception as e:
                    self.get_logger().warn(f"Error reading camera{i+1} parameters: {str(e)}, using defaults")
                    self.log_to_file(f"Error reading camera{i+1} parameters: {str(e)}, using defaults", level='warn')
                    
                    # Add default config for this camera
                    self.camera_configs.append({
                        'topic': default_topic,
                        'device': default_device,
                        'frame_id': default_frame_id,
                        'width': default_width,
                        'height': default_height,
                        'fps': default_fps,
                        'flip': False,
                        'enable': True
                    })
                    
        except Exception as e:
            self.get_logger().error(f"Error reading camera parameters: {str(e)}")
            self.log_to_file(f"Error reading camera parameters: {str(e)}", level='error')
    
    def setup_cameras_and_publishers(self):
        """Setup semua kamera dan publisher berdasarkan konfigurasi."""
        try:
            for i, camera in enumerate(self.camera_configs):
                if not camera.get('enable', True):
                    continue
                    
                topic = camera['topic']
                device = camera['device']
                frame_id = camera['frame_id']
                width = camera['width']
                height = camera['height']
                fps = camera['fps']
                
                self.get_logger().info(f"Setting up camera {i+1}: {device} -> {topic}")
                
                # Setup publisher dengan QoS profile yang sesuai untuk video streaming
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1  # Hanya simpan frame terbaru
                )
                
                # Publisher untuk raw image
                self.publishers[device] = self.create_publisher(Image, topic, qos)
                
                # Publisher untuk camera info jika diperlukan
                if self.publish_camera_info:
                    info_topic = topic.rsplit('/', 1)[0] + '/camera_info'  # e.g., /camera_front/camera_info
                    self.info_publishers[device] = self.create_publisher(CameraInfo, info_topic, qos)
                
                # Initialize tracking variables
                self.frame_counts[device] = 0
                self.failed_reads[device] = 0
                self.retry_counts[device] = 0
                self.last_frame_time[device] = time.time()
                
                # Coba open camera
                self.open_camera(device, width, height, fps)
                
        except Exception as e:
            self.get_logger().error(f"Error setting up cameras: {str(e)}")
            self.log_to_file(f"Error setting up cameras: {str(e)}\n{traceback.format_exc()}", level='error')
    
    def open_camera(self, device, width, height, fps):
        """
        Buka koneksi ke kamera tertentu dengan retry hingga retry_count kali.
        
        Args:
            device (str): Path device kamera ('csi://X' atau '/dev/videoX')
            width (int): Resolusi width yang diinginkan
            height (int): Resolusi height yang diinginkan
            fps (float): Frame rate yang diinginkan
        
        Returns:
            bool: True jika sukses, False jika gagal setelah semua retry
        """
        # Simple conversion for CSI camera index
        if device.startswith('csi://'):
            try:
                # Konversi device string ke device number
                cam_id = int(device.replace('csi://', ''))
                # Di simulasi, ganti dengan device file
                if self.use_sim_time:
                    device_path = f"/dev/video{cam_id}"
                    self.get_logger().info(f"Using simulation mode, mapping {device} to {device_path}")
                else:
                    device_path = cam_id  # Use numeric ID for OpenCV in CSI mode
            except ValueError:
                self.get_logger().error(f"Invalid CSI device format: {device}, expected 'csi://N' where N is an integer")
                self.log_to_file(f"Invalid CSI device format: {device}, expected 'csi://N' where N is an integer", level='error')
                self.camera_active[device] = False
                return False
        else:
            device_path = device
            
        # Initialize retry counter
        retry = 0
        
        # Attempt to open camera with retries
        while retry <= self.retry_count:
            try:
                self.get_logger().info(f"Attempting to open camera {device} (retry {retry}/{self.retry_count})")
                
                # Try to open the camera
                cap = cv2.VideoCapture(device_path)
                
                # Check if camera opened successfully
                if not cap.isOpened():
                    self.get_logger().warn(f"Failed to open camera {device} (attempt {retry+1}/{self.retry_count+1})")
                    retry += 1
                    
                    if retry <= self.retry_count:
                        self.get_logger().info(f"Retrying in {self.retry_delay} seconds...")
                        time.sleep(self.retry_delay)
                        continue
                    else:
                        # Try fallback to video files if enabled
                        if self.fallback_to_video_files:
                            self.get_logger().warn(f"Trying fallback video file for {device}")
                            success = self.try_video_file_fallback(device, device_path)
                            if success:
                                return True
                        
                        self.get_logger().error(f"Camera {device} could not be opened after {retry} attempts")
                        self.log_to_file(f"Camera {device} could not be opened after {retry} attempts", level='error')
                        self.camera_active[device] = False
                        return False
                
                # Set camera properties
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                
                # Read test frame untuk verifikasi
                ret, test_frame = cap.read()
                if not ret or test_frame is None:
                    self.get_logger().warn(f"Camera {device} opened but could not read test frame (attempt {retry+1}/{self.retry_count+1})")
                    cap.release()
                    retry += 1
                    
                    if retry <= self.retry_count:
                        self.get_logger().info(f"Retrying in {self.retry_delay} seconds...")
                        time.sleep(self.retry_delay)
                        continue
                    else:
                        # Try fallback to video files if enabled
                        if self.fallback_to_video_files:
                            self.get_logger().warn(f"Trying fallback video file for {device}")
                            success = self.try_video_file_fallback(device, device_path)
                            if success:
                                return True
                                
                        self.get_logger().error(f"Could not read test frame from camera {device} after {retry} attempts")
                        self.log_to_file(f"Could not read test frame from camera {device} after {retry} attempts", level='error')
                        self.camera_active[device] = False
                        return False
                
                # Camera opened and verified successfully
                actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                
                self.get_logger().info(
                    f"Camera {device} opened successfully with resolution "
                    f"{actual_width}x{actual_height} @ {actual_fps} fps"
                )
                self.log_to_file(
                    f"Camera {device} opened successfully with resolution "
                    f"{actual_width}x{actual_height} @ {actual_fps} fps", 
                    level='info'
                )
                
                # Store the camera capture object
                with self.lock:
                    self.caps[device] = cap
                    self.camera_active[device] = True
                
                return True
                
            except Exception as e:
                self.get_logger().error(f"Error opening camera {device}: {str(e)}")
                self.log_to_file(f"Error opening camera {device}: {str(e)}\n{traceback.format_exc()}", level='error')
                retry += 1
                
                if retry <= self.retry_count:
                    self.get_logger().info(f"Retrying in {self.retry_delay} seconds...")
                    time.sleep(self.retry_delay)
                else:
                    # Try fallback to video files if enabled
                    if self.fallback_to_video_files:
                        self.get_logger().warn(f"Trying fallback video file for {device}")
                        success = self.try_video_file_fallback(device, device_path)
                        if success:
                            return True
                            
                    self.get_logger().error(f"Failed to open camera {device} after {retry} attempts: {str(e)}")
                    self.log_to_file(f"Failed to open camera {device} after {retry} attempts: {str(e)}", level='error')
                    self.camera_active[device] = False
                    return False
    
    def try_video_file_fallback(self, device, device_path):
        """
        Coba buka file video sebagai fallback jika kamera hardware gagal.
        
        Args:
            device (str): Original device string
            device_path (str or int): Path/ID device yang digunakan oleh OpenCV
            
        Returns:
            bool: True jika sukses, False jika gagal
        """
        try:
            if not self.video_file_dir:
                self.get_logger().warn(f"No video_file_dir specified for fallback")
                return False
                
            # Derive fallback file name from device
            if device.startswith('csi://'):
                file_id = device.replace('csi://', '')
                fallback_file = os.path.join(self.video_file_dir, f"camera{file_id}.mp4")
            elif device.startswith('/dev/video'):
                file_id = device.replace('/dev/video', '')
                fallback_file = os.path.join(self.video_file_dir, f"camera{file_id}.mp4")
            else:
                # Use a hash of the device string for unique filename
                import hashlib
                file_id = hashlib.md5(device.encode()).hexdigest()[:8]
                fallback_file = os.path.join(self.video_file_dir, f"camera_{file_id}.mp4")
            
            # Check if file exists
            if not os.path.isfile(fallback_file):
                self.get_logger().warn(f"Fallback video file not found: {fallback_file}")
                return False
                
            # Try to open video file
            cap = cv2.VideoCapture(fallback_file)
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open fallback video file: {fallback_file}")
                return False
                
            # Read test frame
            ret, test_frame = cap.read()
            if not ret or test_frame is None:
                self.get_logger().error(f"Failed to read from fallback video file: {fallback_file}")
                cap.release()
                return False
                
            # Set to loop video
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Rewind to start
            
            # Log success
            self.get_logger().info(f"Successfully opened fallback video file: {fallback_file}")
            self.log_to_file(f"Successfully opened fallback video file: {fallback_file}", level='info')
            
            # Store the camera capture object
            with self.lock:
                self.caps[device] = cap
                self.camera_active[device] = True
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error trying video file fallback for {device}: {str(e)}")
            self.log_to_file(f"Error trying video file fallback for {device}: {str(e)}", level='error')
            return False
    
    def create_timers(self):
        """Buat timer untuk publish images dari semua kamera."""
        try:
            # Calculate timer period in seconds from publish rate
            timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05  # Default to 20 Hz
            
            # Create a single timer that publishes all camera images
            self.timer = self.create_timer(timer_period, self.publish_all_images)
            
        except Exception as e:
            self.get_logger().error(f"Error creating timer: {str(e)}")
            self.log_to_file(f"Error creating timer: {str(e)}", level='error')
    
    def publish_all_images(self):
        """Publish images dari semua kamera yang aktif."""
        try:
            # Iterate through all cameras
            for camera in self.camera_configs:
                if not camera.get('enable', True):
                    continue
                    
                device = camera['device']
                frame_id = camera['frame_id']
                flip = camera.get('flip', False)
                
                # Skip inactive cameras
                if device not in self.camera_active or not self.camera_active[device]:
                    continue
                
                # Get the camera
                with self.lock:
                    if device not in self.caps:
                        continue
                    cap = self.caps[device]
                
                # Read frame from camera
                ret, frame = cap.read()
                
                # If frame read failed
                if not ret or frame is None:
                    self.failed_reads[device] = self.failed_reads.get(device, 0) + 1
                    
                    # If video file, loop back to start
                    if self.fallback_to_video_files and cap.get(cv2.CAP_PROP_POS_FRAMES) >= cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        self.get_logger().debug(f"Rewinding video file for {device}")
                        return
                    
                    # Log warning after consecutive failures
                    if self.failed_reads[device] >= 5:  # Warn after 5 consecutive failures
                        self.get_logger().warn(f"Multiple frame read failures for camera {device}, attempting recovery")
                        self.log_to_file(f"Multiple frame read failures for camera {device}, attempting recovery", level='warn')
                        self.retry_counts[device] = self.retry_counts.get(device, 0) + 1
                        
                        # Try to recover after too many failures
                        if self.retry_counts[device] <= self.retry_count:
                            # Close and reopen camera
                            with self.lock:
                                if device in self.caps:
                                    self.caps[device].release()
                                    del self.caps[device]
                            
                            # Find camera config
                            cam_config = next((c for c in self.camera_configs if c['device'] == device), None)
                            if cam_config:
                                # Try to reopen camera
                                self.open_camera(
                                    device, 
                                    cam_config['width'], 
                                    cam_config['height'], 
                                    cam_config['fps']
                                )
                            
                            # Reset failure counter
                            self.failed_reads[device] = 0
                        else:
                            # Give up after retry limit
                            self.get_logger().error(f"Camera {device} recovery failed after {self.retry_counts[device]} attempts")
                            self.log_to_file(f"Camera {device} recovery failed after {self.retry_counts[device]} attempts", level='error')
                            self.camera_active[device] = False
                    
                    continue
                
                # Reset failure counters on successful read
                self.failed_reads[device] = 0
                self.retry_counts[device] = 0
                self.last_frame_time[device] = time.time()
                self.frame_counts[device] = self.frame_counts.get(device, 0) + 1
                
                # Apply flipping if needed
                if flip:
                    frame = cv2.flip(frame, 1)  # 1 = horizontal flip
                
                # Convert image to ROS format
                try:
                    # Create header with correct timestamp and frame_id
                    header = Header()
                    header.frame_id = frame_id
                    
                    # Use current ROS time (sim time if enabled)
                    header.stamp = self.get_clock().now().to_msg()
                    
                    # Convert OpenCV image to ROS Image message
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    img_msg.header = header
                    
                    # Publish image
                    self.publishers[device].publish(img_msg)
                    
                    # Generate and publish camera_info if needed
                    if self.publish_camera_info and device in self.info_publishers:
                        # Create basic CameraInfo message with same header
                        info_msg = CameraInfo()
                        info_msg.header = header
                        info_msg.width = frame.shape[1]
                        info_msg.height = frame.shape[0]
                        
                        # Publish camera info
                        self.info_publishers[device].publish(info_msg)
                    
                except Exception as e:
                    self.get_logger().error(f"Error converting or publishing frame from {device}: {str(e)}")
                    self.log_to_file(f"Error converting or publishing frame from {device}: {str(e)}", level='error')
                
        except Exception as e:
            self.get_logger().error(f"Error in publish_all_images: {str(e)}")
            self.log_to_file(f"Error in publish_all_images: {str(e)}\n{traceback.format_exc()}", level='error')
    
    def camera_health_check(self):
        """Periodic health check untuk semua kamera."""
        try:
            current_time = time.time()
            
            # Check each active camera
            for device, last_time in self.last_frame_time.items():
                if device not in self.camera_active or not self.camera_active[device]:
                    continue
                    
                # If no frame for 10 seconds, camera might be stuck
                if current_time - last_time > 10.0:
                    self.get_logger().warn(f"Camera {device} has not received frames for {current_time - last_time:.1f} seconds")
                    self.log_to_file(f"Camera {device} has not received frames for {current_time - last_time:.1f} seconds", level='warn')
                    
                    # Increment retry counter
                    self.retry_counts[device] = self.retry_counts.get(device, 0) + 1
                    
                    # Try recovery if not exceeding retry limit
                    if self.retry_counts[device] <= self.retry_count:
                        self.get_logger().info(f"Attempting recovery for camera {device}")
                        
                        # Close old camera
                        with self.lock:
                            if device in self.caps:
                                self.caps[device].release()
                                del self.caps[device]
                        
                        # Find camera config
                        cam_config = next((c for c in self.camera_configs if c['device'] == device), None)
                        if cam_config:
                            # Try to reopen camera
                            self.open_camera(
                                device, 
                                cam_config['width'], 
                                cam_config['height'], 
                                cam_config['fps']
                            )
                    else:
                        # Mark camera inactive after retry limit
                        self.get_logger().error(f"Camera {device} recovery failed after {self.retry_counts[device]} attempts, marking inactive")
                        self.log_to_file(f"Camera {device} recovery failed after {self.retry_counts[device]} attempts, marking inactive", level='error')
                        self.camera_active[device] = False
            
            # Log overall status periodically
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len(self.camera_active)
            
            self.get_logger().info(f"Camera status: {active_count}/{total_count} active")
            
            # Log frame rates
            for device, count in self.frame_counts.items():
                if device in self.camera_active and self.camera_active[device]:
                    # Calculate frames captured since last check (approx 5 seconds)
                    fps = count / 5.0  # Approximate, since timer is 5 seconds
                    self.get_logger().debug(f"Camera {device}: {fps:.1f} fps")
                    # Reset counter
                    self.frame_counts[device] = 0
                    
        except Exception as e:
            self.get_logger().error(f"Error in camera_health_check: {str(e)}")
            self.log_to_file(f"Error in camera_health_check: {str(e)}\n{traceback.format_exc()}", level='error')
    
    def restart_cameras_callback(self, request, response):
        """
        Service callback untuk restart semua kamera.
        
        Args:
            request (Trigger.Request): Request message (empty)
            response (Trigger.Response): Response message (success flag and message)
            
        Returns:
            response: Trigger.Response dengan hasil operasi
        """
        try:
            self.get_logger().info("Restarting all cameras")
            self.log_to_file("Restarting all cameras", level='info')
            
            # Close all cameras
            with self.lock:
                for device, cap in self.caps.items():
                    try:
                        cap.release()
                    except Exception as e:
                        self.get_logger().warn(f"Error closing camera {device}: {str(e)}")
                self.caps.clear()
            
            # Reset all tracking variables
            self.camera_active = {}
            self.frame_counts = {}
            self.failed_reads = {}
            self.retry_counts = {}
            self.last_frame_time = {}
            
            # Reopen all cameras
            for camera in self.camera_configs:
                if not camera.get('enable', True):
                    continue
                    
                device = camera['device']
                width = camera['width']
                height = camera['height']
                fps = camera['fps']
                
                self.open_camera(device, width, height, fps)
            
            # Count successful reopens
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len([c for c in self.camera_configs if c.get('enable', True)])
            
            if active_count == total_count:
                response.success = True
                response.message = f"Successfully restarted all {active_count} cameras"
            else:
                response.success = False
                response.message = f"Partially restarted cameras: {active_count}/{total_count} active"
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in restart_cameras_callback: {str(e)}")
            self.log_to_file(f"Error in restart_cameras_callback: {str(e)}\n{traceback.format_exc()}", level='error')
            
            response.success = False
            response.message = f"Failed to restart cameras: {str(e)}"
            return response
    
    def get_status_callback(self, request, response):
        """
        Service callback untuk mendapatkan status semua kamera.
        
        Args:
            request (Trigger.Request): Request message (empty)
            response (Trigger.Response): Response message (success flag and message)
            
        Returns:
            response: Trigger.Response dengan hasil operasi
        """
        try:
            # Count active cameras
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len([c for c in self.camera_configs if c.get('enable', True)])
            
            # Compile status for each camera
            status_lines = [f"Camera Status: {active_count}/{total_count} active"]
            
            for camera in self.camera_configs:
                if not camera.get('enable', True):
                    continue
                    
                device = camera['device']
                topic = camera['topic']
                
                if device in self.camera_active:
                    status = "ACTIVE" if self.camera_active[device] else "INACTIVE"
                    failures = self.failed_reads.get(device, 0)
                    retries = self.retry_counts.get(device, 0)
                    
                    status_lines.append(f"- {device} -> {topic}: {status} (failures: {failures}, retries: {retries})")
                else:
                    status_lines.append(f"- {device} -> {topic}: NOT INITIALIZED")
            
            # Set response
            response.success = True
            response.message = "\n".join(status_lines)
            
            # Also log status
            self.get_logger().info("\n" + response.message)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in get_status_callback: {str(e)}")
            self.log_to_file(f"Error in get_status_callback: {str(e)}\n{traceback.format_exc()}", level='error')
            
            response.success = False
            response.message = f"Failed to get camera status: {str(e)}"
            return response
    
    def ensure_log_directory(self):
        """Create log directory if it doesn't exist."""
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                self.get_logger().info(f"Created log directory: {self.log_dir}")
        except Exception as e:
            self.get_logger().warn(f"Failed to create log directory {self.log_dir}: {str(e)}")
            self.get_logger().warn("Using /tmp for logs")
            self.log_dir = '/tmp'
    
    def log_to_file(self, msg, level='info'):
        """
        Log message ke file.
        
        Args:
            msg (str): Message untuk log
            level (str): Level log ('debug', 'info', 'warn', 'error')
        """
        try:
            if not self.log_file_path:
                return
                
            with open(self.log_file_path, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
        except Exception as e:
            self.get_logger().warn(f"Failed to write to log file: {str(e)}")
    
    def cleanup(self):
        """Clean up resources saat shutdown."""
        try:
            self.get_logger().info("Cleaning up resources...")
            
            # Release all camera captures
            with self.lock:
                for device, cap in self.caps.items():
                    try:
                        if cap and isinstance(cap, cv2.VideoCapture):
                            cap.release()
                            self.get_logger().info(f"Released camera: {device}")
                    except Exception as e:
                        self.get_logger().warn(f"Error releasing camera {device}: {str(e)}")
            
            self.get_logger().info("Cleanup complete")
            self.log_to_file("Node shutdown cleanly", level='info')
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
            self.log_to_file(f"Error during cleanup: {str(e)}", level='error')
    
    def __del__(self):
        """Destructor to ensure camera release."""
        self.cleanup()


def main(args=None):
    """
    Fungsi main untuk entrypoint node ROS2.
    
    Args:
        args: Command line arguments
    """
    try:
        # Initialize ROS
        rclpy.init(args=args)
        
        # Create node
        node = CameraPublisher()
        
        # Use MultiThreadedExecutor for better performance with multiple cameras
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            node.get_logger().info("MultiCamera Publisher running...")
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received, shutting down...")
        except Exception as e:
            node.get_logger().error(f"Error in main executor: {str(e)}")
            node.log_to_file(f"Error in main executor: {str(e)}\n{traceback.format_exc()}", level='error')
        finally:
            # Explicit cleanup
            node.cleanup()
            # Destroy the node explicitly
            node.destroy_node()
            # Shutdown ROS
            rclpy.shutdown()
            
    except Exception as e:
        if rclpy.ok():
            import traceback
            print(f"Error starting node: {str(e)}")
            print(traceback.format_exc())
            rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()