#!/usr/bin/env python3
# Node publisher multicamera Arducam IMX477 untuk Huskybot 360°
# Kompatibel: ROS2 Humble, Gazebo, Jetson AGX Orin, Clearpath Husky A200

import os  # Untuk operasi file, path, dan environment
import sys  # Untuk akses exit code dan sys.argv
import time  # Untuk sleep, timestamp, dan timer
import threading  # Untuk thread safety (lock)
import traceback  # Untuk stack trace detail saat exception
from datetime import datetime  # Untuk timestamp log/diagnostik
from typing import Dict, List, Tuple, Optional, Any  # Type hints untuk dokumentasi kode

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Class Node untuk membuat node ROS2
from rclpy.parameter import Parameter  # Untuk deklarasi dan validasi parameter
from rclpy.exceptions import ParameterNotDeclaredException  # Exception untuk parameter tidak ditemukan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy  # QoS untuk konfigurasi komunikasi
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Untuk deskripsi parameter
from rclpy.executors import MultiThreadedExecutor  # Executor multi-thread untuk performa multicam

from sensor_msgs.msg import Image  # Message type Image untuk publish gambar
from sensor_msgs.msg import CameraInfo  # Message type CameraInfo untuk publish info kamera
from std_msgs.msg import Header  # Header untuk timestamp message
from std_srvs.srv import Trigger  # Service type Trigger untuk restart/status kamera
from cv_bridge import CvBridge  # CvBridge untuk konversi OpenCV <-> ROS images

import cv2  # OpenCV untuk akses kamera dan image processing
import numpy as np  # Numpy untuk operasi array

# ===================== KONFIGURASI DEFAULT =====================
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
    - Publishes: image_raw (sensor_msgs/Image), camera_info (sensor_msgs/CameraInfo)
    - Services: ~/restart_cameras, ~/get_status
    - Parameters: lihat README.md
    """

    def __init__(self):
        """Inisialisasi node kamera, setup publisher, services, dan parameter."""
        super().__init__('multicamera_publisher')  # Inisialisasi node ROS2
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Direktori log default
        self.ensure_log_directory()  # Pastikan folder log ada
        self.get_logger().info("Initializing MultiCamera Publisher Node...")  # Log info inisialisasi

        self.lock = threading.RLock()  # Thread lock untuk thread safety
        self.declare_parameters()  # Deklarasi semua parameter
        self.load_parameters()  # Load semua parameter dari ROS parameter server
        self.bridge = CvBridge()  # Converter OpenCV <-> ROS

        # Inisialisasi container kamera dan publisher
        self.caps = {}  # Dict untuk objek kamera (cv2.VideoCapture)
        self.publishers = {}  # Dict untuk image publishers
        self.info_publishers = {}  # Dict untuk camera info publishers
        self.camera_active = {}  # Status kamera aktif
        self.frame_counts = {}  # Counter frames per kamera
        self.last_frame_time = {}  # Timestamp frame terakhir
        self.failed_reads = {}  # Counter kegagalan baca
        self.retry_counts = {}  # Counter retries
        self.camera_configs = []  # List konfigurasi kamera

        # Setup services
        self.restart_srv = self.create_service(Trigger, 'restart_cameras', self.restart_cameras_callback)
        self.status_srv = self.create_service(Trigger, 'get_status', self.get_status_callback)

        self.read_camera_config()  # Baca konfigurasi kamera dari file/parameter
        self.setup_cameras_and_publishers()  # Setup kamera dan publisher
        self.create_timers()  # Buat timer untuk publish images
        self.health_check_timer = self.create_timer(5.0, self.camera_health_check)  # Timer health check

        self.get_logger().info("MultiCamera Publisher Node initialized successfully!")
        self.log_to_file("MultiCamera Publisher Node initialized successfully!", level='info')

    def declare_parameters(self):
        """Deklarasi semua parameter dengan deskripsi dan tipe yang jelas."""
        try:
            # Parameter deskriptor
            path_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Path to a file or directory')
            bool_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='Boolean flag')
            int_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Integer value')
            float_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Floating point value')

            self.declare_parameter('config_file', '', path_descriptor)
            self.declare_parameter('use_sim_time', False, bool_descriptor)
            self.declare_parameter('retry_count', 5, int_descriptor)
            self.declare_parameter('retry_delay', 2.0, float_descriptor)
            self.declare_parameter('publish_rate', 20.0, float_descriptor)
            self.declare_parameter('publish_camera_info', False, bool_descriptor)
            self.declare_parameter('log_file', os.path.join(self.log_dir, 'multicamera.log'), path_descriptor)
            self.declare_parameter('fallback_to_video_files', False, bool_descriptor)
            self.declare_parameter('video_file_dir', '', path_descriptor)
            # Parameter per kamera
            for i, (topic, device, frame_id, width, height, fps) in enumerate(DEFAULT_CAMERA_CONFIG):
                self.declare_parameter(f'camera{i+1}.enable', True, bool_descriptor)
                self.declare_parameter(f'camera{i+1}.topic', topic, path_descriptor)
                self.declare_parameter(f'camera{i+1}.device', device, path_descriptor)
                self.declare_parameter(f'camera{i+1}.frame_id', frame_id, path_descriptor)
                self.declare_parameter(f'camera{i+1}.width', width, int_descriptor)
                self.declare_parameter(f'camera{i+1}.height', height, int_descriptor)
                self.declare_parameter(f'camera{i+1}.fps', fps, float_descriptor)
                self.declare_parameter(f'camera{i+1}.flip', False, bool_descriptor)
        except Exception as e:
            self.get_logger().error(f"Error declaring parameters: {str(e)}")
            self.log_to_file(f"Error declaring parameters: {str(e)}", level='error')
            raise

    def load_parameters(self):
        """Load semua parameter dari ROS parameter server."""
        try:
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
            self.camera_configs = []
            if self.config_file and os.path.isfile(self.config_file):
                self.get_logger().info(f"Reading camera configuration from file: {self.config_file}")
                self.log_to_file(f"Reading camera configuration from file: {self.config_file}", level='info')
                import yaml
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
                if not isinstance(config, dict) or 'cameras' not in config:
                    raise ValueError("Config file tidak valid, harus ada key 'cameras'")
                for cam in config['cameras']:
                    self.camera_configs.append({
                        'topic': cam.get('topic', ''),
                        'device': cam.get('device', ''),
                        'frame_id': cam.get('frame_id', ''),
                        'width': cam.get('width', 1920),
                        'height': cam.get('height', 1080),
                        'fps': cam.get('fps', 30.0),
                        'flip': cam.get('flip', False),
                        'enable': cam.get('enable', True)
                    })
            else:
                self.get_logger().info("Reading camera configuration from ROS parameters")
                self.log_to_file("Reading camera configuration from ROS parameters", level='info')
                for i, (default_topic, default_device, default_frame_id, default_width, default_height, default_fps) in enumerate(DEFAULT_CAMERA_CONFIG):
                    enable = self.get_parameter(f'camera{i+1}.enable').value
                    if not enable:
                        continue
                    self.camera_configs.append({
                        'topic': self.get_parameter(f'camera{i+1}.topic').value,
                        'device': self.get_parameter(f'camera{i+1}.device').value,
                        'frame_id': self.get_parameter(f'camera{i+1}.frame_id').value,
                        'width': self.get_parameter(f'camera{i+1}.width').value,
                        'height': self.get_parameter(f'camera{i+1}.height').value,
                        'fps': self.get_parameter(f'camera{i+1}.fps').value,
                        'flip': self.get_parameter(f'camera{i+1}.flip').value,
                        'enable': enable
                    })
            if not self.camera_configs:
                self.get_logger().error("No cameras configured! Please check your configuration.")
                self.log_to_file("No cameras configured! Please check your configuration.", level='error')
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
            self.get_logger().info(f"Final camera configuration: {len(self.camera_configs)} cameras")
            for i, cam in enumerate(self.camera_configs):
                self.get_logger().info(f"Camera {i+1}: {cam['device']} -> {cam['topic']} ({cam['width']}x{cam['height']} @ {cam['fps']} fps)")
        except Exception as e:
            self.get_logger().error(f"Error reading camera configuration: {str(e)}")
            self.log_to_file(f"Error reading camera configuration: {str(e)}\n{traceback.format_exc()}", level='error')
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
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1
                )
                self.publishers[device] = self.create_publisher(Image, topic, qos)
                if self.publish_camera_info:
                    self.info_publishers[device] = self.create_publisher(CameraInfo, topic.replace('image_raw', 'camera_info'), qos)
                self.frame_counts[device] = 0
                self.failed_reads[device] = 0
                self.retry_counts[device] = 0
                self.last_frame_time[device] = time.time()
                self.open_camera(device, width, height, fps)
        except Exception as e:
            self.get_logger().error(f"Error setting up cameras: {str(e)}")
            self.log_to_file(f"Error setting up cameras: {str(e)}\n{traceback.format_exc()}", level='error')

    def open_camera(self, device, width, height, fps):
        """Buka koneksi ke kamera tertentu dengan retry hingga retry_count kali."""
        retry = 0
        device_path = device
        if device.startswith('csi://'):
            try:
                cam_id = int(device.replace('csi://', ''))
                device_path = cam_id
            except ValueError:
                self.get_logger().error(f"Invalid CSI device format: {device}, expected 'csi://N'")
                self.log_to_file(f"Invalid CSI device format: {device}, expected 'csi://N'", level='error')
                self.camera_active[device] = False
                return False
        while retry <= self.retry_count:
            try:
                self.get_logger().info(f"Attempting to open camera {device} (retry {retry}/{self.retry_count})")
                cap = cv2.VideoCapture(device_path)
                if not cap.isOpened():
                    raise RuntimeError(f"Camera {device} failed to open")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                ret, test_frame = cap.read()
                if not ret or test_frame is None:
                    raise RuntimeError(f"Camera {device} failed to read test frame")
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
                with self.lock:
                    self.caps[device] = cap
                    self.camera_active[device] = True
                return True
            except Exception as e:
                self.get_logger().error(f"Error opening camera {device}: {str(e)}")
                self.log_to_file(f"Error opening camera {device}: {str(e)}\n{traceback.format_exc()}", level='error')
                retry += 1
                time.sleep(self.retry_delay)
        self.camera_active[device] = False
        return False

    def create_timers(self):
        """Buat timer untuk publish images dari semua kamera."""
        try:
            timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05
            self.timer = self.create_timer(timer_period, self.publish_all_images)
        except Exception as e:
            self.get_logger().error(f"Error creating timer: {str(e)}")
            self.log_to_file(f"Error creating timer: {str(e)}", level='error')

    def publish_all_images(self):
        """Publish images dari semua kamera yang aktif."""
        try:
            for camera in self.camera_configs:
                if not camera.get('enable', True):
                    continue
                device = camera['device']
                topic = camera['topic']
                frame_id = camera['frame_id']
                flip = camera.get('flip', False)
                if device not in self.caps or not self.camera_active.get(device, False):
                    continue
                cap = self.caps[device]
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.failed_reads[device] += 1
                    self.get_logger().warning(f"Failed to read frame from {device}")
                    self.log_to_file(f"Failed to read frame from {device}", level='warning')
                    if self.failed_reads[device] > self.retry_count:
                        self.get_logger().warning(f"Camera {device} failed too many times, attempting to reopen...")
                        self.open_camera(device, camera['width'], camera['height'], camera['fps'])
                    continue
                if flip:
                    frame = cv2.flip(frame, 1)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
                self.publishers[device].publish(msg)
                self.frame_counts[device] += 1
                self.last_frame_time[device] = time.time()
                # Publish camera_info jika diperlukan
                if self.publish_camera_info and device in self.info_publishers:
                    info_msg = CameraInfo()
                    info_msg.header = msg.header
                    info_msg.width = camera['width']
                    info_msg.height = camera['height']
                    self.info_publishers[device].publish(info_msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_all_images: {str(e)}")
            self.log_to_file(f"Error in publish_all_images: {str(e)}\n{traceback.format_exc()}", level='error')

    def camera_health_check(self):
        """Periodic health check untuk semua kamera."""
        try:
            current_time = time.time()
            for device, last_time in self.last_frame_time.items():
                if not self.camera_active.get(device, False):
                    self.get_logger().warning(f"Camera {device} is not active")
                elif current_time - last_time > 2.0:
                    self.get_logger().warning(f"Camera {device} has not published for {current_time - last_time:.2f} seconds")
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len(self.camera_active)
            self.get_logger().info(f"Camera status: {active_count}/{total_count} active")
            for device, count in self.frame_counts.items():
                self.get_logger().info(f"Camera {device}: {count} frames published")
        except Exception as e:
            self.get_logger().error(f"Error in camera_health_check: {str(e)}")
            self.log_to_file(f"Error in camera_health_check: {str(e)}\n{traceback.format_exc()}", level='error')

    def restart_cameras_callback(self, request, response):
        """Service callback untuk restart semua kamera."""
        try:
            self.get_logger().info("Restarting all cameras")
            self.log_to_file("Restarting all cameras", level='info')
            with self.lock:
                for cap in self.caps.values():
                    cap.release()
                self.caps = {}
            self.camera_active = {}
            self.frame_counts = {}
            self.failed_reads = {}
            self.retry_counts = {}
            self.last_frame_time = {}
            for camera in self.camera_configs:
                self.open_camera(camera['device'], camera['width'], camera['height'], camera['fps'])
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len([c for c in self.camera_configs if c.get('enable', True)])
            if active_count == total_count:
                response.success = True
                response.message = "All cameras restarted successfully"
            else:
                response.success = False
                response.message = f"Only {active_count}/{total_count} cameras restarted successfully"
            return response
        except Exception as e:
            self.get_logger().error(f"Error in restart_cameras_callback: {str(e)}")
            self.log_to_file(f"Error in restart_cameras_callback: {str(e)}\n{traceback.format_exc()}", level='error')
            response.success = False
            response.message = f"Failed to restart cameras: {str(e)}"
            return response

    def get_status_callback(self, request, response):
        """Service callback untuk mendapatkan status semua kamera."""
        try:
            active_count = sum(1 for v in self.camera_active.values() if v)
            total_count = len([c for c in self.camera_configs if c.get('enable', True)])
            status_lines = [f"Camera Status: {active_count}/{total_count} active"]
            for camera in self.camera_configs:
                device = camera['device']
                status = "ACTIVE" if self.camera_active.get(device, False) else "INACTIVE"
                status_lines.append(f"{device}: {status}")
            response.success = True
            response.message = "\n".join(status_lines)
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
        except Exception as e:
            self.get_logger().warn(f"Failed to create log directory {self.log_dir}: {str(e)}")
            self.get_logger().warn("Using /tmp for logs")
            self.log_dir = '/tmp'

    def log_to_file(self, msg, level='info'):
        """Log message ke file."""
        try:
            if not hasattr(self, 'log_file_path') or not self.log_file_path:
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
            with self.lock:
                for cap in self.caps.values():
                    cap.release()
                self.caps = {}
            self.get_logger().info("Cleanup complete")
            self.log_to_file("Node shutdown cleanly", level='info')
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
            self.log_to_file(f"Error during cleanup: {str(e)}", level='error')

    def __del__(self):
        """Destructor to ensure camera release."""
        self.cleanup()

def main(args=None):
    """Fungsi main untuk entrypoint node ROS2."""
    try:
        rclpy.init(args=args)
        node = CameraPublisher()
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
            node.cleanup()
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        if rclpy.ok():
            print(f"Error starting node: {str(e)}")
            print(traceback.format_exc())
            rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()