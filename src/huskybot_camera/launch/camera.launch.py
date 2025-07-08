#!/usr/bin/env python3
# Launch file untuk menjalankan 6 kamera Arducam IMX477 (hexagonal) pada Huskybot  # Penjelasan fungsi file
# Kompatibel: ROS2 Humble, Gazebo, Jetson AGX Orin, Clearpath Husky A200  # Kompatibilitas hardware/software

import os  # Untuk operasi file dan path
import sys  # Untuk akses exit code dan sys.argv
import glob  # Untuk pencarian file dengan pattern
import platform  # Untuk deteksi sistem operasi/hardware
from pathlib import Path  # Untuk operasi path cross-platform
from datetime import datetime  # Untuk timestamp log
import subprocess  # Untuk menjalankan command shell
import time  # Untuk delay dan timeout
import traceback  # Untuk stack trace error

from launch import LaunchDescription  # Kelas utama launch file ROS2
from launch_ros.actions import Node  # Untuk membuat node ROS2
from launch.substitutions import LaunchConfiguration  # Untuk parameter dinamis
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, ExecuteProcess, TimerAction  # Untuk deklarasi argumen dan event handler
from launch.conditions import IfCondition  # Untuk kondisi eksekusi node
from launch.events import Shutdown  # Untuk event shutdown
from launch_ros.substitutions import FindPackageShare  # Untuk mencari path package
from launch.event_handlers import OnShutdown, OnProcessExit  # Untuk event handler shutdown/exit
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource  # Untuk include launch XML
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError  # Untuk validasi package

class CameraLaunchConfig:
    """Kelas OOP untuk mengelola konfigurasi launch file kamera."""

    def __init__(self, use_sim_time=False, namespace=''):
        """Inisialisasi konfigurasi launch kamera."""
        self.use_sim_time = use_sim_time  # Flag waktu simulasi
        self.namespace = namespace  # Namespace multi-robot
        self.start_time = datetime.now().strftime("%Y%m%d_%H%M%S")  # Timestamp unik
        # Mapping device ke topic dan frame_id
        self.camera_remap = [
            ('csi://0', '/camera_front/image_raw', 'camera_front_optical_frame'),  # Kamera depan
            ('csi://1', '/camera_front_left/image_raw', 'camera_front_left_optical_frame'),  # Kamera depan kiri
            ('csi://2', '/camera_left/image_raw', 'camera_left_optical_frame'),  # Kamera kiri
            ('csi://3', '/camera_rear/image_raw', 'camera_rear_optical_frame'),  # Kamera belakang
            ('csi://4', '/camera_rear_right/image_raw', 'camera_rear_right_optical_frame'),  # Kamera belakang kanan
            ('csi://5', '/camera_right/image_raw', 'camera_right_optical_frame'),  # Kamera kanan
        ]
        self.default_width = '1920'  # Default resolusi width
        self.default_height = '1080'  # Default resolusi height
        self.default_framerate = '30.0'  # Default framerate
        self.default_codec = 'unknown'  # Codec default
        self.default_latency = 2000  # Latency buffer ms, HARUS integer untuk ros_deep_learning
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Folder log
        self.fallback_log_dir = '/tmp' if platform.system() == 'Linux' else os.path.expanduser('~')  # Fallback log dir
        try:
            if not os.path.exists(self.log_dir):  # Cek folder log
                os.makedirs(self.log_dir)  # Buat folder log jika belum ada
                self.log_to_file("Direktori log dibuat", level='info')
        except Exception as e:
            self.log_to_file(f"Gagal membuat direktori log: {e}", level='warning')
            self.log_dir = self.fallback_log_dir  # Fallback ke /tmp jika gagal
        self.is_jetson = self._detect_jetson()  # Deteksi Jetson
        if self.is_jetson:
            self.log_to_file("Terdeteksi platform Jetson, optimasi untuk CSI cameras diaktifkan", level='info')
        self.validate_core_packages()  # Validasi package penting

    def _detect_jetson(self):
        """Deteksi apakah berjalan di Nvidia Jetson."""
        try:
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'NVIDIA' in model and ('Jetson' in model or 'AGX' in model or 'Orin' in model):
                        self.log_to_file(f"Detected Jetson device: {model.strip()}", level='info')
                        return True
            if 'aarch64' in platform.machine() and 'tegra' in platform.release().lower():
                self.log_to_file("Detected Jetson platform via aarch64/tegra in system info", level='info')
                return True
            for cuda_path in ['/usr/local/cuda', '/usr/local/cuda-*']:
                if glob.glob(cuda_path):
                    self.log_to_file(f"Detected CUDA installation at {cuda_path}, assuming Jetson", level='info')
                    return True
            try:
                result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=2)
                if 'Jetson' in result.stdout or 'AGX' in result.stdout or 'Orin' in result.stdout:
                    self.log_to_file("Detected Jetson platform via nvidia-smi", level='info')
                    return True
            except (subprocess.SubprocessError, OSError, TimeoutError):
                pass
            if os.path.exists('/etc/nv_tegra_release'):
                self.log_to_file("Detected /etc/nv_tegra_release file, confirming Jetson platform", level='info')
                return True
            return False
        except Exception as e:
            self.log_to_file(f"Error saat deteksi platform Jetson: {e}", level='warning')
            return False

    def validate_core_packages(self):
        """Validasi ketersediaan package ROS2 penting."""
        critical_packages = ['ros_deep_learning', 'cv_bridge', 'sensor_msgs', 'std_msgs']
        missing_packages = []
        for package in critical_packages:
            if not self.validate_dependency(package):
                missing_packages.append(package)
        if missing_packages:
            self.log_to_file(f"PERINGATAN: Package penting tidak ditemukan: {', '.join(missing_packages)}", level='warning')
            self.log_to_file("Beberapa fitur mungkin tidak berfungsi dengan benar", level='warning')

    def validate_dependency(self, package_name):
        """Validasi package dependency ada dan terinstall."""
        try:
            get_package_share_directory(package_name)
            return True
        except PackageNotFoundError as e:
            error_msg = f"ERROR: Package {package_name} tidak ditemukan: {e}"
            print(error_msg)
            print(f"ERROR: Install dengan: sudo apt install ros-humble-{package_name.replace('_', '-')}")
            self.log_to_file(error_msg, level='error')
            self.log_to_file(f"Install dengan: sudo apt install ros-humble-{package_name.replace('_', '-')}", level='info')
            return False
        except Exception as e:
            self.log_to_file(f"Error validating package {package_name}: {e}", level='error')
            return False

    def check_camera_device(self, device):
        """Cek apakah device kamera bisa diakses (log warning jika tidak bisa)."""
        if device.startswith("csi://"):
            if self.is_jetson:
                try:
                    cam_id = int(device.replace('csi://', ''))
                except (ValueError, IndexError) as e:
                    self.log_to_file(f"CSI device format error: {e}", level='warning')
            return True
        if device.startswith("/dev/video"):
            if not os.path.exists(device):
                warn_msg = f"WARNING: Device kamera {device} tidak ditemukan"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
            if not os.access(device, os.R_OK):
                warn_msg = f"WARNING: Tidak ada permission untuk membaca device {device}"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                print(f"TIP: Jalankan 'sudo chmod a+r {device}' atau tambahkan user ke group 'video'")
                return False
            try:
                result = subprocess.run(['lsof', device], capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_to_file(f"Device {device} sedang digunakan oleh proses lain", level='warning')
            except (subprocess.SubprocessError, OSError):
                pass
        if device.startswith(('rtsp://', 'http://', 'https://')):
            self.log_to_file(f"INFO: Menggunakan URL camera: {device}", level='info')
        if device.endswith(('.mp4', '.avi', '.mkv', '.mov')):
            if not os.path.isfile(device):
                warn_msg = f"WARNING: File video {device} tidak ditemukan"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
            if not os.access(device, os.R_OK):
                warn_msg = f"WARNING: File video {device} tidak dapat dibaca"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
            try:
                import cv2
                cap = cv2.VideoCapture(device)
                if not cap.isOpened():
                    self.log_to_file(f"WARNING: File video {device} tidak bisa dibuka oleh OpenCV", level='warning')
                cap.release()
            except Exception as e:
                self.log_to_file(f"WARNING: Gagal memvalidasi file video {device}: {e}", level='warning')
        return True

    def generate_camera_args(self):
        """Generate launch arguments untuk semua kamera."""
        args = []
        args.append(DeclareLaunchArgument('use_sim_time', default_value='false', description='Gunakan waktu simulasi (true untuk Gazebo, false untuk hardware real)'))  # Argumen waktu simulasi
        args.append(DeclareLaunchArgument('namespace', default_value='', description='Namespace untuk multi-robot deployment'))  # Argumen namespace
        args.append(DeclareLaunchArgument('respawn_cameras', default_value='true', description='Auto-respawn camera nodes jika crash (true/false)'))  # Argumen auto-respawn
        args.append(DeclareLaunchArgument('log_level', default_value='info', description='Log level untuk nodes (debug|info|warn|error|fatal)'))  # Argumen log level
        args.append(DeclareLaunchArgument('log_file_path', default_value=os.path.join(self.log_dir, f'camera_{self.start_time}.log'), description='Path untuk file log kamera'))  # Argumen path log
        args.append(DeclareLaunchArgument('camera_logger_enabled', default_value='true', description='Enable/disable camera logger node'))  # Argumen logger
        args.append(DeclareLaunchArgument('capture_method', default_value='gstreamer' if self.is_jetson else 'opencv', description='Metode capture kamera (gstreamer, opencv, v4l2)'))  # Argumen capture method
        args.append(DeclareLaunchArgument('enable_yolo_integration', default_value='true', description='Enable integrasi langsung dengan node YOLOv12 detection/segmentation'))  # Argumen YOLO
        args.append(DeclareLaunchArgument('yolo_model_type', default_value='detection', description='Tipe model YOLOv12 (detection, segmentation, obb)'))  # Argumen tipe YOLO
        args.append(DeclareLaunchArgument('diagnostics_enabled', default_value='true', description='Enable diagnostics untuk monitoring kamera'))  # Argumen diagnostics
        args.append(DeclareLaunchArgument('camera_mode', default_value='high_quality', description='Mode kamera (high_quality, balanced, high_fps)'))  # Argumen mode kamera
        for i, (dev, topic, frame_id) in enumerate(self.camera_remap, start=1):
            args.append(DeclareLaunchArgument(f'camera{i}_enable', default_value='true', description=f'Enable/disable kamera {i} (true/false)'))  # Enable kamera
            args.append(DeclareLaunchArgument(f'camera{i}_device', default_value=dev, description=f'Device kamera {i} (misal: {dev})'))  # Device kamera
            args.append(DeclareLaunchArgument(f'camera{i}_topic', default_value=topic, description=f'Topic output kamera {i} (misal: {topic})'))  # Topic kamera
            args.append(DeclareLaunchArgument(f'camera{i}_frame_id', default_value=frame_id, description=f'Frame ID kamera {i} untuk TF dan visualisasi'))  # Frame ID kamera
            args.append(DeclareLaunchArgument(f'camera{i}_width', default_value=self.default_width, description=f'Resolution width kamera {i}'))  # Width kamera
            args.append(DeclareLaunchArgument(f'camera{i}_height', default_value=self.default_height, description=f'Resolution height kamera {i}'))  # Height kamera
            args.append(DeclareLaunchArgument(f'camera{i}_framerate', default_value=self.default_framerate, description=f'Framerate kamera {i}'))  # Framerate kamera
            args.append(DeclareLaunchArgument(f'camera{i}_flip', default_value='', description=f'Flip image kamera {i} (opsi: \"\", \"horizontal\", \"vertical\", \"both\")'))  # Flip kamera
            args.append(DeclareLaunchArgument(f'camera{i}_quality', default_value='85', description=f'JPEG compression quality kamera {i} (0-100)'))  # Quality kamera
            args.append(DeclareLaunchArgument(f'camera{i}_exposure', default_value='-1', description=f'Exposure setting untuk kamera {i} (-1=auto)'))  # Exposure kamera
            args.append(DeclareLaunchArgument(f'camera{i}_white_balance', default_value='-1', description=f'White balance untuk kamera {i} (-1=auto)'))  # White balance kamera
            args.append(DeclareLaunchArgument(f'camera{i}_gain', default_value='-1', description=f'Gain setting untuk kamera {i} (-1=auto)'))  # Gain kamera
            args.append(DeclareLaunchArgument(f'camera{i}_calib_file', default_value='', description=f'Path ke file kalibrasi untuk kamera {i} (kosong=tanpa kalibrasi)'))  # Kalibrasi kamera
        return args

    def generate_camera_nodes(self):
        """Generate node ROS untuk semua kamera."""
        nodes = []
        
        # PERBAIKAN: Topic mapping yang benar sesuai dengan ros_deep_learning
        camera_mappings = [
            ('csi://0', '/camera_front/image_raw', 'camera_front_optical_frame'),
            ('csi://1', '/camera_right/image_raw', 'camera_right_optical_frame'), 
            ('csi://2', '/camera_rear_right/image_raw', 'camera_rear_right_optical_frame'),
            ('csi://3', '/camera_rear/image_raw', 'camera_rear_optical_frame'),
            ('csi://4', '/camera_left/image_raw', 'camera_left_optical_frame'),
            ('csi://5', '/camera_front_left/image_raw', 'camera_front_left_optical_frame')
        ]
        
        if not self.validate_dependency('ros_deep_learning'):
            self.log_to_file("ERROR: Package ros_deep_learning tidak ditemukan!", level='error')
            self.log_to_file("Mencoba fallback ke multicamera_publisher native", level='info')
            nodes.append(
                Node(
                    package='huskybot_camera',
                    executable='multicamera_publisher',
                    name='camera_fallback',
                    output='both',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'log_level': LaunchConfiguration('log_level', default='info'),
                        'log_file': LaunchConfiguration('log_file_path'),
                        'auto_reconnect': True,
                        'retry_count': 5,
                        'retry_delay': 2.0,
                        'camera_count': len(self.camera_remap),
                    }],
                    on_exit=[LogInfo(msg=["Node fallback camera berhenti dengan exit code: ${}.returncode"])]
                )
            )
            return nodes
        
        for i, (device, topic, frame_id) in enumerate(camera_mappings, start=1):
            camera_enable_condition = IfCondition(LaunchConfiguration(f'camera{i}_enable', default='true'))
            
            nodes.append(
                Node(
                    condition=camera_enable_condition,
                    package='ros_deep_learning',
                    executable='video_source',
                    name=f'camera_{i}',  # Unique name
                    output='both',
                    namespace=LaunchConfiguration('namespace', default=''),
                    respawn=LaunchConfiguration('respawn_cameras', default='true'),
                    respawn_delay=3.0,  # Longer delay for stability
                    parameters=[{
                        'resource': device,
                        'width': LaunchConfiguration(f'camera{i}_width', default='1920'),
                        'height': LaunchConfiguration(f'camera{i}_height', default='1080'),
                        'framerate': LaunchConfiguration(f'camera{i}_framerate', default='30.0'),
                        'codec': 'raw',
                        'loop': 0,
                        'latency': 200,  # Increase latency buffer
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'flip': LaunchConfiguration(f'camera{i}_flip', default='rotate-180'),
                        'frame_id': frame_id,
                    }],
                    remappings=[
                        # PERBAIKAN: Correct remapping for ros_deep_learning
                        ('video_source/raw', topic),
                        ('raw', topic),  # Additional fallback
                    ],
                    on_exit=[LogInfo(msg=[
                        f"Camera {i} ({device} -> {topic}) exited with code: ", "${}"
                    ])]
                )
            )
        return nodes

    def log_to_file(self, msg, level='info'):
        """Log pesan ke file log."""
        try:
            log_file = os.path.join(self.log_dir, f'camera_launch_{self.start_time}.log')
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            with open(log_file, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
        except Exception as e:
            print(f"WARNING: Gagal menulis ke log file: {e}")
            print(f"WARNING: Log message: [{level.upper()}] {msg}")

    def validate_camera_frames(self):
        """Validasi frame ID untuk integrasi dengan TF."""
        all_valid = True
        tf_base_frame = 'base_link'
        for _, _, frame_id in self.camera_remap:
            if not frame_id.endswith('_optical_frame') and not frame_id.endswith('_link'):
                self.log_to_file(f"WARNING: Frame ID {frame_id} tidak sesuai konvensi", level='warning')
                all_valid = False
            tf_path = f"{tf_base_frame} -> ... -> {frame_id}"
            self.log_to_file(f"INFO: Expected TF path: {tf_path}", level='info')
        return all_valid

    def check_tf_tree(self):
        """Validasi TF tree ada dan terhubung dengan benar."""
        try:
            result = subprocess.run(['ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'camera_front_optical_frame'],
                                    capture_output=True, text=True, timeout=1)
            if "Could not transform" in result.stderr:
                self.log_to_file("WARNING: TF tree tidak lengkap. Frame camera_front_optical_frame tidak terhubung dengan base_link", level='warning')
                self.log_to_file("TIP: Pastikan robot_state_publisher berjalan dan URDF memiliki transformasi yang benar", level='info')
                return False
            return True
        except (subprocess.SubprocessError, OSError, TimeoutError):
            self.log_to_file("WARNING: Tidak dapat memeriksa TF tree", level='warning')
            return True

def generate_launch_description():
    """Generate launch description untuk kamera dengan mapping yang BENAR."""
    config = CameraLaunchConfig()
    config.log_to_file("Launch file camera.launch.py dimulai", level='info')
    
    # PERBAIKAN: Dokumentasi mapping yang benar
    config.log_to_file("PENTING: Mapping kamera sesuai hardware real:", level='info')
    config.log_to_file("- /camera_front/image_raw = KAMERA BELAKANG (180°)", level='info')
    config.log_to_file("- /camera_front_left/image_raw = KAMERA KIRI BELAKANG (225°)", level='info')
    config.log_to_file("- /camera_left/image_raw = KAMERA KIRI DEPAN (270°)", level='info')
    config.log_to_file("- /camera_rear/image_raw = KAMERA DEPAN (0°)", level='info')
    config.log_to_file("- /camera_rear_right/image_raw = KAMERA KANAN DEPAN (315°)", level='info')
    config.log_to_file("- /camera_right/image_raw = KAMERA KANAN BELAKANG (45°)", level='info')
    
    args = config.generate_camera_args()  # Generate argumen kamera
    nodes = config.generate_camera_nodes()  # Generate node kamera
    config.validate_camera_frames()  # Validasi frame kamera
    config.check_tf_tree()  # Validasi TF tree
    for i, (dev, topic, frame_id) in enumerate(config.camera_remap, start=1):
        config.check_camera_device(dev)  # Validasi device kamera
        config.log_to_file(f"Kamera {i}: {dev} -> {topic} [frame: {frame_id}]", level='info')
    test_image_topics = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5; echo "Testing image topics..."; rostopic list | grep -E "/camera_.*_?/image_raw" || echo "WARNING: No camera image topics found!"'],
        name='test_image_topics',
        output='both',
        condition=IfCondition('false'),  # Nonaktifkan test ini, aktifkan jika perlu
    )
    return LaunchDescription(
        [LogInfo(msg=['Starting camera launch file, initializing 6 Arducam IMX477 cameras...'])] +  # Log info awal
        args +  # Semua argumen kamera
        nodes +  # Semua node kamera
        [LogInfo(msg=['Semua node camera telah diluncurkan. Memulai monitoring performance...'])] +  # Log info monitoring
        [test_image_topics] +  # Test topic image (opsional)
        [LogInfo(msg=[f'Camera launch completed with {len(nodes)} nodes. Listening to topics...'])]  # Log info selesai
    )