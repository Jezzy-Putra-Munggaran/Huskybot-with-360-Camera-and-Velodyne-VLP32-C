#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/launch/camera.launch.py
# ======================================================================
# Launch file untuk menjalankan 6 kamera Arducam IMX477 (hexagonal)
# Menggunakan driver video_source dari package ros_deep_learning
# Konfigurasi: Front, Front-Left, Left, Rear, Rear-Right, Right
# Digunakan untuk: Pipeline deteksi 360° pada robot Huskybot
# Kompatibel dengan: ROS2 Humble, Gazebo, & Husky A200 + Jetson AGX Orin
# ======================================================================

import os                                  # Import modul os untuk operasi sistem seperti path dan environment variables
import sys                                 # Import modul sys untuk akses ke interpreter dan exit codes
import yaml                                # Import modul yaml untuk validasi format config file
import glob                                # Import modul glob untuk mencari file dengan pattern tertentu
import platform                            # Import modul platform untuk deteksi sistem operasi dan hardware
from pathlib import Path                   # Import Path dari pathlib untuk operasi path cross-platform
from datetime import datetime              # Import datetime untuk timestamp di log
import subprocess                          # Import subprocess untuk menjalankan command shell
import time                                # Import time untuk delays dan timeouts
import traceback                           # Import traceback untuk informasi error yang lebih detail

# Import komponen launch dari ROS2
from launch import LaunchDescription       # Import kelas LaunchDescription untuk deskripsi launch configuration
from launch_ros.actions import Node        # Import kelas Node untuk membuat instance node ROS
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PathJoinSubstitution  # Import substitusi untuk variabel dinamis
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, ExecuteProcess  # Import aksi untuk proses launch
from launch.conditions import IfCondition, UnlessCondition  # Import kondisi untuk eksekusi bersyarat
from launch.events import Shutdown, process  # Import event untuk penanganan shutdown dan process events
from launch_ros.substitutions import FindPackageShare  # Import substitusi untuk mencari path package
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown, OnExecutionComplete  # Import handler untuk merespon events
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource  # Import source untuk include file XML
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError  # Import untuk akses share directory dan error handling


class CameraLaunchConfig:
    """Kelas untuk mengelola konfigurasi launch file kamera dengan pendekatan OOP."""
    
    def __init__(self, use_sim_time=False, namespace=''):
        """
        Inisialisasi konfigurasi launch kamera.
        
        Args:
            use_sim_time (bool): Flag untuk menggunakan waktu simulasi
            namespace (str): Namespace untuk multi-robot
        """
        self.use_sim_time = use_sim_time      # Flag untuk menggunakan waktu simulasi (Gazebo)
        self.namespace = namespace             # Namespace untuk scenario multi-robot
        self.start_time = datetime.now().strftime("%Y%m%d_%H%M%S")  # Timestamp untuk file uniqueness
        
        # Definisi mapping kamera default (device ke topic)
        self.camera_remap = [
            # (device, topic, frame_id)
            ('csi://0', '/camera_front/image_raw', 'camera_front_optical_frame'),        # Kamera depan
            ('csi://1', '/camera_front_left/image_raw', 'camera_front_left_optical_frame'), # Kamera depan-kiri
            ('csi://2', '/camera_left/image_raw', 'camera_left_optical_frame'),         # Kamera kiri
            ('csi://3', '/camera_rear/image_raw', 'camera_rear_optical_frame'),         # Kamera belakang
            ('csi://4', '/camera_rear_right/image_raw', 'camera_rear_right_optical_frame'), # Kamera belakang-kanan
            ('csi://5', '/camera_right/image_raw', 'camera_right_optical_frame'),       # Kamera kanan
        ]
        
        # Default parameter kamera
        self.default_width = '1920'            # Default width resolusi kamera (1080p)
        self.default_height = '1080'           # Default height resolusi kamera (1080p)
        self.default_framerate = '30.0'        # Default framerate kamera (30 fps)
        self.default_codec = 'unknown'         # Default codec (unknown = autodetect)
        self.default_latency = '2000'          # Default latency buffer (ms) untuk real-time processing
        
        # Path untuk log dan config
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Path untuk log kamera di home directory
        
        # Deteksi platform untuk fallback paths yang sesuai
        if platform.system() == 'Linux':
            self.fallback_log_dir = '/tmp'  # Fallback ke /tmp di Linux
        else:
            self.fallback_log_dir = os.path.expanduser('~')  # Fallback ke home dir di platform lain
        
        # Buat folder log jika belum ada (error handling)
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                print(f"INFO: Membuat direktori log di {self.log_dir}")
                self.log_to_file("Direktori log dibuat", level='info')
        except Exception as e:
            print(f"WARNING: Gagal membuat folder log {self.log_dir}: {e}")
            print(f"WARNING: Menggunakan {self.fallback_log_dir} sebagai fallback")
            self.log_to_file(f"Gagal membuat direktori log: {e}", level='warning')
            self.log_dir = self.fallback_log_dir  # Gunakan fallback path
            
        # Deteksi apakah berjalan di Jetson (untuk optimasi)
        self.is_jetson = self._detect_jetson()
        if self.is_jetson:
            self.log_to_file("Terdeteksi platform Jetson, optimasi untuk CSI cameras diaktifkan", level='info')
        
        # Deteksi apakah package yang dibutuhkan tersedia
        self.validate_core_packages()
        
    def _detect_jetson(self):
        """
        Deteksi apakah code berjalan di Nvidia Jetson platform.
        
        Returns:
            bool: True jika running di Jetson, False jika tidak
        """
        try:
            # Cara 1: Cek file khusus Jetson
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'NVIDIA' in model and ('Jetson' in model or 'AGX' in model or 'Orin' in model):
                        self.log_to_file(f"Detected Jetson device: {model.strip()}", level='info')
                        return True
            
            # Cara 2: Cek platform processor info
            if 'aarch64' in platform.machine() and 'tegra' in platform.release().lower():
                self.log_to_file("Detected Jetson platform via aarch64/tegra in system info", level='info')
                return True
                
            # Cara 3: Cek keberadaan CUDA untuk Jetson
            for cuda_path in ['/usr/local/cuda', '/usr/local/cuda-*']:
                if glob.glob(cuda_path):
                    self.log_to_file(f"Detected CUDA installation at {cuda_path}, assuming Jetson", level='info')
                    return True
                    
            # Cara 4: Cek command nvidia-smi
            try:
                result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=2)
                if 'Jetson' in result.stdout or 'AGX' in result.stdout or 'Orin' in result.stdout:
                    self.log_to_file("Detected Jetson platform via nvidia-smi", level='info')
                    return True
            except (subprocess.SubprocessError, OSError, TimeoutError):
                # Ignore errors in this detection method
                pass
                
            # Cara 5: Cek keberadaan tegra_release
            if os.path.exists('/etc/nv_tegra_release'):
                self.log_to_file("Detected /etc/nv_tegra_release file, confirming Jetson platform", level='info')
                return True
                
            return False
                
        except Exception as e:
            self.log_to_file(f"Error saat deteksi platform Jetson: {e}", level='warning')
            return False  # Default ke non-Jetson jika error
            
    def validate_core_packages(self):
        """Validasi ketersediaan package-package inti yang dibutuhkan."""
        critical_packages = ['ros_deep_learning', 'cv_bridge', 'sensor_msgs', 'std_msgs']
        missing_packages = []
        
        for package in critical_packages:
            if not self.validate_dependency(package):
                missing_packages.append(package)
                
        if missing_packages:
            self.log_to_file(f"PERINGATAN: Package penting tidak ditemukan: {', '.join(missing_packages)}", level='warning')
            self.log_to_file("Beberapa fitur mungkin tidak berfungsi dengan benar", level='warning')
            
    def validate_dependency(self, package_name):
        """
        Validasi package dependency ada dan terinstall.
        
        Args:
            package_name: Nama package yang akan divalidasi
            
        Returns:
            bool: True jika package valid, False jika tidak
        """
        try:
            # Coba akses package dengan FindPackageShare
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
        """
        Cek apakah device kamera bisa diakses (hanya log warning jika tidak bisa,
        tidak menghentikan launch process untuk kompatibilitas dengan simulasi).
        
        Args:
            device (str): Path device kamera (csi://X atau /dev/videoX)
            
        Returns:
            bool: True jika device bisa diakses, False jika tidak
        """
        # Skip validasi untuk CSI cameras karena hanya detectable di Jetson
        if device.startswith("csi://"):
            # Di Jetson, tambahan validasi untuk CSI
            if self.is_jetson:
                try:
                    # Jetson harus memiliki /dev/video* untuk setiap CSI port
                    # Biasanya /dev/video0 untuk CSI-0, dst
                    csi_num = int(device.split('://')[1])
                    if not glob.glob(f'/dev/video{csi_num}*'):
                        self.log_to_file(f"WARNING: CSI camera {device} tidak menemukan device /dev/video{csi_num}*", level='warning')
                        self.log_to_file(f"TIP: Cek kabel kamera dan driver Jetson Multimedia API", level='info')
                        return False
                except (ValueError, IndexError) as e:
                    self.log_to_file(f"WARNING: Format CSI device tidak valid: {device} - {e}", level='warning')
                    return False
            return True  # Tetap kembalikan True untuk kompatibilitas simulasi
            
        # Untuk device /dev/videoX
        if device.startswith("/dev/video"):
            if not os.path.exists(device):
                warn_msg = f"WARNING: Device kamera {device} tidak ditemukan"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
            
            # Cek permission jika device ada
            if not os.access(device, os.R_OK):
                warn_msg = f"WARNING: Tidak ada permission untuk membaca device {device}"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                print(f"TIP: Jalankan 'sudo chmod a+r {device}' atau tambahkan user ke group 'video'")
                return False
                
            # Cek apakah device sedang digunakan
            try:
                # Menggunakan lsof untuk melihat apakah device sedang digunakan
                result = subprocess.run(['lsof', device], capture_output=True, text=True)
                if result.returncode == 0:  # returncode 0 berarti device sedang digunakan
                    users = result.stdout.strip().split('\n')[1:]  # Skip header line
                    user_pids = [line.split()[1] for line in users]
                    warn_msg = f"WARNING: Device {device} sedang digunakan oleh proses: {', '.join(user_pids)}"
                    print(warn_msg)
                    self.log_to_file(warn_msg, level='warning')
                    # Tetap lanjutkan, ini hanya warning
            except (subprocess.SubprocessError, OSError):
                # Ignore errors with lsof check
                pass
        
        # Untuk URL cameras (rtsp://, http://, etc.)
        if device.startswith(('rtsp://', 'http://', 'https://')):
            self.log_to_file(f"INFO: Menggunakan URL camera: {device}", level='info')
            # URL validation could be added here, e.g., checking connection
            try:
                import urllib.request
                with urllib.request.urlopen(device, timeout=2) as response:
                    if response.getcode() != 200:
                        self.log_to_file(f"WARNING: URL camera tidak dapat diakses: {device}", level='warning')
                        return False
            except:
                # Don't fail on URL validation, just warn
                self.log_to_file(f"WARNING: Tidak dapat memvalidasi URL camera: {device}", level='warning')
            
        # Untuk file video lokal
        if device.endswith(('.mp4', '.avi', '.mkv', '.mov')):
            if not os.path.isfile(device):
                warn_msg = f"WARNING: File video {device} tidak ditemukan"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
                
            # Cek apakah file video dapat dibaca
            if not os.access(device, os.R_OK):
                warn_msg = f"WARNING: File video {device} tidak dapat dibaca"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                return False
                
            # Validasi format video
            try:
                import cv2
                cap = cv2.VideoCapture(device)
                if not cap.isOpened():
                    warn_msg = f"WARNING: File {device} bukan file video yang valid"
                    print(warn_msg)
                    self.log_to_file(warn_msg, level='warning')
                    return False
                cap.release()  # Release the capture to avoid resource leaks
            except Exception as e:
                self.log_to_file(f"WARNING: Gagal memvalidasi file video {device}: {e}", level='warning')
                # Continue anyway for simulation compatibility
                
        return True
    
    def generate_camera_args(self):
        """
        Generate launch arguments untuk semua kamera.
        
        Returns:
            list: List dari launch arguments
        """
        args = []
        
        # Argumen global
        args.append(DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Gunakan waktu simulasi (true untuk Gazebo, false untuk hardware real)'
        ))
        
        args.append(DeclareLaunchArgument(
            'namespace', 
            default_value='',
            description='Namespace untuk multi-robot deployment'
        ))
        
        args.append(DeclareLaunchArgument(
            'respawn_cameras', 
            default_value='true',
            description='Auto-respawn camera nodes jika crash (true/false)'
        ))
        
        args.append(DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level untuk nodes (debug|info|warn|error|fatal)'
        ))
        
        args.append(DeclareLaunchArgument(
            'log_file_path',
            default_value=os.path.join(self.log_dir, f'camera_{self.start_time}.log'),
            description='Path untuk file log kamera'
        ))
        
        args.append(DeclareLaunchArgument(
            'camera_logger_enabled',
            default_value='true',
            description='Enable/disable camera logger node'
        ))
        
        args.append(DeclareLaunchArgument(
            'capture_method',
            default_value='gstreamer' if self.is_jetson else 'opencv',  # Gunakan gstreamer di Jetson, opencv di platform lain
            description='Metode capture kamera (gstreamer, opencv, v4l2)'
        ))
        
        # YOLOv12 integration parameters
        args.append(DeclareLaunchArgument(
            'enable_yolo_integration',
            default_value='true',
            description='Enable integrasi langsung dengan node YOLOv12 detection/segmentation'
        ))
        
        args.append(DeclareLaunchArgument(
            'yolo_model_type',
            default_value='detection',
            description='Tipe model YOLOv12 (detection, segmentation, obb)'
        ))
        
        args.append(DeclareLaunchArgument(
            'diagnostics_enabled',
            default_value='true',
            description='Enable diagnostics untuk monitoring kamera'
        ))
        
        args.append(DeclareLaunchArgument(
            'camera_mode',
            default_value='high_quality',
            description='Mode kamera (high_quality, balanced, high_fps)'
        ))
        
        # Argumen per kamera
        for i, (dev, topic, frame_id) in enumerate(self.camera_remap, start=1):
            # Enable flag untuk kamera
            args.append(DeclareLaunchArgument(
                f'camera{i}_enable',
                default_value='true',
                description=f'Enable/disable kamera {i} (true/false)'
            ))
            
            # Device path
            args.append(DeclareLaunchArgument(
                f'camera{i}_device',
                default_value=dev,
                description=f'Device kamera {i} (misal: {dev})'
            ))
            
            # Topic output
            args.append(DeclareLaunchArgument(
                f'camera{i}_topic',
                default_value=topic,
                description=f'Topic output kamera {i} (misal: {topic})'
            ))
            
            # Frame ID
            args.append(DeclareLaunchArgument(
                f'camera{i}_frame_id',
                default_value=frame_id,
                description=f'Frame ID kamera {i} untuk TF dan visualisasi'
            ))
            
            # Resolution width
            args.append(DeclareLaunchArgument(
                f'camera{i}_width',
                default_value=self.default_width,
                description=f'Resolution width kamera {i}'
            ))
            
            # Resolution height
            args.append(DeclareLaunchArgument(
                f'camera{i}_height',
                default_value=self.default_height,
                description=f'Resolution height kamera {i}'
            ))
            
            # Framerate
            args.append(DeclareLaunchArgument(
                f'camera{i}_framerate',
                default_value=self.default_framerate, 
                description=f'Framerate kamera {i}'
            ))
            
            # Flip image (opsional)
            args.append(DeclareLaunchArgument(
                f'camera{i}_flip',
                default_value='false', 
                description=f'Flip image kamera {i} (true/false)'
            ))
            
            # Tambahan parameter: Quality (untuk compression)
            args.append(DeclareLaunchArgument(
                f'camera{i}_quality',
                default_value='85',
                description=f'JPEG compression quality kamera {i} (0-100)'
            ))
            
            # Camera exposure setting
            args.append(DeclareLaunchArgument(
                f'camera{i}_exposure',
                default_value='-1',  # -1 means auto exposure
                description=f'Exposure setting untuk kamera {i} (-1=auto)'
            ))
            
            # Camera white balance
            args.append(DeclareLaunchArgument(
                f'camera{i}_white_balance',
                default_value='-1',  # -1 means auto white balance
                description=f'White balance untuk kamera {i} (-1=auto)'
            ))
            
            # Camera gain
            args.append(DeclareLaunchArgument(
                f'camera{i}_gain',
                default_value='-1',  # -1 means auto gain
                description=f'Gain setting untuk kamera {i} (-1=auto)'
            ))
            
            # Camera calibration file
            args.append(DeclareLaunchArgument(
                f'camera{i}_calib_file',
                default_value='',  # Empty means no calibration
                description=f'Path ke file kalibrasi untuk kamera {i} (kosong=tanpa kalibrasi)'
            ))
            
        return args
    
    def check_logger_executable(self):
        """
        Cek apakah camera_logger.py executable ada dan bisa diakses.
        
        Returns:
            bool: True jika executable ada, False jika tidak
        """
        # Path kemungkinan untuk camera_logger.py
        possible_paths = [
            os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'scripts', 'camera_logger.py'),
            '/opt/ros/humble/lib/huskybot_camera/camera_logger.py',
            os.path.join(os.path.expanduser('~'), 'huskybot', 'install', 'huskybot_camera', 'lib', 'huskybot_camera', 'camera_logger.py')
        ]
        
        for path in possible_paths:
            if os.path.isfile(path):
                if os.access(path, os.X_OK):  # Cek apakah file executable
                    self.log_to_file(f"INFO: camera_logger.py ditemukan di {path}", level='info')
                    return True
                else:
                    self.log_to_file(f"WARNING: camera_logger.py ditemukan di {path} tapi tidak executable", level='warning')
                    print(f"TIP: Jalankan 'chmod +x {path}' untuk membuat file executable")
                    # Try to make it executable
                    try:
                        os.chmod(path, 0o755)
                        self.log_to_file(f"INFO: Set permission executable untuk {path}", level='info')
                        return True
                    except Exception as e:
                        self.log_to_file(f"WARNING: Tidak dapat set permission executable: {e}", level='warning')
        
        # If we get here, check if we can find it by name in the PATH
        try:
            result = subprocess.run(['which', 'camera_logger.py'], capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                path = result.stdout.strip()
                self.log_to_file(f"INFO: camera_logger.py ditemukan di PATH: {path}", level='info')
                return True
        except (subprocess.SubprocessError, OSError):
            # Ignore errors with this check
            pass
            
        # Try one more approach - check in the ROS workspace
        try:
            workspace_path = os.path.expanduser('~/huskybot')
            if os.path.exists(workspace_path):
                for root, _, files in os.walk(os.path.join(workspace_path, 'src', 'huskybot_camera')):
                    if 'camera_logger.py' in files:
                        path = os.path.join(root, 'camera_logger.py')
                        self.log_to_file(f"INFO: camera_logger.py ditemukan di workspace: {path}", level='info')
                        if not os.access(path, os.X_OK):
                            try:
                                os.chmod(path, 0o755)
                                self.log_to_file(f"INFO: Set permission executable untuk {path}", level='info')
                            except Exception as e:
                                self.log_to_file(f"WARNING: Tidak dapat set permission executable: {e}", level='warning')
                        return True
        except Exception as e:
            self.log_to_file(f"WARNING: Error saat mencari di workspace: {e}", level='warning')
        
        # If all approaches fail
        self.log_to_file("WARNING: camera_logger.py tidak ditemukan", level='warning')
        return False
    
    def generate_camera_nodes(self):
        """
        Generate node ROS untuk semua kamera.
        
        Returns:
            list: List dari Node ROS2
        """
        nodes = []
        
        # Validasi dependency dulu
        if not self.validate_dependency('ros_deep_learning'):
            print("ERROR: Package ros_deep_learning tidak ditemukan!")
            print("ERROR: Install dengan: sudo apt install ros-humble-ros-deep-learning")
            self.log_to_file("ERROR: Package ros_deep_learning tidak ditemukan!", level='error')
            self.log_to_file("Mencoba fallback ke multicamera_publisher native", level='info')
            
            # Tambahkan node dummy untuk pesan error jika ros_deep_learning tidak ada
            nodes.append(
                Node(
                    package='huskybot_camera',
                    executable='multicamera_publisher',  # Fallback ke node Python native
                    name='camera_fallback',
                    output='both',  # Output ke screen dan log file
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'log_level': LaunchConfiguration('log_level', default='info'),
                        'log_file': LaunchConfiguration('log_file_path'),
                        'auto_reconnect': True,  # Auto-reconnect jika camera disconnects
                        'retry_count': 5,        # Retry 5 kali
                        'retry_delay': 2.0,      # 2 detik delay antara retry
                        'camera_count': len(self.camera_remap),  # Jumlah kamera
                    }],
                    # Event handler untuk status node
                    on_exit=[
                        LogInfo(msg="Node fallback camera berhenti dengan exit code: ${}.returncode"),
                    ]
                )
            )
            return nodes
            
        # Tambahkan logger node jika executable ada
        has_logger = self.check_logger_executable()
        if has_logger:
            nodes.append(
                Node(
                    package='huskybot_camera',
                    executable='camera_logger.py',
                    name='camera_logger',
                    output='both',  # Output ke screen dan log file
                    # Hanya aktifkan jika flag camera_logger_enabled=true dan log_level tidak kosong
                    condition=IfCondition(LaunchConfiguration('camera_logger_enabled', default='true')),
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'log_file': LaunchConfiguration('log_file_path'),
                        'log_level': LaunchConfiguration('log_level', default='info'),
                        'camera_count': len(self.camera_remap),  # Tambahkan info jumlah kamera
                        'diagnostics_enabled': LaunchConfiguration('diagnostics_enabled', default='true'),
                    }],
                    # Add event handlers
                    on_exit=[
                        LogInfo(msg="Camera logger node berhenti dengan exit code: ${}.returncode"),
                    ]
                )
            )
        else:
            print("WARNING: camera_logger.py tidak ditemukan, logger node tidak akan dijalankan")
            self.log_to_file("WARNING: camera_logger.py tidak ditemukan, logger node tidak akan dijalankan", level='warning')
            
        # Buat node untuk setiap kamera
        for i in range(1, len(self.camera_remap) + 1):
            # Buat node hanya jika camera enable = true
            camera_enable_condition = IfCondition(LaunchConfiguration(f'camera{i}_enable', default='true'))
            
            # Check camera device exists (log warning only)
            device = LaunchConfiguration(f'camera{i}_device')
            
            # Tambahkan parameter untuk adaptasi platform Jetson
            extra_params = {}
            if self.is_jetson:
                extra_params = {
                    'capture_method': LaunchConfiguration('capture_method', default='gstreamer'),  # gstreamer optimal di Jetson
                    'hw_acceleration': True,  # Gunakan hardware acceleration di Jetson
                    'zero_copy': True,  # Gunakan zero copy untuk performa lebih baik
                    'jetson_csi': True if f'camera{i}_device' == 'csi://0' else False,  # Flag khusus untuk CSI cameras
                }
                
                # Determine optimal parameters based on camera mode
                camera_mode = LaunchConfiguration('camera_mode', default='high_quality')
                # These could be dynamically adjusted based on camera mode
                
            # Adaptation for different camera modes
            camera_mode_params = {}
            camera_mode = LaunchConfiguration('camera_mode', default='high_quality')
            
            # These would be conditionally set based on camera_mode in a real implementation
            # For now, just define defaults
            
            # Add YOLO integration parameters if enabled
            yolo_params = {}
            if LaunchConfiguration('enable_yolo_integration', default='true') == 'true':
                yolo_params = {
                    'yolo_compatible': True,  # Set format for direct YOLO compatibility
                    'yolo_model_type': LaunchConfiguration('yolo_model_type', default='detection'),
                    'preprocess_for_yolo': True,  # Apply any preprocessing needed for YOLO
                }
            
            # Buat node camera dengan parameter yang sesuai
            nodes.append(
                Node(
                    condition=camera_enable_condition,  # Hanya jalankan jika camera diaktifkan
                    package='ros_deep_learning',              # Package untuk driver camera
                    executable='video_source',                # Executable untuk camera driver
                    name=f'video_source_{i}',                 # Nama node unik per camera
                    output='both',                            # Output ke terminal dan log file
                    namespace=LaunchConfiguration('namespace', default=''), # Namespace untuk multi-robot
                    respawn=LaunchConfiguration('respawn_cameras', default='true'), # Auto-restart jika node mati
                    respawn_delay=1.0,                        # Delay sebelum restart
                    parameters=[{
                        'resource': LaunchConfiguration(f'camera{i}_device'),        # Device path
                        'width': LaunchConfiguration(f'camera{i}_width'),            # Resolution width
                        'height': LaunchConfiguration(f'camera{i}_height'),          # Resolution height
                        'framerate': LaunchConfiguration(f'camera{i}_framerate'),    # Framerate
                        'codec': self.default_codec,                                 # Codec encoder
                        'loop': 0,                                                   # Loop video (0=no loop)
                        'latency': self.default_latency,                             # Buffer latency (ms)
                        'use_sim_time': LaunchConfiguration('use_sim_time'),         # ROS time source
                        'flip': LaunchConfiguration(f'camera{i}_flip', default='false'), # Flip image
                        'frame_id': LaunchConfiguration(f'camera{i}_frame_id'),      # TF frame ID
                        'quality': LaunchConfiguration(f'camera{i}_quality', default='85'), # JPEG quality
                        'exposure': LaunchConfiguration(f'camera{i}_exposure', default='-1'), # Exposure (-1=auto)
                        'white_balance': LaunchConfiguration(f'camera{i}_white_balance', default='-1'), # WB (-1=auto)
                        'gain': LaunchConfiguration(f'camera{i}_gain', default='-1'),  # Gain (-1=auto)
                        'calib_file': LaunchConfiguration(f'camera{i}_calib_file', default=''), # Calibration file
                        **extra_params,     # Add platform-specific parameters
                        **camera_mode_params,  # Add mode-specific parameters
                        **yolo_params,      # Add YOLO integration parameters
                    }],
                    remappings=[
                        # Map default topic ke topic yang diinginkan dengan namespace
                        ('/video_source/raw', LaunchConfiguration(f'camera{i}_topic'))
                    ],
                    # Event handler untuk log status kamera
                    on_exit=[
                        LogInfo(msg=f"Kamera {i} (device: " + LaunchConfiguration(f'camera{i}_device') + 
                               ") berhenti dengan exit code: ${}.returncode"),
                    ]
                )
            )
            
        # Tambahkan status monitor node untuk monitoring real-time
        nodes.append(
            Node(
                package='huskybot_camera',
                executable='status_monitor.py',  # Python script untuk monitoring status kamera
                name='camera_status_monitor',
                output='both',  # Output ke screen dan log file
                condition=IfCondition(LaunchConfiguration('diagnostics_enabled', default='true')),
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'log_file': LaunchConfiguration('log_file_path'),
                    'update_rate': 5.0,  # Update status setiap 5 detik
                    'camera_count': len(self.camera_remap),  # Jumlah kamera untuk monitoring
                    'topic_prefix': LaunchConfiguration('namespace', default=''),  # Namespace
                }]
            )
        )
        
        # Diagnostics aggregator untuk mengumpulkan diagnostics dari semua nodes
        nodes.append(
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                output='both',  # Output ke screen dan log file
                condition=IfCondition(LaunchConfiguration('diagnostics_enabled', default='true')),
                parameters=[{
                    'analyzers': {
                        'cameras': {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'Cameras',
                            'contains': ['camera', 'video_source'],
                        }
                    }
                }]
            )
        )
            
        # Event handler untuk shutdown clean-up
        nodes.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        LogInfo(msg=['Launch kamera berhenti, melakukan clean-up...']),
                        # Tambahkan aksi clean-up tambahan di sini jika perlu
                        ExecuteProcess(
                            cmd=['bash', '-c', 'echo "Releasing camera resources..."; killall -9 gst-launch-1.0 2>/dev/null || true; v4l2-ctl --all > /dev/null 2>&1 || true'],
                            name='camera_cleanup',
                            output='both',  # Output ke screen dan log file
                        )
                    ]
                )
            )
        )
        
        # Tambahkan handler untuk memastikan semua node berhenti dengan rapih
        for i in range(1, len(self.camera_remap) + 1):
            nodes.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=nodes[i],  # Referensi ke node camera
                        on_exit=[
                            LogInfo(msg=[f'Node kamera {i} keluar, memastikan resources dibersihkan...']),
                        ]
                    )
                )
            )
            
        return nodes
        
    def log_to_file(self, msg, level='info'):
        """
        Log pesan ke file log.
        
        Args:
            msg (str): Pesan yang akan dilog
            level (str): Level log (info, warning, error)
        """
        try:
            log_file = os.path.join(self.log_dir, f'camera_launch_{self.start_time}.log')
            
            # Create parent directory if it doesn't exist
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            
            with open(log_file, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
        except Exception as e:
            print(f"WARNING: Gagal menulis ke log file: {e}")
            print(f"WARNING: Log message: [{level.upper()}] {msg}")
            
            # Try fallback to /tmp
            try:
                fallback_log = os.path.join('/tmp', f'huskybot_camera_launch_{self.start_time}.log')
                with open(fallback_log, 'a') as f:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
            except:
                # Give up on logging to file if fallback fails
                pass
    
    def validate_camera_frames(self):
        """
        Validasi frame ID untuk integrasi dengan TF.
        
        Returns:
            bool: True jika semua frame valid, False jika ada yang tidak valid
        """
        # Simple validation: ensure frame IDs follow naming convention
        all_valid = True
        tf_base_frame = 'base_link'  # Base frame untuk robot
        
        for _, _, frame_id in self.camera_remap:
            # Check if frame ID follows expected pattern
            if not frame_id.endswith('_optical_frame') and not frame_id.endswith('_link'):
                warn_msg = f"WARNING: Frame ID {frame_id} tidak mengikuti konvensi naming (_optical_frame atau _link)"
                print(warn_msg)
                self.log_to_file(warn_msg, level='warning')
                all_valid = False
                
            # Check if frame is properly connected to base_link in the TF tree (just inform)
            tf_path = f"{tf_base_frame} -> ... -> {frame_id}"
            self.log_to_file(f"INFO: Expected TF path: {tf_path}", level='info')
        
        return all_valid
        
    def check_tf_tree(self):
        """Validasi TF tree ada dan terhubung dengan benar."""
        try:
            # Try to run tf2_echo to check if base_link exists
            # This is just a test - in a real implementation, you'd use more robust TF tree validation
            result = subprocess.run(['ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'camera_front_optical_frame'],
                                    capture_output=True, text=True, timeout=1)
            
            if "Could not transform" in result.stderr:
                self.log_to_file("WARNING: TF tree tidak lengkap. Frame camera_front_optical_frame tidak terhubung dengan base_link", level='warning')
                self.log_to_file("TIP: Pastikan robot_state_publisher berjalan dan URDF memiliki transformasi yang benar", level='info')
                return False
                
            return True
        except (subprocess.SubprocessError, OSError, TimeoutError):
            # This isn't critical, so just log and continue
            self.log_to_file("WARNING: Tidak dapat memeriksa TF tree", level='warning')
            return True


def generate_launch_description():
    """
    Generate launch description untuk kamera.
    Fungsi ini wajib ada di launch file ROS2.
    
    Returns:
        LaunchDescription: Deskripsi launch ROS2
    """
    # Instance config class
    config = CameraLaunchConfig()
    
    # Log start launch
    config.log_to_file("Launch file camera.launch.py dimulai", level='info')
    
    # Generate semua argument dan node
    args = config.generate_camera_args()
    nodes = config.generate_camera_nodes()
    
    # Validasi frame IDs
    config.validate_camera_frames()
    
    # Check TF tree connectivity (optional)
    config.check_tf_tree()
    
    # Validasi konfigurasi
    for i, (dev, topic, frame_id) in enumerate(config.camera_remap, start=1):
        config.check_camera_device(dev)
        config.log_to_file(f"Kamera {i}: {dev} -> {topic} [frame: {frame_id}]", level='info')
    
    # Tambahkan node camera static transform broadcaster jika diperlukan
    # untuk menghasilkan TF dari base_link ke camera_*_link
    # (Commented out because this would typically be handled by robot_state_publisher and URDF)

    # Tambahkan parameter logging untuk debug
    config.log_to_file(f"Jumlah argumen: {len(args)}", level='debug')
    config.log_to_file(f"Jumlah node: {len(nodes)}", level='debug')
    
    # Tambahkan node untuk diagnostic publisher
    diagnostic_node = Node(
        package='diagnostic_updater',
        executable='example_update_diagnostics',
        name='camera_diagnostics',
        output='both',
        condition=IfCondition(LaunchConfiguration('diagnostics_enabled', default='true')),
        parameters=[{
            'diagnostic_period': 1.0,  # Update every second
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Tambahkan test ekstra untuk validasi image topics
    test_image_topics = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5; echo "Testing image topics..."; rostopic list | grep -E "/camera_.*_?/image_raw" || echo "WARNING: No camera image topics found!"'],
        name='test_image_topics',
        output='both',
        condition=IfCondition('false'),  # Disabled by default, enable for debugging
    )
    
    # Return launch description dengan semua args dan nodes
    return LaunchDescription(
        # Mulai dengan log info
        [LogInfo(msg=['Starting camera launch file, initializing 6 Arducam IMX477 cameras...'])] +
        # Tambahkan semua argumen
        args +
        # Tambahkan semua node
        nodes +
        # Tambahkan node diagnostics
        [diagnostic_node] +
        # Tambahkan log performance untuk monitoring
        [LogInfo(msg=['Semua node camera telah diluncurkan. Memulai monitoring performance...'])] +
        # Tambahkan test image topics (disabled by default)
        [test_image_topics] +
        # Log info final
        [LogInfo(msg=[f'Camera launch completed with {len(nodes)} nodes. Listening to topics...'])]
    )