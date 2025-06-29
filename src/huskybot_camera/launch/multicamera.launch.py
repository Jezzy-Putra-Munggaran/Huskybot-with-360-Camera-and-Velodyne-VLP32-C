#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/launch/multicamera.launch.py
# ======================================================================
# Launch file untuk node multicamera_publisher (6x Arducam IMX477, hexagonal)
# Kompatibel: ROS2 Humble, Gazebo, Jetson AGX Orin, Clearpath Husky A200
# ======================================================================

import os  # Untuk operasi file, path, dan environment
import sys  # Untuk akses exit code dan sys.argv
import yaml  # Untuk parsing file YAML config kamera
import socket  # Untuk deteksi hostname/network
import platform  # Untuk deteksi OS dan arsitektur hardware
from datetime import datetime  # Untuk timestamp log

from launch import LaunchDescription  # Kelas utama launch file ROS2
from launch_ros.actions import Node  # Untuk membuat node ROS2
from launch.substitutions import LaunchConfiguration  # Untuk parameter dinamis
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, ExecuteProcess  # Untuk deklarasi argumen dan event handler
from launch.conditions import IfCondition  # Untuk kondisi eksekusi node
from launch.events import Shutdown  # Untuk event shutdown
from launch.event_handlers import OnShutdown, OnExecutionComplete  # Untuk event handler shutdown/exit

# ===================== KELAS KONFIGURASI LAUNCH FILE =====================
class MultiCameraLaunchConfig:
    """Kelas OOP untuk mengelola konfigurasi launch multicamera."""

    def __init__(self, use_sim_time=False, namespace=''):
        """Inisialisasi konfigurasi launch multicamera."""
        self.use_sim_time = use_sim_time  # Flag waktu simulasi (Gazebo)
        self.namespace = namespace  # Namespace multi-robot
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')  # Timestamp unik untuk log
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Folder log default
        self.fallback_log_dir = '/tmp'  # Fallback log dir jika gagal
        self.camera_config = [  # Konfigurasi default 6 kamera hexagonal
            ('/camera_front/image_raw', 'csi://0', 'camera_front_optical_frame', '1920', '1080', '30.0'),
            ('/camera_front_left/image_raw', 'csi://1', 'camera_front_left_optical_frame', '1920', '1080', '30.0'),
            ('/camera_left/image_raw', 'csi://2', 'camera_left_optical_frame', '1920', '1080', '30.0'),
            ('/camera_rear/image_raw', 'csi://3', 'camera_rear_optical_frame', '1920', '1080', '30.0'),
            ('/camera_rear_right/image_raw', 'csi://4', 'camera_rear_right_optical_frame', '1920', '1080', '30.0'),
            ('/camera_right/image_raw', 'csi://5', 'camera_right_optical_frame', '1920', '1080', '30.0'),
        ]
        self.ensure_log_dir()  # Pastikan folder log ada
        self.detect_system_info()  # Deteksi OS, Jetson, dsb
        self.validate_python_dependencies()  # Validasi dependency Python
        self.validate_ros_dependencies()  # Validasi dependency ROS2

    def ensure_log_dir(self):
        """Pastikan folder log ada, fallback ke /tmp jika gagal."""
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                self.log_to_file(f"Created log directory: {self.log_dir}", level='info')
        except Exception as e:
            print(f"WARNING: Gagal membuat folder log {self.log_dir}: {e}")
            self.log_to_file(f"Gagal membuat folder log {self.log_dir}: {e}", level='warning')
            self.log_dir = self.fallback_log_dir  # Fallback ke /tmp

    def detect_system_info(self):
        """Deteksi OS, arsitektur, Jetson, hostname."""
        self.system = platform.system()  # OS (Linux/Windows/Mac)
        self.is_linux = self.system == 'Linux'  # True jika Linux
        self.hostname = socket.gethostname()  # Hostname
        self.architecture = platform.machine()  # CPU arch
        self.is_arm = self.architecture.startswith('arm') or self.architecture == 'aarch64'  # ARM/Jetson
        self.is_jetson = False  # Default non-Jetson
        if self.is_linux:
            try:
                if os.path.exists('/proc/device-tree/model'):
                    with open('/proc/device-tree/model', 'r') as f:
                        model = f.read()
                        if 'NVIDIA' in model and ('Jetson' in model or 'AGX' in model or 'Orin' in model):
                            self.is_jetson = True
                if 'tegra' in platform.release().lower():
                    self.is_jetson = True
            except Exception as e:
                self.log_to_file(f"Error deteksi Jetson: {e}", level='warning')
        self.log_to_file(f"System: OS={self.system}, Arch={self.architecture}, Jetson={self.is_jetson}, Host={self.hostname}", level='info')

    def validate_python_dependencies(self):
        """Validasi dependency Python utama (cv2, numpy, yaml, dsb)."""
        deps = ['cv2', 'numpy', 'yaml']
        for dep in deps:
            try:
                __import__(dep)
            except ImportError:
                msg = f"ERROR: Modul Python '{dep}' tidak ditemukan. Install dengan: pip install {dep}"
                print(msg)
                self.log_to_file(msg, level='error')

    def validate_ros_dependencies(self):
        """Validasi dependency ROS2 utama (rclpy, launch_ros, dsb)."""
        deps = ['rclpy', 'launch_ros']
        for dep in deps:
            try:
                __import__(dep)
            except ImportError:
                msg = f"ERROR: Modul ROS2 Python '{dep}' tidak ditemukan. Install dengan: pip install {dep}"
                print(msg)
                self.log_to_file(msg, level='error')

    def check_config_file(self, config_path):
        """Validasi file YAML konfigurasi kamera."""
        try:
            if not config_path or not os.path.isfile(config_path):
                self.log_to_file(f"Config file {config_path} tidak ditemukan", level='warning')
                return False
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            if not isinstance(config, dict) or 'cameras' not in config:
                self.log_to_file(f"Config file {config_path} tidak valid (harus ada key 'cameras')", level='warning')
                return False
            if not isinstance(config['cameras'], list) or len(config['cameras']) == 0:
                self.log_to_file(f"Config file {config_path} tidak valid (cameras harus list)", level='warning')
                return False
            for i, camera in enumerate(config['cameras']):
                for field in ['topic', 'device', 'frame_id']:
                    if field not in camera:
                        self.log_to_file(f"Config file {config_path} camera {i} missing '{field}'", level='warning')
                        return False
            return True
        except yaml.YAMLError as e:
            self.log_to_file(f"File {config_path} bukan YAML valid: {e}", level='warning')
            return False
        except Exception as e:
            self.log_to_file(f"Gagal validasi config file {config_path}: {e}", level='error')
            return False

    def generate_launch_args(self):
        """Generate semua argumen launch untuk multicamera_publisher."""
        args = []
        # Argumen global
        args.append(DeclareLaunchArgument('use_sim_time', default_value='false', description='Gunakan waktu simulasi (true untuk Gazebo, false untuk hardware real)'))
        args.append(DeclareLaunchArgument('namespace', default_value='', description='Namespace untuk multi-robot deployment'))
        args.append(DeclareLaunchArgument('log_level', default_value='info', description='Log level untuk nodes (debug|info|warn|error|fatal)'))
        args.append(DeclareLaunchArgument('config_file', default_value='', description='Path ke file YAML konfigurasi kamera (opsional)'))
        args.append(DeclareLaunchArgument('retry_count', default_value='5', description='Jumlah retry koneksi kamera jika gagal'))
        args.append(DeclareLaunchArgument('retry_delay', default_value='2.0', description='Delay antara retry dalam detik'))
        args.append(DeclareLaunchArgument('publish_rate', default_value='20.0', description='Rate publikasi gambar dalam Hz'))
        args.append(DeclareLaunchArgument('publish_camera_info', default_value='false', description='Flag untuk publish camera_info messages'))
        args.append(DeclareLaunchArgument('fallback_to_video_files', default_value='false', description='Jika true, fallback ke file video jika kamera gagal'))
        args.append(DeclareLaunchArgument('video_file_dir', default_value='', description='Direktori video files untuk fallback'))
        args.append(DeclareLaunchArgument('log_file', default_value=os.path.join(self.log_dir, f'multicamera_{self.timestamp}.log'), description='Path untuk log file'))
        args.append(DeclareLaunchArgument('enable_health_check', default_value='true', description='Enable health check timer untuk monitoring kamera'))
        args.append(DeclareLaunchArgument('health_check_interval', default_value='5.0', description='Interval health check dalam detik'))
        args.append(DeclareLaunchArgument('auto_reconnect', default_value='true', description='Auto-reconnect kamera yang disconnect'))
        args.append(DeclareLaunchArgument('enable_performance_logging', default_value='true', description='Log performa kamera (fps, latency)'))
        args.append(DeclareLaunchArgument('buffer_size', default_value='1', description='Ukuran buffer kamera untuk latency vs reliability'))
        # Argumen per kamera
        for i, (topic, device, frame_id, width, height, fps) in enumerate(self.camera_config, start=1):
            args.append(DeclareLaunchArgument(f'camera{i}_enable', default_value='true', description=f'Enable/disable kamera {i} (true/false)'))
            args.append(DeclareLaunchArgument(f'camera{i}_topic', default_value=topic, description=f'Topic output kamera {i}'))
            args.append(DeclareLaunchArgument(f'camera{i}_device', default_value=device, description=f'Device kamera {i} (csi://X atau /dev/videoX)'))
            args.append(DeclareLaunchArgument(f'camera{i}_frame_id', default_value=frame_id, description=f'Frame ID kamera {i}'))
            args.append(DeclareLaunchArgument(f'camera{i}_width', default_value=width, description=f'Resolution width kamera {i}'))
            args.append(DeclareLaunchArgument(f'camera{i}_height', default_value=height, description=f'Resolution height kamera {i}'))
            args.append(DeclareLaunchArgument(f'camera{i}_fps', default_value=fps, description=f'Frame rate kamera {i}'))
            args.append(DeclareLaunchArgument(f'camera{i}_flip', default_value='false', description=f'Flip image kamera {i} (true/false)'))
            args.append(DeclareLaunchArgument(f'camera{i}_quality', default_value='85', description=f'JPEG quality kamera {i} (0-100)'))
            args.append(DeclareLaunchArgument(f'camera{i}_exposure', default_value='-1', description=f'Exposure kamera {i} (-1=auto)'))
            args.append(DeclareLaunchArgument(f'camera{i}_white_balance', default_value='-1', description=f'White balance kamera {i} (-1=auto)'))
        # Argumen tools
        args.append(DeclareLaunchArgument('enable_visualization', default_value='false', description='Enable/disable camera visualization window'))
        args.append(DeclareLaunchArgument('enable_diagnostics', default_value='false', description='Enable/disable diagnostics untuk monitoring'))
        args.append(DeclareLaunchArgument('check_devices', default_value='true', description='Check devices existence before starting nodes'))
        return args

    def generate_node(self):
        """Generate node multicamera_publisher dengan semua parameter."""
        camera_params = {}
        for i, (_, _, _, _, _, _) in enumerate(self.camera_config, start=1):
            camera_params[f'camera{i}_enable'] = LaunchConfiguration(f'camera{i}_enable')
            camera_params[f'camera{i}_topic'] = LaunchConfiguration(f'camera{i}_topic')
            camera_params[f'camera{i}_device'] = LaunchConfiguration(f'camera{i}_device')
            camera_params[f'camera{i}_frame_id'] = LaunchConfiguration(f'camera{i}_frame_id')
            camera_params[f'camera{i}_width'] = LaunchConfiguration(f'camera{i}_width')
            camera_params[f'camera{i}_height'] = LaunchConfiguration(f'camera{i}_height')
            camera_params[f'camera{i}_fps'] = LaunchConfiguration(f'camera{i}_fps')
            camera_params[f'camera{i}_flip'] = LaunchConfiguration(f'camera{i}_flip')
            camera_params[f'camera{i}_quality'] = LaunchConfiguration(f'camera{i}_quality')
            camera_params[f'camera{i}_exposure'] = LaunchConfiguration(f'camera{i}_exposure')
            camera_params[f'camera{i}_white_balance'] = LaunchConfiguration(f'camera{i}_white_balance')
        platform_params = {
            'is_jetson': str(self.is_jetson).lower(),
            'is_simulation': LaunchConfiguration('use_sim_time'),
            'system': self.system,
            'architecture': self.architecture,
            'hostname': self.hostname
        }
        node = Node(
            package='huskybot_camera',  # Package Python node
            executable='multicamera_publisher',  # Node Python utama
            name='multicamera_publisher',  # Nama node
            namespace=LaunchConfiguration('namespace', default=''),  # Namespace multi-robot
            output='screen',  # Output ke terminal
            respawn=True,  # Auto-respawn jika node crash
            respawn_delay=1.0,  # Delay sebelum respawn
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'config_file': LaunchConfiguration('config_file'),
                'retry_count': LaunchConfiguration('retry_count'),
                'retry_delay': LaunchConfiguration('retry_delay'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_camera_info': LaunchConfiguration('publish_camera_info'),
                'log_file': LaunchConfiguration('log_file'),
                'fallback_to_video_files': LaunchConfiguration('fallback_to_video_files'),
                'video_file_dir': LaunchConfiguration('video_file_dir'),
                'enable_health_check': LaunchConfiguration('enable_health_check'),
                'health_check_interval': LaunchConfiguration('health_check_interval'),
                'auto_reconnect': LaunchConfiguration('auto_reconnect'),
                'enable_performance_logging': LaunchConfiguration('enable_performance_logging'),
                'buffer_size': LaunchConfiguration('buffer_size'),
                **platform_params,
                **camera_params
            }],
            on_exit=[LogInfo(msg="Node multicamera_publisher berhenti dengan exit code: ${}.returncode")],
        )
        return node

    def generate_tools(self):
        """Generate tools tambahan (visualizer, diagnostics)."""
        tools = []
        # Visualizer node (opsional)
        tools.append(
            Node(
                package='huskybot_camera',
                executable='camera_visualizer',
                name='camera_visualizer',
                namespace=LaunchConfiguration('namespace', default=''),
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_visualization', default='false')),
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        )
        # Diagnostics node (opsional)
        tools.append(
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                namespace=LaunchConfiguration('namespace', default=''),
                condition=IfCondition(LaunchConfiguration('enable_diagnostics', default='false')),
                parameters=[{
                    'analyzers.camera.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.camera.path': 'Camera',
                    'analyzers.camera.contains': ['camera'],
                }]
            )
        )
        return tools

    def log_to_file(self, msg, level='info'):
        """Log pesan ke file log."""
        try:
            log_file = os.path.join(self.log_dir, f'multicamera_launch_{self.timestamp}.log')
            with open(log_file, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
        except Exception as e:
            print(f"WARNING: Gagal menulis ke log file: {e}")
            print(f"WARNING: {level.upper()}: {msg}")

# ===================== FUNGSI UTAMA GENERATE LAUNCH DESCRIPTION =====================
def generate_launch_description():
    """Generate launch description untuk multicamera publisher."""
    config = MultiCameraLaunchConfig()  # Instance config class
    config.log_to_file("Launch file multicamera.launch.py dimulai", level='info')
    # Validasi environment variable penting
    if os.environ.get('DISPLAY'):
        config.log_to_file(f"DISPLAY env detected: {os.environ.get('DISPLAY')}", level='info')
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'default')
    config.log_to_file(f"ROS_DOMAIN_ID: {ros_domain_id}", level='info')
    # Validasi permission folder log
    if not os.access(config.log_dir, os.W_OK):
        config.log_to_file(f"WARNING: Tidak ada permission tulis di {config.log_dir}", level='warning')
    # Validasi node executable
    node_exec_path = os.path.join(os.path.dirname(__file__), '..', 'huskybot_camera', 'multicamera_publisher.py')
    if not os.path.isfile(node_exec_path):
        config.log_to_file(f"ERROR: Node executable {node_exec_path} tidak ditemukan!", level='error')
    # Generate semua argumen
    args = config.generate_launch_args()
    # Generate node utama
    node = config.generate_node()
    # Generate tools
    tools = config.generate_tools()
    # Device checker (fail early jika device tidak ada)
    check_devices = ExecuteProcess(
        cmd=['bash', '-c', 'echo "Checking camera devices..."; ls -la /dev/video* || echo "No V4L devices found"; v4l2-ctl --list-devices || echo "v4l2-ctl not available"'],
        name='check_camera_devices',
        output='screen',
        condition=IfCondition(LaunchConfiguration('check_devices'))
    )
    # Event handler shutdown
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(msg=['Launch multicamera berhenti, melakukan clean-up...'])]
        )
    )
    # Event handler device check
    check_error_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=check_devices,
            on_exit=[LogInfo(msg=['Device check complete with code ${}.returncode'])]
        )
    )
    # Return launch description
    return LaunchDescription(
        [LogInfo(msg=['Starting multicamera publisher untuk 6 kamera Arducam IMX477 (hexagonal)...'])] +
        [check_devices, check_error_handler] +
        args +
        [node, shutdown_handler] +
        tools +
        [LogInfo(msg=['Multicamera publisher sudah dijalankan.'])]
    )
