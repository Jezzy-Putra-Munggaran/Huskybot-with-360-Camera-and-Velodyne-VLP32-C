#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/launch/multicamera.launch.py
# ======================================================================
# Launch file untuk menjalankan node multicamera_publisher
# Node ini menghandle 6 kamera Arducam IMX477 (hexagonal) secara simultan
# Konfigurasi: Front, Front-Left, Left, Rear, Rear-Right, Right
# Digunakan untuk: Pipeline deteksi 360° pada robot Huskybot
# Kompatibel dengan: ROS2 Humble, Gazebo, & Husky A200 + Jetson AGX Orin
# ======================================================================

import os                                  # Import modul os untuk operasi sistem seperti path dan environment variables
import sys                                 # Import modul sys untuk akses ke interpreter dan exit codes
import yaml                                # Import modul yaml untuk validasi format config file
import socket                              # Import modul socket untuk deteksi network issues
import platform                            # Import modul platform untuk deteksi sistem dan arsitektur
from datetime import datetime              # Import datetime untuk timestamp di log dan filename

from launch import LaunchDescription       # Import kelas LaunchDescription untuk deskripsi launch configuration
from launch_ros.actions import Node        # Import kelas Node untuk membuat instance node ROS
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PathJoinSubstitution  # Import substitusi untuk variabel dinamis
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, ExecuteProcess  # Import aksi untuk proses launch
from launch.conditions import IfCondition, UnlessCondition  # Import kondisi untuk eksekusi bersyarat
from launch.events import Shutdown, process  # Import event untuk penanganan shutdown dan process events
from launch_ros.substitutions import FindPackageShare  # Import substitusi untuk mencari path package
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown, OnExecutionComplete  # Import handler untuk merespon events


class MultiCameraLaunchConfig:
    """Kelas untuk mengelola konfigurasi launch file multicamera dengan pendekatan OOP."""
    
    def __init__(self, use_sim_time=False, namespace=''):
        """
        Inisialisasi konfigurasi launch multicamera.
        
        Args:
            use_sim_time (bool): Flag untuk menggunakan waktu simulasi
            namespace (str): Namespace untuk multi-robot
        """
        self.use_sim_time = use_sim_time      # Flag untuk menggunakan waktu simulasi (Gazebo)
        self.namespace = namespace             # Namespace untuk scenario multi-robot
        
        # Definisi konfigurasi kamera default untuk 6 kamera dalam array hexagonal
        self.camera_config = [
            # (topic, device, frame_id, width, height, fps)
            ('/camera_front/image_raw', 'csi://0', 'camera_front_optical_frame', '1920', '1080', '30.0'),
            ('/camera_front_left/image_raw', 'csi://1', 'camera_front_left_optical_frame', '1920', '1080', '30.0'),
            ('/camera_left/image_raw', 'csi://2', 'camera_left_optical_frame', '1920', '1080', '30.0'),
            ('/camera_rear/image_raw', 'csi://3', 'camera_rear_optical_frame', '1920', '1080', '30.0'),
            ('/camera_rear_right/image_raw', 'csi://4', 'camera_rear_right_optical_frame', '1920', '1080', '30.0'),
            ('/camera_right/image_raw', 'csi://5', 'camera_right_optical_frame', '1920', '1080', '30.0'),
        ]
        
        # Path untuk log dan config
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')  # Timestamp untuk file names
        self.log_dir = os.path.expanduser('~/huskybot_camera_log')  # Path untuk log kamera
        
        # Buat folder log jika belum ada (error handling)
        try:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                print(f"INFO: Created log directory: {self.log_dir}")
        except Exception as e:
            print(f"WARNING: Gagal membuat folder log {self.log_dir}: {e}")
            print(f"WARNING: Menggunakan /tmp sebagai fallback")
            self.log_dir = '/tmp'
        
        # Deteksi sistem dan hardware
        self.detect_system_info()
    
    def detect_system_info(self):
        """Deteksi informasi sistem untuk konfigurasi optimal."""
        # Deteksi sistem operasi
        self.system = platform.system()
        self.is_linux = self.system == 'Linux'
        self.hostname = socket.gethostname()
        
        # Deteksi arsitektur CPU
        self.architecture = platform.machine()
        self.is_arm = self.architecture.startswith('arm') or self.architecture == 'aarch64'
        
        # Deteksi Jetson (untuk optimasi)
        self.is_jetson = False
        if self.is_linux:
            try:
                # Cek file khusus Jetson
                if os.path.exists('/proc/device-tree/model'):
                    with open('/proc/device-tree/model', 'r') as f:
                        model = f.read()
                        if 'NVIDIA' in model and ('Jetson' in model or 'AGX' in model or 'Orin' in model):
                            self.is_jetson = True
                
                # Cek string tegra di kernel release
                if 'tegra' in platform.release().lower():
                    self.is_jetson = True
            except Exception as e:
                print(f"WARNING: Error saat deteksi Jetson: {e}")
        
        # Log hasil deteksi
        self.log_to_file(f"System detection: OS={self.system}, Arch={self.architecture}, Jetson={self.is_jetson}, Hostname={self.hostname}")
    
    def check_config_file(self, config_path):
        """
        Cek apakah file konfigurasi kamera valid.
        
        Args:
            config_path (str): Path ke file konfigurasi
            
        Returns:
            bool: True jika file valid, False jika tidak
        """
        try:
            if not config_path or not os.path.isfile(config_path):
                print(f"WARNING: Config file {config_path} tidak ditemukan")
                return False
                
            # Coba load file YAML untuk validasi format
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            if not isinstance(config, dict) or 'cameras' not in config:
                print(f"WARNING: Config file {config_path} tidak valid (format harus memiliki key 'cameras')")
                return False
            
            # Validasi isi cameras
            if not isinstance(config['cameras'], list) or len(config['cameras']) == 0:
                print(f"WARNING: Config file {config_path} tidak valid (cameras harus berupa non-empty list)")
                return False
            
            # Validasi setiap entri camera
            for i, camera in enumerate(config['cameras']):
                required_fields = ['topic', 'device', 'frame_id']
                for field in required_fields:
                    if field not in camera:
                        print(f"WARNING: Config file {config_path} tidak valid (camera index {i} missing required field '{field}')")
                        return False
                
            # Semua validasi berhasil
            return True
            
        except yaml.YAMLError as e:
            print(f"WARNING: File {config_path} bukan YAML valid: {e}")
            return False
        except Exception as e:
            print(f"WARNING: Gagal memvalidasi config file {config_path}: {e}")
            return False
            
    def generate_launch_args(self):
        """
        Generate launch arguments untuk node multicamera_publisher.
        
        Returns:
            list: List dari launch arguments
        """
        args = []
        
        # Parameter global
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
            'log_level',
            default_value='info',
            description='Log level untuk nodes (debug|info|warn|error|fatal)'
        ))
        
        args.append(DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path ke file YAML konfigurasi kamera (opsional)'
        ))
        
        args.append(DeclareLaunchArgument(
            'retry_count',
            default_value='5',
            description='Jumlah retry koneksi kamera jika gagal'
        ))
        
        args.append(DeclareLaunchArgument(
            'retry_delay',
            default_value='2.0',
            description='Delay antara retry dalam detik'
        ))
        
        args.append(DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Rate publikasi gambar dalam Hz'
        ))
        
        args.append(DeclareLaunchArgument(
            'publish_camera_info',
            default_value='false',
            description='Flag untuk publish camera_info messages'
        ))
        
        args.append(DeclareLaunchArgument(
            'fallback_to_video_files',
            default_value='false',
            description='Jika true, coba fallback ke file video jika kamera hardware gagal'
        ))
        
        args.append(DeclareLaunchArgument(
            'video_file_dir',
            default_value='',
            description='Direktori berisi video files untuk fallback'
        ))
        
        args.append(DeclareLaunchArgument(
            'log_file',
            default_value=os.path.join(self.log_dir, f'multicamera_{self.timestamp}.log'),
            description='Path untuk log file'
        ))

        # Parameter debugging & optimasi
        args.append(DeclareLaunchArgument(
            'enable_health_check',
            default_value='true',
            description='Enable/disable health check timer untuk monitoring kamera'
        ))
        
        args.append(DeclareLaunchArgument(
            'health_check_interval',
            default_value='5.0',
            description='Interval health check dalam detik'
        ))
        
        args.append(DeclareLaunchArgument(
            'auto_reconnect',
            default_value='true',
            description='Auto-reconnect kamera yang disconnect'
        ))
        
        args.append(DeclareLaunchArgument(
            'enable_performance_logging',
            default_value='true', 
            description='Log performa kamera (fps, latency)'
        ))
        
        args.append(DeclareLaunchArgument(
            'buffer_size',
            default_value='1',
            description='Ukuran buffer kamera untuk latency vs reliability'
        ))
        
        # Menambahkan parameter untuk setiap kamera
        for i, (topic, device, frame_id, width, height, fps) in enumerate(self.camera_config, start=1):
            args.append(DeclareLaunchArgument(
                f'camera{i}_enable',
                default_value='true',
                description=f'Enable/disable kamera {i} (true/false)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_topic',
                default_value=topic,
                description=f'Topic output untuk kamera {i}'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_device',
                default_value=device,
                description=f'Path device kamera {i} (csi://X atau /dev/videoX)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_frame_id',
                default_value=frame_id,
                description=f'Frame ID untuk kamera {i}'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_width',
                default_value=width,
                description=f'Resolution width kamera {i} (pixel)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_height',
                default_value=height,
                description=f'Resolution height kamera {i} (pixel)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_fps',
                default_value=fps,
                description=f'Frame rate kamera {i}'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_flip',
                default_value='false',
                description=f'Flip image kamera {i} (true/false)'
            ))
            
            # Tambahan optimasi parameter per kamera
            args.append(DeclareLaunchArgument(
                f'camera{i}_quality',
                default_value='85',
                description=f'JPEG quality kamera {i} (0-100)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_exposure',
                default_value='-1',
                description=f'Exposure kamera {i} (-1=auto)'
            ))
            
            args.append(DeclareLaunchArgument(
                f'camera{i}_white_balance',
                default_value='-1',
                description=f'White balance kamera {i} (-1=auto)'
            ))
            
        return args
    
    def generate_node(self):
        """
        Generate node ROS2 untuk multicamera_publisher.
        
        Returns:
            Node: Node ROS2 untuk publisher multicamera
        """
        # Buat dictionary parameter untuk kamera
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
        
        # Tambahan parameter deteksi platform
        platform_params = {
            'is_jetson': str(self.is_jetson).lower(),
            'is_simulation': LaunchConfiguration('use_sim_time'),
            'system': self.system,
            'architecture': self.architecture,
            'hostname': self.hostname
        }
        
        # Buat node multicamera_publisher dengan parameter yang sesuai
        node = Node(
            package='huskybot_camera',                              # Package untuk multicamera publisher
            executable='multicamera_publisher',                     # Executable untuk multicamera publisher
            name='multicamera_publisher',                           # Nama node
            namespace=LaunchConfiguration('namespace', default=''), # Namespace untuk multi-robot
            output='screen',                                        # Output ke terminal untuk debugging
            respawn=True,                                           # Auto-respawn jika node crash
            respawn_delay=1.0,                                      # Delay sebelum respawn
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),              # ROS time source (sim/real)
                    'config_file': LaunchConfiguration('config_file'),                # Path ke file konfigurasi
                    'retry_count': LaunchConfiguration('retry_count'),               # Jumlah retry koneksi
                    'retry_delay': LaunchConfiguration('retry_delay'),               # Delay antara retry
                    'publish_rate': LaunchConfiguration('publish_rate'),             # Rate publikasi gambar
                    'publish_camera_info': LaunchConfiguration('publish_camera_info'), # Publish camera_info messages
                    'log_file': LaunchConfiguration('log_file'),                     # Path untuk log file
                    'fallback_to_video_files': LaunchConfiguration('fallback_to_video_files'), # Fallback ke video files
                    'video_file_dir': LaunchConfiguration('video_file_dir'),          # Dir video files
                    'enable_health_check': LaunchConfiguration('enable_health_check'), # Enable health check
                    'health_check_interval': LaunchConfiguration('health_check_interval'), # Interval health check
                    'auto_reconnect': LaunchConfiguration('auto_reconnect'),          # Auto reconnect cameras
                    'enable_performance_logging': LaunchConfiguration('enable_performance_logging'), # Log performa
                    'buffer_size': LaunchConfiguration('buffer_size'),                # Buffer size
                    # Platform detection params
                    **platform_params,
                    # Tambahkan parameter untuk setiap kamera
                    **camera_params
                }
            ],
            # Event handler untuk exit node
            on_exit=[
                LogInfo(msg="Node multicamera_publisher berhenti dengan exit code: ${}.returncode"),
            ]
        )
        
        return node
    
    def generate_tools(self):
        """
        Generate tools tambahan untuk debugging dan monitoring.
        
        Returns:
            list: List dari node tools
        """
        tools = []
        
        # Node visualizer (format kompak untuk multiple camera)
        tools.append(
            Node(
                package='huskybot_camera',
                executable='camera_visualizer',
                name='camera_visualizer',
                namespace=LaunchConfiguration('namespace', default=''),
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_visualization', default='false')),
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            )
        )
        
        # Node diagnostics untuk health check
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
        """
        Log pesan ke file log.
        
        Args:
            msg (str): Pesan yang akan dilog
            level (str): Level log (info, warning, error)
        """
        try:
            log_file = os.path.join(self.log_dir, f'multicamera_launch_{self.timestamp}.log')
            with open(log_file, 'a') as f:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{timestamp}] [{level.upper()}] {msg}\n")
        except Exception as e:
            print(f"WARNING: Gagal menulis ke log file: {e}")
            print(f"WARNING: {level.upper()}: {msg}")


def generate_launch_description():
    """
    Generate launch description untuk multicamera publisher.
    Fungsi ini wajib ada di launch file ROS2.
    
    Returns:
        LaunchDescription: Deskripsi launch ROS2
    """
    # Instance config class
    config = MultiCameraLaunchConfig()
    
    # Log start launch
    config.log_to_file("Launch file multicamera.launch.py dimulai", level='info')
    
    # Check environment variables
    if os.environ.get('DISPLAY'):
        config.log_to_file(f"DISPLAY environment variable detected: {os.environ.get('DISPLAY')}", level='info')
    
    # Deteksi ROS_DOMAIN_ID untuk komunikasi
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'default')
    config.log_to_file(f"ROS_DOMAIN_ID: {ros_domain_id}", level='info')
    
    # Generate semua argument
    args = config.generate_launch_args()
    
    # Add visualization & diagnostic args
    args.append(DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
        description='Enable/disable camera visualization window'
    ))
    
    args.append(DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='false',
        description='Enable/disable diagnostics untuk monitoring'
    ))
    
    args.append(DeclareLaunchArgument(
        'check_devices', 
        default_value='true',
        description='Check devices existence before starting nodes'
    ))
    
    # Generate node
    node = config.generate_node()
    
    # Generate tools
    tools = config.generate_tools()
    
    # Add camera device checker (to fail early if devices don't exist)
    check_devices = ExecuteProcess(
        cmd=['bash', '-c', 'echo "Checking camera devices..."; ls -la /dev/video* || echo "No V4L devices found"; v4l2-ctl --list-devices || echo "v4l2-ctl not available"'],
        name='check_camera_devices',
        output='screen',
        condition=IfCondition(LaunchConfiguration('check_devices'))
    )
    
    # Event handler untuk shutdown cleanup
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Launch multicamera berhenti, melakukan clean-up...']
            )]
        )
    )
    
    # Event handler untuk kegagalan check devices (agar informative)
    check_error_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=check_devices,
            on_exit=[
                LogInfo(msg=['Device check complete with code ${}.returncode'])
            ]
        )
    )
    
    # Return launch description
    return LaunchDescription(
        # Log info awal
        [LogInfo(msg=['Starting multicamera publisher untuk 6 kamera Arducam IMX477 (hexagonal)...'])] +
        # Check devices first
        [check_devices, check_error_handler] +
        # Tambahkan semua argument
        args +
        # Tambahkan node utama
        [
            node,
            shutdown_handler
        ] +
        # Tambahkan tools
        tools +
        # Log info akhir
        [LogInfo(msg=['Multicamera publisher sudah dijalankan.'])]
    )