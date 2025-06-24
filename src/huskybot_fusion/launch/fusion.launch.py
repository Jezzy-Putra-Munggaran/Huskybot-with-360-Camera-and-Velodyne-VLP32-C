#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_fusion/launch/fusion.launch.py
from launch import LaunchDescription  # Import utama untuk mendefinisikan launch description ROS2
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, RegisterEventHandler  # Import untuk argumen, fungsi custom, logging, dan event handler
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable  # Import untuk variabel launch, ekspresi Python, dan variabel lingkungan
from launch.conditions import IfCondition, UnlessCondition  # Import untuk kondisional dalam launch file
from launch_ros.actions import Node, SetParameter  # Import untuk node ROS2 dan setting parameter
from launch.event_handlers import OnProcessStart, OnProcessExit  # Import untuk event handler proses
import os  # Import untuk operasi file dan path
import sys  # Import untuk akses stderr dan exit code
import time  # Import untuk timestamp di log
import glob  # Import untuk pencarian pattern file
import shutil  # Import untuk operasi file lanjutan

# ===================== ERROR HANDLING & LOGGER (ENHANCED) =====================
def check_calibration_file(context, *args, **kwargs):  # Fungsi validasi file kalibrasi dengan retry mechanism
    """Validasi file kalibrasi extrinsic dengan mekanisme retry dan fallback path"""
    calibration_file = LaunchConfiguration('calibration_file').perform(context)  # Ambil path file kalibrasi dari argumen
    expanded = os.path.expandvars(os.path.expanduser(calibration_file))  # Expand ~ dan env var ke absolute path
    
    # Coba beberapa lokasi alternatif jika file tidak ditemukan di path utama
    max_retries = int(LaunchConfiguration('max_retries').perform(context))  # Jumlah retry maksimum
    retry_delay = float(LaunchConfiguration('retry_delay').perform(context))  # Delay antara retry dalam detik
    
    # Daftar path alternatif untuk kalibrasi (fallback mechanism)
    alternative_paths = [
        expanded,  # Path utama dari argumen
        os.path.join(os.path.dirname(expanded), 'extrinsic_lidar_to_camera.yaml'),  # Coba nama file default di folder yang sama
        os.path.expanduser('~/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml'),  # Path default workspace
        os.path.expanduser('~/huskybot_ws/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml'),  # Path alternatif
        '/opt/huskybot/config/extrinsic_lidar_to_camera.yaml',  # Path instalasi sistem
    ]
    
    found_file = None  # File kalibrasi yang ditemukan
    
    # Coba setiap path alternatif dengan beberapa retry
    for path in alternative_paths:  # Iterasi semua path alternatif
        for attempt in range(max_retries):  # Iterasi jumlah retry
            if os.path.isfile(path):  # Cek file fisik ada
                if os.access(path, os.R_OK):  # Cek permission read
                    found_file = path  # Simpan path yang valid
                    log_to_file(f"[INFO] File kalibrasi extrinsic ditemukan (attempt {attempt+1}): {path}")  # Log info
                    print(f"[INFO] File kalibrasi extrinsic ditemukan: {path}")  # Print ke terminal
                    break  # Keluar dari loop retry
                else:
                    log_to_file(f"[WARNING] Tidak ada permission read file kalibrasi: {path}")  # Log warning
                    print(f"[WARNING] Tidak ada permission read file kalibrasi: {path}", file=sys.stderr)  # Print warning
            
            if attempt < max_retries - 1:  # Jika belum retry terakhir
                time.sleep(retry_delay)  # Tunggu sebelum retry selanjutnya
        
        if found_file:  # Jika sudah menemukan file
            break  # Keluar dari loop path alternatif
    
    # Jika tidak menemukan file kalibrasi yang valid di semua path
    if not found_file:
        # Cari file .yaml di beberapa lokasi umum sebagai fallback terakhir
        potential_files = []  # List untuk menyimpan file potensial
        search_paths = [  # Daftar path untuk dicari
            os.path.expanduser('~/huskybot/src/huskybot_calibration/config/'),
            os.path.expanduser('~/huskybot/src/huskybot_fusion/config/'),
            os.path.expanduser('~/huskybot_ws/src/huskybot_calibration/config/'),
            '/opt/huskybot/config/',
        ]
        
        for search_path in search_paths:  # Iterasi path pencarian
            if os.path.isdir(search_path):  # Cek folder ada
                yaml_files = glob.glob(os.path.join(search_path, '*.yaml'))  # Cari file .yaml
                potential_files.extend(yaml_files)  # Tambahkan ke list
        
        if potential_files:  # Jika ada file potensial
            # Pilih file pertama yang bisa diakses
            for file_path in potential_files:  # Iterasi file potensial
                if os.access(file_path, os.R_OK):  # Cek permission read
                    found_file = file_path  # Simpan path valid
                    log_to_file(f"[WARNING] File kalibrasi {expanded} tidak ditemukan. Menggunakan alternatif: {file_path}")  # Log warning
                    print(f"[WARNING] File kalibrasi {expanded} tidak ditemukan. Menggunakan alternatif: {file_path}")  # Print warning
                    break  # Keluar dari loop
    
    # Jika masih tidak menemukan file kalibrasi, error dan exit
    if not found_file:
        error_msg = f"[ERROR] File kalibrasi extrinsic tidak ditemukan di semua lokasi yang dicoba. Pastikan file ada dan dapat diakses."  # Pesan error
        log_to_file(error_msg)  # Log error
        print(error_msg, file=sys.stderr)  # Print error
        sys.exit(2)  # Exit dengan kode error
    
    # Set environment variable agar available untuk node lain
    os.environ['HUSKYBOT_CALIBRATION_FILE'] = found_file  # Set env var
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def check_log_file_permission(context, *args, **kwargs):  # Cek permission file log dengan autofix
    """Cek dan pastikan permission file log dengan mekanisme autofix"""
    log_file = LaunchConfiguration('log_file').perform(context)  # Ambil path file log
    enable_logs = LaunchConfiguration('enable_logs').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status aktif log
    
    if not enable_logs:  # Jika logging tidak diaktifkan
        print("[INFO] File logging disabled via enable_logs parameter")  # Print info
        return []  # Return early
    
    if not log_file or log_file.strip() == '':  # Jika log_file kosong
        print("[INFO] No log file specified, skipping log file permission check")  # Print info
        return []  # Return early
    
    expanded = os.path.expandvars(os.path.expanduser(log_file))  # Expand ~ dan env var
    log_dir = os.path.dirname(expanded) or '.'  # Ambil folder log, default '.'
    
    # Pastikan folder log ada dengan error handling
    if not os.path.isdir(log_dir):  # Jika folder belum ada
        try:
            print(f"[INFO] Creating log directory: {log_dir}")  # Print info
            os.makedirs(log_dir, exist_ok=True)  # Buat folder log dengan exist_ok=True
            log_to_file(f"[INFO] Created log directory: {log_dir}")  # Log info
        except PermissionError:  # Jika error permission
            # Coba di home user jika path asli tidak bisa diakses
            fallback_dir = os.path.expanduser("~/huskybot_logs")  # Folder fallback
            print(f"[WARNING] Cannot create log directory {log_dir}, falling back to {fallback_dir}", file=sys.stderr)  # Print warning
            log_to_file(f"[WARNING] Cannot create log directory {log_dir}, falling back to {fallback_dir}")  # Log warning
            try:
                os.makedirs(fallback_dir, exist_ok=True)  # Buat folder fallback
                log_path = os.path.join(fallback_dir, os.path.basename(expanded))  # Path file fallback
                # Update environment variable untuk node
                os.environ['HUSKYBOT_LOG_FILE'] = log_path  # Set env var
                log_to_file(f"[INFO] Using fallback log file: {log_path}")  # Log info
                return []  # Return early dengan fallback path
            except Exception as e2:  # Jika fallback juga gagal
                print(f"[ERROR] Also failed to create fallback directory {fallback_dir}: {e2}", file=sys.stderr)  # Print error
                log_to_file(f"[ERROR] Also failed to create fallback directory {fallback_dir}: {e2}")  # Log error
                # Tidak fatal, lanjutkan tanpa logging
                return []
        except Exception as e:  # Jika error lainnya
            print(f"[ERROR] Failed to create log directory {log_dir}: {e}", file=sys.stderr)  # Print error
            log_to_file(f"[ERROR] Failed to create log directory {log_dir}: {e}")  # Log error
            # Tidak fatal, lanjutkan tanpa logging
            return []
    
    # Cek permission tulis ke file log
    try:
        with open(expanded, "a") as f:  # Buka file append mode
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')  # Format timestamp
            f.write(f"[{timestamp}] [INFO] Test write log file from check_log_file_permission\n")  # Tulis test
        print(f"[INFO] Log file is writable: {expanded}")  # Print info
        log_to_file(f"[INFO] Log file is writable: {expanded}")  # Log info
        # Set environment variable untuk node
        os.environ['HUSKYBOT_LOG_FILE'] = expanded  # Set env var
    except (PermissionError, IOError) as e:  # Jika error permission atau IO
        print(f"[ERROR] Cannot write to log file {expanded}: {e}", file=sys.stderr)  # Print error
        log_to_file(f"[ERROR] Cannot write to log file {expanded}: {e}")  # Log error
        
        # Coba buat file di lokasi user jika path asli tidak bisa ditulis
        fallback_path = os.path.expanduser(f"~/huskybot_logs/{os.path.basename(expanded)}")  # Path fallback
        fallback_dir = os.path.dirname(fallback_path)  # Folder fallback
        
        try:
            os.makedirs(fallback_dir, exist_ok=True)  # Buat folder fallback
            with open(fallback_path, "a") as f:  # Buka file fallback
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')  # Format timestamp
                f.write(f"[{timestamp}] [INFO] Using fallback log file due to permission error\n")  # Tulis test
            print(f"[INFO] Using fallback log file: {fallback_path}")  # Print info
            log_to_file(f"[INFO] Using fallback log file: {fallback_path}")  # Log info
            # Update environment variable untuk node
            os.environ['HUSKYBOT_LOG_FILE'] = fallback_path  # Set env var
        except Exception as e2:  # Jika fallback juga gagal
            print(f"[ERROR] Also failed to create fallback log file: {e2}", file=sys.stderr)  # Print error
            log_to_file(f"[ERROR] Also failed to create fallback log file: {e2}", level='error')  # Log error
            # Tidak fatal, lanjutkan tanpa logging
    except Exception as e:  # Jika error lainnya
        print(f"[ERROR] Unexpected error with log file {expanded}: {e}", file=sys.stderr)  # Print error
        log_to_file(f"[ERROR] Unexpected error with log file {expanded}: {e}", level='error')  # Log error
        # Tidak fatal, lanjutkan tanpa logging
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def log_to_file(msg, level='info'):  # Fungsi logging ke file dengan fallback mechanism
    """Log message ke file dengan fallback mechanism jika file utama tidak bisa diakses"""
    log_file_path = os.environ.get('HUSKYBOT_LAUNCH_LOG', os.path.expanduser("~/huskybot_fusion_launch.log"))  # Ambil path dari env var atau default
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')  # Format timestamp
    
    try:
        os.makedirs(os.path.dirname(log_file_path) or '.', exist_ok=True)  # Buat folder log jika belum ada
        with open(log_file_path, "a") as logf:  # Buka file append mode
            logf.write(f"[{timestamp}] [{level.upper()}] {msg}\n")  # Tulis log dengan format
    except Exception as e:  # Jika error
        # Coba fallback ke file di home
        fallback_path = os.path.expanduser("~/huskybot_launch_fallback.log")  # Path fallback
        try:
            with open(fallback_path, "a") as logf:  # Buka file fallback
                logf.write(f"[{timestamp}] [{level.upper()}] {msg}\n")  # Tulis log dengan format
                # Jika ini pertama kali fallback, tulis juga error aslinya
                logf.write(f"[{timestamp}] [ERROR] Original log failed: {e}\n")  # Tulis error asli
        except Exception as e2:  # Jika fallback juga gagal
            # Terakhir, print ke stderr jika semua gagal
            print(f"[{timestamp}] [{level.upper()}] {msg} (Log file error: {e}, fallback error: {e2})", file=sys.stderr)  # Print error lengkap

def check_environment_readiness(context, *args, **kwargs):  # Cek kesiapan environment ROS2
    """Validasi environment ROS2 siap untuk menjalankan fusion node"""
    # Check ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO', '').lower()  # Ambil ROS_DISTRO dari env var
    if ros_distro != 'humble':  # Jika bukan humble
        print(f"[WARNING] Expected ROS_DISTRO='humble', found '{ros_distro}'. This package is designed for ROS2 Humble.", file=sys.stderr)  # Print warning
        log_to_file(f"[WARNING] Expected ROS_DISTRO='humble', found '{ros_distro}'. This package is designed for ROS2 Humble.")  # Log warning
    
    # Check ROS_VERSION
    ros_version = os.environ.get('ROS_VERSION', '')  # Ambil ROS_VERSION dari env var
    if ros_version != '2':  # Jika bukan ROS2
        print(f"[ERROR] Expected ROS_VERSION='2', found '{ros_version}'. This package requires ROS2.", file=sys.stderr)  # Print error
        log_to_file(f"[ERROR] Expected ROS_VERSION='2', found '{ros_version}'. This package requires ROS2.", level='error')  # Log error
        sys.exit(10)  # Exit dengan kode error
    
    # Check if environment is sourced
    if not os.environ.get('AMENT_PREFIX_PATH', ''):  # Jika AMENT_PREFIX_PATH kosong
        print("[ERROR] ROS2 environment does not seem to be sourced. Please run 'source /opt/ros/humble/setup.bash'", file=sys.stderr)  # Print error
        log_to_file("[ERROR] ROS2 environment does not seem to be sourced. Please run 'source /opt/ros/humble/setup.bash'", level='error')  # Log error
        sys.exit(11)  # Exit dengan kode error
    
    # Check required ROS2 packages
    required_packages = ['sensor_msgs', 'geometry_msgs', 'visualization_msgs', 'tf2_ros', 'cv_bridge']  # Package wajib
    missing_packages = []  # List untuk package yang tidak ada
    
    # Loop untuk cek semua package wajib
    for package in required_packages:  # Iterasi semua package
        # Use ros2 pkg prefix as a test if package exists
        try:
            import subprocess  # Import subprocess untuk eksekusi command
            result = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True, text=True)  # Run command
            if result.returncode != 0:  # Jika command error
                missing_packages.append(package)  # Tambahkan ke list missing
        except Exception:  # Jika exception
            missing_packages.append(package)  # Tambahkan ke list missing
    
    if missing_packages:  # Jika ada package yang hilang
        print(f"[ERROR] Missing required ROS2 packages: {', '.join(missing_packages)}. Please install them.", file=sys.stderr)  # Print error
        log_to_file(f"[ERROR] Missing required ROS2 packages: {', '.join(missing_packages)}. Please install them.", level='error')  # Log error
        print(f"[INFO] You can install them with: sudo apt install " + " ".join([f"ros-{ros_distro}-{pkg.replace('_', '-')}" for pkg in missing_packages]))  # Print info
        # Warning only, don't exit
    
    # Check for YOLOv12 custom messages
    try:
        import subprocess  # Import subprocess untuk eksekusi command
        result = subprocess.run(['ros2', 'pkg', 'prefix', 'yolov12_msgs'], capture_output=True, text=True)  # Cek package
        if result.returncode != 0:  # Jika command error
            print("[ERROR] yolov12_msgs package not found. Required for fusion with YOLOv12 detections.", file=sys.stderr)  # Print error
            log_to_file("[ERROR] yolov12_msgs package not found. Required for fusion with YOLOv12 detections.", level='error')  # Log error
            print("[INFO] Please make sure yolov12_msgs is built and sourced", file=sys.stderr)  # Print info
            # Warning only, don't exit
    except Exception as e:  # Jika exception
        print(f"[ERROR] Error checking for yolov12_msgs: {e}", file=sys.stderr)  # Print error
        log_to_file(f"[ERROR] Error checking for yolov12_msgs: {e}", level='error')  # Log error
    
    # Check for GPU/CUDA if enabled
    use_gpu = LaunchConfiguration('use_gpu').perform(context).lower() in ('true', '1', 'yes', 'y')  # Cek parameter use_gpu
    if use_gpu:  # Jika use_gpu aktif
        try:
            # Check for CUDA with nvidia-smi
            import subprocess  # Import subprocess untuk eksekusi command
            result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)  # Cek nvidia-smi
            if result.returncode != 0:  # Jika command error
                print("[WARNING] use_gpu=true but nvidia-smi failed. CUDA/GPU may not be available.", file=sys.stderr)  # Print warning
                log_to_file("[WARNING] use_gpu=true but nvidia-smi failed. CUDA/GPU may not be available.", level='warn')  # Log warning
                # Warning only, don't exit
        except Exception:  # Jika exception
            print("[WARNING] use_gpu=true but could not verify GPU availability with nvidia-smi.", file=sys.stderr)  # Print warning
            log_to_file("[WARNING] use_gpu=true but could not verify GPU availability with nvidia-smi.", level='warn')  # Log warning
            # Warning only, don't exit
    
    # Check if we're on Jetson (for optimal configuration)
    is_jetson = os.path.exists('/etc/nv_tegra_release')  # Cek file khusus Jetson
    if is_jetson:  # Jika di Jetson
        print("[INFO] Detected Jetson platform, optimizing for embedded deployment")  # Print info
        log_to_file("[INFO] Detected Jetson platform, optimizing for embedded deployment")  # Log info
        # Set environment variable for other processes
        os.environ['HUSKYBOT_PLATFORM'] = 'jetson'  # Set env var
    else:  # Jika bukan di Jetson
        print("[INFO] Not running on Jetson platform, using standard configuration")  # Print info
        log_to_file("[INFO] Not running on Jetson platform, using standard configuration")  # Log info
        os.environ['HUSKYBOT_PLATFORM'] = 'standard'  # Set env var
    
    # Check if we're in simulation (Gazebo)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes', 'y')  # Cek parameter use_sim_time
    if sim_time:  # Jika use_sim_time aktif
        print("[INFO] Running in simulation mode (use_sim_time=true)")  # Print info
        log_to_file("[INFO] Running in simulation mode (use_sim_time=true)")  # Log info
        # Set environment variable for other processes
        os.environ['HUSKYBOT_SIMULATION'] = 'true'  # Set env var
    else:  # Jika tidak di simulasi
        print("[INFO] Running in real robot mode (use_sim_time=false)")  # Print info
        log_to_file("[INFO] Running in real robot mode (use_sim_time=false)")  # Log info
        os.environ['HUSKYBOT_SIMULATION'] = 'false'  # Set env var
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def validate_rviz_config(context, *args, **kwargs):  # Validasi dan siapkan konfigurasi RViz
    """Validasi dan siapkan konfigurasi RViz dengan fallback dan autofix"""
    rviz_config = LaunchConfiguration('rviz_config').perform(context)  # Ambil path config RViz
    
    if not rviz_config:  # Jika parameter kosong
        print("[WARNING] rviz_config parameter is empty, using default configuration", file=sys.stderr)  # Print warning
        log_to_file("[WARNING] rviz_config parameter is empty, using default configuration", level='warn')  # Log warning
        rviz_config = os.path.join(os.path.dirname(__file__), '..', 'rviz', 'fusion.rviz')  # Set default
    
    expanded = os.path.expandvars(os.path.expanduser(rviz_config))  # Expand ~ dan env var
    
    # Daftar path alternatif untuk config RViz
    alternative_paths = [
        expanded,  # Path utama dari argumen
        os.path.join(os.path.dirname(expanded), 'fusion.rviz'),  # Coba nama file default di folder yang sama
        os.path.join(os.path.dirname(__file__), '..', 'rviz', 'fusion.rviz'),  # Path relatif dalam package
        os.path.expanduser('~/huskybot/src/huskybot_fusion/rviz/fusion.rviz'),  # Path workspace default
        os.path.expanduser('~/huskybot_ws/src/huskybot_fusion/rviz/fusion.rviz'),  # Path workspace alternatif
    ]
    
    found_config = None  # Config yang ditemukan
    
    # Coba semua path alternatif
    for path in alternative_paths:  # Iterasi semua path
        if os.path.isfile(path) and os.access(path, os.R_OK):  # Cek file ada dan bisa dibaca
            found_config = path  # Simpan path valid
            break  # Keluar dari loop
    
    # Jika tidak menemukan konfigurasi, buat default minimal
    if not found_config:  # Jika tidak ada config valid
        print("[WARNING] RViz configuration not found, creating minimal default", file=sys.stderr)  # Print warning
        log_to_file("[WARNING] RViz configuration not found, creating minimal default", level='warn')  # Log warning
        
        # Buat folder rviz jika belum ada
        default_dir = os.path.join(os.path.dirname(__file__), '..', 'rviz')  # Path default folder
        os.makedirs(default_dir, exist_ok=True)  # Buat folder
        
        # Path untuk config default
        default_config = os.path.join(default_dir, 'fusion.rviz')  # Path file default
        
        # Tulis konfigurasi RViz minimal
        minimal_config = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /MarkerArray1
        - /PointCloud21
      Splitter Ratio: 0.5
    Tree Height: 728
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MarkerArray
      Topic: /fusion/objects3d_marker
      Value: true
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: PointCloud2
      Topic: /velodyne_points
      Value: true
  Global Options:
    Fixed Frame: base_link
    Frame Rate: 30
"""
        try:
            with open(default_config, 'w') as f:  # Buka file untuk menulis
                f.write(minimal_config)  # Tulis config minimal
            found_config = default_config  # Update path
            print(f"[INFO] Created minimal RViz configuration at: {default_config}")  # Print info
            log_to_file(f"[INFO] Created minimal RViz configuration at: {default_config}")  # Log info
        except Exception as e:  # Jika gagal menulis
            print(f"[ERROR] Failed to create minimal RViz configuration: {e}", file=sys.stderr)  # Print error
            log_to_file(f"[ERROR] Failed to create minimal RViz configuration: {e}", level='error')  # Log error
            # Set default yang berfungsi meski tanpa file fisik (menggunakan empty string)
            found_config = ""  # Empty untuk default RViz
    
    # Set environment variable untuk node
    if found_config:  # Jika config valid ditemukan
        os.environ['HUSKYBOT_RVIZ_CONFIG'] = found_config  # Set env var
        print(f"[INFO] Using RViz configuration: {found_config}")  # Print info
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def check_custom_msgs(context, *args, **kwargs):  # Cek custom message package
    """Validasi package custom message tersedia dan cocok dengan YOLOv12"""
    # Custom message packages needed for fusion
    custom_pkgs = ['yolov12_msgs', 'huskybot_msgs']  # Daftar package custom message
    missing_pkgs = []  # List untuk missing packages
    
    # Check each package
    for pkg in custom_pkgs:  # Iterasi setiap package
        try:
            # Try to use ros2 package prefix command to check if package is available
            import subprocess  # Import subprocess untuk eksekusi command
            result = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True)  # Cek package
            if result.returncode != 0:  # Jika command error
                missing_pkgs.append(pkg)  # Tambahkan ke list missing
        except Exception:  # Jika exception
            missing_pkgs.append(pkg)  # Tambahkan ke list missing
    
    # If any packages are missing, print warnings
    if missing_pkgs:  # Jika ada package yang hilang
        msg = f"[WARNING] Custom message packages missing: {', '.join(missing_pkgs)}. Fusion may not work properly."  # Pesan warning
        print(msg, file=sys.stderr)  # Print warning
        log_to_file(msg, level='warn')  # Log warning
        
        # Print build instructions
        print("[INFO] To build missing packages, run:")  # Print info
        for pkg in missing_pkgs:  # Iterasi setiap package yang hilang
            print(f"  cd ~/huskybot && colcon build --packages-select {pkg}")  # Print build command
        
        # This is a warning only, we won't exit
        # However, fusion node will likely fail if these are missing
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def check_topics_available(context, *args, **kwargs):  # Cek ketersediaan topic
    """Cek apakah topic utama sudah tersedia (YOLOv12 detection, LiDAR, dll)"""
    # Define expected topics
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)  # Topic LiDAR dari parameter
    detection_topics = ['/detection', '/segmentation', '/tracking', '/obb']  # Daftar topic YOLOv12
    
    # Check if topics are being published
    available_topics = []  # List untuk topic yang tersedia
    
    try:
        # Use ros2 topic list to check available topics
        import subprocess  # Import subprocess untuk eksekusi command
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)  # List topic
        if result.returncode == 0:  # Jika command sukses
            available_topics = result.stdout.strip().split('\n')  # Split output jadi list
        else:  # Jika command error
            print("[WARNING] Failed to list ROS2 topics, cannot verify topic availability", file=sys.stderr)  # Print warning
            log_to_file("[WARNING] Failed to list ROS2 topics, cannot verify topic availability", level='warn')  # Log warning
    except Exception as e:  # Jika exception
        print(f"[WARNING] Error checking topic availability: {e}", file=sys.stderr)  # Print warning
        log_to_file(f"[WARNING] Error checking topic availability: {e}", level='warn')  # Log warning
    
    # Check for LiDAR topic
    if lidar_topic not in available_topics:  # Jika topic LiDAR tidak ada
        print(f"[WARNING] LiDAR topic '{lidar_topic}' not found. Make sure LiDAR driver is running.", file=sys.stderr)  # Print warning
        log_to_file(f"[WARNING] LiDAR topic '{lidar_topic}' not found. Make sure LiDAR driver is running.", level='warn')  # Log warning
        
        # Check if any point cloud topics are available as alternatives
        pc2_topics = [t for t in available_topics if 'points' in t.lower() or 'pointcloud' in t.lower()]  # Filter topic pointcloud
        if pc2_topics:  # Jika ada alternatif
            print(f"[INFO] Alternative point cloud topics found: {', '.join(pc2_topics)}")  # Print info
            log_to_file(f"[INFO] Alternative point cloud topics found: {', '.join(pc2_topics)}")  # Log info
            print(f"[INFO] You can specify an alternative topic with: --ros-args -p lidar_topic:={pc2_topics[0]}")  # Print hint
        else:  # Jika tidak ada alternatif
            print("[WARNING] No point cloud topics found. Fusion cannot work without LiDAR data.", file=sys.stderr)  # Print warning
            log_to_file("[WARNING] No point cloud topics found. Fusion cannot work without LiDAR data.", level='warn')  # Log warning
    
    # Check for YOLOv12 detection topics
    found_detection = False  # Flag jika menemukan topic deteksi
    for topic in detection_topics:  # Iterasi setiap topic deteksi
        if topic in available_topics or any(topic in t for t in available_topics):  # Jika topic ada
            found_detection = True  # Set flag true
            print(f"[INFO] Detection topic found: {topic}")  # Print info
            log_to_file(f"[INFO] Detection topic found: {topic}")  # Log info
            break  # Keluar dari loop
    
    if not found_detection:  # Jika tidak ada topic deteksi
        print("[WARNING] No YOLOv12 detection topics found (/detection, /segmentation, etc). Make sure YOLOv12 node is running.", file=sys.stderr)  # Print warning
        log_to_file("[WARNING] No YOLOv12 detection topics found (/detection, /segmentation, etc). Make sure YOLOv12 node is running.", level='warn')  # Log warning
        print("[INFO] You may need to run: ros2 launch huskybot_recognition launch_yolov12.launch.py")  # Print hint
    
    return []  # Return list kosong (best practice launch OpaqueFunction)

def build_fusion_node(context, *args, **kwargs):  # Fungsi untuk membangun node fusion
    """Bangun node fusion dengan parameter tervalidasi dan optimized untuk robot/simulasi"""
    # Ambil semua parameter konfigurasi
    calibration_file = os.environ.get('HUSKYBOT_CALIBRATION_FILE', LaunchConfiguration('calibration_file').perform(context))  # Path kalibrasi
    confidence_threshold = float(LaunchConfiguration('confidence_threshold').perform(context))  # Threshold confidence
    fusion_method = LaunchConfiguration('fusion_method').perform(context)  # Metode fusion
    namespace = LaunchConfiguration('namespace').perform(context)  # Namespace ROS2
    log_file = os.environ.get('HUSKYBOT_LOG_FILE', LaunchConfiguration('log_file').perform(context))  # Path log
    enable_debug = LaunchConfiguration('enable_debug').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status debug
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status simulation
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)  # Topic LiDAR
    enable_logs = LaunchConfiguration('enable_logs').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status log
    use_gpu = LaunchConfiguration('use_gpu').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status GPU
    diagnostics = LaunchConfiguration('enable_diagnostics').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status diagnostics
    retry_interval = float(LaunchConfiguration('retry_interval').perform(context))  # Interval retry
    
    # Validasi parameter
    if confidence_threshold < 0.0 or confidence_threshold > 1.0:  # Validasi threshold confidence
        print(f"[WARNING] Confidence threshold {confidence_threshold} is outside valid range [0,1], setting to 0.3", file=sys.stderr)  # Print warning
        confidence_threshold = 0.3  # Set default
    
    if fusion_method not in ['nearest', 'iou', 'centroid']:  # Validasi metode fusion
        print(f"[WARNING] Fusion method '{fusion_method}' not recognized, setting to 'nearest'", file=sys.stderr)  # Print warning
        fusion_method = 'nearest'  # Set default
    
    # Base parameters (mandatory)
    params = [
        {'calibration_file': calibration_file},  # Parameter kalibrasi
        {'confidence_threshold': confidence_threshold},  # Parameter threshold confidence
        {'fusion_method': fusion_method},  # Parameter metode fusion
        {'lidar_topic': lidar_topic},  # Parameter topic LiDAR
        {'use_sim_time': use_sim_time},  # Parameter simulation time
    ]
    
    # Optional parameters (only add if enabled/provided)
    if log_file and log_file.strip() != '' and enable_logs:  # Jika log file aktif
        params.append({'log_file': log_file})  # Tambahkan parameter log file
    
    if enable_debug:  # Jika debug aktif
        params.append({'enable_debug': True})  # Tambahkan parameter debug
        print("[INFO] Debug mode enabled", flush=True)  # Print info
    
    if use_gpu:  # Jika GPU aktif
        params.append({'use_gpu': True})  # Tambahkan parameter GPU
        print("[INFO] GPU acceleration enabled", flush=True)  # Print info
    
    if retry_interval > 0:  # Jika retry interval valid
        params.append({'retry_interval': retry_interval})  # Tambahkan parameter retry
    
    # Add diagnostic parameters
    params.append({'enable_diagnostics': diagnostics})  # Parameter diagnostics
    
    # Add platform-specific parameters
    platform = os.environ.get('HUSKYBOT_PLATFORM', 'standard')  # Ambil platform dari env var
    if platform == 'jetson':  # Jika platform Jetson
        params.append({'platform': 'jetson'})  # Parameter platform
        params.append({'optimize_for_jetson': True})  # Parameter optimasi Jetson
        print("[INFO] Adding Jetson-specific optimizations")  # Print info
    
    # Create fusion node
    node = Node(
        package='huskybot_fusion',  # Nama package
        executable='fusion_node',  # Executable node
        name='fusion_node',  # Nama node
        namespace=namespace,  # Namespace multi-robot
        output='screen',  # Output ke terminal
        parameters=params,  # Parameter yang sudah divalidasi
        remappings=[
            # Add topic remappings if needed
            # ('/velodyne_points', lidar_topic),  # Uncomment untuk remap point cloud
            # ('/detection', '/custom_detection_topic'),  # Uncomment untuk remap deteksi
        ],  # Remapping topic jika diperlukan
        arguments=['--ros-args', '--log-level', 'info'],  # Argumen ROS
    )

    # Log info node creation
    print(f"[INFO] Created fusion_node with {len(params)} parameters", flush=True)  # Print info
    log_to_file(f"[INFO] Created fusion_node with {len(params)} parameters")  # Log info
    
    return [node]  # Return node dalam list (best practice launch OpaqueFunction)

def build_rviz_node(context, *args, **kwargs):  # Fungsi untuk membangun node RViz
    """Bangun node RViz2 dengan konfigurasi yang sesuai untuk visualisasi"""
    # Get parameters
    rviz_config = os.environ.get('HUSKYBOT_RVIZ_CONFIG', LaunchConfiguration('rviz_config').perform(context))  # Config RViz
    namespace = LaunchConfiguration('namespace').perform(context)  # Namespace
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status launch RViz
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes', 'y')  # Status simulation
    fixed_frame = LaunchConfiguration('fixed_frame').perform(context)  # Frame referensi
    
    if not launch_rviz:  # Jika RViz tidak dilaunch
        print("[INFO] RViz node disabled via launch_rviz=false parameter", flush=True)  # Print info
        log_to_file("[INFO] RViz node disabled via launch_rviz=false parameter")  # Log info
        return []  # Return empty list (no node)
    
    # Create RViz node with proper arguments
    rviz_args = ['-d', rviz_config] if rviz_config else []  # Argumen config
    
    # Add fixed frame argument if provided
    if fixed_frame:  # Jika fixed frame ada
        rviz_args.extend(['--fixed-frame', fixed_frame])  # Tambahkan argumen fixed frame
    
    node = Node(
        package='rviz2',  # Package RViz2
        executable='rviz2',  # Executable
        name='fusion_rviz',  # Nama node
        namespace=namespace,  # Namespace multi-robot
        output='screen',  # Output ke terminal
        arguments=rviz_args,  # Argumen config dan frame
        parameters=[{'use_sim_time': use_sim_time}],  # Parameter use_sim_time
    )
    
    print(f"[INFO] Created RViz2 node with config: {rviz_config if rviz_config else 'default'}", flush=True)  # Print info
    log_to_file(f"[INFO] Created RViz2 node with config: {rviz_config if rviz_config else 'default'}")  # Log info
    
    return [node]  # Return node dalam list (best practice launch OpaqueFunction)

def generate_launch_description():  # Fungsi utama generate LaunchDescription
    """Fungsi utama untuk membuat launch description dengan error handling dan parameter"""
    try:
        # ===================== SETUP CONSOLE/FILE LOGGING =====================
        # Set default log file for this launch
        os.environ['HUSKYBOT_LAUNCH_LOG'] = os.path.expanduser("~/huskybot_fusion_launch.log")  # Set default log
        print(f"[INFO] Launch log: {os.environ['HUSKYBOT_LAUNCH_LOG']}", flush=True)  # Print log path
        
        # ===================== ARGUMEN LAUNCH (ENHANCED) =====================
        # Core parameters
        calibration_file_arg = DeclareLaunchArgument(
            'calibration_file',  # Nama parameter
            default_value=os.path.expanduser('~/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml'),  # Default path
            description='Path ke file kalibrasi extrinsic lidar-ke-kamera'  # Deskripsi parameter
        )
        
        confidence_threshold_arg = DeclareLaunchArgument(
            'confidence_threshold',  # Nama parameter
            default_value='0.3',  # Default threshold
            description='Threshold confidence minimum untuk publish objek 3D'  # Deskripsi parameter
        )
        
        fusion_method_arg = DeclareLaunchArgument(
            'fusion_method',  # Nama parameter
            default_value='nearest',  # Default metode
            description='Metode asosiasi objek fusion (nearest/iou/centroid)'  # Deskripsi parameter
        )
        
        # Multi-robot parameters
        namespace_arg = DeclareLaunchArgument(
            'namespace',  # Nama parameter
            default_value='',  # Default kosong
            description='Namespace ROS2 untuk multi-robot (opsional)'  # Deskripsi parameter
        )
        
        # Logging parameters
        log_file_arg = DeclareLaunchArgument(
            'log_file',  # Nama parameter
            default_value=os.path.expanduser('~/huskybot_logs/fusion.log'),  # Default path
            description='Path file log untuk fusion node (opsional)'  # Deskripsi parameter
        )
        
        enable_logs_arg = DeclareLaunchArgument(
            'enable_logs',  # Nama parameter
            default_value='true',  # Default enabled
            description='Aktifkan logging ke file (true/false)'  # Deskripsi parameter
        )
        
        # Debug parameters
        enable_debug_arg = DeclareLaunchArgument(
            'enable_debug',  # Nama parameter
            default_value='false',  # Default disabled
            description='Aktifkan debug log di fusion node (opsional)'  # Deskripsi parameter
        )
        
        # Simulation parameters
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',  # Nama parameter
            default_value='false',  # Default disabled (real robot)
            description='Gunakan simulasi time dari Gazebo (true=simulasi, false=robot real)'  # Deskripsi parameter
        )
        
        # LiDAR topic parameter
        lidar_topic_arg = DeclareLaunchArgument(
            'lidar_topic',  # Nama parameter
            default_value='/velodyne_points',  # Default topic
            description='Topic ROS2 untuk point cloud LiDAR'  # Deskripsi parameter
        )
        
        # RViz config parameters
        rviz_config_arg = DeclareLaunchArgument(
            'rviz_config',  # Nama parameter
            default_value=os.path.expanduser('~/huskybot/src/huskybot_fusion/rviz/fusion.rviz'),  # Default path
            description='Path ke file konfigurasi RViz2'  # Deskripsi parameter
        )
        
        launch_rviz_arg = DeclareLaunchArgument(
            'launch_rviz',  # Nama parameter
            default_value='true',  # Default enabled
            description='Launch RViz2 untuk visualisasi (true/false)'  # Deskripsi parameter
        )
        
        fixed_frame_arg = DeclareLaunchArgument(
            'fixed_frame',  # Nama parameter
            default_value='base_link',  # Default frame
            description='Fixed frame untuk visualisasi RViz2'  # Deskripsi parameter
        )
        
        # GPU parameters
        use_gpu_arg = DeclareLaunchArgument(
            'use_gpu',  # Nama parameter
            default_value='true',  # Default enabled
            description='Gunakan akselerasi GPU jika tersedia (true/false)'  # Deskripsi parameter
        )
        
        # Diagnostics parameters
        enable_diagnostics_arg = DeclareLaunchArgument(
            'enable_diagnostics',  # Nama parameter
            default_value='true',  # Default enabled
            description='Aktifkan diagnostics node (true/false)'  # Deskripsi parameter
        )
        
        # Error handling parameters
        max_retries_arg = DeclareLaunchArgument(
            'max_retries',  # Nama parameter
            default_value='5',  # Default 5 retry
            description='Jumlah maksimum retry untuk operasi yang bisa gagal'  # Deskripsi parameter
        )
        
        retry_interval_arg = DeclareLaunchArgument(
            'retry_interval',  # Nama parameter
            default_value='2.0',  # Default 2.0 detik
            description='Interval antara retry dalam detik'  # Deskripsi parameter
        )
        
        retry_delay_arg = DeclareLaunchArgument(
            'retry_delay',  # Nama parameter
            default_value='1.0',  # Default 1.0 detik
            description='Delay antara retry dalam detik'  # Deskripsi parameter
        )

        # ===================== LOGGING INFO LAUNCH STARTING =====================
        log_info = LogInfo(msg="[INFO] Starting fusion node launch file...", name="fusion_launch_start")  # Info launch start
        
        # ===================== ERROR HANDLING ACTIONS (ENHANCED) =====================
        # Check environment readiness first
        check_env_action = OpaqueFunction(function=check_environment_readiness)  # Cek environment
        
        # Check for custom messages
        check_msgs_action = OpaqueFunction(function=check_custom_msgs)  # Cek custom messages
        
        # Check calibration file with retry
        check_calib_action = OpaqueFunction(function=check_calibration_file)  # Cek file kalibrasi
        
        # Check log file permission with autofix
        check_log_action = OpaqueFunction(function=check_log_file_permission)  # Cek permission file log
        
        # Validate RViz config with fallback
        check_rviz_action = OpaqueFunction(function=validate_rviz_config)  # Validasi config RViz
        
        # Check if required topics are available
        check_topics_action = OpaqueFunction(function=check_topics_available)  # Cek ketersediaan topic
        
        # ===================== NODE FUSION & RVIZ (WITH ENHANCED PARAMETERS) =====================
        fusion_node_action = OpaqueFunction(function=build_fusion_node)  # Node fusion
        rviz_node_action = OpaqueFunction(function=build_rviz_node)  # Node RViz
        
        # ===================== RETURN LAUNCH DESCRIPTION =====================
        return LaunchDescription([
            # Log info
            log_info,  # Info launch start
            
            # Core parameters
            calibration_file_arg,  # Parameter file kalibrasi
            confidence_threshold_arg,  # Parameter threshold confidence
            fusion_method_arg,  # Parameter metode fusion
            
            # Multi-robot parameters
            namespace_arg,  # Parameter namespace
            
            # Logging parameters
            log_file_arg,  # Parameter file log
            enable_logs_arg,  # Parameter enable log
            
            # Debug parameters
            enable_debug_arg,  # Parameter enable debug
            
            # Simulation parameters
            use_sim_time_arg,  # Parameter use_sim_time
            
            # LiDAR topic parameter
            lidar_topic_arg,  # Parameter topic LiDAR
            
            # RViz parameters
            rviz_config_arg,  # Parameter config RViz
            launch_rviz_arg,  # Parameter launch RViz
            fixed_frame_arg,  # Parameter fixed frame
            
            # GPU parameters
            use_gpu_arg,  # Parameter use GPU
            
            # Diagnostic parameters
            enable_diagnostics_arg,  # Parameter enable diagnostics
            
            # Error handling parameters
            max_retries_arg,  # Parameter max retries
            retry_interval_arg,  # Parameter retry interval
            retry_delay_arg,  # Parameter retry delay
            
            # Error handling actions in sequence
            check_env_action,  # Cek environment (first)
            check_msgs_action,  # Cek custom messages
            check_calib_action,  # Cek file kalibrasi
            check_log_action,  # Cek permission file log
            check_rviz_action,  # Validasi config RViz
            check_topics_action,  # Cek ketersediaan topic
            
            # Nodes to launch
            fusion_node_action,  # Node fusion
            rviz_node_action,  # Node RViz (conditional based on launch_rviz)
            
            # Log completion
            LogInfo(msg="[INFO] Fusion launch completed successfully!", name="fusion_launch_complete"),  # Log launch complete
        ])
    except Exception as e:  # Jika launch gagal
        import traceback  # Import traceback untuk stack trace
        error_msg = f"[FATAL] Exception in generate_launch_description: {e}\n{traceback.format_exc()}"  # Pesan error dengan traceback
        print(error_msg, file=sys.stderr)  # Print error ke stderr
        log_to_file(error_msg, level='error')  # Log error ke file
        
        # Return minimal launch description with error message
        return LaunchDescription([
            LogInfo(msg=f"[FATAL] Launch failed: {e}", name="fusion_launch_error")  # Log error
        ])  # Return LaunchDescription kosong dengan hanya error log

# ===================== PENJELASAN & SARAN PENINGKATAN =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Parameter log_file hanya ditambahkan jika tidak kosong, sehingga tidak ada warning "Parameter file path is not a file: ."
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah sangat komprehensif:
#   1. Cek environment ROS2 dan dependency
#   2. Cek file kalibrasi dengan retry dan fallback path
#   3. Cek permission file log dengan autofix dan fallback
#   4. Validasi config RViz dengan autocreate jika tidak ada
#   5. Cek ketersediaan topic untuk early warning
#   6. Validasi parameter dengan default value jika invalid
#   7. Cek platform (Jetson/standard) untuk optimasi
#   8. Cek custom message packages
# - Logging info ke terminal dan file untuk audit trail.
# - Sudah siap untuk ROS2 Humble, YOLOv12, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Siap untuk multi-robot dengan namespace dan topic remapping.
# - Enhancement sudah diimplementasikan:
#   1. Autofix untuk directory dan file yang missing
#   2. Retry mechanism untuk resource yang mungkin belum tersedia
#   3. Fallback path untuk file penting
#   4. Otomatic platform detection (Jetson vs standard)
#   5. Integrasi dengan YOLOv12 task topics (detection, segmentation, obb, tracking)
#   6. Parameter untuk enable/disable fitur (RViz, debug, log, dll)
#   7. Environment variable untuk komunikasi antar proses
#   8. Detailed logging untuk debugging
# - Launching RViz hanya jika diperlukan (parameter launch_rviz).
# - Format docstring untuk semua fungsi memudahkan dokumentasi otomatis.
# - Urutan eksekusi sudah optimal untuk mendeteksi error sejak dini.
# - Semua parameter memiliki nilai default yang reasonable.
# - FULL OOP untuk node fusion (di fusion_node.py) dan semua utility di package ini.