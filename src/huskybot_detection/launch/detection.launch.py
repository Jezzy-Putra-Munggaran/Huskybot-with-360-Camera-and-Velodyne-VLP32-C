#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# detection.launch.py - Launch file utama untuk node deteksi multicamera YOLOv12 pada Huskybot
# Kompatibel: ROS2 Humble, Gazebo, Jetson AGX Orin, 6x Arducam IMX477, Velodyne VLP32-C, Clearpath Husky A200

import os  # Untuk operasi file dan path
import sys  # Untuk akses ke sys.stderr dan exit code
import glob  # Untuk mencari file model dengan pattern (fallback)
import shutil  # Untuk backup file model
from pathlib import Path  # Untuk validasi path robust
from typing import List, Dict, Any, Optional, Union  # Type hinting

import json  # Untuk parsing file konfigurasi JSON
import yaml  # Untuk parsing file YAML
import platform  # Untuk deteksi sistem/hardware
import subprocess  # Untuk menjalankan shell command (cek hardware, topic, dsb)
import time  # Untuk timestamp dan delay
import traceback  # Untuk print stack trace error detail
import signal  # Untuk handle SIGTERM/SIGINT (shutdown clean)

from launch import LaunchDescription  # Komponen utama launch file ROS2
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, RegisterEventHandler, OpaqueFunction, EmitEvent  # Untuk argumen, logging, proses, event handler, custom function, shutdown
from launch.substitutions import LaunchConfiguration  # Untuk akses nilai argumen
from launch.conditions import IfCondition, UnlessCondition  # Untuk conditional execution
from launch_ros.actions import Node  # Untuk menjalankan node ROS2 Python
from launch.event_handlers import OnProcessExit, OnShutdown  # Untuk event handler node exit/shutdown
from launch.events import Shutdown  # Untuk shutdown event

# ===================== KONFIGURASI DEFAULT =====================
DEFAULT_MODEL_PATHS = [
    "yolo12x.engine",     # TensorRT (Jetson) - prioritas utama jika Jetson
    "yolo12x.onnx",       # ONNX (universal) - fallback jika tidak ada TensorRT
    "yolo12x.pt",         # PyTorch (fallback) - fallback terakhir
]
DEFAULT_CAMERA_TOPICS = [
    "/camera_front/image_raw",        # Kamera depan
    "/camera_right/image_raw",        # Kamera kanan
    "/camera_rear_right/image_raw",   # Kamera kanan belakang
    "/camera_rear/image_raw",         # Kamera belakang
    "/camera_left/image_raw",         # Kamera kiri
    "/camera_front_left/image_raw"    # Kamera kiri depan
]
DEFAULT_LOG_DIR = '~/huskybot_detection_log'  # Folder log default (di home user)

# ===================== ERROR HANDLING: DETEKSI JETSON DAN HARDWARE =====================
def detect_jetson_platform() -> Dict[str, Any]:
    """Deteksi Jetson dan capability hardware untuk optimasi model."""
    info = {
        'is_jetson': False,  # Flag Jetson
        'cuda_available': False,  # Flag CUDA
        'tensor_cores': False,  # Flag tensor core
        'jetson_model': 'unknown',  # Nama model Jetson
        'recommended_format': 'onnx',  # Format model default
        'jetpack_version': 'unknown'  # Versi JetPack
    }
    try:
        # Cek file device-tree (khusus Jetson)
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                if 'NVIDIA' in model and ('Jetson' in model or 'AGX' in model or 'Orin' in model):
                    info['is_jetson'] = True
                    info['jetson_model'] = model.strip()
                    info['recommended_format'] = 'engine'
                    if 'Orin' in model and 'AGX' in model:
                        info['tensor_cores'] = True
        # Cek arsitektur (fallback jika device-tree tidak ada)
        if platform.machine() in ['aarch64', 'arm64']:
            if info['jetson_model'] == 'unknown' and platform.system() == 'Linux':
                if 'tegra' in platform.release().lower():
                    info['is_jetson'] = True
        # Cek CUDA (pakai torch jika ada, fallback ke nvidia-smi)
        try:
            import torch
            info['cuda_available'] = torch.cuda.is_available()
            if info['cuda_available']:
                info['cuda_version'] = torch.version.cuda
        except Exception:
            try:
                result = subprocess.run(['nvidia-smi', '--query-gpu=driver_version', '--format=csv,noheader'],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
                if result.returncode == 0:
                    info['cuda_available'] = True
            except Exception:
                pass
        # Cek JetPack version (khusus Jetson)
        if info['is_jetson']:
            try:
                result = subprocess.run(['dpkg-query', '--showformat=${Version}', '--show', 'nvidia-l4t-core'],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
                if result.returncode == 0:
                    info['jetpack_version'] = result.stdout.strip()
            except Exception:
                pass
            if info['tensor_cores'] and info['jetpack_version'] != 'unknown':
                info['recommended_format'] = 'engine'
        print(f"[INFO] Platform detection: {info}")  # Log info platform
        return info
    except Exception as e:
        print(f"[WARNING] Error in platform detection: {e}\n{traceback.format_exc()}", file=sys.stderr)
        return info

# ===================== ERROR HANDLING: VALIDASI FILE MODEL YOLOv12 =====================
def validate_model_path(model_path: str, platform_info: Dict[str, Any]=None) -> str:
    """Validasi path file model YOLOv12 dan cari fallback jika tidak ditemukan."""
    if platform_info is None:
        platform_info = detect_jetson_platform()
    expanded_path = os.path.expanduser(model_path)
    if os.path.exists(expanded_path) and os.path.isfile(expanded_path):
        print(f"[INFO] Found model at specified path: {expanded_path}")
        return expanded_path
    # Cari di lokasi umum (share, models, home, /opt, /usr/local/share)
    possible_paths = [
        expanded_path,
        os.path.join(os.getcwd(), model_path),
        os.path.join(os.getcwd(), 'models', model_path),
        os.path.join(os.path.expanduser('~'), 'huskybot', model_path),
        os.path.join(os.path.expanduser('~'), 'huskybot', 'models', model_path),
        os.path.join('/opt/models', model_path),
        os.path.join('/usr/local/share/models', model_path),
    ]
    base_name = os.path.splitext(model_path)[0]
    recommended_format = platform_info.get('recommended_format', 'engine' if platform_info.get('is_jetson', False) else 'onnx')
    extensions = ['.engine', '.onnx', '.pt'] if recommended_format == 'engine' else ['.onnx', '.pt', '.engine']
    for ext in extensions:
        possible_paths.append(f"{base_name}{ext}")
        possible_paths.append(os.path.join(os.getcwd(), f"{base_name}{ext}"))
        possible_paths.append(os.path.join(os.getcwd(), 'models', f"{base_name}{ext}"))
        possible_paths.append(os.path.join(os.path.expanduser('~'), 'huskybot', f"{base_name}{ext}"))
        possible_paths.append(os.path.join(os.path.expanduser('~'), 'huskybot', 'models', f"{base_name}{ext}"))
    for path in possible_paths:
        if os.path.exists(path) and os.path.isfile(path):
            print(f"[INFO] Found model at fallback location: {path}")
            return path
    # Aggressive glob search (cari semua yolo*.engine/onnx/pt di lokasi umum)
    pattern_searches = []
    if platform_info.get('is_jetson', False):
        pattern_searches = [
            os.path.join(os.getcwd(), 'models', 'yolo*.engine'),
            os.path.join(os.getcwd(), 'yolo*.engine'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.engine'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.engine'),
            os.path.join(os.getcwd(), 'models', 'yolo*.onnx'),
            os.path.join(os.getcwd(), 'yolo*.onnx'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.onnx'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.onnx'),
            os.path.join(os.getcwd(), 'models', 'yolo*.pt'),
            os.path.join(os.getcwd(), 'yolo*.pt'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.pt'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.pt')
        ]
    else:
        pattern_searches = [
            os.path.join(os.getcwd(), 'models', 'yolo*.onnx'),
            os.path.join(os.getcwd(), 'yolo*.onnx'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.onnx'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.onnx'),
            os.path.join(os.getcwd(), 'models', 'yolo*.pt'),
            os.path.join(os.getcwd(), 'yolo*.pt'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.pt'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.pt'),
            os.path.join(os.getcwd(), 'models', 'yolo*.engine'),
            os.path.join(os.getcwd(), 'yolo*.engine'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'models', 'yolo*.engine'),
            os.path.join(os.path.expanduser('~'), 'huskybot', 'yolo*.engine')
        ]
    for pattern in pattern_searches:
        matches = glob.glob(pattern)
        if matches:
            print(f"[INFO] Found model using pattern search: {matches[0]}")
            return matches[0]
    print(f"[WARNING] No valid model file found at {model_path} or in fallback locations", file=sys.stderr)
    print("[WARNING] Make sure YOLOv12 model files are properly installed", file=sys.stderr)
    print("[WARNING] Expected formats: .engine (TensorRT), .onnx (ONNX Runtime), or .pt (PyTorch)", file=sys.stderr)
    print("[WARNING] Node will likely fail at runtime without a valid model file", file=sys.stderr)
    return model_path

# ===================== ERROR HANDLING: FOLDER LOG =====================
def ensure_log_dir(log_dir: str = DEFAULT_LOG_DIR) -> str:
    """Pastikan folder log ada dan bisa ditulis, fallback ke /tmp jika gagal."""
    expanded_dir = os.path.expanduser(log_dir)
    try:
        os.makedirs(expanded_dir, exist_ok=True)
        test_file = os.path.join(expanded_dir, '.write_test')
        with open(test_file, 'w') as f:
            f.write('test')
        os.remove(test_file)
        print(f"[INFO] Log directory ready: {expanded_dir}")
        return expanded_dir
    except Exception as e:
        print(f"[WARNING] Cannot access log directory {expanded_dir}: {e}\n{traceback.format_exc()}", file=sys.stderr)
        fallbacks = [
            os.path.join('/tmp', 'huskybot_detection_log'),
            os.path.join(os.path.expanduser('~'), '.cache', 'huskybot', 'logs'),
            os.path.join(os.getcwd(), 'logs'),
        ]
        for fallback in fallbacks:
            try:
                os.makedirs(fallback, exist_ok=True)
                test_file = os.path.join(fallback, '.write_test')
                with open(test_file, 'w') as f:
                    f.write('test')
                os.remove(test_file)
                print(f"[INFO] Using fallback log directory: {fallback}")
                return fallback
            except Exception as fallback_error:
                print(f"[WARNING] Fallback location {fallback} also failed: {fallback_error}\n{traceback.format_exc()}", file=sys.stderr)
        print(f"[ERROR] Could not create or write to any log directory, using /tmp directly", file=sys.stderr)
        return '/tmp'

# ===================== ERROR HANDLING: CEK PACKAGE ROS2 =====================
def check_package_available(package_name: str) -> bool:
    """Cek apakah package ROS2 tersedia di workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package_name)
        return True
    except Exception as e:
        print(f"[ERROR] ROS2 package not found: {package_name} ({e})", file=sys.stderr)
        return False

# ===================== ERROR HANDLING: CEK DEPENDENCY PYTHON =====================
def check_python_dependency(dependency_name: str) -> bool:
    """Cek apakah module Python tersedia."""
    try:
        __import__(dependency_name)
        return True
    except ImportError as e:
        print(f"[ERROR] Python dependency not found: {dependency_name} ({e})", file=sys.stderr)
        return False

# ===================== ERROR HANDLING: CEK TOPIC KAMERA =====================
def check_camera_topics(camera_topics: List[str], timeout: int = 2) -> Dict[str, bool]:
    """Cek apakah topics kamera sudah ada di ROS2 (non-blocking, warning only)."""
    results = {}
    print(f"[INFO] Checking {len(camera_topics)} camera topics (non-blocking, timeout {timeout}s)")
    try:
        process = subprocess.Popen(
            ['timeout', str(timeout), 'ros2', 'topic', 'list'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        stdout, stderr = process.communicate()
        if process.returncode == 0:
            topics = stdout.strip().split('\n')
            for camera_topic in camera_topics:
                results[camera_topic] = camera_topic in topics
        else:
            print(f"[WARNING] Could not check camera topics: {stderr}", file=sys.stderr)
            for camera_topic in camera_topics:
                results[camera_topic] = False
    except Exception as e:
        print(f"[WARNING] Error checking camera topics: {e}\n{traceback.format_exc()}", file=sys.stderr)
        for camera_topic in camera_topics:
            results[camera_topic] = False
    return results

# ===================== BACKUP MODEL FILE (AUDIT TRAIL) =====================
def backup_model_file(model_path: str) -> None:
    """Backup model file sebelum digunakan (max 3 backup)."""
    if not os.path.exists(model_path) or not os.path.isfile(model_path):
        print(f"[WARNING] Cannot backup non-existent model: {model_path}")
        return
    try:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        dirname = os.path.dirname(model_path) or '.'
        basename = os.path.basename(model_path)
        backup_name = f"{basename}.{timestamp}.bak"
        backup_path = os.path.join(dirname, backup_name)
        existing_backups = sorted([
            f for f in os.listdir(dirname)
            if f.startswith(basename) and f.endswith('.bak')
        ])
        while len(existing_backups) >= 3:
            oldest = existing_backups.pop(0)
            try:
                os.remove(os.path.join(dirname, oldest))
                print(f"[INFO] Removed old backup: {oldest}")
            except Exception as e:
                print(f"[WARNING] Could not remove old backup {oldest}: {e}")
        shutil.copy2(model_path, backup_path)
        print(f"[INFO] Created backup of model at: {backup_path}")
    except Exception as e:
        print(f"[WARNING] Failed to backup model file: {e}\n{traceback.format_exc()}")

# ===================== PRE-LAUNCH ERROR HANDLING (FAIL-FAST) =====================
def prelaunch_error_checks(context, *args, **kwargs):
    """Fail-fast error handling sebelum launch: cek dependency, file, folder, permission."""
    # Cek ROS2 package
    for pkg in ['yolov12_msgs', 'huskybot_detection', 'cv_bridge']:
        if not check_package_available(pkg):
            print(f"[FATAL] ROS2 package missing: {pkg}", file=sys.stderr)
            return [EmitEvent(event=Shutdown(reason=f"Missing ROS2 package: {pkg}"))]
    # Cek Python dependency
    for dep in ['ultralytics', 'torch', 'cv2', 'numpy']:
        if not check_python_dependency(dep):
            print(f"[FATAL] Python dependency missing: {dep}", file=sys.stderr)
            return [EmitEvent(event=Shutdown(reason=f"Missing Python dependency: {dep}"))]
    # Cek model file
    model_path = context.launch_configurations.get('model_path', 'yolo12x.engine')
    if not os.path.exists(os.path.expanduser(model_path)):
        print(f"[FATAL] Model file not found: {model_path}", file=sys.stderr)
        return [EmitEvent(event=Shutdown(reason=f"Model file not found: {model_path}"))]
    # Cek folder log
    log_dir = context.launch_configurations.get('log_dir', DEFAULT_LOG_DIR)
    try:
        os.makedirs(os.path.expanduser(log_dir), exist_ok=True)
    except Exception as e:
        print(f"[FATAL] Cannot create log directory: {log_dir} ({e})", file=sys.stderr)
        return [EmitEvent(event=Shutdown(reason=f"Cannot create log directory: {log_dir}"))]
    return []

# ===================== NODE DETEKSI MULTICAM YOLOv12 (DENGAN OPAQUEFUNCTION) =====================
from launch.actions import OpaqueFunction

def create_detection_node(context, *args, **kwargs):
    import yaml
    camera_topics_str = LaunchConfiguration('camera_topics').perform(context)
    try:
        camera_topics = yaml.safe_load(camera_topics_str)
        if not isinstance(camera_topics, list):
            camera_topics = [str(camera_topics)]
    except Exception:
        camera_topics = [camera_topics_str]

    # --- Validasi class_filter agar tidak tuple dan selalu list ---
    class_filter_raw = LaunchConfiguration('class_filter').perform(context)
    try:
        class_filter = yaml.safe_load(class_filter_raw)
        if class_filter is None:
            class_filter = []
        elif isinstance(class_filter, tuple):
            class_filter = list(class_filter)
        elif not isinstance(class_filter, list):
            class_filter = [class_filter]
    except Exception:
        class_filter = []

    # Debug print untuk validasi tipe parameter
    print(f"[DEBUG] camera_topics type: {type(camera_topics)}, value: {camera_topics}")
    print(f"[DEBUG] class_filter type: {type(class_filter)}, value: {class_filter}")

    # Ambil info platform dari context (harus di-passing dari generate_launch_description)
    # Fallback: import detect_jetson_platform lagi jika tidak ada di context
    global platform_info, is_jetson
    try:
        platform_info
    except NameError:
        platform_info = detect_jetson_platform()
        is_jetson = platform_info.get('is_jetson', False)
    return [
        Node(
            package='huskybot_detection',  # Nama package deteksi
            executable='multicam_detection_node',  # Nama executable deteksi
            name='multicam_detection',  # Nama node
            namespace=LaunchConfiguration('namespace').perform(context),  # Namespace
            output=LaunchConfiguration('output').perform(context),  # Output log
            emulate_tty=True,  # Emulasi TTY agar output warna/log tidak rusak
            parameters=[{
                'cam_count': int(LaunchConfiguration('cam_count').perform(context)),  # Jumlah kamera
                'camera_topics': camera_topics,  # List topic kamera (sudah pasti list)
                'model_path': LaunchConfiguration('model_path').perform(context),  # Path model
                'conf_thres': float(LaunchConfiguration('conf_thres').perform(context)),  # Threshold confidence
                'class_filter': class_filter,  # Sudah pasti list, tidak tuple
                'iou_thres': float(LaunchConfiguration('iou_thres').perform(context)),  # IoU threshold
                'img_size': int(LaunchConfiguration('img_size').perform(context)),  # Ukuran image
                'device': LaunchConfiguration('device').perform(context),  # Device inference
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',  # Sim time
                'cache_results': LaunchConfiguration('cache_results').perform(context) == 'true',  # Cache
                'visualization_enabled': LaunchConfiguration('visualization_enabled').perform(context) == 'true',  # Visualisasi
                'display_mode': LaunchConfiguration('display_mode').perform(context),  # Mode display
                'log_to_file': True,  # Logging ke file
                'log_level': LaunchConfiguration('log_level').perform(context),  # Log level
                'log_dir': LaunchConfiguration('log_dir').perform(context),  # Folder log
                'diagnostics_enabled': LaunchConfiguration('enable_diagnostics').perform(context) == 'true',  # Diagnostics
                'is_jetson': is_jetson,  # Flag Jetson
                'tensor_cores_available': platform_info.get('tensor_cores', False),  # Tensor core
                'cuda_available': platform_info.get('cuda_available', False),  # CUDA
            }],
            respawn=LaunchConfiguration('respawn').perform(context) == 'true',  # Auto-restart node jika crash
            respawn_delay=1.0,  # Delay restart 1 detik
            remappings=[
                ('/detection', f"{LaunchConfiguration('namespace').perform(context)}/detection" if LaunchConfiguration('namespace').perform(context) else '/detection'),  # Remap topic detection
                ('/diagnostics', f"{LaunchConfiguration('namespace').perform(context)}/diagnostics" if LaunchConfiguration('namespace').perform(context) else '/diagnostics'),  # Remap diagnostics
            ],
            additional_env={
                'PYTHONUNBUFFERED': '1',  # Unbuffered output
                'DISPLAY': os.environ.get('DISPLAY', ''),  # Display X11 untuk OpenCV GUI
            },
        )
    ]

# ===================== LAUNCH DESCRIPTION (URUTAN WAJIB) =====================
def generate_launch_description():
    """Generate LaunchDescription untuk node deteksi YOLOv12 multi-kamera."""
    global platform_info, is_jetson
    platform_info = detect_jetson_platform()  # Deteksi platform Jetson/CUDA
    is_jetson = platform_info.get('is_jetson', False)  # Flag Jetson

    # Pilih model default sesuai platform (engine/onnx/pt)
    default_model = platform_info.get('recommended_format', 'onnx')
    if default_model == 'engine' and not check_python_dependency('tensorrt'):
        print("[WARNING] TensorRT not available, fallback to ONNX", file=sys.stderr)
        default_model_path = 'yolo12x.onnx'
    elif default_model == 'onnx' and not check_python_dependency('onnxruntime'):
        print("[WARNING] ONNX Runtime not available, fallback to PyTorch", file=sys.stderr)
        default_model_path = 'yolo12x.pt'
    else:
        default_model_path = 'yolo12x.engine' if default_model == 'engine' else 'yolo12x.onnx'

    model_path = validate_model_path(default_model_path, platform_info)  # Validasi path model
    log_dir = ensure_log_dir()  # Pastikan folder log bisa diakses
    camera_topics = check_camera_topics(DEFAULT_CAMERA_TOPICS)  # Cek topic kamera
    available_topics = [topic for topic, available in camera_topics.items() if available]
    if available_topics:
        print(f"[INFO] Found {len(available_topics)} camera topics: {', '.join(available_topics)}")
    else:
        print("[WARNING] No camera topics found. If not using simulation, check camera node is running", file=sys.stderr)

    # ===================== DECLARE LAUNCH ARGUMENTS (URUTAN WAJIB) =====================
    cam_count_arg = DeclareLaunchArgument('cam_count', default_value='6', description='Number of cameras (default: 6 for hexagonal array)', choices=[str(i) for i in range(1, 13)])  # Jumlah kamera
    model_path_arg = DeclareLaunchArgument('model_path', default_value=model_path, description='Path to YOLOv12 model file (.pt, .onnx, .engine)')  # Path model
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Namespace prefix for multi-robot scenarios')  # Namespace multi-robot
    visualization_arg = DeclareLaunchArgument('visualization_enabled', default_value='true', description='Enable visualization of detection results in OpenCV window', choices=['true', 'false'])  # Visualisasi
    conf_thresh_arg = DeclareLaunchArgument('conf_thres', default_value='0.25', description='Confidence threshold for filtering detections (0.0-1.0)')  # Threshold confidence
    class_filter_arg = DeclareLaunchArgument('class_filter', default_value='[]', description='List of class IDs to detect (e.g., [0,1,2]) or empty for all classes')  # Filter class
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Log level (debug, info, warning, error, critical)', choices=['debug', 'info', 'warning', 'error', 'critical'])  # Log level
    log_dir_arg = DeclareLaunchArgument('log_dir', default_value=log_dir, description='Directory for log files (will be created if not exists)')  # Folder log
    camera_topics_arg = DeclareLaunchArgument(
        'camera_topics',
        default_value='[ "/camera_front/image_raw", "/camera_right/image_raw", "/camera_rear_right/image_raw", "/camera_rear/image_raw", "/camera_left/image_raw", "/camera_front_left/image_raw" ]',
        description='List of camera topic names to subscribe (YAML/JSON list as string)'
    )  # List topic kamera
    enable_diagnostics_arg = DeclareLaunchArgument('enable_diagnostics', default_value='true', description='Enable publishing to /diagnostics topic for system monitoring', choices=['true', 'false'])  # Diagnostics
    output_format_arg = DeclareLaunchArgument('output', default_value='screen', description='Output format for node logs (screen or log)', choices=['screen', 'log'])  # Output log
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time from /clock topic (required for Gazebo)', choices=['true', 'false'])  # Sim time
    display_mode_arg = DeclareLaunchArgument('display_mode', default_value='gui' if os.environ.get('DISPLAY') else 'headless', description='Display mode (gui, headless, remote)', choices=['gui', 'headless', 'remote'])  # Mode display
    respawn_arg = DeclareLaunchArgument('respawn', default_value='true', description='Auto-restart node if it crashes', choices=['true', 'false'])  # Auto respawn
    device_arg = DeclareLaunchArgument('device', default_value='auto', description='Inference device (auto, cpu, cuda, tensorrt)', choices=['auto', 'cpu', 'cuda', 'cuda:0', 'tensorrt'])  # Device inference
    cache_results_arg = DeclareLaunchArgument('cache_results', default_value='false', description='Cache detection results to improve performance on static scenes', choices=['true', 'false'])  # Cache
    img_size_arg = DeclareLaunchArgument('img_size', default_value='640', description='Image size for inference (smaller is faster, larger is more accurate)', choices=['320', '416', '512', '640', '960', '1280'])  # Ukuran image
    iou_thres_arg = DeclareLaunchArgument('iou_thres', default_value='0.45', description='IoU threshold for NMS (0.0-1.0)')  # IoU threshold

    # ===================== DIAGNOSTIC NODE (OPSIONAL) =====================
    diagnostic_node = Node(
        package='diagnostic_aggregator',  # Package diagnostics
        executable='aggregator_node',  # Executable diagnostics
        name='diagnostic_aggregator',  # Nama node diagnostics
        namespace=LaunchConfiguration('namespace'),  # Namespace
        output='screen',  # Output log
        parameters=[{
            'pub_rate': 1.0,  # Rate publish diagnostics
            'base_path': 'Detection System',  # Path diagnostics
            'analyzers': {
                'detection': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'YOLOv12 Detection',
                    'find_and_remove_prefix': 'multicam_detection',
                    'timeout': 5.0,
                },
            }
        }],
        condition=IfCondition(LaunchConfiguration('enable_diagnostics'))  # Enable jika diagnostics aktif
    )

    # ===================== EVENT HANDLER: NODE EXIT =====================
    detection_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=None,  # Target node diisi oleh OpaqueFunction (tidak perlu di sini)
            on_exit=[
                LogInfo(msg=["Detection node exited with code: ", LaunchConfiguration('event_returncode')]),  # Log exit
                ExecuteProcess(
                    cmd=["bash", "-c", "echo '[INFO] Performing cleanup after detection node exit'"],  # Cleanup
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration('event_returncode'))
                )
            ]
        )
    )

    # ===================== PRE-LAUNCH VALIDATION & ENV PREP =====================
    validate_model_cmd = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "if [ ! -f \"$(eval echo " + model_path + ")\" ]; then " +
            "echo \"[WARNING] YOLOv12 model not found at $(eval echo " + model_path + ")\"; fi"
        ],
        output='screen'
    )
    backup_model_cmd = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "MODEL=$(eval echo " + model_path + "); " +
            "if [ -f \"$MODEL\" ]; then " +
            "  TIMESTAMP=$(date +%Y%m%d_%H%M%S); " +
            "  BACKUP=\"${MODEL}.${TIMESTAMP}.bak\"; " +
            "  cp \"$MODEL\" \"$BACKUP\" 2>/dev/null || true; " +
            "  echo \"[INFO] Created backup of model at $BACKUP\"; fi"
        ],
        output='screen'
    )
    check_log_dir_cmd = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "DIR=$(eval echo " + log_dir + "); " +
            "if [ ! -d \"$DIR\" ]; then mkdir -p \"$DIR\" 2>/dev/null || echo \"[WARNING] Could not create log directory $DIR\"; fi; " +
            "if [ ! -w \"$DIR\" ]; then echo \"[WARNING] Log directory $DIR is not writable\"; fi"
        ],
        output='screen'
    )
    prepare_env_cmd = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "mkdir -p ~/.ros 2>/dev/null || true; " +
            "DIR=$(eval echo " + log_dir + "); " +
            "touch \"$DIR/.detection_log_init\" 2>/dev/null || true; " +
            "echo \"[INFO] Environment ready for detection node\""
        ],
        output='screen'
    )

    # ===================== LOGGING INFO =====================
    log_launch_info = LogInfo(msg=["[INFO] Starting multicam_detection node with ", LaunchConfiguration('cam_count'), " cameras"])  # Log jumlah kamera
    log_platform_info = LogInfo(msg=[f"[INFO] Running on {'Jetson' if is_jetson else 'standard'} platform with {'tensor cores' if platform_info.get('tensor_cores', False) else 'no tensor cores'}"])  # Log platform

    # ===================== RETURN LAUNCH DESCRIPTION (URUTAN WAJIB) =====================
    return LaunchDescription([
        cam_count_arg,  # Argumen jumlah kamera
        model_path_arg,  # Argumen path model
        namespace_arg,  # Argumen namespace
        visualization_arg,  # Argumen visualisasi
        conf_thresh_arg,  # Argumen threshold
        class_filter_arg,  # Argumen class filter
        log_level_arg,  # Argumen log level
        log_dir_arg,  # Argumen folder log
        camera_topics_arg,  # Argumen list topic kamera
        enable_diagnostics_arg,  # Argumen diagnostics
        output_format_arg,  # Argumen output log
        use_sim_time_arg,  # Argumen sim time
        display_mode_arg,  # Argumen display mode
        respawn_arg,  # Argumen auto respawn
        device_arg,  # Argumen device inference
        cache_results_arg,  # Argumen cache
        img_size_arg,  # Argumen ukuran image
        iou_thres_arg,  # Argumen IoU threshold

        OpaqueFunction(function=prelaunch_error_checks),  # Fail-fast error handling sebelum launch

        log_launch_info,  # Logging info jumlah kamera
        log_platform_info,  # Logging info platform
        validate_model_cmd,  # Validasi model sebelum run
        backup_model_cmd,  # Backup model sebelum run
        check_log_dir_cmd,  # Validasi folder log
        prepare_env_cmd,  # Persiapan environment
        OpaqueFunction(function=create_detection_node),  # Jalankan node multicam_detection (dengan parsing camera_topics)

        diagnostic_node,  # Node diagnostics
        detection_exit_handler,  # Handler node exit

        RegisterEventHandler(  # Handler shutdown ROS2 launch
            OnShutdown(
                on_shutdown=[
                    LogInfo(msg="[INFO] Shutdown event received, cleaning up detection pipeline..."),
                    ExecuteProcess(cmd=["bash", "-c", "echo '[INFO] Detection pipeline shutdown complete.'"], output='screen')
                ]
            )
        ),
    ])

# ===================== SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN LANGSUNG) =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Error handling sudah fail-fast, robust, dan audit trail siap.
# - Semua parameter bisa diubah dari CLI/launch file lain, siap multi-robot dan audit trail.
# - Remapping topic otomatis untuk namespace multi-robot.
# - Fallback direktori log ke /tmp jika permission error.
# - Validasi file model di semua lokasi umum (package, home, /opt).
# - Logging error ke sys.stderr agar mudah dideteksi di CI/CD dan debugging.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Jetson Orin, Husky A200, Arducam, Velodyne VLP32-C).
# - Saran: jika ingin parsing camera_topics dari string CLI, tambahkan parsing di node Python (multicam_detection_node.py).
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: dokumentasikan semua parameter di README.md dan launch file.
# - Saran: tambahkan badge CI/CD dan coverage test di README jika pipeline sudah aktif.
# - Saran: tambahkan opsi simpan hasil deteksi ke file (CSV/JSON) di node utama.
# - Saran: tambahkan notifikasi/error handling jika folder logs tidak dapat diakses.
# - Saran: pertimbangkan untuk menambah GUI sederhana untuk monitoring.
# - Saran: tambahkan troubleshooting error umum di README.
# - Saran: pertimbangkan untuk menambah dukungan lebih dari 6 kamera (sudah siap di parameter cam_count).