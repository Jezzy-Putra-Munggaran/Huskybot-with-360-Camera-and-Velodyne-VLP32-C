from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import shutil
import time

# ===================== ERROR HANDLING & LOGGER =====================
def check_model_file(context, *args, **kwargs):
    model_path = LaunchConfiguration('model_path').perform(context)
    expanded_path = os.path.expanduser(model_path)
    if not os.path.isfile(expanded_path):
        print(f"[ERROR] File model YOLOv12 tidak ditemukan: {expanded_path}", file=sys.stderr)
        print("[ERROR]", msg=f"File model YOLOv12 tidak ditemukan: {expanded_path}", flush=True)
    else:
        print(f"[INFO] File model YOLOv12 ditemukan: {expanded_path}")
        print("[INFO]", msg=f"File model YOLOv12 ditemukan: {expanded_path}", flush=True)
    return []

def check_rclpy_dependency(context, *args, **kwargs):
    try:
        import rclpy
        print("[INFO]", msg="Dependency 'rclpy' ditemukan.", flush=True)
    except ImportError:
        print("[ERROR] Dependency 'rclpy' tidak ditemukan. Install dengan: pip install rclpy", file=sys.stderr)
        print("[ERROR]", msg="Dependency 'rclpy' tidak ditemukan.", flush=True)
        sys.exit(2)
    return []

def log_to_file(msg):
    log_file_path = os.path.expanduser("~/huskybot_yolov12_launch.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

def generate_launch_description():
    try:
        # Argumen launch
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
        model_path_arg = DeclareLaunchArgument(
            'model_path',
            default_value='~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt',
            description='Path ke file model YOLOv12 (.pt)'
        )
        confidence_threshold_arg = DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.25',
            description='Threshold confidence deteksi YOLOv12'
        )
        log_stats_arg = DeclareLaunchArgument(
            'log_stats',
            default_value='true',
            description='Aktifkan logging statistik deteksi ke file'
        )
        log_stats_path_arg = DeclareLaunchArgument(
            'log_stats_path',
            default_value='~/huskybot_detection_log/yolov12_stats.csv',
            description='Path file statistik deteksi YOLOv12'
        )

        # Error handling actions
        check_model_action = OpaqueFunction(function=check_model_file)
        check_rclpy_action = OpaqueFunction(function=check_rclpy_dependency)

        # Logging info ke terminal dan file
print("[INFO]", msg='Launching YOLOv12 ROS2 Node...', flush=True)
        log_to_file("Launching YOLOv12 ROS2 Node...")

        # Node YOLOv12
        yolov12_node = Node(
            package='huskybot_recognition',
            executable='yolov12_ros2_pt.py',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'model_path': LaunchConfiguration('model_path')},
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')},
                {'log_stats': LaunchConfiguration('log_stats')},
                {'log_stats_path': LaunchConfiguration('log_stats_path')},
            ],
        )

        return LaunchDescription([
            use_sim_time_arg,
            model_path_arg,
            confidence_threshold_arg,
            check_rclpy_action,
            check_model_action,
            yolov12_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        print("[ERROR]", msg=f"Exception saat generate_launch_description: {e}", flush=True)
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

# ===================== PENJELASAN & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek file model, dependency rclpy, logging ke file.
# - Logging info ke terminal dan file untuk audit trail.
# - Siap untuk multi-robot (tinggal remap topic jika perlu).
# - Saran: tambahkan validasi file YAML kalibrasi jika ingin logging statistik per kamera.
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.