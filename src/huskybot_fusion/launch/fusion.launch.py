from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, LogWarn, LogError, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys
import time

# ===================== ERROR HANDLING & LOGGER =====================
def check_calibration_file(context, *args, **kwargs):
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    expanded = os.path.expandvars(os.path.expanduser(calibration_file))
    if not os.path.isfile(expanded):
        print(f"[ERROR] File kalibrasi extrinsic tidak ditemukan: {expanded}", file=sys.stderr)
        LogError(msg=f"File kalibrasi extrinsic tidak ditemukan: {expanded}")
        sys.exit(2)
    else:
        print(f"[INFO] File kalibrasi extrinsic ditemukan: {expanded}")
        LogInfo(msg=f"File kalibrasi extrinsic ditemukan: {expanded}")
    return []

def log_to_file(msg):
    log_file_path = os.path.expanduser("~/huskybot_fusion_launch.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

def generate_launch_description():
    try:
        # ===================== ARGUMEN LAUNCH =====================
        calibration_file_arg = DeclareLaunchArgument(
            'calibration_file',
            default_value='/home/jezzy/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml',
            description='Path ke file kalibrasi extrinsic lidar-ke-kamera'
        )
        confidence_threshold_arg = DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
            description='Threshold confidence minimum untuk publish objek 3D'
        )
        fusion_method_arg = DeclareLaunchArgument(
            'fusion_method',
            default_value='nearest',
            description='Metode asosiasi objek fusion (nearest/iou/centroid)'
        )
        namespace_arg = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace ROS2 untuk multi-robot (opsional)'
        )
        log_file_arg = DeclareLaunchArgument(
            'log_file',
            default_value='',
            description='Path file log untuk fusion node (opsional, kosong = tidak log ke file)'
        )

        # Error handling actions
        check_calib_action = OpaqueFunction(function=check_calibration_file)

        # Logging info ke terminal dan file
        log_launch = LogInfo(msg='Launching Fusion Node (kamera 360° + LiDAR)...')
        log_to_file("Launching Fusion Node (kamera 360° + LiDAR)...")

        # ===================== NODE FUSION =====================
        fusion_node = Node(
            package='huskybot_fusion',
            executable='fusion_node.py',
            name='fusion_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'calibration_file': LaunchConfiguration('calibration_file')},
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')},
                {'fusion_method': LaunchConfiguration('fusion_method')},
                {'log_file': LaunchConfiguration('log_file')}
            ],
            # remappings=[('/velodyne_points', '/velodyne_points'), ...] # (Opsional) remap topic jika perlu
        )

        return LaunchDescription([
            calibration_file_arg,
            confidence_threshold_arg,
            fusion_method_arg,
            namespace_arg,
            log_file_arg,
            check_calib_action,
            log_launch,
            fusion_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

# ===================== REVIEW & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek file kalibrasi extrinsic, logging ke file.
# - Logging info ke terminal dan file untuk audit trail.
# - Siap untuk multi-robot (tinggal remap namespace jika perlu).
# - Saran: tambahkan remapping topic jika workspace Anda perlu remap topic sensor.
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.