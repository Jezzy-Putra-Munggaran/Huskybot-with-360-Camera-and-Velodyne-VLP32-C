from launch import LaunchDescription  # Import utama untuk LaunchDescription ROS2
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Untuk deklarasi argumen dan fungsi custom
from launch.substitutions import LaunchConfiguration  # Untuk ambil nilai argumen launch
from launch_ros.actions import Node  # Untuk menjalankan node ROS2 Python
import os  # Untuk operasi file (cek file kalibrasi)
import sys  # Untuk akses error output
import time  # Untuk timestamp log file

# ===================== ERROR HANDLING & LOGGER =====================
def check_calibration_file(context, *args, **kwargs):  # Fungsi validasi file kalibrasi extrinsic
    calibration_file = LaunchConfiguration('calibration_file').perform(context)  # Ambil argumen calibration_file
    expanded = os.path.expandvars(os.path.expanduser(calibration_file))  # Expand ~ dan env var
    if not os.path.isfile(expanded):  # Jika file tidak ada
        print(f"[ERROR] File kalibrasi extrinsic tidak ditemukan: {expanded}", file=sys.stderr)  # Print error ke stderr
        log_to_file(f"[ERROR] File kalibrasi extrinsic tidak ditemukan: {expanded}")  # Log ke file
        sys.exit(2)  # Exit dengan kode error
    else:
        print(f"[INFO] File kalibrasi extrinsic ditemukan: {expanded}")  # Info ke terminal
        log_to_file(f"[INFO] File kalibrasi extrinsic ditemukan: {expanded}")  # Log ke file
    return []  # Harus return list kosong untuk OpaqueFunction

def log_to_file(msg):  # Fungsi logging ke file
    log_file_path = os.path.expanduser("~/huskybot_fusion_launch.log")  # Path file log
    try:
        with open(log_file_path, "a") as logf:  # Append log
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")  # Format log
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)  # Warning jika gagal log

def build_fusion_node(context, *args, **kwargs):  # Fungsi untuk membangun node fusion dengan parameter log_file opsional
    params = [
        {'calibration_file': LaunchConfiguration('calibration_file').perform(context)},  # Path file kalibrasi
        {'confidence_threshold': LaunchConfiguration('confidence_threshold').perform(context)},  # Threshold confidence
        {'fusion_method': LaunchConfiguration('fusion_method').perform(context)}  # Metode fusion
    ]
    log_file = LaunchConfiguration('log_file').perform(context)  # Ambil argumen log_file
    if log_file and log_file.strip() != '':  # Jika log_file tidak kosong
        params.append({'log_file': log_file})  # Tambahkan parameter log_file
    # Bisa tambahkan validasi permission log_file di sini jika ingin lebih robust
    node = Node(
        package='huskybot_fusion',  # Nama package ROS2
        executable='fusion_node',  # [REVISI] Nama entry point Python (bukan .py, sesuai setup.py console_scripts)
        name='fusion_node',  # Nama node di ROS2 graph
        namespace=LaunchConfiguration('namespace').perform(context),  # Namespace (untuk multi-robot)
        output='screen',  # Output ke terminal
        parameters=params,  # Parameter dinamis (termasuk log_file jika ada)
        # remappings=[('/velodyne_points', '/velodyne_points'), ...] # (Opsional) remap topic jika perlu
    )
    return [node]  # Harus return list of actions

def generate_launch_description():  # Fungsi utama generate LaunchDescription
    try:
        # ===================== ARGUMEN LAUNCH =====================
        calibration_file_arg = DeclareLaunchArgument(
            'calibration_file',
            default_value='/mnt/nova_ssd/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml',
            description='Path ke file kalibrasi extrinsic lidar-ke-kamera'  # Penjelasan argumen
        )
        confidence_threshold_arg = DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
            description='Threshold confidence minimum untuk publish objek 3D'  # Penjelasan argumen
        )
        fusion_method_arg = DeclareLaunchArgument(
            'fusion_method',
            default_value='nearest',
            description='Metode asosiasi objek fusion (nearest/iou/centroid)'  # Penjelasan argumen
        )
        namespace_arg = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace ROS2 untuk multi-robot (opsional)'  # Penjelasan argumen
        )
        log_file_arg = DeclareLaunchArgument(
            'log_file',
            default_value='',
            description='Path file log untuk fusion node (opsional, kosong = tidak log ke file)'  # Penjelasan argumen
        )

        # ===================== ERROR HANDLING ACTIONS =====================
        check_calib_action = OpaqueFunction(function=check_calibration_file)  # Cek file kalibrasi sebelum node jalan

        # ===================== LOGGING INFO =====================
        print("[INFO] Launching Fusion Node (kamera 360° + LiDAR)...", flush=True)  # Info ke terminal
        log_to_file("Launching Fusion Node (kamera 360° + LiDAR)...")  # Log ke file

        # ===================== NODE FUSION (DENGAN PARAMETER LOG_FILE OPSIONAL) =====================
        fusion_node_action = OpaqueFunction(function=build_fusion_node)  # Node fusion dengan log_file hanya jika tidak kosong

        # ===================== RETURN LAUNCH DESCRIPTION =====================
        return LaunchDescription([
            calibration_file_arg,  # Argumen file kalibrasi
            confidence_threshold_arg,  # Argumen threshold confidence
            fusion_method_arg,  # Argumen metode fusion
            namespace_arg,  # Argumen namespace
            log_file_arg,  # Argumen file log
            check_calib_action,  # Action cek file kalibrasi
            fusion_node_action  # Node fusion utama (dengan log_file opsional)
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)  # Print fatal error
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")  # Log fatal error
        sys.exit(99)  # Exit dengan kode error

# ===================== PENJELASAN & SARAN PENINGKATAN =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Parameter log_file hanya ditambahkan jika tidak kosong, sehingga tidak ada warning "Parameter file path is not a file: ."
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek file kalibrasi extrinsic, logging ke file, exit jika error.
# - Logging info ke terminal dan file untuk audit trail.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Siap untuk multi-robot (tinggal remap namespace jika perlu).
# - Saran: tambahkan remapping topic jika workspace Anda perlu remap topic sensor.
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.
# - Saran: tambahkan validasi permission file kalibrasi/log agar tidak silent fail.
# - Saran: tambahkan argumen enable_debug agar bisa aktifkan debug log dari CLI.
# - Saran: dokumentasikan semua argumen di README dan contoh command.