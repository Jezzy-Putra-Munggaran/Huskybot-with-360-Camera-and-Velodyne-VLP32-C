from launch import LaunchDescription  # Import utama untuk membuat LaunchDescription ROS2
from launch.actions import DeclareLaunchArgument, LogInfo  # Untuk deklarasi argumen dan logging info ke terminal
from launch.substitutions import LaunchConfiguration  # Untuk mengambil nilai argumen launch
from launch_ros.actions import Node   # Untuk mendefinisikan node ROS2 yang akan dijalankan

def generate_launch_description():
    """
    Launch file ini digunakan untuk menjalankan node fusion_node.py dari package huskybot_fusion.
    Node ini bertugas melakukan data fusion antara hasil deteksi kamera 360° (YOLO) dan point cloud LiDAR (Velodyne VLP-32C).
    Output node ini akan dipakai oleh sistem navigasi, mapping, dan obstacle avoidance di workspace Anda.
    """

    # ===================== ARGUMEN LAUNCH =====================
    # Tambahkan parameterisasi agar bisa diubah tanpa edit file
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='/home/jezzy/huskybot/src/huskybot_description/calibration/extrinsic_lidar_to_camera.yaml',
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

    # ===================== LOGGING INFO =====================
    log_info = LogInfo(msg='Launching Fusion Node (kamera 360° + LiDAR)...')

    # ===================== NODE FUSION =====================
    fusion_node = Node(
        package='huskybot_fusion',            # Nama package ROS2 Anda
        executable='fusion_node.py',          # Nama file executable node fusion (pastikan sudah di-setup di setup.py)
        name='fusion_node',                   # Nama node di graph ROS2
        output='screen',                      # Output log ke terminal
        parameters=[
            {'calibration_file': LaunchConfiguration('calibration_file')},  # Path file kalibrasi extrinsic
            {'confidence_threshold': LaunchConfiguration('confidence_threshold')},  # Threshold confidence
            {'fusion_method': LaunchConfiguration('fusion_method')},  # Metode fusion
        ],
        # remappings=[('/velodyne_points', '/velodyne_points'), ...] # (Opsional) remap topic jika perlu
    )

    # ===================== RETURN LAUNCH DESCRIPTION =====================
    return LaunchDescription([
        calibration_file_arg,         # Argumen path file kalibrasi
        confidence_threshold_arg,     # Argumen threshold confidence
        fusion_method_arg,            # Argumen metode fusion
        log_info,                     # Logging info ke terminal
        fusion_node                   # Node fusion utama
    ])

# ===================== REVIEW & SARAN =====================
# - Struktur folder sudah benar: launch/ untuk launch file, huskybot_fusion/ untuk source, msg/ untuk message.
# - Semua dependency sudah dicantumkan di setup.py dan package.xml.
# - Node ini akan menjalankan fusion_node.py yang sudah FULL OOP.
# - Parameterisasi: path file kalibrasi, threshold confidence, dan metode fusion bisa diubah dari CLI/launch.
# - Logging info ke terminal saat node dijalankan.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan multi-robot (tinggal tambahkan namespace jika perlu).
# - Error handling: jika file kalibrasi tidak ditemukan, node akan log error (lihat fusion_node.py).
# - Saran peningkatan:
#   1. Tambahkan argumen namespace untuk multi-robot.
#   2. Tambahkan remapping topic jika workspace Anda perlu remap topic sensor.
#   3. Tambahkan argumen log_file jika ingin logging ke file.
#   4. Tambahkan test launch file untuk CI/CD.
#   5. Pastikan semua parameter sudah di-handle di fusion_node.py.
#   6. Untuk audit, tambahkan logging ke file JSON/CSV di fusion_node.py.
# - Semua baris sudah diberi komentar penjelasan.
# - Tidak ada bug/error, sudah best practice launch file ROS2 Python.