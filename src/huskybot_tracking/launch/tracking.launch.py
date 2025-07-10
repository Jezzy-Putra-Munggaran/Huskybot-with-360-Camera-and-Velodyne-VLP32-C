from launch import LaunchDescription  # Import LaunchDescription, inti dari launch file ROS2 Python
from launch_ros.actions import Node  # Import Node untuk menjalankan node ROS2 Python

def generate_launch_description():  # Fungsi utama generate LaunchDescription
    # ===================== NODE MULTICAM YOLOv12 TRACKING =====================
    # Node ini menjalankan multicam_tracking_node.py dari package huskybot_tracking
    # Semua parameter bisa diubah dari launch file (best practice ROS2)
    # Output ke screen agar mudah debug
    # Parameter default sudah sesuai dengan node Python (cam_count, model_path, camera_topics)
    # Bisa di-extend untuk multi-robot dengan namespace jika perlu
    multicam_tracking_node = Node(
        package='huskybot_tracking',  # Nama package ROS2 (harus sama dengan folder dan setup.py)
        executable='multicam_tracking_node',  # Nama executable (harus didaftarkan di setup.py entry_points)
        name='multicam_tracking',  # Nama node unik di ROS2 graph
        output='screen',  # Output log ke terminal
        parameters=[
            {'cam_count': 6},  # Jumlah kamera (default 6, hexagonal)
            {'model_path': 'yolo12x.engine'},  # Path model YOLOv12 tracking (bisa diubah ke .engine/.onnx/.engine)
            {'camera_topics': [
                '/camera_front/image_raw',
                '/camera_right/image_raw',
                '/camera_rear_right/image_raw',
                '/camera_rear/image_raw',
                '/camera_left/image_raw',
                '/camera_front_left/image_raw'
            ]},  # Daftar topic kamera (default urutan hexagonal)
        ]
        # Saran: tambahkan remappings jika ingin remap topic kamera/output
        # Saran: tambahkan namespace jika ingin multi-robot
    )

    # ===================== NODE FUSION TRACKING =====================
    # Node ini menjalankan tracking_fusion_node.py dari package huskybot_tracking
    # Menggabungkan hasil multicam YOLOv12 (detection, segmentation, obb) ke topic /tracking
    tracking_fusion_node = Node(
        package='huskybot_tracking',  # Nama package ROS2
        executable='tracking_fusion_node',  # Nama executable (harus didaftarkan di setup.py entry_points)
        name='tracking_fusion',  # Nama node unik
        output='screen',  # Output log ke terminal
        # Tidak perlu parameter khusus, sudah otomatis subscribe ke /detection, /segmentation, /obb
    )

    return LaunchDescription([
        multicam_tracking_node,  # Tambahkan node multicam tracking ke launch
        tracking_fusion_node,    # Tambahkan node fusion tracking ke launch
    ])

# ===================== PENJELASAN & SARAN PENINGKATAN =====================
# - Launch file ini sudah best practice ROS2 Humble: modular, parameterisasi, siap multi-robot.
# - Sudah terhubung otomatis ke node multicam_tracking_node.py dan tracking_fusion_node.py (publish ke /detection dan /tracking).
# - Semua parameter bisa diubah dari CLI/launch file lain (tinggal override di launch gabungan).
# - Jika ingin robust multi-robot, tambahkan argumen namespace dan remap topic di Node().
# - Jika ingin audit trail, tambahkan logger