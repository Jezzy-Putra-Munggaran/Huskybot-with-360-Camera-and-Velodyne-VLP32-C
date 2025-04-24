from launch import LaunchDescription  # Import utama untuk membuat LaunchDescription
from launch_ros.actions import Node   # Untuk mendefinisikan node ROS2 yang akan dijalankan

def generate_launch_description():
    """
    Launch file ini digunakan untuk menjalankan node fusion_node.py dari package yolobot_fusion.
    Node ini bertugas melakukan data fusion antara hasil deteksi kamera 360° (YOLO) dan point cloud LiDAR (Velodyne VLP-32C).
    Output node ini akan dipakai oleh sistem navigasi, mapping, dan obstacle avoidance di workspace Anda.
    """

    return LaunchDescription([
        Node(
            package='yolobot_fusion',            # Nama package ROS2 Anda
            executable='fusion_node.py',         # Nama file executable node fusion (pastikan sudah di-setup di setup.py)
            name='fusion_node',                  # Nama node di graph ROS2
            output='screen',                     # Output log ke terminal
            parameters=[],                       # (Opsional) Tambahkan parameter jika diperlukan
            # remappings=[('/velodyne_points', '/velodyne_points'), ...] # (Opsional) remap topic jika perlu
        )
    ])