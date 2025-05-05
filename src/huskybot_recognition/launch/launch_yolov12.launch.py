from launch import LaunchDescription  # Import class utama untuk mendefinisikan launch file ROS2
from launch.actions import DeclareLaunchArgument, LogInfo  # Import untuk deklarasi argumen dan logging info ke terminal
from launch.substitutions import LaunchConfiguration  # Import untuk mengambil nilai argumen launch
from launch_ros.actions import Node  # Import untuk menjalankan node ROS2 dari package Python/C++

def generate_launch_description():  # Fungsi utama yang akan dipanggil oleh ROS2 saat launch file dijalankan
    return LaunchDescription([  # Kembalikan objek LaunchDescription berisi daftar aksi yang akan dijalankan
        DeclareLaunchArgument(  # Mendeklarasikan argumen launch bernama 'use_sim_time'
            'use_sim_time',  # Nama argumen (bisa di-set saat launch)
            default_value='true',  # Nilai default: true (umumnya untuk simulasi/Gazebo)
            description='Use simulation (Gazebo) clock if true'),  # Penjelasan argumen

        DeclareLaunchArgument(  # Tambahkan argumen untuk path model YOLOv12
            'model_path',
            default_value='~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt',
            description='Path ke file model YOLOv12 (.pt)'
        ),

        DeclareLaunchArgument(  # Tambahkan argumen untuk threshold confidence
            'confidence_threshold',
            default_value='0.25',
            description='Threshold confidence deteksi YOLOv12'
        ),

        DeclareLaunchArgument(  # Tambahkan argumen untuk logging statistik deteksi
            'log_stats',
            default_value='true',
            description='Aktifkan logging statistik deteksi ke file'
        ),

        DeclareLaunchArgument(  # Tambahkan argumen untuk path file statistik deteksi
            'log_stats_path',
            default_value='~/huskybot_detection_log/yolov12_stats.csv',
            description='Path file statistik deteksi YOLOv12'
        ),

        LogInfo(msg='Launching YOLOv12 ROS2 Node...'),  # Logging info ke terminal saat node dijalankan

        Node(  # Menjalankan satu node ROS2
            package='huskybot_recognition',  # Nama package ROS2 tempat node berada (harus sesuai package.xml dan setup.py)
            executable='yolov12_ros2_pt.py',  # Nama executable/script Python yang akan dijalankan (harus terdaftar di setup.py entry_points)
            output='screen',  # Output log node akan ditampilkan di terminal (bukan ke file)
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},  # Parameter agar node pakai waktu simulasi jika true
                {'model_path': LaunchConfiguration('model_path')},  # Parameter path model YOLOv12
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')},  # Parameter threshold confidence
                {'log_stats': LaunchConfiguration('log_stats')},  # Parameter logging statistik
                {'log_stats_path': LaunchConfiguration('log_stats_path')},  # Parameter path file statistik
            ],
            # Bisa tambahkan remappings jika ingin ganti topic
        ),
    ])