from launch import LaunchDescription  # Import class utama untuk mendefinisikan launch file ROS2
from launch.actions import DeclareLaunchArgument  # Import untuk mendeklarasikan argumen launch (opsional)
from launch_ros.actions import Node  # Import untuk menjalankan node ROS2 dari package Python/C++

def generate_launch_description():  # Fungsi utama yang akan dipanggil oleh ROS2 saat launch file dijalankan

    return LaunchDescription([  # Kembalikan objek LaunchDescription berisi daftar aksi yang akan dijalankan
        DeclareLaunchArgument(  # Mendeklarasikan argumen launch bernama 'use_sim_time'
            'use_sim_time',  # Nama argumen (bisa di-set saat launch)
            default_value='true',  # Nilai default: true (umumnya untuk simulasi/Gazebo)
            description='Use simulation (Gazebo) clock if true'),  # Penjelasan argumen
        Node(  # Menjalankan satu node ROS2
            package='huskybot_recognition',  # Nama package ROS2 tempat node berada (harus sesuai package.xml dan setup.py)
            executable='yolov12_ros2_pt.py',  # Nama executable/script Python yang akan dijalankan (harus terdaftar di setup.py entry_points)
            output='screen'),  # Output log node akan ditampilkan di terminal (bukan ke file)
    ])