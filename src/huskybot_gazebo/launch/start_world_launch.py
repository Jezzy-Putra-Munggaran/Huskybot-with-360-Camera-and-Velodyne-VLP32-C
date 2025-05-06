#!/usr/bin/python3  # Shebang agar bisa dieksekusi langsung
# -*- coding: utf-8 -*-  # Encoding file Python

import os                                            # Untuk operasi file/path
import sys                                           # Untuk error output

from ament_index_python.packages import get_package_share_directory  # Cari path share ROS2 package
from launch import LaunchDescription                  # Kelas utama untuk launch file
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # Untuk argumen & include launch
from launch.launch_description_sources import PythonLaunchDescriptionSource # Untuk include launch file Python
from launch.substitutions import LaunchConfiguration  # Untuk ambil nilai argumen launch

def generate_launch_description():                    # Fungsi utama ROS2 untuk launch file

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')           # Path package gazebo_ros (core ROS2-Gazebo)
    pkg_huskybot_gazebo = get_package_share_directory('huskybot_gazebo') # Path package huskybot_gazebo (workspace-mu)

    # ---------- Argumen Modular ----------
    world_default = os.path.join(pkg_huskybot_gazebo, 'worlds', 'yolo_test.world')   # Default world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_default,
        description='Path ke file world SDF Gazebo (misal: yolo_test.world)'         # Penjelasan user-friendly
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Set true untuk output verbose Gazebo (debugging)'               # Argumen verbose untuk debugging
    )
    pause_arg = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Set true untuk memulai Gazebo dalam keadaan pause'              # Argumen pause untuk debugging
    )

    # ---------- Validasi File World ----------
    world_path = LaunchConfiguration('world')                                         # Ambil path world dari argumen
    # Note: LaunchConfiguration belum bisa di-evaluate di Python, jadi validasi manual hanya untuk default
    if not os.path.exists(world_default):                                             # Cek file default world ada
        print(f"[ERROR] World file tidak ditemukan: {world_default}", file=sys.stderr)
        sys.exit(1)

    # ---------- Include Launch Gazebo ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),              # Path ke launch file utama Gazebo
        ),
        launch_arguments={
            'world': world_path,                                                     # Forward argumen world ke gazebo.launch.py
            'verbose': LaunchConfiguration('verbose'),                               # Forward argumen verbose
            'pause': LaunchConfiguration('pause'),                                   # Forward argumen pause
        }.items()
    )

    return LaunchDescription([
        world_arg,           # Argumen world file
        verbose_arg,         # Argumen verbose
        pause_arg,           # Argumen pause
        gazebo               # Jalankan Gazebo dengan world dan argumen yang dipilih
    ])

# ===================== REVIEW & SARAN =====================
# - Struktur folder sudah benar: launch/ untuk launch file, worlds/ untuk world file.
# - File ini hanya bertugas menjalankan Gazebo dengan world file yang dipilih user.
# - Semua dependency (gazebo_ros, huskybot_gazebo) sudah dicari otomatis.
# - Argumen world, verbose, dan pause sudah modular dan bisa diubah dari CLI/launch.
# - Error handling: validasi file world default sudah ada, tapi validasi file world custom (dari argumen) belum bisa dilakukan di level Python (karena LaunchConfiguration dievaluasi saat runtime launch, bukan saat parsing Python).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan multi-robot (tinggal tambahkan namespace jika perlu).
# - Tidak perlu OOP di launch file, sudah best practice ROS2 launch Python.
# - Saran peningkatan:
#   1. Tambahkan validasi file world custom dengan OpaqueFunction jika ingin error handling lebih advance.
#   2. Tambahkan argumen namespace jika ingin multi-robot.
#   3. Tambahkan logging info ke terminal untuk audit trail.
#   4. Tambahkan test launch file untuk CI/CD.
#   5. Tambahkan argumen use_sim_time jika ingin sinkronisasi waktu simulasi.
#   6. Dokumentasikan semua argumen di README.md.
# - Semua baris sudah diberi komentar penjelasan.
# - Tidak ada bug/error, sudah best practice launch file ROS2 Python.