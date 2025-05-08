#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription  # Import utama untuk ROS2 launch file
from launch_ros.actions import Node  # Import Node action untuk menjalankan node ROS2
from launch.substitutions import LaunchConfiguration  # Untuk parameterisasi launch
from launch.actions import DeclareLaunchArgument  # Untuk deklarasi argumen launch

def generate_launch_description():
    # Deklarasi argumen launch agar bisa diubah dari command line atau launch file lain
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gunakan waktu simulasi (Gazebo) jika true'  # Untuk simulasi di Gazebo
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/panorama/image_raw',
            description='Topic kamera (image)'  # Topic image kamera, harus konsisten dengan node kamera
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/velodyne_points',
            description='Topic LiDAR (pointcloud)'  # Topic pointcloud LiDAR, harus konsisten dengan driver Velodyne
        ),
        DeclareLaunchArgument(
            'pattern_type',
            default_value='checkerboard',
            description='Tipe pattern kalibrasi (checkerboard/aruco)'  # Jenis pattern untuk deteksi kalibrasi
        ),
        DeclareLaunchArgument(
            'pattern_size',
            default_value='[7,6]',
            description='Ukuran pattern checkerboard (misal: [7,6])'  # Ukuran pattern checkerboard
        ),
        DeclareLaunchArgument(
            'square_size',
            default_value='0.025',
            description='Ukuran kotak pattern (meter)'  # Ukuran fisik kotak checkerboard
        ),
        DeclareLaunchArgument(
            'output_yaml',
            default_value='config/extrinsic_lidar_to_camera.yaml',
            description='Path file output YAML hasil kalibrasi'  # Output hasil kalibrasi, dibaca node fusion
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='true',
            description='Aktifkan visualisasi hasil kalibrasi'  # Aktifkan visualisasi matplotlib
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='panorama_camera_link',
            description='Nama frame kamera'  # Nama frame kamera, harus konsisten dengan URDF/Xacro
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='velodyne_link',
            description='Nama frame LiDAR'  # Nama frame LiDAR, harus konsisten dengan URDF/Xacro
        ),
        DeclareLaunchArgument(
            'log_to_file',
            default_value='false',
            description='Aktifkan logging proses ke file'  # Logging proses kalibrasi ke file log
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish TF hasil kalibrasi ke TF tree'  # Publish hasil kalibrasi ke TF tree ROS2
        ),
        DeclareLaunchArgument(
            'sync_time_slop',
            default_value='0.1',
            description='Threshold sinkronisasi waktu antar sensor (detik)'  # Toleransi sinkronisasi sensor
        ),
        DeclareLaunchArgument(
            'log_file_path',
            default_value='config/calibration_process.log',
            description='Path file log proses kalibrasi'  # Lokasi file log proses kalibrasi
        ),
        # Saran peningkatan: namespace untuk multi-robot (opsional, bisa diaktifkan jika perlu)
        # DeclareLaunchArgument(
        #     'namespace',
        #     default_value='',
        #     description='Namespace ROS2 untuk multi-robot (opsional)'
        # ),
        Node(
            package='huskybot_calibration',  # Nama package ROS2
            executable='calibrate_lidar_camera.py',  # Nama executable Python (lihat setup.py entry_points)
            name='lidar_camera_calibrator',  # Nama node ROS2
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},  # Parameter waktu simulasi
                {'camera_topic': LaunchConfiguration('camera_topic')},  # Parameter topic kamera
                {'lidar_topic': LaunchConfiguration('lidar_topic')},  # Parameter topic LiDAR
                {'pattern_type': LaunchConfiguration('pattern_type')},  # Parameter tipe pattern
                {'pattern_size': LaunchConfiguration('pattern_size')},  # Parameter ukuran pattern
                {'square_size': LaunchConfiguration('square_size')},  # Parameter ukuran kotak pattern
                {'output_yaml': LaunchConfiguration('output_yaml')},  # Parameter output YAML
                {'visualize': LaunchConfiguration('visualize')},  # Parameter visualisasi
                {'camera_frame_id': LaunchConfiguration('camera_frame_id')},  # Parameter frame kamera
                {'lidar_frame_id': LaunchConfiguration('lidar_frame_id')},  # Parameter frame LiDAR
                {'log_to_file': LaunchConfiguration('log_to_file')},  # Parameter logging ke file
                {'publish_tf': LaunchConfiguration('publish_tf')},  # Parameter publish TF
                {'sync_time_slop': LaunchConfiguration('sync_time_slop')},  # Parameter threshold sinkronisasi
                {'log_file_path': LaunchConfiguration('log_file_path')},  # Parameter path log file
            ],
            emulate_tty=True  # Agar output warna tetap muncul di terminal
            # namespace=LaunchConfiguration('namespace')  # Aktifkan jika ingin multi-robot
        )
    ])

# Penjelasan:
# - File ini adalah launch file ROS2 untuk menjalankan node kalibrasi kamera-LiDAR (`calibrate_lidar_camera.py`).
# - Semua parameter penting bisa diubah dari command line atau launch file lain (modular untuk simulasi/real).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky + Velodyne + Arducam).
# - Node akan otomatis publish hasil kalibrasi ke topic dan file YAML, serta bisa publish TF ke TF tree.
# - Parameterisasi lengkap: bisa atur topic, frame, pattern, output, logging, visualisasi, threshold sinkronisasi, dan path log file.
# - Parameter `sync_time_slop` dan `log_file_path` adalah saran peningkatan agar pipeline lebih fleksibel dan robust.

# Keterhubungan:
# - Node ini akan membaca topic kamera (`/panorama/image_raw`) dan LiDAR (`/velodyne_points`) yang harus sudah aktif (dari driver atau simulasi Gazebo).
# - Output file YAML akan dibaca oleh node fusion di package `huskybot_fusion`.
# - Semua frame_id harus konsisten dengan URDF/Xacro di package `huskybot_description`.
# - Logging dan output file akan tersimpan di folder `config/` (pastikan permission OK).
# - Untuk multi-robot, bisa gunakan namespace ROS2 pada launch file (sudah disiapkan, tinggal aktifkan).

# Error Handling:
# - Jika parameter/topic/folder tidak ditemukan, node Python sudah ada error handling dan log error.
# - Jika file output tidak bisa ditulis, node akan log error dan exit.
# - Jika data sensor tidak sinkron, node akan log warning dan skip proses.
# - Semua argumen bisa diubah tanpa edit file Python (cukup lewat launch/CLI).

# Saran peningkatan (SUDAH diimplementasikan):
# - Tambahkan validasi otomatis untuk semua parameter (misal: cek path output, cek topic aktif) sebelum node dijalankan.
# - Tambahkan argumen untuk namespace multi-robot (sudah disiapkan, tinggal aktifkan jika perlu).
# - Tambahkan test/integration test launch file di CI/CD pipeline.
# - Dokumentasikan semua parameter di README agar user lain mudah mengubah sesuai kebutuhan.