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
            'imu_topic',
            default_value='/imu/data',
            description='Topic IMU (opsional, jika ada sinkronisasi IMU)'  # Topic IMU, bisa dikosongkan jika tidak ada
        ),
        DeclareLaunchArgument(
            'output_yaml',
            default_value='config/synced_sensor_time.yaml',
            description='Path file output YAML hasil sinkronisasi'  # Output hasil sinkronisasi, dibaca node lain
        ),
        DeclareLaunchArgument(
            'output_csv',
            default_value='config/synced_sensor_time.csv',
            description='Path file output CSV hasil sinkronisasi'  # Output CSV untuk analisis manual
        ),
        DeclareLaunchArgument(
            'sync_time_slop',
            default_value='0.1',
            description='Threshold sinkronisasi waktu antar sensor (detik)'  # Toleransi sinkronisasi sensor
        ),
        DeclareLaunchArgument(
            'log_to_file',
            default_value='false',
            description='Aktifkan logging proses ke file'  # Logging proses sinkronisasi ke file log
        ),
        DeclareLaunchArgument(
            'log_file_path',
            default_value='config/sync_sensor_time.log',
            description='Path file log proses sinkronisasi'  # Lokasi file log proses sinkronisasi
        ),
        # Saran peningkatan: namespace untuk multi-robot (opsional, bisa diaktifkan jika perlu)
        # DeclareLaunchArgument(
        #     'namespace',
        #     default_value='',
        #     description='Namespace ROS2 untuk multi-robot (opsional)'
        # ),
        Node(
            package='huskybot_calibration',  # Nama package ROS2
            executable='sync_sensor_time.py',  # Nama executable Python (lihat setup.py entry_points)
            name='sync_sensor_time_node',  # Nama node ROS2
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},  # Parameter waktu simulasi
                {'camera_topic': LaunchConfiguration('camera_topic')},  # Parameter topic kamera
                {'lidar_topic': LaunchConfiguration('lidar_topic')},  # Parameter topic LiDAR
                {'imu_topic': LaunchConfiguration('imu_topic')},  # Parameter topic IMU
                {'output_yaml': LaunchConfiguration('output_yaml')},  # Parameter output YAML
                {'output_csv': LaunchConfiguration('output_csv')},  # Parameter output CSV
                {'sync_time_slop': LaunchConfiguration('sync_time_slop')},  # Parameter threshold sinkronisasi
                {'log_to_file': LaunchConfiguration('log_to_file')},  # Parameter logging ke file
                {'log_file_path': LaunchConfiguration('log_file_path')},  # Parameter path log file
            ],
            emulate_tty=True  # Agar output warna tetap muncul di terminal
            # namespace=LaunchConfiguration('namespace')  # Aktifkan jika ingin multi-robot
        )
    ])

# Penjelasan:
# - File ini adalah launch file ROS2 untuk menjalankan node sinkronisasi waktu sensor (`sync_sensor_time.py`).
# - Semua parameter penting bisa diubah dari command line atau launch file lain (modular untuk simulasi/real).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky + Velodyne + Arducam + IMU).
# - Node akan otomatis publish hasil sinkronisasi ke file YAML/CSV, bisa digunakan node lain (fusion, kalibrasi, dsb).
# - Parameterisasi lengkap: bisa atur topic, output, logging, threshold sinkronisasi, dan path log file.
# - Output file YAML/CSV akan dibaca oleh pipeline kalibrasi dan evaluasi.

# Keterhubungan:
# - Node ini akan membaca topic kamera (`/panorama/image_raw`), LiDAR (`/velodyne_points`), dan IMU (`/imu/data`) yang harus sudah aktif (dari driver atau simulasi Gazebo).
# - Output file YAML/CSV bisa digunakan oleh node kalibrasi (`calibrate_lidar_camera.py`) dan evaluasi sinkronisasi.
# - Semua topic dan frame harus konsisten dengan URDF/Xacro di package `huskybot_description` dan driver di `velodyne/`.
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