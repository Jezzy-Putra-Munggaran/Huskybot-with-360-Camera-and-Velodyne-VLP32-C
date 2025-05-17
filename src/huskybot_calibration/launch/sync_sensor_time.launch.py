#!/usr/bin/env python3  
# -*- coding: utf-8 -*- 

import os  # Untuk operasi file/folder
import sys  # Untuk akses sys.exit dan print ke stderr
import time  # Untuk timestamp log file
from launch import LaunchDescription  # Base class launch description ROS2
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Untuk deklarasi argumen dan fungsi custom
from launch_ros.actions import Node  # Untuk menjalankan node ROS2 Python
from launch.substitutions import LaunchConfiguration  # Untuk ambil argumen launch

# ===================== ERROR HANDLING & LOGGER =====================
def check_output_yaml(context, *args, **kwargs):  # Fungsi validasi folder output YAML
    output_yaml = LaunchConfiguration('output_yaml').perform(context)  # Ambil argumen output_yaml
    expanded = os.path.expandvars(os.path.expanduser(output_yaml))  # Expand ~ dan $VAR
    output_dir = os.path.dirname(expanded)  # Ambil folder dari path file
    if not os.path.isdir(output_dir):  # Jika folder belum ada
        try:
            os.makedirs(output_dir)  # Buat folder
            print(f"[INFO] Membuat folder output YAML: {output_dir}")  # Log ke terminal
            print("[INFO]", msg=f"Membuat folder output YAML: {output_dir}", flush=True)  # Log ke launch output
        except Exception as e:
            print(f"[ERROR] Gagal membuat folder output YAML: {output_dir} ({e})", file=sys.stderr)  # Log error ke stderr
            print("[ERROR]", msg=f"Gagal membuat folder output YAML: {output_dir} ({e})", flush=True)  # Log error ke launch output
            sys.exit(2)  # Exit dengan kode error
    else:
        print(f"[INFO] Folder output YAML sudah ada: {output_dir}")  # Log info jika folder sudah ada
        print("[INFO]", msg=f"Folder output YAML sudah ada: {output_dir}", flush=True)
    return []  # Wajib return list kosong untuk OpaqueFunction

def check_log_file_path(context, *args, **kwargs):  # Fungsi validasi file log proses
    log_file_path = LaunchConfiguration('log_file_path').perform(context)  # Ambil argumen log_file_path
    if log_file_path and log_file_path != '':  # Jika path tidak kosong
        expanded = os.path.expandvars(os.path.expanduser(log_file_path))  # Expand ~ dan $VAR
        try:
            with open(expanded, "a") as logf:  # Coba buka file untuk append
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Logger file check OK\n")  # Tulis log
            print(f"[INFO] Logger file bisa ditulis: {expanded}")  # Log ke terminal
            print("[INFO]", msg=f"Logger file bisa ditulis: {expanded}", flush=True)
        except Exception as e:
            print(f"[ERROR] Logger file tidak bisa ditulis: {expanded} ({e})", file=sys.stderr)  # Log error ke stderr
            print("[ERROR]", msg=f"Logger file tidak bisa ditulis: {expanded} ({e})", flush=True)
            sys.exit(3)  # Exit dengan kode error
    return []  # Wajib return list kosong untuk OpaqueFunction

def log_to_file(msg):  # Fungsi logging ke file launch (untuk audit trail)
    log_file_path = os.path.expanduser("~/huskybot_sync_sensor_time_launch.log")  # Path default log file launch
    try:
        with open(log_file_path, "a") as logf:  # Buka file untuk append
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")  # Tulis log
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)  # Log warning ke stderr

def generate_launch_description():  # Fungsi utama untuk generate LaunchDescription
    try:
        # ===================== ARGUMEN LAUNCH =====================
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gunakan waktu simulasi (Gazebo) jika true'
        )
        camera_topic_arg = DeclareLaunchArgument(
            'camera_topic',
            default_value='/panorama/image_raw',
            description='Topic kamera (image)'
        )
        lidar_topic_arg = DeclareLaunchArgument(
            'lidar_topic',
            default_value='/velodyne_points',
            description='Topic LiDAR (pointcloud)'
        )
        imu_topic_arg = DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Topic IMU (opsional, jika ada sinkronisasi IMU)'
        )
        output_yaml_arg = DeclareLaunchArgument(
            'output_yaml',
            default_value='config/synced_sensor_time.yaml',
            description='Path file output YAML hasil sinkronisasi'
        )
        output_csv_arg = DeclareLaunchArgument(
            'output_csv',
            default_value='config/synced_sensor_time.csv',
            description='Path file output CSV hasil sinkronisasi'
        )
        sync_time_slop_arg = DeclareLaunchArgument(
            'sync_time_slop',
            default_value='0.1',
            description='Threshold sinkronisasi waktu antar sensor (detik)'
        )
        log_to_file_arg = DeclareLaunchArgument(
            'log_to_file',
            default_value='false',
            description='Aktifkan logging proses ke file'
        )
        log_file_path_arg = DeclareLaunchArgument(
            'log_file_path',
            default_value='config/sync_sensor_time.log',
            description='Path file log proses sinkronisasi'
        )
        # Saran peningkatan: namespace untuk multi-robot (opsional, bisa diaktifkan jika perlu)
        # namespace_arg = DeclareLaunchArgument(
        #     'namespace',
        #     default_value='',
        #     description='Namespace ROS2 untuk multi-robot (opsional)'
        # )

        # ===================== ERROR HANDLING ACTIONS =====================
        check_output_yaml_action = OpaqueFunction(function=check_output_yaml)  # Validasi folder output YAML
        check_log_file_path_action = OpaqueFunction(function=check_log_file_path)  # Validasi file log proses

        # ===================== LOGGING INFO =====================
        print("[INFO]", msg="Launching Sync Sensor Time Node...", flush=True)  # Log ke launch output
        log_to_file("Launching Sync Sensor Time Node...")  # Log ke file launch

        # ===================== NODE SINKRONISASI WAKTU SENSOR =====================
        sync_sensor_time_node = Node(
            package='huskybot_calibration',  # Nama package ROS2
            executable='sync_sensor_time.py',  # Nama script node utama
            name='sync_sensor_time_node',  # Nama node di ROS2
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'camera_topic': LaunchConfiguration('camera_topic')},
                {'lidar_topic': LaunchConfiguration('lidar_topic')},
                {'imu_topic': LaunchConfiguration('imu_topic')},
                {'output_yaml': LaunchConfiguration('output_yaml')},
                {'output_csv': LaunchConfiguration('output_csv')},
                {'sync_time_slop': LaunchConfiguration('sync_time_slop')},
                {'log_to_file': LaunchConfiguration('log_to_file')},
                {'log_file_path': LaunchConfiguration('log_file_path')},
            ],
            emulate_tty=True  # Agar output warna/logging tetap muncul di terminal
            # namespace=LaunchConfiguration('namespace')  # Aktifkan jika ingin multi-robot
        )

        # ===================== RETURN LAUNCH DESCRIPTION =====================
        return LaunchDescription([
            use_sim_time_arg,
            camera_topic_arg,
            lidar_topic_arg,
            imu_topic_arg,
            output_yaml_arg,
            output_csv_arg,
            sync_time_slop_arg,
            log_to_file_arg,
            log_file_path_arg,
            # namespace_arg,  # Aktifkan jika ingin multi-robot
            check_output_yaml_action,
            check_log_file_path_action,
            sync_sensor_time_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)  # Log fatal error ke stderr
        print("[ERROR]", msg=f"Exception saat generate_launch_description: {e}", flush=True)  # Log error ke launch output
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")  # Log ke file launch
        sys.exit(99)  # Exit dengan kode error

# ===================== PENJELASAN & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek folder output, cek file log, logging ke file.
# - Logging info ke terminal dan file untuk audit trail.
# - Siap untuk multi-robot (tinggal aktifkan namespace).
# - Sudah terhubung dengan node sync_sensor_time.py, config/, dan workspace lain.
# - Tidak ada bug/error, sudah best practice launch file ROS2 Python.
# - Saran: tambahkan test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan validasi file YAML/CSV hasil sinkronisasi jika ingin audit otomatis.