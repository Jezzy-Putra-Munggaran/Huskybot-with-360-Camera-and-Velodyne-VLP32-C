#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import os
import sys
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, LogWarn, LogError, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# ===================== ERROR HANDLING & LOGGER =====================
def check_output_yaml(context, *args, **kwargs):
    output_yaml = LaunchConfiguration('output_yaml').perform(context)
    expanded = os.path.expandvars(os.path.expanduser(output_yaml))
    output_dir = os.path.dirname(expanded)
    if not os.path.isdir(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"[INFO] Membuat folder output YAML: {output_dir}")
            LogInfo(msg=f"Membuat folder output YAML: {output_dir}")
        except Exception as e:
            print(f"[ERROR] Gagal membuat folder output YAML: {output_dir} ({e})", file=sys.stderr)
            LogError(msg=f"Gagal membuat folder output YAML: {output_dir} ({e})")
            sys.exit(2)
    else:
        print(f"[INFO] Folder output YAML sudah ada: {output_dir}")
        LogInfo(msg=f"Folder output YAML sudah ada: {output_dir}")
    return []

def check_log_file_path(context, *args, **kwargs):
    log_file_path = LaunchConfiguration('log_file_path').perform(context)
    if log_file_path and log_file_path != '':
        expanded = os.path.expandvars(os.path.expanduser(log_file_path))
        try:
            with open(expanded, "a") as logf:
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Logger file check OK\n")
            print(f"[INFO] Logger file bisa ditulis: {expanded}")
            LogInfo(msg=f"Logger file bisa ditulis: {expanded}")
        except Exception as e:
            print(f"[ERROR] Logger file tidak bisa ditulis: {expanded} ({e})", file=sys.stderr)
            LogError(msg=f"Logger file tidak bisa ditulis: {expanded} ({e})")
            sys.exit(3)
    return []

def log_to_file(msg):
    log_file_path = os.path.expanduser("~/huskybot_sync_sensor_time_launch.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

def generate_launch_description():
    try:
        # Deklarasi argumen launch agar bisa diubah dari command line atau launch file lain
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

        # Error handling actions
        check_output_yaml_action = OpaqueFunction(function=check_output_yaml)
        check_log_file_path_action = OpaqueFunction(function=check_log_file_path)

        # Logging info ke terminal dan file
        log_launch = LogInfo(msg="Launching Sync Sensor Time Node...")
        log_to_file("Launching Sync Sensor Time Node...")

        sync_sensor_time_node = Node(
            package='huskybot_calibration',
            executable='sync_sensor_time.py',
            name='sync_sensor_time_node',
            output='screen',
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
            emulate_tty=True
            # namespace=LaunchConfiguration('namespace')  # Aktifkan jika ingin multi-robot
        )

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
            log_launch,
            sync_sensor_time_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

# Penjelasan:
# - File ini sudah ada logger, error handling, dan validasi path output/log file.
# - Semua parameter penting bisa diubah dari command line atau launch file lain (modular untuk simulasi/real).
# - Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real.
# - Saran: aktifkan namespace jika ingin multi-robot.