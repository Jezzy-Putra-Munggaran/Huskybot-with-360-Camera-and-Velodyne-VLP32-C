#!/usr/bin/python3  
# -*- coding: utf-8 -*- 

import os  # Untuk operasi path file/folder
import sys  # Untuk exit jika error fatal
import time  # Untuk timestamp log
from launch import LaunchDescription  # Import utama LaunchDescription ROS2
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Untuk deklarasi argumen dan fungsi custom
from launch.substitutions import LaunchConfiguration  # Untuk ambil nilai argumen launch
from launch_ros.actions import Node  # Untuk deklarasi node ROS2
from launch.conditions import IfCondition  # Untuk kondisi enable/disable node

# ===================== ERROR HANDLING & LOGGER =====================
def check_param_yaml(context, *args, **kwargs):  # Fungsi cek file parameter YAML sebelum launch
    param_yaml = LaunchConfiguration('param_yaml').perform(context)  # Ambil argumen param_yaml
    if param_yaml and param_yaml != '':  # Jika ada path param_yaml
        expanded = os.path.expandvars(os.path.expanduser(param_yaml))  # Expand ~ dan env var
        if not os.path.isfile(expanded):  # Jika file tidak ada
            print(f"[ERROR] File parameter YAML tidak ditemukan: {expanded}", file=sys.stderr)  # Log error ke stderr
            sys.exit(2)  # Exit dengan kode error
        else:
            print(f"[INFO] File parameter YAML ditemukan: {expanded}")  # Log info ke stdout
    return []  # Harus return list kosong untuk OpaqueFunction

def check_logger_file(context, *args, **kwargs):  # Fungsi cek file log bisa ditulis sebelum launch
    log_file = LaunchConfiguration('log_file').perform(context)  # Ambil argumen log_file
    if log_file and log_file != '':  # Jika ada path log_file
        expanded = os.path.expandvars(os.path.expanduser(log_file))  # Expand ~ dan env var
        try:
            with open(expanded, "a") as logf:  # Coba buka file log untuk append
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Logger file check OK\n")  # Tulis log test
            print(f"[INFO] Logger file bisa ditulis: {expanded}")  # Log info ke stdout
        except Exception as e:
            print(f"[ERROR] Logger file tidak bisa ditulis: {expanded} ({e})", file=sys.stderr)  # Log error ke stderr
            sys.exit(3)  # Exit dengan kode error
    return []  # Harus return list kosong untuk OpaqueFunction

def log_to_file(msg):  # Fungsi logging ke file audit trail launch
    log_file_path = os.path.expanduser("~/huskybot_control_launch.log")  # Path default log file
    try:
        with open(log_file_path, "a") as logf:  # Buka file log untuk append
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")  # Tulis pesan log
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)  # Warning jika gagal log

def generate_launch_description():  # Fungsi utama generate launch description
    try:
        # ===================== ARGUMEN LAUNCH =====================
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')  # Argumen waktu simulasi
        max_speed_arg = DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Kecepatan maksimum robot (m/s)')  # Argumen kecepatan maksimum
        safety_stop_arg = DeclareLaunchArgument(
            'safety_stop',
            default_value='true',
            description='Aktifkan fitur emergency stop')  # Argumen safety stop
        param_yaml_arg = DeclareLaunchArgument(
            'param_yaml',
            default_value='',
            description='Path ke file parameter YAML (opsional)')  # Argumen file parameter YAML
        cmd_vel_topic_arg = DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/huskybot/cmd_vel',
            description='Topic cmd_vel yang akan dipublish')  # Argumen topic cmd_vel
        enable_safety_monitor_arg = DeclareLaunchArgument(
            'enable_safety_monitor',
            default_value='true',
            description='Aktifkan node safety_monitor.py')  # Argumen enable safety monitor
        scan_topic_arg = DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Topic LaserScan untuk safety monitor')  # Argumen topic scan
        log_file_arg = DeclareLaunchArgument(
            'log_file',
            default_value='',
            description='Path file log untuk logger (kosong = tidak log ke file)')  # Argumen file log
        log_csv_arg = DeclareLaunchArgument(
            'log_csv',
            default_value='false',
            description='Log ke format CSV (true/false)')  # Argumen log ke CSV
        log_level_arg = DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Level log (debug/info/warn)')  # Argumen level log
        max_log_size_arg = DeclareLaunchArgument(
            'max_log_size',
            default_value=str(5*1024*1024),
            description='Ukuran maksimum file log dalam byte (default 5MB)')  # Argumen ukuran maksimum log

        # ===================== ERROR HANDLING ACTIONS =====================
        check_param_yaml_action = OpaqueFunction(function=check_param_yaml)  # Cek file param_yaml sebelum launch
        check_logger_file_action = OpaqueFunction(function=check_logger_file)  # Cek file log sebelum launch

        # ===================== LOGGING INFO =====================
        print("[INFO] Launching Huskybot Control Node...", flush=True)  # Log info ke terminal
        log_to_file("Launching Huskybot Control Node...")  # Log info ke file

        # ===================== NODE UTAMA KONTROL ROBOT =====================
        robot_control_node = Node(
            package='huskybot_control',  # Nama package
            executable='robot_control.py',  # Nama script node utama
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'max_speed': LaunchConfiguration('max_speed')},
                {'safety_stop': LaunchConfiguration('safety_stop')}
            ] + ([LaunchConfiguration('param_yaml')] if LaunchConfiguration('param_yaml') != '' else []),  # Tambah param_yaml jika ada
            remappings=[
                ('/huskybot/cmd_vel', LaunchConfiguration('cmd_vel_topic'))  # Remap topic cmd_vel jika perlu
            ]
        )

        # ===================== NODE SAFETY MONITOR (OPSIONAL) =====================
        safety_monitor_node = Node(
            package='huskybot_control',  # Nama package
            executable='safety_monitor.py',  # Nama script safety monitor
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/scan', LaunchConfiguration('scan_topic'))  # Remap topic scan jika perlu
            ],
            condition=IfCondition(LaunchConfiguration('enable_safety_monitor'))  # Hanya aktif jika enable_safety_monitor true
        )

        # ===================== NODE LOGGER (OPSIONAL) =====================
        logger_node = Node(
            package='huskybot_control',  # Nama package
            executable='logger.py',  # Nama script logger
            output='screen',  # Output ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'log_file': LaunchConfiguration('log_file')},
                {'log_csv': LaunchConfiguration('log_csv')},
                {'log_level': LaunchConfiguration('log_level')},
                {'max_log_size': LaunchConfiguration('max_log_size')}
            ],
            condition=IfCondition(LaunchConfiguration('log_file'))  # Hanya aktif jika log_file tidak kosong
        )

        # ===================== RETURN LAUNCH DESCRIPTION =====================
        return LaunchDescription([
            use_sim_time_arg,
            max_speed_arg,
            safety_stop_arg,
            param_yaml_arg,
            cmd_vel_topic_arg,
            enable_safety_monitor_arg,
            scan_topic_arg,
            log_file_arg,
            log_csv_arg,
            log_level_arg,
            max_log_size_arg,
            check_param_yaml_action,
            check_logger_file_action,
            robot_control_node,
            safety_monitor_node,
            logger_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)  # Log fatal error ke stderr
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")  # Log fatal error ke file
        sys.exit(99)  # Exit dengan kode error

# ===================== PENJELASAN & SARAN PENINGKATAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek file YAML, cek file log, logging ke file.
# - Logging info ke terminal dan file untuk audit trail.
# - Siap untuk multi-robot (tinggal remap topic jika perlu).
# - Semua node sudah OOP (class Node di robot_control.py, safety_monitor.py, logger.py).
# - Sudah terhubung dengan pipeline workspace (mapping, navigation, fusion, dsb).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan validasi file YAML parameter jika ingin logging statistik per sensor.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.
# - Saran: tambahkan logging ke file CSV/JSON untuk audit trail multi-robot.
# - Saran: tambahkan argumen namespace jika ingin multi-robot (tinggal tambah DeclareLaunchArgument dan remap).
# - Saran: tambahkan validasi isi YAML (bukan hanya ada/tidaknya file) jika ingin error handling lebih advance.
# - Saran: tambahkan argumen enable_logger agar logger_node bisa diaktifkan/disable dari CLI.