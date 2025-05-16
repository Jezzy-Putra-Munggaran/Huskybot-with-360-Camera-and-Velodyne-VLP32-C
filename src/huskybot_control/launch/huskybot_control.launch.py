#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, LogWarn, LogError, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

# ===================== ERROR HANDLING & LOGGER =====================
def check_param_yaml(context, *args, **kwargs):
    param_yaml = LaunchConfiguration('param_yaml').perform(context)
    if param_yaml and param_yaml != '':
        expanded = os.path.expandvars(os.path.expanduser(param_yaml))
        if not os.path.isfile(expanded):
            print(f"[ERROR] File parameter YAML tidak ditemukan: {expanded}", file=sys.stderr)
            LogError(msg=f"File parameter YAML tidak ditemukan: {expanded}")
            sys.exit(2)
        else:
            print(f"[INFO] File parameter YAML ditemukan: {expanded}")
            LogInfo(msg=f"File parameter YAML ditemukan: {expanded}")
    return []

def check_logger_file(context, *args, **kwargs):
    log_file = LaunchConfiguration('log_file').perform(context)
    if log_file and log_file != '':
        expanded = os.path.expandvars(os.path.expanduser(log_file))
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
    log_file_path = os.path.expanduser("~/huskybot_control_launch.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

def generate_launch_description():
    try:
        # Argumen tambahan untuk parameter kontrol dan logger
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
        max_speed_arg = DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Kecepatan maksimum robot (m/s)')
        safety_stop_arg = DeclareLaunchArgument(
            'safety_stop',
            default_value='true',
            description='Aktifkan fitur emergency stop')
        param_yaml_arg = DeclareLaunchArgument(
            'param_yaml',
            default_value='',
            description='Path ke file parameter YAML (opsional)')
        cmd_vel_topic_arg = DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/huskybot/cmd_vel',
            description='Topic cmd_vel yang akan dipublish')
        enable_safety_monitor_arg = DeclareLaunchArgument(
            'enable_safety_monitor',
            default_value='true',
            description='Aktifkan node safety_monitor.py')
        scan_topic_arg = DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Topic LaserScan untuk safety monitor')
        log_file_arg = DeclareLaunchArgument(
            'log_file',
            default_value='',
            description='Path file log untuk logger (kosong = tidak log ke file)')
        log_csv_arg = DeclareLaunchArgument(
            'log_csv',
            default_value='false',
            description='Log ke format CSV (true/false)')
        log_level_arg = DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Level log (debug/info/warn)')
        max_log_size_arg = DeclareLaunchArgument(
            'max_log_size',
            default_value=str(5*1024*1024),
            description='Ukuran maksimum file log dalam byte (default 5MB)')

        # Error handling actions
        check_param_yaml_action = OpaqueFunction(function=check_param_yaml)
        check_logger_file_action = OpaqueFunction(function=check_logger_file)

        # Logging info ke terminal dan file
        log_launch = LogInfo(msg="Launching Huskybot Control Node...")
        log_to_file("Launching Huskybot Control Node...")

        # Node utama kontrol robot
        robot_control_node = Node(
            package='huskybot_control',
            executable='robot_control.py',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'max_speed': LaunchConfiguration('max_speed')},
                {'safety_stop': LaunchConfiguration('safety_stop')}
            ] + ([LaunchConfiguration('param_yaml')] if LaunchConfiguration('param_yaml') != '' else []),
            remappings=[
                ('/huskybot/cmd_vel', LaunchConfiguration('cmd_vel_topic'))
            ]
        )

        # Node safety monitor (opsional)
        safety_monitor_node = Node(
            package='huskybot_control',
            executable='safety_monitor.py',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/scan', LaunchConfiguration('scan_topic'))
            ],
            condition=IfCondition(LaunchConfiguration('enable_safety_monitor'))
        )

        # Node logger (opsional, aktifkan jika ingin logging ke file/csv)
        logger_node = Node(
            package='huskybot_control',
            executable='logger.py',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'log_file': LaunchConfiguration('log_file')},
                {'log_csv': LaunchConfiguration('log_csv')},
                {'log_level': LaunchConfiguration('log_level')},
                {'max_log_size': LaunchConfiguration('max_log_size')}
            ],
            condition=IfCondition(LaunchConfiguration('log_file'))
        )

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
            log_launch,
            robot_control_node,
            LogInfo(msg="Launching Safety Monitor Node..."),
            safety_monitor_node,
            LogInfo(msg="Launching Logger Node..."),
            logger_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

# ===================== PENJELASAN & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch/CLI.
# - Error handling sudah lengkap: cek file YAML, cek file log, logging ke file.
# - Logging info ke terminal dan file untuk audit trail.
# - Siap untuk multi-robot (tinggal remap topic jika perlu).
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan validasi file YAML parameter jika ingin logging statistik per sensor.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.