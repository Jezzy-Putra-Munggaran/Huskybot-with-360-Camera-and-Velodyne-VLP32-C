#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription                                    # Import utama untuk membuat LaunchDescription (wajib di launch file ROS2)
from launch.actions import DeclareLaunchArgument, LogInfo               # Untuk deklarasi argumen launch dan logging info ke terminal
from launch.substitutions import LaunchConfiguration                    # Untuk mengambil nilai argumen launch
from launch_ros.actions import Node                                     # Untuk menjalankan node ROS2 dari package Python/C++
from launch.conditions import IfCondition                               # Untuk enable/disable node dengan argumen

def generate_launch_description():                                      # Fungsi utama ROS2 untuk launch file

    # Argumen tambahan untuk parameter kontrol dan logger
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),      # Argumen agar node bisa pakai clock simulasi jika true
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Kecepatan maksimum robot (m/s)'),             # Argumen untuk max_speed (bisa diakses di node Python)
        DeclareLaunchArgument(
            'safety_stop',
            default_value='true',
            description='Aktifkan fitur emergency stop'),              # Argumen untuk safety_stop (bisa diakses di node Python)
        DeclareLaunchArgument(
            'param_yaml',
            default_value='',
            description='Path ke file parameter YAML (opsional)'),     # Argumen opsional untuk file YAML parameter
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/huskybot/cmd_vel',
            description='Topic cmd_vel yang akan dipublish'),          # Argumen opsional untuk remap topic cmd_vel
        DeclareLaunchArgument(
            'enable_safety_monitor',
            default_value='true',
            description='Aktifkan node safety_monitor.py'),            # Argumen enable/disable node safety monitor
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Topic LaserScan untuk safety monitor'),        # Argumen remap topic sensor (bisa diubah sesuai pipeline)
        # Argumen tambahan untuk logger
        DeclareLaunchArgument(
            'log_file',
            default_value='',
            description='Path file log untuk logger (kosong = tidak log ke file)'),  # Argumen path file log (opsional)
        DeclareLaunchArgument(
            'log_csv',
            default_value='false',
            description='Log ke format CSV (true/false)'),             # Argumen log ke CSV (opsional)
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Level log (debug/info/warn)'),                # Argumen level log (opsional)
        DeclareLaunchArgument(
            'max_log_size',
            default_value=str(5*1024*1024),
            description='Ukuran maksimum file log dalam byte (default 5MB)'),  # Argumen ukuran maksimum file log (opsional)
        LogInfo(msg="Launching Huskybot Control Node..."),             # Logging info saat launch mulai
        Node(
            package='huskybot_control',                                # Nama package yang berisi node kontrol
            executable='robot_control.py',                             # Nama script Python yang dijalankan (harus ada di install/lib/huskybot_control/)
            output='screen',                                           # Output log node ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}, # Parameter use_sim_time diteruskan ke node
                {'max_speed': LaunchConfiguration('max_speed')},       # Parameter max_speed diteruskan ke node
                {'safety_stop': LaunchConfiguration('safety_stop')}    # Parameter safety_stop diteruskan ke node
            ] + ([LaunchConfiguration('param_yaml')] if LaunchConfiguration('param_yaml') != '' else []),  # Tambahkan file YAML jika ada
            remappings=[
                ('/huskybot/cmd_vel', LaunchConfiguration('cmd_vel_topic'))  # Remap topic cmd_vel jika diperlukan
            ]
        ),
        LogInfo(msg="Launching Safety Monitor Node..."),               # Logging info untuk node safety monitor
        Node(
            package='huskybot_control',                                # Contoh multi-node: safety monitor (jika ada)
            executable='safety_monitor.py',                            # Nama script Python safety monitor (pastikan ada di scripts/)
            output='screen',                                           # Output log node ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}  # Parameter use_sim_time diteruskan ke node safety monitor
            ],
            remappings=[
                ('/scan', LaunchConfiguration('scan_topic'))           # Remap topic scan jika diperlukan
            ],
            condition=IfCondition(LaunchConfiguration('enable_safety_monitor'))  # Enable/disable node dengan argumen
        ),
        LogInfo(msg="Launching Logger Node..."),                       # Logging info untuk node logger
        Node(
            package='huskybot_control',                                # Nama package logger
            executable='logger.py',                                    # Nama script logger (pastikan ada di scripts/)
            output='screen',                                           # Output log node ke terminal
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}, # Parameter use_sim_time diteruskan ke logger
                {'log_file': LaunchConfiguration('log_file')},         # Path file log diteruskan ke logger
                {'log_csv': LaunchConfiguration('log_csv')},           # Opsi log ke CSV diteruskan ke logger
                {'log_level': LaunchConfiguration('log_level')},       # Level log diteruskan ke logger
                {'max_log_size': LaunchConfiguration('max_log_size')}  # Ukuran maksimum file log diteruskan ke logger
            ]
        ),
    ])