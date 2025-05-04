#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os  # Import modul os untuk operasi path file
from ament_index_python.packages import get_package_share_directory  # Untuk mencari path share ROS2 package
from launch import LaunchDescription  # Kelas utama untuk launch file ROS2
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo  # Untuk deklarasi argumen, eksekusi proses, dan logging info ke terminal
from launch.substitutions import LaunchConfiguration, Command  # Untuk ambil nilai argumen dan jalankan perintah shell
from launch_ros.actions import Node  # Untuk menjalankan node ROS2
from launch_ros.parameter_descriptions import ParameterValue  # Untuk parameter node yang bisa dieksekusi (misal hasil xacro)

def generate_launch_description():  # Fungsi utama ROS2 untuk launch file

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Ambil argumen use_sim_time (default true)
    robot_description_topic = LaunchConfiguration('robot_description_topic', default='robot_description')  # Argumen untuk remap topic robot_description

    urdf_file = os.path.join(  # Path ke file xacro robot
        get_package_share_directory('huskybot_description'),
        'robot',
        'husky_with_cameras.xacro'
    )

    # ---------- Error Handling: cek file Xacro ada ----------
    if not os.path.exists(urdf_file):  # Jika file Xacro tidak ada
        print(f"[ERROR] File Xacro robot tidak ditemukan: {urdf_file}", flush=True)  # Print error ke terminal
        exit(1)  # Exit agar launch tidak lanjut

    robot_description = ParameterValue(  # Jalankan xacro untuk menghasilkan URDF string
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ---------- Logging info ----------
    log_spawn = LogInfo(msg=f"Spawning robot model: {urdf_file}")  # Logging info model yang di-spawn
    log_topic = LogInfo(msg=["robot_description topic: ", robot_description_topic])  # Logging info topic robot_description

    return LaunchDescription([
        DeclareLaunchArgument(  # Deklarasi argumen use_sim_time (bisa di-set saat launch)
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(  # Argumen untuk remap topic robot_description (opsional)
            'robot_description_topic',
            default_value='robot_description',
            description='Topic tempat robot_description di-publish (default: robot_description)'
        ),
        log_spawn,  # Logging info model yang di-spawn
        log_topic,  # Logging info topic robot_description
        Node(  # Jalankan robot_state_publisher untuk publish TF dan robot_description
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            remappings=[('/robot_description', robot_description_topic)]  # Remap topic robot_description jika diperlukan
        ),
        ExecuteProcess(  # Eksekusi proses eksternal untuk spawn robot ke Gazebo
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-topic', robot_description_topic,  # Ambil URDF dari topic robot_description (bisa di-remap)
                 '-entity', 'husky_with_cameras'],  # Nama entity di Gazebo
            output='screen'
        )
    ])