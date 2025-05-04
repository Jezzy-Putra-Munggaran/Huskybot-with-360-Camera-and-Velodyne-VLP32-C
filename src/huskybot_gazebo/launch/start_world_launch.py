#!/usr/bin/python3
# -*- coding: utf-8 -*-

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
        description='Set true untuk output verbose Gazebo (debugging)'
    )
    pause_arg = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Set true untuk memulai Gazebo dalam keadaan pause'
    )

    # ---------- Validasi File World ----------
    world_path = LaunchConfiguration('world')                                         # Ambil path world dari argumen
    # Note: LaunchConfiguration belum bisa di-evaluate di Python, jadi validasi manual hanya untuk default
    if not os.path.exists(world_default):
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