#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_yolobot_gazebo = get_package_share_directory('yolobot_gazebo')
    pkg_yolobot_description = get_package_share_directory('yolobot_description')
    pkg_yolobot_control = get_package_share_directory('yolobot_control')
    pkg_yolobot_recognition = get_package_share_directory('yolobot_recognition')

    # Set default GUI ke true (full Gazebo GUI)
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable Gazebo GUI (set to true for GUI, false for headless)'
    )

    joy_node = Node(
        package = "joy",
        executable = "joy_node"
    )

    # Forward argumen gui ke start_world_launch.py
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_world_launch.py'),
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
        )
    )     

    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_control, 'launch', 'yolobot_control.launch.py'),
        )
    )  

    yolov8_node = Node(
        package='yolobot_recognition',
        executable='yolov8_ros2_pt.py',
        output='screen'
    )

    yolov8_stitcher_node = Node(
        package='yolobot_recognition',
        executable='yolov8_stitcher_node.py',
        output='screen'
    )
    yolov8_panorama_inference_node = Node(
        package='yolobot_recognition',
        executable='yolov8_panorama_inference.py',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,  # <-- Argumen gui tetap bisa diubah saat launch, tapi default sekarang true (GUI aktif)
        joy_node,
        start_world,
        spawn_robot_world,
        spawn_robot_control,
        yolov8_node,
        yolov8_stitcher_node,
        yolov8_panorama_inference_node,
    ])

# ---------------------------
# CATATAN:
# - Sekarang default GUI aktif (full Gazebo GUI).
# - Untuk headless, jalankan: ros2 launch yolobot_gazebo yolobot_launch.py gui:=false
# - Untuk GUI, cukup: ros2 launch yolobot_gazebo yolobot_launch.py
# ---------------------------