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

    joy_node = Node(
        package = "joy",
        executable = "joy_node"
    )

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_world_launch.py'),
        )
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

    # Jalankan node recognition langsung (opsional, jika ingin eksplisit)
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

    # Jika launch_yolov8.launch.py sudah menjalankan node ini, tidak perlu spawn_yolo lagi.
    # Jika ingin eksplisit, bisa pakai yolov8_node saja.
    return LaunchDescription([
        joy_node,
        start_world,
        spawn_robot_world,
        spawn_robot_control,
        yolov8_node,
        yolov8_stitcher_node,
        yolov8_panorama_inference_node,
    ])