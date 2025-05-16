#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, LogWarn, LogError, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
from typing import List

# ===================== ERROR HANDLING & VALIDASI =====================
def validate_pose(context, *args, **kwargs):
    pose_str = LaunchConfiguration('pose').perform(context)
    parts = pose_str.strip().split()
    if len(parts) != 6:
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        LogError(msg=f"Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'")
        sys.exit(21)
    try:
        [float(p) for p in parts]
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        LogError(msg=f"Semua elemen pose harus berupa angka: '{pose_str}'")
        sys.exit(22)
    return []

def check_dependencies(context, *args, **kwargs):
    if shutil.which('xacro') is None:
        print("[ERROR] Dependency 'xacro' tidak ditemukan di PATH. Install dengan: sudo apt install ros-humble-xacro", flush=True)
        LogError(msg="Dependency 'xacro' tidak ditemukan di PATH.")
        sys.exit(2)
    if shutil.which('ros2') is None:
        print("[ERROR] Dependency 'ros2' tidak ditemukan di PATH. Pastikan ROS2 environment sudah aktif.", flush=True)
        LogError(msg="Dependency 'ros2' tidak ditemukan di PATH.")
        sys.exit(3)
    try:
        get_package_share_directory('gazebo_ros')
        LogInfo(msg="Package 'gazebo_ros' ditemukan.")
    except Exception:
        print("[ERROR] Package 'gazebo_ros' tidak ditemukan. Install dengan: sudo apt install ros-humble-gazebo-ros-pkgs", flush=True)
        LogError(msg="Package 'gazebo_ros' tidak ditemukan.")
        sys.exit(4)
    return []

def check_urdf_file(context, *args, **kwargs):
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    urdf_file_path = os.path.expandvars(os.path.expanduser(urdf_file))
    if not os.path.isfile(urdf_file_path):
        print(f"[ERROR] File URDF/Xacro robot tidak ditemukan: {urdf_file_path}", flush=True)
        LogError(msg=f"File URDF/Xacro robot tidak ditemukan: {urdf_file_path}")
        sys.exit(1)
    if not os.access(urdf_file_path, os.R_OK):
        print(f"[ERROR] File URDF/Xacro robot tidak bisa dibaca (permission denied): {urdf_file_path}", flush=True)
        LogError(msg=f"File URDF/Xacro robot tidak bisa dibaca (permission denied): {urdf_file_path}")
        sys.exit(5)
    return []

def log_to_file(msg):
    log_file_path = os.path.expanduser("~/huskybot_spawn.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

# ===================== POSE SPLIT UTILITY (SAFE) =====================
def split_pose(context, *args, **kwargs) -> List[str]:
    pose_str = LaunchConfiguration('pose').perform(context)
    parts = pose_str.strip().split()
    if len(parts) != 6:
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        LogError(msg=f"Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'")
        sys.exit(21)
    try:
        [float(p) for p in parts]
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        LogError(msg=f"Semua elemen pose harus berupa angka: '{pose_str}'")
        sys.exit(22)
    return parts

# ===================== OPAQUE FUNCTION UNTUK SPAWN ENTITY =====================
def spawn_entity_action(context, *args, **kwargs):
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    pose_parts = split_pose(context)
    x, y, z, R, P, Y = pose_parts
    msg = f"Spawning entity: {entity_name} | Namespace: {robot_namespace} | Pose: {pose_parts} | Reference: {reference_frame}"
    print(f"[INFO] {msg}")
    LogInfo(msg=msg)
    log_to_file(msg)
    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', robot_description_topic,
                '-entity', entity_name,
                '-robot_namespace', robot_namespace,
                '-reference_frame', reference_frame,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', R,
                '-P', P,
                '-Y', Y,
            ],
            output='screen'
        )
    ]

# ===================== LAUNCH DESCRIPTION =====================
def generate_launch_description():
    try:
        urdf_file_arg = DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                get_package_share_directory('huskybot_description'),
                'robot',
                'huskybot.urdf.xacro'
            ),
            description='Path ke file URDF/Xacro robot (default: huskybot.urdf.xacro)'
        )
        entity_name_arg = DeclareLaunchArgument(
            'entity_name',
            default_value='husky_with_cameras',
            description='Nama entity robot di Gazebo'
        )
        pose_arg = DeclareLaunchArgument(
            'pose',
            default_value='0 0 0 0 0 0',
            description='Pose awal robot di Gazebo: x y z roll pitch yaw'
        )
        robot_namespace_arg = DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace robot di Gazebo (opsional)'
        )
        reference_frame_arg = DeclareLaunchArgument(
            'reference_frame',
            default_value='world',
            description='Reference frame untuk spawn entity (default: world)'
        )
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
        robot_description_topic_arg = DeclareLaunchArgument(
            'robot_description_topic',
            default_value='robot_description',
            description='Topic tempat robot_description di-publish (default: robot_description)'
        )

        urdf_file = LaunchConfiguration('urdf_file')
        entity_name = LaunchConfiguration('entity_name')
        pose = LaunchConfiguration('pose')
        robot_namespace = LaunchConfiguration('robot_namespace')
        reference_frame = LaunchConfiguration('reference_frame')
        use_sim_time = LaunchConfiguration('use_sim_time')
        robot_description_topic = LaunchConfiguration('robot_description_topic')

        check_deps_action = OpaqueFunction(function=check_dependencies)
        check_urdf_action = OpaqueFunction(function=check_urdf_file)
        validate_pose_action = OpaqueFunction(function=validate_pose)

        robot_description = ParameterValue(
            Command(['xacro ', urdf_file]),
            value_type=str
        )

        log_spawn = LogInfo(msg=["Spawning robot model: ", urdf_file])
        log_topic = LogInfo(msg=["robot_description topic: ", robot_description_topic])
        log_entity = LogInfo(msg=["Entity name: ", entity_name])
        log_pose = LogInfo(msg=["Spawn pose: ", pose])
        log_ns = LogInfo(msg=["Robot namespace: ", robot_namespace])
        log_ref = LogInfo(msg=["Reference frame: ", reference_frame])
        log_to_file(f"Spawning robot model: {urdf_file.perform({})}")
        log_to_file(f"robot_description topic: {robot_description_topic.perform({})}")
        log_to_file(f"Entity name: {entity_name.perform({})}")
        log_to_file(f"Spawn pose: {pose.perform({})}")
        log_to_file(f"Robot namespace: {robot_namespace.perform({})}")
        log_to_file(f"Reference frame: {reference_frame.perform({})}")

        return LaunchDescription([
            urdf_file_arg,
            entity_name_arg,
            pose_arg,
            robot_namespace_arg,
            reference_frame_arg,
            use_sim_time_arg,
            robot_description_topic_arg,
            check_deps_action,
            check_urdf_action,
            validate_pose_action,
            log_spawn,
            log_topic,
            log_entity,
            log_pose,
            log_ns,
            log_ref,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
                remappings=[('/robot_description', robot_description_topic)]
            ),
            OpaqueFunction(function=spawn_entity_action)
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

# ===================== PENJELASAN & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch.
# - Error handling sudah lengkap: cek dependency, cek file, validasi pose, cek permission.
# - Logging info ke terminal dan file untuk audit trail.
# - Sudah terhubung dengan robot_state_publisher dan spawn_entity Gazebo.
# - Siap untuk multi-robot (tinggal remap namespace dan entity_name).
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan validasi file YAML kalibrasi jika ingin spawn robot dengan sensor baru.
# - Saran: tambahkan logging ke file jika ingin audit lebih detail.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.