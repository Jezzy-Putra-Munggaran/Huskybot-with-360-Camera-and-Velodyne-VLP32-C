#!/usr/bin/python3
# -*- coding: utf-8 -*- 

import os  # Untuk operasi path file
import sys  # Untuk exit/error handling
import shutil  # Untuk cek dependency xacro/ros2 di PATH
import time  # Untuk timestamp log
from ament_index_python.packages import get_package_share_directory  # Untuk cari path share package ROS2
from launch import LaunchDescription  # Base class LaunchDescription ROS2
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # Untuk deklarasi argumen dan fungsi custom
from launch.substitutions import LaunchConfiguration, Command  # Untuk ambil nilai argumen launch
from launch_ros.actions import Node  # Untuk deklarasi node ROS2 di launch file
from launch_ros.parameter_descriptions import ParameterValue  # Untuk parameter robot_description
from launch.actions import ExecuteProcess  # Untuk eksekusi proses eksternal (spawn_entity.py)
from typing import List  # Untuk type hinting list

# ===================== ERROR HANDLING & VALIDASI =====================
def validate_pose(context, *args, **kwargs):  # Validasi argumen pose (x y z roll pitch yaw)
    pose_str = LaunchConfiguration('pose').perform(context)  # Ambil nilai pose dari argumen launch
    parts = pose_str.strip().split()  # Split string pose jadi list
    if len(parts) != 6:  # Harus 6 elemen
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        sys.exit(21)  # Exit dengan kode error
    try:
        [float(p) for p in parts]  # Pastikan semua elemen bisa dikonversi ke float
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        sys.exit(22)
    return []

def check_dependencies(context, *args, **kwargs):  # Cek dependency penting (xacro, ros2, gazebo_ros)
    if shutil.which('xacro') is None:  # Cek xacro di PATH
        print("[ERROR] Dependency 'xacro' tidak ditemukan di PATH. Install dengan: sudo apt install ros-humble-xacro", flush=True)
        sys.exit(2)
    if shutil.which('ros2') is None:  # Cek ros2 di PATH
        print("[ERROR] Dependency 'ros2' tidak ditemukan di PATH. Pastikan ROS2 environment sudah aktif.", flush=True)
        sys.exit(3)
    try:
        get_package_share_directory('gazebo_ros')  # Cek package gazebo_ros
        print("[INFO] Package 'gazebo_ros' ditemukan.", flush=True)
    except Exception:
        print("[ERROR] Package 'gazebo_ros' tidak ditemukan. Install dengan: sudo apt install ros-humble-gazebo-ros-pkgs", flush=True)
        sys.exit(4)
    return []

def check_urdf_file(context, *args, **kwargs):  # Cek file URDF/Xacro ada dan bisa dibaca
    urdf_file = LaunchConfiguration('urdf_file').perform(context)  # Ambil path file URDF dari argumen
    urdf_file_path = os.path.expandvars(os.path.expanduser(urdf_file))  # Expand ~ dan env var
    if not os.path.isfile(urdf_file_path):  # Cek file ada
        print(f"[ERROR] File URDF/Xacro robot tidak ditemukan: {urdf_file_path}", flush=True)
        sys.exit(1)
    if not os.access(urdf_file_path, os.R_OK):  # Cek permission read
        print(f"[ERROR] File URDF/Xacro robot tidak bisa dibaca (permission denied): {urdf_file_path}", flush=True)
        sys.exit(5)
    return []

def log_to_file(msg):  # Logging ke file audit trail
    log_file_path = os.path.expanduser("~/huskybot_spawn.log")  # Path file log
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")  # Tulis log dengan timestamp
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

# ===================== POSE SPLIT UTILITY (SAFE) =====================
def split_pose(context, *args, **kwargs) -> List[str]:  # Utility untuk split pose jadi list string
    pose_str = LaunchConfiguration('pose').perform(context)
    parts = pose_str.strip().split()
    if len(parts) != 6:
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        sys.exit(21)
    try:
        [float(p) for p in parts]
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        sys.exit(22)
    return parts

# ===================== OPAQUE FUNCTION UNTUK LOGGING ARGUMEN =====================
def log_launch_args(context, *args, **kwargs):  # Logging semua argumen launch ke file (dengan context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)
    pose = LaunchConfiguration('pose').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(context)
    log_to_file(f"Spawning robot model: {urdf_file}")
    log_to_file(f"robot_description topic: {robot_description_topic}")
    log_to_file(f"Entity name: {entity_name}")
    log_to_file(f"Spawn pose: {pose}")
    log_to_file(f"Robot namespace: {robot_namespace}")
    log_to_file(f"Reference frame: {reference_frame}")
    return []

# ===================== OPAQUE FUNCTION UNTUK SPAWN ENTITY =====================
def spawn_entity_action(context, *args, **kwargs):  # Fungsi untuk spawn entity ke Gazebo
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(context)  # Topic robot_description
    entity_name = LaunchConfiguration('entity_name').perform(context)  # Nama entity di Gazebo
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)  # Namespace robot
    reference_frame = LaunchConfiguration('reference_frame').perform(context)  # Reference frame spawn
    pose_parts = split_pose(context)  # Ambil pose (x y z R P Y)
    x, y, z, R, P, Y = pose_parts  # Unpack pose
    msg = f"Spawning entity: {entity_name} | Namespace: {robot_namespace} | Pose: {pose_parts} | Reference: {reference_frame}"
    print(f"[INFO] {msg}")  # Info ke terminal
    log_to_file(msg)  # Log ke file
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
def generate_launch_description():  # Fungsi utama generate LaunchDescription
    try:
        urdf_file_arg = DeclareLaunchArgument(
            'urdf_file',  # Nama argumen
            default_value=os.path.join(
                get_package_share_directory('huskybot_description'),  # Path share package
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

        urdf_file = LaunchConfiguration('urdf_file')  # Konfigurasi path URDF
        entity_name = LaunchConfiguration('entity_name')  # Konfigurasi nama entity
        pose = LaunchConfiguration('pose')  # Konfigurasi pose
        robot_namespace = LaunchConfiguration('robot_namespace')  # Konfigurasi namespace
        reference_frame = LaunchConfiguration('reference_frame')  # Konfigurasi reference frame
        use_sim_time = LaunchConfiguration('use_sim_time')  # Konfigurasi use_sim_time
        robot_description_topic = LaunchConfiguration('robot_description_topic')  # Konfigurasi topic robot_description

        check_deps_action = OpaqueFunction(function=check_dependencies)  # Cek dependency sebelum launch
        check_urdf_action = OpaqueFunction(function=check_urdf_file)  # Cek file URDF sebelum launch
        validate_pose_action = OpaqueFunction(function=validate_pose)  # Validasi pose sebelum launch
        log_args_action = OpaqueFunction(function=log_launch_args)  # Logging argumen launch ke file

        robot_description = ParameterValue(
            Command(['xacro ', urdf_file]),  # Jalankan xacro untuk generate URDF string
            value_type=str
        )

        # Logging info ke terminal
        print("[INFO]", "Spawning robot model:", urdf_file, flush=True)
        print("[INFO]", "robot_description topic:", robot_description_topic, flush=True)
        print("[INFO]", "Entity name:", entity_name, flush=True)
        print("[INFO]", "Spawn pose:", pose, flush=True)
        print("[INFO]", "Robot namespace:", robot_namespace, flush=True)
        print("[INFO]", "Reference frame:", reference_frame, flush=True)

        return LaunchDescription([
            urdf_file_arg,  # Argumen path URDF
            entity_name_arg,  # Argumen nama entity
            pose_arg,  # Argumen pose
            robot_namespace_arg,  # Argumen namespace
            reference_frame_arg,  # Argumen reference frame
            use_sim_time_arg,  # Argumen use_sim_time
            robot_description_topic_arg,  # Argumen topic robot_description
            check_deps_action,  # Action cek dependency
            check_urdf_action,  # Action cek file URDF
            validate_pose_action,  # Action validasi pose
            log_args_action,  # Logging argumen launch ke file
            Node(
                package='robot_state_publisher',  # Node robot_state_publisher untuk publish TF
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
                remappings=[('/robot_description', robot_description_topic)]
            ),
            OpaqueFunction(function=spawn_entity_action)  # Action spawn entity ke Gazebo
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
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
# - Saran: jika ingin FULL OOP, bisa wrap semua fungsi ke dalam class, tapi untuk launch file Python ROS2, fungsi modular sudah best practice.