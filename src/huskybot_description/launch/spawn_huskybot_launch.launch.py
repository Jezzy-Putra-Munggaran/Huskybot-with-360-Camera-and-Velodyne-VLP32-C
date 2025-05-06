#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os  # Modul os untuk operasi path file
import shutil  # Untuk cek dependency (xacro, gazebo_ros)
from ament_index_python.packages import get_package_share_directory  # Cari path share ROS2 package
from launch import LaunchDescription  # Kelas utama untuk launch file ROS2
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction  # Untuk deklarasi argumen, logging info, dan fungsi custom
from launch.substitutions import LaunchConfiguration, Command  # Untuk ambil nilai argumen dan jalankan perintah shell
from launch_ros.actions import Node  # Untuk menjalankan node ROS2
from launch_ros.parameter_descriptions import ParameterValue  # Untuk parameter node yang bisa dieksekusi (misal hasil xacro)
from launch.actions import ExecuteProcess  # Untuk menjalankan proses eksternal (spawn_entity)
from typing import List  # Untuk type hint OpaqueFunction

# ===================== ERROR HANDLING & VALIDASI =====================
def validate_pose(context, *args, **kwargs):  # Fungsi custom untuk validasi pose string
    pose_str = LaunchConfiguration('pose').perform(context)  # Ambil pose string dari argumen
    parts = pose_str.strip().split()
    if len(parts) != 6:  # Harus 6 elemen (x y z roll pitch yaw)
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        exit(21)
    try:
        [float(p) for p in parts]  # Cek semua bisa dikonversi ke float
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        exit(22)
    return []  # Tidak perlu return action apapun

def check_dependencies(context, *args, **kwargs):  # Cek dependency penting sebelum launch
    if shutil.which('xacro') is None:  # Pastikan xacro sudah terinstall
        print("[ERROR] Dependency 'xacro' tidak ditemukan di PATH. Install dengan: sudo apt install ros-humble-xacro", flush=True)
        exit(2)
    if shutil.which('ros2') is None:  # Pastikan ros2 CLI ada (untuk spawn_entity.py)
        print("[ERROR] Dependency 'ros2' tidak ditemukan di PATH. Pastikan ROS2 environment sudah aktif.", flush=True)
        exit(3)
    try:
        get_package_share_directory('gazebo_ros')  # Cek package gazebo_ros
    except Exception:
        print("[ERROR] Package 'gazebo_ros' tidak ditemukan. Install dengan: sudo apt install ros-humble-gazebo-ros-pkgs", flush=True)
        exit(4)
    return []

def check_urdf_file(context, *args, **kwargs):  # Cek file URDF/Xacro ada dan bisa dibaca
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    urdf_file_path = os.path.expandvars(os.path.expanduser(urdf_file))
    if not os.path.isfile(urdf_file_path):
        print(f"[ERROR] File URDF/Xacro robot tidak ditemukan: {urdf_file_path}", flush=True)
        exit(1)
    if not os.access(urdf_file_path, os.R_OK):
        print(f"[ERROR] File URDF/Xacro robot tidak bisa dibaca (permission denied): {urdf_file_path}", flush=True)
        exit(5)
    return []

# ===================== POSE SPLIT UTILITY (SAFE) =====================
def split_pose(context, *args, **kwargs) -> List[str]:
    pose_str = LaunchConfiguration('pose').perform(context)
    parts = pose_str.strip().split()
    if len(parts) != 6:
        print(f"[ERROR] Argumen pose harus 6 angka (x y z roll pitch yaw), sekarang: '{pose_str}'", flush=True)
        exit(21)
    try:
        [float(p) for p in parts]
    except Exception:
        print(f"[ERROR] Semua elemen pose harus berupa angka: '{pose_str}'", flush=True)
        exit(22)
    return parts

# ===================== OPAQUE FUNCTION UNTUK SPAWN ENTITY =====================
def spawn_entity_action(context, *args, **kwargs):
    # Ambil semua argumen yang dibutuhkan
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(context)
    entity_name = LaunchConfiguration('entity_name').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    pose_parts = split_pose(context)
    # Mapping pose_parts ke x, y, z, R, P, Y
    x, y, z, R, P, Y = pose_parts
    # Jalankan proses spawn_entity.py dengan argumen yang sudah diparsing
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
def generate_launch_description():  # Fungsi utama ROS2 untuk launch file

    # ---------- Declare Launch Arguments ----------
    urdf_file_arg = DeclareLaunchArgument(  # Argumen path file URDF/Xacro robot (bisa diubah user)
        'urdf_file',
        default_value=os.path.join(
            get_package_share_directory('huskybot_description'),
            'robot',
            'huskybot.urdf.xacro'
        ),
        description='Path ke file URDF/Xacro robot (default: huskybot.urdf.xacro)'
    )
    entity_name_arg = DeclareLaunchArgument(  # Argumen nama entity di Gazebo
        'entity_name',
        default_value='husky_with_cameras',
        description='Nama entity robot di Gazebo'
    )
    pose_arg = DeclareLaunchArgument(  # Argumen pose robot (x y z roll pitch yaw)
        'pose',
        default_value='0 0 0 0 0 0',
        description='Pose awal robot di Gazebo: x y z roll pitch yaw'
    )
    robot_namespace_arg = DeclareLaunchArgument(  # Argumen namespace robot
        'robot_namespace',
        default_value='',
        description='Namespace robot di Gazebo (opsional)'
    )
    reference_frame_arg = DeclareLaunchArgument(  # Argumen reference frame
        'reference_frame',
        default_value='world',
        description='Reference frame untuk spawn entity (default: world)'
    )
    use_sim_time_arg = DeclareLaunchArgument(  # Argumen use_sim_time
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    robot_description_topic_arg = DeclareLaunchArgument(  # Argumen topic robot_description
        'robot_description_topic',
        default_value='robot_description',
        description='Topic tempat robot_description di-publish (default: robot_description)'
    )

    # ---------- Ambil nilai argumen ----------
    urdf_file = LaunchConfiguration('urdf_file')  # Path ke file URDF/Xacro robot
    entity_name = LaunchConfiguration('entity_name')  # Nama entity di Gazebo
    pose = LaunchConfiguration('pose')  # Pose robot
    robot_namespace = LaunchConfiguration('robot_namespace')  # Namespace robot
    reference_frame = LaunchConfiguration('reference_frame')  # Reference frame
    use_sim_time = LaunchConfiguration('use_sim_time')  # Use sim time
    robot_description_topic = LaunchConfiguration('robot_description_topic')  # Topic robot_description

    # ---------- Error Handling: cek dependency & file ----------
    check_deps_action = OpaqueFunction(function=check_dependencies)  # Cek dependency sebelum launch
    check_urdf_action = OpaqueFunction(function=check_urdf_file)     # Cek file URDF/Xacro sebelum launch

    # ---------- Jalankan xacro untuk menghasilkan URDF string ----------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),  # Eksekusi xacro ke file xacro, hasilnya string URDF
        value_type=str
    )

    # ---------- Logging info ----------
    log_spawn = LogInfo(msg=["Spawning robot model: ", urdf_file])  # Logging info model yang di-spawn
    log_topic = LogInfo(msg=["robot_description topic: ", robot_description_topic])  # Logging info topic robot_description
    log_entity = LogInfo(msg=["Entity name: ", entity_name])  # Logging entity name
    log_pose = LogInfo(msg=["Spawn pose: ", pose])  # Logging pose
    log_ns = LogInfo(msg=["Robot namespace: ", robot_namespace])  # Logging robot namespace
    log_ref = LogInfo(msg=["Reference frame: ", reference_frame])  # Logging reference frame

    # ---------- LaunchDescription ----------
    return LaunchDescription([
        urdf_file_arg,  # Argumen path file URDF/Xacro
        entity_name_arg,  # Argumen nama entity
        pose_arg,  # Argumen pose
        robot_namespace_arg,  # Argumen namespace robot
        reference_frame_arg,  # Argumen reference frame
        use_sim_time_arg,  # Argumen use_sim_time
        robot_description_topic_arg,  # Argumen topic robot_description
        check_deps_action,  # Cek dependency sebelum launch
        check_urdf_action,  # Cek file URDF/Xacro sebelum launch
        OpaqueFunction(function=validate_pose),  # Validasi pose string sebelum launch
        log_spawn,  # Logging info model yang di-spawn
        log_topic,  # Logging info topic robot_description
        log_entity,  # Logging entity name
        log_pose,  # Logging pose
        log_ns,    # Logging robot namespace
        log_ref,   # Logging reference frame
        Node(  # Jalankan robot_state_publisher untuk publish TF dan robot_description
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            remappings=[('/robot_description', robot_description_topic)]  # Remap topic robot_description jika diperlukan
        ),
        OpaqueFunction(function=spawn_entity_action)  # Jalankan proses spawn_entity.py dengan parsing pose yang benar
    ])

# ===================== PENJELASAN & SARAN =====================
# - Semua argumen sudah modular dan bisa diubah saat launch.
# - Error handling sudah lengkap: cek dependency, cek file, validasi pose, cek permission.
# - Logging info ke terminal untuk audit trail.
# - Sudah terhubung dengan robot_state_publisher dan spawn_entity Gazebo.
# - Siap untuk multi-robot (tinggal remap namespace dan entity_name).
# - FULL OOP tidak relevan di launch file, tapi sudah best practice dan maintainable.
# - Saran: jika ingin OOP lebih lanjut, buat class Python untuk preset argumen multi-robot.
# - Saran: tambahkan unit test launch file di folder test/ untuk CI/CD.
# - Saran: tambahkan validasi file YAML kalibrasi jika ingin spawn robot dengan sensor baru.
# - Saran: tambahkan logging ke file jika ingin audit lebih detail.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.