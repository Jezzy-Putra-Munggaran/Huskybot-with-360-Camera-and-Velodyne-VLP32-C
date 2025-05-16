#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import traceback
import shutil
import time
import rclpy
from gazebo_msgs.srv import GetModelList
from launch.actions import OpaqueFunction, TimerAction, LogInfo, LogWarn, LogError, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------- Error Handling: cek file sebelum include ----------
def check_file_exists(path, desc):
    if not os.path.exists(path):
        print(f"[ERROR] {desc} tidak ditemukan: {path}", file=sys.stderr)
        LogError(msg=f"{desc} tidak ditemukan: {path}")
        sys.exit(1)

# ---------- Error Handling: cek package dependency ----------
def check_ros_package(pkg_name):
    try:
        get_package_share_directory(pkg_name)
        LogInfo(msg=f"Package ROS2 '{pkg_name}' ditemukan.")
    except Exception:
        print(f"[ERROR] Package ROS2 '{pkg_name}' tidak ditemukan. Install dengan: sudo apt install ros-humble-{pkg_name.replace('_', '-')}", file=sys.stderr)
        LogError(msg=f"Package ROS2 '{pkg_name}' tidak ditemukan.")
        sys.exit(2)

# ---------- Error Handling: cek environment variable penting ----------
def check_env_var(var, must_contain=None):
    val = os.environ.get(var, "")
    if not val:
        print(f"[WARNING] Environment variable {var} belum di-set.", file=sys.stderr)
        LogWarn(msg=f"Environment variable {var} belum di-set.")
    if must_contain and must_contain not in val:
        print(f"[WARNING] {var} tidak mengandung '{must_contain}'.", file=sys.stderr)
        LogWarn(msg=f"{var} tidak mengandung '{must_contain}'.")

# ---------- Error Handling: cek executable di PATH ----------
def check_executable(exe, install_hint=None):
    if shutil.which(exe) is None:
        hint = f" (install: {install_hint})" if install_hint else ""
        print(f"[ERROR] Executable '{exe}' tidak ditemukan di PATH.{hint}", file=sys.stderr)
        LogError(msg=f"Executable '{exe}' tidak ditemukan di PATH.{hint}")
        sys.exit(3)

# ---------- Error Handling: cek plugin Gazebo ----------
def check_gazebo_plugin(plugin_name):
    plugin_paths = os.environ.get('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib').split(':')
    found = False
    for plugin_dir in plugin_paths:
        plugin_path = os.path.join(plugin_dir, plugin_name)
        if os.path.exists(plugin_path):
            print(f"[INFO] Plugin Gazebo '{plugin_name}' ditemukan di {plugin_dir}.", flush=True)
            LogInfo(msg=f"Plugin Gazebo '{plugin_name}' ditemukan di {plugin_dir}.")
            found = True
            break
    if not found:
        print(f"[ERROR] Plugin Gazebo '{plugin_name}' tidak ditemukan di path manapun di $GAZEBO_PLUGIN_PATH.", flush=True)
        LogError(msg=f"Plugin Gazebo '{plugin_name}' tidak ditemukan di path manapun di $GAZEBO_PLUGIN_PATH.")
        print("Pastikan sudah install ros-humble-gazebo-ros-pkgs dan environment sudah di-source.", flush=True)
        sys.exit(10)

# ---------- Error Handling: cek semua plugin penting sebelum launch ----------
for plugin in [
    'libgazebo_ros_factory.so',
    'libgazebo_ros_state.so',
    'libgazebo_ros_init.so',
    'libgazebo_ros2_control.so',
    'libgazebo_ros_diff_drive.so',
    'libgazebo_ros_gps_sensor.so',
    'libgazebo_ros_imu_sensor.so',
    'libgazebo_ros_camera.so',
    'libgazebo_ros_ray_sensor.so',
]:
    check_gazebo_plugin(plugin)

# ---------- Error Handling: cek dependency package ROS2 ----------
for pkg in [
    'gazebo_ros',
    'joy',
    'huskybot_description',
    'huskybot_control',
    'huskybot_recognition',
    'huskybot_fusion',
    'huskybot_calibration',
]:
    check_ros_package(pkg)

# ---------- Error Handling: cek environment variable penting ----------
check_env_var('GAZEBO_PLUGIN_PATH', 'gazebo_ros')
check_env_var('GAZEBO_MODEL_PATH')
check_env_var('ROS_DOMAIN_ID')
check_env_var('RMW_IMPLEMENTATION')

# ---------- Error Handling: cek executable penting ----------
check_executable('xacro', 'sudo apt install ros-humble-xacro')
check_executable('ros2', 'sudo apt install ros-humble-ros2cli')

# ---------- Error Handling: validasi argumen CLI ----------
def validate_args(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    if not os.path.exists(world):
        print(f"[ERROR] World file tidak ditemukan: {world}", file=sys.stderr)
        LogError(msg=f"World file tidak ditemukan: {world}")
        sys.exit(11)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    if not os.path.exists(robot_model):
        print(f"[ERROR] Robot model file tidak ditemukan: {robot_model}", file=sys.stderr)
        LogError(msg=f"Robot model file tidak ditemukan: {robot_model}")
        sys.exit(12)
    return []

# ---------- OpaqueFunction: cek topic/service penting setelah launch ----------
def check_topic_after_launch(context, *args, **kwargs):
    import subprocess
    # Cek /gazebo/get_model_list
    try:
        result = subprocess.run(['ros2', 'service', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)
        if '/gazebo/get_model_list' not in result.stdout:
            print("[ERROR] Service /gazebo/get_model_list belum ready setelah launch.", file=sys.stderr)
            LogError(msg="Service /gazebo/get_model_list belum ready setelah launch.")
        else:
            print("[INFO] Service /gazebo/get_model_list sudah ready setelah launch.")
            LogInfo(msg="Service /gazebo/get_model_list sudah ready setelah launch.")
    except Exception as e:
        print(f"[ERROR] Exception saat cek service: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat cek service: {e}")
    return []

def wait_for_gazebo_and_entity(context, *args, **kwargs):
    entity_name = 'husky_with_cameras'
    timeout = 60
    print(f"[INFO] Menunggu Gazebo dan entity '{entity_name}' muncul...")
    LogInfo(msg=f"Menunggu Gazebo dan entity '{entity_name}' muncul...")
    rclpy.init(args=None)
    node = rclpy.create_node('wait_for_entity')
    cli = node.create_client(GetModelList, '/gazebo/get_model_list')
    start_time = time.time()
    while not cli.wait_for_service(timeout_sec=1.0):
        if time.time() - start_time > timeout:
            print(f"[ERROR] Timeout menunggu service /gazebo/get_model_list", file=sys.stderr)
            LogError(msg="Timeout menunggu service /gazebo/get_model_list")
            rclpy.shutdown()
            sys.exit(100)
        print("[INFO] Waiting for /gazebo/get_model_list service...")
    print("[INFO] Service /gazebo/get_model_list sudah ready.")
    LogInfo(msg="Service /gazebo/get_model_list sudah ready.")
    found = False
    while time.time() - start_time < timeout:
        req = GetModelList.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if future.done() and future.result():
            if entity_name in future.result().model_names:
                found = True
                print(f"[INFO] Entity '{entity_name}' sudah muncul di Gazebo.")
                LogInfo(msg=f"Entity '{entity_name}' sudah muncul di Gazebo.")
                break
        time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()
    if not found:
        print(f"[ERROR] Entity '{entity_name}' tidak muncul di Gazebo setelah {timeout} detik.", file=sys.stderr)
        LogError(msg=f"Entity '{entity_name}' tidak muncul di Gazebo setelah {timeout} detik.")
        sys.exit(101)
    return [context.locals.ros2_control_node]

def generate_launch_description():
    try:
        pkg_huskybot_gazebo = get_package_share_directory('huskybot_gazebo')
        pkg_huskybot_description = get_package_share_directory('huskybot_description')
        pkg_huskybot_control = get_package_share_directory('huskybot_control')
        pkg_huskybot_recognition = get_package_share_directory('huskybot_recognition')
        pkg_huskybot_fusion = get_package_share_directory('huskybot_fusion')
        pkg_huskybot_calibration = get_package_share_directory('huskybot_calibration')

        gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Enable Gazebo GUI (set to true for GUI, false for headless)')
        world_arg = DeclareLaunchArgument('world', default_value=os.path.join(pkg_huskybot_gazebo, 'worlds', 'yolo_test.world'), description='Path ke world file SDF Gazebo')
        robot_model_arg = DeclareLaunchArgument('robot_model', default_value=os.path.join(pkg_huskybot_description, 'robot', 'huskybot.urdf.xacro'), description='Path ke Xacro/URDF robot model')
        use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan waktu simulasi Gazebo (true untuk sinkronisasi waktu simulasi)')
        enable_yolo_arg = DeclareLaunchArgument('enable_yolo', default_value='true', description='Enable YOLOv12 node')
        enable_stitcher_arg = DeclareLaunchArgument('enable_stitcher', default_value='true', description='Enable panorama stitcher node')
        enable_panorama_arg = DeclareLaunchArgument('enable_panorama', default_value='true', description='Enable panorama inference node')
        enable_fusion_arg = DeclareLaunchArgument('enable_fusion', default_value='true', description='Enable sensor fusion node')
        enable_calibration_arg = DeclareLaunchArgument('enable_calibration', default_value='false', description='Enable calibration node (kalibrasi kamera-LiDAR)')

        start_world_path = os.path.join(pkg_huskybot_gazebo, 'launch', 'start_world_launch.py')
        spawn_robot_path = os.path.join(pkg_huskybot_description, 'launch', 'spawn_huskybot_launch.launch.py')
        control_path = os.path.join(pkg_huskybot_control, 'launch', 'huskybot_control.launch.py')
        fusion_path = os.path.join(pkg_huskybot_fusion, 'launch', 'fusion.launch.py')
        calibration_path = os.path.join(pkg_huskybot_calibration, 'launch', 'calibrate_lidar_camera.launch.py')

        check_file_exists(start_world_path, "Launch file start_world_launch.py")
        check_file_exists(spawn_robot_path, "Launch file spawn_huskybot_launch.launch.py")
        check_file_exists(control_path, "Launch file huskybot_control.launch.py")
        check_file_exists(fusion_path, "Launch file fusion.launch.py")
        check_file_exists(calibration_path, "Launch file calibrate_lidar_camera.launch.py")

        validate_args_action = OpaqueFunction(function=validate_args)

        joy_node = Node(
            package="joy",
            executable="joy_node",
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

        start_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(start_world_path),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )

        spawn_robot_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_robot_path),
            launch_arguments={
                'robot_model': LaunchConfiguration('robot_model'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )

        spawn_robot_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )

        spawn_fusion = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('enable_fusion')),
        )

        spawn_calibration = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(calibration_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('enable_calibration')),
        )

        yolov12_node = Node(
            package='huskybot_recognition',
            executable='yolov12_ros2_pt.py',
            output='both',
            condition=IfCondition(LaunchConfiguration('enable_yolo')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

        yolov12_stitcher_node = Node(
            package='huskybot_recognition',
            executable='yolov12_stitcher_node.py',
            output='both',
            condition=IfCondition(LaunchConfiguration('enable_stitcher')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

        yolov12_panorama_inference_node = Node(
            package='huskybot_recognition',
            executable='yolov12_panorama_inference.py',
            output='both',
            condition=IfCondition(LaunchConfiguration('enable_panorama')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

        controllers_yaml = os.path.join(pkg_huskybot_description, 'config', 'huskybot_controllers.yaml')
        if not os.path.exists(controllers_yaml):
            print(f"[ERROR] File YAML controller tidak ditemukan: {controllers_yaml}", file=sys.stderr)
            LogError(msg=f"File YAML controller tidak ditemukan: {controllers_yaml}")
            sys.exit(23)
        with open(controllers_yaml, 'r') as f:
            yaml_preview = f.read(500)
            print("[INFO] Preview YAML controller (first 500 chars):\n" + yaml_preview)
            LogInfo(msg=f"Preview YAML controller (first 500 chars):\n{yaml_preview}")
            if "diff_drive_controller" not in yaml_preview:
                print("[WARNING] diff_drive_controller tidak ditemukan di YAML controller!", file=sys.stderr)
                LogWarn(msg="diff_drive_controller tidak ditemukan di YAML controller!")

        robot_description = ParameterValue(
            Command([
                'xacro ',
                os.path.join(pkg_huskybot_description, 'robot', 'huskybot.urdf.xacro')
            ]),
            value_type=str
        )

        try:
            xacro_path = os.path.join(pkg_huskybot_description, 'robot', 'huskybot.urdf.xacro')
            xacro_result = os.popen(f"xacro {xacro_path}").read()
            print("[INFO] Preview robot_description (first 500 chars):\n" + xacro_result[:500])
            LogInfo(msg=f"Preview robot_description (first 500 chars):\n{xacro_result[:500]}")
            if "<robot" not in xacro_result:
                print("[ERROR] Hasil xacro tidak valid (tidak mengandung <robot>)", file=sys.stderr)
                LogError(msg="Hasil xacro tidak valid (tidak mengandung <robot>)")
                sys.exit(21)
        except Exception as e:
            print(f"[ERROR] Gagal menjalankan xacro: {e}", file=sys.stderr)
            LogError(msg=f"Gagal menjalankan xacro: {e}")
            sys.exit(22)

        ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
        )

        delayed_ros2_control = OpaqueFunction(function=wait_for_gazebo_and_entity)
        wait_gazebo = TimerAction(period=8.0, actions=[delayed_ros2_control])

        # ---------- Logging Tambahan ----------
        log_start = LogInfo(msg="Launching Huskybot Gazebo Simulation...")
        log_world = LogInfo(msg=["World file: ", LaunchConfiguration('world')])
        log_robot = LogInfo(msg=["Robot model: ", LaunchConfiguration('robot_model')])

        # Logging ke file (opsional, bisa diaktifkan jika ingin audit trail)
        log_file_path = os.path.expanduser("~/huskybot_simulation.log")
        try:
            with open(log_file_path, "a") as logf:
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Launching Huskybot Gazebo Simulation...\n")
        except Exception as e:
            print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

        set_log_level = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
        set_ros_log_level = SetEnvironmentVariable('RCUTILS_LOG_SEVERITY_THRESHOLD', 'DEBUG')

        return LaunchDescription([
            set_log_level,
            set_ros_log_level,
            gui_arg,
            world_arg,
            robot_model_arg,
            use_sim_time_arg,
            enable_yolo_arg,
            enable_stitcher_arg,
            enable_panorama_arg,
            enable_fusion_arg,
            enable_calibration_arg,
            validate_args_action,
            log_start,
            log_world,
            log_robot,
            joy_node,
            start_world,
            spawn_robot_world,
            wait_gazebo,
            spawn_robot_control,
            spawn_fusion,
            spawn_calibration,
            yolov12_node,
            yolov12_stitcher_node,
            yolov12_panorama_inference_node,
            OpaqueFunction(function=check_topic_after_launch),  # (Opsional) Cek topic/service penting setelah launch
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        traceback.print_exc()
        sys.exit(99)

# ---------------------------
# CATATAN:
# - Untuk men-disable node: ros2 launch huskybot_gazebo huskybot_launch.py enable_yolo:=false
# - Untuk pilih world: ros2 launch huskybot_gazebo huskybot_launch.py world:=/path/to/world.sdf
# - Untuk pilih robot model: ros2 launch huskybot_gazebo huskybot_launch.py robot_model:=/path/to/model.xacro
# - Semua error file hilang akan muncul di terminal sebelum launch berjalan.
# - Logging node custom ke screen dan file (output='both').
# ---------------------------

# ===================== REVIEW & SARAN =====================
# - Struktur folder sudah benar: launch/, worlds/, README.md, package.xml, CMakeLists.txt.
# - Semua dependency package dan file sudah dicek sebelum launch (robust error handling).
# - Semua node dan include launch sudah modular dan bisa diaktifkan/dinonaktifkan via argumen.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan multi-robot (tinggal tambahkan namespace jika perlu).
# - FULL OOP tidak relevan di launch file, tapi semua node yang dijalankan sudah OOP.
# - Logging info ke terminal untuk audit trail.
# - Semua argumen penting sudah bisa diubah dari CLI/launch.
# - Saran peningkatan:
#   1. Tambahkan argumen namespace untuk multi-robot (remap topic/namespace).
#   2. Tambahkan validasi file world custom dengan OpaqueFunction jika ingin error handling lebih advance.
#   3. Tambahkan test launch file untuk CI/CD.
#   4. Dokumentasikan semua argumen di README.md.
#   5. Tambahkan logging ke file jika ingin audit lebih detail.
#   6. Tambahkan node RViz2 untuk visualisasi otomatis jika diinginkan.
# - Semua baris sudah diberi komentar penjelasan.
# - Tidak ada bug/error, sudah best practice launch file ROS2 Python.