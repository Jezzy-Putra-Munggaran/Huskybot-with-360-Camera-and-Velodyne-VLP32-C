#!/usr/bin/python3  
# -*- coding: utf-8 -*- 

import os  # Untuk operasi path file (mencari file RViz config)
import shutil  # Untuk pengecekan dependency RViz2 (error handling)
from ament_index_python.packages import get_package_share_directory  # Cari path share ROS2 package
from launch import LaunchDescription  # Kelas utama untuk launch file ROS2
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction  # Untuk deklarasi argumen, logging, dan fungsi custom
from launch.substitutions import LaunchConfiguration  # Untuk ambil nilai argumen launch
from launch_ros.actions import Node  # Untuk menjalankan node ROS2

# ===================== OOP CONFIG WRAPPER =====================
class RvizDisplayConfig:
    def __init__(self, package_name='huskybot_description', default_rviz='huskybot.rviz'):
        self.package_name = package_name  # Nama package
        self.default_rviz = default_rviz  # Nama file RViz config default
        # Preset sensor topic default (bisa di-extend sesuai kebutuhan multi-robot)
        self.sensor_topics = {
            'velodyne_points': '/velodyne_points',
            'imu_data': '/imu/data'
        }
        # Preset kamera 6 arah (front, front_left, left, rear, rear_right, right)
        self.camera_names = [
            'front', 'front_left', 'left', 'rear', 'rear_right', 'right'
        ]

    def get_default_rviz_config(self):
        # Mengembalikan path absolut file RViz config default
        return os.path.join(
            get_package_share_directory(self.package_name),  # Path share package
            'rviz',
            self.default_rviz
        )

    def get_sensor_topic(self, key):
        # Mengambil topic sensor dari preset, default '' jika tidak ada
        return self.sensor_topics.get(key, '')

    def get_camera_topic(self, camera_name, robot_ns=''):
        # Menghasilkan topic kamera sesuai nama dan namespace (multi-robot ready)
        ns = f"/{robot_ns.strip('/')}" if robot_ns else ""
        return f"{ns}/camera_{camera_name}/image_raw"

    def generate_multi_robot_args(self, robot_ns):
        # Membuat preset argumen multi-robot otomatis (untuk launch multi robot, 6 kamera)
        ns = robot_ns.strip('/')
        args = {
            'robot_description_topic': f'/{ns}/robot_description',
            'tf_prefix': f'{ns}/',
            'velodyne_topic': f'/{ns}/velodyne_points',
            'imu_data_topic': f'/{ns}/imu/data'
        }
        # Tambahkan semua kamera ke argumen
        for cam in self.camera_names:
            args[f'camera_{cam}_image_topic'] = f'/{ns}/camera_{cam}/image_raw'
        return args

# ===================== ERROR HANDLING =====================
def validate_rviz_config(context, *args, **kwargs):
    # Validasi file RViz config sebelum launch
    rvizconfig = LaunchConfiguration('rvizconfig').perform(context)
    if not os.path.isfile(rvizconfig):
        print(f"[WARNING] File RViz config tidak ditemukan: {rvizconfig}. RViz2 akan jalan tanpa konfigurasi.", flush=True)
    return []

def check_rviz2_dependency(context, *args, **kwargs):
    # Cek dependency RViz2 sebelum launch
    if shutil.which('rviz2') is None:
        print("[ERROR] Dependency 'rviz2' tidak ditemukan di PATH. Install dengan: sudo apt install ros-humble-rviz2", flush=True)
        exit(2)
    return []

def check_rviz_config_permission(context, *args, **kwargs):
    # Cek permission file RViz config (tidak readable)
    rvizconfig = LaunchConfiguration('rvizconfig').perform(context)
    if os.path.isfile(rvizconfig) and not os.access(rvizconfig, os.R_OK):
        print(f"[ERROR] File RViz config tidak bisa dibaca (permission denied): {rvizconfig}", flush=True)
        exit(3)
    return []

def check_sensor_topic_conflict(context, *args, **kwargs):
    # Cek duplikasi/remap topic kamera (prevent typo multi-robot)
    camera_topics = set()
    for cam in RvizDisplayConfig().camera_names:
        topic = LaunchConfiguration(f'camera_{cam}_image_topic').perform(context)
        if topic in camera_topics:
            print(f"[ERROR] Topic kamera duplikat/remap: {topic}", flush=True)
            exit(4)
        camera_topics.add(topic)
    return []

# ===================== LAUNCH DESCRIPTION =====================
def generate_launch_description():
    rviz_cfg = RvizDisplayConfig()  # Inisialisasi konfigurasi RViz OOP

    # Argumen file konfigurasi RViz (bisa diubah user saat launch)
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=rviz_cfg.get_default_rviz_config(),
        description='Path ke file konfigurasi RViz2'
    )

    # Argumen remap topic robot_description (untuk multi-robot atau custom topic)
    robot_description_topic_arg = DeclareLaunchArgument(
        'robot_description_topic',
        default_value='robot_description',
        description='Topic robot_description yang akan di-subscribe RViz2'
    )

    # Argumen remap TF prefix (multi-robot support)
    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='TF Prefix untuk multi-robot (kosongkan jika single robot)'
    )

    # Argumen sensor lain (Velodyne, IMU)
    velodyne_topic_arg = DeclareLaunchArgument(
        'velodyne_topic',
        default_value=rviz_cfg.get_sensor_topic('velodyne_points'),
        description='Topic pointcloud Velodyne (untuk multi-robot bisa di-remap)'
    )
    imu_data_topic_arg = DeclareLaunchArgument(
        'imu_data_topic',
        default_value=rviz_cfg.get_sensor_topic('imu_data'),
        description='Topic IMU data (untuk multi-robot bisa di-remap)'
    )

    # Argumen kamera 6 arah (otomatis, bisa di-loop)
    camera_args = []
    for cam in rviz_cfg.camera_names:
        camera_args.append(
            DeclareLaunchArgument(
                f'camera_{cam}_image_topic',
                default_value=rviz_cfg.get_camera_topic(cam),
                description=f'Topic kamera {cam} (untuk multi-robot bisa di-remap)'
            )
        )

    # Error handling: Cek dependency RViz2 sebelum launch
    check_rviz2_action = OpaqueFunction(function=check_rviz2_dependency)
    # Error handling: Validasi file RViz config sebelum launch
    validate_rviz_action = OpaqueFunction(function=validate_rviz_config)
    # Error handling: Cek permission file RViz config
    check_rviz_config_permission_action = OpaqueFunction(function=check_rviz_config_permission)
    # Error handling: Cek duplikasi/remap topic kamera
    check_sensor_topic_conflict_action = OpaqueFunction(function=check_sensor_topic_conflict)

    # Logging info config RViz ke terminal
    log_rviz = LogInfo(msg=["Menjalankan RViz2 dengan config: ", LaunchConfiguration('rvizconfig')])

    # Node robot_state_publisher agar RViz2 bisa standalone (bisa di-comment jika sudah dijalankan launch lain)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '',  # Kosongkan, asumsikan sudah di-publish oleh launch utama, atau bisa diisi dari file jika standalone
            'use_sim_time': True
        }],
        remappings=[('/robot_description', LaunchConfiguration('robot_description_topic'))]
    )

    # Node RViz2, siap untuk remap topic dan TF prefix (multi-robot & multi-sensor ready)
    # Remap semua kamera 6 arah secara dinamis
    camera_remaps = [
        (f'/camera_{cam}/image_raw', LaunchConfiguration(f'camera_{cam}_image_topic'))
        for cam in rviz_cfg.camera_names
    ]
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        remappings=[
            ('/robot_description', LaunchConfiguration('robot_description_topic')),
            ('/velodyne_points', LaunchConfiguration('velodyne_topic')),
            ('/imu/data', LaunchConfiguration('imu_data_topic')),
            *camera_remaps  # Unpack semua remap kamera
        ],
        parameters=[{'tf_prefix': LaunchConfiguration('tf_prefix')}],
    )

    # LaunchDescription: Semua argumen, validasi, logging, dan node dikumpulkan
    return LaunchDescription([
        rviz_config_arg,                        # Argumen file konfigurasi RViz
        robot_description_topic_arg,            # Argumen topic robot_description
        tf_prefix_arg,                          # Argumen TF prefix (multi-robot)
        velodyne_topic_arg,                     # Argumen topic sensor (multi-robot)
        imu_data_topic_arg,                     # Argumen topic IMU
        *camera_args,                           # Semua argumen kamera 6 arah
        check_rviz2_action,                     # Pengecekan dependency RViz2 sebelum launch
        validate_rviz_action,                   # Validasi file RViz config sebelum launch
        check_rviz_config_permission_action,    # Cek permission file RViz config
        check_sensor_topic_conflict_action,     # Cek duplikasi/remap topic kamera
        log_rviz,                               # Logging info config RViz
        robot_state_publisher_node,             # Node robot_state_publisher (standalone)
        rviz_node                               # Node RViz2
    ])

# ===================== CONTOH PENGGUNAAN MULTI-ROBOT (6 KAMERA) =====================
# Gunakan method generate_multi_robot_args dari class RvizDisplayConfig untuk preset argumen:
# Contoh:
# rviz_cfg = RvizDisplayConfig()
# multi_robot_args = rviz_cfg.generate_multi_robot_args('robot1')
# ros2 launch huskybot_description display.launch.py \
#     robot_description_topic:=${multi_robot_args['robot_description_topic']} \
#     tf_prefix:=${multi_robot_args['tf_prefix']} \
#     velodyne_topic:=${multi_robot_args['velodyne_topic']} \
#     imu_data_topic:=${multi_robot_args['imu_data_topic']} \
#     camera_front_image_topic:=${multi_robot_args['camera_front_image_topic']} \
#     camera_front_left_image_topic:=${multi_robot_args['camera_front_left_image_topic']} \
#     camera_left_image_topic:=${multi_robot_args['camera_left_image_topic']} \
#     camera_rear_image_topic:=${multi_robot_args['camera_rear_image_topic']} \
#     camera_rear_right_image_topic:=${multi_robot_args['camera_rear_right_image_topic']} \
#     camera_right_image_topic:=${multi_robot_args['camera_right_image_topic']}