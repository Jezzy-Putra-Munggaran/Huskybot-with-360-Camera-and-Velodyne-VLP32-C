#!/usr/bin/python3  
# -*- coding: utf-8 -*- 

import os
import sys
import shutil
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, LogWarn, LogError, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ===================== OOP CONFIG WRAPPER =====================
class RvizDisplayConfig:
    def __init__(self, package_name='huskybot_description', default_rviz='huskybot.rviz'):
        self.package_name = package_name
        self.default_rviz = default_rviz
        self.sensor_topics = {
            'velodyne_points': '/velodyne_points',
            'imu_data': '/imu/data'
        }
        self.camera_names = [
            'front', 'front_left', 'left', 'rear', 'rear_right', 'right'
        ]

    def get_default_rviz_config(self):
        return os.path.join(
            get_package_share_directory(self.package_name),
            'rviz',
            self.default_rviz
        )

    def get_sensor_topic(self, key):
        return self.sensor_topics.get(key, '')

    def get_camera_topic(self, camera_name, robot_ns=''):
        ns = f"/{robot_ns.strip('/')}" if robot_ns else ""
        return f"{ns}/camera_{camera_name}/image_raw"

    def generate_multi_robot_args(self, robot_ns):
        ns = robot_ns.strip('/')
        args = {
            'robot_description_topic': f'/{ns}/robot_description',
            'tf_prefix': f'{ns}/',
            'velodyne_topic': f'/{ns}/velodyne_points',
            'imu_data_topic': f'/{ns}/imu/data'
        }
        for cam in self.camera_names:
            args[f'camera_{cam}_image_topic'] = f'/{ns}/camera_{cam}/image_raw'
        return args

# ===================== ERROR HANDLING & LOGGER =====================
def check_rviz2_dependency(context, *args, **kwargs):
    if shutil.which('rviz2') is None:
        print("[ERROR] Dependency 'rviz2' tidak ditemukan di PATH. Install dengan: sudo apt install ros-humble-rviz2", flush=True)
        LogError(msg="Dependency 'rviz2' tidak ditemukan di PATH.")
        sys.exit(2)
    else:
        print("[INFO] Dependency 'rviz2' ditemukan di PATH.")
        LogInfo(msg="Dependency 'rviz2' ditemukan di PATH.")
    return []

def validate_rviz_config(context, *args, **kwargs):
    rvizconfig = LaunchConfiguration('rvizconfig').perform(context)
    if not os.path.isfile(rvizconfig):
        print(f"[WARNING] File RViz config tidak ditemukan: {rvizconfig}. RViz2 akan jalan tanpa konfigurasi.", flush=True)
        LogWarn(msg=f"File RViz config tidak ditemukan: {rvizconfig}. RViz2 akan jalan tanpa konfigurasi.")
    else:
        print(f"[INFO] File RViz config ditemukan: {rvizconfig}")
        LogInfo(msg=f"File RViz config ditemukan: {rvizconfig}")
    return []

def check_rviz_config_permission(context, *args, **kwargs):
    rvizconfig = LaunchConfiguration('rvizconfig').perform(context)
    if os.path.isfile(rvizconfig) and not os.access(rvizconfig, os.R_OK):
        print(f"[ERROR] File RViz config tidak bisa dibaca (permission denied): {rvizconfig}", flush=True)
        LogError(msg=f"File RViz config tidak bisa dibaca (permission denied): {rvizconfig}")
        sys.exit(3)
    return []

def check_sensor_topic_conflict(context, *args, **kwargs):
    camera_topics = set()
    for cam in RvizDisplayConfig().camera_names:
        topic = LaunchConfiguration(f'camera_{cam}_image_topic').perform(context)
        if topic in camera_topics:
            print(f"[ERROR] Topic kamera duplikat/remap: {topic}", flush=True)
            LogError(msg=f"Topic kamera duplikat/remap: {topic}")
            sys.exit(4)
        camera_topics.add(topic)
    return []

def log_to_file(msg):
    log_file_path = os.path.expanduser("~/huskybot_rviz_display.log")
    try:
        with open(log_file_path, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file_path} ({e})", file=sys.stderr)

# ===================== LAUNCH DESCRIPTION =====================
def generate_launch_description():
    try:
        rviz_cfg = RvizDisplayConfig()

        rviz_config_arg = DeclareLaunchArgument(
            'rvizconfig',
            default_value=rviz_cfg.get_default_rviz_config(),
            description='Path ke file konfigurasi RViz2'
        )
        robot_description_topic_arg = DeclareLaunchArgument(
            'robot_description_topic',
            default_value='robot_description',
            description='Topic robot_description yang akan di-subscribe RViz2'
        )
        tf_prefix_arg = DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='TF Prefix untuk multi-robot (kosongkan jika single robot)'
        )
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

        camera_args = []
        for cam in rviz_cfg.camera_names:
            camera_args.append(
                DeclareLaunchArgument(
                    f'camera_{cam}_image_topic',
                    default_value=rviz_cfg.get_camera_topic(cam),
                    description=f'Topic kamera {cam} (untuk multi-robot bisa di-remap)'
                )
            )

        # Error handling actions
        check_rviz2_action = OpaqueFunction(function=check_rviz2_dependency)
        validate_rviz_action = OpaqueFunction(function=validate_rviz_config)
        check_rviz_config_permission_action = OpaqueFunction(function=check_rviz_config_permission)
        check_sensor_topic_conflict_action = OpaqueFunction(function=check_sensor_topic_conflict)

        # Logging info config RViz ke terminal dan file
        log_rviz = LogInfo(msg=["Menjalankan RViz2 dengan config: ", LaunchConfiguration('rvizconfig')])
        log_to_file(f"Menjalankan RViz2 dengan config: {rviz_cfg.get_default_rviz_config()}")

        # Node robot_state_publisher agar RViz2 bisa standalone
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': '',
                'use_sim_time': True
            }],
            remappings=[('/robot_description', LaunchConfiguration('robot_description_topic'))]
        )

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
                *camera_remaps
            ],
            parameters=[{'tf_prefix': LaunchConfiguration('tf_prefix')}],
        )

        return LaunchDescription([
            rviz_config_arg,
            robot_description_topic_arg,
            tf_prefix_arg,
            velodyne_topic_arg,
            imu_data_topic_arg,
            *camera_args,
            check_rviz2_action,
            validate_rviz_action,
            check_rviz_config_permission_action,
            check_sensor_topic_conflict_action,
            log_rviz,
            robot_state_publisher_node,
            rviz_node
        ])
    except Exception as e:
        print(f"[FATAL] Exception saat generate_launch_description: {e}", file=sys.stderr)
        LogError(msg=f"Exception saat generate_launch_description: {e}")
        log_to_file(f"[FATAL] Exception saat generate_launch_description: {e}")
        sys.exit(99)

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