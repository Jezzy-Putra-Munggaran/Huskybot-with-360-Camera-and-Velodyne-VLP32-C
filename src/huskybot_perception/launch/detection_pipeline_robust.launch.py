#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Velodyne FIRST
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('velodyne'),
                    'launch', 'velodyne-all-nodes-VLP32C-launch.py'
                )
            ])
        ),
        
        # 2. Wait then start cameras with optimized parameters
        TimerAction(
            period=5.0,
            actions=[
                # Camera nodes with individual launch for better control
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_1',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://0',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_front_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_front/image_raw'),
                        ('raw', '/camera_front/image_raw'),
                    ],
                    respawn=False,  # Disable respawn to prevent cascade failures
                ),
                
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_2',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://1',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_right_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_right/image_raw'),
                        ('raw', '/camera_right/image_raw'),
                    ],
                    respawn=False,
                ),
                
                # Add remaining cameras similarly...
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_3',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://2',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_rear_right_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_rear_right/image_raw'),
                        ('raw', '/camera_rear_right/image_raw'),
                    ],
                    respawn=False,
                ),
                
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_4',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://3',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_rear_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_rear/image_raw'),
                        ('raw', '/camera_rear/image_raw'),
                    ],
                    respawn=False,
                ),
                
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_5',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://4',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_left_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_left/image_raw'),
                        ('raw', '/camera_left/image_raw'),
                    ],
                    respawn=False,
                ),
                
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_6',
                    output='screen',
                    parameters=[{
                        'resource': 'csi://5',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'latency': 2000,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_front_left_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_front_left/image_raw'),
                        ('raw', '/camera_front_left/image_raw'),
                    ],
                    respawn=False,
                ),
            ]
        ),
        
        # 3. Wait then start detection with fixed device parameter
        TimerAction(
            period=15.0,  # Wait longer for cameras to stabilize
            actions=[
                Node(
                    package='huskybot_detection',
                    executable='multicam_detection_node',
                    name='multicam_detection',
                    output='screen',
                    parameters=[{
                        'cam_count': 6,
                        'model_path': 'yolo12x.engine',
                        'camera_topics_str': str([
                            '/camera_front/image_raw',
                            '/camera_right/image_raw', 
                            '/camera_rear_right/image_raw',
                            '/camera_rear/image_raw',
                            '/camera_left/image_raw',
                            '/camera_front_left/image_raw'
                        ]),
                        'class_filter_str': "[]",
                        'conf_thres': 0.25,
                        'visualization_enabled': True,
                        'log_to_file': True,
                        'log_level': 'info',
                        'device': 'cuda:0',  # FIXED: Proper device string
                    }],
                    respawn=False,
                )
            ]
        ),
        
        # 4. Wait then start fusion
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion_node',
                    output='screen',
                    parameters=[{
                        'max_laser_distance': 50.0,
                        'min_laser_distance': 0.5,
                        'confidence_threshold': 0.25,
                        'publish_rate': 2.0,
                        'image_width': 1920,
                        'image_height': 1080
                    }],
                    respawn=False,
                )
            ]
        ),
    ])