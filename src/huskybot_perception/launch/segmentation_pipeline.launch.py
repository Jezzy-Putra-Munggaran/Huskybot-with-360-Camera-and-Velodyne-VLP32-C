#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Camera nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('huskybot_camera'),
                    'launch', 'camera.launch.py'
                )
            ])
        ),
        
        # 2. Segmentation node (YOLOv11-seg)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('huskybot_segmentation'),
                    'launch', 'segmentation.launch.py'
                )
            ])
        ),
        
        # 3. Velodyne nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('velodyne'),
                    'launch', 'velodyne-all-nodes-VLP32C-launch.py'
                )
            ])
        ),
        
        # 4. Fusion node
        Node(
            package='huskybot_fusion',
            executable='simple_fusion_node',
            name='simple_fusion_node',
            output='screen'
        ),
    ])