#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    """Launch optimized untuk Jetson AGX Orin dengan CSI cameras."""
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')
    
    # Camera configurations with staggered startup
    camera_configs = [
        ('csi://0', '/camera_front/image_raw', 'camera_front_optical_frame', 0.0),
        ('csi://1', '/camera_right/image_raw', 'camera_right_optical_frame', 2.0),
        ('csi://2', '/camera_rear_right/image_raw', 'camera_rear_right_optical_frame', 4.0),
        ('csi://3', '/camera_rear/image_raw', 'camera_rear_optical_frame', 6.0),
        ('csi://4', '/camera_left/image_raw', 'camera_left_optical_frame', 8.0),
        ('csi://5', '/camera_front_left/image_raw', 'camera_front_left_optical_frame', 10.0),
    ]
    
    nodes = []
    
    # Create staggered camera nodes
    for i, (device, topic, frame_id, delay) in enumerate(camera_configs):
        camera_node = Node(
            package='ros_deep_learning',
            executable='video_source',
            name=f'camera_{i+1}',
            output='both',
            parameters=[{
                'resource': device,
                'width': 1920,
                'height': 1080,
                'framerate': 30.0,
                'codec': 'raw',
                'latency': 200,  # Reduced latency
                'flip': 'rotate-180',
                'frame_id': frame_id,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('video_source/raw', topic),
                ('raw', topic),
            ],
            respawn=True,
            respawn_delay=5.0,  # Wait 5s before respawn
        )
        
        if delay > 0:
            # Stagger camera startup
            nodes.append(TimerAction(
                period=delay,
                actions=[camera_node]
            ))
        else:
            nodes.append(camera_node)
    
    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
    ] + nodes)