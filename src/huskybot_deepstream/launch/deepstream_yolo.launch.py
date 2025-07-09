#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument('model_engine', default_value='yolo11x-seg.engine'),
        DeclareLaunchArgument('fps_target', default_value='120'),
        DeclareLaunchArgument('batch_size', default_value='6'),
        
        # DeepStream YOLO Node for 100+ FPS
        Node(
            package='huskybot_deepstream',
            executable='deepstream_yolo_node',
            name='deepstream_yolo',
            output='screen',
            parameters=[{
                'model_engine': LaunchConfiguration('model_engine'),
                'input_width': 640,
                'input_height': 640,
                'batch_size': LaunchConfiguration('batch_size'),
                'fps_target': LaunchConfiguration('fps_target'),
                'device_id': 0
            }],
            respawn=True,
            respawn_delay=2.0
        ),
        
    ])