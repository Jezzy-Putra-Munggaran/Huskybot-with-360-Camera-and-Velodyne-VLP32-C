#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('model_path', default_value='yolo11x-seg.engine'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('target_fps', default_value='30.0'),
        
        # Main segmentation node (existing)
        Node(
            package='huskybot_segmentation',
            executable='multicam_segmentation_node',
            name='multicam_segmentation',
            output='screen',
            parameters=[{
                'cam_count': 6,
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'conf_thres': 0.25,
                'visualization_enabled': True,
                'grid_layout': True,
                'skip_masks': True,  # Fast detection mode
                'viz_scale': 0.5,
                'inference_threads': 3,
                'input_size': 320,
                'async_publish': True,
            }]
        ),
        
        # DeepStream visualizer
        Node(
            package='huskybot_segmentation',
            executable='deepstream_visualizer',
            name='deepstream_visualizer',
            output='screen'
        ),
        
    ])