#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('model_path', default_value='yolo11x-seg.engine'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('target_fps', default_value='15.0'),
        
        # OPTIMIZED Segmentation node
        Node(
            package='huskybot_segmentation',
            executable='multicam_segmentation_node',
            name='multicam_segmentation',
            output='screen',
            parameters=[{
                # Core parameters
                'cam_count': 6,
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'conf_thres': 0.4,
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # FIXED: Match model input size
                'inference_threads': 6,
                'input_size': 640,  # ✅ MATCH MODEL SIZE
                'half_precision': True,
                'batch_size': 1,
                'max_det': 10,
                
                # Visualization optimized for 640x640 model
                'viz_scale': 0.3,  # ⚡ Larger scale for 640 model
                'viz_fps_limit': 10.0,
                'show_fps': True,
                'grid_layout': True,
                'skip_masks': True,  # ⚡ Skip masks for speed
                'simple_viz': True,
                'show_confidence': False,
                'show_labels': False,
                'mask_alpha': 0.3,
                
                # Speed optimizations
                'queue_size': 1,
                'async_publish': True,
                'memory_pool': True,
                'process_every_nth_frame': 3,  # ⚡ Process every 3rd frame
                
                # Camera topics
                'camera_topic_0': '/camera_front/image_raw',
                'camera_topic_1': '/camera_front_left/image_raw', 
                'camera_topic_2': '/camera_left/image_raw',
                'camera_topic_3': '/camera_rear/image_raw',
                'camera_topic_4': '/camera_rear_right/image_raw',
                'camera_topic_5': '/camera_right/image_raw',
            }],
            respawn=True,
            respawn_delay=2.0
        )
    ])