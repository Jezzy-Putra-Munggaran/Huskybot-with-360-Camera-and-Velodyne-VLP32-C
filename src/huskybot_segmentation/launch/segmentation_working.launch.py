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
        
        # WORKING Segmentation node
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
                'conf_thres': 0.25,  # ✅ Lower threshold
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # CRITICAL: Match exactly with exported model
                'inference_threads': 3,  # ✅ Reduced threads
                'input_size': 640,       # ✅ EXACTLY match model
                'half_precision': True,  # ✅ Match export settings
                'batch_size': 1,         # ✅ Match export batch
                'max_det': 15,
                
                # Conservative visualization for stability
                'viz_scale': 0.2,        # ✅ Small for speed
                'viz_fps_limit': 10.0,
                'show_fps': True,
                'grid_layout': True,
                'skip_masks': False,     # ✅ Test with masks
                'simple_viz': False,     # ✅ Full viz for debugging
                'show_confidence': True,
                'show_labels': True,
                'mask_alpha': 0.3,
                
                # Conservative optimizations
                'queue_size': 1,
                'async_publish': False,  # ✅ Sync for debugging
                'memory_pool': True,
                'process_every_nth_frame': 2,  # ✅ Process every 2nd frame
                
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