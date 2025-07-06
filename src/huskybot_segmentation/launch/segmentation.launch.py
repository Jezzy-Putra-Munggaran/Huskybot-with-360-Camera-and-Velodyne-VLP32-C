#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Model detection
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    model_names = ['yolo11n-seg', 'yolo11s-seg', 'yolo11m-seg', 'yolo11x-seg']
    
    search_paths = [
        os.getcwd(),
        os.path.join(os.getcwd(), 'models'),
        os.path.expanduser('~'),
        os.path.join(os.path.expanduser('~'), 'huskybot'),
        os.path.join(os.path.expanduser('~'), 'huskybot', 'models'),
        os.path.join(os.path.expanduser('~'), 'jezzy', 'huskybot'),
        os.path.join(os.path.expanduser('~'), 'jezzy', 'huskybot', 'models')
    ]
    
    for search_path in search_paths:
        for model_name in model_names:
            for ext in model_extensions:
                potential_path = os.path.join(search_path, f"{model_name}{ext}")
                if os.path.exists(potential_path):
                    model_file = potential_path
                    print(f"[INFO] Found optimized segmentation model: {os.path.basename(model_file)}")
                    break
            if model_file:
                break
        if model_file:
            break
    
    if not model_file:
        model_file = 'yolo11x-seg.engine'
        print(f"[WARNING] Using default: {model_file}")

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value=model_file,
            description='Path to optimized YOLOv11 segmentation model'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device for inference'
        ),
        
        DeclareLaunchArgument(
            'target_fps',
            default_value='10.0',
            description='Realistic target FPS'
        ),
        
        # Optimized segmentation node
        Node(
            package='huskybot_segmentation',
            executable='multicam_segmentation_node',
            name='multicam_segmentation',
            output='screen',
            parameters=[{
                # Basic parameters
                'cam_count': 6,
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'conf_thres': 0.3,  # Higher for quality
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # OPTIMIZED Performance parameters
                'inference_threads': 3,  # Reduced for stability
                'input_size': 480,  # Smaller for speed
                'half_precision': True,
                'batch_size': 1,
                'max_det': 50,  # Limit detections
                
                # LARGE visualization parameters
                'viz_scale': 0.8,  # Much larger display
                'viz_fps_limit': 15.0,  # Lower for performance
                'show_fps': True,
                'grid_layout': True,
                'skip_masks': False,  # Keep segmentation
                'simple_viz': False,  # Full quality
                'show_confidence': True,
                'show_labels': True,
                'mask_alpha': 0.4,  # Good visibility
                
                # Speed optimizations
                'queue_size': 1,  # Minimal latency
                'async_publish': True,
                'memory_pool': True,
                'process_every_nth_frame': 3,  # Skip frames for speed
                
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