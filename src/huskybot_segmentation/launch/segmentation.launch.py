#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Model detection - prioritize smaller models for speed
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    # PRIORITIZE SMALLER MODELS FOR HIGHER FPS
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
                    print(f"[INFO] Found HIGH-SPEED segmentation model: {os.path.basename(model_file)}")
                    break
            if model_file:
                break
        if model_file:
            break
    
    if not model_file:
        model_file = 'yolo11n-seg.engine'  # Default to fastest model
        print(f"[WARNING] Using default FASTEST model: {model_file}")

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value=model_file,
            description='Path to OPTIMIZED YOLOv11 segmentation model'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device for inference'
        ),
        
        DeclareLaunchArgument(
            'target_fps',
            default_value='15.0',  # Reduce FPS for testing
            description='HIGH-SPEED target FPS'
        ),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug mode'
        ),
        
        # ULTRA-OPTIMIZED segmentation node for HIGH FPS
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
                'conf_thres': 0.5,  # Higher threshold = fewer detections = faster
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # ULTRA-OPTIMIZED Performance parameters for HIGH FPS
                'inference_threads': 3,  # Reduce threads for debugging
                'input_size': 320,  # Smaller input = much faster
                'half_precision': True,
                'batch_size': 1,
                'max_det': 25,  # Fewer detections = faster processing
                
                # OPTIMIZED visualization parameters
                'viz_scale': 0.4,  # Even smaller for testing
                'viz_fps_limit': 15.0,  # Match target FPS
                'show_fps': True,
                'grid_layout': True,
                'skip_masks': False,  # Keep segmentation but optimize
                'simple_viz': False,
                'show_confidence': True,
                'show_labels': True,
                'mask_alpha': 0.3,  # Lighter masks = faster blending
                
                # AGGRESSIVE speed optimizations
                'queue_size': 1,  # Minimal latency
                'async_publish': True,
                'memory_pool': True,
                'process_every_nth_frame': 3,  # Process every 3rd frame for testing
                
                # DEBUG: Try different topic patterns
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