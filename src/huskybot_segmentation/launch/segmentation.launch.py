#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_segmentation/launch/segmentation.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Model detection
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    model_names = ['yolo11n-seg', 'yolo11s-seg', 'yolo11m-seg']  # Prefer smaller/faster models
    
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
                    print(f"[INFO] Found ultra-fast model: {os.path.basename(model_file)}")
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
            description='Path to YOLOv11 segmentation model'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device for inference'
        ),
        
        DeclareLaunchArgument(
            'target_fps',
            default_value='60.0',
            description='Target FPS (60+)'
        ),
        
        # Ultra high-performance segmentation node
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
                'conf_thres': 0.5,  # Higher threshold for speed
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # Ultra performance optimizations
                'inference_threads': 6,  # One per camera
                'input_size': 320,  # Much smaller for ultra speed
                'half_precision': True,
                'batch_size': 1,
                'max_det': 50,  # Aggressively limit detections
                
                # Ultra visualization optimizations
                'viz_scale': 0.25,  # Very small visualization
                'viz_fps_limit': 30.0,  # Limit viz refresh
                'show_fps': True,
                'grid_layout': True,
                'skip_masks': True,  # Skip mask processing
                'simple_viz': True,  # Ultra simple visualization
                
                # Performance tuning
                'queue_size': 1,  # Minimal latency
                'async_publish': True,  # Async publishing
                'memory_pool': True,  # Pre-allocated memory
                
                # Camera topics
                'camera_topic_0': '/camera_front/image_raw',
                'camera_topic_1': '/camera_front_left/image_raw',
                'camera_topic_2': '/camera_left/image_raw', 
                'camera_topic_3': '/camera_rear/image_raw',
                'camera_topic_4': '/camera_rear_right/image_raw',
                'camera_topic_5': '/camera_right/image_raw',
            }],
            respawn=True,
            respawn_delay=1.0
        )
    ])