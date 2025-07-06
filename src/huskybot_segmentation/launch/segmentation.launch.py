#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Find model with debugging
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    model_names = ['yolo11n-seg', 'yolov8n-seg']
    
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
                    print(f"[INFO] Found DEBUG model: {os.path.basename(model_file)}")
                    break
            if model_file:
                break
        if model_file:
            break
    
    if not model_file:
        model_file = 'yolo11n-seg.engine'
        print(f"[WARNING] Using default DEBUG model: {model_file}")

    return LaunchDescription([
        # Launch arguments for DEBUGGING
        DeclareLaunchArgument(
            'model_path',
            default_value=model_file,
            description='Path to YOLOv11 model for debugging'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='GPU device for inference'
        ),
        
        DeclareLaunchArgument(
            'target_fps',
            default_value='30.0',  # Lower target for debugging
            description='Target FPS for debugging'
        ),
        
        # DEBUG segmentation node
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
                'conf_thres': 0.5,  # Lower for more detections
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # DEBUG Performance parameters
                'inference_threads': 6,
                'input_size': 320,  # Slightly larger for stability
                'half_precision': True,
                'batch_size': 1,
                'max_det': 25,  # More detections for testing
                
                # DEBUG visualization - FULL features
                'viz_scale': 0.5,  # Larger for visibility
                'viz_fps_limit': 30.0,
                'show_fps': True,
                'grid_layout': True,  # ENABLE for debugging
                'skip_masks': False,  # ENABLE masks
                'simple_viz': False,  # Full visualization
                'show_confidence': True,  # Show confidence
                'show_labels': True,  # Show labels
                'mask_alpha': 0.3,
                
                # DEBUG optimizations
                'queue_size': 2,  # Larger queue
                'async_publish': False,  # Synchronous for debugging
                'memory_pool': True,
                'process_every_nth_frame': 1,  # Process all frames
                
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