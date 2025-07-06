#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # ULTRA-AGGRESSIVE model prioritization for MAXIMUM FPS
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    # PRIORITIZE NANO MODEL ONLY for 60+ FPS
    model_names = ['yolo11n-seg']  # ONLY nano model for speed
    
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
                    print(f"[INFO] Found ULTRA-FAST model: {os.path.basename(model_file)}")
                    break
            if model_file:
                break
        if model_file:
            break
    
    if not model_file:
        model_file = 'yolo11n-seg.engine'
        print(f"[WARNING] Using default ULTRA-FAST model: {model_file}")

    return LaunchDescription([
        # Launch arguments for MAXIMUM SPEED
        DeclareLaunchArgument(
            'model_path',
            default_value=model_file,
            description='Path to ULTRA-OPTIMIZED YOLOv11 nano model'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='GPU device for inference'
        ),
        
        DeclareLaunchArgument(
            'target_fps',
            default_value='60.0',  # TARGET 60 FPS
            description='ULTRA-HIGH target FPS'
        ),
        
        # ULTRA-OPTIMIZED segmentation node for 60+ FPS
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
                'conf_thres': 0.7,  # HIGHER threshold = much fewer detections = FASTER
                'visualization_enabled': True,
                'publish_rate': LaunchConfiguration('target_fps'),
                
                # ULTRA-AGGRESSIVE Performance for 60+ FPS
                'inference_threads': 6,  # One thread per camera
                'input_size': 256,  # MUCH smaller input = MUCH faster (was 320)
                'half_precision': True,
                'batch_size': 1,
                'max_det': 10,  # VERY few detections = MUCH faster (was 25)
                
                # MINIMAL visualization for speed
                'viz_scale': 0.25,  # Much smaller visualization
                'viz_fps_limit': 30.0,  # Limit viz FPS to save resources
                'show_fps': True,
                'grid_layout': False,  # DISABLE grid for speed
                'skip_masks': True,  # SKIP segmentation masks for speed
                'simple_viz': True,  # Use simple visualization
                'show_confidence': False,  # Disable confidence display
                'show_labels': False,  # Disable labels for speed
                'mask_alpha': 0.1,
                
                # MAXIMUM speed optimizations
                'queue_size': 1,
                'async_publish': True,
                'memory_pool': True,
                'process_every_nth_frame': 1,  # Process EVERY frame for 60 FPS
                
                # Optimized camera topics
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