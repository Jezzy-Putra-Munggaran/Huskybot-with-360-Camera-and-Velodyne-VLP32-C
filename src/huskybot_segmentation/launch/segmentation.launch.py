#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():
    # Deteksi model file
    model_file = None
    model_extensions = ['.engine', '.onnx', '.pt']
    model_names = ['yolo11x-seg', 'yolov8n-seg', 'yolov8s-seg', 'yolov8m-seg', 'yolov8l-seg', 'yolov8x-seg']
    
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
                    print(f"[INFO] File model YOLOv11 ditemukan: {os.path.basename(model_file)}")
                    print(f"[INFO] File model YOLOv11 valid: {model_file}")
                    print(f"[INFO] File model YOLOv12 valid: {model_file}")
                    break
            if model_file:
                break
        if model_file:
            break
    
    if not model_file:
        model_file = 'yolo11x-seg.pt'  # Default fallback
        print(f"[WARNING] Model tidak ditemukan, menggunakan default: {model_file}")
    
    # Print launch info
    print("[INFO] Launching huskybot_segmentation node...")
    
    return LaunchDescription([
        # Launch arguments dengan tipe yang eksplisit
        DeclareLaunchArgument(
            'model_path',
            default_value=model_file,
            description='Path to YOLOv11 segmentation model'
        ),
        
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device for inference (cuda:0/cpu)'
        ),
        
        DeclareLaunchArgument(
            'conf_thres',
            default_value='0.25',
            description='Confidence threshold'
        ),
        
        DeclareLaunchArgument(
            'visualization_enabled',
            default_value='true',
            description='Enable visualization output'
        ),
        
        # Segmentation node dengan parameter yang diperbaiki
        Node(
            package='huskybot_segmentation',
            executable='multicam_segmentation_node',
            name='multicam_segmentation',
            output='screen',
            parameters=[{
                # FIXED: Semua parameters dengan tipe data yang eksplisit
                'cam_count': 6,
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),  # Pastikan ini string
                'conf_thres': 0.25,
                'visualization_enabled': True,
                'publish_rate': 10.0,
                
                # Camera topics sebagai individual parameters
                'camera_topic_0': '/camera_front/image_raw',
                'camera_topic_1': '/camera_front_left/image_raw',
                'camera_topic_2': '/camera_left/image_raw', 
                'camera_topic_3': '/camera_rear/image_raw',
                'camera_topic_4': '/camera_rear_right/image_raw',
                'camera_topic_5': '/camera_right/image_raw',
                
                # Additional parameters
                'image_width': 1920,
                'image_height': 1080,
                'max_detection_distance': 50.0,
                'min_detection_size': 0.01,
                'enable_diagnostics': True,
                'log_level': 'INFO',
            }],
            respawn=True,
            respawn_delay=2.0
        )
    ])