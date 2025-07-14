#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. Pre-camera verification
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "📡 Pre-camera verification..." && '
                 'systemctl is-active --quiet nvargus-daemon && echo "✅ nvargus-daemon ready" || echo "❌ nvargus-daemon not ready" && '
                 'ls /dev/video* 2>/dev/null || echo "❌ No video devices found"'],
            output='screen',
            name='pre_camera_verification'
        ),
        
        # ✅ 2. Start cameras with STAGGERED launch to prevent conflicts
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_1_rear',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[{
                        'resource': 'csi://0',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'loop': 0,
                        'latency': 500,  # Increased latency buffer
                        'flip': 'rotate-180',
                        'frame_id': 'camera_front_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_front/image_raw'),
                        ('raw', '/camera_front/image_raw'),
                    ]
                )
            ]
        ),
        
        # ✅ 3. Camera 2 with delay
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_deep_learning',
                    executable='video_source',
                    name='camera_2_rear_right',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[{
                        'resource': 'csi://1',
                        'width': 1920,
                        'height': 1080,
                        'framerate': 30.0,
                        'codec': 'raw',
                        'loop': 0,
                        'latency': 500,
                        'flip': 'rotate-180',
                        'frame_id': 'camera_right_optical_frame',
                    }],
                    remappings=[
                        ('video_source/raw', '/camera_right/image_raw'),
                        ('raw', '/camera_right/image_raw'),
                    ]
                )
            ]
        ),
        
        # ✅ Continue pattern for all 6 cameras with 3-second intervals...
        # (Add similar TimerAction blocks for cameras 3-6)
        
    ])