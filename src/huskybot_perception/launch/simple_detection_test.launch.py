#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Start detection node only
        Node(
            package='huskybot_detection',
            executable='multicam_detection_node',
            name='multicam_detection',
            output='screen',
            parameters=[{
                'cam_count': 6,
                'model_path': 'yolo12x.engine',
                'device': 'cuda:0',
                'conf_thres': 0.25,
                'visualization_enabled': True,
            }],
        ),
        
        # Start fusion node after 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='huskybot_fusion',
                    executable='simple_fusion_node',
                    name='simple_fusion',
                    output='screen',
                    parameters=[{
                        'max_laser_distance': 50.0,
                        'min_laser_distance': 0.5,
                        'confidence_threshold': 0.25,
                        'publish_rate': 5.0,
                    }],
                )
            ]
        ),
    ])