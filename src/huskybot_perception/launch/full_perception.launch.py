from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='huskybot_detection', executable='multicam_detection_node', output='screen'),
        Node(package='huskybot_segmentation', executable='multicam_segmentation_node', output='screen'),
        Node(package='huskybot_classification', executable='multicam_classification_node', output='screen'),
        Node(package='huskybot_obb', executable='multicam_obb_node', output='screen'),
        Node(package='huskybot_tracking', executable='tracking_fusion_node', output='screen'),
        Node(package='huskybot_fusion', executable='fusion_node', output='screen'),
        Node(package='huskybot_perception', executable='visualizer_node', output='screen'),
        Node(package='huskybot_perception', executable='logger_node', output='screen'),
    ])