from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='huskybot_multicam',
            executable='multicam_yolo',
            name='multicam_yolo',
            parameters=['config/multicam.yaml'],
            output='screen'
        )
    ])