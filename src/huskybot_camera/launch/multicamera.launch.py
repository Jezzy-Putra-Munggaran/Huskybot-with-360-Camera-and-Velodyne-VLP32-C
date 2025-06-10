from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='huskybot_camera',
            executable='multicamera_publisher',
            name='multicamera_publisher',
            output='screen'
        )
    ])