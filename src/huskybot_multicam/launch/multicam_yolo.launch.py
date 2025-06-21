from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory('huskybot_multicam'),
        'config',
        'multicam.yaml'
    )
    return LaunchDescription([
        Node(
            package='huskybot_multicam',
            executable='multicam_yolo',
            name='multicam_yolo',
            parameters=[param_path],
            output='screen'
        )
    ])