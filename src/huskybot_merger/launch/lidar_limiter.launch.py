from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='huskybot_merger',
            executable='lidar_limiter',
            name='lidar_limiter',
            output='screen',
            parameters=[{'maxrange': 1.0}],  # Bisa diubah sesuai kebutuhan
        )
    ])