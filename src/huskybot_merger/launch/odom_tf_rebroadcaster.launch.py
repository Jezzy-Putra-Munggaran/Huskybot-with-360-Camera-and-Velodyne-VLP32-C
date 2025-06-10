from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='huskybot_merger',
            executable='odom_tf_rebroadcaster',
            name='odom_tf_rebroadcaster',
            output='screen',
            parameters=[{'maxrange': 1.0}],  # Bisa diubah sesuai kebutuhan
        )
    ])