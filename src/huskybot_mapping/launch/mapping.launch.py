from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', '/home/jezzy/huskybot/src/huskybot_mapping/config',
                '-configuration_basename', 'velodyne_3d.lua'
            ],
            remappings=[
                ('/points2', '/velodyne_points')  # Pastikan topic ini sesuai output dari Velodyne Anda
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }]
        )
    ])