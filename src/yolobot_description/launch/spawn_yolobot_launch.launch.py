import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = os.path.join(get_package_share_directory('yolobot_description'), 'robot/', 'yolobot.urdf')
    assert os.path.exists(urdf), "The yolobot.urdf doesnt exist in "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'yolobot',
            '-file', urdf,
            '-x', '0',
            '-y', '0',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # Pastikan robot selalu spawn di tengah (x=0, y=0, z=0.05)
        spawn_entity,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
    ])