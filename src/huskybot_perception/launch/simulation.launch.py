import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import xacro
import yaml

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_type = LaunchConfiguration('model_type', default='detection')
    
    # Get paths to packages
    huskybot_description_path = get_package_share_directory('huskybot_description')
    huskybot_gazebo_path = get_package_share_directory('huskybot_gazebo')
    velodyne_path = get_package_share_directory('velodyne')
    huskybot_perception_path = get_package_share_directory('huskybot_perception')
    huskybot_camera_path = get_package_share_directory('huskybot_camera')
    
    # Use xacro to process the robot description file
    xacro_file = os.path.join(huskybot_description_path, 'robot', 'huskybot.urdf.xacro')
    
    # Check if file exists
    if not os.path.exists(xacro_file):
        print(f"ERROR: URDF file not found: {xacro_file}")
        # Create a simpler URDF for testing
        simple_urdf = """<?xml version="1.0"?>
        <robot name="huskybot">
            <link name="base_link">
                <visual>
                    <geometry>
                        <box size="1.0 0.5 0.2"/>
                    </geometry>
                </visual>
                <collision>
                    <geometry>
                        <box size="1.0 0.5 0.2"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="50.0"/>
                    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
                </inertial>
            </link>
        </robot>
        """
        xacro_file = '/tmp/simple_huskybot.urdf'
        with open(xacro_file, 'w') as f:
            f.write(simple_urdf)
    
    try:
        robot_description_config = xacro.process_file(xacro_file)
        robot_description = robot_description_config.toxml()
    except Exception as e:
        print(f"ERROR: Failed to process XACRO file: {e}")
        robot_description = """<?xml version="1.0"?>
        <robot name="huskybot">
            <link name="base_link"/>
        </robot>
        """
    
    # Check for controller config
    controllers_file = os.path.join(huskybot_description_path, 'config', 'huskybot_controllers.yaml')
    if not os.path.exists(controllers_file):
        print(f"WARNING: Controllers file not found: {controllers_file}")
        # Create a minimal controller config
        controllers_config = {
            'controller_manager': {
                'ros__parameters': {
                    'update_rate': 100,
                    'use_sim_time': True,
                    'joint_state_broadcaster': {
                        'type': 'joint_state_broadcaster/JointStateBroadcaster'
                    },
                    'velocity_controller': {
                        'type': 'diff_drive_controller/DiffDriveController'
                    }
                }
            },
            'velocity_controller': {
                'ros__parameters': {
                    'left_wheel_names': ['front_left_wheel', 'rear_left_wheel'],
                    'right_wheel_names': ['front_right_wheel', 'rear_right_wheel'],
                    'wheel_separation': 0.5,
                    'wheel_radius': 0.1
                }
            }
        }
        controllers_file = '/tmp/huskybot_controllers.yaml'
        with open(controllers_file, 'w') as f:
            yaml.dump(controllers_config, f)
    
    # Define launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'huskybot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': use_sim_time}]
    )
    
    # Controller spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'velocity_controller'],
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(huskybot_description_path, 'rviz', 'huskybot.rviz')
    if not os.path.exists(rviz_config):
        print(f"WARNING: RViz config file not found: {rviz_config}")
        rviz_config = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(huskybot_camera_path, 'launch', 'camera.launch.py')
        ])
    )
    
    # Perception launch based on model_type
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(huskybot_perception_path, 'launch', 'main.launch.py')
        ]),
        launch_arguments={'model_type': model_type}.items()
    )
    
    # Delay some nodes to ensure Gazebo is fully loaded
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])
    delayed_controllers = TimerAction(period=10.0, actions=[controller_spawner])
    delayed_perception = TimerAction(period=15.0, actions=[perception_launch])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_type',
            default_value='detection',
            description='Model type: detection or segmentation'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Start in order
        gazebo_launch,
        robot_state_publisher,
        delayed_spawn,
        delayed_controllers,
        rviz_node,
        delayed_perception
    ])