from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path ke package
    huskybot_camera_dir = get_package_share_directory('huskybot_camera')
    huskybot_detection_dir = get_package_share_directory('huskybot_detection')
    velodyne_dir = get_package_share_directory('velodyne')
    
    # Launch camera dari huskybot_camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(huskybot_camera_dir, 'launch', 'camera.launch.py')
        )
    )
    
    # Launch detection dengan model yang benar
    detection_launch = Node(
        package='huskybot_detection',
        executable='multicam_detection_node',
        name='multicam_detection',
        parameters=[
            {'model_path': os.path.join(huskybot_detection_dir, 'models', 'yolo12x.engine')},
            {'cam_count': 6},
            {'confidence_threshold': 0.5},
            {'publish_topic': '/detection'},  # Use same topic for fusion
            {'enable_visualization': True}
        ],
        output='screen'
    )
    
    # Launch velodyne
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_dir, 'launch', 'velodyne-all-nodes-VLP32C.launch')
        )
    )
    
    # Node fusion yang sederhana (tanpa calibration)
    fusion_node = Node(
        package='huskybot_fusion',
        executable='simple_fusion_node',
        name='simple_fusion',
        parameters=[
            {'use_calibration': False},
            {'max_laser_distance': 100.0},
            {'confidence_threshold': 0.25},
            {'detection_topics': ['/detection']},
            {'laserscan_topic': '/scan'},
            {'pointcloud_topic': '/velodyne_points'},
            {'output_topic': '/fusion/objects3d'}
        ],
        output='screen'
    )
    
    # Visualizer node
    visualizer_node = Node(
        package='huskybot_perception',
        executable='fusion_visualizer_node',
        name='fusion_visualizer',
        parameters=[
            {'show_bounding_box': True},
            {'show_distance': True},
            {'show_coordinates': True},
            {'show_class': True},
            {'show_confidence': True}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_launch,
        detection_launch,
        velodyne_launch,
        fusion_node,
        visualizer_node
    ])