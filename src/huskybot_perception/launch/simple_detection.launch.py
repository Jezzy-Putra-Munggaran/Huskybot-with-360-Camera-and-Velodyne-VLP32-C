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
    
    # Launch multicam dari huskybot_camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(huskybot_camera_dir, 'launch', 'multicam.launch.py')
        )
    )
    
    # Launch detection
    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(huskybot_detection_dir, 'launch', 'detection.launch.py')
        )
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
        executable='simple_fusion_node',  # Akan kita buat
        name='simple_fusion',
        parameters=[
            {'use_calibration': False},  # Tidak menggunakan calibration
            {'max_laser_distance': 100.0},  # Jarak maksimum untuk LaserScan (meter)
            {'confidence_threshold': 0.25}  # Threshold untuk deteksi
        ],
        output='screen'
    )
    
    # Visualizer node
    visualizer_node = Node(
        package='huskybot_perception',
        executable='fusion_visualizer_node',  # Akan kita buat
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