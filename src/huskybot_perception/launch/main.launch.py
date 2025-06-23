from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directory
    huskybot_perception_dir = get_package_share_directory('huskybot_perception')
    
    # Launch arguments
    model_type = LaunchConfiguration('model_type', default='detection')
    
    # Model type argument
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='detection',
        description='Type of model to use: detection or segmentation'
    )
    
    # Select launch file based on model_type
    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(huskybot_perception_dir, 'launch', 'simple_detection.launch.py')
        ]),
        condition=LaunchConfiguration('model_type').perform(context=None) == 'detection'
    )
    
    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(huskybot_perception_dir, 'launch', 'simple_segmentation.launch.py')
        ]),
        condition=LaunchConfiguration('model_type').perform(context=None) == 'segmentation'
    )
    
    return LaunchDescription([
        model_type_arg,
        detection_launch,
        segmentation_launch
    ])