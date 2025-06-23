from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path ke package
    perception_dir = get_package_share_directory('huskybot_perception')
    
    # Arguments
    model_type_arg = DeclareLaunchArgument(
        'model_type', 
        default_value='detection',
        description='Model type to use: "detection" or "segmentation"'
    )
    
    def launch_setup(context):
        model_type = LaunchConfiguration('model_type').perform(context)
        
        # Choose launch file based on model_type
        if model_type == 'segmentation':
            launch_file = os.path.join(perception_dir, 'launch', 'simple_segmentation.launch.py')
        else:
            launch_file = os.path.join(perception_dir, 'launch', 'simple_detection.launch.py')
            
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file)
            )
        ]
    
    return LaunchDescription([
        model_type_arg,
        OpaqueFunction(function=launch_setup)
    ])