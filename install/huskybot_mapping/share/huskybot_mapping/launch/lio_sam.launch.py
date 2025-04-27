from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Velodyne Driver
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=['/home/jezzy/huskybot/src/velodyne/velodyne_driver/config/VLP32C-velodyne_driver_node-params.yaml']
        ),
        # Velodyne Pointcloud
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=['/home/jezzy/huskybot/src/velodyne/velodyne_pointcloud/config/VLP32C-velodyne_transform_node-params.yaml']
        ),
        # (Optional) IMU Node - sesuaikan dengan hardware Anda
        # Node(
        #     package='your_imu_driver_package',
        #     executable='your_imu_node',
        #     name='imu_node',
        #     output='screen',
        #     parameters=[...]
        # ),
        # LIO-SAM
        Node(
            package='lio_sam',
            executable='lio_sam_node',
            name='lio_sam',
            output='screen',
            parameters=['/home/jezzy/huskybot/src/LIO-SAM/config/params.yaml'],
            remappings=[
                ('/points_raw', '/velodyne_points'),
                ('/imu/data', '/imu/data')
            ]
        )
    ])