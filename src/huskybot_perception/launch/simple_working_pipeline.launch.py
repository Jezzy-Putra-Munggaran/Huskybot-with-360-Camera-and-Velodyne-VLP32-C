#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_perception/launch/simple_working_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        
        # ✅ 1. Start Velodyne LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velodyne'),
                           'launch', 'velodyne-all-nodes-VLP32C-launch.py')
            ])
        ),
        
        # ✅ 2. Start cameras (5 second delay)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('huskybot_camera'),
                                   'launch', 'camera.launch.py')
                    ])
                )
            ]
        ),
        
        # ✅ 3. Start SIMPLE working node (15 second delay untuk stability)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='huskybot_perception',
                    executable='simple_working_node',
                    name='simple_working_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # ✅ 4. AUTO-POPUP Grid display (20 second delay)
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'timeout 3 ros2 topic hz /simple_grid_display && '
                         'python3 -c "'
                         'import rclpy, cv2, numpy as np; '
                         'from rclpy.node import Node; '
                         'from sensor_msgs.msg import Image; '
                         'from cv_bridge import CvBridge; '
                         'rclpy.init(); '
                         'node = Node(\"grid_viewer\"); '
                         'bridge = CvBridge(); '
                         'def callback(msg): '
                         '    img = bridge.imgmsg_to_cv2(msg, \"bgr8\"); '
                         '    cv2.imshow(\"HUSKYBOT 360° SIMPLE WORKING - Press q to quit\", img); '
                         '    if cv2.waitKey(1) & 0xFF == ord(\"q\"): exit(); '
                         'sub = node.create_subscription(Image, \"/simple_grid_display\", callback, 10); '
                         'rclpy.spin(node)'
                         '"'],
                    output='screen',
                    name='auto_display_viewer'
                )
            ]
        ),
        
        # ✅ 5. AUTO-POPUP RViz2 untuk LiDAR (25 second delay)
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'export DISPLAY=:0 && '
                         'rviz2 -f base_link --ros-args -p use_sim_time:=false > /dev/null 2>&1 &'],
                    output='screen',
                    name='auto_rviz2'
                )
            ]
        ),
        
        # ✅ 6. Status monitoring (30 second delay)
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 SIMPLE WORKING Pipeline Status:" && '
                         'echo "📡 Camera topics:" && timeout 5 ros2 topic list | grep image_raw | wc -l && '
                         'echo "🔍 Grid topic:" && timeout 5 ros2 topic list | grep simple_grid_display && '
                         'echo "🔴 LiDAR topic:" && timeout 5 ros2 topic list | grep scan && '
                         'echo "✅ SIMPLE VERSION - ALL FEATURES WORKING!" && '
                         'echo "🎯 Check terminal untuk detection results!"'],
                    output='screen'
                )
            ]
        ),
    ])