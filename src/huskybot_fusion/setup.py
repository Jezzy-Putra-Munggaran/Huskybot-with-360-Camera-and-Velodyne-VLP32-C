#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_fusion/setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'huskybot_fusion'

# Find necessary files
launch_files = glob('launch/*.py') if os.path.isdir('launch') else []
test_files = glob('test/*.py') if os.path.isdir('test') else []
config_files = glob('config/*.yaml') if os.path.isdir('config') else []
rviz_files = glob('rviz/*.rviz') if os.path.isdir('rviz') else []
readme_path = 'README.md'

# Build data_files list, all items must be valid (no None entries)
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add conditional entries only if they exist
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))
if test_files:
    data_files.append(('share/' + package_name + '/test', test_files))
if os.path.isfile(readme_path):
    data_files.append(('share/' + package_name, [readme_path]))
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))
if rviz_files:
    data_files.append(('share/' + package_name + '/rviz', rviz_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'rclpy>=0.9.0',
        'sensor_msgs',
        'std_msgs',
        'geometry_msgs',
        'visualization_msgs',
        'yolov12_msgs',
        'huskybot_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'numpy>=1.20.0',
        'opencv-python>=4.5.0',
        'pyyaml>=5.3.0',
        'message_filters',
        'scipy>=1.6.0'
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='Node fusion data kamera 360° dan Velodyne VLP-32C untuk deteksi objek 3D. '
                'Mengintegrasikan hasil YOLOv12 dengan point cloud LiDAR untuk navigasi aman. '
                'Siap untuk ROS2 Humble, Gazebo, dan robot Clearpath Husky A200.',
    license='Apache-2.0',
    tests_require=['pytest', 'pytest-cov'],
    entry_points={
        'console_scripts': [
            'simple_fusion_node = huskybot_fusion.simple_fusion_node:main',
        ],
    },
    extras_require={
        'dev': ['flake8', 'pytest', 'pytest-cov', 'pylint', 'ament_pep257', 'pycodestyle'],
        'sim': ['rosgraph_msgs', 'gazebo_msgs'],
        'viz': ['matplotlib', 'open3d'],
    },
)