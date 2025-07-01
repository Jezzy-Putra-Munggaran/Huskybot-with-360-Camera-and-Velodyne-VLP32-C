#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
import os
import glob

package_name = 'huskybot_fusion'

# Data files for ROS2 installation
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch files if they exist
launch_files = glob.glob('launch/*.py')
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

# Add config files if they exist
config_files = glob.glob('config/*.yaml')
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'sensor_msgs',
        'std_msgs', 
        'geometry_msgs',
        'visualization_msgs',
        'yolov12_msgs',
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='Fusion node for Jetson AGX Orin - camera and LiDAR integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_fusion_node = huskybot_fusion.simple_fusion_node:main',
        ],
    },
)