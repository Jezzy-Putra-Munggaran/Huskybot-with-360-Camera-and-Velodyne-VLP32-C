#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_perception/setup.py

import os
import sys
from glob import glob
from setuptools import find_packages, setup

package_name = 'huskybot_perception'

# Check for README
has_readme = os.path.exists('README.md')

# Find all launch files
launch_files = glob('launch/*.launch.py')

# Create config directory if needed
config_dir = 'config'
rviz_dir = 'rviz'

for directory in [config_dir, rviz_dir]:
    if not os.path.exists(directory):
        try:
            os.makedirs(directory, exist_ok=True)
        except Exception:
            pass

# Build data_files list
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

if has_readme:
    data_files.append(('share/' + package_name, ['README.md']))

if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

if os.path.exists(config_dir):
    config_files = glob('config/*.yaml')
    if config_files:
        data_files.append(('share/' + package_name + '/config', config_files))

if os.path.exists(rviz_dir):
    rviz_files = glob('rviz/*.rviz')
    if rviz_files:
        data_files.append(('share/' + package_name + '/rviz', rviz_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
        'geometry_msgs',
        'cv_bridge',
        'yolov12_msgs',
        'ultralytics',
        'numpy',
        'opencv-python',
        'pyyaml',
        'tf2_ros',
        'visualization_msgs',
        'message_filters',
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='100+ FPS perception pipeline with AUTO-POPUP display',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_rviz_config = huskybot_perception.create_rviz_config:main',
            'auto_grid_viewer = huskybot_perception.auto_grid_viewer:main',
            'visualizer_node = huskybot_perception.visualizer_node:main',
            'logger_node = huskybot_perception.logger_node:main',
            'fusion_visualizer_node = huskybot_perception.fusion_visualizer_node:main',
            'ultra_mega_segmentation_node = huskybot_perception.ultra_mega_segmentation_node:main',
            'simple_working_node = huskybot_perception.simple_working_node:main',
            'ultimate_display_manager = huskybot_perception.ultimate_display_manager:main',
            'ultimate_100fps_node = huskybot_perception.ultimate_100fps_node:main',  # ✅ ADD THIS
            'ultimate_auto_popup_manager = huskybot_perception.ultimate_auto_popup_manager:main',  # ✅ ADD THIS
        ],
    },
)