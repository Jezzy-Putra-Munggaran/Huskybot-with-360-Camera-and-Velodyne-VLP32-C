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

# Check if we're on a Jetson platform
is_jetson = os.path.exists('/etc/nv_tegra_release')

# Create config directory if needed
config_dir = 'config'
if not os.path.exists(config_dir):
    try:
        os.makedirs(config_dir, exist_ok=True)
        default_config_path = os.path.join(config_dir, 'default_params.yaml')
        with open(default_config_path, 'w') as f:
            f.write("""# Default parameters for huskybot_perception
fusion_visualizer_node:
  ros__parameters:
    show_bounding_box: true
    show_class: true
    show_confidence: true
    show_distance: true
    show_coordinates: true
    fusion_topic: '/fusion/objects3d'
    display_window: true
""")
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

# ✅ FIXED - Single setup() only
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
    description='100+ FPS perception pipeline with auto-display',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_rviz_config = huskybot_perception.create_rviz_config:create_rviz_config',  # ✅ FIXED
            'visualizer_node = huskybot_perception.visualizer_node:main',
            'logger_node = huskybot_perception.logger_node:main',
            'fusion_visualizer_node = huskybot_perception.fusion_visualizer_node:main',
        ],
    },
)