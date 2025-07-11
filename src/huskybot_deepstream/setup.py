#!/usr/bin/env python3

import os
import sys
from glob import glob
from setuptools import find_packages, setup

# ✅ FIXED: Correct package name
package_name = 'huskybot_deepstream'

# ✅ FIXED: Proper data files structure
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch files if they exist
if os.path.exists('launch'):
    launch_files = glob('launch/*.launch.py')
    if launch_files:
        data_files.append(('share/' + package_name + '/launch', launch_files))

# Add config files if they exist
if os.path.exists('huskybot_deepstream/config'):
    config_files = glob('huskybot_deepstream/config/*')
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
    description='DeepStream YOLO for 100+ FPS inference with ultra-fast processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ✅ FIXED: Correct executable mapping
            'ultra_maximum_deepstream.py = huskybot_deepstream.ultra_maximum_deepstream:main',
            'deepstream_yolo_node = huskybot_deepstream.deepstream_yolo_node:main',
        ],
    },
)
