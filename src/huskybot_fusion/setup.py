#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_fusion/setup.py

from setuptools import setup
import os

package_name = 'huskybot_fusion'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch files if they exist
if os.path.isdir('launch'):
    data_files.append(('share/' + package_name + '/launch', ['launch/*.py']))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'rclpy>=0.9.0',
        'numpy>=1.20.0',
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