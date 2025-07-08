#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_perception/setup.py

import os
import sys
from glob import glob
from setuptools import find_packages, setup

package_name = 'huskybot_perception'

# Redirect all informational prints to stderr during package discovery
# Colcon will ignore stderr but capture stdout for parsing
def info_print(message):
    print(message, file=sys.stderr, flush=True)

# Find all launch files
launch_files = glob('launch/*.launch.py')

# Check if we're on a Jetson platform
is_jetson = os.path.exists('/etc/nv_tegra_release')

# Check for README
has_readme = os.path.exists('README.md')

# Check if config directory exists, create if needed (quietly)
config_dir = 'config'
if not os.path.exists(config_dir):
    try:
        os.makedirs(config_dir, exist_ok=True)
        # Create default config file without printing
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
        # Just ignore errors during directory creation
        pass

# Build data_files list without None entries
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add README if it exists
if has_readme:
    data_files.append(('share/' + package_name, ['README.md']))

# Add launch files if they exist
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

# Add config files if they exist
if os.path.exists(config_dir):
    config_files = glob('config/*.yaml')
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
        'python3-csv',             
        'pyyaml',                  
        'tf2_ros',                 
        'visualization_msgs',      
        'message_filters',         
        'python-dateutil',         
    ],
    extras_require={
        'dev': [
            'flake8',              
            'pep257',              
            'pytest',              
            'pytest-cov',          
        ],
        'tensorrt': [
            'torch',               
            'onnx',                
            'onnxruntime-gpu',     
        ],
        'jetson': [
            'jetson-stats',        
            'jtop',                
        ],
    },
    zip_safe=True,                 
    maintainer='Jezzy Putra Munggaran',  
    maintainer_email='mungguran.jezzy.putra@gmail.com',  
    description='Node visualizer dan logger multitask YOLOv12 untuk Huskybot (360° Arducam IMX477, Velodyne VLP32-C). '
                'Visualisasi hasil multitask ke window dan logging ke CSV. '
                'Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real '
                '(Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',
    long_description=open('README.md').read() if has_readme else '',
    long_description_content_type='text/markdown',
    license='Apache-2.0',
    tests_require=['pytest'],
    python_requires='>=3.8',  
    entry_points={
        'console_scripts': [
            'visualizer_node = huskybot_perception.visualizer_node:main',
            'logger_node = huskybot_perception.logger_node:main',
            'fusion_visualizer_node = huskybot_perception.fusion_visualizer_node:main',
            'perception_node = huskybot_perception.visualizer_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)

# Only print debug information when running directly
if __name__ == '__main__':
    info_print(f"\n[INFO] Building {package_name} package:")
    info_print(f"- Detected modules: {', '.join([os.path.basename(f) for f in glob(f'{package_name}/*.py') if not f.endswith('__init__.py')])}")
    info_print(f"- Launch files: {', '.join([os.path.basename(f) for f in launch_files])}")
    info_print(f"- Platform: {'Jetson AGX Orin' if is_jetson else 'Standard/Simulation'}")
    info_print(f"- Dependencies: OpenCV, NumPy, Ultralytics YOLOv12, TF2, cv_bridge, ROS2 Humble")
    info_print(f"- Compatible with: Gazebo simulation and real hardware (Clearpath Husky A200 + Velodyne VLP32-C)")


from setuptools import setup

package_name = 'huskybot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pipeline_100fps.launch.py']),
        ('share/' + package_name + '/rviz', []),  # Create rviz directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='100+ FPS perception pipeline with auto-display',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_rviz_config.py = huskybot_perception.create_rviz_config:create_rviz_config',
        ],
    },
)