from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'huskybot_multicam_parallel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'ultralytics',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='Multicamera parallel processing for 100+ FPS YOLO segmentation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_camera_processor = huskybot_multicam_parallel.single_camera_processor:main',
            'multicam_parallel_node = huskybot_multicam_parallel.multicam_parallel_node:main',
        ],
    },
)
