from setuptools import setup
import os
from glob import glob

package_name = 'huskybot_single_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
    description='Single camera YOLO segmentation node for performance testing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_camera_ultimate_node = huskybot_single_camera.single_camera_ultimate_node:main',
        ],
    },
)
