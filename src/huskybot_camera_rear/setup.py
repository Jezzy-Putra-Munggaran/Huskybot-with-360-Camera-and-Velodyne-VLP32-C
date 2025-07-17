from setuptools import setup
import os
from glob import glob

package_name = 'huskybot_camera_rear'

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
    description='Camera rear ultimate node - 100% mirip simple_ultimate_working_node.py',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_rear_ultimate_node = huskybot_camera_rear.camera_rear_ultimate_node:main',
        ],
    },
)
