from setuptools import setup
import os
from glob import glob

package_name = 'huskybot_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jezzy',
    maintainer_email='your@email.com',
    description='Recognition node for Huskybot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Daftarkan semua script yang ingin bisa dijalankan via ros2 run/launch
            'yolov12_ros2_pt.py = huskybot_recognition.scripts.yolov12_ros2_pt:main',
            'yolov12_panorama_inference.py = huskybot_recognition.scripts.yolov12_panorama_inference:main',
            'yolov12_stitcher_node.py = huskybot_recognition.scripts.yolov12_stitcher_node:main',
            # Tambahkan script lain jika perlu
        ],
    },
)