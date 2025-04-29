from setuptools import setup
import os
from glob import glob

package_name = 'huskybot_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Agar msg bisa ditemukan dan dibuild
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jezzy',
    maintainer_email='your@email.com',
    description='Fusion node for camera 360 and Velodyne VLP-32C',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node.py = huskybot_fusion.fusion_node:main',
        ],
    },
    # Tambahkan ini agar ROS2 mengenali message interface
    package_data={
        '': ['msg/*.msg'],
    },
)