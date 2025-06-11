from setuptools import find_packages, setup

package_name = 'huskybot_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros1_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='ROS1-ROS2 bridge package for Huskybot (cmd_vel, odom, velodyne_points bridging).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tidak ada node Python, hanya launch file
        ],
    },
)