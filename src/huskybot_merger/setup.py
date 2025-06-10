from setuptools import find_packages, setup

package_name = 'huskybot_merger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/odom_tf_rebroadcaster.launch.py',
            'launch/lidar_limiter.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'nav_msgs',
        'sensor_msgs',
        'geometry_msgs',
        'tf2_ros',
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='Node merger/relay TF dan LaserScan untuk integrasi multi-device Huskybot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_rebroadcaster = huskybot_merger.odom_tf_rebroadcaster:main',
            'lidar_limiter = huskybot_merger.lidar_limiter:main',
        ],
    },
)