from setuptools import setup

package_name = 'huskybot_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multicamera.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'cv_bridge',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='Publisher kamera Arducam IMX477 asli untuk pipeline Huskybot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multicamera_publisher = huskybot_camera.multicamera_publisher:main',
        ],
    },
)
