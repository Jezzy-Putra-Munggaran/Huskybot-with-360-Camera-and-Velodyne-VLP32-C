from setuptools import setup

package_name = 'huskybot_deepstream'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/deepstream_yolo.launch.py']),
        ('share/' + package_name + '/config', ['huskybot_deepstream/config/config_infer_yolo11.txt']),
        ('share/' + package_name + '/config', ['huskybot_deepstream/config/labels.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jezzy Putra Munggaran',
    maintainer_email='mungguran.jezzy.putra@gmail.com',
    description='DeepStream YOLO for 100+ FPS inference',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepstream_yolo_node = huskybot_deepstream.deepstream_yolo_node:main',
        ],
    },
)
