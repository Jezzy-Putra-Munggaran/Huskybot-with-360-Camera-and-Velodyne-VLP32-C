from setuptools import find_packages
from setuptools import setup

setup(
    name='yolov12_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('yolov12_msgs', 'yolov12_msgs.*')),
)
