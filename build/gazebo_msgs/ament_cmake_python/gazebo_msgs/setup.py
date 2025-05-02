from setuptools import find_packages
from setuptools import setup

setup(
    name='gazebo_msgs',
    version='3.5.2',
    packages=find_packages(
        include=('gazebo_msgs', 'gazebo_msgs.*')),
)
