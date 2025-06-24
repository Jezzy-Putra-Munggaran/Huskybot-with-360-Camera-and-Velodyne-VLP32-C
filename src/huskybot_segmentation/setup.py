from setuptools import find_packages, setup  # Import setuptools functions needed for ROS2 Python packages

package_name = 'huskybot_segmentation'  # Define package name for consistent reference throughout setup.py

setup(
    name=package_name,  # Set package name (must match directory name and package.xml)
    version='0.1.0',  # Version number (should be kept in sync with package.xml)
    packages=find_packages(exclude=['test']),  # Automatically find all Python subpackages except test directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # Register this package with ROS2 index
        ('share/' + package_name, ['package.xml']),  # Install package.xml for ROS2 package info
        ('share/' + package_name + '/launch', ['launch/segmentation.launch.py']),  # Install launch files for ros2 launch command
        ('share/' + package_name + '/README.md', ['README.md']),  # Install README for documentation
        # Add any model directories if they exist (commented out until created)
        # ('share/' + package_name + '/models', glob('models/*.engine')),  # YOLOv12 engine files
    ],
    install_requires=[
        'setuptools',  # Required for Python package building
        'rclpy',  # Core ROS2 Python client library
        'sensor_msgs',  # For handling Image messages from cameras
        'cv_bridge',  # For converting between ROS2 images and OpenCV formats
        'yolov12_msgs',  # Custom messages for YOLOv12 results
        'ultralytics',  # YOLOv12 Python API
        'numpy',  # For numerical operations and array handling
        'opencv-python',  # For computer vision operations
        'std_msgs',  # Standard message types (Header, etc.) - added from package.xml
        'std_srvs',  # Standard service types (Trigger, etc.) - added from package.xml
        'diagnostic_msgs',  # For diagnostics publishing - added from package.xml
        'rcl_interfaces',  # For parameter handling - added from package.xml
    ],  # All Python package dependencies (matching exec_depend in package.xml)
    zip_safe=True,  # Package can be safely installed in a zip file
    maintainer='Jezzy Putra Munggaran',  # Package maintainer name (must match package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Maintainer email (must match package.xml)
    description='Node segmentasi YOLOv12 multicamera untuk Huskybot (360° Arducam IMX477). Publish hasil segmentasi ke topic /detection (Yolov12Inference). Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',  # Package description (should match package.xml)
    license='Apache-2.0',  # License type (must match package.xml)
    tests_require=['pytest'],  # Testing framework requirements
    entry_points={
        'console_scripts': [
            'multicam_segmentation_node = huskybot_segmentation.multicam_segmentation_node:main',  # Register main node executable
        ],
    },
    python_requires='>=3.8',  # Specify minimum Python version (for ROS2 Humble)
    classifiers=[  # Add standard Python package classifiers
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)

# ===================== REVIEW & SARAN PENINGKATAN =====================
# - Dependencies sudah diupdate untuk match dengan package.xml (std_msgs, std_srvs, diagnostic_msgs, rcl_interfaces)
# - Sudah menambahkan classifiers untuk meningkatkan metadata package Python
# - Sudah menambahkan python_requires untuk memastikan kompatibilitas dengan ROS2 Humble (Python 3.8+)
# - Sudah siap untuk integrasi dengan pipeline segmentation -> fusion -> visualization
# - Package sudah siap untuk deployment di Jetson Orin dan robot real Husky A200
# - Error handling: Package akan gagal build jika dependencies tidak terpenuhi
# - Saran: Tambahkan directory models/ untuk menyimpan YOLOv12 segmentation models (TensorRT, ONNX, PyTorch)
