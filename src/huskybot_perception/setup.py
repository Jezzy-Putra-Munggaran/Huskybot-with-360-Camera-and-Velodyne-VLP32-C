#!/usr/bin/env python3  # Shebang untuk Python3 (opsional untuk setup.py tapi good practice)
# -*- coding: utf-8 -*-  # Encoding declaration untuk support UTF-8 characters

"""
Setup script for huskybot_perception package.

This package provides visualization and logging for YOLOv12 multitask results
from 360° camera array and Velodyne VLP32-C LiDAR data on Huskybot platform.
Designed for ROS2 Humble, Gazebo simulation, and real hardware deployment.

Author: Jezzy Putra Munggaran
License: Apache-2.0
"""

import os  # Library untuk file dan path operations
import sys  # Library untuk akses system dan environment
from glob import glob  # Library untuk pattern matching file
from setuptools import find_packages, setup  # Tools untuk packaging Python

# Package name (WAJIB sama dengan folder dan package.xml)
package_name = 'huskybot_perception'

try:
    # Cek file yang wajib ada sebelum build
    critical_files = [
        os.path.join(package_name, 'fusion_visualizer_node.py'),
        os.path.join(package_name, 'logger_node.py'),
        os.path.join(package_name, 'visualizer_node.py'),
        'launch/full_perception.launch.py',
        'resource/' + package_name,
        'package.xml'
    ]
    
    # Validasi semua file yang wajib ada
    missing_files = [f for f in critical_files if not os.path.exists(f)]
    if missing_files:
        print(f"[WARNING] Missing critical files: {', '.join(missing_files)}", file=sys.stderr)
        print("[WARNING] Package may not function correctly without these files", file=sys.stderr)
    
    # Cari semua launch files yang ada
    launch_files = glob('launch/*.launch.py')
    if not launch_files:
        print("[WARNING] No launch files found. Package may not be launchable.", file=sys.stderr)
    else:
        print(f"[INFO] Found {len(launch_files)} launch files: {', '.join(os.path.basename(f) for f in launch_files)}")
    
    # Cek apakah kita berada di platform Jetson
    is_jetson = os.path.exists('/etc/nv_tegra_release')
    if is_jetson:
        print("[INFO] Detected Jetson platform - optimizing for Jetson AGX ORIN")
    
    # Cek keberadaan README untuk long_description
    has_readme = os.path.exists('README.md')
    
    # Buat directory config jika belum ada (untuk parameter YAML)
    config_dir = 'config'
    if not os.path.exists(config_dir):
        try:
            os.makedirs(config_dir, exist_ok=True)
            print(f"[INFO] Created config directory: {config_dir}")
            
            # Buat default config file sebagai contoh
            default_config_path = os.path.join(config_dir, 'default_params.yaml')
            with open(default_config_path, 'w') as f:
                f.write("""# Default parameters for huskybot_perception
fusion_visualizer_node:
  ros__parameters:
    show_bounding_box: true
    show_class: true
    show_confidence: true
    show_distance: true
    show_coordinates: true
    fusion_topic: '/fusion/objects3d'
    display_window: true
""")
            print(f"[INFO] Created default configuration file: {default_config_path}")
        except (PermissionError, IOError) as e:
            print(f"[WARNING] Could not create config directory: {e}", file=sys.stderr)
    
    # Setup package dengan dokumentasi yang lengkap
    setup(
        name=package_name,  # Nama package (WAJIB sama dengan folder dan package.xml)
        version='0.1.0',  # Versi package dengan semantic versioning (major.minor.patch)
        packages=find_packages(exclude=['test']),  # Cari otomatis semua Python modules di package, kecuali test
        
        # File-file yang akan diinstall bersama package
        data_files=[
            # File resource index untuk package discovery ROS2 (WAJIB)
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            
            # Package manifest (WAJIB untuk metadata dan dependency ROS2)
            ('share/' + package_name, ['package.xml']),
            
            # Dokumentasi README (best practice)
            ('share/' + package_name, ['README.md'] if has_readme else []),
            
            # Semua launch files di directory launch/ (untuk ros2 launch)
            ('share/' + package_name + '/launch', launch_files),
            
            # Config files jika ada (untuk parameter dan konfigurasi runtime)
            ('share/' + package_name + '/config', glob('config/*.yaml') if os.path.exists(config_dir) else []),
        ],
        
        # Python dependencies runtime (wajib sudah diinstall di sistem atau pip)
        install_requires=[
            'setuptools',              # Required untuk build dan packaging Python
            'rclpy',                   # ROS2 Python client library (WAJIB untuk node Python)
            'sensor_msgs',             # Untuk message types Image, PointCloud2, LaserScan
            'std_msgs',                # Untuk message types standard seperti Header, Bool
            'geometry_msgs',           # Untuk message types koordinat dan transformasi
            'cv_bridge',               # Untuk konversi ROS2 Image <-> OpenCV
            'yolov12_msgs',            # Custom message untuk YOLOv12 results
            'ultralytics',             # Library untuk YOLOv12 inference
            'numpy',                   # Library untuk array dan matrix operations
            'opencv-python',           # Library untuk image processing
            'python3-csv',             # Library untuk CSV logging
            'pyyaml',                  # Library untuk YAML config parsing
            'tf2_ros',                 # Library untuk transform frames
            'visualization_msgs',      # Library untuk rviz visualization markers
            'message_filters',         # Library untuk synchronize messages
            'python-dateutil',         # Library untuk manipulasi timestamp
        ],
        
        # Optional dependencies untuk kasus khusus
        extras_require={
            'dev': [
                'flake8',              # Code linting
                'pep257',              # Docstring style checking
                'pytest',              # Unit testing
                'pytest-cov',          # Test coverage
            ],
            'tensorrt': [
                'torch',               # PyTorch untuk TensorRT conversion
                'onnx',                # ONNX untuk model interchange
                'onnxruntime-gpu',     # ONNX runtime untuk GPU acceleration
            ],
            'jetson': [
                'jetson-stats',        # Untuk monitoring hardware Jetson
                'jtop',                # Untuk monitoring performance Jetson
            ],
        },
        
        # Metadata package
        zip_safe=True,                 # Package aman untuk diinstall sebagai ZIP (standard di ROS2)
        maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sync dengan package.xml)
        maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email contact (sync dengan package.xml)
        
        # Deskripsi package (jelaskan fungsi utama dan hardware compatibility)
        description='Node visualizer dan logger multitask YOLOv12 untuk Huskybot (360° Arducam IMX477, Velodyne VLP32-C). '
                    'Visualisasi hasil multitask ke window dan logging ke CSV. '
                    'Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real '
                    '(Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',
        
        # Long description dari README jika ada
        long_description=open('README.md').read() if has_readme else '',
        long_description_content_type='text/markdown',
        
        # Informasi lisensi (sync dengan package.xml)
        license='Apache-2.0',
        
        # Unit testing requirements
        tests_require=['pytest'],
        
        # Python version constraint
        python_requires='>=3.8',  # ROS2 Humble requires Python 3.8+
        
        # Command-line scripts yang akan diinstall
        entry_points={
            'console_scripts': [
                # Node untuk visualisasi hasil multitask YOLOv12
                'visualizer_node = huskybot_perception.visualizer_node:main',
                
                # Node untuk logging hasil ke CSV
                'logger_node = huskybot_perception.logger_node:main',
                
                # Node untuk visualisasi hasil fusion kamera+LiDAR
                'fusion_visualizer_node = huskybot_perception.fusion_visualizer_node:main',
                
                # Perception node utama (integrasi semua node perception)
                'perception_node = huskybot_perception.visualizer_node:main',  # Alias ke visualizer_node karena belum dibuat file terpisah
            ],
        },
        
        # Package classifiers untuk PyPI
        classifiers=[
            'Development Status :: 4 - Beta',
            'Intended Audience :: Science/Research',
            'License :: OSI Approved :: Apache Software License',
            'Programming Language :: Python',
            'Topic :: Scientific/Engineering :: Artificial Intelligence',
            'Topic :: Software Development :: Libraries :: Python Modules',
        ],
    )

    # Print info build untuk membantu debugging
    print(f"\n[INFO] Building {package_name} package:")
    print(f"- Detected modules: {', '.join([os.path.basename(f) for f in glob(f'{package_name}/*.py') if not f.endswith('__init__.py')])}")
    print(f"- Launch files: {', '.join([os.path.basename(f) for f in launch_files])}")
    print(f"- Platform: {'Jetson AGX Orin' if is_jetson else 'Standard/Simulation'}")
    print(f"- Dependencies: OpenCV, NumPy, Ultralytics YOLOv12, TF2, cv_bridge, ROS2 Humble")
    print(f"- Compatible with: Gazebo simulation and real hardware (Clearpath Husky A200 + Velodyne VLP32-C)")

except Exception as e:
    # Error handling general untuk seluruh proses setup
    print(f"[ERROR] Setup failed: {e}", file=sys.stderr)
    print(f"[ERROR] Details: {type(e).__name__}: {str(e)}", file=sys.stderr)
    print("[ERROR] Please fix the issues and try again", file=sys.stderr)
    # Tidak re-raise exception agar setup masih bisa berjalan dengan warnings