from setuptools import find_packages, setup  # Import fungsi setup dan find_packages untuk build/install Python package

package_name = 'huskybot_calibration'  # Nama package, harus sama dengan folder utama dan package.xml

setup(
    name=package_name,  # Nama package (wajib, harus sama dengan folder dan package.xml)
    version='0.1.0',  # Versi package, update jika ada perubahan besar (sinkron dengan package.xml)
    packages=find_packages(exclude=['test']),  # Temukan semua sub-package Python, kecuali folder test/
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Resource index agar dikenali ROS2/colcon
        ('share/' + package_name, ['package.xml']),  # Install package.xml ke share/package_name (wajib agar metadata ROS2 terbaca)
        ('share/' + package_name + '/launch', ['launch/calibrate_lidar_camera.launch.py', 'launch/sync_sensor_time.launch.py']),  # Install semua launch file ke share/package_name/launch/
        ('share/' + package_name + '/config', ['config/extrinsic_lidar_to_camera.yaml']),  # Install file kalibrasi YAML ke share/package_name/config/
        ('share/' + package_name + '/scripts', ['scripts/record_calib_data.py']),  # Install script CLI ke share/package_name/scripts/
        ('share/' + package_name + '/test', [
            'test/test_calibration.py',
            'test/test_utils.py',
            'test/test_copyright.py',
            'test/test_flake8.py',
            'test/test_pep257.py'
        ]),  # Install semua test file ke share/package_name/test/
        ('share/' + package_name, ['README.md']),  # Install README untuk dokumentasi
    ],
    install_requires=[
        'setuptools',  # Dependency utama Python package
        'rclpy',  # Core ROS2 Python API
        'sensor_msgs',  # Message sensor (Image, PointCloud2, dsb)
        'std_msgs',  # Message standar ROS2
        'cv_bridge',  # Konversi ROS Image <-> OpenCV
        'numpy',  # Komputasi numerik, array, matrix
        'pyyaml',  # Baca/tulis file YAML kalibrasi
        'opencv-python',  # OpenCV untuk deteksi pattern, image processing
        'message_filters',  # Sinkronisasi waktu antar sensor
        'geometry_msgs',  # Untuk publish transformasi, dsb
        'tf2_ros',  # Untuk publish/lookup transformasi antar frame
        'pcl-msgs',  # Untuk PointCloud2 jika perlu
        'ament_index_python',  # Untuk lookup path package lain
        'matplotlib',  # (Opsional) Untuk visualisasi hasil kalibrasi
    ],  # Semua dependency Python utama (pastikan sudah di package.xml juga)
    zip_safe=True,  # Package aman untuk di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sinkron dengan package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (sinkron dengan package.xml)
    description='Kalibrasi otomatis kamera-LiDAR dan sinkronisasi waktu sensor untuk Huskybot (360° Arducam + Velodyne VLP-32C). Siap untuk ROS2 Humble, Gazebo, dan robot Husky A200 asli.',  # Deskripsi singkat package
    license='Apache-2.0',  # Lisensi package (harus sama dengan package.xml)
    tests_require=['pytest'],  # Dependency untuk unit test Python
    entry_points={
        'console_scripts': [
            # Daftarkan semua script yang ingin bisa dijalankan via ros2 run/launch
            'calibrate_lidar_camera.py = huskybot_calibration.calibrate_lidar_camera:main',  # Node utama kalibrasi extrinsic
            'sync_sensor_time.py = huskybot_calibration.sync_sensor_time:main',  # Node sinkronisasi waktu sensor
            'record_calib_data.py = scripts.record_calib_data:main',  # Script CLI untuk rekam data kalibrasi
        ],
    },
)