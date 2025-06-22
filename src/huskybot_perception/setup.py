from setuptools import find_packages, setup  # Import fungsi setup dan find_packages dari setuptools (wajib untuk ROS2 Python package)

package_name = 'huskybot_perception'  # Nama package, harus sama dengan folder utama dan package.xml

setup(
    name=package_name,  # Nama package (wajib, harus sama dengan folder dan package.xml)
    version='0.1.0',  # Versi package (sinkron dengan package.xml, update jika release baru)
    packages=find_packages(exclude=['test']),  # Cari semua subpackage Python, kecuali folder test/ (agar test tidak diinstall sebagai modul)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # File resource agar ROS2 bisa menemukan package ini
        ('share/' + package_name, ['package.xml']),  # Install package.xml ke share agar metadata ROS2 bisa ditemukan
        ('share/' + package_name + '/README.md', ['README.md']),  # Install README.md ke share/package/ (dokumentasi ikut terinstall)
        ('share/' + package_name + '/launch', ['launch/full_perception.launch.py']),  # Install launch file ke share/package/launch (wajib agar bisa ros2 launch)
    ],
    install_requires=[
        'setuptools',  # Dependency build Python
        'rclpy',  # Dependency utama node ROS2 Python
        'sensor_msgs',  # Untuk pesan Image, PointCloud2
        'cv_bridge',  # Untuk konversi ROS <-> OpenCV
        'yolov12_msgs',  # Untuk custom message YOLOv12
        'ultralytics',  # Untuk YOLOv12 Python
        'numpy',  # Untuk operasi array/matrix
        'opencv-python',  # Untuk image processing
        'python3-csv',  # Untuk logging ke CSV (logger_node)
    ],  # Semua dependency Python utama (pastikan sudah di package.xml juga)
    zip_safe=True,  # Package aman untuk di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sinkron dengan package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (sinkron dengan package.xml)
    description='Node visualizer dan logger multitask YOLOv12 untuk Huskybot (360° Arducam IMX477, Velodyne VLP32-C). Visualisasi hasil multitask ke window dan logging ke CSV. Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',  # Deskripsi singkat package (sinkron dengan package.xml)
    license='Apache-2.0',  # Lisensi package (sinkron dengan package.xml)
    tests_require=['pytest'],  # Dependency untuk unit test Python
    entry_points={
        'console_scripts': [
            'visualizer_node = huskybot_perception.visualizer_node:main',  # Daftarkan node visualizer agar bisa di-run via ros2 run/launch
            'logger_node = huskybot_perception.logger_node:main',  # Daftarkan node logger agar bisa di-run via ros2 run/launch
        ],
    },
)
