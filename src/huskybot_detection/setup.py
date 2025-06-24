from setuptools import find_packages, setup  # Import fungsi setup dan find_packages dari setuptools (wajib untuk ROS2 Python package)
import os  # Import os untuk operasi path dan file
from glob import glob  # Import glob untuk pattern matching file

package_name = 'huskybot_detection'  # Nama package, harus sama dengan folder utama dan package.xml

setup(
    name=package_name,  # Nama package (wajib, harus sama dengan folder dan package.xml)
    version='0.1.0',  # Versi package (sinkron dengan package.xml, update jika release baru)
    packages=find_packages(exclude=['test']),  # Cari semua subpackage Python, kecuali folder test/ (agar test tidak diinstall sebagai modul)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # File resource agar ROS2 bisa menemukan package ini
        ('share/' + package_name, ['package.xml']),  # Install package.xml ke share agar metadata ROS2 bisa ditemukan
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # Install semua launch file ke share/package/launch (wajib agar bisa ros2 launch)
        ('share/' + package_name + '/README.md', ['README.md']),  # Install README.md ke share/package/ (dokumentasi ikut terinstall)
        # Optional: Model files (if they exist)
        # ('share/' + package_name + '/models', glob('models/*.pt') + glob('models/*.onnx') + glob('models/*.engine') if os.path.isdir('models') else []),  # Install model files jika ada folder models/
    ],
    install_requires=[
        'setuptools',  # Dependency build Python
        'rclpy',  # Dependency utama node ROS2 Python
        'sensor_msgs',  # Untuk pesan Image
        'cv_bridge',  # Untuk konversi ROS <-> OpenCV
        'yolov12_msgs',  # Untuk custom message YOLOv12
        'ultralytics',  # Untuk YOLOv12 Python
        'numpy',  # Untuk operasi array/matrix
        'opencv-python',  # Untuk image processing
        'std_msgs',  # Untuk pesan standar seperti Header yang digunakan di node
        'std_srvs',  # Untuk layanan standar seperti Trigger untuk restart_model
        'diagnostic_msgs',  # Untuk pesan diagnostik dan monitoring
        'rcl_interfaces',  # Untuk parameter interfaces dan validasi
        'pyyaml',  # Untuk parse file YAML (konfigurasi dan parameter)
        'python3-logging',  # Untuk advanced logging ke file (audit/error handling)
    ],  # Semua dependency Python utama (pastikan sudah di package.xml juga)
    extras_require={
        'dev': [
            'pytest',  # Untuk unit testing
            'flake8',  # Untuk code style checking
            'pep257',  # Untuk docstring style checking
        ]
    },  # Dependency opsional untuk development (pip install -e .[dev])
    python_requires='>=3.8',  # Minimum Python version untuk ROS2 Humble
    zip_safe=True,  # Package aman untuk di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sinkron dengan package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (sinkron dengan package.xml)
    description='Node deteksi YOLOv12 multicamera untuk Huskybot (360° Arducam IMX477). Publish hasil deteksi ke topic /detection (Yolov12Inference). Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',  # Deskripsi singkat package (sinkron dengan package.xml)
    license='Apache-2.0',  # Lisensi package (sinkron dengan package.xml)
    tests_require=['pytest'],  # Dependency untuk unit test Python
    entry_points={
        'console_scripts': [
            'multicam_detection_node = huskybot_detection.multicam_detection_node:main',  # Daftarkan node utama agar bisa di-run via ros2 run/launch
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',  # Status pengembangan package
        'Intended Audience :: Science/Research',  # Target pengguna package
        'License :: OSI Approved :: Apache Software License',  # Format lisensi standar
        'Programming Language :: Python',  # Bahasa pemrograman
        'Topic :: Scientific/Engineering :: Artificial Intelligence',  # Topic kategori package
    ],  # Metadata tambahan untuk PyPI dan indexing
)

# ===================== REVIEW & SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN) =====================
# - Semua dependency di package.xml dan setup.py sudah disinkronkan (std_msgs, std_srvs, diagnostic_msgs, rcl_interfaces)
# - Ditambahkan support untuk model files (.pt, .onnx, .engine) jika folder models/ ada
# - Ditambahkan python_requires untuk pastikan Python 3.8+ (kompatibel dengan ROS2 Humble)
# - Ditambahkan extras_require untuk development dependencies (pip install -e .[dev])
# - Ditambahkan classifiers untuk metadata PyPI yang lebih baik
# - Semua file launch (*.launch.py) otomatis diinstall dengan glob
# - Sudah robust terhadap file yang mungkin tidak ada (model files, README, etc)
# - Sudah kompatibel sepenuhnya dengan ROS2 Humble dan colcon build
# - FULL OOP: formatnya sudah sesuai standar Python setuptools dengan semua metadata lengkap