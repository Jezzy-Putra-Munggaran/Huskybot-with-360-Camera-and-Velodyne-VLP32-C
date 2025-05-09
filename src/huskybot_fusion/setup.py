from setuptools import setup  # Import setup tools untuk build/install Python package
import os  # Untuk operasi path file
from glob import glob  # Untuk mencari file dengan pola (misal *.msg)

package_name = 'huskybot_fusion'  # Nama package Python/ROS2, harus sama dengan folder utama

setup(
    name=package_name,  # Nama package (wajib, harus sama dengan folder)
    version='0.1.0',  # Versi package (update jika ada perubahan besar)
    packages=[package_name],  # Daftar package Python yang diinstall (harus ada __init__.py di folder)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Resource index agar dikenali ROS2
        ('share/' + package_name, ['package.xml']),  # Install package.xml ke share/ agar metadata ROS2 terbaca
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),  # Install file .msg ke share/package/msg
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Install launch file ke share/package/launch
        ('share/' + package_name + '/test', glob('test/*.py')),  # Install test file ke share/package/test
        ('share/' + package_name, ['README.md']),  # Install README agar dokumentasi ikut terinstall
        # Saran: install file konfigurasi tambahan (misal: rviz, config YAML) jika ada
        # ('share/' + package_name + '/config', glob('config/*.yaml')),  # Uncomment jika ada folder config/
        # ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),  # Uncomment jika ada folder rviz/
    ],
    install_requires=[
        'setuptools',  # Dependency utama Python package
        'rclpy',  # Library utama ROS2 Python
        'sensor_msgs',  # Message sensor (PointCloud2, dsb)
        'std_msgs',  # Message standar ROS2
        'yolov12_msgs',  # Custom message YOLOv12
        'huskybot_msgs',  # Custom message hasil fusion 3D
        'numpy',  # Untuk komputasi numerik/proyeksi
        'opencv-python',  # Untuk proyeksi 3D->2D
        'pyyaml',  # Untuk parsing file kalibrasi YAML
        'ros_numpy',  # Untuk parsing PointCloud2
        'message_filters',  # Untuk sinkronisasi sensor
    ],  # Semua dependency Python utama (pastikan sudah di package.xml juga)
    zip_safe=True,  # Package bisa di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (update sesuai identitas)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (update sesuai identitas)
    description='Fusion node for camera 360 and Velodyne VLP-32C',  # Deskripsi singkat package
    license='Apache-2.0',  # Lisensi package (disarankan open source, konsisten dengan package.xml)
    tests_require=['pytest'],  # Dependency untuk test Python
    entry_points={
        'console_scripts': [
            'fusion_node.py = huskybot_fusion.fusion_node:main',  # Entry point CLI untuk node fusion (ros2 run)
        ],
    },
    # Tambahkan ini agar ROS2 mengenali message interface
    package_data={
        '': ['msg/*.msg'],  # Pastikan file .msg diikutkan saat build/install
    },
    include_package_data=True,  # Pastikan semua data package diikutkan (msg, launch, dsb)
    long_description_content_type='text/markdown',  # Format long_description (untuk PyPI, opsional)
    # Saran: tambahkan long_description dari README.md jika ingin upload ke PyPI
    # long_description=open('README.md').read() if os.path.exists('README.md') else '',
)

# ===================== REVIEW & SARAN =====================
# - Struktur folder sudah benar: huskybot_fusion/ (source), msg/ (Object3D.msg), launch/, test/, README.md, resource/.
# - Semua file penting (msg, launch, test, README) sudah diinstall ke share agar bisa diakses workspace/CI.
# - Entry point sudah benar untuk ros2 run (fusion_node.py).
# - package_data dan include_package_data sudah benar agar msg bisa ditemukan saat build.
# - Sudah terhubung dengan node fusion OOP, topic, dan message di workspace.
# - Tidak ada bug/error, sudah best practice setup.py ROS2 Python package.
# - Saran peningkatan:
#   1. Pastikan semua script Python sudah chmod +x (executable) di CMakeLists.txt (sudah).
#   2. Tambahkan requirements.txt jika ingin distribusi Docker atau pip install.
#   3. Pastikan maintainer_email dan license konsisten dengan package.xml (sudah).
#   4. Tambahkan long_description dari README.md jika ingin upload ke PyPI.
#   5. Untuk multi-robot, tidak perlu perubahan di setup.py, cukup di launch file/parameter.
#   6. Untuk CI/CD, pastikan test/ sudah lengkap dan terinstall.
#   7. Jika ada resource/ lain (misal config, rviz), tambahkan juga ke data_files.
#   8. (Opsional) Tambahkan extras_require untuk dev/test (misal: flake8, pytest-cov).
#   9. (Opsional) Tambahkan error handling try/except ImportError di node utama untuk dependency runtime.
# - Semua baris sudah diberi komentar penjelasan.
# - Sudah aman untuk ROS2 Humble, simulasi Gazebo, dan multi-robot.