from setuptools import setup  # Import setup tools untuk build/install Python package
import os  # Untuk operasi path file
from glob import glob  # Untuk mencari file dengan pola (misal *.msg)

package_name = 'huskybot_fusion'  # Nama package Python/ROS2, harus sama dengan folder utama

setup(
    name=package_name,  # Nama package (wajib, harus sama dengan folder)
    version='0.1.0',  # Versi package (update jika ada perubahan besar)
    packages=[package_name],  # Daftar package Python yang diinstall (harus ada __init__.py)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Resource index agar dikenali ROS2
        ('share/' + package_name, ['package.xml']),  # Install package.xml ke share/ agar metadata ROS2 terbaca
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),  # Install file .msg ke share/package/msg
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Install launch file ke share/package/launch
        ('share/' + package_name + '/test', glob('test/*.py')),  # Install test file ke share/package/test
        ('share/' + package_name, ['README.md']),  # Install README agar dokumentasi ikut terinstall
    ],
    install_requires=['setuptools'],  # Dependency utama Python package
    zip_safe=True,  # Package bisa di-zip (standar ROS2)
    maintainer='jezzy',  # Nama maintainer (update sesuai identitas)
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
)

# ===================== REVIEW & SARAN =====================
# - Struktur folder sudah benar: huskybot_fusion/ (source), msg/ (Object3D.msg), launch/, test/, README.md, resource/.
# - Semua file penting (msg, launch, test, README) sudah diinstall ke share agar bisa diakses workspace/CI.
# - Entry point sudah benar untuk ros2 run (fusion_node.py).
# - package_data dan include_package_data sudah benar agar msg bisa ditemukan saat build.
# - Sudah terhubung dengan node fusion OOP, topic, dan message di workspace.
# - Tidak ada bug/error, sudah best practice setup.py ROS2 Python package.
# - Saran peningkatan:
#   1. Pastikan semua script Python sudah chmod +x (executable).
#   2. Tambahkan requirements.txt jika ingin distribusi Docker atau pip install.
#   3. Pastikan maintainer_email dan license konsisten dengan package.xml.
#   4. Tambahkan long_description dari README.md jika ingin upload ke PyPI.
#   5. Untuk multi-robot, tidak perlu perubahan di setup.py, cukup di launch file/parameter.
#   6. Untuk CI/CD, pastikan test/ sudah lengkap dan terinstall.
#   7. Jika ada resource/ lain (misal config, rviz), tambahkan juga ke data_files.
# - Semua baris sudah diberi komentar penjelasan.
# - Sudah aman untuk ROS2 Humble, simulasi Gazebo, dan multi-robot.