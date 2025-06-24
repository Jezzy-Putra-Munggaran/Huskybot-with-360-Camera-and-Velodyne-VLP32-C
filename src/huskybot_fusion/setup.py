#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_fusion/setup.py

from setuptools import setup  # Import setup tools utama untuk build/install package Python ROS2
import os  # Import modul os untuk operasi path dan filesystem (cek file, folder, permission)
import sys  # Import sys untuk akses stderr dan exit code (error handling lebih baik)
from glob import glob  # Import glob untuk pattern matching file (yaml, launch, etc)
import subprocess  # Import subprocess untuk check_output (validasi dependency Python)

# ===================== VALIDASI LINGKUNGAN PYTHON =====================
PYTHON_MINIMUM = (3, 8)  # Versi minimum Python untuk ROS2 Humble

if sys.version_info < PYTHON_MINIMUM:  # Validasi versi Python minimum
    sys.stderr.write(
        f"[FATAL] Python {'.'.join(map(str, PYTHON_MINIMUM))} atau lebih baru diperlukan! "
        f"Anda menggunakan Python {'.'.join(map(str, sys.version_info[:3]))}\n"
    )
    sys.exit(1)  # Exit dengan error code 1 jika versi Python dibawah minimum

package_name = 'huskybot_fusion'  # Nama package Python/ROS2, harus sama dengan folder utama

# ===================== ERROR HANDLING: CEK FILE/FOLDER WAJIB =====================
# Validasi struktur folder package ROS2 (wajib agar dikenali sebagai package valid)
required_folders = ['huskybot_fusion', 'launch', 'resource']  # Folder wajib untuk package ROS2 valid
for folder in required_folders:  # Iterasi setiap folder wajib
    if not os.path.isdir(folder):  # Cek folder ada
        sys.stderr.write(f"[FATAL] Folder {folder}/ tidak ditemukan. Wajib untuk package ROS2 valid!\n")
        sys.exit(2)  # Exit dengan error code 2 jika folder tidak ditemukan

# Validasi file marker resource (wajib agar package dikenali ROS2)
resource_marker = os.path.join('resource', package_name)  # Path file marker resource
if not os.path.isfile(resource_marker):  # Cek file marker wajib ada
    sys.stderr.write(f"[FATAL] File marker {resource_marker} tidak ditemukan. Wajib ada agar package dikenali ROS2.\n")
    sys.exit(3)  # Exit dengan error code 3 jika file marker tidak ditemukan

# Validasi package.xml (wajib untuk metadata ROS2)
if not os.path.isfile('package.xml'):  # Cek package.xml wajib ada
    sys.stderr.write("[FATAL] package.xml tidak ditemukan. Wajib ada agar metadata ROS2 terbaca.\n")
    sys.exit(4)  # Exit dengan error code 4 jika package.xml tidak ditemukan

# Validasi README.md untuk dokumentasi
readme_path = 'README.md'  # Path file README
if not os.path.isfile(readme_path):  # Cek README wajib ada
    sys.stderr.write("[WARNING] README.md tidak ditemukan. Disarankan ada untuk dokumentasi package.\n")
    # Warning saja, bukan fatal error

# Cek permission folder launch (wajib executable untuk ros2 launch)
if os.path.isdir('launch') and not os.access('launch', os.X_OK):  # Cek permission folder launch
    sys.stderr.write("[WARNING] Folder launch/ tidak memiliki permission executable. "
                     "Launch file mungkin tidak berfungsi.\n")
    try:
        os.chmod('launch', os.stat('launch').st_mode | 0o111)  # Tambah executable permission ke folder
        sys.stderr.write("  + Permission folder launch/ ditambahkan otomatis.\n")
    except Exception as e:
        sys.stderr.write(f"  - Error saat tambah permission: {e}\n")

# Cek permission semua launch file Python (wajib executable untuk ros2 launch)
try:  # Try-except untuk validasi launch file
    for launch_file in glob('launch/*.py'):  # Iterasi semua file Python di folder launch/
        if not os.access(launch_file, os.X_OK):  # Cek permission file launch
            sys.stderr.write(f"[WARNING] Launch file {launch_file} tidak executable. "
                             "Tidak akan berfungsi dengan ros2 launch.\n")
            try:
                os.chmod(launch_file, os.stat(launch_file).st_mode | 0o111)  # Tambah executable permission
                sys.stderr.write(f"  + Permission {launch_file} ditambahkan otomatis.\n")
            except Exception as e:
                sys.stderr.write(f"  - Error saat tambah permission: {e}\n")
except Exception as e:
    sys.stderr.write(f"[WARNING] Error saat cek permission launch file: {e}\n")

# ===================== VALIDASI DEPENDENCY PENTING =====================
try:  # Try-except untuk validasi dependency penting
    # Cek dependency Python utama (numpy, cv2, yaml)
    for module_name, pip_name in [
        ('numpy', 'numpy'),
        ('cv2', 'opencv-python'),
        ('yaml', 'pyyaml'),
        ('rclpy', 'rclpy'),
        ('message_filters', 'message_filters')
    ]:
        try:
            __import__(module_name)  # Coba import module untuk validasi
        except ImportError:
            sys.stderr.write(f"[WARNING] Module {module_name} tidak ditemukan. "
                             f"Install dengan: pip install {pip_name}\n")
except Exception as e:
    sys.stderr.write(f"[WARNING] Error saat validasi dependency: {e}\n")

# ===================== PERSIAPAN PATH FILE UNTUK DATA_FILES =====================
# Cari semua file msg, launch, test, config, dan rviz secara aman dengan error handling
msgs = glob('msg/*.msg') if os.path.isdir('msg') else []  # File .msg (jika folder ada)
launch_files = glob('launch/*.py') if os.path.isdir('launch') else []  # File launch (jika folder ada)
test_files = glob('test/*.py') if os.path.isdir('test') else []  # File test (jika folder ada)
config_files = glob('config/*.yaml') if os.path.isdir('config') else []  # File config (jika folder ada)
rviz_files = glob('rviz/*.rviz') if os.path.isdir('rviz') else []  # File rviz (jika folder ada)

# Tambahan: validasi format README untuk long_description
readme_content = ''  # Default kosong
if os.path.isfile(readme_path):  # Cek README ada
    try:
        with open(readme_path, 'r', encoding='utf-8') as f:  # Buka README dengan UTF-8
            readme_content = f.read()  # Baca isi README
    except Exception as e:
        sys.stderr.write(f"[WARNING] Error saat baca README: {e}\n")

# ===================== SETUP PACKAGE ROS2 =====================
setup(
    name=package_name,  # Nama package (wajib sama dengan folder dan package.xml)
    version='0.1.0',  # Versi package (sinkron dengan package.xml, update jika ada major changes)
    packages=[package_name],  # Daftar package Python yang diinstall (harus ada __init__.py di folder)
    data_files=[
        # File resource index wajib untuk ROS2, memberikan informasi letak package di filesystem
        ('share/ament_index/resource_index/packages', [resource_marker]),
        
        # Package.xml wajib diinstall ke share/ agar metadata ROS2 terbaca
        ('share/' + package_name, ['package.xml']),
        
        # Install semua file .msg ke share/package/msg jika ada (untuk custom message ROS2)
        (os.path.join('share', package_name, 'msg'), msgs) if msgs else None,
        
        # Install semua file launch ke share/package/launch (wajib untuk ros2 launch)
        ('share/' + package_name + '/launch', launch_files) if launch_files else None,
        
        # Install semua file test ke share/package/test (untuk unit test dan CI/CD)
        ('share/' + package_name + '/test', test_files) if test_files else None,
        
        # Install README ke share/package/ (untuk dokumentasi yang bisa diakses runtime)
        ('share/' + package_name, [readme_path]) if os.path.isfile(readme_path) else None,
        
        # Install file kalibrasi/config ke share/package/config (untuk parameter runtime)
        ('share/' + package_name + '/config', config_files) if config_files else None,
        
        # Install file RViz ke share/package/rviz (untuk konfigurasi visualisasi)
        ('share/' + package_name + '/rviz', rviz_files) if rviz_files else None,
    ],  # Filter None values dari data_files untuk hindari error "NoneType object is not iterable"
    install_requires=[
        'setuptools',  # Dependency utama build Python package
        'rclpy>=0.9.0',  # Library utama ROS2 Python (minimal ROS2 Humble)
        'sensor_msgs',  # Message sensor (PointCloud2, LaserScan)
        'std_msgs',  # Message standar ROS2 (Header, Bool)
        'geometry_msgs',  # Message geometry (Point, Quaternion, PoseStamped)
        'visualization_msgs',  # Message visualisasi (Marker, MarkerArray)
        'yolov12_msgs',  # Custom message YOLOv12 (deteksi objek) - wajib untuk fusion dengan hasil YOLOv12
        'huskybot_msgs',  # Custom message hasil fusion 3D - wajib untuk output node fusion
        'tf2_ros',  # Untuk transformasi koordinat antar frame (LiDAR ke kamera)
        'tf2_geometry_msgs',  # Untuk transformasi message geometry dengan TF2
        'numpy>=1.20.0',  # Untuk komputasi numerik/proyeksi (minimal versi dengan type hints)
        'opencv-python>=4.5.0',  # Untuk proyeksi 3D->2D dan pengolahan gambar (min versi compatible dengan ROS2)
        'pyyaml>=5.3.0',  # Untuk parsing file kalibrasi YAML (min versi dengan safe_load secure)
        'message_filters',  # Untuk sinkronisasi sensor (time sync dan approximate sync)
        'scipy>=1.6.0',  # Untuk komputasi matematika kompleks dan transformasi
    ],  # Semua dependency Python utama (pastikan sinkron dengan <build_depend> dan <exec_depend> di package.xml)
    python_requires='>=3.8',  # Minimum Python version untuk ROS2 Humble
    zip_safe=True,  # Package aman untuk di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sinkron dengan package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (sinkron dengan package.xml)
    description=('Node fusion data kamera 360° dan Velodyne VLP-32C untuk deteksi objek 3D. '  # Deskripsi singkat
                 'Mengintegrasikan hasil YOLOv12 dengan point cloud LiDAR untuk navigasi aman. '  # Bagian tengah
                 'Siap untuk ROS2 Humble, Gazebo, dan robot Clearpath Husky A200.'),  # Bagian akhir - wajib sinkron dengan package.xml
    license='Apache-2.0',  # Lisensi package (disarankan open source, sinkron dengan package.xml)
    tests_require=['pytest', 'pytest-cov'],  # Dependency untuk test Python dan coverage
    entry_points={
        'console_scripts': [
            'fusion_node = huskybot_fusion.fusion_node:main',  # Entry point node fusion utama
            'simple_fusion_node = huskybot_fusion.simple_fusion_node:main',  # Entry point node fusion sederhana
        ],  # Daftarkan semua script executable (wajib agar bisa dijalankan dengan ros2 run)
    },
    classifiers=[  # Classifier untuk metadata PyPI
        'Development Status :: 4 - Beta',  # Status development package
        'Intended Audience :: Science/Research',  # Target user package
        'License :: OSI Approved :: Apache Software License',  # License classifier sesuai license
        'Programming Language :: Python',  # Bahasa pemrograman
        'Topic :: Scientific/Engineering :: Artificial Intelligence',  # Topik AI (YOLOv12)
        'Topic :: Software Development :: Libraries :: Python Modules',  # Topik library Python
        'Framework :: Robot Framework :: Tool',  # Framework robot
    ],
    include_package_data=True,  # Pastikan semua data package diikutkan (msg, launch, dsb)
    long_description_content_type='text/markdown',  # Format long_description adalah Markdown
    long_description=readme_content,  # Isi long_description dari README.md jika berhasil dibaca
    project_urls={  # URL tambahan untuk project (dokumentasi, issues, dll)
        'Source': 'https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C',
        'Issues': 'https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/issues',
        'Documentation': 'https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C',
    },
    extras_require={
        'dev': [  # Dependencies khusus untuk development
            'flake8',  # Linter Python untuk code quality
            'pytest',  # Unit testing framework
            'pytest-cov',  # Coverage reporting untuk test
            'pylint',  # Advanced linter
            'ament_pep257',  # Linter docstring untuk ROS2
            'pycodestyle',  # Style checker (PEP8)
        ],
        'sim': [  # Dependencies khusus untuk simulasi Gazebo
            'rosgraph_msgs',  # Messages untuk simulasi clock 
            'gazebo_msgs',  # Messages untuk interaksi dengan Gazebo
        ],
        'viz': [  # Dependencies khusus untuk visualisasi
            'matplotlib',  # Plotting library untuk visualisasi tambahan
            'open3d',  # Library untuk visualisasi 3D point cloud (opsional)
        ],
    },
)

# ===================== POST-SETUP VALIDATION =====================
# Tambahan validasi post-setup untuk memastikan semuanya sudah benar
print(f"[INFO] Setup package '{package_name}' selesai.")

# Cek apakah file __init__.py ada di folder package (wajib agar bisa diimport)
init_file = os.path.join(package_name, '__init__.py')
if not os.path.isfile(init_file):
    sys.stderr.write(f"[WARNING] File {init_file} tidak ditemukan. "
                     "Package tidak akan bisa diimport! Buat file kosong ini.\n")
    try:
        with open(init_file, 'w') as f:  # Buat file kosong __init__.py
            f.write('# File auto-generated saat setup untuk memastikan package bisa diimport\n')
        print(f"[INFO] File {init_file} dibuat otomatis.")
    except Exception as e:
        sys.stderr.write(f"  - Error membuat {init_file}: {e}\n")

# Cek apakah file fusion_node.py dan simple_fusion_node.py ada di folder package
required_modules = {
    'fusion_node.py': 'fusion_node = huskybot_fusion.fusion_node:main',
    'simple_fusion_node.py': 'simple_fusion_node = huskybot_fusion.simple_fusion_node:main'
}

for module_file, entry_point in required_modules.items():
    module_path = os.path.join(package_name, module_file)
    if not os.path.isfile(module_path):
        sys.stderr.write(f"[ERROR] File {module_path} untuk entry point '{entry_point}' tidak ditemukan. "
                         "Entry point tidak akan berfungsi!\n")
    elif not os.access(module_path, os.X_OK):
        sys.stderr.write(f"[WARNING] File {module_path} tidak executable. "
                         "Entry point mungkin tidak berfungsi dengan ros2 run.\n")
        try:
            os.chmod(module_path, os.stat(module_path).st_mode | 0o111)  # Tambah executable permission
            print(f"[INFO] Permission {module_path} ditambahkan otomatis.")
        except Exception as e:
            sys.stderr.write(f"  - Error saat tambah permission: {e}\n")

# Periksa jika ada potensi konflik namespace package
namespace_dirs = []
for root, dirs, files in os.walk('.'):
    if root == '.' or not root.startswith('./'):
        continue
    if '__pycache__' in dirs:
        dirs.remove('__pycache__')
    if root.count('/') == 1 and package_name in dirs:  # Check for potential conflict
        namespace_dirs.append(os.path.basename(root))

if namespace_dirs:
    sys.stderr.write(f"[WARNING] Potensi konflik namespace package '{package_name}' di folder: "
                     f"{', '.join(namespace_dirs)}. Ini dapat menyebabkan import error!\n")

# Validasi referensi file di package.xml vs setup.py
try:
    from xml.etree import ElementTree
    tree = ElementTree.parse('package.xml')
    root = tree.getroot()
    
    # Cek versi di package.xml vs setup.py
    pkg_version = root.find('version').text if root.find('version') is not None else None
    if pkg_version and pkg_version != '0.1.0':
        sys.stderr.write(f"[WARNING] Versi di package.xml ({pkg_version}) berbeda dengan "
                         f"setup.py (0.1.0). Sinkronkan keduanya!\n")
    
    # Cek maintainer info
    pkg_maintainer = root.find('maintainer').text if root.find('maintainer') is not None else None
    pkg_email = root.find('maintainer').get('email') if root.find('maintainer') is not None else None
    if pkg_maintainer and pkg_maintainer != 'Jezzy Putra Munggaran':
        sys.stderr.write(f"[WARNING] Nama maintainer di package.xml ({pkg_maintainer}) berbeda dengan "
                         f"setup.py (Jezzy Putra Munggaran). Sinkronkan keduanya!\n")
    
    # Cek dependency yang disebutkan di package.xml tapi tidak ada di setup.py
    setup_deps = [
        'setuptools', 'rclpy', 'sensor_msgs', 'std_msgs', 'geometry_msgs', 'visualization_msgs',
        'yolov12_msgs', 'huskybot_msgs', 'tf2_ros', 'tf2_geometry_msgs', 'numpy',
        'opencv-python', 'pyyaml', 'message_filters', 'scipy'
    ]
    
    # Ekstrak semua dependency dari package.xml (build_depend, exec_depend, dll)
    xml_deps = []
    for dep_tag in ['build_depend', 'exec_depend', 'depend']:
        for dep in root.findall(dep_tag):
            if dep.text and dep.text not in xml_deps and dep.text != 'rclcpp':
                xml_deps.append(dep.text)
    
    # Filter dependency Python saja dari package.xml
    python_deps = [dep for dep in xml_deps if not dep.startswith(
        ('ros-humble-', 'ament_', 'rosidl_'))]
    
    # Cek dependency Python yang tidak ada di setup.py
    missing_deps = []
    for dep in python_deps:
        matched = False
        for setup_dep in setup_deps:
            # Normalize dependency names (e.g., opencv-python vs python3-opencv)
            if (dep == setup_dep or
                (dep.startswith('python3-') and setup_dep == dep[8:]) or
                (dep == 'opencv' and 'opencv-python' == setup_dep)):
                matched = True
                break
        if not matched:
            missing_deps.append(dep)
    
    if missing_deps:
        sys.stderr.write(f"[WARNING] Dependency di package.xml tidak ada di setup.py: "
                         f"{', '.join(missing_deps)}. Sinkronkan keduanya!\n")
    
except Exception as e:
    sys.stderr.write(f"[WARNING] Error saat validasi package.xml: {e}\n")

print("[INFO] Setup dan validasi package selesai.")