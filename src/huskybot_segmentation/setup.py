# ===================== SETUP.PY HUSKYBOT_SEGMENTATION (FULL KOMENTAR & ERROR HANDLING) =====================

from setuptools import find_packages, setup  # Import fungsi utama setuptools untuk build/install package Python ROS2
import os  # Untuk operasi file/path (cek file penting)
from glob import glob  # Untuk globbing file (launch, test, model, dsb)
import sys  # Untuk sys.exit jika error fatal saat build/install

package_name = 'huskybot_segmentation'  # Nama package (harus sama dengan folder, package.xml, dan resource/)

# ===================== ERROR HANDLING: CEK FILE PENTING SEBELUM BUILD =====================
# Cek file wajib (README.md, package.xml, launch, resource)
required_files = [
    'README.md',  # Dokumentasi package (wajib untuk audit dan PyPI)
    'package.xml',  # Metadata ROS2 package (wajib untuk colcon build)
    'resource/' + package_name,  # Resource index ROS2 (wajib untuk ament_python)
    'launch/segmentation.launch.py',  # Launch file utama (wajib untuk ros2 launch)
    'huskybot_segmentation/multicam_segmentation_node.py',  # Node utama (wajib untuk entry_points)
]
for f in required_files:
    if not os.path.exists(f):
        print(f"[FATAL] File wajib tidak ditemukan: {f}", file=sys.stderr)
        sys.exit(2)  # Fail-fast jika file penting hilang

# ===================== GLOB FILES: LAUNCH, TEST, MODEL, DLL =====================
# Globbing semua launch file, test file, dan model (jika ada)
launch_files = glob('launch/*.launch.py')  # Semua launch file Python
test_files = glob('test/*.py')  # Semua unit test/linter
model_files = glob('models/*.engine') + glob('models/*.onnx') + glob('models/*.pt')  # Semua model YOLOv12 (opsional)

# ===================== DATA FILES UNTUK INSTALL (SHARE KE ROS2) =====================
data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),  # Resource index ROS2
    (f'share/{package_name}', ['package.xml']),  # Metadata package.xml
    (f'share/{package_name}', ['README.md']),  # Dokumentasi README
]
# Tambahkan semua launch file ke share/launch/
for lf in launch_files:
    data_files.append((f'share/{package_name}/launch', [lf]))
# Tambahkan semua test file ke share/test/
for tf in test_files:
    data_files.append((f'share/{package_name}/test', [tf]))
# Tambahkan semua model file ke share/models/ (jika ada)
for mf in model_files:
    data_files.append((f'share/{package_name}/models', [mf]))

# ===================== INSTALL REQUIRES (DEPENDENCY PYTHON) =====================
install_requires = [
    'setuptools',  # Build system Python
    'rclpy',  # Core ROS2 Python client library
    'sensor_msgs',  # Untuk pesan Image dari kamera
    'cv_bridge',  # Konversi ROS <-> OpenCV
    'yolov12_msgs',  # Custom message YOLOv12 (Yolov12Inference, InferenceResult)
    'ultralytics',  # YOLOv12 Python API (inference segmentasi)
    'numpy',  # Operasi array/matrix
    'opencv-python',  # Image processing dan visualisasi
    'std_msgs',  # Header dan tipe message standar
    'std_srvs',  # Service Trigger (restart_model, get_status)
    'diagnostic_msgs',  # DiagnosticArray (monitoring node)
    'rcl_interfaces',  # Parameter descriptor/dynamic parameter
]
# ===================== ERROR HANDLING: CEK DEPENDENCY SAAT BUILD =====================
for dep in install_requires:
    try:
        __import__(dep.replace('-', '_'))  # Import test (pip install harus sukses)
    except ImportError:
        print(f"[WARNING] Dependency Python '{dep}' tidak ditemukan. Pastikan sudah install via pip/rosdep!", file=sys.stderr)
        # Tidak sys.exit, biarkan pip/colcon yang fail jika benar-benar missing

# ===================== ENTRY POINTS: NODE UTAMA =====================
entry_points = {
    'console_scripts': [
        'multicam_segmentation_node = huskybot_segmentation.multicam_segmentation_node:main',  # Node utama segmentasi multicam
    ],
}

# ===================== SETUP() =====================
setup(
    name=package_name,  # Nama package (wajib sama dengan folder dan package.xml)
    version='0.1.0',  # Versi package (sinkron dengan package.xml)
    packages=find_packages(exclude=['test']),  # Otomatis temukan semua subpackage Python kecuali test/
    data_files=data_files,  # Semua file yang harus di-share ke ROS2 (launch, test, model, resource, README)
    install_requires=install_requires,  # Semua dependency Python (sinkron dengan package.xml)
    zip_safe=True,  # Package aman di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer (sinkron dengan package.xml)
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer (sinkron dengan package.xml)
    description='Node segmentasi YOLOv12 multicamera untuk Huskybot (360° Arducam IMX477). Publish hasil segmentasi ke topic /detection (Yolov12Inference). Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C).',  # Deskripsi singkat (sinkron dengan package.xml)
    license='Apache-2.0',  # Lisensi package (sinkron dengan package.xml)
    tests_require=['pytest'],  # Dependency untuk unit test Python
    entry_points=entry_points,  # Daftar node utama (console_scripts)
    python_requires='>=3.8',  # Minimum Python version (ROS2 Humble = Python 3.8+)
    classifiers=[  # Metadata PyPI/ROS2
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    # ===================== SARAN PENINGKATAN: DEV/TEST DEPENDENCIES =====================
    extras_require={
        'dev': [
            'pytest',  # Unit test
            'flake8',  # Linter PEP8
            'pep257',  # Linter docstring
            'ament_copyright',
            'ament_flake8',
            'ament_pep257',
        ]
    },
)

# ===================== SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN LANGSUNG) =====================
# - Semua file penting (README, package.xml, resource, launch, node) dicek sebelum build (fail-fast jika hilang)
# - Semua launch/test/model file otomatis di-share ke ROS2 (tidak perlu hardcode, siap CI/CD)
# - Semua dependency Python sudah dicek dan warning jika belum install (fail-fast di pip/colcon)
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, Jetson Orin, dan robot real (Clearpath Husky A200)
# - FULL OOP di node utama (multicam_segmentation_node.py), setup.py hanya build system
# - Sudah siap audit trail, multi-robot, dan integrasi pipeline workspace (fusion, logger, dsb)
# - Saran: Tambahkan glob('models/*') jika folder models/ sudah ada (untuk YOLOv12 .engine/.onnx/.pt)
# - Saran: Tambahkan badge CI/CD dan coverage test di README jika pipeline sudah aktif
# - Saran: Tambahkan test/launch/test_segmentation_launch.py untuk CI/CD
# - Saran: Dokumentasikan semua parameter di README dan launch file
# - Saran: Tambahkan validasi permission file/folder di node utama (sudah di multicam_segmentation_node.py)
# - Saran: Jika ingin distribusi Docker, tambahkan requirements.txt dan .dockerignore
# - Saran: Sinkronkan dependency dengan package.xml setiap update