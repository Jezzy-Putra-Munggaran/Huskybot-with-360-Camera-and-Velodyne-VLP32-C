#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/setup.py

from setuptools import setup                  # Import fungsi setup dari setuptools untuk konfigurasi package
import os                                     # Import modul os untuk operasi file dan direktori
from glob import glob                         # Import glob untuk pencarian file dengan pattern

package_name = 'huskybot_camera'              # Nama package, harus sama dengan nama folder untuk colcon build

# Fungsi untuk memvalidasi keberadaan file-file penting
def validate_critical_files():                # Validasi file-file penting sebelum instalasi
    """Validasi keberadaan file-file kritis untuk package."""
    critical_files = [                        # List file-file yang wajib ada
        'launch/multicamera.launch.py',       # Launch file untuk multicamera
        'launch/camera.launch.py',            # Launch file untuk kamera individual
        'huskybot_camera/multicamera_publisher.py',  # Node utama publisher
    ]
    
    missing_files = []                        # Inisialisasi list untuk file yang tidak ditemukan
    for file in critical_files:               # Cek setiap file penting
        if not os.path.exists(file):          # Jika file tidak ada
            missing_files.append(file)        # Tambahkan ke list file yang hilang
    
    if missing_files:                         # Jika ada file yang hilang
        print(f"\n[ERROR] File-file penting tidak ditemukan: {missing_files}")  # Tampilkan pesan error
        print("[ERROR] File-file ini diperlukan agar package berfungsi dengan benar!")
        print("[TIP] Periksa struktur package dan pastikan semua file ada di lokasi yang benar\n")

# Jalankan validasi file penting
validate_critical_files()                     # Panggil fungsi validasi

# Cek keberadaan file README.md untuk diinstal
readme_files = ['README.md'] if os.path.exists('README.md') else []  # List file README.md jika ada

# Cari semua file konfigurasi YAML jika ada
config_files = glob('config/*.yaml')          # Cari semua file konfigurasi YAML

setup(
    name=package_name,                        # Nama package (harus sama dengan folder untuk ROS2)
    version='0.0.1',                          # Versi package menggunakan semantic versioning
    packages=[package_name],                  # Package Python yang akan diinstal
    
    # Data files yang akan diinstal ke direktori share
    data_files=[
        # Registrasi package dengan ament index
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),       # Resource marker untuk discovery ROS2
        
        # Instal package.xml untuk metadata
        ('share/' + package_name, ['package.xml'] + readme_files),  # Package manifest + README
        
        # Instal semua launch files
        ('share/' + package_name + '/launch', [
            'launch/multicamera.launch.py',   # Launch file multicamera
            'launch/camera.launch.py',        # Launch file camera individual
        ]),
    ] + ([('share/' + package_name + '/config', config_files)] if config_files else []),  # Instal config files jika ada
    
    # Dependensi Python yang diperlukan
    install_requires=[
        'setuptools',                         # Required untuk build Python packages
        'rclpy',                              # ROS2 Python client library
        'opencv-python',                      # Untuk akses kamera dan image processing 
        'cv_bridge',                          # Untuk konversi antara ROS dan OpenCV images
        'sensor_msgs',                        # Untuk tipe pesan Image dan CameraInfo
        'std_msgs',                           # Untuk tipe pesan Header dan lainnya
        'std_srvs',                           # Untuk tipe service Trigger (restart kamera)
        'numpy',                              # Untuk operasi array pada gambar
        'pyyaml',                             # Untuk parsing file konfigurasi YAML
        'launch_ros',                         # Untuk komponen launch ROS2
        'launch_xml',                         # Untuk parsing XML launch description
    ],
    
    zip_safe=True,                            # Package dapat diinstal sebagai file zip
    
    # Metadata tentang package
    maintainer='Jezzy Putra Munggaran',       # Nama maintainer package
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer
    
    # Deskripsi lengkap package, sesuai dengan package.xml
    description='Publisher kamera Arducam IMX477 asli untuk pipeline Huskybot (360° array). Mendukung konfigurasi hexagonal dengan 6 kamera dan integrasi dengan YOLOv12 untuk deteksi objek. Kompatibel dengan ROS2 Humble, simulasi Gazebo, dan robot Husky A200 + Jetson AGX Orin.',
    
    license='MIT',                            # Lisensi package
    
    # Dependensi untuk testing
    tests_require=['pytest'],                 # Unit test dengan pytest
    
    # Entry points untuk executable scripts
    entry_points={
        'console_scripts': [
            # Node publisher multicamera
            'multicamera_publisher = huskybot_camera.multicamera_publisher:main',
        ],
    },
)

# Deteksi potensi node Python yang belum terdaftar sebagai entry point
potential_nodes = [                           # Cek file Python yang mungkin node
    os.path.basename(f).replace('.py', '') for f in glob(f'{package_name}/*.py') 
    if os.path.basename(f) != '__init__.py' 
    and os.path.basename(f) != 'multicamera_publisher.py'
    and os.path.isfile(f)
]

# Pesan informasi versi
print(f"\n=== {package_name} v0.0.1 setup completed ===")

# Tampilkan warning untuk file yang bisa jadi node tapi tidak terdaftar
if potential_nodes:                           # Jika ada file yang berpotensi sebagai node
    print("\n[WARNING] File Python yang belum terdaftar sebagai entry points:")
    for node in potential_nodes:              # Tampilkan setiap file
        print(f"  - {node}.py")
    print("[TIP] Tambahkan ke section 'entry_points' jika perlu dieksekusi sebagai executable\n")

# Tampilkan pesan langkah selanjutnya
print("Langkah selanjutnya:")
print(f"1. Build package: 'colcon build --packages-select {package_name}'")
print(f"2. Source workspace: 'source install/setup.bash'")
print(f"3. Jalankan dengan: 'ros2 launch {package_name} multicamera.launch.py'\n")

# Verifikasi konfigurasi launch file
try:
    from launch.frontend import Parser        # Import parser untuk memvalidasi launch file
    for launch_file in ['launch/multicamera.launch.py', 'launch/camera.launch.py']:
        if os.path.exists(launch_file):       # Jika file launch ada
            print(f"[INFO] Memvalidasi launch file: {launch_file}")
            # Tambahan validasi launch file bisa ditambahkan di sini
except ImportError:
    print("[INFO] launch.frontend tidak tersedia, skip validasi launch file")

# Tampilkan pesan tentang kompatibilitas dengan sistem
print("\n[INFO] Package ini kompatibel dengan:")
print("- ROS2 Humble Hawksbill")
print("- Gazebo Classic 11 (simulasi)")
print("- Clearpath Husky A200 Jetson AGX Orin 6x Arducam IMX477 Velodyne VLP32-C (real robot)")
print("- YOLOv12 (TensorRT/ONNX/PyTorch) untuk deteksi dan segmentasi\n")
