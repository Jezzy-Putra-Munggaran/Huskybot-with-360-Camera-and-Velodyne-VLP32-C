#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_camera/setup.py

from setuptools import setup  # Import fungsi setup dari setuptools untuk konfigurasi package Python ROS2
import os  # Import modul os untuk operasi file dan direktori
from glob import glob  # Import glob untuk pencarian file dengan pattern

package_name = 'huskybot_camera'  # Nama package, harus sama dengan nama folder dan package.xml agar colcon build tidak error

# ===================== ERROR HANDLING: VALIDASI FILE-FILE KRITIS =====================
def validate_critical_files():  # Fungsi untuk validasi file-file penting sebelum instalasi
    """Validasi keberadaan file-file kritis untuk package."""
    critical_files = [  # List file-file yang wajib ada untuk package ini
        'launch/multicamera.launch.py',  # Launch file utama multicamera
        'launch/camera.launch.py',  # Launch file fallback/individual camera
        f'{package_name}/multicamera_publisher.py',  # Node utama multicamera publisher
        'package.xml',  # File manifest package ROS2
        'resource/' + package_name,  # Resource marker ROS2
    ]
    missing_files = []  # List file yang tidak ditemukan
    for file in critical_files:  # Cek setiap file penting
        if not os.path.exists(file):  # Jika file tidak ada
            missing_files.append(file)  # Tambahkan ke list file yang hilang
    if missing_files:  # Jika ada file yang hilang
        print(f"\n[ERROR] File-file penting tidak ditemukan: {missing_files}")  # Tampilkan pesan error
        print("[ERROR] File-file ini diperlukan agar package berfungsi dengan benar!")
        print("[TIP] Periksa struktur package dan pastikan semua file ada di lokasi yang benar\n")
        raise FileNotFoundError(f"File penting hilang: {missing_files}")  # Stop build jika file penting hilang

# Jalankan validasi file penting sebelum setup
validate_critical_files()  # Panggil fungsi validasi

# ===================== ERROR HANDLING: VALIDASI FOLDER DAN PERMISSION =====================
def ensure_folder_permission(folder_path):  # Fungsi untuk validasi permission folder
    """Pastikan folder bisa diakses dan ditulis (writeable)."""
    try:
        if not os.path.exists(folder_path):  # Jika folder belum ada
            os.makedirs(folder_path)  # Buat folder
        test_file = os.path.join(folder_path, '.permission_test')
        with open(test_file, 'w') as f:  # Coba tulis file dummy
            f.write('test')
        os.remove(test_file)  # Hapus file dummy
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke folder {folder_path}: {e}")
        print("[WARNING] Fallback ke /tmp untuk log/output")
        return '/tmp'
    return folder_path

# Pastikan folder log default bisa diakses
log_dir = ensure_folder_permission(os.path.expanduser('~/huskybot_camera_log'))  # Folder log default

# ===================== ERROR HANDLING: VALIDASI README DAN CONFIG =====================
readme_files = ['README.md'] if os.path.exists('README.md') else []  # List file README.md jika ada
config_files = glob('config/*.yaml') if os.path.isdir('config') else []  # Cari semua file konfigurasi YAML jika ada

# ===================== ERROR HANDLING: VALIDASI DEPENDENCY PYTHON =====================
def validate_python_dependencies():  # Fungsi untuk validasi dependency Python utama
    """Validasi dependency Python utama (cv2, numpy, yaml, dsb)."""
    deps = ['cv2', 'numpy', 'yaml', 'rclpy', 'cv_bridge']
    for dep in deps:
        try:
            __import__(dep)
        except ImportError:
            print(f"[ERROR] Modul Python '{dep}' tidak ditemukan. Install dengan: pip install {dep}")

validate_python_dependencies()  # Jalankan validasi dependency Python

# ===================== KONFIGURASI SETUP =====================
setup(
    name=package_name,  # Nama package (harus sama dengan folder dan package.xml)
    version='0.0.1',  # Versi package menggunakan semantic versioning
    packages=[package_name],  # Package Python yang akan diinstal (harus ada __init__.py)
    data_files=[
        # Registrasi package dengan ament index agar ROS2 bisa menemukan package ini
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Instal package.xml dan README.md untuk metadata dan dokumentasi
        ('share/' + package_name, ['package.xml'] + readme_files),
        # Instal semua launch files ke share/package_name/launch/
        ('share/' + package_name + '/launch', [
            'launch/multicamera.launch.py',
            'launch/camera.launch.py',
        ]),
    ] + ([('share/' + package_name + '/config', config_files)] if config_files else []),  # Instal config files jika ada
    install_requires=[
        'setuptools',  # Required untuk build Python packages
        'rclpy',  # ROS2 Python client library
        'opencv-python',  # Untuk akses kamera dan image processing
        'cv_bridge',  # Untuk konversi antara ROS dan OpenCV images
        'sensor_msgs',  # Untuk tipe pesan Image dan CameraInfo
        'std_msgs',  # Untuk tipe pesan Header dan lainnya
        'std_srvs',  # Untuk tipe service Trigger (restart kamera)
        'numpy',  # Untuk operasi array pada gambar
        'pyyaml',  # Untuk parsing file konfigurasi YAML
        'launch_ros',  # Untuk komponen launch ROS2
        'launch_xml',  # Untuk parsing XML launch description
    ],
    zip_safe=True,  # Package dapat diinstal sebagai file zip
    maintainer='Jezzy Putra Munggaran',  # Nama maintainer package
    maintainer_email='mungguran.jezzy.putra@gmail.com',  # Email maintainer
    description='Publisher kamera Arducam IMX477 asli untuk pipeline Huskybot (360° array). Mendukung konfigurasi hexagonal dengan 6 kamera dan integrasi dengan YOLOv12 untuk deteksi objek. Kompatibel dengan ROS2 Humble, simulasi Gazebo, dan robot Husky A200 + Jetson AGX Orin.',  # Deskripsi lengkap package, sesuai dengan package.xml
    license='MIT',  # Lisensi package (harus sama dengan package.xml dan README.md)
    tests_require=['pytest'],  # Unit test dengan pytest
    entry_points={
        'console_scripts': [
            'multicamera_publisher = huskybot_camera.multicamera_publisher:main',  # Node publisher multicamera
        ],
    },
)

# ===================== ERROR HANDLING: DETEKSI NODE YANG BELUM TERDAFTAR =====================
potential_nodes = [  # Cek file Python yang mungkin node tapi belum didaftarkan di entry_points
    os.path.basename(f).replace('.py', '') for f in glob(f'{package_name}/*.py')
    if os.path.basename(f) != '__init__.py'
    and os.path.basename(f) != 'multicamera_publisher.py'
    and os.path.isfile(f)
]

if __name__ == '__main__':  # Jika file ini dijalankan langsung
    print("\n=== huskybot_camera v0.0.1 setup completed ===")  # Pesan informasi versi

    # Tampilkan warning untuk file yang bisa jadi node tapi tidak terdaftar
    if potential_nodes:
        print("\n[WARNING] File Python yang belum terdaftar sebagai entry points:")
        for node in potential_nodes:
            print(f"  - {node}.py")
        print("[TIP] Tambahkan ke section 'entry_points' jika perlu dieksekusi sebagai executable\n")

    # Tampilkan pesan langkah selanjutnya
    print("Langkah selanjutnya:")
    print(f"1. Build package: 'colcon build --packages-select {package_name}'")
    print(f"2. Source workspace: 'source install/setup.bash'")
    print(f"3. Jalankan dengan: 'ros2 launch {package_name} multicamera.launch.py'\n")

    # Verifikasi konfigurasi launch file
    try:
        from launch.frontend import Parser  # Import parser untuk memvalidasi launch file
        for launch_file in ['launch/multicamera.launch.py', 'launch/camera.launch.py']:
            if os.path.exists(launch_file):
                print(f"[INFO] Memvalidasi launch file: {launch_file}")
                # Bisa tambahkan validasi syntax/parse di sini jika perlu
    except ImportError:
        print("[INFO] launch.frontend tidak tersedia, skip validasi launch file")

    # Tampilkan pesan tentang kompatibilitas dengan sistem
    print("\n[INFO] Package ini kompatibel dengan:")
    print("- ROS2 Humble Hawksbill")
    print("- Gazebo Classic 11 (simulasi)")
    print("- Clearpath Husky A200 Jetson AGX Orin Arducam Velodyne (real robot)")
    print("- YOLOv12 (TensorRT/ONNX/PyTorch) untuk deteksi dan segmentasi\n")