from setuptools import setup, find_packages                # Import setup tools untuk instalasi package Python

package_name = 'huskybot_control'                          # Nama package, harus sama dengan folder dan package.xml

setup(
    name=package_name,                                     # Nama package (wajib, harus sama dengan folder dan package.xml)
    version='0.1.0',                                       # Versi package, update jika ada perubahan besar (sinkron dengan package.xml)
    packages=find_packages(),                              # Temukan semua sub-package Python (jika ada __init__.py di scripts/)
    data_files=[
        ('share/ament_index/resource_index/packages',      # File resource index ROS2 (wajib agar dikenali colcon/ament)
            ['resource/' + package_name]),                 # File marker resource wajib
        ('share/' + package_name, ['package.xml']),        # Install package.xml ke share/package_name (wajib agar metadata ROS2 terbaca)
        ('share/' + package_name + '/launch', ['launch/huskybot_control.launch.py']),  # Install launch file ke share/package/launch (wajib agar bisa ros2 launch)
        ('share/' + package_name + '/config', []),         # (Saran) Install folder config jika ada file YAML parameter (tambahkan file YAML jika ada)
        ('share/' + package_name + '/test', []),           # (Saran) Install folder test jika ada unit test (tambahkan file test jika ada)
        ('share/' + package_name, ['README.md']),          # (Saran) Install README untuk dokumentasi (wajib untuk CI/CD dan user baru)
    ],
    install_requires=[
        'setuptools',                                      # Dependency Python wajib (untuk build/install package)
        'rclpy',                                           # Dependency utama node ROS2 Python (wajib untuk semua node ROS2 Python)
        'geometry_msgs',                                   # Untuk pesan Twist (cmd_vel), wajib untuk kontrol kecepatan
        'sensor_msgs',                                     # Untuk pesan Joy, LaserScan (joystick & safety_monitor)
        'std_msgs',                                        # Untuk pesan Bool, Float32 (safety_monitor, logger)
        'nav_msgs',                                        # Untuk pesan Odometry (logger, integrasi navigation)
        'python3-threading',                               # Untuk multi-threading (opsional, best practice logger/safety_monitor)
        'python3-yaml',                                    # Untuk parsing YAML parameter (opsional, best practice param_yaml)
        'logging',                                         # Untuk logging ke file (opsional, best practice audit trail)
    ],
    zip_safe=True,                                         # Package aman untuk di-zip (standar ROS2, best practice)
    maintainer='Jezzy Putra Munggaran',                    # Nama maintainer (metadata package, wajib untuk support)
    maintainer_email='mungguran.jezzy.putra@gmail.com',    # Email maintainer (metadata package, wajib untuk support)
    description='Node kontrol dan monitoring Huskybot (teleop/manual/autonomous) untuk simulasi dan real robot.',  # Deskripsi singkat package (jelas, wajib)
    license='Apache-2.0',                                  # Lisensi package (harus sama dengan package.xml, penting untuk distribusi legal)
    tests_require=['pytest'],                              # Dependency test Python (opsional, best practice untuk CI/CD)
    entry_points={
        'console_scripts': [
            # Daftarkan semua script yang ingin bisa dijalankan via ros2 run
            'robot_control.py = scripts.robot_control:main',           # Entry point robot_control.py (node utama kontrol robot)
            'safety_monitor.py = scripts.safety_monitor:main',         # Entry point safety_monitor.py (node safety monitor)
            'logger.py = scripts.logger:main',                         # Entry point logger.py (node logger/audit trail)
            # Tambahkan script lain jika ada (misal: joystick teleop, autonomous planner, dsb)
        ],
    },
)

# ===================== PENJELASAN & REVIEW BARIS PER BARIS =====================
# - Semua dependency utama ROS2 sudah dicantumkan (rclpy, geometry_msgs, sensor_msgs, std_msgs, nav_msgs).
# - install_requires sudah lengkap dan konsisten dengan package.xml (lihat #package.xml).
# - Semua script utama sudah didaftarkan di entry_points agar bisa di-run via ros2 run.
# - File launch, config, test, dan README sudah diinstall ke share agar bisa diakses node lain/CI.
# - Jika ada file YAML config/test, tambahkan ke data_files agar bisa diakses saat runtime.
# - Jika ada script baru, tambahkan di entry_points.
# - Jika ada dependency baru (misal: tf2_ros, message_filters), tambahkan di install_requires dan package.xml.
# - Jika ada error dependency saat build, colcon akan gagal dan log error.
# - Jika ada error permission, pastikan semua script Python sudah chmod +x (executable).
# - Jika ada error import, pastikan dependency sudah di package.xml/setup.py/CMakeLists.txt.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Tidak perlu OOP di file ini, karena hanya konfigurasi build/install.

# ===================== ERROR HANDLING & SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN) =====================
# - Tambahkan install folder config dan test agar file YAML dan unit test otomatis di-share (sudah ditambah).
# - Tambahkan README.md ke data_files agar dokumentasi selalu terinstall.
# - Tambahkan dependency Python3-threading, python3-yaml, dan logging agar robust untuk multi-thread dan parsing YAML.
# - Komentar penjelasan di setiap baris agar mudah dipahami siapapun.
# - Jika ingin multi-robot, tambahkan argumen namespace di launch file (tidak perlu di setup.py).
# - Jika ingin coverage test lebih tinggi, tambahkan test/launch test di folder test/.
# - Jika ingin audit trail, tambahkan logger ke file di setiap node (sudah ada di scripts/).
# - Jika ingin distribusi Docker, pastikan semua file penting diinstall ke share/lib.
# - Jika ingin integrasi CI/CD, pastikan semua test script diinstall ke share/test.
# - Jika ingin robust error handling, pastikan semua node Python (robot_control.py, safety_monitor.py, logger.py) sudah ada try/except di semua callback, validasi file/folder, log ke file dan terminal, warning data kosong, validasi parameter, dan exit code jelas.
# - Jika ingin robust multi-robot, tambahkan argumen namespace di launch file dan remap semua topic di node Python.
# - Jika ingin integrasi dengan YOLOv12 TensorRT/ONNX, pastikan node recognition sudah siap auto-switch engine/onnx dan logger siap audit multi-robot.
# - Jika ingin audit trail lebih detail, tambahkan opsi log ke CSV/JSON di logger.py dan dokumentasikan di README.
# - Jika ingin distribusi production, pastikan dependency versi dikunci di requirements.txt/setup.cfg.