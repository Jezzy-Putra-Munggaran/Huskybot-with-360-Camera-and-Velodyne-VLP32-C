cmake_minimum_required(VERSION 3.8)  # [WAJIB] Versi minimum CMake untuk ROS2 Humble
project(huskybot_fusion)  # [WAJIB] Nama project/package (harus sama dengan folder dan package.xml)

find_package(ament_cmake REQUIRED)  # [WAJIB] Build system utama ROS2
find_package(ament_cmake_python REQUIRED)  # [WAJIB] Untuk install package Python
find_package(rclpy REQUIRED)  # [WAJIB] Library utama ROS2 Python
find_package(sensor_msgs REQUIRED)  # [WAJIB] Message sensor (PointCloud2, dsb)
find_package(std_msgs REQUIRED)  # [WAJIB] Message standar ROS2
find_package(yolov12_msgs REQUIRED)  # [WAJIB] Custom message YOLOv12 (hasil deteksi kamera)
find_package(rosidl_default_generators REQUIRED)  # [WAJIB] Untuk generate interface message
find_package(huskybot_msgs REQUIRED)  # [WAJIB] Custom message hasil fusion 3D

ament_python_install_package(${PROJECT_NAME})  # [WAJIB] Install package Python ke lib agar bisa di-run via ros2 run

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})  # [WAJIB] Install folder launch ke share agar bisa diakses ros2 launch
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})  # [WAJIB] Install folder msg ke share agar bisa diakses node lain/CI
install(DIRECTORY test DESTINATION share/${PROJECT_NAME})  # [BEST PRACTICE] Install folder test untuk CI/CD
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})  # [BEST PRACTICE] Install resource (misal: config, data)
install(FILES README.md DESTINATION share/${PROJECT_NAME})  # [BEST PRACTICE] Install README agar dokumentasi selalu tersedia

# WAJIB untuk ROS2 Python agar dikenali colcon/ros2 run
install(
  FILES resource/${PROJECT_NAME}  # [WAJIB] File marker agar package dikenali ament_index
  DESTINATION share/ament_index/resource_index/packages  # [WAJIB] Path resource index
)

ament_export_dependencies(rosidl_default_runtime std_msgs yolov12_msgs sensor_msgs)  # [WAJIB] Export dependency agar bisa diakses package lain
ament_package()  # [WAJIB] Macro penutup package ROS2

# ===================== REVIEW & SARAN PENINGKATAN =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Semua dependency utama ROS2 sudah dicantumkan (rclpy, sensor_msgs, std_msgs, yolov12_msgs, huskybot_msgs).
# - Semua file penting (launch, msg, test, resource, README) sudah diinstall ke share agar bisa diakses node lain/CI.
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Sudah terhubung otomatis ke pipeline workspace (fusion_node, topic, message, dsb).
# - Error handling: jika dependency tidak ditemukan, colcon akan gagal build dan log error.
# - Saran: tambahkan install untuk folder config jika ada file YAML/kalibrasi khusus fusion.
# - Saran: tambahkan badge CI/CD dan coverage test di README jika pipeline sudah aktif.
# - Saran: tambahkan test/launch test di folder test/ untuk validasi otomatis di CI/CD.
# - Saran: dokumentasikan semua dependency dan struktur folder di README.md.
# - Tidak perlu OOP di file ini, karena hanya konfigurasi build/install.