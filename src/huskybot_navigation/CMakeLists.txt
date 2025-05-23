cmake_minimum_required(VERSION 3.8)                # [WAJIB] Versi minimum CMake (3.8 untuk ROS2 Humble, aman di semua OS)
project(huskybot_navigation)                       # [WAJIB] Nama package, harus sama dengan folder dan package.xml

find_package(ament_cmake REQUIRED)                 # [WAJIB] Dependency utama semua package ROS2 (build system)
find_package(rclpy REQUIRED)                       # [WAJIB] Dependency Python ROS2 (untuk node Python)
find_package(nav2_msgs REQUIRED)                   # [WAJIB] Dependency pesan Nav2 (untuk navigation2)
find_package(nav_msgs REQUIRED)                    # [WAJIB] Dependency pesan nav_msgs (untuk path, costmap)
find_package(std_msgs REQUIRED)                    # [WAJIB] Dependency pesan std_msgs (untuk publish info)
find_package(sensor_msgs REQUIRED)                 # [WAJIB] Dependency pesan sensor_msgs (untuk obstacle/costmap)
find_package(geometry_msgs REQUIRED)               # [WAJIB] Dependency pesan geometry_msgs (untuk pose, twist)
find_package(tf2_ros REQUIRED)                     # [WAJIB] Untuk publish/lookup transformasi antar frame
find_package(ament_index_python REQUIRED)          # [WAJIB] Untuk lookup path package lain
find_package(ament_lint_auto REQUIRED)             # [OPSIONAL] Untuk linting otomatis saat testing (best practice CI)

# ===================== INSTALL RESOURCE & DATA FILES =====================
install(DIRECTORY
  launch/                                          # [WAJIB] Install semua launch file ke share/package_name/launch/
  config/                                          # [WAJIB] Install file konfigurasi Nav2/costmap ke share/package_name/config/
  rviz/                                            # [OPSIONAL] Install file konfigurasi RViz2 ke share/package_name/rviz/
  resource/                                        # [WAJIB] Install resource agar package dikenali colcon/ROS2
  DESTINATION share/${PROJECT_NAME}/               # [WAJIB] Semua diinstall ke share/package_name/ (standar ROS2)
)

install(DIRECTORY
  test/                                            # [OPSIONAL] Install test script ke share/package_name/test/
  DESTINATION share/${PROJECT_NAME}/test
)

install(FILES
  README.md                                        # [OPSIONAL] Install README untuk dokumentasi offline
  DESTINATION share/${PROJECT_NAME}/
)

# ===================== INSTALL SCRIPTS (NODE) =====================
install(PROGRAMS
  scripts/navigation_node.py                       # [OPSIONAL] Script node utama navigation (pastikan chmod +x)
  DESTINATION lib/${PROJECT_NAME}                  # [WAJIB] Script Python diinstall ke lib/package_name/ (standar ROS2)
)

# ===================== CHMOD +X UNTUK SEMUA SCRIPT PYTHON =====================
install(CODE "execute_process(COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/scripts/navigation_node.py)") # [WAJIB] Pastikan script Python executable

# ===================== AMENT PYTHON INSTALL (WAJIB UNTUK PACKAGE PYTHON) =====================
ament_python_install_package(${PROJECT_NAME})       # [WAJIB] Install package Python ke environment aktif saat build

# ===================== EXPORT INTERFACE (OPSIONAL UNTUK PACKAGE LAIN) =====================
ament_export_dependencies(nav2_msgs nav_msgs std_msgs sensor_msgs geometry_msgs tf2_ros ament_index_python) # [OPSIONAL] Export dependency agar bisa dipakai package lain

# ===================== LINTING & TESTING OTOMATIS (BEST PRACTICE CI/CD) =====================
if(BUILD_TESTING)
  ament_lint_auto_find_test_dependencies()          # [OPSIONAL] Otomatis cari dependency linter/test
endif()

ament_package()                                    # [WAJIB] Macro penutup package ROS2 agar bisa di-build dan diinstall

# ===================== PENJELASAN & HUBUNGAN DENGAN WORKSPACE =====================
# - Semua dependency ROS2 sudah dicantumkan (nav2_msgs, nav_msgs, geometry_msgs, dsb).
# - Semua file penting (launch, config, rviz, resource, test, README) diinstall ke share agar bisa diakses node lain.
# - Script node diinstall ke lib agar bisa di-run via ros2 run.
# - ament_python_install_package memastikan package Python bisa diimport node lain (misal: node fusion, control).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Arducam IMX477 + Velodyne VLP32-C).
# - FULL OOP: Semua node utama diimplementasi sebagai class Node di huskybot_navigation/.
# - Semua path file YAML sudah konsisten dengan node lain dan README.

# ===================== ERROR HANDLING & SARAN PENINGKATAN =====================
# - Jika dependency tidak ditemukan saat build, colcon akan gagal dan log error.
# - Jika file/folder hilang, node akan log error dan exit (lihat error handling di setiap node Python).
# - Jika script Python tidak executable, install(CODE chmod +x ...) akan memastikan permission benar.
# - Jika ada error saat install/build, colcon akan fail-fast.
# - Saran peningkatan:
#   1. Tambahkan test/launch test otomatis untuk CI/CD.
#   2. Tambahkan install untuk file dataset/ jika ingin distribusi data navigasi.
#   3. Tambahkan export interface jika ingin digunakan package lain.
#   4. Tambahkan check untuk versi minimum dependency di setup.py.
#   5. Tambahkan opsi install_requires untuk dependency test (pytest, flake8, pep257) di setup.py.
#   6. Tambahkan opsi extras_require untuk dependency opsional (open3d, rosbag2_py, ros2cli) di setup.py.
#   7. Tambahkan validasi file YAML di node Python (sudah ada di source).
#   8. Tambahkan logging error ke file untuk audit trail.