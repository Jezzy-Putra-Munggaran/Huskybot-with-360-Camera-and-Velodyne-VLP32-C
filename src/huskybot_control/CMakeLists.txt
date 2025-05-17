cmake_minimum_required(VERSION 3.5)                # Versi minimum CMake yang dibutuhkan (standar ROS2, Humble min 3.5)
project(huskybot_control)                           # Nama package (harus sama dengan folder dan package.xml)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                          # Set standar C ke C99 jika belum diset (untuk source C, opsional di Python-only)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                        # Set standar C++ ke C++14 jika belum diset (untuk source C++, opsional di Python-only)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)     # Tambahkan warning compile untuk GCC/Clang (good practice, opsional di Python-only)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)                  # Dependency utama semua package ROS2 (wajib)
find_package(rclpy REQUIRED)                        # Dependency Python ROS2 (wajib untuk node Python)
find_package(geometry_msgs REQUIRED)                # Dependency pesan geometry_msgs (untuk Twist, dsb)
find_package(sensor_msgs REQUIRED)                  # Dependency pesan sensor_msgs (untuk Joy, LaserScan, dsb)
find_package(huskybot_description REQUIRED)         # (Opsional) Dependency ke package lain jika perlu (URDF/mesh)
find_package(huskybot_gazebo REQUIRED)              # (Opsional) Dependency ke package lain jika perlu (simulasi)
# find_package(<dependency> REQUIRED)               # (Contoh) Tambahkan dependency lain jika ada import di scripts/

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)            # Untuk linting otomatis saat testing (best practice CI)
  ament_lint_auto_find_test_dependencies()          # Cari dependency test lint otomatis
  # add_subdirectory(test)                          # Aktifkan jika ada folder test/ (unit test Python/C++)
endif()

install(
  DIRECTORY
    scripts                                         # Install folder scripts ke share/package/ (agar bisa diakses ros2 run/launch)
    launch                                          # Install folder launch ke share/package/ (agar bisa diakses ros2 launch)
    config                                          # Install folder config ke share/package/ (untuk file YAML parameter, jika ada)
    test                                            # Install folder test ke share/package/ (untuk unit test, jika ada)
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/robot_control.py                        # Install script utama robot_control.py ke lib/package (agar bisa dieksekusi ros2 run)
    scripts/safety_monitor.py                       # Install script safety_monitor.py ke lib/package (agar bisa dieksekusi ros2 run)
    scripts/logger.py                               # Install script logger.py ke lib/package (agar bisa dieksekusi ros2 run)
    launch/huskybot_control.launch.py               # Install launch file ke lib/package (agar bisa dieksekusi ros2 launch)
  DESTINATION lib/${PROJECT_NAME}
)

# ===================== ERROR HANDLING & BEST PRACTICE =====================
# - Semua dependency utama sudah dicantumkan (ament_cmake, rclpy, geometry_msgs, sensor_msgs).
# - Dependency ke huskybot_description dan huskybot_gazebo sudah ada untuk integrasi URDF/simulasi.
# - Semua script Python sudah diinstall ke lib agar bisa di-run via ros2 run.
# - Semua folder penting (scripts, launch, config, test) diinstall ke share agar bisa diakses node lain/CI.
# - Jika ada dependency baru (misal: std_msgs, nav_msgs), tambahkan find_package di atas.
# - Jika ada script baru, tambahkan di section PROGRAMS.
# - Jika ada folder baru (misal: resource, dataset), tambahkan di section DIRECTORY.
# - Jika ada file permission error, tambahkan install(CODE "execute_process(COMMAND chmod +x ...)") untuk script Python.
# - Jika ada error dependency saat build, colcon akan gagal dan log error.
# - Jika ada error permission, pastikan semua script Python sudah chmod +x.
# - Jika ada error import, pastikan dependency sudah di package.xml/setup.py juga.

ament_package()                                     # Macro wajib ROS2 agar package dikenali oleh ament/colcon

# ===================== SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN) =====================
# - Tambahkan install folder config dan test agar file YAML dan unit test otomatis di-share (sudah ditambah).
# - Tambahkan komentar penjelasan di setiap baris agar mudah dipahami siapapun.
# - Jika ingin multi-robot, tambahkan argumen namespace di launch file (tidak perlu di CMake).
# - Jika ingin coverage test lebih tinggi, tambahkan test/launch test di folder test/.
# - Jika ingin audit trail, tambahkan logger ke file di setiap node (sudah ada di scripts/).
# - Jika ingin distribusi Docker, pastikan semua file penting diinstall ke share/lib.
# - Jika ingin integrasi CI/CD, pastikan semua test script diinstall ke share/test.
# - Jika ingin integrasi dengan package lain, tambahkan ament_export_dependencies di bawah (opsional).