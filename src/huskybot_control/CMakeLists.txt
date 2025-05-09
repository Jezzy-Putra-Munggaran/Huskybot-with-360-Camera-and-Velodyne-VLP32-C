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
  #set(ament_cmake_copyright_FOUND TRUE)            # Skip copyright lint jika belum ada copyright
  #set(ament_cmake_cpplint_FOUND TRUE)              # Skip cpplint jika bukan repo git
  ament_lint_auto_find_test_dependencies()          # Cari dependency test lint otomatis
  # Tambahkan test script jika ada
  # add_subdirectory(test)                          # Aktifkan jika ada folder test/ (unit test Python/C++)
endif()

install(
  DIRECTORY
    scripts                                         # Install folder scripts ke share/package/ (agar bisa diakses ros2 run/launch)
    launch                                          # Install folder launch ke share/package/ (agar bisa diakses ros2 launch)
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/robot_control.py                          # Install script utama robot_control.py ke lib/package (agar bisa dieksekusi ros2 run)
  scripts/safety_monitor.py                         # Install script safety_monitor.py ke lib/package (agar bisa dieksekusi ros2 run)
  scripts/logger.py                                 # Install script logger.py ke lib/package (agar bisa dieksekusi ros2 run)
  launch/huskybot_control.launch.py                 # Install launch file ke lib/package (agar bisa dieksekusi ros2 launch)
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()                                     # Macro wajib ROS2 agar package dikenali oleh ament/colcon