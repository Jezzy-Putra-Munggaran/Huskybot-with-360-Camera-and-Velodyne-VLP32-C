cmake_minimum_required(VERSION 3.5)                   # Versi minimum CMake yang dibutuhkan (standar ROS2 Humble)
project(huskybot_description)                          # Nama package, harus sama dengan folder dan package.xml

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                             # Set standar C ke C99 jika belum diset (untuk file C, antisipasi jika ada C)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                           # Set standar C++ ke C++14 jika belum diset (untuk file C++, antisipasi jika ada C++)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)        # Tambahkan warning compile untuk GCC/Clang (good practice, cegah bug silent)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)                     # Dependency utama semua package ROS2 (wajib)
find_package(urdf REQUIRED)                            # Dependency untuk parsing URDF/Xacro (wajib untuk robot_description)
find_package(xacro REQUIRED)                           # Dependency xacro (untuk parsing xacro di launch, wajib jika pakai .xacro)
find_package(sensor_msgs REQUIRED)                     # Dependency sensor_msgs (untuk pesan sensor, jika diperlukan)
find_package(gazebo_ros REQUIRED)                      # Dependency untuk integrasi Gazebo (plugin, spawn_entity, dsb)
find_package(std_msgs REQUIRED)                        # Dependency pesan standar ROS2 (jika ada node custom)
# Tambahkan dependency lain jika ada node custom lain (misal: nav_msgs, geometry_msgs, dsb)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)               # Untuk linting otomatis saat testing (best practice CI/CD)
  ament_lint_auto_find_test_dependencies()             # Cari dependency test lint otomatis (otomatis lint semua file)

  # Test validasi Xacro/URDF (fail build jika URDF/Xacro tidak valid)
  find_program(XACRO_EXECUTABLE xacro)                 # Cari executable xacro di sistem
  if(XACRO_EXECUTABLE)
    add_test(
      NAME xacro_check_huskybot
      COMMAND ${XACRO_EXECUTABLE} --check-order ${CMAKE_CURRENT_SOURCE_DIR}/robot/huskybot.urdf.xacro
    )                                                  # Test: validasi file xacro utama (fail jika error sintaks/urutan)
  endif()
  find_program(CHECK_URDF_EXECUTABLE check_urdf)       # Cari executable check_urdf di sistem
  if(CHECK_URDF_EXECUTABLE)
    add_test(
      NAME urdf_check_huskybot
      COMMAND ${CHECK_URDF_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/robot/huskybot.urdf.xacro
    )                                                  # Test: validasi hasil URDF dari xacro (fail jika error URDF)
  endif()
endif()

install(
  DIRECTORY
    launch                                            # Install folder launch ke share/package/ (agar launch file bisa diakses ros2 launch)
    robot                                             # Install folder robot ke share/package/ (URDF/Xacro robot)
    calibration                                       # Install folder calibration ke share/package/ (file kalibrasi sensor/YAML config)
    meshes                                            # Install folder meshes ke share/package/ (mesh visual robot/sensor)
  DESTINATION
    share/${PROJECT_NAME}/                            # Semua diinstall ke share/package_name/ (standar ROS2, agar bisa diakses package lain)
)

install(
  PROGRAMS
    launch/spawn_huskybot.py                          # Install script Python agar bisa dieksekusi dari ros2 run (pastikan chmod +x)
    launch/spawn_huskybot_launch.launch.py            # Install script Python agar bisa dieksekusi dari ros2 run (pastikan chmod +x)
  DESTINATION lib/${PROJECT_NAME}                     # Script Python diinstall ke lib/package_name/ (standar ROS2)
)

# Pastikan semua script Python sudah chmod +x sebelum build (otomatis, cegah error permission)
install(CODE "execute_process(COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/launch/spawn_huskybot.py)")
install(CODE "execute_process(COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/launch/spawn_huskybot_launch.launch.py)")

# Dokumentasi tambahan:
# - Semua folder penting diinstall ke share/package_name/ agar bisa diakses ros2 launch/run.
# - Semua dependency utama dan opsional sudah dicantumkan.
# - Test otomatis validasi Xacro/URDF jika BUILD_TESTING aktif.
# - Script Python dipastikan executable.

ament_package()                                       # Macro wajib ROS2 agar package dikenali oleh ament (wajib di akhir)