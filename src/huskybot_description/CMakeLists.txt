cmake_minimum_required(VERSION 3.5)                   # Versi minimum CMake yang dibutuhkan (standar ROS2)
project(huskybot_description)                          # Nama package

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                             # Set standar C ke C99 jika belum diset
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                           # Set standar C++ ke C++14 jika belum diset
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)        # Tambahkan warning compile untuk GCC/Clang
endif()

# find dependencies
find_package(ament_cmake REQUIRED)                     # Dependency utama semua package ROS2
find_package(urdf REQUIRED)                            # Dependency untuk parsing URDF/Xacro
find_package(xacro REQUIRED)                           # Tambahkan dependency xacro (untuk parsing xacro di launch)
find_package(sensor_msgs REQUIRED)                     # Tambahkan dependency sensor_msgs (untuk pesan sensor, jika diperlukan)
# find_package(<dependency> REQUIRED)                  # Tambahkan dependency lain jika perlu

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)               # Untuk linting otomatis saat testing
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()             # Cari dependency test lint otomatis
endif()

install(
  DIRECTORY
    launch                                            # Install folder launch ke share/package/
    robot                                             # Install folder robot ke share/package/
    calibration                                       # Install folder calibration ke share/package/ (saran 1)
    meshes                                            # Install folder meshes ke share/package/ (saran 1)
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    launch/spawn_huskybot.py                          # Install script Python agar bisa dieksekusi dari ros2 run
    launch/spawn_huskybot_launch.launch.py            # Install script Python agar bisa dieksekusi dari ros2 run
  DESTINATION lib/${PROJECT_NAME}
)

# Saran 5: Pastikan file Python punya shebang dan permission executable
# (Sudah ada shebang di spawn_huskybot.py, pastikan chmod +x sebelum build)

ament_package()                                       # Macro wajib ROS2 agar package dikenali oleh ament