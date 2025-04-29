cmake_minimum_required(VERSION 3.5)                  # Versi minimum CMake yang dibutuhkan (standar ROS2 Foxy+)
project(yolov12_msgs)                                # Nama package ROS2

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                           # Set standar C ke C99 jika belum diset
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                         # Set standar C++ ke C++14 jika belum diset
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)      # Tambahkan opsi warning untuk kompilasi C++
endif()

# find dependencies
find_package(ament_cmake REQUIRED)                   # Dependency utama build system ROS2
find_package(std_msgs REQUIRED)                      # Dependency message standar ROS2 (untuk Header)
find_package(builtin_interfaces REQUIRED)            # Dependency interface built-in ROS2 (misal: waktu, dsb)
find_package(rosidl_default_generators REQUIRED)     # Dependency generator interface ROS2 (untuk .msg)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/InferenceResult.msg"                        # Daftarkan file message custom InferenceResult
    "msg/Yolov12Inference.msg"                       # Daftarkan file message custom Yolov12Inference
    DEPENDENCIES std_msgs                            # Message ini depend pada std_msgs (Header)
 )
ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces) # Export dependency runtime agar package lain bisa pakai

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)             # Untuk linting otomatis saat testing
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()           # Otomatis cari dependency linting untuk testing
endif()

ament_package()                                     # Macro wajib penutup package ROS2 agar bisa di-build dan diinstall