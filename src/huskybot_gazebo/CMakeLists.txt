cmake_minimum_required(VERSION 3.5)                # Versi minimum CMake yang dibutuhkan (standar ROS2)
project(huskybot_gazebo)                           # Nama package (harus sama dengan folder dan package.xml)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                         # Set standar C ke C99 jika belum diset
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                       # Set standar C++ ke C++14 jika belum diset
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)    # Tambahkan warning compile untuk GCC/Clang (good practice)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)                 # Dependency utama semua package ROS2
find_package(rclpy REQUIRED)                       # Dependency Python ROS2 (jika ada node Python, walau package ini hanya world/launch)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)              # (Contoh) Tambahkan dependency lain jika perlu

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)           # Untuk linting otomatis saat testing
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()         # Cari dependency test lint otomatis
endif()

install(
  DIRECTORY
    launch                                        # Install folder launch ke share/package/
    worlds                                        # Install folder worlds ke share/package/
  DESTINATION
    share/${PROJECT_NAME}/                        # Tujuan install di install/share/huskybot_gazebo/
)

ament_python_install_package(${PROJECT_NAME})

ament_package()                                   # Macro wajib penutup package ROS2