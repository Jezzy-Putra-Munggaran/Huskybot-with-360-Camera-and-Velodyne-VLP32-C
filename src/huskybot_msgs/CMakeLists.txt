cmake_minimum_required(VERSION 3.8)  # Versi minimum CMake yang didukung ROS2 Humble
project(huskybot_msgs)  # Nama package, harus sama dengan folder dan package.xml

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # Aktifkan warning compile untuk C++ (best practice, cegah bug silent)
endif()

# ==================== DEPENDENCY WAJIB UNTUK MESSAGE ====================
find_package(ament_cmake REQUIRED)  # Build system utama ROS2
find_package(std_msgs REQUIRED)  # Dependency message std_msgs (Header, dsb)
find_package(rosidl_default_generators REQUIRED)  # Untuk generate interface message ROS2

# ==================== BUILD CUSTOM MESSAGE ====================
rosidl_generate_interfaces(${PROJECT_NAME}  # Macro untuk generate Python/C++ message
  "msg/Object3D.msg"  # Daftar file message yang ingin di-build
  DEPENDENCIES std_msgs  # Dependency message lain yang digunakan (WAJIB jika pakai std_msgs/Header)
)

# ==================== EXPORT DEPENDENCY RUNTIME ====================
ament_export_dependencies(rosidl_default_runtime std_msgs)  # Agar package lain bisa pakai message ini

# ==================== TESTING & LINTING (OPSIONAL, BEST PRACTICE) ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Linting otomatis saat testing (best practice CI)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright lint jika belum ada copyright
  set(ament_cmake_cpplint_FOUND TRUE)  # Skip cpplint jika belum ada repo git
  ament_lint_auto_find_test_dependencies()  # Otomatis cari dependency test
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()  # Macro wajib penutup package ROS2 agar bisa di-build dan diinstall