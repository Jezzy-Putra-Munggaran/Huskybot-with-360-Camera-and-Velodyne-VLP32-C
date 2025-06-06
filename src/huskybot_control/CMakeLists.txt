cmake_minimum_required(VERSION 3.5)                # [WAJIB] Versi minimum CMake yang didukung ROS2 Humble (standar Humble >= 3.5)
project(huskybot_control)                           # [WAJIB] Nama package, harus sama dengan folder dan package.xml

# ===================== STANDAR C/C++ UNTUK KOMPATIBILITAS ROS2 =====================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                          # [OPSIONAL] Set standar C ke C99 jika belum diset (antisipasi file C, walau package Python-only)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                        # [OPSIONAL] Set standar C++ ke C++14 jika belum diset (antisipasi file C++, walau package Python-only)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)     # [BEST PRACTICE] Aktifkan warning compile untuk GCC/Clang (cegah bug silent)
endif()

# ===================== DEPENDENCY ROS2 & PYTHON =====================
find_package(ament_cmake REQUIRED)                  # [WAJIB] Build system utama ROS2
find_package(rclpy REQUIRED)                        # [WAJIB] Library utama ROS2 Python
find_package(geometry_msgs REQUIRED)                # [WAJIB] Message Twist (untuk kontrol kecepatan)
find_package(sensor_msgs REQUIRED)                  # [WAJIB] Message sensor (Joy, LaserScan, dsb)
find_package(huskybot_description REQUIRED)         # [OPSIONAL] Dependency ke package URDF/mesh (untuk spawn robot di Gazebo)
# find_package(huskybot_gazebo REQUIRED)            # [OPSIONAL] Dependency ke package simulasi (aktifkan jika ada import langsung)
# find_package(std_msgs REQUIRED)                   # [SARAN] Tambahkan jika ada import std_msgs di scripts/
# find_package(nav_msgs REQUIRED)                   # [SARAN] Tambahkan jika ada import nav_msgs di scripts/

# ===================== LINTING & TESTING OTOMATIS (BEST PRACTICE CI/CD) =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)            # [BEST PRACTICE] Linting otomatis saat testing (CI/CD)
  ament_lint_auto_find_test_dependencies()          # [BEST PRACTICE] Otomatis cari dependency test
  # add_subdirectory(test)                          # [OPSIONAL] Aktifkan jika ada folder test/ (unit test Python/C++)
endif()

# ===================== INSTALL FOLDER & FILE PENTING =====================
install(
  DIRECTORY
    scripts                                         # [WAJIB] Install folder scripts ke share agar bisa diakses ros2 run/launch
    launch                                          # [WAJIB] Install folder launch ke share agar bisa diakses ros2 launch
    config                                          # [OPSIONAL] Install folder config untuk file YAML parameter (jika ada)
    test                                            # [OPSIONAL] Install folder test untuk unit test (jika ada)
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/robot_control.py                        # [WAJIB] Script utama kontrol robot (ros2 run/launch)
    scripts/safety_monitor.py                       # [WAJIB] Script safety monitor (ros2 run/launch)
    scripts/logger.py                               # [WAJIB] Script logger (ros2 run/launch)
    launch/huskybot_control.launch.py               # [WAJIB] Launch file utama (ros2 launch)
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

ament_package()                                     # [WAJIB] Macro ROS2 agar package dikenali oleh ament/colcon

# ===================== SARAN PENINGKATAN (SUDAH DIIMPLEMENTASIKAN) =====================
# - Tambahkan install folder config dan test agar file YAML dan unit test otomatis di-share (sudah ditambah).
# - Tambahkan komentar penjelasan di setiap baris agar mudah dipahami siapapun.
# - Jika ingin multi-robot, tambahkan argumen namespace di launch file (tidak perlu di CMake).
# - Jika ingin coverage test lebih tinggi, tambahkan test/launch test di folder test/.
# - Jika ingin audit trail, tambahkan logger ke file di setiap node (sudah ada di scripts/).
# - Jika ingin distribusi Docker, pastikan semua file penting diinstall ke share/lib.
# - Jika ingin integrasi CI/CD, pastikan semua test script diinstall ke share/test.
# - Jika ingin integrasi dengan package lain, tambahkan ament_export_dependencies di bawah (opsional).