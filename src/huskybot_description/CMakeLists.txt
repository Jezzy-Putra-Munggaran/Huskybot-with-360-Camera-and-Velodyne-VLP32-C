cmake_minimum_required(VERSION 3.5)                   # Versi minimum CMake yang dibutuhkan (standar ROS2 Humble)
project(huskybot_description)                          # Nama package, harus sama dengan folder dan package.xml

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                             # Set standar C ke C99 jika belum diset (untuk file C, antisipasi jika ada C)
endif()

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
find_package(nav_msgs REQUIRED)                        # Dependency pesan navigasi (Path, Odometry, dsb)
find_package(geometry_msgs REQUIRED)                   # Dependency pesan geometri (Twist, Pose, dsb)
# find_package(launch REQUIRED)                        # HAPUS: launch adalah package Python, TIDAK perlu find_package di CMake!
# find_package(launch_ros REQUIRED)                    # HAPUS: launch_ros adalah package Python, TIDAK perlu find_package di CMake!
# find_package(ament_index_python REQUIRED)            # HAPUS: ament_index_python adalah modul Python, TIDAK perlu find_package di CMake!
# Tambahkan dependency lain jika ada node custom lain (misal: image_transport, tf2_ros, dsb)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)               # Untuk linting otomatis saat testing (best practice CI/CD)
  ament_lint_auto_find_test_dependencies()             # Cari dependency test lint otomatis (otomatis lint semua file)

  # Test validasi Xacro/URDF (fail build jika URDF/Xacro tidak valid)
  file(GLOB XACRO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/robot/*.xacro") # Cari semua file xacro
  foreach(xacro_file ${XACRO_FILES})
    find_program(XACRO_EXECUTABLE xacro)
    if(XACRO_EXECUTABLE)
      add_test(
        NAME xacro_check_${xacro_file}
        COMMAND ${XACRO_EXECUTABLE} --check-order ${xacro_file}
      )                                                # Test: validasi semua file xacro (fail jika error sintaks/urutan)
    endif()
  endforeach()
  file(GLOB URDF_FILES "${CMAKE_CURRENT_SOURCE_DIR}/robot/*.urdf" "${CMAKE_CURRENT_SOURCE_DIR}/robot/*.xacro")
  foreach(urdf_file ${URDF_FILES})
    find_program(CHECK_URDF_EXECUTABLE check_urdf)
    if(CHECK_URDF_EXECUTABLE)
      add_test(
        NAME urdf_check_${urdf_file}
        COMMAND ${CHECK_URDF_EXECUTABLE} ${urdf_file}
      )                                                # Test: validasi semua hasil URDF dari xacro (fail jika error URDF)
    endif()
  endforeach()
  # Saran: Tambahkan test launch file di folder test/ untuk CI/CD (opsional, best practice)
endif()

# Variabel folder agar maintainable
set(LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/launch)     # Folder launch file Python
set(ROBOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/robot)       # Folder URDF/Xacro robot
set(CALIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/calibration) # Folder file kalibrasi sensor
set(MESHES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/meshes)     # Folder mesh visual sensor/robot
set(RVIZ_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rviz)         # Folder file konfigurasi RViz

install(
  DIRECTORY
    ${LAUNCH_DIR}                                      # Install folder launch ke share/package/
    ${ROBOT_DIR}                                       # Install folder robot ke share/package/
    ${CALIB_DIR}                                       # Install folder calibration ke share/package/
    ${MESHES_DIR}                                      # Install folder meshes ke share/package/
    ${RVIZ_DIR}                                        # Install folder rviz ke share/package/
  DESTINATION
    share/${PROJECT_NAME}/                             # Semua diinstall ke share/package_name/ (standar ROS2)
)

install(
  PROGRAMS
    ${LAUNCH_DIR}/spawn_huskybot.py                    # Install script Python agar bisa dieksekusi dari ros2 run (pastikan chmod +x)
    ${LAUNCH_DIR}/spawn_huskybot_launch.launch.py      # Install script Python agar bisa dieksekusi dari ros2 run (pastikan chmod +x)
  DESTINATION lib/${PROJECT_NAME}                      # Script Python diinstall ke lib/package_name/ (standar ROS2)
)

# Pastikan semua script Python sudah chmod +x sebelum build (otomatis, cegah error permission)
install(CODE "execute_process(COMMAND chmod +x ${LAUNCH_DIR}/spawn_huskybot.py)")              # Cek permission script
install(CODE "execute_process(COMMAND chmod +x ${LAUNCH_DIR}/spawn_huskybot_launch.launch.py)")# Cek permission script

# Install README.md ke share agar dokumentasi bisa diakses offline
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md DESTINATION share/${PROJECT_NAME}/)        # Install README ke share

# Install semua file YAML kalibrasi secara eksplisit (jika ada file baru)
file(GLOB CALIB_YAML_FILES "${CALIB_DIR}/*.yaml")                                             # Cari semua file YAML kalibrasi
install(FILES ${CALIB_YAML_FILES} DESTINATION share/${PROJECT_NAME}/calibration)               # Install file YAML ke share

# Saran: Tambahkan install CHANGELOG.md dan CONTRIBUTING.md jika ada (untuk dokumentasi dan kontribusi)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.md")
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.md DESTINATION share/${PROJECT_NAME}/)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CONTRIBUTING.md")
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/CONTRIBUTING.md DESTINATION share/${PROJECT_NAME}/)
endif()

# Saran: Tambahkan folder test/ jika ingin unit test/launch test otomatis (best practice CI/CD)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()                                       # Macro wajib ROS2 agar package dikenali oleh ament (wajib di akhir)