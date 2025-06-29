cmake_minimum_required(VERSION 3.5)                  # [WAJIB] Versi minimum CMake untuk ROS2 Humble/Foxy+ (standar Humble >= 3.5). ERROR HANDLING: Fail-fast jika versi terlalu rendah.
project(yolov12_msgs)                                # [WAJIB] Nama package ROS2, harus sama dengan folder dan package.xml. ERROR HANDLING: Jika tidak sama, colcon build/launch akan error.

# ===================== COMPILER STANDARD SETTINGS =====================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                           # [BEST PRACTICE] Set standar C ke C99 jika belum diset. ERROR HANDLING: Hindari error build jika ada source C.
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                         # [BEST PRACTICE] Set standar C++ ke C++14 jika belum diset. ERROR HANDLING: Hindari error build pada integrasi C++ node lain.
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)      # [BEST PRACTICE] Aktifkan warning compile. ERROR HANDLING: Deteksi bug silent lebih awal.
endif()
if(EXISTS "/etc/nv_tegra_release")
  message(STATUS "Jetson platform detected, optimizing for CUDA/TensorRT")  # [INFO] Deteksi Jetson untuk debugging build.
  add_compile_options(-DJETSON_PLATFORM)             # [JETSON] Define untuk optimasi Jetson. ERROR HANDLING: Pastikan build optimal di Jetson Orin.
endif()

# ===================== FILE EXISTENCE VALIDATION =====================
set(MESSAGE_FILES
    "msg/InferenceResult.msg"
    "msg/Yolov12Inference.msg"
)                                                    # [WAJIB] Daftar file .msg yang harus ada. ERROR HANDLING: Validasi fail-fast jika file hilang.
foreach(msg_file ${MESSAGE_FILES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${msg_file}")
    message(FATAL_ERROR "Message file tidak ditemukan: ${msg_file}. Pastikan semua file .msg ada di folder msg/.") # [ERROR HANDLING] Fail-fast jika file .msg hilang.
  endif()
endforeach()

# ===================== FIND DEPENDENCIES =====================
find_package(ament_cmake REQUIRED)                   # [WAJIB] Build system utama ROS2. ERROR HANDLING: Build gagal jika tidak ada.
find_package(std_msgs REQUIRED)                      # [WAJIB] Dependency std_msgs/Header. ERROR HANDLING: Build gagal jika tidak ada.
find_package(builtin_interfaces REQUIRED)            # [WAJIB] Dependency interface ROS2. ERROR HANDLING: Build gagal jika tidak ada.
find_package(rosidl_default_generators REQUIRED)     # [WAJIB] Generator interface .msg. ERROR HANDLING: Build gagal jika tidak ada.
find_package(sensor_msgs REQUIRED)                   # [WAJIB] Untuk integrasi sensor_msgs (PointCloud2, dsb). ERROR HANDLING: Build gagal jika tidak ada.

message(STATUS "Building yolov12_msgs for ROS2 Humble integration")  # [INFO] Log status build untuk tracking/debugging.

# ===================== BUILD CUSTOM MESSAGE =====================
set(GENERATE_MESSAGE_VERBOSE TRUE)                   # [DEBUG] Aktifkan output detail saat generate message.
rosidl_generate_interfaces(${PROJECT_NAME}           # [WAJIB] Macro generate Python/C++ message dari .msg.
    ${MESSAGE_FILES}                                 # [WAJIB] Daftar file .msg yang sudah divalidasi.
    DEPENDENCIES std_msgs sensor_msgs                # [WAJIB] Dependency message lain yang digunakan di .msg.
    ADD_LINTER_TESTS                                 # [BEST PRACTICE] Linter test untuk validasi syntax .msg.
 )

# Export dependencies untuk packages lain yang depend pada package ini
ament_export_dependencies(
  rosidl_default_runtime 
  std_msgs 
  builtin_interfaces 
  sensor_msgs
)                                                    # [WAJIB] Export dependency runtime agar package lain bisa resolve message ini.

# ===================== TESTING & LINTING (BEST PRACTICE) =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)             # [BEST PRACTICE] Linting otomatis saat testing (CI/CD).
  # set(ament_cmake_copyright_FOUND TRUE)            # [OPSIONAL] Skip copyright linter.
  # set(ament_cmake_cpplint_FOUND TRUE)              # [OPSIONAL] Skip cpplint jika bukan git repo.
  ament_lint_auto_find_test_dependencies()           # [BEST PRACTICE] Otomatis cari dependency linting untuk testing.
endif()

# ===================== INSTALL FILES & DIRECTORIES =====================
install(FILES
  README.md                                          # [BEST PRACTICE] Install README.md ke share/package.
  DESTINATION share/${PROJECT_NAME}
)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  message(WARNING "Folder test/ tidak ditemukan. Sebaiknya buat folder test/ dengan unit test message.")  # [ERROR HANDLING] Warning jika tidak ada test.
else()
  install(DIRECTORY
    test/                                            # [BEST PRACTICE] Install folder test/ jika ada.
    DESTINATION share/${PROJECT_NAME}/test
    OPTIONAL
  )
endif()
install(DIRECTORY
  config/                                            # [OPSIONAL] Folder config untuk kalibrasi/fusion.
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  install(DIRECTORY
    test/integration/                                # [OPSIONAL] Folder integration test.
    DESTINATION share/${PROJECT_NAME}/test/integration
    OPTIONAL
  )
endif()

# ===================== AMENT PACKAGE (WAJIB) =====================
ament_package()                                      # [WAJIB] Macro penutup package agar bisa di-build dan diinstall.

# ===================== ERROR HANDLING CHECKS SUMMARY =====================
# 1. Validasi existensi file .msg sebelum build (fail-fast with clear message)
# 2. REQUIRED flag pada semua find_package untuk dependency checking
# 3. Platform detection untuk Jetson vs PC reguler
# 4. Warning untuk missing test directory
# 5. OPTIONAL flag pada install untuk non-critical paths
# 6. ADD_LINTER_TESTS untuk validate message format
# 7. Message status logging untuk debug
# 8. Conditional paths untuk test dan integration
# 9. Export lengkap semua dependency yang diperlukan runtime
# 10. Komentar detail untuk semua error case dan solusinya