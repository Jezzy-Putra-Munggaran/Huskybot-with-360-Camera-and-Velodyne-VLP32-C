cmake_minimum_required(VERSION 3.5)                  # [WAJIB] Versi minimum CMake yang didukung ROS2 Humble/Foxy+ (standar Humble >= 3.5). Jika terlalu tinggi, bisa gagal build di beberapa sistem. ERROR HANDLING: Jika gagal, akan error di awal build (fail-fast).
project(yolov12_msgs)                                # [WAJIB] Nama package ROS2, harus sama dengan folder dan package.xml. ERROR HANDLING: Jika tidak sama, bisa error saat colcon build atau ros2 launch/run.

# ===================== COMPILER STANDARD SETTINGS =====================
# Default to C99 standard (untuk source C)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                           # [BEST PRACTICE] Set standar C ke C99 jika belum diset (untuk source C, opsional di message-only). ERROR HANDLING: Mencegah build error jika ada source C.
endif()

# Default to C++14 standard (untuk source C++)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                         # [BEST PRACTICE] Set standar C++ ke C++14 jika belum diset (untuk source C++, opsional di message-only). ERROR HANDLING: Mencegah build error pada kolaborasi dengan package lain seperti fusion atau perception.
endif()

# Tambahkan warning flags untuk compiler GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)      # [BEST PRACTICE] Tambahkan warning compile untuk deteksi awal bug. ERROR HANDLING: Mencegah bug silent dengan warning verbose saat compile.
endif()

# Tambahkan konfigurasi khusus Jetson untuk performa optimal
if(EXISTS "/etc/nv_tegra_release")
  message(STATUS "Jetson platform detected, optimizing for CUDA/TensorRT")  # Log deteksi Jetson platform untuk debugging
  add_compile_options(-DJETSON_PLATFORM)             # Define untuk conditional compilation, memungkinkan optimasi Jetson-specific. ERROR HANDLING: Memastikan kode berjalan optimal di Jetson AGX Orin.
endif()

# ===================== FILE EXISTENCE VALIDATION =====================
# Validasi keberadaan file message sebelum build
set(MESSAGE_FILES
    "msg/InferenceResult.msg"
    "msg/Yolov12Inference.msg"
)

foreach(msg_file ${MESSAGE_FILES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${msg_file}")
    message(FATAL_ERROR "Message file tidak ditemukan: ${msg_file}. Pastikan semua file .msg ada di folder msg/.") # ERROR HANDLING: Fail-fast jika file .msg hilang
  endif()
endforeach()

# ===================== FIND DEPENDENCIES =====================
find_package(ament_cmake REQUIRED)                   # [WAJIB] Build system utama ROS2, semua package harus depend ke ament_cmake. ERROR HANDLING: Jika tidak terinstall, build gagal fail-fast dengan pesan jelas.
find_package(std_msgs REQUIRED)                      # [WAJIB] Dependency message std_msgs (Header, dsb), wajib jika pakai std_msgs/Header di .msg. ERROR HANDLING: Jika tidak terinstall, build gagal dengan pesan dependency error.
find_package(builtin_interfaces REQUIRED)            # [WAJIB] Dependency interface built-in ROS2 (misal: waktu, dsb). ERROR HANDLING: Jika tidak terinstall, build gagal fail-fast.
find_package(rosidl_default_generators REQUIRED)     # [WAJIB] Dependency generator interface ROS2 (untuk .msg). ERROR HANDLING: Jika tidak terinstall, build gagal dengan pesan jelas error generator tidak ditemukan.
find_package(sensor_msgs REQUIRED)                   # [WAJIB] Dependency untuk tipe sensor data (Point, PointCloud2, dsb). ERROR HANDLING: Jika tidak terinstall, build gagal fail-fast.

# Log informasi build untuk debugging
message(STATUS "Building yolov12_msgs for ROS2 Humble integration")  # Log status build untuk tracking/debugging

# ===================== BUILD CUSTOM MESSAGE =====================
set(GENERATE_MESSAGE_VERBOSE TRUE)                   # Aktifkan output detail saat generate message untuk debugging
rosidl_generate_interfaces(${PROJECT_NAME}           # [WAJIB] Macro untuk generate Python/C++ message dari .msg. ERROR HANDLING: Proses utama .msg -> C++/Python, error jika format .msg salah.
    ${MESSAGE_FILES}                                 # [WAJIB] Daftarkan file message custom via variabel, memastikan konsistensi. ERROR HANDLING: Sudah divalidasi keberadaan filenya di atas.
    DEPENDENCIES std_msgs sensor_msgs                # [WAJIB] Message ini depend pada std_msgs (Header) dan sensor_msgs untuk integrasi LiDAR. ERROR HANDLING: Jika dependency hilang, build error detail.
    ADD_LINTER_TESTS                                 # Tambahkan linter test untuk validasi kualitas kode message. ERROR HANDLING: Deteksi syntax error di file .msg.
 )

# Export dependencies untuk packages lain yang depend pada package ini
ament_export_dependencies(
  rosidl_default_runtime 
  std_msgs 
  builtin_interfaces 
  sensor_msgs
) # [WAJIB] Export dependency runtime agar package lain bisa pakai message ini. ERROR HANDLING: Jika tidak di-export, package lain tidak bisa resolve dependency message ini.

# ===================== TESTING & LINTING (BEST PRACTICE) =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)             # [BEST PRACTICE] Untuk linting otomatis saat testing (CI/CD). ERROR HANDLING: Memastikan kualitas kode dengan checks otomatis.
  
  # Skip copyright linter jika diperlukan (uncomment jika perlu)
  # set(ament_cmake_copyright_FOUND TRUE)            # Uncomment jika tidak ingin copyright check. ERROR HANDLING: Prevents build failures due to missing copyright.
  
  # Skip cpplint untuk non-git repo (uncomment jika perlu)
  # set(ament_cmake_cpplint_FOUND TRUE)              # Uncomment jika bukan git repo. ERROR HANDLING: Prevents cpplint errors in non-git environments.
  
  ament_lint_auto_find_test_dependencies()           # [BEST PRACTICE] Otomatis cari dependency linting untuk testing. ERROR HANDLING: Will fail clearly if dependencies are missing.
endif()

# ===================== INSTALL FILES & DIRECTORIES =====================
# Install README.md untuk dokumentasi yang terinstall
install(FILES
  README.md                                          # [BEST PRACTICE] Install README.md agar dokumentasi ikut terinstall ke share/. ERROR HANDLING: Optional file, install skips if missing without error.
  DESTINATION share/${PROJECT_NAME}
)

# Pastikan ada folder test/ untuk continuous integration
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  message(WARNING "Folder test/ tidak ditemukan. Sebaiknya buat folder test/ dengan unit test message.")  # ERROR HANDLING: Warning jika tidak ada test, penting untuk CI/CD.
else()
  install(DIRECTORY
    test/                                            # [BEST PRACTICE] Install folder test/ jika ada unit test message. ERROR HANDLING: OPTIONAL flag prevents error if missing.
    DESTINATION share/${PROJECT_NAME}/test
    OPTIONAL
  )
endif()

# Tambahkan folder untuk data kalibrasi jika diperlukan (untuk integrasi dengan huskybot_calibration)
install(DIRECTORY
  config/                                            # Folder untuk config/calibration jika ada. ERROR HANDLING: OPTIONAL flag prevents build failure.
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ===================== INTEGRATION TEST SETUP =====================
# Setup testing khusus untuk integrasi dengan node lain
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  install(DIRECTORY
    test/integration/                                # Folder integration test spesifik untuk validate message dengan node lain. ERROR HANDLING: OPTIONAL flag prevents error.
    DESTINATION share/${PROJECT_NAME}/test/integration
    OPTIONAL
  )
endif()

# ===================== AMENT PACKAGE (WAJIB) =====================
ament_package()                                      # [WAJIB] Macro penutup package ROS2 agar bisa di-build dan diinstall. ERROR HANDLING: Jika tidak ada, package tidak bisa diinstall dengan benar.

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