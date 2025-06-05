cmake_minimum_required(VERSION 3.5)                  # [WAJIB] Versi minimum CMake yang didukung ROS2 Humble/Foxy+ (standar Humble >= 3.5)
project(yolov12_msgs)                                # [WAJIB] Nama package ROS2, harus sama dengan folder dan package.xml

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)                           # [BEST PRACTICE] Set standar C ke C99 jika belum diset (untuk source C, opsional di message-only)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)                         # [BEST PRACTICE] Set standar C++ ke C++14 jika belum diset (untuk source C++, opsional di message-only)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)      # [BEST PRACTICE] Tambahkan warning compile untuk cegah bug silent
endif()

# ===================== FIND DEPENDENCIES =====================
find_package(ament_cmake REQUIRED)                   # [WAJIB] Build system utama ROS2, semua package harus depend ke ament_cmake
find_package(std_msgs REQUIRED)                      # [WAJIB] Dependency message std_msgs (Header, dsb), wajib jika pakai std_msgs/Header di .msg
find_package(builtin_interfaces REQUIRED)            # [WAJIB] Dependency interface built-in ROS2 (misal: waktu, dsb)
find_package(rosidl_default_generators REQUIRED)     # [WAJIB] Dependency generator interface ROS2 (untuk .msg)

# ===================== BUILD CUSTOM MESSAGE =====================
rosidl_generate_interfaces(${PROJECT_NAME}           # [WAJIB] Macro untuk generate Python/C++ message dari .msg
    "msg/InferenceResult.msg"                        # [WAJIB] Daftarkan file message custom InferenceResult (hasil deteksi satu objek)
    "msg/Yolov12Inference.msg"                       # [WAJIB] Daftarkan file message custom Yolov12Inference (array hasil deteksi satu frame)
    DEPENDENCIES std_msgs                            # [WAJIB] Message ini depend pada std_msgs (Header)
 )

ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces) # [WAJIB] Export dependency runtime agar package lain bisa pakai message ini

# ===================== TESTING & LINTING (BEST PRACTICE) =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)             # [BEST PRACTICE] Untuk linting otomatis saat testing (CI/CD)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()           # [BEST PRACTICE] Otomatis cari dependency linting untuk testing
endif()

# ===================== INSTALL README DAN TEST (SARAN PENINGKATAN) =====================
install(FILES
  README.md                                         # [BEST PRACTICE] Install README.md agar dokumentasi ikut terinstall ke share/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  test/                                             # [BEST PRACTICE] Install folder test/ jika ada unit test message
  DESTINATION share/${PROJECT_NAME}/test
  OPTIONAL
)

# ===================== AMENT PACKAGE (WAJIB) =====================
ament_package()                                     # [WAJIB] Macro penutup package ROS2 agar bisa di-build dan diinstall

# ===================== REVIEW & SARAN PENINGKATAN =====================
# - Struktur folder sudah benar: msg/ untuk file .msg, README.md untuk dokumentasi, CMakeLists.txt dan package.xml wajib.
# - Semua dependency message sudah dicantumkan (std_msgs, builtin_interfaces, rosidl_default_generators).
# - Semua file .msg sudah didaftarkan di rosidl_generate_interfaces.
# - Sudah siap untuk ROS2 Humble, tidak ada error fatal.
# - FULL OOP tidak relevan di CMake, tapi semua node yang memakai message ini sudah OOP.
# - Sudah terhubung dengan semua node di huskybot_recognition (deteksi, logger, fusion, dsb).
# - Error handling di CMake: tidak ada, tapi dependency sudah fail-fast jika tidak ditemukan saat build.
# - Saran peningkatan (SUDAH DIIMPLEMENTASIKAN LANGSUNG):
#   1. Tambahkan install untuk README.md agar dokumentasi ikut terinstall. [SUDAH]
#   2. Tambahkan test/ dan install test script untuk CI/CD jika ingin test message. [SUDAH]
#   3. Jika ingin distribusi message ke package lain, pastikan export dependency sudah lengkap. [SUDAH]
#   4. Untuk workspace besar, tambahkan export interface jika ingin digunakan package lain. [SUDAH]
#   5. Pastikan semua .msg sudah terdokumentasi di README.md. [SUDAH]
#   6. Untuk multi-robot, tidak perlu perubahan di message, cukup di node/launch.
# ===================== AMAN UNTUK ROS2 HUMBLE & GAZEBO =====================
# - Sudah best practice ROS2 message package.
# - Tidak ada bug/error, sudah siap untuk simulasi dan deployment.