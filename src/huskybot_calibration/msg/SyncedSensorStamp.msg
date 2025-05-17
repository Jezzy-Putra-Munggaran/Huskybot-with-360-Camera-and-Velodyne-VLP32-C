std_msgs/Header header           # Header standar ROS2, wajib untuk sinkronisasi dan timestamp utama
# Digunakan untuk timestamp utama dan frame_id sinkronisasi antar node ROS2.

builtin_interfaces/Time camera_stamp   # Timestamp dari sensor kamera (Image.header.stamp)
# Waktu pengambilan data kamera, penting untuk sinkronisasi dan analisis delay.

builtin_interfaces/Time lidar_stamp    # Timestamp dari sensor LiDAR (PointCloud2.header.stamp)
# Waktu pengambilan data LiDAR, penting untuk sinkronisasi dan analisis delay.

builtin_interfaces/Time imu_stamp      # Timestamp dari sensor IMU (Imu.header.stamp)
# Waktu pengambilan data IMU, penting untuk sinkronisasi dan analisis delay.

string camera_frame_id           # Nama frame kamera (misal: "panorama_camera_link")
# Nama frame kamera, harus konsisten dengan URDF/Xacro dan node lain.

string lidar_frame_id            # Nama frame LiDAR (misal: "velodyne_link")
# Nama frame LiDAR, harus konsisten dengan URDF/Xacro dan node lain.

string imu_frame_id              # Nama frame IMU (misal: "imu_link")
# Nama frame IMU, harus konsisten dengan URDF/Xacro dan node lain.

bool is_synced                   # True jika data benar-benar sinkron (bisa diisi di node Python)
# Flag untuk menandai apakah data benar-benar sinkron (misal, selisih waktu < threshold).

float32 sync_quality             # Nilai kualitas sinkronisasi (misal: selisih waktu max antar sensor, dalam detik)
# Nilai kualitas sinkronisasi, semakin kecil semakin baik (0 = sempurna).

geometry_msgs/Pose camera_pose      # Pose kamera (jika tersedia, misal dari extrinsic atau odom)
# Pose kamera dalam world/base_link, bisa diisi dari TF atau hasil kalibrasi.

geometry_msgs/Pose lidar_pose       # Pose LiDAR (jika tersedia)
# Pose LiDAR dalam world/base_link, bisa diisi dari TF atau hasil kalibrasi.

geometry_msgs/Pose imu_pose         # Pose IMU (jika tersedia)
# Pose IMU dalam world/base_link, bisa diisi dari TF atau hasil kalibrasi.

geometry_msgs/Twist imu_velocity    # Velocity IMU (jika tersedia, misal dari /imu/data)
# Data kecepatan IMU, bisa diisi dari /imu/data jika tersedia.

float64[36] camera_covariance       # Covariance pose kamera (jika tersedia, urutan row-major)
# Matriks kovarians pose kamera, penting untuk analisis ketidakpastian.

float64[36] lidar_covariance        # Covariance pose LiDAR (jika tersedia)
# Matriks kovarians pose LiDAR, penting untuk analisis ketidakpastian.

float64[36] imu_covariance          # Covariance pose IMU (jika tersedia)
# Matriks kovarians pose IMU, penting untuk analisis ketidakpastian.

uint8 camera_status                 # Status sensor kamera (0: OK, 1: WARNING, 2: ERROR, dst)
# Status kesehatan sensor kamera, bisa digunakan untuk monitoring dan error handling.

uint8 lidar_status                  # Status sensor LiDAR
# Status kesehatan sensor LiDAR, bisa digunakan untuk monitoring dan error handling.

uint8 imu_status                    # Status sensor IMU
# Status kesehatan sensor IMU, bisa digunakan untuk monitoring dan error handling.

string camera_status_detail         # (Opsional) Pesan error/detail status kamera
# Penjelasan error atau status detail kamera, untuk debugging lebih mudah.

string lidar_status_detail          # (Opsional) Pesan error/detail status LiDAR
# Penjelasan error atau status detail LiDAR, untuk debugging lebih mudah.

string imu_status_detail            # (Opsional) Pesan error/detail status IMU
# Penjelasan error atau status detail IMU, untuk debugging lebih mudah.

uint8 sync_error_code               # (Opsional) Kode error sinkronisasi (0: OK, 1: TIMEOUT, 2: DELAY, dst)
# Kode error sinkronisasi, untuk monitoring dan troubleshooting otomatis.

float64[8] reserved                 # (Opsional) Field cadangan untuk future-proofing pipeline
# Field cadangan, bisa digunakan untuk data tambahan di masa depan.

# (Opsional, bisa tambah field lain sesuai kebutuhan pipeline)
# Tambahkan field lain jika pipeline bertambah (misal: status sinkronisasi lanjutan, data sensor lain, dsb).