# Deskripsi: Message untuk menyimpan hasil deteksi YOLOv12 dari satu frame/gambar
# Digunakan di: huskybot_detection, huskybot_segmentation, huskybot_fusion, huskybot_perception
# Kompatibel dengan: ROS2 Humble, Gazebo, yolo12x.engine (detection), yolo11x-seg.engine (segmentation)
# Update terakhir: 2025-06-23
# Versi: 1.0

# ===================== FIELD WAJIB (SELALU DIISI) =====================
std_msgs/Header header           # Header ROS2 standar (waktu, frame_id). WAJIB diisi. Untuk sinkronisasi waktu, TF, dan integrasi multi-sensor. Timestamp harus valid (tidak terlalu lama dari waktu sekarang). Frame_id harus valid (ada di TF tree), misal: "camera_front", "lidar", "map", dsb.
InferenceResult[] yolov12_inference  # Array hasil deteksi (list InferenceResult). WAJIB diisi (bisa array kosong jika tidak ada deteksi). Satu frame bisa punya banyak deteksi (0-N). Dipakai semua node multicam, logger, fusion, visualizer.
string camera_name               # Nama kamera/sumber deteksi (misal: "camera_front", "camera_front_left", "camera_left", "camera_rear", "camera_rear_right", "camera_right", "panorama", dsb). WAJIB diisi. Untuk multi-kamera, panorama, dan audit trail. Digunakan logger, fusion, dsb.

# ===================== FIELD OPSIONAL (BISA KOSONG) =====================
string frame_type                # Jenis frame: salah satu dari ["raw", "stitch", "panorama", "warped", "augmented"]. Untuk identifikasi pipeline (misal: hasil stitching/panorama). Default: "raw".
string task                      # Jenis task YOLO: salah satu dari ["detect", "segment", "classify", "obb", "track"]. Untuk identifikasi jenis inference. Default: "detect".
string note                      # Catatan tambahan (misal: kondisi sensor, error, data fusion LiDAR, dsb). Untuk debugging, audit, atau metadata (distance, koordinat xyz, dsb). Bisa dikosongkan.

# ===================== ERROR HANDLING & BEST PRACTICE =====================
# - Field wajib (header, yolov12_inference, camera_name) harus selalu diisi, tidak boleh null/empty
# - Header.stamp harus valid, tidak lebih dari beberapa detik dari waktu sekarang (threshold sesuai kebutuhan)
# - Header.frame_id harus valid dan ada di TF tree, pastikan konsisten dengan camera_name
# - camera_name harus salah satu dari daftar kamera yang valid di sistem
# - frame_type harus salah satu dari ["raw", "stitch", "panorama", "warped", "augmented"]
# - task harus salah satu dari ["detect", "segment", "classify", "obb", "track"]
# - Array yolov12_inference boleh kosong (tidak ada deteksi), tapi tidak boleh null
# - Semua node subscriber harus validasi message sebelum diproses (misal: validate_yolov12_inference())
# - Jika integrasi dengan RViz, pastikan frame_id sesuai dengan frame yang ada di TF tree
# - Untuk fusion dengan LiDAR, simpan data jarak (meter) dan koordinat [x, y, z] di field note (format JSON atau key-value)
# - Jika menggunakan namespace untuk multi-robot, topic lebih baik prefixed dengan namespace robot
# - Waktu publish/subscribe sebaiknya ditambahkan QoS setting reliability (best effort untuk kamera)
# - Validasi timestamp untuk deteksi message yang terlalu lama (reject jika timestamp > threshold)
# - Batasi ukuran array inference untuk mencegah overload jaringan/memori (batasi max deteksi per frame, misal max 100)
# - Jangan gunakan field note untuk data besar atau binary data (gunakan topic terpisah)
# - Untuk error/warning di sistem, gunakan format standard di note: "ERROR: [kode]" atau "WARN: [kode]"
# - Pastikan semua node pemakai message ini melakukan validasi terhadap nilai yang diterima

# ===================== FORMAT FIELD NOTE UNTUK FUSION =====================
# Format untuk fusion dengan LiDAR (disarankan):
# {
#   "distance": 5.23,         # Jarak dalam meter
#   "coordinate": [2.1, 0.5, 1.2],  # Koordinat [x, y, z] dalam meter
#   "lidar_frame": "velodyne",      # Frame LiDAR yang digunakan
#   "confidence_3d": 0.85,          # Confidence koordinat 3D (opsional)
#   "timestamp_fusion": 1685432876  # Timestamp saat fusion (opsional)
# }