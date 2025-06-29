# Deskripsi: Message untuk menyimpan satu hasil deteksi YOLOv12 (single bounding box dengan kelasnya)  # WAJIB: Penjelasan fungsi utama
# Digunakan di: huskybot_detection, huskybot_segmentation, huskybot_fusion, huskybot_perception  # WAJIB: Daftar package pengguna
# Kompatibel dengan: ROS2 Humble, Gazebo, yolo12x.engine (detection), yolo11x-seg.engine (segmentation)  # WAJIB: Kompatibilitas
# Update terakhir: 2025-06-23  # WAJIB: Tanggal update

# ===================== FIELD WAJIB (SELALU DIISI) =====================
string class_name         # Nama class hasil deteksi (misal: "person", "car"). WAJIB diisi, digunakan untuk label visualisasi, logger, dan evaluasi.
float32 confidence        # Confidence score hasil deteksi (0.0 - 1.0). WAJIB diisi, digunakan untuk thresholding dan evaluasi.
int64 top                 # Koordinat atas bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 left                # Koordinat kiri bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 bottom              # Koordinat bawah bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 right               # Koordinat kanan bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.

# ===================== FIELD OPSIONAL (TERGANTUNG MODE YOLO) =====================
int64 track_id            # ID tracking objek (misal: DeepSORT, YOLO tracking). WAJIB diisi jika mode tracking aktif, -1 jika tidak ada.
int64 obb_angle           # Sudut OBB (oriented bounding box) dalam derajat. WAJIB diisi jika mode OBB aktif, -1 jika tidak ada.
int64[] mask_indices      # Indeks pixel mask segmentasi (flattened). WAJIB diisi jika mode segmentasi aktif, [] jika tidak ada.

# ===================== ERROR HANDLING & BEST PRACTICE =====================
# - Field wajib (class_name, confidence, top, left, bottom, right) harus selalu diisi
# - Untuk field opsional: track_id dan obb_angle diisi -1 jika tidak digunakan
# - Untuk field opsional: mask_indices diisi [] (array kosong) jika tidak digunakan
# - Jika confidence di bawah threshold yang diinginkan, objek seharusnya tidak masuk ke message ini
# - Validasi semua field di subscriber untuk hindari error null/div-by-zero
# - Bounding box harus valid (top < bottom, left < right) dan dalam batasan gambar (0 <= x,y <= image_size)
# - Gunakan validation function di setiap node yang menggunakan message ini (contoh: validate_inference_result())
# - Untuk multi-task (misal: deteksi + tracking), isi semua field yang relevan sesuai task
# - Untuk fusion dengan LiDAR, gunakan note field di Yolov12Inference untuk menyimpan data jarak dan koordinat
# - Hilangkan field WAJIB kosong dapat menyebabkan crash pada node visualization dan logger