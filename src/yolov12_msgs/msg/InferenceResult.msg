string class_name         # Nama class hasil deteksi (misal: "person", "car"). WAJIB diisi, digunakan untuk label visualisasi, logger, dan evaluasi.
float32 confidence        # Confidence score hasil deteksi (0.0 - 1.0). WAJIB diisi, digunakan untuk thresholding dan evaluasi.
int64 top                 # Koordinat atas bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 left                # Koordinat kiri bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 bottom              # Koordinat bawah bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.
int64 right               # Koordinat kanan bounding box (pixel). WAJIB diisi, untuk cropping dan visualisasi.

int64 track_id            # (opsional) ID tracking objek (misal: DeepSORT, YOLO tracking). WAJIB diisi jika mode tracking aktif, -1 jika tidak ada.
int64 obb_angle           # (opsional) Sudut OBB (oriented bounding box) dalam derajat atau radian. WAJIB diisi jika mode OBB aktif, -1 jika tidak ada.
int64[] mask_indices      # (opsional) Indeks pixel mask segmentasi (flattened, misal: [0,1,2,...] atau format lain sesuai kebutuhan). WAJIB diisi jika mode segmentasi aktif, [] jika tidak ada.