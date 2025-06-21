string class_name
float32 confidence
int64 top
int64 left
int64 bottom
int64 right
# Tambahkan field berikut:
int64 track_id         # (opsional) ID tracking objek
int64 obb_angle        # (opsional) Sudut OBB (derajat/radian)
int64[] mask_indices   # (opsional) Indeks pixel mask segmentasi (atau gunakan format lain sesuai kebutuhan)