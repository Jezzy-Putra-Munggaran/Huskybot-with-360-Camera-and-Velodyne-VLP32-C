string class_name   # Nama kelas objek hasil deteksi (misal: "person", "car", "dog", dst)
float32 confidence  # Skor confidence deteksi (0.0 - 1.0)
int64 top           # Koordinat atas bounding box (pixel, biasanya y1)
int64 left          # Koordinat kiri bounding box (pixel, biasanya x1)
int64 bottom        # Koordinat bawah bounding box (pixel, biasanya y2)
int64 right         # Koordinat kanan bounding box (pixel, biasanya x2)