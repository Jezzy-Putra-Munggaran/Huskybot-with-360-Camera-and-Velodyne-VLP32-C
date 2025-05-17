std_msgs/Header header         # [WAJIB] Header ROS2 (timestamp dan frame_id, untuk sinkronisasi waktu dan referensi frame). Penting agar message bisa diurutkan dan di-transform antar frame (misal: map, base_link, sensor_link). Digunakan oleh semua node yang subscribe/publish Object3D, baik di simulasi Gazebo maupun robot real.

string label                  # [WAJIB] Nama kelas objek hasil fusion (misal: "person", "car", dsb). Label ini diisi oleh node deteksi (YOLOv12, dsb) dan fusion, digunakan untuk identifikasi objek di pipeline workspace (recognition, fusion, logger, dsb).

float32[3] center             # [WAJIB] [x, y, z] Titik tengah bounding box 3D dalam meter (koordinat dunia/robot). Digunakan untuk posisi objek di dunia nyata/simulasi, penting untuk navigasi, visualisasi, dan evaluasi deteksi.

float32[3] size               # [WAJIB] [dx, dy, dz] Ukuran bounding box 3D dalam meter (panjang, lebar, tinggi). Digunakan untuk dimensi objek, penting untuk visualisasi, collision checking, dan evaluasi deteksi.

float32[4] orientation        # [WAJIB] Quaternion [x, y, z, w] orientasi bounding box (default axis-aligned jika [0,0,0,1]). Digunakan untuk orientasi objek di dunia 3D, penting untuk visualisasi, transformasi, dan evaluasi orientasi objek.

float32 confidence            # [WAJIB] Skor confidence hasil fusion (0.0 - 1.0). Confidence hasil deteksi/fusion, digunakan untuk filtering, thresholding, dan evaluasi performa deteksi/fusion.