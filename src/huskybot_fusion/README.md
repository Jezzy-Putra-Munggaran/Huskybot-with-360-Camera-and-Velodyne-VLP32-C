# huskybot_fusion  <!-- Judul utama README, nama package (harus sama dengan folder, WAJIB agar colcon build dan ros2 launch/run tidak error) -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)  <!-- Badge CI, update link ke repository Anda yang benar untuk monitoring CI/CD -->

Node fusion data deteksi objek kamera 360° (YOLOv12) dan point cloud Velodyne VLP-32C untuk deteksi objek 3D. Mendukung YOLOv12 (TensorRT/ONNX).  <!-- Deskripsi singkat package dengan penjelasan dukungan untuk model YOLOv12 yang berjalan pada Jetson AGX Orin -->

---

## Fitur  <!-- Daftar fitur utama package -->
- Subscribe ke `/velodyne_points` dan hasil deteksi YOLOv12 dari semua kamera.  <!-- Node ini menerima data point cloud dan deteksi YOLO dari semua kamera, bukan hanya panorama -->
- Support multi-task YOLOv12: `/detection`, `/segmentation`, `/obb`, `/tracking`.  <!-- Support semua tipe task YOLOv12 untuk peningkatan fleksibilitas pipeline -->
- Proyeksi bounding box 2D ke 3D menggunakan kalibrasi extrinsic kamera-LiDAR.  <!-- Fungsi utama: proyeksi hasil deteksi kamera ke 3D menggunakan data kalibrasi -->
- Publish hasil fusion objek 3D ke `/fusion/objects3d` dan marker visualisasi.  <!-- Output utama untuk pipeline navigasi dan visualisasi -->
- FULL OOP dengan error handling komprehensif di semua tahapan.  <!-- Implementasi OOP lengkap dengan error handling di semua fungsi -->
- Logging debug dan audit trail untuk analisis hasil.  <!-- Sistem logging untuk debugging dan evaluasi hasil -->
- Optimized untuk Nvidia Jetson AGX Orin 32GB.  <!-- Optimasi khusus untuk platform Jetson -->
- Mendukung simulasi Gazebo dan robot real Husky A200.  <!-- Dukungan untuk simulasi dan deployment ke robot nyata -->

---

## Struktur Folder  <!-- Penjelasan struktur folder utama package -->
- `huskybot_fusion/` : Source code node fusion.  <!-- Folder utama source code Python (fusion_node.py, fusion_utils.py), semua OOP dan robust error handling -->
- `launch/` : Launch file fusion.  <!-- Folder launch file untuk menjalankan node fusion, wajib untuk ros2 launch -->
- `test/` : Unit test dan linter (flake8, pep257, copyright).  <!-- Folder test untuk CI/CD, wajib untuk coverage dan kualitas kode -->
- `resource/` : Resource ROS2 (wajib untuk ament_python).  <!-- Folder resource agar package dikenali ROS2, wajib untuk ros2 run/launch -->
- `config/` : File kalibrasi YAML extrinsic kamera-LiDAR.  <!-- Folder config untuk file kalibrasi, wajib untuk integrasi sensor -->
- `rviz/` : File konfigurasi RViz2 untuk visualisasi hasil fusion.  <!-- Folder rviz untuk file RViz2, wajib untuk visualisasi marker 3D -->
- `README.md` : Dokumentasi package ini.  <!-- File dokumentasi utama, wajib untuk kolaborasi dan troubleshooting -->
- `CMakeLists.txt`, `package.xml` : Konfigurasi build dan dependency ROS2.  <!-- File build system dan dependency, wajib untuk colcon build -->
- `setup.py`, `setup.cfg` : Konfigurasi Python package.  <!-- File konfigurasi Python package, wajib untuk install dan run -->

---

## Diagram Arsitektur Pipeline  <!-- Diagram visual menunjukkan posisi fusion node dalam pipeline keseluruhan -->
                ┌───────────────────────────┐
                │    6x Arducam IMX477      │
                │   (Konfigurasi Hexagon)   │
                └─────────────┬─────────────┘
                              │ 
                ┌─────────────▼─────────────┐
                │      YOLOv12 (ONNX/       │
                │     TensorRT) Jetson      │
                └┬────────────┬────────────┬┘
                 │            │            │
        ┌────────▼─┐    ┌─────▼────┐   ┌──▼───────┐
        │/detection │    │/segment  │   │/tracking │
        └────────┬─┘    └─────┬────┘   └──┬───────┘
                 │            │            │
                 └────────────┼────────────┘
                              │          ┌──────────────────┐
                              │          │   Velodyne       │
                              │          │   VLP-32C        │
                              │          └─────────┬────────┘
                              │                    │
                              │                    │
                              │          ┌─────────▼────────┐
                              │          │ /velodyne_points │
                              │          └─────────┬────────┘
                              │                    │
                   ┌──────────▼────────────────────▼────────┐
                   │             huskybot_fusion            │
                   │     (Kamera-LiDAR Fusion Node)         │
                   └───┬───────────────────────────────┬────┘
                       │                               │
            ┌──────────▼────────────┐     ┌────────────▼───────────┐
            │  /fusion/objects3d    │     │ /fusion/objects3d_marker│
            │  (Hasil Deteksi 3D)   │     │  (Visualisasi RViz)     │
            └──────────┬────────────┘     └────────────────────────┘
                       │
            ┌──────────▼────────────┐
            │  Navigation & Obstacle │
            │      Avoidance         │
            └───────────────────────┘

## Cara Pakai  <!-- Cara menjalankan package ini di ROS2 Humble/Gazebo -->

**Jalankan node fusion:**  <!-- Instruksi menjalankan node fusion via launch file -->
```sh
ros2 launch huskybot_fusion fusion.launch.py
```

**Parameter penting:**  <!-- Penjelasan parameter penting yang sering digunakan -->
- `calibration_file`: Path ke file kalibrasi kamera-LiDAR.  <!-- Path file YAML hasil kalibrasi extrinsic, wajib benar agar hasil fusion akurat -->
- `fusion_method`: Metode asosiasi objek (misal: nearest, IoU, dsb).  <!-- Parameter metode fusion, bisa diubah sesuai kebutuhan pipeline -->
- `confidence_threshold`: Threshold confidence minimum untuk publish objek 3D.  <!-- Parameter threshold confidence, default 0.3, wajib untuk filter hasil deteksi -->
- `namespace`: Namespace ROS2 untuk multi-robot (opsional).  <!-- Namespace untuk multi-robot, bisa diatur di launch file agar topic tidak bentrok -->
- `log_file`: Path file log untuk fusion node (opsional).  <!-- Path file log audit trail, opsional, aktifkan untuk audit/debugging -->

---

## Visualisasi Label 3D di RViz2

- Jalankan:
  ```sh
  rviz2 -d huskybot_fusion/rviz/fusion.rviz
  ```
  <!-- Perintah untuk membuka RViz2 dengan konfigurasi marker hasil fusion -->
- Pastikan display `MarkerArray` dengan topic `/fusion/objects3d_marker` sudah aktif.  <!-- Display utama untuk visualisasi marker hasil fusion -->
- Label `[class]\nJarak: ...\nPosisi: ...` akan muncul di atas objek 3D.  <!-- Penjelasan label yang muncul di RViz2 -->

---

## Saran CI  <!-- Saran best practice untuk CI/CD agar package selalu aman -->
- Tambahkan unit test untuk fungsi proyeksi dan asosiasi objek.  <!-- Saran test otomatis untuk fungsi utama fusion, wajib untuk pipeline riset -->
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.  <!-- Saran framework test Python, wajib untuk coverage -->
- Tambahkan test launch file di folder `test/` untuk integrasi CI/CD.  <!-- Saran test launch file otomatis, wajib untuk integrasi pipeline -->
- Gunakan [ament_lint_auto](https://index.ros.org/p/ament_lint_auto/) untuk linting kode Python dan XML.  <!-- Saran linting otomatis, wajib untuk kualitas kode -->
- Pastikan badge CI di README sudah update agar status pipeline selalu terlihat.  <!-- Saran badge CI agar status test selalu terlihat, penting untuk kolaborasi tim -->

---

## Catatan  <!-- Catatan penting untuk integrasi workspace -->
Pastikan file kalibrasi kamera-LiDAR sudah benar untuk hasil fusion yang akurat.  <!-- Penjelasan penting: file kalibrasi extrinsic wajib benar agar hasil fusion tidak error -->
Jika workspace multi-robot, gunakan argumen `namespace` di launch file.  <!-- Penjelasan: multi-robot harus pakai namespace agar topic tidak bentrok -->
Semua topic dan frame harus konsisten dengan URDF/Xacro di `huskybot_description` dan hasil kalibrasi di `huskybot_calibration`.  <!-- Penjelasan: konsistensi frame penting untuk integrasi multi-package -->
Jika ingin audit trail, aktifkan logging ke file JSON di parameter node fusion.  <!-- Penjelasan: audit trail bisa diaktifkan via log_file, penting untuk debugging dan evaluasi hasil fusion -->

---

## Diagram Arsitektur Fusion  <!-- Diagram visual arsitektur pipeline fusion -->
```
[YOLOv12 Deteksi 2D]      [Velodyne PointCloud]
         |                       |
         +----------+------------+
                    |
             [Fusion Node]
                    |
         [Deteksi Objek 3D /fusion/objects3d]
```
<!-- Penjelasan: node fusion menerima input dari YOLOv12 dan LiDAR, output ke topic hasil deteksi 3D -->

---

## Contoh Dataset Fusion  <!-- Contoh struktur dan isi file log hasil fusion -->
```
fusion_logs/
  fusion_20250423_1.json
  fusion_20250423_2.json
  ...
```
<!-- Penjelasan: hasil fusion bisa di-log ke file JSON untuk audit trail dan evaluasi -->

**Contoh isi fusion_20250423_1.json:**  <!-- Contoh format file JSON hasil fusion -->
```json
[
  {"label": "person", "center": [1.2, 0.5, 0.8], "size": [0.5, 0.5, 1.7], "confidence": 0.92},
  ...
]
```

---

## Penjelasan Parameter  <!-- Penjelasan detail parameter node fusion -->
- `calibration_file`: Path ke file kalibrasi extrinsic.  <!-- Path file YAML hasil kalibrasi extrinsic kamera-LiDAR, wajib untuk proyeksi 2D-3D -->
- `fusion_method`: nearest/iou/centroid.  <!-- Metode asosiasi objek, bisa diubah sesuai kebutuhan pipeline -->
- `confidence_threshold`: Threshold confidence minimum publish objek 3D.  <!-- Threshold confidence, default 0.3, wajib untuk filter hasil deteksi -->
- `namespace`: Namespace ROS2 untuk multi-robot.  <!-- Namespace untuk multi-robot, bisa diatur di launch file -->
- `log_file`: Path file log untuk audit trail.  <!-- Path file log audit trail, aktifkan untuk debugging dan evaluasi hasil fusion -->

---

## Troubleshooting  <!-- Tips troubleshooting jika ada error saat simulasi/visualisasi -->
- Jika objek 3D tidak muncul, cek sinkronisasi topic dan kalibrasi.  <!-- Saran cek sinkronisasi topic dan file kalibrasi, wajib untuk integrasi sensor -->
- Jika hasil fusion tidak akurat, cek parameter kalibrasi dan proyeksi.  <!-- Saran cek parameter kalibrasi dan fungsi proyeksi, wajib untuk validasi hasil fusion -->
- Jika node fusion error import, pastikan dependency Python (`opencv-python`, `pyyaml`, dsb) sudah diinstall di environment yang aktif.  <!-- Saran cek dependency Python jika error import, wajib untuk runtime ROS2 -->
- Jika file kalibrasi tidak ditemukan, cek path dan permission file YAML di parameter/launch file.  <!-- Saran cek path dan permission file kalibrasi, wajib untuk integrasi sensor -->
- Jika simulasi di Gazebo tidak sinkron, pastikan `use_sim_time:=true` di launch file.  <!-- Saran gunakan waktu simulasi di Gazebo, wajib untuk sinkronisasi waktu -->
- Jika log file tidak terbuat, cek permission folder dan path log_file.  <!-- Saran cek permission folder log, wajib untuk audit trail -->

---

## Saran Peningkatan  <!-- Saran untuk pengembangan dan maintain package ke depan -->
- Tambahkan unit test untuk fungsi proyeksi dan asosiasi objek di folder `test/`.  <!-- Saran test otomatis untuk fungsi utama fusion, wajib untuk pipeline riset -->
- Tambahkan test launch file untuk integrasi CI/CD.  <!-- Saran test launch file otomatis, wajib untuk integrasi pipeline -->
- Tambahkan badge coverage test jika pipeline CI sudah aktif.  <!-- Saran badge coverage test, penting untuk monitoring kualitas kode -->
- Dokumentasikan semua argumen launch file dan parameter node di README.  <!-- Saran dokumentasi parameter, wajib untuk kolaborasi tim -->
- Tambahkan contoh file YAML/CSV hasil kalibrasi di folder config/ untuk referensi user baru.  <!-- Saran contoh file kalibrasi, penting untuk user baru -->
- Tambahkan tips integrasi multi-robot dan cloud jika workspace berkembang.  <!-- Saran tips multi-robot dan cloud, penting untuk scaling pipeline -->
- Tambahkan tips audit trail dan logging ke file untuk debugging pipeline besar.  <!-- Saran audit trail dan logging, penting untuk debugging dan evaluasi hasil fusion -->
- Jika ingin coverage test lebih tinggi, tambahkan test launch file di folder test/.  <!-- Saran coverage test launch file, wajib untuk CI/CD -->
- Pastikan semua dependency Python juga ada di `setup.py` dan `requirements.txt` (untuk Docker/pip).  <!-- Saran sinkronisasi dependency, wajib untuk deployment -->
- Tambahkan badge CI dan coverage di README agar status pipeline selalu terlihat.  <!-- Saran badge CI dan coverage, penting untuk monitoring pipeline -->

---

# ===================== REVIEW & SARAN PENINGKATAN =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Struktur folder, dependency, dan instruksi sudah konsisten dengan workspace dan pipeline utama.
# - Semua node, topic, file, dan folder sudah saling terhubung dengan baik ke pipeline workspace (mapping, fusion, recognition, dsb).
# - Sudah siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Tidak perlu OOP di README, tapi semua node Python di huskybot_fusion sudah FULL OOP.
# - Error handling: semua troubleshooting dan dependency sudah dijelaskan, serta saran error handling di setiap bagian.
# - Saran: tambahkan badge coverage test jika pipeline CI sudah aktif.
# - Saran: tambahkan contoh file YAML/CSV hasil kalibrasi di folder config/ untuk referensi user baru.
# - Saran: tambahkan link ke dokumentasi workspace utama di bagian atas README.
# - Saran: tambahkan tips integrasi multi-robot dan cloud jika workspace berkembang.
# - Saran: tambahkan tips audit trail dan logging ke file untuk debugging pipeline besar.
# - Jika ingin coverage test lebih tinggi, tambahkan test launch file di folder test/.
# - Sudah best practice ROS2, CI/CD, dan aman untuk pipeline besar, simulasi, dan robot real.