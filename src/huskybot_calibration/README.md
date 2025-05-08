# huskybot_calibration <!-- Judul utama README, nama package (harus sama dengan folder) -->

[![Build Status](https://github.com/jezzy/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/jezzy/huskybot/actions) <!-- Badge CI, update link jika pipeline sudah aktif -->

Kalibrasi otomatis kamera-LiDAR dan sinkronisasi waktu sensor untuk Huskybot (360° Arducam IMX477 + Velodyne VLP-32C). Siap untuk ROS2 Humble, simulasi Gazebo, dan robot Husky A200 asli. <!-- Deskripsi singkat, jelas, dan spesifik -->

---

## Struktur Folder <!-- Penjelasan struktur folder utama package -->
- `calibration_data/` : Data mentah hasil rekaman kalibrasi (image, pointcloud, dsb). <!-- Untuk menyimpan data kalibrasi sebelum diproses -->
- `config/` : File hasil kalibrasi (YAML extrinsic, dsb). <!-- Output hasil kalibrasi, dibaca node fusion -->
- `huskybot_calibration/` : Source code Python utama (OOP, modular). <!-- Semua node/script utama ada di sini -->
- `launch/` : Launch file untuk pipeline kalibrasi dan sinkronisasi waktu. <!-- Untuk menjalankan node via ros2 launch -->
- `resource/` : File resource ROS2 (wajib untuk ament_python). <!-- Agar package dikenali colcon/ROS2 -->
- `scripts/` : Script CLI untuk rekam data kalibrasi. <!-- Untuk merekam data sinkron kamera-LiDAR -->
- `test/` : Unit test dan linter untuk CI/CD. <!-- Untuk menjaga kualitas kode dan pipeline -->
- `README.md` : Dokumentasi lengkap package ini. <!-- File ini sendiri -->
- `package.xml`, `setup.py`, `setup.cfg` : Konfigurasi build dan dependency ROS2. <!-- File build system dan dependency -->

---

## Fitur Utama <!-- Daftar fitur utama package -->
- Kalibrasi extrinsic otomatis kamera-LiDAR (checkerboard/ArUco, OOP, YAML output).
- Sinkronisasi waktu sensor (kamera, LiDAR, IMU) dengan ROS2 message_filters.
- Kompatibel dengan ROS2 Humble, simulasi Gazebo, dan robot Husky A200 asli.
- Visualisasi hasil kalibrasi (opsional, matplotlib).
- Error handling lengkap di setiap node (cek file, topic, format, dsb).
- Unit test dan linter untuk CI/CD.

---

## Cara Instalasi & Build <!-- Instruksi build dan setup -->
```sh
cd ~/huskybot
rosdep install --from-paths src --ignore-src -r  # Install semua dependency
colcon build --symlink-install
source install/setup.bash
```
<!-- Pastikan dependency sudah lengkap sebelum build. -->

---

## Cara Menjalankan Kalibrasi <!-- Instruksi menjalankan pipeline kalibrasi -->
1. Rekam data sinkron kamera-LiDAR:
   ```sh
   ros2 run huskybot_calibration record_calib_data.py --output calibration_data/
   ```
2. Jalankan node kalibrasi extrinsic:
   ```sh
   ros2 launch huskybot_calibration calibrate_lidar_camera.launch.py use_sim_time:=true
   ```
3. Hasil kalibrasi akan tersimpan di `config/extrinsic_lidar_to_camera.yaml`.
4. Integrasikan file YAML ke node fusion di package `huskybot_fusion`.

---

## Integrasi dengan Workspace Lain <!-- Penjelasan keterhubungan dengan package lain -->
- File hasil kalibrasi (`config/extrinsic_lidar_to_camera.yaml`) **WAJIB** dibaca oleh node fusion (`huskybot_fusion/fusion_node.py`).
- Data mentah di `calibration_data/` bisa digunakan untuk training, evaluasi, atau re-kalibrasi.
- Semua topic dan frame harus konsisten dengan URDF/Xacro di `huskybot_description` dan driver di `velodyne/`.
- Untuk multi-robot, gunakan argumen `namespace` di launch file.

---

## Error Handling & Best Practice <!-- Penjelasan error handling dan tips -->
- Semua node sudah ada error handling untuk file YAML, format data, dan topic ROS2.
- Jika file kalibrasi tidak ditemukan, node akan log error dan exit.
- Jika data sensor kosong/corrupt, node akan log warning dan skip proses.
- Semua script Python sudah FULL OOP (class Node, modular).
- Semua topic bisa di-remap via launch file.
- Untuk multi-robot, gunakan namespace ROS2 pada launch file.
- Untuk audit, gunakan logger node untuk logging ke file.
- Unit test wajib dijalankan sebelum push ke repo.

---

## Contoh Output File Kalibrasi <!-- Contoh format file YAML hasil kalibrasi -->
```yaml
T_lidar_camera:
  rows: 4
  cols: 4
  data: [ ... 16 nilai matriks ... ]
```
<!-- Format ini harus konsisten dengan yang dibaca node fusion di package lain -->

---

## Troubleshooting <!-- Tips troubleshooting umum -->
- Jika node tidak jalan, cek dependency dengan `rosdep check`.
- Jika file YAML tidak ditemukan, cek path di config/ dan permission file.
- Jika data sensor tidak sinkron, cek topic dan header.stamp di ROS2.
- Jika error import module, pastikan sudah build dan source environment.
- Jika simulasi di Gazebo tidak sinkron, pastikan `use_sim_time:=true` di launch file.

---

## Saran Peningkatan <!-- Saran untuk pengembangan dan maintain package ke depan -->
- Tambahkan folder `dataset/` untuk data training/validasi.
- Tambahkan file `CHANGELOG.md` dan `CONTRIBUTING.md` untuk dokumentasi dan kontribusi.
- Update badge CI jika repo sudah publik dan pipeline aktif.
- Dokumentasikan semua frame dan topic sensor di README agar integrasi lebih mudah.
- Tambahkan contoh penggunaan multi-robot di README.
- Tambahkan troubleshooting untuk error umum di ROS2 Humble/Gazebo.
- Tambahkan test otomatis validasi file YAML dan sinkronisasi data di folder `test/`.

---

## Lisensi <!-- Penjelasan lisensi package -->
[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) <!-- Lisensi package, sesuai dengan package.xml -->

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan -->