# huskybot_detection  <!-- Judul utama README, nama package (harus sama dengan folder dan package.xml, wajib agar colcon build dan ros2 launch/run tidak error) -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)  <!-- Badge CI, update link jika pipeline sudah aktif -->

Node deteksi YOLOv12 multicamera untuk Huskybot (360° Arducam IMX477). Publish hasil deteksi ke topic `/detection` (Yolov12Inference). Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C). <!-- Deskripsi singkat fungsi utama package, hardware, dan integrasi -->

---

## Fitur  <!-- Section fitur utama package -->
- Deteksi objek YOLOv12 secara paralel dari 6 kamera 360° (hexagonal). <!-- Fitur utama: multicam detection -->
- Publish hasil deteksi ke topic `/detection` dengan message `Yolov12Inference`. <!-- Keterhubungan dengan pipeline fusion/logger -->
- FULL OOP, robust error handling, siap untuk multi-robot dan audit trail. <!-- Best practice coding dan deployment -->
- Kompatibel dengan ROS2 Humble, Gazebo, dan hardware real. <!-- Penegasan kompatibilitas -->

---

## Struktur Folder  <!-- Struktur folder package -->
- `huskybot_detection/` : Source code node Python. <!-- Folder utama source code -->
- `launch/` : File launch ROS2 Python. <!-- Folder launch file -->
- `resource/` : File resource ROS2 (wajib untuk ament_python). <!-- File resource ROS2 -->
- `test/` : Unit test dan linter (flake8, pep257, copyright). <!-- Folder test -->
- `README.md` : Dokumentasi package ini. <!-- File dokumentasi -->
- `setup.py`, `setup.cfg`, `package.xml` : Build system dan metadata ROS2. <!-- File build system dan metadata -->

---

## Cara Build & Run  <!-- Cara build dan menjalankan node -->
**Build package:**
```sh
colcon build --packages-select huskybot_detection  # Build hanya package ini
source install/setup.bash                          # Source environment agar node bisa di-run
```
**Jalankan node deteksi multicam:**
```sh
ros2 launch huskybot_detection detection.launch.py  # Jalankan node multicam YOLOv12 via launch file
```
<!-- Build dan source wajib agar node bisa di-run dan topic bisa diakses node lain -->

---

## Parameter Launch File  <!-- Penjelasan parameter utama di launch file -->
- `cam_count`: Jumlah kamera (default 6, hexagonal). <!-- Parameter jumlah kamera -->
- `model_path`: Path file model YOLOv12 (default "yolo12n.engine"). <!-- Parameter path model YOLOv12 -->
- `camera_topics`: List topic kamera (default urutan hexagonal). <!-- Parameter list topic kamera -->
- Semua parameter bisa diubah dari CLI/launch file lain. <!-- Parameterisasi siap untuk deployment besar/multi-robot -->

---

## Contoh Output Message  <!-- Contoh message hasil deteksi -->
```yaml
header:
  stamp: {sec: 1680000000, nanosec: 123456789}
  frame_id: "Camera_1"
camera_name: "Camera_1"
frame_type: "raw"
task: "detect"
note: ""
yolov12_inference:
  - class_name: "person"
    confidence: 0.95
    top: 50
    left: 100
    bottom: 300
    right: 200
    track_id: 1
    obb_angle: -1
    mask_indices: []
```
<!-- Contoh message untuk validasi node fusion/logger/visualizer -->

---

## Keterhubungan dengan Workspace  <!-- Penjelasan integrasi dengan pipeline lain -->
- Node ini publish ke topic `/detection` (Yolov12Inference), siap untuk di-subscribe node fusion, logger, dsb. <!-- Keterhubungan pipeline -->
- Sudah terhubung otomatis ke pipeline workspace (mapping, fusion, recognition, dsb). <!-- Integrasi antar package -->
- Semua dependency sudah ada di `package.xml` dan `setup.py`. <!-- Dependency sudah lengkap -->
- Siap untuk multi-robot (tinggal remap topic/namespace di launch file). <!-- Siap multi-robot deployment -->

---

## Error Handling & Best Practice  <!-- Penjelasan error handling dan tips -->
- Semua error/exception di callback dan fungsi utama sudah di-log ke file dan terminal. <!-- Logging ke file dan terminal untuk audit -->
- Jika file model tidak ditemukan, node akan log error dan exit. <!-- Error handling file model hilang -->
- Jika topic kamera tidak ada, node akan log error. <!-- Error handling topic hilang -->
- Jika konversi image gagal, node akan log warning dan skip frame. <!-- Error handling konversi image -->
- Jika inference gagal, node akan log error dan publish gambar kosong. <!-- Error handling inference -->
- Jika publish gagal, node akan log error. <!-- Error handling publish -->
- Semua parameter bisa diubah via launch file. <!-- Parameterisasi siap untuk deployment besar -->
- Semua node sudah FULL OOP (class Node). <!-- Best practice OOP -->
- Untuk multi-robot, gunakan namespace di launch file. <!-- Namespace bisa diatur di launch untuk multi-robot -->
- Untuk audit, gunakan logger node untuk logging ke CSV/JSON. <!-- Logger node siap untuk audit trail dan debugging -->

---

## Troubleshooting  <!-- Tips troubleshooting umum -->
- Jika node tidak bisa di-run, pastikan sudah build dan source environment. <!-- Sering terjadi jika lupa source install/setup.bash -->
- Jika error import dependency, pastikan semua dependency sudah diinstall (`pip install -r requirements.txt` atau `rosdep install`). <!-- Error import dependency -->
- Jika topic kamera tidak muncul, cek node kamera dan remap topic di launch file. <!-- Error topic kamera -->
- Jika hasil deteksi tidak keluar, cek log output dan parameter model_path. <!-- Error hasil deteksi -->
- Jika error permission, cek permission folder workspace dan file model. <!-- Error permission sering terjadi di WSL2/VM -->
- Jika error visualisasi OpenCV, pastikan environment mendukung GUI (atau jalankan di mode headless). <!-- Error visualisasi OpenCV -->

---

## Saran Peningkatan (langsung diimplementasikan)  <!-- Saran pengembangan ke depan -->
- Tambahkan logger node di launch file gabungan untuk audit trail. <!-- Saran audit trail -->
- Tambahkan test/launch/test_detection_launch.py untuk CI/CD. <!-- Saran test otomatis -->
- Tambahkan parameterisasi threshold dan class filter di node. <!-- Saran parameterisasi deteksi -->
- Tambahkan badge CI/CD dan coverage test di README jika pipeline sudah aktif. <!-- Saran badge CI/CD -->
- Dokumentasikan semua parameter di README dan launch file. <!-- Saran dokumentasi parameter -->
- Tambahkan troubleshooting error umum di README. <!-- Saran troubleshooting -->

---

## Link Dokumentasi & Referensi  <!-- Link referensi resmi dan repo utama -->
- [ROS2 Custom Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Node/Creating-A-Node-Py.html) <!-- Referensi resmi cara buat node ROS2 Python -->
- [Ultralytics YOLOv12](https://docs.ultralytics.com/models/yolo12/) <!-- Dokumentasi YOLOv12 -->
- [GitHub Huskybot](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C) <!-- Repo utama workspace -->

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan. WAJIB: Semua baris ada komentar agar mudah dipahami siapapun. -->