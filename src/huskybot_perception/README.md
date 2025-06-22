# huskybot_perception  <!-- Judul utama README, nama package (harus sama dengan folder dan package.xml, wajib agar colcon build dan ros2 launch/run tidak error) -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)  <!-- Badge CI, update link jika pipeline sudah aktif -->

Node visualizer dan logger multitask YOLOv12 untuk Huskybot (360° Arducam IMX477, Velodyne VLP32-C). Visualisasi hasil multitask ke window dan logging ke CSV. Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C). <!-- Deskripsi singkat fungsi utama package, hardware, dan integrasi -->

---

## Fitur  <!-- Section fitur utama package -->
- Visualisasi hasil multitask YOLOv12 (detection, segmentation, OBB, tracking) dari semua kamera 360°. <!-- Fitur visualizer -->
- Logging hasil multitask ke file CSV untuk audit trail dan debugging. <!-- Fitur logger -->
- FULL OOP, robust error handling, siap untuk multi-robot dan audit trail. <!-- Best practice coding dan deployment -->
- Kompatibel dengan ROS2 Humble, Gazebo, dan hardware real. <!-- Penegasan kompatibilitas -->

---

## Struktur Folder  <!-- Struktur folder package -->
- `huskybot_perception/` : Source code node Python (visualizer_node.py, logger_node.py). <!-- Folder utama source code -->
- `launch/` : File launch ROS2 Python (full_perception.launch.py). <!-- Folder launch file -->
- `resource/` : File resource ROS2 (wajib untuk ament_python). <!-- File resource ROS2 -->
- `README.md` : Dokumentasi package ini. <!-- File dokumentasi -->
- `setup.py`, `setup.cfg`, `package.xml` : Build system dan metadata ROS2. <!-- File build system dan metadata -->

---

## Cara Build & Run  <!-- Cara build dan menjalankan node -->
**Build package:**
```sh
colcon build --packages-select huskybot_perception  # Build hanya package ini
source install/setup.bash                           # Source environment agar node bisa di-run
```
**Jalankan node visualizer & logger:**
```sh
ros2 launch huskybot_perception full_perception.launch.py  # Jalankan semua node multitask via launch file
```
<!-- Build dan source wajib agar node bisa di-run dan topic bisa diakses node lain -->

---

## Parameter Launch File  <!-- Penjelasan parameter utama di launch file -->
- Semua node sudah siap di-launch secara paralel (deteksi, segmentasi, OBB, tracking, fusion, visualizer, logger). <!-- Semua pipeline multitask -->
- Untuk multi-robot, tambahkan argumen namespace dan remap topic di launch file. <!-- Siap multi-robot deployment -->
- Path log CSV bisa diubah di logger_node.py (atau jadikan parameter launch file untuk riset besar). <!-- Parameterisasi path log -->

---

## Contoh Output Log CSV  <!-- Contoh hasil file log -->
```csv
timestamp,camera_name,task,class_name,confidence,top,left,bottom,right,track_id,obb_angle
1729431234.123,front,detection,person,0.98,100,200,300,400,12,0.12
1729431234.124,left,tracking,car,0.87,120,220,320,420,7,0.00
```
<!-- Contoh log CSV untuk validasi logger dan audit trail -->

---

## Keterhubungan dengan Workspace  <!-- Penjelasan integrasi dengan pipeline lain -->
- Node ini subscribe ke topic `/detection`, `/segmentation`, `/obb`, `/tracking` (Yolov12Inference), siap untuk diintegrasikan dengan pipeline fusion, mapping, dsb. <!-- Keterhubungan pipeline -->
- Sudah terhubung otomatis ke pipeline workspace (mapping, fusion, recognition, dsb). <!-- Integrasi antar package -->
- Semua dependency sudah ada di `package.xml` dan `setup.py`. <!-- Dependency sudah lengkap -->
- Siap untuk multi-robot (tinggal remap topic/namespace di launch file). <!-- Siap multi-robot deployment -->

---

## Error Handling & Best Practice  <!-- Penjelasan error handling dan tips -->
- Semua error/exception di callback dan fungsi utama sudah di-log ke file dan terminal. <!-- Logging ke file dan terminal untuk audit -->
- Jika file log tidak bisa dibuka, node akan log error dan exit. <!-- Error handling file log hilang -->
- Jika topic multitask tidak ada, node akan log error. <!-- Error handling topic hilang -->
- Jika write row CSV gagal, node akan log warning dan skip row. <!-- Error handling write row -->
- Jika visualisasi gagal (misal headless/server), node akan log warning. <!-- Error handling visualisasi -->
- Semua node sudah FULL OOP (class Node). <!-- Best practice OOP -->
- Untuk multi-robot, gunakan namespace di launch file. <!-- Namespace bisa diatur di launch untuk multi-robot -->
- Untuk audit, gunakan logger node untuk logging ke CSV/JSON. <!-- Logger node siap untuk audit trail dan debugging -->
- Error handling destroy_node dan rclpy.shutdown sudah fail-safe. <!-- Shutdown node aman -->
- Jika error import dependency, pastikan sudah install via pip/rosdep. <!-- Error handling dependency -->
- Jika error permission, cek permission folder workspace dan file log. <!-- Error handling permission -->

---

## Troubleshooting  <!-- Tips troubleshooting umum -->
- Jika node tidak bisa di-run, pastikan sudah build dan source environment. <!-- Sering terjadi jika lupa source install/setup.bash -->
- Jika error import dependency, pastikan semua dependency sudah diinstall (`pip install -r requirements.txt` atau `rosdep install`). <!-- Error import dependency -->
- Jika topic multitask tidak muncul, cek node upstream dan remap topic di launch file. <!-- Error topic multitask -->
- Jika hasil visualisasi/log tidak keluar, cek log output dan parameter path log. <!-- Error hasil visualisasi/log -->
- Jika error permission, cek permission folder workspace dan file log. <!-- Error permission sering terjadi di WSL2/VM -->
- Jika error visualisasi OpenCV, pastikan environment mendukung GUI (atau jalankan di mode headless). <!-- Error visualisasi OpenCV -->

---

## Saran Peningkatan (langsung diimplementasikan)  <!-- Saran pengembangan ke depan -->
- Tambahkan logger node di launch file gabungan untuk audit trail. <!-- Saran audit trail -->
- Tambahkan test/launch/test_full_perception_launch.py untuk CI/CD. <!-- Saran test otomatis -->
- Tambahkan parameterisasi path log di logger_node.py dan launch file. <!-- Saran parameterisasi log -->
- Tambahkan badge CI/CD dan coverage test di README jika pipeline sudah aktif. <!-- Saran badge CI/CD -->
- Dokumentasikan semua parameter di README dan launch file. <!-- Saran dokumentasi parameter -->
- Tambahkan troubleshooting error umum di README. <!-- Saran troubleshooting -->

---

## Link Dokumentasi & Referensi  <!-- Link referensi resmi dan repo utama -->
- [ROS2 Custom Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Node/Creating-A-Node-Py.html) <!-- Referensi resmi cara buat node ROS2 Python -->
- [Ultralytics YOLOv12](https://docs.ultralytics.com/models/yolo12/) <!-- Dokumentasi YOLOv12 -->
- [GitHub Huskybot](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C) <!-- Repo utama workspace -->

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan. WAJIB: Semua baris ada komentar agar mudah dipahami siapapun. --