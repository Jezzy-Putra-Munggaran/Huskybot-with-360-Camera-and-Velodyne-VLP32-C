# huskybot_recognition  <!-- Judul utama README, nama package (harus sama dengan folder, wajib agar colcon build dan ros2 launch/run tidak error) -->

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions) <!-- Badge CI, update link jika repo sudah publik dan pipeline aktif. Memudahkan monitoring CI/CD. -->

Node deteksi objek berbasis YOLOv12 dari kamera 360° (6 Arducam IMX477) dan panorama stitcher. <!-- Deskripsi singkat package, menjelaskan fungsi utama dan sensor utama. -->

---

## Fitur  <!-- Daftar fitur utama package -->
- Subscribe ke 6 kamera, stitch panorama. <!-- Node stitcher akan subscribe ke 6 kamera dan melakukan stitching panorama. -->
- Deteksi objek dengan YOLOv12. <!-- Node YOLOv12 akan melakukan deteksi objek pada hasil stitching. -->
- Publish hasil ke `/panorama/yolov12_inference`. <!-- Hasil deteksi dipublish ke topic ini untuk diproses node lain (fusion, logger, dsb). -->

---

## Struktur Folder  <!-- Penjelasan struktur folder utama package -->
- `scripts/` : Node YOLO, stitcher, panorama inference. <!-- Semua node utama Python ada di sini. -->
- `launch/` : Launch file deteksi. <!-- Semua launch file untuk menjalankan node ada di sini. -->
- `test/` : Unit test dan launch test. <!-- Folder test untuk CI/CD dan regression test. -->
- `calibration/` : File YAML kalibrasi kamera (symlink dari huskybot_description). <!-- File kalibrasi wajib untuk stitcher/fusion. -->

---

## Cara Pakai  <!-- Cara menjalankan package ini di ROS2 Humble/Gazebo -->

**Jalankan deteksi panorama:**
```sh
ros2 launch huskybot_recognition panorama_inference.launch.py
```
<!-- Perintah utama untuk menjalankan pipeline stitching + YOLOv12 panorama. -->

**Parameter penting:**
- `model_path`: Path ke model YOLOv12. <!-- Path file .pt/.engine/.onnx model YOLOv12, bisa diubah di launch file. -->
- `confidence_threshold`: Threshold confidence deteksi. <!-- Threshold confidence YOLOv12, bisa diubah di launch file. -->
- `model_format`: engine/onnx/pt (auto pilih node). <!-- Format model, auto-switch node YOLOv12. -->
- `log_stats_path`: Path file statistik deteksi. <!-- Path file CSV log deteksi, bisa diubah. -->
- `calib_yaml_path`: Path file YAML kalibrasi kamera. <!-- Path file kalibrasi, wajib untuk stitcher/fusion. -->

---

## Saran CI  <!-- Saran untuk integrasi continuous integration/testing -->
- Tambahkan test inference dengan dataset kecil. <!-- Disarankan menambah test dataset untuk regression test. -->
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python. <!-- Gunakan pytest untuk unit test node Python. -->
- Tambahkan test launch file di folder test/. <!-- Test launch file untuk integrasi pipeline. -->
- Gunakan [ament_lint_auto](https://index.ros.org/p/ament_lint_auto/) untuk linting kode Python dan XML. <!-- Linting otomatis agar coding style konsisten. -->

---

## Catatan  <!-- Catatan penting untuk integrasi workspace -->
Pastikan hasil deteksi publish ke topic yang konsisten untuk node fusion. <!-- Penjelasan penting agar integrasi node fusion tidak error. -->
- Semua topic sudah namespace-ready, bisa multi-robot. <!-- Siap untuk multi-robot deployment. -->
- Semua node sudah OOP dan robust error handling. <!-- Semua node Python sudah class-based dan modular. -->

## Diagram Alur Deteksi  <!-- Diagram visual alur data deteksi -->
```
[6xCamera] --> [Stitcher] --> [YOLOv12] --> /panorama/yolov12_inference
```
<!-- Menjelaskan pipeline utama: 6 kamera -> stitcher -> YOLOv12 -> topic hasil. -->

## Contoh Dataset Gambar  <!-- Contoh struktur dataset panorama -->
```
dataset/
  panorama_0001.jpg
  panorama_0002.jpg
  ...
```
<!-- Contoh struktur folder dataset jika ingin test offline. -->

## Penjelasan Parameter  <!-- Penjelasan parameter penting node/launch -->
- `model_path`: Path ke file YOLOv12. <!-- Path file model YOLOv12. -->
- `confidence_threshold`: 0.5 (default). <!-- Nilai default threshold confidence. -->
- `model_format`: engine/onnx/pt. <!-- Format model, auto pilih node. -->
- `log_stats_path`: Path file statistik deteksi. <!-- Path file CSV log deteksi. -->
- `calib_yaml_path`: Path file YAML kalibrasi kamera. <!-- Path file kalibrasi kamera. -->

## Troubleshooting  <!-- Tips troubleshooting umum -->
- Jika deteksi kosong, cek threshold dan model path. <!-- Saran jika hasil deteksi tidak keluar. -->
- Jika stitcher error, cek urutan dan sinkronisasi kamera. <!-- Saran jika panorama gagal. -->
- Jika node tidak jalan, cek dependency dengan `rosdep check` dan pastikan sudah build & source environment. <!-- Cek dependency jika node gagal jalan. -->
- Jika file model/kalibrasi tidak ditemukan, cek path di parameter/launch file dan permission folder. <!-- Cek path dan permission file model/kalibrasi. -->
- Jika error import module, pastikan sudah build dan source environment. <!-- Cek environment jika error import. -->
- Jika simulasi di Gazebo tidak sinkron, pastikan `use_sim_time:=true` di launch file. <!-- Gunakan waktu simulasi di Gazebo. -->
- Jika log file tidak terbuat, cek permission folder dan path log_file di launch file. <!-- Cek permission folder log. -->
- Untuk multi-robot, gunakan argumen `namespace` di launch file dan pastikan semua topic sudah namespace-ready. <!-- Namespace bisa diatur di launch untuk multi-robot. -->

---

## Tips Audit Trail & Logging

- Aktifkan logger node untuk logging ke file CSV/LOG agar mudah audit dan debugging. <!-- Logger node siap untuk audit trail dan debugging. -->
- Semua node sudah ada logging ke file dan terminal, bisa diubah path log_file via parameter/launch file. <!-- Logging ke file dan terminal untuk audit. -->

---

## Saran Peningkatan README (langsung diimplementasikan di bawah):

- Tambahkan penjelasan node utama dan topic yang digunakan. <!-- Saran agar user baru langsung tahu node dan topic utama. -->
- Tambahkan contoh launch file dan parameterisasi. <!-- Saran agar user bisa custom parameter dari awal. -->
- Tambahkan penjelasan error handling dan best practice. <!-- Saran agar user paham error handling pipeline. -->
- Tambahkan link ke dokumentasi ROS2 dan YOLOv12. <!-- Saran agar user mudah cari referensi. -->
- Tambahkan tips multi-robot dan namespace. <!-- Saran agar siap untuk multi-robot deployment. -->
- Tambahkan info tentang file kalibrasi dan model. <!-- Saran agar user tidak error file hilang. -->
- Tambahkan troubleshooting dan tips audit trail. <!-- Saran agar user mudah debug pipeline besar. -->
- Tambahkan badge CI dan link kontribusi. <!-- Saran agar kolaborasi lebih mudah. -->

---

## Node Utama & Topic

| Node Script                        | Fungsi Utama                        | Topic Input                        | Topic Output                        |
|-------------------------------------|-------------------------------------|------------------------------------|-------------------------------------|
| `yolov12_ros2_pt.py`                | Deteksi YOLOv12 multi-kamera        | `/camera_*` (6 kamera)             | `/Yolov12_Inference`, `/inference_result` |
| `yolov12_ros2_trt.py`               | Deteksi YOLOv12 TensorRT (.engine)  | `/camera_*` (6 kamera)             | `/Yolov12_Inference`, `/inference_result` |
| `yolov12_ros2_onnx.py`              | Deteksi YOLOv12 ONNX (.onnx)        | `/camera_*` (6 kamera)             | `/Yolov12_Inference`, `/inference_result` |
| `yolov12_stitcher_node.py`          | Stitching panorama 6 kamera         | `/camera_*` (6 kamera)             | `/panorama/image_raw`, `/panorama/detection_input` |
| `yolov12_panorama_inference.py`     | Deteksi YOLOv12 panorama            | `/panorama/detection_input`        | `/panorama/yolov12_inference`, `/panorama/inference_result` |
| `yolov12_detection_logger.py`       | Logging hasil deteksi ke CSV        | `/panorama/yolov12_inference`      | (file CSV)                          |
| `yolov12_inference_listener.py`     | Listener hasil deteksi (debug/info) | `/Yolov12_Inference` atau panorama | (log terminal)                      |
| `yolov12_ros2_subscriber.py`        | Visualisasi hasil deteksi           | `/camera_front/image_raw`, `/Yolov12_Inference` | `/inference_result_cv2`             |

---

## Contoh Launch File (parameterisasi)

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='~/jezzy/huskybot/src/huskybot_recognition/huskybot_recognition/scripts/yolo12n.pt'),  # Default TensorRT .engine
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('model_format', default_value='engine'),  # engine/onnx/pt
        Node(
            package='huskybot_recognition',
            executable='yolov12_ros2_trt.py',  # Ganti otomatis sesuai model_format di launch_yolov12.launch.py
            parameters=[
                {'model_path': LaunchConfiguration('model_path')},
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')}
            ],
            output='screen'
        ),
    ])
```
<!-- Contoh launch file untuk menjalankan node dengan parameterisasi. -->

---

## Error Handling & Best Practice

- Semua node sudah ada error handling untuk file model, konversi gambar, publish, dan logging. <!-- Semua node Python sudah robust error handling. -->
- Jika file model tidak ditemukan, node akan log error dan exit. <!-- Error handling file model hilang. -->
- Jika file kalibrasi kamera tidak ditemukan, node stitcher akan log error dan exit. <!-- Error handling file kalibrasi hilang. -->
- Semua node Python sudah FULL OOP (class Node). <!-- Semua node sudah OOP, modular, mudah di-maintain. -->
- Semua topic sudah konsisten dan bisa di-remap via launch file. <!-- Topic bisa diubah tanpa edit source code. -->
- Untuk multi-robot, gunakan namespace ROS2 pada launch file. <!-- Namespace bisa diatur di launch untuk multi-robot. -->
- Untuk audit, gunakan logger node untuk logging ke CSV. <!-- Logger node siap untuk audit trail dan debugging. -->
- Semua error/exception di callback dan fungsi utama sudah di-log ke file dan terminal. <!-- Logging ke file dan terminal untuk audit. -->
- Validasi file model, parameter, dan message sudah lengkap. <!-- Semua parameter wajib divalidasi. -->
- Monitoring health check sensor (jumlah deteksi, class). <!-- Monitoring pipeline deteksi. -->
- Semua parameter bisa diubah via launch file. <!-- Parameterisasi siap untuk deployment besar. -->
- Siap multi-robot (tinggal remap topic via launch file). <!-- Siap untuk deployment multi-robot. -->
- Siap audit trail dan integrasi logger/fusion. <!-- Siap untuk pipeline besar dan debugging. -->
- Try/except untuk error permission file log/stats. <!-- Error handling permission file/folder. -->
- Error handling destroy_node dan rclpy.shutdown. <!-- Shutdown node aman. -->
- Komentar penjelasan di setiap baris coding. <!-- Semua file sudah diberi komentar penjelasan. -->
- Import cv2 di dalam blok try agar tidak error jika cv2 belum terinstall (misal di server headless). <!-- Robust import dependency. -->

---

## Link Dokumentasi

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html) <!-- Link dokumentasi ROS2 Humble. -->
- [Ultralytics YOLOv12](https://docs.ultralytics.com/) <!-- Link dokumentasi YOLOv12. -->
- [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge) <!-- Link dokumentasi cv_bridge. -->
- [pytest](https://docs.pytest.org/en/stable/) <!-- Link dokumentasi pytest. -->
- [NVIDIA Jetson AGX Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <!-- Link hardware Jetson Orin. -->
- [DeepStream SDK](https://docs.ultralytics.com/guides/deepstream-nvidia-jetson/) <!-- Link DeepStream SDK. -->
- [Triton Inference Server](https://docs.ultralytics.com/guides/triton-inference-server/) <!-- Link Triton Inference Server. -->

---

## File Kalibrasi & Model

- File model YOLOv12 (`yolo12n.pt`, `yolo12n.pt`, `yolo12n.onnx`) ada di `scripts/`. <!-- File model YOLOv12 wajib ada di scripts/. -->
- File kalibrasi kamera (`intrinsic_camera_*.yaml`) ada di `huskybot_description/calibration/`. <!-- File kalibrasi wajib ada di folder ini. -->
- Pastikan path file model dan kalibrasi sudah benar di parameter/launch file. <!-- Saran agar tidak error file hilang. -->
- Untuk Jetson Orin, gunakan model .engine (TensorRT) untuk performa maksimal, fallback ke .onnx jika perlu. <!-- Best practice Jetson Orin. -->

---

## Kontribusi & Issue

- Silakan buat issue di [https://github.com/jezzy/huskybot/issues](https://github.com/jezzy/huskybot/issues) jika ada bug/fitur baru. <!-- Link issue tracker. -->
- Pull request sangat diterima, pastikan sudah lulus CI dan test. <!-- Saran kontribusi, pastikan CI lulus. -->
- Dokumentasikan semua parameter dan struktur folder di README.md. <!-- Dokumentasi wajib untuk kolaborasi. -->

---

## Changelog

- 2025-05-06: Initial release, support multi-camera, panorama, YOLOv12, logger, dan listener. <!-- Catatan perubahan utama. -->

---

## Daftar Periksa Sebelum Uji Coba

1. **Cek Dependensi:** Pastikan semua dependensi yang diperlukan untuk proyekmu terinstal. Periksa file `package.xml` dan `CMakeLists.txt` untuk memastikan semua paket yang diperlukan sudah terdaftar.

2. **Cek Versi VPI:** Dari output build, ada peringatan tentang VPI. Pastikan kamu memiliki versi VPI yang sesuai dengan yang dibutuhkan proyek. Jika perlu, instal versi yang tepat.

3. **Perbaiki Peringatan di Kode:** Ada beberapa peringatan di `ros_deep_learning` yang menunjukkan bahwa fungsi tidak mengembalikan nilai. Perbaiki fungsi-fungsi tersebut agar mengembalikan nilai yang sesuai.

4. **Cek Konfigurasi ROS:** Pastikan bahwa semua konfigurasi ROS sudah benar, termasuk pengaturan untuk kamera dan LiDAR. Periksa file launch dan parameter yang digunakan.

5. **Uji Coba dengan Satu Kamera:** Sebelum mencoba dengan enam kamera, lakukan uji coba dengan satu kamera terlebih dahulu untuk memastikan semuanya berjalan dengan baik.

6. **Log dan Debug:** Periksa log output saat menjalankan node untuk melihat apakah ada kesalahan atau peringatan yang muncul. Gunakan `ros2 run` untuk menjalankan node secara manual dan lihat outputnya.

7. **Cek Jaringan:** Jika menggunakan beberapa kamera, pastikan bahwa semua perangkat terhubung dengan benar dan dapat berkomunikasi satu sama lain.

8. **Dokumentasi:** Rujuk ke dokumentasi yang relevan untuk setiap komponen yang digunakan, seperti OpenStitcher dan YOLOv12, untuk memastikan bahwa semua langkah konfigurasi telah diikuti dengan benar.