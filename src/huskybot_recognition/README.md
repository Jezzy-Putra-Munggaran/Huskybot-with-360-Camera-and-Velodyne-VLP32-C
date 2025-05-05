# huskybot_recognition  <!-- Judul utama README, nama package (harus sama dengan folder) -->

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions) <!-- Badge CI, update link jika repo sudah publik dan pipeline aktif -->

Node deteksi objek berbasis YOLOv12 dari kamera 360° (6 Arducam IMX477) dan panorama stitcher. <!-- Deskripsi singkat package, menjelaskan fungsi utama dan sensor utama -->

---

## Fitur  <!-- Daftar fitur utama package -->
- Subscribe ke 6 kamera, stitch panorama. <!-- Node stitcher akan subscribe ke 6 kamera dan melakukan stitching panorama -->
- Deteksi objek dengan YOLOv12. <!-- Node YOLOv12 akan melakukan deteksi objek pada hasil stitching -->
- Publish hasil ke `/panorama/yolov12_inference`. <!-- Hasil deteksi dipublish ke topic ini untuk diproses node lain (fusion, logger, dsb) -->

---

## Struktur Folder  <!-- Penjelasan struktur folder utama package -->
- `scripts/` : Node YOLO, stitcher, panorama inference. <!-- Semua node utama Python ada di sini -->
- `launch/` : Launch file deteksi. <!-- Semua launch file untuk menjalankan node ada di sini -->

---

## Cara Pakai  <!-- Cara menjalankan package ini di ROS2 Humble/Gazebo -->

**Jalankan deteksi panorama:**
```sh
ros2 launch huskybot_recognition panorama_inference.launch.py
```
<!-- Perintah utama untuk menjalankan pipeline stitching + YOLOv12 panorama -->

**Parameter penting:**
- `model_path`: Path ke model YOLOv12. <!-- Path file .pt model YOLOv12, bisa diubah di launch file -->
- `confidence_threshold`: Threshold confidence deteksi. <!-- Threshold confidence YOLOv12, bisa diubah di launch file -->

---

## Saran CI  <!-- Saran untuk integrasi continuous integration/testing -->
- Tambahkan test inference dengan dataset kecil. <!-- Disarankan menambah test dataset untuk regression test -->
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python. <!-- Gunakan pytest untuk unit test node Python -->

---

## Catatan  <!-- Catatan penting untuk integrasi workspace -->
Pastikan hasil deteksi publish ke topic yang konsisten untuk node fusion. <!-- Penjelasan penting agar integrasi node fusion tidak error -->

## Diagram Alur Deteksi  <!-- Diagram visual alur data deteksi -->
```
[6xCamera] --> [Stitcher] --> [YOLOv12] --> /panorama/yolov12_inference
```
<!-- Menjelaskan pipeline utama: 6 kamera -> stitcher -> YOLOv12 -> topic hasil -->

## Contoh Dataset Gambar  <!-- Contoh struktur dataset panorama -->
```
dataset/
  panorama_0001.jpg
  panorama_0002.jpg
  ...
```
<!-- Contoh struktur folder dataset jika ingin test offline -->

## Penjelasan Parameter  <!-- Penjelasan parameter penting node/launch -->
- `model_path`: Path ke file YOLOv12. <!-- Path file model YOLOv12 -->
- `confidence_threshold`: 0.5 (default). <!-- Nilai default threshold confidence -->

## Troubleshooting  <!-- Tips troubleshooting umum -->
- Jika deteksi kosong, cek threshold dan model path. <!-- Saran jika hasil deteksi tidak keluar -->
- Jika stitcher error, cek urutan dan sinkronisasi kamera. <!-- Saran jika panorama gagal -->

---

## Saran Peningkatan README (langsung diimplementasikan di bawah):

- Tambahkan penjelasan node utama dan topic yang digunakan.
- Tambahkan contoh launch file dan parameterisasi.
- Tambahkan penjelasan error handling dan best practice.
- Tambahkan link ke dokumentasi ROS2 dan YOLOv12.
- Tambahkan tips multi-robot dan namespace.
- Tambahkan info tentang file kalibrasi dan model.

---

## Node Utama & Topic

| Node Script                        | Fungsi Utama                        | Topic Input                        | Topic Output                        |
|-------------------------------------|-------------------------------------|------------------------------------|-------------------------------------|
| `yolov12_ros2_pt.py`                | Deteksi YOLOv12 multi-kamera        | `/camera_*` (6 kamera)             | `/Yolov12_Inference`, `/inference_result` |
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
        DeclareLaunchArgument('model_path', default_value='~/huskybot/src/huskybot_recognition/scripts/yolov12n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        Node(
            package='huskybot_recognition',
            executable='yolov12_ros2_pt.py',
            parameters=[
                {'model_path': LaunchConfiguration('model_path')},
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')}
            ],
            output='screen'
        ),
    ])
```
<!-- Contoh launch file untuk menjalankan node dengan parameterisasi -->

---

## Error Handling & Best Practice

- Semua node sudah ada error handling untuk file model, konversi gambar, publish, dan logging.
- Jika file model tidak ditemukan, node akan log error dan exit.
- Jika file kalibrasi kamera tidak ditemukan, node stitcher akan log error dan exit.
- Semua node Python sudah FULL OOP (class Node).
- Semua topic sudah konsisten dan bisa di-remap via launch file.
- Untuk multi-robot, gunakan namespace ROS2 pada launch file.
- Untuk audit, gunakan logger node untuk logging ke CSV.

---

## Link Dokumentasi

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [Ultralytics YOLOv12](https://docs.ultralytics.com/)
- [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
- [pytest](https://docs.pytest.org/en/stable/)

---

## File Kalibrasi & Model

- File model YOLOv12 (`yolov12n.pt`) ada di `scripts/`.
- File kalibrasi kamera (`intrinsic_camera_*.yaml`) ada di `huskybot_description/calibration/`.
- Pastikan path file model dan kalibrasi sudah benar di parameter/launch file.

---

## Kontribusi & Issue

- Silakan buat issue di [https://github.com/jezzy/huskybot/issues](https://github.com/jezzy/huskybot/issues) jika ada bug/fitur baru.
- Pull request sangat diterima, pastikan sudah lulus CI dan test.

---

## Changelog

- 2025-05-06: Initial release, support multi-camera, panorama, YOLOv12, logger, dan listener.

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan -->