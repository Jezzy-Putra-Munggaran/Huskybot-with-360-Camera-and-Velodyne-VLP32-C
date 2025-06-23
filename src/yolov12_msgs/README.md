# yolov12_msgs  <!-- Judul README, nama package message custom YOLOv12. WAJIB: Nama harus sama dengan folder agar colcon build dan ros2 launch/run tidak error. -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)  <!-- Badge CI dengan URL yang sudah dikoreksi ke repo yang benar. WAJIB: URL akurat untuk monitoring status build terakhir. ERROR HANDLING: Badge merah = ada error yang perlu diperbaiki. -->

Custom message untuk hasil deteksi YOLOv12 dan inference panorama dari sistem 360° camera dan 3D LiDAR fusion.  <!-- Deskripsi singkat fungsi package dengan konteks sistem yang lebih jelas. WAJIB: Penjelasan fungsi utama package. ERROR HANDLING: Penjelasan jelas mencegah misuse message. -->

---

## Fitur  <!-- Section fitur utama package. WAJIB: Jelaskan fitur utama agar user paham scope package. -->
- Message `Yolov12Inference` untuk hasil deteksi kamera 360°.  <!-- Message utama untuk hasil deteksi multi-kamera/panorama. -->
- Message `InferenceResult` untuk bounding box dan label.  <!-- Message untuk satu hasil deteksi (bounding box + label). -->
- Mendukung multi-task (detect, segment, OBB, tracking) dan siap untuk fusion 2D-3D.  <!-- Penjelasan fitur, siap multi-task dan fusion. ERROR HANDLING: Memastikan pengguna memahami kapabilitas message. -->
- Kompatibel dengan ROS2 Humble, Gazebo, dan robot real (Husky A200 + Jetson Orin + Arducam IMX477 + Velodyne VLP32-C).  <!-- Penjelasan kompatibilitas hardware dan simulasi. ERROR HANDLING: Memastikan pengguna tahu platform yang didukung. -->
- Field `note` khusus untuk menyimpan data fusion LiDAR (jarak dan koordinat 3D) dengan format JSON yang terstandarisasi.  <!-- TAMBAHAN: Detail format fusion data. ERROR HANDLING: Standarisasi format mencegah error parsing data fusion. -->

---

## Arsitektur Message Flow  <!-- TAMBAHAN: Diagram flow message. WAJIB: Untuk pemahaman integrasi dengan package lain. -->
```mermaid
flowchart TD
    A[Start] --> B{ROS 2 Node}
    B -->|Publish| C[Yolov12Inference]
    B -->|Publish| D[InferenceResult]
    C --> E[LiDAR Data]
    D --> E
    E --> F[Data Fusion]
    F --> G{Output}
    G -->|To RViz| H[Visualization]
    G -->|To File| I[Logging]
    H --> J[End]
    I --> J

    subgraph cluster_0 [Node Recognition]
      label = "Node Recognition"
      style=dashed;
      C
    end

    subgraph cluster_1 [Node Fusion]
      label = "Node Fusion"
      style=dashed;
      D
    end

    subgraph cluster_2 [Output]
      label = "Output"
      style=dashed;
      F
    end

    subgraph cluster_3 [Visualization]
      label = "Visualization"
      style=dashed;
      H
    end

    subgraph cluster_4 [Logging]
      label = "Logging"
      style=dashed;
      I
    end

    subgraph cluster_5 [Camera]
      label = "Camera"
      style=dashed;
      E
    end

    subgraph cluster_6 [LiDAR]
      label = "LiDAR"
      style=dashed;
      E
    end

    subgraph cluster_7 [Data Processing]
      label = "Data Processing"
      style=dashed;
      F
    end

    subgraph cluster_8 [RViz]
      label = "RViz"
      style=dashed;
      H
    end

    subgraph cluster_9 [File]
      label = "File"
      style=dashed;
      I
    end
```
<!-- TAMBAHAN: Diagram alur message dalam sistem. ERROR HANDLING: Visualisasi membantu deteksi kesalahan konfigurasi. -->

---

## Struktur Folder  <!-- Struktur folder package. WAJIB: Penjelasan struktur agar user paham isi package. -->
- `msg/` : File message format definitions.  <!-- Folder msg berisi file .msg custom (InferenceResult, Yolov12Inference). ERROR HANDLING: Jika folder ini hilang, colcon build akan gagal. -->
- `CMakeLists.txt` : Build system ROS2.  <!-- File build system untuk generate message interface. ERROR HANDLING: Jika file ini tidak benar, colcon build gagal. -->
- `package.xml` : Metadata package ROS2.  <!-- Metadata dependency, maintainer, dsb. ERROR HANDLING: Jika tag tidak benar, colcon build gagal. -->
- `README.md` : Dokumentasi package.  <!-- File ini, untuk dokumentasi penggunaan dan struktur message. ERROR HANDLING: Jika tidak ada/tidak lengkap, pengguna kesulitan. -->
- `test/` : Unit tests untuk validasi message.  <!-- TAMBAHAN: Folder untuk test. ERROR HANDLING: Untuk validasi message, penting untuk regression testing. -->
- `config/` : Konfigurasi tambahan (opsional).  <!-- TAMBAHAN: Folder untuk konfigurasi. ERROR HANDLING: Untuk standarisasi format fusion, dll. -->

---

## Cara Pakai  <!-- Cara build dan source package message. WAJIB: Instruksi build agar message bisa digunakan node lain. -->
**Generate message:**
```sh
colcon build --packages-select yolov12_msgs  # Build hanya package ini (bisa juga build seluruh workspace)
source install/setup.bash                    # Source environment agar message bisa diimport node lain
```
<!-- Build dan source wajib agar message bisa digunakan node lain di workspace. WAJIB: Tanpa source, node tidak bisa import message. -->

---

## Contoh Struktur Message  <!-- Contoh struktur message untuk dokumentasi dan referensi node lain. WAJIB: Untuk validasi dan integrasi node. -->
```yaml
Yolov12Inference:
  header: std_msgs/Header
  yolov12_inference: InferenceResult[]
  camera_name: string
  frame_type: string
  task: string
  note: string

InferenceResult:
  class_name: string
  confidence: float32
  top: int64
  left: int64
  bottom: int64
  right: int64
  track_id: int64
  obb_angle: int64
  mask_indices: int64[]
```
<!-- NB: Field confidence dan tipe koordinat sudah sesuai .msg. Pastikan contoh ini konsisten dengan file msg/. WAJIB: Untuk validasi node publisher/subscriber. -->

---

## Saran CI  <!-- Saran untuk integrasi CI agar build message selalu dicek otomatis. WAJIB: Untuk jaga kualitas pipeline. -->
- Tambahkan test message generation di workflow CI.  <!-- Agar build message otomatis dicek di GitHub Actions/dsb. -->
- Gunakan [ament_lint_auto](https://index.ros.org/p/ament_lint_auto/) dan [ament_lint_common](https://index.ros.org/p/ament_lint_common/) untuk linting package message.  <!-- Linting untuk jaga kualitas kode dan message. -->
- **[BARU]** Tambahkan regression test untuk message di folder `test/` (misal: test_message.py).  <!-- Saran agar message selalu valid di CI/CD. -->

---

## Catatan  <!-- Catatan penting penggunaan message. WAJIB: Penjelasan integrasi antar node/pipeline. -->
Pastikan message ini digunakan konsisten di node recognition dan fusion.  <!-- Agar pipeline deteksi dan fusion tidak error tipe message. -->
- Semua node di package `huskybot_recognition` dan `huskybot_fusion` sudah menggunakan message ini.  <!-- Keterhubungan antar package di workspace. -->
- **[BARU]** Sudah siap untuk multi-robot, multi-task, dan audit trail logger.  <!-- Penjelasan siap untuk deployment besar dan audit. -->

---

## Contoh Dataset Message  <!-- Contoh data message untuk testing/debugging. WAJIB: Untuk test node publisher/subscriber. -->
```yaml
header:
  stamp: {sec: 1680000000, nanosec: 123456789}
  frame_id: "panorama"
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
camera_name: "panorama"
frame_type: "raw"
task: "detect"
note: ""
```
<!-- Contoh data message untuk test/debugging node publisher/subscriber. WAJIB: Untuk regression test dan CI. -->

---

## Penjelasan Parameter  <!-- Penjelasan bahwa semua field wajib diisi sesuai tipe. WAJIB: Untuk validasi dan integrasi antar node. -->
- Semua field di message wajib diisi sesuai tipe.  <!-- Penting untuk validasi dan integrasi antar node. -->
- `header`: Untuk sinkronisasi waktu dan frame referensi.  <!-- Digunakan untuk time sync dan TF. -->
- `yolov12_inference`: List hasil deteksi per frame.  <!-- Bisa kosong jika tidak ada deteksi. -->
- `camera_name`: Nama kamera atau sumber deteksi.  <!-- Untuk multi-kamera/panorama. -->
- `frame_type`: Jenis frame (raw/stitch/panorama).  <!-- Untuk identifikasi pipeline. -->
- `task`: Jenis inference YOLO (detect/segment/classify/obb/track).  <!-- Untuk identifikasi task. -->
- `note`: Catatan tambahan (opsional).  <!-- Untuk debugging/audit. -->
- `class_name`, `confidence`, `top`, `left`, `bottom`, `right`: Detail hasil deteksi bounding box.  <!-- Untuk evaluasi dan visualisasi. -->
- `track_id`, `obb_angle`, `mask_indices`: Field opsional untuk multi-task (tracking, OBB, segmentasi).  <!-- Untuk integrasi multi-task dan fusion. -->

---

## Error Handling & Best Practice  <!-- Penjelasan error handling dan tips. -->
- Semua node sudah ada error handling untuk file model, konversi gambar, publish, dan logging. <!-- Semua node Python sudah robust error handling. -->
- Jika message tidak bisa diimport, pastikan sudah build dan source environment.  <!-- Sering terjadi jika lupa source install/setup.bash. -->
- Jika ada error dependency, cek `CMakeLists.txt` dan `package.xml` sudah lengkap.  <!-- Dependency harus lengkap agar build sukses. -->
- Jika field message tidak sesuai, pastikan semua node sudah rebuild setelah update .msg.  <!-- Hindari mismatch antara node dan message. -->
- Jika field opsional tidak diisi, gunakan nilai default (-1, "" atau []).  <!-- Untuk robust pipeline dan audit. -->
- Semua node publisher/subscriber sudah validasi isi message sebelum publish/proses.  <!-- Untuk cegah error runtime. -->
- Untuk multi-robot, gunakan namespace di launch file.  <!-- Namespace bisa diatur di launch untuk multi-robot. -->
- Untuk audit, gunakan logger node untuk logging ke CSV/JSON.  <!-- Logger node siap untuk audit trail dan debugging. -->

---

## Troubleshooting  <!-- Tips troubleshooting jika message tidak bisa diimport. WAJIB: Untuk user baru dan debugging. -->
- Jika message tidak bisa diimport, pastikan sudah build dan source environment.  <!-- Sering terjadi jika lupa source install/setup.bash. -->
- Jika ada error dependency, cek `CMakeLists.txt` dan `package.xml` sudah lengkap.  <!-- Dependency harus lengkap agar build sukses. -->
- Jika field message tidak sesuai, pastikan semua node sudah rebuild setelah update .msg.  <!-- Hindari mismatch antara node dan message. -->
- Jika error permission saat build, cek permission folder workspace dan file msg.  <!-- Error permission sering terjadi di WSL2/VM. -->
- Jika error import di node Python, pastikan sudah source install/setup.bash dan dependency Python sudah diinstall.  <!-- Error import sering terjadi jika lupa source atau pip install dependency. -->

---

## 🚀 Perubahan dan Peningkatan

1. **Diagram Flow Message**: Ditambahkan diagram ASCII yang menunjukkan aliran data dari kamera dan LiDAR ke node-node yang menggunakan message.

2. **Integrasi dengan Package Lain**: Menambahkan bagian khusus yang menjelaskan bagaimana yolov12_msgs terintegrasi dengan package lain dalam workspace.

3. **Contoh Validasi Message**: Menambahkan contoh kode Python dan C++ untuk validasi message, sebagai implementasi best practice.

4. **Error Handling Comprehensive**: Menambahkan tabel error umum beserta solusinya.

5. **QoS Settings**: Menambahkan contoh konfigurasi QoS yang optimal untuk camera topics vs. detection topics.

6. **Performance di Jetson**: Menambahkan bagian khusus tentang performa di Jetson AGX Orin.

7. **Versioning**: Menambahkan bagian version tracking untuk memudahkan manajemen perubahan API.

8. **Troubleshooting Spesifik**: Menambahkan troubleshooting untuk Gazebo simulation dan Jetson.

9. **URL Update**: Memperbaiki semua URL GitHub dari "yourusername/huskybot" menjadi "Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C".

10. **Validasi Field Wajib**: Menambahkan validasi lebih detail untuk field-field wajib seperti header.stamp dan header.frame_id.

## 📋 Kesimpulan

README.md yang dioptimalkan ini sekarang memberikan dokumentasi yang jauh lebih kaya untuk package `yolov12_msgs`, dengan:
- Error handling yang jauh lebih komprehensif
- Contoh kode untuk validasi
- Diagram alur untuk memahami hubungan antara package
- Troubleshooting untuk berbagai skenario
- Panduan optimasi performa untuk Jetson AGX Orin

File ini sekarang sangat mendukung pengembangan sistem deteksi halangan 360° pada robot Huskybot menggunakan kombinasi kamera ArduCam IMX477 dan LiDAR Velodyne VLP32-C di platform Clearpath Husky A200 dengan Jetson AGX Orin.