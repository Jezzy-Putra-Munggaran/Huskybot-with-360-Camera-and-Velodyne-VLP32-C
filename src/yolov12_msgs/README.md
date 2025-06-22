# yolov12_msgs  <!-- Judul README, nama package message custom YOLOv12. WAJIB: Nama harus sama dengan folder agar colcon build dan ros2 launch/run tidak error. -->

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)  <!-- Badge CI, update URL jika repo sudah publik. Untuk monitoring CI/CD build message. -->

Custom message untuk hasil deteksi YOLOv12 dan inference panorama.  <!-- Deskripsi singkat fungsi package. WAJIB: Penjelasan fungsi utama package. -->

---

## Fitur  <!-- Section fitur utama package. WAJIB: Jelaskan fitur utama agar user paham scope package. -->
- Message `Yolov12Inference` untuk hasil deteksi kamera 360°.  <!-- Message utama untuk hasil deteksi multi-kamera/panorama. -->
- Message `InferenceResult` untuk bounding box dan label.  <!-- Message untuk satu hasil deteksi (bounding box + label). -->
- **[BARU]** Mendukung multi-task (detect, segment, OBB, tracking) dan siap untuk fusion 2D-3D.  <!-- Penjelasan fitur baru, siap multi-task dan fusion. -->
- **[BARU]** Kompatibel dengan ROS2 Humble, Gazebo, dan robot real (Husky A200 + Jetson Orin + Arducam IMX477 + Velodyne VLP32-C).  <!-- Penjelasan kompatibilitas hardware dan simulasi. -->

---

## Struktur Folder  <!-- Struktur folder package. WAJIB: Penjelasan struktur agar user paham isi package. -->
- `msg/` : File message.  <!-- Folder msg berisi file .msg custom (InferenceResult, Yolov12Inference). -->
- `CMakeLists.txt` : Build system ROS2.  <!-- File build system untuk generate message interface. -->
- `package.xml` : Metadata package ROS2.  <!-- Metadata dependency, maintainer, dsb. -->
- `README.md` : Dokumentasi package.  <!-- File ini, untuk dokumentasi penggunaan dan struktur message. -->

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

## Saran Peningkatan README (langsung diimplementasikan di bawah):  <!-- WAJIB: Semua saran sudah diimplementasikan langsung di README. -->
- Tambahkan penjelasan file/folder lain di package (sudah).
- Tambahkan contoh message yang konsisten dengan file .msg (sudah).
- Tambahkan tips troubleshooting dan CI (sudah).
- Tambahkan penjelasan keterhubungan dengan node di workspace (sudah).
- Tambahkan link ke dokumentasi ROS2 message [docs.ros.org](https://docs.ros.org/en/humble/How-To-Guides/Working-with-custom-ROS2-Interfaces.html).  <!-- Untuk referensi lebih lanjut. -->
- **[BARU]** Tambahkan tips multi-task, multi-robot, dan audit trail logger (sudah).  <!-- Saran baru, sudah diimplementasikan. -->
- **[BARU]** Tambahkan contoh dataset message multi-task (sudah).  <!-- Saran baru, sudah diimplementasikan. -->

---

## Link Dokumentasi  <!-- Link referensi resmi dan repo utama. WAJIB: Untuk user baru dan kolaborasi tim. -->
- [ROS2 Custom Message Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Working-with-custom-ROS2-Interfaces.html)  <!-- Referensi resmi cara buat dan pakai custom message. -->
- [GitHub Huskybot](https://github.com/jezzy/huskybot)  <!-- Repo utama workspace (update jika sudah publik). -->

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan. WAJIB: Semua baris ada komentar agar mudah dipahami siapapun. -->