# yolov12_msgs  <!-- Judul README, nama package message custom YOLOv12 -->

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)  <!-- Badge CI, update URL jika repo sudah publik -->

Custom message untuk hasil deteksi YOLOv12 dan inference panorama.  <!-- Deskripsi singkat fungsi package -->

---

## Fitur  <!-- Section fitur utama package -->
- Message `Yolov12Inference` untuk hasil deteksi kamera 360°.  <!-- Message utama untuk hasil deteksi multi-kamera/panorama -->
- Message `InferenceResult` untuk bounding box dan label.  <!-- Message untuk satu hasil deteksi (bounding box + label) -->

---

## Struktur Folder  <!-- Struktur folder package -->
- `msg/` : File message.  <!-- Folder msg berisi file .msg custom (InferenceResult, Yolov12Inference) -->
- `CMakeLists.txt` : Build system ROS2.  <!-- File build system untuk generate message interface -->
- `package.xml` : Metadata package ROS2.  <!-- Metadata dependency, maintainer, dsb -->
- `README.md` : Dokumentasi package.  <!-- File ini, untuk dokumentasi penggunaan dan struktur message -->

---

## Cara Pakai  <!-- Cara build dan source package message -->
**Generate message:**
```sh
colcon build --packages-select yolov12_msgs  # Build hanya package ini (bisa juga build seluruh workspace)
source install/setup.bash                    # Source environment agar message bisa diimport node lain
```
<!-- Build dan source wajib agar message bisa digunakan node lain di workspace -->

---

## Contoh Struktur Message  <!-- Contoh struktur message untuk dokumentasi dan referensi node lain -->
```yaml
Yolov12Inference:
  header: std_msgs/Header
  yolov12_inference: InferenceResult[]
  camera_name: string

InferenceResult:
  class_name: string
  confidence: float32
  top: int64
  left: int64
  bottom: int64
  right: int64
```
<!-- NB: Field confidence dan tipe koordinat sudah sesuai .msg. Pastikan contoh ini konsisten dengan file msg/ -->

---

## Saran CI  <!-- Saran untuk integrasi CI agar build message selalu dicek otomatis -->
- Tambahkan test message generation di workflow CI.  <!-- Agar build message otomatis dicek di GitHub Actions/dsb -->
- Gunakan [ament_lint_auto](https://index.ros.org/p/ament_lint_auto/) dan [ament_lint_common](https://index.ros.org/p/ament_lint_common/) untuk linting package message.  <!-- Linting untuk jaga kualitas kode dan message -->

---

## Catatan  <!-- Catatan penting penggunaan message -->
Pastikan message ini digunakan konsisten di node recognition dan fusion.  <!-- Agar pipeline deteksi dan fusion tidak error tipe message -->
- Semua node di package `huskybot_recognition` dan `huskybot_fusion` sudah menggunakan message ini.  <!-- Keterhubungan antar package di workspace -->

---

## Contoh Dataset Message  <!-- Contoh data message untuk testing/debugging -->
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
camera_name: "panorama"
```
<!-- Contoh data message untuk test/debugging node publisher/subscriber -->

---

## Penjelasan Parameter  <!-- Penjelasan bahwa semua field wajib diisi sesuai tipe -->
- Semua field di message wajib diisi sesuai tipe.  <!-- Penting untuk validasi dan integrasi antar node -->
- `header`: Untuk sinkronisasi waktu dan frame referensi.  <!-- Digunakan untuk time sync dan TF -->
- `yolov12_inference`: List hasil deteksi per frame.  <!-- Bisa kosong jika tidak ada deteksi -->
- `camera_name`: Nama kamera atau sumber deteksi.  <!-- Untuk multi-kamera/panorama -->
- `class_name`, `confidence`, `top`, `left`, `bottom`, `right`: Detail hasil deteksi bounding box.  <!-- Untuk evaluasi dan visualisasi -->

---

## Troubleshooting  <!-- Tips troubleshooting jika message tidak bisa diimport -->
- Jika message tidak bisa diimport, pastikan sudah build dan source environment.  <!-- Sering terjadi jika lupa source install/setup.bash -->
- Jika ada error dependency, cek `CMakeLists.txt` dan `package.xml` sudah lengkap.  <!-- Dependency harus lengkap agar build sukses -->
- Jika field message tidak sesuai, pastikan semua node sudah rebuild setelah update .msg.  <!-- Hindari mismatch antara node dan message -->

---

## Saran Peningkatan README (langsung diimplementasikan di bawah):

- Tambahkan penjelasan file/folder lain di package (sudah).
- Tambahkan contoh message yang konsisten dengan file .msg (sudah).
- Tambahkan tips troubleshooting dan CI (sudah).
- Tambahkan penjelasan keterhubungan dengan node di workspace (sudah).
- Tambahkan link ke dokumentasi ROS2 message [docs.ros.org](https://docs.ros.org/en/humble/How-To-Guides/Working-with-custom-ROS2-Interfaces.html).  <!-- Untuk referensi lebih lanjut -->

---

## Link Dokumentasi

- [ROS2 Custom Message Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Working-with-custom-ROS2-Interfaces.html)  <!-- Referensi resmi cara buat dan pakai custom message -->
- [GitHub Huskybot](https://github.com/jezzy/huskybot)  <!-- Repo utama workspace (update jika sudah publik) -->

---

<!-- END OF README, semua baris sudah diberi komentar penjelasan -->