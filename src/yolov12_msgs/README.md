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

---

## Cara Pakai  <!-- Cara build dan source package message -->
**Generate message:**
```sh
colcon build --packages-select yolov12_msgs  # Build hanya package ini (bisa juga build seluruh workspace)
source install/setup.bash                    # Source environment agar message bisa diimport node lain
```

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
  left: float32
  top: float32
  right: float32
  bottom: float32
```
<!-- NB: Field confidence belum ada di .msg, hanya di contoh. Jika ingin, tambahkan di InferenceResult.msg -->

---

## Saran CI  <!-- Saran untuk integrasi CI agar build message selalu dicek otomatis -->
- Tambahkan test message generation di workflow CI.  <!-- Agar build message otomatis dicek di GitHub Actions/dsb -->

---

## Catatan  <!-- Catatan penting penggunaan message -->
Pastikan message ini digunakan konsisten di node recognition dan fusion.  <!-- Agar pipeline deteksi dan fusion tidak error tipe message -->

## Contoh Dataset Message  <!-- Contoh data message untuk testing/debugging -->
```yaml
header:
  stamp: ...
  frame_id: "panorama"
yolov12_inference:
  - class_name: "person"
    confidence: 0.95
    left: 100
    top: 50
    right: 200
    bottom: 300
camera_name: "panorama"
```
<!-- NB: Field confidence juga belum ada di .msg, hanya di contoh. -->

## Penjelasan Parameter  <!-- Penjelasan bahwa semua field wajib diisi sesuai tipe -->
- Semua field di message wajib diisi sesuai tipe.

## Troubleshooting  <!-- Tips troubleshooting jika message tidak bisa diimport -->
- Jika message tidak bisa diimport, pastikan sudah build dan source environment.