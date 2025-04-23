# yolobot_recognition

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

Node deteksi objek berbasis YOLOv8 dari kamera 360° (6 Arducam IMX477) dan panorama stitcher.

---

## Fitur
- Subscribe ke 6 kamera, stitch panorama.
- Deteksi objek dengan YOLOv8.
- Publish hasil ke `/panorama/yolov8_inference`.

---

## Struktur Folder
- `scripts/` : Node YOLO, stitcher, panorama inference.
- `launch/` : Launch file deteksi.

---

## Cara Pakai

**Jalankan deteksi panorama:**
```sh
ros2 launch yolobot_recognition panorama_inference.launch.py
```

**Parameter penting:**
- `model_path`: Path ke model YOLOv8.
- `confidence_threshold`: Threshold confidence deteksi.

---

## Saran CI
- Tambahkan test inference dengan dataset kecil.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan hasil deteksi publish ke topic yang konsisten untuk node fusion.

## Diagram Alur Deteksi

```
[6xCamera] --> [Stitcher] --> [YOLOv8] --> /panorama/yolov8_inference
```

## Contoh Dataset Gambar

```
dataset/
  panorama_0001.jpg
  panorama_0002.jpg
  ...
```

## Penjelasan Parameter

- `model_path`: Path ke file YOLOv8.
- `confidence_threshold`: 0.5 (default).

## Troubleshooting

- Jika deteksi kosong, cek threshold dan model path.
- Jika stitcher error, cek urutan dan sinkronisasi kamera.