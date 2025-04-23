# yolov8_msgs

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

Custom message untuk hasil deteksi YOLOv8 dan inference panorama.

---

## Fitur
- Message `Yolov8Inference` untuk hasil deteksi kamera 360°.
- Message `InferenceResult` untuk bounding box dan label.

---

## Struktur Folder
- `msg/` : File message.

---

## Cara Pakai

**Generate message:**
```sh
colcon build --packages-select yolov8_msgs
source install/setup.bash
```

---

## Contoh Struktur Message
```yaml
Yolov8Inference:
  header: std_msgs/Header
  yolov8_inference: InferenceResult[]
  camera_name: string

InferenceResult:
  class_name: string
  confidence: float32
  left: float32
  top: float32
  right: float32
  bottom: float32
```

---

## Saran CI
- Tambahkan test message generation di workflow CI.

---

## Catatan
Pastikan message ini digunakan konsisten di node recognition dan fusion.

## Contoh Dataset Message

```yaml
header:
  stamp: ...
  frame_id: "panorama"
yolov8_inference:
  - class_name: "person"
    confidence: 0.95
    left: 100
    top: 50
    right: 200
    bottom: 300
camera_name: "panorama"
```

## Penjelasan Parameter

- Semua field di message wajib diisi sesuai tipe.

## Troubleshooting

- Jika message tidak bisa diimport, pastikan sudah build dan source environment.