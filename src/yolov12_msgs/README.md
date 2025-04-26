# yolov12_msgs

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)

Custom message untuk hasil deteksi YOLOv12 dan inference panorama.

---

## Fitur
- Message `Yolov12Inference` untuk hasil deteksi kamera 360°.
- Message `InferenceResult` untuk bounding box dan label.

---

## Struktur Folder
- `msg/` : File message.

---

## Cara Pakai

**Generate message:**
```sh
colcon build --packages-select yolov12_msgs
source install/setup.bash
```

---

## Contoh Struktur Message
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
yolov12_inference:
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