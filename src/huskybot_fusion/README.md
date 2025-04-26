# huskybot_fusion

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)

Node fusion data deteksi objek kamera 360° (YOLO) dan point cloud Velodyne VLP-32C untuk deteksi objek 3D.

---

## Fitur
- Subscribe ke `/velodyne_points` dan `/panorama/yolov12_inference`.
- Proyeksi bounding box 2D ke 3D (menggunakan kalibrasi).
- Publish hasil deteksi objek 3D ke `/fusion/objects3d`.

---

## Struktur Folder
- `huskybot_fusion/` : Source code node fusion.
- `msg/` : Custom message `Object3D`.
- `launch/` : Launch file fusion.

---

## Cara Pakai

**Jalankan node fusion:**
```sh
ros2 launch huskybot_fusion fusion.launch.py
```

**Parameter penting:**
- `calibration_file`: Path ke file kalibrasi kamera-LiDAR.
- `fusion_method`: Metode asosiasi objek (misal: nearest, IoU, dsb).

---

## Contoh Command Visualisasi
```sh
rviz2 -d huskybot_fusion/rviz/fusion.rviz
```

---

## Saran CI
- Tambahkan unit test untuk fungsi proyeksi dan asosiasi objek.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan file kalibrasi kamera-LiDAR sudah benar untuk hasil fusion yang akurat.

## Diagram Arsitektur Fusion

```
[YOLOv12 Deteksi 2D]      [Velodyne PointCloud]
         |                       |
         +----------+------------+
                    |
             [Fusion Node]
                    |
         [Deteksi Objek 3D /fusion/objects3d]
```

## Contoh Dataset Fusion

```
fusion_logs/
  fusion_20250423_1.json
  fusion_20250423_2.json
  ...
```

**Contoh isi fusion_20250423_1.json:**
```json
[
  {"label": "person", "center": [1.2, 0.5, 0.8], "size": [0.5, 0.5, 1.7], "confidence": 0.92},
  ...
]
```

## Penjelasan Parameter

- `calibration_file`: Path ke file kalibrasi extrinsic.
- `fusion_method`: nearest/iou/centroid.

## Troubleshooting

- Jika objek 3D tidak muncul, cek sinkronisasi topic dan kalibrasi.
- Jika hasil fusion tidak akurat, cek parameter kalibrasi dan proyeksi.