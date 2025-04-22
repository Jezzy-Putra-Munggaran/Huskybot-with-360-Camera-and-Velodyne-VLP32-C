# yolobot_fusion

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

Node fusion data deteksi objek kamera 360° (YOLO) dan point cloud Velodyne VLP-32C untuk deteksi objek 3D.

---

## Fitur
- Subscribe ke `/velodyne_points` dan `/panorama/yolov8_inference`.
- Proyeksi bounding box 2D ke 3D (menggunakan kalibrasi).
- Publish hasil deteksi objek 3D ke `/fusion/objects3d`.

---

## Struktur Folder
- `yolobot_fusion/` : Source code node fusion.
- `msg/` : Custom message `Object3D`.
- `launch/` : Launch file fusion.

---

## Cara Pakai

**Jalankan node fusion:**
```sh
ros2 launch yolobot_fusion fusion.launch.py
```

**Parameter penting:**
- `calibration_file`: Path ke file kalibrasi kamera-LiDAR.
- `fusion_method`: Metode asosiasi objek (misal: nearest, IoU, dsb).

---

## Contoh Command Visualisasi
```sh
rviz2 -d yolobot_fusion/rviz/fusion.rviz
```

---

## Saran CI
- Tambahkan unit test untuk fungsi proyeksi dan asosiasi objek.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan file kalibrasi kamera-LiDAR sudah benar untuk hasil fusion yang akurat.