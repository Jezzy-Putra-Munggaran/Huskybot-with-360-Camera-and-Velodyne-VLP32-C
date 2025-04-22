# yolobot_mapping

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

Integrasi SLAM 3D (Cartographer ROS2) untuk mapping berbasis point cloud Velodyne VLP-32C.

---

## Fitur
- Launch file Cartographer ROS2.
- Konfigurasi mapping 3D (`velodyne_3d.lua`).
- Remap topic `/velodyne_points` ke `/points2`.

---

## Struktur Folder
- `launch/` : Launch file mapping.
- `config/` : File konfigurasi Cartographer.

---

## Cara Pakai

**Jalankan mapping:**
```sh
ros2 launch yolobot_mapping mapping.launch.py
```

**Parameter penting di config:**
- `min_range`, `max_range`: Jarak minimum/maksimum LiDAR.
- `tracking_frame`: Frame utama robot (`base_link`).
- `num_point_clouds`: Jumlah input point cloud (1 untuk Velodyne).

---

## Contoh Command Visualisasi
```sh
rviz2 -d yolobot_mapping/rviz/mapping.rviz
```

---

## Saran CI
- Tambahkan test launch file mapping.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan topic point cloud dan frame konsisten dengan URDF dan node lain.