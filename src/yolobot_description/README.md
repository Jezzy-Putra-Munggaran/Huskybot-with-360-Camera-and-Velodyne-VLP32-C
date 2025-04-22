# yolobot_description

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

Deskripsi model robot Husky A200 beserta mounting sensor (Velodyne VLP-32C dan 6 kamera Arducam IMX477) dalam format URDF/Xacro.

---

## Fitur
- Model robot Husky A200 lengkap dengan semua link dan joint.
- Mounting Velodyne VLP-32C dan 6 kamera 360° (hexagonal).
- Frame transform (TF) yang konsisten untuk integrasi sensor.
- File kalibrasi sensor (opsional).

---

## Struktur Folder
- `urdf/` : File Xacro/URDF robot dan sensor.
- `meshes/` : Mesh visual sensor dan robot.
- `calibration/` : File kalibrasi kamera-LiDAR (jika ada).

---

## Cara Pakai

**Load URDF ke RViz2:**
```sh
ros2 launch yolobot_description display.launch.py
```

**Parameter penting:**
- `robot_description`: Path ke file Xacro/URDF utama.
- `use_sim_time`: Gunakan waktu simulasi (default: true untuk simulasi).

---

## Contoh Frame TF
```
map
 └── odom
      └── base_link
           ├── velodyne_link
           ├── camera_front_link
           ├── camera_left_link
           ├── camera_right_link
           ├── camera_rear_link
           ├── camera_front_left_link
           └── camera_rear_right_link
```

---

## Saran CI
- Tambahkan test URDF dengan [xacro test](http://wiki.ros.org/xacro#Testing).
- Tambahkan check URDF dengan [check_urdf](http://wiki.ros.org/check_urdf).

---

## Catatan
Pastikan semua frame (`base_link`, `velodyne_link`, `camera_*_link`) konsisten dengan node lain di workspace.

## Diagram Arsitektur Sensor

```
         +-------------------+
         |   Husky A200      |
         |                   |
         |   [base_link]     |
         |      |            |
         |  +---+---+        |
         |  |       |        |
         |[velodyne][camera*6] (hexagonal)
         +-------------------+
```

## Contoh Dataset Kalibrasi

```
calibration/
  extrinsic_lidar_to_camera.yaml
  intrinsic_camera_0.yaml
  ...
```

**Contoh isi extrinsic_lidar_to_camera.yaml:**
```yaml
T_lidar_camera:
  rows: 4
  cols: 4
  data: [ ... 16 nilai matriks ... ]
```

## Parameter Penting

- `robot_description`: Path ke file Xacro utama.
- `use_sim_time`: true/false.

## Troubleshooting

- Jika model tidak muncul di RViz2, cek path mesh dan urdf.
- Jika TF tidak lengkap, cek joint dan link di Xacro.