# huskybot_navigation

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)

Node dan konfigurasi untuk obstacle avoidance dan path planning berbasis 3D map dan hasil fusion objek.

---

## Fitur
- Integrasi dengan Nav2 (Navigation2) ROS2.
- Custom costmap layer untuk obstacle 3D dari hasil fusion.
- Path planning dan obstacle avoidance.

---

## Struktur Folder
- `scripts/` : Node navigation (custom, jika ada).
- `launch/` : Launch file navigation.
- `config/` : Konfigurasi Nav2/costmap.

---

## Cara Pakai

**Jalankan navigation:**
```sh
ros2 launch huskybot_navigation navigation.launch.py
```

**Parameter penting:**
- `planner_type`: Jenis planner (DWB, TEB, custom 3D).
- `costmap_params`: Path ke file parameter costmap.

---

## Contoh Command Visualisasi
```sh
rviz2 -d huskybot_navigation/rviz/navigation.rviz
```

---

## Saran CI
- Tambahkan test launch file navigation.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan input dari `/map` (Cartographer) dan `/fusion/objects3d` sudah tersedia.

## Diagram Alur Navigasi

```
/map + /fusion/objects3d --> [Nav2/Custom Navigation] --> /cmd_vel
```

## Contoh Parameter Costmap

```yaml
obstacle_layer:
  enabled: true
  observation_sources: fusion_objects
  fusion_objects:
    topic: /fusion/objects3d
    data_type: PointCloud2
    marking: true
    clearing: false
```

## Troubleshooting

- Jika robot tidak menghindar obstacle, cek input costmap dan topic fusion.
- Jika path tidak muncul, cek planner dan goal pose.