# huskybot_gazebo

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)

Konfigurasi dan launch file simulasi Gazebo untuk robot Husky A200 dengan Velodyne VLP-32C dan kamera 360°.

---

## Fitur
- Launch file untuk spawn robot di Gazebo.
- World file untuk simulasi.
- Plugin sensor (Velodyne, kamera).
- Integrasi dengan ROS2 dan topic `/velodyne_points`.

---

## Struktur Folder
- `launch/` : Launch file simulasi.
- `worlds/` : File world Gazebo.
- `models/` : Model tambahan (opsional).

---

## Cara Pakai

**Jalankan simulasi Gazebo:**
```sh
ros2 launch huskybot_gazebo huskybot_launch.py
```

**Parameter penting:**
- `use_sim_time`: Gunakan waktu simulasi (default: true).
- `world`: Path ke file world Gazebo.

---

## Contoh Plugin Velodyne di URDF
```xml
<gazebo>
  <plugin name="gazebo_ros_velodyne_controller" filename="libgazebo_ros_velodyne_laser.so">
    <robotNamespace>/velodyne</robotNamespace>
    <frameName>velodyne_link</frameName>
    <topicName>velodyne_points</topicName>
  </plugin>
</gazebo>
```

---

## Saran CI
- Tambahkan test launch file dengan [pytest](https://docs.pytest.org/en/stable/).

---

## Catatan
Gunakan argumen `use_sim_time` untuk sinkronisasi waktu simulasi.

## Diagram Arsitektur Simulasi

```
[Gazebo World]
    |
    +-- [Husky A200 + Velodyne + 6xCamera]
           |
           +-- Plugin Velodyne (publish /velodyne_points)
           +-- Plugin Camera (publish /camera_X/image_raw)
```

## Contoh Dataset Simulasi

- World file: `worlds/yolo_test.world`
- Model file: `models/husky_with_sensors/model.sdf`

## Parameter Penting di Launch

```yaml
world: worlds/yolo_test.world
use_sim_time: true
```

## Troubleshooting

- Jika sensor tidak publish, cek plugin di URDF/Xacro.
- Jika robot tidak muncul, cek path model dan dependency.