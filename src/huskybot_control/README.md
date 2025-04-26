# huskybot_control

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions)

Node kontrol robot Husky A200 (teleop/manual/autonomous) untuk simulasi dan real robot.

---

## Fitur
- Node kontrol kecepatan (`/cmd_vel`).
- Integrasi dengan joystick atau autonomous planner.
- Safety check (opsional).

---

## Struktur Folder
- `scripts/` : Script kontrol (Python/C++).
- `launch/` : Launch file kontrol.

---

## Cara Pakai

**Jalankan teleop:**
```sh
ros2 launch huskybot_control teleop.launch.py
```

**Parameter penting:**
- `max_speed`: Kecepatan maksimum robot.
- `safety_stop`: Aktifkan fitur emergency stop.

---

## Saran CI
- Tambahkan unit test untuk node kontrol.
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python.

---

## Catatan
Pastikan node ini berjalan bersamaan dengan mapping dan navigation untuk autonomous mode.

## Diagram Alur Kontrol

```
[Joystick/Planner] --> [huskybot_control] --> /cmd_vel --> [Robot/Gazebo]
```

## Contoh Parameter YAML

```yaml
max_speed: 1.0
safety_stop: true
```

## Troubleshooting

- Jika robot tidak bergerak, cek topic `/cmd_vel` dan remapping.
- Jika safety stop aktif terus, cek sensor obstacle.

## Mapping Joystick ke Perintah Gerak

- **Arah maju/mundur:** Stick kiri atas/bawah → `linear.x`
- **Belok kiri/kanan:** Stick kiri kiri/kanan → `angular.z`
- **Tombol lain:** (tambahkan sesuai kebutuhan)

Contoh di kode:
```python
vel_msg.linear.x = float(data.axes[1])  # maju/mundur
vel_msg.angular.z = float(data.axes[0]) # belok kiri/kanan
```