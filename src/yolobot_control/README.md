# yolobot_control

[![Build Status](https://github.com/yourusername/yolobot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/yolobot/actions)

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
ros2 launch yolobot_control teleop.launch.py
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