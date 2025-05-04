# huskybot_control                                                      <!-- Judul utama README, nama package (harus sama dengan folder) -->

[![Build Status](https://github.com/jezzy/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/jezzy/huskybot/actions) <!-- Badge CI, update link jika repo sudah publik dan pipeline aktif -->

Node kontrol robot Husky A200 (teleop/manual/autonomous) untuk simulasi dan real robot. <!-- Deskripsi singkat package, menjelaskan fungsi utama node -->

---

## Fitur
- Node kontrol kecepatan (`/cmd_vel`).                                 <!-- Node utama publish kecepatan ke topic /cmd_vel -->
- Integrasi dengan joystick atau autonomous planner.                   <!-- Bisa menerima input dari joystick atau planner autonomous -->
- Safety check (opsional).                                             <!-- Ada fitur safety monitor (misal: stop jika obstacle dekat) -->
- Logger/monitor node (opsional, update jika ada).                     <!-- Tambahkan jika ada fitur logger/monitor baru -->

---

## Struktur Folder
- `scripts/` : Script kontrol (Python/C++).                            <!-- Folder untuk script node Python/C++ -->
- `launch/` : Launch file kontrol.                                     <!-- Folder untuk launch file ROS2 -->
- `config/` : File konfigurasi parameter (jika ada).                   <!-- Folder untuk file YAML/LUA konfigurasi -->
- `test/` : Unit test (jika ada).                                      <!-- Folder untuk unit test -->

---

## Cara Pakai

**Jalankan teleop (manual/joystick):**
```sh
ros2 launch huskybot_control teleop.launch.py
```
<!-- Contoh perintah menjalankan teleop, pastikan launch file ada di folder launch/ -->

**Jalankan autonomous mode (planner):**
```sh
ros2 launch huskybot_control autonomous.launch.py
```
<!-- Contoh perintah autonomous, tambahkan jika sudah ada launch file autonomous -->

**Jalankan hanya safety monitor:**
```sh
ros2 launch huskybot_control safety_only.launch.py
```
<!-- Contoh perintah safety only, tambahkan jika sudah ada launch file khusus safety -->

---

## Parameter penting

- `max_speed`: Kecepatan maksimum robot.                               <!-- Parameter untuk membatasi kecepatan, bisa di-set dari launch/YAML -->
- `safety_stop`: Aktifkan fitur emergency stop.                        <!-- Parameter untuk mengaktifkan fitur safety stop -->
- `cmd_vel_topic`: Topic tujuan publish kecepatan (default: `/huskybot/cmd_vel`). <!-- Bisa di-remap via launch/parameter -->
- `scan_topic`: Topic input LaserScan untuk safety monitor (default: `/scan`).    <!-- Bisa di-remap via launch/parameter -->
- `param_yaml`: Path ke file YAML parameter (opsional).                <!-- Bisa digunakan untuk load parameter dari file YAML -->

---

## Saran CI
- Tambahkan unit test untuk node kontrol.                              <!-- Saran agar kualitas kode terjaga dengan unit test -->
- Gunakan [pytest](https://docs.pytest.org/en/stable/) untuk test Python. <!-- Rekomendasi framework test Python -->
- Tambahkan test launch file untuk autonomous dan safety mode.          <!-- Saran agar semua mode teruji otomatis -->

---

## Catatan
Pastikan node ini berjalan bersamaan dengan mapping dan navigation untuk autonomous mode. <!-- Catatan penting: node kontrol harus jalan bareng mapping/navigation agar autonomous berfungsi -->

Lihat dokumentasi workspace utama di [huskybot/README.md](../../../README.md) untuk instruksi lengkap simulasi dan integrasi. <!-- Link ke dokumentasi workspace utama -->

---

## Diagram Alur Kontrol

```
[Joystick/Planner] --> [huskybot_control] --> /cmd_vel --> [Robot/Gazebo]
```
<!-- Diagram sederhana alur data dari input ke output, sangat membantu pemahaman user baru -->

---

## Contoh Parameter YAML

```yaml
max_speed: 1.0
safety_stop: true
cmd_vel_topic: /huskybot/cmd_vel
scan_topic: /scan
```
<!-- Contoh file YAML untuk parameter, bisa digunakan di launch file dengan argumen param_yaml -->

---

## Troubleshooting

- Jika robot tidak bergerak, cek topic `/cmd_vel` dan remapping.        <!-- Saran troubleshooting jika robot tidak jalan -->
- Jika safety stop aktif terus, cek sensor obstacle.                    <!-- Saran troubleshooting jika robot selalu berhenti -->
- Jika autonomous tidak jalan, pastikan planner dan mapping aktif.      <!-- Saran troubleshooting untuk mode autonomous -->
- Jika parameter tidak terbaca, cek path dan format file YAML.          <!-- Saran troubleshooting parameter -->

---

## Mapping Joystick ke Perintah Gerak

- **Arah maju/mundur:** Stick kiri atas/bawah → `linear.x`              <!-- Penjelasan mapping joystick ke Twist -->
- **Belok kiri/kanan:** Stick kiri kiri/kanan → `angular.z`
- **Tombol lain:** (tambahkan sesuai kebutuhan)

Contoh di kode:
```python
vel_msg.linear.x = float(data.axes[1])  # maju/mundur
vel_msg.angular.z = float(data.axes[0]) # belok kiri/kanan
```
<!-- Contoh kode Python mapping joystick ke Twist, sesuai dengan implementasi di scripts/robot_control.py -->

---

## Kontribusi

- Pull request dan issue dipersilakan melalui [GitHub repo](https://github.com/jezzy/huskybot). <!-- Link kontribusi -->
- Lihat [CONTRIBUTING.md](https://github.com/jezzy/huskybot/blob/main/CONTRIBUTING.md) untuk panduan kontribusi. <!-- Panduan kontribusi jika ada -->

---

## Lisensi

[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) <!-- Lisensi package, sesuai dengan package.xml -->
