# huskybot_description  <!-- Judul utama README, nama package (harus sama dengan folder) -->

[![Build Status](https://github.com/yourusername/huskybot/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/huskybot/actions) <!-- Badge CI, update link jika repo sudah publik dan pipeline aktif -->

Deskripsi model robot Husky A200 beserta mounting sensor (Velodyne VLP-32C dan 6 kamera Arducam IMX477) dalam format URDF/Xacro. <!-- Deskripsi singkat package, menjelaskan fungsi utama dan sensor utama -->

---

## Fitur  <!-- Daftar fitur utama package -->
- Model robot Husky A200 lengkap dengan semua link dan joint. <!-- Menjelaskan model robot sudah lengkap -->
- Mounting Velodyne VLP-32C dan 6 kamera 360° (hexagonal). <!-- Menjelaskan mounting sensor utama -->
- Frame transform (TF) yang konsisten untuk integrasi sensor. <!-- Menjelaskan TF sudah konsisten untuk integrasi -->
- File kalibrasi sensor (opsional). <!-- Menjelaskan ada file kalibrasi jika diperlukan -->

---

## Struktur Folder  <!-- Penjelasan struktur folder utama package -->
- `robot/` : File Xacro/URDF robot dan sensor. <!-- Folder robot berisi file deskripsi utama -->
- `meshes/` : Mesh visual sensor dan robot. <!-- Folder meshes berisi file mesh 3D sensor dan robot -->
- `calibration/` : File kalibrasi kamera (INTRINSIC SAJA, legacy). <!-- Folder calibration hanya untuk file intrinsic kamera, legacy -->
- `launch/` : Launch file Python untuk RViz2, spawn robot, dsb. <!-- Folder launch berisi file launch Python -->
- `rviz/` : File konfigurasi RViz2. <!-- Folder rviz berisi konfigurasi visualisasi RViz -->
- `README.md` : Dokumentasi package ini. <!-- File ini sendiri sebagai dokumentasi utama -->
- `CMakeLists.txt`, `package.xml` : Konfigurasi build dan dependency ROS2. <!-- File build system dan dependency -->

> **Catatan:**  
> File kalibrasi extrinsic (kamera-LiDAR) **sekarang disimpan di** `huskybot_calibration/config/extrinsic_lidar_to_camera.yaml` **bukan di** `calibration/` **lagi!**  
> Semua node fusion dan pipeline lain WAJIB membaca file extrinsic dari path baru tersebut.

---

## Cara Pakai  <!-- Cara menjalankan package ini di ROS2 Humble/Gazebo -->

**Load URDF ke RViz2:** <!-- Instruksi menjalankan visualisasi URDF di RViz2 -->
```sh
ros2 launch huskybot_description display.launch.py
```

**Spawn robot ke Gazebo:** <!-- Instruksi menjalankan spawn robot ke simulasi Gazebo -->
```sh
ros2 launch huskybot_description spawn_huskybot_launch.launch.py
```

**Parameter penting:** <!-- Penjelasan parameter penting yang sering digunakan -->
- `robot_description`: Path ke file Xacro/URDF utama.
- `use_sim_time`: Gunakan waktu simulasi (default: true untuk simulasi).
- `entity_name`: Nama entity robot di Gazebo (untuk multi-robot).
- `robot_namespace`: Namespace robot (untuk multi-robot).
- `pose`: Pose awal robot di Gazebo (x y z roll pitch yaw).

---

## Contoh Frame TF  <!-- Contoh struktur TF yang dihasilkan oleh URDF/Xacro ini -->
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
<!-- Penjelasan: Semua frame sensor sudah konsisten, siap untuk integrasi mapping, fusion, dsb -->

---

## Saran CI  <!-- Saran best practice untuk CI/CD agar package selalu aman -->
- Tambahkan test URDF dengan [xacro test](http://wiki.ros.org/xacro#Testing). <!-- Saran test otomatis validasi Xacro -->
- Tambahkan check URDF dengan [check_urdf](http://wiki.ros.org/check_urdf). <!-- Saran test otomatis validasi URDF -->
- Tambahkan unit test/launch test di folder `test/` untuk integrasi CI/CD. <!-- Saran test launch file otomatis -->
- Gunakan [ament_lint_auto](https://index.ros.org/p/ament_lint_auto/) untuk linting kode Python dan XML. <!-- Saran linting otomatis -->

---

## Catatan  <!-- Catatan penting untuk integrasi workspace -->
Pastikan semua frame (`base_link`, `velodyne_link`, `camera_*_link`) konsisten dengan node lain di workspace. <!-- Penjelasan penting agar integrasi sensor dan node lain tidak error -->
Jika ingin menambah sensor baru, tambahkan mesh, link, joint, dan plugin di file Xacro, serta update file kalibrasi di `calibration/` untuk intrinsic kamera.  
**Untuk file kalibrasi extrinsic kamera-LiDAR, update hanya di** `huskybot_calibration/config/extrinsic_lidar_to_camera.yaml`.

---

## Diagram Arsitektur Sensor  <!-- Diagram visual arsitektur mounting sensor -->
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
<!-- Penjelasan: Velodyne di atas, 6 kamera di sisi tower hexagonal -->

---

## Contoh Dataset Kalibrasi  <!-- Contoh struktur dan isi file kalibrasi sensor -->
```
calibration/
  intrinsic_camera_front.yaml
  intrinsic_camera_left.yaml
  intrinsic_camera_right.yaml
  intrinsic_camera_rear.yaml
  intrinsic_camera_front_left.yaml
  intrinsic_camera_rear_right.yaml

huskybot_calibration/config/
  extrinsic_lidar_to_camera.yaml
```
<!-- Penjelasan: File intrinsic kamera tetap di calibration/, file extrinsic kamera-LiDAR sekarang di huskybot_calibration/config/ -->

**Contoh isi extrinsic_lidar_to_camera.yaml:** <!-- Contoh format file YAML kalibrasi extrinsic -->
```yaml
T_lidar_camera:
  rows: 4
  cols: 4
  data: [ ... 16 nilai matriks ... ]
```

---

## Parameter Penting  <!-- Daftar parameter penting untuk integrasi dan troubleshooting -->

- `robot_description`: Path ke file Xacro utama.
- `use_sim_time`: true/false.
- `robot_namespace`: Namespace robot (untuk multi-robot).
- `entity_name`: Nama entity robot di Gazebo.
- `pose`: Pose awal robot di Gazebo.

---

## Troubleshooting  <!-- Tips troubleshooting jika ada error saat simulasi/visualisasi -->

- Jika model tidak muncul di RViz2, cek path mesh dan urdf. <!-- Saran cek path file jika model tidak muncul -->
- Jika TF tidak lengkap, cek joint dan link di Xacro. <!-- Saran cek struktur joint/link jika TF error -->
- Jika sensor tidak publish di Gazebo, cek plugin dan remap topic di launch file. <!-- Saran cek plugin dan remap topic jika sensor tidak publish -->
- Jika spawn robot gagal, cek dependency xacro, gazebo_ros, dan permission file. <!-- Saran cek dependency dan permission jika spawn gagal -->
- Jika kalibrasi tidak terbaca, cek format dan path file YAML di `huskybot_calibration/config/` untuk extrinsic, dan di `calibration/` untuk intrinsic kamera. <!-- Saran cek file kalibrasi jika tidak terbaca node lain -->

---

## Saran Peningkatan  <!-- Saran untuk pengembangan dan maintain package ke depan -->
- Tambahkan folder `test/` untuk unit test dan launch test otomatis (CI/CD).
- Tambahkan file `CHANGELOG.md` dan `CONTRIBUTING.md` untuk dokumentasi dan kontribusi.
- Update badge CI jika repo sudah publik dan pipeline aktif.
- Dokumentasikan semua frame dan topic sensor di README agar integrasi lebih mudah.
- Tambahkan contoh penggunaan multi-robot di README.
- Tambahkan troubleshooting untuk error umum di ROS2 Humble/Gazebo.

---