# huskybot_camera <!-- Nama package sebagai header -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/Huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions) <!-- Badge status build CI/CD -->

Node publisher untuk array 6 kamera Arducam IMX477 (konfigurasi heksagonal) pada robot Huskybot. <!-- Deskripsi singkat package -->
Mendukung konfigurasi multi-kamera dengan output gambar individual untuk pipeline deteksi 360° pada robot Huskybot. <!-- Detil fungsionalitas -->
Kompatibel dengan ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson AGX Orin + Velodyne VLP32-C). <!-- Kompatibilitas hardware -->

---

## Fitur <!-- Seksi fitur utama -->
- Mendukung 6 kamera Arducam IMX477 dalam konfigurasi heksagonal (Front, Front-Left, Left, Rear, Rear-Right, Right) <!-- Fitur utama - dukungan kamera hexagonal -->
- Optimasi khusus untuk Nvidia Jetson AGX Orin (deteksi otomatis platform) <!-- Fitur optimasi Jetson -->
- Fallback ke file video jika kamera fisik mengalami kendala <!-- Fitur fallback -->
- Auto-recovery untuk kamera yang kehilangan koneksi <!-- Fitur recovery otomatis -->
- Service API untuk status dan restart kamera <!-- Fitur API service -->
- Logging komprehensif untuk monitoring dan debugging <!-- Fitur logging -->
- Mendukung konfigurasi via file YAML atau parameter ROS2 <!-- Fleksibilitas konfigurasi -->
- Integrasi seamless dengan YOLOv12 untuk pipeline deteksi objek <!-- Integrasi dengan YOLOv12 -->
- Health check dan diagnostics otomatis untuk monitoring reliabilitas <!-- Fitur monitoring -->
- Thread-safe processing dengan multi-threaded executor <!-- Dukungan multi-thread -->

## Diagram Arsitektur <!-- Seksi diagram arsitektur -->

                         +------------------+
                         |                  |
                         | multicamera_node |  <-- Main node yang mengelola semua kamera
                         |                  |
                         +--------+---------+
                                  |
                 +---------------------------------+
                 |              |                  |
          +------+-------+      |           +------+-------+
          |              |      |           |              |
 +--------+--------+     |      |     +-----+-----------+  |
 |                 |     |      |     |                 |  |
 +----+----+ +-----+---+ | +----+----+ +----------+-+ | | | | | | | | | | Front | | Front | | | Rear | | Rear | | Camera | | Left | | | Camera | | Right | | | | Camera | | | | | Camera | +---------+ +---------+ | +---------+ +------------+ | +------+-------+ +-------------+ | | | | | Left Camera | | Right Camera| | | | | +--------------+ +-------------+

 <!-- Visual diagram showing the camera arrangement and node structure -->

## Integrasi Pipeline <!-- Pipeline integration section -->

<!-- Diagram showing how this package fits into the overall detection pipeline -->

## Instalasi <!-- Installation section -->

### Prerequisite
- ROS2 Humble Hawksbill <!-- Required ROS2 version -->
- Ubuntu 22.04 <!-- Required OS version -->
- OpenCV <!-- Required library -->
- Gazebo (untuk simulasi) <!-- Required for simulation -->

### Build
```bash
# Di root workspace
cd ~/huskybot
colcon build --packages-select huskybot_camera
source install/setup.bash
```