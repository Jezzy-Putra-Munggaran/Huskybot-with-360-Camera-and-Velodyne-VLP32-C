# Yolobot: Sistem Deteksi Halangan 360° + 3D LiDAR untuk Kendaraan Otonom

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C/actions)

---

## Judul Penelitian

**Pengembangan Sistem Deteksi Halangan Berbasis Kamera 360 Derajat dan 3D LiDAR untuk Kendaraan Otonom**

Proyek ini bertujuan mengembangkan sistem deteksi halangan menggunakan kombinasi kamera array 360° dan sensor 3D LiDAR. Sistem mampu mendeteksi dan mengklasifikasikan halangan di sekitar kendaraan otonom secara akurat dan komprehensif, meningkatkan kemampuan kendaraan untuk menghindari rintangan dan meningkatkan keselamatan navigasi.

---

## Daftar Isi

- [Hardware](#hardware)
- [Software & Dependency](#software--dependency)
- [Instalasi & Setup](#instalasi--setup)
- [Build Workspace](#build-workspace)
- [Struktur Folder](#struktur-folder)
- [Instruksi Menjalankan Simulasi](#instruksi-menjalankan-simulasi)
- [Checklist Fitur](#checklist-fitur)
- [Troubleshooting](#troubleshooting)
- [Saran & Rekomendasi](#saran--rekomendasi)
- [Lisensi](#lisensi)

---

## Hardware

- **Nvidia Jetson AGX Orin 32GB**
- **3D LiDAR:** Velodyne VLP-32C
- **Kamera 360°:** 6x Arducam IMX477 (hexagonal)
- **Platform Robot:** Clearpath Husky A200

---

## Software & Dependency

- **OS:** Ubuntu 22.04.5 LTS
- **ROS2:** Humble Hawksbill
- **Gazebo:** versi 11
- **RVIZ2**
- **Visual Studio Code**
- **Roboflow**
- **YOLOv8/YOLOv12**
- **Python 3.10+**
- **libpcap-dev** (untuk Velodyne)
- **OpenCV, numpy, PyYAML, cv_bridge, dll** (lihat requirements.txt jika ada)

---

## Instalasi & Setup

### 1. **Install Ubuntu 22.04.5 LTS**

### 2. **Install ROS2 Humble**
```sh
sudo apt update && sudo apt upgrade
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install ros-humble-desktop python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

### 3. **Install Gazebo 11**
```sh
sudo apt install gazebo11 libgazebo11-dev
```

### 4. **Install Dependency Lain**
```sh
sudo apt install python3-pip python3-colcon-common-extensions python3-vcstool python3-opencv python3-numpy python3-yaml python3-pyqt5 libpcap-dev
pip3 install opencv-python roboflow
```

### 5. **Clone Repo Ini**
```sh
git clone https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C.git
cd Yolobot-with-360-Camera-and-Velodyne-VLP32-C
```

### 6. **Source ROS2**
```sh
source /opt/ros/humble/setup.bash
```

---

## Build Workspace

```sh
cd Yolobot-with-360-Camera-and-Velodyne-VLP32-C
colcon build
source install/setup.bash
```

---

## Struktur Folder

```
src/
  yolobot_description/      # URDF/Xacro robot, mesh, kalibrasi
  yolobot_gazebo/          # Launch file simulasi Gazebo
  yolobot_control/         # Node kontrol & safety
  yolobot_recognition/     # Stitcher panorama & YOLOv8 inference
  yolobot_fusion/          # Fusion 2D-3D (kamera-LiDAR)
  yolobot_mapping/         # Mapping (Cartographer)
  yolobot_navigation/      # Navigation & obstacle avoidance
  yolov8_msgs/             # Custom message YOLOv8
  velodyne/                # Driver & pointcloud Velodyne
```

---

## Instruksi Menjalankan Simulasi

### 1. **Jalankan Gazebo + Robot + Sensor**
```sh
ros2 launch yolobot_gazebo yolobot_launch.py
```

### 2. **Jalankan Node Mapping (3D Mapping dengan LiDAR)**
```sh
ros2 launch yolobot_mapping mapping.launch.py
```

### 3. **Jalankan Node Stitcher (Panorama 360°)**
```sh
ros2 run yolobot_recognition yolov8_stitcher_node.py
# atau jika ada launch file:
ros2 launch yolobot_recognition panorama_stitcher.launch.py
```

### 4. **Jalankan Node Deteksi YOLOv8 (Object Detection 360°)**
```sh
ros2 launch yolobot_recognition launch_yolov8.launch.py
```

### 5. **Jalankan Node Fusion (2D-3D Kamera & LiDAR)**
```sh
ros2 launch yolobot_fusion fusion.launch.py
```

### 6. **Jalankan Node Navigation (Obstacle Avoidance & Path Planning)**
```sh
ros2 launch yolobot_navigation navigation.launch.py
```

### 7. **Jalankan Node Kontrol Robot (Joystick/Safety)**
```sh
ros2 launch yolobot_control yolobot_control.launch.py
```

### 8. **Visualisasi di RViz2**
```sh
rviz2
```
- Tambahkan topic: `/map`, `/velodyne_points`, `/panorama/image_raw`, `/fusion/objects3d`, dll.

---

## Checklist Fitur

| Target Penelitian                        | Status | Node/Launch File                                      |
|------------------------------------------|:------:|-------------------------------------------------------|
| 3D Mapping                              |   ✅   | `yolobot_mapping/mapping.launch.py`                   |
| Model Deteksi Halangan (YOLOv8)          |   ✅   | `yolobot_recognition/launch_yolov8.launch.py`         |
| Obstacle Avoidance                       |   ✅   | `yolobot_navigation/navigation.launch.py`             |
| Algoritma Pengukuran Jarak (2D-3D)       |   ✅   | `yolobot_fusion/fusion.launch.py`                     |
| Algoritma Penghitungan Koordinat Posisi  |   ✅   | `yolobot_fusion/fusion.launch.py`                     |

---

## Troubleshooting

- **Build error:**  
  Pastikan sudah install semua dependency dan source ROS2.
- **Kamera tidak tampil di rqt:**  
  Cek plugin kamera di URDF/Xacro, cek resource komputer.
- **Stitching panorama cembung/fisheye:**  
  Pastikan file kalibrasi intrinsic kamera sudah benar.
- **Velodyne error `pcap.h`:**  
  Install `libpcap-dev`.
- **Topic tidak muncul:**  
  Cek dengan `ros2 topic list` dan pastikan semua node sudah jalan.

---

## Saran & Rekomendasi

- **Backup folder `calibration/`, `dataset/`, dan `config/` secara rutin.**
- **Tambahkan unit test dan CI/CD untuk setiap package.**
- **Dokumentasikan pipeline dan urutan launch di README.**
- **Gunakan file YAML untuk parameter tuning (threshold, dsb).**
- **Simpan log hasil deteksi dan mapping untuk evaluasi riset.**
- **Tambahkan troubleshooting section di setiap README package.**
- **Jika ingin otomatis, buat launch file gabungan untuk semua node utama.**

---

## Lisensi

MIT License

---

**Repo ini: [https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C](https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C)**

---

> Untuk pertanyaan, kontribusi, atau kolaborasi, silakan buka [Issues](https://github.com/Jezzy-Putra-Munggaran/Yolobot-with-360-Camera-and-Velodyne-VLP32-C/issues) di repo ini.
