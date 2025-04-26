# huskybot: Sistem Deteksi Halangan 360° + 3D LiDAR untuk Kendaraan Otonom

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)

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
- **YOLOv12**
- **Python 3.10+**
- **libpcap-dev** (untuk Velodyne)
- **OpenCV, numpy, PyYAML, cv_bridge, dll** (lihat requirements.txt jika ada)

---

## Instalasi & Setup

### 1. **Install Ubuntu 22.04.5 LTS**

### 2. **Update Sistem**
```sh
sudo apt update
sudo apt upgrade -y
sudo apt install --only-upgrade libsystemd0 systemd udev libudev1 -y
```

### 3. **Install ROS2 Humble**
Ikuti panduan resmi di [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html):
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

### 4. **Install Gazebo 11**
Ikuti panduan resmi di [Gazebo Documentation](https://gazebosim.org/docs/latest/ros_installation):
```sh
sudo apt install gazebo11 libgazebo11-dev
```

### 5. **Install Dependency Lain**
```sh
sudo apt install python3-pip python3-colcon-common-extensions python3-vcstool python3-opencv python3-numpy python3-yaml python3-pyqt5 libpcap-dev
pip3 install opencv-python roboflow PyQt6 PySide6
```

### 6. **Install C++ Tools**
```sh
sudo apt install build-essential gdb
g++ --version
```

### 7. **Install Terminator**
```sh
sudo apt install terminator
```

### 8. **Install Visual Studio Code**
```sh
sudo snap install code --classic
sudo snap install --classic code
```
Tambahkan ekstensi:
- ROS Extensions
- CMake Extensions

### 9. **Set GitHub SSH**
Ikuti panduan di [GitHub SSH Documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

### 10. **Install Ultralytics**
Ikuti panduan di [Ultralytics Documentation](https://docs.ultralytics.com/quickstart/).

### 11. **Clone Repo Ini**
```sh
git clone https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C.git
cd huskybot-with-360-Camera-and-Velodyne-VLP32-C
```

### 12. **Source ROS2**
```sh
source /opt/ros/humble/setup.bash
```

### 13. **Install Husky Dependencies**
Ikuti panduan di [Husky Documentation](http://wiki.ros.org/Robots/Husky).

### 14. **Install Velodyne Dependencies**
Ikuti panduan di [Velodyne ROS2 Driver](https://github.com/ros-drivers/velodyne).

### 15. **Install LIO-SAM Dependencies**
Ikuti panduan di [LIO-SAM Documentation](https://github.com/TixiaoShan/LIO-SAM).

---

## Build Workspace

```sh
cd huskybot-with-360-Camera-and-Velodyne-VLP32-C
colcon build
source install/setup.bash
```

---

## Struktur Folder

```
src/
  huskybot_description/      # URDF/Xacro robot, mesh, kalibrasi
  huskybot_gazebo/          # Launch file simulasi Gazebo
  huskybot_control/         # Node kontrol & safety
  huskybot_recognition/     # Stitcher panorama & YOLOv12 inference
  huskybot_fusion/          # Fusion 2D-3D (kamera-LiDAR)
  huskybot_mapping/         # Mapping (LIO-SAM)
  huskybot_navigation/      # Navigation & obstacle avoidance
  yolov12_msgs/            # Custom message YOLOv12
  velodyne/                # Driver & pointcloud Velodyne
```

---

## Instruksi Menjalankan Simulasi

### 1. **Jalankan Gazebo + Robot + Sensor**
```sh
ros2 launch huskybot_gazebo huskybot_launch.py
```

### 2. **Jalankan Node Mapping (3D Mapping dengan LiDAR)**
```sh
ros2 launch huskybot_mapping mapping.launch.py
```

### 3. **Jalankan Node Stitcher (Panorama 360°)**
```sh
ros2 run huskybot_recognition yolov12_stitcher_node.py
# atau jika ada launch file:
ros2 launch huskybot_recognition panorama_stitcher.launch.py
```

### 4. **Jalankan Node Deteksi YOLOv12 (Object Detection 360°)**
```sh
ros2 launch huskybot_recognition launch_yolov12.launch.py
```

### 5. **Jalankan Node Fusion (2D-3D Kamera & LiDAR)**
```sh
ros2 launch huskybot_fusion fusion.launch.py
```

### 6. **Jalankan Node Navigation (Obstacle Avoidance & Path Planning)**
```sh
ros2 launch huskybot_navigation navigation.launch.py
```

### 7. **Jalankan Node Kontrol Robot (Joystick/Safety)**
```sh
ros2 launch huskybot_control huskybot_control.launch.py
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
| 3D Mapping                              |   ✅   | `huskybot_mapping/mapping.launch.py`                   |
| Model Deteksi Halangan (YOLOv12)         |   ✅   | `huskybot_recognition/launch_yolov12.launch.py`        |
| Obstacle Avoidance                       |   ✅   | `huskybot_navigation/navigation.launch.py`             |
| Algoritma Pengukuran Jarak (2D-3D)       |   ✅   | `huskybot_fusion/fusion.launch.py`                     |
| Algoritma Penghitungan Koordinat Posisi  |   ✅   | `huskybot_fusion/fusion.launch.py`                     |

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

**Repo ini: [https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C)**

---

> Untuk pertanyaan, kontribusi, atau kolaborasi, silakan buka [Issues](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/issues) di repo ini.
