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

- **OS:** Ubuntu 22.04.5 LTS (WSL2/Native)
- **ROS2:** Humble Hawksbill
- **Gazebo:** Classic 11 (default ROS2 Humble)  
  (Jika ingin Fortress/Harmonic, lihat [panduan Gazebo ROS2](https://gazebosim.org/docs/latest/ros_installation/))
- **RVIZ2**
- **Visual Studio Code**
- **Roboflow**
- **YOLOv12**
- **Python 3.10+**
- **libpcap-dev** (untuk Velodyne)
- **OpenCV, numpy, PyYAML, cv_bridge, dll** (lihat requirements.txt jika ada)
- **GTSAM** (build from source, wajib untuk LIO-SAM)
- **colcon, rosdep, vcstool, pip, build-essential, gdb, terminator**

---

## Instalasi & Setup

### 1. **Install Ubuntu 22.04.5 LTS**  
  (WSL2 dari Microsoft Store atau native)

### 2. **Update Sistem**
```sh
sudo apt update
sudo apt upgrade -y
sudo apt install --only-upgrade libsystemd0 systemd udev libudev1 -y
```

### 3. **Set Locale**
```sh
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 4. **Tambahkan Repository ROS2**
```sh
sudo apt install software-properties-common curl gnupg2 lsb-release -y
sudo add-apt-repository universe
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### 5. **Import GPG Key Gazebo (Jika Perlu)**
```sh
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://packages.osrfoundation.org/gazebo.key | sudo tee /etc/apt/keyrings/gazebo-archive-keyring.gpg > /dev/null
# Edit /etc/apt/sources.list.d/gazebo-latest.list agar menggunakan:
# deb [signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main
sudo apt update
```

### 6. **Install ROS2 Humble Desktop (Full)**
```sh
sudo apt install ros-humble-desktop -y
```

Tambahkan ke `~/.bashrc`:
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. **Install ROS Tools & Build Tools**
```sh
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip -y
```

### 8. **Inisialisasi rosdep**
```sh
sudo rosdep init
rosdep update
```

### 9. **Install Gazebo Classic 11 (default ROS2 Humble)**
```sh
sudo apt install ros-humble-gazebo-* -y
```
> Gazebo 11 adalah pairing default untuk ROS2 Humble di Ubuntu 22.04.

### 10. **Install Dependency Lain**
```sh
sudo apt install python3-opencv python3-numpy python3-yaml python3-pyqt5 libpcap-dev terminator build-essential gdb
pip3 install opencv-python roboflow PyQt6 PySide6 ultralytics
```

### 11. **Install Visual Studio Code**
```sh
sudo snap install --classic code
```
Tambahkan ekstensi: ROS, CMake, Python, dsb.

### 12. **Install GTSAM (Build from Source)**
```sh
sudo apt install git cmake build-essential libboost-all-dev libtbb-dev libeigen3-dev libmetis-dev
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2.0
rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
Jika workspace tidak menemukan GTSAM, tambahkan ke environment:
```sh
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/GTSAM' >> ~/.bashrc
source ~/.bashrc
```

### 13. **Clone Repo Ini**
```sh
git clone https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C.git huskybot
cd huskybot
```

### 14. **Source ROS2**
```sh
source /opt/ros/humble/setup.bash
```

### 15. **Build Workspace**
```sh
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```
Tambahkan ke `~/.bashrc` jika ingin otomatis:
```sh
echo "source ~/huskybot/install/setup.bash" >> ~/.bashrc
```

### 16. **Install Dependency Husky, Velodyne, LIO-SAM**
- **Husky:**  
  Tidak tersedia di ROS2 Humble binary, gunakan package dari [https://github.com/husky/husky](https://github.com/husky/husky) jika butuh.
- **Velodyne:**  
  ```sh
  sudo apt install ros-humble-velodyne* -y
  ```
- **LIO-SAM:**  
  Ikuti instruksi build dari [https://github.com/TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) (pastikan dependency GTSAM sudah terinstall).

---

## Build Workspace

```sh
cd huskybot
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
  yolov12_msgs/             # Custom message YOLOv12
  velodyne/                 # Driver & pointcloud Velodyne
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
- **GTSAM tidak ditemukan:**  
  Build dan install GTSAM dari source, pastikan `CMAKE_PREFIX_PATH` sudah benar.
- **Gazebo error GPG key:**  
  Import ulang key Gazebo dan pastikan repo pakai `[signed-by=...]`.
- **Build error dependency:**  
  Jalankan `rosdep install --from-paths src --ignore-src -r -y`.
- **Husky package tidak ada di ROS2:**  
  Build dari source, atau gunakan custom package di repo ini.
- **RViz2 tidak muncul:**  
  Pastikan `rviz2` sudah terinstall dan tidak ada error library.

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
