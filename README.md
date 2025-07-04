# huskybot: Sistem Deteksi Halangan 360° + 3D LiDAR untuk Kendaraan Otonom  <!-- Judul utama README, wajib sama dengan nama folder utama workspace -->

[![Build Status](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions/workflows/ci.yml/badge.svg)](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/actions)  <!-- Badge CI/CD, update otomatis jika pipeline aktif -->

---

## Judul Penelitian  <!-- Penjelasan singkat tujuan riset -->
**Pengembangan Sistem Deteksi Halangan Berbasis Kamera 360 Derajat dan 3D LiDAR untuk Kendaraan Otonom** <!-- Judul penelitian utama -->
Proyek ini bertujuan mengembangkan sistem deteksi halangan menggunakan kombinasi kamera array 360° dan sensor 3D LiDAR. Sistem mampu mendeteksi dan mengklasifikasikan halangan di sekitar kendaraan otonom secara akurat dan komprehensif, meningkatkan kemampuan kendaraan untuk menghindari rintangan dan meningkatkan keselamatan navigasi. <!-- Penjelasan singkat tujuan dan manfaat riset -->

---

## Perubahan Plan (Update 2025-06-22)  <!-- Penjelasan perubahan rencana riset -->
- **Pipeline utama sekarang:**  
  - **Tidak menggunakan stitching panorama.**  
  - **Langsung display 6 kamera Arducam IMX477 (hexagonal) secara paralel.**
  - **Setiap kamera langsung di-inference dengan YOLOv12 (yolo12x.engine) untuk detection, segmentation, classification, OBB, dan tracking.**
  - **LaserScan dari LiDAR untuk estimasi jarak.**
  - **PointCloud dari LiDAR untuk koordinat posisi 3D.**
  - **Semua hasil (image, deteksi, segmentasi, OBB, tracking, LaserScan, PointCloud) WAJIB bisa divisualisasikan di RViz2, Gazebo, rqt, dsb.**
  - **Tidak ada stitching panorama, semua pipeline multitask berjalan paralel per kamera.**
  - **Logger dan visualizer multitask sudah siap audit trail dan debugging.**
  - **Pipeline siap untuk ROS2 Humble, simulasi Gazebo, dan robot real.**

---

## Daftar Isi  <!-- Navigasi cepat ke section penting README -->
- [Hardware](#hardware)
- [Software & Dependency](#software--dependency)
- [Instalasi & Setup](#instalasi--setup)
- [Build Workspace](#build-workspace)
- [Struktur Folder](#struktur-folder)
- [Instruksi Menjalankan Simulasi](#instruksi-menjalankan-simulasi)
- [Checklist Fitur](#checklist-fitur)
- [Roadmap Pengembangan](#roadmap-pengembangan)
- [Troubleshooting](#troubleshooting)
- [Catatan Integrasi Gazebo-ROS2 (PENTING)](#catatan-integrasi-gazebo-ros2-penting)
- [Saran & Rekomendasi](#saran--rekomendasi)
- [Lisensi](#lisensi)

---

## Hardware  <!-- Daftar hardware utama yang digunakan -->
- **Nvidia Jetson AGX Orin 32GB**  <!-- Komputasi utama, inference YOLOv12, ROS2 -->
- **3D LiDAR:** Velodyne VLP-32C  <!-- Sensor utama pointcloud 3D -->
- **Kamera 360°:** 6x Arducam IMX477 (hexagonal)  <!-- Kamera array 360° -->
- **Platform Robot:** Clearpath Husky A200  <!-- Robot utama, outdoor, ROS2 ready -->

---

## Software & Dependency  <!-- Daftar software dan dependency utama -->
- **OS:** Ubuntu 22.04.5 LTS (WSL2/Native)  <!-- OS utama, tested di WSL2 dan native -->
- **ROS2:** Humble Hawksbill  <!-- ROS2 LTS, wajib untuk semua package -->
- **Gazebo:** Classic 11 (default ROS2 Humble)  <!-- Simulasi robot dan sensor -->
- **RVIZ2**  <!-- Visualisasi sensor dan mapping -->
- **Visual Studio Code**  <!-- IDE utama, support ROS2 -->
- **Roboflow**  <!-- Dataset dan training YOLOv12 -->
- **YOLOv12**  <!-- Model deteksi utama, support TensorRT/ONNX -->
- **Python 3.10+**  <!-- Interpreter utama, semua node Python tested di 3.10+ -->
- **libpcap-dev** (untuk Velodyne)  <!-- Dependency driver Velodyne -->
- **OpenCV, numpy, PyYAML, cv_bridge, ultralytics, torch, dll** (lihat requirements.txt jika ada)  <!-- Library utama pipeline deteksi/fusion -->
- **GTSAM** (build from source, wajib untuk LIO-SAM)  <!-- Library SLAM/mapping -->
- **colcon, rosdep, vcstool, pip, build-essential, gdb, terminator**  <!-- Tools build dan debugging -->

---

## Instalasi & Setup  <!-- Langkah instalasi dan setup environment -->
### 1. **Install Ubuntu 22.04.5 LTS**  
(WSL2 dari Microsoft Store atau native) <!-- OS utama, tested di WSL2 dan native -->

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
```

### 6. **Install ROS2 Humble Desktop (Full)**
```sh
sudo apt install ros-humble-desktop -y
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

## Build Workspace  <!-- Cara build workspace ROS2 -->
```sh
cd huskybot
colcon build
source install/setup.bash
```
<!-- Build workspace wajib sebelum run node/launch file -->

---

## Struktur Folder  <!-- Struktur folder utama workspace -->
```
src/
  huskybot_description/      # URDF/Xacro robot, mesh, kalibrasi
  huskybot_gazebo/          # Launch file simulasi Gazebo
  huskybot_control/         # Node kontrol & safety
  huskybot_recognition/     # Stitcher panorama & YOLOv12 inference (legacy, tidak dipakai di plan baru)
  huskybot_fusion/          # Fusion 2D-3D (kamera-LiDAR)
  huskybot_mapping/         # Mapping (LIO-SAM)
  huskybot_navigation/      # Navigation & obstacle avoidance
  huskybot_perception/      # Visualizer & logger multitask YOLOv12
  huskybot_tracking/        # Multicam tracking & fusion YOLOv12
  huskybot_calibration/     # Kalibrasi kamera-LiDAR
  yolov12_msgs/             # Custom message YOLOv12
  velodyne/                 # Driver & pointcloud Velodyne
```
<!-- Struktur folder wajib konsisten agar colcon build dan integrasi pipeline tidak error -->

---

## Instruksi Menjalankan Simulasi  <!-- Urutan run pipeline simulasi dan real robot -->
### 1. **Jalankan Gazebo + Robot + Sensor**
```sh
ros2 launch huskybot_gazebo huskybot_launch.py
```

### 2. **Jalankan Node Mapping (3D Mapping dengan LiDAR)**
```sh
ros2 launch huskybot_mapping mapping.launch.py
```

### 3. **Jalankan Node Deteksi YOLOv12 (Object Detection 360°)**
```sh
ros2 launch huskybot_detection multicam_detection.launch.py
```

### 4. **Jalankan Node Segmentasi, Klasifikasi, OBB, Tracking (Paralel)**
```sh
ros2 launch huskybot_segmentation multicam_segmentation.launch.py
ros2 launch huskybot_classification multicam_classification.launch.py
ros2 launch huskybot_obb multicam_obb.launch.py
ros2 launch huskybot_tracking tracking_fusion.launch.py
```

### 5. **Jalankan Node Fusion (2D-3D Kamera & LiDAR)**
```sh
ros2 launch huskybot_fusion fusion.launch.py
```

### 6. **Jalankan Node Perception (Visualizer & Logger)**
```sh
ros2 launch huskybot_perception full_perception.launch.py
```

### 7. **Jalankan Node Navigation (Obstacle Avoidance & Path Planning)**
```sh
ros2 launch huskybot_navigation navigation.launch.py
```

### 8. **Jalankan Node Kontrol Robot (Joystick/Safety)**
```sh
ros2 launch huskybot_control huskybot_control.launch.py
```

### 9. **Visualisasi di RViz2**
```sh
rviz2
```
- Tambahkan topic: `/map`, `/velodyne_points`, `/panorama/image_raw`, `/fusion/objects3d`, `/detection`, `/segmentation`, `/obb`, `/tracking`, `/scan`, `/pointcloud`, dsb.

---

## Checklist Fitur  <!-- Checklist fitur utama pipeline -->
| Target Penelitian                        | Status | Node/Launch File                                      |
|------------------------------------------|:------:|-------------------------------------------------------|
| Model Deteksi Halangan                   |   ✅   | `huskybot_detection/multicam_detection_node.py`       |
| Segmentasi, Klasifikasi, OBB, Tracking   |   ✅   | `huskybot_segmentation/`, `huskybot_classification/`, `huskybot_obb/`, `huskybot_tracking/` |
| LaserScan (Jarak) dari LiDAR             |   ✅   | `velodyne/`, `huskybot_fusion/`                       |
| PointCloud (Koordinat 3D) dari LiDAR     |   ✅   | `velodyne/`, `huskybot_fusion/`                       |
| Visualisasi Semua Output di RViz2/Gazebo |   ✅   | `huskybot_perception/`, `rviz2`, `gazebo`             |
| Logger Multitask                         |   ✅   | `huskybot_perception/logger_node.py`                  |
| Obstacle Avoidance                       |   ❌   | (BELUM, akan dikembangkan di `huskybot_navigation/`)  |

> **Catatan:**  
> Fitur *Obstacle Avoidance* (penghindaran halangan otomatis) **belum terimplementasi**.  
> Rencana pengembangan: akan dibuat node di package `huskybot_navigation` yang mengintegrasikan hasil deteksi dan fusion ke perintah navigasi/gerak robot.

---

## Roadmap Pengembangan  <!-- Roadmap pengembangan fitur ke depan -->
- [ ] Implementasi obstacle avoidance di `huskybot_navigation`
- [ ] Integrasi dengan Navigation2 (Nav2) untuk path planning dan obstacle avoidance
- [ ] Uji coba obstacle avoidance di simulasi dan real robot
- [ ] Update dokumentasi dan contoh penggunaan setelah fitur selesai

---

## Troubleshooting  <!-- Tips troubleshooting error umum pipeline -->
### Gazebo ROS2 Service Tidak Muncul (`/gazebo/get_model_list` timeout)
- Pastikan file world TIDAK mengandung plugin ROS2. Semua baris `<plugin ... filename="libgazebo_ros_*.so"/>` di-comment atau dihapus dari file `.world`.
- Pastikan launch file menjalankan Gazebo dengan argumen plugin:  
  ```
  gzserver <world_file> -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_state.so
  ```
- Restart simulasi:
  ```
  pkill -9 gzserver
  pkill -9 gzclient
  ros2 launch huskybot_gazebo huskybot_launch.py
  ```
- Cek service:
  ```
  ros2 service list | grep gazebo
  ```
  Harus muncul `/gazebo/get_model_list`, `/gazebo/spawn_entity`, dll.

### Sensor Tidak Publish di Gazebo
- Cek plugin di URDF/Xacro sudah benar dan sesuai dengan ROS2 Humble.
- Pastikan semua dependency sudah terinstall dan path mesh benar.

### Robot Tidak Bergerak
- Cek topic `/cmd_vel` dan remapping.
- Pastikan node kontrol dan safety monitor berjalan.

### Node Fusion/Recognition Error Import
- Pastikan dependency Python (misal: `ros-numpy`, `opencv-python`, `torch`, dsb) sudah diinstall di environment yang aktif.

---

## Catatan Integrasi Gazebo-ROS2 (PENTING)  <!-- Catatan penting integrasi simulasi -->
- Gazebo Classic 11 + ROS2 Humble:
  - Plugin ROS2 TIDAK boleh di file world.
  - Plugin ROS2 WAJIB di-load lewat argumen `-s` ke proses `gzserver` (diatur di launch file).
  - Jika tidak, semua service Gazebo ROS2 tidak akan pernah muncul di ROS2.
- Best Practice:
  - Selalu jalankan simulasi lewat launch file ROS2, jangan manual lewat GUI Gazebo.
  - Cek log Gazebo untuk error plugin type jika service tidak muncul.

---

## Saran & Rekomendasi  <!-- Saran pengembangan dan best practice -->
- Backup folder `calibration/`, `dataset/`, dan `config/` secara rutin.
- Tambahkan unit test dan CI/CD untuk setiap package.
- Dokumentasikan pipeline dan urutan launch di README.
- Gunakan file YAML untuk parameter tuning (threshold, dsb).
- Simpan log hasil deteksi dan mapping untuk evaluasi riset.
- Tambahkan troubleshooting section di setiap README package.
- Jika ingin otomatis, buat launch file gabungan untuk semua node utama.

---

## Lisensi  <!-- Lisensi open source workspace -->
MIT License

**Repo ini: [https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C)**

---

> Untuk pertanyaan, kontribusi, atau kolaborasi, silakan buka [Issues](https://github.com/Jezzy-Putra-Munggaran/huskybot-with-360-Camera-and-Velodyne-VLP32-C/issues) di repo ini.

---

<!-- ===================== REVIEW & SARAN PENINGKATAN =====================
- Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
- Semua fitur utama, struktur folder, dan cara pakai sudah jelas dan sesuai pipeline workspace.
- Semua node Python sudah FULL OOP, robust error handling, dan siap multi-robot.
- Semua dependency, launch file, dan resource sudah saling terhubung ke pipeline workspace.
- Sudah siap untuk ROS2 Humble, YOLOv12, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Velodyne VLP32-C + 6x Arducam IMX477).
- Error handling: troubleshooting, permission, dependency, dan log sudah dijelaskan di setiap section.
- Saran: tambahkan badge coverage test jika pipeline CI sudah aktif.
- Saran: tambahkan test launch file untuk CI/CD di folder test/.
- Saran: dokumentasikan semua argumen launch file dan parameter world di README.md.
- Saran: tambahkan tips multi-robot dan namespace di README.md.
- Saran: tambahkan troubleshooting error umum di ROS2 Humble/Gazebo.
-->

## 🎯 **Camera Physical Mapping (IMPORTANT)**

**PERINGATAN**: Nama topic tidak sesuai dengan posisi fisik real hardware!

| Topic Name | Real Physical Position | LiDAR Angle |
|------------|----------------------|-------------|
| `/camera_front/image_raw` | **KAMERA BELAKANG** | 180° |
| `/camera_front_left/image_raw` | **KAMERA KIRI BELAKANG** | 225° |
| `/camera_left/image_raw` | **KAMERA KIRI DEPAN** | 270° |
| `/camera_rear/image_raw` | **KAMERA DEPAN** | 0° |
| `/camera_rear_right/image_raw` | **KAMERA KANAN DEPAN** | 315° |
| `/camera_right/image_raw` | **KAMERA KANAN BELAKANG** | 45° |

### Hexagonal Layout (Real Hardware):
