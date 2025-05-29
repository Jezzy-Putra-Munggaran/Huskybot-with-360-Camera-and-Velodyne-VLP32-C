# ===================== BASE IMAGE =====================
FROM nvcr.io/nvidia/l4t-base:r36.2.0

LABEL maintainer="Jezzy Putra Munggaran <mungguran.jezzy.putra@gmail.com>"
LABEL description="Docker image for Huskybot 360° + 3D LiDAR AI research (ROS2 Humble, YOLOv12, Jetson Orin, JetPack 6)"

# ===================== ENV & LOCALE =====================
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe

# ===================== ADD ROS2 HUMBLE REPO & KEY =====================
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# ===================== ROS2 HUMBLE & TOOLS =====================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip python3-vcstool python3-rosdep \
        build-essential terminator gdb \
        python3-opencv python3-numpy python3-yaml python3-pyqt5 \
        libpcap-dev \
        ros-humble-desktop \
        ros-humble-vision-opencv \
        ros-humble-velodyne* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ===================== ROS2 PYTHON TOOLS TAMBAHAN =====================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-ros \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ===================== ROSDEP INIT (ERROR HANDLING) =====================
RUN set -e; \
    if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init || echo "[WARNING] rosdep init failed (maybe already initialized)"; \
    fi && \
    rosdep update || echo "[WARNING] rosdep update failed"

# ===================== PYTORCH, TORCHVISION, TORCHAUDIO, PYQT6 (JETSON WHEEL) =====================
COPY wheels/torch-2.3.0-cp310-cp310-linux_aarch64.whl /tmp/
COPY wheels/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl /tmp/
COPY wheels/torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl /tmp/
COPY wheels/PyQt6-6.9.0-cp310-cp310-manylinux_2_39_aarch64.whl /tmp/
RUN pip3 install --upgrade pip && \
    pip3 install /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install /tmp/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl && \
    pip3 install /tmp/torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl && \
    pip3 install /tmp/PyQt6-6.9.0-cp310-cp310-manylinux_2_39_aarch64.whl

# ===================== PYTHON DEPENDENCIES (AI/YOLO/UTILS) =====================
RUN pip3 install \
        ultralytics==8.1.0 \
        opencv-python \
        roboflow \
        PySide6 \
        matplotlib \
        scipy \
        setuptools \
        pytest \
        flake8 \
        pep257 \
        pyyaml \
        coverage \
        pillow \
        tqdm \
        scikit-image \
        scikit-learn \
        pandas \
        open3d \
        pyquaternion \
    && pip3 cache purge

# ===================== ERROR HANDLING: CEK VERSI DEPENDENCY =====================
RUN python3 -c "import torch; print('PyTorch:', torch.__version__)" || (echo '[ERROR] PyTorch not installed!' && exit 1)
RUN python3 -c "import ultralytics; print('Ultralytics:', ultralytics.__version__)" || (echo '[ERROR] Ultralytics not installed!' && exit 1)
RUN python3 -c "import cv2; print('OpenCV:', cv2.__version__)" || (echo '[ERROR] OpenCV not installed!' && exit 1)
RUN python3 -c "import rclpy; print('rclpy OK')" || (echo '[ERROR] rclpy not installed!' && exit 1)

# ===================== WORKDIR & ENTRY =====================
WORKDIR /workspace

# ===================== SUGGESTED VOLUME (LOG, DATASET, KALIBRASI) =====================
VOLUME ["/workspace/log", "/workspace/dataset", "/workspace/calibration"]

# ===================== DEFAULT CMD =====================
CMD ["/bin/bash"]

# ===================== SARAN PENINGKATAN (langsung diimplementasikan) =====================
# - Gunakan .dockerignore untuk exclude file yang tidak perlu (misal: *.md, .git, dataset besar)
# - Pin versi dependency Python (sudah, misal ultralytics==8.1.0)
# - Tambahkan LABEL untuk metadata image
# - Gunakan ENV untuk path penting (misal: ENV HUSKYBOT_DATASET=/workspace/dataset)
# - Tambahkan test import dependency di build (sudah)
# - Gunakan --no-cache saat build untuk update dependency
# - Jika ingin multi-stage build (misal: build C++ lalu copy ke image runtime), bisa dioptimasi lagi
# - Untuk CI/CD, build image di pipeline dan push ke registry internal
# - Untuk keamanan, bisa tambah USER non-root (opsional, jika tidak butuh akses hardware langsung)
# - Untuk GUI (Gazebo/RViz2), jalankan container dengan X11 forwarding atau VNC jika perlu

# ===================== END DOCKERFILE =====================
