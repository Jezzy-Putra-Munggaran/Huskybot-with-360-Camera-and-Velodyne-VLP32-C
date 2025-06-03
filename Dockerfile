# ===================== BASE IMAGE =====================
FROM nvcr.io/nvidia/l4t-base:r36.2.0

LABEL maintainer="Jezzy Putra Munggaran <mungguran.jezzy.putra@gmail.com>"
LABEL description="Docker image for Huskybot 360° + 3D LiDAR AI research (ROS2 Humble, YOLOv12 ONNX/TensorRT, Jetson Orin, JetPack 6)"

# ===================== ENV & LOCALE =====================
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales software-properties-common \
        python3-pip python3-opencv python3-numpy python3-yaml python3-pyqt5 \
        libpcap-dev \
        build-essential terminator gdb \
        ros-humble-desktop \
        ros-humble-vision-opencv \
        ros-humble-velodyne* \
        python3-msgpack python3-empy python3-pybind11 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ===================== ROS2 PYTHON TOOLS TAMBAHAN =====================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
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

# ===================== PYTHON DEPENDENCIES (NO ULTRALYTICS) =====================
RUN pip3 install --upgrade pip
RUN pip3 install --ignore-installed --no-cache-dir \
    onnxruntime-gpu \
    roboflow \
    PySide6 \
    matplotlib \
    scipy \
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
    pyquaternion

RUN pip3 cache purge

# ===================== PYTORCH, TORCHVISION, TORCHAUDIO (JETSON WHEEL) =====================
COPY wheels/torch-2.3.0-cp310-cp310-linux_aarch64.whl /tmp/
COPY wheels/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl /tmp/
COPY wheels/torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl /tmp/
RUN pip3 install --force-reinstall --ignore-installed /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install --force-reinstall --ignore-installed /tmp/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl && \
    pip3 install --force-reinstall --ignore-installed /tmp/torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl

# ===================== TEST IMPORT (OPSIONAL, DEBUGGING) =====================
RUN python3 -c "import torch; print('PyTorch:', torch.__version__, 'CUDA:', torch.cuda.is_available())" || (echo '[ERROR] PyTorch not installed!' && exit 1)
RUN python3 -c "import onnxruntime; print('ONNXRuntime:', onnxruntime.__version__)" || (echo '[ERROR] ONNXRuntime not installed!' && exit 1)
RUN python3 -c "import cv2; print('OpenCV:', cv2.__version__)" || (echo '[ERROR] OpenCV not installed!' && exit 1)
RUN python3 -c "import rclpy; print('rclpy OK')" || (echo '[ERROR] rclpy not installed!' && exit 1)

# ===================== WORKDIR & ENTRY =====================
WORKDIR /workspace

VOLUME ["/workspace/log", "/workspace/dataset", "/workspace/calibration"]

CMD ["/bin/bash"]
