# ===================== BASE IMAGE =====================
FROM nvcr.io/nvidia/l4t-base:r36.2.0

LABEL maintainer="Jezzy Putra Munggaran <mungguran.jezzy.putra@gmail.com>"
LABEL description="Docker image for Huskybot 360° + 3D LiDAR AI research (ROS2 Humble, YOLOv12 ONNX/TensorRT, Jetson Orin, JetPack 6)"

# ===================== ENV & LOCALE =====================
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ===================== SYSTEM & ROS2 DEPENDENCIES =====================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales software-properties-common \
        python3-pip python3-opencv python3-numpy python3-yaml python3-pyqt5 \
        libpcap-dev \
        build-essential terminator gdb \
        ros-humble-desktop \
        ros-humble-vision-opencv \
        ros-humble-velodyne* \
        ros-humble-gazebo-* \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-joy \
        ros-humble-controller-manager \
        ros-humble-robot-state-publisher \
        ros-humble-ament-cmake \
        ros-humble-colcon-common-extensions \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-ament-pep257 \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
        python3-pyqt5 python3-pyside2.qtcore python3-pyside2.qtgui python3-pyside2.qtnetwork \
        python3-pyside2.qtwidgets python3-pyside2.qtopengl \
        python3-pandas python3-matplotlib python3-scipy python3-pillow python3-tqdm \
        python3-scikit-image python3-scikit-learn python3-open3d python3-pyquaternion \
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

# ===================== PYTHON DEPENDENCIES (INCLUDE ULTRALYTICS) =====================
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
    pyquaternion \
    opencv-python \
    ultralytics[export]

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
RUN python3 -c "import ultralytics; print('Ultralytics:', ultralytics.__version__)" || (echo '[ERROR] Ultralytics not installed!' && exit 1)

# ===================== COPY WORKSPACE & BUILD =====================
COPY . /workspace
WORKDIR /workspace

RUN . /opt/ros/humble/setup.sh && \
    colcon build --event-handlers console_direct+ --parallel-workers 1 || (cat log/latest_build/* && exit 1)

# ===================== SOURCE ENVIRONMENT OTOMATIS =====================
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

# ===================== VOLUME UNTUK DATA & LOG =====================
VOLUME ["/workspace/log", "/workspace/dataset", "/workspace/calibration"]

# ===================== ENTRYPOINT =====================
CMD ["/bin/bash"]

# ===================== HEALTHCHECK (OPSIONAL) =====================
HEALTHCHECK CMD python3 -c "import torch, onnxruntime, cv2, rclpy, ultralytics"
