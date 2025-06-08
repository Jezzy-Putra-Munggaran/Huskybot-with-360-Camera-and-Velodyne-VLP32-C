#!/bin/bash

# ===================== KONFIGURASI DAN PATH MODEL =====================
MODEL_DIR="$HOME/jezzy/huskybot/src/huskybot_recognition/scripts"  # Folder model YOLOv12
MODEL_ENGINE="$MODEL_DIR/yolo12n.engine"  # Path model YOLOv12 TensorRT (.engine, utama)
MODEL_ONNX="$MODEL_DIR/yolo12n.onnx"      # Path model YOLOv12 ONNX (backup)
MODEL_PT="$MODEL_DIR/yolo12n.pt"          # Path model YOLOv12 PyTorch (backup)
MODEL_URL_ENGINE="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12n.engine"  # URL download model .engine
MODEL_URL_ONNX="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12n.onnx"      # URL download model .onnx
MODEL_URL_PT="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12n.pt"          # URL download model .pt

# ===================== ERROR HANDLING: CEK PERMISSION FOLDER MODEL =====================
if [ ! -w "$MODEL_DIR" ]; then  # Cek permission tulis folder model
  echo "[ERROR] Tidak bisa menulis ke folder model: $MODEL_DIR"
  exit 4
fi

# ===================== FUNGSI DOWNLOAD MODEL DENGAN FALLBACK =====================
download_model() {
  local url="$1"  # URL model
  local path="$2" # Path file model
  local desc="$3" # Deskripsi model
  echo "[INFO] Model $desc belum ada, akan download dari $url ..."
  wget -O "$path" "$url"
  if [ $? -eq 0 ] && [ -f "$path" ]; then
    echo "[INFO] Model $desc berhasil didownload ke $path"
    return 0
  else
    echo "[WARNING] Gagal download model $desc!"
    return 1
  fi
}

# ===================== LOGIKA FALLBACK MODEL =====================
MODEL_PATH=""  # Path model yang akan dipakai
MODEL_FORMAT=""  # Format model yang akan dipakai

# 1. Coba .engine (TensorRT)
if [ ! -f "$MODEL_ENGINE" ]; then
  download_model "$MODEL_URL_ENGINE" "$MODEL_ENGINE" ".engine (TensorRT)"
fi
if [ -f "$MODEL_ENGINE" ]; then
  MODEL_PATH="$MODEL_ENGINE"
  MODEL_FORMAT="engine"
  echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.engine/TensorRT)"
else
  # 2. Coba .onnx (ONNX)
  if [ ! -f "$MODEL_ONNX" ]; then
    download_model "$MODEL_URL_ONNX" "$MODEL_ONNX" ".onnx (ONNX)"
  fi
  if [ -f "$MODEL_ONNX" ]; then
    MODEL_PATH="$MODEL_ONNX"
    MODEL_FORMAT="onnx"
    echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.onnx/ONNX)"
  else
    # 3. Coba .pt (PyTorch)
    if [ ! -f "$MODEL_PT" ]; then
      download_model "$MODEL_URL_PT" "$MODEL_PT" ".pt (PyTorch)"
    fi
    if [ -f "$MODEL_PT" ]; then
      MODEL_PATH="$MODEL_PT"
      MODEL_FORMAT="pt"
      echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.pt/PyTorch)"
    else
      echo "[FATAL] Tidak ada model YOLOv12 (.engine/.onnx/.pt) yang bisa dipakai!"
      exit 2
    fi
  fi
fi

# ===================== KILL GAZEBO JIKA MASIH ADA =====================
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null

# ===================== CEK ROS2 CLI =====================
if ! command -v ros2 &> /dev/null; then
  echo "[ERROR] ros2 CLI tidak ditemukan! Pastikan ROS2 sudah terinstall dan di-source."
  exit 3
fi

# ===================== SOURCE ENVIRONMENT ROS2 DAN WORKSPACE =====================
source /opt/ros/humble/setup.bash
source $HOME/jezzy/huskybot/install/setup.bash

# ===================== CEK PLUGIN ROS2_CONTROL =====================
PLUGIN_PATH="$HOME/jezzy/huskybot/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so"
if [ ! -f "$PLUGIN_PATH" ]; then
  echo "[ERROR] Plugin libgazebo_ros2_control.so tidak ditemukan di $PLUGIN_PATH!"
  echo "Pastikan sudah colcon build dan source workspace."
  exit 5
fi

# ===================== EXPORT ENVIRONMENT GAZEBO =====================
export GAZEBO_PLUGIN_PATH=$HOME/jezzy/huskybot/install/gazebo_ros/lib:$HOME/jezzy/huskybot/install/gazebo_plugins/lib:$HOME/jezzy/huskybot/install/gazebo_ros2_control/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$HOME/jezzy/huskybot/install/huskybot_description/share/huskybot_description/models:$HOME/jezzy/huskybot/install/husky_description/share/husky_description/models:$GAZEBO_MODEL_PATH

# ===================== CEK WORKSPACE LAIN YANG BENTROK =====================
if [[ "$GAZEBO_PLUGIN_PATH" == *"gazebo_ros2_ws"* ]]; then
  echo "[WARNING] Detected other workspace in GAZEBO_PLUGIN_PATH! Please clean your environment."
  exit 1
fi
if [[ "$LD_LIBRARY_PATH" == *"gazebo_ros2_ws"* ]]; then
  echo "[WARNING] Detected other workspace in LD_LIBRARY_PATH! Please clean your environment."
  exit 1
fi

echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

# ===================== CEK DEPENDENCY PYTHON UTAMA YOLOV12 =====================
PYTHON_DEPS=("numpy" "cv2" "ultralytics")
for dep in "${PYTHON_DEPS[@]}"; do
  python3 -c "import $dep" 2>/dev/null || { echo "[ERROR] Python dependency '$dep' belum terinstall!"; exit 6; }
done

# Print versi OpenCV untuk debugging
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"

# ===================== CEK GPU DAN TENSORRT (JETSON ORIN) =====================
if command -v nvidia-smi &> /dev/null; then
  nvidia-smi
else
  echo "[WARNING] nvidia-smi tidak ditemukan, pastikan Jetson Orin environment sudah benar."
fi
if python3 -c "import tensorrt" 2>/dev/null; then
  echo "[INFO] TensorRT Python module terdeteksi."
else
  echo "[WARNING] TensorRT Python module tidak ditemukan! Pastikan sudah install python3-tensorrt."
fi

# ===================== JALANKAN SIMULASI GAZEBO + ROBOT + SENSOR =====================
ros2 launch huskybot_gazebo huskybot_launch.py model_path:="$MODEL_PATH" model_format:="$MODEL_FORMAT"

echo "Log launch lengkap ada di: ~/.ros/log/latest_launch.log"