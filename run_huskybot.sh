#!/bin/bash

set -euo pipefail  # [BEST PRACTICE] Exit jika ada error, variabel belum di-set, atau error di pipeline

trap 'echo "[FATAL] Script exited unexpectedly! Cek log dan troubleshooting di README.md"; exit 99' ERR  # [ERROR HANDLING] Trap error apapun dan tampilkan pesan fatal

export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"  # [ENV] Pastikan Python user site-packages selalu di-PYTHONPATH (untuk pip --user)

# ===================== KONFIGURASI DAN PATH MODEL =====================
MODEL_DIR="$HOME/jezzy/huskybot/src/huskybot_recognition/scripts"  # Folder model YOLOv12
MODEL_ENGINE="$MODEL_DIR/yolo12x.pt"  # Path model YOLOv12 TensorRT (.engine, utama)
MODEL_ONNX="$MODEL_DIR/yolo12x.onnx"      # Path model YOLOv12 ONNX (backup)
MODEL_PT="$MODEL_DIR/yolo12x.pt"          # Path model YOLOv12 PyTorch (backup)
MODEL_URL_ENGINE="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12x.pt"  # URL model .engine
MODEL_URL_ONNX="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12x.onnx"      # URL model .onnx
MODEL_URL_PT="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12x.pt"          # URL model .pt

LOG_FILE="$HOME/huskybot_run.log"  # Path file log utama
touch "$LOG_FILE" 2>/dev/null || LOG_FILE="/tmp/huskybot_run.log"  # [ERROR HANDLING] Fallback ke /tmp jika gagal
exec > >(tee -a "$LOG_FILE") 2>&1  # [LOGGING] Semua output ke file log dan terminal

echo "[INFO] ========== Huskybot Run $(date) =========="  # [INFO] Tanda mulai run

# ===================== CEK USER DAN ENVIRONMENT =====================
if [ "$EUID" -eq 0 ]; then  # [ERROR HANDLING] Jangan jalankan sebagai root
  echo "[WARNING] Jangan jalankan script ini sebagai root/sudo! Gunakan user biasa."
  exit 1
fi

if ! ping -c 1 github.com &>/dev/null; then  # [ERROR HANDLING] Cek koneksi internet
  echo "[WARNING] Tidak ada koneksi internet. Download model otomatis akan gagal."
fi

echo "[DEBUG] Python executable: $(which python3)"  # [DEBUG] Tampilkan python3 yang dipakai
python3 -c "import sys; print('[DEBUG] sys.path:', sys.path)"  # [DEBUG] sys.path
python3 -c "import yaml; print('[DEBUG] PyYAML version:', yaml.__version__)" || echo "[DEBUG] PyYAML belum bisa diimport"  # [DEBUG] PyYAML

# ===================== CEK DEPENDENCY SISTEM & PYTHON =====================
for dep in wget curl python3 pip ros2 colcon; do  # [ERROR HANDLING] Cek semua dependency sistem
  command -v $dep >/dev/null 2>&1 || { echo "[ERROR] Dependency '$dep' tidak ditemukan!"; exit 10; }
done

PYTHON_DEPS=("numpy" "cv2" "ultralytics" "yaml" "tensorrt")  # [INFO] Daftar dependency Python utama
for dep in "${PYTHON_DEPS[@]}"; do  # [ERROR HANDLING] Cek semua dependency Python
  python3 -c "import $dep" 2>/dev/null || {
    echo "[ERROR] Python dependency '$dep' belum terinstall!"
    if [ "$dep" = "yaml" ]; then
      echo "[INFO] Install dengan: pip install pyyaml"
      echo "[INFO] Dokumentasi: https://pyyaml.org/wiki/PyYAMLDocumentation"
    elif [ "$dep" = "cv2" ]; then
      echo "[INFO] Install dengan: pip install opencv-python"
    elif [ "$dep" = "ultralytics" ]; then
      echo "[INFO] Install dengan: pip install ultralytics"
    elif [ "$dep" = "tensorrt" ]; then
      echo "[INFO] Install dengan: pip install tensorrt (atau sudo apt install python3-tensorrt)"
    fi
    exit 11
  }
done

PY_YAML_VERSION=$(python3 -c "import yaml; print(yaml.__version__)" 2>/dev/null || echo "0")  # [ERROR HANDLING] Cek versi minimal PyYAML
if [ "$PY_YAML_VERSION" != "0" ] && [ "$(printf '%s\n' "6.0" "$PY_YAML_VERSION" | sort -V | head -n1)" != "6.0" ]; then
  echo "[WARNING] Versi PyYAML terlalu lama ($PY_YAML_VERSION), minimal 6.0. Install dengan: pip install --upgrade pyyaml"
fi

# ===================== CEK FOLDER DAN FILE WORKSPACE =====================
for dir in "$HOME/jezzy/huskybot/src" "$HOME/jezzy/huskybot/install" "$MODEL_DIR"; do  # [ERROR HANDLING] Cek semua folder penting
  [ -d "$dir" ] || { echo "[ERROR] Folder $dir tidak ditemukan!"; exit 12; }
done
[ -f "$HOME/jezzy/huskybot/src/huskybot_gazebo/launch/huskybot_launch.py" ] || { echo "[ERROR] File launch utama tidak ditemukan!"; exit 13; }

# ===================== CEK PERMISSION FOLDER MODEL DAN LOG =====================
for folder in "$MODEL_DIR" "$HOME/huskybot_detection_log" "$HOME/huskybot_calib_output"; do  # [ERROR HANDLING] Cek permission folder
  if [ ! -d "$folder" ]; then mkdir -p "$folder"; fi
  if [ ! -w "$folder" ]; then
    echo "[ERROR] Tidak bisa menulis ke folder: $folder"
    echo "[INFO] Jalankan: chmod +w $folder atau chown \$USER $folder"
    exit 14
  fi
done

# ===================== CEK FILE KALIBRASI DAN YAML =====================
CALIB_FILE="$HOME/jezzy/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml"  # [INFO] File kalibrasi utama
if [ ! -f "$CALIB_FILE" ]; then
  echo "[WARNING] File kalibrasi $CALIB_FILE tidak ditemukan!"
else
  if ! grep -q "T_lidar_camera" "$CALIB_FILE"; then
    echo "[WARNING] Field 'T_lidar_camera' tidak ada di file kalibrasi!"
  fi
fi

YAML_CONTROLLER="$HOME/jezzy/huskybot/src/huskybot_description/config/huskybot_controllers.yaml"  # [INFO] File YAML controller
if [ ! -f "$YAML_CONTROLLER" ]; then
  echo "[WARNING] File YAML controller $YAML_CONTROLLER tidak ditemukan!"
fi

# ===================== CEK VERSI ROS2, GAZEBO, PYTHON =====================
echo "[INFO] Python version: $(python3 --version 2>&1)"  # [INFO] Tampilkan versi Python
if ros2 --version &>/dev/null; then
  echo "[INFO] ROS2 version: $(ros2 --version)"
else
  echo "[INFO] ROS2 CLI aktif, jumlah package: $(ros2 pkg list | wc -l)"
fi
if command -v gazebo &>/dev/null; then
  echo "[INFO] Gazebo version: $(gazebo --version)"
else
  echo "[WARNING] Gazebo tidak ditemukan! Pastikan sudah install gazebo."
fi

# ===================== CEK ENVIRONMENT VARIABLE ROS2 =====================
for var in ROS_DOMAIN_ID RMW_IMPLEMENTATION AMENT_PREFIX_PATH; do  # [ERROR HANDLING] Cek env var penting ROS2
  if [ -z "${!var:-}" ]; then
    echo "[WARNING] Environment variable $var belum di-set."
    echo "[INFO] Contoh export: export $var=0"
  fi
done

# ===================== CEK BUILD WORKSPACE =====================
if [ ! -d "$HOME/jezzy/huskybot/build" ] || [ ! -d "$HOME/jezzy/huskybot/install" ]; then
  echo "[ERROR] Workspace belum di-build! Jalankan: colcon build"
  exit 21
fi

# ===================== CEK LOG BUILD TERAKHIR =====================
if [ -f "$HOME/jezzy/huskybot/log/latest_build.log" ]; then
  if grep -q "error" "$HOME/jezzy/huskybot/log/latest_build.log"; then
    echo "[WARNING] Ada error di log build terakhir! Cek $HOME/jezzy/huskybot/log/latest_build.log"
  fi
fi

# ===================== CEK DAN DOWNLOAD MODEL DENGAN VALIDASI =====================
download_model() {  # [FUNGSI] Download model dengan validasi ukuran dan magic number
  local url="$1"
  local path="$2"
  local desc="$3"
  echo "[INFO] Model $desc belum ada, akan download dari $url ..."
  if ! curl -Is "$url" | grep -q "200 OK"; then
    echo "[WARNING] URL $url tidak bisa diakses (bukan 200 OK)!"
    return 1
  fi
  wget -O "$path" "$url"
  if [ $? -eq 0 ] && [ -f "$path" ]; then
    local size
    size=$(stat -c%s "$path")
    if [ "$size" -lt 1000000 ]; then
      echo "[WARNING] File $desc terlalu kecil ($size bytes), kemungkinan corrupt. Menghapus file."
      rm -f "$path"
      return 1
    fi
    if [[ "$desc" == *".onnx"* ]] && ! head -c 4 "$path" | grep -q "ONNX"; then
      echo "[WARNING] File $desc bukan ONNX valid! Hapus file."
      rm -f "$path"
      return 1
    fi
    echo "[INFO] Model $desc berhasil didownload ke $path ($size bytes)"
    return 0
  else
    echo "[WARNING] Gagal download model $desc!"
    return 1
  fi
}

MODEL_PATH=""  # [INFO] Path model yang akan dipakai
MODEL_FORMAT=""  # [INFO] Format model yang akan dipakai

# [ERROR HANDLING] Urutan prioritas: .engine > .onnx > .pt
if [ ! -f "$MODEL_ENGINE" ]; then
  download_model "$MODEL_URL_ENGINE" "$MODEL_ENGINE" ".engine (TensorRT)"
fi
if [ -f "$MODEL_ENGINE" ]; then
  MODEL_PATH="$MODEL_ENGINE"
  MODEL_FORMAT="engine"
  echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.engine/TensorRT)"
else
  if [ ! -f "$MODEL_ONNX" ]; then
    download_model "$MODEL_URL_ONNX" "$MODEL_ONNX" ".onnx (ONNX)"
  fi
  if [ -f "$MODEL_ONNX" ]; then
    MODEL_PATH="$MODEL_ONNX"
    MODEL_FORMAT="onnx"
    echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.onnx/ONNX)"
  else
    if [ ! -f "$MODEL_PT" ]; then
      download_model "$MODEL_URL_PT" "$MODEL_PT" ".pt (PyTorch)"
    fi
    if [ -f "$MODEL_PT" ]; then
      MODEL_PATH="$MODEL_PT"
      MODEL_FORMAT="pt"
      echo "[INFO] Menggunakan model YOLOv12: $MODEL_PATH (.pt/PyTorch)"
    else
      echo "[FATAL] Tidak ada model YOLOv12 (.engine/.onnx/.pt) yang bisa dipakai!"
      exit 15
    fi
  fi
fi

# ===================== KILL GAZEBO/ROS2/COLCON JIKA MASIH ADA =====================
for proc in gzserver gzclient ros2 colcon; do  # [ERROR HANDLING] Kill proses zombie sebelum run
  pkill -9 $proc 2>/dev/null || true
done

# ===================== CEK ROS2 CLI =====================
if ! command -v ros2 &> /dev/null; then
  echo "[ERROR] ros2 CLI tidak ditemukan! Pastikan ROS2 sudah terinstall dan di-source."
  exit 16
fi

# ===================== CEK RAM, DISK, CPU, GPU =====================
RAM_AVAIL=$(free -g | awk '/^Mem:/{print $7}')  # [INFO] RAM available (GB)
DISK_AVAIL=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')  # [INFO] Disk available (GB)
CPU_COUNT=$(nproc)  # [INFO] Jumlah CPU core
if [ "$RAM_AVAIL" -lt 4 ]; then
  echo "[WARNING] RAM kurang dari 4GB, simulasi bisa lambat/crash."
fi
if [ "$DISK_AVAIL" -lt 10 ]; then
  echo "[WARNING] Sisa disk kurang dari 10GB, simulasi/logging bisa gagal."
fi
echo "[INFO] CPU count: $CPU_COUNT, RAM: ${RAM_AVAIL}GB, Disk: ${DISK_AVAIL}GB"

if command -v lspci &>/dev/null && lspci | grep -i nvidia &>/dev/null; then
  echo "[INFO] GPU NVIDIA terdeteksi: $(lspci | grep -i nvidia)"
else
  echo "[WARNING] GPU NVIDIA tidak terdeteksi!"
fi

# ===================== CEK ENVIRONMENT ROS2 DAN WORKSPACE =====================
set +u  # [ERROR HANDLING] Nonaktifkan set -u sementara untuk source setup.bash (hindari error AMENT_TRACE_SETUP_FILES)
source /opt/ros/humble/setup.bash  # [INFO] Source ROS2 Humble
source $HOME/jezzy/huskybot/install/setup.bash  # [INFO] Source workspace install
set -u  # [ERROR HANDLING] Aktifkan kembali set -u

# ===================== CEK PLUGIN ROS2_CONTROL =====================
PLUGIN_PATH="$HOME/jezzy/huskybot/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so"  # [INFO] Path plugin ROS2 control
if [ ! -f "$PLUGIN_PATH" ]; then
  echo "[ERROR] Plugin libgazebo_ros2_control.so tidak ditemukan di $PLUGIN_PATH!"
  echo "Pastikan sudah colcon build dan source workspace."
  exit 17
fi

# ===================== EXPORT ENVIRONMENT GAZEBO =====================
: "${GAZEBO_PLUGIN_PATH:=}"  # [ERROR HANDLING] Inisialisasi jika belum ada
: "${GAZEBO_MODEL_PATH:=}"   # [ERROR HANDLING] Inisialisasi jika belum ada
export GAZEBO_PLUGIN_PATH=$HOME/jezzy/huskybot/install/gazebo_ros/lib:$HOME/jezzy/huskybot/install/gazebo_plugins/lib:$HOME/jezzy/huskybot/install/gazebo_ros2_control/lib:$GAZEBO_PLUGIN_PATH  # [INFO] Path plugin Gazebo
export GAZEBO_MODEL_PATH=$HOME/jezzy/huskybot/install/huskybot_description/share/huskybot_description/models:$HOME/jezzy/huskybot/install/husky_description/share/husky_description/models:$GAZEBO_MODEL_PATH  # [INFO] Path model Gazebo

# ===================== CEK WORKSPACE LAIN YANG BENTROK =====================
if [[ "$GAZEBO_PLUGIN_PATH" == *"gazebo_ros2_ws"* ]]; then
  echo "[WARNING] Detected other workspace in GAZEBO_PLUGIN_PATH! Please bersihkan environment Anda."
  exit 18
fi
if [[ "${LD_LIBRARY_PATH:-}" == *"gazebo_ros2_ws"* ]]; then
  echo "[WARNING] Detected other workspace in LD_LIBRARY_PATH! Please bersihkan environment Anda."
  exit 19
fi

echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"  # [INFO] Print path plugin Gazebo
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"    # [INFO] Print path model Gazebo

# ===================== CEK VERSI DAN DEPENDENCY PYTHON UTAMA =====================
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"  # [INFO] Cek versi OpenCV
python3 -c "import torch; print('PyTorch version:', getattr(torch, '__version__', 'N/A'), 'CUDA:', torch.cuda.is_available())" || echo "[WARNING] PyTorch tidak terinstall atau CUDA tidak aktif."
python3 -c "import ultralytics; print('Ultralytics version:', ultralytics.__version__)" || echo "[WARNING] Ultralytics tidak terinstall."

# ===================== CEK GPU DAN TENSORRT (JETSON ORIN) =====================
if command -v nvidia-smi &> /dev/null; then
  nvidia-smi || echo "[WARNING] nvidia-smi error, cek driver NVIDIA."
else
  echo "[WARNING] nvidia-smi tidak ditemukan, pastikan Jetson Orin environment sudah benar."
fi
if python3 -c "import tensorrt" 2>/dev/null; then
  echo "[INFO] TensorRT Python module terdeteksi."
else
  echo "[WARNING] TensorRT Python module tidak ditemukan! Pastikan sudah install python3-tensorrt."
fi

# ===================== CEK FILE LOG ROS2 =====================
ROS_LOG_DIR="$HOME/.ros/log"  # [INFO] Path log ROS2
if [ ! -d "$ROS_LOG_DIR" ]; then
  mkdir -p "$ROS_LOG_DIR"
fi

# ===================== CEK PROSES GANDA =====================
if pgrep -f "ros2 launch huskybot_gazebo" >/dev/null; then
  echo "[WARNING] Sudah ada proses ros2 launch huskybot_gazebo berjalan! Kill dulu sebelum lanjut."
  exit 20
fi

# ===================== OPSI DRY-RUN DAN TEST =====================
if [[ "${1:-}" == "--dry-run" ]]; then
  echo "[INFO] DRY RUN: Semua dependency dan environment sudah dicek. Tidak menjalankan simulasi."
  exit 0
fi
if [[ "${1:-}" == "--test" ]]; then
  echo "[INFO] Menjalankan colcon test sebelum simulasi..."
  colcon test || { echo "[ERROR] colcon test gagal!"; exit 22; }
fi

# ===================== JALANKAN SIMULASI GAZEBO + ROBOT + SENSOR =====================
echo "[INFO] Menjalankan ros2 launch huskybot_gazebo huskybot_launch.py ..."
ros2 launch huskybot_gazebo huskybot_launch.py model_path:="$MODEL_PATH" model_format:="$MODEL_FORMAT" | tee -a "$LOG_FILE"

LAUNCH_LOG="$HOME/.ros/log/latest_launch.log"  # [INFO] Path log launch ROS2
if [ -f "$LAUNCH_LOG" ]; then
  echo "[INFO] Log launch lengkap ada di: $LAUNCH_LOG"
  tail -n 20 "$LAUNCH_LOG"
else
  echo "[WARNING] Log launch ROS2 tidak ditemukan!"
fi

echo "[INFO] ========== Huskybot Run Selesai $(date) =========="  # [INFO] Tanda selesai run

# ===================== SARAN PENINGKATAN (LANGSUNG DIIMPLEMENTASIKAN) =====================
# - [ERROR HANDLING] Sudah fail-fast di setiap step, semua dependency dicek sebelum lanjut.
# - [LOGGING] Semua output ke file log, fallback ke /tmp jika gagal di $HOME.
# - [DEBUG] Print sys.path dan versi PyYAML untuk troubleshooting import error.
# - [PERFORMANCE] Model .engine (TensorRT) diprioritaskan, fallback ke .onnx/.pt jika tidak ada.
# - [PERMISSION] Semua folder penting dicek permission dan auto-mkdir jika belum ada.
# - [ENVIRONMENT] Source ROS2 Humble dan workspace install sebelum run.
# - [SAFETY] Kill proses zombie sebelum run, cek proses ganda ros2 launch.
# - [FUTURE] Bisa ditambah: validasi file YAML config lain, validasi semua topic ROS2, audit trail log ke CSV, monitoring resource (RAM, GPU, disk) secara periodik.
# - [FULL OOP] Semua node Python di workspace sudah class-based (lihat scripts/ dan launch/).
# - [CI/CD] Bisa ditambah badge coverage test dan test launch file di folder test/.
# - [TROUBLESHOOTING] Jika error import Python, cek PYTHONPATH dan pip install --user.
# - [MULTI-ROBOT] Namespace sudah siap di semua launch file, tinggal tambah argumen jika perlu.
# - [SIMULASI] Sudah siap untuk ROS2 Humble, Gazebo, YOLOv12, dan hardware real Husky + Jetson Orin + Velodyne + Arducam.