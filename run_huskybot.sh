#!/bin/bash

MODEL_DST="/mnt/nova_ssd/huskybot/src/huskybot_recognition/scripts/yolo12n.pt"
MODEL_URL="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo12n.pt"

# ---------- Cek permission folder model ----------
if [ ! -w "$(dirname "$MODEL_DST")" ]; then
  echo "[ERROR] Tidak bisa menulis ke folder model: $(dirname "$MODEL_DST")"
  exit 4
fi

# ---------- Download model YOLO jika belum ada ----------
if [ ! -f "$MODEL_DST" ]; then
    echo "[INFO] Model yolo12n.pt belum ada, akan download langsung dari Ultralytics..."
    wget -O "$MODEL_DST" "$MODEL_URL"
    if [ $? -eq 0 ] && [ -f "$MODEL_DST" ]; then
        echo "[INFO] Model berhasil didownload ke $MODEL_DST"
    else
        echo "[ERROR] Gagal download model yolo12n.pt!"
        exit 2
    fi
else
    echo "[INFO] Model yolo12n.pt sudah ada di $MODEL_DST"
fi

# ---------- Kill Gazebo jika masih ada ----------
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null

# ---------- Cek ros2 CLI ----------
if ! command -v ros2 &> /dev/null; then
  echo "[ERROR] ros2 CLI tidak ditemukan! Pastikan ROS2 sudah terinstall dan di-source."
  exit 3
fi

# ---------- Source environment ----------
source /opt/ros/humble/setup.bash
source /mnt/nova_ssd/huskybot/install/setup.bash

# ---------- Cek plugin ros2_control ----------
PLUGIN_PATH="/mnt/nova_ssd/huskybot/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so"
if [ ! -f "$PLUGIN_PATH" ]; then
  echo "[ERROR] Plugin libgazebo_ros2_control.so tidak ditemukan di $PLUGIN_PATH!"
  echo "Pastikan sudah colcon build dan source workspace."
  exit 5
fi

# ---------- Export environment Gazebo ----------
export GAZEBO_PLUGIN_PATH=/mnt/nova_ssd/huskybot/install/gazebo_ros/lib:/mnt/nova_ssd/huskybot/install/gazebo_plugins/lib:/mnt/nova_ssd/huskybot/install/gazebo_ros2_control/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=/mnt/nova_ssd/huskybot/install/huskybot_description/share/huskybot_description:/mnt/nova_ssd/huskybot/install/husky_description/share/husky_description:$GAZEBO_MODEL_PATH

# ---------- Cek workspace lain yang bentrok ----------
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

# ---------- Jalankan simulasi ----------
ros2 launch huskybot_gazebo huskybot_launch.py

echo "Log launch lengkap ada di: ~/.ros/log/latest_launch.log"