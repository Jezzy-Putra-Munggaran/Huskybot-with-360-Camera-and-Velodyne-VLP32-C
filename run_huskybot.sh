#!/bin/bash
# Aktifkan virtual environment
source ~/venv-pytorch-cuda/bin/activate

# Download model YOLO jika belum ada
python3 -c "from ultralytics import YOLO; YOLO('yolo12n.pt')"

# Pindahkan model ke folder scripts
mv ~/yolo12n.pt ~/huskybot/src/huskybot_recognition/scripts/

# Matikan proses gazebo jika masih berjalan
pkill -9 gzserver
pkill -9 gzclient

# Source ROS dan workspace
source /opt/ros/humble/setup.bash
source ~/gazebo_ros2_ws/install/setup.bash
source ~/huskybot/install/setup.bash

# Set environment variable untuk plugin dan model Gazebo
export GAZEBO_PLUGIN_PATH=$HOME/huskybot/install/gazebo_ros/lib:$HOME/huskybot/install/gazebo_plugins/lib:$HOME/huskybot/install/gazebo_ros2_control/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$HOME/huskybot/install/huskybot_description/share/huskybot_description:$HOME/huskybot/install/husky_description/share/husky_description:$GAZEBO_MODEL_PATH

# Tampilkan path untuk verifikasi
echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

# Jalankan launch file (tanpa -v debug, karena tidak didukung di ROS2 Humble)
ros2 launch huskybot_gazebo huskybot_launch.py

# Setelah selesai, tampilkan lokasi file log launch
echo "Log launch lengkap ada di: ~/.ros/log/latest_launch.log"