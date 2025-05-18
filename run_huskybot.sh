#!/bin/bash

# (1) Download model YOLO jika belum ada
if [ ! -f /mnt/nova_ssd/huskybot/src/huskybot_recognition/scripts/yolo12n.pt ]; then
    python3 -c "from ultralytics import YOLO; YOLO('yolo12n.pt')"
    mv ~/yolo12n.pt /mnt/nova_ssd/huskybot/src/huskybot_recognition/scripts/
fi

# (2) Matikan proses gazebo jika masih berjalan
pkill -9 gzserver
pkill -9 gzclient

# (3) Source ROS dan workspace
source /opt/ros/humble/setup.bash
source ~/gazebo_ros2_ws/install/setup.bash
source /mnt/nova_ssd/huskybot/install/setup.bash

# (4) Set environment variable untuk plugin dan model Gazebo
export GAZEBO_PLUGIN_PATH=/mnt/nova_ssd/huskybot/install/gazebo_ros/lib:/mnt/nova_ssd/huskybot/install/gazebo_plugins/lib:/mnt/nova_ssd/huskybot/install/gazebo_ros2_control/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=/mnt/nova_ssd/huskybot/install/huskybot_description/share/huskybot_description:/mnt/nova_ssd/huskybot/install/husky_description/share/husky_description:$GAZEBO_MODEL_PATH

# (5) Tampilkan path untuk verifikasi
echo "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

# (6) Jalankan launch file (tanpa -v debug, karena tidak didukung di ROS2 Humble)
ros2 launch huskybot_gazebo huskybot_launch.py

# (7) Setelah selesai, tampilkan lokasi file log launch
echo "Log launch lengkap ada di: ~/.ros/log/latest_launch.log"