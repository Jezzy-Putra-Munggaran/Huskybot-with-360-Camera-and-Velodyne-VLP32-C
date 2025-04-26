#!/bin/bash
# filepath: ~/huskybot/rename_yolov8_to_yolov12.sh

set -e

cd ~/huskybot/src

echo "=== Rename folder dan file ==="
# Rename folder
if [ -d yolov8_msgs ]; then
    mv yolov8_msgs yolov12_msgs
fi

# Rename file dan folder di huskybot_recognition
find huskybot_recognition -type f -name '*yolov8*' | while read f; do
    newf=$(echo "$f" | sed 's/yolov8/yolov12/g')
    mv "$f" "$newf"
done

find huskybot_recognition -type d -name '*yolov8*' | while read d; do
    newd=$(echo "$d" | sed 's/yolov8/yolov12/g')
    mv "$d" "$newd"
done

# Rename file message
if [ -f yolov12_msgs/msg/Yolov8Inference.msg ]; then
    mv yolov12_msgs/msg/Yolov8Inference.msg yolov12_msgs/msg/Yolov12Inference.msg
fi

echo "=== Ganti isi file (yolov8 -> yolov12, case-insensitive) ==="
# Ganti semua isi file (yolov8 -> yolov12, Yolov8 -> Yolov12, YOLOv8 -> YOLOv12)
grep -rl --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log yolov8 . | xargs sed -i 's/yolov8/yolov12/g'
grep -rl --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log Yolov8 . | xargs sed -i 's/Yolov8/Yolov12/g'
grep -rl --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log YOLOv8 . | xargs sed -i 's/YOLOv8/YOLOv12/g'

# Ganti topic dan message name
grep -rl '/Yolov8_Inference' . | xargs sed -i 's/\/Yolov8_Inference/\/Yolov12_Inference/g'
grep -rl 'yolov8n.pt' . | xargs sed -i 's/yolov8n.pt/yolo12n.pt/g'

echo "=== Selesai. Silakan cek hasil rename, lalu rebuild workspace ==="
