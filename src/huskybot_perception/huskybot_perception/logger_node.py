#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Custom message hasil deteksi YOLOv12 (harus ada di workspace)
import csv  # Untuk logging ke file CSV
import os  # Untuk path file dan validasi permission

class MultiTaskLogger(Node):  # Node logger multitask, FULL OOP
    def __init__(self):
        super().__init__('multitask_logger')  # Inisialisasi node ROS2 dengan nama unik
        self.log_path = os.path.expanduser("~/huskybot_multitask_log.csv")  # Path file log CSV (bisa diubah ke parameter)
        try:
            # Cek permission folder sebelum open file
            log_dir = os.path.dirname(self.log_path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)  # Buat folder log jika belum ada
            self.file = open(self.log_path, 'a', newline='')  # Open file log (append mode)
        except Exception as e:
            self.get_logger().error(f"Error open log file: {e}")  # Error handling file log tidak bisa dibuka
            raise
        try:
            self.writer = csv.writer(self.file)  # Inisialisasi writer CSV
            if os.stat(self.log_path).st_size == 0:
                # Tulis header hanya jika file kosong
                self.writer.writerow(['timestamp', 'camera_name', 'task', 'class_name', 'confidence', 'top', 'left', 'bottom', 'right', 'track_id', 'obb_angle'])
        except Exception as e:
            self.get_logger().error(f"Error init CSV writer: {e}")  # Error handling CSV writer
            raise
        self.subs = []  # List subscription ke topic multitask
        for topic in ['/detection', '/segmentation', '/obb', '/tracking']:  # Daftar topic multitask
            try:
                self.subs.append(self.create_subscription(
                    Yolov12Inference, topic, self.callback, 10))  # Subscribe ke semua topic multitask
            except Exception as e:
                self.get_logger().error(f"Error subscribe topic {topic}: {e}")  # Error handling jika topic tidak ada

    def callback(self, msg):
        try:
            for det in msg.yolov12_inference:  # Loop semua hasil deteksi di message
                try:
                    self.writer.writerow([
                        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,  # Timestamp deteksi
                        msg.camera_name,  # Nama kamera
                        msg.task,  # Task (detection/segmentation/obb/tracking)
                        getattr(det, 'class_name', ''),  # Nama class deteksi
                        getattr(det, 'confidence', 0),  # Confidence score
                        getattr(det, 'top', 0),  # Koordinat bounding box
                        getattr(det, 'left', 0),
                        getattr(det, 'bottom', 0),
                        getattr(det, 'right', 0),
                        getattr(det, 'track_id', -1),  # Track ID (jika ada)
                        getattr(det, 'obb_angle', 0)  # OBB angle (jika ada)
                    ])
                except Exception as e:
                    self.get_logger().warning(f"Error write row CSV: {e}")  # Error handling per row
            self.file.flush()  # Flush file agar data langsung tersimpan
        except Exception as e:
            self.get_logger().error(f"Error di callback logger: {e}")  # Error handling callback global

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = MultiTaskLogger()  # Inisialisasi node logger
    try:
        rclpy.spin(node)  # Spin node sampai shutdown
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')  # Info shutdown via Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}")  # Error handling fatal
    finally:
        try:
            node.file.close()  # Tutup file log
        except Exception as e:
            print(f"Error close log file: {e}")
        try:
            node.destroy_node()  # Destroy node ROS2
        except Exception as e:
            print(f"Error destroy_node: {e}")
        rclpy.shutdown()  # Shutdown ROS2