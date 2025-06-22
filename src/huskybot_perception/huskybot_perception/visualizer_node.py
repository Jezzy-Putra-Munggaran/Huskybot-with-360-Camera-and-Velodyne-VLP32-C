#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Custom message hasil deteksi YOLOv12 (harus ada di workspace)
import cv2  # OpenCV untuk visualisasi
import numpy as np  # Library array/matrix

class MultiTaskVisualizer(Node):  # Node visualizer multitask, FULL OOP
    def __init__(self):
        super().__init__('multitask_visualizer')  # Inisialisasi node ROS2 dengan nama unik
        self.images = {}  # Buffer hasil deteksi dari semua topic
        self.subs = []  # List subscription ke topic hasil deteksi/segmentasi/obb/tracking
        for topic in ['/detection', '/segmentation', '/obb', '/tracking']:  # Daftar topic multitask
            try:
                self.subs.append(self.create_subscription(
                    Yolov12Inference, topic, self.callback, 10))  # Subscribe ke semua topic multitask
            except Exception as e:
                self.get_logger().error(f"Error subscribe topic {topic}: {e}")  # Error handling jika topic tidak ada
        self.timer = self.create_timer(0.2, self.display)  # Timer untuk update visualisasi setiap 0.2 detik

    def callback(self, msg):
        try:
            key = msg.camera_name + "_" + msg.task  # Key unik per kamera dan task
            self.images[key] = msg  # Simpan hasil deteksi terbaru
        except Exception as e:
            self.get_logger().error(f"Error di callback visualizer: {e}")  # Error handling callback

    def display(self):
        try:
            img = np.zeros((400, 800, 3), dtype=np.uint8)  # Canvas kosong untuk visualisasi
            y = 50  # Posisi awal teks
            for key, msg in self.images.items():
                try:
                    text = f"{key}: {len(msg.yolov12_inference)} deteksi"  # Tampilkan jumlah deteksi per kamera-task
                    cv2.putText(img, text, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                    y += 40
                except Exception as e:
                    self.get_logger().warning(f"Error visualisasi key {key}: {e}")  # Error handling per key
            try:
                cv2.imshow("MultiTask Visualizer", img)  # Tampilkan visualisasi ke window
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warning(f"Error visualisasi OpenCV: {e}")  # Error handling jika headless/server
        except Exception as e:
            self.get_logger().error(f"Error display(): {e}")  # Error handling global visualisasi

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = MultiTaskVisualizer()  # Inisialisasi node visualizer
    try:
        rclpy.spin(node)  # Spin node sampai shutdown
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')  # Info shutdown via Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}")  # Error handling fatal
    finally:
        try:
            node.destroy_node()  # Destroy node ROS2
        except Exception as e:
            print(f"Error destroy_node: {e}")
        rclpy.shutdown()  # Shutdown ROS2