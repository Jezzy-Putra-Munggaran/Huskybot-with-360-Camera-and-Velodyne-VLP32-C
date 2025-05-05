#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)

def validate_yolov12_inference(msg):  # Fungsi validasi message sebelum proses
    if not isinstance(msg, Yolov12Inference):  # Pastikan tipe message benar
        return False
    if not hasattr(msg, 'header') or not hasattr(msg, 'camera_name') or not hasattr(msg, 'yolov12_inference'):  # Pastikan field utama ada
        return False
    return True

class Yolov12InferenceListener(Node):  # Definisi class node listener deteksi YOLOv12 (OOP, reusable)
    def __init__(self):  # Konstruktor class
        super().__init__('yolov12_inference_listener')  # Inisialisasi node dengan nama 'yolov12_inference_listener'
        # Parameterisasi topic via parameter node/launch file
        self.declare_parameter('inference_topic', '/Yolov12_Inference')  # Default topic inference
        topic = self.get_parameter('inference_topic').get_parameter_value().string_value  # Ambil topic dari parameter
        self.subscription = self.create_subscription(  # Membuat subscriber ROS2
            Yolov12Inference,  # Tipe message yang disubscribe (hasil deteksi YOLOv12)
            topic,  # Nama topic yang disubscribe (bisa diubah via parameter)
            self.listener_callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )
        self.subscription  # Simpan subscription agar tidak di-GC

    def listener_callback(self, msg):  # Fungsi callback saat pesan deteksi diterima
        if not validate_yolov12_inference(msg):  # Validasi message sebelum proses
            self.get_logger().error("Yolov12Inference message tidak valid, skip.")
            return
        try:
            self.get_logger().info(  # Log info ke terminal
                f'Kamera: {msg.camera_name}, Jumlah Deteksi: {len(msg.yolov12_inference)}'  # Tampilkan nama kamera dan jumlah deteksi
            )
            for det in msg.yolov12_inference:  # Loop semua hasil deteksi pada pesan
                self.get_logger().info(  # Log info setiap deteksi
                    f'  Class: {det.class_name}, Confidence: {det.confidence:.2f}, Box: ({det.top}, {det.left}, {det.bottom}, {det.right})'  # Tampilkan info bounding box dan confidence
                )
        except Exception as e:
            self.get_logger().error(f"Error parsing Yolov12Inference: {e}")

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = Yolov12InferenceListener()  # Buat instance node listener
    try:
        rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Exception utama: {e}")
    finally:
        node.destroy_node()  # Cleanup saat node selesai
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main

# --- Penjelasan & Review Baris per Baris ---
# - Struktur folder sudah benar: scripts/ untuk node, launch/ untuk launch file.
# - Node ini subscribe ke topic hasil deteksi YOLOv12 (default: /Yolov12_Inference, bisa diubah via parameter).
# - Sudah terhubung dengan node publisher YOLOv12 di workspace (yolov12_ros2_pt.py, panorama_inference, dsb).
# - FULL OOP: semua logic dalam class Node.
# - Error handling: validasi message, try/except parsing, logging error.
# - Parameterisasi: topic bisa diubah dari launch file/CLI.
# - Saran peningkatan:
#   1. Tambahkan opsi logging ke file (opsional, untuk audit trail).
#   2. Tambahkan unit test untuk callback (pytest).
#   3. Tambahkan filter class/threshold via parameter.
#   4. Untuk multi-robot, gunakan namespace ROS2.
# - File ini sudah aman, tidak ada bug fatal, siap untuk ROS2 Humble/Gazebo.