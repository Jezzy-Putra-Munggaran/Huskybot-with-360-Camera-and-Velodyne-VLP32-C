#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)
import csv  # Untuk menulis file CSV
import os  # Untuk operasi file dan path
import traceback  # Untuk logging error detail

def validate_yolov12_inference(msg):  # Fungsi validasi message sebelum proses
    if not isinstance(msg, Yolov12Inference):  # Pastikan tipe message benar
        return False
    if not hasattr(msg, 'header') or not hasattr(msg, 'camera_name') or not hasattr(msg, 'yolov12_inference'):  # Pastikan field utama ada
        return False
    return True

class Yolov12DetectionLogger(Node):  # Definisi class node logger deteksi YOLOv12 (OOP, reusable)
    def __init__(self):  # Konstruktor class
        super().__init__('yolov12_detection_logger')  # Inisialisasi node dengan nama 'yolov12_detection_logger'
        self.declare_parameter('inference_topic', '/panorama/yolov12_inference')  # Parameterisasi topic, default panorama
        self.declare_parameter('log_dir', os.path.expanduser('~/huskybot_detection_log'))  # Parameterisasi folder log
        self.declare_parameter('log_file', 'detections.csv')  # Nama file log
        topic = self.get_parameter('inference_topic').get_parameter_value().string_value  # Ambil topic dari parameter
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value  # Ambil folder log dari parameter
        log_file = self.get_parameter('log_file').get_parameter_value().string_value  # Ambil nama file log dari parameter

        os.makedirs(log_dir, exist_ok=True)  # Membuat folder jika belum ada
        self.csvfile = open(os.path.join(log_dir, log_file), 'a', newline='')  # Membuka file CSV untuk append
        self.writer = csv.writer(self.csvfile)  # Membuat writer CSV
        if os.stat(os.path.join(log_dir, log_file)).st_size == 0:  # Jika file baru, tulis header
            self.writer.writerow(['stamp', 'camera', 'class', 'confidence', 'top', 'left', 'bottom', 'right'])  # Header kolom CSV

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
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Mengambil timestamp deteksi (detik float)
            camera = getattr(msg, 'camera_name', 'unknown')  # Mengambil nama kamera (atau 'unknown' jika tidak ada)
            for det in msg.yolov12_inference:  # Loop semua hasil deteksi pada pesan
                self.writer.writerow([  # Tulis satu baris ke CSV untuk setiap deteksi
                    stamp,  # Timestamp deteksi
                    camera,  # Nama kamera/panorama
                    det.class_name,  # Nama kelas objek terdeteksi (harus ada di InferenceResult.msg)
                    det.confidence,  # Tambahkan confidence ke log
                    det.top,  # Koordinat bounding box atas
                    det.left,  # Koordinat bounding box kiri
                    det.bottom,  # Koordinat bounding box bawah
                    det.right  # Koordinat bounding box kanan
                ])
            self.csvfile.flush()  # Pastikan data langsung ditulis ke file
            self.get_logger().info(f"Logged {len(msg.yolov12_inference)} detections from {camera} at {stamp:.3f}")  # Logging info ke terminal
        except Exception as e:
            self.get_logger().error(f"Error logging detection: {e}\n{traceback.format_exc()}")

    def destroy_node(self):  # Override fungsi destroy_node untuk cleanup
        try:
            self.csvfile.close()  # Tutup file CSV saat node dimatikan
        except Exception as e:
            self.get_logger().warn(f"Error closing CSV file: {e}")
        super().destroy_node()  # Panggil destroy_node parent

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = Yolov12DetectionLogger()  # Buat instance node logger
    try:
        rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Exception utama: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()  # Cleanup saat node selesai
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main

# --- Penjelasan & Review Baris per Baris ---
# - Struktur folder sudah benar: scripts/ untuk node, launch/ untuk launch file.
# - Node ini subscribe ke topic hasil deteksi YOLOv12 (default: /panorama/yolov12_inference, bisa diubah via parameter/launch).
# - Sudah terhubung dengan node publisher YOLOv12 panorama di workspace.
# - FULL OOP: semua logic dalam class Node.
# - Error handling: validasi message, try/except parsing, logging error, flush file, close file.
# - Parameterisasi: topic dan path log bisa diubah dari launch file/CLI.
# - Logging info ke terminal setiap kali log deteksi.
# - Saran peningkatan:
#   1. Tambahkan filter class/threshold via parameter (untuk audit spesifik class).
#   2. Tambahkan opsi logging ke format lain (JSON, Parquet) jika dataset besar.
#   3. Tambahkan unit test untuk callback (pytest).
#   4. Untuk multi-robot, gunakan namespace ROS2.
#   5. Tambahkan opsi rotate file log jika terlalu besar.
# - File ini sudah aman, tidak ada bug fatal, siap untuk ROS2 Humble/Gazebo.