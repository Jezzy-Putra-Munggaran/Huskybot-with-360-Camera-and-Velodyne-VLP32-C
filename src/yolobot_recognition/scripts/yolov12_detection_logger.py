#!/usr/bin/env python3  # Shebang agar bisa dieksekusi langsung di terminal Linux

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Import custom message hasil deteksi YOLOv12
import csv  # Untuk menulis file CSV
import os  # Untuk operasi file dan path

class Yolov12DetectionLogger(Node):  # Definisi class node logger deteksi YOLOv12
    def __init__(self):  # Konstruktor class
        super().__init__('yolov12_detection_logger')  # Inisialisasi node dengan nama 'yolov12_detection_logger'
        self.subscription = self.create_subscription(  # Membuat subscriber ROS2
            Yolov12Inference,  # Tipe message yang disubscribe (hasil deteksi YOLOv12)
            '/panorama/yolov12_inference',  # Nama topic yang disubscribe (ganti jika topic Anda berbeda)
            self.listener_callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )
        save_dir = os.path.expanduser('~/yolobot_detection_log')  # Path folder untuk menyimpan log (di home user)
        os.makedirs(save_dir, exist_ok=True)  # Membuat folder jika belum ada
        self.csvfile = open(os.path.join(save_dir, 'detections.csv'), 'w', newline='')  # Membuka file CSV untuk ditulis
        self.writer = csv.writer(self.csvfile)  # Membuat writer CSV
        self.writer.writerow(['stamp', 'camera', 'class', 'top', 'left', 'bottom', 'right'])  # Header kolom CSV

    def listener_callback(self, msg):  # Fungsi callback saat pesan deteksi diterima
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Mengambil timestamp deteksi (detik float)
        camera = getattr(msg, 'camera_name', 'unknown')  # Mengambil nama kamera (atau 'unknown' jika tidak ada)
        for det in msg.yolov12_inference:  # Loop semua hasil deteksi pada pesan
            self.writer.writerow([  # Tulis satu baris ke CSV untuk setiap deteksi
                stamp,  # Timestamp deteksi
                camera,  # Nama kamera/panorama
                det.class_name,  # Nama kelas objek terdeteksi
                det.top,  # Koordinat bounding box atas
                det.left,  # Koordinat bounding box kiri
                det.bottom,  # Koordinat bounding box bawah
                det.right  # Koordinat bounding box kanan
            ])
        self.csvfile.flush()  # Pastikan data langsung ditulis ke file

    def destroy_node(self):  # Override fungsi destroy_node untuk cleanup
        self.csvfile.close()  # Tutup file CSV saat node dimatikan
        super().destroy_node()  # Panggil destroy_node parent

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = Yolov12DetectionLogger()  # Buat instance node logger
    try:
        rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    finally:
        node.destroy_node()  # Cleanup saat node selesai
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main