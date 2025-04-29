#!/usr/bin/env python3

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)

class Yolov12InferenceListener(Node):  # Definisi class node listener deteksi YOLOv12
    def __init__(self):  # Konstruktor class
        super().__init__('yolov12_inference_listener')  # Inisialisasi node dengan nama 'yolov12_inference_listener'
        self.subscription = self.create_subscription(  # Membuat subscriber ROS2
            Yolov12Inference,  # Tipe message yang disubscribe (hasil deteksi YOLOv12)
            '/Yolov12_Inference',  # Nama topic yang disubscribe (harus sama dengan publisher deteksi YOLOv12 di workspace)
            self.listener_callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )

    def listener_callback(self, msg):  # Fungsi callback saat pesan deteksi diterima
        self.get_logger().info(  # Log info ke terminal
            f'Kamera: {msg.camera_name}, Jumlah Deteksi: {len(msg.yolov12_inference)}'  # Tampilkan nama kamera dan jumlah deteksi
        )
        for det in msg.yolov12_inference:  # Loop semua hasil deteksi pada pesan
            self.get_logger().info(  # Log info setiap deteksi
                f'  Class: {det.class_name}, Confidence: {det.confidence:.2f}, Box: ({det.top}, {det.left}, {det.bottom}, {det.right})'  # Tampilkan info bounding box dan confidence
            )

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = Yolov12InferenceListener()  # Buat instance node listener
    rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    node.destroy_node()  # Cleanup saat node selesai
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main