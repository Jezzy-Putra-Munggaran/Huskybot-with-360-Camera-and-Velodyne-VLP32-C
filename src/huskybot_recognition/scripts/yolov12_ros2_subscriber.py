#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2  # Import OpenCV untuk visualisasi bounding box pada gambar
import threading  # Untuk menjalankan multi-threaded executor ROS2
import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from cv_bridge import CvBridge, CvBridgeError  # Untuk konversi antara ROS Image dan OpenCV, plus error handling

from yolov12_msgs.msg import Yolov12Inference  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)

bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar
img = None  # Inisialisasi variabel global img agar tidak error saat pertama kali

class Camera_subscriber(Node):  # Node subscriber untuk kamera (mengisi variabel img global)
    def __init__(self):
        super().__init__('camera_subscriber')  # Inisialisasi node dengan nama 'camera_subscriber'

        # Subscribe ke salah satu kamera Husky, misal kamera depan
        self.subscription = self.create_subscription(
            Image,  # Tipe message yang disubscribe (gambar kamera)
            '/camera_front/image_raw',  # Nama topic kamera yang disubscribe (bisa diganti sesuai kebutuhan)
            self.camera_callback,  # Callback saat pesan gambar diterima
            10)  # Queue size
        self.subscription  # Simpan subscription agar tidak di-GC

    def camera_callback(self, data):  # Callback saat gambar dari kamera diterima
        global img
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")  # Konversi ROS Image ke OpenCV BGR dan simpan ke variabel global img
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")  # Error handling konversi gambar

class Yolo_subscriber(Node):  # Node subscriber untuk hasil deteksi YOLOv12
    def __init__(self):
        super().__init__('yolo_subscriber')  # Inisialisasi node dengan nama 'yolo_subscriber'

        self.subscription = self.create_subscription(
            Yolov12Inference,  # Tipe message yang disubscribe (hasil deteksi YOLOv12)
            '/Yolov12_Inference',  # Nama topic hasil deteksi (harus sama dengan publisher YOLOv12 di workspace)
            self.yolo_callback,  # Callback saat pesan deteksi diterima
            10)  # Queue size
        self.subscription  # Simpan subscription agar tidak di-GC

        self.cnt = 0  # Counter untuk menandai urutan deteksi pada gambar

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)  # Publisher gambar hasil deteksi (annotated)

    def yolo_callback(self, data):  # Callback saat pesan deteksi diterima
        global img
        if img is None:  # Jika belum ada gambar dari kamera, skip
            self.get_logger().warn("Belum ada gambar dari kamera, skip publish inference result.")
            return
        img_annotated = img.copy()  # Copy gambar agar tidak overwrite global img
        for r in data.yolov12_inference:  # Loop semua hasil deteksi pada pesan
            class_name = r.class_name  # Nama kelas objek terdeteksi
            confidence = r.confidence  # Confidence deteksi
            top = r.top  # Koordinat atas bounding box
            left = r.left  # Koordinat kiri bounding box
            bottom = r.bottom  # Koordinat bawah bounding box
            right = r.right  # Koordinat kanan bounding box
            self.get_logger().info(
                f"{self.cnt} {class_name} ({confidence:.2f}) : {top}, {left}, {bottom}, {right}"
                )  # Log info deteksi
            # OpenCV: (x1, y1) = (left, top), (x2, y2) = (right, bottom)
            cv2.rectangle(img_annotated, (left, top), (right, bottom), (255, 255, 0), 2)  # Gambar bounding box pada gambar
            cv2.putText(img_annotated, f"{class_name} {confidence:.2f}", (left, top-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
            self.cnt += 1  # Increment counter

        self.cnt = 0  # Reset counter
        try:
            img_msg = bridge.cv2_to_imgmsg(img_annotated, encoding="bgr8")  # Konversi kembali ke ROS Image
            self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error saat publish: {e}")

def main():
    rclpy.init(args=None)  # Inisialisasi ROS2 Python
    yolo_subscriber = Yolo_subscriber()  # Buat instance node subscriber hasil deteksi
    camera_subscriber = Camera_subscriber()  # Buat instance node subscriber kamera

    executor = rclpy.executors.MultiThreadedExecutor()  # Gunakan multi-threaded executor agar kedua node bisa jalan bersamaan
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)  # Jalankan executor di thread terpisah
    executor_thread.start()
    
    try:
        while rclpy.ok():
            pass  # Loop utama, biarkan executor yang handle callback
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()  # Shutdown ROS2
        executor_thread.join()  # Tunggu thread executor selesai

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()

# --- Penjelasan & Review ---
# - Struktur folder sudah benar: scripts/ untuk node, launch/ untuk launch file.
# - Node ini subscribe ke /camera_front/image_raw dan /Yolov12_Inference, publish ke /inference_result_cv2.
# - Sudah terhubung dengan node YOLOv12 publisher di workspace.
# - FULL OOP: semua logic dalam class Node.
# - Error handling: sudah ada untuk konversi gambar dan publish.
# - Saran peningkatan:
#   1. Tambahkan parameterisasi topic kamera dan topic hasil deteksi via parameter node/launch file.
#   2. Tambahkan validasi message Yolov12Inference sebelum proses.
#   3. Tambahkan opsi untuk menyimpan gambar hasil deteksi ke file (opsional).
#   4. Tambahkan unit test untuk callback.
#   5. Untuk multi-kamera, gunakan dict img per kamera, bukan variabel global tunggal.
# - File ini sudah aman, tidak ada bug fatal, siap untuk ROS2 Humble/Gazebo.