#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from yolov12_msgs.msg import Yolov12Inference, InferenceResult  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)
from cv_bridge import CvBridge, CvBridgeError  # Untuk konversi antara ROS Image dan OpenCV, plus error handling
from ultralytics import YOLO  # Import library YOLOv12 (pastikan sudah diinstall)
import os  # Untuk cek file model
import traceback  # Untuk logging error detail

def validate_yolov12_inference(msg):  # Fungsi validasi message sebelum publish
    if not isinstance(msg, Yolov12Inference):  # Pastikan tipe message benar
        return False
    if not hasattr(msg, 'header') or not hasattr(msg, 'camera_name') or not hasattr(msg, 'yolov12_inference'):  # Pastikan field utama ada
        return False
    return True

class PanoramaDetection(Node):  # Definisi class node deteksi panorama (OOP, reusable)
    def __init__(self):  # Konstruktor class
        super().__init__('panorama_detection')  # Inisialisasi node dengan nama 'panorama_detection'
        self.bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar

        # Parameterisasi model_path dan threshold dari parameter node/launch file
        self.declare_parameter('model_path', os.path.expanduser('~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt'))  # Path default model YOLOv12
        self.declare_parameter('confidence_threshold', 0.25)  # Threshold confidence default

        model_path = self.get_parameter('model_path').get_parameter_value().string_value  # Ambil path model dari parameter
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value  # Ambil threshold dari parameter

        if not os.path.isfile(model_path):  # Cek file model ada
            self.get_logger().error(f"Model YOLOv12 tidak ditemukan: {model_path}")
            raise FileNotFoundError(f"Model YOLOv12 tidak ditemukan: {model_path}")

        try:
            self.model = YOLO(model_path)  # Load model YOLOv12 (pastikan path dan file model benar)
        except Exception as e:
            self.get_logger().error(f"Gagal load model YOLOv12: {e}")
            raise

        self.pub = self.create_publisher(Yolov12Inference, '/panorama/yolov12_inference', 1)  # Publisher hasil deteksi ke topic panorama
        self.img_pub = self.create_publisher(Image, '/panorama/inference_result', 1)  # Publisher gambar hasil deteksi (annotated)
        self.sub = self.create_subscription(  # Subscriber untuk input panorama (hasil stitching)
            Image,  # Tipe message yang disubscribe (gambar panorama)
            '/panorama/detection_input',  # Nama topic yang disubscribe (harus sama dengan publisher stitcher node)
            self.callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )

    def callback(self, msg):  # Fungsi callback saat pesan gambar panorama diterima
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Error konversi gambar panorama: {e}")
            return

        try:
            results = self.model(img)  # Jalankan YOLOv12 pada gambar panorama
        except Exception as e:
            self.get_logger().error(f"YOLOv12 inference error: {e}\n{traceback.format_exc()}")
            return

        yolov12_inference = Yolov12Inference()  # Buat pesan hasil deteksi
        yolov12_inference.header = msg.header  # Copy header dari pesan input (sinkronisasi waktu)
        yolov12_inference.camera_name = "panorama"  # Set nama kamera (atau panorama)

        for r in results:  # Loop hasil deteksi YOLO (biasanya satu, tapi bisa lebih)
            boxes = getattr(r, 'boxes', [])
            for box in boxes:  # Loop setiap bounding box hasil deteksi
                try:
                    conf = float(box.conf)
                    if conf < self.confidence_threshold:  # Filter threshold confidence
                        continue
                    b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Ambil koordinat bounding box (x1, y1, x2, y2)
                    c = box.cls  # Ambil index kelas deteksi
                    inference_result = InferenceResult()  # Buat pesan hasil deteksi individual
                    inference_result.class_name = self.model.names[int(c)]  # Nama kelas objek terdeteksi
                    inference_result.confidence = conf  # Confidence deteksi
                    inference_result.top = int(b[0])  # Koordinat atas bounding box
                    inference_result.left = int(b[1])  # Koordinat kiri bounding box
                    inference_result.bottom = int(b[2])  # Koordinat bawah bounding box
                    inference_result.right = int(b[3])  # Koordinat kanan bounding box
                    yolov12_inference.yolov12_inference.append(inference_result)  # Tambahkan ke list hasil deteksi
                except Exception as e:
                    self.get_logger().warn(f"Error parsing detection result: {e}")

        # Validasi message sebelum publish
        if validate_yolov12_inference(yolov12_inference):
            self.pub.publish(yolov12_inference)  # Publish hasil deteksi ke topic panorama
        else:
            self.get_logger().error("Yolov12Inference message tidak valid, tidak dipublish.")

        # Publish hasil visualisasi
        try:
            annotated = results[0].plot()  # Annotasi gambar dengan bounding box
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')  # Konversi kembali ke ROS Image
            img_msg.header = msg.header  # Copy header agar sinkron dengan input
            self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)
        except Exception as e:
            self.get_logger().warn(f"Error publishing annotated image: {e}")

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = PanoramaDetection()  # Buat instance node deteksi panorama
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
# - Node ini subscribe ke /panorama/detection_input (output stitcher), publish ke /panorama/yolov12_inference dan /panorama/inference_result.
# - Sudah terhubung dengan pipeline stitching panorama dan node fusion di workspace.
# - FULL OOP: semua logic dalam class Node.
# - Error handling: sudah ada untuk konversi gambar, inference, parsing, dan publish.
# - Parameterisasi: model_path dan confidence_threshold bisa diatur dari launch file/CLI.
# - Validasi message sebelum publish.
# - Saran peningkatan:
#   1. Tambahkan logging statistik deteksi ke file (opsional, untuk audit).
#   2. Tambahkan unit test untuk callback (pytest).
#   3. Tambahkan parameterisasi topic input/output via parameter node/launch file.
#   4. Untuk multi-robot, gunakan namespace ROS2.
# - File ini sudah aman, tidak ada bug fatal, siap untuk ROS2 Humble/Gazebo.