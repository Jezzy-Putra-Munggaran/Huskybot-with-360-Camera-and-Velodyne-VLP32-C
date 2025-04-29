#!/usr/bin/env python3

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from yolov12_msgs.msg import Yolov12Inference, InferenceResult  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)
from cv_bridge import CvBridge  # Untuk konversi antara ROS Image dan OpenCV
from ultralytics import YOLO  # Import library YOLOv12 (pastikan sudah diinstall)

class PanoramaDetection(Node):  # Definisi class node deteksi panorama
    def __init__(self):  # Konstruktor class
        super().__init__('panorama_detection')  # Inisialisasi node dengan nama 'panorama_detection'
        self.bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar
        self.model = YOLO('~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt')  # Load model YOLOv12 (pastikan path dan file model benar)
        self.pub = self.create_publisher(Yolov12Inference, '/panorama/yolov12_inference', 1)  # Publisher hasil deteksi ke topic panorama
        self.img_pub = self.create_publisher(Image, '/panorama/inference_result', 1)  # Publisher gambar hasil deteksi (annotated)
        self.sub = self.create_subscription(  # Subscriber untuk input panorama (hasil stitching)
            Image,  # Tipe message yang disubscribe (gambar panorama)
            '/panorama/detection_input',  # Nama topic yang disubscribe (harus sama dengan publisher stitcher node)
            self.callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )

    def callback(self, msg):  # Fungsi callback saat pesan gambar panorama diterima
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        results = self.model(img)  # Jalankan YOLOv12 pada gambar panorama
        yolov12_inference = Yolov12Inference()  # Buat pesan hasil deteksi
        yolov12_inference.header = msg.header  # Copy header dari pesan input (sinkronisasi waktu)
        yolov12_inference.camera_name = "panorama"  # Set nama kamera (atau panorama)
        for r in results:  # Loop hasil deteksi YOLO (biasanya satu, tapi bisa lebih)
            for box in r.boxes:  # Loop setiap bounding box hasil deteksi
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Ambil koordinat bounding box (x1, y1, x2, y2)
                c = box.cls  # Ambil index kelas deteksi
                inference_result = InferenceResult()  # Buat pesan hasil deteksi individual
                inference_result.class_name = self.model.names[int(c)]  # Nama kelas objek terdeteksi
                inference_result.confidence = float(box.conf)  # Tambahkan baris ini untuk confidence
                inference_result.top = int(b[0])  # Koordinat atas bounding box
                inference_result.left = int(b[1])  # Koordinat kiri bounding box
                inference_result.bottom = int(b[2])  # Koordinat bawah bounding box
                inference_result.right = int(b[3])  # Koordinat kanan bounding box
                yolov12_inference.yolov12_inference.append(inference_result)  # Tambahkan ke list hasil deteksi
        self.pub.publish(yolov12_inference)  # Publish hasil deteksi ke topic panorama
        # Publish hasil visualisasi
        annotated = results[0].plot()  # Annotasi gambar dengan bounding box
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')  # Konversi kembali ke ROS Image
        img_msg.header = msg.header  # Copy header agar sinkron dengan input
        self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = PanoramaDetection()  # Buat instance node deteksi panorama
    rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    node.destroy_node()  # Cleanup saat node selesai
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main