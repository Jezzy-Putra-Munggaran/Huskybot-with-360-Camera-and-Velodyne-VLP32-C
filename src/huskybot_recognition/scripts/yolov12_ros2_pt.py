#!/usr/bin/env python3

from ultralytics import YOLO  # Import library YOLO (pastikan sudah diinstall di environment Python)
import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from cv_bridge import CvBridge  # Untuk konversi antara ROS Image dan OpenCV

from yolov12_msgs.msg import InferenceResult  # Import custom message hasil deteksi (harus sudah di-build di workspace)
from yolov12_msgs.msg import Yolov12Inference  # Import custom message untuk list hasil deteksi

bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar

class Camera_subscriber(Node):  # Definisi class node subscriber kamera
    def __init__(self):  # Konstruktor class
        super().__init__('camera_subscriber')  # Inisialisasi node dengan nama 'camera_subscriber'

        self.model = YOLO('~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt')  # Load model YOLOv12 (pastikan path dan file model benar)

        self.yolov12_pub = self.create_publisher(Yolov12Inference, "/Yolov12_Inference", 1)  # Publisher hasil deteksi ke topic utama
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)  # Publisher gambar hasil deteksi (annotated)

        # Daftar topic kamera dan label (disesuaikan dengan Xacro Husky 6 kamera)
        self.camera_topics = {
            'camera_front':        '/camera_front/image_raw',        # Kamera depan (menghadap 0° yaw, arah maju robot, frame: camera_front_link)
            'camera_front_left':   '/camera_front_left/image_raw',   # Kamera depan kiri (sekitar +60° CounterClockWise dari depan, frame: camera_front_left_link)
            'camera_left':         '/camera_left/image_raw',         # Kamera kiri (sekitar +120° CounterClockWise dari depan, frame: camera_left_link)
            'camera_rear':         '/camera_rear/image_raw',         # Kamera belakang (menghadap 180° yaw, arah mundur robot, frame: camera_rear_link)
            'camera_rear_right':   '/camera_rear_right/image_raw',   # Kamera belakang kanan (sekitar -120° ClockWise dari depan, frame: camera_rear_right_link)
            'camera_right':        '/camera_right/image_raw'         # Kamera kanan (sekitar -60° ClockWise dari depan, frame: camera_right_link)
        }

        self._my_subscriptions = []  # Simpan subscription agar tidak di-GC
        for cam_name, topic in self.camera_topics.items():  # Loop semua kamera
            sub = self.create_subscription(
                Image,  # Tipe message yang disubscribe (gambar kamera)
                topic,  # Nama topic kamera yang disubscribe
                lambda msg, cam=cam_name: self.camera_callback(msg, cam),  # Callback dengan binding nama kamera
                10  # Queue size
            )
            self._my_subscriptions.append(sub)  # Simpan subscription

    def camera_callback(self, data, cam_name):  # Callback saat gambar dari kamera diterima
        img = bridge.imgmsg_to_cv2(data, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        results = self.model(img)  # Jalankan YOLOv12 pada gambar

        yolov12_inference = Yolov12Inference()  # Buat pesan hasil deteksi
        yolov12_inference.header.frame_id = cam_name  # Set frame_id sesuai kamera
        yolov12_inference.header.stamp = self.get_clock().now().to_msg()  # Set timestamp sekarang
        yolov12_inference.camera_name = cam_name  # Set nama kamera (field harus ada di msg)

        for r in results:  # Loop hasil deteksi YOLO (biasanya satu, tapi bisa lebih)
            boxes = r.boxes
            for box in boxes:  # Loop setiap bounding box hasil deteksi
                inference_result = InferenceResult()  # Buat pesan hasil deteksi individual
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Ambil koordinat bounding box (x1, y1, x2, y2)
                c = box.cls  # Ambil index kelas deteksi
                inference_result.class_name = self.model.names[int(c)]  # Nama kelas objek terdeteksi
                inference_result.confidence = float(box.conf)  # Tambahkan baris ini untuk confidence
                inference_result.top = int(b[0])  # Koordinat atas bounding box
                inference_result.left = int(b[1])  # Koordinat kiri bounding box
                inference_result.bottom = int(b[2])  # Koordinat bawah bounding box
                inference_result.right = int(b[3])  # Koordinat kanan bounding box
                yolov12_inference.yolov12_inference.append(inference_result)  # Tambahkan ke list hasil deteksi

        annotated_frame = results[0].plot()  # Annotasi gambar dengan bounding box
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  # Konversi kembali ke ROS Image
        img_msg.header = data.header  # Ikut header kamera asli

        self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)
        self.yolov12_pub.publish(yolov12_inference)  # Publish hasil deteksi ke topic utama

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    camera_subscriber = Camera_subscriber()  # Buat instance node subscriber kamera
    rclpy.spin(camera_subscriber)  # Jalankan node hingga Ctrl+C
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main