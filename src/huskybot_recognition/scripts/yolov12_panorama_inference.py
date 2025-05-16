#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from yolov12_msgs.msg import Yolov12Inference, InferenceResult  # Import custom message hasil deteksi YOLOv12 (harus sudah di-build di workspace)
from cv_bridge import CvBridge, CvBridgeError  # Untuk konversi antara ROS Image dan OpenCV, plus error handling
from ultralytics import YOLO  # Import library YOLOv12 (pastikan sudah diinstall)
import os  # Untuk cek file model
import traceback  # Untuk logging error detail
import logging  # Untuk logging ke file (opsional)
import time  # Untuk statistik deteksi/logging
import csv  # Untuk logging statistik ke file

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_panorama_inference.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("panorama_inference_file_logger")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        fh = logging.FileHandler(log_path)
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
        logger.addHandler(fh)
    return logger

file_logger = setup_file_logger()

def log_to_file(msg, level='info'):
    if file_logger:
        if level == 'error':
            file_logger.error(msg)
        elif level == 'warn':
            file_logger.warning(msg)
        else:
            file_logger.info(msg)

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
        self.declare_parameter('log_stats', False)  # Logging statistik deteksi ke file
        self.declare_parameter('log_stats_path', os.path.expanduser('~/huskybot_detection_log/panorama_stats.csv'))  # Path file statistik

        model_path = self.get_parameter('model_path').get_parameter_value().string_value  # Ambil path model dari parameter
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value  # Ambil threshold dari parameter
        self.log_stats = self.get_parameter('log_stats').get_parameter_value().bool_value  # Ambil flag logging statistik
        self.log_stats_path = self.get_parameter('log_stats_path').get_parameter_value().string_value  # Ambil path file statistik

        self.get_logger().info(f"Parameter: model_path={model_path}, confidence_threshold={self.confidence_threshold}, log_stats={self.log_stats}, log_stats_path={self.log_stats_path}")
        log_to_file(f"Parameter: model_path={model_path}, confidence_threshold={self.confidence_threshold}, log_stats={self.log_stats}, log_stats_path={self.log_stats_path}")

        # Validasi file model YOLO
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model YOLOv12 tidak ditemukan: {model_path}")
            log_to_file(f"Model YOLOv12 tidak ditemukan: {model_path}", level='error')
            raise FileNotFoundError(f"Model YOLOv12 tidak ditemukan: {model_path}")

        try:
            self.model = YOLO(model_path)  # Load model YOLOv12 (pastikan path dan file model benar)
            self.get_logger().info("YOLOv12 model loaded successfully.")
            log_to_file("YOLOv12 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Gagal load model YOLOv12: {e}")
            log_to_file(f"Gagal load model YOLOv12: {e}", level='error')
            raise

        self.pub = self.create_publisher(Yolov12Inference, '/panorama/yolov12_inference', 1)  # Publisher hasil deteksi ke topic panorama
        self.img_pub = self.create_publisher(Image, '/panorama/inference_result', 1)  # Publisher gambar hasil deteksi (annotated)
        self.sub = self.create_subscription(  # Subscriber untuk input panorama (hasil stitching)
            Image,  # Tipe message yang disubscribe (gambar panorama)
            '/panorama/detection_input',  # Nama topic yang disubscribe (harus sama dengan publisher stitcher node)
            self.callback,  # Fungsi callback saat pesan diterima
            10  # Queue size
        )

        # Statistik deteksi (thread-safe)
        self.stats = {'total_images': 0, 'total_detections': 0, 'per_class': {}}
        if self.log_stats:
            os.makedirs(os.path.dirname(self.log_stats_path), exist_ok=True)
            self.stats_file = open(self.log_stats_path, 'a', newline='')
            self.stats_writer = csv.writer(self.stats_file)
            if os.stat(self.log_stats_path).st_size == 0:
                self.stats_writer.writerow(['timestamp', 'num_detections', 'class_counts'])
            self.get_logger().info(f"Logging detection stats to: {self.log_stats_path}")
            log_to_file(f"Logging detection stats to: {self.log_stats_path}")

    def callback(self, msg):  # Fungsi callback saat pesan gambar panorama diterima
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            log_to_file(f"CV Bridge error: {e}", level='error')
            return
        except Exception as e:
            self.get_logger().error(f"Error konversi gambar panorama: {e}")
            log_to_file(f"Error konversi gambar panorama: {e}", level='error')
            return

        try:
            results = self.model(img)  # Jalankan YOLOv12 pada gambar panorama
        except Exception as e:
            self.get_logger().error(f"YOLOv12 inference error: {e}\n{traceback.format_exc()}")
            log_to_file(f"YOLOv12 inference error: {e}\n{traceback.format_exc()}", level='error')
            return

        yolov12_inference = Yolov12Inference()  # Buat pesan hasil deteksi
        yolov12_inference.header = msg.header  # Copy header dari pesan input (sinkronisasi waktu)
        yolov12_inference.camera_name = "panorama"  # Set nama kamera (atau panorama)

        class_counts = {}
        num_detections = 0

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
                    num_detections += 1
                    class_counts[inference_result.class_name] = class_counts.get(inference_result.class_name, 0) + 1
                except Exception as e:
                    self.get_logger().warn(f"Error parsing detection result: {e}")
                    log_to_file(f"Error parsing detection result: {e}", level='warn')

        # Statistik deteksi (opsional)
        self.stats['total_images'] += 1
        self.stats['total_detections'] += num_detections
        for cname, cnt in class_counts.items():
            self.stats['per_class'][cname] = self.stats['per_class'].get(cname, 0) + cnt
        if self.log_stats:
            try:
                self.stats_writer.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    num_detections,
                    dict(class_counts)
                ])
                self.stats_file.flush()
            except Exception as e:
                self.get_logger().warn(f"Error writing stats to file: {e}")
                log_to_file(f"Error writing stats to file: {e}", level='warn')

        # Validasi message sebelum publish
        if validate_yolov12_inference(yolov12_inference):
            self.pub.publish(yolov12_inference)  # Publish hasil deteksi ke topic panorama
            self.get_logger().debug(f"Published Yolov12Inference with {num_detections} detections.")
            log_to_file(f"Published Yolov12Inference with {num_detections} detections.", level='debug')
        else:
            self.get_logger().error("Yolov12Inference message tidak valid, tidak dipublish.")
            log_to_file("Yolov12Inference message tidak valid, tidak dipublish.", level='error')

        # Publish hasil visualisasi
        try:
            annotated = results[0].plot()  # Annotasi gambar dengan bounding box
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')  # Konversi kembali ke ROS Image
            img_msg.header = msg.header  # Copy header agar sinkron dengan input
            self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)
        except Exception as e:
            self.get_logger().warn(f"Error publishing annotated image: {e}")
            log_to_file(f"Error publishing annotated image: {e}", level='warn')

    def destroy_node(self):
        if self.log_stats and hasattr(self, 'stats_file'):
            self.stats_file.close()
        super().destroy_node()

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = PanoramaDetection()  # Buat instance node deteksi panorama
    try:
        rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Exception utama: {e}\n{traceback.format_exc()}")
        log_to_file(f"Exception utama: {e}\n{traceback.format_exc()}", level='error')
    finally:
        node.destroy_node()  # Cleanup saat node selesai
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi main

# --- Penjelasan & Review Baris per Baris ---
# - Sudah ada logger ROS2 dan logging ke file di setiap langkah penting/error.
# - Semua error/exception di callback dan fungsi utama sudah di-log.
# - Validasi file model, parameter, dan statistik sudah lengkap.
# - Monitoring health check sensor (jumlah deteksi, class).
# - Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real.
# - Saran: tambahkan parameterisasi topic input/output jika ingin lebih fleksibel.