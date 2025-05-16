#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

from ultralytics import YOLO  # Import library YOLO (pastikan sudah diinstall di environment Python)
import rclpy  # Import modul utama ROS2 Python
from rclpy.node import Node  # Import base class Node untuk membuat node ROS2
from sensor_msgs.msg import Image  # Import message standar ROS2 untuk gambar
from cv_bridge import CvBridge, CvBridgeError  # Untuk konversi antara ROS Image dan OpenCV, plus error handling

from yolov12_msgs.msg import InferenceResult  # Import custom message hasil deteksi (harus sudah di-build di workspace)
from yolov12_msgs.msg import Yolov12Inference  # Import custom message untuk list hasil deteksi

import os  # Untuk cek file model YOLO
import traceback  # Untuk error logging detail
import threading  # Untuk retry publisher/subscriber
import time  # Untuk statistik deteksi/logging
import csv  # Untuk logging statistik ke file
import logging  # Untuk logging ke file (opsional)

bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_yolov12_node.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("yolov12_ros2_pt_file_logger")
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

class Camera_subscriber(Node):  # Definisi class node subscriber kamera (OOP, reusable)
    def __init__(self):  # Konstruktor class
        super().__init__('camera_subscriber')  # Inisialisasi node dengan nama 'camera_subscriber'
        try:
            # Parameterization: model_path dan threshold dari parameter node/launch file
            self.declare_parameter('model_path', os.path.expanduser('~/huskybot/src/huskybot_recognition/scripts/yolo12n.pt'))  # Path default model YOLOv12
            self.declare_parameter('confidence_threshold', 0.25)  # Threshold confidence default
            self.declare_parameter('log_stats', True)  # Logging statistik deteksi ke file
            self.declare_parameter('log_stats_path', os.path.expanduser('~/huskybot_detection_log/yolov12_stats.csv'))  # Path file statistik

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

            # Retry publisher jika error (robust untuk ROS2)
            self.yolov12_pub = self._create_publisher_with_retry(Yolov12Inference, "/Yolov12_Inference", 1)
            self.img_pub = self._create_publisher_with_retry(Image, "/inference_result", 1)

            # Daftar topic kamera dan label (disesuaikan dengan Xacro Husky 6 kamera)
            self.camera_topics = {
                'camera_front':        '/camera_front/image_raw',
                'camera_front_left':   '/camera_front_left/image_raw',
                'camera_left':         '/camera_left/image_raw',
                'camera_rear':         '/camera_rear/image_raw',
                'camera_rear_right':   '/camera_rear_right/image_raw',
                'camera_right':        '/camera_right/image_raw'
            }

            self._my_subscriptions = []  # Simpan subscription agar tidak di-GC
            for cam_name, topic in self.camera_topics.items():  # Loop semua kamera
                sub = self._create_subscription_with_retry(
                    Image,
                    topic,
                    lambda msg, cam=cam_name: self.camera_callback(msg, cam),  # Callback dengan binding nama kamera
                    10
                )
                self._my_subscriptions.append(sub)
                self.get_logger().info(f"Subscribed to camera topic: {topic} ({cam_name})")
                log_to_file(f"Subscribed to camera topic: {topic} ({cam_name})")

            # Statistik deteksi (thread-safe)
            self.stats_lock = threading.Lock()
            self.stats = {'total_images': 0, 'total_detections': 0, 'per_class': {}}
            if self.log_stats:
                os.makedirs(os.path.dirname(self.log_stats_path), exist_ok=True)  # Pastikan folder log ada
                self.stats_file = open(self.log_stats_path, 'a', newline='')  # Buka file log statistik
                self.stats_writer = csv.writer(self.stats_file)
                if os.stat(self.log_stats_path).st_size == 0:  # Jika file baru, tulis header
                    self.stats_writer.writerow(['timestamp', 'camera', 'num_detections', 'class_counts'])
                self.get_logger().info(f"Logging detection stats to: {self.log_stats_path}")
                log_to_file(f"Logging detection stats to: {self.log_stats_path}")
        except Exception as e:
            self.get_logger().error(f"Error initializing Camera_subscriber: {e}\n{traceback.format_exc()}")
            log_to_file(f"Error initializing Camera_subscriber: {e}\n{traceback.format_exc()}", level='error')
            raise

    def _create_publisher_with_retry(self, msg_type, topic, queue_size, max_retry=5):  # Retry publisher jika error
        for i in range(max_retry):
            try:
                pub = self.create_publisher(msg_type, topic, queue_size)
                return pub
            except Exception as e:
                self.get_logger().warn(f"Retry publisher {topic} ({i+1}/{max_retry}): {e}")
                log_to_file(f"Retry publisher {topic} ({i+1}/{max_retry}): {e}", level='warn')
                time.sleep(1)
        self.get_logger().error(f"Gagal membuat publisher {topic} setelah {max_retry} percobaan.")
        log_to_file(f"Gagal membuat publisher {topic} setelah {max_retry} percobaan.", level='error')
        raise RuntimeError(f"Gagal membuat publisher {topic} setelah {max_retry} percobaan.")

    def _create_subscription_with_retry(self, msg_type, topic, callback, queue_size, max_retry=5):  # Retry subscription jika error
        for i in range(max_retry):
            try:
                sub = self.create_subscription(msg_type, topic, callback, queue_size)
                return sub
            except Exception as e:
                self.get_logger().warn(f"Retry subscription {topic} ({i+1}/{max_retry}): {e}")
                log_to_file(f"Retry subscription {topic} ({i+1}/{max_retry}): {e}", level='warn')
                time.sleep(1)
        self.get_logger().error(f"Gagal membuat subscription {topic} setelah {max_retry} percobaan.")
        log_to_file(f"Gagal membuat subscription {topic} setelah {max_retry} percobaan.", level='error')
        raise RuntimeError(f"Gagal membuat subscription {topic} setelah {max_retry} percobaan.")

    def camera_callback(self, data, cam_name):  # Callback saat gambar dari kamera diterima
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            log_to_file(f"CV Bridge error: {e}", level='error')
            return

        try:
            results = self.model(img)  # Jalankan YOLOv12 pada gambar
        except Exception as e:
            self.get_logger().error(f"YOLOv12 inference error: {e}\n{traceback.format_exc()}")
            log_to_file(f"YOLOv12 inference error: {e}\n{traceback.format_exc()}", level='error')
            return

        yolov12_inference = Yolov12Inference()  # Buat pesan hasil deteksi
        yolov12_inference.header.frame_id = cam_name  # Set frame_id sesuai kamera
        yolov12_inference.header.stamp = self.get_clock().now().to_msg()  # Set timestamp sekarang
        yolov12_inference.camera_name = cam_name  # Set nama kamera

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
                    c = box.cls
                    class_name = self.model.names[int(c)]
                    inference_result = InferenceResult()
                    inference_result.class_name = class_name
                    inference_result.confidence = conf
                    inference_result.top = int(b[0])
                    inference_result.left = int(b[1])
                    inference_result.bottom = int(b[2])
                    inference_result.right = int(b[3])
                    yolov12_inference.yolov12_inference.append(inference_result)
                    num_detections += 1
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1
                except Exception as e:
                    self.get_logger().warn(f"Error parsing detection result: {e}")
                    log_to_file(f"Error parsing detection result: {e}", level='warn')

        # Statistik deteksi (thread-safe)
        with self.stats_lock:
            self.stats['total_images'] += 1
            self.stats['total_detections'] += num_detections
            for cname, cnt in class_counts.items():
                self.stats['per_class'][cname] = self.stats['per_class'].get(cname, 0) + cnt
            if self.log_stats:
                try:
                    self.stats_writer.writerow([
                        time.strftime('%Y-%m-%d %H:%M:%S'),
                        cam_name,
                        num_detections,
                        dict(class_counts)
                    ])
                    self.stats_file.flush()
                except Exception as e:
                    self.get_logger().warn(f"Error writing stats to file: {e}")
                    log_to_file(f"Error writing stats to file: {e}", level='warn')

        try:
            annotated_frame = results[0].plot()  # Annotasi gambar dengan bounding box
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')  # Konversi kembali ke ROS Image
            img_msg.header = data.header  # Ikut header kamera asli
            self.img_pub.publish(img_msg)  # Publish gambar hasil deteksi (annotated)
        except Exception as e:
            self.get_logger().warn(f"Error publishing annotated image: {e}")
            log_to_file(f"Error publishing annotated image: {e}", level='warn')

        # Validasi message sebelum publish
        if validate_yolov12_inference(yolov12_inference):
            self.yolov12_pub.publish(yolov12_inference)  # Publish hasil deteksi ke topic utama
            self.get_logger().debug(f"Published Yolov12Inference for {cam_name} with {num_detections} detections.")
            log_to_file(f"Published Yolov12Inference for {cam_name} with {num_detections} detections.", level='debug')
        else:
            self.get_logger().error("Yolov12Inference message tidak valid, tidak dipublish.")
            log_to_file("Yolov12Inference message tidak valid, tidak dipublish.", level='error')

    def destroy_node(self):  # Cleanup node
        if self.log_stats and hasattr(self, 'stats_file'):
            self.stats_file.close()  # Tutup file statistik jika perlu
        super().destroy_node()

# Unit test sederhana untuk inference dan publish
def unit_test_inference_and_publish():
    import numpy as np
    rclpy.init(args=None)
    node = Camera_subscriber()
    # Buat gambar dummy (hitam)
    dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
    # Simulasi callback
    class DummyHeader:
        frame_id = 'camera_front'
        stamp = node.get_clock().now().to_msg()
    class DummyData:
        header = DummyHeader()
    try:
        img_msg = bridge.cv2_to_imgmsg(dummy_img, encoding='bgr8')
        img_msg.header = DummyHeader()
        node.camera_callback(img_msg, 'camera_front')
        print("Unit test inference and publish: OK")
    except Exception as e:
        print(f"Unit test inference and publish: FAIL - {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):  # Fungsi utama untuk menjalankan node
    rclpy.init(args=args)
    try:
        camera_subscriber = Camera_subscriber()
        # Jalankan unit test jika parameter 'run_unit_test' true
        if camera_subscriber.get_parameter_or('run_unit_test', False):
            unit_test_inference_and_publish() # Jalankan unit test
        else:
            rclpy.spin(camera_subscriber) # Jalankan node sampai shutdown
    except Exception as e:
        print(f"[ERROR] {e}\n{traceback.format_exc()}") # Print error jika ada
        log_to_file(f"Exception utama: {e}\n{traceback.format_exc()}", level='error')
    finally:
        rclpy.shutdown() # Shutdown ROS2

if __name__ == '__main__':  # Jika file dijalankan langsung
    main()  # Panggil fungsi utama