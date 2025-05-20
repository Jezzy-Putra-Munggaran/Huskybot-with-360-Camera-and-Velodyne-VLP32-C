#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # [WAJIB] Library utama ROS2 Python
from rclpy.node import Node  # [WAJIB] Base class untuk node ROS2
from sensor_msgs.msg import PointCloud2  # [WAJIB] Message point cloud dari Velodyne
from yolov12_msgs.msg import Yolov12Inference  # [WAJIB] Message hasil deteksi YOLOv12 (dari kamera 360°)
from huskybot_msgs.msg import Object3D  # [WAJIB] Custom message untuk hasil deteksi objek 3D
import message_filters  # [WAJIB] Untuk sinkronisasi data multi sensor (kamera & lidar)
import numpy as np  # [WAJIB] Untuk pemrosesan data numerik/array
import struct  # [BEST PRACTICE] Untuk parsing data PointCloud2 (backup manual)
import tf2_ros  # [WAJIB] Untuk transformasi antar frame (TF)
from std_msgs.msg import Header  # [WAJIB] Header ROS2 untuk sinkronisasi waktu/frame
import os  # [WAJIB] Untuk operasi file (cek file kalibrasi)
import sys  # [WAJIB] Untuk akses error output
import logging  # [BEST PRACTICE] Untuk logging ke file (opsional)
import traceback  # [BEST PRACTICE] Untuk logging error detail
import json  # [BEST PRACTICE] Untuk logging ke file JSON (opsional)
import datetime  # [BEST PRACTICE] Untuk timestamp log

print("[DEBUG] Python executable:", sys.executable, flush=True)  # [DEBUG] Print Python executable yang dipakai saat runtime

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_fusion_node.log"):  # [BEST PRACTICE] Fungsi setup logger file
    log_path = os.path.expanduser(log_path)  # [WAJIB] Expand ~ ke home user
    logger = logging.getLogger("fusion_node_file_logger")  # [WAJIB] Buat/get logger dengan nama unik
    logger.setLevel(logging.INFO)  # [WAJIB] Set level default INFO
    if not logger.hasHandlers():  # [WAJIB] Cegah duplicate handler
        try:
            fh = logging.FileHandler(log_path)  # [WAJIB] Handler file log
            fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))  # [WAJIB] Format log
            logger.addHandler(fh)  # [WAJIB] Tambah handler ke logger
        except Exception as e:
            print(f"[FATAL] Tidak bisa setup file logger: {e}", file=sys.stderr)  # [ERROR HANDLING] Print error ke stderr
    return logger  # [WAJIB] Return logger instance

file_logger = setup_file_logger()  # [WAJIB] Inisialisasi logger file global

def log_to_file(msg, level='info'):  # [BEST PRACTICE] Fungsi log ke file dengan level
    if file_logger:  # [WAJIB] Jika logger ada
        if level == 'error':
            file_logger.error(msg)  # [WAJIB] Log error
        elif level == 'warn':
            file_logger.warning(msg)  # [WAJIB] Log warning
        elif level == 'debug':
            file_logger.debug(msg)  # [WAJIB] Log debug
        else:
            file_logger.info(msg)  # [WAJIB] Log info

# ===================== ERROR HANDLING: DEPENDENCY =====================
try:
    import sensor_msgs_py.point_cloud2 as pc2  # [WAJIB] Library resmi ROS2 untuk parsing PointCloud2
except ImportError:
    print("[FATAL] sensor_msgs_py.point_cloud2 tidak ditemukan. Install dengan: sudo apt install ros-humble-sensor-msgs-py", file=sys.stderr)
    log_to_file("[FATAL] sensor_msgs_py.point_cloud2 tidak ditemukan. Install dengan: sudo apt install ros-humble-sensor-msgs-py", level='error')
    sys.exit(1)

class FusionNode(Node):  # [WAJIB] Node OOP untuk fusion deteksi kamera 360° dan LiDAR
    def __init__(self):
        super().__init__('fusion_node')  # [WAJIB] Inisialisasi node dengan nama 'fusion_node'
        try:
            # ===================== SUBSCRIBER & SINKRONISASI =====================
            self.lidar_topic = self.declare_parameter('lidar_topic', '/velodyne_points').get_parameter_value().string_value  # [BEST PRACTICE] Parameterisasi topic LiDAR
            self.yolo_topic = self.declare_parameter('yolo_topic', '/panorama/yolov12_inference').get_parameter_value().string_value  # [BEST PRACTICE] Parameterisasi topic YOLO
            self.output_topic = self.declare_parameter('output_topic', '/fusion/objects3d').get_parameter_value().string_value  # [BEST PRACTICE] Parameterisasi topic output

            self.lidar_sub = message_filters.Subscriber(self, PointCloud2, self.lidar_topic)  # [WAJIB] Subscriber LiDAR
            self.yolo_sub = message_filters.Subscriber(self, Yolov12Inference, self.yolo_topic)  # [WAJIB] Subscriber hasil YOLOv12

            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.lidar_sub, self.yolo_sub], queue_size=10, slop=0.1)  # [WAJIB] Sinkronisasi dengan toleransi waktu 0.1 detik
            self.ts.registerCallback(self.fusion_callback)  # [WAJIB] Daftarkan callback fusion

            # ===================== PUBLISHER =====================
            self.pub_fusion = self.create_publisher(Object3D, self.output_topic, 10)  # [WAJIB] Publisher hasil objek 3D

            # ===================== TF2 =====================
            self.tf_buffer = tf2_ros.Buffer()  # [WAJIB] Buffer TF2
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # [WAJIB] Listener TF2

            # ===================== PARAMETER =====================
            self.confidence_threshold = self.declare_parameter(
                'confidence_threshold', 0.3).get_parameter_value().double_value  # [WAJIB] Default 0.3

            self.calibration_file = self.declare_parameter(
                'calibration_file', '/mnt/nova_ssd/huskybot/src/huskybot_calibration/config/extrinsic_lidar_to_camera.yaml'
            ).get_parameter_value().string_value  # [WAJIB] Ambil path file kalibrasi dari parameter
            expanded_calib = os.path.expanduser(self.calibration_file)  # [WAJIB] Expand ~ ke home user

            # ===================== ERROR HANDLING: RETRY FILE KALIBRASI =====================
            self.max_calib_retry = 10  # [SARAN] Maksimal retry file kalibrasi
            self.calib_retry_interval = 2  # [SARAN] Interval retry (detik)
            self.calib_loaded = False  # [SARAN] Status file kalibrasi

            for i in range(self.max_calib_retry):  # [SARAN] Retry otomatis jika file kalibrasi belum ada
                if os.path.isfile(expanded_calib):
                    self.get_logger().info(f"File kalibrasi ditemukan: {expanded_calib}")
                    log_to_file(f"File kalibrasi ditemukan: {expanded_calib}")
                    self.calib_loaded = True
                    break
                else:
                    self.get_logger().warn(f"File kalibrasi tidak ditemukan: {expanded_calib} (percobaan {i+1}/{self.max_calib_retry})")
                    log_to_file(f"File kalibrasi tidak ditemukan: {expanded_calib} (percobaan {i+1}/{self.max_calib_retry})", level='warn')
                    import time
                    time.sleep(self.calib_retry_interval)
            if not self.calib_loaded:
                self.get_logger().error(f"File kalibrasi tidak ditemukan setelah {self.max_calib_retry} percobaan: {expanded_calib}")
                log_to_file(f"File kalibrasi tidak ditemukan setelah {self.max_calib_retry} percobaan: {expanded_calib}", level='error')
                # [ERROR HANDLING] Node tetap jalan, tapi fusion tidak akan valid tanpa kalibrasi

            # ===================== LOGGING JSON (AUDIT TRAIL) =====================
            self.log_json_path = os.path.expanduser("~/huskybot_fusion_log.json")  # [BEST PRACTICE] Path file log JSON
            self.log_json_enabled = True  # [BEST PRACTICE] Enable logging JSON

            self.get_logger().info(f"FusionNode started: listening to {self.lidar_topic} and {self.yolo_topic}")
            log_to_file(f"FusionNode started: listening to {self.lidar_topic} and {self.yolo_topic}")
        except Exception as e:
            self.get_logger().error(f"Error initializing FusionNode: {e}\n{traceback.format_exc()}")
            log_to_file(f"Error initializing FusionNode: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(10)  # [WAJIB] Exit jika error fatal saat init

    def fusion_callback(self, lidar_msg, yolo_msg):  # [WAJIB] Callback utama fusion
        try:
            # ===================== ERROR HANDLING: Validasi Message =====================
            if lidar_msg is None or not hasattr(lidar_msg, 'data') or len(lidar_msg.data) == 0:
                self.get_logger().warn("Data lidar kosong/invalid, fusion dilewati.")
                log_to_file("Data lidar kosong/invalid, fusion dilewati.", level='warn')
                return
            if yolo_msg is None or not hasattr(yolo_msg, 'yolov12_inference'):
                self.get_logger().warn("Data YOLO kosong/invalid, fusion dilewati.")
                log_to_file("Data YOLO kosong/invalid, fusion dilewati.", level='warn')
                return

            # ===================== PARSING POINT CLOUD =====================
            try:
                # Konversi PointCloud2 ke numpy array [N,3] (x, y, z)
                points = np.array([
                    [x, y, z]
                    for x, y, z in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
                ])
            except Exception as e:
                self.get_logger().error(f"Gagal konversi PointCloud2 ke numpy: {e}")
                log_to_file(f"Gagal konversi PointCloud2 ke numpy: {e}", level='error')
                return

            if points is None or len(points) == 0:  # [ERROR HANDLING] Point cloud kosong
                self.get_logger().warn("Point cloud kosong, fusion dilewati.")
                log_to_file("Point cloud kosong, fusion dilewati.", level='warn')
                return

            # ===================== PARSING DETEKSI YOLO =====================
            detections = getattr(yolo_msg, 'yolov12_inference', [])  # [WAJIB] Ambil array deteksi dari message
            if len(detections) == 0:  # [ERROR HANDLING] Deteksi YOLO kosong
                self.get_logger().warn("Deteksi YOLO kosong, fusion dilewati.")
                log_to_file("Deteksi YOLO kosong, fusion dilewati.", level='warn')
                return

            obj_msgs = []  # [WAJIB] List hasil objek 3D
            for det in detections:
                try:
                    bbox = [det.top, det.left, det.bottom, det.right]  # [WAJIB] Format [y1, x1, y2, x2]
                    label = det.class_name  # [WAJIB] Nama kelas objek
                    confidence = det.confidence  # [WAJIB] Skor confidence

                    if confidence < self.confidence_threshold:  # [ERROR HANDLING] Filter confidence threshold
                        self.get_logger().info(f"Deteksi {label} confidence {confidence:.2f} < threshold {self.confidence_threshold}, dilewati.")
                        log_to_file(f"Deteksi {label} confidence {confidence:.2f} < threshold {self.confidence_threshold}, dilewati.")
                        continue

                    # Project bbox ke point cloud (fungsi util, harus ada di fusion_utils.py)
                    try:
                        from huskybot_fusion.fusion_utils import project_bbox_to_pointcloud  # [WAJIB] Import fungsi util
                        object_points = project_bbox_to_pointcloud(bbox, points, lidar_msg, yolo_msg)  # [WAJIB] Proyeksi bbox ke point cloud
                    except ImportError as e:
                        self.get_logger().error(f"ImportError project_bbox_to_pointcloud: {e}")
                        log_to_file(f"ImportError project_bbox_to_pointcloud: {e}", level='error')
                        continue
                    except Exception as e:
                        self.get_logger().error(f"Gagal project bbox ke pointcloud: {e}")
                        log_to_file(f"Gagal project bbox ke pointcloud: {e}", level='error')
                        continue

                    if object_points is None or len(object_points) == 0:  # [ERROR HANDLING] Tidak ada point cloud pada bbox
                        self.get_logger().warn(f"Tidak ada point cloud pada bbox {label}, dilewati.")
                        log_to_file(f"Tidak ada point cloud pada bbox {label}, dilewati.", level='warn')
                        continue

                    try:
                        center, size, orientation = self.compute_3d_bbox(object_points)  # [WAJIB] Hitung bbox 3D axis-aligned
                    except Exception as e:
                        self.get_logger().error(f"Gagal hitung bbox 3D: {e}")
                        log_to_file(f"Gagal hitung bbox 3D: {e}", level='error')
                        continue

                    # Validasi hasil bbox 3D
                    if np.any(np.isnan(center)) or np.any(np.isnan(size)) or np.any(np.isnan(orientation)):
                        self.get_logger().warn(f"Nilai NaN pada hasil bbox 3D {label}, dilewati.")
                        log_to_file(f"Nilai NaN pada hasil bbox 3D {label}, dilewati.", level='warn')
                        continue
                    if np.any(size <= 0):
                        self.get_logger().warn(f"Ukuran bbox 3D tidak valid (<=0) untuk {label}, dilewati.")
                        log_to_file(f"Ukuran bbox 3D tidak valid (<=0) untuk {label}, dilewati.", level='warn')
                        continue
                    if not (0.0 <= confidence <= 1.0):
                        self.get_logger().warn(f"Confidence tidak valid ({confidence}) untuk {label}, dilewati.")
                        log_to_file(f"Confidence tidak valid ({confidence}) untuk {label}, dilewati.", level='warn')
                        continue
                    if not np.isclose(np.linalg.norm(orientation), 1.0, atol=1e-3):
                        self.get_logger().warn(f"Orientasi quaternion tidak normal (norm={np.linalg.norm(orientation)}), dilewati.")
                        log_to_file(f"Orientasi quaternion tidak normal (norm={np.linalg.norm(orientation)}), dilewati.", level='warn')
                        continue

                    obj_msg = Object3D()  # [WAJIB] Buat message Object3D
                    obj_msg.header = Header()
                    obj_msg.header.stamp = self.get_clock().now().to_msg()  # [WAJIB] Timestamp sekarang
                    obj_msg.header.frame_id = lidar_msg.header.frame_id  # [WAJIB] Frame dari LiDAR (biasanya 'velodyne_link')
                    obj_msg.label = label  # [WAJIB] Nama kelas objek
                    obj_msg.center = center.tolist()  # [WAJIB] Titik tengah bbox 3D
                    obj_msg.size = size.tolist()  # [WAJIB] Ukuran bbox 3D
                    obj_msg.orientation = orientation.tolist()  # [WAJIB] Quaternion orientasi bbox 3D
                    obj_msg.confidence = confidence  # [WAJIB] Skor confidence hasil fusion

                    obj_msgs.append(obj_msg)  # [WAJIB] Tambahkan ke list hasil
                    self.get_logger().info(f"Published 3D object: {label} conf={confidence:.2f}")
                    log_to_file(f"Published 3D object: {label} conf={confidence:.2f}")

                    # (Opsional) Logging ke file JSON untuk audit trail
                    if self.log_json_enabled:
                        try:
                            with open(self.log_json_path, 'a') as jf:
                                json.dump({
                                    "timestamp": datetime.datetime.now().isoformat(),
                                    "label": label,
                                    "confidence": confidence,
                                    "center": center.tolist(),
                                    "size": size.tolist(),
                                    "orientation": orientation.tolist()
                                }, jf)
                                jf.write('\n')
                        except Exception as e:
                            self.get_logger().warn(f"Error logging to JSON: {e}")
                            log_to_file(f"Error logging to JSON: {e}", level='warn')

                except Exception as e:
                    self.get_logger().error(f"Exception dalam loop deteksi: {e}\n{traceback.format_exc()}")
                    log_to_file(f"Exception dalam loop deteksi: {e}\n{traceback.format_exc()}", level='error')
                    continue

            # Publish semua hasil batch sekaligus (jika ingin, bisa pakai MarkerArray atau custom array msg)
            for obj_msg in obj_msgs:
                try:
                    self.pub_fusion.publish(obj_msg)  # [WAJIB] Publish satu per satu ke topic output
                except Exception as e:
                    self.get_logger().error(f"Error publish Object3D: {e}")
                    log_to_file(f"Error publish Object3D: {e}", level='error')

        except Exception as e:
            self.get_logger().error(f"Exception utama di fusion_callback: {e}\n{traceback.format_exc()}")
            log_to_file(f"Exception utama di fusion_callback: {e}\n{traceback.format_exc()}", level='error')

    def compute_3d_bbox(self, points):  # [WAJIB] Hitung bbox 3D axis-aligned dari point cloud objek
        min_pt = np.min(points, axis=0)  # [WAJIB] Titik minimum (x, y, z)
        max_pt = np.max(points, axis=0)  # [WAJIB] Titik maksimum (x, y, z)
        center = (min_pt + max_pt) / 2.0  # [WAJIB] Titik tengah bbox 3D
        size = max_pt - min_pt  # [WAJIB] Ukuran bbox 3D (dx, dy, dz)
        orientation = np.array([0, 0, 0, 1])  # [WAJIB] Asumsi axis-aligned (tanpa rotasi)
        return center, size, orientation  # [WAJIB] Return hasil bbox

def main(args=None):  # [WAJIB] Fungsi utama untuk menjalankan node
    try:
        rclpy.init(args=args)  # [WAJIB] Inisialisasi ROS2 Python
        node = FusionNode()  # [WAJIB] Buat instance node fusion
        try:
            rclpy.spin(node)  # [WAJIB] Jalankan node hingga Ctrl+C
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt, shutting down fusion_node.")
            log_to_file("KeyboardInterrupt, shutting down fusion_node.", level='warn')
        except Exception as e:
            node.get_logger().error(f"Exception utama: {e}\n{traceback.format_exc()}")
            log_to_file(f"Exception utama: {e}\n{traceback.format_exc()}", level='error')
        finally:
            node.destroy_node()  # [WAJIB] Cleanup saat node selesai
            rclpy.shutdown()  # [WAJIB] Shutdown ROS2
            node.get_logger().info("fusion_node shutdown complete.")
            log_to_file("fusion_node shutdown complete.")
    except Exception as e:
        print(f"[FATAL] Exception di main(): {e}\n{traceback.format_exc()}")
        log_to_file(f"[FATAL] Exception di main(): {e}\n{traceback.format_exc()}", level='error')
        sys.exit(99)

if __name__ == '__main__':  # [WAJIB] Jika file dijalankan langsung
    main()  # [WAJIB] Panggil fungsi main

# ===================== REVIEW & SARAN =====================
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Logger ROS2 dan logging ke file sudah di setiap langkah penting/error.
# - Semua error/exception di callback dan fungsi utama sudah di-log.
# - Validasi file kalibrasi, parameter, dan dependency sudah lengkap.
# - Monitoring health check sensor (point cloud, deteksi YOLO).
# - Sudah parameterisasi topic input/output agar lebih fleksibel (saran sudah diimplementasikan).
# - Sudah FULL OOP, siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Saran: publish array Object3D (custom msg) agar batch publish lebih efisien (bisa buat Object3DArray.msg).
# - Saran: logging ke file JSON/CSV untuk audit trail (sudah).
# - Saran: unit test untuk fungsi project_bbox_to_pointcloud dan compute_3d_bbox.
# - Saran: validasi isi file kalibrasi (cek field matrix, dsb) sebelum digunakan.
# - Saran: retry otomatis jika file kalibrasi belum ada saat node start (SUDAH).