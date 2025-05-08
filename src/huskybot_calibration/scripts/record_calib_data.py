#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class untuk node ROS2
from sensor_msgs.msg import Image, PointCloud2  # Message ROS2 untuk kamera dan LiDAR
import os  # Untuk operasi file/folder
import argparse  # Untuk parsing argumen CLI
import datetime  # Untuk timestamp file
import yaml  # Untuk simpan metadata
import logging  # Untuk logging error/info
from cv_bridge import CvBridge  # Konversi ROS Image <-> OpenCV
import numpy as np  # Untuk operasi numerik
import time  # Untuk validasi topic aktif

def validate_meta_yaml(meta_path, logger=None):  # Fungsi validasi file metadata YAML
    """
    Validasi isi file metadata hasil rekaman sample kalibrasi.
    Return True jika valid, False jika tidak.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    if not os.path.isfile(meta_path):  # Cek file metadata ada
        msg = f"File metadata tidak ditemukan: {meta_path}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        return False
    try:
        with open(meta_path, 'r') as f:
            data = yaml.safe_load(f)  # Load isi file YAML
        for k in ['image_stamp', 'lidar_stamp', 'camera_topic', 'lidar_topic', 'image_file', 'lidar_file']:
            if k not in data:  # Cek semua field wajib ada
                msg = f"Field wajib '{k}' tidak ada di metadata."
                if logger: logger.error(msg)
                else: logging.error(msg)
                return False
        if not os.path.isfile(data['image_file']):  # Cek file image ada
            msg = f"File image tidak ditemukan: {data['image_file']}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            return False
        if not os.path.isfile(data['lidar_file']):  # Cek file pointcloud ada
            msg = f"File pointcloud tidak ditemukan: {data['lidar_file']}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            return False
        return True  # Semua valid
    except Exception as e:
        msg = f"Error validasi file metadata: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        return False

def wait_for_topic(node, topic, timeout=10.0, min_publishers=1):  # Fungsi validasi topic aktif
    """Tunggu sampai topic punya minimal publisher aktif, atau timeout."""
    start = time.time()
    while time.time() - start < timeout:
        count = node.count_publishers(topic)
        if count >= min_publishers:
            node.get_logger().info(f"Topic {topic} aktif dengan {count} publisher.")
            return True
        node.get_logger().warn(f"Menunggu publisher aktif di topic {topic}...")
        time.sleep(0.5)
    node.get_logger().error(f"Timeout: Topic {topic} tidak punya publisher aktif setelah {timeout} detik.")
    return False

class CalibDataRecorder(Node):  # Node OOP untuk rekam data sinkron kamera-LiDAR
    def __init__(self, output_dir, camera_topic, lidar_topic, max_samples):
        super().__init__('calib_data_recorder')  # Inisialisasi node ROS2
        self.output_dir = output_dir  # Folder output data
        self.camera_topic = camera_topic  # Topic kamera
        self.lidar_topic = lidar_topic  # Topic LiDAR
        self.max_samples = max_samples  # Jumlah sample yang ingin direkam
        self.bridge = CvBridge()  # Bridge untuk konversi image
        self.sample_count = 0  # Counter sample

        # Validasi topic aktif sebelum subscribe
        if not wait_for_topic(self, self.camera_topic):
            self.get_logger().error(f"Topic kamera {self.camera_topic} tidak aktif. Node exit.")
            rclpy.shutdown()
            exit(1)
        if not wait_for_topic(self, self.lidar_topic):
            self.get_logger().error(f"Topic LiDAR {self.lidar_topic} tidak aktif. Node exit.")
            rclpy.shutdown()
            exit(2)

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)  # Sub kamera
        self.lidar_sub = self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)  # Sub LiDAR
        self.last_image = None  # Buffer image terakhir
        self.last_image_stamp = None  # Timestamp image terakhir
        self.last_lidar = None  # Buffer LiDAR terakhir
        self.last_lidar_stamp = None  # Timestamp LiDAR terakhir
        self.ensure_dir_exists(self.output_dir)  # Pastikan folder output ada
        self.get_logger().info(f"Recorder siap. Output: {self.output_dir}")

    def image_callback(self, msg):  # Callback saat data kamera masuk
        self.last_image = msg  # Simpan image terakhir
        self.last_image_stamp = msg.header.stamp  # Simpan timestamp image

    def lidar_callback(self, msg):  # Callback saat data LiDAR masuk
        self.last_lidar = msg  # Simpan LiDAR terakhir
        self.last_lidar_stamp = msg.header.stamp  # Simpan timestamp LiDAR
        self.try_save_sample()  # Coba simpan sample jika data sinkron

    def try_save_sample(self):  # Fungsi untuk cek dan simpan sample sinkron
        if self.last_image is None or self.last_lidar is None:
            return  # Tunggu data lengkap
        # Sinkronisasi sederhana: cek selisih waktu < 0.1 detik
        t_img = self.last_image_stamp.sec + self.last_image_stamp.nanosec * 1e-9
        t_lidar = self.last_lidar_stamp.sec + self.last_lidar_stamp.nanosec * 1e-9
        if abs(t_img - t_lidar) > 0.1:
            return  # Data tidak sinkron, skip
        # Simpan data
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        img_path = os.path.join(self.output_dir, f'image_{timestamp}.png')
        lidar_path = os.path.join(self.output_dir, f'lidar_{timestamp}.pcd')
        meta_path = os.path.join(self.output_dir, f'meta_{timestamp}.yaml')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='bgr8')  # Konversi ke OpenCV
            import cv2
            cv2.imwrite(img_path, cv_image)  # Simpan image ke file
        except Exception as e:
            self.get_logger().error(f"Gagal simpan image: {e}")
            return
        try:
            # Simpan pointcloud ke file PCD (format ASCII sederhana)
            from sensor_msgs_py import point_cloud2
            points = np.array([p[:3] for p in point_cloud2.read_points(self.last_lidar, field_names=("x", "y", "z"), skip_nans=True)])
            with open(lidar_path, 'w') as f:
                f.write(f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {points.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {points.shape[0]}\nDATA ascii\n")
                for pt in points:
                    f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
        except Exception as e:
            self.get_logger().error(f"Gagal simpan pointcloud: {e}")
            return
        try:
            # Simpan metadata (timestamp, topic, dsb)
            meta = {
                'image_stamp': float(t_img),
                'lidar_stamp': float(t_lidar),
                'camera_topic': self.camera_topic,
                'lidar_topic': self.lidar_topic,
                'image_file': img_path,
                'lidar_file': lidar_path
            }
            with open(meta_path, 'w') as f:
                yaml.dump(meta, f)
        except Exception as e:
            self.get_logger().error(f"Gagal simpan metadata: {e}")
            return
        # Validasi isi file metadata dan file output
        if not validate_meta_yaml(meta_path, logger=self.get_logger()):
            self.get_logger().error(f"Validasi file metadata/output GAGAL: {meta_path}")
            return
        self.sample_count += 1
        self.get_logger().info(f"Sample {self.sample_count} disimpan: {img_path}, {lidar_path}")
        # Reset buffer agar tidak double save
        self.last_image = None
        self.last_lidar = None
        if self.sample_count >= self.max_samples:
            self.get_logger().info("Jumlah sample maksimum tercapai, shutdown node.")
            rclpy.shutdown()

    @staticmethod
    def ensure_dir_exists(path):  # Pastikan folder ada
        try:
            if not os.path.exists(path):
                os.makedirs(path)
        except Exception as e:
            logging.error(f"Gagal membuat folder {path}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Rekam data sinkron kamera-LiDAR untuk kalibrasi")
    parser.add_argument('--output', type=str, default='calibration_data/', help='Folder output data')
    parser.add_argument('--camera_topic', type=str, default='/panorama/image_raw', help='Topic kamera')
    parser.add_argument('--lidar_topic', type=str, default='/velodyne_points', help='Topic LiDAR')
    parser.add_argument('--max_samples', type=int, default=20, help='Jumlah sample yang ingin direkam')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = CalibDataRecorder(args.output, args.camera_topic, args.lidar_topic, args.max_samples)
        rclpy.spin(node)
    except Exception as e:
        logging.error(f"Fatal error saat menjalankan recorder: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Penjelasan:
# - Validasi topic aktif sebelum subscribe sudah diimplementasikan (wait_for_topic).
# - Semua error handling sudah best practice: log error, exit jika fatal, warning jika recoverable.
# - Kode sudah FULL OOP, modular, dan siap untuk ROS2 Humble, Gazebo, dan robot real.
# - Keterhubungan: topic, output folder, metadata, dan file sudah konsisten dengan workspace lain.
# - Saran peningkatan: bisa tambahkan validasi topic subscriber jika node ini publisher, dan unit test untuk wait_for_topic.