#!/usr/bin/env python3  # Shebang agar bisa dieksekusi langsung
# -*- coding: utf-8 -*-  # Encoding file Python

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class untuk node ROS2
from sensor_msgs.msg import Image, PointCloud2, Imu  # Message ROS2 untuk kamera, LiDAR, IMU
from std_msgs.msg import Header  # Untuk header ROS2
from message_filters import ApproximateTimeSynchronizer, Subscriber  # Sinkronisasi data sensor
import traceback  # Untuk logging error detail
import logging  # Logging error/info
import os  # Untuk cek/simpan file
import yaml  # Untuk simpan data sinkron ke file YAML
import csv  # Untuk simpan data sinkron ke file CSV

# --- Custom message sinkronisasi (jika ingin publish data lengkap, bisa buat msg di msg/ lalu import di sini) ---
# from huskybot_calibration.msg import SyncedSensorStamp

class SensorTimeSynchronizer(Node):  # Node OOP untuk sinkronisasi waktu sensor
    def __init__(self):
        super().__init__('sensor_time_synchronizer')  # Inisialisasi node ROS2
        self.declare_parameter('camera_topic', '/panorama/image_raw')  # Topic kamera (bisa diubah via launch)
        self.declare_parameter('lidar_topic', '/velodyne_points')  # Topic LiDAR (bisa diubah via launch)
        self.declare_parameter('imu_topic', '/imu/data')  # Topic IMU (bisa diubah via launch)
        self.declare_parameter('queue_size', 10)  # Ukuran queue sinkronisasi
        self.declare_parameter('slop', 0.05)  # Toleransi waktu sinkronisasi (detik)
        self.declare_parameter('output_file', 'calibration_data/sync_data.yaml')  # File output data sinkron
        self.declare_parameter('output_csv', 'calibration_data/sync_data.csv')  # File output data sinkron (CSV)
        self.declare_parameter('publish_synced', True)  # Apakah publish topic sinkronisasi

        # Ambil parameter node
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value
        self.publish_synced = self.get_parameter('publish_synced').get_parameter_value().bool_value

        # Error handling: cek topic tidak kosong
        if not camera_topic or not lidar_topic or not imu_topic:
            self.get_logger().error("Topic kamera, LiDAR, atau IMU tidak boleh kosong!")
            raise ValueError("Topic sensor tidak boleh kosong.")

        # Error handling: cek folder output YAML/CSV
        for out_path in [self.output_file, self.output_csv]:
            output_dir = os.path.dirname(out_path)
            if output_dir and not os.path.exists(output_dir):
                try:
                    os.makedirs(output_dir)
                except Exception as e:
                    self.get_logger().error(f"Gagal membuat folder output: {e}")
                    raise

        # Publisher untuk data sinkron (std_msgs/Header, bisa diganti custom msg jika perlu)
        if self.publish_synced:
            self.synced_pub = self.create_publisher(Header, 'synced_header', 10)  # Publish header sinkron

        # Inisialisasi subscriber message_filters
        try:
            self.camera_sub = Subscriber(self, Image, camera_topic)
            self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic)
            self.imu_sub = Subscriber(self, Imu, imu_topic)
            self.ts = ApproximateTimeSynchronizer(
                [self.camera_sub, self.lidar_sub, self.imu_sub], queue_size=queue_size, slop=slop
            )
            self.ts.registerCallback(self.sync_callback)
        except Exception as e:
            self.get_logger().error(f"Error inisialisasi subscriber: {e}")
            self.get_logger().debug(traceback.format_exc())
            raise

        # Inisialisasi header CSV jika file baru
        if not os.path.exists(self.output_csv):
            try:
                with open(self.output_csv, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'camera_sec', 'camera_nanosec',
                        'lidar_sec', 'lidar_nanosec',
                        'imu_sec', 'imu_nanosec'
                    ])
            except Exception as e:
                self.get_logger().error(f"Gagal inisialisasi file CSV: {e}")

        self.get_logger().info("SensorTimeSynchronizer node siap. Sinkronisasi waktu sensor aktif.")

    def sync_callback(self, img_msg, lidar_msg, imu_msg):
        # Callback saat data kamera, LiDAR, dan IMU sinkron
        try:
            # Validasi header timestamp
            if img_msg.header.stamp.sec == 0 or lidar_msg.header.stamp.sec == 0 or imu_msg.header.stamp.sec == 0:
                self.get_logger().warning("Ada sensor dengan timestamp kosong!")
                return

            # Logging info sinkronisasi
            self.get_logger().info(
                f"Data sinkron: Kamera={img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec} | "
                f"LiDAR={lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec} | "
                f"IMU={imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec}"
            )

            # Publish header sinkron ke topic (bisa diganti custom msg jika perlu)
            if self.publish_synced:
                synced_header = Header()
                synced_header.stamp = img_msg.header.stamp  # Gunakan timestamp kamera sebagai acuan
                synced_header.frame_id = "synced"
                self.synced_pub.publish(synced_header)

            # Simpan data sinkron ke file YAML (append)
            try:
                sync_data = {
                    'camera_stamp': {
                        'sec': int(img_msg.header.stamp.sec),
                        'nanosec': int(img_msg.header.stamp.nanosec)
                    },
                    'lidar_stamp': {
                        'sec': int(lidar_msg.header.stamp.sec),
                        'nanosec': int(lidar_msg.header.stamp.nanosec)
                    },
                    'imu_stamp': {
                        'sec': int(imu_msg.header.stamp.sec),
                        'nanosec': int(imu_msg.header.stamp.nanosec)
                    }
                }
                with open(self.output_file, 'a') as f:
                    yaml.dump([sync_data], f)
            except Exception as e:
                self.get_logger().error(f"Gagal simpan data sinkron ke file YAML: {e}")
                self.get_logger().debug(traceback.format_exc())

            # Simpan data sinkron ke file CSV (append)
            try:
                with open(self.output_csv, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        int(img_msg.header.stamp.sec), int(img_msg.header.stamp.nanosec),
                        int(lidar_msg.header.stamp.sec), int(lidar_msg.header.stamp.nanosec),
                        int(imu_msg.header.stamp.sec), int(imu_msg.header.stamp.nanosec)
                    ])
            except Exception as e:
                self.get_logger().error(f"Gagal simpan data sinkron ke file CSV: {e}")
                self.get_logger().debug(traceback.format_exc())

        except Exception as e:
            self.get_logger().error(f"Error di sync_callback: {e}")
            self.get_logger().debug(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi ROS2
    try:
        node = SensorTimeSynchronizer()  # Buat node sinkronisasi waktu sensor
        rclpy.spin(node)  # Jalankan node
    except Exception as e:
        logging.error(f"Fatal error saat menjalankan node: {e}")
        logging.debug(traceback.format_exc())
    finally:
        rclpy.shutdown()  # Shutdown ROS2