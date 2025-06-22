#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from yolov12_msgs.msg import Yolov12Inference  # Custom message hasil deteksi YOLOv12 (harus ada di workspace)
from std_msgs.msg import Header  # Header ROS2 standar

class TrackingFusionNode(Node):  # Node fusion hasil multicam YOLOv12, FULL OOP
    def __init__(self):
        super().__init__('tracking_fusion')  # Inisialisasi node ROS2 dengan nama unik
        self.subs = []  # List subscription ke topic hasil deteksi/segmentasi/obb
        self.results = {}  # Buffer hasil deteksi dari semua topic
        self.publisher = self.create_publisher(Yolov12Inference, '/tracking', 10)  # Publisher hasil fusion ke topic /tracking
        topics = ['/detection', '/segmentation', '/obb']  # Daftar topic hasil deteksi multicam
        for topic in topics:
            try:
                self.subs.append(self.create_subscription(
                    Yolov12Inference, topic, self.callback, 10))  # Subscribe ke semua topic hasil deteksi
            except Exception as e:
                self.get_logger().error(f"Error subscribe topic {topic}: {e}")  # Error handling jika topic tidak ada
        self.timer = self.create_timer(0.1, self.publish_tracking)  # Timer publish hasil fusion setiap 0.1 detik

    def callback(self, msg):
        try:
            key = msg.camera_name + "_" + msg.task  # Key unik per kamera dan task
            self.results[key] = msg  # Simpan hasil deteksi terbaru
        except Exception as e:
            self.get_logger().error(f"Error di callback fusion: {e}")  # Error handling callback

    def publish_tracking(self):
        msg = Yolov12Inference()  # Inisialisasi message hasil fusion
        msg.header = Header()  # Header ROS2 standar
        msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp ROS2
        msg.header.frame_id = "tracking"  # Frame ID hasil fusion
        msg.camera_name = "tracking"  # Nama kamera hasil fusion (gabungan)
        msg.frame_type = "fusion"  # Tipe frame hasil fusion
        msg.task = "track"  # Task hasil fusion (tracking)
        msg.note = "Gabungan hasil multi-task"  # Catatan hasil fusion
        msg.yolov12_inference = []  # List hasil deteksi gabungan
        try:
            for key in self.results:
                if hasattr(self.results[key], "yolov12_inference"):
                    msg.yolov12_inference.extend(self.results[key].yolov12_inference)  # Gabungkan semua hasil deteksi
                else:
                    self.get_logger().warning(f"Message {key} tidak punya field yolov12_inference")  # Error handling field kosong
            self.publisher.publish(msg)  # Publish hasil fusion ke topic /tracking
        except Exception as e:
            self.get_logger().error(f"Error publish_tracking: {e}")  # Error handling publish

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = TrackingFusionNode()  # Inisialisasi node fusion
    try:
        rclpy.spin(node)  # Spin node sampai shutdown
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')  # Info shutdown via Ctrl+C
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}")  # Error handling fatal
    finally:
        try:
            node.destroy_node()  # Destroy node ROS2
        except Exception as e:
            print(f"Error destroy_node: {e}")
        rclpy.shutdown()  # Shutdown ROS2