#!/usr/bin/env python3

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class untuk node ROS2
from sensor_msgs.msg import PointCloud2  # Message point cloud dari Velodyne
from yolov12_msgs.msg import Yolov12Inference  # Message hasil deteksi YOLOv12 (dari kamera 360°)
from huskybot_fusion.msg import Object3D  # Custom message untuk hasil deteksi objek 3D
import message_filters  # Untuk sinkronisasi data multi sensor (kamera & lidar)
import numpy as np  # Untuk pemrosesan data numerik/array
import struct  # Untuk parsing data PointCloud2
import tf2_ros  # Untuk transformasi antar frame (TF)
from std_msgs.msg import Header

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Sinkronisasi message: PointCloud2 (LiDAR) dan Yolov12Inference (kamera 360°)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/velodyne_points')
        self.yolo_sub = message_filters.Subscriber(self, Yolov12Inference, '/panorama/yolov12_inference')

        # ApproximateTimeSynchronizer untuk sinkronisasi data yang timestamp-nya tidak persis sama
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.yolo_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.fusion_callback)

        # Publisher hasil fusion ke topic baru (bisa divisualisasikan di RViz atau dipakai navigation)
        self.pub_fusion = self.create_publisher(Object3D, '/fusion/objects3d', 10)

        # TF buffer dan listener untuk transformasi antar frame (misal dari kamera ke lidar)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("FusionNode started: listening to /velodyne_points and /panorama/yolov12_inference")

    def fusion_callback(self, lidar_msg, yolo_msg):
        """
        Fungsi utama untuk menggabungkan hasil deteksi kamera 360° (YOLO) dan point cloud LiDAR.
        - lidar_msg: sensor_msgs/PointCloud2 (output dari Velodyne VLP-32C)
        - yolo_msg: yolov12_msgs/Yolov12Inference (output deteksi objek panorama kamera 360°)
        """
        # 1. Parse point cloud dari LiDAR
        points = self.pointcloud2_to_xyz(lidar_msg)

        # 2. Loop semua hasil deteksi objek dari kamera 360°
        for det in yolo_msg.detections:
            # Ambil bounding box 2D dari deteksi kamera
            bbox = det.bbox  # [xmin, ymin, xmax, ymax]
            label = det.label
            confidence = det.confidence

            # 3. Proyeksikan bbox 2D ke 3D (menggunakan kalibrasi kamera-LiDAR, diimplementasikan di fusion_utils.py)
            # Di sini diasumsikan ada fungsi project_bbox_to_pointcloud di fusion_utils.py
            from huskybot_fusion.fusion_utils import project_bbox_to_pointcloud

            # Dapatkan subset point cloud yang sesuai dengan bbox deteksi kamera
            object_points = project_bbox_to_pointcloud(bbox, points, lidar_msg, yolo_msg)

            if object_points is not None and len(object_points) > 0:
                # 4. Hitung bounding box 3D dari subset point cloud
                center, size, orientation = self.compute_3d_bbox(object_points)

                # 5. Buat dan publish message Object3D
                obj_msg = Object3D()
                obj_msg.header = Header()
                obj_msg.header.stamp = self.get_clock().now().to_msg()
                obj_msg.header.frame_id = lidar_msg.header.frame_id
                obj_msg.label = label
                obj_msg.center = center.tolist()
                obj_msg.size = size.tolist()
                obj_msg.orientation = orientation.tolist()
                obj_msg.confidence = confidence

                self.pub_fusion.publish(obj_msg)
                self.get_logger().info(f"Published 3D object: {label} conf={confidence:.2f}")

    def pointcloud2_to_xyz(self, cloud_msg):
        """
        Konversi sensor_msgs/PointCloud2 ke array numpy [N,3] (x, y, z).
        """
        # Ambil field offset
        fmt = 'fff'  # x, y, z float32
        points = []
        for i in range(cloud_msg.width * cloud_msg.height):
            offset = i * cloud_msg.point_step
            x, y, z = struct.unpack_from(fmt, cloud_msg.data, offset)
            points.append([x, y, z])
        return np.array(points)

    def compute_3d_bbox(self, points):
        """
        Hitung bounding box 3D axis-aligned dari point cloud objek.
        Return: center (3,), size (3,), orientation (4,) (quaternion, default [0,0,0,1])
        """
        min_pt = np.min(points, axis=0)
        max_pt = np.max(points, axis=0)
        center = (min_pt + max_pt) / 2.0
        size = max_pt - min_pt
        orientation = np.array([0, 0, 0, 1])  # Asumsi axis-aligned (tanpa rotasi)
        return center, size, orientation

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()