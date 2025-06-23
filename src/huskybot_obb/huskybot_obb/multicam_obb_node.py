#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from sensor_msgs.msg import Image  # Message ROS2 untuk image kamera
from cv_bridge import CvBridge  # Konversi ROS Image <-> OpenCV
from ultralytics import YOLO  # Library YOLOv12 (pastikan sudah install ultralytics>=v12)
import numpy as np  # Library array/matrix
import cv2  # OpenCV untuk image processing
from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil deteksi YOLOv12
from std_msgs.msg import Header  # Header ROS2 standar

class MultiCamOBBNode(Node):  # Node multicam YOLOv12 OBB, FULL OOP
    def __init__(self):
        super().__init__('multicam_obb')  # Inisialisasi node ROS2 dengan nama unik
        self.declare_parameter('cam_count', 6)  # Jumlah kamera (default 6, hexagonal)
        self.declare_parameter('model_path', "yolo11x-obb.engine")  # Path model YOLOv12 OBB
        self.declare_parameter('camera_topics', [  # Daftar topic kamera (default urutan hexagonal)
            '/camera_front/image_raw',
            '/camera_right/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_rear/image_raw',
            '/camera_left/image_raw',
            '/camera_front_left/image_raw'
        ])
        self.cam_count = self.get_parameter('cam_count').value  # Ambil jumlah kamera dari parameter
        self.model_path = self.get_parameter('model_path').value  # Ambil path model dari parameter
        self.camera_topics = self.get_parameter('camera_topics').value  # Ambil daftar topic kamera

        self.bridge = CvBridge()  # Inisialisasi bridge konversi image
        try:
            self.model = YOLO(self.model_path, task="obb")  # Load model YOLOv12 OBB (pastikan file ada)
        except Exception as e:
            self.get_logger().error(f"Error load model YOLOv12 OBB: {e}")  # Error handling file model hilang/format salah
            raise
        self.images = [None] * self.cam_count  # Buffer image untuk setiap kamera

        self.publisher = self.create_publisher(Yolov12Inference, '/detection', 10)  # Publisher hasil OBB ke topic /detection

        # Buat subscription untuk setiap kamera
        for i, topic in enumerate(self.camera_topics):
            try:
                self.create_subscription(
                    Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10  # Callback image per kamera
                )
            except Exception as e:
                self.get_logger().error(f"Error subscribe topic {topic}: {e}")  # Error handling jika topic tidak ada

        self.timer = self.create_timer(0.2, self.display_images)  # Timer untuk proses OBB dan visualisasi

    def image_callback(self, msg, idx):
        try:
            self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV
        except Exception as e:
            self.get_logger().warning(f"Error convert image: {e}")  # Error handling konversi image

    def display_images(self):
        if all(img is not None for img in self.images):  # Jika semua kamera sudah ada gambar
            try:
                annotated_images = []
                for idx, img in enumerate(self.images):
                    try:
                        results = self.model(img, verbose=False)  # Inference YOLOv12 OBB
                        annotated = results[0].plot()  # Visualisasi hasil OBB
                        annotated_images.append(annotated)
                        if hasattr(results[0], "obb"):
                            for obb in results[0].obb:
                                self.get_logger().info(f"[OBB] Camera {idx+1}: OBB={obb}")  # Logging info OBB
                        self.publish_results(results, f"Camera_{idx+1}")  # Publish hasil OBB per kamera
                    except Exception as e:
                        self.get_logger().error(f"Error YOLOv12 OBB camera {idx+1}: {e}")  # Error handling inference per kamera
                        annotated_images.append(np.zeros_like(img))  # Fallback: gambar kosong
                target_height = 240  # Tinggi target visualisasi
                resized_images = []
                for image in annotated_images:
                    h, w = image.shape[:2]
                    scale = target_height / h if h > target_height else 1
                    new_w = int(w * scale)
                    resized_image = cv2.resize(image, (new_w, target_height))
                    resized_images.append(resized_image)
                border_thickness = 5  # Ketebalan border antar kamera
                bordered_images = []
                for idx, img in enumerate(resized_images):
                    bordered_images.append(img)
                    if idx < len(resized_images) - 1:
                        border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                        bordered_images.append(border)
                combined_image = cv2.hconcat(bordered_images)  # Gabungkan semua kamera
                try:
                    cv2.imshow("MultiCam YOLOv12 OBB", combined_image)  # Tampilkan visualisasi
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warning(f"Error visualisasi OpenCV: {e}")  # Error handling jika headless/server
                self.images = [None] * self.cam_count  # Reset buffer image
            except Exception as e:
                self.get_logger().error(f"Error display_images: {e}")  # Error handling global
                self.images = [None] * self.cam_count
        else:
            available = sum(1 for img in self.images if img is not None)
            self.get_logger().info(f"Available camera feeds: {available}/{self.cam_count}")  # Info jumlah kamera aktif

    def publish_results(self, results, camera_name):
        msg = Yolov12Inference()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_name
        msg.camera_name = camera_name
        msg.frame_type = "raw"
        msg.task = "obb"  # Tipe task OBB
        msg.note = ""
        msg.yolov12_inference = []
        try:
            # Untuk OBB, hasil utama ada di boxes dan obb
            if hasattr(results[0], "boxes") and results[0].boxes is not None:
                for box in results[0].boxes:
                    det = InferenceResult()
                    det.class_name = str(box.cls.item())
                    det.confidence = float(box.conf.item())
                    det.top = int(box.xyxy[0][1])
                    det.left = int(box.xyxy[0][0])
                    det.bottom = int(box.xyxy[0][3])
                    det.right = int(box.xyxy[0][2])
                    det.track_id = int(box.id.item()) if hasattr(box, "id") and box.id is not None else -1
                    # OBB angle jika ada
                    if hasattr(box, "obb") and box.obb is not None and len(box.obb) > 0:
                        det.obb_angle = float(box.obb[0])  # Asumsi angle di index 0
                    else:
                        det.obb_angle = 0
                    det.mask_indices = []  # Kosongkan untuk OBB
                    msg.yolov12_inference.append(det)
            else:
                self.get_logger().warning(f"[publish_results] boxes attribute not found in results.")
            self.publisher.publish(msg)  # Publish hasil OBB ke topic
        except Exception as e:
            self.get_logger().error(f"Error publish_results: {e}")  # Error handling publish

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = MultiCamOBBNode()  # Inisialisasi node multicam OBB
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