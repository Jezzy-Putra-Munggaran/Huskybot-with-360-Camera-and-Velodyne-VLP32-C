#!/usr/bin/env python3

# Import library ROS2 Python client
import rclpy
from rclpy.node import Node
# Import pesan Image dari ROS2 sensor_msgs
from sensor_msgs.msg import Image
# Import bridge untuk konversi ROS Image <-> OpenCV
from cv_bridge import CvBridge
# Import OpenCV untuk image processing
import cv2
# Import os untuk operasi file dan path
import os
# Import yaml untuk membaca file kalibrasi YAML
import yaml
# Import numpy untuk operasi array/matrix
import numpy as np
# Import time untuk monitoring
import time

# Definisikan node ROS2 untuk stitching panorama
class PanoramaStitcher(Node):
    def __init__(self):
        super().__init__('panorama_stitcher')  # Inisialisasi node dengan nama 'panorama_stitcher'
        self.bridge = CvBridge()  # Bridge untuk konversi ROS <-> OpenCV

        # Daftar urutan kamera sesuai fisik pemasangan (hexagonal, searah jarum jam)
        self.camera_topics = [
            'camera_front',
            'camera_front_left',
            'camera_left',
            'camera_rear',
            'camera_rear_right',
            'camera_right'
        ]
        # Mapping nama kamera ke topic ROS2
        self.topic_map = {
            'camera_front':      '/camera_front/image_raw',
            'camera_front_left': '/camera_front_left/image_raw',
            'camera_left':       '/camera_left/image_raw',
            'camera_rear':       '/camera_rear/image_raw',
            'camera_rear_right': '/camera_rear_right/image_raw',
            'camera_right':      '/camera_right/image_raw'
        }

        # Load parameter intrinsic (K) dan distortion (D) dari file YAML untuk setiap kamera
        self.K = {}
        self.D = {}
        calib_dir = os.path.expanduser("~/huskybot/src/huskybot_description/calibration")
        for cam in self.camera_topics:
            calib_file = os.path.join(calib_dir, f"intrinsic_{cam}.yaml")
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)
                self.K[cam] = np.array(calib['camera_matrix']['data']).reshape(3, 3)
                self.D[cam] = np.array(calib['distortion_coefficients']['data'])

        # Dictionary untuk menyimpan image dan timestamp terbaru dari setiap kamera
        self.latest_images = {}
        self.latest_stamps = {}
        # List subscription untuk setiap kamera
        self._my_subscriptions = []
        for cam_name, topic in self.topic_map.items():
            # Subscribe ke setiap topic kamera, gunakan lambda agar cam_name tetap
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=cam_name: self.image_callback(msg, cam),
                10
            )
            self._my_subscriptions.append(sub)

        # Publisher untuk panorama hasil stitching
        self.panorama_pub = self.create_publisher(Image, '/panorama/image_raw', 1)
        # Publisher untuk panorama yang akan dideteksi objek (bisa sama dengan panorama_pub)
        self.panorama_det_pub = self.create_publisher(Image, '/panorama/detection_input', 1)
        # Inisialisasi OpenCV Stitcher
        self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)

        # Folder untuk menyimpan hasil panorama sebagai file gambar
        self.save_dir = os.path.expanduser("~/panorama_results")
        os.makedirs(self.save_dir, exist_ok=True)
        self.save_count = 0  # Counter untuk penamaan file hasil

        # Monitoring dan fallback
        self.last_monitor_time = time.time()
        self.monitor_interval = 2.0  # detik
        self.max_frame_age = 1.0     # detik, fallback jika frame kamera terlalu lama

    # Callback untuk setiap pesan image dari kamera
    def image_callback(self, msg, cam_name):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        # === PROSES UNDISTORT ===
        K = self.K[cam_name]  # Matrix intrinsic kamera
        D = self.D[cam_name]  # Koefisien distorsi kamera
        h, w = img.shape[:2]  # Ambil ukuran gambar
        # Hitung matrix kamera baru yang optimal untuk undistort
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        # Undistort image menggunakan parameter kalibrasi
        img_undistorted = cv2.undistort(img, K, D, None, newcameramtx)
        # Simpan image dan timestamp terbaru untuk kamera ini
        self.latest_images[cam_name] = img_undistorted
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_stamps[cam_name] = now

        # Monitoring: cek setiap monitor_interval detik
        if time.time() - self.last_monitor_time > self.monitor_interval:
            for cam in self.camera_topics:
                if cam not in self.latest_images:
                    self.get_logger().warn(f"Kamera {cam} belum pernah publish frame!")
                else:
                    age = now - self.latest_stamps[cam]
                    if age > self.max_frame_age:
                        self.get_logger().warn(f"Frame kamera {cam} sudah lama ({age:.2f}s), kemungkinan delay atau drop!")
            self.last_monitor_time = time.time()

        # Sinkronisasi sederhana: fallback jika ada kamera delay
        if all(name in self.latest_images for name in self.camera_topics):
            stamps = [self.latest_stamps[name] for name in self.camera_topics]
            # Jika ada frame terlalu lama, warning dan tetap stitching dengan frame terakhir
            if max(stamps) - min(stamps) > 0.2:
                self.get_logger().warn("Frame kamera tidak sinkron, stitching tetap dilakukan dengan frame terakhir (fallback).")
            base_shape = self.latest_images[self.camera_topics[0]].shape[:2]  # Ukuran referensi
            images = []
            for name in self.camera_topics:
                im = self.latest_images[name]
                # Resize jika ukuran tidak sama
                if im.shape[:2] != base_shape:
                    im = cv2.resize(im, (base_shape[1], base_shape[0]))
                images.append(im)

            # Proses stitching dengan OpenCV
            status, pano = self.stitcher.stitch(images)
            if status == cv2.Stitcher_OK:
                # Konversi hasil panorama ke ROS Image dan publish
                pano_msg = self.bridge.cv2_to_imgmsg(pano, encoding='bgr8')
                pano_msg.header = msg.header
                self.panorama_pub.publish(pano_msg)
                self.panorama_det_pub.publish(pano_msg)
                self.get_logger().info("Panorama berhasil dipublish.")

                # Simpan panorama ke file
                filename = os.path.join(self.save_dir, f"panorama_{self.save_count:05d}.jpg")
                cv2.imwrite(filename, pano)
                self.save_count += 1
            else:
                self.get_logger().warn(
                    f"Stitching gagal, kode error: {status}. "
                    "Pastikan orientasi mesh tower dan yaw kamera di Xacro sudah benar (kamera menghadap keluar sisi heksagonal)."
                )

# Fungsi utama untuk menjalankan node ROS2
def main(args=None):
    rclpy.init(args=args)
    node = PanoramaStitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()