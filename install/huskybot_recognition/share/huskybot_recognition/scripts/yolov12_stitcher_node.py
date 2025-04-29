#!/usr/bin/env python3  # Shebang agar file bisa dieksekusi langsung di terminal Linux

# Import library ROS2 Python client
import rclpy  # Modul utama ROS2 Python
from rclpy.node import Node  # Base class Node untuk membuat node ROS2
# Import pesan Image dari ROS2 sensor_msgs
from sensor_msgs.msg import Image  # Message standar ROS2 untuk gambar
# Import bridge untuk konversi ROS Image <-> OpenCV
from cv_bridge import CvBridge  # Untuk konversi antara ROS Image dan OpenCV
# Import OpenCV untuk image processing
import cv2  # Library utama untuk image processing dan stitching
# Import os untuk operasi file dan path
import os  # Untuk operasi file dan folder
# Import yaml untuk membaca file kalibrasi YAML
import yaml  # Untuk membaca file YAML kalibrasi kamera
# Import numpy untuk operasi array/matrix
import numpy as np  # Untuk operasi matrix dan array
# Import time untuk monitoring
import time  # Untuk monitoring waktu dan sinkronisasi

# Definisikan node ROS2 untuk stitching panorama
class PanoramaStitcher(Node):
    def __init__(self):
        super().__init__('panorama_stitcher')  # Inisialisasi node dengan nama 'panorama_stitcher'
        self.bridge = CvBridge()  # Bridge untuk konversi ROS <-> OpenCV

        # Daftar urutan kamera sesuai fisik pemasangan (hexagonal, searah jarum jam)
        self.camera_topics = [
            'camera_front',         # Kamera depan (0° yaw)
            'camera_front_left',    # Kamera depan kiri (+60° CCW)
            'camera_left',          # Kamera kiri (+120° CCW)
            'camera_rear',          # Kamera belakang (180° yaw)
            'camera_rear_right',    # Kamera belakang kanan (-120° CW)
            'camera_right'          # Kamera kanan (-60° CW)
        ]
        # Mapping nama kamera ke topic ROS2
        self.topic_map = {
            'camera_front':      '/camera_front/image_raw',        # Topic kamera depan
            'camera_front_left': '/camera_front_left/image_raw',   # Topic kamera depan kiri
            'camera_left':       '/camera_left/image_raw',         # Topic kamera kiri
            'camera_rear':       '/camera_rear/image_raw',         # Topic kamera belakang
            'camera_rear_right': '/camera_rear_right/image_raw',   # Topic kamera belakang kanan
            'camera_right':      '/camera_right/image_raw'         # Topic kamera kanan
        }

        # Load parameter intrinsic (K) dan distortion (D) dari file YAML untuk setiap kamera
        self.K = {}  # Dictionary matrix intrinsic kamera
        self.D = {}  # Dictionary koefisien distorsi kamera
        calib_dir = os.path.expanduser("~/huskybot/src/huskybot_description/calibration")  # Folder kalibrasi
        for cam in self.camera_topics:
            calib_file = os.path.join(calib_dir, f"intrinsic_{cam}.yaml")  # Path file kalibrasi
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)  # Baca file YAML
                self.K[cam] = np.array(calib['camera_matrix']['data']).reshape(3, 3)  # Matrix intrinsic
                self.D[cam] = np.array(calib['distortion_coefficients']['data'])      # Koefisien distorsi

        # Dictionary untuk menyimpan image dan timestamp terbaru dari setiap kamera
        self.latest_images = {}  # Menyimpan image terbaru per kamera
        self.latest_stamps = {}  # Menyimpan timestamp terbaru per kamera
        # List subscription untuk setiap kamera
        self._my_subscriptions = []
        for cam_name, topic in self.topic_map.items():
            # Subscribe ke setiap topic kamera, gunakan lambda agar cam_name tetap
            sub = self.create_subscription(
                Image,  # Tipe message yang disubscribe
                topic,  # Nama topic kamera
                lambda msg, cam=cam_name: self.image_callback(msg, cam),  # Callback dengan binding nama kamera
                10  # Queue size
            )
            self._my_subscriptions.append(sub)  # Simpan subscription agar tidak di-GC

        # Publisher untuk panorama hasil stitching
        self.panorama_pub = self.create_publisher(Image, '/panorama/image_raw', 1)  # Publish panorama ke topic ini
        # Publisher untuk panorama yang akan dideteksi objek (bisa sama dengan panorama_pub)
        self.panorama_det_pub = self.create_publisher(Image, '/panorama/detection_input', 1)  # Untuk pipeline deteksi
        # Inisialisasi OpenCV Stitcher
        self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)  # Inisialisasi objek stitcher OpenCV

        # Folder untuk menyimpan hasil panorama sebagai file gambar
        self.save_dir = os.path.expanduser("~/panorama_results")  # Folder hasil panorama
        os.makedirs(self.save_dir, exist_ok=True)  # Buat folder jika belum ada
        self.save_count = 0  # Counter untuk penamaan file hasil

        # Monitoring dan fallback
        self.last_monitor_time = time.time()  # Waktu monitoring terakhir
        self.monitor_interval = 2.0  # detik, interval monitoring
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
                    self.get_logger().warn(f"Kamera {cam} belum pernah publish frame!")  # Warning jika kamera belum pernah publish
                else:
                    age = now - self.latest_stamps[cam]
                    if age > self.max_frame_age:
                        self.get_logger().warn(f"Frame kamera {cam} sudah lama ({age:.2f}s), kemungkinan delay atau drop!")  # Warning jika frame lama
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
    rclpy.init(args=args)  # Inisialisasi ROS2 Python
    node = PanoramaStitcher()  # Buat instance node panorama stitcher
    rclpy.spin(node)  # Jalankan node hingga Ctrl+C
    node.destroy_node()  # Cleanup saat node selesai
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()  # Panggil fungsi main