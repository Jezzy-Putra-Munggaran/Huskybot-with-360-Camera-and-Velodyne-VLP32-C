#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from collections import defaultdict

class PanoramaStitcher(Node):
    def __init__(self):
        super().__init__('panorama_stitcher')
        self.bridge = CvBridge()

        self.camera_topics = [
            'cam_front',
            'cam_front_right',
            'cam_back_right',
            'cam_back',
            'cam_back_left',
            'cam_front_left'
        ]
        self.topic_map = {
            'cam_front':        '/cam_front/image_raw',
            'cam_front_right':  '/cam_front_right/image_raw',
            'cam_back_right':   '/cam_back_right/image_raw',
            'cam_back':         '/cam_back/image_raw',
            'cam_back_left':    '/cam_back_left/image_raw',
            'cam_front_left':   '/cam_front_left/image_raw'
        }

        self.latest_images = {}
        self.latest_stamps = {}
        # Ganti nama variabel agar tidak bentrok dengan property Node
        self._my_subscriptions = []
        for cam_name, topic in self.topic_map.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=cam_name: self.image_callback(msg, cam),
                10
            )
            self._my_subscriptions.append(sub)

        self.panorama_pub = self.create_publisher(Image, '/panorama/image_raw', 1)
        # Publish ke topic lain untuk deteksi objek 360°
        self.panorama_det_pub = self.create_publisher(Image, '/panorama/detection_input', 1)
        self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)

        # Folder untuk simpan panorama
        self.save_dir = os.path.expanduser("~/panorama_results")
        os.makedirs(self.save_dir, exist_ok=True)
        self.save_count = 0

    def image_callback(self, msg, cam_name):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_images[cam_name] = img
        self.latest_stamps[cam_name] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Sinkronisasi sederhana: hanya stitching jika semua frame dalam rentang waktu 0.1 detik
        if all(name in self.latest_images for name in self.camera_topics):
            stamps = [self.latest_stamps[name] for name in self.camera_topics]
            if max(stamps) - min(stamps) > 0.1:
                self.get_logger().warn("Frame kamera tidak sinkron, stitching dilewati.")
                return

            base_shape = self.latest_images[self.camera_topics[0]].shape[:2]
            images = []
            for name in self.camera_topics:
                im = self.latest_images[name]
                if name == 'cam_back':
                    im = cv2.rotate(im, cv2.ROTATE_180)
                if im.shape[:2] != base_shape:
                    im = cv2.resize(im, (base_shape[1], base_shape[0]))
                images.append(im)

            status, pano = self.stitcher.stitch(images)
            if status == cv2.Stitcher_OK:
                pano_msg = self.bridge.cv2_to_imgmsg(pano, encoding='bgr8')
                pano_msg.header = msg.header
                self.panorama_pub.publish(pano_msg)
                self.panorama_det_pub.publish(pano_msg)  # Untuk deteksi objek 360°
                self.get_logger().info("Panorama berhasil dipublish.")

                # Simpan panorama ke file
                filename = os.path.join(self.save_dir, f"panorama_{self.save_count:05d}.jpg")
                cv2.imwrite(filename, pano)
                self.save_count += 1
            else:
                self.get_logger().warn(f"Stitching gagal, kode error: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = PanoramaStitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()