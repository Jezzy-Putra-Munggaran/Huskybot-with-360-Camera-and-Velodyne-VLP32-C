#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
import traceback
import logging
import sys

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_panorama_stitcher.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("panorama_stitcher_file_logger")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        try:
            fh = logging.FileHandler(log_path)
            fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
            logger.addHandler(fh)
        except Exception as e:
            print(f"[ERROR] Tidak bisa membuat file log: {e}")
    return logger

file_logger = setup_file_logger()

def log_to_file(msg, level='info'):
    if file_logger:
        try:
            if level == 'error':
                file_logger.error(msg)
            elif level == 'warn':
                file_logger.warning(msg)
            elif level == 'debug':
                file_logger.debug(msg)
            else:
                file_logger.info(msg)
        except Exception as e:
            print(f"[ERROR] Gagal logging ke file: {e}")

class PanoramaStitcher(Node):
    def __init__(self):
        super().__init__('panorama_stitcher')
        self.bridge = CvBridge()

        # ===================== PARAMETERISASI NODE =====================
        self.declare_parameter('save_dir', os.path.expanduser("~/panorama_results"))
        self.declare_parameter('monitor_interval', 2.0)
        self.declare_parameter('max_frame_age', 1.0)
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_file_path', os.path.expanduser("~/huskybot_panorama_stitcher.log"))
        self.declare_parameter('output_topic', '/panorama/image_raw')
        self.declare_parameter('detection_input_topic', '/panorama/detection_input')

        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.monitor_interval = self.get_parameter('monitor_interval').get_parameter_value().double_value
        self.max_frame_age = self.get_parameter('max_frame_age').get_parameter_value().double_value
        self.log_to_file_flag = self.get_parameter('log_to_file').get_parameter_value().bool_value
        self.log_file_path = self.get_parameter('log_file_path').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.detection_input_topic = self.get_parameter('detection_input_topic').get_parameter_value().string_value

        self.get_logger().info(f"Parameter: save_dir={self.save_dir}, monitor_interval={self.monitor_interval}, max_frame_age={self.max_frame_age}, log_to_file={self.log_to_file_flag}, log_file_path={self.log_file_path}, output_topic={self.output_topic}, detection_input_topic={self.detection_input_topic}")
        log_to_file(f"Parameter: save_dir={self.save_dir}, monitor_interval={self.monitor_interval}, max_frame_age={self.max_frame_age}, log_to_file={self.log_to_file_flag}, log_file_path={self.log_file_path}, output_topic={self.output_topic}, detection_input_topic={self.detection_input_topic}")

        # ===================== DAFTAR KAMERA DAN TOPIC =====================
        self.camera_topics = [
            'camera_front',
            'camera_front_left',
            'camera_left',
            'camera_rear',
            'camera_rear_right',
            'camera_right'
        ]
        self.topic_map = {
            'camera_front':      '/camera_front/image_raw',
            'camera_front_left': '/camera_front_left/image_raw',
            'camera_left':       '/camera_left/image_raw',
            'camera_rear':       '/camera_rear/image_raw',
            'camera_rear_right': '/camera_rear_right/image_raw',
            'camera_right':      '/camera_right/image_raw'
        }

        # ===================== SUBSCRIPTION KAMERA =====================
        self.latest_images = {}
        self.latest_stamps = {}
        self._my_subscriptions = []
        for cam_name, topic in self.topic_map.items():
            try:
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam=cam_name: self.image_callback(msg, cam),
                    10
                )
                self._my_subscriptions.append(sub)
                self.get_logger().info(f"Subscribed to camera topic: {topic} ({cam_name})")
                log_to_file(f"Subscribed to camera topic: {topic} ({cam_name})")
            except Exception as e:
                self.get_logger().error(f"Gagal subscribe ke {topic}: {e}")
                log_to_file(f"Gagal subscribe ke {topic}: {e}", level='error')

        # ===================== PUBLISHER PANORAMA =====================
        self.panorama_pub = self.create_publisher(Image, self.output_topic, 1)
        self.panorama_det_pub = self.create_publisher(Image, self.detection_input_topic, 1)
        try:
            self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            self.get_logger().info("OpenCV Stitcher initialized.")
            log_to_file("OpenCV Stitcher initialized.")
        except Exception as e:
            self.get_logger().error(f"Gagal inisialisasi OpenCV Stitcher: {e}")
            log_to_file(f"Gagal inisialisasi OpenCV Stitcher: {e}", level='error')
            raise

        # ===================== FOLDER SIMPAN PANORAMA =====================
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f"Panorama save directory: {self.save_dir}")
            log_to_file(f"Panorama save directory: {self.save_dir}")
        except Exception as e:
            self.get_logger().error(f"Gagal membuat folder panorama: {e}")
            log_to_file(f"Gagal membuat folder panorama: {e}", level='error')
            raise
        self.save_count = 0

        # ===================== MONITORING =====================
        self.last_monitor_time = time.time()

    def image_callback(self, msg, cam_name):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error kamera {cam_name}: {e}")
            log_to_file(f"CV Bridge error kamera {cam_name}: {e}", level='error')
            return
        except Exception as e:
            self.get_logger().error(f"Error konversi image kamera {cam_name}: {e}")
            log_to_file(f"Error konversi image kamera {cam_name}: {e}", level='error')
            return

        self.latest_images[cam_name] = img
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_stamps[cam_name] = now

        # ===================== MONITORING HEALTH KAMERA =====================
        if time.time() - self.last_monitor_time > self.monitor_interval:
            for cam in self.camera_topics:
                if cam not in self.latest_images:
                    self.get_logger().warn(f"Kamera {cam} belum pernah publish frame!")
                    log_to_file(f"Kamera {cam} belum pernah publish frame!", level='warn')
                else:
                    age = now - self.latest_stamps[cam]
                    if age > self.max_frame_age:
                        self.get_logger().warn(f"Frame kamera {cam} sudah lama ({age:.2f}s), kemungkinan delay atau drop!")
                        log_to_file(f"Frame kamera {cam} sudah lama ({age:.2f}s), kemungkinan delay atau drop!", level='warn')
            self.last_monitor_time = time.time()

        # ===================== SINKRONISASI DAN STITCHING =====================
        if all(name in self.latest_images for name in self.camera_topics):
            stamps = [self.latest_stamps[name] for name in self.camera_topics]
            if max(stamps) - min(stamps) > 0.2:
                self.get_logger().warn("Frame kamera tidak sinkron, stitching tetap dilakukan dengan frame terakhir (fallback).")
                log_to_file("Frame kamera tidak sinkron, stitching tetap dilakukan dengan frame terakhir (fallback).", level='warn')
            base_shape = self.latest_images[self.camera_topics[0]].shape[:2]
            images = []
            for name in self.camera_topics:
                im = self.latest_images[name]
                if im.shape[:2] != base_shape:
                    im = cv2.resize(im, (base_shape[1], base_shape[0]))
                images.append(im)

            try:
                status, pano = self.stitcher.stitch(images)
                if status == cv2.Stitcher_OK:
                    pano_msg = self.bridge.cv2_to_imgmsg(pano, encoding='bgr8')
                    pano_msg.header = msg.header
                    self.panorama_pub.publish(pano_msg)
                    self.panorama_det_pub.publish(pano_msg)
                    self.get_logger().info("Panorama berhasil dipublish.")
                    log_to_file("Panorama berhasil dipublish.")

                    filename = os.path.join(self.save_dir, f"panorama_{self.save_count:05d}.jpg")
                    try:
                        cv2.imwrite(filename, pano)
                        self.save_count += 1
                    except Exception as e:
                        self.get_logger().warn(f"Error simpan file panorama: {e}")
                        log_to_file(f"Error simpan file panorama: {e}", level='warn')
                else:
                    warn_msg = (
                        f"Stitching gagal, kode error: {status}. "
                        "Pastikan orientasi mesh tower dan yaw kamera di Xacro sudah benar (kamera menghadap keluar sisi heksagonal)."
                    )
                    self.get_logger().warn(warn_msg)
                    log_to_file(warn_msg, level='warn')
            except Exception as e:
                self.get_logger().error(f"Error saat stitching panorama: {e}\n{traceback.format_exc()}")
                log_to_file(f"Error saat stitching panorama: {e}\n{traceback.format_exc()}", level='error')

def main(args=None):
    try:
        rclpy.init(args=args)
        node = PanoramaStitcher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt, shutting down panorama_stitcher node.")
            log_to_file("KeyboardInterrupt, shutting down panorama_stitcher node.", level='warn')
        except Exception as e:
            node.get_logger().error(f"Exception utama: {e}\n{traceback.format_exc()}")
            log_to_file(f"Exception utama: {e}\n{traceback.format_exc()}", level='error')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            node.get_logger().info("panorama_stitcher node shutdown complete.")
            log_to_file("panorama_stitcher node shutdown complete.")
    except Exception as e:
        print(f"[FATAL] Exception di main(): {e}\n{traceback.format_exc()}")
        log_to_file(f"[FATAL] Exception di main(): {e}\n{traceback.format_exc()}", level='error')
        sys.exit(99)

if __name__ == '__main__':
    main()