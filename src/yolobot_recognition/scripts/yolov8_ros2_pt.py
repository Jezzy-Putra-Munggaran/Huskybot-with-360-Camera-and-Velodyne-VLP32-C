#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolo12n.pt')

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        # Daftar topic kamera dan label (disesuaikan dengan Xacro Husky 6 kamera)
        self.camera_topics = {
            'camera_front':        '/camera_front/image_raw',
            'camera_front_left':   '/camera_front_left/image_raw',
            'camera_left':         '/camera_left/image_raw',
            'camera_rear':         '/camera_rear/image_raw',
            'camera_rear_right':   '/camera_rear_right/image_raw',
            'camera_right':        '/camera_right/image_raw'
        }

        # Ganti nama variabel agar tidak bentrok dengan property Node
        self._my_subscriptions = []
        for cam_name, topic in self.camera_topics.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=cam_name: self.camera_callback(msg, cam),
                10
            )
            self._my_subscriptions.append(sub)

    def camera_callback(self, data, cam_name):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        yolov8_inference = Yolov8Inference()
        yolov8_inference.header.frame_id = cam_name
        yolov8_inference.header.stamp = self.get_clock().now().to_msg()
        # Jika ingin menambah info asal kamera, pastikan ada field di msg
        yolov8_inference.camera_name = cam_name

        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                inference_result.class_name = self.model.names[int(c)]
                inference_result.top = int(b[0])
                inference_result.left = int(b[1])
                inference_result.bottom = int(b[2])
                inference_result.right = int(b[3])
                yolov8_inference.yolov8_inference.append(inference_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        img_msg.header = data.header  # ikut header kamera asli

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(yolov8_inference)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
