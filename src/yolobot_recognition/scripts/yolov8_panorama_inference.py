#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov8_msgs.msg import Yolov8Inference, InferenceResult
from cv_bridge import CvBridge
from ultralytics import YOLO

class PanoramaDetection(Node):
    def __init__(self):
        super().__init__('panorama_detection')
        self.bridge = CvBridge()
        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolo11n.pt')
        self.pub = self.create_publisher(Yolov8Inference, '/panorama/yolov8_inference', 1)
        self.img_pub = self.create_publisher(Image, '/panorama/inference_result', 1)
        self.sub = self.create_subscription(
            Image,
            '/panorama/detection_input',  # Sudah sesuai, ini hasil stitching, bukan kamera fisik
            self.callback,
            10
        )

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)
        yolov8_inference = Yolov8Inference()
        yolov8_inference.header = msg.header
        yolov8_inference.camera_name = "panorama"
        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                inference_result = InferenceResult()
                inference_result.class_name = self.model.names[int(c)]
                inference_result.top = int(b[0])
                inference_result.left = int(b[1])
                inference_result.bottom = int(b[2])
                inference_result.right = int(b[3])
                yolov8_inference.yolov8_inference.append(inference_result)
        self.pub.publish(yolov8_inference)
        # Publish hasil visualisasi
        annotated = results[0].plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header = msg.header
        self.img_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PanoramaDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()