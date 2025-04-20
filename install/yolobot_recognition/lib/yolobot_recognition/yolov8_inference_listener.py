#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Yolov8Inference

class Yolov8InferenceListener(Node):
    def __init__(self):
        super().__init__('yolov8_inference_listener')
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Kamera: {msg.camera_name}, Jumlah Deteksi: {len(msg.yolov8_inference)}'
        )
        for det in msg.yolov8_inference:
            self.get_logger().info(
                f'  Class: {det.class_name}, Box: ({det.top}, {det.left}, {det.bottom}, {det.right})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8InferenceListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()