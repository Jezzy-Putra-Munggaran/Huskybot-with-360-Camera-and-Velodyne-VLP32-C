#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov12_msgs.msg import Yolov12Inference

class Yolov12InferenceListener(Node):
    def __init__(self):
        super().__init__('yolov12_inference_listener')
        self.subscription = self.create_subscription(
            Yolov12Inference,
            '/Yolov12_Inference',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Kamera: {msg.camera_name}, Jumlah Deteksi: {len(msg.yolov12_inference)}'
        )
        for det in msg.yolov12_inference:
            self.get_logger().info(
                f'  Class: {det.class_name}, Box: ({det.top}, {det.left}, {det.bottom}, {det.right})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = Yolov12InferenceListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()