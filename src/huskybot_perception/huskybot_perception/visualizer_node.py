import rclpy
from rclpy.node import Node
from yolov12_msgs.msg import Yolov12Inference
import cv2
import numpy as np

class MultiTaskVisualizer(Node):
    def __init__(self):
        super().__init__('multitask_visualizer')
        self.images = {}
        self.subs = []
        for topic in ['/detection', '/segmentation', '/obb', '/tracking']:
            self.subs.append(self.create_subscription(
                Yolov12Inference, topic, self.callback, 10))
        self.timer = self.create_timer(0.2, self.display)

    def callback(self, msg):
        self.images[msg.camera_name + "_" + msg.task] = msg

    def display(self):
        img = np.zeros((400, 800, 3), dtype=np.uint8)
        y = 50
        for key, msg in self.images.items():
            text = f"{key}: {len(msg.yolov12_inference)} deteksi"
            cv2.putText(img, text, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            y += 40
        cv2.imshow("MultiTask Visualizer", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiTaskVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()