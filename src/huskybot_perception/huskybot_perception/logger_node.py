import rclpy
from rclpy.node import Node
from yolov12_msgs.msg import Yolov12Inference
import csv
import os

class MultiTaskLogger(Node):
    def __init__(self):
        super().__init__('multitask_logger')
        self.log_path = os.path.expanduser("~/huskybot_multitask_log.csv")
        self.file = open(self.log_path, 'a', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'camera_name', 'task', 'class_name', 'confidence', 'top', 'left', 'bottom', 'right', 'track_id', 'obb_angle'])
        self.subs = []
        for topic in ['/detection', '/segmentation', '/obb', '/tracking']:
            self.subs.append(self.create_subscription(
                Yolov12Inference, topic, self.callback, 10))

    def callback(self, msg):
        for det in msg.yolov12_inference:
            self.writer.writerow([
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                msg.camera_name,
                msg.task,
                det.class_name,
                det.confidence,
                det.top,
                det.left,
                det.bottom,
                det.right,
                getattr(det, 'track_id', -1),
                getattr(det, 'obb_angle', 0)
            ])
        self.file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = MultiTaskLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()