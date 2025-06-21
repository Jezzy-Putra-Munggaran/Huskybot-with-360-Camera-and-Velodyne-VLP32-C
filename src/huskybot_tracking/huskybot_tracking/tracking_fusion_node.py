import rclpy
from rclpy.node import Node
from yolov12_msgs.msg import Yolov12Inference
from std_msgs.msg import Header

class TrackingFusionNode(Node):
    def __init__(self):
        super().__init__('tracking_fusion')
        self.subs = []
        self.results = {}
        self.publisher = self.create_publisher(Yolov12Inference, '/tracking', 10)
        topics = ['/detection', '/segmentation', '/obb']
        for topic in topics:
            self.subs.append(self.create_subscription(
                Yolov12Inference, topic, self.callback, 10))
        self.timer = self.create_timer(0.1, self.publish_tracking)

    def callback(self, msg):
        self.results[msg.camera_name + "_" + msg.task] = msg

    def publish_tracking(self):
        msg = Yolov12Inference()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tracking"
        msg.camera_name = "tracking"
        msg.frame_type = "fusion"
        msg.task = "track"
        msg.note = "Gabungan hasil multi-task"
        msg.yolov12_inference = []
        for key in self.results:
            msg.yolov12_inference.extend(self.results[key].yolov12_inference)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()