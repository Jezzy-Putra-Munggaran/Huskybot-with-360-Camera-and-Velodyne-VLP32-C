import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CAMERA_CONFIG = [
    ('/camera_front/image_raw', 'csi://0'),
    ('/camera_front_left/image_raw', 'csi://1'),
    ('/camera_left/image_raw', 'csi://2'),
    ('/camera_rear/image_raw', 'csi://3'),
    ('/camera_rear_right/image_raw', 'csi://4'),
    ('/camera_right/image_raw', 'csi://5'),
]

class CameraPublisher(Node):
    def __init__(self, topic, device):
        super().__init__(f'camera_publisher_{topic.replace("/", "_")}')
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)
        self.timer = self.create_timer(0.05, self.publish_image)  # 20 FPS

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodes = []
    for topic, device in CAMERA_CONFIG:
        node = CameraPublisher(topic, device)
        nodes.append(node)
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()