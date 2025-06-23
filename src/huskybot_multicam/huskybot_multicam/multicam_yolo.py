import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2

class MultiCamYoloNode(Node):
    def __init__(self):
        super().__init__('multicam_yolo')
        # Parameterisasi sesuai lictic
        self.declare_parameter('cam_count', 6)
        self.declare_parameter('segmentation_model', "yolo12n.engine")
        self.declare_parameter('camera_topics', [
            '/camera_front/image_raw',
            '/camera_right/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_rear/image_raw',
            '/camera_left/image_raw',
            '/camera_front_left/image_raw'
        ])

        self.cam_count = self.get_parameter('cam_count').value
        self.segmentation_model = self.get_parameter('segmentation_model').value
        self.camera_topics = self.get_parameter('camera_topics').value

        self.bridge = CvBridge()
        self.model = YOLO(self.segmentation_model)
        self.images = [None] * self.cam_count

        # Subscribe ke semua kamera
        for i, topic in enumerate(self.camera_topics):
            self.create_subscription(
                Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10
            )

        self.timer = self.create_timer(0.2, self.display_images)

    def image_callback(self, msg, idx):
        try:
            self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convert image: {e}")

    def display_images(self):
        # Tampilkan 6 kamera secara horizontal, deteksi YOLOv12 per kamera
        if all(img is not None for img in self.images):
            try:
                annotated_images = []
                for idx, img in enumerate(self.images):
                    results = self.model(img, verbose=False)
                    annotated = results[0].plot()
                    annotated_images.append(annotated)
                    # Log hasil deteksi ke terminal
                    for box in results[0].boxes:
                        class_id = int(box.cls.item())
                        conf = float(box.conf.item())
                        self.get_logger().info(f"Camera {idx+1}: class={class_id}, conf={conf:.2f}")
                # Resize & concat horizontal
                target_height = 240
                resized_images = []
                for image in annotated_images:
                    h, w = image.shape[:2]
                    scale = target_height / h if h > target_height else 1
                    new_w = int(w * scale)
                    resized_image = cv2.resize(image, (new_w, target_height))
                    resized_images.append(resized_image)
                border_thickness = 5
                bordered_images = []
                for idx, img in enumerate(resized_images):
                    bordered_images.append(img)
                    if idx < len(resized_images) - 1:
                        border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                        bordered_images.append(border)
                combined_image = cv2.hconcat(bordered_images)
                cv2.imshow("MultiCam YOLOv12", combined_image)
                cv2.waitKey(1)
                self.images = [None] * self.cam_count
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.images = [None] * self.cam_count
        else:
            available = sum(1 for img in self.images if img is not None)
            self.get_logger().info(f"Available camera feeds: {available}/{self.cam_count}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()