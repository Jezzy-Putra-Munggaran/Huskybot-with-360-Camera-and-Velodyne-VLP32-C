import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
from yolov12_msgs.msg import InferenceResult, Yolov12Inference
from std_msgs.msg import Header

class MultiCamClassificationNode(Node):
    def __init__(self):
        super().__init__('multicam_classification')
        self.declare_parameter('cam_count', 6)
        self.declare_parameter('model_path', "yolo12n-cls.engine")
        self.declare_parameter('camera_topics', [
            '/camera_front/image_raw',
            '/camera_right/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_rear/image_raw',
            '/camera_left/image_raw',
            '/camera_front_left/image_raw'
        ])
        self.cam_count = self.get_parameter('cam_count').value
        self.model_path = self.get_parameter('model_path').value
        self.camera_topics = self.get_parameter('camera_topics').value

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="classify")
        self.images = [None] * self.cam_count

        for i, topic in enumerate(self.camera_topics):
            self.create_subscription(
                Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10
            )
        self.timer = self.create_timer(0.2, self.display_images)
        self.publisher = self.create_publisher(Yolov12Inference, '/detection', 10)

    def image_callback(self, msg, idx):
        try:
            self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convert image: {e}")

    def display_images(self):
        if all(img is not None for img in self.images):
            try:
                annotated_images = []
                for idx, img in enumerate(self.images):
                    results = self.model(img, verbose=False)
                    annotated = img.copy()
                    if hasattr(results[0], "probs"):
                        class_id = int(np.argmax(results[0].probs))
                        conf = float(np.max(results[0].probs))
                        self.get_logger().info(f"[Classification] Camera {idx+1}: class={class_id}, conf={conf:.2f}")
                        cv2.putText(annotated, f"Class: {class_id} ({conf:.2f})", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    annotated_images.append(annotated)
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
                cv2.imshow("MultiCam YOLOv12 Classification", combined_image)
                cv2.waitKey(1)
                self.images = [None] * self.cam_count
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.images = [None] * self.cam_count
        else:
            available = sum(1 for img in self.images if img is not None)
            self.get_logger().info(f"Available camera feeds: {available}/{self.cam_count}")

    def publish_results(self, results, camera_name):
        msg = Yolov12Inference()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_name
        msg.camera_name = camera_name
        msg.frame_type = "raw"
        msg.task = "detect"
        msg.note = ""
        msg.yolov12_inference = []
        for box in results[0].boxes:
            det = InferenceResult()
            det.class_name = str(box.cls.item())
            det.confidence = float(box.conf.item())
            det.top = int(box.xyxy[0][1])
            det.left = int(box.xyxy[0][0])
            det.bottom = int(box.xyxy[0][3])
            det.right = int(box.xyxy[0][2])
            det.track_id = int(box.id.item()) if hasattr(box, "id") and box.id is not None else -1
            det.obb_angle = 0  # isi jika OBB
            det.mask_indices = []  # isi jika segmentation
            msg.yolov12_inference.append(det)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamClassificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()