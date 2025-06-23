#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolov12_msgs.msg import Yolov12Inference
import cv2
import numpy as np
import threading
import message_filters
from std_msgs.msg import Header

class FusionVisualizerNode(Node):
    def __init__(self):
        super().__init__('fusion_visualizer')
        
        # Declare parameters
        self.declare_parameter('show_bounding_box', True)
        self.declare_parameter('show_distance', True)
        self.declare_parameter('show_coordinates', True)
        self.declare_parameter('show_class', True)
        self.declare_parameter('show_confidence', True)
        
        # Get parameters
        self.show_bounding_box = self.get_parameter('show_bounding_box').value
        self.show_distance = self.get_parameter('show_distance').value
        self.show_coordinates = self.get_parameter('show_coordinates').value
        self.show_class = self.get_parameter('show_class').value
        self.show_confidence = self.get_parameter('show_confidence').value
        
        # Camera topics
        self.camera_topics = [
            '/camera_front/image_raw',
            '/camera_right/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_rear/image_raw',
            '/camera_left/image_raw',
            '/camera_front_left/image_raw'
        ]
        
        # Image bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.latest_images = [None] * len(self.camera_topics)
        self.latest_fusion = None
        self.lock = threading.Lock()
        
        # Subscribe to camera topics
        self.camera_subs = []
        for idx, topic in enumerate(self.camera_topics):
            sub = self.create_subscription(
                Image, 
                topic, 
                lambda msg, idx=idx: self.image_callback(msg, idx), 
                10
            )
            self.camera_subs.append(sub)
        
        # Subscribe to fusion results
        self.fusion_sub = self.create_subscription(
            Yolov12Inference,
            '/fusion/objects3d',
            self.fusion_callback,
            10
        )
        
        # Create publishers for annotated images
        self.annotated_pubs = []
        for idx, topic in enumerate(self.camera_topics):
            pub = self.create_publisher(
                Image,
                f'/visualizer/camera_{idx}/annotated',
                10
            )
            self.annotated_pubs.append(pub)
        
        # Timer for visualization
        self.viz_timer = self.create_timer(0.1, self.visualization_callback)
        
        self.get_logger().info("Fusion visualizer node initialized")
    
    def image_callback(self, msg, camera_idx):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_images[camera_idx] = (cv_img, msg.header)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
    
    def fusion_callback(self, msg):
        with self.lock:
            self.latest_fusion = msg
    
    def visualization_callback(self):
        with self.lock:
            if not self.latest_fusion:
                return
                
            # Process each camera image if available
            for idx, item in enumerate(self.latest_images):
                if not item:
                    continue
                    
                cv_img, header = item
                
                # Create a copy for annotation
                annotated_img = cv_img.copy()
                
                # Find detections for this camera
                camera_name = f"Camera_{idx+1}"
                
                # Process each detection in fusion results
                for det in self.latest_fusion.yolov12_inference:
                    # Draw bounding box
                    if self.show_bounding_box:
                        cv2.rectangle(
                            annotated_img,
                            (det.left, det.top),
                            (det.right, det.bottom),
                            (0, 255, 0),
                            2
                        )
                    
                    # Prepare text to display
                    text_items = []
                    
                    if self.show_class:
                        text_items.append(f"Class: {det.class_name}")
                    
                    if self.show_confidence:
                        text_items.append(f"Conf: {det.confidence:.2f}")
                    
                    # Parse distance and coordinates from note field
                    if self.show_distance or self.show_coordinates:
                        if hasattr(det, 'note') and det.note:
                            if "Distance:" in det.note and self.show_distance:
                                distance_part = det.note.split("Distance:")[1].split(",")[0].strip()
                                text_items.append(f"Dist: {distance_part}")
                            
                            if "Coord:" in det.note and self.show_coordinates:
                                coord_part = det.note.split("Coord:")[1].strip()
                                text_items.append(f"Coord: {coord_part}")
                    
                    # Draw text
                    y_offset = det.top - 10
                    for i, text in enumerate(text_items):
                        y = y_offset - (i * 20)
                        if y < 15:  # Ensure text is visible
                            y = 15 + (i * 20)
                        
                        cv2.putText(
                            annotated_img,
                            text,
                            (det.left, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )
                
                # Publish annotated image
                try:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
                    annotated_msg.header = header
                    self.annotated_pubs[idx].publish(annotated_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f"Error converting annotated image: {str(e)}")
                
                # Also display in OpenCV window (if not headless)
                try:
                    cv2.imshow(f"Camera {idx+1} with Fusion", annotated_img)
                    cv2.waitKey(1)
                except Exception as e:
                    pass  # Silently fail if display is not available

def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error in visualizer node: {str(e)}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()