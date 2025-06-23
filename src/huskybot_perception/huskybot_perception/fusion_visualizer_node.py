#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference
import threading

class FusionVisualizerNode(Node):
    def __init__(self):
        super().__init__('fusion_visualizer_node')
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        
        # Declare parameters
        self.declare_parameter('show_bounding_box', True)
        self.declare_parameter('show_class', True)
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('show_distance', True)
        self.declare_parameter('show_coordinates', True)
        self.declare_parameter('text_scale', 0.7)
        self.declare_parameter('text_thickness', 2)
        self.declare_parameter('box_thickness', 2)
        self.declare_parameter('fusion_topic', '/fusion/objects3d')
        
        # Get parameters
        self.show_bbox = self.get_parameter('show_bounding_box').value
        self.show_class = self.get_parameter('show_class').value
        self.show_confidence = self.get_parameter('show_confidence').value
        self.show_distance = self.get_parameter('show_distance').value
        self.show_coordinates = self.get_parameter('show_coordinates').value
        self.text_scale = self.get_parameter('text_scale').value
        self.text_thickness = self.get_parameter('text_thickness').value
        self.box_thickness = self.get_parameter('box_thickness').value
        self.fusion_topic = self.get_parameter('fusion_topic').value
        
        # Subscribe to all camera topics
        self.camera_topics = [
            '/camera_front/image_raw',
            '/camera_front_left/image_raw',
            '/camera_left/image_raw',
            '/camera_rear/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_right/image_raw'
        ]
        
        self.camera_subs = {}
        self.latest_images = {}
        
        for topic in self.camera_topics:
            camera_name = topic.split('/')[1]  # Extract camera name from topic
            self.camera_subs[camera_name] = self.create_subscription(
                Image, 
                topic, 
                lambda msg, cam=camera_name: self.image_callback(msg, cam), 
                10
            )
            self.latest_images[camera_name] = None
        
        # Subscribe to fusion results
        self.fusion_sub = self.create_subscription(
            Yolov12Inference,
            self.fusion_topic,
            self.fusion_callback,
            10
        )
        
        # Publishers for annotated images
        self.annotated_pubs = {}
        for camera_name in self.camera_subs.keys():
            self.annotated_pubs[camera_name] = self.create_publisher(
                Image,
                f'/{camera_name}/annotated',
                10
            )
            
        self.get_logger().info('Fusion Visualizer Node initialized')
    
    def image_callback(self, msg, camera_name):
        with self.lock:
            try:
                self.latest_images[camera_name] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error: {e}')
    
    def fusion_callback(self, msg):
        with self.lock:
            camera_name = msg.camera_name
            if camera_name not in self.latest_images or self.latest_images[camera_name] is None:
                return
                
            # Get a copy of the image to annotate
            image = self.latest_images[camera_name].copy()
            
            # Draw detections with fusion information
            for det in msg.yolov12_inference:
                # Extract bounding box
                x1, y1, x2, y2 = int(det.left), int(det.top), int(det.right), int(det.bottom)
                
                # Draw bounding box if enabled
                if self.show_bbox:
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), self.box_thickness)
                
                # Prepare text to display
                text_parts = []
                
                if self.show_class:
                    text_parts.append(f"{det.class_name}")
                    
                if self.show_confidence:
                    text_parts.append(f"{det.confidence:.2f}")
                
                # Parse distance and coordinates from note if available
                if det.note and "Distance:" in det.note:
                    if self.show_distance:
                        distance_text = det.note.split(",")[0].strip()
                        text_parts.append(distance_text)
                        
                    if self.show_coordinates and "Coord:" in det.note:
                        coord_text = det.note.split("Coord:")[1].strip()
                        text_parts.append(f"Coord: {coord_text}")
                
                # Draw text
                text = " | ".join(text_parts)
                cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                           self.text_scale, (0, 255, 0), self.text_thickness)
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                annotated_msg.header = msg.header
                if camera_name in self.annotated_pubs:
                    self.annotated_pubs[camera_name].publish(annotated_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error when publishing: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()