#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
import time
import threading
from sensor_msgs.msg import LaserScan, PointCloud2
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion_node')
        
        # Declare parameters
        self.declare_parameter('use_calibration', False)
        self.declare_parameter('max_laser_distance', 100.0)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('detection_topics', [
            '/detection',  # Will contain results from detection/segmentation
        ])
        self.declare_parameter('laserscan_topic', '/scan')
        self.declare_parameter('pointcloud_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/fusion/objects3d')
        
        # Get parameters
        self.use_calibration = self.get_parameter('use_calibration').value
        self.max_laser_distance = self.get_parameter('max_laser_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_topics = self.get_parameter('detection_topics').value
        self.laserscan_topic = self.get_parameter('laserscan_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Initialize data holders
        self.latest_scan = None
        self.latest_cloud = None
        self.latest_detections = {}
        self.lock = threading.RLock()
        
        # Camera field of view angles (approx) - Updated for hexagonal arrangement
        self.camera_fov = {
            'camera_front': (-30, 30),
            'camera_front_left': (-90, -30),
            'camera_left': (-150, -90),
            'camera_rear': (150, -150),
            'camera_rear_right': (90, 150),
            'camera_right': (30, 90)
        }
        
        # Subscribe to LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.laserscan_topic,
            self.laserscan_callback,
            10
        )
        
        # Subscribe to PointCloud2
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        # Subscribe to detection topics from all cameras
        self.detection_subs = []
        for topic in self.detection_topics:
            self.detection_subs.append(
                self.create_subscription(
                    Yolov12Inference,
                    topic,
                    self.detection_callback,
                    10
                )
            )
        
        # Publishers
        self.result_pub = self.create_publisher(Yolov12Inference, self.output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fusion/markers', 10)
        
        # Timer for fusion processing
        self.timer = self.create_timer(0.1, self.process_fusion)
        self.get_logger().info('Simple Fusion Node initialized')
    
    def laserscan_callback(self, msg):
        with self.lock:
            self.latest_scan = msg
    
    def pointcloud_callback(self, msg):
        with self.lock:
            self.latest_cloud = msg
    
    def detection_callback(self, msg):
        if not msg.camera_name:
            return
        
        with self.lock:
            self.latest_detections[msg.camera_name] = msg
    
    def get_distance_from_laserscan(self, camera_name, obj_center_angle):
        """Get distance from LaserScan based on camera name and object angle"""
        if self.latest_scan is None:
            return None
        
        scan = self.latest_scan
        
        # Convert object's position to global angle
        # For simplicity, we just map the center of the bounding box to an angle
        # within the camera's FOV, then convert to the LiDAR's coordinate system
        
        # Get camera FOV
        if camera_name not in self.camera_fov:
            return None
            
        cam_min_angle, cam_max_angle = self.camera_fov[camera_name]
        
        # Map obj_center_angle (0-1 range) to camera FOV
        global_angle = cam_min_angle + obj_center_angle * (cam_max_angle - cam_min_angle)
        
        # Convert to LiDAR angle (radians)
        lidar_angle = math.radians(global_angle)
        
        # Normalize to range [0, 2π) or [-π, π) based on LaserScan
        while lidar_angle < scan.angle_min:
            lidar_angle += 2 * math.pi
        while lidar_angle > scan.angle_max:
            lidar_angle -= 2 * math.pi
        
        # Find closest scan ray
        ray_idx = int((lidar_angle - scan.angle_min) / scan.angle_increment)
        
        # Ensure index is in range
        if ray_idx < 0 or ray_idx >= len(scan.ranges):
            return None
            
        # Get distance
        distance = scan.ranges[ray_idx]
        
        # Check if distance is valid
        if distance < scan.range_min or distance > scan.range_max or distance > self.max_laser_distance:
            return None
            
        return distance
    
    def get_coordinates_from_pointcloud(self, camera_name, obj_center_angle, distance):
        """Get 3D coordinates from PointCloud based on camera, angle, and distance"""
        if self.latest_cloud is None or distance is None:
            return None
            
        # Convert the angle to global coordinates (same as in get_distance_from_laserscan)
        cam_min_angle, cam_max_angle = self.camera_fov[camera_name]
        global_angle = cam_min_angle + obj_center_angle * (cam_max_angle - cam_min_angle)
        lidar_angle = math.radians(global_angle)
        
        # Simple trigonometry to get XYZ
        # For a more accurate approach, you'd need proper camera-LiDAR calibration
        x = distance * math.cos(lidar_angle)
        y = distance * math.sin(lidar_angle)
        z = 0.0  # Assume object is on ground level for simplicity
        
        # For better Z accuracy, you could search nearby points in the point cloud
        # within a radius around (x,y) and get the average or maximum Z
        
        # For PointCloud2 processing (more accurate Z)
        points = list(pc2.read_points(self.latest_cloud, field_names=("x", "y", "z"),
                                     skip_nans=True))
        
        # Find points close to our (x,y) location
        nearby_points = []
        search_radius = 0.5  # meters
        
        for p in points:
            dx = p[0] - x
            dy = p[1] - y
            dist_2d = math.sqrt(dx*dx + dy*dy)
            if dist_2d < search_radius:
                nearby_points.append(p)
        
        # If we found nearby points, update Z
        if nearby_points:
            # Use the average Z of nearby points (could also use max)
            z = sum(p[2] for p in nearby_points) / len(nearby_points)
        
        return (x, y, z)
    
    def process_fusion(self):
        """Process fusion of detections with LiDAR data"""
        with self.lock:
            if not self.latest_detections:
                return
                
            if self.latest_scan is None or self.latest_cloud is None:
                return
            
            marker_array = MarkerArray()
            marker_id = 0
            
            # Process each camera's detections
            for camera_name, detections in self.latest_detections.items():
                output_msg = Yolov12Inference()
                output_msg.header = detections.header
                output_msg.camera_name = camera_name
                output_msg.frame_type = "fused"
                output_msg.task = "fusion"
                output_msg.note = "simple fusion without calibration"
                
                # Process each detection
                for det in detections.yolov12_inference:
                    if det.confidence < self.confidence_threshold:
                        continue
                    
                    # Calculate object center angle (0-1 range)
                    # For simplicity, we just use the horizontal center of bbox
                    obj_center_ratio = (det.left + det.right) / 2.0 / 640.0  # Assuming image width is 640
                    
                    # Get distance from LaserScan
                    distance = self.get_distance_from_laserscan(camera_name, obj_center_ratio)
                    
                    # Get coordinates from PointCloud
                    coords = self.get_coordinates_from_pointcloud(camera_name, obj_center_ratio, distance)
                    
                    # Create enriched result
                    result = InferenceResult()
                    result.class_name = det.class_name
                    result.confidence = det.confidence
                    result.top = det.top
                    result.left = det.left
                    result.bottom = det.bottom
                    result.right = det.right
                    
                    # Add distance and coordinates if available
                    if distance is not None:
                        result.note = f"Distance: {distance:.2f}m"
                        
                        if coords is not None:
                            result.note += f", Coord: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            
                            # Create visualization marker
                            marker = Marker()
                            marker.header = detections.header
                            marker.ns = "object_markers"
                            marker.id = marker_id
                            marker_id += 1
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            marker.pose.position.x = coords[0]
                            marker.pose.position.y = coords[1]
                            marker.pose.position.z = coords[2]
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.5
                            marker.scale.y = 0.5
                            marker.scale.z = 0.5
                            marker.color.r = 1.0 if "person" in det.class_name else 0.0
                            marker.color.g = 0.0 if "person" in det.class_name else 1.0
                            marker.color.b = 0.0
                            marker.color.a = 0.7
                            marker.lifetime.sec = 1
                            marker_array.markers.append(marker)
                    
                    output_msg.yolov12_inference.append(result)
                
                # Publish results
                if output_msg.yolov12_inference:
                    self.result_pub.publish(output_msg)
            
            # Publish markers for visualization
            if marker_array.markers:
                self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()