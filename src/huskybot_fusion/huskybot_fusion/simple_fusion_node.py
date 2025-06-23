#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from std_msgs.msg import Header
import numpy as np
import math
import threading
import message_filters
import time

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion')
        
        # Declare parameters
        self.declare_parameter('use_calibration', False)
        self.declare_parameter('max_laser_distance', 100.0)
        self.declare_parameter('confidence_threshold', 0.25)
        
        # Get parameters
        self.use_calibration = self.get_parameter('use_calibration').value
        self.max_laser_distance = self.get_parameter('max_laser_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Create publisher for fused results
        self.fusion_pub = self.create_publisher(
            Yolov12Inference, 
            '/fusion/objects3d', 
            10
        )
        
        # Variables to store latest data
        self.latest_detection = None
        self.latest_laserscan = None
        self.latest_pointcloud = None
        self.lock = threading.Lock()
        
        # Subscribe to detection, laserscan, and pointcloud
        self.detection_sub = self.create_subscription(
            Yolov12Inference,
            '/detection',
            self.detection_callback,
            10
        )
        
        self.laserscan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10
        )
        
        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)
        
        self.get_logger().info("Simple fusion node initialized")
    
    def detection_callback(self, msg):
        with self.lock:
            self.latest_detection = msg
    
    def laserscan_callback(self, msg):
        with self.lock:
            self.latest_laserscan = msg
    
    def pointcloud_callback(self, msg):
        with self.lock:
            self.latest_pointcloud = msg
    
    def get_distance_from_laserscan(self, bbox_center_x, camera_index):
        """
        Get distance estimation from LaserScan data.
        Using a simplified approach without calibration.
        """
        if not self.latest_laserscan:
            return None
            
        # Normalize bbox_center_x to -1.0 to 1.0 (from image coords)
        # Assuming image width is 640
        normalized_x = (bbox_center_x - 320) / 320.0
        
        # Convert normalized_x to angle based on camera FOV and position
        # This is a simplified approach!
        camera_angles = [0, 60, 120, 180, 240, 300]  # In degrees, for hexagonal arrangement
        camera_fov = 90  # Camera FOV in degrees
        
        # Get base angle for this camera
        base_angle = camera_angles[camera_index % 6]
        
        # Calculate angle within camera's FOV
        angle_within_fov = normalized_x * (camera_fov / 2.0)
        
        # Total angle in global frame
        total_angle = base_angle + angle_within_fov
        
        # Convert to radians and normalize to LaserScan range (-π to π)
        angle_rad = math.radians(total_angle)
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        
        # Find closest angle in LaserScan
        ranges = self.latest_laserscan.ranges
        angle_min = self.latest_laserscan.angle_min
        angle_increment = self.latest_laserscan.angle_increment
        
        closest_index = int(round((angle_rad - angle_min) / angle_increment))
        
        # Bounds checking
        if closest_index < 0 or closest_index >= len(ranges):
            return None
            
        distance = ranges[closest_index]
        
        # Check if distance is valid
        if not math.isfinite(distance) or distance > self.max_laser_distance:
            return None
            
        return distance
    
    def get_coordinates_from_pointcloud(self, distance, bbox_center_x, camera_index):
        """
        Get 3D coordinates from PointCloud data.
        Using a simplified approach without calibration.
        """
        if not distance or not self.latest_pointcloud:
            return None, None, None
            
        # Simplified approach: use distance and angle to estimate 3D position
        # This assumes robot center as origin
        camera_angles = [0, 60, 120, 180, 240, 300]  # In degrees
        camera_fov = 90  # Camera FOV in degrees
        
        # Normalize bbox_center_x to -1.0 to 1.0
        normalized_x = (bbox_center_x - 320) / 320.0
        
        # Get base angle for this camera
        base_angle = camera_angles[camera_index % 6]
        
        # Calculate angle within camera's FOV
        angle_within_fov = normalized_x * (camera_fov / 2.0)
        
        # Total angle in global frame
        total_angle = base_angle + angle_within_fov
        
        # Convert to radians
        angle_rad = math.radians(total_angle)
        
        # Calculate X, Y (in robot frame)
        x = distance * math.cos(angle_rad)  # Forward
        y = distance * math.sin(angle_rad)  # Left
        z = 0.0  # Assume ground level
        
        return x, y, z
    
    def fusion_callback(self):
        with self.lock:
            if not self.latest_detection or not self.latest_laserscan or not self.latest_pointcloud:
                return
                
            # Copy detection message
            fused_msg = Yolov12Inference()
            fused_msg.header = Header()
            fused_msg.header.stamp = self.get_clock().now().to_msg()
            fused_msg.header.frame_id = "map"  # Global frame
            fused_msg.frame_type = "fusion"
            fused_msg.task = self.latest_detection.task
            fused_msg.note = "Fusion: detection + distance + coordinates"
            
            camera_name = self.latest_detection.camera_name
            camera_index = -1
            
            # Extract camera index from name (assuming format "Camera_X")
            if "_" in camera_name:
                try:
                    camera_index = int(camera_name.split("_")[1]) - 1  # 0-based index
                except ValueError:
                    camera_index = 0
            
            # Process each detection
            for det in self.latest_detection.yolov12_inference:
                if det.confidence < self.confidence_threshold:
                    continue
                    
                # Calculate center of bounding box
                center_x = (det.left + det.right) / 2
                center_y = (det.top + det.bottom) / 2
                
                # Get distance from LaserScan
                distance = self.get_distance_from_laserscan(center_x, camera_index)
                
                # Get 3D coordinates from PointCloud
                x, y, z = self.get_coordinates_from_pointcloud(distance, center_x, camera_index)
                
                # Create fused detection result
                fused_det = InferenceResult()
                fused_det.class_name = det.class_name
                fused_det.confidence = det.confidence
                fused_det.top = det.top
                fused_det.left = det.left
                fused_det.bottom = det.bottom
                fused_det.right = det.right
                fused_det.track_id = -1  # Not using tracking
                
                # Add distance and coordinates to note field
                if distance and x is not None and y is not None:
                    fused_det.note = f"Distance: {distance:.2f}m, Coord: ({x:.2f}, {y:.2f}, {z:.2f})"
                else:
                    fused_det.note = "No distance/coordinates available"
                
                # Add mask if available
                if hasattr(det, 'mask_indices') and det.mask_indices:
                    fused_det.mask_indices = det.mask_indices
                
                fused_msg.yolov12_inference.append(fused_det)
            
            # Publish fused results
            if fused_msg.yolov12_inference:
                self.fusion_pub.publish(fused_msg)
                self.get_logger().info(f"Published fusion results with {len(fused_msg.yolov12_inference)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error in fusion node: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()