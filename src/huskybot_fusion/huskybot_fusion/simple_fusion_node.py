#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math  # FIXED: Import math yang hilang
import time
import threading
import os
import sys
import traceback

# FIXED: Robust point cloud import with proper fallback
try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    try:
        from sensor_msgs import point_cloud2 as pc2
        PC2_AVAILABLE = True
    except ImportError:
        print("[WARNING] Could not import point_cloud2 utilities", file=sys.stderr)
        PC2_AVAILABLE = False
        pc2 = None

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion_node')
        
        # Initialize locks and data storage
        self.lock = threading.Lock()
        self.latest_scan = None
        self.latest_detections = {}
        self.latest_pointcloud = None
        
        # Setup parameters
        self._setup_parameters()
        
        # Create subscriptions
        self._create_subscriptions()
        
        # Create publishers
        self._create_publishers()
        
        # Create timers
        self._create_timers()
        
        # Statistics
        self.stats = {
            'successful_fusions': 0,
            'failed_fusions': 0,
            'total_detections': 0
        }
        
        self.get_logger().info("Simple Fusion Node initialized successfully")

    def _setup_parameters(self):
        """Setup parameters with Jetson-specific optimizations"""
        # Declare parameters
        self.declare_parameter('max_laser_distance', 50.0)
        self.declare_parameter('min_laser_distance', 0.5)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('search_radius', 0.3)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        
        # Get parameters
        self.max_laser_distance = self.get_parameter('max_laser_distance').value
        self.min_laser_distance = self.get_parameter('min_laser_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.search_radius = self.get_parameter('search_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

    def _create_subscriptions(self):
        """Create subscriptions for sensor data"""
        # Subscribe to LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Subscribe to PointCloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        
        # Subscribe to detection results from all cameras
        camera_topics = [
            '/camera_front/detection',
            '/camera_front_left/detection',
            '/camera_left/detection',
            '/camera_rear/detection',
            '/camera_rear_right/detection',
            '/camera_right/detection'
        ]
        
        # Create detection subscription with fallback to main detection topic
        self.detection_sub = self.create_subscription(
            Yolov12Inference, '/detection', self.detection_callback, 10)

    def _create_publishers(self):
        """Create publishers for fusion results"""
        self.result_pub = self.create_publisher(
            Yolov12Inference, '/detection_with_distance', 10)
        
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fusion_markers', 10)

    def _create_timers(self):
        """Create processing timers"""
        # Main fusion processing timer
        self.fusion_timer = self.create_timer(
            1.0 / self.publish_rate, self.process_fusion)
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.log_statistics)

    def scan_callback(self, msg):
        """Callback for LaserScan data"""
        try:
            with self.lock:
                self.latest_scan = msg
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {e}")

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 data"""
        try:
            with self.lock:
                self.latest_pointcloud = msg
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud callback: {e}")

    def detection_callback(self, msg):
        """Callback for detection results"""
        try:
            with self.lock:
                self.latest_detections[msg.camera_name] = msg
                self.stats['total_detections'] += len(msg.yolov12_inference)
        except Exception as e:
            self.get_logger().error(f"Error in detection callback: {e}")

    def get_distance_from_laserscan(self, camera_name, obj_center_ratio):
        """Get distance from LaserScan dengan mapping yang benar"""
        try:
            if not self.latest_scan or not self.latest_scan.ranges:
                return None
                
            scan = self.latest_scan
            
            # Mapping kamera ke sudut LiDAR (sesuai hardware real)
            camera_angles = {
                'camera_front': 180,        # BELAKANG
                'camera_front_left': 225,   # KIRI BELAKANG
                'camera_left': 270,         # KIRI DEPAN
                'camera_rear': 0,           # DEPAN
                'camera_rear_right': 315,   # KANAN DEPAN
                'camera_right': 45,         # KANAN BELAKANG
            }
            
            if camera_name not in camera_angles:
                return None
                
            # Calculate target angle
            base_angle = camera_angles[camera_name]
            camera_fov = 60  # Assume 60° FOV per camera
            angle_offset = (obj_center_ratio - 0.5) * camera_fov
            target_angle_deg = base_angle + angle_offset
            
            # Normalize angle
            if target_angle_deg > 180:
                target_angle_deg -= 360
            elif target_angle_deg < -180:
                target_angle_deg += 360
                
            target_angle_rad = math.radians(target_angle_deg)
            
            # Calculate ray index
            if abs(scan.angle_increment) < 1e-6:
                return None
                
            ray_idx = int((target_angle_rad - scan.angle_min) / scan.angle_increment)
            
            # Check multiple rays around target for better reliability
            search_range = 5
            for offset in range(-search_range, search_range + 1):
                check_idx = ray_idx + offset
                if 0 <= check_idx < len(scan.ranges):
                    distance = scan.ranges[check_idx]
                    if (distance > 0 and 
                        self.min_laser_distance <= distance <= self.max_laser_distance and 
                        math.isfinite(distance)):
                        return distance
            
            return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting distance for {camera_name}: {e}")
            return None

    def get_coordinates_from_pointcloud(self, camera_name, obj_center_ratio, distance):
        """Calculate 3D coordinates dengan mapping yang benar"""
        try:
            if not distance:
                return None
                
            # Use same mapping as distance calculation
            camera_angles = {
                'camera_front': 180,
                'camera_front_left': 225,
                'camera_left': 270,
                'camera_rear': 0,
                'camera_rear_right': 315,
                'camera_right': 45,
            }
            
            if camera_name not in camera_angles:
                return None
                
            base_angle = camera_angles[camera_name]
            camera_fov = 60
            angle_offset = (obj_center_ratio - 0.5) * camera_fov
            target_angle_deg = base_angle + angle_offset
            
            # Normalize angle
            if target_angle_deg > 180:
                target_angle_deg -= 360
            elif target_angle_deg < -180:
                target_angle_deg += 360
                
            target_angle_rad = math.radians(target_angle_deg)
            
            # Calculate 3D coordinates
            x = distance * math.cos(target_angle_rad)
            y = distance * math.sin(target_angle_rad)
            z = 0.0  # Ground level assumption
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f"Error calculating coordinates: {e}")
            return None

    def process_fusion(self):
        """Main fusion processing"""
        try:
            with self.lock:
                if not self.latest_scan or not self.latest_detections:
                    return
                
                output_messages = []
                
                for camera_name, detections in self.latest_detections.items():
                    enhanced_msg = Yolov12Inference()
                    enhanced_msg.header = detections.header
                    enhanced_msg.camera_name = camera_name
                    enhanced_msg.frame_type = "fused"
                    enhanced_msg.task = "fusion"
                    
                    for det in detections.yolov12_inference:
                        if det.confidence < self.confidence_threshold:
                            continue
                        
                        # Calculate object center for LiDAR mapping
                        bbox_center_x = (det.left + det.right) / 2.0
                        obj_center_ratio = bbox_center_x / self.image_width
                        
                        # Get distance and coordinates
                        distance = self.get_distance_from_laserscan(camera_name, obj_center_ratio)
                        coords = None
                        
                        if distance:
                            coords = self.get_coordinates_from_pointcloud(camera_name, obj_center_ratio, distance)
                            self.stats['successful_fusions'] += 1
                        else:
                            self.stats['failed_fusions'] += 1
                        
                        # Create enhanced result
                        result = InferenceResult()
                        result.class_name = det.class_name
                        result.confidence = det.confidence
                        result.left = det.left
                        result.top = det.top
                        result.right = det.right
                        result.bottom = det.bottom
                        result.track_id = -1
                        result.obb_angle = -1.0
                        result.mask_indices = []
                        
                        # Add distance and coordinate info
                        if distance and coords:
                            # Format target output: Class=..., Confidence=..., Distance: ...m, Coordinate: (..., ..., ...)
                            if hasattr(result, 'note'):
                                result.note = f"Distance={distance:.2f}m, Coordinate=({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            
                            self.get_logger().info(
                                f"🎯 {camera_name}: Class={det.class_name}, "
                                f"Confidence={det.confidence:.2f}, "
                                f"Distance: {distance:.2f} m, "
                                f"Coordinate: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            )
                        
                        enhanced_msg.yolov12_inference.append(result)
                    
                    if enhanced_msg.yolov12_inference:
                        output_messages.append(enhanced_msg)
                
                # Publish results
                for msg in output_messages:
                    self.result_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in process_fusion: {str(e)}")

    def log_statistics(self):
        """Log performance statistics"""
        try:
            self.get_logger().info(
                f"📊 Fusion Stats: Success={self.stats['successful_fusions']}, "
                f"Failed={self.stats['failed_fusions']}, "
                f"Total={self.stats['total_detections']}"
            )
        except Exception as e:
            self.get_logger().error(f"Error logging statistics: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimpleFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
    except Exception as e:
        print(f"Fatal error: {e}")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()