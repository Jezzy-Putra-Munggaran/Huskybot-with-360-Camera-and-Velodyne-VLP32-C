#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import time
import threading
import os
import sys
import traceback

# PERBAIKAN: Robust point cloud import with multiple fallbacks
def import_point_cloud2():
    """Import point cloud utilities with comprehensive fallback for Jetson"""
    
    # Method 1: Try sensor_msgs_py (ROS2 Humble standard)
    try:
        from sensor_msgs_py import point_cloud2 as pc2
        print("INFO: Using sensor_msgs_py for point cloud parsing")
        return pc2
    except ImportError:
        pass
    
    # Method 2: Try direct sensor_msgs import
    try:
        import sensor_msgs.point_cloud2 as pc2
        print("INFO: Using sensor_msgs.point_cloud2 for point cloud parsing")
        return pc2
    except ImportError:
        pass
    
    # Method 3: Try alternative sensor_msgs import
    try:
        from sensor_msgs import point_cloud2 as pc2
        print("INFO: Using alternative sensor_msgs import for point cloud parsing")
        return pc2
    except ImportError:
        pass
    
    # Method 4: Create dummy implementation
    print("WARNING: Could not import point_cloud2, using dummy implementation")
    
    class DummyPC2:
        @staticmethod
        def read_points(cloud, field_names=None, skip_nans=True):
            """Dummy implementation for compatibility"""
            print("WARNING: Using dummy point cloud reader - no actual point parsing")
            return []
        
        @staticmethod 
        def create_cloud_xyz32(header, points):
            """Dummy implementation for cloud creation"""
            print("WARNING: Using dummy point cloud creator")
            return PointCloud2()
    
    return DummyPC2()

# Initialize point cloud utilities with fallback
pc2 = import_point_cloud2()

class SimpleFusionNode(Node):
    def __init__(self):
        """Initialize the SimpleFusionNode with better error handling for Jetson"""
        try:
            super().__init__('simple_fusion_node')
            
            # Detect if running on Jetson
            self.is_jetson = self._detect_jetson()
            if self.is_jetson:
                self.get_logger().info("Running on Jetson AGX Orin - enabling ARM64 optimizations")
            
            # Setup parameters with Jetson-specific defaults
            self._setup_parameters()
            
            # Initialize data structures
            self.latest_scan = None
            self.latest_cloud = None
            self.latest_detections = {}
            self.lock = threading.RLock()
            
            # Statistics
            self.stats = {
                'processed_detections': 0,
                'successful_fusions': 0,
                'failed_fusions': 0,
                'start_time': time.time()
            }
            
            # Camera FOV mapping (hexagonal setup)
            self.camera_fov = {
                'camera_front': (-30, 30),
                'camera_front_left': (-90, -30),
                'camera_left': (-150, -90),
                'camera_rear': (150, -150),
                'camera_rear_right': (90, 150),
                'camera_right': (30, 90)
            }
            
            # Create subscribers and publishers
            self._create_subscriptions()
            self._create_publishers()
            self._create_timers()
            
            self.get_logger().info('SimpleFusionNode initialized successfully on Jetson AGX Orin')
            
        except Exception as e:
            self.get_logger().error(f"Error during initialization: {str(e)}\n{traceback.format_exc()}")
            raise
    
    def _detect_jetson(self):
        """Detect if running on Jetson platform"""
        try:
            import platform
            return platform.machine() in ['aarch64', 'arm64']
        except:
            return False
    
    def _setup_parameters(self):
        """Setup parameters with Jetson-specific optimizations"""
        # Declare parameters
        self.declare_parameter('max_laser_distance', 50.0)
        self.declare_parameter('min_laser_distance', 0.5)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('search_radius', 0.3)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('image_width', 1920)  # PERBAIKAN: Sesuai dengan resolusi Arducam
        self.declare_parameter('image_height', 1080)  # PERBAIKAN: Sesuai dengan resolusi Arducam
        
        # Get parameters
        self.max_laser_distance = self.get_parameter('max_laser_distance').value
        self.min_laser_distance = self.get_parameter('min_laser_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.search_radius = self.get_parameter('search_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        self.get_logger().info(f"Configured for Jetson: max_distance={self.max_laser_distance}m, rate={self.publish_rate}Hz")
    
    def _create_subscriptions(self):
        """Create subscriptions with Jetson-optimized queue sizes"""
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, 5)
        
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pointcloud_callback, 3)
        
        self.detection_sub = self.create_subscription(
            Yolov12Inference, '/detection', self.detection_callback, 5)
    
    def _create_publishers(self):
        """Create publishers"""
        self.result_pub = self.create_publisher(
            Yolov12Inference, '/detection_with_distance', 5)
        
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fusion/markers', 5)
    
    def _create_timers(self):
        """Create timers with Jetson-optimized rates"""
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_fusion)
        
        # Statistics timer (less frequent)
        self.stats_timer = self.create_timer(30.0, self.log_statistics)
    
    def laserscan_callback(self, msg):
        """Process LaserScan with validation"""
        try:
            if msg and len(msg.ranges) > 0:
                with self.lock:
                    self.latest_scan = msg
        except Exception as e:
            self.get_logger().debug(f"Error in laserscan_callback: {e}")
    
    def pointcloud_callback(self, msg):
        """Process PointCloud2 with validation"""
        try:
            if msg and msg.width * msg.height > 0:
                with self.lock:
                    self.latest_cloud = msg
        except Exception as e:
            self.get_logger().debug(f"Error in pointcloud_callback: {e}")
    
    def detection_callback(self, msg):
        """Process detections with validation"""
        try:
            if msg and msg.camera_name:
                with self.lock:
                    self.latest_detections[msg.camera_name] = msg
                    self.stats['processed_detections'] += len(msg.yolov12_inference)
        except Exception as e:
            self.get_logger().debug(f"Error in detection_callback: {e}")
    
    def get_distance_from_laserscan(self, camera_name, obj_center_ratio):
        """Get distance from LaserScan based on camera and object position"""
        try:
            if not self.latest_scan or camera_name not in self.camera_fov:
                return None
            
            scan = self.latest_scan
            cam_min_angle, cam_max_angle = self.camera_fov[camera_name]
            
            # Map object position to global angle
            global_angle = cam_min_angle + obj_center_ratio * (cam_max_angle - cam_min_angle)
            lidar_angle = math.radians(global_angle)
            
            # Find corresponding laser ray
            ray_idx = int((lidar_angle - scan.angle_min) / scan.angle_increment)
            
            if 0 <= ray_idx < len(scan.ranges):
                distance = scan.ranges[ray_idx]
                if (self.min_laser_distance <= distance <= self.max_laser_distance and 
                    math.isfinite(distance)):
                    return distance
            
            return None
            
        except Exception as e:
            self.get_logger().debug(f"Error getting distance: {e}")
            return None
    
    def get_coordinates_from_pointcloud(self, camera_name, obj_center_ratio, distance):
        """Get 3D coordinates using simple trigonometry"""
        try:
            if not distance or camera_name not in self.camera_fov:
                return None
            
            cam_min_angle, cam_max_angle = self.camera_fov[camera_name]
            global_angle = cam_min_angle + obj_center_ratio * (cam_max_angle - cam_min_angle)
            lidar_angle = math.radians(global_angle)
            
            # Calculate coordinates
            x = distance * math.cos(lidar_angle)
            y = distance * math.sin(lidar_angle)
            z = 0.0  # Ground level assumption
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().debug(f"Error calculating coordinates: {e}")
            return None
    
    def process_fusion(self):
        """Main fusion processing with Jetson optimizations"""
        try:
            with self.lock:
                if not self.latest_scan or not self.latest_detections:
                    return
                
                output_messages = []
                markers = []
                marker_id = 0
                
                for camera_name, detections in self.latest_detections.items():
                    enhanced_msg = Yolov12Inference()
                    enhanced_msg.header = detections.header
                    enhanced_msg.camera_name = camera_name
                    enhanced_msg.frame_type = "fused"
                    enhanced_msg.task = "fusion"
                    enhanced_msg.note = "jetson_optimized_fusion"
                    
                    for det in detections.yolov12_inference:
                        if det.confidence < self.confidence_threshold:
                            continue
                        
                        # PERBAIKAN: Properly calculate object center based on actual coordinates
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
                        result.obb_angle = -1
                        result.mask_indices = []
                        
                        # PERBAIKAN: Check if 'note' field exists before setting
                        if hasattr(result, 'note'):
                            if distance and coords:
                                result.note = f"Distance: {distance:.2f}m, Coord: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            else:
                                result.note = "No distance/coordinate data available"
                        else:
                            # If note field doesn't exist, log distance info separately
                            if distance and coords:
                                self.get_logger().debug(
                                    f"{det.class_name}: Distance={distance:.2f}m, "
                                    f"Coord=({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                                )
                        
                        if distance and coords:
                            # Create marker for visualization
                            marker = Marker()
                            marker.header = detections.header
                            marker.ns = "fusion_objects"
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
                            marker.color.r = 1.0 if "person" in det.class_name.lower() else 0.0
                            marker.color.g = 0.0 if "person" in det.class_name.lower() else 1.0
                            marker.color.b = 0.0
                            marker.color.a = 0.7
                            marker.lifetime.sec = 2
                            markers.append(marker)
                        
                        enhanced_msg.yolov12_inference.append(result)
                    
                    if enhanced_msg.yolov12_inference:
                        output_messages.append(enhanced_msg)
                
                # Publish results
                for msg in output_messages:
                    self.result_pub.publish(msg)
                
                if markers:
                    marker_array = MarkerArray()
                    marker_array.markers = markers
                    self.marker_pub.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error in process_fusion: {str(e)}")
    
    def log_statistics(self):
        """Log performance statistics"""
        try:
            runtime = time.time() - self.stats['start_time']
            processed = self.stats['processed_detections']
            successful = self.stats['successful_fusions']
            
            success_rate = (successful / processed * 100) if processed > 0 else 0
            processing_rate = processed / runtime if runtime > 0 else 0
            
            self.get_logger().info(
                f"Fusion Stats - Processed: {processed}, Success: {successful} ({success_rate:.1f}%), "
                f"Rate: {processing_rate:.1f} det/sec"
            )
            
        except Exception as e:
            self.get_logger().debug(f"Error logging statistics: {e}")

def main(args=None):
    """Main entry point"""
    try:
        rclpy.init(args=args)
        node = SimpleFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped by keyboard interrupt")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()