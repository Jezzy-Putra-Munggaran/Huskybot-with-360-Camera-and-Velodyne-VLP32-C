#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from geometry_msgs.msg import Point
import numpy as np
import math
import time

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion')
        
        # Camera angle mapping (corrected for real hardware)
        self.camera_angles = {
            'front': 180,      # Real: BELAKANG
            'front_left': 225, # Real: KIRI BELAKANG
            'left': 270,       # Real: KIRI DEPAN
            'rear': 0,         # Real: DEPAN
            'rear_right': 315, # Real: KANAN DEPAN
            'right': 45        # Real: KANAN BELAKANG
        }
        
        # Subscribe to YOLO results
        self.detection_subs = []
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        for name in camera_names:
            # Support both detection and segmentation
            det_sub = self.create_subscription(
                Yolov12Inference,
                f'/camera_{name}/detections',
                lambda msg, cam=name: self.detection_callback(msg, cam),
                10
            )
            seg_sub = self.create_subscription(
                Yolov12Inference,
                f'/camera_{name}/segmentation',
                lambda msg, cam=name: self.detection_callback(msg, cam),
                10
            )
            self.detection_subs.extend([det_sub, seg_sub])
        
        # Subscribe to LiDAR
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        
        # Publishers for enhanced results
        self.fused_pubs = {}
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/fused_detections', 10)
            self.fused_pubs[name] = pub
        
        # Publisher for 3D visualization
        self.objects_3d_pub = self.create_publisher(
            PointCloud2, '/objects_3d_pointcloud', 10)
        
        # Data storage
        self.latest_detections = {}
        self.latest_laser = None
        self.latest_pointcloud = None
        
        self.get_logger().info("🔗 Simple Fusion Node initialized")
        self.get_logger().info(f"📐 Camera angles: {self.camera_angles}")

    def detection_callback(self, msg, camera_name):
        """Process detection/segmentation results with enhanced fusion"""
        self.latest_detections[camera_name] = msg
        
        if not self.latest_laser:
            self.get_logger().warn("⚠️ No LiDAR data available for fusion")
            return
            
        # Enhanced fusion with distance and coordinates
        self.enhanced_fusion(msg, camera_name)

    def laser_callback(self, msg):
        """Process LaserScan for distance measurement"""
        self.latest_laser = msg

    def pointcloud_callback(self, msg):
        """Process PointCloud for 3D coordinates"""
        self.latest_pointcloud = msg

    def enhanced_fusion(self, detection_msg, camera_name):
        """Enhanced fusion with distance and 3D coordinates"""
        try:
            base_angle = self.camera_angles.get(camera_name, 0)
            
            # Enhanced message with fusion data
            enhanced_msg = Yolov12Inference()
            enhanced_msg.header = detection_msg.header
            enhanced_msg.camera_name = detection_msg.camera_name
            enhanced_msg.task = detection_msg.task
            enhanced_msg.frame_type = detection_msg.frame_type + "_fused"
            enhanced_msg.note = f"Enhanced with LiDAR fusion from {camera_name}"  # ✅ ADDED
            
            # Process each detection with enhanced data
            for detection in detection_msg.yolov12_inference:
                enhanced_detection = InferenceResult()
                
                # Copy original detection data
                enhanced_detection.class_name = detection.class_name
                enhanced_detection.confidence = detection.confidence
                enhanced_detection.left = detection.left
                enhanced_detection.top = detection.top
                enhanced_detection.right = detection.right
                enhanced_detection.bottom = detection.bottom
                
                # Calculate object angle within camera FOV
                bbox_center_x = (detection.left + detection.right) / 2
                image_width = 1920  # Arducam IMX477 resolution
                
                # Map bbox position to angle offset (-30° to +30° for 60° FOV)
                angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                object_angle = (base_angle + angle_offset) % 360
                
                # Get distance from LaserScan
                distance = self.get_distance_from_laser(object_angle)
                
                # Calculate 3D coordinates (enhanced)
                x, y, z = self.calculate_3d_coordinates(object_angle, distance)
                
                # Add enhanced fusion data
                enhanced_detection.distance = distance
                enhanced_detection.coordinate_x = x
                enhanced_detection.coordinate_y = y
                enhanced_detection.coordinate_z = z
                enhanced_detection.angle = object_angle
                
                # Add to enhanced message
                enhanced_msg.yolov12_inference.append(enhanced_detection)
                
                # Log TARGET FORMAT
                self.get_logger().info(
                    f"🎯 Camera {camera_name}: "
                    f"Class={detection.class_name}, "
                    f"Confidence={detection.confidence:.2f}, "
                    f"Distance: {distance:.1f}m, "
                    f"Coordinate: ({x:.1f}, {y:.1f}, {z:.1f})"
                )
            
            # Publish enhanced result
            if camera_name in self.fused_pubs:
                self.fused_pubs[camera_name].publish(enhanced_msg)
            
            # Create 3D visualization
            self.create_3d_visualization(enhanced_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced fusion error: {e}")

    def get_distance_from_laser(self, angle_deg):
        """Get distance from LaserScan at specific angle"""
        if not self.latest_laser:
            return 10.0  # Default distance
            
        try:
            # Convert angle to LaserScan index
            angle_rad = math.radians(angle_deg)
            
            # Normalize angle to LaserScan range
            angle_normalized = (angle_rad - self.latest_laser.angle_min) / self.latest_laser.angle_increment
            index = int(angle_normalized) % len(self.latest_laser.ranges)
            
            distance = self.latest_laser.ranges[index]
            
            # Filter invalid readings
            if math.isinf(distance) or math.isnan(distance) or distance <= 0:
                return 15.0  # Default for invalid readings
                
            return min(distance, 100.0)  # Cap at 100m
            
        except Exception as e:
            self.get_logger().error(f"❌ Distance calculation error: {e}")
            return 10.0

    def calculate_3d_coordinates(self, angle_deg, distance):
        """Calculate enhanced 3D coordinates"""
        try:
            # Convert to radians
            angle_rad = math.radians(angle_deg)
            
            # Calculate 3D position relative to robot base
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            z = 0.5  # Assume average object height
            
            return x, y, z
            
        except Exception as e:
            self.get_logger().error(f"❌ 3D coordinate calculation error: {e}")
            return 0.0, 0.0, 0.0

    def create_3d_visualization(self, enhanced_msg):
        """Create 3D PointCloud visualization for RViz2"""
        try:
            # Create PointCloud2 message for visualization
            from sensor_msgs.msg import PointCloud2, PointField
            import struct
            
            # Create points for each detected object
            points = []
            for detection in enhanced_msg.yolov12_inference:
                # Add object center point
                points.append([
                    detection.coordinate_x,
                    detection.coordinate_y, 
                    detection.coordinate_z,
                    1.0  # Intensity/confidence
                ])
            
            if points:
                # Create PointCloud2 message
                pc_msg = PointCloud2()
                pc_msg.header.stamp = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = "base_link"
                
                # Define fields
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]
                
                # Pack data
                pc_msg.height = 1
                pc_msg.width = len(points)
                pc_msg.point_step = 16
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.is_dense = True
                
                buffer = []
                for point in points:
                    buffer.append(struct.pack('ffff', *point))
                pc_msg.data = b''.join(buffer)
                
                # Publish 3D visualization
                self.objects_3d_pub.publish(pc_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ 3D visualization error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()