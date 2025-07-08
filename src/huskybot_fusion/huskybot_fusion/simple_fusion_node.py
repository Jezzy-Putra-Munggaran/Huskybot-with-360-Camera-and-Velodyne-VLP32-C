#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from geometry_msgs.msg import Point
import numpy as np
import math
import time
import struct

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion')
        
        # ✅ CORRECTED Camera angle mapping for real hardware
        self.camera_angles = {
            'front': 0,        # DEPAN
            'front_left': 315, # KIRI DEPAN  
            'left': 270,       # KIRI
            'rear': 180,       # BELAKANG
            'rear_right': 225, # KIRI BELAKANG
            'right': 45        # KANAN
        }
        
        # ✅ ENHANCED subscriptions to both detection and segmentation
        self.detection_subs = []
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        for name in camera_names:
            # Subscribe to both detection and segmentation from DeepStream
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
        
        # ✅ ENHANCED LiDAR subscriptions with better error handling
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
        
        # ✅ ENHANCED 3D visualization publisher
        self.objects_3d_pub = self.create_publisher(
            PointCloud2, '/objects_3d_pointcloud', 10)
        
        # Data storage
        self.latest_detections = {}
        self.latest_laser = None
        self.latest_pointcloud = None
        self.last_laser_time = 0
        self.last_pointcloud_time = 0
        
        # ✅ ENHANCED monitoring
        self.fusion_count = 0
        self.fusion_timer = self.create_timer(5.0, self.log_fusion_stats)
        
        self.get_logger().info("🔗 Enhanced Simple Fusion Node initialized")
        self.get_logger().info(f"📐 Camera angles (corrected): {self.camera_angles}")

    def detection_callback(self, msg, camera_name):
        """Process detection/segmentation results with enhanced fusion"""
        self.latest_detections[camera_name] = msg
        
        # ✅ CHECK LiDAR data availability with better logging
        current_time = time.time()
        if not self.latest_laser or (current_time - self.last_laser_time) > 2.0:
            if not self.latest_laser:
                self.get_logger().warn("⚠️ No LiDAR LaserScan data available yet")
            else:
                self.get_logger().warn(f"⚠️ LiDAR LaserScan data is stale ({current_time - self.last_laser_time:.1f}s old)")
            return
            
        # Enhanced fusion with distance and coordinates
        self.enhanced_fusion(msg, camera_name)

    def laser_callback(self, msg):
        """Process LaserScan for distance measurement"""
        self.latest_laser = msg
        self.last_laser_time = time.time()
        
        # ✅ LOG LiDAR data reception
        valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r) or r <= 0)]
        self.get_logger().info(
            f"📡 LiDAR LaserScan: {len(valid_ranges)}/{len(msg.ranges)} valid ranges, "
            f"angle: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°"
        )

    def pointcloud_callback(self, msg):
        """Process PointCloud for 3D coordinates"""
        self.latest_pointcloud = msg
        self.last_pointcloud_time = time.time()
        
        # ✅ LOG PointCloud data reception
        point_count = msg.width * msg.height
        self.get_logger().info(f"📡 LiDAR PointCloud: {point_count} points")

    def enhanced_fusion(self, detection_msg, camera_name):
        """Enhanced fusion with REAL distance and 3D coordinates"""
        try:
            base_angle = self.camera_angles.get(camera_name, 0)
            
            # Enhanced message with fusion data
            enhanced_msg = Yolov12Inference()
            enhanced_msg.header = detection_msg.header
            enhanced_msg.camera_name = detection_msg.camera_name
            enhanced_msg.task = detection_msg.task
            enhanced_msg.frame_type = detection_msg.frame_type + "_fused"
            enhanced_msg.note = f"Enhanced with LiDAR fusion from {camera_name}"
            
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
                
                # ✅ ENHANCED Calculate object angle within camera FOV
                bbox_center_x = (detection.left + detection.right) / 2
                image_width = 1920  # Arducam IMX477 resolution
                
                # Map bbox position to angle offset (-30° to +30° for 60° FOV)
                angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                object_angle = (base_angle + angle_offset) % 360
                
                # ✅ ENHANCED Get distance from LaserScan with detailed logging
                distance, ray_index = self.get_distance_from_laser(object_angle)
                
                # ✅ ENHANCED Calculate 3D coordinates
                x, y, z = self.calculate_3d_coordinates(object_angle, distance)
                
                # Add enhanced fusion data
                enhanced_detection.distance = distance
                enhanced_detection.coordinate_x = x
                enhanced_detection.coordinate_y = y
                enhanced_detection.coordinate_z = z
                enhanced_detection.angle = object_angle
                
                # Add to enhanced message
                enhanced_msg.yolov12_inference.append(enhanced_detection)
                
                # ✅ ENHANCED Log TARGET FORMAT with ray info
                self.get_logger().info(
                    f"🎯 Camera {camera_name}: Found distance {distance:.2f}m at ray {ray_index} (target angle={object_angle:.1f}°)"
                )
                self.get_logger().info(
                    f"🎯 Camera {camera_name}: {detection.class_name} conf={detection.confidence:.2f} "
                    f"Distance={distance:.2f}m Coordinate=({x:.2f}, {y:.2f}, {z:.2f})"
                )
                
                self.fusion_count += 1
            
            # Publish enhanced result
            if camera_name in self.fused_pubs:
                self.fused_pubs[camera_name].publish(enhanced_msg)
            
            # ✅ ENHANCED Create 3D visualization
            self.create_enhanced_3d_visualization(enhanced_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced fusion error: {e}")
            import traceback
            traceback.print_exc()

    def get_distance_from_laser(self, angle_deg):
        """Get distance from LaserScan at specific angle with enhanced logging"""
        if not self.latest_laser:
            return 10.0, -1  # Default distance, invalid ray
            
        try:
            # Convert angle to LaserScan index
            angle_rad = math.radians(angle_deg)
            
            # ✅ ENHANCED angle normalization for LaserScan range
            # Normalize angle to LaserScan coordinate system
            laser_angle = angle_rad
            
            # Handle angle wrap-around
            while laser_angle > math.pi:
                laser_angle -= 2 * math.pi
            while laser_angle < -math.pi:
                laser_angle += 2 * math.pi
            
            # Calculate index in LaserScan ranges array
            if laser_angle < self.latest_laser.angle_min or laser_angle > self.latest_laser.angle_max:
                # ✅ Handle out-of-range angles
                self.get_logger().warn(
                    f"⚠️ Angle {angle_deg:.1f}° ({laser_angle:.3f} rad) outside LaserScan range "
                    f"[{math.degrees(self.latest_laser.angle_min):.1f}°, {math.degrees(self.latest_laser.angle_max):.1f}°]"
                )
                return 15.0, -1
            
            angle_normalized = (laser_angle - self.latest_laser.angle_min) / self.latest_laser.angle_increment
            ray_index = int(angle_normalized) % len(self.latest_laser.ranges)
            
            distance = self.latest_laser.ranges[ray_index]
            
            # ✅ ENHANCED Filter invalid readings with logging
            if math.isinf(distance) or math.isnan(distance) or distance <= 0:
                self.get_logger().warn(f"⚠️ Invalid distance reading at ray {ray_index}: {distance}")
                return 15.0, ray_index  # Default for invalid readings
            
            # Cap at reasonable maximum
            distance = min(distance, 100.0)
            
            # ✅ LOG successful distance measurement
            if ray_index % 50 == 0:  # Log every 50th measurement to avoid spam
                self.get_logger().info(
                    f"📏 Ray {ray_index}: angle={angle_deg:.1f}° -> distance={distance:.2f}m"
                )
                
            return distance, ray_index
            
        except Exception as e:
            self.get_logger().error(f"❌ Distance calculation error: {e}")
            return 10.0, -1

    def calculate_3d_coordinates(self, angle_deg, distance):
        """Calculate enhanced 3D coordinates in robot base frame"""
        try:
            # Convert to radians
            angle_rad = math.radians(angle_deg)
            
            # ✅ ENHANCED 3D position relative to robot base_link
            # Standard ROS coordinate system: x=forward, y=left, z=up
            x = distance * math.cos(angle_rad)  # Forward/backward
            y = distance * math.sin(angle_rad)  # Left/right
            z = 0.5  # Assume average object height above ground
            
            return x, y, z
            
        except Exception as e:
            self.get_logger().error(f"❌ 3D coordinate calculation error: {e}")
            return 0.0, 0.0, 0.0

    def create_enhanced_3d_visualization(self, enhanced_msg):
        """Create ENHANCED 3D PointCloud visualization for RViz2"""
        try:
            # Create points for each detected object
            points = []
            for detection in enhanced_msg.yolov12_inference:
                # ✅ Add object center point with confidence as intensity
                points.append([
                    detection.coordinate_x,
                    detection.coordinate_y, 
                    detection.coordinate_z,
                    detection.confidence  # Use confidence as intensity/color
                ])
                
                # ✅ ADD additional visualization points around object
                # Create a small cluster for better visibility
                for dx in [-0.1, 0.1]:
                    for dy in [-0.1, 0.1]:
                        points.append([
                            detection.coordinate_x + dx,
                            detection.coordinate_y + dy,
                            detection.coordinate_z + 0.1,
                            detection.confidence * 0.8  # Slightly lower intensity
                        ])
            
            if points:
                # ✅ ENHANCED PointCloud2 message creation
                pc_msg = PointCloud2()
                pc_msg.header.stamp = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = "base_link"  # Robot base frame
                
                # Define fields with enhanced structure
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
                
                # Publish enhanced 3D visualization
                self.objects_3d_pub.publish(pc_msg)
                
                self.get_logger().info(
                    f"📊 Published 3D PointCloud: {len(enhanced_msg.yolov12_inference)} objects, "
                    f"{len(points)} total points to /objects_3d_pointcloud"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced 3D visualization error: {e}")
            import traceback
            traceback.print_exc()

    def log_fusion_stats(self):
        """Log fusion statistics"""
        current_time = time.time()
        
        # Check data freshness
        laser_age = current_time - self.last_laser_time if self.last_laser_time > 0 else float('inf')
        pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time > 0 else float('inf')
        
        self.get_logger().info(
            f"🔗 Fusion Stats: {self.fusion_count} objects fused in 5s, "
            f"LaserScan age: {laser_age:.1f}s, PointCloud age: {pc_age:.1f}s"
        )
        
        if laser_age > 5.0:
            self.get_logger().warn("⚠️ LaserScan data is too old!")
        if pc_age > 5.0:
            self.get_logger().warn("⚠️ PointCloud data is too old!")
            
        # Reset counter
        self.fusion_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()