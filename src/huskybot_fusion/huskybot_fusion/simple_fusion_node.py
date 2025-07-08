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
        
        # ✅ Camera angle mapping
        self.camera_angles = {
            'rear': 180,        
            'rear_left': 225,   
            'front_left': 315,  
            'front': 0,         
            'front_right': 45,  
            'rear_right': 135   
        }
        
        # ✅ Enhanced subscriptions
        self.detection_subs = []
        self.camera_names = ['rear', 'rear_left', 'front_left', 'front', 'front_right', 'rear_right']
        
        for name in self.camera_names:
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
        
        # ✅ LiDAR subscriptions
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        
        # ✅ Publishers for enhanced results
        self.fused_pubs = {}
        for name in self.camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/fused_detections', 10)
            self.fused_pubs[name] = pub
        
        # ✅ FIXED 3D visualization publisher
        self.objects_3d_pub = self.create_publisher(
            PointCloud2, '/objects_3d_pointcloud', 10)
        
        # Data storage
        self.latest_detections = {}
        self.latest_laser = None
        self.latest_pointcloud = None
        self.last_laser_time = 0
        self.last_pointcloud_time = 0
        
        # ✅ Enhanced monitoring
        self.fusion_count = 0
        self.fusion_timer = self.create_timer(5.0, self.log_fusion_stats)
        
        self.get_logger().info("🔗 ULTRA-Enhanced Fusion Node initialized")

    def detection_callback(self, msg, camera_name):
        """Process detection results with enhanced fusion"""
        self.latest_detections[camera_name] = msg
        
        # ✅ Check LiDAR availability
        current_time = time.time()
        if not self.latest_laser or (current_time - self.last_laser_time) > 2.0:
            return
            
        # Enhanced fusion
        self.enhanced_fusion(msg, camera_name)

    def laser_callback(self, msg):
        """Process LaserScan"""
        self.latest_laser = msg
        self.last_laser_time = time.time()

    def pointcloud_callback(self, msg):
        """Process PointCloud"""
        self.latest_pointcloud = msg
        self.last_pointcloud_time = time.time()

    def enhanced_fusion(self, detection_msg, camera_name):
        """Enhanced fusion with REAL distance and coordinates"""
        try:
            base_angle = self.camera_angles.get(camera_name, 0)
            
            # Create enhanced message
            enhanced_msg = Yolov12Inference()
            enhanced_msg.header = detection_msg.header
            enhanced_msg.camera_name = detection_msg.camera_name
            enhanced_msg.task = detection_msg.task
            enhanced_msg.frame_type = detection_msg.frame_type + "_fused"
            enhanced_msg.note = f"Enhanced with LiDAR fusion from {camera_name}"
            
            for detection in detection_msg.yolov12_inference:
                # ✅ Copy all detection fields properly
                enhanced_detection = InferenceResult()
                enhanced_detection.class_name = detection.class_name
                enhanced_detection.confidence = detection.confidence
                enhanced_detection.left = detection.left
                enhanced_detection.top = detection.top
                enhanced_detection.right = detection.right
                enhanced_detection.bottom = detection.bottom
                
                # Copy mask and color data
                enhanced_detection.mask_data = detection.mask_data if hasattr(detection, 'mask_data') else []
                enhanced_detection.mask_width = detection.mask_width if hasattr(detection, 'mask_width') else 0
                enhanced_detection.mask_height = detection.mask_height if hasattr(detection, 'mask_height') else 0
                enhanced_detection.color_r = detection.color_r if hasattr(detection, 'color_r') else 255
                enhanced_detection.color_g = detection.color_g if hasattr(detection, 'color_g') else 255
                enhanced_detection.color_b = detection.color_b if hasattr(detection, 'color_b') else 255
                
                # ✅ Calculate object angle
                bbox_center_x = (detection.left + detection.right) / 2
                image_width = 1920
                angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                object_angle = (base_angle + angle_offset) % 360
                enhanced_detection.angle = object_angle
                
                # ✅ Get REAL distance from LaserScan
                distance, ray_index = self.get_distance_from_laser(object_angle)
                enhanced_detection.distance = distance
                
                # ✅ Calculate REAL 3D coordinates
                x, y, z = self.calculate_3d_coordinates(object_angle, distance)
                enhanced_detection.coordinate_x = x
                enhanced_detection.coordinate_y = y
                enhanced_detection.coordinate_z = z
                
                enhanced_msg.yolov12_inference.append(enhanced_detection)
                self.fusion_count += 1
            
            # Publish enhanced result
            if camera_name in self.fused_pubs:
                self.fused_pubs[camera_name].publish(enhanced_msg)
            
            # ✅ Create FIXED 3D visualization
            self.create_fixed_3d_visualization(enhanced_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Fusion error: {e}")

    def get_distance_from_laser(self, angle_deg):
        """Get distance from LaserScan at specific angle"""
        if not self.latest_laser:
            return 10.0, -1
            
        try:
            angle_rad = math.radians(angle_deg)
            
            # Handle angle wrap-around
            while angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            while angle_rad < -math.pi:
                angle_rad += 2 * math.pi
            
            # Calculate index
            if angle_rad < self.latest_laser.angle_min or angle_rad > self.latest_laser.angle_max:
                return 15.0, -1
            
            angle_normalized = (angle_rad - self.latest_laser.angle_min) / self.latest_laser.angle_increment
            ray_index = int(angle_normalized) % len(self.latest_laser.ranges)
            
            distance = self.latest_laser.ranges[ray_index]
            
            # Filter invalid readings
            if math.isinf(distance) or math.isnan(distance) or distance <= 0:
                return 15.0, ray_index
            
            return min(distance, 100.0), ray_index
            
        except Exception as e:
            return 10.0, -1

    def calculate_3d_coordinates(self, angle_deg, distance):
        """Calculate 3D coordinates in robot base frame"""
        try:
            angle_rad = math.radians(angle_deg)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            z = 0.5  # Average object height
            return x, y, z
        except Exception as e:
            return 0.0, 0.0, 0.0

    def create_fixed_3d_visualization(self, enhanced_msg):
        """Create FIXED 3D PointCloud visualization for RViz2"""
        try:
            points = []
            for detection in enhanced_msg.yolov12_inference:
                # ✅ Add object center point
                points.append([
                    float(detection.coordinate_x),
                    float(detection.coordinate_y), 
                    float(detection.coordinate_z),
                    float(detection.confidence * 100.0)  # Scale confidence for visibility
                ])
                
                # ✅ Add cluster points for better visibility
                for dx in [-0.3, 0.0, 0.3]:
                    for dy in [-0.3, 0.0, 0.3]:
                        for dz in [0.0, 0.3, 0.6]:
                            points.append([
                                float(detection.coordinate_x + dx),
                                float(detection.coordinate_y + dy),
                                float(detection.coordinate_z + dz),
                                float(detection.confidence * 80.0)
                            ])
            
            if points:
                # ✅ FIXED PointCloud2 message
                pc_msg = PointCloud2()
                pc_msg.header.stamp = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = "base_link"  # ✅ Correct frame
                
                # ✅ FIXED field definitions
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]
                
                # ✅ FIXED packing
                pc_msg.height = 1
                pc_msg.width = len(points)
                pc_msg.point_step = 16
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.is_dense = True
                
                # ✅ FIXED data packing
                data = bytearray()
                for point in points:
                    data.extend(struct.pack('ffff', point[0], point[1], point[2], point[3]))
                pc_msg.data = bytes(data)
                
                # Publish
                self.objects_3d_pub.publish(pc_msg)
                
                self.get_logger().info(
                    f"📊 Published 3D PointCloud: {len(enhanced_msg.yolov12_inference)} objects, "
                    f"{len(points)} points to /objects_3d_pointcloud"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ 3D visualization error: {e}")

    def log_fusion_stats(self):
        """Log fusion statistics"""
        current_time = time.time()
        laser_age = current_time - self.last_laser_time if self.last_laser_time > 0 else float('inf')
        pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time > 0 else float('inf')
        
        self.get_logger().info(
            f"🔗 Fusion: {self.fusion_count} objects/5s, "
            f"Laser: {laser_age:.1f}s, PC: {pc_age:.1f}s"
        )
        
        self.fusion_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()