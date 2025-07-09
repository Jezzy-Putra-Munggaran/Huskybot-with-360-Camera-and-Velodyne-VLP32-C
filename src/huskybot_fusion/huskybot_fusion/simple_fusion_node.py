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

class EnhancedFusionNode(Node):
    def __init__(self):
        super().__init__('enhanced_fusion')
        
        # ✅ FIXED camera angle mapping
        self.camera_angles = {
            'rear': 180,        # KAMERA BELAKANG
            'rear_left': 225,   # KAMERA KIRI BELAKANG
            'front_left': 315,  # KAMERA KIRI DEPAN
            'front': 0,         # KAMERA DEPAN
            'front_right': 45,  # KAMERA KANAN DEPAN
            'rear_right': 135   # KAMERA KANAN BELAKANG
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
        
        # ✅ Enhanced publishers
        self.fused_pubs = {}
        for name in self.camera_names:
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
        
        # ✅ Enhanced monitoring
        self.fusion_count = 0
        self.fusion_timer = self.create_timer(5.0, self.log_fusion_stats)
        self.publish_timer = self.create_timer(0.1, self.create_continuous_3d_visualization)
        
        self.get_logger().info("🔗 ENHANCED Fusion Node initialized")

    def detection_callback(self, msg, camera_name):
        """Process detection results with enhanced fusion"""
        self.latest_detections[camera_name] = msg
        
        # ✅ Enhanced fusion process
        self.enhanced_fusion_with_lidar(msg, camera_name)

    def laser_callback(self, msg):
        """Process LaserScan"""
        self.latest_laser = msg
        self.last_laser_time = time.time()

    def pointcloud_callback(self, msg):
        """Process PointCloud"""
        self.latest_pointcloud = msg
        self.last_pointcloud_time = time.time()

    def enhanced_fusion_with_lidar(self, detection_msg, camera_name):
        """Enhanced fusion with ACCURATE distance and coordinates"""
        try:
            base_angle = self.camera_angles.get(camera_name, 0)
            
            # Create enhanced message
            enhanced_msg = Yolov12Inference()
            enhanced_msg.header = detection_msg.header
            enhanced_msg.camera_name = detection_msg.camera_name
            enhanced_msg.task = detection_msg.task
            enhanced_msg.frame_type = detection_msg.frame_type + "_enhanced_fusion"
            enhanced_msg.note = f"Enhanced LiDAR fusion from {camera_name} at {base_angle}°"
            
            for detection in detection_msg.yolov12_inference:
                # ✅ Copy and enhance detection
                enhanced_detection = InferenceResult()
                enhanced_detection.class_name = detection.class_name
                enhanced_detection.confidence = detection.confidence
                enhanced_detection.left = detection.left
                enhanced_detection.top = detection.top
                enhanced_detection.right = detection.right
                enhanced_detection.bottom = detection.bottom
                
                # Copy visual data
                enhanced_detection.mask_data = detection.mask_data if hasattr(detection, 'mask_data') else []
                enhanced_detection.mask_width = detection.mask_width if hasattr(detection, 'mask_width') else 0
                enhanced_detection.mask_height = detection.mask_height if hasattr(detection, 'mask_height') else 0
                enhanced_detection.color_r = detection.color_r if hasattr(detection, 'color_r') else 255
                enhanced_detection.color_g = detection.color_g if hasattr(detection, 'color_g') else 255
                enhanced_detection.color_b = detection.color_b if hasattr(detection, 'color_b') else 255
                
                # ✅ ENHANCED: Use detection angle if available, otherwise calculate
                if hasattr(detection, 'angle') and detection.angle > 0:
                    object_angle = detection.angle
                else:
                    bbox_center_x = (detection.left + detection.right) / 2
                    image_width = 1920
                    angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                    object_angle = (base_angle + angle_offset) % 360
                
                enhanced_detection.angle = object_angle
                
                # ✅ ENHANCED: Get ACCURATE distance from LaserScan
                if self.latest_laser:
                    accurate_distance, ray_index = self.get_accurate_distance_from_laser(object_angle)
                    enhanced_detection.distance = accurate_distance
                else:
                    # Use detection distance if available
                    enhanced_detection.distance = detection.distance if hasattr(detection, 'distance') and detection.distance > 0 else 10.0
                
                # ✅ ENHANCED: Calculate ACCURATE 3D coordinates
                x, y, z = self.calculate_accurate_3d_coordinates(object_angle, enhanced_detection.distance, detection)
                enhanced_detection.coordinate_x = x
                enhanced_detection.coordinate_y = y
                enhanced_detection.coordinate_z = z
                
                enhanced_msg.yolov12_inference.append(enhanced_detection)
                self.fusion_count += 1
            
            # Publish enhanced result
            if camera_name in self.fused_pubs:
                self.fused_pubs[camera_name].publish(enhanced_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced fusion error: {e}")

    def get_accurate_distance_from_laser(self, angle_deg):
        """Get ACCURATE distance from LaserScan at specific angle"""
        if not self.latest_laser:
            return 10.0, -1
            
        try:
            angle_rad = math.radians(angle_deg)
            
            # Handle angle wrap-around properly
            while angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            while angle_rad < -math.pi:
                angle_rad += 2 * math.pi
            
            # ✅ Enhanced angle calculation
            if angle_rad < self.latest_laser.angle_min or angle_rad > self.latest_laser.angle_max:
                return 15.0, -1
            
            angle_normalized = (angle_rad - self.latest_laser.angle_min) / self.latest_laser.angle_increment
            ray_index = int(round(angle_normalized)) % len(self.latest_laser.ranges)
            
            # ✅ Enhanced distance filtering with multiple rays
            distances = []
            for offset in [-2, -1, 0, 1, 2]:  # Check nearby rays
                idx = (ray_index + offset) % len(self.latest_laser.ranges)
                dist = self.latest_laser.ranges[idx]
                if not (math.isinf(dist) or math.isnan(dist) or dist <= 0):
                    distances.append(dist)
            
            if distances:
                # Use median for better accuracy
                distance = np.median(distances)
                return min(distance, 100.0), ray_index
            else:
                return 15.0, ray_index
            
        except Exception as e:
            return 10.0, -1

    def calculate_accurate_3d_coordinates(self, angle_deg, distance, detection):
        """Calculate ACCURATE 3D coordinates in robot base frame"""
        try:
            angle_rad = math.radians(angle_deg)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            
            # ✅ Enhanced Z calculation based on detection size
            if hasattr(detection, 'bottom') and hasattr(detection, 'top'):
                bbox_height = detection.bottom - detection.top
                # Estimate object height based on bbox size and distance
                estimated_real_height = (bbox_height / 720.0) * distance * 0.5  # Rough estimation
                z = max(0.2, min(3.0, estimated_real_height))  # Clamp between reasonable values
            else:
                z = 0.5  # Default height
            
            return x, y, z
        except Exception as e:
            return 0.0, 0.0, 0.5

    def create_continuous_3d_visualization(self):
        """✅ CONTINUOUS 3D object visualization for RViz2"""
        try:
            all_points = []
            
            # ✅ Collect ALL detections from ALL cameras
            for camera_name, detection_msg in self.latest_detections.items():
                if detection_msg and hasattr(detection_msg, 'yolov12_inference'):
                    for detection in detection_msg.yolov12_inference:
                        if hasattr(detection, 'coordinate_x'):
                            # ✅ ENHANCED: Create object-shaped point cloud
                            center_x = detection.coordinate_x
                            center_y = detection.coordinate_y
                            center_z = detection.coordinate_z
                            
                            # ✅ Create object shape based on class
                            object_size = self.get_object_size(detection.class_name)
                            intensity = detection.confidence * 255.0
                            
                            # ✅ Generate points based on object type
                            if detection.class_name in ['person', 'bicycle', 'motorcycle']:
                                # Vertical objects
                                for dz in np.linspace(0, object_size['height'], 8):
                                    for dx in np.linspace(-object_size['width']/2, object_size['width']/2, 4):
                                        for dy in np.linspace(-object_size['depth']/2, object_size['depth']/2, 4):
                                            all_points.append([
                                                center_x + dx,
                                                center_y + dy,
                                                dz,
                                                intensity
                                            ])
                            elif detection.class_name in ['car', 'truck', 'bus']:
                                # Rectangular objects
                                for dx in np.linspace(-object_size['width']/2, object_size['width']/2, 6):
                                    for dy in np.linspace(-object_size['depth']/2, object_size['depth']/2, 10):
                                        for dz in np.linspace(0, object_size['height'], 4):
                                            all_points.append([
                                                center_x + dx,
                                                center_y + dy,
                                                dz,
                                                intensity
                                            ])
                            else:
                                # Generic objects - sphere-like
                                for i in range(30):
                                    theta = np.random.uniform(0, 2*np.pi)
                                    phi = np.random.uniform(0, np.pi)
                                    r = np.random.uniform(0, object_size['width']/2)
                                    
                                    dx = r * np.sin(phi) * np.cos(theta)
                                    dy = r * np.sin(phi) * np.sin(theta)
                                    dz = r * np.cos(phi) + object_size['height']/2
                                    
                                    all_points.append([
                                        center_x + dx,
                                        center_y + dy,
                                        max(0, dz),
                                        intensity
                                    ])
            
            if all_points:
                # ✅ ENHANCED PointCloud2 message
                pc_msg = PointCloud2()
                pc_msg.header.stamp = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = "base_link"
                
                # ✅ CORRECT field definitions for RViz2
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]
                
                # ✅ CORRECT message format
                pc_msg.height = 1
                pc_msg.width = len(all_points)
                pc_msg.point_step = 16
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.is_dense = True
                pc_msg.is_bigendian = False
                
                # ✅ ENHANCED data packing
                data = bytearray()
                for point in all_points:
                    data.extend(struct.pack('<ffff', point[0], point[1], point[2], point[3]))
                pc_msg.data = bytes(data)
                
                # Publish
                self.objects_3d_pub.publish(pc_msg)
                
            else:
                # ✅ Publish empty pointcloud to keep topic alive
                pc_msg = PointCloud2()
                pc_msg.header.stamp = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = "base_link"
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]
                pc_msg.height = 1
                pc_msg.width = 0
                pc_msg.point_step = 16
                pc_msg.row_step = 0
                pc_msg.is_dense = True
                pc_msg.is_bigendian = False
                pc_msg.data = bytes()
                self.objects_3d_pub.publish(pc_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ 3D visualization error: {e}")

    def get_object_size(self, class_name):
        """Get estimated object size for better visualization"""
        sizes = {
            'person': {'width': 0.6, 'depth': 0.4, 'height': 1.7},
            'bicycle': {'width': 1.8, 'depth': 0.6, 'height': 1.2},
            'car': {'width': 1.8, 'depth': 4.5, 'height': 1.5},
            'motorcycle': {'width': 0.8, 'depth': 2.0, 'height': 1.2},
            'bus': {'width': 2.5, 'depth': 12.0, 'height': 3.0},
            'truck': {'width': 2.5, 'depth': 8.0, 'height': 3.5},
            'traffic light': {'width': 0.3, 'depth': 0.3, 'height': 0.8},
            'stop sign': {'width': 0.8, 'depth': 0.1, 'height': 0.8},
        }
        
        return sizes.get(class_name, {'width': 0.5, 'depth': 0.5, 'height': 0.5})

    def log_fusion_stats(self):
        """Enhanced fusion statistics"""
        current_time = time.time()
        laser_age = current_time - self.last_laser_time if self.last_laser_time > 0 else float('inf')
        pc_age = current_time - self.last_pointcloud_time if self.last_pointcloud_time > 0 else float('inf')
        
        total_objects = sum(len(msg.yolov12_inference) if msg and hasattr(msg, 'yolov12_inference') else 0 
                           for msg in self.latest_detections.values())
        
        self.get_logger().info(
            f"🔗 Enhanced Fusion: {self.fusion_count} processed, {total_objects} active objects, "
            f"Laser: {laser_age:.1f}s, PC: {pc_age:.1f}s"
        )
        
        self.fusion_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()