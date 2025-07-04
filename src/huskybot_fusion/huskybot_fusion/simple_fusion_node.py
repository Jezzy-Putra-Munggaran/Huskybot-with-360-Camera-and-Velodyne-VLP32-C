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
        
        # PERBAIKAN: Add debug timer
        self.debug_timer = self.create_timer(5.0, self.debug_lidar_status)
        
        # Tambahkan debug detail timer
        self.debug_detail_timer = self.create_timer(10.0, self.debug_lidar_detailed)
    
    def laserscan_callback(self, msg):
        """Process LaserScan with validation and debugging"""
        try:
            if msg and len(msg.ranges) > 0:
                with self.lock:
                    self.latest_scan = msg
                    
                # PERBAIKAN: Log scan info for debugging
                valid_ranges = sum(1 for r in msg.ranges if math.isfinite(r) and r > 0)
                self.get_logger().debug(
                    f"LaserScan: {valid_ranges}/{len(msg.ranges)} valid ranges, "
                    f"angle_min={math.degrees(msg.angle_min):.1f}°, "
                    f"angle_max={math.degrees(msg.angle_max):.1f}°"
                )
                
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
        """PERBAIKAN: Mapping yang BENAR sesuai hardware real"""
        try:
            if not self.latest_scan or not self.latest_scan.ranges:
                return None
                
            scan = self.latest_scan
            
            # PERBAIKAN: Mapping yang BENAR sesuai posisi fisik real
            camera_angles = {
                # Topic name → Real position → LiDAR angle
                'camera_front': 180,        # KAMERA BELAKANG → 180°
                'camera_front_left': 225,   # KAMERA KIRI BELAKANG → 225° (-135°)
                'camera_left': 270,         # KAMERA KIRI DEPAN → 270° (-90°)
                'camera_rear': 0,           # KAMERA DEPAN → 0°
                'camera_rear_right': 315,   # KAMERA KANAN DEPAN → 315° (-45°)
                'camera_right': 45,         # KAMERA KANAN BELAKANG → 45°
            }
            
            if camera_name not in camera_angles:
                self.get_logger().warning(f"Unknown camera: {camera_name}")
                return None
                
            # Calculate target angle based on object position in image
            base_angle = camera_angles[camera_name]
            camera_fov = 60  # Assume 60° FOV per camera
            
            # Map object position to angle offset within camera FOV
            angle_offset = (obj_center_ratio - 0.5) * camera_fov  # -30° to +30°
            target_angle_deg = base_angle + angle_offset
            
            # Normalize to -180° to +180°
            if target_angle_deg > 180:
                target_angle_deg -= 360
            elif target_angle_deg < -180:
                target_angle_deg += 360
                
            target_angle_rad = math.radians(target_angle_deg)
            
            # PERBAIKAN: Ensure angle is within LiDAR range
            if target_angle_rad < scan.angle_min:
                target_angle_rad += 2 * math.pi
            elif target_angle_rad > scan.angle_max:
                target_angle_rad -= 2 * math.pi
                
            # Calculate ray index
            if abs(scan.angle_increment) < 1e-6:
                self.get_logger().warning("Invalid angle_increment")
                return None
                
            ray_idx = int((target_angle_rad - scan.angle_min) / scan.angle_increment)
            
            # PERBAIKAN: Check multiple rays around target for better reliability
            search_range = 5  # Check ±5 rays around target
            best_distance = None
            best_ray = -1
            
            for offset in range(-search_range, search_range + 1):
                check_idx = ray_idx + offset
                if 0 <= check_idx < len(scan.ranges):
                    distance = scan.ranges[check_idx]
                    if (distance > 0 and 
                        self.min_laser_distance <= distance <= self.max_laser_distance and 
                        math.isfinite(distance)):
                        best_distance = distance
                        best_ray = check_idx
                        break
            
            if best_distance:
                self.get_logger().info(
                    f"✅ {camera_name} (REAL: {self._get_real_position(camera_name)}): "
                    f"Found distance {best_distance:.2f}m at ray {best_ray} "
                    f"(target_angle={target_angle_deg:.1f}°)"
                )
                return best_distance
            else:
                self.get_logger().debug(
                    f"❌ {camera_name} (REAL: {self._get_real_position(camera_name)}): "
                    f"No valid distance found around ray {ray_idx} "
                    f"(target_angle={target_angle_deg:.1f}°)"
                )
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting distance for {camera_name}: {e}")
            return None

    def _get_real_position(self, camera_name):
        """Helper untuk mendapatkan posisi real kamera"""
        real_positions = {
            'camera_front': 'BELAKANG',
            'camera_front_left': 'KIRI_BELAKANG', 
            'camera_left': 'KIRI_DEPAN',
            'camera_rear': 'DEPAN',
            'camera_rear_right': 'KANAN_DEPAN',
            'camera_right': 'KANAN_BELAKANG'
        }
        return real_positions.get(camera_name, 'UNKNOWN')

    def get_coordinates_from_pointcloud(self, camera_name, obj_center_ratio, distance):
        """PERBAIKAN: Calculate 3D coordinates dengan mapping yang BENAR"""
        try:
            if not distance:
                return None
                
            # Use CORRECTED mapping sesuai posisi fisik real
            camera_angles = {
                'camera_front': 180,        # KAMERA BELAKANG
                'camera_front_left': 225,   # KAMERA KIRI BELAKANG
                'camera_left': 270,         # KAMERA KIRI DEPAN
                'camera_rear': 0,           # KAMERA DEPAN
                'camera_rear_right': 315,   # KAMERA KANAN DEPAN
                'camera_right': 45,         # KAMERA KANAN BELAKANG
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
            
            # Calculate 3D coordinates (Velodyne frame)
            x = distance * math.cos(target_angle_rad)
            y = distance * math.sin(target_angle_rad)
            z = 0.0  # Ground level assumption
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f"Error calculating coordinates: {e}")
            return None
    
    def process_fusion(self):
        """Main fusion processing dengan output jarak dan koordinat yang jelas"""
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
                    enhanced_msg.note = "fusion_with_lidar"
                    
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
                        result.obb_angle = -1.0  # PERBAIKAN: Float value
                        result.mask_indices = []
                        
                        # PERBAIKAN: Set note field dengan format yang jelas
                        if hasattr(result, 'note'):
                            if distance and coords:
                                result.note = f"Distance={distance:.2f}m, Coordinate=({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            else:
                                result.note = "No LiDAR data available"
                        
                        # PERBAIKAN: Log ke console dengan format yang jelas
                        if distance and coords:
                            self.get_logger().info(
                                f"🎯 {camera_name}: {det.class_name} conf={det.confidence:.2f} "
                                f"Distance={distance:.2f}m Coordinate=({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})"
                            )
                            
                            # Create marker for RViz2 visualization
                            marker = Marker()
                            marker.header = detections.header
                            marker.header.frame_id = "velodyne"  # PERBAIKAN: Set frame_id yang benar
                            marker.ns = "fusion_objects"
                            marker.id = marker_id
                            marker_id += 1
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            marker.pose.position.x = coords[0]
                            marker.pose.position.y = coords[1] 
                            marker.pose.position.z = coords[2] + 0.5  # Raise untuk visibility
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.5
                            marker.scale.y = 0.5
                            marker.scale.z = 1.0
                            
                            # Color coding berdasarkan class
                            if "person" in det.class_name.lower():
                                marker.color.r = 1.0  # Red untuk person
                                marker.color.g = 0.0
                                marker.color.b = 0.0
                            elif "car" in det.class_name.lower():
                                marker.color.r = 0.0  # Blue untuk car
                                marker.color.g = 0.0
                                marker.color.b = 1.0
                            else:
                                marker.color.r = 0.0  # Green untuk lainnya
                                marker.color.g = 1.0
                                marker.color.b = 0.0
                            
                            marker.color.a = 0.8
                            marker.lifetime.sec = 3
                            markers.append(marker)
                        else:
                            self.get_logger().warning(
                                f"❌ {camera_name}: {det.class_name} conf={det.confidence:.2f} "
                                f"NO_LIDAR_DATA"
                            )
                        
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
                    self.get_logger().info(f"📍 Published {len(markers)} 3D markers to RViz2")
                
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
    
    def debug_lidar_status(self):
        """Debug method to check LiDAR data availability"""
        try:
            if self.latest_scan:
                valid_ranges = sum(1 for r in self.latest_scan.ranges if math.isfinite(r) and r > 0)
                self.get_logger().info(
                    f"🛰️ LiDAR Status: {valid_ranges}/{len(self.latest_scan.ranges)} valid ranges"
                )
            else:
                self.get_logger().warning("🛰️ LiDAR Status: NO SCAN DATA")
                
            if self.latest_cloud:
                self.get_logger().info(
                    f"☁️ PointCloud Status: {self.latest_cloud.width}x{self.latest_cloud.height} points"
                )
            else:
                self.get_logger().warning("☁️ PointCloud Status: NO CLOUD DATA")
                
        except Exception as e:
            self.get_logger().error(f"Error in debug_lidar_status: {e}")

    def debug_lidar_mapping(self):
        """Debug method untuk cek mapping LiDAR secara detail"""
        try:
            if not self.latest_scan:
                self.get_logger().warning("No LiDAR scan data")
                return
                
            scan = self.latest_scan
            
            # Log scan properties
            self.get_logger().info(
                f"🛰️ LiDAR Debug: "
                f"angle_min={math.degrees(scan.angle_min):.1f}°, "
                f"angle_max={math.degrees(scan.angle_max):.1f}°, "
                f"angle_increment={math.degrees(scan.angle_increment):.3f}°, "
                f"ranges={len(scan.ranges)}"
            )
            
            # Sample beberapa ray untuk debugging
            sample_indices = [0, len(scan.ranges)//4, len(scan.ranges)//2, 3*len(scan.ranges)//4, len(scan.ranges)-1]
            for idx in sample_indices:
                if idx < len(scan.ranges):
                    angle = scan.angle_min + idx * scan.angle_increment
                    distance = scan.ranges[idx]
                    self.get_logger().info(
                        f"  Ray[{idx}]: angle={math.degrees(angle):.1f}°, distance={distance:.2f}m"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"Error in debug_lidar_mapping: {e}")

    def debug_lidar_detailed(self):
        """Debug LiDAR data secara detail untuk troubleshooting"""
        try:
            if not self.latest_scan:
                self.get_logger().warning("No LiDAR scan data available")
                return
                
            scan = self.latest_scan
            
            # Log basic info
            valid_ranges = [r for r in scan.ranges if math.isfinite(r) and r > 0]
            self.get_logger().info(
                f"🛰️ LiDAR: {len(valid_ranges)}/{len(scan.ranges)} valid, "
                f"range: {math.degrees(scan.angle_min):.1f}° to {math.degrees(scan.angle_max):.1f}°"
            )
            
            # Sample data at camera angles
            camera_angles = [0, 60, 120, 180, 240, 300]  # Expected camera positions
            for i, angle_deg in enumerate(camera_angles):
                angle_rad = math.radians(angle_deg)
                
                # Normalize to scan range
                if angle_rad > scan.angle_max:
                    angle_rad -= 2 * math.pi
                elif angle_rad < scan.angle_min:
                    angle_rad += 2 * math.pi
                    
                if scan.angle_min <= angle_rad <= scan.angle_max:
                    ray_idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
                    if 0 <= ray_idx < len(scan.ranges):
                        distance = scan.ranges[ray_idx]
                        self.get_logger().info(
                            f"  Camera_{i} direction ({angle_deg}°): "
                            f"ray[{ray_idx}] = {distance:.2f}m"
                        )
        
        except Exception as e:
            self.get_logger().error(f"Error in debug_lidar_detailed: {e}")

    # Add method untuk format output target
    def _format_detection_with_3d(self, detection_msg, camera_id):
        """Format detection dengan Class, Confidence, Distance, Coordinate"""
        try:
            # Get camera direction (0°, 60°, 120°, 180°, 240°, 300°)
            camera_angle = camera_id * 60.0
            
            # Calculate distance from LaserScan
            distance = self._get_distance_from_lidar(camera_angle)
            
            # Calculate 3D coordinate
            coordinate = self._calculate_3d_coordinate(detection_msg, camera_angle, distance)
            
            # Format output string untuk overlay di bounding box
            formatted_output = (
                f"Class={detection_msg.class_name}, "
                f"Confidence={detection_msg.confidence:.2f}, "
                f"Distance: {distance:.2f} m, "
                f"Coordinate: ({coordinate[0]:.2f}, {coordinate[1]:.2f}, {coordinate[2]:.2f})"
            )
            
            return formatted_output, coordinate, distance
            
        except Exception as e:
            self.get_logger().error(f"Error formatting detection: {e}")
            return None, None, None

    def _calculate_3d_coordinate(self, detection_msg, camera_angle, distance):
        """Calculate 3D world coordinate dari bounding box center"""
        try:
            # Simple projection tanpa kalibrasi (kasar tapi works)
            # Assume bounding box center adalah pixel (u, v)
            u = (detection_msg.xmin + detection_msg.xmax) / 2.0
            v = (detection_msg.ymin + detection_msg.ymax) / 2.0
            
            # Convert pixel ke world coordinate (simplified)
            # Asumsi: camera FOV 90°, image center = world center
            angle_offset = (u - 960) * (90.0 / 1920.0)  # 960 = image_width/2
            world_angle = math.radians(camera_angle + angle_offset)
            
            # Calculate 3D position
            x = distance * math.cos(world_angle)
            y = distance * math.sin(world_angle)
            z = 0.5  # Assume object height 0.5m from ground
            
            return [x, y, z]
            
        except Exception as e:
            self.get_logger().error(f"Error calculating 3D coordinate: {e}")
            return [0.0, 0.0, 0.0]

def main(args=None):
    """Main entry point with proper shutdown handling"""
    node = None
    try:
        rclpy.init(args=args)
        node = SimpleFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped by keyboard interrupt")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Warning: Error destroying node: {e}")
        
        # Only shutdown if rclpy is still OK
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Warning: Error in rclpy shutdown: {e}")

if __name__ == '__main__':
    main()