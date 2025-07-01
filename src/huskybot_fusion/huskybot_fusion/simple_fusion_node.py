#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                    # ROS2 Python client library for node creation and management
from rclpy.node import Node                     # Base class for ROS2 nodes
from sensor_msgs.msg import PointCloud2, LaserScan  # Message types for LiDAR data
from yolov12_msgs.msg import Yolov12Inference, InferenceResult  # Custom messages for YOLOv12 detection results
import numpy as np                              # For numerical operations and array handling
import math                                     # For mathematical operations like trigonometry
import time                                     # For timing operations and delays
import threading                                # For thread management and synchronization
import os                                       # For file/path operations
import sys                                      # For system-level operations and error handling
import traceback                                # For detailed exception information
from datetime import datetime                   # For timestamping logs and data
import json                                     # For storing structured data
from pathlib import Path                        # For path manipulation with better error handling

class SimpleFusionNode(Node):
    """
    Node for simple fusion of YOLOv12 detections with LiDAR data.
    Provides 3D positioning of detected objects without requiring complex calibration.
    
    This node integrates with:
    - Velodyne VLP32-C LiDAR for 3D data
    - YOLOv12 detections from hexagonal camera arrangement
    - RViz2 for visualization
    - Gazebo for simulation
    """
    def __init__(self):
        """Initialize the SimpleFusionNode with parameters, subscribers, publishers, and data structures."""
        try:
            # Initialize the ROS2 node with name 'simple_fusion_node'
            super().__init__('simple_fusion_node')
            
            # Set up logging
            self.get_logger().info("Initializing SimpleFusionNode")
            
            # Declare parameters with documentation
            self.declare_parameter('use_calibration', False)        # Whether to use calibration data for more accurate fusion
            self.declare_parameter('max_laser_distance', 100.0)     # Maximum valid distance from LiDAR (meters)
            self.declare_parameter('min_laser_distance', 0.5)       # Minimum valid distance from LiDAR (meters)
            self.declare_parameter('confidence_threshold', 0.25)    # Minimum detection confidence to process
            self.declare_parameter('detection_topics', [            # List of topics containing object detection results
                '/detection',                                       # Primary detection topic from YOLOv12
                '/segmentation'                                     # Optional segmentation topic (if available)
            ])
            self.declare_parameter('laserscan_topic', '/scan')      # Topic for LaserScan data
            self.declare_parameter('pointcloud_topic', '/velodyne_points')  # Topic for PointCloud2 data
            self.declare_parameter('output_topic', '/fusion/objects3d')     # Output topic for fused detections
            self.declare_parameter('marker_topic', '/fusion/markers')       # Topic for visualization markers
            self.declare_parameter('image_width', 640)              # Width of camera images in pixels
            self.declare_parameter('image_height', 480)             # Height of camera images in pixels
            self.declare_parameter('search_radius', 0.5)            # Radius to search for nearby points (meters)
            self.declare_parameter('log_level', 'info')             # Logging level (debug, info, warn, error)
            self.declare_parameter('marker_lifetime', 1)            # Lifetime of visualization markers (seconds)
            self.declare_parameter('publish_rate', 10.0)            # Rate to publish fusion results (Hz)
            self.declare_parameter('log_path', '~/huskybot_fusion_logs')  # Directory to store log files
            self.declare_parameter('use_sim_time', False)           # Whether we're running in simulation
            self.declare_parameter('z_offset', 0.0)                 # Z offset for object visualization (meters)
            
            # Get parameters with validation
            try:
                self.use_calibration = self.get_parameter('use_calibration').value
                self.max_laser_distance = float(self.get_parameter('max_laser_distance').value)
                if self.max_laser_distance <= 0.0:
                    self.get_logger().warn("Invalid max_laser_distance (≤0), using default 100.0")
                    self.max_laser_distance = 100.0
                
                self.min_laser_distance = float(self.get_parameter('min_laser_distance').value)
                if self.min_laser_distance < 0.0:
                    self.get_logger().warn("Invalid min_laser_distance (<0), using default 0.5")
                    self.min_laser_distance = 0.5
                
                self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
                if not 0.0 <= self.confidence_threshold <= 1.0:
                    self.get_logger().warn(f"Invalid confidence_threshold ({self.confidence_threshold}), using default 0.25")
                    self.confidence_threshold = 0.25
                
                self.detection_topics = self.get_parameter('detection_topics').value
                if not self.detection_topics:
                    self.get_logger().error("No detection topics specified")
                    self.detection_topics = ['/detection']  # Default to standard detection topic
                
                self.laserscan_topic = self.get_parameter('laserscan_topic').value
                self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
                self.output_topic = self.get_parameter('output_topic').value
                self.marker_topic = self.get_parameter('marker_topic').value
                
                self.image_width = int(self.get_parameter('image_width').value)
                if self.image_width <= 0:
                    self.get_logger().warn("Invalid image_width (≤0), using default 640")
                    self.image_width = 640
                
                self.image_height = int(self.get_parameter('image_height').value)
                if self.image_height <= 0:
                    self.get_logger().warn("Invalid image_height (≤0), using default 480")
                    self.image_height = 480
                
                self.search_radius = float(self.get_parameter('search_radius').value)
                if self.search_radius <= 0.0:
                    self.get_logger().warn("Invalid search_radius (≤0), using default 0.5")
                    self.search_radius = 0.5
                
                self.marker_lifetime = int(self.get_parameter('marker_lifetime').value)
                if self.marker_lifetime <= 0:
                    self.get_logger().warn("Invalid marker_lifetime (≤0), using default 1")
                    self.marker_lifetime = 1
                
                self.publish_rate = float(self.get_parameter('publish_rate').value)
                if self.publish_rate <= 0.0:
                    self.get_logger().warn("Invalid publish_rate (≤0), using default 10.0")
                    self.publish_rate = 10.0
                
                log_level = self.get_parameter('log_level').value.lower()
                valid_levels = ['debug', 'info', 'warn', 'error']
                if log_level not in valid_levels:
                    self.get_logger().warn(f"Invalid log_level '{log_level}', using default 'info'")
                    log_level = 'info'
                self.log_level = log_level
                
                self.use_sim_time = self.get_parameter('use_sim_time').value
                self.z_offset = float(self.get_parameter('z_offset').value)
                
                # Log path management
                try:
                    log_path = self.get_parameter('log_path').value
                    self.log_path = os.path.expanduser(log_path)
                    os.makedirs(self.log_path, exist_ok=True)  # Create log directory if it doesn't exist
                    
                    # Create statistics log file
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    self.stats_file = os.path.join(self.log_path, f"fusion_stats_{timestamp}.json")
                    self.get_logger().info(f"Logging statistics to {self.stats_file}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to setup log directory: {e}")
                    self.stats_file = None
                
                # Log parameters
                param_str = (f"Parameters:\n"
                            f"  use_calibration: {self.use_calibration}\n"
                            f"  max_laser_distance: {self.max_laser_distance}\n"
                            f"  min_laser_distance: {self.min_laser_distance}\n"
                            f"  confidence_threshold: {self.confidence_threshold}\n"
                            f"  detection_topics: {self.detection_topics}\n"
                            f"  laserscan_topic: {self.laserscan_topic}\n"
                            f"  pointcloud_topic: {self.pointcloud_topic}\n"
                            f"  output_topic: {self.output_topic}\n"
                            f"  marker_topic: {self.marker_topic}\n"
                            f"  image dimensions: {self.image_width}x{self.image_height}\n"
                            f"  search_radius: {self.search_radius}\n"
                            f"  marker_lifetime: {self.marker_lifetime}\n"
                            f"  publish_rate: {self.publish_rate}\n"
                            f"  log_level: {self.log_level}\n"
                            f"  use_sim_time: {self.use_sim_time}\n"
                            f"  z_offset: {self.z_offset}")
                self.get_logger().info(param_str)
            
            except Exception as e:
                self.get_logger().error(f"Error getting parameters: {str(e)}\n{traceback.format_exc()}")
                # Use defaults for critical parameters to continue operation
                self.max_laser_distance = 100.0
                self.min_laser_distance = 0.5
                self.confidence_threshold = 0.25
                self.search_radius = 0.5
                self.detection_topics = ['/detection']
                self.stats_file = None
            
            # Initialize data holders with thread safety
            self.latest_scan = None            # Latest LaserScan message
            self.latest_cloud = None           # Latest PointCloud2 message
            self.latest_detections = {}        # Dictionary of latest detections by camera name
            self.lock = threading.RLock()      # Reentrant lock for thread safety
            
            # Statistics tracking
            self.stats = {
                'processed_detections': 0,
                'successful_fusions': 0,
                'failed_fusions': 0,
                'by_camera': {},
                'by_class': {},
                'start_time': time.time()
            }
            
            # Camera field of view angles (approx) - Updated for hexagonal arrangement
            # These angles map each camera's horizontal FOV to global coordinates in degrees
            # Format: {camera_name: (min_angle, max_angle)}
            self.camera_fov = {
                'camera_front': (-30, 30),            # Forward-facing camera
                'camera_front_left': (-90, -30),      # Front-left camera
                'camera_left': (-150, -90),           # Left-side camera
                'camera_rear': (150, -150),           # Rear-facing camera
                'camera_rear_right': (90, 150),       # Rear-right camera
                'camera_right': (30, 90)              # Right-side camera
            }
            
            # Subscribe to LaserScan for distance measurements
            self.scan_sub = self.create_subscription(
                LaserScan,                      # Message type
                self.laserscan_topic,           # Topic name
                self.laserscan_callback,        # Callback function
                10                              # Queue size
            )
            self.get_logger().debug(f"Subscribed to LaserScan topic: {self.laserscan_topic}")
            
            # Subscribe to PointCloud2 for 3D position data
            self.cloud_sub = self.create_subscription(
                PointCloud2,                    # Message type
                self.pointcloud_topic,          # Topic name
                self.pointcloud_callback,       # Callback function
                10                              # Queue size
            )
            self.get_logger().debug(f"Subscribed to PointCloud2 topic: {self.pointcloud_topic}")
            
            # Subscribe to detection topics from all cameras
            self.detection_subs = []
            for topic in self.detection_topics:
                try:
                    self.detection_subs.append(
                        self.create_subscription(
                            Yolov12Inference,       # Message type
                            topic,                  # Topic name
                            self.detection_callback, # Callback function
                            10                      # Queue size
                        )
                    )
                    self.get_logger().debug(f"Subscribed to detection topic: {topic}")
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to topic {topic}: {str(e)}")
            
            # Publishers for results and visualization
            self.result_pub = self.create_publisher(
                Yolov12Inference,              # Message type
                self.output_topic,             # Topic name
                10                             # Queue size
            )
            self.get_logger().debug(f"Created publisher for topic: {self.output_topic}")
            
            self.marker_pub = self.create_publisher(
                MarkerArray,                   # Message type
                self.marker_topic,             # Topic name
                10                             # Queue size
            )
            self.get_logger().debug(f"Created publisher for topic: {self.marker_topic}")
            
            # Timer for periodic fusion processing
            timer_period = 1.0 / self.publish_rate  # Convert Hz to seconds
            self.timer = self.create_timer(
                timer_period,                  # Period in seconds
                self.process_fusion            # Callback function
            )
            self.get_logger().debug(f"Created fusion timer with period: {timer_period:.3f} seconds")
            
            # Add stats timer (every 60 seconds)
            self.stats_timer = self.create_timer(
                60.0,                          # Run every 60 seconds
                self.log_statistics            # Callback for statistics logging
            )
            
            # Setup complete
            self.get_logger().info('Simple Fusion Node initialized successfully')
        
        except Exception as e:
            self.get_logger().error(f"Error during initialization: {str(e)}\n{traceback.format_exc()}")
            # Re-raise to ensure node fails to start if critical initialization fails
            raise
    
    def log_statistics(self):
        """Log statistics about fusion performance periodically"""
        try:
            # Calculate runtime
            runtime = time.time() - self.stats['start_time']
            
            # Calculate rates
            processed_per_sec = self.stats['processed_detections'] / runtime if runtime > 0 else 0
            success_rate = (self.stats['successful_fusions'] / self.stats['processed_detections'] * 100
                            if self.stats['processed_detections'] > 0 else 0)
            
            # Log to console
            self.get_logger().info(
                f"Fusion statistics: {self.stats['processed_detections']} detections processed, "
                f"{self.stats['successful_fusions']} successful fusions ({success_rate:.1f}%), "
                f"processing {processed_per_sec:.1f} detections/sec"
            )
            
            # Log top classes
            if self.stats['by_class']:
                top_classes = sorted(self.stats['by_class'].items(), key=lambda x: x[1], reverse=True)[:5]
                class_str = ", ".join([f"{cls}: {count}" for cls, count in top_classes])
                self.get_logger().info(f"Top classes: {class_str}")
            
            # Write to JSON file
            if self.stats_file:
                try:
                    with open(self.stats_file, 'w') as f:
                        json.dump({
                            'timestamp': datetime.now().isoformat(),
                            'runtime_seconds': runtime,
                            'stats': self.stats
                        }, f, indent=2)
                except Exception as e:
                    self.get_logger().warn(f"Failed to write statistics to file: {e}")
        
        except Exception as e:
            self.get_logger().error(f"Error in log_statistics: {str(e)}")
    
    def laserscan_callback(self, msg):
        """
        Process incoming LaserScan messages.
        
        Args:
            msg (LaserScan): The LaserScan message from the LiDAR
        """
        try:
            if msg is None:
                self.get_logger().warn("Received None LaserScan message")
                return
                
            # Validate LaserScan message
            if len(msg.ranges) == 0:
                self.get_logger().warn("Received empty LaserScan ranges")
                return
                
            # Check for NaN or Inf values
            valid_ranges = [r for r in msg.ranges if math.isfinite(r)]
            if len(valid_ranges) < len(msg.ranges) * 0.5:  # If less than 50% valid
                self.get_logger().warn(f"LaserScan has {len(msg.ranges) - len(valid_ranges)} non-finite values out of {len(msg.ranges)}")
            
            # Store the message with thread safety
            with self.lock:
                self.latest_scan = msg
                
            if self.log_level == 'debug':
                self.get_logger().debug(f"Received LaserScan: frame_id={msg.header.frame_id}, size={len(msg.ranges)}")
        
        except Exception as e:
            self.get_logger().error(f"Error in laserscan_callback: {str(e)}\n{traceback.format_exc()}")
    
    def pointcloud_callback(self, msg):
        """
        Process incoming PointCloud2 messages.
        
        Args:
            msg (PointCloud2): The PointCloud2 message from the LiDAR
        """
        try:
            if msg is None:
                self.get_logger().warn("Received None PointCloud2 message")
                return
                
            # Validate PointCloud2 message
            if msg.width * msg.height == 0 or len(msg.data) == 0:
                self.get_logger().warn("Received empty PointCloud2")
                return
                
            # Store the message with thread safety
            with self.lock:
                self.latest_cloud = msg
                
            if self.log_level == 'debug':
                self.get_logger().debug(f"Received PointCloud2: frame_id={msg.header.frame_id}, size={msg.width}x{msg.height}")
        
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud_callback: {str(e)}\n{traceback.format_exc()}")
    
    def detection_callback(self, msg):
        """
        Process incoming YOLOv12 detection messages.
        
        Args:
            msg (Yolov12Inference): The YOLOv12 detection message
        """
        try:
            if msg is None:
                self.get_logger().warn("Received None detection message")
                return
                
            # Validate detection message
            if not msg.camera_name:
                self.get_logger().warn("Detection message missing camera_name")
                return
                
            # Update statistics
            with self.lock:
                # Update camera stats
                if msg.camera_name not in self.stats['by_camera']:
                    self.stats['by_camera'][msg.camera_name] = 0
                self.stats['by_camera'][msg.camera_name] += len(msg.yolov12_inference)
                
                # Update class stats
                for det in msg.yolov12_inference:
                    class_name = det.class_name
                    if class_name not in self.stats['by_class']:
                        self.stats['by_class'][class_name] = 0
                    self.stats['by_class'][class_name] += 1
                
                # Store the message with thread safety
                self.latest_detections[msg.camera_name] = msg
                
            if self.log_level == 'debug':
                self.get_logger().debug(
                    f"Received detections for camera '{msg.camera_name}': "
                    f"task='{msg.task}', count={len(msg.yolov12_inference)}"
                )
        
        except Exception as e:
            self.get_logger().error(f"Error in detection_callback: {str(e)}\n{traceback.format_exc()}")
    
    def get_distance_from_laserscan(self, camera_name, obj_center_angle):
        """
        Get distance from LaserScan based on camera name and object angle.
        
        Args:
            camera_name (str): Name of the camera that detected the object
            obj_center_angle (float): Normalized horizontal position (0-1) of the object in the camera image
            
        Returns:
            float: Distance to the object in meters, or None if unavailable/invalid
        """
        try:
            if self.latest_scan is None:
                if self.log_level == 'debug':
                    self.get_logger().debug("No LaserScan data available")
                return None
            
            scan = self.latest_scan
            
            # Check that we have camera FOV data for this camera
            if camera_name not in self.camera_fov:
                self.get_logger().warn(f"No FOV data for camera '{camera_name}'")
                return None
                
            # Get camera FOV angles
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
                self.get_logger().warn(f"Calculated ray index {ray_idx} is out of bounds [0, {len(scan.ranges)-1}]")
                return None
                
            # Get distance
            distance = scan.ranges[ray_idx]
            
            # Check if distance is valid
            if (distance < scan.range_min or 
                distance > scan.range_max or 
                distance > self.max_laser_distance or
                distance < self.min_laser_distance or
                not math.isfinite(distance)):
                
                if self.log_level == 'debug':
                    self.get_logger().debug(
                        f"Invalid distance: {distance} (range: {scan.range_min}-{scan.range_max}, "
                        f"limits: {self.min_laser_distance}-{self.max_laser_distance})"
                    )
                return None
                
            return distance
            
        except Exception as e:
            self.get_logger().error(f"Error in get_distance_from_laserscan: {str(e)}\n{traceback.format_exc()}")
            return None
    
    def get_coordinates_from_pointcloud(self, camera_name, obj_center_angle, distance):
        """
        Get 3D coordinates from PointCloud based on camera, angle, and distance.
        
        Args:
            camera_name (str): Name of the camera that detected the object
            obj_center_angle (float): Normalized horizontal position (0-1) of the object in the camera image
            distance (float): Distance to the object in meters
            
        Returns:
            tuple: (x, y, z) coordinates in meters, or None if unavailable/invalid
        """
        try:
            if self.latest_cloud is None:
                self.get_logger().debug("No PointCloud data available")
                return None
                
            if distance is None:
                self.get_logger().debug("No valid distance available")
                return None
            
            # Check that we have camera FOV data for this camera
            if camera_name not in self.camera_fov:
                self.get_logger().warn(f"No FOV data for camera '{camera_name}'")
                return None
                
            # Convert the angle to global coordinates (same as in get_distance_from_laserscan)
            cam_min_angle, cam_max_angle = self.camera_fov[camera_name]
            global_angle = cam_min_angle + obj_center_angle * (cam_max_angle - cam_min_angle)
            lidar_angle = math.radians(global_angle)
            
            # Simple trigonometry to get XYZ
            # For a more accurate approach, proper camera-LiDAR calibration would be used
            x = distance * math.cos(lidar_angle)
            y = distance * math.sin(lidar_angle)
            z = 0.0 + self.z_offset  # Start with ground level + any configured offset
            
            # For better Z accuracy, search nearby points in the point cloud
            try:
                # Extract point cloud data within our search radius of the X,Y position
                points_list = []
                for point in pc2.read_points(
                    self.latest_cloud,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ):
                    # Calculate distance from our X,Y to this point's X,Y
                    dx = point[0] - x
                    dy = point[1] - y
                    dist_xy = math.sqrt(dx*dx + dy*dy)
                    
                    # If within search radius, add to our list
                    if dist_xy <= self.search_radius:
                        points_list.append(point)
                
                # If we found points, use their Z values
                if points_list:
                    # Get list of Z values
                    z_values = [p[2] for p in points_list]
                    
                    # Filter outliers (Z values far from the median)
                    if len(z_values) >= 3:  # Need at least 3 points for meaningful statistics
                        z_median = sorted(z_values)[len(z_values) // 2]  # Simple median calculation
                        z_filtered = [z for z in z_values if abs(z - z_median) < 1.0]  # Filter values within 1m of median
                        
                        if z_filtered:  # If we still have points after filtering
                            # Use average of filtered Z values
                            z = sum(z_filtered) / len(z_filtered) + self.z_offset
                    else:
                        # Just use average if too few points
                        z = sum(z_values) / len(z_values) + self.z_offset
                        
                    if self.log_level == 'debug':
                        self.get_logger().debug(f"Found {len(points_list)} points near ({x:.2f}, {y:.2f}), estimated z={z:.2f}")
                else:
                    if self.log_level == 'debug':
                        self.get_logger().debug(f"No points found within {self.search_radius}m of ({x:.2f}, {y:.2f})")
            
            except Exception as e:
                self.get_logger().warn(f"Error finding Z coordinate: {str(e)}")
                # Continue with default Z = 0.0 + offset
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f"Error in get_coordinates_from_pointcloud: {str(e)}\n{traceback.format_exc()}")
            return None
    
    def process_fusion(self):
        """
        Process fusion of detections with LiDAR data.
        This method runs periodically based on the timer.
        """
        try:
            # Use lock for thread safety during entire fusion process
            with self.lock:
                # Skip if we don't have both LiDAR data types
                if not self.latest_scan or not self.latest_cloud:
                    if self.log_level == 'debug':
                        self.get_logger().debug("Missing scan or pointcloud data, skipping fusion")
                    return
                
                # Skip if no detections available
                if not self.latest_detections:
                    if self.log_level == 'debug':
                        self.get_logger().debug("No detections available, skipping fusion")
                    return
                
                marker_array = MarkerArray()
                marker_id = 0
                
                # Process each camera's detections
                for camera_name, detections in self.latest_detections.items():
                    # Create output message
                    output_msg = Yolov12Inference()
                    output_msg.header = detections.header
                    output_msg.camera_name = camera_name
                    output_msg.frame_type = "fused"
                    output_msg.task = "fusion"
                    output_msg.note = "simple fusion without calibration"
                    
                    # Process each detection
                    for det in detections.yolov12_inference:
                        # Track statistics
                        self.stats['processed_detections'] += 1
                        
                        # Skip low confidence detections
                        if det.confidence < self.confidence_threshold:
                            continue
                        
                        # Calculate object center angle (0-1 range)
                        # For simplicity, we just use the horizontal center of bbox
                        obj_center_ratio = (det.left + det.right) / 2.0 / float(self.image_width)
                        
                        # Get distance from LaserScan
                        distance = self.get_distance_from_laserscan(camera_name, obj_center_ratio)
                        
                        # Get coordinates from PointCloud if we have a valid distance
                        coords = None
                        if distance is not None:
                            coords = self.get_coordinates_from_pointcloud(camera_name, obj_center_ratio, distance)
                        
                        # Create enriched result
                        result = InferenceResult()
                        result.class_name = det.class_name
                        result.confidence = det.confidence
                        result.top = det.top
                        result.left = det.left
                        result.bottom = det.bottom
                        result.right = det.right
                        
                        # Add distance and coordinates to notes if available
                        if distance is not None:
                            result.note = f"Distance: {distance:.2f}m"
                            self.stats['successful_fusions'] += 1
                            
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
                                
                                # Set size based on class
                                if "person" in det.class_name.lower():
                                    marker.scale.x = 0.5
                                    marker.scale.y = 0.5
                                    marker.scale.z = 1.8
                                elif "car" in det.class_name.lower() or "vehicle" in det.class_name.lower():
                                    marker.scale.x = 2.0
                                    marker.scale.y = 1.0
                                    marker.scale.z = 1.5
                                else:
                                    marker.scale.x = 0.5
                                    marker.scale.y = 0.5
                                    marker.scale.z = 0.5
                                
                                # Set color based on class
                                marker.color.r = 1.0 if "person" in det.class_name.lower() else 0.0
                                marker.color.g = 0.0 if "person" in det.class_name.lower() else 1.0
                                marker.color.b = 0.0
                                marker.color.a = 0.7
                                marker.lifetime.sec = self.marker_lifetime
                                marker_array.markers.append(marker)
                                
                                # Add text marker
                                text_marker = Marker()
                                text_marker.header = detections.header
                                text_marker.ns = "object_labels"
                                text_marker.id = marker_id
                                marker_id += 1
                                text_marker.type = Marker.TEXT_VIEW_FACING
                                text_marker.action = Marker.ADD
                                text_marker.pose.position.x = coords[0]
                                text_marker.pose.position.y = coords[1]
                                text_marker.pose.position.z = coords[2] + marker.scale.z/2 + 0.2
                                text_marker.text = f"{det.class_name}\n{det.confidence:.2f}\n{distance:.2f}m"
                                text_marker.scale.z = 0.3
                                text_marker.color.r = 1.0
                                text_marker.color.g = 1.0
                                text_marker.color.b = 1.0
                                text_marker.color.a = 1.0
                                text_marker.lifetime.sec = self.marker_lifetime
                                marker_array.markers.append(text_marker)
                        else:
                            # Count as a failed fusion if we couldn't get distance
                            self.stats['failed_fusions'] += 1
                    
                        output_msg.yolov12_inference.append(result)
                    
                    # Publish results if we have any
                    if output_msg.yolov12_inference:
                        try:
                            self.result_pub.publish(output_msg)
                            if self.log_level == 'debug':
                                self.get_logger().debug(
                                    f"Published fusion results for camera '{camera_name}': {len(output_msg.yolov12_inference)} objects"
                                )
                        except Exception as e:
                            self.get_logger().error(f"Error publishing fusion results: {str(e)}")
                
                # Publish markers for visualization if we have any
                if marker_array.markers:
                    try:
                        self.marker_pub.publish(marker_array)
                        if self.log_level == 'debug':
                            self.get_logger().debug(f"Published {len(marker_array.markers)} markers")
                    except Exception as e:
                        self.get_logger().error(f"Error publishing markers: {str(e)}")
        
        except Exception as e:
            self.get_logger().error(f"Error in process_fusion: {str(e)}\n{traceback.format_exc()}")
    
    def create_marker(self, header, marker_id, coords, class_name, confidence, distance=None):
        """
        Create a visualization marker for RViz2.
        
        Args:
            header (Header): Header from the detection message
            marker_id (int): Unique ID for this marker
            coords (tuple): (x, y, z) coordinates for the marker
            class_name (str): Class name of the detected object
            confidence (float): Confidence score of the detection
            distance (float, optional): Distance to the object
            
        Returns:
            Marker: Visualization marker for RViz2
        """
        try:
            marker = Marker()
            marker.header = header
            marker.ns = "object_markers"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = coords[0]
            marker.pose.position.y = coords[1]
            marker.pose.position.z = coords[2]
            marker.pose.orientation.w = 1.0  # Default orientation (no rotation)
            
            # Set scale based on object class (approximate size)
            if "person" in class_name.lower():
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 1.8
            elif "car" in class_name.lower() or "vehicle" in class_name.lower():
                marker.scale.x = 2.0
                marker.scale.y = 1.0
                marker.scale.z = 1.5
            else:
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            
            # Set color based on class (red for people, green for others)
            marker.color.r = 1.0 if "person" in class_name.lower() else 0.0
            marker.color.g = 0.0 if "person" in class_name.lower() else 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7  # Semi-transparent
            
            # Set marker lifetime
            marker.lifetime.sec = self.marker_lifetime
            
            # Prepare text for distance information
            distance_text = f"\n{distance:.2f}m" if distance is not None else ""
            
            # Add text marker with class name and confidence
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "object_labels"
            text_marker.id = marker_id + 10000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = coords[0]
            text_marker.pose.position.y = coords[1]
            text_marker.pose.position.z = coords[2] + marker.scale.z/2 + 0.2  # Position above the object
            text_marker.text = f"{class_name}\n{confidence:.2f}{distance_text}"
            text_marker.scale.z = 0.3  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = self.marker_lifetime
            
            return marker, text_marker
            
        except Exception as e:
            self.get_logger().error(f"Error creating marker: {str(e)}")
            return None, None
    
    def shutdown(self):
        """Perform clean shutdown operations."""
        try:
            self.get_logger().info("Shutting down SimpleFusionNode")
            
            # Log final statistics
            runtime = time.time() - self.stats['start_time']
            self.get_logger().info(
                f"Final statistics - Runtime: {runtime:.1f}s, "
                f"Processed: {self.stats['processed_detections']}, "
                f"Success rate: {(self.stats['successful_fusions'] / max(1, self.stats['processed_detections']) * 100):.1f}%"
            )
            
            # Write final stats to file
            if self.stats_file:
                try:
                    with open(self.stats_file, 'w') as f:
                        json.dump({
                            'final_timestamp': datetime.now().isoformat(),
                            'runtime_seconds': runtime,
                            'stats': self.stats
                        }, f, indent=2)
                    self.get_logger().info(f"Final statistics written to {self.stats_file}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to write final statistics: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")


def main(args=None):
    """Main entry point for the node."""
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create and run the node
        node = SimpleFusionNode()
        
        try:
            # Spin the node to execute callbacks
            rclpy.spin(node)
        except KeyboardInterrupt:
            # Handle graceful shutdown on Ctrl+C
            print("Node stopped by keyboard interrupt")
        except Exception as e:
            # Handle other exceptions
            print(f"Error while spinning node: {str(e)}")
            print(traceback.format_exc())
        finally:
            # Clean shutdown
            try:
                node.shutdown()  # Perform clean shutdown operations
            except Exception as e:
                print(f"Error during node shutdown: {str(e)}")
                
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"Error in main: {str(e)}")
        print(traceback.format_exc())
        # Ensure ROS2 is properly shut down
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()