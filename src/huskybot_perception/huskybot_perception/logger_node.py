#!/usr/bin/env python3  # Shebang line for Python3 (required for ROS2 nodes)
# -*- coding: utf-8 -*-  # Encoding declaration for proper Unicode handling

"""
MultiTask Logger Node for Huskybot

This node logs results from multiple YOLOv12 tasks (detection, segmentation, OBB, tracking)
to a CSV file for later analysis, debugging, and audit trail purposes. It supports
customization via ROS2 parameters and implements robust error handling.

Author: Jezzy Putra Munggaran
License: Apache-2.0
"""

import rclpy  # Import ROS2 Python client library
from rclpy.node import Node  # Import base class for ROS2 nodes
from rclpy.parameter import Parameter  # Import Parameter class for typed parameters
from rclpy.exceptions import ParameterNotDeclaredException  # Import exception for parameter errors
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # Import QoS settings for subscriptions
from yolov12_msgs.msg import Yolov12Inference  # Import custom message for YOLOv12 results
import csv  # Import CSV module for file writing
import os  # Import OS module for file/path operations
import sys  # Import sys module for exit handling
import threading  # Import threading for thread-safety mechanisms
import time  # Import time for timestamps
import traceback  # Import traceback for detailed error reporting
from datetime import datetime  # Import datetime for human-readable timestamps
import logging  # Import logging for file-based logs independent of ROS
import signal  # Import signal for handling system signals

# Set up logging for tracing errors independently of ROS2 logs
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    filename=os.path.expanduser('~/huskybot_logs/logger_node.log'),
    filemode='a'
)
logger = logging.getLogger('huskybot_logger')  # Create named logger instance

class MultiTaskLogger(Node):
    """
    ROS2 Node for logging YOLOv12 multitask inference results.
    
    Subscribes to multiple topics (/detection, /segmentation, /obb, /tracking)
    and logs the results to a CSV file for later analysis and debugging.
    Implements thread safety, error handling, and file rotation.
    """
    
    def __init__(self):
        """
        Initialize the MultiTaskLogger node.
        
        Sets up parameters, file handling, and subscriptions.
        Implements comprehensive error handling for all initialization steps.
        """
        super().__init__('multitask_logger')  # Initialize the ROS2 node with name 'multitask_logger'
        
        # Catch SIGINT (Ctrl+C) and SIGTERM for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info("Starting MultiTaskLogger Node")  # Log startup to ROS
        logger.info("Starting MultiTaskLogger Node")  # Log startup to file
        
        # Declare all parameters with documentation
        self.declare_parameters(
            namespace='',
            parameters=[
                ('log_dir', Parameter.Type.STRING, os.path.expanduser('~/huskybot_logs')),  # Directory for log files
                ('log_file', Parameter.Type.STRING, 'multitask_log.csv'),  # Log filename
                ('max_log_size', Parameter.Type.INTEGER, 10*1024*1024),  # Max log size before rotation (10MB default)
                ('topics', Parameter.Type.STRING_ARRAY, 
                 ['/detection', '/segmentation', '/obb', '/tracking']),  # Topics to monitor
                ('log_level', Parameter.Type.STRING, 'info'),  # Log level (debug, info, warn, error)
                ('class_filter', Parameter.Type.STRING, ''),  # Optional class filter (empty = log all)
                ('min_confidence', Parameter.Type.DOUBLE, 0.0),  # Minimum confidence to log
                ('check_file_interval', Parameter.Type.DOUBLE, 60.0),  # How often to check file size (seconds)
                ('max_rotation_files', Parameter.Type.INTEGER, 10),  # Maximum number of rotated files to keep
                ('inactive_topic_timeout', Parameter.Type.DOUBLE, 300.0),  # Timeout to warn about inactive topics (seconds)
                ('log_rotation_policy', Parameter.Type.STRING, 'size'),  # Rotation policy: 'size', 'time', or 'both'
                ('log_rotation_time_hours', Parameter.Type.INTEGER, 24),  # Rotate files every N hours if policy includes 'time'
                ('flush_interval', Parameter.Type.INTEGER, 10),  # Flush to disk every N messages
                ('enable_extended_stats', Parameter.Type.BOOL, True),  # Enable extended statistics
            ]
        )
        
        # Get parameters with error handling
        try:
            self._get_parameters()  # Get all parameters in a separate method for better organization
            
            # Log startup information
            self._log_startup_info()  # Log startup info in a separate method
            
            # Ensure log directory exists with proper permissions
            self._create_log_directory()  # Create log directory in a separate method
            
            # Thread safety lock for file operations
            self.lock = threading.Lock()
            
            # Initialize statistics tracking
            self._initialize_stats()  # Initialize statistics in a separate method
            
            # Open the log file and initialize CSV writer
            self._open_log_file()  # Open log file in a separate method
            
        except Exception as e:
            self.get_logger().error(f"Error initializing MultiTaskLogger: {e}\n{traceback.format_exc()}")
            logger.error(f"Error initializing MultiTaskLogger: {e}\n{traceback.format_exc()}")
            # Cannot continue if initialization fails
            sys.exit(1)
        
        # Create timer for periodic file size check and statistics reporting
        self._create_timers()  # Create timers in a separate method
        
        # Create subscriptions to all specified topics
        self._subscribe_to_topics()  # Subscribe to topics in a separate method
        
        # Log successful initialization
        self.get_logger().info("MultiTaskLogger node initialized successfully")
        logger.info("MultiTaskLogger node initialized successfully")
    
    def _signal_handler(self, sig, frame):
        """
        Handle system signals for graceful shutdown.
        
        Args:
            sig: Signal number
            frame: Current stack frame
        """
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully")
        logger.info(f"Received signal {sig}, shutting down gracefully")
        self.destroy_node()  # Clean up resources
        rclpy.shutdown()  # Shutdown ROS2
        sys.exit(0)  # Exit cleanly
    
    def _get_parameters(self):
        """
        Get and validate all parameters with error handling.
        """
        try:
            # Get all parameters with proper type conversion
            self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self.log_file = self.get_parameter('log_file').get_parameter_value().string_value
            self.max_log_size = self.get_parameter('max_log_size').get_parameter_value().integer_value
            self.topics = self.get_parameter('topics').get_parameter_value().string_array_value
            self.log_level = self.get_parameter('log_level').get_parameter_value().string_value.lower()
            self.class_filter = self.get_parameter('class_filter').get_parameter_value().string_value.strip().lower()
            self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
            self.check_file_interval = self.get_parameter('check_file_interval').get_parameter_value().double_value
            self.max_rotation_files = self.get_parameter('max_rotation_files').get_parameter_value().integer_value
            self.inactive_topic_timeout = self.get_parameter('inactive_topic_timeout').get_parameter_value().double_value
            self.log_rotation_policy = self.get_parameter('log_rotation_policy').get_parameter_value().string_value.lower()
            self.log_rotation_time_hours = self.get_parameter('log_rotation_time_hours').get_parameter_value().integer_value
            self.flush_interval = self.get_parameter('flush_interval').get_parameter_value().integer_value
            self.enable_extended_stats = self.get_parameter('enable_extended_stats').get_parameter_value().bool_value
            
            # Construct full log path
            self.log_path = os.path.join(self.log_dir, self.log_file)
            
            # Basic parameter validation
            self._validate_parameters()
            
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f"Required parameter not found: {e}")
            logger.error(f"Required parameter not found: {e}")
            raise  # Re-raise as this is fatal
        except Exception as e:
            self.get_logger().error(f"Error getting parameters: {e}")
            logger.error(f"Error getting parameters: {e}")
            raise  # Re-raise as this is fatal
    
    def _validate_parameters(self):
        """
        Validate parameters and apply corrections if needed.
        """
        # Validate max_log_size
        if self.max_log_size <= 0:
            self.get_logger().warn("Invalid max_log_size, setting to default (10MB)")
            logger.warn("Invalid max_log_size, setting to default (10MB)")
            self.max_log_size = 10 * 1024 * 1024
        
        # Validate min_confidence
        if self.min_confidence < 0 or self.min_confidence > 1:
            self.get_logger().warn(f"Invalid min_confidence ({self.min_confidence}), setting to default (0.0)")
            logger.warn(f"Invalid min_confidence ({self.min_confidence}), setting to default (0.0)")
            self.min_confidence = 0.0
        
        # Validate log_level
        valid_log_levels = ['debug', 'info', 'warn', 'error', 'fatal']
        if self.log_level not in valid_log_levels:
            self.get_logger().warn(f"Invalid log_level ({self.log_level}), setting to default (info)")
            logger.warn(f"Invalid log_level ({self.log_level}), setting to default (info)")
            self.log_level = 'info'
        
        # Validate check_file_interval
        if self.check_file_interval <= 0:
            self.get_logger().warn(f"Invalid check_file_interval ({self.check_file_interval}), setting to default (60s)")
            logger.warn(f"Invalid check_file_interval ({self.check_file_interval}), setting to default (60s)")
            self.check_file_interval = 60.0
        
        # Validate max_rotation_files
        if self.max_rotation_files <= 0:
            self.get_logger().warn(f"Invalid max_rotation_files ({self.max_rotation_files}), setting to default (10)")
            logger.warn(f"Invalid max_rotation_files ({self.max_rotation_files}), setting to default (10)")
            self.max_rotation_files = 10
        
        # Validate log_rotation_policy
        valid_policies = ['size', 'time', 'both']
        if self.log_rotation_policy not in valid_policies:
            self.get_logger().warn(f"Invalid log_rotation_policy ({self.log_rotation_policy}), setting to default (size)")
            logger.warn(f"Invalid log_rotation_policy ({self.log_rotation_policy}), setting to default (size)")
            self.log_rotation_policy = 'size'
        
        # Validate log_rotation_time_hours
        if self.log_rotation_time_hours <= 0:
            self.get_logger().warn(f"Invalid log_rotation_time_hours ({self.log_rotation_time_hours}), setting to default (24)")
            logger.warn(f"Invalid log_rotation_time_hours ({self.log_rotation_time_hours}), setting to default (24)")
            self.log_rotation_time_hours = 24
        
        # Validate flush_interval
        if self.flush_interval <= 0:
            self.get_logger().warn(f"Invalid flush_interval ({self.flush_interval}), setting to default (10)")
            logger.warn(f"Invalid flush_interval ({self.flush_interval}), setting to default (10)")
            self.flush_interval = 10
    
    def _log_startup_info(self):
        """
        Log startup information including parameter values.
        """
        self.get_logger().info(f"MultiTaskLogger starting, will log to: {self.log_path}")
        self.get_logger().info(f"Monitoring topics: {', '.join(self.topics)}")
        logger.info(f"MultiTaskLogger starting, will log to: {self.log_path}")
        logger.info(f"Monitoring topics: {', '.join(self.topics)}")
        
        # Log additional parameter information
        if self.class_filter:
            self.get_logger().info(f"Filtering for classes: {self.class_filter}")
            logger.info(f"Filtering for classes: {self.class_filter}")
            
        if self.min_confidence > 0:
            self.get_logger().info(f"Minimum confidence threshold: {self.min_confidence}")
            logger.info(f"Minimum confidence threshold: {self.min_confidence}")
        
        # Log rotation settings
        self.get_logger().info(f"Log rotation policy: {self.log_rotation_policy} "
                             f"(Size: {self.max_log_size/1024/1024:.1f}MB, Time: {self.log_rotation_time_hours}h)")
        logger.info(f"Log rotation policy: {self.log_rotation_policy} "
                   f"(Size: {self.max_log_size/1024/1024:.1f}MB, Time: {self.log_rotation_time_hours}h)")
    
    def _create_log_directory(self):
        """
        Create log directory with proper error handling.
        """
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            # Test write permissions by creating and removing a test file
            test_file = os.path.join(self.log_dir, ".test_write_permission")
            with open(test_file, 'w') as f:
                f.write("test")
            os.remove(test_file)
            self.get_logger().info(f"Log directory ready: {self.log_dir}")
        except PermissionError as e:
            self.get_logger().error(f"Permission error with log directory: {e}")
            logger.error(f"Permission error with log directory: {e}")
            # Try to use alternative directory
            alt_dir = os.path.join("/tmp", "huskybot_logs")
            self.get_logger().warn(f"Attempting to use alternative log directory: {alt_dir}")
            logger.warn(f"Attempting to use alternative log directory: {alt_dir}")
            try:
                os.makedirs(alt_dir, exist_ok=True)
                self.log_dir = alt_dir
                self.log_path = os.path.join(self.log_dir, self.log_file)
                self.get_logger().info(f"Using alternative log directory: {self.log_dir}")
                logger.info(f"Using alternative log directory: {self.log_dir}")
            except Exception as e2:
                self.get_logger().error(f"Error creating alternative log directory: {e2}")
                logger.error(f"Error creating alternative log directory: {e2}")
                raise  # Re-raise as this is fatal
        except Exception as e:
            self.get_logger().error(f"Error creating log directory: {e}\n{traceback.format_exc()}")
            logger.error(f"Error creating log directory: {e}\n{traceback.format_exc()}")
            raise  # Re-raise as this is fatal
    
    def _initialize_stats(self):
        """
        Initialize statistics tracking structures.
        """
        self.stats = {
            'start_time': time.time(),
            'total_messages': 0,
            'total_detections': 0,
            'by_topic': {topic: 0 for topic in self.topics},
            'by_class': {},
            'by_camera': {},
            'errors': 0,
            'last_status_print': time.time(),
            'message_rates': {topic: {'count': 0, 'last_time': time.time()} for topic in self.topics},
            'topic_last_seen': {topic: 0 for topic in self.topics},
            'rotations': 0,
            'bytes_written': 0,
            'processing_times': []
        }
        
        # Add extended statistics if enabled
        if self.enable_extended_stats:
            self.stats.update({
                'max_detections_per_message': 0,
                'min_detections_per_message': float('inf'),
                'avg_detections_per_message': 0,
                'confidence_distribution': {
                    '0.0-0.2': 0,
                    '0.2-0.4': 0,
                    '0.4-0.6': 0,
                    '0.6-0.8': 0,
                    '0.8-1.0': 0
                },
                'processing_errors': {
                    'message_parse': 0,
                    'file_write': 0,
                    'attribute_error': 0,
                    'other': 0
                }
            })
    
    def _create_timers(self):
        """
        Create all periodic timers for the node.
        """
        # Timer for checking file size periodically
        self.file_check_timer = self.create_timer(
            self.check_file_interval, 
            self._periodic_file_check
        )
        
        # Timer for reporting statistics periodically
        self.stats_timer = self.create_timer(
            300.0,  # Report every 5 minutes
            self._report_statistics
        )
        
        # Timer for checking inactive topics
        self.inactive_topic_timer = self.create_timer(
            60.0,  # Check every minute
            self._check_inactive_topics
        )
        
        # Timer for time-based log rotation if enabled
        if self.log_rotation_policy in ['time', 'both']:
            rotation_secs = self.log_rotation_time_hours * 3600.0
            self.rotation_timer = self.create_timer(
                rotation_secs,
                self._time_based_rotation
            )
    
    def _open_log_file(self):
        """
        Open or create log file with proper error handling and rotation logic.
        
        Rotates the file if it exceeds the configured size limit.
        """
        try:
            # Check if file exists and needs rotation based on size
            if self._should_rotate_file():
                self._rotate_log_file()
            
            # Open file for writing (append mode)
            self.file = open(self.log_path, 'a', newline='')
            self.writer = csv.writer(self.file)
            self.message_count_since_flush = 0  # Counter for flushing
            
            # Write header if file is new/empty
            if os.path.getsize(self.log_path) == 0:
                self._write_csv_header()
                
            self.get_logger().info(f"Log file ready: {self.log_path}")
            logger.info(f"Log file ready: {self.log_path}")
            
        except PermissionError as e:
            self.get_logger().error(f"Permission error opening log file: {e}\n{traceback.format_exc()}")
            logger.error(f"Permission error opening log file: {e}\n{traceback.format_exc()}")
            # Try to use alternative file
            alt_path = os.path.join("/tmp", f"huskybot_log_{int(time.time())}.csv")
            self.get_logger().warn(f"Attempting to use alternative log file: {alt_path}")
            logger.warn(f"Attempting to use alternative log file: {alt_path}")
            try:
                self.file = open(alt_path, 'w', newline='')
                self.writer = csv.writer(self.file)
                self.log_path = alt_path
                self._write_csv_header()
                self.get_logger().info(f"Using alternative log file: {self.log_path}")
                logger.info(f"Using alternative log file: {self.log_path}")
            except Exception as e2:
                self.get_logger().error(f"Error opening alternative log file: {e2}")
                logger.error(f"Error opening alternative log file: {e2}")
                raise  # Re-raise as this is fatal
        except Exception as e:
            self.get_logger().error(f"Error opening log file: {e}\n{traceback.format_exc()}")
            logger.error(f"Error opening log file: {e}\n{traceback.format_exc()}")
            raise  # Re-raise as this is fatal
    
    def _write_csv_header(self):
        """
        Write CSV header row to the log file.
        """
        try:
            header = [
                'timestamp',      # Unix timestamp
                'datetime',       # Human-readable datetime
                'camera_name',    # Source camera
                'topic',          # Source topic
                'task',           # Task type (detection, segmentation, etc.)
                'class_name',     # Detected class
                'confidence',     # Detection confidence
                'top',            # Bounding box coordinates
                'left',
                'bottom',
                'right',
                'track_id',       # Object tracking ID (if available)
                'obb_angle',      # Oriented bounding box angle (if available)
                'process_time'    # Time taken to process this detection (new field)
            ]
            self.writer.writerow(header)
            self.file.flush()  # Ensure header is written immediately
        except Exception as e:
            self.get_logger().error(f"Error writing CSV header: {e}")
            logger.error(f"Error writing CSV header: {e}")
            raise  # Re-raise as this is fatal
    
    def _should_rotate_file(self):
        """
        Check if the log file should be rotated based on configured policy.
        
        Returns:
            bool: True if file should be rotated, False otherwise
        """
        if not os.path.exists(self.log_path):
            return False
            
        # Size-based rotation
        if self.log_rotation_policy in ['size', 'both']:
            try:
                if os.path.getsize(self.log_path) > self.max_log_size:
                    return True
            except OSError as e:
                self.get_logger().warn(f"Error checking file size: {e}")
                logger.warn(f"Error checking file size: {e}")
                return False
                
        # Time-based rotation handled by timer
        return False
    
    def _rotate_log_file(self):
        """
        Rotate the current log file with timestamp.
        """
        if not os.path.exists(self.log_path):
            return
            
        try:
            # Close current file if it's open
            if hasattr(self, 'file') and self.file:
                self.file.close()
                
            # Generate timestamped filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base, ext = os.path.splitext(self.log_path)
            rotated_path = f"{base}_{timestamp}{ext}"
            
            # Rename current file
            os.rename(self.log_path, rotated_path)
            self.get_logger().info(f"Rotated log file to: {rotated_path}")
            logger.info(f"Rotated log file to: {rotated_path}")
            
            # Update statistics
            with self.lock:
                self.stats['rotations'] += 1
            
            # Clean up old rotation files if we exceed max limit
            self._cleanup_old_rotations()
            
        except Exception as e:
            self.get_logger().error(f"Failed to rotate log file: {e}")
            logger.error(f"Failed to rotate log file: {e}")
            # Continue with current file if rotation fails
    
    def _cleanup_old_rotations(self):
        """
        Remove oldest rotation files if we exceed max_rotation_files.
        """
        try:
            # Find all rotation files
            base, ext = os.path.splitext(self.log_path)
            base_name = os.path.basename(base)
            dir_name = os.path.dirname(self.log_path)
            
            rotation_files = []
            for filename in os.listdir(dir_name):
                if filename.startswith(base_name) and filename.endswith(ext) and "_" in filename:
                    full_path = os.path.join(dir_name, filename)
                    rotation_files.append((os.path.getmtime(full_path), full_path))
            
            # Sort by modification time (oldest first)
            rotation_files.sort()
            
            # Remove oldest files if we have too many
            while len(rotation_files) > self.max_rotation_files:
                _, oldest_file = rotation_files.pop(0)
                try:
                    os.remove(oldest_file)
                    self.get_logger().info(f"Removed old rotation file: {oldest_file}")
                    logger.info(f"Removed old rotation file: {oldest_file}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to remove old rotation file {oldest_file}: {e}")
                    logger.warn(f"Failed to remove old rotation file {oldest_file}: {e}")
                    
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up old rotation files: {e}")
            logger.warn(f"Error cleaning up old rotation files: {e}")
    
    def _time_based_rotation(self):
        """
        Perform time-based log rotation.
        """
        self.get_logger().info("Performing scheduled time-based log rotation")
        logger.info("Performing scheduled time-based log rotation")
        
        with self.lock:
            try:
                # Close current file
                if hasattr(self, 'file') and self.file:
                    self.file.close()
                    
                # Rotate file
                self._rotate_log_file()
                
                # Open new log file
                self._open_log_file()
            except Exception as e:
                self.get_logger().error(f"Error in time-based rotation: {e}")
                logger.error(f"Error in time-based rotation: {e}")
    
    def _periodic_file_check(self):
        """
        Periodically check file size and rotate if needed.
        This runs on a timer independent of message reception.
        """
        with self.lock:
            # Check if we should rotate
            if self._should_rotate_file():
                self.get_logger().info(f"Log file size ({os.path.getsize(self.log_path)} bytes) "
                                     f"exceeds limit ({self.max_log_size} bytes), rotating...")
                logger.info(f"Log file size ({os.path.getsize(self.log_path)} bytes) "
                          f"exceeds limit ({self.max_log_size} bytes), rotating...")
                
                try:
                    # Close current file
                    if hasattr(self, 'file') and self.file:
                        self.file.close()
                    
                    # Rotate and open new file
                    self._rotate_log_file()
                    self._open_log_file()
                    
                    return True  # Rotation performed
                except Exception as e:
                    self.get_logger().error(f"Error rotating log file: {e}")
                    logger.error(f"Error rotating log file: {e}")
                
        return False  # No rotation performed
    
    def _report_statistics(self):
        """
        Periodically report statistics about logged messages.
        """
        with self.lock:
            try:
                current_time = time.time()
                uptime = current_time - self.stats['start_time']
                
                # Calculate message rates
                topic_rates = {}
                for topic, data in self.stats['message_rates'].items():
                    elapsed = current_time - data['last_time']
                    if elapsed > 0:
                        rate = data['count'] / elapsed
                        topic_rates[topic] = f"{rate:.2f} msg/s"
                        # Reset counters
                        data['count'] = 0
                        data['last_time'] = current_time
                
                # Basic statistics message
                stats_msg = (
                    f"Statistics - Uptime: {int(uptime)}s | "
                    f"Messages: {self.stats['total_messages']} | "
                    f"Detections: {self.stats['total_detections']} | "
                    f"Errors: {self.stats['errors']} | "
                    f"Rotations: {self.stats['rotations']}"
                )
                self.get_logger().info(stats_msg)
                logger.info(stats_msg)
                
                # Topic statistics
                topics_msg = f"Topic rates: {topic_rates}"
                self.get_logger().info(topics_msg)
                logger.info(topics_msg)
                
                # Class statistics if we have data
                if self.stats['by_class']:
                    top_classes = sorted(
                        self.stats['by_class'].items(),
                        key=lambda x: x[1],
                        reverse=True
                    )[:5]  # Top 5 classes
                    classes_msg = f"Top classes: {dict(top_classes)}"
                    self.get_logger().info(classes_msg)
                    logger.info(classes_msg)
                
                # Reset flag for next interval
                self.stats['last_status_print'] = current_time
                
                # Extended statistics if enabled
                if self.enable_extended_stats and self.stats['total_messages'] > 0:
                    # Calculate average processing time if we have data
                    if self.stats['processing_times']:
                        avg_proc = sum(self.stats['processing_times']) / len(self.stats['processing_times'])
                        max_proc = max(self.stats['processing_times'])
                        proc_msg = f"Processing times - Avg: {avg_proc*1000:.2f}ms, Max: {max_proc*1000:.2f}ms"
                        self.get_logger().info(proc_msg)
                        logger.info(proc_msg)
                        
                        # Reset for next interval
                        self.stats['processing_times'] = []
                    
                    # Average detections per message
                    avg_det = self.stats['total_detections'] / self.stats['total_messages']
                    det_msg = (
                        f"Detections per message - "
                        f"Avg: {avg_det:.2f}, "
                        f"Min: {self.stats['min_detections_per_message']}, "
                        f"Max: {self.stats['max_detections_per_message']}"
                    )
                    self.get_logger().info(det_msg)
                    logger.info(det_msg)
                    
                    # Confidence distribution
                    conf_msg = f"Confidence distribution: {self.stats['confidence_distribution']}"
                    self.get_logger().info(conf_msg)
                    logger.info(conf_msg)
                    
                    # Error distribution
                    if sum(self.stats['processing_errors'].values()) > 0:
                        err_msg = f"Error distribution: {self.stats['processing_errors']}"
                        self.get_logger().info(err_msg)
                        logger.info(err_msg)
            
            except Exception as e:
                self.get_logger().error(f"Error reporting statistics: {e}")
                logger.error(f"Error reporting statistics: {e}")
    
    def _check_inactive_topics(self):
        """
        Check for inactive topics and warn if they haven't sent messages recently.
        """
        current_time = time.time()
        
        with self.lock:
            for topic, last_seen in self.stats['topic_last_seen'].items():
                if last_seen > 0:  # Topic has been seen before
                    elapsed = current_time - last_seen
                    if elapsed > self.inactive_topic_timeout:
                        self.get_logger().warn(f"Topic {topic} inactive for {elapsed:.1f}s (timeout: {self.inactive_topic_timeout}s)")
                        logger.warn(f"Topic {topic} inactive for {elapsed:.1f}s (timeout: {self.inactive_topic_timeout}s)")
    
    def _update_confidence_stats(self, confidence):
        """
        Update confidence distribution statistics.
        
        Args:
            confidence: Detection confidence value
        """
        if not self.enable_extended_stats:
            return
            
        if confidence < 0.2:
            self.stats['confidence_distribution']['0.0-0.2'] += 1
        elif confidence < 0.4:
            self.stats['confidence_distribution']['0.2-0.4'] += 1
        elif confidence < 0.6:
            self.stats['confidence_distribution']['0.4-0.6'] += 1
        elif confidence < 0.8:
            self.stats['confidence_distribution']['0.6-0.8'] += 1
        else:
            self.stats['confidence_distribution']['0.8-1.0'] += 1
    
    def _subscribe_to_topics(self):
        """
        Create subscriptions to all specified topics.
        """
        # Check if topic list is empty
        if not self.topics:
            self.get_logger().error("No topics specified for monitoring")
            logger.error("No topics specified for monitoring")
            raise ValueError("No topics specified for monitoring")
        
        # Use reliable QoS to ensure we don't miss messages
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriptions with error handling
        self.subs = []  # Store subscription references to prevent garbage collection
        successful_subs = 0
        
        for topic in self.topics:
            try:
                sub = self.create_subscription(
                    Yolov12Inference,  # Message type for YOLOv12 inferences
                    topic,             # Topic name from parameters
                    self.callback,      # Callback function
                    qos                # QoS profile
                )
                self.subs.append(sub)
                self.get_logger().info(f"Subscribed to topic: {topic}")
                logger.info(f"Subscribed to topic: {topic}")
                successful_subs += 1
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to topic {topic}: {e}\n{traceback.format_exc()}")
                logger.error(f"Failed to subscribe to topic {topic}: {e}\n{traceback.format_exc()}")
                # Continue with other topics even if one fails
        
        # Check if we have at least one successful subscription
        if successful_subs == 0:
            self.get_logger().error("Failed to subscribe to any topics")
            logger.error("Failed to subscribe to any topics")
            raise RuntimeError("Failed to subscribe to any topics")
    
    def callback(self, msg):
        """
        Process incoming YOLOv12 inference messages and log to CSV.
        
        Args:
            msg (Yolov12Inference): Message containing inference results
                                    from detection, segmentation, OBB or tracking
        """
        start_time = time.time()  # Track processing time
        
        try:
            # Validate message
            if not self._validate_message(msg):
                return
            
            # Update statistics
            with self.lock:
                # Basic statistics
                self._update_basic_stats(msg)
                
                # Update topic's last seen timestamp
                self.stats['topic_last_seen'][msg.header.frame_id] = time.time()
                
                # Update message rate statistics
                if msg.header.frame_id in self.stats['message_rates']:
                    self.stats['message_rates'][msg.header.frame_id]['count'] += 1
            
            # Process each detection in the message
            detection_count = len(msg.yolov12_inference)
            
            # Update min/max detection counts for extended stats
            if self.enable_extended_stats:
                self.stats['max_detections_per_message'] = max(
                    self.stats['max_detections_per_message'], 
                    detection_count
                )
                if detection_count > 0:  # Only update min if we have detections
                    self.stats['min_detections_per_message'] = min(
                        self.stats['min_detections_per_message'], 
                        detection_count
                    )
                if self.stats['total_messages'] > 0:
                    self.stats['avg_detections_per_message'] = (
                        self.stats['total_detections'] / self.stats['total_messages']
                    )
            
            for det in msg.yolov12_inference:
                try:
                    # Apply filters
                    if not self._should_log_detection(det):
                        continue
                    
                    # Format timestamp for human readability
                    unix_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    dt_str = datetime.fromtimestamp(unix_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    # Calculate processing time for this message
                    process_time = time.time() - start_time
                    
                    # Thread-safe file write
                    with self.lock:
                        self._write_detection_to_csv(msg, det, unix_timestamp, dt_str, process_time)
                        
                        # Update class statistics
                        cls = getattr(det, 'class_name', 'unknown')
                        if cls not in self.stats['by_class']:
                            self.stats['by_class'][cls] = 0
                        self.stats['by_class'][cls] += 1
                        
                        # Update confidence statistics
                        if self.enable_extended_stats:
                            self._update_confidence_stats(getattr(det, 'confidence', 0.0))
                
                except AttributeError as e:
                    self.get_logger().warn(f"Missing attribute in detection: {e}")
                    logger.warn(f"Missing attribute in detection: {e}")
                    with self.lock:
                        self.stats['errors'] += 1
                        if self.enable_extended_stats:
                            self.stats['processing_errors']['attribute_error'] += 1
                    continue  # Skip this detection but continue with others
                    
                except Exception as e:
                    self.get_logger().warning(f"Error processing individual detection: {e}")
                    logger.warning(f"Error processing individual detection: {e}")
                    with self.lock:
                        self.stats['errors'] += 1
                        if self.enable_extended_stats:
                            self.stats['processing_errors']['other'] += 1
                    continue  # Skip this detection but continue with others
            
            # Flush file if needed based on message count
            self.message_count_since_flush += 1
            if self.message_count_since_flush >= self.flush_interval:
                with self.lock:
                    self._flush_file()
                    self.message_count_since_flush = 0
                    
            # Log debugging info if appropriate level
            if self.log_level == 'debug':
                self.get_logger().debug(
                    f"Logged {detection_count} detections from {msg.camera_name} "
                    f"(task: {msg.task}, topic: {msg.header.frame_id})"
                )
                
            # Track processing time for performance metrics
            process_time = time.time() - start_time
            with self.lock:
                self.stats['processing_times'].append(process_time)
                
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}\n{traceback.format_exc()}")
            logger.error(f"Error in callback: {e}\n{traceback.format_exc()}")
            with self.lock:
                self.stats['errors'] += 1
                if self.enable_extended_stats:
                    self.stats['processing_errors']['message_parse'] += 1
    
    def _validate_message(self, msg):
        """
        Validate incoming message structure.
        
        Args:
            msg: Message to validate
            
        Returns:
            bool: True if message is valid, False otherwise
        """
        try:
            # Check for required fields
            if not hasattr(msg, 'header') or not hasattr(msg, 'yolov12_inference'):
                self.get_logger().warn("Message missing required fields (header or yolov12_inference)")
                logger.warn("Message missing required fields (header or yolov12_inference)")
                return False
                
            # Check if camera_name is present and not empty
            if not hasattr(msg, 'camera_name') or not msg.camera_name:
                self.get_logger().warn("Message missing camera_name")
                logger.warn("Message missing camera_name")
                # Don't reject message, just warn
                
            # Check if task is present and not empty
            if not hasattr(msg, 'task') or not msg.task:
                self.get_logger().warn("Message missing task field")
                logger.warn("Message missing task field")
                # Don't reject message, just warn
                
            return True
                
        except Exception as e:
            self.get_logger().error(f"Error validating message: {e}")
            logger.error(f"Error validating message: {e}")
            return False
    
    def _update_basic_stats(self, msg):
        """
        Update basic statistics from message.
        
        Args:
            msg: Message containing inference results
        """
        self.stats['total_messages'] += 1
        self.stats['by_topic'][msg.header.frame_id] = self.stats['by_topic'].get(msg.header.frame_id, 0) + 1
        
        # Update camera statistics
        if msg.camera_name not in self.stats['by_camera']:
            self.stats['by_camera'][msg.camera_name] = 0
        self.stats['by_camera'][msg.camera_name] += 1
        
        # Log detection counts
        detection_count = len(msg.yolov12_inference)
        self.stats['total_detections'] += detection_count
        
        # Periodically log statistics (every 5 minutes)
        current_time = time.time()
        if current_time - self.stats['last_status_print'] > 300:  # 5 minutes
            self._report_statistics()
    
    def _should_log_detection(self, det):
        """
        Apply filtering to determine if a detection should be logged.
        
        Args:
            det: Detection to filter
            
        Returns:
            bool: True if detection should be logged, False otherwise
        """
        # Apply confidence filtering if configured
        try:
            if hasattr(det, 'confidence') and det.confidence < self.min_confidence:
                return False
        except (AttributeError, TypeError):
            # If confidence is missing or invalid, still log the detection
            pass
            
        # Apply class filtering if configured
        try:
            if self.class_filter and hasattr(det, 'class_name') and det.class_name.lower() not in self.class_filter.split(','):
                return False
        except (AttributeError, TypeError):
            # If class_name is missing or invalid, still log the detection
            pass
            
        return True
    
    def _write_detection_to_csv(self, msg, det, unix_timestamp, dt_str, process_time):
        """
        Write a single detection to the CSV file.
        
        Args:
            msg: Source message
            det: Detection object
            unix_timestamp: Unix timestamp
            dt_str: Formatted datetime string
            process_time: Processing time for this detection
        """
        try:
            # Safely get attributes with default values
            class_name = getattr(det, 'class_name', '')
            confidence = getattr(det, 'confidence', 0.0)
            top = getattr(det, 'top', 0)
            left = getattr(det, 'left', 0)
            bottom = getattr(det, 'bottom', 0)
            right = getattr(det, 'right', 0)
            track_id = getattr(det, 'track_id', -1)
            obb_angle = getattr(det, 'obb_angle', 0.0)
            
            # Write data row
            self.writer.writerow([
                unix_timestamp,   # Unix timestamp
                dt_str,           # Human-readable timestamp
                msg.camera_name,  # Camera source
                msg.header.frame_id,  # Topic name
                msg.task,         # Task type
                class_name,       # Class name
                confidence,       # Confidence score
                top,              # Bounding box coordinates
                left,
                bottom,
                right,
                track_id,         # Track ID
                obb_angle,        # OBB angle
                process_time      # Processing time
            ])
            
            # Update bytes written statistic
            if self.enable_extended_stats:
                # Roughly estimate bytes written (not exact but useful for monitoring)
                # Sum length of all string fields + estimated numeric fields size
                self.stats['bytes_written'] += (
                    len(str(unix_timestamp)) + len(dt_str) + len(msg.camera_name) +
                    len(msg.header.frame_id) + len(msg.task) + len(class_name) +
                    50  # Approximate size of numeric fields and CSV formatting
                )
                
        except Exception as e:
            self.get_logger().error(f"Error writing detection to CSV: {e}")
            logger.error(f"Error writing detection to CSV: {e}")
            self.stats['errors'] += 1
            if self.enable_extended_stats:
                self.stats['processing_errors']['file_write'] += 1
            raise  # Re-raise for caller to handle
    
    def _flush_file(self):
        """
        Flush file buffer to disk.
        """
        try:
            if hasattr(self, 'file') and self.file:
                self.file.flush()
        except Exception as e:
            self.get_logger().error(f"Error flushing log file: {e}")
            logger.error(f"Error flushing log file: {e}")
            # Continue operation even if flush fails
    
    def destroy_node(self):
        """
        Clean up resources when node is being destroyed.
        
        Overrides the destroy_node method to ensure proper cleanup.
        """
        try:
            self.get_logger().info("Shutting down MultiTaskLogger node")
            logger.info("Shutting down MultiTaskLogger node")
            
            with self.lock:
                # Log final statistics
                uptime = time.time() - self.stats['start_time']
                self.get_logger().info(
                    f"Node shutting down after {int(uptime)}s uptime | "
                    f"Messages: {self.stats['total_messages']} | "
                    f"Detections: {self.stats['total_detections']} | "
                    f"Errors: {self.stats['errors']} | "
                    f"Rotations: {self.stats['rotations']}"
                )
                logger.info(
                    f"Node shutting down after {int(uptime)}s uptime | "
                    f"Messages: {self.stats['total_messages']} | "
                    f"Detections: {self.stats['total_detections']} | "
                    f"Errors: {self.stats['errors']} | "
                    f"Rotations: {self.stats['rotations']}"
                )
                
                # Close file if it's open
                if hasattr(self, 'file') and self.file:
                    try:
                        self.file.flush()
                        self.file.close()
                        self.get_logger().info(f"Log file closed: {self.log_path}")
                        logger.info(f"Log file closed: {self.log_path}")
                    except Exception as e:
                        self.get_logger().error(f"Error closing log file: {e}")
                        logger.error(f"Error closing log file: {e}")
                
                # Cancel all timers
                for timer_attr in ['file_check_timer', 'stats_timer', 'inactive_topic_timer', 'rotation_timer']:
                    if hasattr(self, timer_attr):
                        timer = getattr(self, timer_attr)
                        timer.cancel()
                
        except Exception as e:
            self.get_logger().error(f"Error during node cleanup: {e}")
            logger.error(f"Error during node cleanup: {e}")
            
        # Call parent's destroy_node to complete cleanup
        super().destroy_node()


def main(args=None):
    """
    Entry point for the node.
    
    Initializes ROS2, creates the node, and handles the lifecycle.
    Implements comprehensive error handling for all stages.
    
    Args:
        args: Command line arguments passed to rclpy.init
    """
    # Setup signal handling for the main thread
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the node
        node = MultiTaskLogger()
        
        # Log successful start
        node.get_logger().info("MultiTaskLogger node is running")
        logger.info("MultiTaskLogger node is running")
        
        try:
            # Spin the node to execute callbacks
            rclpy.spin(node)
            
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            node.get_logger().info("KeyboardInterrupt, shutting down...")
            logger.info("KeyboardInterrupt, shutting down...")
            
        except Exception as e:
            # Handle unexpected runtime errors
            node.get_logger().error(f"Runtime error: {e}\n{traceback.format_exc()}")
            logger.error(f"Runtime error: {e}\n{traceback.format_exc()}")
            
        finally:
            # Clean up regardless of how we exit the spin
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error during node cleanup: {e}")
                logger.error(f"Error during node cleanup: {e}")
                
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error during ROS2 shutdown: {e}")
                logger.error(f"Error during ROS2 shutdown: {e}")
    
    except Exception as e:
        # Handle errors during initialization
        print(f"Fatal error during initialization: {e}\n{traceback.format_exc()}")
        logger.critical(f"Fatal error during initialization: {e}\n{traceback.format_exc()}")
        
        # Try to shutdown ROS2 if it was initialized
        try:
            rclpy.shutdown()
        except Exception:
            pass
        
        # Exit with error code
        sys.exit(1)


if __name__ == '__main__':
    """
    Execute the node when the script is run directly.
    """
    main()  # Call the main function