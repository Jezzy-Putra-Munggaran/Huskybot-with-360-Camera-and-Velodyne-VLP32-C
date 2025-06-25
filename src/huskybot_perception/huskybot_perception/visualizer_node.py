#!/usr/bin/env python3  # Shebang for Python3 interpreter (required for ROS2 executables)
# -*- coding: utf-8 -*-  # Encoding declaration for Unicode support

"""
MultiTask Visualizer Node for Huskybot

This node visualizes results from multiple YOLOv12 tasks (detection, segmentation, OBB, tracking)
from the 360° camera array. It displays task results for all cameras in a single window
for monitoring and debugging purposes.

Compatible with ROS2 Humble Hawksbill, Gazebo simulation, and real hardware:
- Clearpath Husky A200 robot
- Nvidia Jetson AGX Orin 32GB
- 6x Arducam IMX477 in hexagonal arrangement
- Velodyne VLP32-C 3D LiDAR

Author: Jezzy Putra Munggaran
License: Apache-2.0
"""

import os  # For file/directory operations and environment variables
import time  # For timestamps and timing operations
import threading  # For thread-safety in callbacks
import logging  # For structured logging to files
import traceback  # For detailed stack traces in error logs
from typing import Dict, List, Optional, Tuple, Union  # Type hints for better code documentation
from datetime import datetime  # For human-readable timestamps

import numpy as np  # For array/matrix operations and numerical processing
import cv2  # For OpenCV visualization and image processing
import yaml  # For YAML configuration file parsing

import rclpy  # Main ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from rclpy.parameter import Parameter  # For parameter handling and validation
from rclpy.exceptions import ParameterNotDeclaredException  # For parameter error handling
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # For QoS configuration

from yolov12_msgs.msg import Yolov12Inference, Yolov12Instance  # Custom message types for YOLOv12 results

# Define constant colors for visualization of different object classes
# Using BGR format (OpenCV standard)
CLASS_COLORS = {
    "person": (0, 0, 255),     # Red for person
    "bicycle": (255, 0, 0),    # Blue for bicycle
    "car": (0, 255, 0),        # Green for car
    "motorcycle": (255, 255, 0), # Cyan for motorcycle
    "bus": (255, 0, 255),      # Magenta for bus
    "truck": (0, 255, 255),    # Yellow for truck
    "traffic light": (128, 0, 128),  # Purple for traffic light
    "stop sign": (255, 128, 0),  # Orange for stop sign
    "default": (0, 255, 0)     # Default green for other classes
}

# Set up logging directory for error tracking and debugging
LOG_DIR = os.path.expanduser("~/huskybot_logs")  # Default log directory in user's home
if not os.path.exists(LOG_DIR):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)  # Create log directory if it doesn't exist
    except Exception as e:
        LOG_DIR = "/tmp"  # Fallback to /tmp if home directory isn't writable
        print(f"Warning: Could not create log directory in home, using /tmp: {e}")  # Print fallback warning

# Configure logging to file
logging.basicConfig(
    filename=os.path.join(LOG_DIR, "multitask_visualizer.log"),  # Log file path
    level=logging.INFO,  # Set logging level to INFO (includes warnings and errors)
    format='%(asctime)s - %(levelname)s - %(message)s',  # Include timestamp, level and message
    filemode='a'  # Append to existing file rather than overwrite
)

class MultiTaskVisualizer(Node):  # Main node class inheriting from ROS2 Node
    """
    ROS2 Node for visualizing results from multiple YOLOv12 tasks.
    
    Subscribes to multiple topics (/detection, /segmentation, /obb, /tracking)
    and visualizes the results in a single window, showing statistics for each
    camera-task combination.
    
    Features:
    - Thread-safe processing with RLock
    - Configurable display parameters via ROS2 parameters
    - Support for multiple YOLOv12 task types
    - Comprehensive error handling and logging
    - Support for both headless and GUI environments
    - Optional screenshot capability for recording results
    """
    
    def __init__(self) -> None:
        """
        Initialize the MultiTaskVisualizer node.
        
        Sets up parameters, subscribers, and visualization timer.
        Implements FULL OOP design with comprehensive error handling.
        """
        super().__init__('multitask_visualizer')  # Initialize ROS2 node with unique name
        
        # Log node startup
        self.get_logger().info("Initializing MultiTask Visualizer Node")
        logging.info("Initializing MultiTask Visualizer Node")
        
        # Declare parameters with default values and documentation
        self.declare_parameters(
            namespace='',  # No parameter namespace
            parameters=[
                ('window_width', Parameter.Type.INTEGER, 1280),  # Window width in pixels (larger default for better visibility)
                ('window_height', Parameter.Type.INTEGER, 720),  # Window height in pixels (16:9 aspect ratio)
                ('text_size', Parameter.Type.DOUBLE, 0.8),  # Text size scale factor (slightly larger for readability)
                ('text_color_r', Parameter.Type.INTEGER, 0),  # Text color R component (0-255)
                ('text_color_g', Parameter.Type.INTEGER, 255),  # Text color G component (0-255)
                ('text_color_b', Parameter.Type.INTEGER, 0),  # Text color B component (0-255)
                ('text_thickness', Parameter.Type.INTEGER, 2),  # Text thickness in pixels
                ('background_color_r', Parameter.Type.INTEGER, 0),  # Background color R component
                ('background_color_g', Parameter.Type.INTEGER, 0),  # Background color G component
                ('background_color_b', Parameter.Type.INTEGER, 0),  # Background color B component
                ('update_interval', Parameter.Type.DOUBLE, 0.1),  # Visualization update interval in seconds (faster for responsiveness)
                ('enable_display', Parameter.Type.BOOL, True),  # Enable/disable display window (for headless environments)
                ('show_stats', Parameter.Type.BOOL, True),  # Show statistics about detections
                ('topics', Parameter.Type.STRING_ARRAY, 
                 ['/detection', '/segmentation', '/obb', '/tracking']),  # Topics to subscribe
                ('verbose_logging', Parameter.Type.BOOL, False),  # Enable verbose logging
                ('save_screenshots', Parameter.Type.BOOL, False),  # Save screenshots of visualization
                ('screenshot_dir', Parameter.Type.STRING, os.path.expanduser("~/huskybot_screenshots")),  # Directory for screenshots
                ('screenshot_interval', Parameter.Type.DOUBLE, 5.0),  # Seconds between screenshots (to avoid filling disk)
                ('max_displayed_classes', Parameter.Type.INTEGER, 10),  # Maximum number of classes to display in verbose mode
            ]
        )
        
        # Get parameters with error handling
        try:
            self.window_width = self.get_parameter('window_width').value  # Get window width parameter
            self.window_height = self.get_parameter('window_height').value  # Get window height parameter
            self.text_size = self.get_parameter('text_size').value  # Get text size parameter
            self.text_color = (  # Create BGR color tuple from individual components
                self.get_parameter('text_color_b').value,
                self.get_parameter('text_color_g').value,
                self.get_parameter('text_color_r').value  # OpenCV uses BGR color order
            )
            self.text_thickness = self.get_parameter('text_thickness').value  # Get text thickness parameter
            self.background_color = (  # Create BGR color tuple from individual components
                self.get_parameter('background_color_b').value,
                self.get_parameter('background_color_g').value,
                self.get_parameter('background_color_r').value  # OpenCV uses BGR color order
            )
            self.update_interval = self.get_parameter('update_interval').value  # Get update interval parameter
            self.enable_display = self.get_parameter('enable_display').value  # Get display enable parameter
            self.show_stats = self.get_parameter('show_stats').value  # Get stats display parameter
            self.topics = self.get_parameter('topics').value  # Get topics list parameter
            self.verbose_logging = self.get_parameter('verbose_logging').value  # Get verbose logging parameter
            self.save_screenshots = self.get_parameter('save_screenshots').value  # Get screenshot saving parameter
            self.screenshot_dir = self.get_parameter('screenshot_dir').value  # Get screenshot directory parameter
            self.screenshot_interval = self.get_parameter('screenshot_interval').value  # Get screenshot interval parameter
            self.max_displayed_classes = self.get_parameter('max_displayed_classes').value  # Get max classes to display
            
            # Validate parameters and apply corrections for robustness
            if self.window_width <= 0 or self.window_height <= 0:  # Check for invalid window dimensions
                self.get_logger().warning("Invalid window dimensions, resetting to defaults (1280x720)")
                logging.warning("Invalid window dimensions, resetting to defaults (1280x720)")
                self.window_width = 1280  # Reset to default width
                self.window_height = 720  # Reset to default height
                
            if self.text_size <= 0:  # Check for invalid text size
                self.get_logger().warning("Invalid text size, resetting to default (0.8)")
                logging.warning("Invalid text size, resetting to default (0.8)")
                self.text_size = 0.8  # Reset to default text size
                
            if self.text_thickness <= 0:  # Check for invalid text thickness
                self.get_logger().warning("Invalid text thickness, resetting to default (2)")
                logging.warning("Invalid text thickness, resetting to default (2)")
                self.text_thickness = 2  # Reset to default text thickness
                
            if self.update_interval <= 0:  # Check for invalid update interval
                self.get_logger().warning("Invalid update interval, resetting to default (0.1)")
                logging.warning("Invalid update interval, resetting to default (0.1)")
                self.update_interval = 0.1  # Reset to default update interval

            if self.screenshot_interval <= 0:  # Check for invalid screenshot interval
                self.get_logger().warning("Invalid screenshot interval, resetting to default (5.0)")
                logging.warning("Invalid screenshot interval, resetting to default (5.0)")
                self.screenshot_interval = 5.0  # Reset to default screenshot interval

            if self.max_displayed_classes <= 0:  # Check for invalid max classes
                self.get_logger().warning("Invalid max displayed classes, resetting to default (10)")
                logging.warning("Invalid max displayed classes, resetting to default (10)")
                self.max_displayed_classes = 10  # Reset to default max classes
                
        except ParameterNotDeclaredException as e:  # Handle parameter not declared exception
            self.get_logger().error(f"Parameter error: {e}")
            logging.error(f"Parameter error: {e}")
            # Use default values if parameter error occurs
            self.window_width = 1280  # Default window width
            self.window_height = 720  # Default window height
            self.text_size = 0.8  # Default text size
            self.text_color = (0, 255, 0)  # Default green text
            self.text_thickness = 2  # Default text thickness
            self.background_color = (0, 0, 0)  # Default black background
            self.update_interval = 0.1  # Default update interval
            self.enable_display = True  # Default enable display
            self.show_stats = True  # Default show statistics
            self.topics = ['/detection', '/segmentation', '/obb', '/tracking']  # Default topics
            self.verbose_logging = False  # Default disable verbose logging
            self.save_screenshots = False  # Default disable screenshot saving
            self.screenshot_dir = os.path.expanduser("~/huskybot_screenshots")  # Default screenshot directory
            self.screenshot_interval = 5.0  # Default screenshot interval
            self.max_displayed_classes = 10  # Default max classes to display
        
        # Create screenshot directory if enabled
        if self.save_screenshots:
            try:
                os.makedirs(self.screenshot_dir, exist_ok=True)  # Create directory, ignore if exists
                self.get_logger().info(f"Screenshots will be saved to: {self.screenshot_dir}")
                logging.info(f"Screenshots will be saved to: {self.screenshot_dir}")
                self.last_screenshot_time = 0  # Initialize last screenshot time for throttling
            except Exception as e:
                self.get_logger().error(f"Failed to create screenshot directory: {e}")
                logging.error(f"Failed to create screenshot directory: {e}")
                self.save_screenshots = False  # Disable screenshots if directory creation fails
        
        # Dictionary to store inference messages by camera and task
        self.images: Dict[str, Yolov12Inference] = {}  # Buffer for detection results from all topics
        
        # Thread lock for thread-safety between callbacks and display
        self.lock = threading.RLock()  # Use RLock for re-entrant capability
        
        # Set up QoS for reliable subscription
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Use reliable QoS to ensure delivery
            history=QoSHistoryPolicy.KEEP_LAST,  # Only keep latest messages
            depth=10  # Keep last 10 messages in case of processing delays
        )
        
        # List to hold all subscriptions to prevent garbage collection
        self.subs = []  # List of subscriptions to detection/segmentation/obb/tracking topics
        
        # Subscribe to all specified topics
        for topic in self.topics:
            try:
                sub = self.create_subscription(
                    Yolov12Inference,  # Message type for YOLOv12 inference results
                    topic,              # Topic name to subscribe to
                    self.callback,      # Callback function to process messages
                    qos                 # QoS profile for this subscription
                )
                self.subs.append(sub)  # Add subscription to list to prevent garbage collection
                self.get_logger().info(f"Subscribed to topic: {topic}")  # Log successful subscription
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to topic {topic}: {e}")  # Log subscription failure
                logging.error(f"Failed to subscribe to topic {topic}: {e}\n{traceback.format_exc()}")  # Log detailed error
        
        # Timer for regular display updates
        self.timer = self.create_timer(
            self.update_interval,  # Time between display updates (seconds)
            self.display           # Display function to call
        )
        
        # Statistics tracking
        self.stats = {
            'total_messages': 0,  # Total messages received
            'total_detections': 0,  # Total detections processed
            'start_time': time.time(),  # Node start time for uptime calculation
            'last_update': time.time(),  # Last update time for rate calculation
            'topic_counts': {topic: 0 for topic in self.topics},  # Count messages per topic
            'class_counts': {},  # Count detections per class
            'error_counts': {  # Count different types of errors
                'callback': 0,
                'display': 0,
                'opencv': 0
            },
            'fps': 0.0,  # FPS calculation
            'frame_count': 0  # Frames processed for FPS calculation
        }
        
        # Check for OpenCV GUI support
        if self.enable_display:
            try:
                # Test OpenCV window creation
                cv2.namedWindow("Huskybot MultiTask Visualizer", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Huskybot MultiTask Visualizer", self.window_width, self.window_height)
                self.get_logger().info("OpenCV GUI initialized successfully")
                logging.info("OpenCV GUI initialized successfully")
            except Exception as e:
                self.get_logger().warning(f"Failed to initialize OpenCV GUI, running in headless mode: {e}")
                logging.warning(f"Failed to initialize OpenCV GUI, running in headless mode: {e}")
                self.enable_display = False  # Disable GUI if initialization fails
        
        # Log initialization completion
        self.get_logger().info(f"MultiTask Visualizer initialized with {len(self.topics)} topics")
        logging.info(f"MultiTask Visualizer initialized with {len(self.topics)} topics")
        if not self.enable_display:
            self.get_logger().info("Display window disabled (running in headless mode)")
            logging.info("Display window disabled (running in headless mode)")

    def callback(self, msg: Yolov12Inference) -> None:
        """
        Callback for YOLOv12 inference messages.
        
        Stores messages by camera name and task for visualization.
        Thread-safe implementation with comprehensive error handling.
        
        Args:
            msg: YOLOv12 inference message containing detection results
        """
        try:
            with self.lock:  # Thread-safe access to shared data
                # Validate message
                if not hasattr(msg, 'camera_name') or not hasattr(msg, 'task'):
                    self.get_logger().warning("Received invalid message format (missing camera_name or task)")
                    logging.warning("Received invalid message format (missing camera_name or task)")
                    return  # Skip processing invalid message
                
                # Create a unique key combining camera name and task
                key = msg.camera_name + "_" + msg.task
                
                # Store the message
                self.images[key] = msg
                
                # Update statistics
                self.stats['total_messages'] += 1  # Increment total message count
                self.stats['total_detections'] += len(msg.yolov12_inference)  # Add detections to total
                self.stats['last_update'] = time.time()  # Update timestamp
                self.stats['frame_count'] += 1  # Increment frame count for FPS calculation
                
                if msg.header.frame_id in self.stats['topic_counts']:
                    self.stats['topic_counts'][msg.header.frame_id] += 1  # Count by topic
                
                # Update class statistics for classes in this message
                for det in msg.yolov12_inference:
                    try:
                        class_name = det.class_name if hasattr(det, 'class_name') else 'unknown'
                        if class_name not in self.stats['class_counts']:
                            self.stats['class_counts'][class_name] = 0
                        self.stats['class_counts'][class_name] += 1
                    except Exception as e:
                        # Handle error in class statistics update
                        if self.verbose_logging:
                            self.get_logger().warning(f"Error updating class stats: {e}")
                
                # Calculate FPS (every 30 frames)
                if self.stats['frame_count'] % 30 == 0:
                    elapsed = time.time() - self.stats['start_time']
                    if elapsed > 0:  # Avoid division by zero
                        self.stats['fps'] = self.stats['frame_count'] / elapsed
                
                # Log info if verbose logging is enabled
                if self.verbose_logging:
                    detection_count = len(msg.yolov12_inference)
                    self.get_logger().info(f"Received {detection_count} detections from {key}")
                    
        except Exception as e:
            self.stats['error_counts']['callback'] += 1  # Count callback errors
            self.get_logger().error(f"Error in callback: {e}")  # Log error in callback
            logging.error(f"Error in callback: {e}\n{traceback.format_exc()}")  # Log detailed error with stack trace

    def display(self) -> None:
        """
        Display YOLOv12 inference results in a visualization window.
        
        Combines results from all subscribed topics and displays statistics.
        Handles threading, errors, and supports both headless and GUI modes.
        
        Features:
        - Thread-safe implementation
        - Error handling for visualization
        - Optional screenshot capability
        - Performance statistics display
        - Classification result visualization
        """
        if not self.enable_display and not self.save_screenshots:
            return  # Skip visualization if display is disabled and not saving screenshots
        
        try:
            with self.lock:  # Thread-safe access to shared data
                # Create a blank image canvas with specified dimensions
                img = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
                if self.background_color != (0, 0, 0):  # If not black background
                    img[:] = self.background_color  # Fill with specified background color
                
                # Setup for drawing text
                y_pos = 50  # Starting vertical position for text
                x_pos = 50  # Left margin for text
                line_height = 30  # Vertical spacing between lines
                
                # Add title with timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(
                    img, 
                    f"Huskybot MultiTask Visualization - {timestamp}", 
                    (x_pos, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    self.text_size * 1.2,  # Larger font for title
                    self.text_color, 
                    self.text_thickness
                )
                y_pos += line_height * 2  # Extra space after title
                
                # Display stats if enabled
                if self.show_stats:
                    uptime = time.time() - self.stats['start_time']  # Calculate uptime
                    fps = self.stats['fps']  # Get calculated FPS
                    
                    # Format statistics line with uptime, messages, detections, and FPS
                    stats_text = (f"Uptime: {int(uptime)}s | Messages: {self.stats['total_messages']} | "
                                 f"Detections: {self.stats['total_detections']} | FPS: {fps:.1f}")
                    
                    cv2.putText(
                        img, 
                        stats_text, 
                        (x_pos, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        self.text_size * 0.8,  # Smaller font for stats
                        self.text_color, 
                        self.text_thickness
                    )
                    y_pos += line_height
                
                # Display error counts if any errors have occurred
                if sum(self.stats['error_counts'].values()) > 0:
                    error_text = f"Errors: Callback={self.stats['error_counts']['callback']}, " \
                                f"Display={self.stats['error_counts']['display']}, " \
                                f"OpenCV={self.stats['error_counts']['opencv']}"
                    
                    cv2.putText(
                        img, 
                        error_text, 
                        (x_pos, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        self.text_size * 0.8,  # Smaller font for errors
                        (0, 0, 255),  # Red color for errors
                        self.text_thickness
                    )
                    y_pos += line_height
                
                # Show header for task results if we have data
                if self.images:
                    cv2.putText(
                        img, 
                        "Camera + Task: Detections Count", 
                        (x_pos, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        self.text_size,
                        self.text_color, 
                        self.text_thickness
                    )
                    y_pos += line_height
                else:
                    # Show waiting message if no data received yet
                    cv2.putText(
                        img, 
                        "Waiting for detection data...", 
                        (x_pos, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        self.text_size,
                        (0, 165, 255),  # Orange color for waiting
                        self.text_thickness
                    )
                    y_pos += line_height
                
                # Display each camera + task result in sorted order for consistency
                for key, msg in sorted(self.images.items()):
                    try:
                        # Format text with camera name, task, and detection count
                        detection_count = len(msg.yolov12_inference)
                        text = f"{key}: {detection_count} detections"
                        
                        # Draw text on image
                        cv2.putText(
                            img, 
                            text, 
                            (x_pos, y_pos), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            self.text_size, 
                            self.text_color, 
                            self.text_thickness
                        )
                        y_pos += line_height
                        
                        # Add more details for verbose mode
                        if self.verbose_logging and detection_count > 0:
                            # Count detections by class
                            classes = {}  # Dictionary to count by class
                            for det in msg.yolov12_inference:
                                class_name = det.class_name if hasattr(det, 'class_name') else 'unknown'
                                classes[class_name] = classes.get(class_name, 0) + 1
                            
                            # Limit display to max_displayed_classes for readability
                            class_items = list(classes.items())
                            if len(class_items) > self.max_displayed_classes:
                                class_items = sorted(class_items, key=lambda x: x[1], reverse=True)[:self.max_displayed_classes]
                                class_items.append(('others', sum(count for cls, count in classes.items() 
                                                              if cls not in [c for c, _ in class_items])))
                            
                            # Format class counts
                            class_text = " | ".join([f"{cls}: {count}" for cls, count in class_items])
                            cv2.putText(
                                img, 
                                f"  Classes: {class_text}", 
                                (x_pos + 20, y_pos), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                self.text_size * 0.7, 
                                self.text_color, 
                                self.text_thickness - 1
                            )
                            y_pos += line_height
                            
                    except Exception as e:
                        self.get_logger().warning(f"Error visualizing key {key}: {e}")  # Log visualization error
                        logging.warning(f"Error visualizing key {key}: {e}\n{traceback.format_exc()}")  # Log detailed error
                
                # Check if y_position exceeds window height and add scroll indicator if needed
                if y_pos > self.window_height - 60:
                    cv2.putText(
                        img, 
                        "▼ More data below (not shown) ▼", 
                        (self.window_width // 2 - 150, self.window_height - 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        self.text_size * 0.7, 
                        (0, 165, 255),  # Orange color for warning
                        self.text_thickness - 1
                    )
                
                # Display help/info text at bottom
                bottom_text = "Press 'Q' to quit | Press 'S' to save screenshot | Camera feeds processed by YOLOv12"
                cv2.putText(
                    img, 
                    bottom_text, 
                    (x_pos, self.window_height - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    self.text_size * 0.7, 
                    self.text_color, 
                    self.text_thickness - 1
                )
                
                # Save screenshot if enabled and interval has passed
                if self.save_screenshots:
                    current_time = time.time()
                    if current_time - self.last_screenshot_time >= self.screenshot_interval:
                        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                        screenshot_path = os.path.join(self.screenshot_dir, f"multitask_viz_{timestamp_str}.jpg")
                        try:
                            cv2.imwrite(screenshot_path, img)  # Save image to file
                            self.last_screenshot_time = current_time  # Update last screenshot time
                            if self.verbose_logging:
                                self.get_logger().info(f"Screenshot saved: {screenshot_path}")
                                logging.info(f"Screenshot saved: {screenshot_path}")
                        except Exception as e:
                            self.get_logger().warning(f"Failed to save screenshot: {e}")
                            logging.warning(f"Failed to save screenshot: {e}\n{traceback.format_exc()}")
                
                # Display the image if enabled
                if self.enable_display:
                    try:
                        cv2.imshow("Huskybot MultiTask Visualizer", img)  # Display image in window
                        key = cv2.waitKey(1) & 0xFF  # Check for key press (1ms wait)
                        
                        # Handle keyboard input
                        if key == ord('q') or key == ord('Q'):  # Q to quit
                            self.get_logger().info("User requested shutdown via 'Q' key")
                            logging.info("User requested shutdown via 'Q' key")
                            # Request ROS2 to shutdown this node
                            rclpy.shutdown()
                        elif key == ord('s') or key == ord('S'):  # S to save screenshot
                            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                            screenshot_path = os.path.join(self.screenshot_dir, f"manual_screenshot_{timestamp_str}.jpg")
                            try:
                                os.makedirs(self.screenshot_dir, exist_ok=True)  # Ensure directory exists
                                cv2.imwrite(screenshot_path, img)  # Save image to file
                                self.get_logger().info(f"Manual screenshot saved: {screenshot_path}")
                                logging.info(f"Manual screenshot saved: {screenshot_path}")
                            except Exception as e:
                                self.get_logger().warning(f"Failed to save manual screenshot: {e}")
                                logging.warning(f"Failed to save manual screenshot: {e}")
                    except cv2.error as e:
                        self.stats['error_counts']['opencv'] += 1  # Count OpenCV errors
                        self.get_logger().warning(f"OpenCV error: {e}")  # Log OpenCV error
                        logging.warning(f"OpenCV error: {e}")
                        
                        # Handle headless environment errors
                        if "cannot open display" in str(e).lower() or "gtk" in str(e).lower():
                            self.get_logger().warning("Running in headless environment, disabling display window")
                            logging.warning("Running in headless environment, disabling display window")
                            self.enable_display = False  # Disable display if in headless environment
                
        except Exception as e:
            self.stats['error_counts']['display'] += 1  # Count display errors
            self.get_logger().error(f"Error in display(): {e}")  # Log display error
            logging.error(f"Error in display(): {e}\n{traceback.format_exc()}")  # Log detailed display error
    
    def on_shutdown(self) -> None:
        """
        Clean up resources when node is shutting down.
        
        Closes OpenCV windows and logs final statistics.
        Ensures proper cleanup of all resources.
        """
        try:
            # Close OpenCV windows if they were created
            if self.enable_display:
                cv2.destroyAllWindows()
                self.get_logger().info("Closed OpenCV windows")
                logging.info("Closed OpenCV windows")
                
            # Log statistics before shutdown
            uptime = time.time() - self.stats['start_time']
            self.get_logger().info(f"Node shutting down after {uptime:.1f}s uptime")
            self.get_logger().info(f"Processed {self.stats['total_messages']} messages with {self.stats['total_detections']} total detections")
            self.get_logger().info(f"Final FPS: {self.stats['fps']:.1f}")
            self.get_logger().info(f"Error counts: callback={self.stats['error_counts']['callback']}, " +
                                  f"display={self.stats['error_counts']['display']}, " +
                                  f"opencv={self.stats['error_counts']['opencv']}")
            
            # Log top classes if any detections were processed
            if self.stats['class_counts']:
                top_classes = sorted(self.stats['class_counts'].items(), key=lambda x: x[1], reverse=True)[:5]
                class_info = ", ".join([f"{cls}: {count}" for cls, count in top_classes])
                self.get_logger().info(f"Top 5 detected classes: {class_info}")
            
            # Log all statistics to file for post-analysis
            logging.info(f"Node shutting down after {uptime:.1f}s uptime")
            logging.info(f"Processed {self.stats['total_messages']} messages with {self.stats['total_detections']} total detections")
            logging.info(f"Final FPS: {self.stats['fps']:.1f}")
            logging.info(f"Error counts: callback={self.stats['error_counts']['callback']}, " +
                        f"display={self.stats['error_counts']['display']}, " +
                        f"opencv={self.stats['error_counts']['opencv']}")
            
            # Write final stats to dedicated file for analysis
            try:
                stats_file = os.path.join(LOG_DIR, "multitask_visualizer_stats.json")
                import json
                with open(stats_file, 'w') as f:
                    # Create serializable version of stats
                    stats_copy = self.stats.copy()
                    stats_copy['uptime'] = uptime
                    stats_copy['start_time_str'] = datetime.fromtimestamp(
                        self.stats['start_time']).strftime("%Y-%m-%d %H:%M:%S")
                    stats_copy['end_time_str'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    stats_copy['start_time'] = str(stats_copy['start_time'])  # Convert to string for JSON
                    stats_copy['last_update'] = str(stats_copy['last_update'])  # Convert to string for JSON
                    
                    json.dump(stats_copy, f, indent=2)
                    self.get_logger().info(f"Saved final statistics to {stats_file}")
                    logging.info(f"Saved final statistics to {stats_file}")
            except Exception as e:
                self.get_logger().warning(f"Failed to save statistics to file: {e}")
                logging.warning(f"Failed to save statistics to file: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
            logging.error(f"Error during shutdown: {e}\n{traceback.format_exc()}")


def main(args=None) -> None:
    """
    Main function to initialize and run the MultiTaskVisualizer node.
    
    Handles the complete node lifecycle with comprehensive error handling.
    Ensures proper cleanup in all scenarios including exceptions.
    
    Args:
        args: Command line arguments passed to the node
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create node instance
        node = MultiTaskVisualizer()
        
        try:
            # Log node startup
            node.get_logger().info("MultiTask Visualizer Node is running")
            logging.info("MultiTask Visualizer Node is running")
            
            # Spin node (blocks until node is shutdown)
            rclpy.spin(node)
            
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            node.get_logger().info('KeyboardInterrupt, shutting down node')
            logging.info('KeyboardInterrupt, shutting down node')
            
        except Exception as e:
            # Handle runtime exceptions
            node.get_logger().error(f"Runtime error: {e}")
            logging.error(f"Runtime error: {e}\n{traceback.format_exc()}")
            
        finally:
            # Perform cleanup on shutdown
            try:
                node.on_shutdown()
                node.destroy_node()
                logging.info("Node resources cleaned up successfully")
            except Exception as e:
                print(f"Error during node cleanup: {e}")
                logging.error(f"Error during node cleanup: {e}\n{traceback.format_exc()}")
    
    except Exception as e:
        # Handle exceptions during initialization
        print(f"Fatal error initializing node: {e}")
        logging.critical(f"Fatal error initializing node: {e}\n{traceback.format_exc()}")
        
        # Ensure ROS2 is shutdown even if initialization fails
        try:
            rclpy.shutdown()
            logging.info("ROS2 shutdown completed after initialization error")
        except Exception as ex:
            logging.critical(f"Failed to shutdown ROS2 after initialization error: {ex}")
        
        # Exit with error code (commenting out to allow further execution in interactive environments)
        # sys.exit(1)


if __name__ == '__main__':
    """
    Entry point when script is run directly.
    
    Calls the main function to start the node.
    """
    main()  # Call main function to start the node